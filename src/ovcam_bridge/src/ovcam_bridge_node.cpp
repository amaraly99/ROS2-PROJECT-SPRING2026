// ─────────────────────────────────────────────────────────────────
// WHAT DOES THIS NODE DO?
//
// 1. Opens the same shared memory the producer created
// 2. Opens the same named semaphore
// 3. Background thread: waits on semaphore, reads latest frame,
//    copies it into a ROS2 Image message, publishes it
//
// Topic published: /ovcam/image_raw  (sensor_msgs/Image, NV12)
// ─────────────────────────────────────────────────────────────────

#include "ovcam_shm.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// POSIX headers
#include <fcntl.h>      // O_RDONLY
#include <semaphore.h>  // sem_open, sem_timedwait
#include <sys/mman.h>   // shm_open, mmap
#include <cstring>      // memcpy
#include <thread>
#include <atomic>
#include <chrono>
#include <stdexcept>

using Image = sensor_msgs::msg::Image;

class OvcamBridge : public rclcpp::Node {
public:
    OvcamBridge() : Node("ovcam_bridge") {

        // ── ROS2 parameters (can be set on command line) ──────────
        // ros2 run ovcam_bridge ovcam_bridge_node --ros-args -p shm_name:=/other
        shm_name_ = declare_parameter("shm_name", std::string("/ovcam_frames"));
        sem_name_ = declare_parameter("sem_name", std::string("/ovcam_ready"));
        frame_id_ = declare_parameter("frame_id", std::string("camera"));

        // ── Publisher ─────────────────────────────────────────────
        // KeepLast(2) = only buffer 2 messages, drop older ones
        // best_effort = don't retry dropped messages (fine for video)
        pub_ = create_publisher<Image>("/ovcam/image_raw",
                                       rclcpp::QoS(2).best_effort());

        // ── Open shared memory ────────────────────────────────────
        // The producer may not have started yet, so retry for 30 seconds
        int fd = -1;
        for (int i = 0; i < 60 && fd < 0; ++i) {
            fd = shm_open(shm_name_.c_str(), O_RDONLY, 0);
            if (fd < 0) {
                RCLCPP_WARN(get_logger(), "waiting for shm '%s'...",
                            shm_name_.c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        if (fd < 0) throw std::runtime_error("shm_open failed after retries");

        // Read just the global header first to find the total size
        void* probe = mmap(nullptr, sizeof(ShmGlobal), PROT_READ,
                           MAP_SHARED, fd, 0);
        auto* gh    = static_cast<const ShmGlobal*>(probe);

        // Verify we're talking to the right producer
        if (gh->magic != OVCAM_MAGIC)
            throw std::runtime_error("SHM magic mismatch — wrong producer?");

        // Compute total shm size from header, then remap fully
        size_t total = sizeof(ShmGlobal) + gh->slot_count * gh->slot_size;
        munmap(probe, sizeof(ShmGlobal));

        base_ = mmap(nullptr, total, PROT_READ, MAP_SHARED, fd, 0);
        close(fd); // fd not needed after mmap

        if (base_ == MAP_FAILED) throw std::runtime_error("mmap failed");

        shm_total_ = total;
        gh_        = static_cast<const ShmGlobal*>(base_);

        RCLCPP_INFO(get_logger(), "shm mapped: %ux%u NV12, %u slots",
                    gh_->width, gh_->height, gh_->slot_count);

        // ── Open named semaphore ──────────────────────────────────
        // 0 as second arg = attach to existing, don't create
        sem_ = sem_open(sem_name_.c_str(), 0);
        if (sem_ == SEM_FAILED)
            throw std::runtime_error("sem_open failed — is producer running?");

        // ── Start background capture thread ───────────────────────
        running_ = true;
        thread_  = std::thread(&OvcamBridge::loop, this);
    }

    ~OvcamBridge() {
        running_ = false;
        sem_post(sem_);                        // unblock the waiting thread
        if (thread_.joinable()) thread_.join();// wait for thread to exit
        sem_close(sem_);
        munmap(base_, shm_total_);
    }

private:

    // ── Background thread: wakes on semaphore, publishes frames ───
    void loop() {
        uint64_t last_fn = 0; // last frame number we processed

        while (running_) {

            // Wait for producer to post (with 1s timeout so we can
            // check running_ and exit cleanly on shutdown)
            struct timespec deadline{};
            clock_gettime(CLOCK_REALTIME, &deadline);
            deadline.tv_sec += 1;

            if (sem_timedwait(sem_, &deadline) != 0) continue; // timeout or signal
            if (!running_) break;

            // How many frames are ready?
            uint64_t latest = ws_load(gh_);

            // Process every frame we missed since last time
            // (usually just 1, but catches up if we were slow)
            for (uint64_t fn = last_fn + 1; fn <= latest; ++fn)
                publish_frame(fn, latest);

            last_fn = latest;
        }
    }

    // ── Read one frame from shm and publish it ─────────────────────
    void publish_frame(uint64_t fn, uint64_t latest) {

        // Which slot holds frame fn?
        uint32_t idx  = (uint32_t)((fn - 1) % gh_->slot_count);
        auto*    slot = slot_at(base_, gh_->slot_size, idx);

        // ── SEQLOCK READ ──────────────────────────────────────────
        // Retry up to 256 times if we catch the writer mid-write
        for (int retry = 0; retry < 256; ++retry) {

            uint64_t s1 = seq_load(slot);

            // Odd = writer is active right now, spin briefly
            if (s1 % 2 != 0) {
                __asm__ volatile("yield" ::: "memory"); // ARM: hint to scheduler
                continue;
            }

            // 0 = slot never written, nothing to read
            if (s1 == 0) return;

            // Check this slot actually holds the frame we want.
            // If s1 > fn*2, the ring wrapped and this slot has a newer frame.
            // Our frame fn is gone — skip it.
            if (s1 != fn * 2) {
                if (latest - fn > gh_->slot_count)
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "consumer too slow, dropped frame %llu",
                        (unsigned long long)fn);
                return;
            }

            // ── Build ROS2 Image message ──────────────────────────
            auto msg             = std::make_unique<Image>();
            msg->header.frame_id = frame_id_;

            // Convert nanoseconds to ROS2 timestamp
            msg->header.stamp = rclcpp::Time(
                (int32_t)(slot->t_ns / 1000000000ULL),
                (uint32_t)(slot->t_ns % 1000000000ULL));

            msg->width    = slot->width;
            msg->height   = slot->height;
            msg->step     = slot->stride;  // bytes per row
            msg->encoding = "nv12";
            msg->data.resize(slot->bytes_used);

            // THE ONE COPY: shm → ROS2 message buffer
            std::memcpy(msg->data.data(), frame_data(slot), slot->bytes_used);

            // ── Validate seqlock ──────────────────────────────────
            // If seq changed while we were copying, data may be torn.
            // Retry the whole thing.
            uint64_t s2 = seq_load(slot);
            if (s2 != s1) continue;

            // Data is consistent — publish
            pub_->publish(std::move(msg));
            return;
        }

        RCLCPP_WARN(get_logger(), "seqlock retry exhausted for frame %llu",
                    (unsigned long long)fn);
    }

    // ── Member variables ──────────────────────────────────────────
    std::string shm_name_, sem_name_, frame_id_;

    void*            base_      = MAP_FAILED;
    size_t           shm_total_ = 0;
    const ShmGlobal* gh_        = nullptr;
    sem_t*           sem_       = SEM_FAILED;

    std::atomic<bool> running_{false};
    std::thread       thread_;

    rclcpp::Publisher<Image>::SharedPtr pub_;
};

// ── Entry point ────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OvcamBridge>());
    rclcpp::shutdown();
}
