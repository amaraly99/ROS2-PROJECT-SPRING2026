// sim_camera_bridge_node — HIL only.
//
// Subscribes to a synthetic ROS image topic (rgb8 from MATLAB/Unreal),
// converts to NV12, writes into the /ovcam_frames POSIX SHM that
// yolo_producer (host) and ovcam_bridge (Docker) already consume.
//
// This node REPLACES ovcam_producer when running in HIL mode. Every
// downstream component (ovcam_bridge, yolo_producer, yolo_bridge,
// ov2slam, visp_servo) runs unchanged because the SHM wire format —
// magic, slot layout, seqlock, semaphore — is identical to the one
// produced by src/ovcam_producer/src/producer.cpp. The binary contract
// is enforced by including the same ovcam_shm.hpp header.

#include "ovcam_shm.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <ctime>
#include <fstream>
#include <stdexcept>

using sensor_msgs::msg::Image;

class SimCameraBridge : public rclcpp::Node {
public:
    SimCameraBridge() : Node("sim_camera_bridge") {
        input_topic_ = declare_parameter("input_topic", std::string("/sim/camera/image_raw"));
        width_       = declare_parameter("width",  640);
        height_      = declare_parameter("height", 480);
        slots_       = declare_parameter("slots",  4);
        shm_name_    = declare_parameter("shm_name", std::string("/ovcam_frames"));
        sem_name_    = declare_parameter("sem_name", std::string("/ovcam_ready"));

        // use_source_stamp: carry the INCOMING header.stamp through the ring in
        // SlotHeader::t_src_ns, instead of letting the consumer fall back to the
        // Pi's own arrival time. Required for stereo — two bridge instances must
        // republish a left/right pair with ONE shared stamp, and arrival time
        // differs per instance by network jitter. Default false so mono behaves
        // exactly as before and every published mono result stays valid.
        use_source_stamp_ = declare_parameter("use_source_stamp", false);

        // stamp_log: per-frame CSV for latency stages S1 (MATLAB→Pi delivery,
        // cross-clock — header.stamp is MATLAB's epoch) and S2 (recv→SHM write,
        // both CLOCK_MONOTONIC). Empty = off (default; zero overhead).
        auto stamp_log = declare_parameter("stamp_log", std::string(""));
        if (!stamp_log.empty()) {
            stamp_log_.open(stamp_log, std::ios::out | std::ios::trunc);
            if (stamp_log_.is_open()) {
                stamp_log_ << "frame,t_sim_stamp_ns,t_recv_mono_ns,t_shm_write_mono_ns\n";
                RCLCPP_INFO(get_logger(), "stamp log → %s", stamp_log.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "cannot open stamp_log '%s' — disabled",
                            stamp_log.c_str());
            }
        }

        // Stride padding: producer.cpp uses (W + 31) & ~31u. At 640 / 480 / 1280
        // these values are already 32-aligned, so stride == W and we can write
        // tightly-packed rows. Refuse anything else loudly rather than silently
        // breaking row indexing in downstream consumers.
        const uint32_t padded = (static_cast<uint32_t>(width_) + 31u) & ~31u;
        if (padded != static_cast<uint32_t>(width_)) {
            throw std::runtime_error(
                "sim_camera_bridge: width must be a multiple of 32 to match "
                "ovcam_producer's stride padding rule");
        }

        create_shm();

        // BEST_EFFORT: drop dropped UDP fragments rather than retransmit. For raw
        // 921 KB video frames, RELIABLE causes backpressure & rate collapse.
        // MATLAB-side publisher must also be BEST_EFFORT — QoS handshake otherwise fails.
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        qos.best_effort().durability_volatile();
        sub_ = create_subscription<Image>(
            input_topic_, qos,
            std::bind(&SimCameraBridge::on_image, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "sim_camera_bridge: ready — subscribed to %s (BEST_EFFORT), writing %s SHM "
            "(%dx%d NV12, %d slots). NOTE: MATLAB publisher must also be BEST_EFFORT.",
            input_topic_.c_str(), shm_name_.c_str(),
            width_, height_, slots_);
    }

    ~SimCameraBridge() override {
        if (sem_  != SEM_FAILED) { sem_close(sem_); sem_unlink(sem_name_.c_str()); }
        if (base_ != MAP_FAILED) { munmap(base_, total_); shm_unlink(shm_name_.c_str()); }
    }

private:
    void create_shm() {
        const uint32_t W = static_cast<uint32_t>(width_);
        const uint32_t H = static_cast<uint32_t>(height_);
        const uint32_t N = static_cast<uint32_t>(slots_);

        const uint32_t stride      = W;                        // already 32-aligned (checked above)
        const uint64_t frame_bytes = static_cast<uint64_t>(stride) * H * 3 / 2;
        slot_size_ = ((sizeof(SlotHeader) + frame_bytes) + SHM_ALIGN - 1)
                     & ~(SHM_ALIGN - 1);
        total_     = sizeof(ShmGlobal) + N * slot_size_;

        shm_unlink(shm_name_.c_str());
        int fd = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd < 0) throw std::runtime_error("shm_open failed");
        if (ftruncate(fd, static_cast<off_t>(total_)) != 0)
            throw std::runtime_error("ftruncate failed");

        base_ = mmap(nullptr, total_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        if (base_ == MAP_FAILED) throw std::runtime_error("mmap failed");
        std::memset(base_, 0, total_);

        g_ = static_cast<ShmGlobal*>(base_);
        g_->magic           = OVCAM_MAGIC;
        g_->version         = 1;
        g_->slot_count      = N;
        g_->slot_size       = slot_size_;
        g_->max_frame_bytes = frame_bytes;
        g_->width           = W;
        g_->height          = H;
        g_->stride          = stride;
        g_->fourcc          = FOURCC_NV12;
        g_->write_seq       = 0;

        sem_unlink(sem_name_.c_str());
        sem_ = sem_open(sem_name_.c_str(), O_CREAT, 0666, 0);
        if (sem_ == SEM_FAILED) throw std::runtime_error("sem_open failed");
    }

    void on_image(const Image::ConstSharedPtr msg) {
        struct timespec ts_recv{};
        clock_gettime(CLOCK_MONOTONIC, &ts_recv);
        if (static_cast<int>(msg->width)  != width_ ||
            static_cast<int>(msg->height) != height_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "incoming frame %ux%u does not match configured %dx%d — dropping",
                msg->width, msg->height, width_, height_);
            return;
        }

        cv::Mat rgb;
        try {
            rgb = cv_bridge::toCvCopy(msg, "rgb8")->image;
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "cv_bridge conversion failed: %s", e.what());
            return;
        }

        // RGB → I420 (planar Y / U / V), then repack U+V into NV12 interleaved UV.
        cv::Mat yuv_i420;
        cv::cvtColor(rgb, yuv_i420, cv::COLOR_RGB2YUV_I420);

        const int W = width_;
        const int H = height_;
        const uint8_t* y_in = yuv_i420.data;
        const uint8_t* u_in = y_in + static_cast<size_t>(H) * W;
        const uint8_t* v_in = u_in + (static_cast<size_t>(H) * W) / 4;

        const uint64_t fn  = ++fn_;
        const uint32_t idx = static_cast<uint32_t>((fn - 1) % slots_);
        SlotHeader* slot = slot_at(base_, slot_size_, idx);
        uint8_t*    dst  = frame_data(slot);

        // seqlock open: odd = "being written"
        seq_store(slot, fn * 2 - 1, __ATOMIC_RELAXED);
        __atomic_thread_fence(__ATOMIC_RELEASE);

        struct timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);

        // Y plane (H * W bytes)
        std::memcpy(dst, y_in, static_cast<size_t>(H) * W);

        // UV interleaved plane: H/2 rows of W bytes (W/2 U+V pairs per row)
        uint8_t* uv = dst + static_cast<size_t>(H) * W;
        const int uv_rows = H / 2;
        const int uv_pairs_per_row = W / 2;
        for (int i = 0; i < uv_rows; ++i) {
            const uint8_t* u_row = u_in + i * uv_pairs_per_row;
            const uint8_t* v_row = v_in + i * uv_pairs_per_row;
            uint8_t*       o_row = uv   + i * W;
            for (int j = 0; j < uv_pairs_per_row; ++j) {
                o_row[2*j]     = u_row[j];
                o_row[2*j + 1] = v_row[j];
            }
        }

        slot->t_ns       = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
        slot->width      = static_cast<uint32_t>(W);
        slot->height     = static_cast<uint32_t>(H);
        slot->stride     = static_cast<uint32_t>(W);
        slot->fourcc     = FOURCC_NV12;
        slot->bytes_used = static_cast<uint32_t>(H * W * 3 / 2);

        // Source stamp, or 0 meaning "absent" so the consumer falls back to
        // t_ns. Always written (not only when enabled) so a slot can never
        // carry a stale value left behind by an earlier frame in this ring.
        slot->t_src_ns   = use_source_stamp_
            ? (static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL
               + msg->header.stamp.nanosec)
            : 0ULL;

        // seqlock close: even = "complete"
        seq_store(slot, fn * 2, __ATOMIC_RELEASE);
        ws_store(g_, fn);
        sem_post(sem_);

        if (stamp_log_.is_open()) {
            const uint64_t t_sim = static_cast<uint64_t>(msg->header.stamp.sec)
                                   * 1000000000ULL + msg->header.stamp.nanosec;
            const uint64_t t_recv = static_cast<uint64_t>(ts_recv.tv_sec)
                                    * 1000000000ULL + ts_recv.tv_nsec;
            stamp_log_ << fn << ',' << t_sim << ',' << t_recv << ','
                       << slot->t_ns << '\n';
            if (fn % 100 == 0) stamp_log_.flush();
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "sim_camera_bridge: frame %lu  %ux%u %s  stamp=%u.%09u",
            static_cast<unsigned long>(fn),
            msg->width, msg->height, msg->encoding.c_str(),
            msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    // params
    std::string input_topic_, shm_name_, sem_name_;
    int width_{640}, height_{480}, slots_{4};
    bool use_source_stamp_{false};

    // shm state
    void*       base_{MAP_FAILED};
    size_t      total_{0};
    uint64_t    slot_size_{0};
    ShmGlobal*  g_{nullptr};
    sem_t*      sem_{SEM_FAILED};
    uint64_t    fn_{0};
    std::ofstream stamp_log_;

    rclcpp::Subscription<Image>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimCameraBridge>());
    rclcpp::shutdown();
    return 0;
}
