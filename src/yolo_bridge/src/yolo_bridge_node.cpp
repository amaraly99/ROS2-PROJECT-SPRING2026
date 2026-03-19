// ─────────────────────────────────────────────────────────────────
// yolo_bridge_node — reads /yolo_shm, publishes ROS 2 messages
//
// Published topics:
//   /yolo/detections  (yolo_msgs/DetectionArray)  best_effort QoS
//   /yolo/annotated   (sensor_msgs/Image, bgr8)   best_effort QoS
//
// Mirrors the ovcam_bridge pattern: background thread waits on
// semaphore, reads shm via seqlock, publishes to ROS 2.
// ─────────────────────────────────────────────────────────────────

#include "yolo_shm.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <yolo_msgs/msg/detection.hpp>

#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <stdexcept>
#include <string>
#include <vector>

using Image          = sensor_msgs::msg::Image;
using Detection      = yolo_msgs::msg::Detection;
using DetectionArray = yolo_msgs::msg::DetectionArray;

// COCO 80 class names
static const char* COCO_NAMES[80] = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard",
    "sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
    "donut","cake","chair","couch","potted plant","bed","dining table","toilet",
    "tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush",
};

class YoloBridge : public rclcpp::Node {
public:
    YoloBridge() : Node("yolo_bridge") {

        shm_name_ = declare_parameter("shm_name", std::string("/yolo_shm"));
        sem_name_ = declare_parameter("sem_name", std::string("/yolo_ready"));
        frame_id_ = declare_parameter("frame_id", std::string("camera"));

        // Best-effort QoS: YOLO data is ephemeral — always use freshest
        auto qos = rclcpp::QoS(1).best_effort();
        pub_dets_ = create_publisher<DetectionArray>("/yolo/detections", qos);
        pub_img_  = create_publisher<Image>("/yolo/annotated", qos);

        // Open shared memory with retry
        int fd = -1;
        for (int i = 0; i < 120 && fd < 0; ++i) {
            fd = shm_open(shm_name_.c_str(), O_RDONLY, 0);
            if (fd < 0) {
                RCLCPP_WARN(get_logger(), "waiting for shm '%s'...",
                            shm_name_.c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        if (fd < 0) throw std::runtime_error("shm_open failed after retries");

        // Read global header
        void* probe = mmap(nullptr, sizeof(YoloShmGlobal), PROT_READ,
                           MAP_SHARED, fd, 0);
        auto* gh    = static_cast<const YoloShmGlobal*>(probe);

        if (gh->magic != YOLO_MAGIC)
            throw std::runtime_error("YOLO SHM magic mismatch");

        size_t total = sizeof(YoloShmGlobal) + gh->slot_count * gh->slot_size;
        munmap(probe, sizeof(YoloShmGlobal));

        base_ = mmap(nullptr, total, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);
        if (base_ == MAP_FAILED) throw std::runtime_error("mmap failed");

        shm_total_ = total;
        gh_        = static_cast<const YoloShmGlobal*>(base_);

        RCLCPP_INFO(get_logger(), "yolo_shm mapped: %ux%u, %u slots",
                    gh_->img_width, gh_->img_height, gh_->slot_count);

        // Open semaphore
        sem_ = sem_open(sem_name_.c_str(), 0);
        if (sem_ == SEM_FAILED)
            throw std::runtime_error("sem_open failed — is yolo_producer running?");

        running_ = true;
        thread_  = std::thread(&YoloBridge::loop, this);
    }

    ~YoloBridge() {
        running_ = false;
        sem_post(sem_);
        if (thread_.joinable()) thread_.join();
        sem_close(sem_);
        munmap(base_, shm_total_);
    }

private:
    void loop() {
        uint64_t last_fn = 0;

        while (running_) {
            struct timespec deadline{};
            clock_gettime(CLOCK_REALTIME, &deadline);
            deadline.tv_sec += 1;

            if (sem_timedwait(sem_, &deadline) != 0) continue;
            if (!running_) break;

            uint64_t latest = yolo_ws_load(gh_);
            // Only publish the latest frame (skip any we missed)
            if (latest > last_fn) {
                publish_frame(latest);
                last_fn = latest;
            }
        }
    }

    void publish_frame(uint64_t fn) {
        uint32_t idx  = (uint32_t)((fn - 1) % gh_->slot_count);
        auto*    slot = yolo_slot_at(base_, gh_->slot_size, idx);

        for (int retry = 0; retry < 256; ++retry) {
            uint64_t s1 = yolo_seq_load(slot);
            if (s1 & 1) {
                __asm__ volatile("yield" ::: "memory");
                continue;
            }
            if (s1 == 0) return;
            if (s1 != fn * 2) return;  // slot was overwritten

            // Read metadata
            uint32_t det_count = slot->det_count;
            uint32_t img_w     = slot->img_width;
            uint32_t img_h     = slot->img_height;
            uint32_t img_stride = slot->img_stride;
            uint32_t img_bytes = slot->img_bytes;
            uint64_t t_ns      = slot->t_ns;

            // Build timestamp
            auto stamp = rclcpp::Time(
                (int32_t)(t_ns / 1000000000ULL),
                (uint32_t)(t_ns % 1000000000ULL));

            // Read detections
            det_count = std::min(det_count, YOLO_MAX_DETS);
            auto* dets_src = yolo_dets(slot);

            auto det_msg = std::make_unique<DetectionArray>();
            det_msg->header.frame_id = frame_id_;
            det_msg->header.stamp    = stamp;
            det_msg->detections.reserve(det_count);

            for (uint32_t i = 0; i < det_count; ++i) {
                const auto& d = dets_src[i];
                Detection det;
                det.class_name  = (d.class_id < 80)
                    ? COCO_NAMES[d.class_id] : std::to_string(d.class_id);
                det.confidence  = d.confidence;
                det.center_x    = (d.x1 + d.x2) / 2.0;
                det.center_y    = (d.y1 + d.y2) / 2.0;
                det.size_width  = static_cast<double>(d.x2 - d.x1);
                det.size_height = static_cast<double>(d.y2 - d.y1);
                det_msg->detections.push_back(std::move(det));
            }

            // Validate seqlock before copying data
            uint64_t s2 = yolo_seq_load(slot);
            if (s2 != s1) continue;  // torn read, retry

            // Seqlock valid — publish detections
            pub_dets_->publish(std::move(det_msg));

            // Only copy + publish image if producer included one
            if (img_bytes > 0) {
                auto* img_src = yolo_img_data(slot);
                auto img_msg             = std::make_unique<Image>();
                img_msg->header.frame_id = frame_id_;
                img_msg->header.stamp    = stamp;
                img_msg->width    = img_w;
                img_msg->height   = img_h;
                img_msg->encoding = "rgb8";
                img_msg->step     = img_stride;
                img_msg->data.resize(img_bytes);
                std::memcpy(img_msg->data.data(), img_src, img_bytes);
                pub_img_->publish(std::move(img_msg));
            }
            return;
        }

        RCLCPP_WARN(get_logger(), "seqlock retry exhausted for frame %llu",
                    (unsigned long long)fn);
    }

    std::string shm_name_, sem_name_, frame_id_;

    void*                 base_      = MAP_FAILED;
    size_t                shm_total_ = 0;
    const YoloShmGlobal*  gh_        = nullptr;
    sem_t*                sem_       = SEM_FAILED;

    std::atomic<bool> running_{false};
    std::thread       thread_;

    rclcpp::Publisher<DetectionArray>::SharedPtr pub_dets_;
    rclcpp::Publisher<Image>::SharedPtr          pub_img_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloBridge>());
    rclcpp::shutdown();
}
