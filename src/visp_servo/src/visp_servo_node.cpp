// ─────────────────────────────────────────────────────────────────
// visp_servo_node — IBVS visual servoing via YOLO detections
//
// Phase 1: skeleton — subscribes, loads camera params, logs state
// Phase 2: IBVS control law + velocity output (next step)
//
// Subscriptions:
//   /yolo/detections   (yolo_msgs/DetectionArray)  — target bbox
//   /ovcam/image_raw   (sensor_msgs/Image, mono8)  — camera feed
//   /vo_pose           (geometry_msgs/PoseStamped)  — SLAM pose
//
// Publications:
//   /cmd_vel           (geometry_msgs/Twist)        — velocity cmd
//   /visp/debug_image  (sensor_msgs/Image, bgr8)    — annotated debug
// ─────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <deque>
#include <string>
#include <cmath>
#include <algorithm>
#include <vector>

using DetectionArray = yolo_msgs::msg::DetectionArray;
using Detection      = yolo_msgs::msg::Detection;
using Image          = sensor_msgs::msg::Image;
using PointCloud2    = sensor_msgs::msg::PointCloud2;
using PoseStamped    = geometry_msgs::msg::PoseStamped;
using Twist          = geometry_msgs::msg::Twist;

// ─────────────────────────────────────────────────────────────────
// Node
// ─────────────────────────────────────────────────────────────────
class VispServoNode : public rclcpp::Node {
public:
    VispServoNode() : Node("visp_servo_node") {
        declare_parameters();
        load_parameters();
        setup_camera_model();
        setup_servo();
        setup_ros();

        RCLCPP_INFO(get_logger(),
            "visp_servo_node started — tracking '%s' (conf >= %.2f)",
            target_class_.c_str(), min_confidence_);
        RCLCPP_INFO(get_logger(),
            "Camera: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            cam_.get_px(), cam_.get_py(), cam_.get_u0(), cam_.get_v0());
        RCLCPP_INFO(get_logger(), "ViSP version: %d.%d.%d",
            VISP_VERSION_MAJOR, VISP_VERSION_MINOR, VISP_VERSION_PATCH);
    }

private:
    // ── Parameters ───────────────────────────────────────────
    std::string target_class_;
    double min_confidence_;
    int    image_width_, image_height_;
    double lambda_xy_, lambda_z_;
    double max_linear_, max_angular_;
    double target_bbox_ratio_, bbox_ratio_tolerance_;
    int    lost_threshold_;
    int    smoothing_window_;

    // ── Camera model ─────────────────────────────────────────
    vpCameraParameters cam_;

    // ── ViSP servo task ──────────────────────────────────────
    vpServo servo_;
    vpFeaturePoint s_cur_, s_des_;   // current & desired 2D point features

    // ── State ────────────────────────────────────────────────
    enum class State { WAITING, TRACKING, LOST };
    State state_ = State::WAITING;
    int   lost_count_ = 0;

    // Smoothing buffers
    std::deque<double> cx_hist_, cy_hist_, bw_hist_, bh_hist_;

    // Latest camera image (for debug overlay)
    cv::Mat latest_frame_;

    // Latest SLAM pose (as Tcw for point projection)
    PoseStamped latest_pose_;
    Eigen::Matrix3d Rcw_  = Eigen::Matrix3d::Identity();
    Eigen::Vector3d tcw_  = Eigen::Vector3d::Zero();
    bool has_pose_ = false;

    // Latest SLAM point cloud (world frame)
    PointCloud2::SharedPtr latest_cloud_;
    bool has_cloud_ = false;

    // ── ROS interfaces ───────────────────────────────────────
    rclcpp::Subscription<DetectionArray>::SharedPtr sub_dets_;
    rclcpp::Subscription<Image>::SharedPtr          sub_image_;
    rclcpp::Subscription<PoseStamped>::SharedPtr    sub_pose_;
    rclcpp::Subscription<PointCloud2>::SharedPtr    sub_cloud_;

    rclcpp::Publisher<Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<Image>::SharedPtr pub_debug_;

    // Throttle logging
    rclcpp::Time last_log_time_;

    // ─────────────────────────────────────────────────────────
    void declare_parameters() {
        declare_parameter("target_class",          std::string("person"));
        declare_parameter("min_confidence",         0.3);
        declare_parameter("image_width",            640);
        declare_parameter("image_height",           480);
        declare_parameter("lambda_xy",              0.5);
        declare_parameter("lambda_z",               0.3);
        declare_parameter("max_linear",             0.5);
        declare_parameter("max_angular",            0.5);
        declare_parameter("target_bbox_ratio",      0.35);
        declare_parameter("bbox_ratio_tolerance",   0.05);
        declare_parameter("lost_threshold",         15);
        declare_parameter("smoothing_window",       5);

        // Camera intrinsics (defaults from ov5647 calibration)
        declare_parameter("cam_fx", 543.098);
        declare_parameter("cam_fy", 539.702);
        declare_parameter("cam_cx", 310.687);
        declare_parameter("cam_cy", 222.928);
    }

    void load_parameters() {
        target_class_         = get_parameter("target_class").as_string();
        min_confidence_       = get_parameter("min_confidence").as_double();
        image_width_          = get_parameter("image_width").as_int();
        image_height_         = get_parameter("image_height").as_int();
        lambda_xy_            = get_parameter("lambda_xy").as_double();
        lambda_z_             = get_parameter("lambda_z").as_double();
        max_linear_           = get_parameter("max_linear").as_double();
        max_angular_          = get_parameter("max_angular").as_double();
        target_bbox_ratio_    = get_parameter("target_bbox_ratio").as_double();
        bbox_ratio_tolerance_ = get_parameter("bbox_ratio_tolerance").as_double();
        lost_threshold_       = get_parameter("lost_threshold").as_int();
        smoothing_window_     = get_parameter("smoothing_window").as_int();
    }

    // ─────────────────────────────────────────────────────────
    void setup_camera_model() {
        double fx = get_parameter("cam_fx").as_double();
        double fy = get_parameter("cam_fy").as_double();
        double cx = get_parameter("cam_cx").as_double();
        double cy = get_parameter("cam_cy").as_double();

        // vpCameraParameters: px, py, u0, v0 (no distortion model
        // needed — we servo on raw pixel → normalized coords)
        cam_.initPersProjWithoutDistortion(fx, fy, cx, cy);
    }

    // ─────────────────────────────────────────────────────────
    void setup_servo() {
        // Eye-in-hand: camera moves with the robot
        servo_.setServo(vpServo::EYEINHAND_CAMERA);
        servo_.setInteractionMatrixType(vpServo::CURRENT);
        servo_.setLambda(lambda_xy_);

        // Desired feature: target at image center (normalized coords)
        double u0 = static_cast<double>(image_width_)  / 2.0;
        double v0 = static_cast<double>(image_height_) / 2.0;

        double x_des, y_des;
        vpPixelMeterConversion::convertPoint(cam_, u0, v0, x_des, y_des);
        s_des_.buildFrom(x_des, y_des, 1.0);  // Z=1.0 placeholder

        // Current feature: will be updated each frame
        s_cur_.buildFrom(0, 0, 1.0);

        // Add feature pair to servo task
        servo_.addFeature(s_cur_, s_des_);
    }

    // ─────────────────────────────────────────────────────────
    void setup_ros() {
        // Subscribers
        auto qos_be = rclcpp::QoS(1).best_effort();
        auto qos_rel = rclcpp::QoS(2).reliable();

        sub_dets_ = create_subscription<DetectionArray>(
            "/yolo/detections", qos_be,
            std::bind(&VispServoNode::on_detections, this, std::placeholders::_1));

        sub_image_ = create_subscription<Image>(
            "/ovcam/image_raw", qos_rel,
            std::bind(&VispServoNode::on_image, this, std::placeholders::_1));

        sub_pose_ = create_subscription<PoseStamped>(
            "/vo_pose", rclcpp::QoS(10),
            std::bind(&VispServoNode::on_pose, this, std::placeholders::_1));

        sub_cloud_ = create_subscription<PointCloud2>(
            "/point_cloud", rclcpp::QoS(1).best_effort(),
            std::bind(&VispServoNode::on_cloud, this, std::placeholders::_1));

        // Publishers
        pub_cmd_   = create_publisher<Twist>("/cmd_vel", 10);
        pub_debug_ = create_publisher<Image>("/visp/debug_image",
                        rclcpp::QoS(1).best_effort());

        last_log_time_ = now();
    }

    // ─────────────────────────────────────────────────────────
    // Callbacks
    // ─────────────────────────────────────────────────────────

    void on_image(const Image::SharedPtr msg) {
        try {
            auto cv_ptr = cv_bridge::toCvShare(msg, "mono8");
            cv::cvtColor(cv_ptr->image, latest_frame_, cv::COLOR_GRAY2BGR);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "cv_bridge error: %s", e.what());
        }
    }

    void on_pose(const PoseStamped::SharedPtr msg) {
        latest_pose_ = *msg;
        has_pose_ = true;

        // Decompose Twc (camera-in-world) into Rcw, tcw for fast point projection
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;
        Eigen::Quaterniond Rwc_q(q.w, q.x, q.y, q.z);
        Eigen::Matrix3d Rwc = Rwc_q.normalized().toRotationMatrix();
        Eigen::Vector3d twc(p.x, p.y, p.z);
        // Invert: Tcw = Twc^{-1}
        Rcw_ = Rwc.transpose();
        tcw_ = -Rcw_ * twc;
    }

    void on_cloud(const PointCloud2::SharedPtr msg) {
        latest_cloud_ = msg;
        has_cloud_ = true;
    }

    void on_detections(const DetectionArray::SharedPtr msg) {
        // 1. Filter by target class and confidence
        const Detection* best = nullptr;
        double best_conf = 0.0;

        for (const auto& det : msg->detections) {
            if (det.class_name == target_class_ &&
                det.confidence >= min_confidence_ &&
                det.confidence > best_conf) {
                best = &det;
                best_conf = det.confidence;
            }
        }

        if (!best) {
            handle_no_detection();
            return;
        }

        // 2. Target found — update state
        state_ = State::TRACKING;
        lost_count_ = 0;

        // 3. Smooth the bbox center and size
        push_smooth(cx_hist_, best->center_x);
        push_smooth(cy_hist_, best->center_y);
        push_smooth(bw_hist_, best->size_width);
        push_smooth(bh_hist_, best->size_height);

        double cx = avg(cx_hist_);
        double cy = avg(cy_hist_);
        double bw = avg(bw_hist_);
        double bh = avg(bh_hist_);

        // 4. Convert pixel center to normalized image coordinates
        double x_n, y_n;
        vpPixelMeterConversion::convertPoint(cam_, cx, cy, x_n, y_n);

        // 5. Estimate depth — use SLAM point cloud if available, else bbox heuristic
        double bbox_ratio = bh / static_cast<double>(image_height_);
        double Z_slam = try_slam_depth(cx, cy);
        bool using_slam_depth = (Z_slam > 0.0);
        double Z_est = using_slam_depth ? Z_slam : estimate_depth_from_bbox(bbox_ratio);

        // 6. Update current feature
        s_cur_.buildFrom(x_n, y_n, Z_est);

        // 7. Compute control law
        vpColVector v = servo_.computeControlLaw();

        // 8. Extract 4-DOF and clamp
        //    ViSP returns [vx, vy, vz, wx, wy, wz] in camera frame
        //    For drone: vx = forward, vy = left/right, vz = up/down, wz = yaw
        Twist cmd;
        cmd.linear.x  = clamp_vel(v[2], max_linear_);   // camera Z → drone forward
        cmd.linear.y  = clamp_vel(-v[0], max_linear_);   // camera X → drone left/right (inverted)
        cmd.linear.z  = clamp_vel(-v[1], max_linear_);   // camera Y → drone up/down (inverted)
        cmd.angular.z = clamp_vel(-v[4], max_angular_);   // camera wy → drone yaw

        // 9. Apply approach/retreat based on bbox ratio
        double ratio_err = bbox_ratio - target_bbox_ratio_;
        if (std::abs(ratio_err) < bbox_ratio_tolerance_) {
            cmd.linear.x = 0.0;  // Within dead zone — hold distance
        } else if (ratio_err > 0.0) {
            // Target too close — retreat (negative forward vel)
            cmd.linear.x = std::min(cmd.linear.x, 0.0);
        }

        pub_cmd_->publish(cmd);

        // 10. Log + debug image
        throttled_log(cx, cy, bw, bh, Z_est, best_conf, cmd, using_slam_depth);
        publish_debug(cx, cy, bw, bh, cmd);
    }

    // ─────────────────────────────────────────────────────────
    void handle_no_detection() {
        lost_count_++;
        if (lost_count_ >= lost_threshold_ && state_ == State::TRACKING) {
            state_ = State::LOST;
            RCLCPP_WARN(get_logger(), "Target LOST — publishing zero velocity (hover)");
        }

        if (state_ == State::LOST || state_ == State::WAITING) {
            // Publish zero velocity (hover)
            pub_cmd_->publish(Twist{});
        }
    }

    // ─────────────────────────────────────────────────────────
    // Depth heuristic: Z ∝ 1/bbox_ratio
    // Calibrated so that bbox_ratio = target_bbox_ratio_ → Z = 2.0m
    double estimate_depth_from_bbox(double bbox_ratio) const {
        if (bbox_ratio < 0.01) return 10.0;  // very far away cap
        return target_bbox_ratio_ * 2.0 / bbox_ratio;
    }

    // ─────────────────────────────────────────────────────────
    // SLAM depth: project SLAM map points into the current camera frame,
    // find those that land near the target pixel (cx, cy), return median Z.
    // Returns -1.0 if SLAM data unavailable or no nearby points found.
    double try_slam_depth(double cx, double cy) {
        if (!has_cloud_ || !has_pose_ || !latest_cloud_) return -1.0;
        if (latest_cloud_->width * latest_cloud_->height == 0) return -1.0;

        const double search_radius_px = 30.0;
        const double fx = cam_.get_px();
        const double fy = cam_.get_py();
        const double u0 = cam_.get_u0();
        const double v0 = cam_.get_v0();

        std::vector<double> depths;
        depths.reserve(32);

        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud_, "z");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                if (!std::isfinite(*iter_x)) continue;

                // Transform world point to camera frame
                Eigen::Vector3d Pw(*iter_x, *iter_y, *iter_z);
                Eigen::Vector3d Pc = Rcw_ * Pw + tcw_;

                if (Pc[2] <= 0.1) continue;  // behind or too close to camera

                // Project to pixel
                double u_proj = fx * Pc[0] / Pc[2] + u0;
                double v_proj = fy * Pc[1] / Pc[2] + v0;

                double du = u_proj - cx;
                double dv = v_proj - cy;
                if ((du * du + dv * dv) < search_radius_px * search_radius_px) {
                    depths.push_back(Pc[2]);
                }
            }
        } catch (const std::runtime_error&) {
            // Cloud missing expected fields (e.g. before first keyframe)
            return -1.0;
        }

        if (depths.empty()) return -1.0;

        std::sort(depths.begin(), depths.end());
        return depths[depths.size() / 2];
    }

    // ─────────────────────────────────────────────────────────
    void push_smooth(std::deque<double>& hist, double val) {
        hist.push_back(val);
        while (static_cast<int>(hist.size()) > smoothing_window_)
            hist.pop_front();
    }

    double avg(const std::deque<double>& hist) const {
        if (hist.empty()) return 0.0;
        double sum = 0.0;
        for (double v : hist) sum += v;
        return sum / static_cast<double>(hist.size());
    }

    double clamp_vel(double v, double lim) const {
        return std::clamp(v, -lim, lim);
    }

    // ─────────────────────────────────────────────────────────
    void throttled_log(double cx, double cy, double bw, double bh,
                       double Z, double conf, const Twist& cmd,
                       bool slam_depth) {
        auto now_t = now();
        if ((now_t - last_log_time_).seconds() < 0.5) return;
        last_log_time_ = now_t;

        const char* state_str =
            (state_ == State::TRACKING) ? "TRACKING" :
            (state_ == State::LOST)     ? "LOST" : "WAITING";

        RCLCPP_INFO(get_logger(),
            "[%s] %s conf=%.2f cx=%.0f cy=%.0f bw=%.0f bh=%.0f "
            "Z=%.2fm(%s) cmd: vx=%.2f vy=%.2f vz=%.2f wz=%.2f",
            state_str, target_class_.c_str(), conf,
            cx, cy, bw, bh, Z, slam_depth ? "SLAM" : "bbox",
            cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z);
    }

    // ─────────────────────────────────────────────────────────
    void publish_debug(double cx, double cy, double bw, double bh,
                       const Twist& cmd) {
        if (latest_frame_.empty()) return;
        if (pub_debug_->get_subscription_count() == 0) return;

        cv::Mat debug = latest_frame_.clone();

        // Draw target bbox (green)
        int x1 = static_cast<int>(cx - bw / 2.0);
        int y1 = static_cast<int>(cy - bh / 2.0);
        int x2 = static_cast<int>(cx + bw / 2.0);
        int y2 = static_cast<int>(cy + bh / 2.0);
        cv::rectangle(debug, {x1, y1}, {x2, y2}, {0, 255, 0}, 2);

        // Draw bbox center (red dot)
        cv::circle(debug, {static_cast<int>(cx), static_cast<int>(cy)},
                   5, {0, 0, 255}, -1);

        // Draw image center crosshair (white)
        int ic_x = image_width_ / 2, ic_y = image_height_ / 2;
        cv::line(debug, {ic_x - 15, ic_y}, {ic_x + 15, ic_y}, {255, 255, 255}, 1);
        cv::line(debug, {ic_x, ic_y - 15}, {ic_x, ic_y + 15}, {255, 255, 255}, 1);

        // Draw velocity arrow from center (cyan)
        int arrow_scale = 100;
        cv::Point2i arrow_end(
            ic_x + static_cast<int>(cmd.linear.y * arrow_scale),
            ic_y - static_cast<int>(cmd.linear.z * arrow_scale));
        cv::arrowedLine(debug, {ic_x, ic_y}, arrow_end, {255, 255, 0}, 2);

        // State text
        const char* state_str =
            (state_ == State::TRACKING) ? "TRACKING" :
            (state_ == State::LOST)     ? "LOST" : "WAITING";
        cv::putText(debug, state_str, {10, 25},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

        // Velocity text
        char buf[128];
        std::snprintf(buf, sizeof(buf), "vx=%.2f vy=%.2f vz=%.2f wz=%.2f",
                      cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z);
        cv::putText(debug, buf, {10, 50},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);

        // Publish
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug).toImageMsg();
        msg->header.stamp = now();
        msg->header.frame_id = "camera";
        pub_debug_->publish(*msg);
    }
};

// ─────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VispServoNode>());
    rclcpp::shutdown();
    return 0;
}
