// ─────────────────────────────────────────────────────────────────
// visp_servo_node — Multi-feature IBVS visual servoing via YOLO
//
// Uses 4 bbox corner features (8 visual features) for properly
// constrained 4-DOF drone control (vx, vy, vz, wz).
// The interaction matrix naturally handles depth via bbox size.
//
// State machine:
//   SEARCHING → APPROACHING → TRACKING → HOVERING
//       ↑            ↑            ↓           ↓
//       └────────────┴── LOST ←──┴───────────┘
//
// Subscriptions:
//   /yolo/detections   (yolo_msgs/DetectionArray)  — target bbox
//   /ovcam/image_raw   (sensor_msgs/Image, mono8)  — camera feed
//   /vo_pose           (geometry_msgs/PoseStamped)  — SLAM pose
//   /point_cloud       (sensor_msgs/PointCloud2)    — SLAM map
//
// Publications:
//   /cmd_vel           (geometry_msgs/Twist)        — velocity cmd
//   /visp/debug_image  (sensor_msgs/Image, bgr8)   — annotated debug
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
#include <chrono>

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
        RCLCPP_INFO(get_logger(),
            "IBVS: 4-corner features, lambda=%.2f, target_ratio=%.2f",
            lambda_, target_bbox_ratio_);
    }

private:
    // ── State machine ─────────────────────────────────────────
    enum class State { SEARCHING, APPROACHING, TRACKING, HOVERING, LOST };

    static const char* state_name(State s) {
        switch (s) {
            case State::SEARCHING:   return "SEARCHING";
            case State::APPROACHING: return "APPROACHING";
            case State::TRACKING:    return "TRACKING";
            case State::HOVERING:    return "HOVERING";
            case State::LOST:        return "LOST";
        }
        return "UNKNOWN";
    }

    // ── Parameters ───────────────────────────────────────────
    std::string target_class_;
    double min_confidence_;
    int    image_width_, image_height_;

    // IBVS gain (single adaptive gain used for all features)
    double lambda_;

    // Velocity limits
    double max_linear_, max_angular_;

    // Target distance control
    double target_bbox_ratio_;
    double bbox_ratio_tolerance_;

    // State machine thresholds
    int    lost_threshold_;
    int    search_timeout_;
    double search_yaw_rate_;
    double approach_ratio_;     // bbox_ratio to transition APPROACHING → TRACKING
    double hover_error_thresh_; // servo error norm to transition TRACKING → HOVERING

    // Safety
    double edge_margin_px_;     // pixels from image edge to trigger re-centering
    double vel_ramp_rate_;      // max velocity change per callback (m/s)

    // Smoothing
    int    smoothing_window_;

    // Known target height for depth estimation (meters)
    double known_target_height_;

    // ── Camera model ─────────────────────────────────────────
    vpCameraParameters cam_;

    // ── ViSP servo task with 4 corner features ───────────────
    vpServo servo_;
    // Current features: top-left, top-right, bottom-right, bottom-left
    vpFeaturePoint s_tl_, s_tr_, s_br_, s_bl_;
    // Desired features
    vpFeaturePoint s_tl_d_, s_tr_d_, s_br_d_, s_bl_d_;

    // ── State ────────────────────────────────────────────────
    State state_ = State::SEARCHING;
    int   lost_count_ = 0;
    int   search_count_ = 0;

    // Smoothing buffers for bbox corners
    std::deque<double> x1_hist_, y1_hist_, x2_hist_, y2_hist_;

    // Previous velocity for ramping
    Twist prev_cmd_{};

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

    // Latency tracking for benchmarks
    double cb_latency_sum_ = 0.0;
    int    cb_latency_count_ = 0;

    // ─────────────────────────────────────────────────────────
    void declare_parameters() {
        declare_parameter("target_class",          std::string("person"));
        declare_parameter("min_confidence",         0.3);
        declare_parameter("image_width",            640);
        declare_parameter("image_height",           480);

        // IBVS gain
        declare_parameter("lambda",                 0.5);

        // Velocity limits
        declare_parameter("max_linear",             0.5);
        declare_parameter("max_angular",            0.5);

        // Target distance
        declare_parameter("target_bbox_ratio",      0.35);
        declare_parameter("bbox_ratio_tolerance",   0.05);

        // State machine
        declare_parameter("lost_threshold",         15);
        declare_parameter("search_timeout",         300);
        declare_parameter("search_yaw_rate",        0.3);
        declare_parameter("approach_ratio",         0.15);
        declare_parameter("hover_error_thresh",     0.02);

        // Safety
        declare_parameter("edge_margin_px",         50.0);
        declare_parameter("vel_ramp_rate",          0.1);

        // Smoothing
        declare_parameter("smoothing_window",       3);

        // Known target height (meters) for depth estimation
        declare_parameter("known_target_height",    1.7);

        // Camera intrinsics (defaults from ov5647 calibration)
        declare_parameter("cam_fx", 543.098);
        declare_parameter("cam_fy", 539.702);
        declare_parameter("cam_cx", 310.687);
        declare_parameter("cam_cy", 222.928);
    }

    void load_parameters() {
        target_class_          = get_parameter("target_class").as_string();
        min_confidence_        = get_parameter("min_confidence").as_double();
        image_width_           = get_parameter("image_width").as_int();
        image_height_          = get_parameter("image_height").as_int();
        lambda_                = get_parameter("lambda").as_double();
        max_linear_            = get_parameter("max_linear").as_double();
        max_angular_           = get_parameter("max_angular").as_double();
        target_bbox_ratio_     = get_parameter("target_bbox_ratio").as_double();
        bbox_ratio_tolerance_  = get_parameter("bbox_ratio_tolerance").as_double();
        lost_threshold_        = get_parameter("lost_threshold").as_int();
        search_timeout_        = get_parameter("search_timeout").as_int();
        search_yaw_rate_       = get_parameter("search_yaw_rate").as_double();
        approach_ratio_        = get_parameter("approach_ratio").as_double();
        hover_error_thresh_    = get_parameter("hover_error_thresh").as_double();
        edge_margin_px_        = get_parameter("edge_margin_px").as_double();
        vel_ramp_rate_         = get_parameter("vel_ramp_rate").as_double();
        smoothing_window_      = get_parameter("smoothing_window").as_int();
        known_target_height_   = get_parameter("known_target_height").as_double();
    }

    // ─────────────────────────────────────────────────────────
    void setup_camera_model() {
        double fx = get_parameter("cam_fx").as_double();
        double fy = get_parameter("cam_fy").as_double();
        double cx = get_parameter("cam_cx").as_double();
        double cy = get_parameter("cam_cy").as_double();
        cam_.initPersProjWithoutDistortion(fx, fy, cx, cy);
    }

    // ─────────────────────────────────────────────────────────
    void setup_servo() {
        servo_.setServo(vpServo::EYEINHAND_CAMERA);
        servo_.setInteractionMatrixType(vpServo::CURRENT);
        servo_.setLambda(lambda_);

        // Compute desired bbox corners (target centered, at target_bbox_ratio_)
        double des_bh = target_bbox_ratio_ * image_height_;
        // Assume roughly square aspect ratio for person bbox (w ≈ 0.4 * h)
        double des_bw = des_bh * 0.4;

        double cx_img = image_width_  / 2.0;
        double cy_img = image_height_ / 2.0;

        double des_x1 = cx_img - des_bw / 2.0;
        double des_y1 = cy_img - des_bh / 2.0;
        double des_x2 = cx_img + des_bw / 2.0;
        double des_y2 = cy_img + des_bh / 2.0;

        // Desired depth at target distance
        double Z_des = cam_.get_px() * known_target_height_ / des_bh;

        // Convert desired corners to normalized coordinates
        double xn, yn;

        vpPixelMeterConversion::convertPoint(cam_, des_x1, des_y1, xn, yn);
        s_tl_d_.buildFrom(xn, yn, Z_des);

        vpPixelMeterConversion::convertPoint(cam_, des_x2, des_y1, xn, yn);
        s_tr_d_.buildFrom(xn, yn, Z_des);

        vpPixelMeterConversion::convertPoint(cam_, des_x2, des_y2, xn, yn);
        s_br_d_.buildFrom(xn, yn, Z_des);

        vpPixelMeterConversion::convertPoint(cam_, des_x1, des_y2, xn, yn);
        s_bl_d_.buildFrom(xn, yn, Z_des);

        // Initialize current features (will be updated each frame)
        s_tl_.buildFrom(0, 0, Z_des);
        s_tr_.buildFrom(0, 0, Z_des);
        s_br_.buildFrom(0, 0, Z_des);
        s_bl_.buildFrom(0, 0, Z_des);

        // Add 4 feature pairs to servo task
        servo_.addFeature(s_tl_, s_tl_d_);
        servo_.addFeature(s_tr_, s_tr_d_);
        servo_.addFeature(s_br_, s_br_d_);
        servo_.addFeature(s_bl_, s_bl_d_);

        RCLCPP_INFO(get_logger(),
            "Desired bbox: [%.0f,%.0f]-[%.0f,%.0f] Z=%.2fm",
            des_x1, des_y1, des_x2, des_y2, Z_des);
    }

    // ─────────────────────────────────────────────────────────
    void setup_ros() {
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

        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;
        Eigen::Quaterniond Rwc_q(q.w, q.x, q.y, q.z);
        Eigen::Matrix3d Rwc = Rwc_q.normalized().toRotationMatrix();
        Eigen::Vector3d twc(p.x, p.y, p.z);
        Rcw_ = Rwc.transpose();
        tcw_ = -Rcw_ * twc;
    }

    void on_cloud(const PointCloud2::SharedPtr msg) {
        latest_cloud_ = msg;
        has_cloud_ = true;
    }

    // ─────────────────────────────────────────────────────────
    void on_detections(const DetectionArray::SharedPtr msg) {
        auto t_cb_start = std::chrono::high_resolution_clock::now();

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

        // 2. Target found — compute bbox corners
        double raw_x1 = best->center_x - best->size_width  / 2.0;
        double raw_y1 = best->center_y - best->size_height / 2.0;
        double raw_x2 = best->center_x + best->size_width  / 2.0;
        double raw_y2 = best->center_y + best->size_height / 2.0;

        // 3. Smooth bbox corners
        push_smooth(x1_hist_, raw_x1);
        push_smooth(y1_hist_, raw_y1);
        push_smooth(x2_hist_, raw_x2);
        push_smooth(y2_hist_, raw_y2);

        double x1 = avg(x1_hist_);
        double y1 = avg(y1_hist_);
        double x2 = avg(x2_hist_);
        double y2 = avg(y2_hist_);

        double bw = x2 - x1;
        double bh = y2 - y1;
        double cx = (x1 + x2) / 2.0;
        double cy = (y1 + y2) / 2.0;
        double bbox_ratio = bh / static_cast<double>(image_height_);

        // 4. Estimate depth
        double Z_slam = try_slam_depth(cx, cy, bw, bh);
        bool using_slam_depth = (Z_slam > 0.0);
        double Z_est = using_slam_depth ? Z_slam
            : estimate_depth_from_bbox(bh);

        // 5. Update current corner features with estimated depth
        double xn, yn;

        vpPixelMeterConversion::convertPoint(cam_, x1, y1, xn, yn);
        s_tl_.buildFrom(xn, yn, Z_est);

        vpPixelMeterConversion::convertPoint(cam_, x2, y1, xn, yn);
        s_tr_.buildFrom(xn, yn, Z_est);

        vpPixelMeterConversion::convertPoint(cam_, x2, y2, xn, yn);
        s_br_.buildFrom(xn, yn, Z_est);

        vpPixelMeterConversion::convertPoint(cam_, x1, y2, xn, yn);
        s_bl_.buildFrom(xn, yn, Z_est);

        // 6. Compute IBVS control law
        vpColVector v = servo_.computeControlLaw();
        double error_norm = servo_.getError().frobeniusNorm();

        // 7. Update state machine
        update_state(bbox_ratio, error_norm);

        // 8. Build velocity command based on state
        Twist cmd = build_command(v, bbox_ratio);

        // 9. Apply velocity ramping for safety
        ramp_velocity(cmd);

        // 10. Apply edge safety margin
        apply_edge_safety(cmd, cx, cy, bw, bh);

        prev_cmd_ = cmd;
        pub_cmd_->publish(cmd);

        // Measure callback latency
        auto t_cb_end = std::chrono::high_resolution_clock::now();
        double cb_ms = std::chrono::duration<double, std::milli>(t_cb_end - t_cb_start).count();
        cb_latency_sum_ += cb_ms;
        cb_latency_count_++;

        // Log + debug
        throttled_log(cx, cy, bw, bh, Z_est, best_conf, error_norm, cmd, using_slam_depth);
        publish_debug(x1, y1, x2, y2, cmd, error_norm);
    }

    // ─────────────────────────────────────────────────────────
    // State machine
    // ─────────────────────────────────────────────────────────

    void update_state(double bbox_ratio, double error_norm) {
        State prev = state_;
        lost_count_ = 0;
        search_count_ = 0;

        switch (state_) {
        case State::SEARCHING:
        case State::LOST:
            // Target reacquired
            state_ = (bbox_ratio >= approach_ratio_)
                ? State::TRACKING : State::APPROACHING;
            break;

        case State::APPROACHING:
            if (bbox_ratio >= target_bbox_ratio_ - bbox_ratio_tolerance_) {
                state_ = State::TRACKING;
            }
            break;

        case State::TRACKING:
            if (error_norm < hover_error_thresh_ &&
                std::abs(bbox_ratio - target_bbox_ratio_) < bbox_ratio_tolerance_) {
                state_ = State::HOVERING;
            }
            break;

        case State::HOVERING:
            if (error_norm > hover_error_thresh_ * 3.0 ||
                std::abs(bbox_ratio - target_bbox_ratio_) > bbox_ratio_tolerance_ * 2.0) {
                state_ = State::TRACKING;
            }
            break;
        }

        if (state_ != prev) {
            RCLCPP_INFO(get_logger(), "State: %s -> %s",
                state_name(prev), state_name(state_));
        }
    }

    void handle_no_detection() {
        lost_count_++;

        if (state_ == State::SEARCHING) {
            search_count_++;
            // Slow yaw to scan for target
            Twist cmd{};
            cmd.angular.z = search_yaw_rate_;
            ramp_velocity(cmd);
            prev_cmd_ = cmd;
            pub_cmd_->publish(cmd);

            if (search_count_ > search_timeout_) {
                // Stop searching after timeout
                pub_cmd_->publish(Twist{});
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Search timeout — hovering");
            }
            return;
        }

        if (lost_count_ >= lost_threshold_) {
            if (state_ != State::LOST && state_ != State::SEARCHING) {
                State prev = state_;
                state_ = State::LOST;
                RCLCPP_WARN(get_logger(), "State: %s -> LOST",
                    state_name(prev));
            }

            // After being lost for a while, start searching
            if (lost_count_ >= lost_threshold_ * 3) {
                state_ = State::SEARCHING;
                search_count_ = 0;
                RCLCPP_INFO(get_logger(), "State: LOST -> SEARCHING");
            }
        }

        // Publish hover (zero velocity) with ramp-down
        Twist cmd{};
        ramp_velocity(cmd);
        prev_cmd_ = cmd;
        pub_cmd_->publish(cmd);
    }

    // ─────────────────────────────────────────────────────────
    // Velocity command construction
    // ─────────────────────────────────────────────────────────

    Twist build_command(const vpColVector& v, double bbox_ratio) {
        // ViSP returns [vx, vy, vz, wx, wy, wz] in camera frame
        // Map to drone body frame:
        //   camera Z → drone forward (vx)
        //   camera -X → drone left/right (vy)
        //   camera -Y → drone up/down (vz)
        //   camera -wy → drone yaw (wz)
        Twist cmd;
        cmd.linear.x  = clamp_vel( v[2], max_linear_);
        cmd.linear.y  = clamp_vel(-v[0], max_linear_);
        cmd.linear.z  = clamp_vel(-v[1], max_linear_);
        cmd.angular.z = clamp_vel(-v[4], max_angular_);

        // State-dependent gain scaling
        switch (state_) {
        case State::APPROACHING:
            // Aggressive forward, moderate lateral
            cmd.linear.x *= 1.2;
            break;

        case State::HOVERING:
            // Reduce all velocities for station-keeping
            cmd.linear.x  *= 0.3;
            cmd.linear.y  *= 0.3;
            cmd.linear.z  *= 0.3;
            cmd.angular.z *= 0.3;
            break;

        case State::TRACKING:
            // Normal gains — no scaling
            break;

        default:
            break;
        }

        // Re-clamp after scaling
        cmd.linear.x  = clamp_vel(cmd.linear.x,  max_linear_);
        cmd.linear.y  = clamp_vel(cmd.linear.y,  max_linear_);
        cmd.linear.z  = clamp_vel(cmd.linear.z,  max_linear_);
        cmd.angular.z = clamp_vel(cmd.angular.z, max_angular_);

        return cmd;
    }

    // ─────────────────────────────────────────────────────────
    // Safety features
    // ─────────────────────────────────────────────────────────

    void ramp_velocity(Twist& cmd) {
        auto ramp = [this](double cur, double prev) {
            double delta = cur - prev;
            if (std::abs(delta) > vel_ramp_rate_) {
                return prev + std::copysign(vel_ramp_rate_, delta);
            }
            return cur;
        };
        cmd.linear.x  = ramp(cmd.linear.x,  prev_cmd_.linear.x);
        cmd.linear.y  = ramp(cmd.linear.y,  prev_cmd_.linear.y);
        cmd.linear.z  = ramp(cmd.linear.z,  prev_cmd_.linear.z);
        cmd.angular.z = ramp(cmd.angular.z, prev_cmd_.angular.z);
    }

    void apply_edge_safety(Twist& cmd, double cx, double cy,
                           double bw, double bh) {
        // If target is near image edge, nudge velocity to re-center
        double x1 = cx - bw / 2.0;
        double y1 = cy - bh / 2.0;
        double x2 = cx + bw / 2.0;
        double y2 = cy + bh / 2.0;

        double margin = edge_margin_px_;
        double nudge = 0.1;  // m/s nudge strength

        // Left edge: move right (positive vy)
        if (x1 < margin)
            cmd.linear.y = std::max(cmd.linear.y, nudge);
        // Right edge: move left (negative vy)
        if (x2 > image_width_ - margin)
            cmd.linear.y = std::min(cmd.linear.y, -nudge);
        // Top edge: move up (positive vz)
        if (y1 < margin)
            cmd.linear.z = std::max(cmd.linear.z, nudge);
        // Bottom edge: move down (negative vz)
        if (y2 > image_height_ - margin)
            cmd.linear.z = std::min(cmd.linear.z, -nudge);
    }

    // ─────────────────────────────────────────────────────────
    // Depth estimation
    // ─────────────────────────────────────────────────────────

    double estimate_depth_from_bbox(double bbox_height_px) const {
        if (bbox_height_px < 1.0) return 10.0;  // very far away cap
        return cam_.get_py() * known_target_height_ / bbox_height_px;
    }

    double try_slam_depth(double cx, double cy, double bw, double bh) {
        if (!has_cloud_ || !has_pose_ || !latest_cloud_) return -1.0;
        if (latest_cloud_->width * latest_cloud_->height == 0) return -1.0;

        const double fx = cam_.get_px();
        const double fy = cam_.get_py();
        const double u0 = cam_.get_u0();
        const double v0 = cam_.get_v0();

        // Search within the bbox, not a fixed radius
        double search_x1 = cx - bw / 2.0;
        double search_y1 = cy - bh / 2.0;
        double search_x2 = cx + bw / 2.0;
        double search_y2 = cy + bh / 2.0;

        std::vector<double> depths;
        depths.reserve(32);

        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud_, "z");

            int skip = 0;
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                // Subsample: process every 4th point for performance
                if (++skip & 3) continue;
                if (!std::isfinite(*iter_x)) continue;

                Eigen::Vector3d Pw(*iter_x, *iter_y, *iter_z);
                Eigen::Vector3d Pc = Rcw_ * Pw + tcw_;

                if (Pc[2] <= 0.1) continue;

                double u_proj = fx * Pc[0] / Pc[2] + u0;
                double v_proj = fy * Pc[1] / Pc[2] + v0;

                if (u_proj >= search_x1 && u_proj <= search_x2 &&
                    v_proj >= search_y1 && v_proj <= search_y2) {
                    depths.push_back(Pc[2]);
                }
            }
        } catch (const std::runtime_error&) {
            return -1.0;
        }

        if (depths.empty()) return -1.0;

        std::sort(depths.begin(), depths.end());
        return depths[depths.size() / 2];
    }

    // ─────────────────────────────────────────────────────────
    // Smoothing utilities
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
    // Logging
    // ─────────────────────────────────────────────────────────

    void throttled_log(double cx, double cy, double bw, double bh,
                       double Z, double conf, double error_norm,
                       const Twist& cmd, bool slam_depth) {
        auto now_t = now();
        if ((now_t - last_log_time_).seconds() < 0.5) return;
        last_log_time_ = now_t;

        double avg_cb_ms = (cb_latency_count_ > 0)
            ? cb_latency_sum_ / cb_latency_count_ : 0.0;

        RCLCPP_INFO(get_logger(),
            "[%s] %s conf=%.2f bbox=[%.0f,%.0f %.0fx%.0f] "
            "Z=%.2fm(%s) err=%.4f "
            "cmd: vx=%.2f vy=%.2f vz=%.2f wz=%.2f "
            "cb=%.2fms",
            state_name(state_), target_class_.c_str(), conf,
            cx - bw/2, cy - bh/2, bw, bh,
            Z, slam_depth ? "SLAM" : "bbox", error_norm,
            cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z,
            avg_cb_ms);
    }

    // ─────────────────────────────────────────────────────────
    // Debug visualization
    // ─────────────────────────────────────────────────────────

    void publish_debug(double x1, double y1, double x2, double y2,
                       const Twist& cmd, double error_norm) {
        if (latest_frame_.empty()) return;
        if (pub_debug_->get_subscription_count() == 0) return;

        cv::Mat debug = latest_frame_.clone();

        // Draw current bbox (green)
        cv::rectangle(debug,
            {static_cast<int>(x1), static_cast<int>(y1)},
            {static_cast<int>(x2), static_cast<int>(y2)},
            {0, 255, 0}, 2);

        // Draw desired bbox (yellow, dashed-like via thinner line)
        double des_bh = target_bbox_ratio_ * image_height_;
        double des_bw = des_bh * 0.4;
        double des_cx = image_width_ / 2.0;
        double des_cy = image_height_ / 2.0;
        cv::rectangle(debug,
            {static_cast<int>(des_cx - des_bw/2), static_cast<int>(des_cy - des_bh/2)},
            {static_cast<int>(des_cx + des_bw/2), static_cast<int>(des_cy + des_bh/2)},
            {0, 255, 255}, 1);

        // Draw bbox center (red dot)
        double cx = (x1 + x2) / 2.0, cy = (y1 + y2) / 2.0;
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

        // Edge safety margin (red border)
        int m = static_cast<int>(edge_margin_px_);
        cv::rectangle(debug, {m, m},
            {image_width_ - m, image_height_ - m},
            {0, 0, 100}, 1);

        // State + error text
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s  err=%.4f",
                      state_name(state_), error_norm);
        cv::putText(debug, buf, {10, 25},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);

        // Velocity text
        std::snprintf(buf, sizeof(buf), "vx=%.2f vy=%.2f vz=%.2f wz=%.2f",
                      cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z);
        cv::putText(debug, buf, {10, 50},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, {255, 255, 255}, 1);

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