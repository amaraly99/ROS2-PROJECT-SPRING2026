// ─────────────────────────────────────────────────────────────────
// visp_servo/slam_pose_source.hpp
//
// SLAM availability gate used by IBVSController to decide whether to
// switch vx from bbox-ratio to SLAM-range-based approach.
//
// Option 1 — hard staleness gate (always active):
//   is_usable() returns false when /slam/pose is older than kStalenessSec.
//   Works with any SLAM (OV2SLAM, ORB-SLAM2, ORB-SLAM3).
//
// Option 2 — tracking-state confidence gate (active when ORB-SLAM2 is running):
//   is_usable() additionally requires /slam/tracking_state == kTrackingStateOK (2).
//   ORB-SLAM2 publishes this after the monocular-slam-node.cpp patch.
//   OV2SLAM does NOT publish it → topic never fires → tracking_state_ stays -1
//   → only the staleness gate applies (graceful degradation, no config needed).
//
// TODO-P (Option 3 — weighted fusion, deferred):
//   Monocular scale drift makes raw translation blending unreliable.
//   Only yaw/bearing is scale-invariant and a candidate for blending.
//   Defer until a GT-anchored scale correction is in place (see TODO-L).
// ─────────────────────────────────────────────────────────────────
#ifndef VISP_SERVO_SLAM_POSE_SOURCE_HPP
#define VISP_SERVO_SLAM_POSE_SOURCE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

namespace visp_servo {

class SlamPoseSource {
public:
    // ORB-SLAM2 eTrackingState::OK = 2 (from Tracking.h).
    // OV2SLAM / ORB-SLAM3 do not publish this topic → tracking_state_ stays
    // at kNoData (-1) and the tracking check is skipped.
    static constexpr int    kTrackingStateOK = 2;
    static constexpr int    kNoData          = -1;
    static constexpr double kStalenessSec    = 2.0;

    static constexpr const char* kPoseTopic    = "/slam/pose";
    static constexpr const char* kStateTopic   = "/slam/tracking_state";

    explicit SlamPoseSource(rclcpp::Node* node);

    // True when SLAM data is fresh and (if tracking state is available) OK.
    bool is_usable() const;

    // Description for logging.
    std::string mode_str() const;

private:
    void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void on_tracking_state(const std_msgs::msg::Int32::SharedPtr msg);

    rclcpp::Node* node_;
    rclcpp::Time  last_pose_time_{0, 0, RCL_ROS_TIME};
    bool          has_pose_{false};
    int           tracking_state_{kNoData};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            sub_state_;
};

}  // namespace visp_servo

#endif  // VISP_SERVO_SLAM_POSE_SOURCE_HPP
