// ─────────────────────────────────────────────────────────────────
// visp_servo/slam_pose_source.cpp
// ─────────────────────────────────────────────────────────────────
#include "visp_servo/slam_pose_source.hpp"

namespace visp_servo {

SlamPoseSource::SlamPoseSource(rclcpp::Node* node) : node_(node) {
    sub_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        kPoseTopic, rclcpp::QoS(10),
        std::bind(&SlamPoseSource::on_pose, this, std::placeholders::_1));

    // Best-effort subscription: ORB-SLAM2 publishes after patching; OV2SLAM
    // never publishes this topic → tracking_state_ stays kNoData → only
    // staleness gate applies.
    sub_state_ = node_->create_subscription<std_msgs::msg::Int32>(
        kStateTopic, rclcpp::QoS(10),
        std::bind(&SlamPoseSource::on_tracking_state, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
        "SlamPoseSource: staleness=%.1fs  confidence_topic='%s'",
        kStalenessSec, kStateTopic);
}

void SlamPoseSource::on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr /*msg*/) {
    last_pose_time_ = node_->now();
    has_pose_ = true;
}

void SlamPoseSource::on_tracking_state(const std_msgs::msg::Int32::SharedPtr msg) {
    tracking_state_ = msg->data;
}

bool SlamPoseSource::is_usable() const {
    if (!has_pose_) return false;
    if ((node_->now() - last_pose_time_).seconds() > kStalenessSec) return false;
    // Confidence gate: only active when ORB-SLAM2 has published at least once.
    if (tracking_state_ != kNoData && tracking_state_ != kTrackingStateOK) return false;
    return true;
}

std::string SlamPoseSource::mode_str() const {
    if (tracking_state_ == kNoData) return "staleness-only";
    return "staleness+confidence";
}

}  // namespace visp_servo
