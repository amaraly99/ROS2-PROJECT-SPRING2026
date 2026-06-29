// ─────────────────────────────────────────────────────────────────
// visp_servo/depth/slam_depth_source.cpp
// ─────────────────────────────────────────────────────────────────
#include "visp_servo/depth/slam_depth_source.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace visp_servo {

// NOTE: on_pose/on_cloud and depth() all run on the visp node's SingleThreadedExecutor
// (visp_servo_main uses rclcpp::spin) — same thread, serialized, so Rcw_/tcw_/cloud_ need
// no mutex. If this node ever moves to a MultiThreadedExecutor, add one.
SlamDepthSource::SlamDepthSource(rclcpp::Node* node, double depth_scale)
    : node_(node), depth_scale_(depth_scale) {
    sub_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        kPoseTopic, rclcpp::QoS(10),
        std::bind(&SlamDepthSource::on_pose, this, std::placeholders::_1));
    sub_cloud_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        kCloudTopic, rclcpp::QoS(1).best_effort(),
        std::bind(&SlamDepthSource::on_cloud, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),
        "SlamDepthSource active — pose='%s' cloud='%s' scale=%.4f",
        kPoseTopic, kCloudTopic, depth_scale_);
}

void SlamDepthSource::init(const servo_core::ServoInputs& cfg) {
    fx_ = cfg.fx; fy_ = cfg.fy; u0_ = cfg.cx0; v0_ = cfg.cy0;
}

void SlamDepthSource::on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto& p = msg->pose.position;
    const auto& q = msg->pose.orientation;
    Eigen::Quaterniond Rwc_q(q.w, q.x, q.y, q.z);
    Eigen::Matrix3d Rwc = Rwc_q.normalized().toRotationMatrix();
    Eigen::Vector3d twc(p.x, p.y, p.z);
    Rcw_ = Rwc.transpose();
    tcw_ = -Rcw_ * twc;
    last_pose_time_ = node_->now();
    has_pose_ = true;
}

void SlamDepthSource::on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    cloud_ = msg;
    has_cloud_ = true;
}

double SlamDepthSource::depth(const servo_core::ServoInputs& in) {
    // No SLAM data, or pose went stale (tracking lost) → bbox fallback.
    const bool stale = has_pose_ &&
        (node_->now() - last_pose_time_).seconds() > kStalenessSec;
    if (!has_cloud_ || !has_pose_ || !cloud_ || stale ||
        cloud_->width * cloud_->height == 0) {
        if (stale)
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "SlamDepthSource: pose stale (>%.1fs) — bbox fallback", kStalenessSec);
        return in.Z;
    }

    const double x1 = in.cx - in.bw / 2.0, y1 = in.cy - in.bh / 2.0;
    const double x2 = in.cx + in.bw / 2.0, y2 = in.cy + in.bh / 2.0;

    std::vector<double> depths;
    depths.reserve(32);
    try {
        sensor_msgs::PointCloud2ConstIterator<float> ix(*cloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(*cloud_, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iz(*cloud_, "z");
        int skip = 0;
        for (; ix != ix.end(); ++ix, ++iy, ++iz) {
            if (++skip & 3) continue;            // subsample every 4th point
            if (!std::isfinite(*ix)) continue;
            Eigen::Vector3d Pw(*ix, *iy, *iz);
            Eigen::Vector3d Pc = Rcw_ * Pw + tcw_;
            if (Pc[2] <= 0.1) continue;
            double u = fx_ * Pc[0] / Pc[2] + u0_;
            double v = fy_ * Pc[1] / Pc[2] + v0_;
            if (u >= x1 && u <= x2 && v >= y1 && v <= y2) depths.push_back(Pc[2]);
        }
    } catch (const std::runtime_error&) {
        return in.Z;
    }
    if (depths.empty()) return in.Z;
    std::sort(depths.begin(), depths.end());
    return depth_scale_ * depths[depths.size() / 2];   // scaled median
}

}  // namespace visp_servo