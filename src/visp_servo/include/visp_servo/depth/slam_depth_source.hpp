// ─────────────────────────────────────────────────────────────────
// visp_servo/depth/slam_depth_source.hpp
//
// Optional SLAM depth source. Subscribes /vo_pose + /point_cloud and
// returns the median depth of map points that project inside the bbox.
// Falls back to the bbox estimate when SLAM data is unavailable.
//
// THIS is the "add SLAM by a simple subscription" path: enable it with
// use_slam_depth:=true and these two subscriptions appear. The IBVS law
// is untouched — it just gets a better Z.
// ─────────────────────────────────────────────────────────────────
#ifndef VISP_SERVO_SLAM_DEPTH_SOURCE_HPP
#define VISP_SERVO_SLAM_DEPTH_SOURCE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Geometry>

#include "visp_servo/depth/depth_source.hpp"

namespace visp_servo {

class SlamDepthSource : public IDepthSource {
public:
    explicit SlamDepthSource(rclcpp::Node* node);

    void init(const servo_core::ServoInputs& cfg) override;
    double depth(const servo_core::ServoInputs& in) override;
    const char* name() const override { return "slam(fallback=bbox)"; }

private:
    void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Node* node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   sub_cloud_;

    double fx_{554.0}, fy_{554.0}, u0_{320.0}, v0_{240.0};
    Eigen::Matrix3d Rcw_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d tcw_ = Eigen::Vector3d::Zero();
    bool has_pose_ = false;

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_;
    bool has_cloud_ = false;
};

}  // namespace visp_servo

#endif  // VISP_SERVO_SLAM_DEPTH_SOURCE_HPP