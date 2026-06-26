#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "visp_pbvs_servo/pbvs_controller.hpp"

PBVSController::PBVSController(rclcpp::Node* node) : node_(node) {
    lambda_ = node_->declare_parameter<double>("lambda", 0.3);
    k_fwd_  = node_->declare_parameter<double>("k_fwd",  3.0);
}

servo_core::ServoVel PBVSController::computeApproach(const servo_core::ServoInputs& in) {
    if (in.Z < 0.5) return {};  // guard: stale or invalid depth

    // lateral position of sign center in camera frame (metres, right=+)
    double tx = (in.cx - in.cx0) / in.fx * in.Z;

    servo_core::ServoVel vel;
    // forward: bbox-ratio error — naturally bounded, same formula as TS2
    vel.vx = k_fwd_ * std::max(in.target_bbox_ratio - in.bbox_ratio, 0.0);
    // lateral: metric depth-scaled — the PBVS contribution vs TS2's pixel-space gain
    vel.vy = std::clamp(-lambda_ * tx, -1.5, 1.5);
    vel.vz = -0.5 * in.ey_norm;
    vel.wz = 0.0;
    return vel;
}
