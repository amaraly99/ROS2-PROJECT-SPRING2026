// ─────────────────────────────────────────────────────────────────
// hil_servo/proportional_controller.cpp   (TS2)
// ─────────────────────────────────────────────────────────────────
#include "hil_servo/proportional_controller.hpp"

#include <cmath>

namespace hil_servo {

ProportionalController::ProportionalController(rclcpp::Node* node)
    : node_(node) {
    // Controller-specific gains live with the controller, not the FSM core.
    k_fwd_         = node_->declare_parameter<double>("k_fwd",         3.0);
    k_lat_         = node_->declare_parameter<double>("k_lat",         1.5);
    k_vz_          = node_->declare_parameter<double>("k_vz",          1.5);
    vert_deadband_ = node_->declare_parameter<double>("vert_deadband", 0.08);

    RCLCPP_INFO(node_->get_logger(),
        "ProportionalController (TS2): k_fwd=%.2f k_lat=%.2f k_vz=%.2f deadband=%.2f",
        k_fwd_, k_lat_, k_vz_, vert_deadband_);
}

servo_core::ServoVel ProportionalController::computeApproach(
        const servo_core::ServoInputs& in) {
    servo_core::ServoVel v;

    // vx — distance closure via apparent size. Tapers to 0 at the setpoint;
    // never reverse (REACHED handles overshoot).
    v.vx = k_fwd_ * (in.target_bbox_ratio - in.bbox_ratio);
    if (v.vx < 0.0) v.vx = 0.0;

    // vy — lateral centring (Simulink +y = LEFT).
    v.vy = -k_lat_ * in.ex_norm;

    // vz — vertical centring with deadband; never climb during approach.
    v.vz = 0.0;
    if (std::abs(in.ey_norm) > vert_deadband_) v.vz = -k_vz_ * in.ey_norm;
    if (v.vz > 0.0) v.vz = 0.0;

    // wz — yaw is never used to centre during approach.
    v.wz = 0.0;

    return v;
}

}  // namespace hil_servo