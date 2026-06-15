// ─────────────────────────────────────────────────────────────────
// hil_servo/proportional_controller.hpp   (TS2)
//
// The MATLAB sim-HIL controller's servoing law as a pluggable
// IServoController. Decoupled proportional control on the image-plane
// error — NO interaction matrix, NO pseudo-inverse:
//
//   vx = k_fwd * (target_bbox_ratio - bbox_ratio)   (>= 0, distance closure)
//   vy = -k_lat * ex_norm                           (lateral centring)
//   vz = -k_vz  * ey_norm   (deadband, <= 0)        (vertical centring)
//   wz = 0                                          (yaw never used to centre)
//
// Each axis is independent. Depth is implicit in bbox_ratio. This is the
// servoing block only — the shared ServoFsmNode owns the FSM and filters.
// ─────────────────────────────────────────────────────────────────
#ifndef HIL_SERVO_PROPORTIONAL_CONTROLLER_HPP
#define HIL_SERVO_PROPORTIONAL_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "servo_core/servo_controller.hpp"

namespace hil_servo {

class ProportionalController : public servo_core::IServoController {
public:
    explicit ProportionalController(rclcpp::Node* node);

    servo_core::ServoVel computeApproach(const servo_core::ServoInputs& in) override;
    const char* name() const override { return "proportional"; }

private:
    rclcpp::Node* node_;
    double k_fwd_;
    double k_lat_;
    double k_vz_;
    double vert_deadband_;
};

}  // namespace hil_servo

#endif  // HIL_SERVO_PROPORTIONAL_CONTROLLER_HPP