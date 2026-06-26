#ifndef VISP_PBVS_SERVO_PBVS_CONTROLLER_HPP
#define VISP_PBVS_SERVO_PBVS_CONTROLLER_HPP

#include "servo_core/servo_controller.hpp"
#include <rclcpp/rclcpp.hpp>

class PBVSController : public servo_core::IServoController {
private:
    rclcpp::Node* node_;
    double lambda_;  // lateral depth-scaled gain (k_lat)
    double k_fwd_;   // forward bbox-ratio gain (same semantics as TS2)

public:
    explicit PBVSController(rclcpp::Node* node);
    servo_core::ServoVel computeApproach(const servo_core::ServoInputs& in) override;
    const char* name() const override { return "pbvs"; }
};

#endif // VISP_PBVS_SERVO_PBVS_CONTROLLER_HPP
