// ─────────────────────────────────────────────────────────────────
// visp_servo_main.cpp   (TS1 executable)
//
// A ROS2 node: shared ServoFsmNode + the IBVS (vpServo) servoing law.
// Node name "visp_servo_node". Only ONE controller runs per benchmark
// (both publish /cmd_vel).
// ─────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "servo_core/servo_fsm_node.hpp"
#include "visp_servo/ibvs_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<servo_core::ServoFsmNode>("visp_servo_node");
    node->set_controller(std::make_unique<visp_servo::IBVSController>(node.get()));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}