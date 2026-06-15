// ─────────────────────────────────────────────────────────────────
// hil_servo_main.cpp   (TS2 executable)
//
// A ROS2 node: shared ServoFsmNode + the proportional servoing law.
// Node name "hil_servo_node" so it can run side-by-side conceptually
// with the IBVS node (only ONE runs at a time during a benchmark — both
// publish /cmd_vel).
// ─────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "servo_core/servo_fsm_node.hpp"
#include "hil_servo/proportional_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<servo_core::ServoFsmNode>("hil_servo_node");
    node->set_controller(std::make_unique<hil_servo::ProportionalController>(node.get()));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}