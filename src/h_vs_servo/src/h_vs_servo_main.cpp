// ─────────────────────────────────────────────────────────────────
// h_vs_servo_main.cpp   (TS3 executable)
//
// Shared ServoFsmNode + Benhimane & Malis homography-based 2D VS.
// Node name "h_vs_servo_node". Identical pattern to visp_servo_main.cpp.
// ─────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "servo_core/servo_fsm_node.hpp"
#include "h_vs_servo/hvs_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<servo_core::ServoFsmNode>("h_vs_servo_node");
    node->set_controller(std::make_unique<h_vs_servo::HVSController>(node.get()));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}