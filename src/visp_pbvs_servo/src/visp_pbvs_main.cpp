#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "servo_core/servo_fsm_node.hpp"
#include "visp_pbvs_servo/pbvs_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<servo_core::ServoFsmNode>("visp_pbvs_node");
    node->set_controller(std::make_unique<PBVSController>(node.get()));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}