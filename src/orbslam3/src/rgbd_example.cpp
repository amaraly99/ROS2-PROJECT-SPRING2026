/*
* ROS2 RGB-D wrapper entrypoint (2026-10-03) for ORB-SLAM3.
*/

#include "ros2_orb_slam3/common_rgbd.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RgbdMode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
