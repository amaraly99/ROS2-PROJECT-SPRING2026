#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "BenchmarkUtils.h"
#include"ORB_SLAM2/System.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ORB_SLAM2::SetBenchmarkThreadName("ORBFrontEnd");

    auto clean_args = rclcpp::remove_ros_arguments(argc, argv);
    if(clean_args.size() != 3)
    {
        cerr << endl << "Usage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }

    ORB_SLAM2::System SLAM(clean_args[1], clean_args[2], ORB_SLAM2::System::MONOCULAR);
    auto node = std::make_shared<MonocularSlamNode>(&SLAM, clean_args[1], clean_args[2]);

    rclcpp::spin(node);
    

    rclcpp::shutdown();

    return 0;
}



