#include<iostream>
#include<csignal>

#include<opencv2/core/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.hpp>

#include "BenchmarkUtils.h"
#include "System.h"

using namespace std;

using namespace std::placeholders;

using ImageMsg = sensor_msgs::msg::Image;
using PoseMsg  = geometry_msgs::msg::PoseStamped;
using Int32Msg = std_msgs::msg::Int32;

rclcpp::Node::SharedPtr g_node = nullptr;
ORB_SLAM2::System* g_slam = nullptr;

namespace
{
constexpr const char* kStereoTrajectoryFile = "Stereo_KeyFrameTrajectory.txt";

void SaveStereoTrajectory()
{
    if(!g_slam)
    {
        return;
    }

    std::cout << "Saving stereo keyframe trajectory to " << kStereoTrajectoryFile << " ..." << std::endl;
    g_slam->Shutdown();
    g_slam->SaveKeyFrameTrajectoryTUM(kStereoTrajectoryFile);
    std::cout << "Stereo trajectory saved!" << std::endl;
    g_slam = nullptr;
}

void HandleShutdownSignal(int signum)
{
    std::cout << "Stereo wrapper received signal " << signum << ", shutting down ROS ..." << std::endl;
    if(rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}
}


class ImageGrabber
{
public:
    explicit ImageGrabber(ORB_SLAM2::System* pSLAM)
        : mpSLAM(pSLAM)
    {
    }

    void GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight);

    ORB_SLAM2::System* mpSLAM;

    // HIL: the stack's backend-agnostic contract, identical to the mono node
    // (monocular-slam-node.cpp): slam_pose -> /slam/pose, slam_tracking_state ->
    // /slam/tracking_state via the stack yaml's remaps. Values: OK=2, LOST=3
    // (Tracking.h eTrackingState). Upstream stereo.cpp published nothing.
    rclcpp::Publisher<PoseMsg>::SharedPtr pose_pub_;
    rclcpp::Publisher<Int32Msg>::SharedPtr tracking_state_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("orbslam");
    ORB_SLAM2::SetBenchmarkThreadName("ORBFrontEnd");

    // HIL: run_stack_hil.sh appends `--ros-args -r ...` to every sidecar command,
    // so argc is never exactly 3 in the stack. Strip ROS args first, as mono.cpp does.
    auto clean_args = rclcpp::remove_ros_arguments(argc, argv);
    if(clean_args.size() != 3)
    {
        cerr << endl << "Usage: ros2 run orbslam stereo path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }

    ORB_SLAM2::System SLAM(clean_args[1], clean_args[2], ORB_SLAM2::System::STEREO);
    g_slam = &SLAM;
    ImageGrabber igb(&SLAM);
    igb.pose_pub_           = g_node->create_publisher<PoseMsg>("slam_pose", rclcpp::QoS(10));
    igb.tracking_state_pub_ = g_node->create_publisher<Int32Msg>("slam_tracking_state", rclcpp::QoS(10));

    std::signal(SIGINT, HandleShutdownSignal);
    std::signal(SIGTERM, HandleShutdownSignal);

    // The current wrapper expects already-rectified images on these topics.
    message_filters::Subscriber<ImageMsg> left_sub(g_node, "camera/left");
    message_filters::Subscriber<ImageMsg> right_sub(g_node, "camera/right");

    using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
    message_filters::Synchronizer<approximate_sync_policy> syncApproximate(approximate_sync_policy(10), left_sub, right_sub);
    syncApproximate.registerCallback(&ImageGrabber::GrabStereo, &igb);

    rclcpp::spin(g_node);

    SaveStereoTrajectory();

    if(rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    g_node = nullptr;

    return 0;
}

void ImageGrabber::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    ORB_SLAM2::SetBenchmarkThreadName("ORBFrontEnd");
    ORB_SLAM2::ScopedBenchmarkTimer callback_total(
        "wrapper/callback_total",
        [this](){ return mpSLAM ? static_cast<long>(mpSLAM->GetCurrentFrame().mnId) : -1L; },
        ORB_SLAM2::ScopedBenchmarkTimer::LongProvider(),
        [this](){ return mpSLAM ? mpSLAM->GetTrackingState() : ORB_SLAM2::Tracking::SYSTEM_NOT_READY; });

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(g_node->get_logger(), "cv_bridge left exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(g_node->get_logger(), "cv_bridge right exception: %s", e.what());
        return;
    }

    const double timestamp =
        static_cast<double>(msgLeft->header.stamp.sec) +
        static_cast<double>(msgLeft->header.stamp.nanosec) * 1e-9;

    cv::Mat Tcw;
    {
        ORB_SLAM2::ScopedBenchmarkTimer frontend_total(
            "frontend/full_tracking",
            [this](){ return mpSLAM ? static_cast<long>(mpSLAM->GetCurrentFrame().mnId) : -1L; },
            ORB_SLAM2::ScopedBenchmarkTimer::LongProvider(),
            [this](){ return mpSLAM ? mpSLAM->GetTrackingState() : ORB_SLAM2::Tracking::SYSTEM_NOT_READY; });
        ORB_SLAM2::ScopedBenchmarkTimer core_track(
            "frontend/core_track_call",
            [this](){ return mpSLAM ? static_cast<long>(mpSLAM->GetCurrentFrame().mnId) : -1L; },
            ORB_SLAM2::ScopedBenchmarkTimer::LongProvider(),
            [this](){ return mpSLAM ? mpSLAM->GetTrackingState() : ORB_SLAM2::Tracking::SYSTEM_NOT_READY; });
        Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, timestamp);
    }

    // HIL: publish tracking state every frame (the orchestrator's readiness check
    // polls it), and the camera pose in the map frame whenever tracking produced one.
    // Same conversion as monocular-slam-node.cpp: Twc = Tcw^-1, rotation -> quaternion.
    {
        Int32Msg ts_msg;
        ts_msg.data = static_cast<int>(mpSLAM->GetTrackingState());
        tracking_state_pub_->publish(ts_msg);
    }
    if (!Tcw.empty() && Tcw.rows == 4 && Tcw.cols == 4) {
        cv::Mat Twc = Tcw.inv();
        PoseMsg ps;
        ps.header.stamp    = msgLeft->header.stamp;
        ps.header.frame_id = "map";
        ps.pose.position.x = static_cast<double>(Twc.at<float>(0,3));
        ps.pose.position.y = static_cast<double>(Twc.at<float>(1,3));
        ps.pose.position.z = static_cast<double>(Twc.at<float>(2,3));
        cv::Mat R = Twc.rowRange(0,3).colRange(0,3);
        float tr = R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2);
        float qw, qx, qy, qz;
        if (tr > 0.0f) {
            float s = sqrtf(tr + 1.0f) * 2.0f;
            qw = 0.25f * s;
            qx = (R.at<float>(2,1) - R.at<float>(1,2)) / s;
            qy = (R.at<float>(0,2) - R.at<float>(2,0)) / s;
            qz = (R.at<float>(1,0) - R.at<float>(0,1)) / s;
        } else if (R.at<float>(0,0) > R.at<float>(1,1) && R.at<float>(0,0) > R.at<float>(2,2)) {
            float s = sqrtf(1.0f + R.at<float>(0,0) - R.at<float>(1,1) - R.at<float>(2,2)) * 2.0f;
            qw = (R.at<float>(2,1) - R.at<float>(1,2)) / s;
            qx = 0.25f * s;
            qy = (R.at<float>(0,1) + R.at<float>(1,0)) / s;
            qz = (R.at<float>(0,2) + R.at<float>(2,0)) / s;
        } else if (R.at<float>(1,1) > R.at<float>(2,2)) {
            float s = sqrtf(1.0f + R.at<float>(1,1) - R.at<float>(0,0) - R.at<float>(2,2)) * 2.0f;
            qw = (R.at<float>(0,2) - R.at<float>(2,0)) / s;
            qx = (R.at<float>(0,1) + R.at<float>(1,0)) / s;
            qy = 0.25f * s;
            qz = (R.at<float>(1,2) + R.at<float>(2,1)) / s;
        } else {
            float s = sqrtf(1.0f + R.at<float>(2,2) - R.at<float>(0,0) - R.at<float>(1,1)) * 2.0f;
            qw = (R.at<float>(1,0) - R.at<float>(0,1)) / s;
            qx = (R.at<float>(0,2) + R.at<float>(2,0)) / s;
            qy = (R.at<float>(1,2) + R.at<float>(2,1)) / s;
            qz = 0.25f * s;
        }
        ps.pose.orientation.x = static_cast<double>(qx);
        ps.pose.orientation.y = static_cast<double>(qy);
        ps.pose.orientation.z = static_cast<double>(qz);
        ps.pose.orientation.w = static_cast<double>(qw);
        pose_pub_->publish(ps);
    }
}
