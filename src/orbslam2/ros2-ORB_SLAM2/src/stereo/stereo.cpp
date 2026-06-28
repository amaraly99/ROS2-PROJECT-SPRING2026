#include<iostream>
#include<csignal>

#include<opencv2/core/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.hpp>

#include "BenchmarkUtils.h"
#include "System.h"

using namespace std;

using namespace std::placeholders;

using ImageMsg = sensor_msgs::msg::Image;

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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("orbslam");
    ORB_SLAM2::SetBenchmarkThreadName("ORBFrontEnd");

    if(argc != 3)
    {
        cerr << endl << "Usage: ros2 run orbslam stereo path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO);
    g_slam = &SLAM;
    ImageGrabber igb(&SLAM);

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
        mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, timestamp);
    }
}
