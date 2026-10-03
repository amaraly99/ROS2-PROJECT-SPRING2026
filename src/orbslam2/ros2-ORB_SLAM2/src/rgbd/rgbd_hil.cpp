// rgbd_hil.cpp -- HIL RGB-D wrapper for ORB-SLAM2 (created 2026-10-01).
// Port of src/stereo/stereo.cpp (the HIL stereo node) to RGB-D. Differences:
//   * inputs camera/rgb + camera/depth, paired by EXACT stamp (MATLAB stamps color
//     and depth from one clock read; TUM bags write both with the RGB stamp);
//   * color is converted to bgr8 (Camera.RGB: 0 in every settings file), so the
//     gray conversion inside Tracking::GrabImageRGBD always uses the right channels;
//   * depth is passed through (16UC1 millimetres -> DepthMapFactor: 1000);
//   * TrackRGBD with the full sub-second stamp (stock rgbd.cpp passed whole seconds).
// Publishes the stack's contract unchanged: slam_pose, slam_tracking_state.
// The stock src/rgbd/rgbd.cpp is left untouched.
#include<iostream>
#include<csignal>

#include<opencv2/core/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"

#include <cv_bridge/cv_bridge.hpp>

#include "BenchmarkUtils.h"
#include "System.h"

using namespace std;

using ImageMsg = sensor_msgs::msg::Image;
using PoseMsg  = geometry_msgs::msg::PoseStamped;
using Int32Msg = std_msgs::msg::Int32;

rclcpp::Node::SharedPtr g_node = nullptr;
ORB_SLAM2::System* g_slam = nullptr;

namespace
{
constexpr const char* kRgbdTrajectoryFile = "RGBD_KeyFrameTrajectory.txt";

void SaveRgbdTrajectory()
{
    if(!g_slam)
    {
        return;
    }

    std::cout << "Saving RGB-D keyframe trajectory to " << kRgbdTrajectoryFile << " ..." << std::endl;
    g_slam->Shutdown();
    g_slam->SaveKeyFrameTrajectoryTUM(kRgbdTrajectoryFile);
    std::cout << "RGB-D trajectory saved!" << std::endl;
    g_slam = nullptr;
}

void HandleShutdownSignal(int signum)
{
    std::cout << "RGB-D wrapper received signal " << signum << ", shutting down ROS ..." << std::endl;
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

    void GrabRGBD(const ImageMsg::ConstSharedPtr msgRGB, const ImageMsg::ConstSharedPtr msgD);

    ORB_SLAM2::System* mpSLAM;

    // HIL contract, identical to the stereo and mono nodes: slam_pose -> /slam/pose,
    // slam_tracking_state -> /slam/tracking_state via the stack yaml's remaps.
    rclcpp::Publisher<PoseMsg>::SharedPtr pose_pub_;
    rclcpp::Publisher<Int32Msg>::SharedPtr tracking_state_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("orbslam");
    ORB_SLAM2::SetBenchmarkThreadName("ORBFrontEnd");

    // run_stack_hil*.sh appends `--ros-args -r ...`; strip ROS args first, as stereo.cpp does.
    auto clean_args = rclcpp::remove_ros_arguments(argc, argv);
    if(clean_args.size() != 3)
    {
        cerr << endl << "Usage: ros2 run orbslam rgbd_hil path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }

    ORB_SLAM2::System SLAM(clean_args[1], clean_args[2], ORB_SLAM2::System::RGBD);
    g_slam = &SLAM;
    ImageGrabber igb(&SLAM);
    igb.pose_pub_           = g_node->create_publisher<PoseMsg>("slam_pose", rclcpp::QoS(10));
    igb.tracking_state_pub_ = g_node->create_publisher<Int32Msg>("slam_tracking_state", rclcpp::QoS(10));

    std::signal(SIGINT, HandleShutdownSignal);
    std::signal(SIGTERM, HandleShutdownSignal);

    // Best-effort (sensor-data QoS): MATLAB publishes color and depth best-effort, and a
    // reliable subscriber never matches a best-effort publisher. Best-effort also accepts
    // reliable publishers (/ovcam, ros2 bag play), so TUM replays keep working.
    message_filters::Subscriber<ImageMsg> rgb_sub(g_node, "camera/rgb", rmw_qos_profile_sensor_data);
    message_filters::Subscriber<ImageMsg> depth_sub(g_node, "camera/depth", rmw_qos_profile_sensor_data);

    using exact_sync_policy = message_filters::sync_policies::ExactTime<ImageMsg, ImageMsg>;
    message_filters::Synchronizer<exact_sync_policy> syncExact(exact_sync_policy(10), rgb_sub, depth_sub);
    syncExact.registerCallback(&ImageGrabber::GrabRGBD, &igb);

    rclcpp::spin(g_node);

    SaveRgbdTrajectory();

    if(rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    g_node = nullptr;

    return 0;
}

void ImageGrabber::GrabRGBD(const ImageMsg::ConstSharedPtr msgRGB, const ImageMsg::ConstSharedPtr msgD)
{
    ORB_SLAM2::SetBenchmarkThreadName("ORBFrontEnd");
    ORB_SLAM2::ScopedBenchmarkTimer callback_total(
        "wrapper/callback_total",
        [this](){ return mpSLAM ? static_cast<long>(mpSLAM->GetCurrentFrame().mnId) : -1L; },
        ORB_SLAM2::ScopedBenchmarkTimer::LongProvider(),
        [this](){ return mpSLAM ? mpSLAM->GetTrackingState() : ORB_SLAM2::Tracking::SYSTEM_NOT_READY; });

    // Color -> bgr8 always (mono8 / rgb8 inputs are converted), so Camera.RGB: 0 is
    // correct for every source.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(g_node->get_logger(), "cv_bridge rgb exception: %s", e.what());
        return;
    }

    // Depth as published (16UC1 mm); Tracking scales it by 1/DepthMapFactor.
    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(g_node->get_logger(), "cv_bridge depth exception: %s", e.what());
        return;
    }

    const double timestamp =
        static_cast<double>(msgRGB->header.stamp.sec) +
        static_cast<double>(msgRGB->header.stamp.nanosec) * 1e-9;

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
        Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, timestamp);
    }

    {
        Int32Msg ts_msg;
        ts_msg.data = static_cast<int>(mpSLAM->GetTrackingState());
        tracking_state_pub_->publish(ts_msg);
    }
    if (!Tcw.empty() && Tcw.rows == 4 && Tcw.cols == 4) {
        cv::Mat Twc = Tcw.inv();
        PoseMsg ps;
        ps.header.stamp    = msgRGB->header.stamp;
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
