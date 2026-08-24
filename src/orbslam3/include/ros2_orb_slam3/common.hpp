#pragma once

#include <memory>
#include <map>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>   // HIL: /slam/pose
#include <std_msgs/msg/int32.hpp>               // HIL: /slam/tracking_state
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "System.h"

class OrbSlamNodeBase : public rclcpp::Node
{
public:
    OrbSlamNodeBase(
        const std::string &node_name,
        const std::string &topic_prefix,
        const std::string &config_subdirectory,
        ORB_SLAM3::System::eSensor sensor_type);
    ~OrbSlamNodeBase() override;

protected:
    using ImageMsg = sensor_msgs::msg::Image;

    void experimentSetting_callback(const std_msgs::msg::String &msg);
    void Timestep_callback(const std_msgs::msg::Float64 &time_msg);
    void Shutdown_callback(const std_msgs::msg::Bool &msg);
    void ShutdownService_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void initializeVSLAM(const std::string &config_string);
    bool FinalizeAndShutdown(
        const std::string &shutdown_origin,
        bool shutdown_ack_success,
        std::string *status_message);
    void ScheduleProcessShutdown(const std::string &reason);
    void PublishMapPointCloud(double timestamp_seconds);
    double GetImageTimestampSeconds(const sensor_msgs::msg::Image &msg) const;
    void ParseExperimentSettings(const std::string &message_data);
    bool WriteRunMetadata(const std::string &metadata_file_path) const;
    bool WriteWrapperTimingStats(const std::string &stats_file_path) const;
    void RecordCallbackTiming(double callback_prep_ms, double track_call_ms, double callback_total_ms);

    std::string packagePath;
    std::string nodeName;
    std::string vocFilePath;
    std::string settingsFilePathBase;
    std::string experimentConfig;
    std::string experimentBagLabel;
    std::string topicPrefix;
    std::string configSubdirectory;
    std::string shutdownServiceName;
    std::string lastCameraTrajectoryFile;
    std::string lastKeyFrameTrajectoryFile;
    std::string lastMetadataFile;
    std::string lastWrapperTimingFile;
    std::string lastShutdownOrigin;
    std::string lastShutdownReason;

    ORB_SLAM3::System::eSensor sensorType;
    ORB_SLAM3::System *pAgent;

    bool bSettingsFromPython;
    bool enablePangolinWindow;
    bool enableOpenCVWindow;
    bool initialized_;
    bool finalized_;
    bool publishMapPoints_;
    bool shutdownScheduled_;
    bool lastShutdownAckSuccess_;
    double timeStep;
    std::vector<double> callbackPrepTimesMs_;
    std::vector<double> trackCallTimesMs_;
    std::vector<double> callbackTotalTimesMs_;

    std::string subexperimentconfigName;
    std::string pubconfigackName;
    std::string subTimestepMsgName;
    std::string subShutdownMsgName;
    std::string pubPointCloudName;
    std::string pubPoseName;           // HIL
    std::string pubTrackingStateName;  // HIL

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    // HIL: upstream discards the SE3 returned by TrackMonocular and publishes no
    // pose at all -- its trajectory output is a text file written at shutdown.
    // The HIL stack keys every backend off /slam/pose, so we publish it here.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr trackingState_publisher_;
    void PublishPose(const Sophus::SE3f &Tcw, double timestamp_seconds);  // HIL
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_service_;
    rclcpp::TimerBase::SharedPtr deferred_shutdown_timer_;
};

class MonocularMode : public OrbSlamNodeBase
{
public:
    MonocularMode();

private:
    void Img_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    std::string subImgMsgName;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
};

class StereoMode : public OrbSlamNodeBase
{
public:
    StereoMode();

private:
    void LeftImg_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void RightImg_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void TryMatchStereoPair(int64_t stamp_ns);
    void ProcessStereoPair(
        const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &right_msg,
        double pair_wait_ms);
    int64_t GetStampNanoseconds(const sensor_msgs::msg::Image &msg) const;
    void PruneStereoBuffers();

    std::string subLeftImgMsgName;
    std::string subRightImgMsgName;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_subscription_;
    std::map<int64_t, sensor_msgs::msg::Image::ConstSharedPtr> left_image_buffer_;
    std::map<int64_t, sensor_msgs::msg::Image::ConstSharedPtr> right_image_buffer_;
    std::map<int64_t, std::chrono::steady_clock::time_point> left_arrival_times_;
    std::map<int64_t, std::chrono::steady_clock::time_point> right_arrival_times_;
    std::vector<double> stereoPairWaitTimesMs_;
    std::size_t maxStereoBufferSize_;
};
