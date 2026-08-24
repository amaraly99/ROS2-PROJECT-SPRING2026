/*

A ROS2 wrapper around ORB-SLAM3 supporting mono and stereo inputs.

Author: Azmyin Md. Kamal
Date: 01/01/24

*/

#include "ros2_orb_slam3/common.hpp"
#include <cstdlib>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <vector>

#include <opencv2/core.hpp>

namespace {
std::string GetUtcPlus4TimestampForFilename()
{
    const auto utc_plus_4_now = std::chrono::system_clock::now() + std::chrono::hours(4);
    const std::time_t time_now = std::chrono::system_clock::to_time_t(utc_plus_4_now);

    std::tm utc_plus_4_tm{};
    gmtime_r(&time_now, &utc_plus_4_tm);

    std::ostringstream stream;
    stream << std::put_time(&utc_plus_4_tm, "%Y%m%d_%H%M%S") << "_UTCp4";
    return stream.str();
}

double MeanOrZero(const std::vector<double> &values)
{
    if (values.empty())
    {
        return 0.0;
    }

    double sum = 0.0;
    for (const double value : values)
    {
        sum += value;
    }

    return sum / static_cast<double>(values.size());
}
}

OrbSlamNodeBase::OrbSlamNodeBase(
    const std::string &node_name,
    const std::string &topic_prefix,
    const std::string &config_subdirectory,
    ORB_SLAM3::System::eSensor sensor_type)
    : Node(node_name),
      topicPrefix(topic_prefix),
      configSubdirectory(config_subdirectory),
      sensorType(sensor_type),
      pAgent(nullptr),
      bSettingsFromPython(false),
      enablePangolinWindow(false),
      enableOpenCVWindow(false),
      initialized_(false),
      finalized_(false),
      publishMapPoints_(false),
      shutdownScheduled_(false),
      lastShutdownAckSuccess_(false),
      timeStep(0.0)
{
    packagePath = "/workspace/src/";

    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 ROS2 node started");

    this->declare_parameter("node_name_arg", "not_given");
    this->declare_parameter("voc_file_arg", "file_not_set");
    this->declare_parameter("settings_file_path_arg", "file_path_not_set");
    this->declare_parameter("publish_map_points", false);

    nodeName = this->get_parameter("node_name_arg").as_string();
    vocFilePath = this->get_parameter("voc_file_arg").as_string();
    settingsFilePathBase = this->get_parameter("settings_file_path_arg").as_string();
    publishMapPoints_ = this->get_parameter("publish_map_points").as_bool();

    if (vocFilePath == "file_not_set")
    {
        vocFilePath = packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    }

    if (settingsFilePathBase == "file_path_not_set")
    {
        settingsFilePathBase = packagePath + "orb_slam3/config/" + configSubdirectory + "/";
    }

    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePathBase.c_str());

    subexperimentconfigName = topicPrefix + "/experiment_settings";
    pubconfigackName = topicPrefix + "/exp_settings_ack";
    subTimestepMsgName = topicPrefix + "/timestep_msg";
    subShutdownMsgName = topicPrefix + "/shutdown";
    pubPointCloudName = topicPrefix + "/map_points";
    // HIL: relative names so the stack yaml can remap them, matching the
    // orbslam2 config convention (slam_pose:=/slam/pose).
    pubPoseName = "slam_pose";
    pubTrackingStateName = "slam_tracking_state";
    shutdownServiceName = topicPrefix + "/shutdown_request";

    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
        subexperimentconfigName,
        1,
        std::bind(&OrbSlamNodeBase::experimentSetting_callback, this, std::placeholders::_1));

    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pubPoseName, 10);          // HIL
    trackingState_publisher_ = this->create_publisher<std_msgs::msg::Int32>(pubTrackingStateName, 10);  // HIL

    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        subTimestepMsgName,
        10,
        std::bind(&OrbSlamNodeBase::Timestep_callback, this, std::placeholders::_1));

    shutdown_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        subShutdownMsgName,
        10,
        std::bind(&OrbSlamNodeBase::Shutdown_callback, this, std::placeholders::_1));

    if (publishMapPoints_)
    {
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pubPointCloudName, 1);
        RCLCPP_INFO(this->get_logger(), "Map point publishing enabled on %s", pubPointCloudName.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Map point publishing disabled");
    }
    shutdown_service_ = this->create_service<std_srvs::srv::Trigger>(
        shutdownServiceName,
        std::bind(&OrbSlamNodeBase::ShutdownService_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake...");
}

OrbSlamNodeBase::~OrbSlamNodeBase()
{
    FinalizeAndShutdown("destructor", false, nullptr);
}

void OrbSlamNodeBase::experimentSetting_callback(const std_msgs::msg::String &msg)
{
    bSettingsFromPython = true;
    ParseExperimentSettings(msg.data);

    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", experimentConfig.c_str());
    if (!experimentBagLabel.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Bag label: %s", experimentBagLabel.c_str());
    }

    if (!initialized_)
    {
        initializeVSLAM(experimentConfig);
    }

    if (!initialized_)
    {
        RCLCPP_ERROR(this->get_logger(), "ORB-SLAM3 initialization failed, withholding ACK");
        return;
    }

    auto ack_message = std_msgs::msg::String();
    ack_message.data = "ACK";
    configAck_publisher_->publish(ack_message);
}

void OrbSlamNodeBase::Timestep_callback(const std_msgs::msg::Float64 &time_msg)
{
    timeStep = time_msg.data;
}

void OrbSlamNodeBase::Shutdown_callback(const std_msgs::msg::Bool &msg)
{
    if (!msg.data)
    {
        return;
    }

    lastShutdownReason = "legacy shutdown topic";
    RCLCPP_INFO(this->get_logger(), "Received legacy shutdown signal");

    std::string status_message;
    const bool success = FinalizeAndShutdown("shutdown_topic", false, &status_message);

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "%s", status_message.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "%s", status_message.c_str());
    }

    ScheduleProcessShutdown(lastShutdownReason);
}

void OrbSlamNodeBase::ShutdownService_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    lastShutdownReason = "shutdown_request service";
    std::string status_message;

    response->success = FinalizeAndShutdown("shutdown_service", true, &status_message);
    response->message = status_message;

    if (response->success)
    {
        ScheduleProcessShutdown(lastShutdownReason);
    }
}

void OrbSlamNodeBase::initializeVSLAM(const std::string &config_string)
{
    if (initialized_)
    {
        return;
    }

    if (vocFilePath == "file_not_set" || settingsFilePathBase == "file_path_not_set")
    {
        RCLCPP_ERROR(this->get_logger(), "Please provide valid voc_file and settings_file paths");
        rclcpp::shutdown();
        return;
    }

    const std::string settings_file_path = settingsFilePathBase + config_string + ".yaml";
    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settings_file_path.c_str());

    settingsFilePathBase = settings_file_path;
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePathBase, sensorType, enablePangolinWindow);
    initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 node initialized");
}

bool OrbSlamNodeBase::FinalizeAndShutdown(
    const std::string &shutdown_origin,
    bool shutdown_ack_success,
    std::string *status_message)
{
    if (finalized_)
    {
        if (status_message != nullptr)
        {
            std::ostringstream stream;
            stream << "ORB-SLAM3 already finalized";
            if (!lastCameraTrajectoryFile.empty() || !lastKeyFrameTrajectoryFile.empty())
            {
                stream << " (camera=" << lastCameraTrajectoryFile
                       << ", keyframes=" << lastKeyFrameTrajectoryFile << ")";
            }
            *status_message = stream.str();
        }
        return true;
    }

    lastShutdownOrigin = shutdown_origin;
    lastShutdownAckSuccess_ = shutdown_ack_success;

    if (pAgent == nullptr)
    {
        finalized_ = true;
        if (status_message != nullptr)
        {
            *status_message = "ORB-SLAM3 was not initialized; nothing to finalize.";
        }
        return true;
    }

    const std::string timestamp = GetUtcPlus4TimestampForFilename();
    lastCameraTrajectoryFile = "CameraTrajectory_" + timestamp + ".txt";
    lastKeyFrameTrajectoryFile = "KeyFrameTrajectory_" + timestamp + ".txt";
    lastMetadataFile = "RunMetadata_" + timestamp + ".txt";
    lastWrapperTimingFile = "WrapperTimingStats_" + timestamp + ".txt";

    pAgent->Shutdown();
    pAgent->SaveTrajectoryTUM(lastCameraTrajectoryFile);
    pAgent->SaveKeyFrameTrajectoryTUM(lastKeyFrameTrajectoryFile);

    const bool metadata_written = WriteRunMetadata(lastMetadataFile);
    const bool timing_written = WriteWrapperTimingStats(lastWrapperTimingFile);

    delete pAgent;
    pAgent = nullptr;
    finalized_ = true;

    if (status_message != nullptr)
    {
        std::ostringstream stream;
        stream << "Saved trajectories to " << lastCameraTrajectoryFile
               << " and " << lastKeyFrameTrajectoryFile;
        if (metadata_written)
        {
            stream << "; metadata: " << lastMetadataFile;
        }
        else
        {
            stream << "; metadata write failed";
        }
        if (timing_written)
        {
            stream << "; wrapper timings: " << lastWrapperTimingFile;
        }
        else
        {
            stream << "; wrapper timing write failed";
        }
        *status_message = stream.str();
    }

    return true;
}

void OrbSlamNodeBase::ScheduleProcessShutdown(const std::string &reason)
{
    if (shutdownScheduled_)
    {
        return;
    }

    shutdownScheduled_ = true;
    lastShutdownReason = reason;

    deferred_shutdown_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() {
            if (deferred_shutdown_timer_ != nullptr)
            {
                deferred_shutdown_timer_->cancel();
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Shutting down process after finalize (origin=%s, reason=%s)",
                lastShutdownOrigin.c_str(),
                lastShutdownReason.c_str());
            rclcpp::shutdown();
        });
}

void OrbSlamNodeBase::ParseExperimentSettings(const std::string &message_data)
{
    experimentConfig = message_data;
    experimentBagLabel.clear();

    const std::size_t separator = message_data.find('|');
    if (separator == std::string::npos)
    {
        return;
    }

    experimentConfig = message_data.substr(0, separator);
    experimentBagLabel = message_data.substr(separator + 1);
}

bool OrbSlamNodeBase::WriteRunMetadata(const std::string &metadata_file_path) const
{
    std::ofstream metadata_stream(metadata_file_path);
    if (!metadata_stream.is_open())
    {
        return false;
    }

    double camera_fps = 0.0;
    double stereo_th_depth = 0.0;
    int orb_nfeatures = 0;
    double orb_scale_factor = 0.0;
    int orb_nlevels = 0;
    int orb_ini_fast = 0;
    int orb_min_fast = 0;

    cv::FileStorage settings_file(settingsFilePathBase, cv::FileStorage::READ);
    if (settings_file.isOpened())
    {
        settings_file["Camera.fps"] >> camera_fps;
        settings_file["Stereo.ThDepth"] >> stereo_th_depth;
        settings_file["ORBextractor.nFeatures"] >> orb_nfeatures;
        settings_file["ORBextractor.scaleFactor"] >> orb_scale_factor;
        settings_file["ORBextractor.nLevels"] >> orb_nlevels;
        settings_file["ORBextractor.iniThFAST"] >> orb_ini_fast;
        settings_file["ORBextractor.minThFAST"] >> orb_min_fast;
    }

    metadata_stream << std::boolalpha;
    metadata_stream << "shutdown_origin: " << lastShutdownOrigin << '\n';
    metadata_stream << "shutdown_reason: " << lastShutdownReason << '\n';
    metadata_stream << "shutdown_ack_success: " << lastShutdownAckSuccess_ << '\n';
    metadata_stream << "config_name: " << experimentConfig << '\n';
    metadata_stream << "bag_label: " << experimentBagLabel << '\n';
    metadata_stream << "settings_file: " << settingsFilePathBase << '\n';
    metadata_stream << "camera_trajectory_file: " << lastCameraTrajectoryFile << '\n';
    metadata_stream << "keyframe_trajectory_file: " << lastKeyFrameTrajectoryFile << '\n';
    metadata_stream << "wrapper_timing_file: " << lastWrapperTimingFile << '\n';
    metadata_stream << "orbslam_exec_mean_file: ExecMean.txt" << '\n';
    metadata_stream << "orbslam_tracking_stats_file: TrackingTimeStats.txt" << '\n';
    metadata_stream << "orbslam_local_mapping_stats_file: LocalMapTimeStats.txt" << '\n';
    metadata_stream << "orbslam_lba_stats_file: LBA_Stats.txt" << '\n';
    metadata_stream << "camera_fps: " << camera_fps << '\n';
    metadata_stream << "stereo_th_depth: " << stereo_th_depth << '\n';
    metadata_stream << "orb_nfeatures: " << orb_nfeatures << '\n';
    metadata_stream << "orb_scale_factor: " << orb_scale_factor << '\n';
    metadata_stream << "orb_nlevels: " << orb_nlevels << '\n';
    metadata_stream << "orb_ini_fast: " << orb_ini_fast << '\n';
    metadata_stream << "orb_min_fast: " << orb_min_fast << '\n';
    metadata_stream << "publish_map_points: " << publishMapPoints_ << '\n';
    metadata_stream << "wrapper_callback_prep_mean_ms: " << MeanOrZero(callbackPrepTimesMs_) << '\n';
    metadata_stream << "wrapper_track_call_mean_ms: " << MeanOrZero(trackCallTimesMs_) << '\n';
    metadata_stream << "wrapper_callback_total_mean_ms: " << MeanOrZero(callbackTotalTimesMs_) << '\n';

    return true;
}

bool OrbSlamNodeBase::WriteWrapperTimingStats(const std::string &stats_file_path) const
{
    std::ofstream stats_stream(stats_file_path);
    if (!stats_stream.is_open())
    {
        return false;
    }

    const std::size_t sample_count = std::min(
        callbackPrepTimesMs_.size(),
        std::min(trackCallTimesMs_.size(), callbackTotalTimesMs_.size()));

    stats_stream << std::fixed << std::setprecision(6);
    stats_stream << "#callback_prep_ms,track_call_ms,callback_total_ms\n";
    for (std::size_t index = 0; index < sample_count; ++index)
    {
        stats_stream << callbackPrepTimesMs_[index] << ','
                     << trackCallTimesMs_[index] << ','
                     << callbackTotalTimesMs_[index] << '\n';
    }

    return true;
}

void OrbSlamNodeBase::RecordCallbackTiming(
    double callback_prep_ms,
    double track_call_ms,
    double callback_total_ms)
{
    callbackPrepTimesMs_.push_back(callback_prep_ms);
    trackCallTimesMs_.push_back(track_call_ms);
    callbackTotalTimesMs_.push_back(callback_total_ms);

    // HIL: append per-frame DURING the run, not only at shutdown.
    //
    // Upstream flushes these vectors from the shutdown path only, and the HIL
    // stack tears the sidecar down with a forced container removal (SIGKILL).
    // The shutdown block therefore never executes and nothing was ever written,
    // leaving ORB-SLAM3 with no latency column while OV2SLAM and ORB-SLAM2 both
    // append per frame and survive the kill.
    //
    // Schema and category names match ORB-SLAM2's ScopedBenchmarkTimer output so
    // the benchmark aggregator reads all three backends with no per-backend
    // special-casing. track_call_ms is the time inside TrackMonocular -- feature
    // extraction plus pose estimation -- which is the layer corresponding to
    // OV2SLAM's frontend/full_tracking. Comparing any other layer would
    // understate one backend against the other by roughly 3x.
    static std::ofstream hil_csv;
    static bool hil_csv_tried = false;
    if (!hil_csv_tried)
    {
        hil_csv_tried = true;
        const char *csv_path = std::getenv("ORB_BENCH_TIMING_CSV");
        if (csv_path != nullptr && csv_path[0] != 0)
        {
            hil_csv.open(csv_path, std::ios::out | std::ios::trunc);
            if (hil_csv.is_open())
            {
                hil_csv << "wall_ts,thread_role,category,duration_ms,frame_id,"
                        << "keyframe_id,tracking_state" << std::endl;
            }
        }
    }
    if (hil_csv.is_open())
    {
        const double wall = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const int state = (pAgent != nullptr) ? pAgent->GetTrackingState() : -1;
        const long fid = static_cast<long>(trackCallTimesMs_.size()) - 1;
        hil_csv.setf(std::ios::fixed);
        hil_csv.precision(6);
        hil_csv << wall << ",ORB3FrontEnd,frontend/full_tracking," << track_call_ms
                << "," << fid << ",," << state << "\n";
        hil_csv << wall << ",ORB3FrontEnd,wrapper/callback_total," << callback_total_ms
                << "," << fid << ",," << state << "\n";
        hil_csv << wall << ",ORB3FrontEnd,wrapper/callback_prep," << callback_prep_ms
                << "," << fid << ",," << state << "\n";
        hil_csv.flush();   // per frame: the process is SIGKILLed at teardown
    }
}

double OrbSlamNodeBase::GetImageTimestampSeconds(const sensor_msgs::msg::Image &msg) const
{
    if (msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0)
    {
        return static_cast<double>(msg.header.stamp.sec) +
               static_cast<double>(msg.header.stamp.nanosec) / 1e9;
    }

    return timeStep;
}

// HIL: convert the SE3 returned by TrackMonocular into the stack's pose contract.
// ORB-SLAM3 returns Tcw (world -> camera); every consumer here wants the camera
// pose in the world frame, so invert it. Pose is published ONLY while tracking is
// healthy (state 2 == OK): during initialisation the returned transform is not a
// valid pose, and emitting identity would look like a real measurement at the
// origin to the evaluator.
void OrbSlamNodeBase::PublishPose(const Sophus::SE3f &Tcw, double timestamp_seconds)
{
    if (pAgent == nullptr)
    {
        return;
    }

    const int state = pAgent->GetTrackingState();
    if (trackingState_publisher_ != nullptr)
    {
        std_msgs::msg::Int32 state_msg;
        state_msg.data = state;
        trackingState_publisher_->publish(state_msg);
    }

    if (pose_publisher_ == nullptr || state != 2)
    {
        return;
    }

    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp.sec = static_cast<int32_t>(timestamp_seconds);
    msg.header.stamp.nanosec =
        static_cast<uint32_t>((timestamp_seconds - static_cast<double>(msg.header.stamp.sec)) * 1e9);
    msg.header.frame_id = "map";
    msg.pose.position.x = t.x();
    msg.pose.position.y = t.y();
    msg.pose.position.z = t.z();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    pose_publisher_->publish(msg);
}

void OrbSlamNodeBase::PublishMapPointCloud(double timestamp_seconds)
{
    if (!publishMapPoints_ || pointcloud_publisher_ == nullptr || !initialized_ || pAgent == nullptr)
    {
        return;
    }

    const auto map_points = pAgent->GetAllMapPoints();
    std::vector<Eigen::Vector3f> valid_points;
    valid_points.reserve(map_points.size());

    for (auto *map_point : map_points)
    {
        if (map_point == nullptr || map_point->isBad())
        {
            continue;
        }

        const Eigen::Vector3f world_point = map_point->GetWorldPos();
        if (!std::isfinite(world_point.x()) || !std::isfinite(world_point.y()) || !std::isfinite(world_point.z()))
        {
            continue;
        }

        valid_points.push_back(world_point);
    }

    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pointcloud_msg.header.frame_id = "map";

    if (timestamp_seconds > 0.0)
    {
        const int64_t stamp_ns = static_cast<int64_t>(std::llround(timestamp_seconds * 1e9));
        pointcloud_msg.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
        pointcloud_msg.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    }
    else
    {
        const int64_t now_ns = this->now().nanoseconds();
        pointcloud_msg.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
        pointcloud_msg.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
    }

    sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(valid_points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");

    for (const auto &point : valid_points)
    {
        *iter_x = point.x();
        *iter_y = point.y();
        *iter_z = point.z();

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    pointcloud_publisher_->publish(pointcloud_msg);
}

MonocularMode::MonocularMode()
    : OrbSlamNodeBase(
          "mono_node_cpp",
          "/mono_py_driver",
          "Monocular",
          ORB_SLAM3::System::MONOCULAR)
{
    subImgMsgName = topicPrefix + "/img_msg";
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName,
        rclcpp::SensorDataQoS(),
        std::bind(&MonocularMode::Img_callback, this, std::placeholders::_1));
}

void MonocularMode::Img_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if (!initialized_ || pAgent == nullptr)
    {
        return;
    }

    const auto callback_start = std::chrono::steady_clock::now();
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    }
    catch (const cv_bridge::Exception &)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading image");
        return;
    }

    const auto after_bridge = std::chrono::steady_clock::now();
    const double timestamp = GetImageTimestampSeconds(*msg);
    const Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timestamp);  // HIL: was discarded
    const auto after_track = std::chrono::steady_clock::now();
    PublishPose(Tcw, timestamp);  // HIL
    PublishMapPointCloud(timestamp);

    RecordCallbackTiming(
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(after_bridge - callback_start).count(),
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(after_track - after_bridge).count(),
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(after_track - callback_start).count());
}

StereoMode::StereoMode()
    : OrbSlamNodeBase(
          "stereo_node_cpp",
          "/stereo_py_driver",
          "Stereo",
          ORB_SLAM3::System::STEREO)
{
    subLeftImgMsgName = topicPrefix + "/left_img_msg";
    subRightImgMsgName = topicPrefix + "/right_img_msg";
    maxStereoBufferSize_ = 20;

    left_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subLeftImgMsgName,
        rclcpp::SensorDataQoS(),
        std::bind(&StereoMode::LeftImg_callback, this, std::placeholders::_1));
    right_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subRightImgMsgName,
        rclcpp::SensorDataQoS(),
        std::bind(&StereoMode::RightImg_callback, this, std::placeholders::_1));
}

int64_t StereoMode::GetStampNanoseconds(const sensor_msgs::msg::Image &msg) const
{
    return (static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL) +
           static_cast<int64_t>(msg.header.stamp.nanosec);
}

void StereoMode::PruneStereoBuffers()
{
    while (left_image_buffer_.size() > maxStereoBufferSize_)
    {
        const int64_t stamp_ns = left_image_buffer_.begin()->first;
        left_image_buffer_.erase(left_image_buffer_.begin());
        left_arrival_times_.erase(stamp_ns);
    }

    while (right_image_buffer_.size() > maxStereoBufferSize_)
    {
        const int64_t stamp_ns = right_image_buffer_.begin()->first;
        right_image_buffer_.erase(right_image_buffer_.begin());
        right_arrival_times_.erase(stamp_ns);
    }
}

void StereoMode::LeftImg_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    const int64_t stamp_ns = GetStampNanoseconds(*msg);
    left_image_buffer_[stamp_ns] = msg;
    left_arrival_times_[stamp_ns] = std::chrono::steady_clock::now();
    TryMatchStereoPair(stamp_ns);
    PruneStereoBuffers();
}

void StereoMode::RightImg_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    const int64_t stamp_ns = GetStampNanoseconds(*msg);
    right_image_buffer_[stamp_ns] = msg;
    right_arrival_times_[stamp_ns] = std::chrono::steady_clock::now();
    TryMatchStereoPair(stamp_ns);
    PruneStereoBuffers();
}

void StereoMode::TryMatchStereoPair(int64_t stamp_ns)
{
    const auto left_it = left_image_buffer_.find(stamp_ns);
    const auto right_it = right_image_buffer_.find(stamp_ns);
    if (left_it == left_image_buffer_.end() || right_it == right_image_buffer_.end())
    {
        return;
    }

    double pair_wait_ms = 0.0;
    const auto left_arrival_it = left_arrival_times_.find(stamp_ns);
    const auto right_arrival_it = right_arrival_times_.find(stamp_ns);
    if (left_arrival_it != left_arrival_times_.end() && right_arrival_it != right_arrival_times_.end())
    {
        const auto wait_delta = (left_arrival_it->second > right_arrival_it->second)
                                    ? (left_arrival_it->second - right_arrival_it->second)
                                    : (right_arrival_it->second - left_arrival_it->second);
        pair_wait_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(wait_delta).count();
        stereoPairWaitTimesMs_.push_back(pair_wait_ms);
    }

    const auto left_msg = left_it->second;
    const auto right_msg = right_it->second;
    left_image_buffer_.erase(left_it);
    right_image_buffer_.erase(right_it);
    left_arrival_times_.erase(stamp_ns);
    right_arrival_times_.erase(stamp_ns);
    ProcessStereoPair(left_msg, right_msg, pair_wait_ms);
}

void StereoMode::ProcessStereoPair(
    const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &right_msg,
    double pair_wait_ms)
{
    if (!initialized_ || pAgent == nullptr)
    {
        return;
    }

    const auto callback_start = std::chrono::steady_clock::now();
    cv_bridge::CvImageConstPtr left_cv_ptr;
    cv_bridge::CvImageConstPtr right_cv_ptr;

    try
    {
        left_cv_ptr = cv_bridge::toCvShare(left_msg, left_msg->encoding);
        right_cv_ptr = cv_bridge::toCvShare(right_msg, right_msg->encoding);
    }
    catch (const cv_bridge::Exception &)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading stereo image pair");
        return;
    }

    const auto after_bridge = std::chrono::steady_clock::now();
    double timestamp = GetImageTimestampSeconds(*left_msg);
    if (timestamp <= 0.0)
    {
        timestamp = GetImageTimestampSeconds(*right_msg);
    }

    pAgent->TrackStereo(left_cv_ptr->image, right_cv_ptr->image, timestamp);
    const auto after_track = std::chrono::steady_clock::now();
    PublishMapPointCloud(timestamp);

    RecordCallbackTiming(
        pair_wait_ms +
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(after_bridge - callback_start)
                .count(),
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(after_track - after_bridge).count(),
        pair_wait_ms +
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(after_track - callback_start)
                .count());
}
