"""Generate the ORB-SLAM3 RGB-D wrapper files from the existing ones (2026-10-03).
Run on the Pi from src/orbslam3. Writes NEW files only:
  include/ros2_orb_slam3/common_rgbd.hpp  = common.hpp  + RgbdMode declaration
  src/common_rgbd.cpp                     = common.cpp  (include switched) + RgbdMode implementation
  src/rgbd_example.cpp                    = stereo_example.cpp with RgbdMode
common.hpp / common.cpp / stereo_example.cpp are read, never written."""
from pathlib import Path

hpp = Path('include/ros2_orb_slam3/common.hpp').read_text()
cpp = Path('src/common.cpp').read_text()
ex = Path('src/stereo_example.cpp').read_text()

DECL = '''
// RGB-D (added 2026-10-03, common_rgbd.hpp only): mirror of StereoMode with a color
// image + its registered depth, paired by exact stamp, fed to TrackRGBD.
class RgbdMode : public OrbSlamNodeBase
{
public:
    RgbdMode();

private:
    void RgbImg_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void DepthImg_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void TryMatchRgbdPair(int64_t stamp_ns);
    void ProcessRgbdPair(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        double pair_wait_ms);
    int64_t GetStampNanoseconds(const sensor_msgs::msg::Image &msg) const;
    void PruneRgbdBuffers();

    std::string subRgbImgMsgName;
    std::string subDepthImgMsgName;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription_;
    std::map<int64_t, sensor_msgs::msg::Image::ConstSharedPtr> rgb_image_buffer_;
    std::map<int64_t, sensor_msgs::msg::Image::ConstSharedPtr> depth_image_buffer_;
    std::map<int64_t, std::chrono::steady_clock::time_point> rgb_arrival_times_;
    std::map<int64_t, std::chrono::steady_clock::time_point> depth_arrival_times_;
    std::vector<double> rgbdPairWaitTimesMs_;
    std::size_t maxRgbdBufferSize_;
};
'''

IMPL = r'''

// ===========================================================================
// RGB-D (added 2026-10-03, common_rgbd.cpp only). Line-for-line mirror of the
// StereoMode implementation above: same best-effort subscriptions, same exact-stamp
// pairing buffers (size 20), same PublishPose / timing calls. Differences: inputs are
// a color image (converted to bgr8, so Camera.RGB: 0 is always right) and a 16UC1
// depth image passed through (RGBD.DepthMapFactor scales it), and TrackRGBD.
// ===========================================================================
RgbdMode::RgbdMode()
    : OrbSlamNodeBase(
          "rgbd_node_cpp",
          "/rgbd_py_driver",
          "RGB-D",
          ORB_SLAM3::System::RGBD)
{
    subRgbImgMsgName = topicPrefix + "/rgb_img_msg";
    subDepthImgMsgName = topicPrefix + "/depth_img_msg";
    maxRgbdBufferSize_ = 20;

    rgb_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subRgbImgMsgName,
        rclcpp::SensorDataQoS(),
        std::bind(&RgbdMode::RgbImg_callback, this, std::placeholders::_1));
    depth_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subDepthImgMsgName,
        rclcpp::SensorDataQoS(),
        std::bind(&RgbdMode::DepthImg_callback, this, std::placeholders::_1));
}

int64_t RgbdMode::GetStampNanoseconds(const sensor_msgs::msg::Image &msg) const
{
    return (static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL) +
           static_cast<int64_t>(msg.header.stamp.nanosec);
}

void RgbdMode::PruneRgbdBuffers()
{
    while (rgb_image_buffer_.size() > maxRgbdBufferSize_)
    {
        const int64_t stamp_ns = rgb_image_buffer_.begin()->first;
        rgb_image_buffer_.erase(rgb_image_buffer_.begin());
        rgb_arrival_times_.erase(stamp_ns);
    }

    while (depth_image_buffer_.size() > maxRgbdBufferSize_)
    {
        const int64_t stamp_ns = depth_image_buffer_.begin()->first;
        depth_image_buffer_.erase(depth_image_buffer_.begin());
        depth_arrival_times_.erase(stamp_ns);
    }
}

void RgbdMode::RgbImg_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    const int64_t stamp_ns = GetStampNanoseconds(*msg);
    rgb_image_buffer_[stamp_ns] = msg;
    rgb_arrival_times_[stamp_ns] = std::chrono::steady_clock::now();
    TryMatchRgbdPair(stamp_ns);
    PruneRgbdBuffers();
}

void RgbdMode::DepthImg_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    const int64_t stamp_ns = GetStampNanoseconds(*msg);
    depth_image_buffer_[stamp_ns] = msg;
    depth_arrival_times_[stamp_ns] = std::chrono::steady_clock::now();
    TryMatchRgbdPair(stamp_ns);
    PruneRgbdBuffers();
}

void RgbdMode::TryMatchRgbdPair(int64_t stamp_ns)
{
    const auto rgb_it = rgb_image_buffer_.find(stamp_ns);
    const auto depth_it = depth_image_buffer_.find(stamp_ns);
    if (rgb_it == rgb_image_buffer_.end() || depth_it == depth_image_buffer_.end())
    {
        return;
    }

    double pair_wait_ms = 0.0;
    const auto rgb_arrival_it = rgb_arrival_times_.find(stamp_ns);
    const auto depth_arrival_it = depth_arrival_times_.find(stamp_ns);
    if (rgb_arrival_it != rgb_arrival_times_.end() && depth_arrival_it != depth_arrival_times_.end())
    {
        const auto wait_delta = (rgb_arrival_it->second > depth_arrival_it->second)
                                    ? (rgb_arrival_it->second - depth_arrival_it->second)
                                    : (depth_arrival_it->second - rgb_arrival_it->second);
        pair_wait_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(wait_delta).count();
        rgbdPairWaitTimesMs_.push_back(pair_wait_ms);
    }

    const auto rgb_msg = rgb_it->second;
    const auto depth_msg = depth_it->second;
    rgb_image_buffer_.erase(rgb_it);
    depth_image_buffer_.erase(depth_it);
    rgb_arrival_times_.erase(stamp_ns);
    depth_arrival_times_.erase(stamp_ns);
    ProcessRgbdPair(rgb_msg, depth_msg, pair_wait_ms);
}

void RgbdMode::ProcessRgbdPair(
    const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
    double pair_wait_ms)
{
    if (!initialized_ || pAgent == nullptr)
    {
        return;
    }

    const auto callback_start = std::chrono::steady_clock::now();
    cv_bridge::CvImageConstPtr rgb_cv_ptr;
    cv_bridge::CvImageConstPtr depth_cv_ptr;

    try
    {
        rgb_cv_ptr = cv_bridge::toCvShare(rgb_msg, "bgr8");   // mono8/rgb8 converted; Camera.RGB: 0
        depth_cv_ptr = cv_bridge::toCvShare(depth_msg);        // 16UC1 mm, passed through
    }
    catch (const cv_bridge::Exception &)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading RGB-D image pair");
        return;
    }

    const auto after_bridge = std::chrono::steady_clock::now();
    double timestamp = GetImageTimestampSeconds(*rgb_msg);
    if (timestamp <= 0.0)
    {
        timestamp = GetImageTimestampSeconds(*depth_msg);
    }

    const Sophus::SE3f Tcw = pAgent->TrackRGBD(rgb_cv_ptr->image, depth_cv_ptr->image, timestamp);
    const auto after_track = std::chrono::steady_clock::now();
    PublishPose(Tcw, timestamp);
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
'''

assert hpp.rstrip().endswith('};')
Path('include/ros2_orb_slam3/common_rgbd.hpp').write_text(
    '// common_rgbd.hpp: GENERATED 2026-10-03 from common.hpp + RgbdMode. Do not hand-edit;\n'
    '// regenerate with make_orb3_rgbd.py if common.hpp changes.\n' + hpp.rstrip() + '\n' + DECL)
inc = '#include "ros2_orb_slam3/common.hpp"'
assert cpp.count(inc) == 1
Path('src/common_rgbd.cpp').write_text(
    '// common_rgbd.cpp: GENERATED 2026-10-03 from common.cpp + RgbdMode. Do not hand-edit;\n'
    '// regenerate with make_orb3_rgbd.py if common.cpp changes.\n'
    + cpp.replace(inc, '#include "ros2_orb_slam3/common_rgbd.hpp"').rstrip() + '\n' + IMPL)
assert ex.count('StereoMode') == 1 and ex.count('common.hpp') == 1
Path('src/rgbd_example.cpp').write_text(
    ex.replace('ROS2 stereo wrapper entrypoint', 'ROS2 RGB-D wrapper entrypoint (2026-10-03)')
      .replace('ros2_orb_slam3/common.hpp', 'ros2_orb_slam3/common_rgbd.hpp')
      .replace('StereoMode', 'RgbdMode'))
print('generated common_rgbd.hpp, common_rgbd.cpp, rgbd_example.cpp')
