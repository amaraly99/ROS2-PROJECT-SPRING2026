#ifndef ORB_SLAM2_BENCHMARKUTILS_H
#define ORB_SLAM2_BENCHMARKUTILS_H

#include <chrono>
#include <functional>
#include <string>

namespace ORB_SLAM2
{

bool BenchmarkTimingEnabled();

void SetBenchmarkThreadName(const char* name);
std::string GetBenchmarkThreadName();

class ScopedBenchmarkTimer
{
public:
    typedef std::function<long()> LongProvider;
    typedef std::function<int()> IntProvider;

    ScopedBenchmarkTimer(
        const std::string& category,
        LongProvider frame_id_provider = LongProvider(),
        LongProvider keyframe_id_provider = LongProvider(),
        IntProvider tracking_state_provider = IntProvider());

    ~ScopedBenchmarkTimer();

    ScopedBenchmarkTimer(const ScopedBenchmarkTimer&) = delete;
    ScopedBenchmarkTimer& operator=(const ScopedBenchmarkTimer&) = delete;

private:
    bool mbEnabled;
    std::string mCategory;
    LongProvider mFrameIdProvider;
    LongProvider mKeyframeIdProvider;
    IntProvider mTrackingStateProvider;
    std::chrono::steady_clock::time_point mStartTime;
};

} // namespace ORB_SLAM2

#endif // ORB_SLAM2_BENCHMARKUTILS_H
