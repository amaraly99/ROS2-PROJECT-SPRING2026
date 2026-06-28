#include "BenchmarkUtils.h"

#include <pthread.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <mutex>

namespace ORB_SLAM2
{
namespace
{

const int kUnsetTrackingState = std::numeric_limits<int>::min();

class BenchmarkTimingLogger
{
public:
    static BenchmarkTimingLogger& Instance()
    {
        static BenchmarkTimingLogger logger;
        return logger;
    }

    bool Enabled()
    {
        EnsureInitialized();
        return mbEnabled;
    }

    void Record(
        const std::string& category,
        double duration_ms,
        long frame_id,
        long keyframe_id,
        int tracking_state)
    {
        EnsureInitialized();
        if(!mbEnabled)
            return;

        const double wall_ts =
            std::chrono::duration_cast<std::chrono::duration<double> >(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        const std::string thread_role = GetBenchmarkThreadName();

        std::lock_guard<std::mutex> lock(mWriteMutex);
        if(!mStream.is_open())
            return;

        mStream.setf(std::ios::fixed, std::ios::floatfield);
        mStream.precision(6);
        mStream << wall_ts << ','
                << thread_role << ','
                << category << ','
                << duration_ms << ',';
        if(frame_id >= 0)
            mStream << frame_id;
        mStream << ',';
        if(keyframe_id >= 0)
            mStream << keyframe_id;
        mStream << ',';
        if(tracking_state != kUnsetTrackingState)
            mStream << tracking_state;
        mStream << '\n';
        mStream.flush();
    }

private:
    BenchmarkTimingLogger()
        : mbInitialized(false), mbEnabled(false)
    {
    }

    void EnsureInitialized()
    {
        std::lock_guard<std::mutex> lock(mInitMutex);
        if(mbInitialized)
            return;
        mbInitialized = true;

        const char* raw_path = std::getenv("ORB_BENCH_TIMING_CSV");
        if(!raw_path || !raw_path[0])
            return;

        mStream.open(raw_path, std::ios::out | std::ios::trunc);
        if(!mStream.is_open())
            return;

        mbEnabled = true;
        mStream << "wall_ts,thread_role,category,duration_ms,frame_id,keyframe_id,tracking_state\n";
        mStream.flush();
    }

    bool mbInitialized;
    bool mbEnabled;
    std::ofstream mStream;
    std::mutex mInitMutex;
    std::mutex mWriteMutex;
};

long ResolveLongProvider(const ScopedBenchmarkTimer::LongProvider& provider)
{
    if(!provider)
        return -1;
    return provider();
}

int ResolveIntProvider(const ScopedBenchmarkTimer::IntProvider& provider)
{
    if(!provider)
        return kUnsetTrackingState;
    return provider();
}

} // namespace

bool BenchmarkTimingEnabled()
{
    return BenchmarkTimingLogger::Instance().Enabled();
}

void SetBenchmarkThreadName(const char* name)
{
    if(!name || !name[0])
        return;
    pthread_setname_np(pthread_self(), name);
}

std::string GetBenchmarkThreadName()
{
    char name[16] = {0};
    if(pthread_getname_np(pthread_self(), name, sizeof(name)) == 0 && name[0])
        return std::string(name);
    return std::string("unknown");
}

ScopedBenchmarkTimer::ScopedBenchmarkTimer(
    const std::string& category,
    LongProvider frame_id_provider,
    LongProvider keyframe_id_provider,
    IntProvider tracking_state_provider)
    : mbEnabled(BenchmarkTimingEnabled()),
      mCategory(category),
      mFrameIdProvider(frame_id_provider),
      mKeyframeIdProvider(keyframe_id_provider),
      mTrackingStateProvider(tracking_state_provider),
      mStartTime(std::chrono::steady_clock::now())
{
}

ScopedBenchmarkTimer::~ScopedBenchmarkTimer()
{
    if(!mbEnabled)
        return;

    const double duration_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(
            std::chrono::steady_clock::now() - mStartTime)
            .count();

    BenchmarkTimingLogger::Instance().Record(
        mCategory,
        duration_ms,
        ResolveLongProvider(mFrameIdProvider),
        ResolveLongProvider(mKeyframeIdProvider),
        ResolveIntProvider(mTrackingStateProvider));
}

} // namespace ORB_SLAM2
