#pragma once

#include <iostream>
#include <pthread.h>
#include <string>
#include <sys/syscall.h>
#include <unistd.h>

namespace ORB_SLAM3 {
namespace ThreadNaming {

inline long GetCurrentThreadId()
{
    return static_cast<long>(::syscall(SYS_gettid));
}

inline void NameCurrentThread(const std::string &thread_name)
{
    thread_local bool already_named = false;
    if (already_named)
    {
        return;
    }

    const std::string truncated_name = thread_name.substr(0, 15);
    const int result = pthread_setname_np(pthread_self(), truncated_name.c_str());

    if (result == 0)
    {
        std::cout << "[thread] " << truncated_name
                  << " tid=" << GetCurrentThreadId() << std::endl;
    }
    else
    {
        std::cout << "[thread] failed to name thread " << truncated_name
                  << " tid=" << GetCurrentThreadId()
                  << " errno=" << result << std::endl;
    }

    already_named = true;
}

}  // namespace ThreadNaming
}  // namespace ORB_SLAM3
