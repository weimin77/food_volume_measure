#pragma once

#ifdef VM_ENABLE_PROFILING

#include <chrono>
#include <mutex>
#include <string>
#include <vector>
#include <sys/resource.h>

namespace vm::detail {

struct ProfileEntry {
    std::string name;
    long calls = 0;
    double wall_ms = 0.0;
    double usr_ms = 0.0;   // 用户态 CPU 时间（含被调函数）
    double sys_ms = 0.0;   // 内核态 CPU 时间（含被调函数）
    double heap_kb = 0.0;           // 净增堆内存（含被调函数，可正可负）
    double peak_rss_delta_kb = 0.0; // 该函数期间进程峰值 RSS 的增量（ru_maxrss 水线，含被调函数）
    long nvcsw = 0;                 // 自愿上下文切换（ru_nvcsw）
    long nivcsw = 0;                // 非自愿上下文切换（ru_nivcsw）
    long minflt = 0;                // minor 缺页（内存分配）
    long majflt = 0;                // major 缺页（换页/读文件 IO）
};

// 单次采样增量（ScopedTimer 析构时填充，交给 Profiler 累加）。
struct ProfileSample {
    double wall_ms = 0.0;
    double usr_ms = 0.0;
    double sys_ms = 0.0;
    double heap_delta_kb = 0.0;
    double peak_rss_delta_kb = 0.0;
    double current_heap_kb = 0.0;
    long nvcsw = 0;
    long nivcsw = 0;
    long minflt = 0;
    long majflt = 0;
};

class Profiler {
public:
    static Profiler& instance();
    void add(const char* name, const ProfileSample& sample);
    ~Profiler();

private:
    std::mutex mu_;
    std::vector<ProfileEntry> entries_;
    double peak_heap_kb_ = 0.0;
};

class ScopedTimer {
public:
    explicit ScopedTimer(const char* name);
    ~ScopedTimer();

private:
    const char* name_;
    std::chrono::steady_clock::time_point wall_;
    rusage cpu0_;
    long long heap0_;
};

} // namespace vm::detail

#define VM_PROFILE_FUNC() ::vm::detail::ScopedTimer vm_profile_##__LINE__(__FUNCTION__)

#else
#define VM_PROFILE_FUNC() ((void)0)
#endif
