#include "volume_profiler.hpp"

#ifdef VM_ENABLE_PROFILING

#include <algorithm>
#include <cstdio>
#include <unistd.h>

#if defined(__GLIBC__)
#include <malloc.h>
#endif

namespace vm::detail {

namespace {

long long current_heap_bytes() {
#if defined(__GLIBC__)
    const struct mallinfo2 mi = mallinfo2();
    return static_cast<long long>(mi.uordblks) + static_cast<long long>(mi.hblkhd);
#else
    return 0;
#endif
}

double peak_rss_kb() {
    rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    return static_cast<double>(ru.ru_maxrss);  // Linux 下单位为 KB
}

} // namespace

Profiler& Profiler::instance() {
    static Profiler profiler;
    return profiler;
}

void Profiler::add(const char* name, const ProfileSample& s) {
    std::lock_guard<std::mutex> lock(mu_);
    if (s.current_heap_kb > peak_heap_kb_) {
        peak_heap_kb_ = s.current_heap_kb;
    }
    for (auto& e : entries_) {
        if (e.name == name) {
            ++e.calls;
            e.wall_ms += s.wall_ms;
            e.usr_ms += s.usr_ms;
            e.sys_ms += s.sys_ms;
            e.heap_kb += s.heap_delta_kb;
            e.peak_rss_delta_kb += s.peak_rss_delta_kb;
            e.nvcsw += s.nvcsw;
            e.nivcsw += s.nivcsw;
            e.minflt += s.minflt;
            e.majflt += s.majflt;
            return;
        }
    }
    ProfileEntry e{};
    e.name = name;
    e.calls = 1;
    e.wall_ms = s.wall_ms;
    e.usr_ms = s.usr_ms;
    e.sys_ms = s.sys_ms;
    e.heap_kb = s.heap_delta_kb;
    e.peak_rss_delta_kb = s.peak_rss_delta_kb;
    e.nvcsw = s.nvcsw;
    e.nivcsw = s.nivcsw;
    e.minflt = s.minflt;
    e.majflt = s.majflt;
    entries_.push_back(std::move(e));
}

Profiler::~Profiler() {
    std::sort(entries_.begin(), entries_.end(),
              [](const ProfileEntry& a, const ProfileEntry& b) {
                  return a.peak_rss_delta_kb > b.peak_rss_delta_kb;
              });
    std::fprintf(stderr, "\n===== 函数级资源统计 (含被调函数) =====\n");
    const long nproc = sysconf(_SC_NPROCESSORS_ONLN);
    std::fprintf(stderr, "%-40s %7s %9s %9s %8s %10s %8s %8s %13s %10s %7s %7s %8s %8s\n",
                 "函数", "calls", "wall(ms)", "cpu(ms)", "AvgCores", "SystemCPU%", "usr(ms)", "sys(ms)",
                 "峰值RSS(MB)", "堆净增(MB)", "主动切换", "被动切换", "min缺页", "maj缺页");
    for (const auto& e : entries_) {
        const double cpu_ms = e.usr_ms + e.sys_ms;
        const double avg_cores = e.wall_ms > 0.0 ? cpu_ms / e.wall_ms : 0.0;
        const double sys_cpu_pct =
            nproc > 0 ? avg_cores / static_cast<double>(nproc) * 100.0 : 0.0;
        std::fprintf(stderr, "%-40s %7ld %9.2f %9.2f %8.3f %9.1f%% %8.2f %8.2f %13.4f %10.4f %7ld %7ld %8ld %8ld\n",
                     e.name.c_str(), e.calls, e.wall_ms, cpu_ms, avg_cores, sys_cpu_pct, e.usr_ms, e.sys_ms,
                     e.peak_rss_delta_kb / 1024.0, e.heap_kb / 1024.0, e.nvcsw, e.nivcsw, e.minflt, e.majflt);
    }
    std::fprintf(stderr, "峰值堆内存: %.2f MB | 峰值 RSS: %.2f MB\n", peak_heap_kb_ / 1024.0,
                 peak_rss_kb() / 1024.0);
}

ScopedTimer::ScopedTimer(const char* name)
    : name_(name), wall_(std::chrono::steady_clock::now()), heap0_(current_heap_bytes()) {
    getrusage(RUSAGE_SELF, &cpu0_);
}

ScopedTimer::~ScopedTimer() {
    rusage cpu1;
    getrusage(RUSAGE_SELF, &cpu1);

    ProfileSample s;
    s.wall_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_).count();
    const auto tv_ms = [](const timeval& a, const timeval& b) {
        return (b.tv_sec - a.tv_sec) * 1000.0 + (b.tv_usec - a.tv_usec) / 1000.0;
    };
    s.usr_ms = tv_ms(cpu0_.ru_utime, cpu1.ru_utime);
    s.sys_ms = tv_ms(cpu0_.ru_stime, cpu1.ru_stime);
    const long long heap1 = current_heap_bytes();
    s.heap_delta_kb = (heap1 - heap0_) / 1024.0;
    s.peak_rss_delta_kb =
        static_cast<double>(cpu1.ru_maxrss) - static_cast<double>(cpu0_.ru_maxrss);
    s.current_heap_kb = heap1 / 1024.0;
    s.nvcsw = static_cast<long>(cpu1.ru_nvcsw) - static_cast<long>(cpu0_.ru_nvcsw);
    s.nivcsw = static_cast<long>(cpu1.ru_nivcsw) - static_cast<long>(cpu0_.ru_nivcsw);
    s.minflt = static_cast<long>(cpu1.ru_minflt) - static_cast<long>(cpu0_.ru_minflt);
    s.majflt = static_cast<long>(cpu1.ru_majflt) - static_cast<long>(cpu0_.ru_majflt);
    Profiler::instance().add(name_, s);
}

} // namespace vm::detail

#endif
