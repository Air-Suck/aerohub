#pragma once
// Optional diagnostic only: clocks are local to this process/host.
#include <ctime>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <atomic>
#include <functional>
#include <iomanip>
namespace stage3_profile {
inline int64_t ns(clockid_t clock) {
    timespec value{};
    return clock_gettime(clock, &value)==0 ? int64_t(value.tv_sec)*1000000000LL+value.tv_nsec : -1;
}
struct Sink {
    std::ofstream stream;
    std::mutex mutex;
    Sink() {
        const char *path=std::getenv("SUPER_PROFILE_PATH");
        if (path && *path) stream.open(path, std::ios::app);
    }
};
inline Sink &sink() { static Sink value; return value; }
inline uint64_t &current_scope() { static thread_local uint64_t value=0; return value; }
inline uint64_t next_id() { static std::atomic<uint64_t> value{0}; return ++value; }
class Scope {
    const char *name_;
    bool enabled_;
    int64_t wall_=0, cpu_=0, process_=0;
    uint64_t id_=0, parent_=0;
    std::function<double()> sim_clock_;
    double sim_start_=0;
    const char *outcome_="unreported";
    int64_t work_=0, commit_ns_=0;
    double commit_sim_s_=0;
public:
    explicit Scope(const char *name, std::function<double()> clock = {}): name_(name), enabled_(sink().stream.is_open()), sim_clock_(clock) {
        if (enabled_) {
            id_=next_id(); parent_=current_scope(); current_scope()=id_;
            if (sim_clock_) sim_start_=sim_clock_();
            wall_=ns(CLOCK_MONOTONIC);
            cpu_=ns(CLOCK_THREAD_CPUTIME_ID);
            process_=ns(CLOCK_PROCESS_CPUTIME_ID);
        }
    }
    void outcome(const char *value) { outcome_=value; }
    void committed() {
        outcome_="committed";
        if (enabled_) { commit_ns_=ns(CLOCK_MONOTONIC); if (sim_clock_) commit_sim_s_=sim_clock_(); }
    }
    void work(int64_t value) { work_=value; }
    ~Scope() {
        if (!enabled_) return;
        const auto cpu=ns(CLOCK_THREAD_CPUTIME_ID), process=ns(CLOCK_PROCESS_CPUTIME_ID), end=ns(CLOCK_MONOTONIC);
        current_scope()=parent_;
        const double sim_end=sim_clock_ ? sim_clock_() : 0;
        auto &output=sink();
        std::lock_guard<std::mutex> guard(output.mutex);
        output.stream << std::setprecision(17) << "{\"id\":" << id_ << ",\"parent_id\":" << parent_
            << ",\"outcome\":\"" << outcome_ << "\",\"work\":" << work_;
        if (commit_ns_) output.stream << ",\"commit_ns\":" << commit_ns_ << ",\"commit_sim_s\":" << commit_sim_s_;
        if (sim_clock_) output.stream << ",\"sim_start_s\":" << sim_start_ << ",\"sim_end_s\":" << sim_end;
        output.stream << ",\"scope\":\"" << name_ << "\",\"start_ns\":" << wall_
            << ",\"end_ns\":" << end << ",\"wall_ns\":" << end-wall_
            << ",\"thread_cpu_ns\":" << (cpu_<0 || cpu<0 ? -1 : cpu-cpu_)
            << ",\"process_cpu_ns\":" << (process_<0 || process<0 ? -1 : process-process_) << "}\n";
    }
};
}
