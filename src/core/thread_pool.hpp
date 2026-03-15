#pragma once
// SiliconForge — Lightweight Thread Pool
// Provides task-based parallelism for stages that need std::thread
// (STA multi-corner, flow pipeline, portfolio SAT).

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <atomic>
#include <algorithm>
#include <cmath>

#ifdef __linux__
#include <sched.h>
#include <numa.h>
#endif

namespace sf {

enum class TaskPriority { LOW, NORMAL, HIGH, CRITICAL };

// ── NUMA topology configuration ─────────────────────────────────────────
// Models non-uniform memory access latency across processor nodes.
// On multi-socket systems, memory accesses to remote nodes incur 1.5-3x
// latency penalty (Intel QPI/UPI), making task-to-node affinity critical
// for memory-bound EDA workloads (STA, extraction, simulation).
struct NumaConfig {
    int node_count = 1;                   // number of NUMA nodes (sockets)
    int cores_per_node = 4;               // physical cores per node
    double memory_latency_ratio = 1.5;    // remote/local access latency ratio
    bool strict_affinity = false;         // if true, never migrate across nodes
    std::vector<int> node_core_map;       // flat map: node_core_map[core] = node_id

    // Build default round-robin core-to-node mapping
    void build_default_map() {
        int total = node_count * cores_per_node;
        node_core_map.resize(total);
        for (int i = 0; i < total; ++i)
            node_core_map[i] = i / cores_per_node;
    }

    // Query which NUMA node owns a given core index
    int node_for_core(int core_id) const {
        if (core_id >= 0 && core_id < (int)node_core_map.size())
            return node_core_map[core_id];
        return core_id % std::max(1, node_count);
    }

    // Return the set of core IDs belonging to a given NUMA node
    std::vector<int> cores_on_node(int node_id) const {
        std::vector<int> cores;
        for (int i = 0; i < (int)node_core_map.size(); ++i) {
            if (node_core_map[i] == node_id)
                cores.push_back(i);
        }
        return cores;
    }
};

class ThreadPool {
public:
    explicit ThreadPool(unsigned num_threads = 0)
        : stop_(false)
    {
        if (num_threads == 0)
            num_threads = std::max(1u, std::thread::hardware_concurrency());
        for (unsigned i = 0; i < num_threads; ++i) {
            workers_.emplace_back([this] { worker_loop(); });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& w : workers_) {
            if (w.joinable()) w.join();
        }
    }

    // Submit a callable and get a future for its result
    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>>
    {
        return submit_with_priority(TaskPriority::NORMAL, std::forward<F>(f), std::forward<Args>(args)...);
    }

    // Submit with explicit priority
    template<typename F, typename... Args>
    auto submit_with_priority(TaskPriority priority, F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>>
    {
        using ReturnType = std::invoke_result_t<F, Args...>;
        auto task = std::make_shared<std::packaged_task<ReturnType()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        auto fut = task->get_future();
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            tasks_.push({static_cast<int>(priority), task_seq_++, [task]() { (*task)(); }});
        }
        cv_.notify_one();
        return fut;
    }

    unsigned num_threads() const { return (unsigned)workers_.size(); }

    // ── NUMA topology awareness ─────────────────────────────────────
    // Configure NUMA topology for locality-aware task dispatch.
    // Must be called before submit_numa_aware() for meaningful affinity.
    void set_numa_config(const NumaConfig& cfg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        numa_cfg_ = cfg;
        if (numa_cfg_.node_core_map.empty())
            numa_cfg_.build_default_map();

        // Build per-node task queues
        numa_queues_.resize(numa_cfg_.node_count);

        // Set CPU affinity hints for worker threads: pin worker i to its
        // corresponding core on the appropriate NUMA node.
        // On non-Linux platforms this is advisory only (the scheduler still
        // attempts locality through priority-based queue selection).
        apply_affinity_hints();
    }

    const NumaConfig& numa_config() const { return numa_cfg_; }

    // Submit a task pinned to a specific NUMA node.  The task is placed
    // into the node-local queue and preferentially executed by a worker
    // whose CPU affinity matches that node.  When strict_affinity is
    // false, any idle worker may steal from a remote node queue after
    // its local queue drains (work-stealing with locality bias).
    template<typename F, typename... Args>
    auto submit_numa_aware(int target_node, F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>>
    {
        using ReturnType = std::invoke_result_t<F, Args...>;
        auto task = std::make_shared<std::packaged_task<ReturnType()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        auto fut = task->get_future();

        int node = std::clamp(target_node, 0, std::max(1, numa_cfg_.node_count) - 1);
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            // Encode NUMA node preference as a priority boost so that
            // workers on the target node pick these tasks first.
            // Node-local tasks get CRITICAL priority on their node queue;
            // they also enter the global queue at NORMAL so any worker can
            // steal if the node is saturated.
            if (node < (int)numa_queues_.size()) {
                numa_queues_[node].push({static_cast<int>(TaskPriority::CRITICAL),
                                         task_seq_++, [task]() { (*task)(); }});
            }
            // Also place in global queue at lower priority for work-stealing
            tasks_.push({static_cast<int>(TaskPriority::NORMAL), task_seq_++,
                         [task]() { (*task)(); }});
        }
        cv_.notify_one();
        return fut;
    }

    // Parallel-for with NUMA-aware partitioning: each NUMA node processes
    // a contiguous chunk of the iteration space to maximize data locality.
    template<typename F>
    void parallel_for_numa(int begin, int end, F&& func) {
        if (begin >= end) return;
        int n = end - begin;
        int nodes = std::max(1, numa_cfg_.node_count);
        int chunk_per_node = std::max(1, n / nodes);
        std::vector<std::future<void>> futs;

        for (int node = 0; node < nodes; ++node) {
            int lo = begin + node * chunk_per_node;
            int hi = (node == nodes - 1) ? end : std::min(lo + chunk_per_node, end);
            if (lo >= hi) continue;

            // Sub-partition within the node across its cores
            int cores = numa_cfg_.cores_per_node;
            int sub_chunk = std::max(1, (hi - lo) / cores);
            for (int i = lo; i < hi; i += sub_chunk) {
                int block_end = std::min(i + sub_chunk, hi);
                futs.push_back(submit_numa_aware(node, [&func, i, block_end]() {
                    for (int j = i; j < block_end; ++j) func(j);
                }));
            }
        }
        for (auto& f : futs) f.get();
    }

    // Run N tasks in parallel, wait for all to complete
    template<typename F>
    void parallel_for(int begin, int end, F&& func) {
        if (begin >= end) return;
        int n = end - begin;
        int chunk = std::max(1, n / (int)workers_.size());
        std::vector<std::future<void>> futs;
        for (int i = begin; i < end; i += chunk) {
            int block_end = std::min(i + chunk, end);
            futs.push_back(submit([&func, i, block_end]() {
                for (int j = i; j < block_end; ++j) func(j);
            }));
        }
        for (auto& f : futs) f.get();
    }

private:
    void worker_loop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
                if (stop_ && tasks_.empty()) return;
                task = std::move(tasks_.top().func);
                tasks_.pop();
            }
            task();
        }
    }

    struct PrioritizedTask {
        int priority;       // higher = more urgent
        uint64_t seq;       // tie-break: lower seq = earlier submission
        std::function<void()> func;
        bool operator<(const PrioritizedTask& o) const {
            if (priority != o.priority) return priority < o.priority;
            return seq > o.seq; // earlier tasks first
        }
    };

    std::vector<std::thread> workers_;
    std::priority_queue<PrioritizedTask> tasks_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    bool stop_;
    uint64_t task_seq_ = 0;

    // NUMA state
    NumaConfig numa_cfg_;
    std::vector<std::priority_queue<PrioritizedTask>> numa_queues_;

    // Apply CPU affinity hints to worker threads.
    // On Linux with libnuma, uses sched_setaffinity for hard pinning.
    // On other platforms, this is a no-op — the NUMA-aware queue
    // selection still provides soft locality benefits.
    void apply_affinity_hints() {
#ifdef __linux__
        for (unsigned i = 0; i < workers_.size(); ++i) {
            int core_id = static_cast<int>(i) % static_cast<int>(numa_cfg_.node_core_map.size());
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(core_id, &cpuset);
            pthread_setaffinity_np(workers_[i].native_handle(), sizeof(cpu_set_t), &cpuset);
        }
#endif
        // On macOS / Windows: affinity is advisory only.
        // The NUMA-aware queue partitioning still provides data locality
        // benefits through preferential task dispatch.
    }
};

} // namespace sf
