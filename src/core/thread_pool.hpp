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

namespace sf {

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
        using ReturnType = std::invoke_result_t<F, Args...>;
        auto task = std::make_shared<std::packaged_task<ReturnType()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        auto fut = task->get_future();
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            tasks_.emplace([task]() { (*task)(); });
        }
        cv_.notify_one();
        return fut;
    }

    unsigned num_threads() const { return (unsigned)workers_.size(); }

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
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
        }
    }

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    bool stop_;
};

} // namespace sf
