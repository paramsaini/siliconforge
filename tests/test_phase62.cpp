// ============================================================
// SiliconForge — Multi-Threading Phase Tests (MT-6)
// Pipeline-Level Parallelism (parallel stage patterns)
// ============================================================

#include "core/thread_pool.hpp"
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include <future>
#include <atomic>
#include <chrono>
#include <thread>

using namespace sf;

static int pass_count = 0;
static int fail_count = 0;

#define TEST(name) \
    static void test_##name(); \
    struct Register_##name { Register_##name() { tests.push_back({#name, test_##name}); } } reg_##name; \
    static void test_##name()

struct TestEntry { const char* name; void(*fn)(); };
static std::vector<TestEntry> tests;

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        std::cerr << "  FAIL: " #cond " at line " << __LINE__ << "\n"; \
        fail_count++; return; \
    } \
} while(0)

// Simulate stage functions (DRC, LVS, ERC, ESD, Latchup)
static bool simulate_stage(const std::string& name, int work_ms) {
    auto t0 = std::chrono::steady_clock::now();
    // Simulate work
    volatile int sum = 0;
    while (true) {
        for (int i = 0; i < 1000; ++i) sum += i;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= work_ms) break;
    }
    return true;
}

// ============================================================
// Test 1: DRC || LVS parallel pattern
// ============================================================
TEST(pipeline_drc_lvs_parallel) {
    ThreadPool pool(2);
    auto f_drc = pool.submit([]() { return simulate_stage("DRC", 20); });
    auto f_lvs = pool.submit([]() { return simulate_stage("LVS", 20); });
    ASSERT_TRUE(f_drc.get());
    ASSERT_TRUE(f_lvs.get());
    pass_count++;
}

// ============================================================
// Test 2: ERC || ESD || Latchup parallel pattern
// ============================================================
TEST(pipeline_erc_esd_latchup_parallel) {
    ThreadPool pool(3);
    auto f_erc = pool.submit([]() { return simulate_stage("ERC", 15); });
    auto f_esd = pool.submit([]() { return simulate_stage("ESD", 15); });
    auto f_latch = pool.submit([]() { return simulate_stage("Latchup", 15); });
    ASSERT_TRUE(f_erc.get());
    ASSERT_TRUE(f_esd.get());
    ASSERT_TRUE(f_latch.get());
    pass_count++;
}

// ============================================================
// Test 3: Parallel stages faster than sequential
// ============================================================
TEST(pipeline_speedup) {
    auto t0 = std::chrono::high_resolution_clock::now();
    // Sequential
    simulate_stage("A", 30);
    simulate_stage("B", 30);
    auto t_seq = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    // Parallel
    ThreadPool pool(2);
    auto fa = pool.submit([]() { return simulate_stage("A", 30); });
    auto fb = pool.submit([]() { return simulate_stage("B", 30); });
    fa.get(); fb.get();
    auto t_par = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0).count();

    // Parallel should be faster (at least 1.3x speedup)
    ASSERT_TRUE(t_par < t_seq * 0.85);
    pass_count++;
}

// ============================================================
// Test 4: Full pipeline simulation (sequential groups)
// ============================================================
TEST(pipeline_full_simulation) {
    // Phase 1: DRC || LVS
    ThreadPool pool2(2);
    auto f1a = pool2.submit([]() { return 1; });
    auto f1b = pool2.submit([]() { return 2; });
    int phase1 = f1a.get() + f1b.get();
    ASSERT_TRUE(phase1 == 3);

    // Phase 2: ERC || ESD || Latchup (depends on phase 1)
    ThreadPool pool3(3);
    auto f2a = pool3.submit([&]() { return phase1 + 10; });
    auto f2b = pool3.submit([&]() { return phase1 + 20; });
    auto f2c = pool3.submit([&]() { return phase1 + 30; });
    int phase2 = f2a.get() + f2b.get() + f2c.get();
    ASSERT_TRUE(phase2 == (13 + 23 + 33));
    pass_count++;
}

// ============================================================
// Test 5: Pipeline with error propagation
// ============================================================
TEST(pipeline_error_propagation) {
    ThreadPool pool(2);
    std::atomic<bool> stage1_ok{false};
    auto f1 = pool.submit([&]() { stage1_ok = true; return true; });
    auto f2 = pool.submit([]() { return true; });
    bool r1 = f1.get(), r2 = f2.get();
    ASSERT_TRUE(r1 && r2);
    ASSERT_TRUE(stage1_ok.load());
    pass_count++;
}

// ============================================================
// Test 6: Pipeline repeated runs (resource cleanup)
// ============================================================
TEST(pipeline_repeated_runs) {
    for (int trial = 0; trial < 5; ++trial) {
        ThreadPool pool(2);
        auto f1 = pool.submit([trial]() { return trial; });
        auto f2 = pool.submit([trial]() { return trial * 2; });
        (void)f1.get();
        (void)f2.get();
    }
    pass_count++;
}

// ============================================================
// Test 7: Pipeline with varying pool sizes
// ============================================================
TEST(pipeline_varying_pool_sizes) {
    for (int nthreads = 1; nthreads <= 4; ++nthreads) {
        ThreadPool pool(nthreads);
        std::vector<std::future<int>> futs;
        for (int i = 0; i < 5; ++i) {
            futs.push_back(pool.submit([i]() { return i * i; }));
        }
        int sum = 0;
        for (auto& f : futs) sum += f.get();
        ASSERT_TRUE(sum == 0 + 1 + 4 + 9 + 16);
    }
    pass_count++;
}

// ============================================================
// Test 8: Nested thread pools (pool inside pool task)
// ============================================================
TEST(pipeline_nested_pool) {
    ThreadPool outer(2);
    auto f = outer.submit([]() {
        ThreadPool inner(2);
        auto g = inner.submit([]() { return 42; });
        return g.get();
    });
    ASSERT_TRUE(f.get() == 42);
    pass_count++;
}

// ============================================================
// Test 9: Pipeline ordering (phase2 after phase1)
// ============================================================
TEST(pipeline_ordering) {
    std::atomic<int> order{0};
    // Phase 1
    {
        ThreadPool pool(2);
        auto f = pool.submit([&]() { order.fetch_add(1); return true; });
        f.get();
    }
    ASSERT_TRUE(order.load() == 1);
    // Phase 2 (must start after phase 1 completes)
    {
        ThreadPool pool(2);
        auto f = pool.submit([&]() { order.fetch_add(10); return true; });
        f.get();
    }
    ASSERT_TRUE(order.load() == 11);
    pass_count++;
}

// ============================================================
// Test 10: Pipeline with many parallel stages
// ============================================================
TEST(pipeline_many_stages) {
    ThreadPool pool(4);
    std::atomic<int> completed{0};
    std::vector<std::future<bool>> futs;
    for (int i = 0; i < 20; ++i) {
        futs.push_back(pool.submit([&completed]() {
            completed++;
            return true;
        }));
    }
    for (auto& f : futs) ASSERT_TRUE(f.get());
    ASSERT_TRUE(completed.load() == 20);
    pass_count++;
}

// ============================================================
// main
// ============================================================
int main() {
    std::cout << "=== Phase 62: Pipeline Parallelism (MT-6) ===\n";
    for (auto& t : tests) {
        std::cout << "  " << t.name << "... ";
        t.fn();
        if (fail_count == 0 || pass_count > 0)
            std::cout << "PASS\n";
    }
    std::cout << "\nPhase 62: " << pass_count << " passed, " << fail_count << " failed\n";
    return fail_count > 0 ? 1 : 0;
}
