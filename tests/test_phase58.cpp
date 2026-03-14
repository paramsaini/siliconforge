// ============================================================
// SiliconForge — Multi-Threading Phase Tests (MT-1 + MT-2)
// Thread pool infrastructure + Technology Mapping parallelism
// ============================================================

#include "core/thread_pool.hpp"
#include "synth/tech_mapper.hpp"
#include "core/aig.hpp"
#include "core/liberty_parser.hpp"
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include <atomic>
#include <numeric>
#include <cmath>

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

// ============================================================
// Test 1: ThreadPool basic construction
// ============================================================
TEST(thread_pool_construct) {
    ThreadPool pool(2);
    ASSERT_TRUE(pool.num_threads() == 2);
    pass_count++;
}

// ============================================================
// Test 2: ThreadPool submit and get result
// ============================================================
TEST(thread_pool_submit) {
    ThreadPool pool(2);
    auto f = pool.submit([]() { return 42; });
    ASSERT_TRUE(f.get() == 42);
    pass_count++;
}

// ============================================================
// Test 3: ThreadPool multiple tasks
// ============================================================
TEST(thread_pool_multi_tasks) {
    ThreadPool pool(4);
    std::vector<std::future<int>> futs;
    for (int i = 0; i < 100; ++i) {
        futs.push_back(pool.submit([i]() { return i * i; }));
    }
    for (int i = 0; i < 100; ++i) {
        ASSERT_TRUE(futs[i].get() == i * i);
    }
    pass_count++;
}

// ============================================================
// Test 4: ThreadPool parallel_for
// ============================================================
TEST(thread_pool_parallel_for) {
    ThreadPool pool(4);
    std::vector<int> data(1000, 0);
    pool.parallel_for(0, 1000, [&](int i) {
        data[i] = i + 1;
    });
    for (int i = 0; i < 1000; ++i) {
        ASSERT_TRUE(data[i] == i + 1);
    }
    pass_count++;
}

// ============================================================
// Test 5: ThreadPool default thread count
// ============================================================
TEST(thread_pool_default_threads) {
    ThreadPool pool;
    ASSERT_TRUE(pool.num_threads() >= 1);
    pass_count++;
}

// ============================================================
// Test 6: ThreadPool concurrent accumulation
// ============================================================
TEST(thread_pool_atomic_accum) {
    ThreadPool pool(4);
    std::atomic<int> sum{0};
    std::vector<std::future<void>> futs;
    for (int i = 0; i < 100; ++i) {
        futs.push_back(pool.submit([&sum, i]() {
            sum += i;
        }));
    }
    for (auto& f : futs) f.get();
    ASSERT_TRUE(sum.load() == 4950);
    pass_count++;
}

// ============================================================
// Test 7: ThreadPool empty range parallel_for
// ============================================================
TEST(thread_pool_empty_range) {
    ThreadPool pool(2);
    int called = 0;
    pool.parallel_for(5, 5, [&](int) { called++; });
    ASSERT_TRUE(called == 0);
    pool.parallel_for(10, 3, [&](int) { called++; });
    ASSERT_TRUE(called == 0);
    pass_count++;
}

// ============================================================
// Test 8: ThreadPool single thread
// ============================================================
TEST(thread_pool_single_thread) {
    ThreadPool pool(1);
    ASSERT_TRUE(pool.num_threads() == 1);
    auto f = pool.submit([]() { return 99; });
    ASSERT_TRUE(f.get() == 99);
    pass_count++;
}

// ============================================================
// Test 9: Tech mapper produces valid result (with MT enabled)
// ============================================================
TEST(tech_map_basic_mt) {
    AigGraph aig;
    auto a = aig.create_input();
    auto b = aig.create_input();
    auto ab = aig.create_and(a, b);
    aig.add_output(ab);

    LibertyLibrary lib;
    LibertyCell nand2;
    nand2.name = "NAND2_X1";
    nand2.area = 2.0;
    nand2.leakage_power = 0.001;
    LibertyPin pa, pb, py;
    pa.name = "A"; pa.direction = "input";
    pb.name = "B"; pb.direction = "input";
    py.name = "Y"; py.direction = "output"; py.function = "!(A & B)";
    nand2.pins = {pa, pb, py};
    lib.cells.push_back(nand2);

    LibertyCell inv;
    inv.name = "INV_X1";
    inv.area = 1.0;
    inv.leakage_power = 0.0005;
    LibertyPin ia, iy;
    ia.name = "A"; ia.direction = "input";
    iy.name = "Y"; iy.direction = "output"; iy.function = "!A";
    inv.pins = {ia, iy};
    lib.cells.push_back(inv);

    TechMapper mapper(aig, lib);
    auto nl = mapper.map();
    ASSERT_TRUE(nl.num_gates() > 0);
    pass_count++;
}

// ============================================================
// Test 10: Tech mapper with larger AIG (exercises parallel path)
// ============================================================
TEST(tech_map_larger_aig) {
    AigGraph aig;
    std::vector<AigLit> inputs;
    for (int i = 0; i < 8; ++i) inputs.push_back(aig.create_input());

    // Build a chain of AND gates
    AigLit prev = inputs[0];
    for (int i = 1; i < 8; ++i) {
        prev = aig.create_and(prev, inputs[i]);
    }
    aig.add_output(prev);

    LibertyLibrary lib;
    LibertyCell and2;
    and2.name = "AND2_X1";
    and2.area = 2.0;
    and2.leakage_power = 0.001;
    LibertyPin pa, pb, py;
    pa.name = "A"; pa.direction = "input";
    pb.name = "B"; pb.direction = "input";
    py.name = "Y"; py.direction = "output"; py.function = "A & B";
    and2.pins = {pa, pb, py};
    lib.cells.push_back(and2);

    LibertyCell inv;
    inv.name = "INV_X1"; inv.area = 1.0; inv.leakage_power = 0.0005;
    LibertyPin ia, iy;
    ia.name = "A"; ia.direction = "input";
    iy.name = "Y"; iy.direction = "output"; iy.function = "!A";
    inv.pins = {ia, iy};
    lib.cells.push_back(inv);

    TechMapper mapper(aig, lib);
    mapper.set_goal(TechMapper::MapGoal::AREA);
    auto nl = mapper.map(true);
    ASSERT_TRUE(nl.num_gates() >= 7); // at least 7 AND gates in chain
    pass_count++;
}

// ============================================================
// Test 11: map_optimal with parallel inner loop
// ============================================================
TEST(tech_map_optimal_mt) {
    AigGraph aig;
    auto a = aig.create_input();
    auto b = aig.create_input();
    auto c = aig.create_input();
    auto ab = aig.create_and(a, b);
    auto abc = aig.create_and(ab, c);
    aig.add_output(abc);

    LibertyLibrary lib;
    LibertyCell and2;
    and2.name = "AND2_X1"; and2.area = 2.0; and2.leakage_power = 0.001;
    LibertyPin pa, pb, py;
    pa.name = "A"; pa.direction = "input";
    pb.name = "B"; pb.direction = "input";
    py.name = "Y"; py.direction = "output"; py.function = "A & B";
    and2.pins = {pa, pb, py};
    lib.cells.push_back(and2);

    LibertyCell inv;
    inv.name = "INV_X1"; inv.area = 1.0; inv.leakage_power = 0.0005;
    LibertyPin ia, iy;
    ia.name = "A"; ia.direction = "input";
    iy.name = "Y"; iy.direction = "output"; iy.function = "!A";
    inv.pins = {ia, iy};
    lib.cells.push_back(inv);

    TechMapper mapper(aig, lib);
    TechMapper::IlpMapConfig cfg;
    cfg.lagrangian_iterations = 3;
    auto nl = mapper.map_optimal(cfg);
    ASSERT_TRUE(nl.num_gates() > 0);
    pass_count++;
}

// ============================================================
// Test 12: ThreadPool destructor joins cleanly
// ============================================================
TEST(thread_pool_destroy) {
    for (int trial = 0; trial < 5; ++trial) {
        ThreadPool pool(4);
        std::vector<std::future<int>> futs;
        for (int i = 0; i < 50; ++i) {
            futs.push_back(pool.submit([i]() { return i; }));
        }
        for (auto& f : futs) f.get();
    }
    pass_count++;
}

// ============================================================
// main
// ============================================================
int main() {
    std::cout << "=== Phase 58: Multi-Threading (MT-1 + MT-2) ===\n";
    for (auto& t : tests) {
        std::cout << "  " << t.name << "... ";
        t.fn();
        if (fail_count == 0 || pass_count > 0)
            std::cout << "PASS\n";
    }
    std::cout << "\nPhase 58: " << pass_count << " passed, " << fail_count << " failed\n";
    return fail_count > 0 ? 1 : 0;
}
