// ============================================================
// SiliconForge — Multi-Threading Phase Tests (MT-3)
// STA Multi-Corner (MCMM) Parallelism
// ============================================================

#include "timing/mcmm.hpp"
#include "core/thread_pool.hpp"
#include "core/netlist.hpp"
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

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

static Netlist make_simple_nl() {
    Netlist nl;
    auto clk = nl.add_net("clk");
    auto a = nl.add_net("a");
    auto b = nl.add_net("b");
    auto y = nl.add_net("y");
    nl.add_gate(GateType::INPUT, {}, a, "a_in");
    nl.add_gate(GateType::INPUT, {}, b, "b_in");
    nl.add_gate(GateType::AND, {a, b}, y, "g0");
    nl.add_gate(GateType::OUTPUT, {y}, -1, "y_out");
    return nl;
}

// ============================================================
// Test 1: MCMM with default corners (parallelized)
// ============================================================
TEST(mcmm_default_corners_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();
    auto r = mcmm.analyze();
    ASSERT_TRUE(r.scenarios > 0);
    ASSERT_TRUE(r.corners >= 3);
    ASSERT_TRUE(r.modes >= 2);
    pass_count++;
}

// ============================================================
// Test 2: MCMM foundry corners (7 corners × modes)
// ============================================================
TEST(mcmm_foundry_corners_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    mcmm.load_foundry_corners();
    mcmm.load_default_modes();
    auto r = mcmm.analyze();
    ASSERT_TRUE(r.corners >= 7);
    ASSERT_TRUE(r.scenarios >= 14);
    pass_count++;
}

// ============================================================
// Test 3: MCMM worst tracking is correct after parallel merge
// ============================================================
TEST(mcmm_worst_tracking_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();
    auto r = mcmm.analyze();
    // Worst WNS should be the minimum across all scenarios
    double manual_worst = 1e18;
    for (auto& d : r.details) {
        if (d.active && d.sta.wns < manual_worst) manual_worst = d.sta.wns;
    }
    ASSERT_TRUE(std::abs(r.worst_setup_wns - manual_worst) < 1e-6);
    pass_count++;
}

// ============================================================
// Test 4: MCMM power tracking after parallel merge
// ============================================================
TEST(mcmm_power_tracking_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();
    auto r = mcmm.analyze();
    double manual_max = 0;
    for (auto& d : r.details) {
        if (d.active && d.power.total_power_mw > manual_max)
            manual_max = d.power.total_power_mw;
    }
    ASSERT_TRUE(std::abs(r.max_power_mw - manual_max) < 1e-6);
    pass_count++;
}

// ============================================================
// Test 5: MCMM with sensitivity enabled
// ============================================================
TEST(mcmm_sensitivity_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();
    McmmConfig cfg;
    cfg.enable_sensitivity = true;
    mcmm.set_config(cfg);
    auto r = mcmm.analyze();
    ASSERT_TRUE(r.sensitivities.size() > 0);
    pass_count++;
}

// ============================================================
// Test 6: MCMM with scenario pruning
// ============================================================
TEST(mcmm_pruning_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();
    McmmConfig cfg;
    cfg.enable_scenario_pruning = true;
    mcmm.set_config(cfg);
    auto r = mcmm.analyze();
    ASSERT_TRUE(r.scenarios > 0);
    pass_count++;
}

// ============================================================
// Test 7: MCMM active scenario count matches
// ============================================================
TEST(mcmm_active_count_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();
    auto r = mcmm.analyze();
    int active = 0;
    for (auto& d : r.details) if (d.active) active++;
    ASSERT_TRUE(active == r.active_scenarios);
    pass_count++;
}

// ============================================================
// Test 8: MCMM backward compatibility (worst_wns = worst_setup_wns)
// ============================================================
TEST(mcmm_backward_compat_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();
    auto r = mcmm.analyze();
    ASSERT_TRUE(r.worst_wns == r.worst_setup_wns);
    ASSERT_TRUE(r.worst_corner == r.worst_setup_scenario);
    pass_count++;
}

// ============================================================
// Test 9: MCMM signoff summaries populated
// ============================================================
TEST(mcmm_signoff_summary_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();
    auto r = mcmm.analyze();
    ASSERT_TRUE(r.setup_signoff.scenario_count > 0);
    ASSERT_TRUE(r.hold_signoff.scenario_count > 0);
    pass_count++;
}

// ============================================================
// Test 10: MCMM single corner (degenerate case)
// ============================================================
TEST(mcmm_single_corner_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    PvtCorner c;
    c.name = "typical"; c.voltage = 1.0; c.temperature_c = 25;
    mcmm.add_corner(c);
    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);
    auto r = mcmm.analyze();
    ASSERT_TRUE(r.scenarios == 1);
    ASSERT_TRUE(r.active_scenarios == 1);
    pass_count++;
}

// ============================================================
// Test 11: MCMM result message is non-empty
// ============================================================
TEST(mcmm_message_mt) {
    auto nl = make_simple_nl();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();
    auto r = mcmm.analyze();
    ASSERT_TRUE(!r.message.empty());
    pass_count++;
}

// ============================================================
// Test 12: ThreadPool used inside MCMM (verify no deadlock)
// ============================================================
TEST(mcmm_no_deadlock_mt) {
    auto nl = make_simple_nl();
    // Run MCMM 3 times to stress-test thread pool reuse
    for (int trial = 0; trial < 3; ++trial) {
        McmmAnalyzer mcmm(nl);
        mcmm.load_foundry_corners();
        mcmm.load_default_modes();
        auto r = mcmm.analyze();
        ASSERT_TRUE(r.scenarios > 0);
    }
    pass_count++;
}

// ============================================================
// main
// ============================================================
int main() {
    std::cout << "=== Phase 59: Multi-Corner STA Parallelism (MT-3) ===\n";
    for (auto& t : tests) {
        std::cout << "  " << t.name << "... ";
        t.fn();
        if (fail_count == 0 || pass_count > 0)
            std::cout << "PASS\n";
    }
    std::cout << "\nPhase 59: " << pass_count << " passed, " << fail_count << " failed\n";
    return fail_count > 0 ? 1 : 0;
}
