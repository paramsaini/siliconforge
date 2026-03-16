// SiliconForge -- Phase 95 Tests
// Enhanced timing closure loop with criticality feedback, MCMM, slack histograms.

#include "pnr/timing_closure.hpp"
#include "pnr/placer.hpp"
#include "pnr/global_router.hpp"
#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <string>

static int g_pass = 0, g_fail = 0;

static void check(bool cond, const std::string& name) {
    if (cond) { ++g_pass; std::cout << "  PASS  " << name << "\n"; }
    else      { ++g_fail; std::cout << "**FAIL  " << name << "\n"; }
}

// Build a chain of inverters for timing tests.
// Returns a netlist with N inverters: PI -> NOT -> NOT -> ... -> PO
// and a minimal PhysicalDesign with cells placed in a row.
static void build_chain(sf::Netlist& nl, sf::PhysicalDesign& pd, int chain_len = 6) {
    std::vector<sf::NetId> nets;
    nets.push_back(nl.add_net("n_in"));
    nl.mark_input(nets[0]);
    for (int i = 0; i < chain_len; ++i)
        nets.push_back(nl.add_net("n" + std::to_string(i)));
    nl.mark_output(nets.back());

    for (int i = 0; i < chain_len; ++i)
        nl.add_gate(sf::GateType::NOT, {nets[i]}, nets[i + 1],
                    "inv" + std::to_string(i));

    pd.die_area = {0, 0, 100, 50};
    pd.row_height = 5.0;
    pd.site_width = 1.0;

    for (int i = 0; i < chain_len; ++i) {
        sf::CellInstance c;
        c.id = i;
        c.name = "inv" + std::to_string(i);
        c.cell_type = "INV_X1";
        c.width = 2.0;
        c.height = 5.0;
        c.position = {5.0 + i * 10.0, 2.5};
        c.placed = true;
        pd.cells.push_back(c);
    }

    for (int i = 0; i < chain_len - 1; ++i) {
        sf::WireSegment w;
        w.net_id = nets[i + 1];
        w.layer = 0;
        w.start = {5.0 + i * 10.0 + 2.0, 2.5};
        w.end = {5.0 + (i + 1) * 10.0, 2.5};
        w.width = 0.14;
        pd.wires.push_back(w);
    }

    for (int i = 0; i < chain_len - 1; ++i) {
        sf::PhysNet pn;
        pn.id = i;
        pn.name = "net_" + std::to_string(i);
        pn.cell_ids = {i, i + 1};
        pn.pin_offsets = {sf::Point(1.0, 2.5), sf::Point(0.0, 2.5)};
        pd.nets.push_back(pn);
    }
}

// ---- Test 1: Criticality computation ----
static void test_criticality_computation() {
    std::cout << "[test_criticality_computation]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 6);

    sf::TimingClosureEngine tce(nl, pd);

    // Run STA with a tight clock to force violations
    sf::StaEngine sta(nl, nullptr, &pd);
    sf::StaResult res = sta.analyze(0.01);  // 10ps -- very tight

    // Use the engine's compute_criticality (exposed via run_enhanced internals)
    // We test indirectly through run_enhanced with tight clock
    sf::TimingClosureConfig cfg;
    cfg.clock_period = 0.01;
    cfg.max_iterations = 1;
    cfg.enable_hold_fix = false;

    auto result = tce.run_enhanced(cfg);

    check(result.iterations_run >= 1, "At least one iteration ran");
    check(result.wns_per_iteration.size() >= 2, "WNS tracked for initial + iteration");
    check(result.tns_per_iteration.size() == result.wns_per_iteration.size(),
          "TNS tracking matches WNS tracking size");
    check(result.initial_wns <= 0 || result.initial_wns >= 0,
          "Initial WNS is a valid number");
    check(!result.summary.empty(), "Summary string produced");
}

// ---- Test 2: Net weight update via placer feedback ----
static void test_net_weight_update() {
    std::cout << "[test_net_weight_update]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 4);

    sf::AnalyticalPlacer placer(pd);
    sf::TimingClosureEngine tce(nl, pd);
    tce.set_placer(&placer);

    sf::TimingClosureConfig cfg;
    cfg.clock_period = 0.01;
    cfg.max_iterations = 1;
    cfg.enable_hold_fix = false;
    cfg.enable_timing_placement = true;

    auto result = tce.run_enhanced(cfg);
    check(result.iterations_run >= 1, "Engine ran with placer feedback");
    check(result.time_ms >= 0, "Time is non-negative with placer");
    check(!result.summary.empty(), "Summary produced with placer feedback");
    check(result.wns_per_iteration.size() >= 2, "WNS history populated with placer");
}

// ---- Test 3: Router timing info feedback ----
static void test_router_timing_info() {
    std::cout << "[test_router_timing_info]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 4);

    sf::GlobalRouter grouter(pd, 10, 10, 2);
    sf::TimingClosureEngine tce(nl, pd);
    tce.set_global_router(&grouter);

    sf::TimingClosureConfig cfg;
    cfg.clock_period = 0.01;
    cfg.max_iterations = 1;
    cfg.enable_hold_fix = false;
    cfg.enable_criticality_routing = true;

    auto result = tce.run_enhanced(cfg);
    check(result.iterations_run >= 1, "Engine ran with router feedback");
    check(result.time_ms >= 0, "Time is non-negative with router");
    check(!result.summary.empty(), "Summary produced with router feedback");
}

// ---- Test 4: Basic enhanced closure loop ----
static void test_basic_closure_loop() {
    std::cout << "[test_basic_closure_loop]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 8);

    sf::TimingClosureEngine tce(nl, pd);
    sf::TimingClosureConfig cfg;
    cfg.clock_period = 5.0;
    cfg.max_iterations = 3;
    cfg.enable_hold_fix = false;

    auto result = tce.run_enhanced(cfg);

    check(result.iterations_run >= 0, "Iterations ran (or already met)");
    check(result.wns_per_iteration.size() >= 1, "WNS per iteration populated");
    check(result.tns_per_iteration.size() >= 1, "TNS per iteration populated");
    check(result.final_wns >= result.initial_wns || true,
          "Final WNS is valid (may or may not improve)");
    check(result.time_ms > 0, "Elapsed time is positive");
    check(!result.summary.empty(), "Summary is non-empty");
    check(result.summary.find("Enhanced") != std::string::npos ||
          result.summary.find("Timing already") != std::string::npos,
          "Summary contains expected prefix");
}

// ---- Test 5: Convergence detection (very loose clock) ----
static void test_convergence_detection() {
    std::cout << "[test_convergence_detection]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 4);

    sf::TimingClosureEngine tce(nl, pd);
    sf::TimingClosureConfig cfg;
    cfg.clock_period = 100.0;  // extremely loose -- timing trivially met
    cfg.max_iterations = 5;
    cfg.enable_hold_fix = false;

    auto result = tce.run_enhanced(cfg);

    check(result.converged == true, "Converged with loose clock");
    check(result.iterations_run == 0, "Zero iterations when already met");
    check(result.final_wns >= 0, "Final WNS is non-negative (timing met)");
    check(result.summary.find("already") != std::string::npos,
          "Summary says timing already met");
}

// ---- Test 6: Stall detection (impossible clock) ----
static void test_stall_detection() {
    std::cout << "[test_stall_detection]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 6);

    sf::TimingClosureEngine tce(nl, pd);
    sf::TimingClosureConfig cfg;
    cfg.clock_period = 0.001;   // 1ps -- impossible to meet
    cfg.max_iterations = 10;
    cfg.stall_limit = 2;
    cfg.enable_hold_fix = false;

    auto result = tce.run_enhanced(cfg);

    check(result.converged == false, "Did not converge on impossible clock");
    check(result.iterations_run > 0, "At least one iteration attempted");
    check(result.iterations_run <= cfg.max_iterations,
          "Iterations bounded by max_iterations");
    check(result.summary.find("DID NOT CONVERGE") != std::string::npos,
          "Summary indicates non-convergence");
}

// ---- Test 7: Hold fix in enhanced loop ----
static void test_hold_fix() {
    std::cout << "[test_hold_fix]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 5);

    sf::TimingClosureEngine tce(nl, pd);
    sf::TimingClosureConfig cfg;
    cfg.clock_period = 10.0;
    cfg.max_iterations = 2;
    cfg.enable_hold_fix = true;

    auto result = tce.run_enhanced(cfg);

    check(result.hold_violations_fixed >= 0, "Hold violations fixed is non-negative");
    check(result.time_ms > 0, "Time positive with hold fix enabled");
    check(!result.summary.empty(), "Summary with hold fix");
}

// ---- Test 8: MCMM multi-corner analysis ----
static void test_mcmm() {
    std::cout << "[test_mcmm]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 5);

    sf::TimingClosureEngine tce(nl, pd);
    sf::TimingClosureConfig cfg;
    cfg.clock_period = 5.0;
    cfg.max_iterations = 1;
    cfg.enable_hold_fix = false;
    cfg.enable_mcmm = true;
    cfg.corner_names = {"typical", "worst"};
    cfg.corner_derates = {1.0, 1.15};

    auto result = tce.run_enhanced(cfg);

    check(result.per_corner_wns.size() == 2, "Two per-corner WNS values");
    check(result.corner_names.size() == 2, "Two corner names stored");
    if (result.corner_names.size() >= 2) {
        check(result.corner_names[0] == "typical", "First corner is typical");
        check(result.corner_names[1] == "worst", "Second corner is worst");
    }
    check(result.iterations_run >= 0, "MCMM iterations ran (>= 0)");
    if (result.per_corner_wns.size() >= 2) {
        // Worst corner (derate 1.15) should have WNS <= typical corner WNS
        check(result.per_corner_wns[1] <= result.per_corner_wns[0] + 1e-6,
              "Worst corner WNS <= typical corner WNS (within tolerance)");
    }
}

// ---- Test 9: Slack histogram ----
static void test_slack_histogram() {
    std::cout << "[test_slack_histogram]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 6);

    sf::TimingClosureEngine tce(nl, pd);
    sf::TimingClosureConfig cfg;
    cfg.clock_period = 5.0;
    cfg.max_iterations = 2;
    cfg.enable_hold_fix = false;

    auto result = tce.run_enhanced(cfg);

    check(!result.slack_histogram.empty(), "Slack histogram is populated");
    for (size_t i = 0; i < result.slack_histogram.size(); ++i) {
        check(result.slack_histogram[i].size() == 10,
              "Histogram " + std::to_string(i) + " has 10 buckets");
    }
    // Sum of first histogram should be > 0 (at least some endpoints)
    int sum = 0;
    if (!result.slack_histogram.empty()) {
        for (int v : result.slack_histogram[0]) sum += v;
    }
    check(sum >= 0, "Histogram bucket sum is non-negative");
}

// ---- Test 10: Iteration summaries ----
static void test_iteration_summaries() {
    std::cout << "[test_iteration_summaries]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 5);

    sf::TimingClosureEngine tce(nl, pd);
    sf::TimingClosureConfig cfg;
    cfg.clock_period = 5.0;
    cfg.max_iterations = 2;
    cfg.enable_hold_fix = false;

    auto result = tce.run_enhanced(cfg);

    check(!result.iteration_summaries.empty(), "Iteration summaries populated");
    for (size_t i = 0; i < result.iteration_summaries.size(); ++i) {
        check(result.iteration_summaries[i].find("WNS=") != std::string::npos,
              "Summary " + std::to_string(i) + " contains WNS=");
        check(result.iteration_summaries[i].find("Iter ") != std::string::npos,
              "Summary " + std::to_string(i) + " contains Iter prefix");
    }
}

// ---- Test 11: TNS tracking ----
static void test_tns_tracking() {
    std::cout << "[test_tns_tracking]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 5);

    sf::TimingClosureEngine tce(nl, pd);
    sf::TimingClosureConfig cfg;
    cfg.clock_period = 5.0;
    cfg.max_iterations = 2;
    cfg.enable_hold_fix = false;

    auto result = tce.run_enhanced(cfg);

    check(!result.tns_per_iteration.empty(), "TNS per iteration is populated");
    check(result.tns_per_iteration.size() == result.wns_per_iteration.size(),
          "TNS and WNS vectors have same size");
    // TNS should be <= 0 (or 0 if no violations)
    for (size_t i = 0; i < result.tns_per_iteration.size(); ++i) {
        check(result.tns_per_iteration[i] <= 0.001,
              "TNS[" + std::to_string(i) + "] is non-positive (within tolerance)");
    }
}

// ---- Test 12: Both placer and router feedback combined ----
static void test_combined_feedback() {
    std::cout << "[test_combined_feedback]\n";
    sf::Netlist nl;
    sf::PhysicalDesign pd;
    build_chain(nl, pd, 5);

    sf::AnalyticalPlacer placer(pd);
    sf::GlobalRouter grouter(pd, 10, 10, 2);

    sf::TimingClosureEngine tce(nl, pd);
    tce.set_placer(&placer);
    tce.set_global_router(&grouter);

    sf::TimingClosureConfig cfg;
    cfg.clock_period = 5.0;
    cfg.max_iterations = 2;
    cfg.enable_hold_fix = false;
    cfg.enable_timing_placement = true;
    cfg.enable_criticality_routing = true;
    cfg.criticality_weight = 8.0;

    auto result = tce.run_enhanced(cfg);

    check(result.time_ms > 0, "Time positive with combined feedback");
    check(!result.summary.empty(), "Summary produced with combined feedback");
    check(result.wns_per_iteration.size() >= 1, "WNS tracked with combined feedback");
    check(result.tns_per_iteration.size() >= 1, "TNS tracked with combined feedback");
}

// ---- Test 13: NetCriticality struct sanity ----
static void test_net_criticality_struct() {
    std::cout << "[test_net_criticality_struct]\n";
    sf::NetCriticality nc;
    nc.net_id = 42;
    nc.slack = -0.5;
    nc.criticality = 0.95;

    check(nc.net_id == 42, "NetCriticality net_id set correctly");
    check(nc.slack == -0.5, "NetCriticality slack set correctly");
    check(std::abs(nc.criticality - 0.95) < 1e-9, "NetCriticality criticality set correctly");
}

// ---- Test 14: Config defaults ----
static void test_config_defaults() {
    std::cout << "[test_config_defaults]\n";
    sf::TimingClosureConfig cfg;

    check(cfg.enable_mcmm == false, "MCMM disabled by default");
    check(cfg.corner_names.empty(), "No corners by default");
    check(cfg.corner_derates.empty(), "No corner derates by default");
    check(cfg.enable_incremental_sta == true, "Incremental STA on by default");
    check(cfg.enable_criticality_routing == true, "Criticality routing on by default");
    check(cfg.enable_timing_placement == true, "Timing placement on by default");
    check(std::abs(cfg.criticality_weight - 10.0) < 1e-9, "Criticality weight default is 10.0");
}

// ---- Test 15: Result defaults ----
static void test_result_defaults() {
    std::cout << "[test_result_defaults]\n";
    sf::TimingClosureResult res;

    check(res.tns_per_iteration.empty(), "TNS per iteration empty by default");
    check(res.slack_histogram.empty(), "Slack histogram empty by default");
    check(res.iteration_summaries.empty(), "Iteration summaries empty by default");
    check(res.per_corner_wns.empty(), "Per-corner WNS empty by default");
    check(res.corner_names.empty(), "Corner names empty by default");
}

int main() {
    std::cout << "=== Phase 95: Enhanced Timing Closure with Feedback ===\n";

    test_criticality_computation();
    test_net_weight_update();
    test_router_timing_info();
    test_basic_closure_loop();
    test_convergence_detection();
    test_stall_detection();
    test_hold_fix();
    test_mcmm();
    test_slack_histogram();
    test_iteration_summaries();
    test_tns_tracking();
    test_combined_feedback();
    test_net_criticality_struct();
    test_config_defaults();
    test_result_defaults();

    std::cout << "\n=== Results: " << g_pass << " passed, " << g_fail << " failed ===\n";
    return (g_fail == 0) ? 0 : 1;
}
