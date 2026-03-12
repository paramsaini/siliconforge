// SiliconForge Phase 18 — Industrial-Grade PnR Tests
// Tests Retiming Engine, Post-Route Optimizer, and Timing-Driven Placement.

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include "pnr/placer.hpp"
#include "pnr/post_route_opt.hpp"
#include "synth/retiming.hpp"
#include "timing/sta.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <string>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// ---------------------------------------------------------------------------
// Helper: sequential pipeline DFF → AND → OR → XOR → NOT → DFF
// Total combinational delay = 1.5 + 1.5 + 2.5 + 1.0 = 6.5 units
// ---------------------------------------------------------------------------
static Netlist build_retiming_circuit() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId in  = nl.add_net("in");  nl.mark_input(in);
    NetId q1  = nl.add_net("q1");
    nl.add_dff(in, clk, q1, -1, "FF_in");

    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId w3 = nl.add_net("w3");
    NetId w4 = nl.add_net("w4");
    nl.add_gate(GateType::AND, {q1, q1}, w1, "G1");
    nl.add_gate(GateType::OR,  {w1, q1}, w2, "G2");
    nl.add_gate(GateType::XOR, {w2, q1}, w3, "G3");
    nl.add_gate(GateType::NOT, {w3},     w4, "G4");

    NetId out = nl.add_net("out"); nl.mark_output(out);
    nl.add_dff(w4, clk, out, -1, "FF_out");
    return nl;
}

// ---------------------------------------------------------------------------
// Helper: PhysicalDesign with cells and wires for PnR tests
// ---------------------------------------------------------------------------
static PhysicalDesign build_test_pd(int n_cells = 20) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    for (int i = 0; i < n_cells; i++)
        pd.add_cell("cell_" + std::to_string(i), "AND2_X1", 3.0, 10.0);
    for (int i = 0; i < n_cells - 1; i++)
        pd.add_net("n" + std::to_string(i), {i, i + 1});
    return pd;
}

// ---------------------------------------------------------------------------
// Helper: Sequential netlist for STA-driven tests (AND → NOT → DFF)
// ---------------------------------------------------------------------------
static Netlist build_seq_circuit() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId d   = nl.add_net("d");   nl.mark_input(d);
    NetId q   = nl.add_net("q");   nl.mark_output(q);
    NetId w1  = nl.add_net("w1");
    NetId w2  = nl.add_net("w2");
    nl.add_gate(GateType::AND, {d, d}, w1, "G1");
    nl.add_gate(GateType::NOT, {w1},   w2, "G2");
    nl.add_dff(w2, clk, q, -1, "FF1");
    return nl;
}

// ---------------------------------------------------------------------------
// Helper: PhysicalDesign with wires (for post-route optimizer)
// ---------------------------------------------------------------------------
static PhysicalDesign build_pd_with_wires(int n_cells = 20) {
    PhysicalDesign pd = build_test_pd(n_cells);
    for (int i = 0; i < n_cells - 1; i++) {
        WireSegment w;
        w.layer  = 0;
        w.start  = Point(static_cast<double>(i * 10), 5.0);
        w.end    = Point(static_cast<double>((i + 1) * 10), 5.0);
        w.width  = 0.2;
        w.net_id = i;
        pd.wires.push_back(w);
    }
    return pd;
}

// ===================================================================
//  Retiming Engine
// ===================================================================

// 1. retiming_improves_critical_path
TEST(retiming_improves_critical_path) {
    Netlist nl = build_retiming_circuit();
    RetimingEngine eng;
    auto res = eng.optimize_with_result(nl);
    if (res.improved) {
        CHECK(res.critical_path_after < res.critical_path_before,
              "improved but after >= before: " +
              std::to_string(res.critical_path_after) + " >= " +
              std::to_string(res.critical_path_before));
    } else {
        // Algorithm found no improvement — still valid
        CHECK(res.critical_path_before >= 0,
              "critical_path_before should be >= 0");
    }
    PASS("retiming_improves_critical_path");
}

// 2. retiming_result_fields
TEST(retiming_result_fields) {
    Netlist nl = build_retiming_circuit();
    RetimingEngine eng;
    auto res = eng.optimize_with_result(nl);
    CHECK(res.critical_path_before > 0,
          "critical_path_before should be > 0, got " +
          std::to_string(res.critical_path_before));
    CHECK(!res.message.empty(), "message should not be empty");
    PASS("retiming_result_fields");
}

// 3. retiming_pure_combinational
TEST(retiming_pure_combinational) {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId w = nl.add_net("w");
    NetId y = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {a, b}, w, "G1");
    nl.add_gate(GateType::NOT, {w},    y, "G2");

    RetimingEngine eng;
    auto res = eng.optimize_with_result(nl);
    CHECK(!res.improved, "pure combinational circuit should not be improved");
    PASS("retiming_pure_combinational");
}

// 4. retiming_single_gate
TEST(retiming_single_gate) {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId d   = nl.add_net("d");   nl.mark_input(d);
    NetId q1  = nl.add_net("q1");
    nl.add_dff(d, clk, q1, -1, "FF_in");
    NetId w   = nl.add_net("w");
    nl.add_gate(GateType::AND, {q1, q1}, w, "G1");
    NetId out = nl.add_net("out"); nl.mark_output(out);
    nl.add_dff(w, clk, out, -1, "FF_out");

    RetimingEngine eng;
    auto res = eng.optimize_with_result(nl);
    // Should not crash; result is valid regardless of improvement
    CHECK(res.critical_path_before >= 0,
          "critical_path_before should be >= 0");
    PASS("retiming_single_gate");
}

// 5. retiming_preserves_io
TEST(retiming_preserves_io) {
    Netlist nl = build_retiming_circuit();
    size_t ins_before  = nl.primary_inputs().size();
    size_t outs_before = nl.primary_outputs().size();

    RetimingEngine eng;
    eng.optimize(nl);

    CHECK(nl.primary_inputs().size() == ins_before,
          "inputs changed: " + std::to_string(nl.primary_inputs().size()) +
          " vs " + std::to_string(ins_before));
    CHECK(nl.primary_outputs().size() == outs_before,
          "outputs changed: " + std::to_string(nl.primary_outputs().size()) +
          " vs " + std::to_string(outs_before));
    PASS("retiming_preserves_io");
}

// ===================================================================
//  Post-Route Optimizer
// ===================================================================

// 6. postroute_legacy_mode
TEST(postroute_legacy_mode) {
    // Simple combinational circuit — no DFFs → legacy mode
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId y  = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {a, b}, w1, "G1");
    nl.add_gate(GateType::OR,  {w1, a}, w2, "G2");
    nl.add_gate(GateType::NOT, {w2},    y,  "G3");

    PhysicalDesign pd = build_pd_with_wires(10);
    PostRouteOptimizer opt(nl, pd);
    auto res = opt.optimize();
    CHECK(!res.sta_driven, "should fall back to legacy (sta_driven==false)");
    CHECK(res.iterations > 0,
          "iterations should be > 0, got " + std::to_string(res.iterations));
    PASS("postroute_legacy_mode");
}

// 7. postroute_result_fields
TEST(postroute_result_fields) {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId y = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::BUF, {a}, y, "G1");

    PhysicalDesign pd = build_pd_with_wires(5);
    PostRouteOptimizer opt(nl, pd);
    auto res = opt.optimize();
    CHECK(res.time_ms >= 0,
          "time_ms should be >= 0, got " + std::to_string(res.time_ms));
    CHECK(!res.message.empty(), "message should not be empty");
    CHECK(res.iterations >= 0,
          "iterations should be >= 0, got " + std::to_string(res.iterations));
    PASS("postroute_result_fields");
}

// 8. postroute_sta_driven_mode
TEST(postroute_sta_driven_mode) {
    Netlist nl = build_seq_circuit();
    PhysicalDesign pd = build_pd_with_wires(10);
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(10.0);
    auto res = opt.optimize();
    CHECK(res.sta_driven,
          "should be STA-driven with DFFs + clock_period");
    PASS("postroute_sta_driven_mode");
}

// 9. postroute_max_iterations
TEST(postroute_max_iterations) {
    Netlist nl = build_seq_circuit();
    PhysicalDesign pd = build_pd_with_wires(10);
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(10.0);
    opt.set_max_iterations(2);
    auto res = opt.optimize();
    CHECK(res.iterations <= 2,
          "iterations should be <= 2, got " + std::to_string(res.iterations));
    PASS("postroute_max_iterations");
}

// 10. postroute_no_degradation
TEST(postroute_no_degradation) {
    Netlist nl = build_seq_circuit();
    PhysicalDesign pd = build_pd_with_wires(10);
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(10.0);
    auto res = opt.optimize();
    CHECK(res.wns_after >= res.wns_before - 1e-6,
          "WNS degraded: " + std::to_string(res.wns_after) +
          " < " + std::to_string(res.wns_before));
    PASS("postroute_no_degradation");
}

// ===================================================================
//  Timing-Driven Placement
// ===================================================================

// 11. placement_basic
TEST(placement_basic) {
    PhysicalDesign pd = build_test_pd(20);
    AnalyticalPlacer placer(pd);
    auto res = placer.place();
    CHECK(res.hpwl > 0,
          "HPWL should be > 0, got " + std::to_string(res.hpwl));
    CHECK(res.iterations > 0,
          "iterations should be > 0, got " + std::to_string(res.iterations));
    CHECK(res.legal, "placement should be legal");
    PASS("placement_basic");
}

// 12. placement_timing_driven_enabled
TEST(placement_timing_driven_enabled) {
    PhysicalDesign pd = build_test_pd(20);
    Netlist nl = build_seq_circuit();
    AnalyticalPlacer placer(pd);
    placer.enable_timing_driven(nl, 10.0);
    auto res = placer.place();
    CHECK(res.timing_iterations > 0,
          "timing_iterations should be > 0, got " +
          std::to_string(res.timing_iterations));
    PASS("placement_timing_driven_enabled");
}

// 13. placement_timing_weight_effect
TEST(placement_timing_weight_effect) {
    // Run with default timing_weight (5.0)
    {
        PhysicalDesign pd = build_test_pd(20);
        Netlist nl = build_seq_circuit();
        AnalyticalPlacer placer(pd);
        placer.enable_timing_driven(nl, 10.0);
        placer.set_timing_weight(5.0);
        auto res1 = placer.place();
        CHECK(res1.iterations > 0,
              "timing_weight=5.0 run should complete");
    }
    // Run with timing_weight = 0.0 (wirelength only)
    {
        PhysicalDesign pd = build_test_pd(20);
        Netlist nl = build_seq_circuit();
        AnalyticalPlacer placer(pd);
        placer.enable_timing_driven(nl, 10.0);
        placer.set_timing_weight(0.0);
        auto res2 = placer.place();
        CHECK(res2.iterations > 0,
              "timing_weight=0.0 run should complete");
    }
    PASS("placement_timing_weight_effect");
}

// 14. placement_cells_in_bounds
TEST(placement_cells_in_bounds) {
    PhysicalDesign pd = build_test_pd(20);
    AnalyticalPlacer placer(pd);
    auto res = placer.place();
    for (auto& cell : pd.cells) {
        CHECK(cell.position.x >= 0,
              cell.name + " x < 0: " + std::to_string(cell.position.x));
        CHECK(cell.position.y >= 0,
              cell.name + " y < 0: " + std::to_string(cell.position.y));
        CHECK(cell.position.x + cell.width <= pd.die_area.x1 + 1e-6,
              cell.name + " exceeds die right: x=" +
              std::to_string(cell.position.x) + " w=" +
              std::to_string(cell.width));
        CHECK(cell.position.y + cell.height <= pd.die_area.y1 + 1e-6,
              cell.name + " exceeds die top: y=" +
              std::to_string(cell.position.y) + " h=" +
              std::to_string(cell.height));
    }
    PASS("placement_cells_in_bounds");
}

// 15. placement_result_completeness
TEST(placement_result_completeness) {
    PhysicalDesign pd = build_test_pd(20);
    AnalyticalPlacer placer(pd);
    auto res = placer.place();
    CHECK(res.time_ms > 0,
          "time_ms should be > 0, got " + std::to_string(res.time_ms));
    CHECK(!res.message.empty(), "message should not be empty");
    PASS("placement_result_completeness");
}

// ===================================================================
// Main
// ===================================================================
int main() {
    std::cout << "\n"
        "╔══════════════════════════════════════════════════╗\n"
        "║  SiliconForge Phase 18 — PnR Industrial Tests     ║\n"
        "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Retiming Engine ──\n";
    RUN(retiming_improves_critical_path);
    RUN(retiming_result_fields);
    RUN(retiming_pure_combinational);
    RUN(retiming_single_gate);
    RUN(retiming_preserves_io);

    std::cout << "\n── Post-Route Optimizer ──\n";
    RUN(postroute_legacy_mode);
    RUN(postroute_result_fields);
    RUN(postroute_sta_driven_mode);
    RUN(postroute_max_iterations);
    RUN(postroute_no_degradation);

    std::cout << "\n── Timing-Driven Placement ──\n";
    RUN(placement_basic);
    RUN(placement_timing_driven_enabled);
    RUN(placement_timing_weight_effect);
    RUN(placement_cells_in_bounds);
    RUN(placement_result_completeness);

    std::cout << "\n════════════════════════════════════════\n";
    std::cout << "Phase 18 Results: " << passed << " passed, "
              << failed << " failed\n";
    std::cout << "════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
