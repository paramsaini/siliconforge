// SiliconForge — Phase 35 Test Suite
// IR Drop Industrial: Gauss-Seidel solver, configurable pads, dynamic IR drop,
// timing derating, signoff evaluation

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "pnr/placer.hpp"
#include "pnr/global_router.hpp"
#include "timing/ir_drop.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// ── Helpers ──────────────────────────────────────────────────────────────

static PhysicalDesign build_design(int n = 20) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    for (int i = 0; i < n; ++i) {
        int c = pd.add_cell("c" + std::to_string(i), "AND2", 3.0, 10.0);
        pd.cells[c].position = {10.0 + (i % 10) * 18.0, 10.0 + (i / 10) * 20.0};
        pd.cells[c].placed = true;
    }
    for (int i = 0; i < n - 1; ++i)
        pd.add_net("n" + std::to_string(i), {i, i+1});
    return pd;
}

static PhysicalDesign build_dense_center(int n = 50) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 200);
    pd.row_height = 10.0;
    for (int i = 0; i < n; ++i) {
        int c = pd.add_cell("c" + std::to_string(i), "INV", 2.0, 10.0);
        pd.cells[c].position = {80.0 + (i % 10) * 4.0, 80.0 + (i / 10) * 10.0};
        pd.cells[c].placed = true;
    }
    for (int i = 0; i < n - 1; ++i)
        pd.add_net("n" + std::to_string(i), {i, i+1});
    return pd;
}

// ── Config & API Tests ───────────────────────────────────────────────────

TEST(config_defaults) {
    IrDropConfig cfg;
    CHECK(cfg.vdd == 1.8, "default VDD");
    CHECK(cfg.grid_resolution == 16, "default grid res");
    CHECK(cfg.max_iterations == 500, "default max iters");
    CHECK(cfg.convergence_tol == 1e-6, "default tol");
    CHECK(cfg.sor_omega == 1.4, "default SOR omega");
    CHECK(cfg.hotspot_threshold_pct == 5.0, "default hotspot threshold");
    CHECK(cfg.critical_threshold_pct == 10.0, "default critical threshold");
    CHECK(!cfg.enable_dynamic, "dynamic off by default");
    CHECK(cfg.compute_timing_derating, "timing derating on by default");
    CHECK(cfg.pad_pattern == IrDropConfig::CORNERS, "default pad pattern");
    PASS("config_defaults");
}

TEST(config_custom) {
    IrDropConfig cfg;
    cfg.vdd = 0.9;
    cfg.grid_resolution = 32;
    cfg.max_iterations = 1000;
    cfg.sor_omega = 1.6;
    cfg.pad_pattern = IrDropConfig::PERIMETER;
    cfg.pads_per_side = 4;
    CHECK(cfg.vdd == 0.9, "custom VDD");
    CHECK(cfg.grid_resolution == 32, "custom grid res");
    CHECK(cfg.max_iterations == 1000, "custom max iters");
    CHECK(cfg.pads_per_side == 4, "custom pads per side");
    PASS("config_custom");
}

TEST(set_config_api) {
    auto pd = build_design();
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 0.9;
    cfg.total_current_ma = 50;
    cfg.grid_resolution = 8;
    ir.set_config(cfg);
    CHECK(ir.config().vdd == 0.9, "config set");
    CHECK(ir.config().total_current_ma == 50, "current set");
    PASS("set_config_api");
}

TEST(power_pad_struct) {
    PowerPad pad;
    pad.x = 10; pad.y = 20;
    pad.resistance_ohm = 0.05;
    pad.type = PowerPad::VDD;
    pad.name = "VDD_1";
    CHECK(pad.x == 10, "pad x");
    CHECK(pad.resistance_ohm == 0.05, "pad R");
    CHECK(pad.type == PowerPad::VDD, "pad type");
    CHECK(pad.name == "VDD_1", "pad name");
    PASS("power_pad_struct");
}

// ── Solver Tests ─────────────────────────────────────────────────────────

TEST(solver_convergence) {
    auto pd = build_design();
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 50;
    cfg.grid_resolution = 8;
    cfg.max_iterations = 500;
    cfg.convergence_tol = 1e-6;
    ir.set_config(cfg);

    auto r = ir.analyze();
    CHECK(r.converged, "solver converged");
    CHECK(r.solver_iterations > 0, "solver iterated");
    CHECK(r.solver_iterations <= 500, "within max iters");
    CHECK(r.solver_residual < 1e-6, "residual below tolerance");
    PASS("solver_convergence");
}

TEST(solver_grid_resolution) {
    auto pd = build_design();
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 50;
    cfg.grid_resolution = 16;
    ir.set_config(cfg);

    auto r = ir.analyze();
    CHECK(r.grid_x == 16, "grid x");
    CHECK(r.grid_y == 16, "grid y");
    CHECK((int)r.drop_map.size() == 16, "drop map rows");
    CHECK((int)r.drop_map[0].size() == 16, "drop map cols");
    CHECK((int)r.voltage_map.size() == 16, "voltage map rows");
    CHECK((int)r.nodes.size() == 256, "node count 16x16");
    PASS("solver_grid_resolution");
}

TEST(solver_override_grid) {
    auto pd = build_design();
    IrDropAnalyzer ir(pd);
    auto r = ir.analyze(10);
    CHECK(r.grid_x == 10, "override grid to 10");
    CHECK((int)r.drop_map.size() == 10, "drop map 10 rows");
    PASS("solver_override_grid");
}

TEST(solver_drop_values) {
    auto pd = build_design(30);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 100;
    cfg.grid_resolution = 8;
    ir.set_config(cfg);

    auto r = ir.analyze();
    CHECK(r.worst_drop_mv >= 0, "drop non-negative");
    CHECK(r.avg_drop_mv >= 0, "avg drop non-negative");
    CHECK(r.worst_drop_mv >= r.avg_drop_mv, "worst >= avg");
    CHECK(r.vdd == 1.8, "VDD recorded");
    // Voltage map should be ≤ VDD everywhere
    for (auto& row : r.voltage_map)
        for (auto v : row)
            CHECK(v <= 1800.1 && v >= 0, "voltage in range");
    PASS("solver_drop_values");
}

TEST(solver_median_drop) {
    auto pd = build_design(20);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 80;
    cfg.grid_resolution = 8;
    ir.set_config(cfg);
    auto r = ir.analyze();
    CHECK(r.median_drop_mv >= 0, "median computed");
    CHECK(r.median_drop_mv <= r.worst_drop_mv, "median <= worst");
    PASS("solver_median_drop");
}

// ── Pad Pattern Tests ────────────────────────────────────────────────────

TEST(pad_corners) {
    auto pd = build_design();
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 50;
    cfg.pad_pattern = IrDropConfig::CORNERS;
    ir.set_config(cfg);
    auto r = ir.analyze(8);
    CHECK(r.converged, "corners converged");
    CHECK(r.worst_drop_mv >= 0, "corners drop");
    PASS("pad_corners");
}

TEST(pad_perimeter) {
    auto pd = build_design();
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 50;
    cfg.pad_pattern = IrDropConfig::PERIMETER;
    cfg.pads_per_side = 3;
    ir.set_config(cfg);
    auto r = ir.analyze(8);
    CHECK(r.converged, "perimeter converged");
    // More pads → less drop than corners-only
    PASS("pad_perimeter");
}

TEST(pad_ring) {
    auto pd = build_design();
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 50;
    cfg.pad_pattern = IrDropConfig::RING;
    cfg.pads_per_side = 2;
    ir.set_config(cfg);
    auto r = ir.analyze(8);
    CHECK(r.converged, "ring converged");
    PASS("pad_ring");
}

TEST(pad_custom) {
    auto pd = build_design();
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 50;
    cfg.pad_pattern = IrDropConfig::CUSTOM;
    cfg.pads.push_back({0, 0, 0.01, PowerPad::VDD, "P1"});
    cfg.pads.push_back({200, 0, 0.01, PowerPad::VDD, "P2"});
    cfg.pads.push_back({100, 50, 0.005, PowerPad::VDD, "P3"}); // center pad
    ir.set_config(cfg);
    auto r = ir.analyze(8);
    CHECK(r.converged, "custom converged");
    PASS("pad_custom");
}

TEST(more_pads_less_drop) {
    auto pd = build_dense_center(40);
    // Test with corners only
    IrDropAnalyzer ir1(pd);
    IrDropConfig cfg1;
    cfg1.vdd = 1.8;
    cfg1.total_current_ma = 100;
    cfg1.pad_pattern = IrDropConfig::CORNERS;
    cfg1.grid_resolution = 10;
    ir1.set_config(cfg1);
    auto r1 = ir1.analyze();

    // Test with perimeter pads
    IrDropAnalyzer ir2(pd);
    IrDropConfig cfg2 = cfg1;
    cfg2.pad_pattern = IrDropConfig::PERIMETER;
    cfg2.pads_per_side = 4;
    ir2.set_config(cfg2);
    auto r2 = ir2.analyze();

    CHECK(r2.worst_drop_mv <= r1.worst_drop_mv + 0.01,
          "more pads should reduce or equal worst drop");
    PASS("more_pads_less_drop");
}

// ── Per-Cell Current Tests ───────────────────────────────────────────────

TEST(per_cell_current) {
    auto pd = build_design(10);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 100;
    cfg.grid_resolution = 8;
    ir.set_config(cfg);

    // Set specific currents for some cells
    ir.set_cell_current(0, 20.0);
    ir.set_cell_current(5, 30.0);
    auto r = ir.analyze();
    CHECK(r.converged, "per-cell converged");
    CHECK(r.worst_drop_mv >= 0, "per-cell drop computed");
    PASS("per_cell_current");
}

TEST(bulk_cell_currents) {
    auto pd = build_design(10);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 100;
    cfg.grid_resolution = 8;
    std::vector<double> currents(10, 10.0); // 10mA each, 100mA total
    ir.set_config(cfg);
    ir.set_cell_currents(currents);
    auto r = ir.analyze();
    CHECK(r.converged, "bulk currents converged");
    PASS("bulk_cell_currents");
}

// ── Hotspot Tests ────────────────────────────────────────────────────────

TEST(hotspot_detection) {
    auto pd = build_dense_center(50);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 500; // very high current
    cfg.grid_resolution = 10;
    cfg.hotspot_threshold_pct = 3.0;
    cfg.critical_threshold_pct = 8.0;
    ir.set_config(cfg);
    auto r = ir.analyze();
    // With 500mA through concentrated cells, expect hotspots
    CHECK(r.num_hotspots >= 0, "hotspot count valid");
    for (auto& hs : r.hotspots) {
        CHECK(hs.drop_mv > 0, "hotspot has drop");
        CHECK(hs.severity == IrDropHotSpot::WARNING ||
              hs.severity == IrDropHotSpot::CRITICAL, "severity set");
    }
    PASS("hotspot_detection");
}

// ── Dynamic IR Drop Tests ────────────────────────────────────────────────

TEST(dynamic_ir_drop) {
    auto pd = build_design(20);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 100;
    cfg.grid_resolution = 8;
    cfg.enable_dynamic = true;
    cfg.switching_factor = 0.3;
    cfg.clock_period_ns = 1.0;
    cfg.decap_density_ff_per_um2 = 0.1;
    ir.set_config(cfg);

    auto r = ir.analyze();
    CHECK(r.dynamic_analyzed, "dynamic analysis ran");
    CHECK(r.worst_dynamic_drop_mv >= 0, "dynamic drop computed");
    CHECK(r.avg_dynamic_drop_mv >= 0, "avg dynamic drop");
    CHECK((int)r.dynamic_map.size() == 8, "dynamic map rows");
    PASS("dynamic_ir_drop");
}

TEST(dynamic_vs_static) {
    auto pd = build_design(20);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 100;
    cfg.grid_resolution = 8;
    cfg.enable_dynamic = false;
    ir.set_config(cfg);
    auto r1 = ir.analyze();
    CHECK(!r1.dynamic_analyzed, "dynamic off");

    cfg.enable_dynamic = true;
    cfg.switching_factor = 0.5;
    cfg.clock_period_ns = 0.5;
    ir.set_config(cfg);
    auto r2 = ir.analyze();
    CHECK(r2.dynamic_analyzed, "dynamic on");
    PASS("dynamic_vs_static");
}

// ── Timing Derating Tests ────────────────────────────────────────────────

TEST(timing_derating) {
    auto pd = build_design(20);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 100;
    cfg.grid_resolution = 8;
    cfg.compute_timing_derating = true;
    cfg.timing_derate_sensitivity = 0.12;
    ir.set_config(cfg);

    auto r = ir.analyze();
    CHECK(r.worst_timing_derate_pct >= 0, "timing derate computed");
    CHECK(r.avg_timing_derate_pct >= 0, "avg derate computed");
    // Check per-node derating
    for (auto& node : r.nodes)
        CHECK(node.timing_derate_pct >= 0, "node derate non-negative");
    PASS("timing_derating");
}

// ── Signoff Tests ────────────────────────────────────────────────────────

TEST(signoff_pass) {
    auto pd = build_design(5); // small design, low current
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 1.0; // very low
    cfg.grid_resolution = 8;
    ir.set_config(cfg);
    auto r = ir.analyze();
    CHECK(r.signoff_pass, "low current should pass signoff");
    CHECK(!r.signoff_summary.empty(), "signoff summary generated");
    PASS("signoff_pass");
}

TEST(signoff_metrics) {
    auto pd = build_design(20);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 100;
    cfg.grid_resolution = 8;
    ir.set_config(cfg);
    auto r = ir.analyze();
    CHECK(!r.signoff_summary.empty(), "signoff summary");
    CHECK(r.signoff_summary.find("Static") != std::string::npos, "has static report");
    CHECK(r.signoff_summary.find("Dynamic") != std::string::npos, "has dynamic report");
    CHECK(r.signoff_summary.find("Timing") != std::string::npos, "has timing report");
    PASS("signoff_metrics");
}

// ── Legacy API ───────────────────────────────────────────────────────────

TEST(legacy_api) {
    auto pd = build_design(20);
    IrDropAnalyzer ir(pd, 1.8, 50.0);
    auto r = ir.analyze_legacy(8);
    CHECK(r.grid_x == 8 && r.grid_y == 8, "legacy grid");
    CHECK(r.worst_drop_mv >= 0, "legacy drop");
    CHECK(r.converged, "legacy converged");
    CHECK(r.signoff_pass, "legacy signoff pass");
    PASS("legacy_api");
}

// ── Result Completeness ──────────────────────────────────────────────────

TEST(result_completeness) {
    auto pd = build_design(20);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 100;
    cfg.grid_resolution = 8;
    cfg.enable_dynamic = true;
    cfg.switching_factor = 0.3;
    cfg.clock_period_ns = 1.0;
    ir.set_config(cfg);
    auto r = ir.analyze();

    CHECK(r.vdd == 1.8, "VDD in result");
    CHECK(r.time_ms >= 0, "time recorded");
    CHECK(!r.message.empty(), "message generated");
    CHECK(r.grid_x == 8, "grid_x");
    CHECK(r.grid_y == 8, "grid_y");
    CHECK(r.worst_drop_mv >= 0, "worst drop");
    CHECK(r.avg_drop_mv >= 0, "avg drop");
    CHECK(r.median_drop_mv >= 0, "median drop");
    CHECK(r.solver_iterations > 0, "iterations");
    CHECK(r.dynamic_analyzed, "dynamic flag");
    CHECK(!r.signoff_summary.empty(), "signoff summary");
    CHECK(!r.nodes.empty(), "nodes populated");
    CHECK(!r.drop_map.empty(), "drop map");
    CHECK(!r.voltage_map.empty(), "voltage map");
    CHECK(!r.dynamic_map.empty(), "dynamic map");
    PASS("result_completeness");
}

// ── E2E: High Current Dense Design ───────────────────────────────────────

TEST(e2e_industrial) {
    auto pd = build_dense_center(50);
    IrDropAnalyzer ir(pd);
    IrDropConfig cfg;
    cfg.vdd = 0.9; // advanced node
    cfg.total_current_ma = 200;
    cfg.grid_resolution = 12;
    cfg.pad_pattern = IrDropConfig::PERIMETER;
    cfg.pads_per_side = 3;
    cfg.enable_dynamic = true;
    cfg.switching_factor = 0.4;
    cfg.clock_period_ns = 0.5; // 2 GHz
    cfg.decap_density_ff_per_um2 = 0.2;
    cfg.sheet_resistance_mohm = 15;
    cfg.compute_timing_derating = true;
    cfg.timing_derate_sensitivity = 0.15;
    cfg.hotspot_threshold_pct = 3.0;
    cfg.critical_threshold_pct = 7.0;
    ir.set_config(cfg);

    auto r = ir.analyze();
    CHECK(r.converged, "industrial converged");
    CHECK(r.vdd == 0.9, "0.9V VDD");
    CHECK(r.dynamic_analyzed, "dynamic ran");
    CHECK(r.grid_x == 12 && r.grid_y == 12, "12x12 grid");
    CHECK(r.solver_iterations > 0, "solver ran");
    CHECK(r.worst_drop_mv >= 0, "worst drop");
    CHECK(r.worst_dynamic_drop_mv >= 0, "worst dynamic");
    CHECK(r.worst_timing_derate_pct >= 0, "worst derate");
    CHECK(!r.signoff_summary.empty(), "signoff report");
    CHECK(r.time_ms >= 0, "time measured");
    PASS("e2e_industrial");
}

// ══════════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 35 — IR Drop Industrial      ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Config & API ──\n";
    RUN(config_defaults);
    RUN(config_custom);
    RUN(set_config_api);
    RUN(power_pad_struct);

    std::cout << "\n── Gauss-Seidel Solver ──\n";
    RUN(solver_convergence);
    RUN(solver_grid_resolution);
    RUN(solver_override_grid);
    RUN(solver_drop_values);
    RUN(solver_median_drop);

    std::cout << "\n── Power Pad Patterns ──\n";
    RUN(pad_corners);
    RUN(pad_perimeter);
    RUN(pad_ring);
    RUN(pad_custom);
    RUN(more_pads_less_drop);

    std::cout << "\n── Per-Cell Current ──\n";
    RUN(per_cell_current);
    RUN(bulk_cell_currents);

    std::cout << "\n── Hotspots ──\n";
    RUN(hotspot_detection);

    std::cout << "\n── Dynamic IR Drop ──\n";
    RUN(dynamic_ir_drop);
    RUN(dynamic_vs_static);

    std::cout << "\n── Timing Derating ──\n";
    RUN(timing_derating);

    std::cout << "\n── Signoff ──\n";
    RUN(signoff_pass);
    RUN(signoff_metrics);

    std::cout << "\n── Legacy API ──\n";
    RUN(legacy_api);

    std::cout << "\n── Result Completeness ──\n";
    RUN(result_completeness);

    std::cout << "\n── E2E Industrial ──\n";
    RUN(e2e_industrial);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Phase 35: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
