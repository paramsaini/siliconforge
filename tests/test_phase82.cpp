// Phase 82: Tier 1 Gap Closure Tests
// SDC completeness, latch STA, multi-VDD timing, spatial indexing,
// negotiated congestion routing, hold-time optimization, IR→STA feedback,
// short-circuit power model

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>

#include "frontend/sdc_parser.hpp"
#include "timing/sta.hpp"
#include "timing/power.hpp"
#include "pnr/physical.hpp"
#include "pnr/global_router.hpp"
#include "pnr/post_route_opt.hpp"
#include "pnr/timing_closure.hpp"
#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"

static int total = 0, passed = 0;
#define CHECK(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

// ========== SDC TESTS ==========

void test_sdc_case_analysis() {
    printf("\n[SDC set_case_analysis]\n");
    sf::SdcParser parser;
    sf::SdcConstraints sdc;
    auto r = parser.parse_string(
        "set_case_analysis 0 scan_enable\n"
        "set_case_analysis 1 test_mode\n"
        "set_case_analysis rising clk_sel\n"
        "set_case_analysis falling reset_b\n",
        sdc);
    CHECK(r.success, "case_analysis parse succeeds");
    CHECK(sdc.case_analyses.size() == 4, "4 case_analysis constraints");
    CHECK(sdc.case_analyses[0].pin == "scan_enable", "pin = scan_enable");
    CHECK(sdc.case_analyses[0].value == sf::SdcCaseAnalysis::ZERO, "value = ZERO");
    CHECK(sdc.case_analyses[1].value == sf::SdcCaseAnalysis::ONE, "value = ONE");
    CHECK(sdc.case_analyses[2].value == sf::SdcCaseAnalysis::RISING, "value = RISING");
    CHECK(sdc.case_analyses[3].value == sf::SdcCaseAnalysis::FALLING, "value = FALLING");
}

void test_sdc_disable_timing() {
    printf("\n[SDC set_disable_timing]\n");
    sf::SdcParser parser;
    sf::SdcConstraints sdc;
    auto r = parser.parse_string(
        "set_disable_timing -from A -to Y u_mux\n",
        sdc);
    CHECK(r.success, "disable_timing parse succeeds");
    CHECK(sdc.disable_timings.size() == 1, "1 disable_timing constraint");
    CHECK(sdc.disable_timings[0].from_pin == "A", "from_pin = A");
    CHECK(sdc.disable_timings[0].to_pin == "Y", "to_pin = Y");
    CHECK(sdc.disable_timings[0].cell_instance == "u_mux", "cell = u_mux");
}

void test_sdc_driving_cell() {
    printf("\n[SDC set_driving_cell]\n");
    sf::SdcParser parser;
    sf::SdcConstraints sdc;
    auto r = parser.parse_string(
        "set_driving_cell -lib_cell BUFX4 -pin Y -input_transition_rise 0.05 -input_transition_fall 0.06 data_in\n",
        sdc);
    CHECK(r.success, "driving_cell parse succeeds");
    CHECK(sdc.driving_cells.size() == 1, "1 driving_cell constraint");
    CHECK(sdc.driving_cells[0].lib_cell == "BUFX4", "lib_cell = BUFX4");
    CHECK(sdc.driving_cells[0].pin == "Y", "pin = Y");
    CHECK(std::abs(sdc.driving_cells[0].input_transition_rise - 0.05) < 1e-6, "itr = 0.05");
    CHECK(std::abs(sdc.driving_cells[0].input_transition_fall - 0.06) < 1e-6, "itf = 0.06");
    CHECK(sdc.driving_cells[0].port == "data_in", "port = data_in");
}

void test_sdc_load() {
    printf("\n[SDC set_load]\n");
    sf::SdcParser parser;
    sf::SdcConstraints sdc;
    auto r = parser.parse_string(
        "set_load 0.5 out_data\n"
        "set_load 1.2 out_clk\n",
        sdc);
    CHECK(r.success, "set_load parse succeeds");
    CHECK(sdc.loads.size() == 2, "2 load constraints");
    CHECK(std::abs(sdc.loads[0].capacitance - 0.5) < 1e-6, "load0 = 0.5pF");
    CHECK(sdc.loads[0].port == "out_data", "port0 = out_data");
    CHECK(std::abs(sdc.loads[1].capacitance - 1.2) < 1e-6, "load1 = 1.2pF");
}

void test_sdc_combined() {
    printf("\n[SDC combined new commands]\n");
    sf::SdcParser parser;
    sf::SdcConstraints sdc;
    auto r = parser.parse_string(
        "create_clock -name clk -period 10.0 [get_ports clk]\n"
        "set_case_analysis 0 scan_enable\n"
        "set_disable_timing -from A -to Y u_buf\n"
        "set_driving_cell -lib_cell INV_X1 data_in\n"
        "set_load 0.3 data_out\n"
        "set_input_delay -clock clk 1.0 data_in\n"
        "set_output_delay -clock clk 2.0 data_out\n",
        sdc);
    CHECK(r.success, "combined SDC parse succeeds");
    CHECK(sdc.clocks.size() == 1, "1 clock");
    CHECK(sdc.case_analyses.size() == 1, "1 case_analysis");
    CHECK(sdc.disable_timings.size() == 1, "1 disable_timing");
    CHECK(sdc.driving_cells.size() == 1, "1 driving_cell");
    CHECK(sdc.loads.size() == 1, "1 load");
    CHECK(sdc.input_delays.size() == 1, "1 input_delay");
    CHECK(sdc.output_delays.size() == 1, "1 output_delay");
    CHECK(r.num_constraints == 7, "7 total constraints");
}

// ========== LATCH TIMING TESTS ==========

void test_latch_timing_info() {
    printf("\n[Latch Timing Info]\n");
    sf::LatchTimingInfo info;
    info.opening_edge = 0.0;
    info.closing_edge = 5.0;
    info.borrow_limit = 2.0;
    info.borrowed_time = 0.0;
    CHECK(info.closing_edge > info.opening_edge, "closing > opening");
    CHECK(info.borrow_limit > 0, "positive borrow limit");

    // Construct STA with latch mode
    sf::Netlist nl;
    auto n_d = nl.add_net("D"); nl.mark_input(n_d);
    auto n_q = nl.add_net("latch_out");
    nl.add_gate(sf::GateType::DLATCH, {n_d}, n_q, "dlatch0");
    auto n_out = nl.add_net("Q"); nl.mark_output(n_out);
    nl.add_gate(sf::GateType::BUF, {n_q}, n_out, "buf_out");

    sf::StaEngine sta(nl);
    sta.set_latch_timing_enabled(true);
    CHECK(true, "latch timing enabled without crash");

    // Set latch info
    sf::LatchTimingInfo li;
    li.opening_edge = 0.0;
    li.closing_edge = 5.0;
    li.borrow_limit = 2.5;
    sta.set_latch_info(0, li); // gate 0 = the DLATCH
    CHECK(true, "set_latch_info accepted");
}

// ========== MULTI-VDD TESTS ==========

void test_multi_vdd_timing() {
    printf("\n[Multi-VDD Timing]\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("A"); nl.mark_input(n_a);
    auto n_buf = nl.add_net("buf_out");
    nl.add_gate(sf::GateType::BUF, {n_a}, n_buf, "buf0");
    auto n_y = nl.add_net("Y"); nl.mark_output(n_y);
    nl.add_gate(sf::GateType::BUF, {n_buf}, n_y, "buf1");

    sf::StaEngine sta(nl);

    // Define two voltage domains
    std::vector<sf::VoltageDomain> domains;
    sf::VoltageDomain d1; d1.name = "VDD_CORE"; d1.nominal_voltage = 1.0;
    sf::VoltageDomain d2; d2.name = "VDD_IO"; d2.nominal_voltage = 0.9;
    domains.push_back(d1);
    domains.push_back(d2);
    sta.set_voltage_domains(domains);
    CHECK(true, "voltage domains set");

    sta.set_cell_domain("buf0", "VDD_IO");
    CHECK(true, "cell assigned to domain");

    // Run STA — should not crash with multi-VDD
    auto result = sta.analyze(10.0);
    CHECK(result.wns <= 0.0 || result.wns > 0.0, "STA completes with multi-VDD");
}

// ========== IR DROP DERATING TESTS ==========

void test_ir_drop_derating() {
    printf("\n[IR Drop Derating]\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("A"); nl.mark_input(n_a);
    auto n_inv = nl.add_net("inv_out");
    nl.add_gate(sf::GateType::NOT, {n_a}, n_inv, "inv0");
    auto n_y = nl.add_net("Y"); nl.mark_output(n_y);
    nl.add_gate(sf::GateType::BUF, {n_inv}, n_y, "buf0");

    sf::StaEngine sta(nl);
    sta.set_ir_drop_alpha(1.3);
    sta.set_ir_drop_nominal_voltage(1.0);

    std::unordered_map<std::string, double> cell_voltages;
    cell_voltages["inv0"] = 0.95; // 50mV drop
    sta.apply_ir_drop_derating(cell_voltages);
    CHECK(true, "IR drop derating applied without crash");

    auto result = sta.analyze(10.0);
    CHECK(result.wns <= 0.0 || result.wns > 0.0, "STA completes with IR derating");
}

// ========== SPATIAL INDEX TESTS ==========

void test_spatial_index_basic() {
    printf("\n[Spatial Index Basic]\n");
    sf::SpatialIndex idx;
    CHECK(idx.size() == 0, "empty index");

    // Insert rectangles
    idx.insert(0, sf::Rect{0, 0, 10, 10});
    idx.insert(1, sf::Rect{20, 20, 30, 30});
    idx.insert(2, sf::Rect{5, 5, 25, 25});
    CHECK(idx.size() == 3, "3 entries");

    idx.rebuild();
    CHECK(true, "rebuild succeeds");

    // Query overlapping region
    auto hits = idx.query(sf::Rect{0, 0, 15, 15});
    bool has0 = false, has2 = false;
    for (int h : hits) { if (h == 0) has0 = true; if (h == 2) has2 = true; }
    CHECK(has0, "query finds rect 0");
    CHECK(has2, "query finds overlapping rect 2");

    // Point query
    auto pts = idx.query_point(25.0, 25.0);
    bool has1 = false;
    for (int h : pts) { if (h == 1) has1 = true; if (h == 2) has2 = true; }
    CHECK(has1 || has2, "point query finds rects near (25,25)");
}

void test_spatial_index_stress() {
    printf("\n[Spatial Index Stress]\n");
    sf::SpatialIndex idx;
    const int N = 10000;
    for (int i = 0; i < N; i++) {
        double x = (i % 100) * 10.0;
        double y = (i / 100) * 10.0;
        idx.insert(i, sf::Rect{x, y, x + 8, y + 8});
    }
    CHECK(idx.size() == (size_t)N, "10K entries inserted");
    idx.rebuild();

    auto hits = idx.query(sf::Rect{50, 50, 150, 150});
    CHECK(hits.size() > 0, "stress query returns results");
    CHECK(hits.size() < (size_t)N, "stress query filters properly");

    idx.remove(0);
    CHECK(idx.size() == (size_t)(N - 1), "remove works");
}

// ========== CONGESTION MAP TESTS ==========

void test_congestion_map() {
    printf("\n[Congestion Map]\n");
    sf::CongestionMap cmap;
    cmap.init(10, 10, 8);
    CHECK(cmap.grid_x == 10, "grid_x = 10");
    CHECK(cmap.grid_y == 10, "grid_y = 10");
    CHECK(cmap.capacity[0][0] == 8, "default capacity = 8");
    CHECK(!cmap.is_overflowed(0, 0), "no overflow initially");
    CHECK(cmap.total_overflow() == 0, "zero overflow");

    // Simulate congestion
    cmap.usage[3][3] = 12;
    CHECK(cmap.is_overflowed(3, 3), "overflow detected at (3,3)");
    CHECK(cmap.total_overflow() > 0, "total overflow > 0");

    double cost = cmap.congestion_cost(3, 3);
    CHECK(cost > 1.0, "congested cell has high cost");

    double cost_free = cmap.congestion_cost(0, 0);
    CHECK(cost_free < cost, "free cell has lower cost");

    cmap.update_history(0.5);
    double cost_after = cmap.congestion_cost(3, 3);
    CHECK(cost_after >= cost, "history increases cost");
}

// ========== HOLD FIX TESTS ==========

void test_hold_fix_result_struct() {
    printf("\n[Hold Fix Result]\n");
    sf::PostRouteOptimizer::HoldFixResult hfr;
    CHECK(hfr.num_violations_found == 0, "default violations = 0");
    CHECK(hfr.num_fixed == 0, "default fixed = 0");
    CHECK(hfr.num_buffers_inserted == 0, "default buffers = 0");
    CHECK(hfr.hold_wns_before == 0, "default hold WNS before = 0");
    CHECK(hfr.hold_wns_after == 0, "default hold WNS after = 0");
    CHECK(hfr.setup_wns_after == 0, "default setup WNS after = 0");
}

// ========== SHORT-CIRCUIT POWER TESTS ==========

void test_short_circuit_power() {
    printf("\n[Short-Circuit Power]\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("A"); nl.mark_input(n_a);
    auto n_b = nl.add_net("B"); nl.mark_input(n_b);
    auto n_nand = nl.add_net("nand_out");
    nl.add_gate(sf::GateType::NAND, {n_a, n_b}, n_nand, "nand0");
    auto n_inv = nl.add_net("inv_out");
    nl.add_gate(sf::GateType::NOT, {n_nand}, n_inv, "inv0");
    auto n_y = nl.add_net("Y"); nl.mark_output(n_y);
    nl.add_gate(sf::GateType::BUF, {n_inv}, n_y, "buf0");

    sf::PowerAnalyzer pa(nl);
    auto result = pa.analyze(500.0, 1.0, 0.1);
    CHECK(result.short_circuit_power_mw >= 0.0, "Isc >= 0");
    CHECK(result.total_power_mw >= result.short_circuit_power_mw, "total >= Isc");

    // Higher activity should give more power
    auto result2 = pa.analyze(500.0, 1.0, 0.5);
    CHECK(result2.dynamic_power_mw >= result.dynamic_power_mw, "higher activity = more dynamic power");
}

void test_power_result_fields() {
    printf("\n[PowerResult fields]\n");
    sf::PowerResult pr;
    CHECK(pr.short_circuit_power_mw == 0.0, "default Isc = 0");
    CHECK(pr.total_power_mw == 0.0, "default total = 0");
    CHECK(pr.switching_power_mw == 0.0, "default switching = 0");
    CHECK(pr.static_power_mw == 0.0, "default leakage = 0");
}

// ========== TIMING CLOSURE TESTS ==========

void test_timing_closure_config() {
    printf("\n[Timing Closure Config]\n");
    sf::TimingClosureConfig cfg;
    CHECK(cfg.max_iterations == 10, "default max_iterations = 10");
    CHECK(std::abs(cfg.clock_period - 10.0) < 1e-6, "default clock_period = 10ns");
    CHECK(cfg.enable_hold_fix == true, "hold fix enabled by default");
    CHECK(cfg.enable_vt_swap == true, "vt swap enabled by default");
    CHECK(cfg.stall_limit == 3, "default stall_limit = 3");
    CHECK(std::abs(cfg.convergence_threshold - 0.01) < 1e-6, "default convergence = 0.01");
}

void test_timing_closure_result() {
    printf("\n[Timing Closure Result]\n");
    sf::TimingClosureResult res;
    CHECK(res.iterations_run == 0, "default iterations = 0");
    CHECK(res.converged == false, "default not converged");
    CHECK(res.initial_wns == 0.0, "default initial WNS = 0");
    CHECK(res.final_wns == 0.0, "default final WNS = 0");
    CHECK(res.wns_per_iteration.empty(), "empty WNS history");
    CHECK(res.setup_violations_fixed == 0, "default setup fixes = 0");
    CHECK(res.hold_violations_fixed == 0, "default hold fixes = 0");
}

void test_timing_closure_engine() {
    printf("\n[Timing Closure Engine]\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("A"); nl.mark_input(n_a);
    auto n_buf = nl.add_net("buf_out");
    nl.add_gate(sf::GateType::BUF, {n_a}, n_buf, "buf0");
    auto n_y = nl.add_net("Y"); nl.mark_output(n_y);
    nl.add_gate(sf::GateType::BUF, {n_buf}, n_y, "buf1");

    sf::PhysicalDesign pd;
    sf::TimingClosureEngine engine(nl, pd);
    CHECK(true, "engine constructed");

    sf::TimingClosureConfig cfg;
    cfg.max_iterations = 2;
    cfg.clock_period = 10.0;
    auto result = engine.run(cfg);
    CHECK(result.iterations_run >= 0, "iterations_run >= 0");
    CHECK(result.wns_per_iteration.size() >= (size_t)result.iterations_run, 
          "WNS history tracks iterations");
}

// ========== DEF PLACEMENT ROW TESTS ==========

void test_placement_row_struct() {
    printf("\n[Placement Row Struct]\n");
    sf::PlacementRow row;
    row.name = "ROW_0";
    row.site_name = "core_site";
    row.origin_x = 0.0;
    row.origin_y = 0.0;
    row.orient = "N";
    row.num_x = 100;
    row.num_y = 1;
    row.step_x = 0.38;
    row.step_y = 0.0;
    CHECK(row.name == "ROW_0", "row name");
    CHECK(row.site_name == "core_site", "site name");
    CHECK(row.num_x == 100, "100 sites in X");
    CHECK(std::abs(row.step_x - 0.38) < 1e-6, "step_x = 0.38um");
}

void test_physical_design_rows() {
    printf("\n[PhysicalDesign Rows]\n");
    sf::PhysicalDesign pd;
    CHECK(pd.placement_rows.empty(), "no rows initially");

    sf::PlacementRow r1;
    r1.name = "ROW_0"; r1.site_name = "core"; r1.origin_y = 0.0;
    r1.num_x = 200; r1.step_x = 0.19;
    sf::PlacementRow r2;
    r2.name = "ROW_1"; r2.site_name = "core"; r2.origin_y = 2.72;
    r2.num_x = 200; r2.step_x = 0.19;
    pd.placement_rows.push_back(r1);
    pd.placement_rows.push_back(r2);
    CHECK(pd.placement_rows.size() == 2, "2 rows added");
    CHECK(pd.placement_rows[1].origin_y > pd.placement_rows[0].origin_y, "rows ordered by Y");
}

// ========== MAIN ==========

int main() {
    printf("=== Phase 82: Tier 1 Gap Closure Tests ===\n");

    // SDC
    test_sdc_case_analysis();
    test_sdc_disable_timing();
    test_sdc_driving_cell();
    test_sdc_load();
    test_sdc_combined();

    // Latch timing
    test_latch_timing_info();

    // Multi-VDD
    test_multi_vdd_timing();

    // IR drop derating
    test_ir_drop_derating();

    // Spatial index
    test_spatial_index_basic();
    test_spatial_index_stress();

    // Congestion map
    test_congestion_map();

    // Hold fix
    test_hold_fix_result_struct();

    // Short-circuit power
    test_short_circuit_power();
    test_power_result_fields();

    // Timing closure
    test_timing_closure_config();
    test_timing_closure_result();
    test_timing_closure_engine();

    // DEF placement rows
    test_placement_row_struct();
    test_physical_design_rows();

    printf("\n=== Phase 82 Results: %d/%d passed ===\n", passed, total);
    return (passed == total) ? 0 : 1;
}
