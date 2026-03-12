// SiliconForge Phase 20 — Industrial STA Features Tests
// CPPR, POCV, PBA, and Crosstalk analysis verification.

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
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
// Helper: Sequential circuit with DFF
// ---------------------------------------------------------------------------
static Netlist build_seq_circuit() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId d = nl.add_net("d"); nl.mark_input(d);
    NetId q = nl.add_net("q"); nl.mark_output(q);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    nl.add_gate(GateType::AND, {d, q}, w1, "G1");
    nl.add_gate(GateType::NOT, {w1}, w2, "G2");
    nl.add_dff(w2, clk, q, -1, "FF1");
    return nl;
}

// ---------------------------------------------------------------------------
// Helper: Deep pipeline for POCV testing (10 inverters → DFF)
// ---------------------------------------------------------------------------
static Netlist build_deep_pipeline() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId in = nl.add_net("in"); nl.mark_input(in);
    NetId prev = in;
    for (int i = 0; i < 10; i++) {
        NetId next = nl.add_net("w" + std::to_string(i));
        nl.add_gate(GateType::NOT, {prev}, next, "G" + std::to_string(i));
        prev = next;
    }
    NetId q = nl.add_net("q"); nl.mark_output(q);
    nl.add_dff(prev, clk, q, -1, "FF1");
    return nl;
}

// ---------------------------------------------------------------------------
// Helper: Two-stage DFF pipeline (FF1 → gates → FF2) for CPPR
// ---------------------------------------------------------------------------
static Netlist build_two_stage_pipeline() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId in = nl.add_net("in"); nl.mark_input(in);
    NetId q1 = nl.add_net("q1");
    nl.add_dff(in, clk, q1, -1, "FF1");
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    nl.add_gate(GateType::AND, {q1, q1}, w1, "G1");
    nl.add_gate(GateType::NOT, {w1}, w2, "G2");
    NetId q2 = nl.add_net("q2"); nl.mark_output(q2);
    nl.add_dff(w2, clk, q2, -1, "FF2");
    return nl;
}

// ---------------------------------------------------------------------------
// Helper: Design with physical wires for crosstalk
// ---------------------------------------------------------------------------
static PhysicalDesign build_xtalk_design() {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 50);
    pd.row_height = 10.0;
    pd.add_cell("c0", "AND2", 3, 10);
    pd.add_cell("c1", "AND2", 3, 10);
    pd.add_cell("c2", "AND2", 3, 10);
    pd.add_net("victim_net", {0, 1});
    pd.add_net("aggressor_net", {1, 2});
    // Two parallel wires very close together (0.3um spacing)
    WireSegment w1;
    w1.layer = 0; w1.start = {0, 10}; w1.end = {50, 10}; w1.width = 0.14; w1.net_id = 0;
    WireSegment w2;
    w2.layer = 0; w2.start = {0, 10.3}; w2.end = {50, 10.3}; w2.width = 0.14; w2.net_id = 1;
    pd.wires.push_back(w1);
    pd.wires.push_back(w2);
    return pd;
}

// ═══════════════════════════════════════════════════════════════════════════
//  CPPR Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(cppr_disabled_by_default) {
    Netlist nl = build_seq_circuit();
    StaEngine sta(nl);
    auto result = sta.analyze(10.0);
    CHECK(result.cppr_enabled == false, "CPPR should be disabled by default");
    for (auto& p : result.critical_paths) {
        CHECK(p.cppr_credit == 0, "cppr_credit should be 0 when CPPR disabled");
    }
    PASS("cppr_disabled_by_default");
}

TEST(cppr_with_ocv) {
    Netlist nl = build_two_stage_pipeline();
    StaEngine sta(nl);
    sta.enable_ocv(1.15, 0.85);
    auto& ffs = nl.flip_flops();
    for (GateId ff : ffs)
        sta.set_clock_insertion(ff, 0.5);
    sta.enable_cppr();
    auto result = sta.analyze(10.0);
    CHECK(result.cppr_enabled == true, "CPPR should be enabled after enable_cppr()");
    bool any_credit = false;
    for (auto& p : result.critical_paths)
        if (p.cppr_credit > 0) any_credit = true;
    CHECK(any_credit || result.cppr_total_credit >= 0,
          "CPPR credit should be non-negative with OCV and shared clock");
    PASS("cppr_with_ocv");
}

TEST(cppr_with_aocv) {
    Netlist nl = build_two_stage_pipeline();
    StaEngine sta(nl);
    sta.enable_aocv(0.10, 0.10);
    auto& ffs = nl.flip_flops();
    for (GateId ff : ffs)
        sta.set_clock_insertion(ff, 0.5);
    sta.enable_cppr();
    auto result = sta.analyze(10.0);
    CHECK(result.cppr_enabled == true, "CPPR should be enabled with AOCV");
    CHECK(result.cppr_total_credit >= 0, "CPPR total credit must be non-negative");
    PASS("cppr_with_aocv");
}

TEST(cppr_reduces_pessimism) {
    Netlist nl = build_two_stage_pipeline();

    // Run 1: OCV without CPPR
    StaEngine sta1(nl);
    sta1.enable_ocv(1.15, 0.85);
    auto& ffs = nl.flip_flops();
    for (GateId ff : ffs)
        sta1.set_clock_insertion(ff, 0.5);
    auto r1 = sta1.analyze(10.0);

    // Run 2: OCV with CPPR
    StaEngine sta2(nl);
    sta2.enable_ocv(1.15, 0.85);
    for (GateId ff : ffs)
        sta2.set_clock_insertion(ff, 0.5);
    sta2.enable_cppr();
    auto r2 = sta2.analyze(10.0);

    CHECK(r2.cppr_total_credit >= 0,
          "CPPR total credit is always non-negative");
    // CPPR should not make things worse — credit removes pessimism
    CHECK(r2.wns >= r1.wns || std::abs(r2.wns - r1.wns) < 1e-9,
          "CPPR should not worsen WNS (removes pessimism)");
    PASS("cppr_reduces_pessimism");
}

// ═══════════════════════════════════════════════════════════════════════════
//  POCV Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(pocv_table_default_sigma) {
    PocvTable pt;
    pt.default_sigma_pct = 0.05;
    CHECK(std::abs(pt.get_sigma("UNKNOWN") - 0.05) < 1e-12,
          "Default sigma should be 0.05 for unknown cell type");
    PASS("pocv_table_default_sigma");
}

TEST(pocv_table_per_cell_sigma) {
    PocvTable pt;
    pt.default_sigma_pct = 0.05;
    pt.set_cell_sigma("AND", 0.08);
    CHECK(std::abs(pt.get_sigma("AND") - 0.08) < 1e-12,
          "AND sigma should be 0.08");
    CHECK(std::abs(pt.get_sigma("NOT") - 0.05) < 1e-12,
          "NOT sigma should fall back to default 0.05");
    PASS("pocv_table_per_cell_sigma");
}

TEST(pocv_mode_enabled) {
    Netlist nl = build_deep_pipeline();
    StaEngine sta(nl);
    sta.enable_pocv(3.0, 0.05);
    auto result = sta.analyze(10.0);
    CHECK(result.ocv_mode == OcvMode::POCV, "OCV mode should be POCV");
    PASS("pocv_mode_enabled");
}

TEST(pocv_with_pba_path_sigma) {
    Netlist nl = build_deep_pipeline();
    StaEngine sta(nl);
    sta.enable_pocv(3.0, 0.05);
    sta.enable_pba();
    auto result = sta.analyze(10.0);
    CHECK(result.ocv_mode == OcvMode::POCV, "Should be in POCV mode");
    for (auto& p : result.critical_paths) {
        CHECK(p.path_sigma >= 0, "path_sigma must be non-negative");
        if (p.pba_valid)
            CHECK(p.path_sigma >= 0, "PBA-valid path should have valid sigma");
    }
    PASS("pocv_with_pba_path_sigma");
}

// ═══════════════════════════════════════════════════════════════════════════
//  PBA Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(pba_disabled_by_default) {
    Netlist nl = build_seq_circuit();
    StaEngine sta(nl);
    auto result = sta.analyze(10.0);
    CHECK(result.pba_enabled == false, "PBA should be disabled by default");
    for (auto& p : result.critical_paths) {
        CHECK(p.pba_valid == false, "pba_valid should be false when PBA disabled");
    }
    PASS("pba_disabled_by_default");
}

TEST(pba_enabled_computes_path_delays) {
    Netlist nl = build_deep_pipeline();
    StaEngine sta(nl);
    sta.enable_pba();
    auto result = sta.analyze(10.0);
    CHECK(result.pba_enabled == true, "PBA should be enabled after enable_pba()");
    bool has_valid = false;
    for (auto& p : result.critical_paths) {
        if (p.pba_valid) {
            has_valid = true;
            CHECK(p.pba_delay > 0, "PBA delay should be > 0 for valid path");
        }
    }
    CHECK(has_valid, "At least one path should have pba_valid == true");
    PASS("pba_enabled_computes_path_delays");
}

TEST(pba_with_aocv) {
    Netlist nl = build_deep_pipeline();
    StaEngine sta(nl);
    sta.enable_aocv(0.10, 0.10);
    sta.enable_pba();
    auto result = sta.analyze(10.0);
    CHECK(result.pba_enabled == true, "PBA should be enabled");
    for (auto& p : result.critical_paths) {
        if (p.pba_valid) {
            CHECK(p.pba_delay > 0, "PBA delay should be positive");
            // pba_slack is set (could be positive or negative)
            CHECK(std::isfinite(p.pba_slack), "PBA slack should be finite");
        }
    }
    PASS("pba_with_aocv");
}

TEST(pba_result_fields) {
    Netlist nl = build_seq_circuit();
    StaEngine sta(nl);
    sta.enable_pba();
    auto result = sta.analyze(10.0);
    CHECK(result.pba_enabled == true, "PBA should be enabled");
    CHECK(std::isfinite(result.pba_wns), "pba_wns should be finite");
    CHECK(std::isfinite(result.pba_tns), "pba_tns should be finite");
    CHECK(result.pba_violations >= 0, "pba_violations must be non-negative");
    PASS("pba_result_fields");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Crosstalk Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(xtalk_disabled_by_default) {
    Netlist nl = build_seq_circuit();
    StaEngine sta(nl);
    auto result = sta.analyze(10.0);
    CHECK(result.crosstalk_enabled == false,
          "Crosstalk should be disabled by default");
    PASS("xtalk_disabled_by_default");
}

TEST(xtalk_enabled_with_config) {
    Netlist nl = build_seq_circuit();
    PhysicalDesign pd = build_xtalk_design();
    StaEngine sta(nl, nullptr, &pd);
    sta.enable_crosstalk(0.00015, 1.5);
    auto result = sta.analyze(10.0);
    CHECK(result.crosstalk_enabled == true,
          "Crosstalk should be enabled after enable_crosstalk()");
    PASS("xtalk_enabled_with_config");
}

TEST(xtalk_delta_on_close_wires) {
    Netlist nl = build_seq_circuit();
    PhysicalDesign pd = build_xtalk_design();
    StaEngine sta(nl, nullptr, &pd);
    sta.enable_crosstalk(0.00015, 1.5);
    auto result = sta.analyze(10.0);
    CHECK(result.max_crosstalk_delta >= 0,
          "max_crosstalk_delta must be non-negative");
    PASS("xtalk_delta_on_close_wires");
}

TEST(xtalk_config_fields) {
    CrosstalkConfig cfg;
    cfg.enabled = true;
    cfg.coupling_cap_per_um = 0.0002;
    cfg.miller_factor = 2.0;
    cfg.aggressor_slew = 0.03;
    cfg.min_spacing_um = 0.10;
    cfg.max_coupling_distance_um = 1.5;
    CHECK(cfg.enabled == true, "enabled field");
    CHECK(std::abs(cfg.coupling_cap_per_um - 0.0002) < 1e-12, "coupling_cap_per_um");
    CHECK(std::abs(cfg.miller_factor - 2.0) < 1e-12, "miller_factor");
    CHECK(std::abs(cfg.aggressor_slew - 0.03) < 1e-12, "aggressor_slew");
    CHECK(std::abs(cfg.min_spacing_um - 0.10) < 1e-12, "min_spacing_um");
    CHECK(std::abs(cfg.max_coupling_distance_um - 1.5) < 1e-12, "max_coupling_distance_um");
    PASS("xtalk_config_fields");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Integration Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(all_industrial_features_together) {
    Netlist nl = build_two_stage_pipeline();
    PhysicalDesign pd = build_xtalk_design();
    StaEngine sta(nl, nullptr, &pd);
    sta.enable_ocv(1.15, 0.85);
    auto& ffs = nl.flip_flops();
    for (GateId ff : ffs)
        sta.set_clock_insertion(ff, 0.5);
    sta.enable_cppr();
    sta.enable_pba();
    sta.enable_crosstalk(0.00015, 1.5);
    auto result = sta.analyze(10.0);
    CHECK(result.cppr_enabled == true, "CPPR must be enabled");
    CHECK(result.pba_enabled == true, "PBA must be enabled");
    CHECK(result.crosstalk_enabled == true, "Crosstalk must be enabled");
    CHECK(result.critical_paths.size() > 0, "Should have critical paths");
    PASS("all_industrial_features_together");
}

TEST(pocv_with_cppr_and_pba) {
    Netlist nl = build_deep_pipeline();
    StaEngine sta(nl);
    sta.enable_pocv(3.0, 0.05);
    sta.enable_cppr();
    sta.enable_pba();
    auto& ffs = nl.flip_flops();
    for (GateId ff : ffs)
        sta.set_clock_insertion(ff, 0.3);
    auto result = sta.analyze(10.0);
    CHECK(result.ocv_mode == OcvMode::POCV, "Should be POCV mode");
    CHECK(result.cppr_enabled == true, "CPPR must be enabled");
    CHECK(result.pba_enabled == true, "PBA must be enabled");
    for (auto& p : result.critical_paths) {
        CHECK(p.path_sigma >= 0, "path_sigma non-negative");
        CHECK(p.cppr_credit >= 0, "cppr_credit non-negative");
    }
    PASS("pocv_with_cppr_and_pba");
}

// ═══════════════════════════════════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "\n"
        "╔═══════════════════════════════════════════════════════╗\n"
        "║  SiliconForge Phase 20 — STA Industrial Features Test  ║\n"
        "╚═══════════════════════════════════════════════════════╝\n\n";

    std::cout << "── CPPR ──\n";
    RUN(cppr_disabled_by_default);
    RUN(cppr_with_ocv);
    RUN(cppr_with_aocv);
    RUN(cppr_reduces_pessimism);

    std::cout << "\n── POCV ──\n";
    RUN(pocv_table_default_sigma);
    RUN(pocv_table_per_cell_sigma);
    RUN(pocv_mode_enabled);
    RUN(pocv_with_pba_path_sigma);

    std::cout << "\n── PBA ──\n";
    RUN(pba_disabled_by_default);
    RUN(pba_enabled_computes_path_delays);
    RUN(pba_with_aocv);
    RUN(pba_result_fields);

    std::cout << "\n── Crosstalk ──\n";
    RUN(xtalk_disabled_by_default);
    RUN(xtalk_enabled_with_config);
    RUN(xtalk_delta_on_close_wires);
    RUN(xtalk_config_fields);

    std::cout << "\n── Integration ──\n";
    RUN(all_industrial_features_together);
    RUN(pocv_with_cppr_and_pba);

    std::cout << "\n════════════════════════════════════════\n";
    std::cout << "Phase 20 Results: " << passed << " passed, "
              << failed << " failed\n";
    std::cout << "════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
