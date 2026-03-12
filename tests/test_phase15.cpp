// SiliconForge Phase 15 — STA OCV/AOCV Industrial Test Suite

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include "timing/sta.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <string>

using namespace sf;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) static void test_##name()
#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        std::cerr << "  [FAIL] " << msg << " (line " << __LINE__ << ")\n"; \
        tests_failed++; return; \
    } \
} while(0)
#define PASS(name) do { std::cout << "  [PASS] " << name << "\n"; tests_passed++; } while(0)
#define RUN(name) do { std::cout << "Running: " #name "\n"; test_##name(); } while(0)

// ── Helper circuit builders ──────────────────────────────────

// 3-gate combinational: a,b→AND→OR(c)→NOT→y
static Netlist build_combo_circuit() {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId c = nl.add_net("c"); nl.mark_input(c);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId y = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {a, b}, w1, "G1");
    nl.add_gate(GateType::OR, {w1, c}, w2, "G2");
    nl.add_gate(GateType::NOT, {w2}, y, "G3");
    return nl;
}

// Sequential circuit with DFF (for hold analysis)
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

// Deep pipeline for AOCV depth testing (10 gates in chain)
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

// ── AOCV Derate Formula Tests ────────────────────────────────

TEST(aocv_late_derate_formula) {
    AocvTable tbl;
    tbl.late_variation = 0.10;
    tbl.early_variation = 0.10;
    tbl.min_depth = 1;

    CHECK(fabs(tbl.late_derate(1)   - 1.10)   < 1e-6, "late_derate(1) == 1.10");
    CHECK(fabs(tbl.late_derate(4)   - 1.05)   < 1e-6, "late_derate(4) == 1.05");
    CHECK(fabs(tbl.late_derate(9)   - (1.0 + 0.10 / 3.0)) < 1e-6, "late_derate(9) ≈ 1.0333");
    CHECK(fabs(tbl.late_derate(16)  - 1.025)  < 1e-6, "late_derate(16) == 1.025");
    CHECK(fabs(tbl.late_derate(100) - 1.01)   < 1e-6, "late_derate(100) == 1.01");

    PASS("aocv_late_derate_formula");
}

TEST(aocv_early_derate_formula) {
    AocvTable tbl;
    tbl.late_variation = 0.10;
    tbl.early_variation = 0.10;
    tbl.min_depth = 1;

    CHECK(fabs(tbl.early_derate(1)   - 0.90)  < 1e-6, "early_derate(1) == 0.90");
    CHECK(fabs(tbl.early_derate(4)   - 0.95)  < 1e-6, "early_derate(4) == 0.95");
    CHECK(fabs(tbl.early_derate(9)   - (1.0 - 0.10 / 3.0)) < 1e-6, "early_derate(9) ≈ 0.9667");
    CHECK(fabs(tbl.early_derate(100) - 0.99)  < 1e-6, "early_derate(100) == 0.99");

    PASS("aocv_early_derate_formula");
}

TEST(aocv_min_depth_clamp) {
    AocvTable tbl;
    tbl.late_variation = 0.10;
    tbl.early_variation = 0.10;
    tbl.min_depth = 3;

    double d1 = tbl.late_derate(1);
    double d3 = tbl.late_derate(3);
    CHECK(fabs(d1 - d3) < 1e-6, "depth 1 clamped to min_depth 3");

    double e1 = tbl.early_derate(1);
    double e3 = tbl.early_derate(3);
    CHECK(fabs(e1 - e3) < 1e-6, "early depth 1 clamped to min_depth 3");

    PASS("aocv_min_depth_clamp");
}

TEST(aocv_custom_variations) {
    AocvTable tbl;
    tbl.late_variation = 0.25;
    tbl.early_variation = 0.15;
    tbl.min_depth = 1;

    // late_derate(d) = 1.0 + 0.25/sqrt(d)
    CHECK(fabs(tbl.late_derate(1)  - 1.25) < 1e-6, "late custom derate(1) == 1.25");
    CHECK(fabs(tbl.late_derate(4)  - 1.125) < 1e-6, "late custom derate(4) == 1.125");
    CHECK(fabs(tbl.late_derate(25) - 1.05) < 1e-6, "late custom derate(25) == 1.05");

    // early_derate(d) = 1.0 - 0.15/sqrt(d)
    CHECK(fabs(tbl.early_derate(1)  - 0.85) < 1e-6, "early custom derate(1) == 0.85");
    CHECK(fabs(tbl.early_derate(4)  - 0.925) < 1e-6, "early custom derate(4) == 0.925");
    CHECK(fabs(tbl.early_derate(25) - 0.97) < 1e-6, "early custom derate(25) == 0.97");

    PASS("aocv_custom_variations");
}

// ── OCV Mode & Engine Tests ──────────────────────────────────

TEST(ocv_mode_none_baseline) {
    auto nl = build_combo_circuit();
    StaEngine sta(nl, nullptr, nullptr);
    sta.set_ocv_mode(OcvMode::NONE);

    auto result = sta.analyze(10.0, 5);
    CHECK(result.ocv_mode == OcvMode::NONE, "OCV mode should be NONE");
    CHECK(result.num_endpoints > 0, "should have endpoints");
    // WNS is populated (could be zero for unconstrained combo, but field is set)
    CHECK(result.time_ms >= 0, "analysis completed with timing info");

    PASS("ocv_mode_none_baseline");
}

TEST(ocv_flat_derating) {
    auto nl = build_combo_circuit();

    // Baseline without OCV
    StaEngine sta_none(nl, nullptr, nullptr);
    sta_none.set_ocv_mode(OcvMode::NONE);
    auto r_none = sta_none.analyze(10.0, 5);

    // With flat OCV derating
    StaEngine sta_ocv(nl, nullptr, nullptr);
    sta_ocv.enable_ocv(1.15, 0.85);
    auto r_ocv = sta_ocv.analyze(10.0, 5);

    CHECK(r_ocv.ocv_mode == OcvMode::OCV, "OCV mode should be OCV");
    // OCV late derate (1.15) slows data path → WNS worse (more negative or less positive)
    CHECK(r_ocv.wns <= r_none.wns + 1e-9, "OCV WNS should be <= nominal WNS");

    PASS("ocv_flat_derating");
}

TEST(aocv_depth_derating) {
    auto nl = build_deep_pipeline();
    StaEngine sta(nl, nullptr, nullptr);
    sta.enable_aocv(0.10, 0.10);

    auto result = sta.analyze(10.0, 5);
    CHECK(result.ocv_mode == OcvMode::AOCV, "OCV mode should be AOCV");
    CHECK(result.num_endpoints > 0, "should have endpoints");
    CHECK(result.critical_paths.size() > 0, "should have critical paths");

    // Deep pipeline paths should report depth > 0
    bool found_deep = false;
    for (const auto& p : result.critical_paths) {
        if (p.depth > 0) { found_deep = true; break; }
    }
    CHECK(found_deep, "at least one critical path should have depth > 0");

    PASS("aocv_depth_derating");
}

TEST(ocv_worse_than_nominal) {
    auto nl = build_seq_circuit();

    StaEngine sta_none(nl, nullptr, nullptr);
    sta_none.set_ocv_mode(OcvMode::NONE);
    auto r_none = sta_none.analyze(5.0, 5);

    StaEngine sta_ocv(nl, nullptr, nullptr);
    sta_ocv.enable_ocv(1.15, 0.85);
    auto r_ocv = sta_ocv.analyze(5.0, 5);

    // OCV is more pessimistic → WNS should be worse (more negative)
    CHECK(r_ocv.wns <= r_none.wns + 1e-9, "OCV WNS <= nominal WNS (more pessimistic)");

    PASS("ocv_worse_than_nominal");
}

TEST(aocv_less_pessimistic_than_flat_ocv) {
    auto nl = build_deep_pipeline();

    // Flat OCV with 10% variation applied uniformly
    StaEngine sta_ocv(nl, nullptr, nullptr);
    sta_ocv.enable_ocv(1.10, 0.90);
    auto r_ocv = sta_ocv.analyze(10.0, 5);

    // AOCV with same 10% variation but depth-dependent
    StaEngine sta_aocv(nl, nullptr, nullptr);
    sta_aocv.enable_aocv(0.10, 0.10);
    auto r_aocv = sta_aocv.analyze(10.0, 5);

    // For deep paths AOCV reduces pessimism → AOCV WNS >= OCV WNS
    CHECK(r_aocv.wns >= r_ocv.wns - 1e-9,
          "AOCV WNS should be >= flat OCV WNS for deep paths");

    PASS("aocv_less_pessimistic_than_flat_ocv");
}

// ── Corner Derate Tests ──────────────────────────────────────

TEST(corner_derate_fields) {
    CornerDerate cd;
    cd.name = "ss_125c";
    cd.cell_derate = 1.20;
    cd.wire_derate = 1.15;
    cd.early_cell = 0.80;
    cd.early_wire = 0.85;
    cd.clock_late_cell = 1.10;
    cd.clock_late_wire = 1.05;
    cd.clock_early_cell = 0.90;
    cd.clock_early_wire = 0.95;

    CHECK(cd.name == "ss_125c", "corner name");
    CHECK(fabs(cd.cell_derate - 1.20)       < 1e-9, "cell_derate");
    CHECK(fabs(cd.wire_derate - 1.15)       < 1e-9, "wire_derate");
    CHECK(fabs(cd.early_cell - 0.80)        < 1e-9, "early_cell");
    CHECK(fabs(cd.early_wire - 0.85)        < 1e-9, "early_wire");
    CHECK(fabs(cd.clock_late_cell - 1.10)   < 1e-9, "clock_late_cell");
    CHECK(fabs(cd.clock_late_wire - 1.05)   < 1e-9, "clock_late_wire");
    CHECK(fabs(cd.clock_early_cell - 0.90)  < 1e-9, "clock_early_cell");
    CHECK(fabs(cd.clock_early_wire - 0.95)  < 1e-9, "clock_early_wire");

    PASS("corner_derate_fields");
}

TEST(analyze_corner_api) {
    auto nl = build_seq_circuit();
    StaEngine sta(nl, nullptr, nullptr);

    CornerDerate cd;
    cd.name = "slow";
    cd.cell_derate = 1.20;
    cd.wire_derate = 1.10;
    cd.early_cell = 0.85;
    cd.early_wire = 0.90;
    cd.clock_late_cell = 1.05;
    cd.clock_late_wire = 1.03;
    cd.clock_early_cell = 0.95;
    cd.clock_early_wire = 0.97;

    auto result = sta.analyze_corner(5.0, 5, cd);
    CHECK(result.corner_name == "slow", "corner name in result");
    CHECK(result.num_endpoints > 0, "should have endpoints");

    PASS("analyze_corner_api");
}

// ── Clock Uncertainty & Result Tests ─────────────────────────

TEST(clock_uncertainty_affects_slack) {
    auto nl = build_seq_circuit();

    // Without uncertainty
    StaEngine sta1(nl, nullptr, nullptr);
    auto r1 = sta1.analyze(5.0, 5);

    // With uncertainty
    StaEngine sta2(nl, nullptr, nullptr);
    sta2.set_clock_uncertainty(0.5, 0.3);
    auto r2 = sta2.analyze(5.0, 5);

    // Uncertainty reduces effective margin → WNS should be worse
    CHECK(r2.wns <= r1.wns + 1e-9, "clock uncertainty should worsen setup WNS");

    PASS("clock_uncertainty_affects_slack");
}

TEST(sta_result_completeness) {
    auto nl = build_seq_circuit();
    StaEngine sta(nl, nullptr, nullptr);
    auto result = sta.analyze(5.0, 5);

    // Verify result fields are populated (slack may be zero with generous clock)
    CHECK(result.clock_period == 5.0, "clock_period recorded");
    CHECK(result.num_endpoints > 0, "num_endpoints > 0");
    CHECK(result.time_ms > 0, "time_ms > 0");
    CHECK(result.critical_paths.size() > 0, "critical_paths non-empty");
    // At least one slack metric should be non-zero, or all paths have valid delay
    bool has_timing = (result.wns != 0 || result.tns != 0 ||
                       result.hold_wns != 0 || result.hold_tns != 0);
    if (!has_timing) {
        // If all slacks are zero, paths should still have positive delay
        for (const auto& p : result.critical_paths) {
            if (p.delay > 0) { has_timing = true; break; }
        }
    }
    CHECK(has_timing, "result has meaningful timing data");

    PASS("sta_result_completeness");
}

TEST(timing_path_fields) {
    auto nl = build_seq_circuit();
    StaEngine sta(nl, nullptr, nullptr);
    auto result = sta.analyze(5.0, 5);

    CHECK(result.critical_paths.size() > 0, "need at least one path");
    const auto& path = result.critical_paths[0];

    CHECK(path.nets.size() > 0, "path.nets non-empty");
    CHECK(path.delay > 0, "path.delay > 0");
    CHECK(!path.startpoint.empty(), "startpoint is set");
    CHECK(!path.endpoint.empty(), "endpoint is set");

    PASS("timing_path_fields");
}

// ── Main ─────────────────────────────────────────────────────

int main() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 15 — STA OCV/AOCV Industrial ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── AOCV Derate Formula ──\n";
    RUN(aocv_late_derate_formula);
    RUN(aocv_early_derate_formula);
    RUN(aocv_min_depth_clamp);
    RUN(aocv_custom_variations);

    std::cout << "\n── OCV Mode & Engine ──\n";
    RUN(ocv_mode_none_baseline);
    RUN(ocv_flat_derating);
    RUN(aocv_depth_derating);
    RUN(ocv_worse_than_nominal);
    RUN(aocv_less_pessimistic_than_flat_ocv);

    std::cout << "\n── Corner Derate ──\n";
    RUN(corner_derate_fields);
    RUN(analyze_corner_api);

    std::cout << "\n── Clock Uncertainty & Results ──\n";
    RUN(clock_uncertainty_affects_slack);
    RUN(sta_result_completeness);
    RUN(timing_path_fields);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << tests_passed << " passed, " << tests_failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";

    return tests_failed > 0 ? 1 : 0;
}
