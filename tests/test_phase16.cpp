// SiliconForge Phase 16 — Industrial-Grade MCMM Test Suite
// Tests Multi-Corner Multi-Mode analysis: PVT corners, functional modes,
// OCV/AOCV derating, scenario cross-product, worst-case aggregation.

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include "timing/sta.hpp"
#include "timing/mcmm.hpp"
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
// Helper: sequential circuit with feedback (AND → NOT → DFF)
// ---------------------------------------------------------------------------
static Netlist build_seq_circuit() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId d   = nl.add_net("d");   nl.mark_input(d);
    NetId q   = nl.add_net("q");   nl.mark_output(q);
    NetId w1  = nl.add_net("w1");
    NetId w2  = nl.add_net("w2");
    nl.add_gate(GateType::AND, {d, q}, w1, "G1");
    nl.add_gate(GateType::NOT, {w1},   w2, "G2");
    nl.add_dff(w2, clk, q, -1, "FF1");
    return nl;
}

// ===================================================================
// 1. Single corner + single mode → 1 scenario
// ===================================================================
TEST(mcmm_single_corner_single_mode) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);

    PvtCorner c;
    c.name = "tt_0p90v_25c";
    c.voltage = 0.90; c.temperature_c = 25;
    c.process = PvtCorner::TYPICAL;
    mcmm.add_corner(c);

    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto r = mcmm.analyze();
    CHECK(r.corners == 1,   "expected 1 corner");
    CHECK(r.modes == 1,     "expected 1 mode");
    CHECK(r.scenarios == 1, "expected 1 scenario");
    CHECK(r.details.size() == 1, "expected 1 detail entry");
    PASS("mcmm_single_corner_single_mode");
}

// ===================================================================
// 2. load_default_corners → 3 corners
// ===================================================================
TEST(mcmm_default_corners) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();

    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto r = mcmm.analyze();
    CHECK(r.corners == 3,   "expected 3 default corners");
    CHECK(r.scenarios == 3, "expected 3 scenarios (3×1)");
    PASS("mcmm_default_corners");
}

// ===================================================================
// 3. load_foundry_corners → 7 corners
// ===================================================================
TEST(mcmm_foundry_corners) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);
    mcmm.load_foundry_corners();

    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto r = mcmm.analyze();
    CHECK(r.corners == 7,   "expected 7 foundry corners");
    CHECK(r.scenarios == 7, "expected 7 scenarios (7×1)");
    PASS("mcmm_foundry_corners");
}

// ===================================================================
// 4. Cross-product: 3 corners × 2 modes = 6 scenarios
// ===================================================================
TEST(mcmm_scenario_count_cross_product) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners(); // 3 corners

    FunctionalMode m1; m1.name = "func"; m1.clock_freq_mhz = 500;
    FunctionalMode m2; m2.name = "test"; m2.clock_freq_mhz = 200;
    mcmm.add_mode(m1);
    mcmm.add_mode(m2);

    auto r = mcmm.analyze();
    CHECK(r.scenarios == 6, "expected 6 scenarios (3×2)");
    CHECK(r.details.size() == 6, "expected 6 detail entries");
    PASS("mcmm_scenario_count_cross_product");
}

// ===================================================================
// 5. Custom corner fields survive round-trip
// ===================================================================
TEST(mcmm_custom_corner_fields) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);

    PvtCorner c;
    c.name = "ss_0p75v_125c";
    c.voltage = 0.75; c.temperature_c = 125;
    c.process = PvtCorner::SLOW;
    c.power_scale = 1.5;
    c.derate.cell_derate = 1.3;
    mcmm.add_corner(c);

    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto r = mcmm.analyze();
    CHECK(r.details.size() == 1, "expected 1 detail");
    auto& sc = r.details[0];
    CHECK(sc.corner.name == "ss_0p75v_125c",            "corner name mismatch");
    CHECK(std::fabs(sc.corner.voltage - 0.75) < 1e-6,   "voltage mismatch");
    CHECK(sc.corner.temperature_c == 125,                "temperature mismatch");
    CHECK(sc.corner.process == PvtCorner::SLOW,          "process mismatch");
    CHECK(std::fabs(sc.corner.power_scale - 1.5) < 1e-6, "power_scale mismatch");
    PASS("mcmm_custom_corner_fields");
}

// ===================================================================
// 6. Per-corner OCV: OCV corner should have worse (smaller) WNS
// ===================================================================
TEST(mcmm_per_corner_ocv) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);

    PvtCorner c_none;
    c_none.name = "tt_none";
    c_none.voltage = 0.90; c_none.temperature_c = 25;
    c_none.process = PvtCorner::TYPICAL;
    c_none.ocv_mode = OcvMode::NONE;
    mcmm.add_corner(c_none);

    PvtCorner c_ocv;
    c_ocv.name = "tt_ocv";
    c_ocv.voltage = 0.90; c_ocv.temperature_c = 25;
    c_ocv.process = PvtCorner::TYPICAL;
    c_ocv.ocv_mode = OcvMode::OCV;
    mcmm.add_corner(c_ocv);

    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto r = mcmm.analyze();
    CHECK(r.scenarios == 2, "expected 2 scenarios");

    double wns_none = 0, wns_ocv = 0;
    for (auto& s : r.details) {
        if (s.corner.name == "tt_none") wns_none = s.sta.wns;
        if (s.corner.name == "tt_ocv")  wns_ocv  = s.sta.wns;
    }
    CHECK(wns_ocv <= wns_none, "OCV corner should have worse (<=) setup WNS");
    PASS("mcmm_per_corner_ocv");
}

// ===================================================================
// 7. Per-corner AOCV: analysis completes successfully
// ===================================================================
TEST(mcmm_per_corner_aocv) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);

    PvtCorner c;
    c.name = "tt_aocv";
    c.voltage = 0.90; c.temperature_c = 25;
    c.process = PvtCorner::TYPICAL;
    c.ocv_mode = OcvMode::AOCV;
    c.aocv_table.late_variation  = 0.12;
    c.aocv_table.early_variation = 0.10;
    c.aocv_table.min_depth = 1;
    mcmm.add_corner(c);

    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto r = mcmm.analyze();
    CHECK(r.scenarios == 1,      "expected 1 scenario");
    CHECK(r.details.size() == 1, "expected 1 detail");
    CHECK(r.details[0].scenario_name.find("tt_aocv") != std::string::npos,
          "scenario name should contain corner name");
    PASS("mcmm_per_corner_aocv");
}

// ===================================================================
// 8. load_default_modes → 3 modes
// ===================================================================
TEST(mcmm_functional_modes) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);

    PvtCorner c;
    c.name = "tt"; c.voltage = 0.90; c.temperature_c = 25;
    c.process = PvtCorner::TYPICAL;
    mcmm.add_corner(c);
    mcmm.load_default_modes(500);

    auto r = mcmm.analyze();
    CHECK(r.modes == 3, "expected 3 default modes");
    PASS("mcmm_functional_modes");
}

// ===================================================================
// 9. Worst-case aggregation: worst_setup_wns == min WNS across details
// ===================================================================
TEST(mcmm_worst_case_aggregation) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);
    mcmm.load_foundry_corners();
    mcmm.load_default_modes(500);

    auto r = mcmm.analyze();
    CHECK(r.details.size() > 0, "should have scenario details");

    double manual_worst = 1e18;
    for (auto& s : r.details) {
        if (s.mode.clock_freq_mhz > 0 || s.mode.clock_period_ns > 0) {
            if (s.sta.wns < manual_worst)
                manual_worst = s.sta.wns;
        }
    }
    CHECK(std::fabs(r.worst_setup_wns - manual_worst) < 1e-6,
          "worst_setup_wns must equal min WNS across all timed scenarios");
    PASS("mcmm_worst_case_aggregation");
}

// ===================================================================
// 10. Slow corner has worse setup WNS than fast corner
// ===================================================================
TEST(mcmm_slow_corner_worst_timing) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);

    PvtCorner fast;
    fast.name = "ff_1p98v_m40c";
    fast.voltage = 1.98; fast.temperature_c = -40;
    fast.process = PvtCorner::FAST;
    fast.delay_scale = 0.6;
    fast.derate.cell_derate = 0.8;
    fast.derate.wire_derate = 0.8;
    fast.derate.early_cell  = 0.7;
    fast.derate.early_wire  = 0.7;
    mcmm.add_corner(fast);

    PvtCorner slow;
    slow.name = "ss_1p62v_125c";
    slow.voltage = 1.62; slow.temperature_c = 125;
    slow.process = PvtCorner::SLOW;
    slow.delay_scale = 1.6;
    slow.derate.cell_derate = 1.3;
    slow.derate.wire_derate = 1.3;
    slow.derate.early_cell  = 1.2;
    slow.derate.early_wire  = 1.2;
    mcmm.add_corner(slow);

    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto r = mcmm.analyze();
    CHECK(r.scenarios == 2, "expected 2 scenarios");

    double wns_fast = 0, wns_slow = 0;
    for (auto& s : r.details) {
        if (s.corner.name == "ff_1p98v_m40c") wns_fast = s.sta.wns;
        if (s.corner.name == "ss_1p62v_125c") wns_slow = s.sta.wns;
    }
    CHECK(wns_slow <= wns_fast,
          "slow corner should have worse (<=) setup WNS than fast corner");
    PASS("mcmm_slow_corner_worst_timing");
}

// ===================================================================
// 11. Higher setup uncertainty → worse WNS
// ===================================================================
TEST(mcmm_per_mode_uncertainty) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);

    PvtCorner c;
    c.name = "tt"; c.voltage = 0.90; c.temperature_c = 25;
    c.process = PvtCorner::TYPICAL;
    mcmm.add_corner(c);

    FunctionalMode m_low;
    m_low.name = "low_unc";  m_low.clock_freq_mhz = 500;
    m_low.setup_uncertainty = 0;
    mcmm.add_mode(m_low);

    FunctionalMode m_high;
    m_high.name = "high_unc"; m_high.clock_freq_mhz = 500;
    m_high.setup_uncertainty = 0.5;
    mcmm.add_mode(m_high);

    auto r = mcmm.analyze();
    CHECK(r.scenarios == 2, "expected 2 scenarios");

    double wns_low = 0, wns_high = 0;
    for (auto& s : r.details) {
        if (s.mode.name == "low_unc")  wns_low  = s.sta.wns;
        if (s.mode.name == "high_unc") wns_high = s.sta.wns;
    }
    CHECK(wns_high <= wns_low,
          "high uncertainty mode should have worse (<=) WNS");
    PASS("mcmm_per_mode_uncertainty");
}

// ===================================================================
// 12. Result completeness after full MCMM run
// ===================================================================
TEST(mcmm_result_completeness) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);
    mcmm.load_foundry_corners();
    mcmm.load_default_modes(500);

    auto r = mcmm.analyze();
    CHECK(r.time_ms > 0,                      "time_ms should be > 0");
    CHECK(!r.worst_setup_scenario.empty(),     "worst_setup_scenario should not be empty");
    CHECK(r.details.size() > 0,                "details should not be empty");
    for (auto& d : r.details) {
        CHECK(!d.scenario_name.empty(), "every detail must have a scenario_name");
    }
    PASS("mcmm_result_completeness");
}

// ===================================================================
// 13. Clear corners and rebuild — only new corners appear
// ===================================================================
TEST(mcmm_clear_and_rebuild) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);

    // Add initial corners
    mcmm.load_default_corners(); // 3 corners

    // Clear and add a single new corner
    mcmm.clear_corners();
    PvtCorner c;
    c.name = "custom_only";
    c.voltage = 0.85; c.temperature_c = 50;
    c.process = PvtCorner::TYPICAL;
    mcmm.add_corner(c);

    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto r = mcmm.analyze();
    CHECK(r.corners == 1,   "after clear+add, should have exactly 1 corner");
    CHECK(r.scenarios == 1, "after clear+add, should have exactly 1 scenario");
    CHECK(r.details[0].corner.name == "custom_only",
          "only the new corner should appear");
    PASS("mcmm_clear_and_rebuild");
}

// ===================================================================
// Main
// ===================================================================
int main() {
    std::cout << "\n"
        "╔══════════════════════════════════════════════════╗\n"
        "║  SiliconForge Phase 16 — MCMM Industrial Tests   ║\n"
        "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Corner / Mode Basics ──\n";
    RUN(mcmm_single_corner_single_mode);
    RUN(mcmm_default_corners);
    RUN(mcmm_foundry_corners);
    RUN(mcmm_scenario_count_cross_product);

    std::cout << "\n── Corner Configuration ──\n";
    RUN(mcmm_custom_corner_fields);
    RUN(mcmm_per_corner_ocv);
    RUN(mcmm_per_corner_aocv);

    std::cout << "\n── Mode Configuration ──\n";
    RUN(mcmm_functional_modes);
    RUN(mcmm_per_mode_uncertainty);

    std::cout << "\n── Aggregation & Completeness ──\n";
    RUN(mcmm_worst_case_aggregation);
    RUN(mcmm_slow_corner_worst_timing);
    RUN(mcmm_result_completeness);
    RUN(mcmm_clear_and_rebuild);

    std::cout << "\n════════════════════════════════════════\n";
    std::cout << "Phase 16 Results: " << passed << " passed, "
              << failed << " failed\n";
    std::cout << "════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
