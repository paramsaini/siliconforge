// test_phase24.cpp — MCMM Industrial Features Tests
// Tests: SignoffType, McmmConfig, CPPR/POCV per corner, scenario pruning,
//        active scenario sets, sensitivity analysis, signoff summaries,
//        scenario constraints, dominated scenario detection, backward compat.
// Reference: Cadence Tempus / Synopsys PrimeTime MCMM signoff methodology

#include "timing/mcmm.hpp"
#include "core/netlist.hpp"
#include <iostream>
#include <cmath>
#include <string>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// Helper: build a simple timing-relevant netlist (IN → AND → DFF → OUT)
static Netlist make_test_netlist() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId d = nl.add_net("d"); nl.mark_input(d);
    NetId w1 = nl.add_net("w1");
    NetId q = nl.add_net("q"); nl.mark_output(q);
    nl.add_gate(GateType::AND, {d, d}, w1, "G_and");
    nl.add_dff(w1, clk, q, -1, "FF1");
    return nl;
}

// Helper: create CornerDerate with named fields (can't brace-init due to std::string)
static CornerDerate make_derate(double cell, double wire, double ec, double ew) {
    CornerDerate d;
    d.cell_derate = cell; d.wire_derate = wire;
    d.early_cell = ec; d.early_wire = ew;
    return d;
}

// ===========================
// Test 1: SignoffType enum + string conversion
// ===========================
TEST(signoff_type_enum) {
    CHECK(std::string(signoff_type_str(SignoffType::SETUP)) == "setup", "SETUP string");
    CHECK(std::string(signoff_type_str(SignoffType::HOLD)) == "hold", "HOLD string");
    CHECK(std::string(signoff_type_str(SignoffType::LEAKAGE)) == "leakage", "LEAKAGE string");
    CHECK(std::string(signoff_type_str(SignoffType::DYNAMIC_POWER)) == "dynamic_power", "DYN string");
    CHECK(std::string(signoff_type_str(SignoffType::TRANSITION)) == "transition", "TRANS string");
    CHECK(std::string(signoff_type_str(SignoffType::ALL)) == "all", "ALL string");
    PASS("signoff_type_enum");
}

// ===========================
// Test 2: PvtCorner new fields default values
// ===========================
TEST(pvt_corner_defaults) {
    PvtCorner c;
    CHECK(c.ocv_mode == OcvMode::NONE, "default OCV mode = NONE");
    CHECK(!c.cppr_enabled, "CPPR disabled by default");
    CHECK(c.signoff == SignoffType::ALL, "default signoff = ALL");
    CHECK(c.power_scale == 1.0, "default power_scale = 1.0");
    CHECK(c.delay_scale == 1.0, "default delay_scale = 1.0");
    PASS("pvt_corner_defaults");
}

// ===========================
// Test 3: FunctionalMode new fields
// ===========================
TEST(functional_mode_defaults) {
    FunctionalMode m;
    CHECK(m.signoff == SignoffType::ALL, "default mode signoff = ALL");
    CHECK(m.clock_period_ns == 0, "default period = 0");
    CHECK(m.setup_uncertainty == 0, "default setup uncertainty");
    CHECK(m.hold_uncertainty == 0, "default hold uncertainty");
    PASS("functional_mode_defaults");
}

// ===========================
// Test 4: McmmConfig defaults
// ===========================
TEST(mcmm_config_defaults) {
    McmmConfig cfg;
    CHECK(!cfg.enable_scenario_pruning, "pruning off by default");
    CHECK(cfg.enable_pocv_per_corner, "POCV per corner on");
    CHECK(cfg.enable_cppr_per_corner, "CPPR per corner on");
    CHECK(!cfg.enable_sensitivity, "sensitivity off by default");
    CHECK(cfg.pruning_slack_margin == 0.05, "margin = 0.05ns");
    CHECK(cfg.max_paths_per_scenario == 10, "max paths = 10");
    PASS("mcmm_config_defaults");
}

// ===========================
// Test 5: Corner with CPPR enabled — tracked in result
// ===========================
TEST(cppr_per_corner) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);

    PvtCorner c;
    c.name = "ss_cppr";
    c.process = PvtCorner::SLOW;
    c.derate = make_derate(1.12, 0.88, 1.12, 0.88);
    c.cppr_enabled = true;
    mcmm.add_corner(c);

    FunctionalMode m;
    m.name = "func";
    m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    McmmConfig cfg;
    cfg.enable_cppr_per_corner = true;
    mcmm.set_config(cfg);

    auto r = mcmm.analyze();
    CHECK(r.cppr_scenarios == 1, "1 CPPR scenario");
    CHECK(r.scenarios == 1, "1 total scenario");
    PASS("cppr_per_corner");
}

// ===========================
// Test 6: Corner with POCV enabled — tracked in result
// ===========================
TEST(pocv_per_corner) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);

    PvtCorner c;
    c.name = "tt_pocv";
    c.process = PvtCorner::TYPICAL;
    c.derate = make_derate(1.0, 1.0, 1.0, 1.0);
    c.ocv_mode = OcvMode::POCV;
    PocvTable pt;
    pt.default_sigma_pct = 0.05;
    pt.n_sigma = 3.0;
    c.pocv_table = pt;
    mcmm.add_corner(c);

    FunctionalMode m;
    m.name = "func";
    m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    McmmConfig cfg;
    cfg.enable_pocv_per_corner = true;
    mcmm.set_config(cfg);

    auto r = mcmm.analyze();
    CHECK(r.pocv_scenarios == 1, "1 POCV scenario");
    PASS("pocv_per_corner");
}

// ===========================
// Test 7: Active scenario set — skip inactive scenarios
// ===========================
TEST(active_scenario_set) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);

    PvtCorner c1; c1.name = "ss"; c1.derate = make_derate(1.12, 0.88, 1.12, 0.88);
    PvtCorner c2; c2.name = "ff"; c2.derate = make_derate(0.88, 1.12, 0.88, 1.12);
    mcmm.add_corner(c1);
    mcmm.add_corner(c2);

    FunctionalMode m; m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    // Only analyze ss/func
    std::unordered_set<std::string> active = {"ss/func"};
    mcmm.set_active_scenarios(active);

    auto r = mcmm.analyze();
    CHECK(r.scenarios == 2, "2 total scenarios");
    CHECK(r.active_scenarios == 1, "only 1 active");
    CHECK(r.pruned_scenarios == 1, "1 pruned (ff/func inactive)");

    // Verify the active one was analyzed
    bool found_active = false;
    for (auto& d : r.details) {
        if (d.scenario_name == "ss/func" && d.active) found_active = true;
    }
    CHECK(found_active, "ss/func was analyzed");
    PASS("active_scenario_set");
}

// ===========================
// Test 8: Scenario pruning — dominated scenario detection
// ===========================
TEST(scenario_pruning) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);

    // Slow corner: will have worse WNS → should dominate
    PvtCorner ss; ss.name = "ss"; ss.process = PvtCorner::SLOW;
    ss.derate = make_derate(1.25, 0.75, 1.25, 0.75);
    mcmm.add_corner(ss);

    // Typical corner: less pessimistic
    PvtCorner tt; tt.name = "tt"; tt.process = PvtCorner::TYPICAL;
    tt.derate = make_derate(1.05, 0.95, 1.05, 0.95);
    mcmm.add_corner(tt);

    FunctionalMode m; m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    McmmConfig cfg;
    cfg.enable_scenario_pruning = true;
    cfg.pruning_slack_margin = 0.01;
    mcmm.set_config(cfg);

    auto r = mcmm.analyze();
    CHECK(r.scenarios == 2, "2 scenarios analyzed");

    // At least one should be non-dominated (the worst one)
    bool has_non_dominated = false;
    for (auto& d : r.details) {
        if (!d.dominated && d.active) has_non_dominated = true;
    }
    CHECK(has_non_dominated, "at least one non-dominated scenario");
    PASS("scenario_pruning");
}

// ===========================
// Test 9: Signoff summaries populated
// ===========================
TEST(signoff_summaries) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();

    auto r = mcmm.analyze();

    // Setup signoff should have scenarios
    CHECK(r.setup_signoff.scenario_count > 0, "setup signoff has scenarios");
    CHECK(!r.setup_signoff.worst_scenario.empty(), "setup signoff worst identified");

    // Hold signoff
    CHECK(r.hold_signoff.scenario_count > 0, "hold signoff has scenarios");
    CHECK(!r.hold_signoff.worst_scenario.empty(), "hold signoff worst identified");

    // Leakage signoff
    CHECK(r.leakage_signoff.scenario_count > 0, "leakage signoff has scenarios");

    // Power signoff
    CHECK(r.power_signoff.scenario_count > 0, "power signoff has scenarios");
    PASS("signoff_summaries");
}

// ===========================
// Test 10: Corner signoff type assignment
// ===========================
TEST(corner_signoff_type) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);

    PvtCorner hold_corner;
    hold_corner.name = "ff_hold";
    hold_corner.process = PvtCorner::FAST;
    hold_corner.derate = make_derate(0.88, 1.12, 0.88, 1.12);
    hold_corner.signoff = SignoffType::HOLD;
    mcmm.add_corner(hold_corner);

    FunctionalMode m; m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto r = mcmm.analyze();
    CHECK(r.details.size() == 1, "1 scenario");
    CHECK(r.details[0].signoff == SignoffType::HOLD, "scenario inherits corner signoff type");
    PASS("corner_signoff_type");
}

// ===========================
// Test 11: Mode signoff type assignment
// ===========================
TEST(mode_signoff_type) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);

    PvtCorner c; c.name = "tt"; c.derate = make_derate(1.0, 1.0, 1.0, 1.0);
    c.signoff = SignoffType::ALL;
    mcmm.add_corner(c);

    FunctionalMode standby;
    standby.name = "standby";
    standby.clock_freq_mhz = 0;
    standby.switching_activity = 0.001;
    standby.signoff = SignoffType::LEAKAGE;
    mcmm.add_mode(standby);

    auto r = mcmm.analyze();
    CHECK(r.details[0].signoff == SignoffType::LEAKAGE, "scenario inherits mode signoff");
    PASS("mode_signoff_type");
}

// ===========================
// Test 12: Sensitivity analysis
// ===========================
TEST(sensitivity_analysis) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);

    PvtCorner c;
    c.name = "tt_sens";
    c.derate = make_derate(1.0, 1.0, 1.0, 1.0);
    mcmm.add_corner(c);

    FunctionalMode m; m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    McmmConfig cfg;
    cfg.enable_sensitivity = true;
    mcmm.set_config(cfg);

    auto r = mcmm.analyze();
    CHECK(r.sensitivities.size() == 1, "1 sensitivity entry");
    CHECK(r.sensitivities[0].first == "tt_sens/func", "correct scenario name");
    // Sensitivity values are finite (not NaN/Inf)
    CHECK(std::isfinite(r.sensitivities[0].second.voltage_sens), "V sensitivity finite");
    CHECK(std::isfinite(r.sensitivities[0].second.temp_sens), "T sensitivity finite");
    PASS("sensitivity_analysis");
}

// ===========================
// Test 13: Foundry corners have 7 entries
// ===========================
TEST(foundry_corners_count) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);
    mcmm.load_foundry_corners();
    CHECK(mcmm.corners().size() == 7, "7 foundry corners loaded");
    PASS("foundry_corners_count");
}

// ===========================
// Test 14: Default modes have 3 entries with signoff types
// ===========================
TEST(default_modes_signoff) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_modes();
    CHECK(mcmm.modes().size() == 3, "3 default modes");

    // func = ALL, scan = SETUP, standby = LEAKAGE
    bool found_func = false, found_scan = false, found_standby = false;
    for (auto& m : mcmm.modes()) {
        if (m.name == "func" && m.signoff == SignoffType::ALL) found_func = true;
        if (m.name == "scan" && m.signoff == SignoffType::SETUP) found_scan = true;
        if (m.name == "standby" && m.signoff == SignoffType::LEAKAGE) found_standby = true;
    }
    CHECK(found_func, "func mode is ALL");
    CHECK(found_scan, "scan mode is SETUP");
    CHECK(found_standby, "standby mode is LEAKAGE");
    PASS("default_modes_signoff");
}

// ===========================
// Test 15: Industrial result metrics
// ===========================
TEST(industrial_result_metrics) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners();
    mcmm.load_default_modes();

    auto r = mcmm.analyze();
    CHECK(r.active_scenarios > 0, "has active scenarios");
    CHECK(r.active_scenarios <= r.scenarios, "active <= total");
    CHECK(r.time_ms >= 0, "time measured");
    CHECK(!r.message.empty(), "message generated");
    PASS("industrial_result_metrics");
}

// ===========================
// Test 16: Clear active set re-enables all scenarios
// ===========================
TEST(clear_active_set) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);

    PvtCorner c1; c1.name = "c1"; c1.derate = make_derate(1.0, 1.0, 1.0, 1.0);
    PvtCorner c2; c2.name = "c2"; c2.derate = make_derate(1.0, 1.0, 1.0, 1.0);
    mcmm.add_corner(c1);
    mcmm.add_corner(c2);

    FunctionalMode m; m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    // Restrict to c1/func only
    std::unordered_set<std::string> active = {"c1/func"};
    mcmm.set_active_scenarios(active);
    auto r1 = mcmm.analyze();
    CHECK(r1.active_scenarios == 1, "only 1 active with set");

    // Clear restriction
    mcmm.clear_active_set();
    auto r2 = mcmm.analyze();
    CHECK(r2.active_scenarios == 2, "all 2 active after clear");
    PASS("clear_active_set");
}

// ===========================
// Test 17: Full foundry signoff E2E
// ===========================
TEST(full_foundry_signoff_e2e) {
    auto nl = make_test_netlist();
    McmmAnalyzer mcmm(nl);
    mcmm.load_foundry_corners();
    mcmm.load_default_modes();

    McmmConfig cfg;
    cfg.enable_scenario_pruning = true;
    cfg.enable_sensitivity = false;  // skip for speed
    mcmm.set_config(cfg);

    auto r = mcmm.analyze();
    CHECK(r.corners == 7, "7 foundry corners");
    CHECK(r.modes == 3, "3 modes");
    CHECK(r.scenarios == 21, "7×3 = 21 scenarios");
    CHECK(!r.worst_setup_scenario.empty(), "worst setup identified");
    CHECK(!r.worst_hold_scenario.empty(), "worst hold identified");
    CHECK(!r.max_power_scenario.empty(), "max power identified");

    // Backward compat
    CHECK(r.worst_wns == r.worst_setup_wns, "backward compat WNS");
    CHECK(r.worst_tns == r.worst_setup_tns, "backward compat TNS");
    CHECK(r.worst_corner == r.worst_setup_scenario, "backward compat corner");
    PASS("full_foundry_signoff_e2e");
}

// ===========================
// Test 18: McmmScenario active/dominated fields
// ===========================
TEST(scenario_fields) {
    McmmScenario s;
    CHECK(s.active == true, "scenario active by default");
    CHECK(s.dominated == false, "scenario not dominated by default");
    CHECK(s.signoff == SignoffType::ALL, "scenario signoff = ALL by default");
    PASS("scenario_fields");
}

int main() {
    std::cout << "=== Phase 24: MCMM Industrial Features Tests ===\n\n";
    RUN(signoff_type_enum);
    RUN(pvt_corner_defaults);
    RUN(functional_mode_defaults);
    RUN(mcmm_config_defaults);
    RUN(cppr_per_corner);
    RUN(pocv_per_corner);
    RUN(active_scenario_set);
    RUN(scenario_pruning);
    RUN(signoff_summaries);
    RUN(corner_signoff_type);
    RUN(mode_signoff_type);
    RUN(sensitivity_analysis);
    RUN(foundry_corners_count);
    RUN(default_modes_signoff);
    RUN(industrial_result_metrics);
    RUN(clear_active_set);
    RUN(full_foundry_signoff_e2e);
    RUN(scenario_fields);

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===\n";
    return failed > 0 ? 1 : 0;
}
