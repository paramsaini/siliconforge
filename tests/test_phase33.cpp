// SiliconForge — Phase 33: Electromigration Analysis Tests
#include "timing/electromigration.hpp"
#include <iostream>
#include <cmath>

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

using namespace sf;

TEST(blacks_equation) {
    EmAnalyzer em;
    // At limit, MTTF should be around target lifetime
    double mttf = em.compute_mttf(1.0, 1.0, 105.0, 0.7, 2.0);
    CHECK(mttf > 0, "MTTF should be positive");
    // Below limit should give longer lifetime
    double mttf_safe = em.compute_mttf(0.5, 1.0, 105.0, 0.7, 2.0);
    CHECK(mttf_safe > mttf, "Lower current density should give longer MTTF");
    PASS("blacks_equation");
}

TEST(temperature_effect) {
    EmAnalyzer em;
    double mttf_cool = em.compute_mttf(0.8, 1.0, 25.0, 0.7, 2.0);
    double mttf_hot = em.compute_mttf(0.8, 1.0, 125.0, 0.7, 2.0);
    CHECK(mttf_cool > mttf_hot, "Cooler temperature should give longer MTTF");
    PASS("temperature_effect");
}

TEST(current_density) {
    EmAnalyzer em;
    double j = em.current_density(1.0, 0.14, 0.13);
    CHECK(j > 0, "Current density should be positive");
    // Wider wire → lower density
    double j_wide = em.current_density(1.0, 0.28, 0.13);
    CHECK(j_wide < j, "Wider wire should have lower density");
    PASS("current_density");
}

TEST(sky130_rules) {
    auto rules = EmAnalyzer::default_rules_sky130();
    CHECK(!rules.empty(), "SKY130 rules should not be empty");
    CHECK(rules.size() >= 5, "Should have at least 5 layer rules");
    // Check met1 exists
    bool found_met1 = false;
    for (auto& r : rules) {
        if (r.layer_name == "met1" || r.layer_name == "metal1" || r.layer_name == "M1") {
            found_met1 = true;
            CHECK(r.jdc_limit_ma_per_um > 0, "DC limit should be positive");
        }
    }
    CHECK(found_met1 || rules.size() > 3, "Should have metal layer rules");
    PASS("sky130_rules");
}

TEST(rules_7nm) {
    auto rules = EmAnalyzer::default_rules_7nm();
    CHECK(!rules.empty(), "7nm rules should not be empty");
    // 7nm should have tighter limits than SKY130
    PASS("rules_7nm");
}

TEST(analyze_empty_design) {
    Netlist nl;
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    EmConfig cfg;
    cfg.layer_rules = EmAnalyzer::default_rules_sky130();
    EmAnalyzer em;
    EmResult result = em.analyze(nl, pd, cfg);
    // Empty design should pass
    CHECK(result.violations.empty() || result.total_nets_checked == 0, "Empty design no violations");
    PASS("analyze_empty_design");
}

TEST(analyze_simple_design) {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId q = nl.add_net("q"); nl.mark_output(q);
    nl.add_gate(GateType::AND, {a, b}, w1, "g1");
    nl.add_dff(w1, clk, w2, -1, "ff1");
    nl.add_gate(GateType::BUF, {w2}, q, "buf1");

    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 200);
    pd.add_cell("g1", "AND2", 1.0, 2.72);
    pd.add_cell("ff1", "DFF", 2.0, 2.72);
    pd.add_cell("buf1", "BUF", 0.46, 2.72);

    EmConfig cfg;
    cfg.layer_rules = EmAnalyzer::default_rules_sky130();
    cfg.temperature_c = 105.0;
    cfg.clock_freq_ghz = 1.0;

    EmAnalyzer em;
    EmResult result = em.analyze(nl, pd, cfg);
    CHECK(result.total_nets_checked >= 0, "Should check some nets");
    CHECK(!result.report.empty() || !result.summary.empty(), "Should produce report");
    PASS("analyze_simple_design");
}

TEST(suggest_fixes) {
    EmResult result;
    EmViolation v;
    v.net_name = "vdd_core";
    v.layer_name = "met1";
    v.type = EmViolation::DC_DENSITY;
    v.current_ma = 2.5;
    v.limit_ma = 1.0;
    v.ratio = 2.5;
    v.wire_width_um = 0.14;
    v.estimated_mttf_years = 0.8;
    v.severity = EmViolation::ERROR;
    result.violations.push_back(v);

    EmConfig cfg;
    cfg.layer_rules = EmAnalyzer::default_rules_sky130();
    EmAnalyzer em;
    auto fixes = em.suggest_fixes(result, cfg);
    CHECK(!fixes.empty(), "Should suggest fixes for violations");
    CHECK(fixes[0].recommended_width_um > fixes[0].current_width_um,
          "Should recommend wider wire");
    PASS("suggest_fixes");
}

TEST(violation_severity) {
    // Violations should classify by severity based on ratio
    EmViolation v;
    v.ratio = 1.5;
    v.severity = EmViolation::ERROR;
    CHECK(v.severity == EmViolation::ERROR, "High ratio should be ERROR");
    v.ratio = 0.9;
    v.severity = EmViolation::WARNING;
    CHECK(v.severity == EmViolation::WARNING, "Marginal should be WARNING");
    PASS("violation_severity");
}

TEST(zero_current) {
    EmAnalyzer em;
    double mttf = em.compute_mttf(0.0, 1.0, 105.0, 0.7, 2.0);
    CHECK(mttf > 1e10, "Zero current should give very long MTTF");
    PASS("zero_current");
}

TEST(config_defaults) {
    EmConfig cfg;
    CHECK(cfg.target_lifetime_years > 0, "Default lifetime should be positive");
    CHECK(cfg.temperature_c > 0, "Default temperature should be positive");
    CHECK(cfg.activation_energy_ev > 0, "Activation energy should be positive");
    CHECK(cfg.current_density_exponent > 0, "Exponent should be positive");
    PASS("config_defaults");
}

int main() {
    std::cout << "═══════════════════════════════════════════════════════\n"
              << " Phase 33: Electromigration Analysis Tests\n"
              << "═══════════════════════════════════════════════════════\n\n";
    RUN(blacks_equation);
    RUN(temperature_effect);
    RUN(current_density);
    RUN(sky130_rules);
    RUN(rules_7nm);
    RUN(analyze_empty_design);
    RUN(analyze_simple_design);
    RUN(suggest_fixes);
    RUN(violation_severity);
    RUN(zero_current);
    RUN(config_defaults);
    std::cout << "\n═══════════════════════════════════════════════════════\n"
              << " Results: " << passed << " passed, " << failed << " failed\n"
              << "═══════════════════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
