// SiliconForge — Phase 38 Test Suite
// DRC Advanced Rules: EM width, multi-patterning, stress voiding,
// ESD spacing, guard ring, reliability width, temperature-variant spacing

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "verify/drc.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// Helper: build a design with wires for DRC testing
static PhysicalDesign build_wire_design() {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    return pd;
}

// ── DrcConfig Tests ──────────────────────────────────────────────────

TEST(config_defaults) {
    DrcConfig cfg;
    CHECK(cfg.temperature_c == 25.0, "default temp");
    CHECK(cfg.em_current_limit_ma == 1.0, "default EM current");
    CHECK(cfg.multi_patterning_colors == 2, "default MP colors");
    CHECK(cfg.enable_reliability_rules == true, "default reliability");
    CHECK(cfg.enable_em_rules == true, "default EM enabled");
    CHECK(cfg.target_lifetime_years == 10.0, "default lifetime");
    PASS("config_defaults");
}

TEST(config_custom) {
    PhysicalDesign pd = build_wire_design();
    DrcEngine eng(pd);
    DrcConfig cfg;
    cfg.temperature_c = 85.0;
    cfg.em_current_limit_ma = 2.5;
    cfg.multi_patterning_colors = 3;
    cfg.enable_reliability_rules = false;
    cfg.target_lifetime_years = 15.0;
    eng.set_config(cfg);
    CHECK(eng.config().temperature_c == 85.0, "custom temp");
    CHECK(eng.config().em_current_limit_ma == 2.5, "custom EM current");
    CHECK(eng.config().multi_patterning_colors == 3, "custom MP colors");
    CHECK(eng.config().enable_reliability_rules == false, "custom reliability off");
    CHECK(eng.config().target_lifetime_years == 15.0, "custom lifetime");
    PASS("config_custom");
}

// ── EM Width Tests ───────────────────────────────────────────────────

TEST(em_width_clean) {
    PhysicalDesign pd = build_wire_design();
    // Wire wide enough for EM
    pd.wires.push_back({0, {10,10}, {50,10}, 0.5, 0});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "M1.EM.W"; rule.description = "EM min width";
    rule.value = 0.3; rule.type = DrcRule::EM_MIN_WIDTH;
    rule.layer = 0; rule.aux_value = 1.0;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.em_width_violations == 0, "no EM violations for wide wire");
    CHECK(r.violations == 0, "clean");
    PASS("em_width_clean");
}

TEST(em_width_violation) {
    PhysicalDesign pd = build_wire_design();
    // Wire too narrow for EM
    pd.wires.push_back({0, {10,10}, {50,10}, 0.1, 0});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "M1.EM.W"; rule.description = "EM min width";
    rule.value = 0.3; rule.type = DrcRule::EM_MIN_WIDTH;
    rule.layer = 0; rule.aux_value = 1.0;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.em_width_violations == 1, "1 EM violation");
    CHECK(r.violations == 1, "1 total violation");
    PASS("em_width_violation");
}

TEST(em_disabled) {
    PhysicalDesign pd = build_wire_design();
    pd.wires.push_back({0, {10,10}, {50,10}, 0.1, 0});
    DrcEngine eng(pd);
    DrcConfig cfg;
    cfg.enable_em_rules = false;
    eng.set_config(cfg);
    DrcRule rule;
    rule.name = "M1.EM.W"; rule.description = "EM min width";
    rule.value = 0.3; rule.type = DrcRule::EM_MIN_WIDTH;
    rule.layer = 0;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.em_width_violations == 0, "EM disabled = no violations");
    CHECK(r.violations == 0, "clean when disabled");
    PASS("em_disabled");
}

// ── Multi-Patterning Tests ───────────────────────────────────────────

TEST(multi_patterning_clean) {
    PhysicalDesign pd = build_wire_design();
    // Two wires with different colors (net_id 0 and 1, mod 2 = 0 and 1)
    pd.wires.push_back({0, {10,10}, {50,10}, 0.2, 0});
    pd.wires.push_back({0, {10,11}, {50,11}, 0.2, 1});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "M1.MP.S"; rule.description = "MP spacing";
    rule.value = 0.5; rule.type = DrcRule::MULTI_PATTERNING_COLOR;
    rule.layer = 0;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.multi_patterning_violations == 0, "different colors = no violation");
    PASS("multi_patterning_clean");
}

TEST(multi_patterning_violation) {
    PhysicalDesign pd = build_wire_design();
    // Two wires with same color (both net_id=0) too close
    pd.wires.push_back({0, {10,10}, {50,10}, 0.2, 0});
    pd.wires.push_back({0, {10,10.5}, {50,10.5}, 0.2, 2}); // 2 % 2 == 0 (same color)
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "M1.MP.S"; rule.description = "MP spacing";
    rule.value = 0.5; rule.type = DrcRule::MULTI_PATTERNING_COLOR;
    rule.layer = 0;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.multi_patterning_violations > 0, "same color too close = violation");
    PASS("multi_patterning_violation");
}

TEST(multi_patterning_triple) {
    PhysicalDesign pd = build_wire_design();
    // Triple patterning: net_id 0,1,2 → colors 0,1,2
    pd.wires.push_back({0, {10,10}, {50,10}, 0.2, 0});
    pd.wires.push_back({0, {10,10.3}, {50,10.3}, 0.2, 1});
    pd.wires.push_back({0, {10,10.6}, {50,10.6}, 0.2, 2});
    DrcEngine eng(pd);
    DrcConfig cfg;
    cfg.multi_patterning_colors = 3;
    eng.set_config(cfg);
    DrcRule rule;
    rule.name = "M1.MP.S"; rule.description = "MP triple spacing";
    rule.value = 0.2; rule.type = DrcRule::MULTI_PATTERNING_COLOR;
    rule.layer = 0;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.multi_patterning_violations == 0, "triple patterning all different colors");
    PASS("multi_patterning_triple");
}

// ── Stress Voiding Tests ─────────────────────────────────────────────

TEST(stress_voiding_clean) {
    PhysicalDesign pd = build_wire_design();
    // Wide wire with vias far apart
    pd.wires.push_back({DrcLayer::MET1, {10,10}, {90,10}, 2.0, 0});
    pd.vias.push_back({{20, 10}, DrcLayer::MET1, DrcLayer::MET2});
    pd.vias.push_back({{80, 10}, DrcLayer::MET1, DrcLayer::MET2});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "M1.SV.D"; rule.description = "Stress voiding";
    rule.value = 10.0; rule.type = DrcRule::STRESS_VOIDING;
    rule.layer = DrcLayer::MET1; rule.aux_value = 1.0;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.stress_violations == 0, "vias far apart = clean");
    PASS("stress_voiding_clean");
}

TEST(stress_voiding_violation) {
    PhysicalDesign pd = build_wire_design();
    // Wide wire with vias too close
    pd.wires.push_back({DrcLayer::MET1, {10,10}, {90,10}, 2.0, 0});
    pd.vias.push_back({{20, 10}, DrcLayer::MET1, DrcLayer::MET2});
    pd.vias.push_back({{23, 10}, DrcLayer::MET1, DrcLayer::MET2});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "M1.SV.D"; rule.description = "Stress voiding";
    rule.value = 10.0; rule.type = DrcRule::STRESS_VOIDING;
    rule.layer = DrcLayer::MET1; rule.aux_value = 1.0;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.stress_violations > 0, "vias too close on wide wire = violation");
    PASS("stress_voiding_violation");
}

// ── ESD Spacing Tests ────────────────────────────────────────────────

TEST(esd_spacing_clean) {
    PhysicalDesign pd = build_wire_design();
    // PAD wire far from MET1 wire
    pd.wires.push_back({DrcLayer::PAD, {10,10}, {20,10}, 5.0, 0});
    pd.wires.push_back({DrcLayer::MET1, {50,10}, {60,10}, 0.5, 1});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "ESD.S"; rule.description = "ESD spacing";
    rule.value = 5.0; rule.type = DrcRule::ESD_SPACING;
    rule.layer = DrcLayer::PAD; rule.layer2 = DrcLayer::MET1;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.esd_violations == 0, "far apart = clean");
    PASS("esd_spacing_clean");
}

TEST(esd_spacing_violation) {
    PhysicalDesign pd = build_wire_design();
    // PAD wire close to MET1 wire
    pd.wires.push_back({DrcLayer::PAD, {10,10}, {20,10}, 5.0, 0});
    pd.wires.push_back({DrcLayer::MET1, {22,10}, {30,10}, 0.5, 1});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "ESD.S"; rule.description = "ESD spacing";
    rule.value = 5.0; rule.type = DrcRule::ESD_SPACING;
    rule.layer = DrcLayer::PAD; rule.layer2 = DrcLayer::MET1;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.esd_violations > 0, "too close = ESD violation");
    PASS("esd_spacing_violation");
}

// ── Guard Ring Tests ─────────────────────────────────────────────────

TEST(guard_ring_clean) {
    PhysicalDesign pd = build_wire_design();
    pd.wires.push_back({DrcLayer::TAP, {10,10}, {50,10}, 1.0, 0});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "GR.W"; rule.description = "Guard ring width";
    rule.value = 0.5; rule.type = DrcRule::GUARD_RING_SPACING;
    rule.layer = DrcLayer::TAP;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.guard_ring_violations == 0, "wide enough = clean");
    PASS("guard_ring_clean");
}

TEST(guard_ring_violation) {
    PhysicalDesign pd = build_wire_design();
    pd.wires.push_back({DrcLayer::TAP, {10,10}, {50,10}, 0.2, 0});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "GR.W"; rule.description = "Guard ring width";
    rule.value = 0.5; rule.type = DrcRule::GUARD_RING_SPACING;
    rule.layer = DrcLayer::TAP;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.guard_ring_violations > 0, "too narrow = violation");
    PASS("guard_ring_violation");
}

// ── Reliability Width Tests ──────────────────────────────────────────

TEST(reliability_width) {
    PhysicalDesign pd = build_wire_design();
    // Wire that passes at 5yr but fails at 20yr
    pd.wires.push_back({0, {10,10}, {50,10}, 0.15, 0});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "M1.REL.W"; rule.description = "Reliability width";
    rule.value = 0.1; rule.type = DrcRule::RELIABILITY_WIDTH;
    rule.layer = 0; rule.aux_value = 0.02;
    eng.add_rule(rule);

    // At 5yr: eff_width = 0.1 * (1 + 0.02*(5-5)) = 0.1 → wire 0.15 passes
    DrcConfig cfg5;
    cfg5.target_lifetime_years = 5.0;
    eng.set_config(cfg5);
    auto r1 = eng.check();
    CHECK(r1.reliability_violations == 0, "5yr lifetime passes");

    // At 20yr: eff_width = 0.1 * (1 + 0.02*(20-5)) = 0.1 * 1.3 = 0.13 → wire 0.15 passes
    DrcConfig cfg20;
    cfg20.target_lifetime_years = 20.0;
    eng.set_config(cfg20);
    auto r2 = eng.check();
    CHECK(r2.reliability_violations == 0, "20yr passes for 0.15 wire");

    // At 50yr: eff_width = 0.1 * (1 + 0.02*(50-5)) = 0.1 * 1.9 = 0.19 → wire 0.15 fails
    DrcConfig cfg50;
    cfg50.target_lifetime_years = 50.0;
    eng.set_config(cfg50);
    auto r3 = eng.check();
    CHECK(r3.reliability_violations > 0, "50yr fails for 0.15 wire");
    PASS("reliability_width");
}

TEST(reliability_disabled) {
    PhysicalDesign pd = build_wire_design();
    pd.wires.push_back({0, {10,10}, {50,10}, 0.01, 0});
    DrcEngine eng(pd);
    DrcConfig cfg;
    cfg.enable_reliability_rules = false;
    cfg.target_lifetime_years = 100.0;
    eng.set_config(cfg);
    DrcRule rule;
    rule.name = "M1.REL.W"; rule.description = "Reliability width";
    rule.value = 0.1; rule.type = DrcRule::RELIABILITY_WIDTH;
    rule.layer = 0; rule.aux_value = 0.02;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.reliability_violations == 0, "reliability disabled = no violations");
    PASS("reliability_disabled");
}

// ── Temperature-Variant Spacing Tests ────────────────────────────────

TEST(temp_variant_spacing) {
    PhysicalDesign pd = build_wire_design();
    // Two wires with spacing that passes at 25C but fails at 125C
    pd.wires.push_back({0, {10,10}, {50,10}, 0.2, 0});
    pd.wires.push_back({0, {10,10.35}, {50,10.35}, 0.2, 1});
    // Spacing = 10.35 - 10 - 0.1 - 0.1 = 0.15 (edge-to-edge approx)
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "M1.TV.S"; rule.description = "Temp spacing";
    rule.value = 0.13; rule.type = DrcRule::TEMP_VARIANT_SPACING;
    rule.layer = 0; rule.aux_value = 0.005; // 0.5% per deg C

    eng.add_rule(rule);

    // At 25C: eff_spacing = 0.13 * (1 + 0.005*0) = 0.13
    DrcConfig cfg25;
    cfg25.temperature_c = 25.0;
    eng.set_config(cfg25);
    auto r1 = eng.check();
    CHECK(r1.temp_violations == 0, "25C passes");

    // At 125C: eff_spacing = 0.13 * (1 + 0.005*100) = 0.13 * 1.5 = 0.195
    DrcConfig cfg125;
    cfg125.temperature_c = 125.0;
    eng.set_config(cfg125);
    auto r2 = eng.check();
    CHECK(r2.temp_violations > 0, "125C fails");
    PASS("temp_variant_spacing");
}

// ── Advanced Node Rule Loading ───────────────────────────────────────

TEST(advanced_node_rules) {
    PhysicalDesign pd = build_wire_design();
    DrcEngine eng(pd);
    eng.load_advanced_node_rules(0.028);
    int count = eng.rule_count();
    CHECK(count > 20, "loaded >20 advanced rules, got " + std::to_string(count));
    // Verify EM rules present
    bool has_em = false, has_mp = false, has_sv = false, has_tv = false, has_rel = false;
    bool has_esd = false, has_gr = false;
    for (auto& r : eng.rules()) {
        if (r.type == DrcRule::EM_MIN_WIDTH) has_em = true;
        if (r.type == DrcRule::MULTI_PATTERNING_COLOR) has_mp = true;
        if (r.type == DrcRule::STRESS_VOIDING) has_sv = true;
        if (r.type == DrcRule::TEMP_VARIANT_SPACING) has_tv = true;
        if (r.type == DrcRule::RELIABILITY_WIDTH) has_rel = true;
        if (r.type == DrcRule::ESD_SPACING) has_esd = true;
        if (r.type == DrcRule::GUARD_RING_SPACING) has_gr = true;
    }
    CHECK(has_em, "has EM rules");
    CHECK(has_mp, "has multi-patterning rules");
    CHECK(has_sv, "has stress voiding rules");
    CHECK(has_tv, "has temp-variant rules");
    CHECK(has_rel, "has reliability rules");
    CHECK(has_esd, "has ESD rules");
    CHECK(has_gr, "has guard ring rules");
    PASS("advanced_node_rules");
}

// ── EM Rule Generation ───────────────────────────────────────────────

TEST(em_rule_generation) {
    PhysicalDesign pd = build_wire_design();
    DrcEngine eng(pd);
    DrcConfig cfg;
    cfg.em_current_limit_ma = 2.0;
    eng.set_config(cfg);
    eng.generate_em_rules(4);
    int em_count = 0;
    for (auto& r : eng.rules())
        if (r.type == DrcRule::EM_MIN_WIDTH) em_count++;
    CHECK(em_count == 4, "generated 4 EM rules, got " + std::to_string(em_count));
    // Verify rules have correct current limit
    for (auto& r : eng.rules()) {
        if (r.type == DrcRule::EM_MIN_WIDTH)
            CHECK(r.aux_value == 2.0, "EM rule uses config current limit");
    }
    PASS("em_rule_generation");
}

// ── Backward Compatibility ───────────────────────────────────────────

TEST(backward_compat) {
    PhysicalDesign pd = build_wire_design();
    for (int i = 0; i < 5; i++) {
        int c = pd.add_cell("c" + std::to_string(i), "INV", 3.0, 10.0);
        pd.cells[c].position = {10.0 + i * 20.0, 10.0};
        pd.cells[c].placed = true;
    }
    DrcEngine eng(pd);
    eng.load_default_rules(0.13);
    auto r = eng.check();
    CHECK(r.total_rules > 0, "default rules still work");
    // Verify existing violation categories still tracked
    CHECK(r.width_violations >= 0, "width_violations field exists");
    CHECK(r.spacing_violations >= 0, "spacing_violations field exists");
    CHECK(r.em_width_violations == 0, "no EM violations with default rules");
    CHECK(r.multi_patterning_violations == 0, "no MP violations with default rules");
    PASS("backward_compat");
}

// ── E2E with All New Rule Types ──────────────────────────────────────

TEST(e2e_advanced_rules) {
    PhysicalDesign pd = build_wire_design();
    // Add some cells
    for (int i = 0; i < 3; i++) {
        int c = pd.add_cell("c" + std::to_string(i), "BUF", 3.0, 10.0);
        pd.cells[c].position = {10.0 + i * 30.0, 10.0};
        pd.cells[c].placed = true;
    }
    // Add wires on MET1
    pd.wires.push_back({DrcLayer::MET1, {10,50}, {90,50}, 0.5, 0});
    pd.wires.push_back({DrcLayer::MET1, {10,51}, {90,51}, 0.5, 1});
    // Add vias
    pd.vias.push_back({{20, 50}, DrcLayer::MET1, DrcLayer::MET2});
    pd.vias.push_back({{80, 50}, DrcLayer::MET1, DrcLayer::MET2});
    // Add TAP wire for guard ring
    pd.wires.push_back({DrcLayer::TAP, {5,5}, {95,5}, 1.0, 2});
    // Add PAD wire far from devices
    pd.wires.push_back({DrcLayer::PAD, {150,80}, {160,80}, 5.0, 3});

    DrcEngine eng(pd);
    DrcConfig cfg;
    cfg.temperature_c = 85.0;
    cfg.target_lifetime_years = 10.0;
    eng.set_config(cfg);

    eng.load_advanced_node_rules(0.028);
    auto r = eng.check();

    CHECK(r.total_rules > 20, "many rules checked");
    // Result should have time_ms set
    CHECK(r.time_ms >= 0, "timing recorded");
    // Message should be non-empty
    CHECK(!r.message.empty(), "result message set");

    // Verify we can access all violation categories
    int total_advanced = r.em_width_violations + r.multi_patterning_violations +
                         r.stress_violations + r.esd_violations +
                         r.guard_ring_violations + r.reliability_violations +
                         r.temp_violations;
    CHECK(total_advanced >= 0, "advanced violation counts accessible");
    PASS("e2e_advanced_rules");
}

// ── Main ─────────────────────────────────────────────────────────────

int main() {
    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Phase 38: DRC Advanced Rules\n";
    std::cout << "══════════════════════════════════════════════════\n\n";

    std::cout << "── DrcConfig ──\n";
    RUN(config_defaults);
    RUN(config_custom);

    std::cout << "\n── EM Width ──\n";
    RUN(em_width_clean);
    RUN(em_width_violation);
    RUN(em_disabled);

    std::cout << "\n── Multi-Patterning ──\n";
    RUN(multi_patterning_clean);
    RUN(multi_patterning_violation);
    RUN(multi_patterning_triple);

    std::cout << "\n── Stress Voiding ──\n";
    RUN(stress_voiding_clean);
    RUN(stress_voiding_violation);

    std::cout << "\n── ESD Spacing ──\n";
    RUN(esd_spacing_clean);
    RUN(esd_spacing_violation);

    std::cout << "\n── Guard Ring ──\n";
    RUN(guard_ring_clean);
    RUN(guard_ring_violation);

    std::cout << "\n── Reliability Width ──\n";
    RUN(reliability_width);
    RUN(reliability_disabled);

    std::cout << "\n── Temperature-Variant Spacing ──\n";
    RUN(temp_variant_spacing);

    std::cout << "\n── Advanced Node Rules ──\n";
    RUN(advanced_node_rules);

    std::cout << "\n── EM Rule Generation ──\n";
    RUN(em_rule_generation);

    std::cout << "\n── Backward Compatibility ──\n";
    RUN(backward_compat);

    std::cout << "\n── E2E Advanced ──\n";
    RUN(e2e_advanced_rules);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Phase 38: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
