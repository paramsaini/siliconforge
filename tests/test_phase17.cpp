// SiliconForge Phase 17 — Industrial-Grade DRC Test Suite
// Tests DRC engine: SKY130 rule deck loading, JSON I/O, rule management,
// violation detection (width, spacing, boundary, enclosure), error handling.

#include "pnr/physical.hpp"
#include "verify/drc.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <string>
#include <fstream>
#include <unordered_set>
#include <algorithm>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// ---------------------------------------------------------------------------
// Helper: find a rule by name in a rule vector
// ---------------------------------------------------------------------------
static const DrcRule* find_rule(const std::vector<DrcRule>& rules,
                                const std::string& name) {
    for (auto& r : rules)
        if (r.name == name) return &r;
    return nullptr;
}

// Helper: build a minimal clean physical design
static PhysicalDesign build_clean_design() {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    pd.row_height = 10.0;
    int c0 = pd.add_cell("c0", "inv", 2.0, 2.0);
    pd.cells[c0].position = Point(10, 10);
    pd.cells[c0].placed = true;
    // A well-formed MET1 wire inside die area, generous width
    WireSegment w;
    w.layer = DrcLayer::MET1;
    w.start = Point(10, 10);
    w.end   = Point(30, 10);
    w.width = 1.0;
    w.net_id = 0;
    pd.wires.push_back(w);
    return pd;
}

// ===================================================================
//  SKY130 Rule Deck
// ===================================================================

// 1. sky130_rule_count
TEST(sky130_rule_count) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    eng.load_sky130_rules();
    CHECK(eng.rule_count() == 224,
          "expected 224 SKY130 rules, got " + std::to_string(eng.rule_count()));
    PASS("sky130_rule_count");
}

// 2. sky130_key_values
TEST(sky130_key_values) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    eng.load_sky130_rules();

    auto& rules = eng.rules();

    auto check_val = [&](const std::string& name, double expected) -> bool {
        auto* r = find_rule(rules, name);
        if (!r) return false;
        return std::fabs(r->value - expected) < 1e-3;
    };

    CHECK(find_rule(rules, "nwell.1") != nullptr, "nwell.1 not found");
    CHECK(check_val("nwell.1", 0.840),  "nwell.1 value mismatch");
    CHECK(check_val("diff.1",  0.150),  "diff.1 value mismatch");
    CHECK(check_val("poly.1a", 0.150),  "poly.1a value mismatch");
    CHECK(check_val("m1.1",    0.140),  "m1.1 value mismatch");
    CHECK(check_val("m1.2",    0.140),  "m1.2 value mismatch");
    CHECK(check_val("m5.1",    1.600),  "m5.1 value mismatch");
    PASS("sky130_key_values");
}

// 3. sky130_layer_assignments
TEST(sky130_layer_assignments) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    eng.load_sky130_rules();

    auto& rules = eng.rules();
    auto* nw = find_rule(rules, "nwell.1");
    auto* po = find_rule(rules, "poly.1a");
    auto* m1 = find_rule(rules, "m1.1");
    auto* m5 = find_rule(rules, "m5.1");

    CHECK(nw && nw->layer == DrcLayer::NWELL, "nwell.1 layer != NWELL(0)");
    CHECK(po && po->layer == DrcLayer::POLY,  "poly.1a layer != POLY(5)");
    CHECK(m1 && m1->layer == DrcLayer::MET1,  "m1.1 layer != MET1(14)");
    CHECK(m5 && m5->layer == DrcLayer::MET5,  "m5.1 layer != MET5(22)");
    PASS("sky130_layer_assignments");
}

// 4. sky130_conditional_rules
TEST(sky130_conditional_rules) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    eng.load_sky130_rules();

    int cond_count = 0;
    bool all_have_min_width = true;
    for (auto& r : eng.rules()) {
        if (r.type == DrcRule::CONDITIONAL_SPACING ||
            r.type == DrcRule::CONDITIONAL_ENCLOSURE) {
            cond_count++;
            if (r.condition_min_width <= 0)
                all_have_min_width = false;
        }
    }
    CHECK(cond_count >= 10,
          "expected >= 10 conditional rules, got " + std::to_string(cond_count));
    CHECK(all_have_min_width, "all conditional rules should have condition_min_width > 0");
    PASS("sky130_conditional_rules");
}

// 5. sky130_all_rule_categories
TEST(sky130_all_rule_categories) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    eng.load_sky130_rules();

    int min_width = 0, min_spacing = 0, min_area = 0;
    int via_enc = 0, antenna = 0, density = 0;
    for (auto& r : eng.rules()) {
        if (r.type == DrcRule::MIN_WIDTH)       min_width++;
        if (r.type == DrcRule::MIN_SPACING)     min_spacing++;
        if (r.type == DrcRule::MIN_AREA)        min_area++;
        if (r.type == DrcRule::VIA_ENCLOSURE)   via_enc++;
        if (r.type == DrcRule::ANTENNA_RATIO || r.type == DrcRule::ANTENNA_AREA)
            antenna++;
        if (r.type == DrcRule::DENSITY_MIN || r.type == DrcRule::DENSITY_MAX)
            density++;
    }
    CHECK(min_width   >= 5, "expected >= 5 MIN_WIDTH rules, got "   + std::to_string(min_width));
    CHECK(min_spacing >= 5, "expected >= 5 MIN_SPACING rules, got " + std::to_string(min_spacing));
    CHECK(min_area    >= 3, "expected >= 3 MIN_AREA rules, got "    + std::to_string(min_area));
    CHECK(via_enc     >= 3, "expected >= 3 VIA_ENCLOSURE rules, got " + std::to_string(via_enc));
    CHECK(antenna     >= 2, "expected >= 2 ANTENNA rules, got "     + std::to_string(antenna));
    CHECK(density     >= 2, "expected >= 2 DENSITY rules, got "     + std::to_string(density));
    PASS("sky130_all_rule_categories");
}

// ===================================================================
//  JSON I/O
// ===================================================================

// 6. json_roundtrip
TEST(json_roundtrip) {
    const std::string tmpfile = "/tmp/sf_test_drc_rt.json";

    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    eng.load_sky130_rules();
    int orig_count = eng.rule_count();
    std::string first_name = eng.rules().front().name;
    std::string last_name  = eng.rules().back().name;

    CHECK(eng.write_rules_to_file(tmpfile), "write_rules_to_file failed");

    DrcEngine eng2(pd);
    int loaded = eng2.load_rules_from_file(tmpfile);
    CHECK(loaded == orig_count,
          "loaded " + std::to_string(loaded) + " != " + std::to_string(orig_count));
    CHECK(eng2.rule_count() == orig_count, "rule_count mismatch after reload");
    CHECK(eng2.rules().front().name == first_name, "first rule name mismatch");
    CHECK(eng2.rules().back().name  == last_name,  "last rule name mismatch");

    std::remove(tmpfile.c_str());
    PASS("json_roundtrip");
}

// 7. json_roundtrip_conditional
TEST(json_roundtrip_conditional) {
    const std::string tmpfile = "/tmp/sf_test_drc_cond.json";

    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    eng.load_sky130_rules();

    // Find a CONDITIONAL_SPACING rule
    std::string cond_name;
    double cond_value = 0, cond_min_w = 0;
    for (auto& r : eng.rules()) {
        if (r.type == DrcRule::CONDITIONAL_SPACING) {
            cond_name  = r.name;
            cond_value = r.value;
            cond_min_w = r.condition_min_width;
            break;
        }
    }
    CHECK(!cond_name.empty(), "no CONDITIONAL_SPACING rule found");

    CHECK(eng.write_rules_to_file(tmpfile), "write failed");

    DrcEngine eng2(pd);
    eng2.load_rules_from_file(tmpfile);
    auto* reloaded = find_rule(eng2.rules(), cond_name);
    CHECK(reloaded != nullptr, "conditional rule not found after reload");
    CHECK(std::fabs(reloaded->value - cond_value) < 1e-6,
          "conditional rule value mismatch");
    CHECK(std::fabs(reloaded->condition_min_width - cond_min_w) < 1e-6,
          "conditional rule condition_min_width mismatch");

    std::remove(tmpfile.c_str());
    PASS("json_roundtrip_conditional");
}

// 8. json_append_mode
TEST(json_append_mode) {
    const std::string tmpfile = "/tmp/sf_test_drc_append.json";

    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    eng.load_sky130_rules();
    int sky_count = eng.rule_count(); // 224
    CHECK(eng.write_rules_to_file(tmpfile), "write failed");

    eng.clear_rules();
    CHECK(eng.rule_count() == 0, "clear should yield 0 rules");

    // Add one manual rule
    DrcRule manual;
    manual.name = "custom.1";
    manual.description = "test manual rule";
    manual.value = 0.5;
    manual.type = DrcRule::MIN_WIDTH;
    manual.layer = DrcLayer::MET1;
    eng.add_rule(manual);
    CHECK(eng.rule_count() == 1, "should have 1 rule after add");

    // Append from file
    eng.load_rules_from_file(tmpfile, true);
    CHECK(eng.rule_count() == 1 + sky_count,
          "expected " + std::to_string(1 + sky_count) + " rules, got " +
          std::to_string(eng.rule_count()));

    std::remove(tmpfile.c_str());
    PASS("json_append_mode");
}

// ===================================================================
//  Rule Management
// ===================================================================

// 9. rule_enable_disable
TEST(rule_enable_disable) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    eng.load_sky130_rules();

    eng.disable_rule("m1.1");
    auto* r1 = find_rule(eng.rules(), "m1.1");
    CHECK(r1 && r1->enabled == false, "m1.1 should be disabled");

    eng.enable_rule("m1.1", true);
    auto* r2 = find_rule(eng.rules(), "m1.1");
    CHECK(r2 && r2->enabled == true, "m1.1 should be re-enabled");
    PASS("rule_enable_disable");
}

// 10. rule_severity_levels
TEST(rule_severity_levels) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);

    DrcRule err_rule;
    err_rule.name = "sev.err"; err_rule.value = 0.1;
    err_rule.type = DrcRule::MIN_WIDTH; err_rule.layer = DrcLayer::MET1;
    err_rule.severity = DrcRule::ERROR;
    eng.add_rule(err_rule);

    DrcRule warn_rule;
    warn_rule.name = "sev.warn"; warn_rule.value = 0.1;
    warn_rule.type = DrcRule::MIN_SPACING; warn_rule.layer = DrcLayer::MET2;
    warn_rule.severity = DrcRule::WARNING;
    eng.add_rule(warn_rule);

    DrcRule info_rule;
    info_rule.name = "sev.info"; info_rule.value = 0.1;
    info_rule.type = DrcRule::MIN_AREA; info_rule.layer = DrcLayer::MET3;
    info_rule.severity = DrcRule::INFO;
    eng.add_rule(info_rule);

    CHECK(eng.rule_count() == 3, "expected 3 rules");
    CHECK(eng.rules()[0].severity == DrcRule::ERROR,   "rule 0 severity != ERROR");
    CHECK(eng.rules()[1].severity == DrcRule::WARNING, "rule 1 severity != WARNING");
    CHECK(eng.rules()[2].severity == DrcRule::INFO,    "rule 2 severity != INFO");
    PASS("rule_severity_levels");
}

// ===================================================================
//  DRC Checking
// ===================================================================

// 11. drc_clean_design
TEST(drc_clean_design) {
    auto pd = build_clean_design();
    DrcEngine eng(pd);
    eng.load_default_rules(0.01); // very relaxed
    auto result = eng.check();
    CHECK(result.violations == 0,
          "clean design should have 0 violations, got " +
          std::to_string(result.violations));
    PASS("drc_clean_design");
}

// 12. drc_boundary_violation
TEST(drc_boundary_violation) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 50, 50);
    int c0 = pd.add_cell("c_out", "inv", 5.0, 5.0);
    pd.cells[c0].position = Point(100, 100); // way outside die
    pd.cells[c0].placed = true;

    DrcEngine eng(pd);
    eng.load_default_rules();
    auto result = eng.check();
    CHECK(result.errors > 0,
          "cell outside die should produce errors, got " +
          std::to_string(result.errors));
    PASS("drc_boundary_violation");
}

// 13. drc_width_violation
TEST(drc_width_violation) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    WireSegment w;
    w.layer = DrcLayer::MET1;
    w.start = Point(10, 10);
    w.end   = Point(50, 10);
    w.width = 0.05; // well below MET1 min_width of 0.140
    w.net_id = 0;
    pd.wires.push_back(w);

    DrcEngine eng(pd);
    eng.load_sky130_rules();
    auto result = eng.check();
    CHECK(result.violations > 0,
          "narrow wire should cause violations, got " +
          std::to_string(result.violations));
    PASS("drc_width_violation");
}

// 14. drc_spacing_violation
TEST(drc_spacing_violation) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);

    // Two parallel MET1 wires with edge-to-edge spacing < 0.140um
    // Wire 1: centerline y=10, width=0.2 → edges y=9.9..10.1
    // Wire 2: centerline y=10.25, width=0.2 → edges y=10.15..10.35
    // Edge-to-edge gap = 10.15 - 10.1 = 0.05um (< 0.140 required)
    WireSegment w1;
    w1.layer = DrcLayer::MET1;
    w1.start = Point(10, 10);
    w1.end   = Point(50, 10);
    w1.width = 0.2;
    w1.net_id = 0;
    pd.wires.push_back(w1);

    WireSegment w2;
    w2.layer = DrcLayer::MET1;
    w2.start = Point(10, 10.25);
    w2.end   = Point(50, 10.25);
    w2.width = 0.2;
    w2.net_id = 1;
    pd.wires.push_back(w2);

    DrcEngine eng(pd);
    eng.load_sky130_rules();
    auto result = eng.check();
    CHECK(result.violations > 0,
          "close wires should cause violations, got " +
          std::to_string(result.violations));
    PASS("drc_spacing_violation");
}

// 15. drc_result_timing
TEST(drc_result_timing) {
    auto pd = build_clean_design();
    DrcEngine eng(pd);
    eng.load_default_rules();
    auto result = eng.check();
    CHECK(result.time_ms >= 0,
          "time_ms should be >= 0, got " + std::to_string(result.time_ms));
    PASS("drc_result_timing");
}

// ===================================================================
//  Error Handling
// ===================================================================

// 16. bad_json_file
TEST(bad_json_file) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    int loaded = eng.load_rules_from_file("/tmp/sf_nonexistent_file_xyz.json");
    CHECK(loaded == 0, "nonexistent file should return 0 rules, got " +
          std::to_string(loaded));
    CHECK(eng.rule_count() == 0, "rule_count should be 0");
    PASS("bad_json_file");
}

// 17. invalid_json_content
TEST(invalid_json_content) {
    const std::string tmpfile = "/tmp/sf_test_bad_json.json";
    {
        std::ofstream ofs(tmpfile);
        ofs << "this is not json at all!!!";
    }

    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    int loaded = eng.load_rules_from_file(tmpfile);
    CHECK(loaded == 0, "invalid JSON should return 0 rules, got " +
          std::to_string(loaded));

    std::remove(tmpfile.c_str());
    PASS("invalid_json_content");
}

// 18. clear_rules_works
TEST(clear_rules_works) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    DrcEngine eng(pd);
    eng.load_sky130_rules();
    CHECK(eng.rule_count() > 0, "should have rules after load");
    eng.clear_rules();
    CHECK(eng.rule_count() == 0,
          "after clear, rule_count should be 0, got " +
          std::to_string(eng.rule_count()));
    PASS("clear_rules_works");
}

// ===================================================================
// Main
// ===================================================================
int main() {
    std::cout << "\n"
        "╔══════════════════════════════════════════════════╗\n"
        "║  SiliconForge Phase 17 — DRC Industrial Tests     ║\n"
        "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── SKY130 Rule Deck ──\n";
    RUN(sky130_rule_count);
    RUN(sky130_key_values);
    RUN(sky130_layer_assignments);
    RUN(sky130_conditional_rules);
    RUN(sky130_all_rule_categories);

    std::cout << "\n── JSON I/O ──\n";
    RUN(json_roundtrip);
    RUN(json_roundtrip_conditional);
    RUN(json_append_mode);

    std::cout << "\n── Rule Management ──\n";
    RUN(rule_enable_disable);
    RUN(rule_severity_levels);

    std::cout << "\n── DRC Checking ──\n";
    RUN(drc_clean_design);
    RUN(drc_boundary_violation);
    RUN(drc_width_violation);
    RUN(drc_spacing_violation);
    RUN(drc_result_timing);

    std::cout << "\n── Error Handling ──\n";
    RUN(bad_json_file);
    RUN(invalid_json_content);
    RUN(clear_rules_works);

    std::cout << "\n════════════════════════════════════════\n";
    std::cout << "Phase 17 Results: " << passed << " passed, "
              << failed << " failed\n";
    std::cout << "════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
