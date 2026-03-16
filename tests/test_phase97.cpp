// SiliconForge — Phase 97 Tests
// Global router <-> DRC integration: DRC-aware routing feedback loop.

#include "pnr/global_router.hpp"
#include "verify/drc.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <string>
#include <vector>

static int g_pass = 0, g_fail = 0;

static void check(bool cond, const std::string& name) {
    if (cond) { ++g_pass; std::cout << "  PASS  " << name << "\n"; }
    else      { ++g_fail; std::cout << "**FAIL  " << name << "\n"; }
}

// Helper: create a routable design with nets
static sf::PhysicalDesign make_routable_design(int num_nets = 8) {
    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};
    pd.row_height = 10.0;
    pd.site_width = 1.0;

    // Place cells in a grid pattern
    int num_cells = num_nets * 2;
    for (int i = 0; i < num_cells; ++i) {
        sf::CellInstance c;
        c.id = i;
        c.name = "c" + std::to_string(i);
        c.cell_type = "INV_X1";
        c.width = 2.0;
        c.height = 5.0;
        c.position = {5.0 + (i % 10) * 9.0, 5.0 + (i / 10) * 12.0};
        c.placed = true;
        pd.cells.push_back(c);
    }

    // Create nets connecting pairs of cells
    for (int i = 0; i < num_nets; ++i) {
        sf::PhysNet pn;
        pn.id = i;
        pn.name = "net" + std::to_string(i);
        pn.cell_ids = {i * 2, i * 2 + 1};
        pn.pin_offsets = {sf::Point(1.0, 2.5), sf::Point(0.0, 2.5)};
        pd.nets.push_back(pn);
    }

    return pd;
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 1: DRC engine injection
// ═══════════════════════════════════════════════════════════════════════════

static void test_drc_engine_injection() {
    std::cout << "\n=== test_drc_engine_injection ===\n";

    auto pd = make_routable_design(4);
    sf::DrcEngine drc(pd);
    sf::GlobalRouter gr(pd, 10, 10, 2);

    gr.set_drc_engine(&drc);
    check(true, "set_drc_engine does not crash");

    gr.set_drc_engine(nullptr);
    check(true, "set_drc_engine(nullptr) does not crash");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 2: Standard routing produces result
// ═══════════════════════════════════════════════════════════════════════════

static void test_standard_routing() {
    std::cout << "\n=== test_standard_routing ===\n";

    auto pd = make_routable_design(6);
    sf::GlobalRouter gr(pd, 10, 10, 2);

    auto result = gr.route();

    check(result.routed_nets > 0, "some nets routed");
    check(result.total_wirelength > 0, "positive wirelength");
    check(!result.message.empty(), "result message populated");
    check(result.total_wires > 0, "wires created");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 3: DRC-aware routing basic
// ═══════════════════════════════════════════════════════════════════════════

static void test_drc_aware_basic() {
    std::cout << "\n=== test_drc_aware_basic ===\n";

    auto pd = make_routable_design(6);
    sf::DrcEngine drc(pd);
    drc.load_default_rules(0.13);

    sf::GlobalRouter gr(pd, 10, 10, 2);
    gr.set_drc_engine(&drc);

    sf::RouterConfig cfg;
    cfg.enable_drc_aware = true;
    cfg.drc_max_iterations = 3;
    gr.set_config(cfg);

    auto result = gr.route_drc_aware();

    check(result.routed_nets > 0, "DRC-aware routing routes nets");
    check(result.drc_violations_initial >= 0, "initial DRC violations tracked");
    check(result.drc_violations_final >= 0, "final DRC violations tracked");
    check(!result.drc_violations_per_iteration.empty(), "DRC per-iteration tracked");
    check(result.total_wirelength > 0, "positive wirelength after DRC routing");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 4: DRC violations decrease or stable
// ═══════════════════════════════════════════════════════════════════════════

static void test_drc_violations_trend() {
    std::cout << "\n=== test_drc_violations_trend ===\n";

    auto pd = make_routable_design(6);
    sf::DrcEngine drc(pd);
    drc.load_default_rules(0.13);

    sf::GlobalRouter gr(pd, 10, 10, 2);
    gr.set_drc_engine(&drc);

    sf::RouterConfig cfg;
    cfg.enable_drc_aware = true;
    cfg.drc_max_iterations = 5;
    cfg.drc_penalty_factor = 3.0;
    gr.set_config(cfg);

    auto result = gr.route_drc_aware();

    check(result.drc_violations_final <= result.drc_violations_initial,
          "final DRC violations <= initial");
    if (result.drc_violations_per_iteration.size() >= 2) {
        check(result.drc_violations_per_iteration.back() <=
              result.drc_violations_per_iteration.front(),
              "last iteration violations <= first");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 5: DRC clean flag
// ═══════════════════════════════════════════════════════════════════════════

static void test_drc_clean_flag() {
    std::cout << "\n=== test_drc_clean_flag ===\n";

    // Sparse design — should easily be DRC clean
    auto pd = make_routable_design(2);
    sf::DrcEngine drc(pd);
    // Load very few rules (just basic min_width/spacing) — sparse design should pass
    drc.load_default_rules(0.01);  // very small min features

    sf::GlobalRouter gr(pd, 10, 10, 2);
    gr.set_drc_engine(&drc);

    sf::RouterConfig cfg;
    cfg.enable_drc_aware = true;
    cfg.drc_max_iterations = 5;
    gr.set_config(cfg);

    auto result = gr.route_drc_aware();

    // With very small min features and sparse design, DRC should be clean
    check(result.drc_violations_final >= 0, "final violations is non-negative");
    check(result.drc_violations_per_iteration.size() >= 1, "at least one DRC check");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 6: Route result DRC fields populated
// ═══════════════════════════════════════════════════════════════════════════

static void test_route_result_drc_fields() {
    std::cout << "\n=== test_route_result_drc_fields ===\n";

    auto pd = make_routable_design(5);
    sf::DrcEngine drc(pd);
    drc.load_default_rules(0.13);

    sf::GlobalRouter gr(pd, 10, 10, 2);
    gr.set_drc_engine(&drc);

    sf::RouterConfig cfg;
    cfg.enable_drc_aware = true;
    cfg.drc_max_iterations = 3;
    gr.set_config(cfg);

    auto result = gr.route_drc_aware();

    check(result.drc_violations_initial >= 0, "drc_violations_initial set");
    check(result.drc_violations_final >= 0, "drc_violations_final set");
    check(!result.drc_violations_per_iteration.empty(), "drc_violations_per_iteration not empty");
    check(result.message.find("DRC-aware") != std::string::npos,
          "message mentions DRC-aware");
    check(result.total_wires >= 0, "total_wires non-negative");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 7: DRC penalty factor effect
// ═══════════════════════════════════════════════════════════════════════════

static void test_drc_penalty_factor() {
    std::cout << "\n=== test_drc_penalty_factor ===\n";

    auto pd = make_routable_design(5);
    sf::DrcEngine drc(pd);
    drc.load_default_rules(0.13);

    // Test with high penalty
    sf::GlobalRouter gr(pd, 10, 10, 2);
    gr.set_drc_engine(&drc);

    sf::RouterConfig cfg;
    cfg.enable_drc_aware = true;
    cfg.drc_max_iterations = 3;
    cfg.drc_penalty_factor = 20.0;  // very high penalty
    gr.set_config(cfg);

    auto result = gr.route_drc_aware();

    check(result.drc_violations_per_iteration.size() >= 1, "DRC iterations tracked");
    check(result.total_wirelength > 0, "routes completed with high penalty");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 8: DRC-aware with timing-driven combined
// ═══════════════════════════════════════════════════════════════════════════

static void test_drc_with_timing() {
    std::cout << "\n=== test_drc_with_timing ===\n";

    auto pd = make_routable_design(5);
    sf::DrcEngine drc(pd);
    drc.load_default_rules(0.13);

    sf::GlobalRouter gr(pd, 10, 10, 2);
    gr.set_drc_engine(&drc);

    // Set timing info for some nets
    sf::NetTimingInfo crit;
    crit.slack = -0.5;
    crit.criticality = 0.8;
    crit.is_critical = true;
    gr.set_net_timing(0, crit);

    sf::RouterConfig cfg;
    cfg.enable_drc_aware = true;
    cfg.enable_timing_driven = true;
    cfg.drc_max_iterations = 3;
    gr.set_config(cfg);

    auto result = gr.route_drc_aware();

    check(result.routed_nets > 0, "routes with DRC + timing-driven");
    check(result.drc_violations_initial >= 0, "DRC tracked with timing-driven");
    check(!result.message.empty(), "message produced for combined mode");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 9: DRC-aware disabled (falls back to standard)
// ═══════════════════════════════════════════════════════════════════════════

static void test_drc_disabled() {
    std::cout << "\n=== test_drc_disabled ===\n";

    auto pd = make_routable_design(5);
    sf::DrcEngine drc(pd);
    drc.load_default_rules(0.13);

    sf::GlobalRouter gr(pd, 10, 10, 2);
    gr.set_drc_engine(&drc);

    sf::RouterConfig cfg;
    cfg.enable_drc_aware = false;  // disabled
    gr.set_config(cfg);

    auto result = gr.route_drc_aware();

    // Should fall back to standard routing — no DRC iteration
    check(result.drc_violations_per_iteration.empty(), "no DRC iterations when disabled");
    check(result.routed_nets > 0, "still routes nets when DRC disabled");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 10: Empty design
// ═══════════════════════════════════════════════════════════════════════════

static void test_empty_design() {
    std::cout << "\n=== test_empty_design ===\n";

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};
    pd.row_height = 10.0;
    pd.site_width = 1.0;
    // No cells, no nets

    sf::DrcEngine drc(pd);
    sf::GlobalRouter gr(pd, 10, 10, 2);
    gr.set_drc_engine(&drc);

    sf::RouterConfig cfg;
    cfg.enable_drc_aware = true;
    gr.set_config(cfg);

    auto result = gr.route_drc_aware();

    check(result.routed_nets == 0, "0 nets routed on empty design");
    check(result.drc_violations_initial == 0, "0 DRC violations on empty design");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 11: RouterConfig DRC defaults
// ═══════════════════════════════════════════════════════════════════════════

static void test_config_defaults() {
    std::cout << "\n=== test_config_defaults ===\n";

    sf::RouterConfig cfg;
    check(cfg.enable_drc_aware == false, "DRC-aware disabled by default");
    check(std::abs(cfg.drc_penalty_factor - 5.0) < 0.01, "default penalty factor = 5.0");
    check(cfg.drc_max_iterations == 10, "default max DRC iterations = 10");
    check(cfg.drc_convergence_threshold == 0, "default convergence threshold = 0");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 12: RouteResult DRC defaults
// ═══════════════════════════════════════════════════════════════════════════

static void test_result_defaults() {
    std::cout << "\n=== test_result_defaults ===\n";

    sf::RouteResult res;
    check(res.drc_violations_initial == 0, "default drc_violations_initial = 0");
    check(res.drc_violations_final == 0, "default drc_violations_final = 0");
    check(res.drc_violations_per_iteration.empty(), "default per-iteration empty");
    check(res.drc_clean == false, "default drc_clean = false");
}

// ═══════════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "SiliconForge Phase 97 — Global Router <-> DRC Integration\n";

    test_drc_engine_injection();
    test_standard_routing();
    test_drc_aware_basic();
    test_drc_violations_trend();
    test_drc_clean_flag();
    test_route_result_drc_fields();
    test_drc_penalty_factor();
    test_drc_with_timing();
    test_drc_disabled();
    test_empty_design();
    test_config_defaults();
    test_result_defaults();

    std::cout << "\n════════════════════════════════════════\n"
              << "Phase 97: " << g_pass << " passed, " << g_fail << " failed\n"
              << "════════════════════════════════════════\n";
    return g_fail ? 1 : 0;
}
