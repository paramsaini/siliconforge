// SiliconForge — Phase 43: Advanced Clock Tree Synthesis Tests
// 25 tests: H-tree, shielding, slew-driven, ECO CTS, derived clocks,
// integration with multi-clock, edge cases

#include "pnr/cts.hpp"
#include "pnr/physical.hpp"
#include <iostream>
#include <cmath>
#include <string>

static int tests_run = 0, tests_passed = 0;

#define RUN(name) do { \
    tests_run++; \
    std::cout << "  [" << tests_run << "] " << name << "... "; \
    std::cout.flush(); \
} while(0)

#define PASS() do { \
    tests_passed++; \
    std::cout << "PASS\n"; \
} while(0)

#define CHECK(cond) do { \
    if (!(cond)) { \
        std::cout << "FAIL (" #cond ") at line " << __LINE__ << "\n"; \
        return; \
    } \
} while(0)

#define CHECKF(val, lo, hi) do { \
    double v_ = (val); \
    if (v_ < (lo) || v_ > (hi)) { \
        std::cout << "FAIL (" #val "=" << v_ << " not in [" << (lo) << "," << (hi) << "]) line " << __LINE__ << "\n"; \
        return; \
    } \
} while(0)

// Helper: create a physical design with placed cells
static sf::PhysicalDesign make_test_design(int ncells, double die_size = 10000) {
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, die_size, die_size);
    pd.row_height = 3.0;
    int grid_n = (int)std::ceil(std::sqrt((double)ncells));
    double step = die_size / (grid_n + 1);
    for (int i = 0; i < ncells; i++) {
        int c = pd.add_cell("ff_" + std::to_string(i), "DFF", 3, 3);
        pd.cells[c].position = sf::Point(step * (1 + i % grid_n),
                                          step * (1 + i / grid_n));
        pd.cells[c].placed = true;
    }
    return pd;
}

static std::vector<int> all_cells(int n) {
    std::vector<int> v(n);
    for (int i = 0; i < n; i++) v[i] = i;
    return v;
}

// ── Test 1: H-tree basic construction ───────────────────────────────────────
void test_htree_basic() {
    RUN("H-tree basic construction (4 sinks)");
    auto pd = make_test_design(4);
    sf::CtsEngine cts(pd);

    sf::HtreeConfig cfg;
    cfg.levels = 2;
    auto result = cts.build_htree(sf::Point(5000, 5000), all_cells(4), cfg);

    CHECK(result.buffers_inserted >= 0);
    CHECK(result.wirelength > 0);
    CHECK(result.skew >= 0);
    CHECK(result.message.find("H-tree") != std::string::npos);
    PASS();
}

// ── Test 2: H-tree multi-level ──────────────────────────────────────────────
void test_htree_levels() {
    RUN("H-tree multi-level (16 sinks, 4 levels)");
    auto pd = make_test_design(16);
    sf::CtsEngine cts(pd);

    sf::HtreeConfig cfg;
    cfg.levels = 4;
    auto result = cts.build_htree(sf::Point(5000, 5000), all_cells(16), cfg);

    CHECK(result.buffers_inserted >= 1);
    CHECK(cts.tree_size() > 16); // internal nodes created
    PASS();
}

// ── Test 3: H-tree wire width ───────────────────────────────────────────────
void test_htree_wire_width() {
    RUN("H-tree with wider wires");
    auto pd = make_test_design(8);
    sf::CtsEngine cts(pd);

    sf::HtreeConfig cfg;
    cfg.levels = 3;
    cfg.wire_width_um = 0.5; // wider for lower R

    auto result = cts.build_htree(sf::Point(5000, 5000), all_cells(8), cfg);
    CHECK(result.wirelength > 0);
    CHECK(cts.clock_wire_count() > 0);
    PASS();
}

// ── Test 4: H-tree vs DME comparison ────────────────────────────────────────
void test_htree_vs_dme() {
    RUN("H-tree vs DME comparison");
    auto pd1 = make_test_design(8);
    sf::CtsEngine cts1(pd1);
    auto dme_result = cts1.build_clock_tree(sf::Point(5000, 5000), all_cells(8));

    auto pd2 = make_test_design(8);
    sf::CtsEngine cts2(pd2);
    sf::HtreeConfig cfg;
    cfg.levels = 3;
    auto htree_result = cts2.build_htree(sf::Point(5000, 5000), all_cells(8), cfg);

    // Both should produce valid trees
    CHECK(dme_result.wirelength > 0);
    CHECK(htree_result.wirelength > 0);
    CHECK(dme_result.skew >= 0);
    CHECK(htree_result.skew >= 0);
    PASS();
}

// ── Test 5: Clock shielding basic ───────────────────────────────────────────
void test_shielding_basic() {
    RUN("Clock shielding basic");
    auto pd = make_test_design(4);
    sf::CtsEngine cts(pd);

    sf::HtreeConfig hcfg;
    hcfg.levels = 2;
    cts.build_htree(sf::Point(5000, 5000), all_cells(4), hcfg);

    int wires_before = (int)pd.wires.size();
    int shields = cts.apply_clock_shielding();

    CHECK(shields > 0);
    CHECK((int)pd.wires.size() > wires_before);
    // Each clock wire gets 2 shields (VDD + GND)
    CHECK(shields == cts.clock_wire_count() * 2);
    PASS();
}

// ── Test 6: Clock shielding VDD only ────────────────────────────────────────
void test_shielding_vdd_only() {
    RUN("Clock shielding VDD only");
    auto pd = make_test_design(4);
    sf::CtsEngine cts(pd);

    sf::HtreeConfig hcfg;
    hcfg.levels = 2;
    cts.build_htree(sf::Point(5000, 5000), all_cells(4), hcfg);

    sf::ClkShieldConfig scfg;
    scfg.shield_vdd = true;
    scfg.shield_gnd = false;
    int shields = cts.apply_clock_shielding(scfg);

    CHECK(shields == cts.clock_wire_count()); // only VDD side
    PASS();
}

// ── Test 7: Coupling reduction factor ───────────────────────────────────────
void test_coupling_reduction() {
    RUN("Coupling reduction factor");
    auto pd = make_test_design(1);
    sf::CtsEngine cts(pd);

    sf::ClkShieldConfig cfg;
    cfg.coupling_cap_ff_per_um = 0.02;
    cfg.shielded_cap_ff_per_um = 0.005;

    double factor = cts.coupling_reduction_factor(cfg);
    CHECKF(factor, 0.24, 0.26); // 0.005/0.02 = 0.25
    PASS();
}

// ── Test 8: Slew-driven basic ───────────────────────────────────────────────
void test_slew_driven_basic() {
    RUN("Slew-driven tree basic");
    auto pd = make_test_design(8);
    sf::CtsEngine cts(pd);

    sf::SlewConfig cfg;
    cfg.max_slew_ps = 100.0;

    auto result = cts.build_slew_driven_tree(sf::Point(5000, 5000), all_cells(8), cfg);
    CHECK(result.wirelength > 0);
    CHECK(result.buffers_inserted >= 0);
    CHECK(result.message.find("Slew-driven") != std::string::npos);
    PASS();
}

// ── Test 9: Slew-driven with tight constraint ──────────────────────────────
void test_slew_driven_tight() {
    RUN("Slew-driven tree (tight slew constraint)");
    auto pd = make_test_design(16);
    sf::CtsEngine cts(pd);

    sf::SlewConfig cfg;
    cfg.max_slew_ps = 50.0; // very tight

    auto result = cts.build_slew_driven_tree(sf::Point(5000, 5000), all_cells(16), cfg);
    // Should insert more buffers with tight constraint
    CHECK(result.buffers_inserted >= 1);
    PASS();
}

// ── Test 10: Slew vs relaxed comparison ─────────────────────────────────────
void test_slew_tight_vs_relaxed() {
    RUN("Slew-driven: tight vs relaxed constraint");
    auto pd1 = make_test_design(12);
    sf::CtsEngine cts1(pd1);
    sf::SlewConfig tight;
    tight.max_slew_ps = 30.0;
    auto r1 = cts1.build_slew_driven_tree(sf::Point(5000, 5000), all_cells(12), tight);

    auto pd2 = make_test_design(12);
    sf::CtsEngine cts2(pd2);
    sf::SlewConfig relaxed;
    relaxed.max_slew_ps = 500.0;
    auto r2 = cts2.build_slew_driven_tree(sf::Point(5000, 5000), all_cells(12), relaxed);

    // Tight constraint should use at least as many buffers
    CHECK(r1.buffers_inserted >= r2.buffers_inserted);
    PASS();
}

// ── Test 11: Slew at node query ─────────────────────────────────────────────
void test_slew_at_node() {
    RUN("Slew at node query");
    auto pd = make_test_design(4);
    sf::CtsEngine cts(pd);

    sf::SlewConfig cfg;
    auto result = cts.build_slew_driven_tree(sf::Point(5000, 5000), all_cells(4), cfg);

    // Query slew at root
    double slew = cts.compute_slew_at_node(cts.tree_size() - 1, cfg);
    CHECK(slew >= 0);
    PASS();
}

// ── Test 12: ECO add sinks ─────────────────────────────────────────────────
void test_eco_add_sinks() {
    RUN("ECO CTS: add sinks");
    auto pd = make_test_design(10);
    sf::CtsEngine cts(pd);

    sf::ClockDomain dom;
    dom.name = "clk_main";
    dom.source = sf::Point(5000, 5000);
    dom.sink_cells = {0, 1, 2, 3, 4};

    sf::CtsConfig ccfg;
    ccfg.domains = {dom};
    cts.build_clock_tree(dom.source, dom.sink_cells);

    // ECO: add 3 more sinks
    sf::EcoCtsRequest req;
    req.domain_name = "clk_main";
    req.add_sinks = {5, 6, 7};

    auto result = cts.apply_eco(req, ccfg);
    CHECK(result.sinks_added == 3);
    CHECK(result.buffers_added > 0);
    CHECK(result.skew_after >= 0);
    PASS();
}

// ── Test 13: ECO remove sinks ───────────────────────────────────────────────
void test_eco_remove_sinks() {
    RUN("ECO CTS: remove sinks");
    auto pd = make_test_design(8);
    sf::CtsEngine cts(pd);

    sf::ClockDomain dom;
    dom.name = "clk";
    dom.source = sf::Point(5000, 5000);
    dom.sink_cells = all_cells(8);

    sf::CtsConfig ccfg;
    ccfg.domains = {dom};
    cts.build_clock_tree(dom.source, dom.sink_cells);

    sf::EcoCtsRequest req;
    req.domain_name = "clk";
    req.remove_sinks = {2, 5};

    auto result = cts.apply_eco(req, ccfg);
    CHECK(result.sinks_removed == 2);
    CHECK(result.message.find("ECO") != std::string::npos);
    PASS();
}

// ── Test 14: ECO add + remove ───────────────────────────────────────────────
void test_eco_add_remove() {
    RUN("ECO CTS: simultaneous add and remove");
    auto pd = make_test_design(12);
    sf::CtsEngine cts(pd);

    sf::ClockDomain dom;
    dom.name = "clk";
    dom.source = sf::Point(5000, 5000);
    dom.sink_cells = {0, 1, 2, 3, 4, 5};

    sf::CtsConfig ccfg;
    ccfg.domains = {dom};
    cts.build_clock_tree(dom.source, dom.sink_cells);

    sf::EcoCtsRequest req;
    req.domain_name = "clk";
    req.add_sinks = {8, 9, 10};
    req.remove_sinks = {1, 3};

    auto result = cts.apply_eco(req, ccfg);
    CHECK(result.sinks_added == 3);
    CHECK(result.sinks_removed == 2);
    PASS();
}

// ── Test 15: ECO invalid domain ─────────────────────────────────────────────
void test_eco_invalid_domain() {
    RUN("ECO CTS: invalid domain name");
    auto pd = make_test_design(4);
    sf::CtsEngine cts(pd);

    sf::CtsConfig ccfg;
    sf::EcoCtsRequest req;
    req.domain_name = "nonexistent";
    req.add_sinks = {0};

    auto result = cts.apply_eco(req, ccfg);
    CHECK(result.sinks_added == 0);
    CHECK(result.message.find("not found") != std::string::npos);
    PASS();
}

// ── Test 16: Derived clock basic ────────────────────────────────────────────
void test_derived_clock_basic() {
    RUN("Derived clock (÷2)");
    auto pd = make_test_design(8);
    sf::CtsEngine cts(pd);

    sf::ClockDomain parent;
    parent.name = "clk_sys";
    parent.source = sf::Point(5000, 5000);
    parent.period_ps = 1000.0;
    parent.uncertainty_ps = 30.0;

    sf::ClockDomain derived;
    derived.name = "clk_slow";
    derived.source = sf::Point(5000, 5000);
    derived.period_ps = 2000.0;
    derived.uncertainty_ps = 10.0;
    derived.is_generated = true;
    derived.parent_clock = "clk_sys";
    derived.divide_by = 2;
    derived.sink_cells = {4, 5, 6, 7};

    auto result = cts.build_derived_clock(derived, parent);

    CHECK(result.effective_period_ps == 2000.0); // 1000 × 2
    CHECKF(result.cumulative_uncertainty_ps, 31.0, 32.0); // sqrt(30² + 10²) ≈ 31.6
    CHECK(result.divider_cells_inserted == 1);
    CHECK(result.phase_shift_ps == 500.0); // half parent period
    CHECK(result.tree_result.wirelength > 0);
    PASS();
}

// ── Test 17: Derived clock ÷4 ──────────────────────────────────────────────
void test_derived_clock_div4() {
    RUN("Derived clock (÷4)");
    auto pd = make_test_design(6);
    sf::CtsEngine cts(pd);

    sf::ClockDomain parent;
    parent.name = "fast_clk";
    parent.source = sf::Point(3000, 3000);
    parent.period_ps = 500.0;
    parent.uncertainty_ps = 20.0;

    sf::ClockDomain derived;
    derived.name = "slow_clk";
    derived.source = sf::Point(3000, 3000);
    derived.divide_by = 4;
    derived.uncertainty_ps = 5.0;
    derived.sink_cells = {0, 1, 2};

    auto result = cts.build_derived_clock(derived, parent);
    CHECK(result.effective_period_ps == 2000.0); // 500 × 4
    CHECK(result.divider_cells_inserted == 1);
    PASS();
}

// ── Test 18: Derived clock no division ──────────────────────────────────────
void test_derived_clock_nodiv() {
    RUN("Derived clock (÷1 = same frequency)");
    auto pd = make_test_design(4);
    sf::CtsEngine cts(pd);

    sf::ClockDomain parent;
    parent.name = "clk";
    parent.source = sf::Point(5000, 5000);
    parent.period_ps = 1000.0;
    parent.uncertainty_ps = 20.0;

    sf::ClockDomain derived;
    derived.name = "clk_buf";
    derived.source = sf::Point(5000, 5000);
    derived.divide_by = 1;
    derived.uncertainty_ps = 5.0;
    derived.sink_cells = {0, 1};

    auto result = cts.build_derived_clock(derived, parent);
    CHECK(result.effective_period_ps == 1000.0);
    CHECK(result.divider_cells_inserted == 0); // no divider needed
    CHECK(result.phase_shift_ps == 0);
    PASS();
}

// ── Test 19: H-tree single sink ─────────────────────────────────────────────
void test_htree_single_sink() {
    RUN("H-tree with single sink");
    auto pd = make_test_design(1);
    sf::CtsEngine cts(pd);

    sf::HtreeConfig cfg;
    cfg.levels = 2;
    auto result = cts.build_htree(sf::Point(5000, 5000), {0}, cfg);
    CHECK(result.wirelength > 0);
    PASS();
}

// ── Test 20: H-tree empty sinks ─────────────────────────────────────────────
void test_htree_empty() {
    RUN("H-tree with no sinks");
    auto pd = make_test_design(0);
    sf::CtsEngine cts(pd);

    auto result = cts.build_htree(sf::Point(5000, 5000), {});
    CHECK(result.buffers_inserted == 0);
    CHECK(result.message.find("No sinks") != std::string::npos);
    PASS();
}

// ── Test 21: Shielding on empty tree ────────────────────────────────────────
void test_shielding_empty() {
    RUN("Shielding on empty tree (no clock wires)");
    auto pd = make_test_design(1);
    sf::CtsEngine cts(pd);

    int shields = cts.apply_clock_shielding();
    CHECK(shields == 0); // no clock wires to shield
    PASS();
}

// ── Test 22: Full pipeline: H-tree + shielding + slew ───────────────────────
void test_full_pipeline() {
    RUN("Full pipeline: H-tree + shielding + slew check");
    auto pd = make_test_design(16);
    sf::CtsEngine cts(pd);

    // Build H-tree
    sf::HtreeConfig hcfg;
    hcfg.levels = 4;
    auto htree_result = cts.build_htree(sf::Point(5000, 5000), all_cells(16), hcfg);
    CHECK(htree_result.wirelength > 0);

    // Apply shielding
    int shields = cts.apply_clock_shielding();
    CHECK(shields > 0);

    // Build slew-driven tree (separate instance to verify it also works)
    auto pd2 = make_test_design(16);
    sf::CtsEngine cts2(pd2);
    sf::SlewConfig scfg;
    scfg.max_slew_ps = 80.0;
    auto slew_result = cts2.build_slew_driven_tree(sf::Point(5000, 5000), all_cells(16), scfg);
    CHECK(slew_result.wirelength > 0);
    PASS();
}

// ── Test 23: Multi-clock with derived ───────────────────────────────────────
void test_multi_clock_derived() {
    RUN("Multi-clock with derived clock domain");
    auto pd = make_test_design(12);
    sf::CtsEngine cts(pd);

    sf::ClockDomain sys_clk;
    sys_clk.name = "sys_clk";
    sys_clk.source = sf::Point(5000, 5000);
    sys_clk.period_ps = 1000.0;
    sys_clk.uncertainty_ps = 30.0;
    sys_clk.sink_cells = {0, 1, 2, 3, 4, 5};

    sf::ClockDomain io_clk;
    io_clk.name = "io_clk";
    io_clk.source = sf::Point(5000, 5000);
    io_clk.period_ps = 2000.0;
    io_clk.uncertainty_ps = 10.0;
    io_clk.is_generated = true;
    io_clk.parent_clock = "sys_clk";
    io_clk.divide_by = 2;
    io_clk.sink_cells = {6, 7, 8, 9, 10, 11};

    // Build parent tree via multi-clock
    sf::CtsConfig ccfg;
    ccfg.domains = {sys_clk, io_clk};
    auto multi = cts.build_multi_clock(ccfg);

    CHECK(multi.domain_results.size() == 2);
    CHECK(multi.total_buffers >= 0);
    CHECK(multi.total_wirelength > 0);

    // Also build derived clock explicitly
    auto derived_result = cts.build_derived_clock(io_clk, sys_clk);
    CHECK(derived_result.effective_period_ps == 2000.0);
    CHECK(derived_result.divider_cells_inserted == 1);
    PASS();
}

// ── Test 24: ECO skew tracking ──────────────────────────────────────────────
void test_eco_skew_tracking() {
    RUN("ECO skew tracking (before/after)");
    auto pd = make_test_design(10);
    sf::CtsEngine cts(pd);

    sf::ClockDomain dom;
    dom.name = "clk";
    dom.source = sf::Point(5000, 5000);
    dom.sink_cells = {0, 1, 2, 3, 4};

    sf::CtsConfig ccfg;
    ccfg.domains = {dom};
    cts.build_clock_tree(dom.source, dom.sink_cells);

    sf::EcoCtsRequest req;
    req.domain_name = "clk";
    req.add_sinks = {8, 9}; // far-away sinks

    auto result = cts.apply_eco(req, ccfg);
    CHECK(result.skew_before >= 0);
    CHECK(result.skew_after >= 0);
    // Skew might change after adding sinks
    CHECK(std::isfinite(result.skew_after));
    PASS();
}

// ── Test 25: E2E advanced CTS ───────────────────────────────────────────────
void test_e2e_advanced_cts() {
    RUN("E2E: Advanced CTS full flow");

    auto pd = make_test_design(32);
    sf::CtsEngine cts(pd);

    // 1. Build H-tree for main clock
    sf::HtreeConfig hcfg;
    hcfg.levels = 5;
    auto h_result = cts.build_htree(sf::Point(5000, 5000), all_cells(32), hcfg);
    CHECK(h_result.wirelength > 0);

    // 2. Apply shielding
    int shields = cts.apply_clock_shielding();
    CHECK(shields > 0);

    // 3. Build slew-driven tree separately for comparison
    auto pd2 = make_test_design(32);
    sf::CtsEngine cts2(pd2);
    sf::SlewConfig scfg;
    scfg.max_slew_ps = 60.0;
    auto s_result = cts2.build_slew_driven_tree(sf::Point(5000, 5000), all_cells(32), scfg);
    CHECK(s_result.wirelength > 0);

    // 4. ECO: add 2 sinks to H-tree
    sf::ClockDomain dom;
    dom.name = "main";
    dom.source = sf::Point(5000, 5000);
    dom.sink_cells = all_cells(32);
    sf::CtsConfig ccfg;
    ccfg.domains = {dom};

    // Add extra cells for ECO
    int c1 = pd.add_cell("eco_ff_1", "DFF", 3, 3);
    pd.cells[c1].position = sf::Point(1000, 9000);
    pd.cells[c1].placed = true;
    int c2 = pd.add_cell("eco_ff_2", "DFF", 3, 3);
    pd.cells[c2].position = sf::Point(9000, 1000);
    pd.cells[c2].placed = true;

    sf::EcoCtsRequest req;
    req.domain_name = "main";
    req.add_sinks = {c1, c2};
    auto eco = cts.apply_eco(req, ccfg);
    CHECK(eco.sinks_added == 2);

    // 5. Derived clock
    sf::ClockDomain parent;
    parent.name = "main";
    parent.source = sf::Point(5000, 5000);
    parent.period_ps = 1000.0;
    parent.uncertainty_ps = 25.0;

    sf::ClockDomain slow;
    slow.name = "slow_clk";
    slow.source = sf::Point(5000, 5000);
    slow.divide_by = 4;
    slow.uncertainty_ps = 5.0;
    slow.sink_cells = {0, 1, 2, 3};

    auto derived = cts.build_derived_clock(slow, parent);
    CHECK(derived.effective_period_ps == 4000.0);
    CHECK(derived.divider_cells_inserted == 1);

    std::cout << "PASS [H-tree wl=" << (int)h_result.wirelength
              << ", shields=" << shields
              << ", slew_bufs=" << s_result.buffers_inserted
              << ", ECO +" << eco.sinks_added
              << ", derived T=" << derived.effective_period_ps << "ps]\n";
    tests_passed++;
}

// ── Main ────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << "  Phase 43: Advanced Clock Tree Synthesis Tests\n";
    std::cout << "═══════════════════════════════════════════════════════════\n\n";

    test_htree_basic();
    test_htree_levels();
    test_htree_wire_width();
    test_htree_vs_dme();
    test_shielding_basic();
    test_shielding_vdd_only();
    test_coupling_reduction();
    test_slew_driven_basic();
    test_slew_driven_tight();
    test_slew_tight_vs_relaxed();
    test_slew_at_node();
    test_eco_add_sinks();
    test_eco_remove_sinks();
    test_eco_add_remove();
    test_eco_invalid_domain();
    test_derived_clock_basic();
    test_derived_clock_div4();
    test_derived_clock_nodiv();
    test_htree_single_sink();
    test_htree_empty();
    test_shielding_empty();
    test_full_pipeline();
    test_multi_clock_derived();
    test_eco_skew_tracking();
    test_e2e_advanced_cts();

    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << "  Results: " << tests_passed << "/" << tests_run << " PASSED\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
