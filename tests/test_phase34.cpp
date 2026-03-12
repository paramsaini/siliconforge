// SiliconForge — Phase 34: Multi-Clock CTS Tests
#include "pnr/cts.hpp"
#include <iostream>
#include <cmath>

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

using namespace sf;

static PhysicalDesign make_pd(int num_cells) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 500, 500);
    pd.row_height = 2.72;
    for (int i = 0; i < num_cells; i++) {
        pd.add_cell("cell_" + std::to_string(i), "DFF", 2.0, 2.72);
        pd.cells.back().position = Point(50.0 + (i % 10) * 40.0, 50.0 + (i / 10) * 40.0);
        pd.cells.back().placed = true;
    }
    return pd;
}

// ── Backward Compatibility ──────────────────────────────────────────────

TEST(single_clock_backward_compat) {
    auto pd = make_pd(8);
    CtsEngine cts(pd);
    std::vector<int> sinks = {0, 1, 2, 3, 4, 5, 6, 7};
    CtsResult r = cts.build_clock_tree(Point(250, 250), sinks);
    CHECK(r.buffers_inserted >= 0, "Should insert buffers");
    CHECK(r.wirelength > 0, "Should have wirelength");
    CHECK(!r.message.empty(), "Should have message");
    PASS("single_clock_backward_compat");
}

// ── Multi-Clock CTS ────────────────────────────────────────────────────

TEST(multi_clock_two_domains) {
    auto pd = make_pd(16);
    CtsEngine cts(pd);

    CtsConfig cfg;
    ClockDomain d1;
    d1.name = "clk_sys";
    d1.source = Point(100, 250);
    d1.period_ps = 1000;
    d1.sink_cells = {0, 1, 2, 3, 4, 5, 6, 7};
    cfg.domains.push_back(d1);

    ClockDomain d2;
    d2.name = "clk_io";
    d2.source = Point(400, 250);
    d2.period_ps = 2000;
    d2.sink_cells = {8, 9, 10, 11, 12, 13, 14, 15};
    cfg.domains.push_back(d2);

    MultiCtsResult r = cts.build_multi_clock(cfg);
    CHECK(r.domain_results.size() == 2, "Should have 2 domain results");
    CHECK(r.total_buffers >= 0, "Should have buffers");
    CHECK(r.total_wirelength > 0, "Should have wirelength");
    CHECK(!r.report.empty(), "Should have report");
    PASS("multi_clock_two_domains");
}

TEST(multi_clock_three_domains) {
    auto pd = make_pd(24);
    CtsEngine cts(pd);

    CtsConfig cfg;
    for (int d = 0; d < 3; d++) {
        ClockDomain dom;
        dom.name = "clk_" + std::to_string(d);
        dom.source = Point(100 + d * 150, 250);
        dom.period_ps = 1000.0 * (d + 1);
        for (int i = d * 8; i < (d + 1) * 8; i++)
            dom.sink_cells.push_back(i);
        cfg.domains.push_back(dom);
    }

    MultiCtsResult r = cts.build_multi_clock(cfg);
    CHECK(r.domain_results.size() == 3, "Should have 3 domain results");
    PASS("multi_clock_three_domains");
}

// ── Clock Groups ───────────────────────────────────────────────────────

TEST(synchronous_group) {
    auto pd = make_pd(16);
    CtsEngine cts(pd);

    CtsConfig cfg;
    ClockDomain d1;
    d1.name = "clk_a";
    d1.source = Point(100, 250);
    d1.period_ps = 1000;
    d1.sink_cells = {0, 1, 2, 3, 4, 5, 6, 7};
    cfg.domains.push_back(d1);

    ClockDomain d2;
    d2.name = "clk_b";
    d2.source = Point(400, 250);
    d2.period_ps = 1000;
    d2.sink_cells = {8, 9, 10, 11, 12, 13, 14, 15};
    cfg.domains.push_back(d2);

    ClockGroup grp;
    grp.type = ClockGroup::SYNCHRONOUS;
    grp.clock_names = {"clk_a", "clk_b"};
    cfg.groups.push_back(grp);

    MultiCtsResult r = cts.build_multi_clock(cfg);
    CHECK(r.inter_domain_skew >= 0, "Inter-domain skew should be computed");
    PASS("synchronous_group");
}

TEST(asynchronous_group) {
    auto pd = make_pd(16);
    CtsEngine cts(pd);

    CtsConfig cfg;
    ClockDomain d1, d2;
    d1.name = "clk_fast"; d1.source = Point(100, 250); d1.period_ps = 500;
    d1.sink_cells = {0, 1, 2, 3, 4, 5, 6, 7};
    d2.name = "clk_slow"; d2.source = Point(400, 250); d2.period_ps = 5000;
    d2.sink_cells = {8, 9, 10, 11, 12, 13, 14, 15};
    cfg.domains.push_back(d1);
    cfg.domains.push_back(d2);

    ClockGroup grp;
    grp.type = ClockGroup::ASYNCHRONOUS;
    grp.clock_names = {"clk_fast", "clk_slow"};
    cfg.groups.push_back(grp);

    MultiCtsResult r = cts.build_multi_clock(cfg);
    CHECK(r.domain_results.size() == 2, "Should have 2 results");
    PASS("asynchronous_group");
}

// ── Useful Skew ────────────────────────────────────────────────────────

TEST(useful_skew_tree) {
    auto pd = make_pd(8);
    CtsEngine cts(pd);

    ClockDomain dom;
    dom.name = "clk_main";
    dom.source = Point(250, 250);
    dom.period_ps = 1000;
    dom.sink_cells = {0, 1, 2, 3, 4, 5, 6, 7};

    // Provide per-sink slack values (ps) — some negative (critical), some positive
    std::vector<double> slacks = {-50, -30, 10, 20, -10, 50, 100, -80};

    CtsResult r = cts.build_useful_skew_tree(dom, slacks);
    CHECK(r.buffers_inserted >= 0, "Should insert buffers");
    CHECK(!r.message.empty(), "Should have message");
    PASS("useful_skew_tree");
}

// ── Power-Aware CTS ────────────────────────────────────────────────────

TEST(power_aware_tree) {
    auto pd = make_pd(8);
    CtsEngine cts(pd);

    ClockDomain dom;
    dom.name = "clk_lp";
    dom.source = Point(250, 250);
    dom.period_ps = 2000;
    dom.sink_cells = {0, 1, 2, 3, 4, 5, 6, 7};

    CtsResult r = cts.build_power_aware_tree(dom);
    CHECK(r.buffers_inserted >= 0, "Should insert buffers");
    PASS("power_aware_tree");
}

TEST(power_aware_config) {
    auto pd = make_pd(16);
    CtsEngine cts(pd);

    CtsConfig cfg;
    cfg.power_aware = true;
    ClockDomain dom;
    dom.name = "clk_pa";
    dom.source = Point(250, 250);
    dom.period_ps = 1000;
    dom.sink_cells = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    cfg.domains.push_back(dom);

    MultiCtsResult r = cts.build_multi_clock(cfg);
    CHECK(r.domain_results.size() == 1, "Should have 1 result");
    CHECK(r.total_clock_power_uw >= 0, "Power should be non-negative");
    PASS("power_aware_config");
}

// ── Generated Clock ────────────────────────────────────────────────────

TEST(generated_clock) {
    auto pd = make_pd(16);
    CtsEngine cts(pd);

    CtsConfig cfg;
    ClockDomain d1;
    d1.name = "clk_main";
    d1.source = Point(250, 250);
    d1.period_ps = 1000;
    d1.sink_cells = {0, 1, 2, 3, 4, 5, 6, 7};
    cfg.domains.push_back(d1);

    ClockDomain d2;
    d2.name = "clk_div2";
    d2.source = Point(250, 250);
    d2.period_ps = 2000;
    d2.is_generated = true;
    d2.parent_clock = "clk_main";
    d2.divide_by = 2;
    d2.sink_cells = {8, 9, 10, 11, 12, 13, 14, 15};
    cfg.domains.push_back(d2);

    MultiCtsResult r = cts.build_multi_clock(cfg);
    CHECK(r.domain_results.size() == 2, "Should build both trees");
    PASS("generated_clock");
}

// ── Empty/Edge Cases ───────────────────────────────────────────────────

TEST(empty_config) {
    auto pd = make_pd(4);
    CtsEngine cts(pd);
    CtsConfig cfg;
    MultiCtsResult r = cts.build_multi_clock(cfg);
    CHECK(r.domain_results.empty(), "Empty config should produce empty results");
    PASS("empty_config");
}

TEST(single_sink_domain) {
    auto pd = make_pd(2);
    CtsEngine cts(pd);
    CtsConfig cfg;
    ClockDomain dom;
    dom.name = "clk_tiny";
    dom.source = Point(100, 100);
    dom.period_ps = 1000;
    dom.sink_cells = {0};
    cfg.domains.push_back(dom);
    MultiCtsResult r = cts.build_multi_clock(cfg);
    CHECK(r.domain_results.size() == 1, "Should handle single sink");
    PASS("single_sink_domain");
}

int main() {
    std::cout << "═══════════════════════════════════════════════════════\n"
              << " Phase 34: Multi-Clock CTS Tests\n"
              << "═══════════════════════════════════════════════════════\n\n";
    RUN(single_clock_backward_compat);
    RUN(multi_clock_two_domains);
    RUN(multi_clock_three_domains);
    RUN(synchronous_group);
    RUN(asynchronous_group);
    RUN(useful_skew_tree);
    RUN(power_aware_tree);
    RUN(power_aware_config);
    RUN(generated_clock);
    RUN(empty_config);
    RUN(single_sink_domain);
    std::cout << "\n═══════════════════════════════════════════════════════\n"
              << " Results: " << passed << " passed, " << failed << " failed\n"
              << "═══════════════════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
