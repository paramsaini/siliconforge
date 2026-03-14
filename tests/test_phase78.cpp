// SiliconForge — Phase 78: Multi-Patterning (SADP/SAQP) Engine Tests
#include "../src/pnr/multi_pattern.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace sf;

static int tests_run = 0, tests_passed = 0;
#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { std::cerr << "FAIL: " << msg << " [" << __LINE__ << "]\n"; } \
    else { tests_passed++; } \
} while(0)

// ────────────────────────────────────────────────────────────────────
// Helpers: create horizontal wire at given y on layer 0
// ────────────────────────────────────────────────────────────────────

static WireSegment hwire(double x0, double y, double x1, double w,
                         int layer = 0, int net = 0) {
    WireSegment ws;
    ws.start = Point(x0, y);
    ws.end = Point(x1, y);
    ws.layer = layer;
    ws.width = w;
    ws.net_id = net;
    return ws;
}

// ────────────────────────────────────────────────────────────────────
// Test 1: Build conflict graph from parallel wires
// ────────────────────────────────────────────────────────────────────

static void test_conflict_graph() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};
    // Two parallel horizontal wires, spacing = 30 - 10 = 20 < min_space 40
    pd.wires.push_back(hwire(0, 100, 500, 10));  // y=100, w=10
    pd.wires.push_back(hwire(0, 130, 500, 10));  // y=130, w=10, gap=20

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 40.0;
    eng.set_config(cfg);

    auto assignments = eng.color_graph(0);
    CHECK(assignments.size() == 2, "conflict graph has 2 nodes");
    CHECK(assignments[0].color != assignments[1].color,
          "conflicting wires get different colors");
}

// ────────────────────────────────────────────────────────────────────
// Test 2: 2-color bipartite graph
// ────────────────────────────────────────────────────────────────────

static void test_2_color_bipartite() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};
    // 4 wires: 0-1 conflict, 2-3 conflict, 1-2 conflict (bipartite chain)
    pd.wires.push_back(hwire(0, 100, 500, 10));
    pd.wires.push_back(hwire(0, 130, 500, 10));
    pd.wires.push_back(hwire(0, 160, 500, 10));
    pd.wires.push_back(hwire(0, 190, 500, 10));

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 40.0;
    cfg.max_colors = 2;
    eng.set_config(cfg);

    auto assignments = eng.color_graph(0);
    CHECK(assignments.size() == 4, "4 wires in graph");
    // Chain should alternate A-B-A-B
    CHECK(assignments[0].color != assignments[1].color, "0-1 different");
    CHECK(assignments[1].color != assignments[2].color, "1-2 different");
    CHECK(assignments[2].color != assignments[3].color, "2-3 different");
}

// ────────────────────────────────────────────────────────────────────
// Test 3: 2-color fails on odd cycle (3 mutually conflicting wires)
// ────────────────────────────────────────────────────────────────────

static void test_2_color_odd_cycle() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};
    // Three wires all within min_space of each other → odd cycle
    pd.wires.push_back(hwire(0, 100, 500, 10));
    pd.wires.push_back(hwire(0, 115, 500, 10));  // gap=5
    pd.wires.push_back(hwire(0, 130, 500, 10));  // gap to wire0=20, gap to wire1=5

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 40.0;
    cfg.max_colors = 2;
    eng.set_config(cfg);

    // color_graph falls back to 4-color when 2-color fails
    auto assignments = eng.color_graph(0);
    CHECK(assignments.size() == 3, "3 wires");
    // All three should have distinct colors (needs >=3 colors)
    CHECK(assignments[0].color != assignments[1].color, "0-1 different");
    CHECK(assignments[1].color != assignments[2].color, "1-2 different");
    CHECK(assignments[0].color != assignments[2].color, "0-2 different");
}

// ────────────────────────────────────────────────────────────────────
// Test 4: 4-color succeeds on odd cycle
// ────────────────────────────────────────────────────────────────────

static void test_4_color() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};
    pd.wires.push_back(hwire(0, 100, 500, 10));
    pd.wires.push_back(hwire(0, 115, 500, 10));
    pd.wires.push_back(hwire(0, 130, 500, 10));

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 40.0;
    cfg.max_colors = 4;
    cfg.type = PatternType::SAQP;
    eng.set_config(cfg);

    auto res = eng.saqp_decompose(0);
    CHECK(res.decomposition_legal, "4-color resolves odd cycle");
    CHECK(res.wires_colored == 3, "3 wires colored");
}

// ────────────────────────────────────────────────────────────────────
// Test 5: SADP decomposition
// ────────────────────────────────────────────────────────────────────

static void test_sadp_decompose() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};
    pd.wires.push_back(hwire(0, 100, 500, 10, 0, 1));
    pd.wires.push_back(hwire(0, 150, 500, 10, 0, 2));  // gap=40, borderline

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 50.0;
    cfg.max_colors = 2;
    eng.set_config(cfg);

    auto res = eng.sadp_decompose(0);
    CHECK(res.wires_colored == 2, "SADP colors 2 wires");
    CHECK(res.decomposition_legal, "SADP legal for bipartite");
    CHECK(res.assignments[0].color != res.assignments[1].color,
          "mandrel vs spacer");
}

// ────────────────────────────────────────────────────────────────────
// Test 6: SAQP decomposition
// ────────────────────────────────────────────────────────────────────

static void test_saqp_decompose() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};
    // Five tightly-spaced wires
    for (int i = 0; i < 5; ++i) {
        pd.wires.push_back(hwire(0, 100 + i * 15, 500, 10));
    }

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 40.0;
    cfg.max_colors = 4;
    cfg.type = PatternType::SAQP;
    eng.set_config(cfg);

    auto res = eng.saqp_decompose(0);
    CHECK(res.wires_colored == 5, "SAQP colors 5 wires");
    CHECK(!res.report.empty(), "SAQP report generated");
}

// ────────────────────────────────────────────────────────────────────
// Test 7: Detect spacing conflicts
// ────────────────────────────────────────────────────────────────────

static void test_detect_conflicts() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};
    pd.wires.push_back(hwire(0, 100, 500, 10));
    pd.wires.push_back(hwire(0, 125, 500, 10));  // gap=15

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 40.0;
    eng.set_config(cfg);

    // Force same color → should detect conflict
    std::vector<ColorAssignment> asgn = {
        {0, 0, Color::COLOR_A},
        {1, 0, Color::COLOR_A}
    };

    auto conflicts = eng.detect_conflicts(asgn);
    CHECK(conflicts.size() == 1, "one same-color conflict detected");
    CHECK(conflicts[0].spacing_nm < 40.0, "spacing below threshold");
}

// ────────────────────────────────────────────────────────────────────
// Test 8: Resolve conflicts by recoloring
// ────────────────────────────────────────────────────────────────────

static void test_resolve_conflicts() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};
    pd.wires.push_back(hwire(0, 100, 500, 10));
    pd.wires.push_back(hwire(0, 125, 500, 10));

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 40.0;
    cfg.max_colors = 2;
    eng.set_config(cfg);

    std::vector<ColorAssignment> asgn = {
        {0, 0, Color::COLOR_A},
        {1, 0, Color::COLOR_A}
    };

    int resolved = eng.resolve_conflicts(asgn, cfg);
    CHECK(resolved >= 1, "at least one conflict resolved");
    CHECK(asgn[0].color != asgn[1].color, "wires now different colors");
}

// ────────────────────────────────────────────────────────────────────
// Test 9: Unresolvable conflict flagged
// ────────────────────────────────────────────────────────────────────

static void test_unresolvable() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};
    // Three mutually conflicting wires with only 2 colors available
    pd.wires.push_back(hwire(0, 100, 500, 10));
    pd.wires.push_back(hwire(0, 115, 500, 10));
    pd.wires.push_back(hwire(0, 130, 500, 10));

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 40.0;
    cfg.max_colors = 2;
    eng.set_config(cfg);

    // Force all same color
    std::vector<ColorAssignment> asgn = {
        {0, 0, Color::COLOR_A},
        {1, 0, Color::COLOR_A},
        {2, 0, Color::COLOR_A}
    };

    eng.resolve_conflicts(asgn, cfg);
    auto remaining = eng.detect_conflicts(asgn);
    CHECK(!remaining.empty(), "odd-cycle conflict unresolvable with 2 colors");
}

// ────────────────────────────────────────────────────────────────────
// Test 10: Empty layer → trivial
// ────────────────────────────────────────────────────────────────────

static void test_empty_layer() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.mp_layers = {0};
    eng.set_config(cfg);

    auto res = eng.sadp_decompose(0);
    CHECK(res.wires_colored == 0, "no wires on empty layer");
    CHECK(res.decomposition_legal, "empty layer is trivially legal");
    CHECK(res.conflicts_found == 0, "no conflicts on empty layer");
}

// ────────────────────────────────────────────────────────────────────
// Test 11: Single wire → trivially colored
// ────────────────────────────────────────────────────────────────────

static void test_single_wire() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};
    pd.wires.push_back(hwire(0, 100, 500, 10));

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 40.0;
    eng.set_config(cfg);

    auto res = eng.sadp_decompose(0);
    CHECK(res.wires_colored == 1, "single wire colored");
    CHECK(res.decomposition_legal, "single wire trivially legal");
    CHECK(res.assignments[0].color == Color::COLOR_A, "first wire gets COLOR_A");
}

// ────────────────────────────────────────────────────────────────────
// Test 12: run_enhanced() full flow
// ────────────────────────────────────────────────────────────────────

static void test_run_enhanced() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 2000, 2000};

    // Layer 0: bipartite (SADP-friendly)
    pd.wires.push_back(hwire(0, 100, 800, 10, 0, 1));
    pd.wires.push_back(hwire(0, 150, 800, 10, 0, 2));

    // Layer 1: odd cycle (needs SAQP)
    pd.wires.push_back(hwire(0, 100, 800, 10, 1, 3));
    pd.wires.push_back(hwire(0, 115, 800, 10, 1, 4));
    pd.wires.push_back(hwire(0, 130, 800, 10, 1, 5));

    MultiPatternEngine eng(pd);
    MultiPatternConfig cfg;
    cfg.min_space_nm = 40.0;
    cfg.max_colors = 2;
    eng.set_config(cfg);

    auto res = eng.run_enhanced();
    CHECK(res.wires_colored == 5, "all 5 wires colored");
    CHECK(res.decomposition_legal, "enhanced flow resolves all");
    CHECK(res.time_ms >= 0.0, "timing recorded");
    CHECK(!res.report.empty(), "report generated");
}

// ────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "=== Phase 78: Multi-Patterning (SADP/SAQP) Tests ===\n";

    test_conflict_graph();
    test_2_color_bipartite();
    test_2_color_odd_cycle();
    test_4_color();
    test_sadp_decompose();
    test_saqp_decompose();
    test_detect_conflicts();
    test_resolve_conflicts();
    test_unresolvable();
    test_empty_layer();
    test_single_wire();
    test_run_enhanced();

    std::cout << tests_passed << "/" << tests_run << " tests passed.\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
