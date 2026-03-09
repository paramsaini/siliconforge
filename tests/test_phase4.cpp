// SiliconForge — Phase 4 Test Suite
// Tests: Physical design, placement, CTS, GDSII writer

#include "pnr/physical.hpp"
#include "pnr/placer.hpp"
#include "pnr/cts.hpp"
#include "pnr/gdsii_writer.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

static PhysicalDesign build_test_design(int n_cells = 20) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;

    for (int i = 0; i < n_cells; ++i)
        pd.add_cell("cell_" + std::to_string(i), "AND2_X1", 3.0, 10.0);

    // Create some nets
    for (int i = 0; i < n_cells - 1; ++i)
        pd.add_net("n" + std::to_string(i), {i, i+1});

    return pd;
}

// ============================================================================
TEST(physical_basic) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    CHECK(pd.die_area.area() == 10000, "die area");
    CHECK(pd.die_area.center().x == 50, "die center x");
    CHECK(pd.die_area.center().y == 50, "die center y");

    int c1 = pd.add_cell("c1", "INV", 2, 10);
    int c2 = pd.add_cell("c2", "AND2", 3, 10);
    CHECK(pd.cells.size() == 2, "2 cells");
    int n = pd.add_net("n1", {c1, c2});
    CHECK(pd.nets.size() == 1, "1 net");
    PASS("physical_basic");
}

TEST(physical_hpwl) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    int c1 = pd.add_cell("c1", "INV", 2, 10);
    int c2 = pd.add_cell("c2", "INV", 2, 10);
    pd.cells[c1].position = {0, 0}; pd.cells[c1].placed = true;
    pd.cells[c2].position = {10, 20}; pd.cells[c2].placed = true;
    pd.add_net("n1", {c1, c2});

    double hpwl = pd.total_wirelength();
    CHECK(hpwl > 0, "HPWL > 0");
    PASS("physical_hpwl");
}

TEST(physical_overlap) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    int c1 = pd.add_cell("c1", "INV", 5, 10);
    int c2 = pd.add_cell("c2", "INV", 5, 10);
    pd.cells[c1].position = {0, 0}; pd.cells[c1].placed = true;
    pd.cells[c2].position = {3, 0}; pd.cells[c2].placed = true;
    CHECK(pd.has_overlaps(), "overlapping cells detected");

    pd.cells[c2].position = {10, 0};
    CHECK(!pd.has_overlaps(), "no overlap after move");
    PASS("physical_overlap");
}

// ============================================================================
TEST(placer_small) {
    auto pd = build_test_design(10);
    AnalyticalPlacer placer(pd);
    auto result = placer.place();

    CHECK(result.hpwl > 0, "HPWL > 0 after placement");
    CHECK(result.time_ms >= 0, "time measured");
    // All cells should be placed
    for (auto& c : pd.cells)
        CHECK(c.placed, "cell placed");
    PASS("placer_small");
}

TEST(placer_medium) {
    auto pd = build_test_design(50);
    AnalyticalPlacer placer(pd);
    auto result = placer.place();

    CHECK(result.hpwl > 0, "HPWL > 0");
    double util = pd.utilization();
    CHECK(util > 0, "utilization > 0");
    PASS("placer_medium");
}

// ============================================================================
TEST(cts_basic) {
    auto pd = build_test_design(10);
    // Place cells first
    AnalyticalPlacer placer(pd);
    placer.place();

    CtsEngine cts(pd);
    std::vector<int> sinks = {0, 1, 2, 3, 4};
    auto result = cts.build_clock_tree({100, 50}, sinks);

    CHECK(result.wirelength > 0, "CTS wirelength > 0");
    CHECK(result.buffers_inserted >= 0, "buffers inserted");
    PASS("cts_basic");
}

// ============================================================================
TEST(gdsii_def_output) {
    auto pd = build_test_design(5);
    AnalyticalPlacer placer(pd);
    placer.place();

    GdsiiWriter writer(pd);
    auto def = writer.to_def();
    CHECK(def.find("DESIGN TOP") != std::string::npos, "DEF has design");
    CHECK(def.find("COMPONENTS") != std::string::npos, "DEF has components");
    CHECK(def.find("NETS") != std::string::npos, "DEF has nets");
    PASS("gdsii_def_output");
}

TEST(gdsii_binary) {
    auto pd = build_test_design(5);
    AnalyticalPlacer placer(pd);
    placer.place();

    GdsiiWriter writer(pd);
    bool ok = writer.write("/tmp/siliconforge_test.gds");
    CHECK(ok, "GDSII file written");
    PASS("gdsii_binary");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 4 — Test Suite               ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Physical Design ──\n";
    RUN(physical_basic);
    RUN(physical_hpwl);
    RUN(physical_overlap);

    std::cout << "\n── Analytical Placer ──\n";
    RUN(placer_small);
    RUN(placer_medium);

    std::cout << "\n── Clock Tree Synthesis ──\n";
    RUN(cts_basic);

    std::cout << "\n── GDSII Writer ──\n";
    RUN(gdsii_def_output);
    RUN(gdsii_binary);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
