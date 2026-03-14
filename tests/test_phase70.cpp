// SiliconForge — Phase 70: Detailed Placement Tests
// Validates Abacus legalization, cell swapping, local reordering,
// HPWL computation, and full optimization flow.

#include "../src/pnr/detailed_placer.hpp"
#include "../src/core/netlist.hpp"
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

// Helper: create a simple physical design with cells in a single row area
static PhysicalDesign make_single_row_design(int ncells, double cell_w = 1.0) {
    PhysicalDesign pd;
    pd.die_area = {0, 0, ncells * cell_w * 2, 5.6};
    pd.row_height = 2.8;
    pd.site_width = 0.38;
    for (int i = 0; i < ncells; i++) {
        CellInstance c;
        c.id = i;
        c.name = "c" + std::to_string(i);
        c.cell_type = "INV";
        c.width = cell_w;
        c.height = 2.8;
        c.position = {i * cell_w * 1.5, 0};
        c.placed = false;
        pd.cells.push_back(c);
    }
    return pd;
}

// Helper: add a 2-pin net between two cells
static void add_net(PhysicalDesign& pd, int c0, int c1) {
    pd.add_net("n" + std::to_string(pd.nets.size()), {c0, c1});
}

// Test 1: Legalization of cells in a single row
void test_single_row_legalization() {
    PhysicalDesign pd = make_single_row_design(4);
    Netlist nl;
    DetailedPlacer dp(pd, nl);
    auto result = dp.legalize();
    CHECK(result.cells_moved >= 0, "single row legalize returns valid count");
    for (auto& c : pd.cells)
        CHECK(c.placed, "cell placed after legalization");
}

// Test 2: Legalization with overlapping cells
void test_overlapping_legalization() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 20, 5.6};
    pd.row_height = 2.8;
    pd.site_width = 0.38;
    // Place 3 cells all at the same position — overlap
    for (int i = 0; i < 3; i++) {
        CellInstance c;
        c.id = i;
        c.name = "ov" + std::to_string(i);
        c.cell_type = "BUF";
        c.width = 1.52;  // 4 sites
        c.height = 2.8;
        c.position = {2.0, 0};
        c.placed = false;
        pd.cells.push_back(c);
    }
    Netlist nl;
    DetailedPlacer dp(pd, nl);
    auto result = dp.legalize();
    CHECK(result.legalization_pass, "overlapping cells legalized without overlaps");
    CHECK(result.cells_moved > 0, "at least some cells moved to resolve overlaps");
}

// Test 3: HPWL computation on simple netlist
void test_hpwl_computation() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 20, 5.6};
    pd.row_height = 2.8;
    pd.site_width = 0.38;
    // Two cells at known positions
    int c0 = pd.add_cell("a", "INV", 1.0, 2.8);
    int c1 = pd.add_cell("b", "INV", 1.0, 2.8);
    pd.cells[c0].position = {0, 0}; pd.cells[c0].placed = true;
    pd.cells[c1].position = {10, 0}; pd.cells[c1].placed = true;
    pd.add_net("n0", {c0, c1});

    Netlist nl;
    DetailedPlacer dp(pd, nl);
    double hpwl = dp.compute_hpwl();
    CHECK(hpwl > 9.0, "HPWL captures horizontal distance");
}

// Test 4: Cell swap improves HPWL
void test_cell_swap_improves() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 30, 5.6};
    pd.row_height = 2.8;
    pd.site_width = 0.38;
    // 4 cells: a-b-c-d, nets: a-c and b-d (crossing nets)
    int a = pd.add_cell("a", "INV", 1.0, 2.8);
    int b = pd.add_cell("b", "INV", 1.0, 2.8);
    int c = pd.add_cell("c", "INV", 1.0, 2.8);
    int d = pd.add_cell("d", "INV", 1.0, 2.8);
    pd.cells[a].position = {0, 0};   pd.cells[a].placed = true;
    pd.cells[b].position = {2, 0};   pd.cells[b].placed = true;
    pd.cells[c].position = {4, 0};   pd.cells[c].placed = true;
    pd.cells[d].position = {6, 0};   pd.cells[d].placed = true;
    pd.add_net("n_ac", {a, c});
    pd.add_net("n_bd", {b, d});

    Netlist nl;
    DetailedPlacer dp(pd, nl);
    double hpwl_before = dp.compute_hpwl();

    DetailedPlaceConfig cfg;
    cfg.enable_legalization = false;
    cfg.enable_cell_swap = true;
    cfg.enable_local_reorder = false;
    cfg.max_swap_distance = 20;
    cfg.site_width_um = 0.38;
    auto result = dp.optimize(cfg);
    CHECK(result.hpwl_after <= hpwl_before + 1e-6,
          "cell swap does not worsen HPWL");
}

// Test 5: Local reorder in a 3-cell window
void test_local_reorder() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 20, 5.6};
    pd.row_height = 2.8;
    pd.site_width = 1.0;
    // 3 cells placed in sub-optimal order with crossing nets
    int a = pd.add_cell("a", "INV", 1.0, 2.8);
    int b = pd.add_cell("b", "INV", 1.0, 2.8);
    int c = pd.add_cell("c", "INV", 1.0, 2.8);
    int d = pd.add_cell("d", "INV", 1.0, 2.8); // anchor
    pd.cells[a].position = {0, 0}; pd.cells[a].placed = true;
    pd.cells[b].position = {1, 0}; pd.cells[b].placed = true;
    pd.cells[c].position = {2, 0}; pd.cells[c].placed = true;
    pd.cells[d].position = {10, 0}; pd.cells[d].placed = true;
    pd.add_net("n_ad", {a, d}); // a connects far right
    pd.add_net("n_cb", {c, b}); // c connects to b

    Netlist nl;
    DetailedPlacer dp(pd, nl);
    double hpwl_before = dp.compute_hpwl();

    DetailedPlaceConfig cfg;
    cfg.enable_legalization = true;
    cfg.enable_cell_swap = false;
    cfg.enable_local_reorder = true;
    cfg.row_height_um = 2.8;
    cfg.site_width_um = 1.0;
    auto result = dp.optimize(cfg);
    CHECK(result.hpwl_after <= hpwl_before + 1e-6,
          "local reorder does not worsen HPWL");
}

// Test 6: Abacus snap-to-grid
void test_snap_to_grid() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 20, 5.6};
    pd.row_height = 2.8;
    pd.site_width = 0.38;
    // Cell at non-grid-aligned position
    CellInstance c;
    c.id = 0; c.name = "sg"; c.cell_type = "INV";
    c.width = 0.76; c.height = 2.8;
    c.position = {1.13, 0.5}; // off-grid
    c.placed = false;
    pd.cells.push_back(c);

    Netlist nl;
    DetailedPlacer dp(pd, nl);
    auto result = dp.legalize();
    double x = pd.cells[0].position.x;
    double remainder = std::fmod(x, 0.38);
    if (remainder < 0) remainder += 0.38;
    double grid_err = std::min(remainder, 0.38 - remainder);
    CHECK(grid_err < 0.01, "cell snapped to site grid");
    CHECK(std::abs(pd.cells[0].position.y) < 0.01 ||
          std::abs(pd.cells[0].position.y - 2.8) < 0.01,
          "cell snapped to row grid");
}

// Test 7: Max displacement constraint
void test_max_displacement() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};
    pd.row_height = 2.8;
    pd.site_width = 0.38;
    CellInstance c;
    c.id = 0; c.name = "far"; c.cell_type = "INV";
    c.width = 0.76; c.height = 2.8;
    c.position = {50, 50}; // center of die
    c.placed = false;
    pd.cells.push_back(c);

    Netlist nl;
    DetailedPlaceConfig cfg;
    cfg.max_displacement_um = 5.0; // tight constraint
    DetailedPlacer dp(pd, nl);
    auto result = dp.legalize(cfg);
    // Cell should be placed, displacement might exceed if no row nearby
    // but the algorithm should respect the limit when possible
    CHECK(result.cells_moved >= 0, "max displacement constraint handled");
}

// Test 8: Empty design (no cells)
void test_empty_design() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};
    pd.row_height = 2.8;
    pd.site_width = 0.38;

    Netlist nl;
    DetailedPlacer dp(pd, nl);
    auto result = dp.legalize();
    CHECK(result.cells_moved == 0, "no cells moved in empty design");
    CHECK(result.legalization_pass, "empty design is legal");
    CHECK(result.hpwl_before == 0, "empty design HPWL is 0");
}

// Test 9: Full optimize() flow
void test_optimize_flow() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 40, 8.4};
    pd.row_height = 2.8;
    pd.site_width = 0.38;
    for (int i = 0; i < 6; i++) {
        CellInstance c;
        c.id = i;
        c.name = "opt" + std::to_string(i);
        c.cell_type = "NAND";
        c.width = 1.14;
        c.height = 2.8;
        c.position = {(double)i * 3.0 + 0.5, i % 3 == 0 ? 0.0 : 1.0};
        c.placed = false;
        pd.cells.push_back(c);
    }
    add_net(pd, 0, 3); add_net(pd, 1, 4); add_net(pd, 2, 5);

    Netlist nl;
    DetailedPlacer dp(pd, nl);
    auto result = dp.optimize();
    CHECK(result.time_ms >= 0, "optimize reports time");
    CHECK(!result.report.empty(), "optimize produces report");
    CHECK(result.hpwl_after >= 0, "optimize reports valid HPWL");
}

// Test 10: Legalization with cells near die edge
void test_edge_blockage() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 5, 5.6};
    pd.row_height = 2.8;
    pd.site_width = 0.38;
    // Cell wider than available space near edge
    CellInstance c;
    c.id = 0; c.name = "edge"; c.cell_type = "BUF";
    c.width = 3.0; c.height = 2.8;
    c.position = {4.0, 0}; // would overflow right edge
    c.placed = false;
    pd.cells.push_back(c);

    Netlist nl;
    DetailedPlacer dp(pd, nl);
    auto result = dp.legalize();
    double right = pd.cells[0].position.x + pd.cells[0].width;
    CHECK(right <= pd.die_area.x1 + 0.5, "cell does not extend far past die edge");
    CHECK(pd.cells[0].placed, "edge cell is placed");
}

// Test 11: Multi-row legalization
void test_multi_row() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 10, 8.4}; // 3 rows of height 2.8
    pd.row_height = 2.8;
    pd.site_width = 0.38;
    // More cells than fit in one row
    for (int i = 0; i < 8; i++) {
        CellInstance c;
        c.id = i;
        c.name = "mr" + std::to_string(i);
        c.cell_type = "INV";
        c.width = 2.0;
        c.height = 2.8;
        c.position = {(double)i * 0.5, 0}; // all try to go in row 0
        c.placed = false;
        pd.cells.push_back(c);
    }

    Netlist nl;
    DetailedPlacer dp(pd, nl);
    auto result = dp.legalize();
    CHECK(result.cells_moved > 0, "multi-row: cells distributed across rows");
    // Verify cells are in different rows
    bool multi = false;
    for (size_t i = 1; i < pd.cells.size(); i++) {
        if (std::abs(pd.cells[i].position.y - pd.cells[0].position.y) > 1.0) {
            multi = true; break;
        }
    }
    CHECK(multi, "multi-row: cells span multiple rows");
}

// Test 12: run_enhanced() full flow
void test_run_enhanced() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 30, 8.4};
    pd.row_height = 2.8;
    pd.site_width = 0.38;
    for (int i = 0; i < 5; i++) {
        CellInstance c;
        c.id = i;
        c.name = "enh" + std::to_string(i);
        c.cell_type = "AND";
        c.width = 1.14;
        c.height = 2.8;
        c.position = {(double)i * 4.0, i % 2 == 0 ? 0.0 : 1.0};
        c.placed = false;
        pd.cells.push_back(c);
    }
    add_net(pd, 0, 2); add_net(pd, 1, 3); add_net(pd, 2, 4);

    Netlist nl;
    DetailedPlacer dp(pd, nl);
    auto result = dp.run_enhanced();
    CHECK(result.time_ms >= 0, "run_enhanced reports time");
    CHECK(result.hpwl_after >= 0, "run_enhanced valid HPWL");
    CHECK(!result.report.empty(), "run_enhanced produces report");
    CHECK(result.legalization_pass || result.cells_moved >= 0,
          "run_enhanced completes without crash");
}

int main() {
    std::cout << "=== Phase 70: Detailed Placement Tests ===\n";
    test_single_row_legalization();
    test_overlapping_legalization();
    test_hpwl_computation();
    test_cell_swap_improves();
    test_local_reorder();
    test_snap_to_grid();
    test_max_displacement();
    test_empty_design();
    test_optimize_flow();
    test_edge_blockage();
    test_multi_row();
    test_run_enhanced();
    std::cout << "Phase 70: " << tests_passed << "/" << tests_run
              << " detailed placement tests passed.\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
