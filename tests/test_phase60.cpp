// ============================================================
// SiliconForge — Multi-Threading Phase Tests (MT-4)
// Placement Parallelism (CG solver, density spreading)
// ============================================================

#include "pnr/placer.hpp"
#include "pnr/physical.hpp"
#include "core/netlist.hpp"
#include "core/thread_pool.hpp"
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using namespace sf;

static int pass_count = 0;
static int fail_count = 0;

#define TEST(name) \
    static void test_##name(); \
    struct Register_##name { Register_##name() { tests.push_back({#name, test_##name}); } } reg_##name; \
    static void test_##name()

struct TestEntry { const char* name; void(*fn)(); };
static std::vector<TestEntry> tests;

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        std::cerr << "  FAIL: " #cond " at line " << __LINE__ << "\n"; \
        fail_count++; return; \
    } \
} while(0)

static std::pair<Netlist, PhysicalDesign> make_design(int ncells) {
    Netlist nl;
    PhysicalDesign pd;
    pd.die_area = {0, 0, 200, 200};
    pd.row_height = 10.0;

    std::vector<NetId> nets;
    for (int i = 0; i < ncells; ++i) {
        nets.push_back(nl.add_net("n" + std::to_string(i)));
    }
    auto out = nl.add_net("out");

    for (int i = 0; i < ncells; ++i) {
        nl.add_gate(GateType::AND, {nets[i], nets[(i+1) % ncells]}, out,
                    "g" + std::to_string(i));
        CellInstance cell;
        cell.name = "c" + std::to_string(i);
        cell.width = 2.0;
        cell.height = 10.0;
        cell.position = {100, 100};
        pd.cells.push_back(cell);
    }

    // Create some nets
    for (int i = 0; i < ncells; ++i) {
        PhysNet pn;
        pn.id = i;
        pn.name = "n" + std::to_string(i);
        pn.cell_ids = {i, (i + 1) % ncells};
        pd.nets.push_back(pn);
    }

    return {std::move(nl), std::move(pd)};
}

// ============================================================
// Test 1: Placer basic placement with MT-enabled CG
// ============================================================
TEST(placer_basic_mt) {
    auto [nl, pd] = make_design(20);
    AnalyticalPlacer placer(pd);
    auto r = placer.place();
    ASSERT_TRUE(r.hpwl >= 0);
    ASSERT_TRUE(r.time_ms >= 0);
    pass_count++;
}

// ============================================================
// Test 2: Placer with larger design (exercises OpenMP paths)
// ============================================================
TEST(placer_large_mt) {
    auto [nl, pd] = make_design(100);
    AnalyticalPlacer placer(pd);
    auto r = placer.place();
    ASSERT_TRUE(r.hpwl >= 0);
    // All cells should be within die area
    for (auto& c : pd.cells) {
        ASSERT_TRUE(c.position.x >= pd.die_area.x0 - 1);
        ASSERT_TRUE(c.position.y >= pd.die_area.y0 - 1);
    }
    pass_count++;
}

// ============================================================
// Test 3: Placer deterministic results
// ============================================================
TEST(placer_deterministic_mt) {
    auto [nl1, pd1] = make_design(30);
    auto [nl2, pd2] = make_design(30);
    AnalyticalPlacer p1(pd1);
    AnalyticalPlacer p2(pd2);
    auto r1 = p1.place();
    auto r2 = p2.place();
    // HPWL should be identical for same input
    ASSERT_TRUE(std::abs(r1.hpwl - r2.hpwl) < 1e-3);
    pass_count++;
}

// ============================================================
// Test 4: Placer timing-driven mode
// ============================================================
TEST(placer_timing_driven_mt) {
    auto [nl, pd] = make_design(30);
    AnalyticalPlacer placer(pd);
    placer.set_timing_weight(0.5);
    auto r = placer.place();
    ASSERT_TRUE(r.hpwl >= 0);
    pass_count++;
}

// ============================================================
// Test 5: Placer single cell
// ============================================================
TEST(placer_single_cell_mt) {
    auto [nl, pd] = make_design(1);
    AnalyticalPlacer placer(pd);
    auto r = placer.place();
    ASSERT_TRUE(r.time_ms >= 0);
    pass_count++;
}

// ============================================================
// Test 6: Placer congestion-driven mode
// ============================================================
TEST(placer_congestion_mt) {
    auto [nl, pd] = make_design(40);
    AnalyticalPlacer placer(pd);
    placer.enable_congestion_driven(true);
    auto r = placer.place();
    ASSERT_TRUE(r.hpwl >= 0);
    pass_count++;
}

// ============================================================
// Test 7: Placer empty design
// ============================================================
TEST(placer_empty_mt) {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};
    pd.row_height = 10.0;
    AnalyticalPlacer placer(pd);
    auto r = placer.place();
    ASSERT_TRUE(r.time_ms >= 0);
    pass_count++;
}

// ============================================================
// Test 8: Placer macro + std cells
// ============================================================
TEST(placer_macro_mt) {
    auto [nl, pd] = make_design(20);
    pd.cells[0].is_macro = true;
    pd.cells[0].width = 40;
    pd.cells[0].height = 40;
    AnalyticalPlacer placer(pd);
    auto r = placer.place();
    ASSERT_TRUE(r.hpwl >= 0);
    pass_count++;
}

// ============================================================
// Test 9: Placer result has positive wirelength
// ============================================================
TEST(placer_hpwl_positive_mt) {
    auto [nl, pd] = make_design(50);
    AnalyticalPlacer placer(pd);
    auto r = placer.place();
    ASSERT_TRUE(r.hpwl > 0);
    pass_count++;
}

// ============================================================
// Test 10: Multiple placements don't crash (pool reuse)
// ============================================================
TEST(placer_repeated_mt) {
    for (int trial = 0; trial < 3; ++trial) {
        auto [nl, pd] = make_design(25);
        AnalyticalPlacer placer(pd);
        auto r = placer.place();
        ASSERT_TRUE(r.hpwl >= 0);
    }
    pass_count++;
}

// ============================================================
// main
// ============================================================
int main() {
    std::cout << "=== Phase 60: Placement Parallelism (MT-4) ===\n";
    for (auto& t : tests) {
        std::cout << "  " << t.name << "... ";
        t.fn();
        if (fail_count == 0 || pass_count > 0)
            std::cout << "PASS\n";
    }
    std::cout << "\nPhase 60: " << pass_count << " passed, " << fail_count << " failed\n";
    return fail_count > 0 ? 1 : 0;
}
