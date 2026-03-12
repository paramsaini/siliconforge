// SiliconForge — Phase 40: Chiplet/Multi-Die Integration Tests
// 25 comprehensive tests covering TSV parasitics, micro-bumps, interposer,
// D2D links, 3D power grid, yield, redundancy, SI, cross-die routing, E2E

#include "core/die_to_die.hpp"
#include "pnr/tsv_mgr.hpp"
#include "pnr/physical.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
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

// ── Test 1: TSV Parasitics Physics ──────────────────────────────────────────
void test_tsv_parasitics_physics() {
    RUN("TSV parasitics physics (R/L/C)");
    sf::TsvTechParams tech;
    tech.radius_um = 2.5;
    tech.height_um = 50.0;
    tech.oxide_thick_um = 0.5;

    auto p = tech.compute_parasitics();

    // R = ρ·h/(π·r²) = 1.68e-8 × 50e-6 / (π × (2.5e-6)²)
    // ≈ 1.68e-8 × 50e-6 / (π × 6.25e-12) ≈ 42.8 mΩ
    CHECKF(p.resistance, 0.01, 0.2);       // tens of mΩ range
    CHECK(p.inductance > 0);                // positive inductance
    CHECK(p.capacitance > 0);               // positive capacitance
    CHECKF(p.delay_ps, 0.0001, 10.0);      // sub-10 ps for typical TSV
    CHECK(p.bandwidth_gbps > 1.0);          // > 1 Gbps bandwidth
    PASS();
}

// ── Test 2: TSV Coupling Capacitance ────────────────────────────────────────
void test_tsv_coupling() {
    RUN("TSV coupling capacitance");
    sf::TsvTechParams tech;
    auto p_near = tech.compute_coupling(10.0);   // 10 μm apart
    auto p_far = tech.compute_coupling(100.0);    // 100 μm apart

    CHECK(p_near.coupling_cap > 0);
    CHECK(p_far.coupling_cap > 0);
    // Closer TSVs should have MORE coupling
    CHECK(p_near.coupling_cap > p_far.coupling_cap);
    PASS();
}

// ── Test 3: TSV Tech Parameter Sensitivity ──────────────────────────────────
void test_tsv_tech_sensitivity() {
    RUN("TSV tech parameter sensitivity");
    sf::TsvTechParams thin, thick;
    thin.height_um = 30.0;
    thick.height_um = 100.0;

    auto p_thin = thin.compute_parasitics();
    auto p_thick = thick.compute_parasitics();

    // Thicker wafer = higher R, L, C
    CHECK(p_thick.resistance > p_thin.resistance);
    CHECK(p_thick.inductance > p_thin.inductance);
    CHECK(p_thick.capacitance > p_thin.capacitance);
    PASS();
}

// ── Test 4: Die Management ──────────────────────────────────────────────────
void test_die_management() {
    RUN("Die add/configure/query");
    sf::PackageDesign pkg;
    int d0 = pkg.add_die("compute", 0);
    int d1 = pkg.add_die("memory", 1);
    int d2 = pkg.add_die("io", 0, 15000, 0); // 2.5D side-by-side

    CHECK(d0 == 0);
    CHECK(d1 == 1);
    CHECK(d2 == 2);
    CHECK(pkg.dies().size() == 3);
    CHECK(pkg.dies()[0].name == "compute");
    CHECK(pkg.dies()[1].z_layer == 1);
    CHECK(pkg.dies()[2].x_offset == 15000);

    pkg.set_die_size(d0, 12000, 12000);
    pkg.set_die_power(d0, 5.0);
    CHECK(pkg.dies()[0].width_um == 12000);
    CHECK(pkg.dies()[0].power_w == 5.0);
    PASS();
}

// ── Test 5: Signal TSV with Parasitics ──────────────────────────────────────
void test_signal_tsv_parasitics() {
    RUN("Signal TSV with computed parasitics");
    sf::PackageDesign pkg;
    pkg.add_die("bot", 0);
    pkg.add_die("top", 1);
    sf::TsvTechParams tech;
    int tid = pkg.add_signal_tsv(42, 0, 1, 100.0f, 200.0f, tech);

    CHECK(tid >= 0);
    CHECK(pkg.tsvs().size() == 1);
    CHECK(pkg.tsvs()[0].net_id == 42);
    CHECK(pkg.tsvs()[0].is_signal == true);
    CHECK(pkg.tsvs()[0].parasitics.resistance > 0);
    CHECK(pkg.tsvs()[0].parasitics.delay_ps > 0);
    PASS();
}

// ── Test 6: Power TSV ───────────────────────────────────────────────────────
void test_power_tsv() {
    RUN("Power TSV (wider, lower R)");
    sf::PackageDesign pkg;
    pkg.add_die("bot", 0);
    pkg.add_die("top", 1);
    sf::TsvTechParams tech;
    int sig_id = pkg.add_signal_tsv(0, 0, 1, 50, 50, tech);
    int pwr_id = pkg.add_power_tsv(true, 0, 1, 100, 100, tech);

    CHECK(pkg.tsvs().size() == 2);
    CHECK(pkg.tsvs()[1].is_signal == false);
    CHECK(pkg.tsvs()[1].net_id == -2); // VDD convention
    // Power TSV has 1.5× radius → lower R
    CHECK(pkg.tsvs()[pwr_id].parasitics.resistance <
          pkg.tsvs()[sig_id].parasitics.resistance);
    PASS();
}

// ── Test 7: Spare TSV ───────────────────────────────────────────────────────
void test_spare_tsv() {
    RUN("Spare TSV for redundancy");
    sf::PackageDesign pkg;
    pkg.add_die("bot", 0);
    pkg.add_die("top", 1);
    pkg.add_signal_tsv(10, 0, 1, 50, 50);
    int spare = pkg.add_spare_tsv(0, 1, 200, 200);

    CHECK(pkg.tsvs()[spare].is_spare == true);
    CHECK(pkg.tsvs()[spare].net_id == -1); // unassigned
    CHECK(pkg.count_spare_tsvs() == 1);
    PASS();
}

// ── Test 8: Micro-bump Signal ───────────────────────────────────────────────
void test_micro_bump_signal() {
    RUN("Micro-bump signal connection");
    sf::PackageDesign pkg;
    pkg.add_die("top", 1);
    pkg.add_die("bot", 0);
    sf::MicroBumpTech btech;
    int bid = pkg.add_micro_bump(7, 0, 1, 500, 500, btech);

    CHECK(bid >= 0);
    CHECK(pkg.bumps().size() == 1);
    CHECK(pkg.bumps()[0].net_id == 7);
    CHECK(pkg.bumps()[0].is_power == false);
    CHECKF(pkg.bumps()[0].contact_resistance, 0.001, 0.1);
    PASS();
}

// ── Test 9: Power Bump ──────────────────────────────────────────────────────
void test_power_bump() {
    RUN("Power bump (VDD/GND)");
    sf::PackageDesign pkg;
    pkg.add_die("top", 1);
    pkg.add_die("bot", 0);
    pkg.add_power_bump(true, 0, 1, 100, 100);   // VDD
    pkg.add_power_bump(false, 0, 1, 200, 200);  // GND

    CHECK(pkg.bumps().size() == 2);
    CHECK(pkg.bumps()[0].is_power == true);
    CHECK(pkg.bumps()[0].net_id == -2);  // VDD
    CHECK(pkg.bumps()[1].net_id == -3);  // GND
    PASS();
}

// ── Test 10: Interposer Trace ───────────────────────────────────────────────
void test_interposer_trace() {
    RUN("Interposer RDL trace R/C");
    sf::PackageDesign pkg;
    sf::InterposerConfig icfg;
    int tid = pkg.add_interposer_trace(5, 0, 0, 0, 1000, 0, icfg);

    CHECK(tid >= 0);
    CHECK(pkg.traces().size() == 1);
    // 1000 μm trace: R = 0.025 × 1000 = 25 Ω
    CHECKF(pkg.traces()[0].resistance(), 20, 30);
    // C = 0.15 fF/μm × 1000 = 150 fF
    CHECKF(pkg.traces()[0].capacitance_ff(), 100, 200);
    PASS();
}

// ── Test 11: Die-to-Die Link ────────────────────────────────────────────────
void test_d2d_link() {
    RUN("Die-to-die link bandwidth/power");
    sf::PackageDesign pkg;
    pkg.add_die("compute", 0);
    pkg.add_die("hbm", 1);
    int lid = pkg.add_d2d_link(0, 1, 16, 32.0);

    CHECK(lid >= 0);
    CHECK(pkg.d2d_links().size() == 1);
    // BW = 16 lanes × 32 Gbps = 512 Gbps
    CHECKF(pkg.d2d_links()[0].bandwidth_gbps(), 510, 514);
    // Power = 16 × 0.5 mW = 8 mW
    CHECKF(pkg.d2d_links()[0].total_power_mw(), 7.5, 8.5);
    CHECK(pkg.total_d2d_bandwidth_gbps() > 500);
    PASS();
}

// ── Test 12: 3D Power Grid ──────────────────────────────────────────────────
void test_3d_power_grid() {
    RUN("3D power grid IR-drop");
    sf::PackageDesign pkg;
    int d0 = pkg.add_die("bot", 0);
    int d1 = pkg.add_die("top", 1);
    pkg.set_die_size(d0, 10000, 10000);
    pkg.set_die_size(d1, 10000, 10000);
    pkg.set_die_power(d0, 2.0);
    pkg.set_die_power(d1, 3.0);

    pkg.build_power_grid(8, 8);

    CHECK(pkg.power_grid().power_tsvs.size() > 0);
    CHECK(pkg.power_grid().ir_drop_map.size() == 2); // 2 dies
    double max_drop = pkg.power_grid().max_ir_drop();
    CHECK(max_drop > 0); // should have some IR drop
    PASS();
}

// ── Test 13: Yield Model — Single Die ───────────────────────────────────────
void test_yield_single_die() {
    RUN("Yield model — single die Poisson");
    sf::YieldModel y;
    y.die_defect_density = 0.5;
    y.die_area_cm2 = 1.0;

    double yield = y.single_die_yield();
    // Y = e^(-0.5×1.0) = e^(-0.5) ≈ 0.6065
    CHECKF(yield, 0.60, 0.62);

    // Larger die = lower yield
    y.die_area_cm2 = 2.0;
    double yield2 = y.single_die_yield();
    CHECK(yield2 < yield);
    PASS();
}

// ── Test 14: Yield Model — TSV Array ────────────────────────────────────────
void test_yield_tsv_array() {
    RUN("Yield model — TSV array with spares");
    sf::YieldModel y;
    y.tsv_defect_rate = 1e-4; // 0.01% defect rate

    double y_no_spare = y.tsv_array_yield(1000, 0);
    double y_with_spare = y.tsv_array_yield(1000, 5);

    CHECK(y_no_spare > 0);
    CHECK(y_no_spare < 1.0);
    // Spares should improve yield
    CHECK(y_with_spare >= y_no_spare);
    PASS();
}

// ── Test 15: Package Yield ──────────────────────────────────────────────────
void test_package_yield() {
    RUN("Package yield — multi-die");
    sf::PackageDesign pkg;
    pkg.add_die("die0", 0);
    pkg.add_die("die1", 1);
    pkg.add_signal_tsv(0, 0, 1, 50, 50);
    pkg.add_signal_tsv(1, 0, 1, 100, 100);
    pkg.add_spare_tsv(0, 1, 200, 200);

    sf::YieldModel ym;
    double yield = pkg.estimate_yield(ym);
    CHECK(yield > 0);
    CHECK(yield <= 1.0);
    PASS();
}

// ── Test 16: TSV Signal Integrity ───────────────────────────────────────────
void test_tsv_signal_integrity() {
    RUN("TSV signal integrity (coupling analysis)");
    sf::PackageDesign pkg;
    pkg.add_die("bot", 0);
    pkg.add_die("top", 1);
    // Two TSVs close together
    pkg.add_signal_tsv(0, 0, 1, 100, 100);
    pkg.add_signal_tsv(1, 0, 1, 110, 100); // 10 μm apart

    sf::TsvTechParams tech;
    auto si0 = pkg.compute_tsv_si(0, tech);
    auto si1 = pkg.compute_tsv_si(1, tech);

    CHECK(si0.coupling_cap > 0);
    CHECK(si1.coupling_cap > 0);
    CHECK(si0.resistance > 0);
    PASS();
}

// ── Test 17: TSV Spacing DRC ────────────────────────────────────────────────
void test_tsv_spacing_drc() {
    RUN("TSV spacing DRC check");
    sf::PackageDesign pkg;
    pkg.add_die("bot", 0);
    pkg.add_die("top", 1);
    pkg.add_signal_tsv(0, 0, 1, 100, 100);
    pkg.add_signal_tsv(1, 0, 1, 105, 100); // only 5 μm apart

    // Should fail with 10 μm minimum spacing
    CHECK(sf::TsvManager::check_tsv_spacing(pkg, 10.0) == false);
    // Should pass with 3 μm minimum spacing
    CHECK(sf::TsvManager::check_tsv_spacing(pkg, 3.0) == true);
    PASS();
}

// ── Test 18: Redundancy — Spare Assignment ──────────────────────────────────
void test_spare_assignment() {
    RUN("Spare TSV assignment on failure");
    sf::PackageDesign pkg;
    pkg.add_die("bot", 0);
    pkg.add_die("top", 1);
    int sig = pkg.add_signal_tsv(42, 0, 1, 100, 100);
    int spare = pkg.add_spare_tsv(0, 1, 200, 200);

    CHECK(pkg.count_spare_tsvs() == 1);

    // Assign spare for failed signal TSV
    int replacement = pkg.assign_spare_tsv(sig);
    CHECK(replacement == spare);
    CHECK(pkg.count_spare_tsvs() == 0); // spare consumed

    // No more spares available
    int no_spare = pkg.assign_spare_tsv(sig);
    CHECK(no_spare == -1);
    PASS();
}

// ── Test 19: Legacy API Backward Compatibility ──────────────────────────────
void test_legacy_api() {
    RUN("Legacy insert_tsvs API");
    sf::PackageDesign pkg;
    pkg.add_die("bot", 0);
    pkg.add_die("top", 1);

    sf::PhysicalDesign die1, die2;
    die1.die_area = sf::Rect(0, 0, 1000, 1000);
    die2.die_area = sf::Rect(0, 0, 1000, 1000);

    // Add shared net (id=5) on both dies
    int c1 = die1.add_cell("cell_a", "INV", 2, 2);
    die1.cells[c1].position = sf::Point(100, 200);
    die1.add_net("net5", {c1});
    die1.nets.back().id = 5;

    int c2 = die2.add_cell("cell_b", "BUF", 2, 2);
    die2.cells[c2].position = sf::Point(300, 400);
    die2.add_net("net5", {c2});
    die2.nets.back().id = 5;

    sf::TsvManager::insert_tsvs(pkg, die1, die2, 0, 1);

    CHECK(pkg.tsvs().size() >= 1);
    CHECK(pkg.tsvs()[0].net_id == 5);
    PASS();
}

// ── Test 20: SI-aware TSV Insertion ─────────────────────────────────────────
void test_si_aware_insertion() {
    RUN("SI-aware TSV insertion (cluster-aware)");
    sf::PackageDesign pkg;
    int d0 = pkg.add_die("bot", 0);
    int d1 = pkg.add_die("top", 1);
    pkg.set_die_size(d0, 10000, 10000);
    pkg.set_die_size(d1, 10000, 10000);

    sf::PhysicalDesign die1, die2;
    die1.die_area = sf::Rect(0, 0, 10000, 10000);
    die2.die_area = sf::Rect(0, 0, 10000, 10000);

    // Create 3 shared nets
    for (int n = 0; n < 3; n++) {
        int c1 = die1.add_cell("c1_" + std::to_string(n), "INV", 2, 2);
        die1.cells[c1].position = sf::Point(1000 + n * 200, 5000);
        die1.add_net("net" + std::to_string(n), {c1});
        die1.nets.back().id = n;

        int c2 = die2.add_cell("c2_" + std::to_string(n), "BUF", 2, 2);
        die2.cells[c2].position = sf::Point(1000 + n * 200, 5000);
        die2.add_net("net" + std::to_string(n), {c2});
        die2.nets.back().id = n;
    }

    sf::TsvPlaceConfig cfg;
    cfg.strategy = sf::TsvPlaceStrategy::CLUSTER_AWARE;
    cfg.insert_power_tsvs = true;
    cfg.power_grid_nx = 4;
    cfg.power_grid_ny = 4;
    cfg.spare_ratio_percent = 10;

    auto result = sf::TsvManager::insert_tsvs_si(pkg, die1, die2, 0, 1, cfg);

    CHECK(result.signal_tsvs_placed == 3);
    CHECK(result.power_tsvs_placed == 16);  // 4×4 grid
    CHECK(result.spare_tsvs_placed >= 0);   // at least attempted
    CHECK(result.max_delay_ps > 0);
    PASS();
}

// ── Test 21: Grid-aligned TSV Placement ─────────────────────────────────────
void test_grid_aligned_placement() {
    RUN("Grid-aligned TSV placement");
    sf::PackageDesign pkg;
    pkg.add_die("bot", 0);
    pkg.add_die("top", 1);

    sf::PhysicalDesign die1, die2;
    int c1 = die1.add_cell("cell", "INV", 2, 2);
    die1.cells[c1].position = sf::Point(103, 207); // off-grid
    die1.add_net("n0", {c1});
    die1.nets.back().id = 0;

    int c2 = die2.add_cell("cell", "BUF", 2, 2);
    die2.cells[c2].position = sf::Point(103, 207);
    die2.add_net("n0", {c2});
    die2.nets.back().id = 0;

    sf::TsvPlaceConfig cfg;
    cfg.strategy = sf::TsvPlaceStrategy::GRID_ALIGNED;
    cfg.insert_power_tsvs = false;
    cfg.spare_ratio_percent = 0;

    sf::TsvManager::insert_tsvs_si(pkg, die1, die2, 0, 1, cfg);

    CHECK(pkg.tsvs().size() >= 1);
    // Should be snapped to grid pitch (10 μm default)
    float tsv_x = pkg.tsvs()[0].x;
    float tsv_y = pkg.tsvs()[0].y;
    double pitch = cfg.tech.pitch_um;
    double rem_x = std::fmod(std::abs(tsv_x), pitch);
    double rem_y = std::fmod(std::abs(tsv_y), pitch);
    // Either aligned or very close (within 0.1)
    CHECK(rem_x < 0.5 || rem_x > pitch - 0.5);
    CHECK(rem_y < 0.5 || rem_y > pitch - 0.5);
    PASS();
}

// ── Test 22: Cross-die Interposer Routing ───────────────────────────────────
void test_interposer_routing() {
    RUN("Cross-die interposer routing");
    sf::PackageDesign pkg;
    pkg.add_die("chiplet_a", 0, 0, 0);
    pkg.add_die("chiplet_b", 0, 15000, 0); // side-by-side on interposer

    // Add bumps for two nets
    pkg.add_micro_bump(0, 0, 1, 5000, 5000);
    pkg.add_micro_bump(0, 0, 1, 15000 + 5000, 5000); // on chiplet B
    pkg.add_micro_bump(1, 0, 1, 3000, 3000);
    pkg.add_micro_bump(1, 0, 1, 15000 + 3000, 3000);

    sf::InterposerConfig icfg;
    int traces = sf::TsvManager::route_interposer(pkg, icfg);

    CHECK(traces > 0);
    CHECK(pkg.traces().size() > 0);
    PASS();
}

// ── Test 23: Chiplet Report ─────────────────────────────────────────────────
void test_chiplet_report() {
    RUN("Full chiplet report generation");
    sf::PackageDesign pkg;
    int d0 = pkg.add_die("compute", 0);
    int d1 = pkg.add_die("hbm", 1);
    pkg.set_die_size(d0, 12000, 12000);
    pkg.set_die_size(d1, 8000, 8000);

    pkg.add_signal_tsv(0, 0, 1, 100, 100);
    pkg.add_signal_tsv(1, 0, 1, 200, 200);
    pkg.add_power_tsv(true, 0, 1, 300, 300);
    pkg.add_spare_tsv(0, 1, 400, 400);
    pkg.add_d2d_link(0, 1, 32, 32.0);
    pkg.add_micro_bump(0, 0, 1, 500, 500);

    sf::TsvTechParams tech;
    sf::YieldModel ym;
    auto rpt = pkg.generate_report(tech, ym);

    CHECK(rpt.total_dies == 2);
    CHECK(rpt.total_tsvs == 4);
    CHECK(rpt.signal_tsvs == 2);
    CHECK(rpt.power_tsvs == 1);
    CHECK(rpt.spare_tsvs == 1);
    CHECK(rpt.total_bumps == 1);
    CHECK(rpt.total_d2d_links == 1);
    CHECKF(rpt.total_d2d_bandwidth_gbps, 1000, 1100); // 32×32=1024
    CHECK(rpt.total_tsv_resistance > 0);
    CHECK(rpt.estimated_yield > 0);
    CHECK(rpt.estimated_yield <= 1.0);
    PASS();
}

// ── Test 24: Edge Cases ─────────────────────────────────────────────────────
void test_edge_cases() {
    RUN("Edge cases (empty, single die, no shared nets)");

    // Empty package
    sf::PackageDesign pkg0;
    auto rpt0 = pkg0.generate_report();
    CHECK(rpt0.total_dies == 0);
    CHECK(rpt0.total_tsvs == 0);

    // Single die — no TSVs needed
    sf::PackageDesign pkg1;
    pkg1.add_die("solo", 0);
    pkg1.build_power_grid(4, 4);
    CHECK(pkg1.power_grid().power_tsvs.empty()); // needs ≥2 dies

    // Two dies but no shared nets
    sf::PackageDesign pkg2;
    pkg2.add_die("a", 0);
    pkg2.add_die("b", 1);
    sf::PhysicalDesign d1, d2;
    d1.add_net("n0", {});
    d1.nets.back().id = 0;
    d2.add_net("n1", {});
    d2.nets.back().id = 1; // different net IDs
    sf::TsvManager::insert_tsvs(pkg2, d1, d2, 0, 1);
    CHECK(pkg2.tsvs().empty()); // no shared → no TSVs

    // Yield with zero TSVs
    sf::YieldModel ym;
    CHECK(ym.tsv_array_yield(0) == 1.0);
    CHECK(ym.bump_array_yield(0) == 1.0);

    PASS();
}

// ── Test 25: E2E Chiplet Integration Flow ───────────────────────────────────
void test_e2e_chiplet_flow() {
    RUN("E2E: Full 3D chiplet integration flow");

    // 1. Create 3-die stack: compute + cache + HBM
    sf::PackageDesign pkg;
    int compute = pkg.add_die("compute_die", 0);
    int cache   = pkg.add_die("cache_die", 1);
    int hbm     = pkg.add_die("hbm_die", 2);
    pkg.set_die_size(compute, 12000, 12000);
    pkg.set_die_size(cache, 10000, 10000);
    pkg.set_die_size(hbm, 8000, 8000);
    pkg.set_die_power(compute, 10.0);
    pkg.set_die_power(cache, 3.0);
    pkg.set_die_power(hbm, 1.5);

    // 2. Create physical designs for compute and cache
    sf::PhysicalDesign pd_compute, pd_cache;
    pd_compute.die_area = sf::Rect(0, 0, 12000, 12000);
    pd_cache.die_area = sf::Rect(0, 0, 10000, 10000);

    // Add 10 shared signal nets
    for (int i = 0; i < 10; i++) {
        int c1 = pd_compute.add_cell("comp_" + std::to_string(i), "NAND", 3, 3);
        pd_compute.cells[c1].position = sf::Point(1000 + i * 500, 6000);
        pd_compute.add_net("bus_" + std::to_string(i), {c1});
        pd_compute.nets.back().id = i;

        int c2 = pd_cache.add_cell("cache_" + std::to_string(i), "BUF", 2, 2);
        pd_cache.cells[c2].position = sf::Point(800 + i * 450, 5000);
        pd_cache.add_net("bus_" + std::to_string(i), {c2});
        pd_cache.nets.back().id = i;
    }

    // 3. SI-aware TSV insertion (compute↔cache)
    sf::TsvPlaceConfig cfg;
    cfg.strategy = sf::TsvPlaceStrategy::CLUSTER_AWARE;
    cfg.insert_power_tsvs = true;
    cfg.power_grid_nx = 6;
    cfg.power_grid_ny = 6;
    cfg.spare_ratio_percent = 10;

    auto result = sf::TsvManager::insert_tsvs_si(pkg, pd_compute, pd_cache, 0, 1, cfg);
    CHECK(result.signal_tsvs_placed == 10);
    CHECK(result.power_tsvs_placed == 36); // 6×6

    // 4. Add D2D links
    pkg.add_d2d_link(compute, cache, 64, 32.0);  // 64-lane high-speed
    pkg.add_d2d_link(cache, hbm, 128, 16.0);     // 128-lane to HBM

    // 5. Build 3D power grid
    pkg.build_power_grid(8, 8);

    // 6. Add micro-bumps for HBM interface
    for (int i = 0; i < 20; i++) {
        pkg.add_micro_bump(100 + i, cache, hbm, 500 + i * 350.0f, 4000);
    }

    // 7. Generate full report
    auto rpt = pkg.generate_report();
    CHECK(rpt.total_dies == 3);
    CHECK(rpt.signal_tsvs >= 10);
    CHECK(rpt.power_tsvs >= 36);
    CHECK(rpt.total_d2d_links == 2);
    CHECK(rpt.total_bumps == 20);
    CHECK(rpt.total_d2d_bandwidth_gbps > 4000); // 64×32 + 128×16

    // 8. Yield estimate
    sf::YieldModel ym;
    double yield = pkg.estimate_yield(ym);
    CHECK(yield > 0);
    CHECK(yield < 1.0); // 3-die = imperfect yield

    // 9. Worst-case TSV delay
    double wc_delay = pkg.worst_case_tsv_delay_ps();
    CHECK(wc_delay > 0);
    CHECK(wc_delay < 100); // should be < 100 ps

    std::cout << "PASS [3-die stack: " << rpt.total_tsvs << " TSVs, "
              << rpt.total_bumps << " bumps, "
              << rpt.total_d2d_bandwidth_gbps << " Gbps, "
              << "yield=" << (yield * 100) << "%]\n";
    tests_passed++;
}

// ── Main ────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << "  Phase 40: Chiplet/Multi-Die Integration Tests\n";
    std::cout << "═══════════════════════════════════════════════════════════\n\n";

    test_tsv_parasitics_physics();
    test_tsv_coupling();
    test_tsv_tech_sensitivity();
    test_die_management();
    test_signal_tsv_parasitics();
    test_power_tsv();
    test_spare_tsv();
    test_micro_bump_signal();
    test_power_bump();
    test_interposer_trace();
    test_d2d_link();
    test_3d_power_grid();
    test_yield_single_die();
    test_yield_tsv_array();
    test_package_yield();
    test_tsv_signal_integrity();
    test_tsv_spacing_drc();
    test_spare_assignment();
    test_legacy_api();
    test_si_aware_insertion();
    test_grid_aligned_placement();
    test_interposer_routing();
    test_chiplet_report();
    test_edge_cases();
    test_e2e_chiplet_flow();

    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << "  Results: " << tests_passed << "/" << tests_run << " PASSED\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
