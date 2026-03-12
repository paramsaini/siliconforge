// SiliconForge — Phase 41: Thermal Analysis Tests
// 25 comprehensive tests: steady-state, transient, 3D stack, hotspots,
// DVFS, sensors, coupled electro-thermal, package model, materials, E2E

#include "timing/thermal.hpp"
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

// ── Test 1: Material Properties ─────────────────────────────────────────────
void test_material_properties() {
    RUN("Material thermal properties");
    auto si = sf::silicon_material();
    CHECK(si.conductivity == 148.0);
    CHECK(si.specific_heat == 712.0);
    CHECK(si.density == 2329.0);
    CHECK(si.diffusivity() > 0);
    // α = k/(ρ·cp) = 148/(2329×712) ≈ 8.93e-5 m²/s
    CHECKF(si.diffusivity(), 8e-5, 1e-4);

    auto cu = sf::copper_material();
    CHECK(cu.conductivity > si.conductivity); // Cu more conductive

    auto sio2 = sf::sio2_material();
    CHECK(sio2.conductivity < si.conductivity); // SiO2 insulator
    PASS();
}

// ── Test 2: Package Thermal Model ───────────────────────────────────────────
void test_package_model() {
    RUN("Package Rjc/Rca thermal model");
    sf::PackageThermalModel pkg;
    pkg.t_ambient = 25.0;
    pkg.r_jc = 0.5;
    pkg.r_ca = 5.0;
    pkg.r_jb = 2.0;

    double r_total = pkg.total_resistance();
    CHECK(r_total > 0);
    // Parallel: (0.5+5.0) × 2.0 / (0.5+5.0+2.0) = 11.0/7.5 ≈ 1.47
    CHECKF(r_total, 1.0, 2.0);

    // Junction temp at 10W
    double tj = pkg.junction_temp(10.0);
    CHECK(tj > 25.0); // above ambient
    CHECK(tj < 80.0); // reasonable for 10W

    // Heatsink reduces thermal resistance
    pkg.has_heatsink = true;
    pkg.heatsink_r = 0.3;
    double r_with_hs = pkg.total_resistance();
    CHECK(r_with_hs < r_total); // heatsink helps
    PASS();
}

// ── Test 3: Uniform Power Steady-State ──────────────────────────────────────
void test_uniform_steady_state() {
    RUN("Uniform power steady-state solve");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 32;
    cfg.grid_ny = 32;
    cfg.die_width_um = 10000;
    cfg.die_height_um = 10000;
    cfg.package.t_ambient = 25.0;

    sf::ThermalAnalyzer ta(cfg);
    ta.set_uniform_power(5.0); // 5W uniform

    auto result = ta.solve_steady_state();

    CHECK(result.converged);
    CHECK(result.max_temperature > 25.0);
    CHECK(result.min_temperature >= 25.0);
    CHECK(result.avg_temperature > 25.0);
    CHECK(result.thermal_gradient >= 0);
    CHECK(result.total_power_w > 4.5 && result.total_power_w < 5.5);
    CHECK(result.temperature_map.size() == 32 * 32);
    PASS();
}

// ── Test 4: Block Power (Hotspot) ───────────────────────────────────────────
void test_block_power_hotspot() {
    RUN("Block power creates hotspot");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 32;
    cfg.grid_ny = 32;
    cfg.die_width_um = 10000;
    cfg.die_height_um = 10000;

    sf::ThermalAnalyzer ta(cfg);
    // Concentrate 10W in a small region (ALU core)
    ta.set_block_power(4000, 4000, 6000, 6000, 10.0);

    auto result = ta.solve_steady_state();

    CHECK(result.converged);
    CHECK(result.thermal_gradient > 0.5); // should have significant gradient
    CHECK(result.max_temperature > result.min_temperature);
    // Center should be hotter
    int cx = 16, cy = 16;
    double center_t = result.temperature_map[cx * 32 + cy];
    double edge_t = result.temperature_map[0 * 32 + 0]; // corner
    CHECK(center_t > edge_t);
    PASS();
}

// ── Test 5: More Power = Higher Temperature ─────────────────────────────────
void test_power_temperature_scaling() {
    RUN("Power-temperature linear scaling");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 16;
    cfg.grid_ny = 16;

    sf::ThermalAnalyzer ta1(cfg);
    ta1.set_uniform_power(5.0);
    auto r1 = ta1.solve_steady_state();

    sf::ThermalAnalyzer ta2(cfg);
    ta2.set_uniform_power(10.0);
    auto r2 = ta2.solve_steady_state();

    CHECK(r2.max_temperature > r1.max_temperature);
    CHECK(r2.avg_temperature > r1.avg_temperature);
    PASS();
}

// ── Test 6: SOR Convergence ─────────────────────────────────────────────────
void test_sor_convergence() {
    RUN("SOR solver convergence");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 16;
    cfg.grid_ny = 16;
    cfg.max_iterations = 10000;
    cfg.convergence_tol = 0.001;

    sf::ThermalAnalyzer ta(cfg);
    ta.set_uniform_power(3.0);
    auto result = ta.solve_steady_state();

    CHECK(result.converged);
    CHECK(result.iterations > 0);
    CHECK(result.iterations < cfg.max_iterations); // converged before max
    PASS();
}

// ── Test 7: Transient Warmup ────────────────────────────────────────────────
void test_transient_warmup() {
    RUN("Transient thermal warmup");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 16;
    cfg.grid_ny = 16;
    cfg.transient_steps = 50;
    cfg.time_step_us = 0.1;

    sf::ThermalAnalyzer ta(cfg);
    ta.set_uniform_power(5.0);

    auto results = ta.solve_transient();

    CHECK(results.size() > 1);
    // Temperature should increase over time
    CHECK(results.back().max_temperature >= results.front().max_temperature);
    // First step should be close to ambient
    CHECKF(results.front().min_temperature, 20.0, 35.0);
    PASS();
}

// ── Test 8: 3D Die Stack ────────────────────────────────────────────────────
void test_3d_die_stack() {
    RUN("3D multi-die thermal stack");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 16;
    cfg.grid_ny = 16;

    sf::ThermalAnalyzer ta(cfg);
    std::vector<double> die_powers = {10.0, 5.0, 2.0}; // bottom to top

    auto result = ta.solve_3d_stack(die_powers, 3);

    CHECK(result.total_power_w > 16.0); // sum ≈ 17W
    CHECK(result.max_temperature > cfg.package.t_ambient);
    CHECK(result.converged);
    PASS();
}

// ── Test 9: 3D Stack — Top Die Hotter ───────────────────────────────────────
void test_3d_top_die_hotter() {
    RUN("3D stack: top die temperature accumulates");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 16;
    cfg.grid_ny = 16;

    sf::ThermalAnalyzer ta(cfg);
    // Equal power per die — top should still be hotter (farther from heatsink)
    std::vector<double> powers_3die = {5.0, 5.0, 5.0};
    auto r3 = ta.solve_3d_stack(powers_3die, 3);

    std::vector<double> powers_1die = {5.0};
    auto r1 = ta.solve_3d_stack(powers_1die, 1);

    // 3 dies with same power should be hotter than 1 die
    CHECK(r3.max_temperature > r1.max_temperature);
    PASS();
}

// ── Test 10: Hotspot Detection ──────────────────────────────────────────────
void test_hotspot_detection() {
    RUN("Hotspot detection and ranking");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 32;
    cfg.grid_ny = 32;

    sf::ThermalAnalyzer ta(cfg);
    // Create hot block
    ta.set_block_power(4000, 4000, 6000, 6000, 15.0);
    ta.solve_steady_state();

    // Low threshold → many hotspots
    auto hs_low = ta.find_hotspots(25.0);
    // High threshold → fewer hotspots
    auto hs_high = ta.find_hotspots(100.0);

    CHECK(hs_low.size() >= hs_high.size());
    // Hotspots should be sorted by temperature descending
    if (hs_low.size() >= 2) {
        CHECK(hs_low[0].temperature >= hs_low[1].temperature);
    }
    PASS();
}

// ── Test 11: DVFS Turbo Mode ────────────────────────────────────────────────
void test_dvfs_turbo() {
    RUN("DVFS turbo at low temperature");
    sf::ThermalAnalyzer ta;
    sf::DvfsConfig dvfs;
    auto state = ta.compute_dvfs(50.0, dvfs); // cool chip

    CHECK(state.mode == "turbo");
    CHECK(state.frequency_ghz == dvfs.turbo_freq_ghz);
    CHECK(state.voltage_v == dvfs.turbo_voltage_v);
    PASS();
}

// ── Test 12: DVFS Throttle Mode ─────────────────────────────────────────────
void test_dvfs_throttle() {
    RUN("DVFS throttle at high temperature");
    sf::ThermalAnalyzer ta;
    sf::DvfsConfig dvfs;
    auto state = ta.compute_dvfs(90.0, dvfs);

    CHECK(state.mode == "throttle");
    CHECK(state.frequency_ghz == dvfs.throttle_freq_ghz);
    CHECK(state.voltage_v < dvfs.nominal_voltage_v);
    PASS();
}

// ── Test 13: DVFS Emergency Mode ────────────────────────────────────────────
void test_dvfs_emergency() {
    RUN("DVFS emergency at critical temperature");
    sf::ThermalAnalyzer ta;
    sf::DvfsConfig dvfs;
    auto state = ta.compute_dvfs(105.0, dvfs);

    CHECK(state.mode == "emergency");
    CHECK(state.frequency_ghz == dvfs.emergency_freq_ghz);

    // Critical mode
    auto crit = ta.compute_dvfs(115.0, dvfs);
    CHECK(crit.mode == "critical");
    PASS();
}

// ── Test 14: DVFS Power Scaling ─────────────────────────────────────────────
void test_dvfs_power_scaling() {
    RUN("DVFS power scaling (P ∝ V²f)");
    sf::DvfsConfig dvfs;
    sf::DvfsState nominal;
    nominal.frequency_ghz = dvfs.nominal_freq_ghz;
    nominal.voltage_v = dvfs.nominal_voltage_v;
    double p_nominal = dvfs.power_scaling(nominal);
    CHECKF(p_nominal, 0.99, 1.01); // normalized to 1.0

    sf::DvfsState throttle;
    throttle.frequency_ghz = dvfs.throttle_freq_ghz;
    throttle.voltage_v = dvfs.throttle_voltage_v;
    double p_throttle = dvfs.power_scaling(throttle);
    CHECK(p_throttle < p_nominal); // throttle uses less power

    sf::DvfsState turbo;
    turbo.frequency_ghz = dvfs.turbo_freq_ghz;
    turbo.voltage_v = dvfs.turbo_voltage_v;
    double p_turbo = dvfs.power_scaling(turbo);
    CHECK(p_turbo > p_nominal); // turbo uses more power
    PASS();
}

// ── Test 15: Sensor Placement ───────────────────────────────────────────────
void test_sensor_placement() {
    RUN("Thermal sensor placement");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 32;
    cfg.grid_ny = 32;

    sf::ThermalAnalyzer ta(cfg);
    ta.set_uniform_power(5.0);
    ta.solve_steady_state();
    ta.place_sensors(9);

    auto sensors = ta.read_sensors();
    CHECK(sensors.size() == 9);
    // Each sensor should have a valid reading
    for (const auto& s : sensors) {
        CHECK(s.reading >= 20.0); // above freezing
        CHECK(s.id >= 0);
    }
    PASS();
}

// ── Test 16: Sensor Reads Grid Temperature ──────────────────────────────────
void test_sensor_grid_reading() {
    RUN("Sensor reads actual grid temperature");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 16;
    cfg.grid_ny = 16;

    sf::ThermalAnalyzer ta(cfg);
    ta.set_uniform_power(8.0);
    ta.solve_steady_state();
    ta.place_sensors(4);

    auto readings = ta.read_sensors();
    CHECK(readings.size() == 4);
    // All sensors should read above ambient
    for (const auto& s : readings) {
        CHECK(s.reading > cfg.package.t_ambient);
    }
    PASS();
}

// ── Test 17: Timing Derating ────────────────────────────────────────────────
void test_timing_derating() {
    RUN("Timing derating with temperature");
    sf::ThermalAnalyzer ta;

    double d25 = ta.timing_derating(25.0);   // room temp
    double d85 = ta.timing_derating(85.0);   // typical worst-case
    double d125 = ta.timing_derating(125.0); // extreme

    CHECKF(d25, 0.99, 1.01); // baseline ≈ 1.0
    CHECK(d85 > d25);   // hotter = slower
    CHECK(d125 > d85);  // even hotter = even slower
    // At 125°C: (398.15/298.15)^1.5 ≈ 1.54
    CHECKF(d125, 1.3, 1.7);
    PASS();
}

// ── Test 18: Leakage Scaling ────────────────────────────────────────────────
void test_leakage_scaling() {
    RUN("Leakage power scaling with temperature");
    sf::ThermalAnalyzer ta;

    double l25 = ta.leakage_scaling(25.0);
    double l85 = ta.leakage_scaling(85.0);
    double l125 = ta.leakage_scaling(125.0);

    CHECKF(l25, 0.99, 1.01); // baseline
    CHECK(l85 > l25);
    CHECK(l125 > l85);
    // Doubles every ~10°C: at +60°C, ~2^6 = 64×
    CHECKF(l85, 30.0, 100.0);
    PASS();
}

// ── Test 19: Power From Cells ───────────────────────────────────────────────
void test_power_from_cells() {
    RUN("Power map from physical design cells");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 16;
    cfg.grid_ny = 16;
    cfg.die_width_um = 10000;
    cfg.die_height_um = 10000;

    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 10000, 10000);
    for (int i = 0; i < 10; i++) {
        int cid = pd.add_cell("cell_" + std::to_string(i), "NAND", 3, 3);
        pd.cells[cid].position = sf::Point(1000 + i * 800, 5000);
        pd.cells[cid].placed = true;
    }

    std::vector<double> powers(10, 50.0); // 50mW each

    sf::ThermalAnalyzer ta(cfg);
    ta.set_power_from_cells(pd, powers);
    auto result = ta.solve_steady_state();

    CHECK(result.converged);
    CHECK(result.max_temperature > cfg.package.t_ambient);
    PASS();
}

// ── Test 20: Zero Power = Ambient ───────────────────────────────────────────
void test_zero_power_ambient() {
    RUN("Zero power → ambient temperature");
    sf::ThermalConfig cfg;
    cfg.grid_nx = 16;
    cfg.grid_ny = 16;
    cfg.package.t_ambient = 30.0;

    sf::ThermalAnalyzer ta(cfg);
    // No power set → should converge to ambient
    auto result = ta.solve_steady_state();

    CHECK(result.converged);
    CHECKF(result.max_temperature, 29.0, 31.0);
    CHECKF(result.avg_temperature, 29.0, 31.0);
    CHECK(result.thermal_gradient < 2.0);
    PASS();
}

// ── Test 21: Heatsink Effect ────────────────────────────────────────────────
void test_heatsink_effect() {
    RUN("Heatsink reduces junction temperature");
    sf::ThermalConfig cfg_no_hs, cfg_hs;
    cfg_no_hs.grid_nx = cfg_hs.grid_nx = 16;
    cfg_no_hs.grid_ny = cfg_hs.grid_ny = 16;
    cfg_hs.package.has_heatsink = true;
    cfg_hs.package.heatsink_r = 0.3;

    sf::ThermalAnalyzer ta_no(cfg_no_hs);
    ta_no.set_uniform_power(10.0);
    auto r_no = ta_no.solve_steady_state();

    sf::ThermalAnalyzer ta_hs(cfg_hs);
    ta_hs.set_uniform_power(10.0);
    auto r_hs = ta_hs.solve_steady_state();

    CHECK(r_hs.max_temperature < r_no.max_temperature);
    PASS();
}

// ── Test 22: Grid Resolution Independence ───────────────────────────────────
void test_grid_resolution() {
    RUN("Grid resolution consistency");
    double power = 5.0;

    sf::ThermalConfig cfg16;
    cfg16.grid_nx = cfg16.grid_ny = 16;
    sf::ThermalAnalyzer ta16(cfg16);
    ta16.set_uniform_power(power);
    auto r16 = ta16.solve_steady_state();

    sf::ThermalConfig cfg32;
    cfg32.grid_nx = cfg32.grid_ny = 32;
    sf::ThermalAnalyzer ta32(cfg32);
    ta32.set_uniform_power(power);
    auto r32 = ta32.solve_steady_state();

    // Results should be similar regardless of grid resolution (within 20%)
    double diff = std::abs(r16.avg_temperature - r32.avg_temperature);
    CHECK(diff < 0.2 * std::abs(r32.avg_temperature - cfg32.package.t_ambient) + 1.0);
    PASS();
}

// ── Test 23: DVFS Full State Machine ────────────────────────────────────────
void test_dvfs_full_state_machine() {
    RUN("DVFS full state machine transitions");
    sf::DvfsConfig dvfs;

    // Test all transitions
    auto s1 = dvfs.compute_state(30.0);
    CHECK(s1.mode == "turbo");

    auto s2 = dvfs.compute_state(75.0);
    CHECK(s2.mode == "nominal");

    auto s3 = dvfs.compute_state(90.0);
    CHECK(s3.mode == "throttle");

    auto s4 = dvfs.compute_state(105.0);
    CHECK(s4.mode == "emergency");

    auto s5 = dvfs.compute_state(115.0);
    CHECK(s5.mode == "critical");

    // Frequencies should decrease with temperature
    CHECK(s1.frequency_ghz >= s2.frequency_ghz);
    CHECK(s2.frequency_ghz >= s3.frequency_ghz);
    CHECK(s3.frequency_ghz >= s4.frequency_ghz);
    PASS();
}

// ── Test 24: Edge Cases ─────────────────────────────────────────────────────
void test_edge_cases() {
    RUN("Edge cases (tiny grid, zero dies, single sensor)");

    // Tiny grid
    sf::ThermalConfig cfg;
    cfg.grid_nx = 4;
    cfg.grid_ny = 4;
    sf::ThermalAnalyzer ta(cfg);
    ta.set_uniform_power(1.0);
    auto r = ta.solve_steady_state();
    CHECK(r.temperature_map.size() == 16);

    // 3D with zero dies → empty result
    auto r0 = ta.solve_3d_stack({}, 0);
    CHECK(r0.total_power_w == 0);

    // Single sensor
    ta.place_sensors(1);
    auto sensors = ta.read_sensors();
    CHECK(sensors.size() == 1);

    // Out-of-bounds power density — should not crash
    ta.set_power_density(-1, -1, 1000.0);
    ta.set_power_density(999, 999, 1000.0);
    auto r2 = ta.solve_steady_state();
    CHECK(r2.converged);

    PASS();
}

// ── Test 25: E2E Thermal Analysis Flow ──────────────────────────────────────
void test_e2e_thermal_flow() {
    RUN("E2E: Full thermal analysis flow");

    // 1. Configure for a realistic chip
    sf::ThermalConfig cfg;
    cfg.grid_nx = 32;
    cfg.grid_ny = 32;
    cfg.die_width_um = 12000;
    cfg.die_height_um = 12000;
    cfg.die_thickness_um = 100;
    cfg.material = sf::silicon_material();
    cfg.package.r_jc = 0.4;
    cfg.package.r_ca = 4.0;
    cfg.package.r_jb = 1.5;
    cfg.package.t_ambient = 25.0;
    cfg.package.has_heatsink = true;
    cfg.package.heatsink_r = 0.25;

    sf::ThermalAnalyzer ta(cfg);

    // 2. Set block-level power map
    ta.set_block_power(1000, 1000, 5000, 5000, 8.0);   // CPU core
    ta.set_block_power(6000, 1000, 11000, 5000, 3.0);  // cache
    ta.set_block_power(1000, 6000, 11000, 11000, 2.0); // I/O
    // Total: 13W

    // 3. Steady-state solve
    auto ss = ta.solve_steady_state();
    CHECK(ss.converged);
    CHECK(ss.total_power_w > 12.0 && ss.total_power_w < 14.0);
    CHECK(ss.max_temperature > cfg.package.t_ambient);

    // 4. Hotspot detection — detect cells above the cooler baseline
    double baseline = ss.min_temperature + 0.5 * ss.thermal_gradient;
    auto hotspots = ta.find_hotspots(baseline);
    // With non-uniform power, should have some cells above baseline
    CHECK(hotspots.size() > 0);

    // 5. DVFS decision
    sf::DvfsConfig dvfs;
    auto state = ta.compute_dvfs(ss.max_temperature, dvfs);
    CHECK(state.mode.size() > 0);

    // 6. Sensor network
    ta.place_sensors(16);
    auto sensors = ta.read_sensors();
    CHECK(sensors.size() == 16);
    double sensor_max = 0;
    for (const auto& s : sensors) sensor_max = std::max(sensor_max, s.reading);
    CHECK(sensor_max > cfg.package.t_ambient);

    // 7. Coupled analysis
    double derating = ta.timing_derating(ss.max_temperature);
    CHECK(derating >= 1.0); // hot = slower
    double leak_scale = ta.leakage_scaling(ss.max_temperature);
    CHECK(leak_scale >= 1.0); // hot = more leakage

    // 8. 3D stack analysis
    auto r3d = ta.solve_3d_stack({8.0, 3.0, 2.0}, 3);
    CHECK(r3d.max_temperature > ss.max_temperature * 0.5); // stack adds heat

    std::cout << "PASS [Tmax=" << ss.max_temperature << "°C, "
              << "Tavg=" << ss.avg_temperature << "°C, "
              << "gradient=" << ss.thermal_gradient << "°C, "
              << "hotspots=" << hotspots.size() << ", "
              << "DVFS=" << state.mode << ", "
              << "derating=" << derating << "x]\n";
    tests_passed++;
}

// ── Main ────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << "  Phase 41: Thermal Analysis Tests\n";
    std::cout << "═══════════════════════════════════════════════════════════\n\n";

    test_material_properties();
    test_package_model();
    test_uniform_steady_state();
    test_block_power_hotspot();
    test_power_temperature_scaling();
    test_sor_convergence();
    test_transient_warmup();
    test_3d_die_stack();
    test_3d_top_die_hotter();
    test_hotspot_detection();
    test_dvfs_turbo();
    test_dvfs_throttle();
    test_dvfs_emergency();
    test_dvfs_power_scaling();
    test_sensor_placement();
    test_sensor_grid_reading();
    test_timing_derating();
    test_leakage_scaling();
    test_power_from_cells();
    test_zero_power_ambient();
    test_heatsink_effect();
    test_grid_resolution();
    test_dvfs_full_state_machine();
    test_edge_cases();
    test_e2e_thermal_flow();

    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << "  Results: " << tests_passed << "/" << tests_run << " PASSED\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
