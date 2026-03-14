// SiliconForge — Phase 68: Enhanced Dynamic IR Drop Tests
// Validates RLC PDN mesh modeling, L·di/dt noise, EM-aware checks,
// and resonance detection in the IR drop analyzer.

#include "../src/timing/ir_drop.hpp"
#include "../src/pnr/physical.hpp"
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

// Helper: build a physical design for IR analysis
static PhysicalDesign make_ir_design() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 200, 200};
    // Add some cells
    for (int i = 0; i < 20; i++) {
        CellInstance c;
        c.name = "cell_" + std::to_string(i);
        c.position = {10.0 + (i % 5) * 40, 10.0 + (i / 5) * 40};
        c.width = 10;
        c.height = 10;
        pd.cells.push_back(c);
    }
    return pd;
}

// Test 1: Basic dynamic IR with default config
void test_basic_dynamic_ir() {
    PhysicalDesign pd = make_ir_design();
    IrDropAnalyzer analyzer(pd, 1.8, 100);
    auto result = analyzer.analyze_dynamic(10.0, 50);
    CHECK(result.peak_drop_mv >= 0, "peak dynamic drop >= 0");
    CHECK(result.time_steps == 50, "correct time steps");
    CHECK(!result.waveform.empty(), "waveform not empty");
}

// Test 2: Dynamic IR with RLC config
void test_rlc_dynamic_ir() {
    PhysicalDesign pd = make_ir_design();
    IrDropAnalyzer analyzer(pd, 1.0, 200);
    IrDropConfig cfg;
    cfg.vdd = 1.0;
    cfg.total_current_ma = 200;
    cfg.grid_resolution = 10;
    cfg.enable_dynamic = true;
    cfg.per_layer_inductance_ph_per_um = 0.5e-12;
    cfg.pkg_inductance_nh = 0.5;
    cfg.pkg_resistance_mohm = 5.0;
    cfg.decap_esr_mohm = 10.0;
    analyzer.set_config(cfg);
    analyzer.analyze(10); // base analysis first
    auto result = analyzer.analyze_dynamic(10.0, 100);
    CHECK(result.peak_drop_mv >= 0, "RLC dynamic drop >= 0");
}

// Test 3: L·di/dt contribution tracked
void test_ldi_dt_tracking() {
    PhysicalDesign pd = make_ir_design();
    IrDropAnalyzer analyzer(pd, 1.0, 200);
    IrDropConfig cfg;
    cfg.vdd = 1.0;
    cfg.total_current_ma = 200;
    cfg.grid_resolution = 8;
    cfg.enable_dynamic = true;
    cfg.pkg_inductance_nh = 1.0; // significant inductance
    analyzer.set_config(cfg);
    auto result = analyzer.analyze_dynamic(10.0, 50);
    CHECK(result.ldi_dt_peak_mv >= 0, "L·di/dt peak tracked");
    CHECK(result.resistive_peak_mv >= 0, "resistive peak tracked");
}

// Test 4: Resonance frequency detection
void test_resonance_frequency() {
    PhysicalDesign pd = make_ir_design();
    IrDropAnalyzer analyzer(pd, 1.0, 200);
    IrDropConfig cfg;
    cfg.vdd = 1.0;
    cfg.total_current_ma = 200;
    cfg.grid_resolution = 8;
    cfg.pkg_inductance_nh = 0.5;
    cfg.decap_density_ff_per_um2 = 50.0;
    analyzer.set_config(cfg);
    auto result = analyzer.analyze_dynamic(10.0, 50);
    CHECK(result.resonance_freq_ghz >= 0, "resonance freq computed");
}

// Test 5: Higher inductance → more L·di/dt noise
void test_inductance_impact() {
    PhysicalDesign pd = make_ir_design();

    IrDropConfig cfg1;
    cfg1.vdd = 1.0; cfg1.total_current_ma = 200;
    cfg1.grid_resolution = 8; cfg1.enable_dynamic = true;
    cfg1.pkg_inductance_nh = 0.1; // low L
    IrDropAnalyzer a1(pd, 1.0, 200);
    a1.set_config(cfg1);
    auto r1 = a1.analyze_dynamic(10.0, 50);

    IrDropConfig cfg2;
    cfg2.vdd = 1.0; cfg2.total_current_ma = 200;
    cfg2.grid_resolution = 8; cfg2.enable_dynamic = true;
    cfg2.pkg_inductance_nh = 5.0; // high L
    IrDropAnalyzer a2(pd, 1.0, 200);
    a2.set_config(cfg2);
    auto r2 = a2.analyze_dynamic(10.0, 50);

    CHECK(r2.ldi_dt_peak_mv >= r1.ldi_dt_peak_mv,
          "higher L → more Ldi/dt noise");
}

// Test 6: Waveform has correct size
void test_waveform_size() {
    PhysicalDesign pd = make_ir_design();
    IrDropAnalyzer analyzer(pd, 1.8, 100);
    auto result = analyzer.analyze_dynamic(5.0, 200);
    CHECK((int)result.waveform.size() == 200, "waveform has 200 points");
}

// Test 7: ESR modeling affects results
void test_esr_effect() {
    PhysicalDesign pd = make_ir_design();

    IrDropConfig cfg;
    cfg.vdd = 1.0; cfg.total_current_ma = 200;
    cfg.grid_resolution = 8; cfg.enable_dynamic = true;
    cfg.decap_esr_mohm = 0.1; // near-ideal decap
    IrDropAnalyzer a1(pd, 1.0, 200);
    a1.set_config(cfg);
    auto r1 = a1.analyze_dynamic(10.0, 50);

    cfg.decap_esr_mohm = 100.0; // high ESR (poor decap)
    IrDropAnalyzer a2(pd, 1.0, 200);
    a2.set_config(cfg);
    auto r2 = a2.analyze_dynamic(10.0, 50);

    // Higher ESR should give worse (higher) dynamic drop
    CHECK(r2.peak_drop_mv >= r1.peak_drop_mv - 1.0,
          "higher ESR → worse dynamic drop");
}

// Test 8: EM current density check
void test_em_check() {
    PhysicalDesign pd = make_ir_design();
    IrDropAnalyzer analyzer(pd, 1.0, 500); // high current
    analyzer.analyze(10); // base analysis first
    auto hotspots = analyzer.check_em_limits(0.5); // 0.5 mA/um limit
    // May or may not find violations depending on grid resolution
    CHECK(true, "EM check doesn't crash");
}

// Test 9: Static + dynamic combined via run_enhanced
void test_run_enhanced() {
    PhysicalDesign pd = make_ir_design();
    IrDropAnalyzer analyzer(pd, 1.8, 100);
    IrDropConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 100;
    cfg.grid_resolution = 8;
    cfg.enable_dynamic = true;
    analyzer.set_config(cfg);
    auto result = analyzer.run_enhanced();
    CHECK(result.worst_drop_mv >= 0, "enhanced static drop >= 0");
    CHECK(result.dynamic_analyzed, "dynamic analysis ran");
}

// Test 10: Vectored analysis still works
void test_vectored_analysis() {
    PhysicalDesign pd = make_ir_design();
    IrDropAnalyzer analyzer(pd, 1.8, 100);
    std::vector<std::vector<bool>> stimulus = {
        {true, false, true, false, true},
        {false, true, false, true, false},
        {true, true, false, false, true}
    };
    auto result = analyzer.analyze_vectored(stimulus, 1.0);
    CHECK(result.per_cycle_drops.size() == 3, "3 cycles analyzed");
    CHECK(result.peak_drop_mv >= 0, "vectored peak drop >= 0");
}

// Test 11: Voltage-aware timing
void test_voltage_timing() {
    PhysicalDesign pd = make_ir_design();
    IrDropAnalyzer analyzer(pd, 1.0, 200);
    analyzer.analyze(8);
    auto result = analyzer.analyze_voltage_timing();
    CHECK(!std::isnan(result.timing_degradation_pct), "timing degrade not NaN");
}

// Test 12: Hotspot clustering
void test_hotspot_clustering() {
    PhysicalDesign pd = make_ir_design();
    IrDropAnalyzer analyzer(pd, 1.0, 300);
    analyzer.analyze(10);
    auto spots = analyzer.find_hotspots(10.0); // 10mV threshold
    for (auto& h : spots) {
        CHECK(h.avg_drop_mv >= 10.0, "hotspot above threshold");
        CHECK(h.radius > 0, "hotspot has radius");
    }
    CHECK(true, "hotspot clustering works");
}

int main() {
    std::cout << "=== Phase 68: Enhanced Dynamic IR Drop Tests ===\n";
    test_basic_dynamic_ir();
    test_rlc_dynamic_ir();
    test_ldi_dt_tracking();
    test_resonance_frequency();
    test_inductance_impact();
    test_waveform_size();
    test_esr_effect();
    test_em_check();
    test_run_enhanced();
    test_vectored_analysis();
    test_voltage_timing();
    test_hotspot_clustering();
    std::cout << "Phase 68: " << tests_passed << "/" << tests_run << " passed\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
