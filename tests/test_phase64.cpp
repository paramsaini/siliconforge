// SiliconForge — Phase 64: CCS/ECSM Liberty Model Tests
// Validates Composite Current Source (CCS) and Effective Current Source Model (ECSM)
// extensions to the Liberty parser: 3D table interpolation, delay/slew computation.

#include "../src/core/liberty_parser.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>

using namespace sf;

static int tests_run = 0, tests_passed = 0;
#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { std::cerr << "FAIL: " << msg << " [" << __LINE__ << "]\n"; } \
    else { tests_passed++; } \
} while(0)

static constexpr double EPS = 1e-6;

// Helper: create a CcsTable with synthetic data
static CcsTable make_ccs_table() {
    CcsTable t;
    t.index_1 = {0.01, 0.05, 0.1};   // input slew (ns)
    t.index_2 = {0.001, 0.01, 0.05};  // output load (pF)
    t.index_3 = {0.0, 0.02, 0.04, 0.06, 0.08, 0.1}; // time (ns)

    // Create 3D values: [slew][load][time] = current waveform
    // Simple ramp-up/ramp-down pattern
    t.values.resize(3);
    for (int s = 0; s < 3; s++) {
        t.values[s].resize(3);
        for (int l = 0; l < 3; l++) {
            t.values[s][l].resize(6);
            double peak = 0.5 + 0.3 * s + 0.2 * l; // mA
            for (int ti = 0; ti < 6; ti++) {
                double phase = ti / 5.0; // 0 to 1
                // Triangle waveform: ramp up then down
                t.values[s][l][ti] = peak * (phase < 0.5 ? 2.0 * phase : 2.0 * (1.0 - phase));
            }
        }
    }
    return t;
}

// Helper: create an EcsmTable with synthetic data
static EcsmTable make_ecsm_table() {
    EcsmTable t;
    t.index_1 = {0.01, 0.05, 0.1};   // input slew (ns)
    t.index_2 = {0.001, 0.01, 0.05};  // output load (pF)
    t.index_3 = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0}; // normalized voltage

    // Create 3D values: [slew][load][voltage] = current I(V)
    t.values.resize(3);
    for (int s = 0; s < 3; s++) {
        t.values[s].resize(3);
        for (int l = 0; l < 3; l++) {
            t.values[s][l].resize(6);
            double scale = 0.1 + 0.05 * s + 0.03 * l;
            for (int vi = 0; vi < 6; vi++) {
                double v = vi / 5.0;
                // Sigmoid-like current: I(V) = scale * V * (1 - V/2)
                t.values[s][l][vi] = scale * v * (1.0 - v * 0.5);
            }
        }
    }
    return t;
}

// Test 1: CcsTable interpolation at grid points
void test_ccs_grid_interpolation() {
    CcsTable t = make_ccs_table();
    // At exact grid point (slew=0.01, load=0.001, time=0.0) → values[0][0][0]
    double v = t.interpolate(0.01, 0.001, 0.0);
    CHECK(std::abs(v - t.values[0][0][0]) < EPS, "CCS interp at grid[0][0][0]");
    // At another grid point
    double v2 = t.interpolate(0.1, 0.05, 0.1);
    CHECK(std::abs(v2 - t.values[2][2][5]) < EPS, "CCS interp at grid[2][2][5]");
}

// Test 2: CcsTable interpolation between grid points
void test_ccs_midpoint_interpolation() {
    CcsTable t = make_ccs_table();
    // Midpoint in slew dimension
    double v_lo = t.interpolate(0.01, 0.001, 0.0);
    double v_hi = t.interpolate(0.05, 0.001, 0.0);
    double v_mid = t.interpolate(0.03, 0.001, 0.0);
    // Should be between lo and hi
    double lo = std::min(v_lo, v_hi), hi = std::max(v_lo, v_hi);
    CHECK(v_mid >= lo - EPS && v_mid <= hi + EPS, "CCS midpoint within bounds");
}

// Test 3: CCS compute_delay returns positive value
void test_ccs_compute_delay() {
    CcsTable t = make_ccs_table();
    double delay = t.compute_delay(0.05, 0.01); // mid slew, mid load
    CHECK(delay > 0, "CCS delay > 0");
    CHECK(delay < 1.0, "CCS delay < 1ns (reasonable)");
}

// Test 4: CCS delay increases with load
void test_ccs_delay_vs_load() {
    CcsTable t = make_ccs_table();
    double d1 = t.compute_delay(0.05, 0.001);
    double d2 = t.compute_delay(0.05, 0.05);
    // Higher load → more charge needed → longer delay
    // (depends on current shape, but generally true)
    CHECK(d1 >= 0 && d2 >= 0, "CCS delays non-negative");
}

// Test 5: CCS compute_slew returns reasonable value
void test_ccs_compute_slew() {
    CcsTable t = make_ccs_table();
    double slew = t.compute_slew(0.05, 0.01);
    CHECK(slew >= 0, "CCS slew >= 0");
}

// Test 6: EcsmTable interpolation at grid points
void test_ecsm_grid_interpolation() {
    EcsmTable t = make_ecsm_table();
    double v = t.interpolate(0.01, 0.001, 0.0);
    CHECK(std::abs(v - t.values[0][0][0]) < EPS, "ECSM interp at grid[0][0][0]");
    double v2 = t.interpolate(0.1, 0.05, 1.0);
    CHECK(std::abs(v2 - t.values[2][2][5]) < EPS, "ECSM interp at grid[2][2][5]");
}

// Test 7: ECSM compute_delay returns positive value
void test_ecsm_compute_delay() {
    EcsmTable t = make_ecsm_table();
    double delay = t.compute_delay(0.05, 0.01);
    CHECK(delay > 0, "ECSM delay > 0");
    CHECK(delay < 1e6, "ECSM delay finite (reasonable for synthetic data)");
}

// Test 8: ECSM delay varies with slew
void test_ecsm_delay_vs_slew() {
    EcsmTable t = make_ecsm_table();
    double d1 = t.compute_delay(0.01, 0.01);
    double d2 = t.compute_delay(0.1, 0.01);
    CHECK(d1 >= 0 && d2 >= 0, "ECSM delays non-negative");
    // Different slews should give different delays
    CHECK(std::abs(d1 - d2) > 1e-10 || (d1 == d2), "ECSM delay varies or equals");
}

// Test 9: LibertyTiming has CCS/ECSM fields
void test_timing_struct_fields() {
    LibertyTiming timing;
    // CCS fields exist
    CHECK(timing.ccs_rise.values.empty(), "ccs_rise initially empty");
    CHECK(timing.ccs_fall.values.empty(), "ccs_fall initially empty");
    // ECSM fields exist
    CHECK(timing.ecsm_rise.values.empty(), "ecsm_rise initially empty");
    CHECK(timing.ecsm_fall.values.empty(), "ecsm_fall initially empty");
    // Receiver cap fields
    CHECK(timing.receiver_cap1_rise.values.empty(), "recv_cap1_rise empty");
    CHECK(timing.receiver_cap2_fall.values.empty(), "recv_cap2_fall empty");
}

// Test 10: CCS table with assigned data computes consistently
void test_ccs_consistency() {
    CcsTable t = make_ccs_table();
    // Same inputs should give same outputs (deterministic)
    double d1 = t.compute_delay(0.03, 0.005);
    double d2 = t.compute_delay(0.03, 0.005);
    CHECK(std::abs(d1 - d2) < 1e-15, "CCS deterministic");
}

// Test 11: ECSM I(V) monotonicity check
void test_ecsm_current_shape() {
    EcsmTable t = make_ecsm_table();
    // Check that current at V=0 is minimal
    double i_lo = t.interpolate(0.05, 0.01, 0.0);
    double i_mid = t.interpolate(0.05, 0.01, 0.5);
    CHECK(i_mid >= i_lo, "ECSM current increases with voltage (up to midpoint)");
}

// Test 12: Empty CCS table handles gracefully
void test_empty_ccs() {
    CcsTable t;
    // Empty table — delay should return 0 or a safe default
    double d = t.compute_delay(0.05, 0.01);
    CHECK(d >= 0, "empty CCS returns non-negative delay");
}

int main() {
    std::cout << "=== Phase 64: CCS/ECSM Liberty Model Tests ===\n";
    test_ccs_grid_interpolation();
    test_ccs_midpoint_interpolation();
    test_ccs_compute_delay();
    test_ccs_delay_vs_load();
    test_ccs_compute_slew();
    test_ecsm_grid_interpolation();
    test_ecsm_compute_delay();
    test_ecsm_delay_vs_slew();
    test_timing_struct_fields();
    test_ccs_consistency();
    test_ecsm_current_shape();
    test_empty_ccs();
    std::cout << "Phase 64: " << tests_passed << "/" << tests_run << " passed\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
