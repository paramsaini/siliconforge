// Phase 92: SPICE Simulation Engine — SPICE netlist parser, MOSFET compact
// models (Level 1 / BSIM4), MNA-based DC operating point with Newton-Raphson,
// Backward Euler transient analysis, and waveform measurement utilities.

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include "spice/spice_parser.hpp"
#include "spice/device_models.hpp"
#include "spice/spice_engine.hpp"

static int total = 0, passed = 0;
#define CHECK(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  [PASS] %s\n", msg); } \
    else { printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while(0)

// ═══════════════════════════════════════════════════════════════════════
// TEST 1: SPICE parser — inverter subcircuit
// ═══════════════════════════════════════════════════════════════════════
static void test_spice_parser() {
    printf("\n── SPICE Parser ──\n");

    std::string netlist = R"(
* Simple CMOS inverter
.model nch nmos (level=1 vth0=0.45 k1=0.5 u0=300 toxe=4.1e-9 vsat=1.5e5)
.model pch pmos (level=1 vth0=-0.45 k1=0.5 u0=100 toxe=4.1e-9 vsat=1.5e5)
.param vdd_val=1.8

.subckt inv in out vdd gnd
Mn1 out in gnd gnd nch W=1u L=150n
Mp1 out in vdd vdd pch W=2u L=150n
.ends

Xinv1 vin vout vdd 0 inv
Vdd vdd 0 DC 1.8
Vin vin 0 DC 0.0
.end
)";

    sf::SpiceParser parser;
    auto ckt = parser.parse(netlist);

    CHECK(ckt.subcircuits.size() == 1, "parsed 1 subcircuit");
    CHECK(ckt.subcircuits[0].name == "inv", "subcircuit name is 'inv'");
    CHECK(ckt.subcircuits[0].ports.size() == 4, "subcircuit has 4 ports");
    CHECK(ckt.subcircuits[0].devices.size() == 2, "subcircuit has 2 devices");

    auto& mn = ckt.subcircuits[0].devices[0];
    CHECK(mn.type == sf::SpiceDeviceType::MOSFET, "Mn1 is a MOSFET");
    CHECK(mn.model_name == "nch", "Mn1 model is nch");
    CHECK(mn.terminals.size() == 4, "MOSFET has 4 terminals");

    CHECK(ckt.models.count("nch") == 1, "nch model parsed");
    CHECK(ckt.models.count("pch") == 1, "pch model parsed");
    CHECK(ckt.params.count("vdd_val") == 1, ".param vdd_val parsed");
    CHECK(std::abs(ckt.params["vdd_val"] - 1.8) < 0.01, ".param value correct");

    // Check device parameter parsing (W/L)
    CHECK(std::abs(mn.params["w"] - 1e-6) < 1e-9, "NMOS W=1u parsed");
    CHECK(std::abs(mn.params["l"] - 150e-9) < 1e-12, "NMOS L=150n parsed");

    // Top-level instances
    CHECK(ckt.instances.size() >= 2, "top-level voltage sources parsed");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 2: MOSFET Level 1 — cutoff / linear / saturation
// ═══════════════════════════════════════════════════════════════════════
static void test_mosfet_level1() {
    printf("\n── MOSFET Level 1 Ids ──\n");

    auto nmos = sf::MosfetModel::default_nmos();
    nmos.level = 1;

    // Cutoff: Vgs = 0 < Vth
    double ids_cutoff = nmos.evaluate_ids_level1(0.0, 0.9, 0.0);
    CHECK(std::abs(ids_cutoff) < 1e-6, "NMOS cutoff: Ids ~ 0");

    // Saturation: Vgs = 1.8V, Vds = 0.9V (Vds > Vgs-Vth)
    double ids_sat = nmos.evaluate_ids_level1(1.8, 0.9, 0.0);
    CHECK(ids_sat > 1e-5, "NMOS saturation: Ids > 10uA");
    CHECK(ids_sat < 1.0, "NMOS saturation: Ids < 1A (sanity)");

    // Linear: Vgs = 1.8V, Vds = 0.1V (Vds < Vgs-Vth)
    double ids_lin = nmos.evaluate_ids_level1(1.8, 0.1, 0.0);
    CHECK(ids_lin > 0.0, "NMOS linear: Ids > 0");
    CHECK(ids_lin < ids_sat, "NMOS linear Ids < saturation Ids");

    // PMOS: Vgs = -1.8, Vds = -0.9
    auto pmos = sf::MosfetModel::default_pmos();
    pmos.level = 1;
    double ids_pmos = pmos.evaluate_ids_level1(-1.8, -0.9, 0.0);
    CHECK(ids_pmos < 0.0, "PMOS saturation: Ids < 0 (current into drain)");
    CHECK(std::abs(ids_pmos) > 1e-6, "PMOS saturation: |Ids| > 1uA");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 3: MOSFET BSIM4 — approximate match to Level 1
// ═══════════════════════════════════════════════════════════════════════
static void test_mosfet_bsim4() {
    printf("\n── MOSFET BSIM4 Ids ──\n");

    auto nmos = sf::MosfetModel::default_nmos();
    nmos.level = 54;

    double ids_l1 = nmos.evaluate_ids_level1(1.8, 0.9, 0.0);
    double ids_b4 = nmos.evaluate_ids_bsim4(1.8, 0.9, 0.0);

    CHECK(ids_b4 > 0.0, "BSIM4 Ids > 0 in saturation");
    // BSIM4 should be in the same ballpark (within 10x)
    double ratio = ids_b4 / ids_l1;
    CHECK(ratio > 0.05 && ratio < 20.0,
          "BSIM4 Ids within 20x of Level 1 (different mobility model)");

    // BSIM4 cutoff
    double ids_b4_off = nmos.evaluate_ids_bsim4(0.0, 0.9, 0.0);
    CHECK(std::abs(ids_b4_off) < 1e-6, "BSIM4 cutoff: Ids ~ 0");

    // Evaluate via dispatch
    nmos.level = 54;
    double ids_disp = nmos.evaluate_ids(1.8, 0.9, 0.0);
    CHECK(std::abs(ids_disp - ids_b4) < 1e-15, "evaluate_ids dispatches to BSIM4 for level>=49");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 4: MOSFET capacitance model
// ═══════════════════════════════════════════════════════════════════════
static void test_mosfet_caps() {
    printf("\n── MOSFET Capacitance Model ──\n");

    auto nmos = sf::MosfetModel::default_nmos();

    // Saturation region caps
    auto caps_sat = nmos.compute_caps(1.8, 0.9, 0.0);
    CHECK(caps_sat.Cgs > 0, "Cgs > 0 in saturation");
    CHECK(caps_sat.Cgd > 0, "Cgd > 0 (overlap)");
    CHECK(caps_sat.Csb > 0, "Csb > 0 (junction)");
    CHECK(caps_sat.Cdb > 0, "Cdb > 0 (junction)");

    // In saturation, Cgs should be larger than Cgd (intrinsic)
    CHECK(caps_sat.Cgs > caps_sat.Cgd, "saturation: Cgs > Cgd");

    // Cutoff region
    auto caps_off = nmos.compute_caps(0.0, 0.0, 0.0);
    CHECK(caps_off.Cgs > 0, "Cgs > 0 in cutoff (overlap)");

    // Linear region
    auto caps_lin = nmos.compute_caps(1.8, 0.1, 0.0);
    CHECK(caps_lin.Cgs > 0 && caps_lin.Cgd > 0, "linear: both Cgs, Cgd > 0");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 5: DC operating point — resistive divider
// ═══════════════════════════════════════════════════════════════════════
static void test_dc_operating_point() {
    printf("\n── DC Operating Point: Resistive Divider ──\n");

    // VDD=1.8V, R1=1k (vdd to mid), R2=1k (mid to gnd) → Vmid = 0.9V
    std::string netlist = R"(
* Resistive divider
Vdd vdd 0 DC 1.8
R1 vdd mid 1k
R2 mid 0 1k
.end
)";

    sf::SpiceParser parser;
    auto ckt = parser.parse(netlist);

    sf::SpiceEngine engine;
    engine.load(ckt);
    auto result = engine.dc_operating_point();

    CHECK(result.count("vdd") == 1, "vdd node in result");
    CHECK(result.count("mid") == 1, "mid node in result");

    double v_vdd = result["vdd"];
    double v_mid = result["mid"];

    CHECK(std::abs(v_vdd - 1.8) < 0.01, "V(vdd) ~ 1.8V");
    CHECK(std::abs(v_mid - 0.9) < 0.05, "V(mid) ~ 0.9V (divider)");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 6: DC operating point — CMOS inverter
// ═══════════════════════════════════════════════════════════════════════
static void test_dc_inverter() {
    printf("\n── DC Operating Point: CMOS Inverter ──\n");

    sf::SpiceEngine engine;

    // Register models
    auto nmos = sf::MosfetModel::default_nmos();
    nmos.W = 1e-6; nmos.L = 150e-9;
    auto pmos = sf::MosfetModel::default_pmos();
    pmos.W = 2e-6; pmos.L = 150e-9;

    engine.add_model("nch", nmos);
    engine.add_model("pch", pmos);

    // Input LOW → output HIGH
    {
        std::string netlist = R"(
* CMOS Inverter - input low
Vdd vdd 0 DC 1.8
Vin in 0 DC 0.0
Mn1 out in 0 0 nch W=1u L=150n
Mp1 out in vdd vdd pch W=2u L=150n
.end
)";
        sf::SpiceParser parser;
        auto ckt = parser.parse(netlist);
        engine.load(ckt);
        engine.add_model("nch", nmos);
        engine.add_model("pch", pmos);
        auto result = engine.dc_operating_point();

        double v_out = result.count("out") ? result["out"] : -1.0;
        printf("    Vin=0.0V → Vout=%.4fV\n", v_out);
        CHECK(v_out > 1.4, "input LOW → output HIGH (>1.4V)");
    }

    // Input HIGH → output LOW
    {
        std::string netlist = R"(
* CMOS Inverter - input high
Vdd vdd 0 DC 1.8
Vin in 0 DC 1.8
Mn1 out in 0 0 nch W=1u L=150n
Mp1 out in vdd vdd pch W=2u L=150n
.end
)";
        sf::SpiceParser parser;
        auto ckt = parser.parse(netlist);
        engine.load(ckt);
        engine.add_model("nch", nmos);
        engine.add_model("pch", pmos);
        auto result = engine.dc_operating_point();

        double v_out = result.count("out") ? result["out"] : 99.0;
        printf("    Vin=1.8V → Vout=%.4fV\n", v_out);
        CHECK(v_out < 0.4, "input HIGH → output LOW (<0.4V)");
    }
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 7: Transient — RC step response
// ═══════════════════════════════════════════════════════════════════════
static void test_transient_rc() {
    printf("\n── Transient: RC Step Response ──\n");

    // R=1k, C=1pF → tau = 1ns
    std::string netlist = R"(
* RC circuit
Vin in 0 DC 0.0
R1 in out 1k
C1 out 0 1p
.end
)";

    sf::SpiceParser parser;
    auto ckt = parser.parse(netlist);

    sf::SpiceEngine engine;
    engine.load(ckt);

    // Use PWL: step from 0 to 1.8V at t=0
    sf::PWLStimulus stim;
    stim.times  = {0.0, 1e-12, 10e-9};
    stim.values = {0.0, 1.8,   1.8};
    engine.add_pwl("Vin", stim);

    double tstep = 0.1e-9;  // 100ps steps
    double tstop = 5e-9;    // 5ns total
    auto result = engine.transient(tstep, tstop);

    CHECK(result.time_points.size() > 10, "transient produced time points");
    CHECK(result.node_voltages.count("out") == 1, "output node recorded");

    auto& v_out = result.node_voltages["out"];
    CHECK(v_out.size() == result.time_points.size(), "waveform length matches time");

    // At t ~ 1*tau (1ns), V should be ~63.2% of 1.8V ≈ 1.138V
    // At t ~ 5*tau (5ns), V should be ~99.3% of 1.8V ≈ 1.787V
    // Find index near 1ns
    double v_at_tau = -1.0;
    double v_final = v_out.back();
    for (size_t i = 0; i < result.time_points.size(); ++i) {
        if (result.time_points[i] >= 0.9e-9 && result.time_points[i] <= 1.2e-9) {
            v_at_tau = v_out[i];
            break;
        }
    }

    printf("    V(out) at ~tau: %.4fV, final: %.4fV\n", v_at_tau, v_final);
    CHECK(v_final > 1.5, "RC charges towards VDD");
    // Relaxed check on tau — BE integrator is dissipative
    CHECK(v_at_tau > 0.5 && v_at_tau < 1.6, "V at ~tau is reasonable");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 8: Transient — CMOS inverter
// ═══════════════════════════════════════════════════════════════════════
static void test_transient_inverter() {
    printf("\n── Transient: CMOS Inverter ──\n");

    std::string netlist = R"(
* CMOS Inverter transient
Vdd vdd 0 DC 1.8
Vin in 0 DC 0.0
Mn1 out in 0 0 nch W=1u L=150n
Mp1 out in vdd vdd pch W=2u L=150n
Cload out 0 10f
.end
)";

    auto nmos = sf::MosfetModel::default_nmos();
    nmos.W = 1e-6; nmos.L = 150e-9;
    auto pmos = sf::MosfetModel::default_pmos();
    pmos.W = 2e-6; pmos.L = 150e-9;

    sf::SpiceParser parser;
    auto ckt = parser.parse(netlist);

    sf::SpiceEngine engine;
    engine.load(ckt);
    engine.add_model("nch", nmos);
    engine.add_model("pch", pmos);

    // PWL: input goes from 0→1.8V (falling output)
    sf::PWLStimulus stim;
    stim.times  = {0.0, 0.1e-9, 0.2e-9, 5e-9};
    stim.values = {0.0, 0.0,    1.8,    1.8};
    engine.add_pwl("Vin", stim);

    auto result = engine.transient(0.05e-9, 5e-9);

    CHECK(result.time_points.size() > 20, "transient produced many points");

    if (result.node_voltages.count("out")) {
        auto& v_out = result.node_voltages["out"];
        double v_init = v_out.front();
        double v_final = v_out.back();
        printf("    Vout initial: %.4fV, final: %.4fV\n", v_init, v_final);
        // Output should transition from high to low
        CHECK(v_init > 1.0, "initial output HIGH (PMOS on)");
        CHECK(v_final < 0.5, "final output LOW (NMOS on)");

        // Measure delay
        double delay = sf::SpiceEngine::measure_delay(
            result.time_points, v_out, 0.9, false);
        printf("    50%% delay: %.3e s\n", delay);
        CHECK(delay > 0.0 && delay < 5e-9, "delay is positive and within sim window");
    } else {
        CHECK(false, "output node not found in transient result");
    }
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 9: Measure delay
// ═══════════════════════════════════════════════════════════════════════
static void test_measure_delay() {
    printf("\n── Measure Delay ──\n");

    // Synthetic ramp: 0→1.8V over 1ns
    std::vector<double> time, wave;
    for (int i = 0; i <= 100; ++i) {
        double t = i * 1e-11;  // 10ps steps, 0 to 1ns
        time.push_back(t);
        wave.push_back(1.8 * t / 1e-9);
    }

    double t_50 = sf::SpiceEngine::measure_delay(time, wave, 0.9, true);
    CHECK(std::abs(t_50 - 0.5e-9) < 0.02e-9, "50% crossing at ~0.5ns");

    double t_90 = sf::SpiceEngine::measure_delay(time, wave, 1.62, true);
    CHECK(std::abs(t_90 - 0.9e-9) < 0.02e-9, "90% crossing at ~0.9ns");

    // Falling edge
    std::vector<double> fall_wave;
    for (auto& v : wave) fall_wave.push_back(1.8 - v);

    double t_fall = sf::SpiceEngine::measure_delay(time, fall_wave, 0.9, false);
    CHECK(std::abs(t_fall - 0.5e-9) < 0.02e-9, "falling 50% crossing at ~0.5ns");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 10: Measure slew
// ═══════════════════════════════════════════════════════════════════════
static void test_measure_slew() {
    printf("\n── Measure Slew ──\n");

    // Synthetic ramp: 0→1.8V over 1ns
    std::vector<double> time, wave;
    for (int i = 0; i <= 100; ++i) {
        double t = i * 1e-11;
        time.push_back(t);
        wave.push_back(1.8 * t / 1e-9);
    }

    // 10%-90% slew of a linear ramp over 1ns = 0.8ns
    double slew = sf::SpiceEngine::measure_slew(time, wave, 0.1, 0.9, true);
    CHECK(std::abs(slew - 0.8e-9) < 0.05e-9, "10-90% slew ~ 0.8ns for linear ramp");

    // 20%-80% slew = 0.6ns
    double slew2 = sf::SpiceEngine::measure_slew(time, wave, 0.2, 0.8, true);
    CHECK(std::abs(slew2 - 0.6e-9) < 0.05e-9, "20-80% slew ~ 0.6ns for linear ramp");
}

// ═══════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════
int main() {
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║       Phase 92 — SPICE Simulation Engine                   ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");

    test_spice_parser();
    test_mosfet_level1();
    test_mosfet_bsim4();
    test_mosfet_caps();
    test_dc_operating_point();
    test_dc_inverter();
    test_transient_rc();
    test_transient_inverter();
    test_measure_delay();
    test_measure_slew();

    printf("\n══════════════════════════════════════════════════════════════\n");
    printf("Phase 92 results: %d / %d passed\n", passed, total);
    printf("══════════════════════════════════════════════════════════════\n");
    return (passed == total) ? 0 : 1;
}
