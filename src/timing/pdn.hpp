#pragma once
// SiliconForge — Power Distribution Network (PDN) Analyzer (Industrial Grade)
// Frequency-domain impedance analysis, target impedance, resonance detection,
// decoupling capacitor modeling & optimization, bump/C4 pad modeling.
//
// References:
//   - Nassif, "Power Grid Analysis Benchmarks", ASP-DAC 2008
//   - Smith, "Decoupling Capacitor Calculations for CMOS Circuits", IEEE EMC 1994
//   - Swaminathan & Engin, "Power Integrity Modeling and Design for Semiconductors"

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <complex>
#include <cmath>

namespace sf {

// ── PDN components ───────────────────────────────────────────────────────

struct PdnStripe {
    enum Dir { HORIZONTAL, VERTICAL } direction;
    double offset;
    double width;
    int layer;
    double resistance_per_um;
};

struct DecapModel {
    std::string name;
    double capacitance_nf = 10.0;
    double esr_mohm = 50.0;       // equivalent series resistance
    double esl_ph = 100.0;        // equivalent series inductance (picohenry)
    double area_um2 = 100.0;      // silicon area
    double leakage_ua = 0.1;
    int quantity = 0;

    // Self-resonant frequency
    double srf_mhz() const {
        double l = esl_ph * 1e-12; // H
        double c = capacitance_nf * 1e-9; // F
        if (l <= 0 || c <= 0) return 0;
        return 1.0 / (2.0 * M_PI * std::sqrt(l * c)) * 1e-6; // MHz
    }
};

struct BumpPad {
    double x, y;
    double resistance_mohm = 10.0;
    double inductance_ph = 30.0;
    enum Type { POWER, GROUND, SIGNAL } type = POWER;
};

struct ViaArray {
    int count = 1;
    double per_via_resistance_mohm = 5.0;
    double per_via_inductance_ph = 10.0;
    int from_layer = 0, to_layer = 1;

    double total_resistance_mohm() const {
        return (count > 0) ? per_via_resistance_mohm / count : per_via_resistance_mohm;
    }
    double total_inductance_ph() const {
        return (count > 0) ? per_via_inductance_ph / count : per_via_inductance_ph;
    }
};

// ── Configuration ────────────────────────────────────────────────────────

struct PdnConfig {
    double vdd = 1.8;
    double total_current_ma = 100;
    double max_transient_current_ma = 300; // peak switching

    // PDN mesh
    std::vector<PdnStripe> stripes;
    int pad_count = 4;
    double pad_resistance = 0.01;
    double em_limit_ma_per_um = 1.0;

    // Bumps / C4
    std::vector<BumpPad> bumps;
    double default_bump_r_mohm = 10.0;
    double default_bump_l_ph = 30.0;

    // Via arrays between layers
    std::vector<ViaArray> via_arrays;

    // Decoupling capacitors
    std::vector<DecapModel> decaps;
    double on_die_cap_nf = 50.0;       // intrinsic on-die capacitance

    // Target impedance
    double voltage_ripple_pct = 5.0;   // max allowed ripple
    double target_impedance_mohm = 0;  // auto-computed if 0

    // Frequency sweep
    double freq_start_mhz = 1.0;
    double freq_stop_mhz = 5000.0;     // 5 GHz
    int freq_points = 100;             // log-spaced
};

// ── Impedance point ──────────────────────────────────────────────────────

struct ImpedancePoint {
    double freq_mhz;
    double magnitude_mohm;
    double phase_deg;
    std::complex<double> z;
    bool exceeds_target = false;
};

// ── Resonance ────────────────────────────────────────────────────────────

struct Resonance {
    double freq_mhz;
    double impedance_mohm;
    double q_factor;
    enum Type { SERIES, PARALLEL } type;
};

// ── Result ───────────────────────────────────────────────────────────────

struct PdnResult {
    // Basic metrics
    double worst_drop_mv = 0;
    double worst_drop_pct = 0;
    double avg_drop_mv = 0;
    int em_violations = 0;
    double worst_current_density = 0;

    // Impedance profile
    std::vector<ImpedancePoint> impedance_profile;
    double target_impedance_mohm = 0;
    int target_violations = 0;        // freq points exceeding target
    double worst_impedance_mohm = 0;
    double worst_impedance_freq_mhz = 0;

    // Resonances
    std::vector<Resonance> resonances;
    int num_resonances = 0;

    // Decap analysis
    double total_decap_nf = 0;
    double total_decap_area_um2 = 0;
    double total_decap_leakage_ua = 0;
    int decap_types_used = 0;

    // Via array
    double total_via_resistance_mohm = 0;

    // Bump/C4
    int power_bumps = 0;
    int ground_bumps = 0;
    double bump_inductance_ph = 0;

    // Power integrity signoff
    bool pi_signoff_pass = false;
    std::string pi_summary;

    // Node data (from spatial analysis)
    struct PdnNode {
        double x, y;
        double voltage;
        double current_draw;
    };
    std::vector<PdnNode> nodes;

    double time_ms = 0;
    std::string message;
};

// ── Target Impedance Detail ──────────────────────────────────────────
struct TargetImpedance {
    double z_target_ohms = 0;
    double frequency_hz = 0;
    double voltage = 0;
    double max_current = 0;
    double ripple_pct = 0;
    bool meets_target = false;
};

// ── Decoupling Optimization Result ──────────────────────────────────
struct DecapOptResult {
    int decaps_added = 0;
    double total_capacitance_nf = 0;
    std::vector<std::pair<double,double>> locations;
    double impedance_improvement_pct = 0;
};

// ── AC Impedance Profile ────────────────────────────────────────────
struct ACImpedanceProfile {
    std::vector<double> frequencies;
    std::vector<double> impedance_mag;
    std::vector<double> impedance_phase;
    double first_resonance_hz = 0;
    double anti_resonance_hz = 0;
};

// ── Package Electrical Model ────────────────────────────────────────
struct PdnPackageModel {
    double pkg_inductance_nh = 1.0;
    double pkg_resistance_mohm = 10.0;
    double pkg_capacitance_pf = 100.0;
    double board_inductance_nh = 5.0;
    double board_capacitance_uf = 10.0;
};

// ── IR-Aware Stripe Fix Config/Result ────────────────────────────────
struct IrFixConfig {
    double target_drop_pct = 5.0;  // max allowed IR drop %
    int max_iterations = 5;        // max fix iterations
    double stripe_width = 2.0;     // added stripe width (um)
    double stripe_r_per_um = 0.01; // resistance of added stripes
    int stripe_layer = 2;          // metal layer for new stripes
    double hotspot_threshold = 0.8;// fraction of worst drop to trigger fix
};

struct IrFixResult {
    int iterations = 0;
    int stripes_added = 0;
    double initial_drop_pct = 0;
    double final_drop_pct = 0;
    bool converged = false;
    std::string message;
};

// ── Analyzer ─────────────────────────────────────────────────────────────

class PdnAnalyzer {
public:
    PdnAnalyzer(const PhysicalDesign& pd) : pd_(pd) {}

    void set_config(const PdnConfig& cfg) { cfg_ = cfg; }
    const PdnConfig& config() const { return cfg_; }
    void auto_config(double vdd = 1.8, double current_ma = 100);

    // Add components
    void add_decap(const DecapModel& d) { cfg_.decaps.push_back(d); }
    void add_bump(const BumpPad& b) { cfg_.bumps.push_back(b); }
    void add_via_array(const ViaArray& v) { cfg_.via_arrays.push_back(v); }

    // Full analysis
    PdnResult analyze(int grid_res = 10);

    // Enhanced PDN analysis
    TargetImpedance compute_target_impedance_detail(double voltage, double max_current_a,
                                                     double ripple_pct = 5.0);
    DecapOptResult optimize_decoupling(double budget_area = 0.0);
    ACImpedanceProfile compute_ac_impedance(double f_start = 1e6, double f_end = 10e9, int points = 100);
    void set_package_model(const PdnPackageModel& pkg);
    PdnResult run_enhanced();

    // Frequency-domain impedance sweep
    std::vector<ImpedancePoint> impedance_sweep() const;

    // Target impedance calculation: Z_target = VDD × ripple% / I_transient
    double compute_target_impedance() const;

private:
    const PhysicalDesign& pd_;
    PdnConfig cfg_;
    PdnPackageModel pkg_model_;

    double nearest_stripe_resistance(double x, double y) const;

    // Impedance model: parallel combination of PDN mesh + decaps + die cap
    std::complex<double> pdn_impedance_at(double freq_hz) const;

    // PDN mesh impedance (R + jωL)
    std::complex<double> mesh_impedance(double freq_hz) const;

    // Single decap impedance: Z = ESR + j(ωL - 1/ωC)
    std::complex<double> decap_impedance(const DecapModel& d, double freq_hz) const;

    // Detect resonances from impedance profile
    std::vector<Resonance> find_resonances(const std::vector<ImpedancePoint>& profile) const;

    // Spatial voltage analysis
    void spatial_analysis(int grid_res, PdnResult& r);

public:
    // IR-aware stripe insertion: iterate analysis → add stripes at hotspots → re-analyze
    IrFixResult fix_ir_hotspots(IrFixConfig fix_cfg = IrFixConfig{}, int grid_res = 10);
};

} // namespace sf
