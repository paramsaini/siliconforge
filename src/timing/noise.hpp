#pragma once
// SiliconForge — Noise / PSIJ (Power Supply Induced Jitter) Analyzer
// Estimates supply noise impact on clock jitter and data path timing.
// Reference: Xu et al., "Power Supply Noise in SoCs", IEEE JSSC 2009

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <cmath>

namespace sf {

// ── Configuration ────────────────────────────────────────────────────────

struct NoiseConfig {
    double vdd = 1.8;
    double noise_margin_pct = 10.0;

    // SSN parameters
    int    num_switching_outputs = 16;       // N outputs switching simultaneously
    double pkg_inductance_nh = 0.5;          // package lead inductance
    double di_dt_ma_per_ns = 10.0;           // current slew per output
    double output_rise_time_ns = 0.2;        // output rise time

    // Frequency-domain
    double freq_start_mhz = 1.0;
    double freq_stop_mhz = 5000.0;
    int    freq_points = 64;

    // Decap model
    double on_die_decap_nf = 50.0;
    double pkg_decap_nf = 100.0;
    double board_esr_mohm = 5.0;

    // Noise-aware timing
    double clock_period_ps = 1000.0;
    double setup_margin_ps = 50.0;
};

// ── SSN result ───────────────────────────────────────────────────────────

struct SsnResult {
    double peak_ssn_mv = 0;
    double l_di_dt_mv = 0;          // L × di/dt voltage
    int    num_outputs = 0;
    double effective_inductance_nh = 0;
};

// ── Frequency spectrum point ─────────────────────────────────────────────

struct FreqPoint {
    double freq_mhz = 0;
    double impedance_mohm = 0;
    double noise_mv = 0;
};

// ── Expanded result ──────────────────────────────────────────────────────

struct NoiseResult {
    // Legacy fields
    double peak_noise_mv = 0;
    double rms_noise_mv = 0;
    double psij_ps = 0;            // jitter in ps
    double noise_margin_mv = 0;
    int noise_violations = 0;       // nets exceeding noise margin
    double time_ms = 0;

    struct NetNoise {
        int net_id;
        std::string name;
        double peak_mv;
        double impact_ps;
        bool violates;
    };
    std::vector<NetNoise> details;
    std::string message;

    // Phase-37 industrial fields
    SsnResult ssn;
    std::vector<FreqPoint> freq_spectrum;
    double noise_budget_mv = 0;
    double noise_slack_mv = 0;           // positive = passing
    double timing_derating_pct = 0;      // noise-induced timing penalty
};

// ── Power Supply Noise ───────────────────────────────────────────────
struct PsnResult {
    double peak_noise_mv = 0;
    double ldi_dt_max = 0;
    double inductance_nh = 0;
    std::vector<std::pair<double,double>> noise_waveform;
};

// ── Simultaneous Switching Output ────────────────────────────────────
struct SsoResult {
    int total_outputs = 0;
    int simultaneous_switching = 0;
    double ground_bounce_mv = 0;
    double vdd_droop_mv = 0;
    bool within_budget = false;
};

// ── Noise Margin Verification ────────────────────────────────────────
struct NoiseMarginResult {
    int total_nets = 0;
    int marginal_nets = 0;
    int failing_nets = 0;
    double worst_margin = 0;
    std::vector<std::pair<int,double>> violations;
};

// ── Jitter Analysis ──────────────────────────────────────────────────
struct JitterResult {
    double rj_rms_ps = 0;
    double dj_pp_ps = 0;
    double tj_at_ber_ps = 0;
    std::string source;
};

// ── Analyzer ─────────────────────────────────────────────────────────────

class NoiseAnalyzer {
public:
    // Legacy constructor (backward compat)
    NoiseAnalyzer(const PhysicalDesign& pd, double vdd = 1.8,
                  double noise_margin_pct = 10.0)
        : pd_(pd), cfg_{} {
        cfg_.vdd = vdd;
        cfg_.noise_margin_pct = noise_margin_pct;
    }

    // Industrial constructor
    NoiseAnalyzer(const PhysicalDesign& pd, const NoiseConfig& cfg)
        : pd_(pd), cfg_(cfg) {}

    void set_config(const NoiseConfig& cfg) { cfg_ = cfg; }
    const NoiseConfig& config() const { return cfg_; }

    NoiseResult analyze();

    // Enhanced noise analysis
    PsnResult analyze_power_supply_noise(double pkg_inductance_nh = 1.0);
    SsoResult analyze_sso(double max_bounce_mv = 100.0);
    NoiseMarginResult verify_noise_margins();
    JitterResult analyze_jitter(double clock_freq_hz = 1e9);
    NoiseResult run_enhanced();

    // SSN calculation: V = N × L × di/dt
    static SsnResult compute_ssn(int num_outputs, double inductance_nh,
                                 double di_dt_ma_per_ns);

    // Frequency-domain impedance and noise
    std::vector<FreqPoint> frequency_sweep() const;

    // Noise-aware timing derating
    static double timing_derating(double peak_noise_mv, double vdd_mv);

private:
    const PhysicalDesign& pd_;
    NoiseConfig cfg_;

    double estimate_supply_noise(double x, double y) const;
    double noise_to_jitter(double noise_mv, double vdd_mv) const;
};

} // namespace sf
