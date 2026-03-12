#pragma once
// SiliconForge — Signal Integrity / Crosstalk Analyzer
// Estimates coupling capacitance and crosstalk-induced delay/glitches.
// Reference: Rabaey, "Digital Integrated Circuits" Ch. 9 — Interconnect

#include "pnr/physical.hpp"
#include "timing/parasitics.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

// ── Configuration ────────────────────────────────────────────────────────

struct SiConfig {
    double vdd = 1.8;
    double noise_threshold_pct = 15.0;       // % of VDD
    double coupling_distance_um = 5.0;       // max wire spacing for coupling
    double miller_factor = 0.75;             // Miller capacitance multiplier (0–1)
    double glitch_filter_ps = 20.0;          // min pulse width to count as glitch
    double gate_prop_delay_ps = 30.0;        // typical gate propagation delay
    double victim_base_cap_ff = 0.5;         // base victim capacitance
    double pin_cap_ff = 0.002;               // per-pin capacitance contribution
};

// ── Aggressor-Victim pair ────────────────────────────────────────────────

struct AggressorVictim {
    int aggressor_net = -1;
    int victim_net = -1;
    double coupling_cap_ff = 0;
    double effective_cap_ff = 0;             // after Miller adjustment
    double timing_overlap = 0;              // 0–1 fraction of window overlap
    bool opposite_switching = true;         // worst case by default
    double aggressor_slew_ps = 50.0;
    double victim_slew_ps = 50.0;
    double noise_mv = 0;
    double cid_ps = 0;                      // crosstalk-induced delay
    double glitch_width_ps = 0;
    bool glitch_filtered = false;           // true if below filter threshold
};

// ── Noise Immunity Curve entry ───────────────────────────────────────────

struct NicEntry {
    std::string cell_type;
    double dc_noise_margin_mv = 0;          // static noise margin
    double ac_noise_margin_mv = 0;          // dynamic noise margin (wider)
    double pulse_width_ps = 0;              // characteristic pulse width
};

// ── Legacy struct (backward compat) ──────────────────────────────────────

struct CrosstalkVictim {
    int net_id;
    std::string net_name;
    double coupling_cap_ff = 0;
    double noise_mv = 0;
    double delay_impact_ps = 0;
    bool is_glitch_risk = false;
};

// ── Expanded result ──────────────────────────────────────────────────────

struct SiResult {
    // Legacy fields
    int nets_analyzed = 0;
    int victims = 0;
    int glitch_risks = 0;
    double worst_noise_mv = 0;
    double worst_delay_ps = 0;
    double time_ms = 0;
    std::vector<CrosstalkVictim> details;
    std::string message;

    // Phase-37 industrial fields
    double worst_cid_ps = 0;
    int worst_aggressor_net = -1;
    int filtered_glitches = 0;
    int timing_window_overlaps = 0;
    double avg_timing_overlap = 0;
    std::vector<AggressorVictim> av_details;
    std::vector<NicEntry> nic_table;
};

// ── Analyzer ─────────────────────────────────────────────────────────────

class SignalIntegrityAnalyzer {
public:
    // Legacy constructor (backward compat)
    SignalIntegrityAnalyzer(const PhysicalDesign& pd, double vdd = 1.8)
        : pd_(pd), cfg_{} { cfg_.vdd = vdd; }

    // Industrial constructor
    SignalIntegrityAnalyzer(const PhysicalDesign& pd, const SiConfig& cfg)
        : pd_(pd), cfg_(cfg) {}

    void set_config(const SiConfig& cfg) { cfg_ = cfg; }
    const SiConfig& config() const { return cfg_; }

    SiResult analyze();

    // Miller-factor effective coupling
    static double effective_coupling(double cc_ff, double miller,
                                     bool opposite_switching);

    // Timing-window overlap fraction
    static double timing_overlap(double a_start, double a_end,
                                 double v_start, double v_end);

    // Crosstalk-induced delay from slew
    static double compute_cid(double cc_eff_ff, double victim_cap_ff,
                              double aggressor_slew_ps);

    // Glitch filtering
    static bool glitch_passes_filter(double glitch_width_ps,
                                     double gate_prop_ps);

    // Build default NIC table
    static std::vector<NicEntry> default_nic_table(double vdd);

private:
    const PhysicalDesign& pd_;
    SiConfig cfg_;

    double coupling_cap(int net_a, int net_b) const;
    double noise_voltage(double c_couple, double c_victim, double vdd) const;
};

} // namespace sf
