#pragma once
// SiliconForge — Electromigration Analysis Engine
// Implements Black's equation for EM lifetime estimation, current density
// checking per metal/via layer, and EM-aware wire sizing recommendations.
// References: J.R. Black (1969), JEDEC JEP122H, Foundry EM rules

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

namespace sf {

// Per-layer EM current density limits
struct EmLayerRule {
    std::string layer_name;
    double jdc_limit_ma_per_um   = 1.0;   // DC current density limit (mA/μm)
    double jrms_limit_ma_per_um  = 5.0;   // RMS current density limit
    double jpeak_limit_ma_per_um = 10.0;   // peak current density limit
    double width_um              = 0.14;   // minimum wire width
    double thickness_um          = 0.13;
    double resistivity_ohm_um   = 0.028;  // copper
    bool   is_via               = false;
    double via_current_limit_ma = 0.5;    // per-via current limit
};

// Analysis configuration
struct EmConfig {
    double target_lifetime_years    = 10.0;
    double temperature_c            = 105.0;  // junction temperature
    double activation_energy_ev     = 0.7;    // Cu: 0.7–0.9 eV, Al: 0.5–0.7 eV
    double current_density_exponent = 2.0;    // Black's n, typically 1–3
    double supply_voltage           = 1.0;
    double clock_freq_ghz           = 1.0;
    double activity_factor          = 0.1;    // average switching activity
    std::vector<EmLayerRule> layer_rules;
    bool   check_signal_em = true;
    bool   check_power_em  = true;
    bool   check_clock_em  = true;
    double jdc_margin      = 0.8;   // use only 80 % of limit
    double jrms_margin     = 0.8;
};

// Single EM violation
struct EmViolation {
    std::string net_name;
    std::string layer_name;

    enum Type { DC_DENSITY, RMS_DENSITY, PEAK_DENSITY, VIA_CURRENT, LIFETIME };
    Type   type;

    double current_ma          = 0;
    double limit_ma            = 0;
    double ratio               = 0;   // current / limit — >1.0 ⇒ violation
    double wire_width_um       = 0;
    double estimated_mttf_years = 0;
    std::string segment_info;         // location description

    enum Severity { ERROR, WARNING, INFO };
    Severity severity = ERROR;
};

// Aggregate analysis results
struct EmResult {
    bool   pass                = true;
    int    total_nets_checked  = 0;
    int    signal_violations   = 0;
    int    power_violations    = 0;
    int    clock_violations    = 0;
    int    via_violations      = 0;
    double worst_mttf_years    = 1e10;
    std::string worst_net;
    double avg_current_density = 0;
    double max_current_density = 0;
    std::vector<EmViolation> violations;
    std::string report;               // detailed text report
    std::string summary;              // one-line summary
};

// Recommendation for fixing a violation
struct EmWireSizing {
    std::string net_name;
    std::string layer_name;
    double current_width_um      = 0;
    double recommended_width_um  = 0;
    int    current_vias          = 0;
    int    recommended_vias      = 0;
    double area_overhead_pct     = 0;
};

// ── Main analyzer ────────────────────────────────────────────────────
class EmAnalyzer {
public:
    EmResult analyze(const Netlist& nl,
                     const PhysicalDesign& pd,
                     const EmConfig& cfg) const;

    // Black's equation: MTTF for a wire/via
    double compute_mttf(double j_actual, double j_limit,
                        double temp_c, double ea_ev, double n) const;

    // Current density from absolute current, wire geometry
    double current_density(double current_ma,
                           double width_um,
                           double thickness_um) const;

    // Suggest wire-sizing / via-doubling fixes
    std::vector<EmWireSizing> suggest_fixes(const EmResult& result,
                                            const EmConfig& cfg) const;

    // Built-in foundry rule sets
    static std::vector<EmLayerRule> default_rules_sky130();
    static std::vector<EmLayerRule> default_rules_7nm();

private:
    // Current estimation helpers
    double estimate_signal_current(const Netlist& nl, int net_id,
                                   double vdd, double freq_ghz,
                                   double activity) const;

    double estimate_power_current(const PhysicalDesign& pd, int net_id,
                                  double total_power_mw) const;

    // Per-segment / per-via checking
    void check_wire_em(const std::string& net_name,
                       const WireSegment& seg,
                       double current_ma,
                       const EmLayerRule& rule,
                       const EmConfig& cfg,
                       EmResult& result) const;

    void check_via_em(const std::string& net_name,
                      const Via& via,
                      double current_ma,
                      const EmLayerRule& rule,
                      const EmConfig& cfg,
                      EmResult& result) const;

    void generate_report(EmResult& result, const EmConfig& cfg) const;
};

} // namespace sf
