#pragma once
// SiliconForge — Static Timing Analysis (STA)
// Graph-based path analysis with slew propagation, hold checks, multi-corner,
// OCV (On-Chip Variation) and AOCV (Advanced OCV) derating.
// Reference: Sapatnekar, "Timing", Springer 2004
// Reference: AOCV: Sirichotiyakul et al., "Statistical SSTA with OCV", ICCAD 2008

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "frontend/sdc_parser.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <limits>
#include <cmath>

namespace sf {

// OCV analysis mode
enum class OcvMode { NONE, OCV, AOCV, POCV };

// AOCV depth-dependent derating table
// Uses √N statistical averaging: deeper paths accumulate less variation.
// Supports per-cell AOCV coefficients loaded from Liberty characterization.
struct AocvTable {
    double late_variation  = 0.10;  // 10% base late variation (setup pessimism)
    double early_variation = 0.10;  // 10% base early variation (hold pessimism)
    int    min_depth       = 1;     // minimum depth clamp

    // Per-cell AOCV coefficients (cell_type → {late_var, early_var})
    // Loaded from Liberty or technology file. Overrides base variation.
    std::unordered_map<std::string, std::pair<double,double>> cell_coeffs;

    // Late derate: multiplier > 1 (makes data path slower for setup worst-case)
    double late_derate(int depth, const std::string& cell_type = "") const {
        depth = std::max(depth, min_depth);
        double var = late_variation;
        auto it = cell_coeffs.find(cell_type);
        if (it != cell_coeffs.end()) var = it->second.first;
        return 1.0 + var / std::sqrt(static_cast<double>(depth));
    }
    // Early derate: multiplier < 1 (makes data path faster for hold worst-case)
    double early_derate(int depth, const std::string& cell_type = "") const {
        depth = std::max(depth, min_depth);
        double var = early_variation;
        auto it = cell_coeffs.find(cell_type);
        if (it != cell_coeffs.end()) var = it->second.second;
        return 1.0 - var / std::sqrt(static_cast<double>(depth));
    }

    // Load per-cell coefficients from a table
    void set_cell_aocv(const std::string& cell_type, double late_var, double early_var) {
        cell_coeffs[cell_type] = {late_var, early_var};
    }
};

// POCV: Parametric OCV with per-cell sigma-based statistical derating
// Reference: Synopsys POCV Application Note
// Each cell has a sigma (std deviation) of its delay variation.
// Path-level variation = RSS(σ_1, σ_2, ..., σ_N) = √(Σ σ_i²)
// Final derate = mean ± N_sigma × path_sigma
struct PocvTable {
    double n_sigma = 3.0;                   // sigma multiplier (3-sigma = 99.7% coverage)
    double default_sigma_pct = 0.05;        // default 5% sigma for cells without explicit data
    std::unordered_map<std::string, double> cell_sigma; // per-cell-type sigma as fraction of delay
    
    // Get sigma fraction for a cell type
    double get_sigma(const std::string& cell_type) const {
        auto it = cell_sigma.find(cell_type);
        return (it != cell_sigma.end()) ? it->second : default_sigma_pct;
    }
    
    // Set sigma for a specific cell type
    void set_cell_sigma(const std::string& type, double sigma_pct) {
        cell_sigma[type] = sigma_pct;
    }
};

// CPPR: Common Path Pessimism Removal
// Removes artificial pessimism from shared launch/capture clock paths
// Reference: DAC'21, Huang et al., "A Provably Good Algorithm for CPPR"
struct CpprConfig {
    bool enabled = false;
    // In full implementation, we'd track clock tree topology.
    // With CTS insertion delays only, we approximate:
    // common_path = min(launch_insertion, capture_insertion)
    // credit = common_delay × |late_derate - early_derate|
};

// Crosstalk delta-delay estimation
// Models coupling capacitance between adjacent wires
// Reference: Devgan, "Efficient Coupled Noise Estimation", ICCAD 1997
struct CrosstalkConfig {
    bool enabled = false;
    double coupling_cap_per_um = 0.00015;   // fF/um coupling cap
    double miller_factor = 1.5;             // 0=same-dir, 2=opposite-dir, 1.5=pessimistic default
    double aggressor_slew = 0.05;           // default aggressor slew (ns)
    double min_spacing_um = 0.14;           // below this, coupling is max
    double max_coupling_distance_um = 1.0;  // beyond this, ignore coupling
};

// Multi-corner derating factors with Temperature/Voltage scaling
struct CornerDerate {
    std::string name = "typical";
    double cell_derate = 1.0;   // multiply cell delay (LATE/setup data path)
    double wire_derate = 1.0;   // multiply wire delay (LATE/setup data path)
    double early_cell  = 1.0;   // for hold (best-case cell, EARLY data path)
    double early_wire  = 1.0;   // for hold (best-case wire, EARLY data path)

    // OCV: separate clock path derating (for capture clock)
    double clock_late_cell  = 1.0;
    double clock_late_wire  = 1.0;
    double clock_early_cell = 1.0;
    double clock_early_wire = 1.0;

    // Temperature / Voltage derating (Tier 2)
    // Reference corner conditions
    double ref_temperature = 25.0;    // °C (nominal)
    double ref_voltage     = 1.0;     // V  (nominal VDD)
    // Actual operating conditions
    double temperature     = 25.0;    // °C
    double voltage         = 1.0;     // V
    // Scaling coefficients (per °C and per V)
    // Positive temp_coeff → delay increases with temperature (typical for thin-oxide)
    // Negative volt_coeff → delay decreases with higher voltage
    double temp_coeff      = 0.0015;  // +0.15% per °C (typical CMOS)
    double volt_coeff      = -1.5;    // -1.5x delay per 1V increase

    // Compute T/V scaling factor relative to reference corner
    double tv_scale() const {
        double dt = temperature - ref_temperature;
        double dv = voltage - ref_voltage;
        double t_factor = 1.0 + temp_coeff * dt;
        double v_factor = 1.0 + volt_coeff * dv;
        // Clamp to reasonable range [0.5, 2.0]
        double combined = t_factor * v_factor;
        return std::min(2.0, std::max(0.5, combined));
    }
};

struct TimingArc {
    NetId from;
    NetId to;
    double delay = 0;
    double slew = 0;
    GateId gate = -1;
};

struct PinTiming {
    double arrival_rise = 0;
    double arrival_fall = 0;
    double required_rise = std::numeric_limits<double>::max();
    double required_fall = std::numeric_limits<double>::max();
    double slack_rise = std::numeric_limits<double>::max();
    double slack_fall = std::numeric_limits<double>::max();
    double slew_rise = 0;
    double slew_fall = 0;
    // Hold analysis
    double hold_arrival_rise = 0;
    double hold_arrival_fall = 0;
    double hold_required_rise = 0;
    double hold_required_fall = 0;
    double hold_slack_rise = std::numeric_limits<double>::max();
    double hold_slack_fall = std::numeric_limits<double>::max();

    double worst_arrival() const { return std::max(arrival_rise, arrival_fall); }
    double worst_slack() const { return std::min(slack_rise, slack_fall); }
    double worst_hold_slack() const { return std::min(hold_slack_rise, hold_slack_fall); }
    double best_arrival() const { return std::min(hold_arrival_rise, hold_arrival_fall); }
};

struct TimingPath {
    std::vector<NetId> nets;
    std::vector<GateId> gates;
    double delay = 0;
    double slack = 0;
    std::string startpoint;
    std::string endpoint;
    bool is_hold = false;
    int depth = 0;              // logic depth (for AOCV reporting)

    // Industrial additions
    double cppr_credit = 0;     // CPPR pessimism removal credit (ns)
    double pba_slack = 0;       // Path-Based Analysis adjusted slack
    double pba_delay = 0;       // Path-Based Analysis delay
    double crosstalk_delta = 0; // Crosstalk-induced delay change (ns)
    double path_sigma = 0;      // POCV: RSS path sigma (ns)
    bool pba_valid = false;     // Whether PBA was computed for this path
};

struct StaResult {
    double wns = 0;         // Worst Negative Slack (setup)
    double tns = 0;         // Total Negative Slack (setup)
    double hold_wns = 0;    // Worst Negative Slack (hold)
    double hold_tns = 0;    // Total Negative Slack (hold)
    double clock_period = 0;
    int num_violations = 0;
    int hold_violations = 0;
    int num_endpoints = 0;
    std::vector<TimingPath> critical_paths; // top N worst (setup + hold)
    std::string corner_name = "typical";
    OcvMode ocv_mode = OcvMode::NONE;
    double time_ms = 0;
    std::string message;

    // Industrial additions
    double cppr_total_credit = 0;   // sum of CPPR credits across critical paths
    double pba_wns = 0;             // PBA-adjusted WNS
    double pba_tns = 0;             // PBA-adjusted TNS
    int pba_violations = 0;         // PBA-adjusted violation count
    double max_crosstalk_delta = 0; // largest crosstalk delay delta
    bool cppr_enabled = false;
    bool pba_enabled = false;
    bool crosstalk_enabled = false;
};

class StaEngine {
public:
    StaEngine(const Netlist& nl, const LibertyLibrary* lib = nullptr,
              const PhysicalDesign* pd = nullptr)
        : nl_(nl), lib_(lib), pd_(pd) {}

    // Run full STA with given clock period (single corner)
    StaResult analyze(double clock_period, int num_paths = 5);

    // Run multi-corner STA — returns per-corner results + merged worst
    std::vector<StaResult> analyze_multicorner(double clock_period, int num_paths = 5);

    // Set clock uncertainty (jitter + skew)
    void set_clock_uncertainty(double setup_unc, double hold_unc) {
        setup_uncertainty_ = setup_unc; hold_uncertainty_ = hold_unc;
    }

    // Set per-DFF clock insertion delay (from CTS)
    void set_clock_insertion(GateId dff_id, double delay) {
        clock_insertion_[dff_id] = delay;
    }

    // --- OCV/AOCV configuration ---
    void set_ocv_mode(OcvMode mode) { ocv_mode_ = mode; }
    void set_aocv_table(const AocvTable& table) { aocv_table_ = table; }

    // Convenience: enable OCV with foundry-standard derating
    void enable_ocv(double late_cell = 1.15, double early_cell = 0.85) {
        ocv_mode_ = OcvMode::OCV;
        ocv_late_cell_ = late_cell;
        ocv_early_cell_ = early_cell;
    }

    // Convenience: enable AOCV with variation percentages
    void enable_aocv(double late_variation = 0.10, double early_variation = 0.10) {
        ocv_mode_ = OcvMode::AOCV;
        aocv_table_.late_variation = late_variation;
        aocv_table_.early_variation = early_variation;
    }

    // --- CPPR Configuration ---
    void enable_cppr(bool enable = true) { cppr_.enabled = enable; }
    
    // --- POCV Configuration ---
    void set_pocv_table(const PocvTable& table) { pocv_table_ = table; }
    void enable_pocv(double n_sigma = 3.0, double default_sigma_pct = 0.05) {
        ocv_mode_ = OcvMode::POCV;
        pocv_table_.n_sigma = n_sigma;
        pocv_table_.default_sigma_pct = default_sigma_pct;
    }
    
    // --- PBA Configuration ---
    void enable_pba(bool enable = true) { pba_enabled_ = enable; }
    
    // --- Crosstalk Configuration ---
    void enable_crosstalk(const CrosstalkConfig& config) { xtalk_ = config; xtalk_.enabled = true; }
    void enable_crosstalk(double coupling_cap = 0.00015, double miller = 1.5) {
        xtalk_.enabled = true;
        xtalk_.coupling_cap_per_um = coupling_cap;
        xtalk_.miller_factor = miller;
    }

    // Get timing for a specific net
    const PinTiming& timing(NetId net) const { return pin_timing_.at(net); }

    // Apply SDC constraints (false paths, multicycle paths, I/O delays)
    void set_sdc_constraints(const SdcConstraints& sdc) { sdc_ = &sdc; }

    // Run STA for a specific corner derate (used by MCMM)
    StaResult analyze_corner(double clock_period, int num_paths, const CornerDerate& d);

    // ── Path-Based Analysis (PBA) ────────────────────────────────────────
    // Recomputes delays with exact per-path slew propagation rather than
    // shared graph-based values, typically recovering ~10% pessimism.
    struct PbaResult {
        struct PathDetail {
            std::vector<int> gates;           // gate sequence
            std::vector<double> cell_delays;  // per-gate delay
            std::vector<double> wire_delays;  // per-net delay
            std::vector<double> arrivals;     // cumulative arrival
            double total_delay;
            double slack;
            bool is_critical;
        };
        std::vector<PathDetail> paths;
        double pba_wns;
        double pba_tns;
        int paths_improved;   // paths that improved vs graph-based
    };
    PbaResult run_path_based(int num_paths = 100);

    // ── SI-Aware Delay (Crosstalk) ───────────────────────────────────────
    struct SiDelay {
        int victim_net;
        std::vector<int> aggressors;
        double nominal_delay;
        double si_delta;          // additional delay from crosstalk
        double total_delay;
    };
    std::vector<SiDelay> compute_si_delays();
    void enable_si_analysis(bool enable) { si_enabled_ = enable; }

    // ── RC Corner Support ────────────────────────────────────────────────
    struct RcCorner {
        std::string name;        // e.g., "Cmax", "Cmin", "RCbest", "RCworst"
        double cap_factor;       // multiply wire cap by this
        double res_factor;       // multiply wire res by this
    };
    void add_rc_corner(const RcCorner& corner);
    void set_active_corner(const std::string& name);

    // ── External Delay Annotation ────────────────────────────────────────
    struct ExternalDelay {
        std::string port_name;
        double input_delay = 0;    // delay at input port
        double output_delay = 0;   // required time at output port
        std::string clock_name;    // relative to this clock
    };
    void set_external_delays(const std::vector<ExternalDelay>& delays);

    // ── Incremental STA ──────────────────────────────────────────────────
    struct IncrStaResult {
        int cones_updated;
        double wns;
        double tns;
        double time_ms;
    };
    IncrStaResult run_incremental(const std::vector<int>& changed_gates);

    // ── Bottleneck Analysis ──────────────────────────────────────────────
    struct Bottleneck {
        int gate_id;
        std::string gate_name;
        int num_critical_paths_through;  // how many critical paths pass through
        double total_slack_impact;       // if improved, total slack improvement
    };
    std::vector<Bottleneck> analyze_bottlenecks(int top_n = 20);

private:
    const Netlist& nl_;
    const LibertyLibrary* lib_;
    const PhysicalDesign* pd_;
    std::unordered_map<NetId, PinTiming> pin_timing_;
    std::vector<TimingArc> arcs_;
    std::vector<GateId> topo_;
    CornerDerate derate_;
    double setup_uncertainty_ = 0;
    double hold_uncertainty_ = 0;
    std::unordered_map<GateId, double> clock_insertion_; // per-DFF CTS delay

    // OCV/AOCV state
    OcvMode ocv_mode_ = OcvMode::NONE;
    AocvTable aocv_table_;
    double ocv_late_cell_ = 1.15;   // flat OCV late derate
    double ocv_early_cell_ = 0.85;  // flat OCV early derate
    std::unordered_map<GateId, int> gate_depth_;  // logic depth per gate (for AOCV)
    bool analyzing_late_ = true;    // true = late/setup analysis, false = early/hold

    // POCV state
    PocvTable pocv_table_;
    
    // CPPR state  
    CpprConfig cppr_;
    
    // PBA state
    bool pba_enabled_ = false;
    
    // Crosstalk state
    CrosstalkConfig xtalk_;

    // SDC constraints
    const SdcConstraints* sdc_ = nullptr;

    // SI-aware analysis
    bool si_enabled_ = false;

    // RC corner support
    std::vector<RcCorner> rc_corners_;
    std::string active_corner_;

    // External delay annotation
    std::vector<ExternalDelay> external_delays_;

    // Last clock period cached for incremental STA
    double last_clock_period_ = 0;

    // Core STA steps
    void build_timing_graph();
    void compute_gate_depths();     // compute logic depth for AOCV
    void forward_propagation(double input_arrival = 0);
    void hold_forward_propagation(double input_arrival = 0);
    void backward_propagation(double clock_period);
    void hold_backward_propagation();
    void compute_slacks();
    std::vector<TimingPath> extract_paths(int count, bool include_hold = true);

    // Delay calculation with slew + OCV/AOCV awareness
    double gate_delay(GateId gid, double input_slew = 0.01) const;
    double output_slew(GateId gid, double input_slew, double load_cap) const;
    double wire_delay(NetId from, NetId to) const;
    double net_load_cap(NetId nid) const;

    // OCV-aware derate factor for a gate
    double effective_cell_derate(GateId gid) const;
    double effective_wire_derate(GateId gid) const;

    // Industrial analysis passes
    void compute_cppr_credits(std::vector<TimingPath>& paths, double clock_period);
    void pba_reanalyze(std::vector<TimingPath>& paths, double clock_period);
    double compute_path_pocv_sigma(const TimingPath& path) const;
    double compute_crosstalk_delta(NetId net) const;

    // PBA helpers
    PbaResult::PathDetail trace_path(int endpoint, const std::vector<double>& arrival);
    double compute_path_delay_exact(const std::vector<int>& gates);

    // SI helpers
    double compute_crosstalk_delay(int victim_net, const std::vector<int>& aggressors);
    std::vector<int> find_aggressors(int net_idx);

    // Incremental helpers
    std::vector<int> find_affected_cone(const std::vector<int>& changed);
};

// ── Multi-Clock Domain STA ───────────────────────────────────────────
// Groups paths by clock domain and reports per-domain WNS/TNS.

struct ClockDomainInfo {
    std::string name;
    int num_flops = 0;
    double wns = 0;
    double tns = 0;
    int violations = 0;
    bool is_generated = false;
    int divide_ratio = 1; // for generated clocks (divide-by-N)
};

struct InterClockPath {
    std::string src_domain;
    std::string dst_domain;
    double slack = 0;
    bool is_async = true; // flagged as async by default
    std::string endpoint;
};

struct MultiClockStaResult {
    std::vector<ClockDomainInfo> domains;
    std::vector<InterClockPath> inter_domain_paths;
    int total_domains = 0;
    int async_crossings = 0;
    double worst_inter_domain_slack = 0;
    std::string message;
};

// Extension method on StaEngine:
// run_multi_clock performs clock domain detection from the netlist,
// groups timing paths by domain, and flags inter-domain crossings.
// Declared as a free function taking StaEngine dependencies.

MultiClockStaResult run_multi_clock_sta(
    const Netlist& nl,
    const LibertyLibrary* lib,
    const PhysicalDesign* pd,
    double clock_period,
    int num_paths = 5,
    const SdcConstraints* sdc = nullptr);

} // namespace sf
