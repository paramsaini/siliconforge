#pragma once
// SiliconForge — Unified End-to-End EDA Flow API
// Orchestrates all 14 phases into a single, cohesive engine.
// Provides full JSON state export for real-time frontend visualization.

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include "frontend/verilog_parser.hpp"
#include "frontend/sva_parser.hpp"
#include "sim/simulator.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

// ── Per-Step Result Structs ──────────────────────────────────────────

struct SynthResultData {
    int nodes_before = 0, nodes_after = 0;
    int depth_before = 0, depth_after = 0;
    double reduction_pct = 0;
};

struct FormalPropResult {
    std::string name;
    std::string status; // "proved", "failed", "unknown"
    int depth = 0;
    int cex_cycle = -1;
};

struct FormalResultData {
    std::vector<FormalPropResult> properties;
    int bmc_depth = 0;
};

struct DftResultData {
    int total_faults = 0, detected = 0, patterns = 0;
    double coverage_pct = 0;
};

struct TimingResultData {
    double wns = 0, tns = 0;
    int num_violations = 0;
    int num_endpoints = 0;
    double clock_period_ns = 1.0;
    struct PathPoint { std::string gate; double arrival; double required; double slack; };
    std::vector<PathPoint> critical_path;
};

struct PowerResultData {
    double dynamic_mw = 0, leakage_mw = 0, total_mw = 0;
    double switching_mw = 0, internal_mw = 0;
    double clock_mw = 0, glitch_mw = 0;
    double clock_freq_mhz = 0;
};

struct DrcResultData {
    int total_rules = 0, violations = 0, errors = 0, warnings = 0;
    struct Viol { std::string rule; std::string message; double x; double y; };
    std::vector<Viol> details;
};

struct LvsResultData {
    bool match = false;
    int schematic_cells = 0, layout_cells = 0, matched_cells = 0;
    int unmatched_schematic = 0, unmatched_layout = 0;
    int net_mismatches = 0;
};

// ── Main Engine ──────────────────────────────────────────────────────

class SiliconForge {
public:
    SiliconForge() = default;

    // --- Frontend & Formal ---
    bool read_verilog(const std::string& filename);
    bool read_sva(const std::string& properties);
    bool run_formal_bmc(int depth = 10);

    // --- Simulation ---
    bool run_simulation();

    // --- Synthesis ---
    bool synthesize();

    // --- DFT ---
    bool run_dft();

    // --- Physical Design ---
    bool initialize_floorplan(double width, double height, double row_height = 10.0);
    bool place();
    bool route();

    // --- Verification & Signoff ---
    bool run_drc();
    bool run_lvs();
    bool run_sta();
    bool run_power();
    bool run_cdc();
    bool run_cts();
    bool run_reliability();
    bool run_lec();

    // --- ML & Tuners ---
    bool optimize_pnr_with_ai();

    // --- Full Flow ---
    bool run_all(double die_w = 200.0, double die_h = 200.0);

    // --- Export ---
    bool write_gds(const std::string& filename);
    bool generate_dashboard(const std::string& filename);
    bool write_json(const std::string& filename) const;
    bool write_full_json(const std::string& filename) const;

    // Reset
    void reset();

    // Accessors
    Netlist& netlist() { return nl_; }
    PhysicalDesign& physical_design() { return pd_; }

private:
    Netlist nl_;
    PhysicalDesign pd_;
    std::vector<SvaProperty> assertions_;
    LibertyLibrary lib_;   // Technology library for synthesis & STA

    // Flow state flags
    bool has_netlist_ = false;
    bool has_floorplan_ = false;
    bool is_placed_ = false;
    bool is_routed_ = false;
    bool is_synthesized_ = false;
    bool is_simulated_ = false;
    bool is_formal_done_ = false;
    bool is_dft_done_ = false;
    bool is_drc_done_ = false;
    bool is_lvs_done_ = false;
    bool is_sta_done_ = false;
    bool is_power_done_ = false;
    bool is_cdc_done_ = false;
    bool is_cts_done_ = false;
    bool is_reliability_done_ = false;
    bool is_lec_done_ = false;
    Netlist pre_synth_nl_; // saved for LEC
    std::unordered_map<int, double> cts_insertion_delays_; // gate_id → insertion delay

    // Per-step stored results for JSON export
    SynthResultData synth_result_;
    FormalResultData formal_result_;
    DftResultData dft_result_;
    TimingResultData timing_result_;
    PowerResultData power_result_;
    DrcResultData drc_result_;
    LvsResultData lvs_result_;
    SimTrace sim_trace_;

    // JSON helpers
    static std::string json_escape(const std::string& s);
};

} // namespace sf
