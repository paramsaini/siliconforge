#pragma once
// SiliconForge — Unified End-to-End EDA Flow API
// Orchestrates all 44 phases into a single, cohesive engine.
// Provides full JSON state export for real-time frontend visualization.

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include "frontend/verilog_parser.hpp"
#include "frontend/sva_parser.hpp"
#include "sim/simulator.hpp"
#include "timing/mcmm.hpp"
#include "timing/ssta.hpp"
#include "timing/ir_drop.hpp"
#include "timing/pdn.hpp"
#include "timing/signal_integrity.hpp"
#include "timing/thermal.hpp"
#include "timing/electromigration.hpp"
#include "timing/noise.hpp"
#include "pnr/cts.hpp"
#include "pnr/post_route_opt.hpp"
#include "pnr/chip_assembler.hpp"
#include "pnr/power_plan.hpp"
#include "pnr/metal_fill.hpp"
#include "pnr/cell_insert.hpp"
#include "pnr/oasis_writer.hpp"
#include "synth/multibit.hpp"
#include "synth/eco.hpp"
#include "synth/clock_gating.hpp"
#include "core/lef_parser.hpp"
#include "verify/erc.hpp"
#include "verify/esd.hpp"
#include "verify/latchup.hpp"
#include "verify/rdc.hpp"
#include "timing/sdf_writer.hpp"
#include "frontend/sdc_parser.hpp"
#include "ml/ml_opt.hpp"
#include "formal/advanced_formal.hpp"
#include "hls/c_parser.hpp"
#include "dft/jtag_bist.hpp"
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

struct PowerPlanResultData {
    int rings = 0, stripes = 0, rails = 0, vias = 0;
    double total_wire_length = 0;
};

struct MetalFillResultData {
    int total_fills = 0;
};

struct CellInsertResultData {
    int fillers = 0, endcaps = 0, well_taps = 0;
};

struct ErcResultData {
    int total_checks = 0, violations = 0, warnings = 0;
    bool clean = false;
};

struct EsdResultData {
    int total_pads = 0, protected_pads = 0, violations = 0, warnings = 0;
    bool clean = false;
    double worst_resistance_ohm = 0;
};

struct LatchupResultData {
    int total_cells = 0, cells_covered = 0, violations = 0, warnings = 0;
    bool clean = false;
    double worst_tap_distance = 0;
};

struct RdcResultData {
    int reset_domains = 0, crossings = 0, violations = 0, warnings = 0;
    bool clean = false;
};

struct MultiClockStaResultData {
    int total_domains = 0, async_crossings = 0;
    double worst_inter_domain_slack = 0;
};

struct DecapResultData {
    int decaps_inserted = 0;
};

struct MultiBitResultData {
    int original_ffs = 0;
    int banked_groups = 0;
    int banked_ffs = 0;
    double area_savings_pct = 0;
};

struct PhysSynthResultData {
    int paths_optimized = 0;
    int gates_resized = 0;
    double timing_improvement_pct = 0;
};

struct FullEcoResultData {
    int changes = 0;
    int gates_added = 0;
    int gates_removed = 0;
    int gates_resized = 0;
    int spare_cells_used = 0;
    int nets_rerouted = 0;
    double timing_impact_ns = 0;
};

struct ClkGateVerifyData {
    int total_icg = 0;
    int latch_based = 0;
    int issues = 0;
    bool clean = true;
};

struct UsefulSkewData {
    int paths_improved = 0;
    double slack_improvement = 0;
    double max_applied_skew = 0;
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
    bool run_power_plan();
    bool run_cell_insert();
    bool run_erc();
    bool run_metal_fill();

    // --- Advanced Analysis (Phases 20-44) ---
    bool run_mcmm();
    bool run_ssta();
    bool run_ir_drop();
    bool run_pdn();
    bool run_signal_integrity();
    bool run_thermal();
    bool run_em();
    bool run_noise();
    bool run_post_route_opt();
    bool run_chip_assemble();
    bool run_adv_formal();

    // --- Phase B: Gap Analysis Features ---
    bool run_esd();
    bool run_latchup();
    bool run_rdc();
    bool run_sdf_export(const std::string& filename = "");
    bool run_decap_insert();

    // --- Phase C: Final Gap Analysis Features ---
    bool run_multibit_banking();
    bool run_physical_synthesis();
    bool run_eco(int mode = 0);  // 0=functional, 1=metal-only, 2=spare-cell
    bool run_clock_gate_verify();
    bool write_oasis(const std::string& filename);
    bool write_lef(const std::string& filename);

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
    bool is_power_plan_done_ = false;
    bool is_cell_insert_done_ = false;
    bool is_erc_done_ = false;
    bool is_metal_fill_done_ = false;
    bool is_mcmm_done_ = false;
    bool is_ssta_done_ = false;
    bool is_ir_drop_done_ = false;
    bool is_pdn_done_ = false;
    bool is_si_done_ = false;
    bool is_thermal_done_ = false;
    bool is_em_done_ = false;
    bool is_noise_done_ = false;
    bool is_post_route_done_ = false;
    bool is_chip_assembled_ = false;
    bool is_ml_done_ = false;
    bool is_adv_formal_done_ = false;
    bool is_esd_done_ = false;
    bool is_latchup_done_ = false;
    bool is_rdc_done_ = false;
    bool is_sdf_done_ = false;
    bool is_decap_done_ = false;
    bool is_multibit_done_ = false;
    bool is_phys_synth_done_ = false;
    bool is_eco_done_ = false;
    bool is_clk_gate_verified_ = false;
    Netlist pre_synth_nl_; // saved for LEC
    std::unordered_map<int, double> cts_insertion_delays_; // gate_id → insertion delay
    SdcConstraints sdc_constraints_;

    // Per-step stored results for JSON export
    SynthResultData synth_result_;
    FormalResultData formal_result_;
    DftResultData dft_result_;
    TimingResultData timing_result_;
    PowerResultData power_result_;
    DrcResultData drc_result_;
    LvsResultData lvs_result_;
    PowerPlanResultData power_plan_result_;
    MetalFillResultData metal_fill_result_;
    CellInsertResultData cell_insert_result_;
    ErcResultData erc_result_;
    EsdResultData esd_result_;
    LatchupResultData latchup_result_;
    RdcResultData rdc_result_;
    MultiClockStaResultData multi_clock_result_;
    DecapResultData decap_result_;
    MultiBitResultData multibit_result_;
    PhysSynthResultData phys_synth_result_;
    FullEcoResultData eco_result_data_;
    ClkGateVerifyData clk_gate_verify_result_;
    UsefulSkewData useful_skew_result_;
    SimTrace sim_trace_;

    // Advanced analysis results (Phases 20-44)
    McmmResult mcmm_result_;
    SstaResult ssta_result_;
    IrDropResult ir_drop_result_;
    PdnResult pdn_result_;
    SiResult si_result_;
    ThermalResult thermal_result_;
    EmResult em_result_;
    NoiseResult noise_result_;
    CtsResult cts_result_;
    PostRouteResult post_route_result_;
    ChipResult chip_result_;
    MlOptResult ml_result_;
    AdvFormalResult adv_formal_result_;

    // JSON helpers
    static std::string json_escape(const std::string& s);
};

} // namespace sf
