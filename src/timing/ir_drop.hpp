#pragma once
// SiliconForge — IR Drop Analyzer (Industrial Grade)
// Sparse-matrix Gauss-Seidel iterative solver for resistive power grid.
// Supports configurable power pads, per-cell current maps, dynamic IR drop,
// STA timing derating from voltage drop, and signoff-quality reporting.
//
// References:
//   - Zhuo et al., "Static and Dynamic IR-Drop Analysis", ICCAD 2004
//   - Nassif, "Power Grid Analysis Benchmarks", ASP-DAC 2008
//   - Kozhaya et al., "Multigrid-Like Technique for Power Grid Analysis", TCAD 2002

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <functional>
#include <utility>
#include <cmath>

namespace sf {

// ── Power pad configuration ──────────────────────────────────────────────

struct PowerPad {
    double x, y;
    double resistance_ohm = 0.01;  // pad-to-grid resistance
    enum Type { VDD, VSS } type = VDD;
    std::string name;
};

// ── Solver configuration ─────────────────────────────────────────────────

struct IrDropConfig {
    double vdd = 1.8;
    double total_current_ma = 100.0;

    // Power pad placement
    std::vector<PowerPad> pads;
    enum PadPattern { CORNERS, PERIMETER, RING, CUSTOM } pad_pattern = CORNERS;
    int pads_per_side = 2;             // for PERIMETER/RING

    // Solver parameters
    int grid_resolution = 16;          // NxN grid (up to 128 for production)
    double sheet_resistance_mohm = 10; // mΩ/□ for power grid metal
    int max_iterations = 500;          // Gauss-Seidel max iterations
    double convergence_tol = 1e-6;     // voltage convergence (V)
    double sor_omega = 1.4;            // SOR relaxation factor (1.0 = pure GS)

    // Hotspot thresholds
    double hotspot_threshold_pct = 5.0;  // % of VDD
    double critical_threshold_pct = 10.0; // severe hotspot

    // Dynamic IR drop
    bool enable_dynamic = false;
    double switching_factor = 0.3;     // fraction of cells switching per cycle
    double clock_period_ns = 1.0;
    double decap_density_ff_per_um2 = 0.1;  // on-die decoupling cap

    // RLC PDN mesh model
    double per_layer_inductance_ph_per_um = 0.5e-12; // typical PDN inductance
    double mesh_pitch_um = 50.0;
    double pkg_inductance_nh = 0.5;
    double pkg_resistance_mohm = 5.0;
    double decap_esr_mohm = 10.0;  // ESR of decoupling caps

    // Per-cell current (optional — overrides area-proportional model)
    std::vector<double> cell_currents_ma;  // indexed by cell index

    // STA integration
    bool compute_timing_derating = true;
    double timing_derate_sensitivity = 0.12; // %delay / %voltage drop
};

// ── Per-node detailed results ────────────────────────────────────────────

struct IrDropNode {
    double x, y;
    double static_drop_mv = 0;
    double dynamic_drop_mv = 0;   // peak transient drop
    double voltage_mv = 0;        // actual supply voltage at node
    double current_ma = 0;
    double timing_derate_pct = 0; // delay derating due to drop
};

// ── Hotspot detail ───────────────────────────────────────────────────────

struct IrDropHotSpot {
    Rect region;
    double drop_mv;
    double current_ma;
    double timing_derate_pct;
    enum Severity { WARNING, CRITICAL } severity;
};

// ── Full result ──────────────────────────────────────────────────────────

struct IrDropResult {
    // Static IR drop
    double worst_drop_mv = 0;
    double avg_drop_mv = 0;
    double median_drop_mv = 0;
    double vdd = 0;

    // Dynamic IR drop
    double worst_dynamic_drop_mv = 0;
    double avg_dynamic_drop_mv = 0;
    bool dynamic_analyzed = false;

    // Hotspots
    int num_hotspots = 0;
    int num_critical = 0;
    double threshold_pct = 5;
    std::vector<IrDropHotSpot> hotspots;

    // Solver metrics
    int solver_iterations = 0;
    double solver_residual = 0;
    bool converged = false;

    // Timing derating
    double worst_timing_derate_pct = 0;
    double avg_timing_derate_pct = 0;

    // Grid map
    std::vector<std::vector<double>> drop_map;       // static drop [y][x] mV
    std::vector<std::vector<double>> dynamic_map;     // dynamic drop [y][x] mV
    std::vector<std::vector<double>> voltage_map;     // supply voltage [y][x] mV
    int grid_x = 0, grid_y = 0;

    // Detailed per-node data
    std::vector<IrDropNode> nodes;

    // Summary
    double time_ms = 0;
    std::string message;

    // Signoff pass/fail
    bool signoff_pass = false;
    std::string signoff_summary;

    // EM violations
    int em_violations = 0;
    double worst_current_density = 0;  // mA/um
};

// ── Analyzer ─────────────────────────────────────────────────────────────

// ── Dynamic IR drop (transient) ──────────────────────────────────────

struct DynamicIrResult {
    double peak_drop_mv;
    double avg_drop_mv;
    std::vector<std::pair<double,double>> waveform;  // time, voltage_drop
    std::string worst_region;
    int time_steps;

    // RLC decomposition
    double ldi_dt_peak_mv = 0;      // peak inductive (L*di/dt) noise
    double resistive_peak_mv = 0;   // peak resistive IR drop
    double resonance_freq_ghz = 0;  // LC resonance frequency
};

// ── VCD-driven vectored analysis ─────────────────────────────────────

struct VectoredIrResult {
    double peak_drop_mv;
    int peak_cycle;
    std::vector<double> per_cycle_drops;
    std::string message;
};

// ── Voltage-drop-aware STA ───────────────────────────────────────────

struct VoltageAwareTimingResult {
    double nominal_wns;
    double voltage_aware_wns;
    double timing_degradation_pct;
    std::vector<std::pair<int,double>> gate_delay_increases;  // gate, extra_delay_ns
};

// ── Hotspot clustering ───────────────────────────────────────────────

struct IrHotspot {
    double x, y;
    double radius;
    double avg_drop_mv;
    int cells_affected;
};

// ── IR drop EM hotspot detail ─────────────────────────────────────────

struct IrEmHotspot {
    int grid_x, grid_y;
    double current_density_ma_per_um;  // actual current density
    double limit_ma_per_um;            // EM limit
};

// ── Frequency-domain PDN impedance analysis ──────────────────────────────

struct FreqDomainPdnConfig {
    double freq_start_hz = 1e3;
    double freq_stop_hz = 1e9;
    int num_points = 100;       // log-spaced
    bool include_package = true;
    double package_r = 0.01;    // ohms
    double package_l = 0.5e-9;  // henries
    double decap_c = 100e-9;    // farads
};

struct FreqDomainPdnResult {
    std::vector<double> frequencies;
    std::vector<double> impedance_mag;
    std::vector<double> impedance_phase;
    double resonant_freq_hz = 0.0;
    double peak_impedance = 0.0;
};

class IrDropAnalyzer {
public:
    IrDropAnalyzer(const PhysicalDesign& pd, double vdd = 1.8, double total_current_ma = 100)
        : pd_(pd) {
        cfg_.vdd = vdd;
        cfg_.total_current_ma = total_current_ma;
    }

    void set_config(const IrDropConfig& cfg) { cfg_ = cfg; }
    const IrDropConfig& config() const { return cfg_; }

    // Set per-cell current (from power analysis)
    void set_cell_current(int cell_idx, double current_ma);
    void set_cell_currents(const std::vector<double>& currents) {
        cfg_.cell_currents_ma = currents;
    }

    // Industrial analysis
    IrDropResult analyze(int grid_res = 0);   // 0 → use config grid_resolution

    // Legacy API (backward-compatible)
    IrDropResult analyze_legacy(int grid_res = 10);

    // ── Enhanced IR drop features ────────────────────────────────────

    DynamicIrResult analyze_dynamic(double time_step_ps = 10.0, int num_steps = 100);

    VectoredIrResult analyze_vectored(const std::vector<std::vector<bool>>& stimulus,
                                       double clock_period_ns = 1.0);

    VoltageAwareTimingResult analyze_voltage_timing();

    std::vector<IrHotspot> find_hotspots(double threshold_mv = 50.0);

    std::vector<IrEmHotspot> check_em_limits(double max_current_density_ma_per_um);

    IrDropResult run_enhanced();

    // ── Frequency-domain PDN impedance analysis ──────────────────────
    FreqDomainPdnResult analyze_pdn_impedance(const FreqDomainPdnConfig& cfg = {});

    // ── Adaptive grid refinement for hotspot regions ─────────────────
    void set_adaptive_refinement(bool enable, int max_refinement_level = 3);

private:
    const PhysicalDesign& pd_;
    IrDropConfig cfg_;

    // Cached last analysis result for enhanced methods
    IrDropResult last_result_;
    bool has_result_ = false;
    int last_grid_n_ = 0;
    double last_cell_w_ = 0, last_cell_h_ = 0;
    std::vector<std::vector<double>> last_current_map_;

    // Adaptive refinement settings
    bool adaptive_refinement_ = false;
    int max_refinement_level_ = 3;

    // Power pad generation
    void generate_pads(int grid_res);
    std::vector<PowerPad> effective_pads_;

    // Current map construction
    void build_current_map(int N, double cell_w, double cell_h,
                           std::vector<std::vector<double>>& current_map);

    // Gauss-Seidel sparse solver
    struct SolverResult {
        std::vector<std::vector<double>> voltage;  // [y][x] in volts
        int iterations;
        double residual;
        bool converged;
    };
    SolverResult solve_gauss_seidel(int N, double cell_w, double cell_h,
                                     const std::vector<std::vector<double>>& current_map);

    // Dynamic IR drop estimation
    void compute_dynamic_drop(int N, double cell_w, double cell_h,
                              const std::vector<std::vector<double>>& current_map,
                              IrDropResult& result);

    // Timing derating computation
    void compute_timing_derating(IrDropResult& result);

    // Signoff evaluation
    void evaluate_signoff(IrDropResult& result);
};

} // namespace sf
