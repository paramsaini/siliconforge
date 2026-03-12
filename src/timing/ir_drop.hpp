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
};

// ── Analyzer ─────────────────────────────────────────────────────────────

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

private:
    const PhysicalDesign& pd_;
    IrDropConfig cfg_;

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
