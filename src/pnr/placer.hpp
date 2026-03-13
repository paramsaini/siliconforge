#pragma once
// SiliconForge — Analytical Placer (SimPL-style) with Timing-Driven Mode
// Quadratic wirelength minimization with look-ahead legalization density spreading.
// Timing-driven: optional STA integration for slack-aware net weighting.
// References:
//   Kim et al., "SimPL: An Effective Placement Algorithm", IEEE TCAD 2012
//   Spindler et al., "Abacus: Fast Legalization of Standard Cell Circuits with
//                    Minimal Movement", ISPD 2008
//   Viswanathan & Chu, "FastPlace: Efficient Analytical Placement using Cell
//                       Shifting, Iterative Local Refinement, and a Hybrid
//                       Net Model", IEEE TCAD 2005
//   Ren et al., "Timing-Driven Placement for FPGAs", ACM/SIGDA 2006
//   Kong, "Timing-Driven Force-Directed Placement", IEEE TCAD 2002

#include "pnr/physical.hpp"
#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <unordered_set>

namespace sf {

// Placement constraint types for industrial fence/blockage support
// Reference: Synopsys IC Compiler create_bounds, Cadence Innovus create_fence
enum class ConstraintType { FENCE, GUIDE, BLOCKAGE };

struct PlacementConstraint {
    ConstraintType type = ConstraintType::FENCE;
    std::string name;
    Rect area;                     // bounding box
    std::vector<int> cell_ids;     // cells confined to this region (fence/guide)
    // FENCE: hard — cells MUST be inside area
    // GUIDE: soft — cells PREFER to be inside area
    // BLOCKAGE: no cells allowed inside area
};

// Cell padding for DRC-aware legalization
struct CellPadding {
    double left = 0;   // extra spacing on left side (um)
    double right = 0;  // extra spacing on right side (um)
};

// Congestion estimation bin (RUDY model)
// Reference: Spindler et al., "Fast and Accurate Routing Demand Estimation", DAC 2007
struct CongestionBin {
    double demand = 0;   // estimated routing demand (wire density)
    double supply = 0;   // available routing tracks
    double overflow() const { return std::max(0.0, demand - supply); }
    double ratio() const { return supply > 0 ? demand / supply : 0; }
};

struct PlaceResult {
    double hpwl = 0;
    double time_ms = 0;
    int iterations = 0;
    bool legal = false;
    std::string message;

    // Timing-driven results (populated when timing mode is enabled)
    double wns = 0;           // worst negative slack after placement
    double tns = 0;           // total negative slack after placement
    int critical_nets = 0;    // number of timing-critical nets (slack < 0)
    int timing_iterations = 0; // number of STA updates during placement

    // Industrial placement metrics
    int timing_swaps = 0;              // cells swapped for timing in detailed placement
    double displacement_avg = 0;       // average cell displacement from global to legal (um)
    double displacement_max = 0;       // worst-case displacement (um)
    double congestion_peak = 0;        // peak congestion ratio (demand/supply)
    double congestion_avg = 0;         // average congestion across bins
    int constraint_violations = 0;     // fence/blockage violations remaining
    int cells_padded = 0;              // cells with non-zero padding applied

    // Slack distribution histogram (10 buckets)
    // [0]: slack < -1.0ns, [1]: -1.0 to -0.5, [2]: -0.5 to -0.2, [3]: -0.2 to -0.1,
    // [4]: -0.1 to 0, [5]: 0 to 0.1, [6]: 0.1 to 0.2, [7]: 0.2 to 0.5,
    // [8]: 0.5 to 1.0, [9]: >= 1.0ns
    std::array<int, 10> slack_histogram = {};
};

class AnalyticalPlacer {
public:
    explicit AnalyticalPlacer(PhysicalDesign& pd) : pd_(pd) {}

    void set_hyperparams(double wl_weight, double density_weight) {
        wl_weight_ = wl_weight;
        density_weight_ = density_weight;
    }

    void set_net_weight(int net_id, double weight) { net_weights_[net_id] = weight; }
    void set_target_density(double d) { target_density_ = d; }
    void set_macro_fixed(int cell_id) { fixed_cells_.insert(cell_id); }

    // Macro halo enforcement (Tier 2)
    // Automatically creates blockage zones around macros with specified keepout distance
    struct MacroHalo {
        int cell_id;
        double halo_north = 2.0;  // keepout in µm
        double halo_south = 2.0;
        double halo_east  = 2.0;
        double halo_west  = 2.0;
    };
    void register_macro_halos(const std::vector<MacroHalo>& halos) { macro_halos_ = halos; }
    void enforce_macro_halos();

    // Thermal-aware placement (Tier 2)
    // Iterative thermal model: power density → temperature → cell spreading
    struct ThermalConfig {
        bool   enabled           = false;
        double ambient_temp      = 25.0;    // °C
        double thermal_resist    = 10.0;    // °C·mm²/W (substrate thermal resistance)
        double max_temp          = 105.0;   // °C (thermal limit)
        int    grid_nx           = 16;
        int    grid_ny           = 16;
        double spreading_factor  = 0.5;     // how aggressively to spread hot cells
    };
    void set_thermal_config(const ThermalConfig& cfg) { thermal_cfg_ = cfg; }
    void thermal_aware_refine();

    // --- Timing-driven placement configuration ---
    // Enable timing-driven mode: requires netlist for STA
    void enable_timing_driven(const Netlist& nl,
                              double clock_period,
                              const LibertyLibrary* lib = nullptr) {
        timing_driven_ = true;
        nl_ = &nl;
        clock_period_ = clock_period;
        lib_ = lib;
    }

    // Timing weight parameters
    void set_timing_weight(double w) { timing_weight_ = w; }
    void set_criticality_exponent(double e) { criticality_exp_ = e; }
    void set_timing_update_interval(int interval) { timing_update_interval_ = interval; }

    // --- Industrial placement constraints ---
    void add_constraint(const PlacementConstraint& c) { constraints_.push_back(c); }
    void add_fence(const std::string& name, const Rect& area, const std::vector<int>& cells) {
        constraints_.push_back({ConstraintType::FENCE, name, area, cells});
    }
    void add_blockage(const std::string& name, const Rect& area) {
        constraints_.push_back({ConstraintType::BLOCKAGE, name, area, {}});
    }

    // Cell padding for DRC-aware legalization
    void set_cell_padding(int cell_id, double left, double right) {
        cell_padding_[cell_id] = {left, right};
    }

    // Congestion-driven placement
    void enable_congestion_driven(bool en = true) { congestion_driven_ = en; }
    void set_congestion_weight(double w) { congestion_weight_ = w; }

    // Set blockages from floorplanner constraints
    void set_blockages(const std::vector<Rect>& blockages) {
        for (auto& b : blockages)
            constraints_.push_back({ConstraintType::BLOCKAGE, "blockage", b, {}});
    }

    // Post-placement congestion-driven refinement using ML predictor
    void congestion_driven_refine(const Netlist& nl);

    // Incremental placement (ECO mode): only move specified cells
    void enable_incremental_mode(const std::vector<int>& cells_to_move) {
        incremental_mode_ = true;
        incremental_cells_.clear();
        for (int c : cells_to_move) incremental_cells_.insert(c);
    }
    void set_max_displacement(double d) { max_displacement_ = d; }

    // FFT-based density smoothing (ePlace-style)
    struct DensityConfig {
        int bin_count_x = 64;
        int bin_count_y = 64;
        double target_density = 0.7;   // target utilization
        double smooth_penalty = 1.0;    // density penalty weight
        int fft_iterations = 5;
    };
    void set_density_config(const DensityConfig& cfg) { density_cfg_ = cfg; }

    // IO pad placement
    struct IoPad {
        std::string name;
        int cell_idx;
        enum class Side { NORTH, SOUTH, EAST, WEST } side;
        double position;  // position along the side (0.0 to 1.0)
    };
    void set_io_pads(const std::vector<IoPad>& pads) { io_pads_ = pads; }
    void place_io_pads();

    // Incremental placement (after ECO)
    PlaceResult incremental_place(const std::vector<int>& modified_cells);

    // Multi-row cell support
    struct MultiRowCell {
        int cell_idx;
        int row_span;    // how many rows this cell occupies
        double height;   // actual height
    };
    void register_multi_row(const std::vector<MultiRowCell>& cells) { multi_row_ = cells; }

    // Wirelength gradient computation
    struct WLGradient {
        std::vector<double> grad_x;
        std::vector<double> grad_y;
        double total_wl;
    };
    WLGradient compute_wl_gradient();

    // Density gradient (smoothed)
    struct DensityGradient {
        std::vector<double> grad_x;
        std::vector<double> grad_y;
        double overflow;
    };
    DensityGradient compute_density_gradient();

    // Enhanced placement with density smoothing
    PlaceResult place_eplace();  // ePlace-style global placement

    PlaceResult place();

    // ── Tier 3: Multi-Vdd Power Domain Placement ────────────────────────
    struct VoltageDomain {
        std::string name;
        double voltage = 1.0;
        Rect region;                        // placement fence for this domain
        std::vector<int> cell_ids;          // cells assigned to this domain
        std::string power_net = "VDD";
        std::string ground_net = "VSS";
    };

    struct LevelShifterCell {
        int from_domain = -1;
        int to_domain = -1;
        int cell_id = -1;                   // placed cell id in pd_
        NetId net_id = -1;                  // net crossing domain boundary
        std::string cell_type = "LS_HL";    // LS_HL or LS_LH
    };

    struct IsolationCell {
        int domain_id = -1;
        int cell_id = -1;
        NetId net_id = -1;
        std::string clamp_type = "CLAMP_0"; // CLAMP_0, CLAMP_1, LATCH
        std::string control_signal;
    };

    struct PowerSwitchCell {
        int domain_id = -1;
        int cell_id = -1;
        Point position;
        double switch_resistance = 0.1;     // ohm
        std::string enable_signal;
    };

    struct DomainPlaceResult {
        int domains_placed = 0;
        int level_shifters_inserted = 0;
        int isolation_cells_inserted = 0;
        int power_switches_inserted = 0;
        int cross_domain_nets = 0;
        std::string message;
    };

    void add_voltage_domain(const VoltageDomain& vd) { voltage_domains_.push_back(vd); }
    void assign_cell_to_domain(int cell_id, int domain_id);
    DomainPlaceResult place_with_domains();
    int insert_level_shifters();
    int insert_isolation_cells(const std::string& control_signal, const std::string& clamp = "CLAMP_0");
    int insert_power_switches(double switch_pitch = 50.0, const std::string& enable = "pwr_en");

private:
    PhysicalDesign& pd_;
    double wl_weight_ = 1.0;
    double density_weight_ = 1.0;
    double target_density_ = 0.85;
    std::unordered_map<int, double> net_weights_;
    std::unordered_set<int> fixed_cells_;

    // Timing-driven state
    bool timing_driven_ = false;
    const Netlist* nl_ = nullptr;
    const LibertyLibrary* lib_ = nullptr;
    double clock_period_ = 0;
    double timing_weight_ = 5.0;       // multiplier for critical net weights
    double criticality_exp_ = 3.0;     // exponent for criticality function
    int timing_update_interval_ = 3;   // run STA every N outer iterations
    std::unordered_map<int, double> timing_net_weights_; // STA-derived weights

    // Industrial state
    std::vector<PlacementConstraint> constraints_;
    std::unordered_map<int, CellPadding> cell_padding_;
    bool congestion_driven_ = false;
    double congestion_weight_ = 2.0;
    bool incremental_mode_ = false;
    std::unordered_set<int> incremental_cells_;
    double max_displacement_ = 50.0; // max allowed displacement in ECO mode (um)

    // Congestion estimation grid
    int cong_nx_ = 0, cong_ny_ = 0;
    std::vector<std::vector<CongestionBin>> congestion_grid_;

    // Timing-driven: run STA and update net weights by criticality
    void update_timing_weights(PlaceResult& result);

    // Industrial passes
    void timing_aware_detail_placement(PlaceResult& result);
    void enforce_constraints(PlaceResult& result);
    void apply_cell_padding();
    void estimate_congestion(PlaceResult& result);
    void compute_slack_histogram(PlaceResult& result);
    void compute_displacement(PlaceResult& result,
                              const std::vector<double>& orig_x,
                              const std::vector<double>& orig_y);

    // Congestion: RUDY (Rectangular Uniform wire DensitY) estimation
    void build_congestion_grid(int nx, int ny);
    void update_congestion_demand();
    void apply_congestion_penalty();

    // Conjugate gradient state
    std::vector<double> x_, y_;             // current positions
    std::vector<double> anchor_x_, anchor_y_; // spreading anchors
    std::vector<double> anchor_w_;          // anchor weights (increase each iter)

    // Step 1: Conjugate gradient quadratic solver
    void solve_quadratic_cg();

    // Step 2: Density-aware bin spreading (look-ahead legalization)
    void density_spread();

    // Step 3: Abacus legalization — optimal row assignment minimizing displacement
    void legalize_abacus();

    // Helper: compute HPWL
    double compute_hpwl() const;

    // Helper: build B2B (bound-to-bound) net model weights
    struct B2BEdge { int i, j; double wx, wy; };
    std::vector<B2BEdge> build_b2b_model() const;

    // Density bin grid
    struct DensityBin {
        double x0, y0, x1, y1;
        double supply;    // available area for cells
        double demand;    // total cell area placed here
        double target_x, target_y; // center after spreading
    };
    std::vector<std::vector<DensityBin>> density_grid_;
    int bin_nx_ = 0, bin_ny_ = 0;
    void build_density_grid(int nx, int ny);
    void compute_density();
    void spread_bins_x();
    void spread_bins_y();

    // ePlace / density smoothing state
    DensityConfig density_cfg_;
    std::vector<IoPad> io_pads_;
    std::vector<MultiRowCell> multi_row_;
    std::vector<MacroHalo> macro_halos_;
    ThermalConfig thermal_cfg_;

    // Thermal grid (Tier 2)
    struct ThermalBin {
        double power_density = 0;  // W/mm²
        double temperature = 25.0; // °C
    };
    std::vector<std::vector<ThermalBin>> thermal_grid_;

    // FFT helpers (simplified 2D DCT for density smoothing)
    std::vector<std::vector<double>> compute_density_map();
    void smooth_density(std::vector<std::vector<double>>& density);

    // Nesterov's method for global placement
    struct NesterovState {
        std::vector<double> x, y;          // current positions
        std::vector<double> x_prev, y_prev; // previous positions
        double step_size = 0.01;
        int iteration = 0;
    };
    void nesterov_step(NesterovState& state, const WLGradient& wl_grad,
                       const DensityGradient& den_grad);

    // Look-ahead legalization
    void legalize_lookahead(std::vector<double>& x, std::vector<double>& y);

    // Multi-row legalization
    void legalize_multi_row(std::vector<double>& x, std::vector<double>& y);

    // Tier 3: Multi-Vdd state
    std::vector<VoltageDomain> voltage_domains_;
    std::vector<LevelShifterCell> level_shifters_;
    std::vector<IsolationCell> isolation_cells_;
    std::vector<PowerSwitchCell> power_switches_;
    std::unordered_map<int, int> cell_domain_map_;  // cell_id → domain index
    int find_domain_for_cell(int cell_id) const;
    bool nets_cross_domains(NetId net) const;
};

} // namespace sf
