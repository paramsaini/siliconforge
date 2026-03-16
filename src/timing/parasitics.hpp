#pragma once
// SiliconForge — Parasitic Extraction (RC Network)
// Extracts wire capacitance and resistance from physical layout.
// Uses simple geometric model for Manhattan routing.

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct ParasiticNet {
    int net_id;
    double total_cap_ff = 0;   // total capacitance in fF
    double total_res_ohm = 0;  // total resistance in Ohm
    double total_ind_nh = 0;   // total inductance in nH
    double elmore_delay_ps = 0;

    struct RCSegment {
        double length;
        double resistance;
        double capacitance;
        double inductance = 0;  // nH
        int layer = -1; // routing layer (-1 = estimated)
    };
    std::vector<RCSegment> segments;

    struct CouplingCap {
        int aggressor_net;
        double coupling_ff;
        double overlap_um;
    };
    std::vector<CouplingCap> coupling;

    struct MutualCoupling {
        int other_net;
        double mutual_ind_nh = 0;  // mutual inductance in nH
    };
    std::vector<MutualCoupling> mutual_inductance;

    double total_coupling_ff = 0;
    int via_count = 0;
};

struct ParasiticResult {
    std::vector<ParasiticNet> nets;
    double time_ms = 0;

    // SPEF-like output
    std::string to_spef() const;

    // Scale parasitics by temperature and voltage
    // R_scaled = R * (1 + TCR * (T - T_ref)), C_scaled mildly with voltage
    void scale_parasitics(double temp_factor, double voltage_factor,
                          double tcr = 0.003, double vcc = 0.001);
};

struct MetalLayerParams {
    double res_per_um = 0.1;   // Ohm/um
    double cap_per_um = 0.05;  // fF/um
    double min_spacing_um = 0.14;
    double width_um = 0.1;
};

struct TechParams {
    double wire_res_per_um = 0.1;   // Ohm/um
    double wire_cap_per_um = 0.05;  // fF/um
    double wire_ind_per_um = 0.5;   // nH/um (self-inductance estimate)
    double via_res = 5.0;           // Ohm per via
    double via_cap = 0.5;           // fF per via
    double coupling_cap_factor = 0.3;
    // Per-layer params (index 0 = M1, etc.)
    std::vector<MetalLayerParams> layers;
    double coupling_coeff = 0.12; // fF/um for adjacent parallel wires
    double fringe_cap_per_um = 0.02; // fF/um fringe capacitance
};

class ParasiticExtractor {
public:
    ParasiticExtractor(const PhysicalDesign& pd, TechParams params = {})
        : pd_(pd), params_(params) {}

    ParasiticResult extract();

    // Phase 98: Enhanced coupling capacitance extraction
    // Uses spatial grid for O(W) wire-pair queries instead of O(W^2) brute force.
    // Supports same-layer and inter-layer coupling.
    struct CouplingConfig {
        double max_coupling_distance_um = 5.0;  // ignore pairs beyond this
        double inter_layer_coupling_factor = 0.6; // Cc between adjacent metal layers
        int spatial_grid_bins = 64;               // grid resolution for spatial indexing
        bool enable_miller_effect = false;         // double coupling for switching aggressors
        double miller_factor = 2.0;
    };
    void set_coupling_config(const CouplingConfig& cfg) { coupling_cfg_ = cfg; }

    // AWE (Asymptotic Waveform Evaluation) delay model
    // Superior to Elmore for deep sub-micron nets with coupling.
    // Computes dominant poles via moment matching on the RC+Cc network.
    struct AweResult {
        double delay_ps = 0;
        double slew_ps = 0;           // 20-80% transition time
        std::vector<double> poles;     // dominant poles
        std::vector<double> residues;  // residues for each pole
    };
    AweResult compute_awe(int net_idx, int order = 2) const;

private:
    const PhysicalDesign& pd_;
    TechParams params_;
    CouplingConfig coupling_cfg_;
    std::vector<ParasiticNet> result_cache_;

    ParasiticNet extract_net(int net_idx);
    void extract_coupling();

    // Phase 98: Spatial-grid-accelerated coupling extraction
    struct WireEntry {
        int net_id;
        int layer;
        double x0, y0, x1, y1;
        double width;
        bool horizontal;  // true if horizontal wire
    };
    struct SpatialBin {
        std::vector<int> wire_indices;  // indices into wire_entries_
    };
    std::vector<WireEntry> wire_entries_;
    std::vector<std::vector<SpatialBin>> spatial_grid_;  // [layer][bin_idx]
    int grid_nx_ = 0, grid_ny_ = 0;
    double grid_x0_ = 0, grid_y0_ = 0, grid_dx_ = 1, grid_dy_ = 1;

    void build_spatial_index();
    void extract_coupling_spatial();
    double compute_coupling_cap(const WireEntry& a, const WireEntry& b) const;
    std::vector<int> query_nearby(int layer, double x0, double y0,
                                   double x1, double y1) const;
};

} // namespace sf
