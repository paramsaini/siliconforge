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
    double elmore_delay_ps = 0;

    struct RCSegment {
        double length;
        double resistance;
        double capacitance;
        int layer = -1; // routing layer (-1 = estimated)
    };
    std::vector<RCSegment> segments;

    struct CouplingCap {
        int aggressor_net;
        double coupling_ff;
        double overlap_um;
    };
    std::vector<CouplingCap> coupling;
    double total_coupling_ff = 0;
    int via_count = 0;
};

struct ParasiticResult {
    std::vector<ParasiticNet> nets;
    double time_ms = 0;

    // SPEF-like output
    std::string to_spef() const;
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

private:
    const PhysicalDesign& pd_;
    TechParams params_;
    std::vector<ParasiticNet> result_cache_;

    ParasiticNet extract_net(int net_idx);
    void extract_coupling();
};

} // namespace sf
