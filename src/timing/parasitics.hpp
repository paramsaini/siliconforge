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
    };
    std::vector<RCSegment> segments;
};

struct ParasiticResult {
    std::vector<ParasiticNet> nets;
    double time_ms = 0;

    // SPEF-like output
    std::string to_spef() const;
};

struct TechParams {
    double wire_res_per_um = 0.1;   // Ohm/um
    double wire_cap_per_um = 0.05;  // fF/um
    double via_res = 5.0;           // Ohm per via
    double via_cap = 0.5;           // fF per via
    double coupling_cap_factor = 0.3;
};

class ParasiticExtractor {
public:
    ParasiticExtractor(const PhysicalDesign& pd, TechParams params = {})
        : pd_(pd), params_(params) {}

    ParasiticResult extract();

private:
    const PhysicalDesign& pd_;
    TechParams params_;

    ParasiticNet extract_net(int net_idx);
};

} // namespace sf
