#pragma once
// SiliconForge — Signal Integrity / Crosstalk Analyzer
// Estimates coupling capacitance and crosstalk-induced delay/glitches.
// Reference: Rabaey, "Digital Integrated Circuits" Ch. 9 — Interconnect

#include "pnr/physical.hpp"
#include "timing/parasitics.hpp"
#include <string>
#include <vector>

namespace sf {

struct CrosstalkVictim {
    int net_id;
    std::string net_name;
    double coupling_cap_ff = 0;
    double noise_mv = 0;
    double delay_impact_ps = 0;
    bool is_glitch_risk = false;
};

struct SiResult {
    int nets_analyzed = 0;
    int victims = 0;
    int glitch_risks = 0;
    double worst_noise_mv = 0;
    double worst_delay_ps = 0;
    double time_ms = 0;
    std::vector<CrosstalkVictim> details;
    std::string message;
};

class SignalIntegrityAnalyzer {
public:
    SignalIntegrityAnalyzer(const PhysicalDesign& pd, double vdd = 1.8)
        : pd_(pd), vdd_(vdd) {}

    SiResult analyze();

private:
    const PhysicalDesign& pd_;
    double vdd_;

    double coupling_cap(int net_a, int net_b) const;
    double noise_voltage(double c_couple, double c_victim, double vdd) const;
};

} // namespace sf
