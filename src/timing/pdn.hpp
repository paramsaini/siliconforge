#pragma once
// SiliconForge — Power Distribution Network (PDN) Analyzer
// Models VDD/VSS grid, computes voltage drops, and validates EM limits.
// Reference: Nassif, "Power Grid Analysis Benchmarks", ASP-DAC 2008

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct PdnStripe {
    enum Dir { HORIZONTAL, VERTICAL } direction;
    double offset;      // position on perpendicular axis
    double width;       // stripe width
    int layer;
    double resistance_per_um;
};

struct PdnConfig {
    double vdd = 1.8;
    double total_current_ma = 100;
    std::vector<PdnStripe> stripes;
    int pad_count = 4;
    double pad_resistance = 0.01;   // Ohm per pad
    double em_limit_ma_per_um = 1.0; // electromigration current limit
};

struct PdnResult {
    double worst_drop_mv = 0;
    double worst_drop_pct = 0;
    double avg_drop_mv = 0;
    int em_violations = 0;
    double worst_current_density = 0;
    double time_ms = 0;

    struct PdnNode {
        double x, y;
        double voltage;
        double current_draw;
    };
    std::vector<PdnNode> nodes;
    std::string message;
};

class PdnAnalyzer {
public:
    PdnAnalyzer(const PhysicalDesign& pd) : pd_(pd) {}

    void set_config(const PdnConfig& cfg) { cfg_ = cfg; }
    void auto_config(double vdd = 1.8, double current_ma = 100);

    PdnResult analyze(int grid_res = 10);

private:
    const PhysicalDesign& pd_;
    PdnConfig cfg_;

    double nearest_stripe_resistance(double x, double y) const;
};

} // namespace sf
