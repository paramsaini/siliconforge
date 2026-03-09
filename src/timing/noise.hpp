#pragma once
// SiliconForge — Noise / PSIJ (Power Supply Induced Jitter) Analyzer
// Estimates supply noise impact on clock jitter and data path timing.
// Reference: Xu et al., "Power Supply Noise in SoCs", IEEE JSSC 2009

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct NoiseResult {
    double peak_noise_mv = 0;
    double rms_noise_mv = 0;
    double psij_ps = 0;            // jitter in ps
    double noise_margin_mv = 0;
    int noise_violations = 0;       // nets exceeding noise margin
    double time_ms = 0;

    struct NetNoise {
        int net_id;
        std::string name;
        double peak_mv;
        double impact_ps;
        bool violates;
    };
    std::vector<NetNoise> details;
    std::string message;
};

class NoiseAnalyzer {
public:
    NoiseAnalyzer(const PhysicalDesign& pd, double vdd = 1.8,
                  double noise_margin_pct = 10.0)
        : pd_(pd), vdd_(vdd), noise_margin_pct_(noise_margin_pct) {}

    NoiseResult analyze();

private:
    const PhysicalDesign& pd_;
    double vdd_;
    double noise_margin_pct_;

    double estimate_supply_noise(double x, double y) const;
    double noise_to_jitter(double noise_mv, double vdd_mv) const;
};

} // namespace sf
