#pragma once
// SiliconForge — IR Drop Analyzer
// Computes supply voltage drop across the power grid.
// Reference: Zhuo et al., "Static and Dynamic IR-Drop Analysis", ICCAD 2004

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct IrDropResult {
    double worst_drop_mv = 0;
    double avg_drop_mv = 0;
    double vdd = 0;
    int num_hotspots = 0;     // regions exceeding threshold
    double threshold_pct = 5; // % of VDD
    double time_ms = 0;

    struct HotSpot {
        Rect region;
        double drop_mv;
        double current_ma;
    };
    std::vector<HotSpot> hotspots;
    std::string message;

    // Grid map of IR drop (for visualization)
    std::vector<std::vector<double>> drop_map; // drop_map[y][x] in mV
    int grid_x = 0, grid_y = 0;
};

class IrDropAnalyzer {
public:
    IrDropAnalyzer(const PhysicalDesign& pd, double vdd = 1.8, double total_current_ma = 100)
        : pd_(pd), vdd_(vdd), total_current_ma_(total_current_ma) {}

    IrDropResult analyze(int grid_res = 10);

private:
    const PhysicalDesign& pd_;
    double vdd_;
    double total_current_ma_;
};

} // namespace sf
