#pragma once
// SiliconForge — Multi-Vt Cell Optimizer
// Assigns High-Vt (low leakage) or Low-Vt (fast) variants to cells
// to optimize power-performance tradeoff on non-critical paths.
// Reference: Sirichotiyakul et al., "Stand-by Power Minimization through
//            Simultaneous Threshold Voltage Selection and Circuit Sizing", DAC 1999

#include "core/netlist.hpp"
#include <string>
#include <vector>

namespace sf {

enum class VtType { SVT, LVT, HVT }; // Standard, Low, High Vt

struct MultiVtConfig {
    double lvt_speed_factor = 0.7;  // LVT is 30% faster
    double hvt_speed_factor = 1.4;  // HVT is 40% slower
    double lvt_leakage_factor = 10; // LVT leaks 10× more
    double hvt_leakage_factor = 0.1;// HVT leaks 10× less
    double timing_margin = 0.1;     // 10% timing margin to preserve
};

struct MultiVtResult {
    int total_cells = 0;
    int svt_cells = 0;
    int lvt_cells = 0;
    int hvt_cells = 0;
    double leakage_reduction_pct = 0;
    double timing_impact_pct = 0;
    double time_ms = 0;

    struct CellAssignment {
        GateId gate;
        std::string name;
        VtType vt;
    };
    std::vector<CellAssignment> assignments;
    std::string message;
};

class MultiVtOptimizer {
public:
    explicit MultiVtOptimizer(const Netlist& nl) : nl_(nl) {}

    void set_config(const MultiVtConfig& cfg) { cfg_ = cfg; }

    MultiVtResult optimize();

private:
    const Netlist& nl_;
    MultiVtConfig cfg_;

    // Compute logic depth from primary inputs
    int compute_depth(GateId gid, std::vector<int>& depth_cache) const;
    int max_depth() const;
};

} // namespace sf
