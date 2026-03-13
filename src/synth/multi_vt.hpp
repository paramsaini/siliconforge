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

    // Slack-based optimization (requires timing info)
    struct TimingInfo {
        std::vector<double> gate_arrival;  // arrival time per gate
        std::vector<double> gate_required; // required time per gate
        std::vector<double> gate_slack;    // slack = required - arrival
        double clock_period = 10.0;        // ns
    };

    // Iterative Vt assignment
    MultiVtResult optimize_iterative(int max_iterations = 5);

    // Sensitivity analysis: which cells benefit most from Vt swap
    struct VtSensitivity {
        GateId gate;
        double timing_gain;    // ns improvement if swapped to LVT
        double leakage_cost;   // leakage increase if swapped to LVT
        double efficiency;     // timing_gain / leakage_cost
    };
    std::vector<VtSensitivity> compute_sensitivity();

    // Leakage recovery pass: swap non-critical LVT cells to HVT
    int leakage_recovery(double timing_margin_ns = 0.5);

    // Set timing information for slack-based optimization
    void set_timing(const TimingInfo& ti) { timing_ = ti; has_timing_ = true; }

private:
    const Netlist& nl_;
    MultiVtConfig cfg_;
    TimingInfo timing_;
    bool has_timing_ = false;
    std::vector<VtType> current_assignment_;

    // Compute logic depth from primary inputs
    int compute_depth(GateId gid, std::vector<int>& depth_cache) const;
    int max_depth() const;

    // Compute static timing from netlist topology
    TimingInfo compute_static_timing() const;

    // Forward/backward propagation
    void forward_propagate(std::vector<double>& arrival) const;
    void backward_propagate(std::vector<double>& required, double clock_period) const;

    // Get delay/leakage multiplier for a given Vt
    double delay_factor(VtType vt) const;
    double leakage_factor(VtType vt) const;
};

} // namespace sf
