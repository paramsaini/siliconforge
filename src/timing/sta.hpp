#pragma once
// SiliconForge — Static Timing Analysis (STA)
// Graph-based path analysis with slew propagation, hold checks, multi-corner.
// Reference: Sapatnekar, "Timing", Springer 2004

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <limits>

namespace sf {

// Multi-corner derating factors
struct CornerDerate {
    std::string name = "typical";
    double cell_derate = 1.0;   // multiply cell delay
    double wire_derate = 1.0;   // multiply wire delay
    double early_cell  = 1.0;   // for hold (best-case cell)
    double early_wire  = 1.0;   // for hold (best-case wire)
};

struct TimingArc {
    NetId from;
    NetId to;
    double delay = 0;
    double slew = 0;
    GateId gate = -1;
};

struct PinTiming {
    double arrival_rise = 0;
    double arrival_fall = 0;
    double required_rise = std::numeric_limits<double>::max();
    double required_fall = std::numeric_limits<double>::max();
    double slack_rise = std::numeric_limits<double>::max();
    double slack_fall = std::numeric_limits<double>::max();
    double slew_rise = 0;
    double slew_fall = 0;
    // Hold analysis
    double hold_arrival_rise = 0;
    double hold_arrival_fall = 0;
    double hold_required_rise = 0;
    double hold_required_fall = 0;
    double hold_slack_rise = std::numeric_limits<double>::max();
    double hold_slack_fall = std::numeric_limits<double>::max();

    double worst_arrival() const { return std::max(arrival_rise, arrival_fall); }
    double worst_slack() const { return std::min(slack_rise, slack_fall); }
    double worst_hold_slack() const { return std::min(hold_slack_rise, hold_slack_fall); }
    double best_arrival() const { return std::min(hold_arrival_rise, hold_arrival_fall); }
};

struct TimingPath {
    std::vector<NetId> nets;
    std::vector<GateId> gates;
    double delay = 0;
    double slack = 0;
    std::string startpoint;
    std::string endpoint;
    bool is_hold = false;
};

struct StaResult {
    double wns = 0;         // Worst Negative Slack (setup)
    double tns = 0;         // Total Negative Slack (setup)
    double hold_wns = 0;    // Worst Negative Slack (hold)
    double hold_tns = 0;    // Total Negative Slack (hold)
    double clock_period = 0;
    int num_violations = 0;
    int hold_violations = 0;
    int num_endpoints = 0;
    std::vector<TimingPath> critical_paths; // top N worst (setup + hold)
    std::string corner_name = "typical";
    double time_ms = 0;
    std::string message;
};

class StaEngine {
public:
    StaEngine(const Netlist& nl, const LibertyLibrary* lib = nullptr,
              const PhysicalDesign* pd = nullptr)
        : nl_(nl), lib_(lib), pd_(pd) {}

    // Run full STA with given clock period (single corner)
    StaResult analyze(double clock_period, int num_paths = 5);

    // Run multi-corner STA — returns per-corner results + merged worst
    std::vector<StaResult> analyze_multicorner(double clock_period, int num_paths = 5);

    // Set clock uncertainty (jitter + skew)
    void set_clock_uncertainty(double setup_unc, double hold_unc) {
        setup_uncertainty_ = setup_unc; hold_uncertainty_ = hold_unc;
    }

    // Set per-DFF clock insertion delay (from CTS)
    void set_clock_insertion(GateId dff_id, double delay) {
        clock_insertion_[dff_id] = delay;
    }

    // Get timing for a specific net
    const PinTiming& timing(NetId net) const { return pin_timing_.at(net); }

private:
    const Netlist& nl_;
    const LibertyLibrary* lib_;
    const PhysicalDesign* pd_;
    std::unordered_map<NetId, PinTiming> pin_timing_;
    std::vector<TimingArc> arcs_;
    std::vector<GateId> topo_;
    CornerDerate derate_;
    double setup_uncertainty_ = 0;
    double hold_uncertainty_ = 0;
    std::unordered_map<GateId, double> clock_insertion_; // per-DFF CTS delay

    // Core STA steps
    void build_timing_graph();
    void forward_propagation(double input_arrival = 0);
    void hold_forward_propagation(double input_arrival = 0);
    void backward_propagation(double clock_period);
    void hold_backward_propagation();
    void compute_slacks();
    std::vector<TimingPath> extract_paths(int count, bool include_hold = true);

    // Delay calculation with slew awareness
    double gate_delay(GateId gid, double input_slew = 0.01) const;
    double output_slew(GateId gid, double input_slew, double load_cap) const;
    double wire_delay(NetId from, NetId to) const;
    double net_load_cap(NetId nid) const;

    // Internal analyze with derate
    StaResult analyze_corner(double clock_period, int num_paths, const CornerDerate& d);
};

} // namespace sf
