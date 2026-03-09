#pragma once
// SiliconForge — Static Timing Analysis (STA)
// Graph-based path analysis using Elmore delay model.
// Supports setup/hold checks, slack computation, and critical path extraction.
// Reference: Sapatnekar, "Timing", Springer 2004

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <limits>

namespace sf {

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

    double worst_arrival() const { return std::max(arrival_rise, arrival_fall); }
    double worst_slack() const { return std::min(slack_rise, slack_fall); }
};

struct TimingPath {
    std::vector<NetId> nets;
    std::vector<GateId> gates;
    double delay = 0;
    double slack = 0;
    std::string startpoint;
    std::string endpoint;
};

struct StaResult {
    double wns = 0;         // Worst Negative Slack
    double tns = 0;         // Total Negative Slack
    double clock_period = 0;
    int num_violations = 0;
    int num_endpoints = 0;
    std::vector<TimingPath> critical_paths; // top N worst
    double time_ms = 0;
    std::string message;
};

class StaEngine {
public:
    StaEngine(const Netlist& nl, const LibertyLibrary* lib = nullptr)
        : nl_(nl), lib_(lib) {}

    // Run full STA with given clock period
    StaResult analyze(double clock_period, int num_paths = 5);

    // Get timing for a specific net
    const PinTiming& timing(NetId net) const { return pin_timing_.at(net); }

private:
    const Netlist& nl_;
    const LibertyLibrary* lib_;
    std::unordered_map<NetId, PinTiming> pin_timing_;
    std::vector<TimingArc> arcs_;
    std::vector<GateId> topo_;

    // Core STA steps
    void build_timing_graph();
    void forward_propagation(double input_arrival = 0);
    void backward_propagation(double clock_period);
    void compute_slacks();
    std::vector<TimingPath> extract_paths(int count);

    // Delay calculation
    double gate_delay(GateId gid) const;
    double wire_delay(NetId from, NetId to) const;
};

} // namespace sf
