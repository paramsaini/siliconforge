#pragma once
// SiliconForge — Event-Driven Simulator
// IEEE 1364-compliant 4-state simulation with event queue.
// True event-driven: only re-evaluates gates whose inputs changed.

#include "core/netlist.hpp"
#include <vector>
#include <string>
#include <functional>
#include <map>
#include <queue>

namespace sf {

// Test vector: values for primary inputs at a given time
struct TestVector {
    uint64_t time;
    std::vector<std::pair<NetId, Logic4>> assignments;
};

// Simulation results
struct SimTrace {
    std::vector<uint64_t> times;
    std::unordered_map<NetId, std::vector<Logic4>> traces;
};

class EventSimulator {
public:
    explicit EventSimulator(Netlist& nl);

    // Apply stimulus
    void set_input(NetId net, Logic4 val);
    void apply_vector(const TestVector& tv);

    // Run simulation
    void initialize();
    void eval_combinational();            // Event-driven: only changed gates
    void clock_edge(NetId clk_net);
    void run(const std::vector<TestVector>& vectors, uint64_t max_time = 1000);

    // VCD output
    std::string generate_vcd(const std::string& module_name = "top") const;

    // Access results
    Logic4 get_net_value(NetId id) const { return nl_.net(id).value; }
    const SimTrace& trace() const { return trace_; }

private:
    Netlist& nl_;
    std::vector<GateId> topo_;
    SimTrace trace_;
    uint64_t current_time_ = 0;
    std::vector<NetId> changed_nets_;

    // Event-driven: net → gates that read this net as input
    std::vector<std::vector<GateId>> net_fanout_gates_;
    // Gate topo level for prioritized evaluation
    std::vector<int> gate_level_;

    void build_event_structures();
    void record_state();
    void propagate_event(NetId net);
};

} // namespace sf
