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

    // ── Tier 3: Delay-Aware Simulation ──────────────────────────────────
    struct DelayModel {
        double rise_delay = 0;   // ps
        double fall_delay = 0;   // ps
        double min_pulse  = 0;   // ps, inertial delay filter
    };

    struct TimingCheck {
        NetId data_net = -1;
        NetId clock_net = -1;
        double setup_ps = 0;
        double hold_ps = 0;
    };

    struct DelaySimConfig {
        bool enable_gate_delays = true;
        bool enable_net_delays = true;
        bool inertial_model = true;        // true=inertial, false=transport
        bool check_setup_hold = true;
        double timescale_ps = 1.0;         // simulation timescale
    };

    struct DelaySimResult {
        int timing_violations = 0;
        int setup_violations = 0;
        int hold_violations = 0;
        int glitches_filtered = 0;         // inertial delay filtering
        double max_path_delay_ps = 0;
        std::string message;
    };

    void set_gate_delay(GateId gate, double rise_ps, double fall_ps);
    void set_net_delay(NetId net, double delay_ps);
    void add_timing_check(const TimingCheck& tc);
    void annotate_sdf(const std::string& sdf_content);
    void set_delay_config(const DelaySimConfig& cfg) { delay_cfg_ = cfg; }
    DelaySimResult run_with_delays(const std::vector<TestVector>& vectors, uint64_t max_time = 10000);

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

    // Tier 3: Delay state
    std::unordered_map<GateId, DelayModel> gate_delays_;
    std::unordered_map<NetId, double> net_delays_;
    std::vector<TimingCheck> timing_checks_;
    DelaySimConfig delay_cfg_;

    // Delay event queue: {time_ps, net_id, new_value}
    struct DelayEvent {
        uint64_t time_ps;
        NetId net;
        Logic4 value;
        bool operator>(const DelayEvent& o) const { return time_ps > o.time_ps; }
    };
    void eval_with_delays(uint64_t current_ps);
    int check_timing_constraints(uint64_t current_ps);
};

} // namespace sf
