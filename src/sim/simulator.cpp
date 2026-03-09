// SiliconForge — Event-Driven Simulator Implementation
#include "sim/simulator.hpp"
#include <sstream>
#include <algorithm>
#include <iostream>
#include <cassert>

namespace sf {

EventSimulator::EventSimulator(Netlist& nl) : nl_(nl) {
    topo_ = nl_.topo_order();
}

void EventSimulator::initialize() {
    // All nets start at X
    for (size_t i = 0; i < nl_.num_nets(); ++i)
        nl_.net(i).value = Logic4::X;

    // Evaluate constant gates
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::CONST0 && g.output >= 0)
            nl_.net(g.output).value = Logic4::ZERO;
        else if (g.type == GateType::CONST1 && g.output >= 0)
            nl_.net(g.output).value = Logic4::ONE;
    }

    // Initialize DFFs to their init values
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0)
            nl_.net(ff.output).value = ff.init_val;
    }

    current_time_ = 0;
    trace_.times.clear();
    trace_.traces.clear();
    record_state();
}

void EventSimulator::set_input(NetId net, Logic4 val) {
    if (nl_.net(net).value != val) {
        nl_.net(net).value = val;
        changed_nets_.push_back(net);
    }
}

void EventSimulator::apply_vector(const TestVector& tv) {
    current_time_ = tv.time;
    for (auto& [net, val] : tv.assignments)
        set_input(net, val);
}

void EventSimulator::eval_combinational() {
    // Evaluate all combinational gates in topological order
    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        std::vector<Logic4> in_vals;
        for (auto ni : g.inputs)
            in_vals.push_back(nl_.net(ni).value);

        Logic4 new_val = Netlist::eval_gate(g.type, in_vals);
        if (nl_.net(g.output).value != new_val) {
            nl_.net(g.output).value = new_val;
            changed_nets_.push_back(g.output);
        }
    }
}

void EventSimulator::clock_edge(NetId clk_net) {
    // Rising clock edge: capture DFF D inputs into Q outputs
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.clk != clk_net) continue;

        // Check reset
        if (ff.reset >= 0 && nl_.net(ff.reset).value == Logic4::ONE) {
            if (ff.output >= 0) nl_.net(ff.output).value = ff.init_val;
            continue;
        }

        // Capture D input
        if (!ff.inputs.empty() && ff.output >= 0) {
            Logic4 d_val = nl_.net(ff.inputs[0]).value;
            nl_.net(ff.output).value = d_val;
        }
    }
}

void EventSimulator::record_state() {
    trace_.times.push_back(current_time_);
    for (size_t i = 0; i < nl_.num_nets(); ++i)
        trace_.traces[i].push_back(nl_.net(i).value);
}

void EventSimulator::run(const std::vector<TestVector>& vectors, uint64_t max_time) {
    initialize();

    std::vector<Logic4> prev_state(nl_.num_nets(), Logic4::X);
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        prev_state[i] = nl_.net(i).value;
    }

    for (auto& tv : vectors) {
        if (tv.time > max_time) break;
        
        apply_vector(tv);
        eval_combinational();
        
        // Detect rising edges
        bool any_edge = false;
        for (size_t i = 0; i < nl_.num_nets(); ++i) {
            if (prev_state[i] == Logic4::ZERO && nl_.net(i).value == Logic4::ONE) {
                clock_edge(i);
                any_edge = true;
            }
        }
        
        // If DFFs changed, re-evaluate combinational logic
        if (any_edge) {
            eval_combinational();
        }

        record_state();
        
        // Update prev_state for next time step
        for (size_t i = 0; i < nl_.num_nets(); ++i) {
            prev_state[i] = nl_.net(i).value;
        }
    }
}

// VCD (Value Change Dump) generation — IEEE 1364-2001 standard
std::string EventSimulator::generate_vcd(const std::string& module_name) const {
    std::ostringstream vcd;

    // Header
    vcd << "$date 2025-01-01 $end\n";
    vcd << "$version SiliconForge 0.1.0 $end\n";
    vcd << "$timescale 1ns $end\n";
    vcd << "$scope module " << module_name << " $end\n";

    // Declare variables — use ASCII chars starting from '!' as identifiers
    std::unordered_map<NetId, char> id_map;
    char id_char = '!';
    for (auto pi : nl_.primary_inputs()) {
        id_map[pi] = id_char;
        vcd << "$var wire 1 " << id_char << " " << nl_.net(pi).name << " $end\n";
        if (++id_char > '~') id_char = '!';
    }
    for (auto po : nl_.primary_outputs()) {
        if (id_map.count(po)) continue;
        id_map[po] = id_char;
        vcd << "$var wire 1 " << id_char << " " << nl_.net(po).name << " $end\n";
        if (++id_char > '~') id_char = '!';
    }
    // Also add FF outputs
    for (auto gid : nl_.flip_flops()) {
        NetId q = nl_.gate(gid).output;
        if (q >= 0 && !id_map.count(q)) {
            id_map[q] = id_char;
            vcd << "$var reg 1 " << id_char << " " << nl_.net(q).name << " $end\n";
            if (++id_char > '~') id_char = '!';
        }
    }

    vcd << "$upscope $end\n";
    vcd << "$enddefinitions $end\n";

    // Dump initial values
    if (!trace_.times.empty()) {
        vcd << "$dumpvars\n";
        for (auto& [nid, ch] : id_map) {
            auto it = trace_.traces.find(nid);
            if (it != trace_.traces.end() && !it->second.empty())
                vcd << logic_to_char(it->second[0]) << ch << "\n";
        }
        vcd << "$end\n";
    }

    // Dump value changes
    for (size_t t = 0; t < trace_.times.size(); ++t) {
        vcd << "#" << trace_.times[t] << "\n";
        for (auto& [nid, ch] : id_map) {
            auto it = trace_.traces.find(nid);
            if (it == trace_.traces.end()) continue;
            Logic4 val = it->second[t];
            // Only dump if changed (or first time)
            if (t == 0 || (t < it->second.size() && it->second[t] != it->second[t-1]))
                vcd << logic_to_char(val) << ch << "\n";
        }
    }

    return vcd.str();
}

} // namespace sf
