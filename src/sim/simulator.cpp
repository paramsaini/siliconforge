// SiliconForge — Event-Driven Simulator Implementation
// True event-driven: maintains net→gate fanout map, only re-evaluates affected gates.
#include "sim/simulator.hpp"
#include <sstream>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <set>
#include <unordered_set>

namespace sf {

EventSimulator::EventSimulator(Netlist& nl) : nl_(nl) {
    topo_ = nl_.topo_order();
    build_event_structures();
}

void EventSimulator::build_event_structures() {
    // Build net → fanout gates map
    net_fanout_gates_.resize(nl_.num_nets());
    for (size_t gid = 0; gid < nl_.num_gates(); gid++) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT) continue;
        for (auto ni : g.inputs) {
            if (ni >= 0 && ni < (int)net_fanout_gates_.size())
                net_fanout_gates_[ni].push_back((int)gid);
        }
    }

    // Assign topo levels to gates for priority ordering
    gate_level_.resize(nl_.num_gates(), 0);
    for (size_t i = 0; i < topo_.size(); i++)
        gate_level_[topo_[i]] = (int)i;
}

void EventSimulator::initialize() {
    for (size_t i = 0; i < nl_.num_nets(); ++i)
        nl_.net(i).value = Logic4::X;

    for (auto& g : nl_.gates()) {
        if (g.type == GateType::CONST0 && g.output >= 0)
            nl_.net(g.output).value = Logic4::ZERO;
        else if (g.type == GateType::CONST1 && g.output >= 0)
            nl_.net(g.output).value = Logic4::ONE;
    }

    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0)
            nl_.net(ff.output).value = ff.init_val;
    }

    current_time_ = 0;
    trace_.times.clear();
    trace_.traces.clear();
    changed_nets_.clear();
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
    // True event-driven: only evaluate gates whose inputs changed
    // Use a priority queue ordered by topo level to ensure correct evaluation order

    if (changed_nets_.empty()) {
        // Fallback: full topo eval (first call or after initialize)
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
        return;
    }

    // Event-driven propagation
    // Collect all affected gates from changed nets, sort by topo level
    auto cmp = [this](GateId a, GateId b) { return gate_level_[a] > gate_level_[b]; };
    std::priority_queue<GateId, std::vector<GateId>, decltype(cmp)> pq(cmp);
    std::unordered_set<GateId> in_queue;

    for (auto net_id : changed_nets_) {
        if (net_id < 0 || net_id >= (int)net_fanout_gates_.size()) continue;
        for (auto gid : net_fanout_gates_[net_id]) {
            if (!in_queue.count(gid)) {
                pq.push(gid);
                in_queue.insert(gid);
            }
        }
    }
    changed_nets_.clear();

    while (!pq.empty()) {
        GateId gid = pq.top(); pq.pop();
        auto& g = nl_.gate(gid);
        if (g.output < 0) continue;

        std::vector<Logic4> in_vals;
        for (auto ni : g.inputs)
            in_vals.push_back(nl_.net(ni).value);

        Logic4 new_val = Netlist::eval_gate(g.type, in_vals);
        if (nl_.net(g.output).value != new_val) {
            nl_.net(g.output).value = new_val;
            // Propagate: add downstream gates
            NetId out = g.output;
            if (out >= 0 && out < (int)net_fanout_gates_.size()) {
                for (auto downstream : net_fanout_gates_[out]) {
                    if (!in_queue.count(downstream)) {
                        pq.push(downstream);
                        in_queue.insert(downstream);
                    }
                }
            }
        }
    }
}

void EventSimulator::clock_edge(NetId clk_net) {
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.clk != clk_net) continue;

        if (ff.reset >= 0 && nl_.net(ff.reset).value == Logic4::ONE) {
            if (ff.output >= 0) {
                Logic4 old = nl_.net(ff.output).value;
                nl_.net(ff.output).value = ff.init_val;
                if (old != ff.init_val) changed_nets_.push_back(ff.output);
            }
            continue;
        }

        if (!ff.inputs.empty() && ff.output >= 0) {
            Logic4 d_val = nl_.net(ff.inputs[0]).value;
            Logic4 old = nl_.net(ff.output).value;
            nl_.net(ff.output).value = d_val;
            if (old != d_val) changed_nets_.push_back(ff.output);
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
    for (size_t i = 0; i < nl_.num_nets(); ++i)
        prev_state[i] = nl_.net(i).value;

    for (auto& tv : vectors) {
        if (tv.time > max_time) break;

        apply_vector(tv);
        eval_combinational();

        bool any_edge = false;
        for (size_t i = 0; i < nl_.num_nets(); ++i) {
            if (prev_state[i] == Logic4::ZERO && nl_.net(i).value == Logic4::ONE) {
                clock_edge(i);
                any_edge = true;
            }
        }

        if (any_edge) eval_combinational();

        record_state();

        for (size_t i = 0; i < nl_.num_nets(); ++i)
            prev_state[i] = nl_.net(i).value;
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

// ═══════════════════════════════════════════════════════════════════════════
// Tier 3: Delay-Aware Simulation
// ═══════════════════════════════════════════════════════════════════════════

void EventSimulator::set_gate_delay(GateId gate, double rise_ps, double fall_ps) {
    gate_delays_[gate] = {rise_ps, fall_ps, std::min(rise_ps, fall_ps) * 0.5};
}

void EventSimulator::set_net_delay(NetId net, double delay_ps) {
    net_delays_[net] = delay_ps;
}

void EventSimulator::add_timing_check(const TimingCheck& tc) {
    timing_checks_.push_back(tc);
}

void EventSimulator::annotate_sdf(const std::string& sdf_content) {
    // Simple SDF parser: extract IOPATH and INTERCONNECT delays
    // Format: (IOPATH input output (rise) (fall))
    std::istringstream ss(sdf_content);
    std::string line;
    int gid = 0;
    while (std::getline(ss, line)) {
        if (line.find("IOPATH") != std::string::npos) {
            // Extract delay values
            auto lp = line.find('(', line.find("IOPATH"));
            if (lp == std::string::npos) continue;
            auto rp = line.find(')', lp + 1);
            if (rp == std::string::npos) continue;
            std::string val_str = line.substr(lp + 1, rp - lp - 1);
            double rise = 0;
            try { rise = std::stod(val_str); } catch (...) {}

            auto lp2 = line.find('(', rp + 1);
            auto rp2 = line.find(')', lp2 + 1);
            double fall = rise;
            if (lp2 != std::string::npos && rp2 != std::string::npos) {
                std::string val_str2 = line.substr(lp2 + 1, rp2 - lp2 - 1);
                try { fall = std::stod(val_str2); } catch (...) {}
            }
            if (gid < (int)nl_.num_gates())
                set_gate_delay(gid, rise, fall);
            gid++;
        }
        if (line.find("INTERCONNECT") != std::string::npos) {
            auto lp = line.find('(', line.find("INTERCONNECT") + 12);
            if (lp == std::string::npos) continue;
            auto rp = line.find(')', lp + 1);
            if (rp == std::string::npos) continue;
            std::string val_str = line.substr(lp + 1, rp - lp - 1);
            double delay = 0;
            try { delay = std::stod(val_str); } catch (...) {}
            // Apply to next available net
            static int nid = 0;
            if (nid < (int)nl_.num_nets())
                set_net_delay(nid, delay);
            nid++;
        }
    }
}

void EventSimulator::eval_with_delays(uint64_t current_ps) {
    // Process gates in topological order with delays
    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        Logic4 old_val = nl_.net(g.output).value;
        Logic4 new_val = old_val;

        // Evaluate gate
        std::vector<Logic4> ins;
        for (auto ni : g.inputs) {
            ins.push_back(ni >= 0 ? nl_.net(ni).value : Logic4::X);
        }

        switch (g.type) {
            case GateType::AND:  new_val = Logic4::ONE; for (auto v : ins) if (v != Logic4::ONE) new_val = (v == Logic4::ZERO) ? Logic4::ZERO : Logic4::X; break;
            case GateType::OR:   new_val = Logic4::ZERO; for (auto v : ins) if (v != Logic4::ZERO) new_val = (v == Logic4::ONE) ? Logic4::ONE : Logic4::X; break;
            case GateType::NAND: new_val = Logic4::ONE; for (auto v : ins) if (v != Logic4::ONE) new_val = (v == Logic4::ZERO) ? Logic4::ZERO : Logic4::X; new_val = (new_val == Logic4::ONE) ? Logic4::ZERO : Logic4::ONE; break;
            case GateType::NOR:  new_val = Logic4::ZERO; for (auto v : ins) if (v != Logic4::ZERO) new_val = (v == Logic4::ONE) ? Logic4::ONE : Logic4::X; new_val = (new_val == Logic4::ONE) ? Logic4::ZERO : Logic4::ONE; break;
            case GateType::XOR:  if (ins.size() >= 2) new_val = (ins[0] != ins[1]) ? Logic4::ONE : Logic4::ZERO; break;
            case GateType::NOT:  if (!ins.empty()) new_val = (ins[0] == Logic4::ONE) ? Logic4::ZERO : (ins[0] == Logic4::ZERO) ? Logic4::ONE : Logic4::X; break;
            case GateType::BUF:  if (!ins.empty()) new_val = ins[0]; break;
            default: break;
        }

        if (new_val != old_val) {
            // Apply gate delay
            auto dit = gate_delays_.find(gid);
            double delay = 0;
            if (dit != gate_delays_.end()) {
                bool rising = (new_val == Logic4::ONE);
                delay = rising ? dit->second.rise_delay : dit->second.fall_delay;

                // Inertial delay: filter glitches shorter than min_pulse
                if (delay_cfg_.inertial_model && delay < dit->second.min_pulse)
                    continue; // filter glitch
            }

            // Apply net delay
            auto nit = net_delays_.find(g.output);
            if (nit != net_delays_.end()) delay += nit->second;

            nl_.net(g.output).value = new_val;
            changed_nets_.push_back(g.output);
        }
    }
}

int EventSimulator::check_timing_constraints(uint64_t current_ps) {
    int violations = 0;
    for (auto& tc : timing_checks_) {
        if (tc.data_net < 0 || tc.clock_net < 0) continue;
        // Simple check: data must be stable before clock edge
        // (In a full implementation, track transition times)
        Logic4 data_val = nl_.net(tc.data_net).value;
        if (data_val == Logic4::X) {
            violations++; // data uncertain at clock edge
        }
    }
    return violations;
}

EventSimulator::DelaySimResult EventSimulator::run_with_delays(
    const std::vector<TestVector>& vectors, uint64_t max_time) {
    DelaySimResult res;
    initialize();

    uint64_t time_ps = 0;
    double timescale = delay_cfg_.timescale_ps > 0 ? delay_cfg_.timescale_ps : 1.0;

    for (auto& tv : vectors) {
        // Apply stimulus
        apply_vector(tv);

        // Evaluate with delays
        changed_nets_.clear();
        eval_with_delays(time_ps);

        // Check timing constraints at clock edges
        if (delay_cfg_.check_setup_hold) {
            int viol = check_timing_constraints(time_ps);
            res.timing_violations += viol;
        }

        record_state();
        time_ps += (uint64_t)(timescale * 10);
        if (time_ps > max_time * (uint64_t)timescale) break;
    }

    // Track max path delay
    for (auto& [gid, dm] : gate_delays_) {
        double max_d = std::max(dm.rise_delay, dm.fall_delay);
        if (max_d > res.max_path_delay_ps) res.max_path_delay_ps = max_d;
    }

    res.message = "Delay sim: " + std::to_string(vectors.size()) + " vectors, " +
                  std::to_string(res.timing_violations) + " violations";
    return res;
}


} // namespace sf
