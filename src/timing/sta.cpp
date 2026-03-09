// SiliconForge — Static Timing Analysis Implementation
#include "timing/sta.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <cmath>
#include <functional>

namespace sf {

double StaEngine::gate_delay(GateId gid) const {
    auto& g = nl_.gate(gid);
    // If Liberty library available, use cell delay
    if (lib_) {
        for (auto& cell : lib_->cells) {
            if (cell.name == g.name || cell.name == std::string(gate_type_str(g.type)) + "_X1") {
                // Use average of rise/fall from cell timing
                for (auto& t : cell.timings)
                    return (t.cell_rise + t.cell_fall) / 2.0;
                // Default based on area
                return cell.area * 0.01;
            }
        }
    }
    // Default delay model based on gate type
    switch (g.type) {
        case GateType::BUF:    return 0.02;
        case GateType::NOT:    return 0.02;
        case GateType::AND:    return 0.05 * g.inputs.size();
        case GateType::OR:     return 0.05 * g.inputs.size();
        case GateType::NAND:   return 0.04 * g.inputs.size();
        case GateType::NOR:    return 0.04 * g.inputs.size();
        case GateType::XOR:    return 0.08;
        case GateType::XNOR:   return 0.08;
        case GateType::MUX:    return 0.06;
        case GateType::DFF:    return 0.10; // CK-to-Q
        default:               return 0.05;
    }
}

double StaEngine::wire_delay(NetId from, NetId to) const {
    // Simple RC delay model: delay ∝ fanout × wire_capacitance
    auto& net = nl_.net(from);
    double fanout = (double)net.fanout.size();
    return 0.001 * fanout; // 1ps per fanout
}

void StaEngine::build_timing_graph() {
    arcs_.clear();
    topo_ = nl_.topo_order();

    // Build timing arcs through combinational gates
    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double gd = gate_delay(gid);
        for (auto ni : g.inputs) {
            double wd = wire_delay(ni, g.output);
            arcs_.push_back({ni, g.output, gd + wd, 0.01, gid});
        }
    }

    // DFF CK-to-Q arcs
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0) {
            arcs_.push_back({ff.clk, ff.output, gate_delay(gid), 0.01, gid});
        }
    }
}

void StaEngine::forward_propagation(double input_arrival) {
    pin_timing_.clear();

    // Initialize all PIs and FF outputs with input arrival time
    for (auto pi : nl_.primary_inputs()) {
        pin_timing_[pi] = {};
        pin_timing_[pi].arrival_rise = input_arrival;
        pin_timing_[pi].arrival_fall = input_arrival;
    }
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0) {
            double ckq = gate_delay(gid);
            pin_timing_[ff.output].arrival_rise = ckq;
            pin_timing_[ff.output].arrival_fall = ckq;
        }
    }

    // Initialize all nets
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        if (!pin_timing_.count(i))
            pin_timing_[i] = {};
    }

    // Propagate in topological order
    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double max_arr = 0;
        for (auto ni : g.inputs) {
            double arr = pin_timing_[ni].worst_arrival();
            max_arr = std::max(max_arr, arr);
        }

        double gd = gate_delay(gid);
        double wd = wire_delay(g.inputs.empty() ? -1 : g.inputs[0], g.output);
        double out_arr = max_arr + gd + wd;

        auto& pt = pin_timing_[g.output];
        pt.arrival_rise = std::max(pt.arrival_rise, out_arr);
        pt.arrival_fall = std::max(pt.arrival_fall, out_arr);
    }
}

void StaEngine::backward_propagation(double clock_period) {
    // Set required times at endpoints (POs and FF D-inputs)
    for (auto po : nl_.primary_outputs()) {
        pin_timing_[po].required_rise = clock_period;
        pin_timing_[po].required_fall = clock_period;
    }

    // FF D-input required time = clock_period - setup_time
    double setup_time = 0.05;
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) {
            NetId d = ff.inputs[0];
            pin_timing_[d].required_rise = std::min(pin_timing_[d].required_rise, clock_period - setup_time);
            pin_timing_[d].required_fall = std::min(pin_timing_[d].required_fall, clock_period - setup_time);
        }
    }

    // Backwards propagation in reverse topo order
    for (int i = (int)topo_.size() - 1; i >= 0; --i) {
        auto& g = nl_.gate(topo_[i]);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double out_req = std::min(pin_timing_[g.output].required_rise,
                                   pin_timing_[g.output].required_fall);
        double gd = gate_delay(topo_[i]);

        for (auto ni : g.inputs) {
            double in_req = out_req - gd;
            pin_timing_[ni].required_rise = std::min(pin_timing_[ni].required_rise, in_req);
            pin_timing_[ni].required_fall = std::min(pin_timing_[ni].required_fall, in_req);
        }
    }
}

void StaEngine::compute_slacks() {
    for (auto& [nid, pt] : pin_timing_) {
        pt.slack_rise = pt.required_rise - pt.arrival_rise;
        pt.slack_fall = pt.required_fall - pt.arrival_fall;
    }
}

std::vector<TimingPath> StaEngine::extract_paths(int count) {
    std::vector<TimingPath> paths;

    // Collect endpoint slacks
    struct EndpointSlack {
        NetId net;
        double slack;
        std::string name;
    };
    std::vector<EndpointSlack> endpoints;

    for (auto po : nl_.primary_outputs()) {
        auto& pt = pin_timing_[po];
        endpoints.push_back({po, pt.worst_slack(), nl_.net(po).name});
    }
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) {
            NetId d = ff.inputs[0];
            auto& pt = pin_timing_[d];
            endpoints.push_back({d, pt.worst_slack(), ff.name + "/D"});
        }
    }

    std::sort(endpoints.begin(), endpoints.end(),
              [](auto& a, auto& b) { return a.slack < b.slack; });

    // For each worst endpoint, trace back the critical path
    for (int p = 0; p < std::min(count, (int)endpoints.size()); ++p) {
        TimingPath path;
        path.slack = endpoints[p].slack;
        path.endpoint = endpoints[p].name;

        // Backwards trace from endpoint
        NetId curr = endpoints[p].net;
        path.nets.push_back(curr);


        for (int depth = 0; depth < 100; ++depth) {
            GateId drv = nl_.net(curr).driver;
            if (drv < 0) break;
            auto& g = nl_.gate(drv);
            if (g.type == GateType::DFF || g.type == GateType::INPUT) {
                path.startpoint = g.name;
                break;
            }
            path.gates.push_back(drv);

            // Find the input with worst arrival
            NetId worst_in = -1;
            double worst_arr = -1;
            for (auto ni : g.inputs) {
                double arr = pin_timing_[ni].worst_arrival();
                if (arr > worst_arr) { worst_arr = arr; worst_in = ni; }
            }
            if (worst_in < 0) break;
            path.nets.push_back(worst_in);
            curr = worst_in;
        }

        path.delay = pin_timing_[endpoints[p].net].worst_arrival();
        std::reverse(path.nets.begin(), path.nets.end());
        std::reverse(path.gates.begin(), path.gates.end());
        paths.push_back(path);
    }

    return paths;
}

StaResult StaEngine::analyze(double clock_period, int num_paths) {
    auto t0 = std::chrono::high_resolution_clock::now();

    build_timing_graph();
    forward_propagation();
    backward_propagation(clock_period);
    compute_slacks();

    StaResult result;
    result.clock_period = clock_period;
    result.critical_paths = extract_paths(num_paths);

    // Compute WNS and TNS
    result.wns = 0;
    result.tns = 0;
    result.num_violations = 0;
    result.num_endpoints = 0;

    auto check_endpoint = [&](NetId nid) {
        result.num_endpoints++;
        double slack = pin_timing_[nid].worst_slack();
        if (slack < 0) {
            result.num_violations++;
            result.tns += slack;
            result.wns = std::min(result.wns, slack);
        }
    };

    for (auto po : nl_.primary_outputs()) check_endpoint(po);
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) check_endpoint(ff.inputs[0]);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = result.num_violations == 0
        ? "TIMING MET — WNS: " + std::to_string(result.wns) + "ns"
        : "TIMING VIOLATED — WNS: " + std::to_string(result.wns) + "ns, " +
          std::to_string(result.num_violations) + " violations";

    return result;
}

} // namespace sf
