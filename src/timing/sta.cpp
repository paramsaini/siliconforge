// SiliconForge — Static Timing Analysis Implementation
// Slew-aware delay, hold checks, multi-corner derating, clock uncertainty.
#include "timing/sta.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <cmath>
#include <functional>

namespace sf {

// === Delay Calculation with Slew Awareness ===

double StaEngine::gate_delay(GateId gid, double input_slew) const {
    auto& g = nl_.gate(gid);
    // Slew-dependent delay: base_delay * (1 + k * slew)
    // k models the slow-input penalty seen in real NLDM tables
    double slew_factor = 1.0 + 0.5 * input_slew; // 50% penalty per ns of slew

    if (lib_) {
        std::string type_str = gate_type_str(g.type);
        int num_in = (int)g.inputs.size();
        std::string candidates[] = {
            g.name,
            type_str + std::to_string(num_in),
            type_str,
            type_str + "_X1",
            type_str + std::to_string(num_in) + "_X1"
        };
        for (auto& cand : candidates) {
            if (auto* cell = lib_->find_cell(cand)) {
                for (auto& t : cell->timings) {
                    double d = (t.cell_rise + t.cell_fall) / 2.0;
                    if (d > 0) return d * slew_factor * derate_.cell_derate;
                }
                if (cell->area > 0) return cell->area * 0.01 * slew_factor * derate_.cell_derate;
            }
        }
    }
    // Default delay model
    double base = 0.05;
    switch (g.type) {
        case GateType::BUF:    base = 0.02; break;
        case GateType::NOT:    base = 0.02; break;
        case GateType::AND:    base = 0.05 * g.inputs.size(); break;
        case GateType::OR:     base = 0.05 * g.inputs.size(); break;
        case GateType::NAND:   base = 0.04 * g.inputs.size(); break;
        case GateType::NOR:    base = 0.04 * g.inputs.size(); break;
        case GateType::XOR:    base = 0.08; break;
        case GateType::XNOR:   base = 0.08; break;
        case GateType::MUX:    base = 0.06; break;
        case GateType::DFF:    base = 0.10; break;
        default:               base = 0.05; break;
    }
    return base * slew_factor * derate_.cell_derate;
}

double StaEngine::output_slew(GateId gid, double input_slew, double load_cap) const {
    auto& g = nl_.gate(gid);
    // Output slew model: slew_out = slew_intrinsic + R_drive * C_load
    // Intrinsic slew depends on cell type and input slew
    double slew_intrinsic = 0.005; // 5ps base
    double r_drive = 0.5; // kOhm effective output resistance

    if (lib_) {
        std::string type_str = gate_type_str(g.type);
        int num_in = (int)g.inputs.size();
        std::string candidates[] = {
            g.name, type_str + std::to_string(num_in),
            type_str, type_str + "_X1"
        };
        for (auto& cand : candidates) {
            if (auto* cell = lib_->find_cell(cand)) {
                for (auto& t : cell->timings) {
                    double s = (t.rise_transition + t.fall_transition) / 2.0;
                    if (s > 0) {
                        // Scale by load: Liberty slew is at nominal load
                        return s * (1.0 + load_cap * 0.5) * derate_.cell_derate;
                    }
                }
                // Estimate from area (larger cell = lower drive resistance)
                if (cell->area > 0) r_drive = 2.0 / cell->area;
            }
        }
    }

    // Gate-type-specific intrinsic slew
    switch (g.type) {
        case GateType::BUF:  case GateType::NOT: slew_intrinsic = 0.003; break;
        case GateType::AND:  case GateType::OR:  slew_intrinsic = 0.005 * g.inputs.size(); break;
        case GateType::NAND: case GateType::NOR: slew_intrinsic = 0.004 * g.inputs.size(); break;
        case GateType::XOR:  case GateType::XNOR: slew_intrinsic = 0.008; break;
        case GateType::DFF:  slew_intrinsic = 0.010; break;
        default: break;
    }

    // Output slew = intrinsic + input_slew_degradation + RC
    double slew_out = slew_intrinsic + input_slew * 0.3 + r_drive * load_cap;
    return std::max(0.001, slew_out * derate_.cell_derate);
}

double StaEngine::net_load_cap(NetId nid) const {
    auto& net = nl_.net(nid);
    double c_wire = 0.001; // 1fF base wire
    double c_load = 0;
    if (lib_) {
        for (auto gid : net.fanout) {
            auto& g = nl_.gate(gid);
            std::string type_str = gate_type_str(g.type);
            int num_in = (int)g.inputs.size();
            std::string candidates[] = {
                type_str + std::to_string(num_in), type_str, type_str + "_X1"
            };
            bool found = false;
            for (auto& cand : candidates) {
                if (auto* cell = lib_->find_cell(cand)) {
                    for (auto& pin : cell->pins) {
                        if (pin.direction == "input" && pin.capacitance > 0) {
                            c_load += pin.capacitance;
                            found = true;
                            break;
                        }
                    }
                    if (found) break;
                }
            }
            if (!found) c_load += 0.002;
        }
    } else {
        c_load = net.fanout.size() * 0.002;
    }
    return c_wire + c_load;
}

double StaEngine::wire_delay(NetId from, NetId to) const {
    auto& net = nl_.net(from);
    double fanout = (double)net.fanout.size();

    if (pd_ && !pd_->wires.empty()) {
        double total_wire_length = 0;
        for (auto& w : pd_->wires)
            total_wire_length += w.start.dist(w.end);
        int num_nets = std::max(1, (int)pd_->nets.size());
        double avg_wire_len = total_wire_length / num_nets;
        double r_per_um = 0.10;
        double c_per_um = 0.0002;
        double elmore = r_per_um * c_per_um * avg_wire_len * avg_wire_len / 2.0;
        double via_penalty = 0.0005 * (double)pd_->vias.size() / num_nets;
        return (elmore + via_penalty + 0.001 * fanout) * derate_.wire_derate;
    }

    return 0.001 * fanout * derate_.wire_derate;
}

// === Timing Graph ===

void StaEngine::build_timing_graph() {
    arcs_.clear();
    topo_ = nl_.topo_order();

    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double in_slew = 0.01; // will be refined in propagation
        double gd = gate_delay(gid, in_slew);
        for (auto ni : g.inputs) {
            double wd = wire_delay(ni, g.output);
            arcs_.push_back({ni, g.output, gd + wd, in_slew, gid});
        }
    }

    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0)
            arcs_.push_back({ff.clk, ff.output, gate_delay(gid), 0.01, gid});
    }
}

// === Forward Propagation with Slew ===

void StaEngine::forward_propagation(double input_arrival) {
    pin_timing_.clear();

    // Initialize PIs
    double pi_slew = 0.01; // 10ps input slew
    for (auto pi : nl_.primary_inputs()) {
        auto& pt = pin_timing_[pi];
        pt.arrival_rise = input_arrival;
        pt.arrival_fall = input_arrival;
        pt.slew_rise = pi_slew;
        pt.slew_fall = pi_slew;
    }
    // FF Q outputs — include clock insertion delay from CTS
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0) {
            double ckq = gate_delay(gid, pi_slew);
            double insertion = 0;
            auto ci = clock_insertion_.find(gid);
            if (ci != clock_insertion_.end()) insertion = ci->second;
            auto& pt = pin_timing_[ff.output];
            pt.arrival_rise = insertion + ckq;
            pt.arrival_fall = insertion + ckq;
            double load = net_load_cap(ff.output);
            pt.slew_rise = output_slew(gid, pi_slew, load);
            pt.slew_fall = pt.slew_rise;
        }
    }

    // Init remaining nets
    for (size_t i = 0; i < nl_.num_nets(); ++i)
        if (!pin_timing_.count(i)) pin_timing_[i] = {};

    // Propagate in topo order — LATE (setup) analysis
    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        // Find worst (latest) input arrival and its slew
        double max_arr = 0;
        double worst_slew = pi_slew;
        for (auto ni : g.inputs) {
            double arr = pin_timing_[ni].worst_arrival();
            if (arr > max_arr) {
                max_arr = arr;
                worst_slew = std::max(pin_timing_[ni].slew_rise, pin_timing_[ni].slew_fall);
            }
        }

        // Slew-aware gate delay
        double gd = gate_delay(gid, worst_slew);
        double wd = wire_delay(g.inputs.empty() ? -1 : g.inputs[0], g.output);
        double out_arr = max_arr + gd + wd;

        auto& pt = pin_timing_[g.output];
        pt.arrival_rise = std::max(pt.arrival_rise, out_arr);
        pt.arrival_fall = std::max(pt.arrival_fall, out_arr);

        // Propagate slew
        double load = net_load_cap(g.output);
        double out_slew = output_slew(gid, worst_slew, load);
        pt.slew_rise = std::max(pt.slew_rise, out_slew);
        pt.slew_fall = std::max(pt.slew_fall, out_slew);
    }
}

// === Hold Forward Propagation (EARLY paths) ===

void StaEngine::hold_forward_propagation(double input_arrival) {
    double pi_slew = 0.01;
    // Initialize PIs — early arrival = 0
    for (auto pi : nl_.primary_inputs()) {
        auto& pt = pin_timing_[pi];
        pt.hold_arrival_rise = input_arrival;
        pt.hold_arrival_fall = input_arrival;
    }
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0) {
            double ckq_early = gate_delay(gid, pi_slew) * derate_.early_cell;
            auto& pt = pin_timing_[ff.output];
            pt.hold_arrival_rise = ckq_early;
            pt.hold_arrival_fall = ckq_early;
        }
    }

    // Propagate MINIMUM (earliest) arrivals
    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double min_arr = std::numeric_limits<double>::max();
        double best_slew = pi_slew;
        for (auto ni : g.inputs) {
            double arr = std::min(pin_timing_[ni].hold_arrival_rise,
                                   pin_timing_[ni].hold_arrival_fall);
            if (arr < min_arr) {
                min_arr = arr;
                best_slew = std::min(pin_timing_[ni].slew_rise, pin_timing_[ni].slew_fall);
            }
        }
        if (min_arr == std::numeric_limits<double>::max()) min_arr = 0;

        // Early delay: cell_delay * early_derate
        double gd = gate_delay(gid, best_slew) * derate_.early_cell / derate_.cell_derate;
        double wd = wire_delay(g.inputs.empty() ? -1 : g.inputs[0], g.output)
                     * derate_.early_wire / derate_.wire_derate;
        double out_arr = min_arr + gd + wd;

        auto& pt = pin_timing_[g.output];
        if (pt.hold_arrival_rise == 0 || out_arr < pt.hold_arrival_rise)
            pt.hold_arrival_rise = out_arr;
        if (pt.hold_arrival_fall == 0 || out_arr < pt.hold_arrival_fall)
            pt.hold_arrival_fall = out_arr;
    }
}

// === Backward Propagation (Setup) ===

void StaEngine::backward_propagation(double clock_period) {
    for (auto po : nl_.primary_outputs()) {
        pin_timing_[po].required_rise = clock_period;
        pin_timing_[po].required_fall = clock_period;
    }

    // FF D-input: required = T - setup_time - uncertainty + clock_insertion_capture
    double setup_time = 0.05;
    double setup_margin = setup_time + setup_uncertainty_;
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) {
            NetId d = ff.inputs[0];
            double capture_insertion = 0;
            auto ci = clock_insertion_.find(gid);
            if (ci != clock_insertion_.end()) capture_insertion = ci->second;
            double req = clock_period + capture_insertion - setup_margin;
            pin_timing_[d].required_rise = std::min(pin_timing_[d].required_rise, req);
            pin_timing_[d].required_fall = std::min(pin_timing_[d].required_fall, req);
        }
    }

    for (int i = (int)topo_.size() - 1; i >= 0; --i) {
        auto& g = nl_.gate(topo_[i]);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double out_req = std::min(pin_timing_[g.output].required_rise,
                                   pin_timing_[g.output].required_fall);
        double worst_slew = 0.01;
        for (auto ni : g.inputs)
            worst_slew = std::max(worst_slew, pin_timing_[ni].slew_rise);
        double gd = gate_delay(topo_[i], worst_slew);

        for (auto ni : g.inputs) {
            double in_req = out_req - gd;
            pin_timing_[ni].required_rise = std::min(pin_timing_[ni].required_rise, in_req);
            pin_timing_[ni].required_fall = std::min(pin_timing_[ni].required_fall, in_req);
        }
    }
}

// === Backward Propagation (Hold) ===

void StaEngine::hold_backward_propagation() {
    // Hold required time at FF D = hold_time + uncertainty (data must be stable)
    double hold_time = 0.02; // 20ps typical hold time
    double hold_margin = hold_time + hold_uncertainty_;

    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) {
            NetId d = ff.inputs[0];
            pin_timing_[d].hold_required_rise = hold_margin;
            pin_timing_[d].hold_required_fall = hold_margin;
        }
    }
    for (auto po : nl_.primary_outputs()) {
        pin_timing_[po].hold_required_rise = 0;
        pin_timing_[po].hold_required_fall = 0;
    }
}

// === Slack Computation ===

void StaEngine::compute_slacks() {
    for (auto& [nid, pt] : pin_timing_) {
        // Setup slack
        pt.slack_rise = pt.required_rise - pt.arrival_rise;
        pt.slack_fall = pt.required_fall - pt.arrival_fall;
        // Hold slack = early_arrival - hold_required
        pt.hold_slack_rise = pt.hold_arrival_rise - pt.hold_required_rise;
        pt.hold_slack_fall = pt.hold_arrival_fall - pt.hold_required_fall;
    }
}

// === Path Extraction ===

std::vector<TimingPath> StaEngine::extract_paths(int count, bool include_hold) {
    std::vector<TimingPath> paths;

    struct EndpointSlack {
        NetId net; double slack; std::string name; bool is_hold;
    };
    std::vector<EndpointSlack> endpoints;

    // Setup endpoints
    for (auto po : nl_.primary_outputs()) {
        auto& pt = pin_timing_[po];
        endpoints.push_back({po, pt.worst_slack(), nl_.net(po).name, false});
    }
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) {
            NetId d = ff.inputs[0];
            auto& pt = pin_timing_[d];
            endpoints.push_back({d, pt.worst_slack(), ff.name + "/D", false});
        }
    }

    // Hold endpoints
    if (include_hold) {
        for (auto gid : nl_.flip_flops()) {
            auto& ff = nl_.gate(gid);
            if (!ff.inputs.empty()) {
                NetId d = ff.inputs[0];
                auto& pt = pin_timing_[d];
                endpoints.push_back({d, pt.worst_hold_slack(), ff.name + "/D(hold)", true});
            }
        }
    }

    std::sort(endpoints.begin(), endpoints.end(),
              [](auto& a, auto& b) { return a.slack < b.slack; });

    for (int p = 0; p < std::min(count, (int)endpoints.size()); ++p) {
        TimingPath path;
        path.slack = endpoints[p].slack;
        path.endpoint = endpoints[p].name;
        path.is_hold = endpoints[p].is_hold;

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

// === Single-Corner Analyze ===

StaResult StaEngine::analyze_corner(double clock_period, int num_paths, const CornerDerate& d) {
    derate_ = d;
    auto t0 = std::chrono::high_resolution_clock::now();

    build_timing_graph();
    forward_propagation();
    hold_forward_propagation();
    backward_propagation(clock_period);
    hold_backward_propagation();
    compute_slacks();

    StaResult result;
    result.clock_period = clock_period;
    result.corner_name = d.name;
    result.critical_paths = extract_paths(num_paths, true);

    result.wns = 0; result.tns = 0; result.num_violations = 0;
    result.hold_wns = 0; result.hold_tns = 0; result.hold_violations = 0;
    result.num_endpoints = 0;

    auto check_endpoint = [&](NetId nid) {
        result.num_endpoints++;
        double slack = pin_timing_[nid].worst_slack();
        if (slack < 0) {
            result.num_violations++;
            result.tns += slack;
            result.wns = std::min(result.wns, slack);
        }
        // Hold check
        double hslack = pin_timing_[nid].worst_hold_slack();
        if (hslack < 0) {
            result.hold_violations++;
            result.hold_tns += hslack;
            result.hold_wns = std::min(result.hold_wns, hslack);
        }
    };

    for (auto po : nl_.primary_outputs()) check_endpoint(po);
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) check_endpoint(ff.inputs[0]);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::string msg = "[" + d.name + "] ";
    if (result.num_violations == 0 && result.hold_violations == 0)
        msg += "TIMING MET — WNS(setup): " + std::to_string(result.wns) +
               "ns, WNS(hold): " + std::to_string(result.hold_wns) + "ns";
    else {
        if (result.num_violations > 0)
            msg += "SETUP VIOLATED — WNS: " + std::to_string(result.wns) + "ns (" +
                   std::to_string(result.num_violations) + " violations). ";
        if (result.hold_violations > 0)
            msg += "HOLD VIOLATED — WNS: " + std::to_string(result.hold_wns) + "ns (" +
                   std::to_string(result.hold_violations) + " violations).";
    }
    result.message = msg;
    return result;
}

// === Public analyze() — typical corner ===

StaResult StaEngine::analyze(double clock_period, int num_paths) {
    CornerDerate typ;
    typ.name = "typical";
    typ.cell_derate = 1.0;
    typ.wire_derate = 1.0;
    typ.early_cell = 1.0;
    typ.early_wire = 1.0;
    return analyze_corner(clock_period, num_paths, typ);
}

// === Multi-Corner Analysis ===

std::vector<StaResult> StaEngine::analyze_multicorner(double clock_period, int num_paths) {
    std::vector<StaResult> results;

    // Best corner (fast process, high voltage, low temperature)
    CornerDerate best;
    best.name = "best";
    best.cell_derate = 0.85;  // 15% faster cells
    best.wire_derate = 0.90;  // 10% faster wires
    best.early_cell = 0.80;   // early data for hold = even faster
    best.early_wire = 0.85;
    results.push_back(analyze_corner(clock_period, num_paths, best));

    // Typical corner
    CornerDerate typ;
    typ.name = "typical";
    results.push_back(analyze_corner(clock_period, num_paths, typ));

    // Worst corner (slow process, low voltage, high temperature)
    CornerDerate worst;
    worst.name = "worst";
    worst.cell_derate = 1.25;  // 25% slower cells
    worst.wire_derate = 1.20;  // 20% slower wires
    worst.early_cell = 1.15;   // early data is still slow
    worst.early_wire = 1.10;
    results.push_back(analyze_corner(clock_period, num_paths, worst));

    return results;
}

} // namespace sf
