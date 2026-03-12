// SiliconForge — Static Timing Analysis Implementation
// Slew-aware delay, hold checks, multi-corner derating, OCV/AOCV.
#include "timing/sta.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <cmath>
#include <functional>

namespace sf {

// === OCV/AOCV Effective Derate ===

double StaEngine::effective_cell_derate(GateId gid) const {
    if (ocv_mode_ == OcvMode::AOCV) {
        int depth = 1;
        auto it = gate_depth_.find(gid);
        if (it != gate_depth_.end()) depth = std::max(1, it->second);

        if (analyzing_late_)
            return aocv_table_.late_derate(depth) * derate_.cell_derate;
        else
            return aocv_table_.early_derate(depth) * derate_.early_cell;
    }
    if (ocv_mode_ == OcvMode::POCV) {
        // POCV: base derate is 1.0, path-level sigma applied later via PBA
        // At gate level, use 1.0 (mean) — the statistical adjustment happens per-path
        return analyzing_late_ ? derate_.cell_derate : derate_.early_cell;
    }
    if (ocv_mode_ == OcvMode::OCV) {
        if (analyzing_late_)
            return derate_.cell_derate * ocv_late_cell_;
        else
            return derate_.early_cell * ocv_early_cell_;
    }
    // NONE: use base corner derate
    return analyzing_late_ ? derate_.cell_derate : derate_.early_cell;
}

double StaEngine::effective_wire_derate(GateId gid) const {
    if (ocv_mode_ == OcvMode::AOCV) {
        int depth = 1;
        auto it = gate_depth_.find(gid);
        if (it != gate_depth_.end()) depth = std::max(1, it->second);

        if (analyzing_late_)
            return (1.0 + aocv_table_.late_variation / std::sqrt((double)depth))
                   * derate_.wire_derate;
        else
            return (1.0 - aocv_table_.early_variation / std::sqrt((double)depth))
                   * derate_.early_wire;
    }
    if (ocv_mode_ == OcvMode::POCV) {
        return analyzing_late_ ? derate_.wire_derate : derate_.early_wire;
    }
    if (ocv_mode_ == OcvMode::OCV) {
        if (analyzing_late_)
            return derate_.wire_derate * ocv_late_cell_;  // use same ratio
        else
            return derate_.early_wire * ocv_early_cell_;
    }
    return analyzing_late_ ? derate_.wire_derate : derate_.early_wire;
}

// === Gate Depth Computation (for AOCV) ===

void StaEngine::compute_gate_depths() {
    gate_depth_.clear();
    if (ocv_mode_ != OcvMode::AOCV && ocv_mode_ != OcvMode::POCV) return;

    // Depth = number of logic levels from nearest DFF/PI
    // BFS from DFF outputs and PIs, counting levels through topo order
    std::unordered_map<NetId, int> net_depth;
    for (auto pi : nl_.primary_inputs()) net_depth[pi] = 0;
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0) net_depth[ff.output] = 0;
    }

    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0)
            continue;

        int max_in_depth = 0;
        for (auto ni : g.inputs) {
            auto it = net_depth.find(ni);
            if (it != net_depth.end())
                max_in_depth = std::max(max_in_depth, it->second);
        }
        int d = max_in_depth + 1;
        gate_depth_[gid] = d;
        net_depth[g.output] = d;
    }
}

// === Delay Calculation with Slew + OCV/AOCV Awareness ===

double StaEngine::gate_delay(GateId gid, double input_slew) const {
    auto& g = nl_.gate(gid);
    double slew_factor = 1.0 + 0.5 * input_slew;
    double load = net_load_cap(g.output >= 0 ? g.output : 0);

    // Determine effective derate (OCV/AOCV-aware)
    double eff_derate = effective_cell_derate(gid);

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
                    if (t.nldm_rise.valid() && t.nldm_fall.valid()) {
                        double d_rise = t.nldm_rise.interpolate(input_slew, load);
                        double d_fall = t.nldm_fall.interpolate(input_slew, load);
                        return std::max(d_rise, d_fall) * eff_derate;
                    }
                    double d = (t.cell_rise + t.cell_fall) / 2.0;
                    if (d > 0) return d * slew_factor * eff_derate;
                }
                if (cell->area > 0) return cell->area * 0.01 * slew_factor * eff_derate;
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
    return base * slew_factor * eff_derate;
}

double StaEngine::output_slew(GateId gid, double input_slew, double load_cap) const {
    auto& g = nl_.gate(gid);
    double slew_intrinsic = 0.005;
    double r_drive = 0.5;

    double eff_derate = effective_cell_derate(gid);

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
                    if (t.nldm_rise_tr.valid() && t.nldm_fall_tr.valid()) {
                        double s_rise = t.nldm_rise_tr.interpolate(input_slew, load_cap);
                        double s_fall = t.nldm_fall_tr.interpolate(input_slew, load_cap);
                        return std::max(s_rise, s_fall) * eff_derate;
                    }
                    double s = (t.rise_transition + t.fall_transition) / 2.0;
                    if (s > 0) {
                        return s * (1.0 + load_cap * 0.5) * eff_derate;
                    }
                }
                if (cell->area > 0) r_drive = 2.0 / cell->area;
            }
        }
    }

    switch (g.type) {
        case GateType::BUF:  case GateType::NOT: slew_intrinsic = 0.003; break;
        case GateType::AND:  case GateType::OR:  slew_intrinsic = 0.005 * g.inputs.size(); break;
        case GateType::NAND: case GateType::NOR: slew_intrinsic = 0.004 * g.inputs.size(); break;
        case GateType::XOR:  case GateType::XNOR: slew_intrinsic = 0.008; break;
        case GateType::DFF:  slew_intrinsic = 0.010; break;
        default: break;
    }

    double slew_out = slew_intrinsic + input_slew * 0.3 + r_drive * load_cap;
    return std::max(0.001, slew_out * eff_derate);
}

double StaEngine::net_load_cap(NetId nid) const {
    auto& net = nl_.net(nid);
    double c_wire = 0.001;
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
    // Bounds check: invalid or out-of-range NetId → zero wire delay
    if (from < 0 || from >= (NetId)nl_.num_nets()) return 0.0;
    auto& net = nl_.net(from);
    double fanout = (double)net.fanout.size();
    double eff_wd = analyzing_late_ ? derate_.wire_derate : derate_.early_wire;

    // For AOCV, wire derate is depth-aware (use driver gate depth)
    if (ocv_mode_ == OcvMode::AOCV && net.driver >= 0) {
        int depth = 1;
        auto it = gate_depth_.find(net.driver);
        if (it != gate_depth_.end()) depth = std::max(1, it->second);
        if (analyzing_late_)
            eff_wd = (1.0 + aocv_table_.late_variation / std::sqrt((double)depth))
                     * derate_.wire_derate;
        else
            eff_wd = (1.0 - aocv_table_.early_variation / std::sqrt((double)depth))
                     * derate_.early_wire;
    } else if (ocv_mode_ == OcvMode::OCV) {
        if (analyzing_late_)
            eff_wd = derate_.wire_derate * ocv_late_cell_;
        else
            eff_wd = derate_.early_wire * ocv_early_cell_;
    }

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
        return (elmore + via_penalty + 0.001 * fanout) * eff_wd;
    }

    return 0.001 * fanout * eff_wd;
}

// === Timing Graph ===

void StaEngine::build_timing_graph() {
    arcs_.clear();
    topo_ = nl_.topo_order();

    // Compute gate depths for AOCV before building arcs
    compute_gate_depths();

    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double in_slew = 0.01;
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

// === Forward Propagation with Slew (LATE paths for setup) ===

void StaEngine::forward_propagation(double input_arrival) {
    analyzing_late_ = true;
    pin_timing_.clear();

    double pi_slew = 0.01;
    for (auto pi : nl_.primary_inputs()) {
        auto& pt = pin_timing_[pi];
        pt.arrival_rise = input_arrival;
        pt.arrival_fall = input_arrival;
        pt.slew_rise = pi_slew;
        pt.slew_fall = pi_slew;
    }
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

    for (size_t i = 0; i < nl_.num_nets(); ++i)
        if (!pin_timing_.count(i)) pin_timing_[i] = {};

    // Propagate LATE (latest) arrivals for setup analysis
    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double max_arr = 0;
        double worst_slew = pi_slew;
        for (auto ni : g.inputs) {
            double arr = pin_timing_[ni].worst_arrival();
            if (arr > max_arr) {
                max_arr = arr;
                worst_slew = std::max(pin_timing_[ni].slew_rise, pin_timing_[ni].slew_fall);
            }
        }

        double gd = gate_delay(gid, worst_slew);
        double wd = wire_delay(g.inputs.empty() ? -1 : g.inputs[0], g.output);
        double out_arr = max_arr + gd + wd;

        auto& pt = pin_timing_[g.output];
        pt.arrival_rise = std::max(pt.arrival_rise, out_arr);
        pt.arrival_fall = std::max(pt.arrival_fall, out_arr);

        double load = net_load_cap(g.output);
        double out_slew = output_slew(gid, worst_slew, load);
        pt.slew_rise = std::max(pt.slew_rise, out_slew);
        pt.slew_fall = std::max(pt.slew_fall, out_slew);
    }
}

// === Hold Forward Propagation (EARLY paths) ===

void StaEngine::hold_forward_propagation(double input_arrival) {
    analyzing_late_ = false;  // switch to early-path derating
    double pi_slew = 0.01;

    for (auto pi : nl_.primary_inputs()) {
        auto& pt = pin_timing_[pi];
        pt.hold_arrival_rise = input_arrival;
        pt.hold_arrival_fall = input_arrival;
    }
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0) {
            // Early CK→Q delay uses early derate (via effective_cell_derate)
            double ckq_early = gate_delay(gid, pi_slew);
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

        // Early delay: gate_delay already applies early derate via effective_cell_derate
        double gd = gate_delay(gid, best_slew);
        double wd = wire_delay(g.inputs.empty() ? -1 : g.inputs[0], g.output);
        double out_arr = min_arr + gd + wd;

        auto& pt = pin_timing_[g.output];
        if (pt.hold_arrival_rise == 0 || out_arr < pt.hold_arrival_rise)
            pt.hold_arrival_rise = out_arr;
        if (pt.hold_arrival_fall == 0 || out_arr < pt.hold_arrival_fall)
            pt.hold_arrival_fall = out_arr;
    }

    analyzing_late_ = true;  // restore default
}

// === Backward Propagation (Setup) with OCV Clock Path Derating ===

void StaEngine::backward_propagation(double clock_period) {
    for (auto po : nl_.primary_outputs()) {
        pin_timing_[po].required_rise = clock_period;
        pin_timing_[po].required_fall = clock_period;
    }

    double setup_time = 0.05;
    double setup_margin = setup_time + setup_uncertainty_;

    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) {
            NetId d = ff.inputs[0];
            double capture_insertion = 0;
            auto ci = clock_insertion_.find(gid);
            if (ci != clock_insertion_.end()) capture_insertion = ci->second;

            // OCV: capture clock path uses EARLY derate for setup analysis
            // (clock arrives early → less time for data → pessimistic)
            double clk_derate = 1.0;
            if (ocv_mode_ == OcvMode::OCV)
                clk_derate = ocv_early_cell_;
            else if (ocv_mode_ == OcvMode::AOCV)
                clk_derate = aocv_table_.early_derate(1); // clock tree depth ~1

            double req = clock_period + capture_insertion * clk_derate - setup_margin;
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

// === Backward Propagation (Hold) with OCV Clock Path Derating ===

void StaEngine::hold_backward_propagation() {
    double hold_time = 0.02;
    double hold_margin = hold_time + hold_uncertainty_;

    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) {
            NetId d = ff.inputs[0];

            // OCV: capture clock for hold uses LATE derate
            // (clock arrives late → hold margin shrinks → pessimistic)
            double clk_derate = 1.0;
            if (ocv_mode_ == OcvMode::OCV)
                clk_derate = ocv_late_cell_;
            else if (ocv_mode_ == OcvMode::AOCV)
                clk_derate = aocv_table_.late_derate(1);

            double req = hold_margin * clk_derate;
            pin_timing_[d].hold_required_rise = req;
            pin_timing_[d].hold_required_fall = req;
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
        pt.slack_rise = pt.required_rise - pt.arrival_rise;
        pt.slack_fall = pt.required_fall - pt.arrival_fall;
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
        int depth = 0;

        for (int d = 0; d < 100; ++d) {
            GateId drv = nl_.net(curr).driver;
            if (drv < 0) break;
            auto& g = nl_.gate(drv);
            if (g.type == GateType::DFF || g.type == GateType::INPUT) {
                path.startpoint = g.name;
                break;
            }
            path.gates.push_back(drv);
            ++depth;

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
        path.depth = depth;
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
    result.ocv_mode = ocv_mode_;
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

    // === Industrial STA passes ===
    
    // 1. Crosstalk: add delta-delay to wire delays (already factored in PBA)
    result.crosstalk_enabled = xtalk_.enabled;
    if (xtalk_.enabled) {
        double max_xt = 0;
        for (auto& path : result.critical_paths) {
            double xt_total = 0;
            for (auto gid : path.gates) {
                auto& g = nl_.gate(gid);
                if (g.output >= 0) {
                    double xt = compute_crosstalk_delta(g.output);
                    xt_total += xt;
                }
            }
            path.crosstalk_delta = xt_total;
            max_xt = std::max(max_xt, xt_total);
        }
        result.max_crosstalk_delta = max_xt;
    }
    
    // 2. CPPR: remove common clock path pessimism
    result.cppr_enabled = cppr_.enabled;
    compute_cppr_credits(result.critical_paths, clock_period);
    if (cppr_.enabled) {
        double total_credit = 0;
        for (auto& p : result.critical_paths)
            total_credit += p.cppr_credit;
        result.cppr_total_credit = total_credit;
    }
    
    // 3. PBA: path-based re-analysis with specific slew + POCV sigma
    result.pba_enabled = pba_enabled_;
    pba_reanalyze(result.critical_paths, clock_period);
    if (pba_enabled_) {
        result.pba_wns = 0;
        result.pba_tns = 0;
        result.pba_violations = 0;
        for (auto& p : result.critical_paths) {
            if (p.pba_valid && !p.is_hold) {
                result.pba_wns = std::min(result.pba_wns, p.pba_slack);
                if (p.pba_slack < 0) {
                    result.pba_tns += p.pba_slack;
                    result.pba_violations++;
                }
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::string ocv_str;
    if (ocv_mode_ == OcvMode::OCV) ocv_str = " +OCV";
    else if (ocv_mode_ == OcvMode::AOCV) ocv_str = " +AOCV";
    else if (ocv_mode_ == OcvMode::POCV) ocv_str = " +POCV";

    std::string msg = "[" + d.name + ocv_str + "] ";
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
    if (cppr_.enabled)
        msg += " CPPR: " + std::to_string(result.cppr_total_credit) + "ns credit.";
    if (pba_enabled_)
        msg += " PBA WNS: " + std::to_string(result.pba_wns) + "ns.";
    if (xtalk_.enabled)
        msg += " Xtalk max: " + std::to_string(result.max_crosstalk_delta) + "ns.";
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
    best.cell_derate = 0.85;
    best.wire_derate = 0.90;
    best.early_cell = 0.80;
    best.early_wire = 0.85;
    best.clock_late_cell = 1.15;   // clock late for hold check
    best.clock_early_cell = 0.80;  // clock early for setup check
    results.push_back(analyze_corner(clock_period, num_paths, best));

    // Typical corner
    CornerDerate typ;
    typ.name = "typical";
    results.push_back(analyze_corner(clock_period, num_paths, typ));

    // Worst corner (slow process, low voltage, high temperature)
    CornerDerate worst;
    worst.name = "worst";
    worst.cell_derate = 1.25;
    worst.wire_derate = 1.20;
    worst.early_cell = 1.15;
    worst.early_wire = 1.10;
    worst.clock_late_cell = 1.25;
    worst.clock_early_cell = 0.85;
    results.push_back(analyze_corner(clock_period, num_paths, worst));

    return results;
}

// === Industrial STA: CPPR ===

void StaEngine::compute_cppr_credits(std::vector<TimingPath>& paths, double clock_period) {
    if (!cppr_.enabled) return;
    if (ocv_mode_ == OcvMode::NONE) return; // No OCV = no pessimism to remove
    
    double late_factor = 1.0, early_factor = 1.0;
    if (ocv_mode_ == OcvMode::OCV) {
        late_factor = ocv_late_cell_;
        early_factor = ocv_early_cell_;
    } else if (ocv_mode_ == OcvMode::AOCV) {
        late_factor = aocv_table_.late_derate(1);
        early_factor = aocv_table_.early_derate(1);
    } else if (ocv_mode_ == OcvMode::POCV) {
        late_factor = 1.0 + pocv_table_.default_sigma_pct * pocv_table_.n_sigma;
        early_factor = 1.0 - pocv_table_.default_sigma_pct * pocv_table_.n_sigma;
    }
    
    for (auto& path : paths) {
        double launch_ins = 0, capture_ins = 0;
        
        for (auto gid : nl_.flip_flops()) {
            auto& ff = nl_.gate(gid);
            if (ff.name == path.startpoint || ff.name + "/Q" == path.startpoint) {
                auto ci = clock_insertion_.find(gid);
                if (ci != clock_insertion_.end()) launch_ins = ci->second;
                break;
            }
        }
        
        std::string cap_name = path.endpoint;
        auto slash_pos = cap_name.find('/');
        if (slash_pos != std::string::npos) cap_name = cap_name.substr(0, slash_pos);
        
        for (auto gid : nl_.flip_flops()) {
            auto& ff = nl_.gate(gid);
            if (ff.name == cap_name) {
                auto ci = clock_insertion_.find(gid);
                if (ci != clock_insertion_.end()) capture_ins = ci->second;
                break;
            }
        }
        
        double common_delay = std::min(launch_ins, capture_ins);
        if (common_delay <= 0) continue;
        
        double credit = common_delay * std::abs(late_factor - early_factor);
        path.cppr_credit = credit;
        
        if (!path.is_hold) {
            path.slack += credit;
        } else {
            path.slack -= credit;
        }
    }
}

// === Industrial STA: POCV Path Sigma ===

double StaEngine::compute_path_pocv_sigma(const TimingPath& path) const {
    if (ocv_mode_ != OcvMode::POCV) return 0;
    
    double sum_sigma_sq = 0;
    
    for (auto gid : path.gates) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT || 
            g.type == GateType::OUTPUT) continue;
        
        std::string type_name = gate_type_str(g.type);
        double sigma_frac = pocv_table_.get_sigma(type_name);
        
        double gd = gate_delay(gid, 0.01) / (analyzing_late_ ? derate_.cell_derate : derate_.early_cell);
        double sigma_i = gd * sigma_frac;
        sum_sigma_sq += sigma_i * sigma_i;
    }
    
    return std::sqrt(sum_sigma_sq);
}

// === Industrial STA: PBA Re-analysis ===

void StaEngine::pba_reanalyze(std::vector<TimingPath>& paths, double clock_period) {
    if (!pba_enabled_) return;
    
    for (auto& path : paths) {
        if (path.nets.size() < 2) continue;
        
        double path_delay = 0;
        double current_slew = 0.01;
        
        for (size_t i = 0; i < path.gates.size(); i++) {
            GateId gid = path.gates[i];
            auto& g = nl_.gate(gid);
            
            double gd = gate_delay(gid, current_slew);
            
            NetId from_net = (i < path.nets.size()) ? path.nets[i] : -1;
            NetId to_net = (i + 1 < path.nets.size()) ? path.nets[i + 1] : g.output;
            double wd = (from_net >= 0 && to_net >= 0) ? wire_delay(from_net, to_net) : 0;
            
            double xt_delta = 0;
            if (xtalk_.enabled && to_net >= 0) {
                xt_delta = compute_crosstalk_delta(to_net);
            }
            
            path_delay += gd + wd + xt_delta;
            
            double load = (g.output >= 0) ? net_load_cap(g.output) : 0.002;
            current_slew = output_slew(gid, current_slew, load);
        }
        
        if (ocv_mode_ == OcvMode::POCV) {
            double path_sigma = compute_path_pocv_sigma(path);
            path.path_sigma = path_sigma;
            if (analyzing_late_) {
                path_delay += pocv_table_.n_sigma * path_sigma;
            } else {
                path_delay -= pocv_table_.n_sigma * path_sigma;
            }
        }
        
        path.pba_delay = path_delay;
        if (!path.is_hold) {
            double req = clock_period;
            if (!path.nets.empty()) {
                NetId ep = path.nets.back();
                if (pin_timing_.count(ep)) {
                    req = std::min(pin_timing_.at(ep).required_rise, 
                                   pin_timing_.at(ep).required_fall);
                }
            }
            path.pba_slack = req - path_delay;
        } else {
            double hold_req = 0.02;
            if (!path.nets.empty()) {
                NetId ep = path.nets.back();
                if (pin_timing_.count(ep)) {
                    hold_req = pin_timing_.at(ep).hold_required_rise;
                }
            }
            path.pba_slack = path_delay - hold_req;
        }
        path.pba_valid = true;
    }
}

// === Industrial STA: Crosstalk Delta-Delay ===

double StaEngine::compute_crosstalk_delta(NetId net) const {
    if (!xtalk_.enabled || !pd_ || pd_->wires.empty()) return 0;
    
    double total_delta = 0;
    
    for (size_t i = 0; i < pd_->wires.size(); i++) {
        auto& victim = pd_->wires[i];
        if (victim.net_id != static_cast<int>(net) && victim.net_id >= 0) continue;
        
        double victim_len = victim.start.dist(victim.end);
        if (victim_len < 0.001) continue;
        
        for (size_t j = 0; j < pd_->wires.size(); j++) {
            if (i == j) continue;
            auto& aggressor = pd_->wires[j];
            if (aggressor.layer != victim.layer) continue;
            if (aggressor.net_id == victim.net_id) continue;
            
            double dx = (aggressor.start.x + aggressor.end.x) / 2.0 - 
                        (victim.start.x + victim.end.x) / 2.0;
            double dy = (aggressor.start.y + aggressor.end.y) / 2.0 -
                        (victim.start.y + victim.end.y) / 2.0;
            double spacing = std::sqrt(dx * dx + dy * dy);
            
            if (spacing > xtalk_.max_coupling_distance_um) continue;
            if (spacing < 0.001) spacing = xtalk_.min_spacing_um;
            
            double agg_len = aggressor.start.dist(aggressor.end);
            double prl = std::min(victim_len, agg_len);
            
            double spacing_factor = spacing / xtalk_.min_spacing_um;
            double cc = xtalk_.coupling_cap_per_um * prl / std::max(1.0, spacing_factor);
            
            double delta = cc * xtalk_.miller_factor / std::max(0.01, xtalk_.aggressor_slew);
            total_delta += delta;
        }
        break; // Only process first matching wire (simplification)
    }
    
    return total_delta;
}

} // namespace sf
