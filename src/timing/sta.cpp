// SiliconForge — Static Timing Analysis Implementation
// Slew-aware delay, hold checks, multi-corner derating, OCV/AOCV.
#include "timing/sta.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <cmath>
#include <functional>
#include <unordered_set>

namespace sf {

// === OCV/AOCV Effective Derate ===

double StaEngine::effective_cell_derate(GateId gid) const {
    // Base OCV/AOCV/POCV derate
    double base_derate;
    if (ocv_mode_ == OcvMode::AOCV) {
        int depth = 1;
        auto it = gate_depth_.find(gid);
        if (it != gate_depth_.end()) depth = std::max(1, it->second);

        // Use per-cell AOCV coefficient if available
        std::string cell_type;
        if (gid >= 0 && static_cast<size_t>(gid) < nl_.num_gates()) {
            cell_type = gate_type_str(nl_.gate(gid).type);
        }

        if (analyzing_late_)
            base_derate = aocv_table_.late_derate(depth, cell_type) * derate_.cell_derate * derate_.tv_scale();
        else
            base_derate = aocv_table_.early_derate(depth, cell_type) * derate_.early_cell * derate_.tv_scale();
    } else if (ocv_mode_ == OcvMode::POCV) {
        base_derate = (analyzing_late_ ? derate_.cell_derate : derate_.early_cell) * derate_.tv_scale();
    } else if (ocv_mode_ == OcvMode::OCV) {
        if (analyzing_late_)
            base_derate = derate_.cell_derate * ocv_late_cell_ * derate_.tv_scale();
        else
            base_derate = derate_.early_cell * ocv_early_cell_ * derate_.tv_scale();
    } else {
        base_derate = (analyzing_late_ ? derate_.cell_derate : derate_.early_cell) * derate_.tv_scale();
    }

    // Apply IR drop voltage derating: delay_scaled = delay * (V_nom / V_actual)^alpha
    if (!ir_drop_voltage_map_.empty() && gid >= 0 &&
        static_cast<size_t>(gid) < nl_.num_gates()) {
        auto vit = ir_drop_voltage_map_.find(nl_.gate(gid).name);
        if (vit != ir_drop_voltage_map_.end() && vit->second > 0) {
            double v_ratio = ir_drop_nominal_v_ / vit->second;
            base_derate *= std::pow(v_ratio, ir_drop_alpha_);
        }
    }

    return base_derate;
}

double StaEngine::effective_wire_derate(GateId gid) const {
    if (ocv_mode_ == OcvMode::AOCV) {
        int depth = 1;
        auto it = gate_depth_.find(gid);
        if (it != gate_depth_.end()) depth = std::max(1, it->second);

        if (analyzing_late_)
            return (1.0 + aocv_table_.late_variation / std::sqrt((double)depth))
                   * derate_.wire_derate * derate_.tv_scale();
        else
            return (1.0 - aocv_table_.early_variation / std::sqrt((double)depth))
                   * derate_.early_wire * derate_.tv_scale();
    }
    if (ocv_mode_ == OcvMode::POCV) {
        return (analyzing_late_ ? derate_.wire_derate : derate_.early_wire) * derate_.tv_scale();
    }
    if (ocv_mode_ == OcvMode::OCV) {
        if (analyzing_late_)
            return derate_.wire_derate * ocv_late_cell_ * derate_.tv_scale();
        else
            return derate_.early_wire * ocv_early_cell_ * derate_.tv_scale();
    }
    return (analyzing_late_ ? derate_.wire_derate : derate_.early_wire) * derate_.tv_scale();
}

// === IR Drop Voltage Derating ===

void StaEngine::apply_ir_drop_derating(
    const std::unordered_map<std::string, double>& cell_voltage_map) {
    ir_drop_voltage_map_ = cell_voltage_map;
    // Infer nominal voltage from corner derate if available, otherwise use stored default
    if (derate_.ref_voltage > 0)
        ir_drop_nominal_v_ = derate_.ref_voltage;
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
    // Latch outputs are also depth-0 sources
    if (latch_timing_enabled_) {
        for (auto gid : find_latches()) {
            auto& latch = nl_.gate(gid);
            if (latch.output >= 0) net_depth[latch.output] = 0;
        }
    }

    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::DLATCH ||
            g.type == GateType::INPUT ||
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
                        return std::max(d_rise, d_fall) * eff_derate * voltage_delay_scale(gid);
                    }
                    double d = (t.cell_rise + t.cell_fall) / 2.0;
                    if (d > 0) return d * slew_factor * eff_derate * voltage_delay_scale(gid);
                }
                if (cell->area > 0) return cell->area * 0.01 * slew_factor * eff_derate * voltage_delay_scale(gid);
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
        case GateType::DLATCH: base = 0.08; break;
        default:               base = 0.05; break;
    }
    return base * slew_factor * eff_derate * voltage_delay_scale(gid);
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
        if (g.type == GateType::DFF || g.type == GateType::DLATCH ||
            g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double in_slew = 0.01;
        double gd = gate_delay(gid, in_slew);
        for (auto ni : g.inputs) {
            double wd = wire_delay(ni, g.output);

            // Level-shifter penalty when crossing voltage domains
            double ls_penalty = 0;
            if (!voltage_domains_.empty() && ni >= 0 && ni < (NetId)nl_.num_nets()) {
                GateId drv = nl_.net(ni).driver;
                if (drv >= 0) ls_penalty = level_shifter_penalty(drv, gid);
            }

            arcs_.push_back({ni, g.output, gd + wd + ls_penalty, in_slew, gid});
        }
    }

    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0)
            arcs_.push_back({ff.clk, ff.output, gate_delay(gid), 0.01, gid});
    }

    // Build arcs for latches (analogous to DFFs but with enable-based timing)
    if (latch_timing_enabled_) {
        for (auto gid : find_latches()) {
            auto& latch = nl_.gate(gid);
            if (latch.output >= 0)
                arcs_.push_back({latch.clk, latch.output, gate_delay(gid), 0.01, gid});
        }
    }
}

// === Forward Propagation with Slew (LATE paths for setup) ===

void StaEngine::forward_propagation(double input_arrival) {
    analyzing_late_ = true;
    pin_timing_.clear();

    double pi_slew = 0.01;
    for (auto pi : nl_.primary_inputs()) {
        auto& pt = pin_timing_[pi];
        double base_arr = input_arrival;
        // Apply SDC input delays
        if (sdc_) {
            for (auto& id : sdc_->input_delays) {
                if (nl_.net(pi).name == id.port && id.is_max) {
                    base_arr = std::max(base_arr, id.delay_ns);
                }
            }
        }
        pt.arrival_rise = base_arr;
        pt.arrival_fall = base_arr;
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

    // Latch outputs: data passes through when enable is active (time borrowing)
    if (latch_timing_enabled_) {
        for (auto gid : find_latches()) {
            auto& latch = nl_.gate(gid);
            if (latch.output >= 0) {
                auto info = compute_latch_edges(gid, last_clock_period_);
                double dq = gate_delay(gid, pi_slew);
                double insertion = 0;
                auto ci = clock_insertion_.find(gid);
                if (ci != clock_insertion_.end()) insertion = ci->second;
                auto& pt = pin_timing_[latch.output];
                // Latch output arrival = opening_edge + D-to-Q delay
                pt.arrival_rise = insertion + info.opening_edge + dq;
                pt.arrival_fall = insertion + info.opening_edge + dq;
                double load = net_load_cap(latch.output);
                pt.slew_rise = output_slew(gid, pi_slew, load);
                pt.slew_fall = pt.slew_rise;
            }
        }
    }

    for (size_t i = 0; i < nl_.num_nets(); ++i)
        if (!pin_timing_.count(i)) pin_timing_[i] = {};

    // Propagate LATE (latest) arrivals for setup analysis
    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::DLATCH ||
            g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double max_arr = 0;
        double worst_slew = pi_slew;
        GateId worst_driver = -1;
        for (auto ni : g.inputs) {
            double arr = pin_timing_[ni].worst_arrival();
            if (arr > max_arr) {
                max_arr = arr;
                worst_slew = std::max(pin_timing_[ni].slew_rise, pin_timing_[ni].slew_fall);
                worst_driver = nl_.net(ni).driver;
            }
        }

        double gd = gate_delay(gid, worst_slew);
        double wd = wire_delay(g.inputs.empty() ? -1 : g.inputs[0], g.output);

        // Level-shifter penalty when crossing voltage domains
        double ls_penalty = 0;
        if (!voltage_domains_.empty() && worst_driver >= 0)
            ls_penalty = level_shifter_penalty(worst_driver, gid);

        double out_arr = max_arr + gd + wd + ls_penalty;

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

    // Latch hold arrivals
    if (latch_timing_enabled_) {
        for (auto gid : find_latches()) {
            auto& latch = nl_.gate(gid);
            if (latch.output >= 0) {
                auto info = compute_latch_edges(gid, last_clock_period_);
                double dq_early = gate_delay(gid, pi_slew);
                auto& pt = pin_timing_[latch.output];
                pt.hold_arrival_rise = info.opening_edge + dq_early;
                pt.hold_arrival_fall = info.opening_edge + dq_early;
            }
        }
    }

    // Propagate MINIMUM (earliest) arrivals
    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::DLATCH ||
            g.type == GateType::INPUT ||
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
        double req = clock_period;
        // Apply SDC output delays: reduces required time at output
        if (sdc_) {
            for (auto& od : sdc_->output_delays) {
                if (nl_.net(po).name == od.port && od.is_max) {
                    req = std::min(req, clock_period - od.delay_ns);
                }
            }
        }
        pin_timing_[po].required_rise = req;
        pin_timing_[po].required_fall = req;
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

    // Latch setup: data must arrive before the closing edge minus setup time
    if (latch_timing_enabled_) {
        for (auto gid : find_latches()) {
            auto& latch = nl_.gate(gid);
            if (!latch.inputs.empty()) {
                NetId d = latch.inputs[0];
                auto info = compute_latch_edges(gid, clock_period);
                // Setup check: data_arrival + setup_time <= closing_edge
                // → required_time = closing_edge - setup_time
                double req = info.closing_edge - setup_margin;
                pin_timing_[d].required_rise = std::min(pin_timing_[d].required_rise, req);
                pin_timing_[d].required_fall = std::min(pin_timing_[d].required_fall, req);
            }
        }
    }

    for (int i = (int)topo_.size() - 1; i >= 0; --i) {
        auto& g = nl_.gate(topo_[i]);
        if (g.type == GateType::DFF || g.type == GateType::DLATCH ||
            g.type == GateType::INPUT ||
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

    // Latch hold: data_arrival >= opening_edge + hold_time
    // → hold_required = opening_edge + hold_margin
    if (latch_timing_enabled_) {
        for (auto gid : find_latches()) {
            auto& latch = nl_.gate(gid);
            if (!latch.inputs.empty()) {
                NetId d = latch.inputs[0];
                auto info = compute_latch_edges(gid, last_clock_period_);
                double clk_derate = 1.0;
                if (ocv_mode_ == OcvMode::OCV)
                    clk_derate = ocv_late_cell_;
                else if (ocv_mode_ == OcvMode::AOCV)
                    clk_derate = aocv_table_.late_derate(1);
                double req = (info.opening_edge + hold_margin) * clk_derate;
                pin_timing_[d].hold_required_rise = req;
                pin_timing_[d].hold_required_fall = req;
            }
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

    // Helper: check if a path from 'start' to 'end' is a false path per SDC
    auto is_false_path = [&](const std::string& start, const std::string& end) -> bool {
        if (!sdc_) return false;
        for (auto& exc : sdc_->exceptions) {
            if (exc.type == SdcException::FALSE_PATH) {
                bool from_match = exc.from.empty() || start.find(exc.from) != std::string::npos;
                bool to_match = exc.to.empty() || end.find(exc.to) != std::string::npos;
                if (from_match && to_match) return true;
            }
        }
        return false;
    };

    // Helper: get multicycle path multiplier
    auto get_multicycle = [&](const std::string& start, const std::string& end) -> int {
        if (!sdc_) return 1;
        for (auto& exc : sdc_->exceptions) {
            if (exc.type == SdcException::MULTICYCLE_PATH && exc.is_setup) {
                bool from_match = exc.from.empty() || start.find(exc.from) != std::string::npos;
                bool to_match = exc.to.empty() || end.find(exc.to) != std::string::npos;
                if (from_match && to_match) return exc.multiplier;
            }
        }
        return 1;
    };

    // Helper: get max_delay constraint for a path (returns -1 if none)
    auto get_max_delay = [&](const std::string& start, const std::string& end) -> double {
        if (!sdc_) return -1;
        for (auto& exc : sdc_->exceptions) {
            if (exc.type == SdcException::MAX_DELAY) {
                bool from_match = exc.from.empty() || start.find(exc.from) != std::string::npos;
                bool to_match = exc.to.empty() || end.find(exc.to) != std::string::npos;
                if (from_match && to_match) return exc.value;
            }
        }
        return -1;
    };

    // Helper: get min_delay constraint for a path (returns -1 if none)
    auto get_min_delay = [&](const std::string& start, const std::string& end) -> double {
        if (!sdc_) return -1;
        for (auto& exc : sdc_->exceptions) {
            if (exc.type == SdcException::MIN_DELAY) {
                bool from_match = exc.from.empty() || start.find(exc.from) != std::string::npos;
                bool to_match = exc.to.empty() || end.find(exc.to) != std::string::npos;
                if (from_match && to_match) return exc.value;
            }
        }
        return -1;
    };

    struct EndpointSlack {
        NetId net; double slack; std::string name; bool is_hold;
        GateId latch_id = -1;  // >=0 if endpoint is a latch
    };
    std::vector<EndpointSlack> endpoints;

    for (auto po : nl_.primary_outputs()) {
        auto& pt = pin_timing_[po];
        endpoints.push_back({po, pt.worst_slack(), nl_.net(po).name, false, -1});
    }
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) {
            NetId d = ff.inputs[0];
            auto& pt = pin_timing_[d];
            endpoints.push_back({d, pt.worst_slack(), ff.name + "/D", false, -1});
        }
    }

    // Latch endpoints (setup)
    if (latch_timing_enabled_) {
        for (auto gid : find_latches()) {
            auto& latch = nl_.gate(gid);
            if (!latch.inputs.empty()) {
                NetId d = latch.inputs[0];
                auto& pt = pin_timing_[d];
                endpoints.push_back({d, pt.worst_slack(), latch.name + "/D", false, gid});
            }
        }
    }

    if (include_hold) {
        for (auto gid : nl_.flip_flops()) {
            auto& ff = nl_.gate(gid);
            if (!ff.inputs.empty()) {
                NetId d = ff.inputs[0];
                auto& pt = pin_timing_[d];
                endpoints.push_back({d, pt.worst_hold_slack(), ff.name + "/D(hold)", true, -1});
            }
        }
        // Latch hold endpoints
        if (latch_timing_enabled_) {
            for (auto gid : find_latches()) {
                auto& latch = nl_.gate(gid);
                if (!latch.inputs.empty()) {
                    NetId d = latch.inputs[0];
                    auto& pt = pin_timing_[d];
                    endpoints.push_back({d, pt.worst_hold_slack(),
                                         latch.name + "/D(hold)", true, gid});
                }
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
            if (g.type == GateType::DFF || g.type == GateType::DLATCH ||
                g.type == GateType::INPUT) {
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

        // SDC: skip false paths
        if (is_false_path(path.startpoint, path.endpoint)) continue;

        // SDC: adjust slack for multicycle paths using actual clock period
        int mc = get_multicycle(path.startpoint, path.endpoint);
        if (mc > 1 && !path.is_hold) {
            // Multicycle relaxes required time by (mc-1) clock periods
            double clock_p = last_clock_period_;
            path.slack += (mc - 1) * clock_p;
        }

        // SDC: enforce set_max_delay — override slack based on explicit constraint
        double max_d = get_max_delay(path.startpoint, path.endpoint);
        if (max_d >= 0 && !path.is_hold) {
            // Slack = max_delay_constraint - actual_delay
            path.slack = max_d - path.delay;
        }

        // SDC: enforce set_min_delay — for hold paths
        double min_d = get_min_delay(path.startpoint, path.endpoint);
        if (min_d >= 0 && path.is_hold) {
            // Hold slack = actual_delay - min_delay_constraint
            path.slack = path.delay - min_d;
        }

        // Latch time borrowing: if endpoint is a latch, compute borrowed time
        GateId ep_latch = endpoints[p].latch_id;
        if (latch_timing_enabled_ && ep_latch >= 0 && !path.is_hold) {
            path.is_latch_path = true;
            auto info = compute_latch_edges(ep_latch, last_clock_period_);
            double data_arrival = pin_timing_[endpoints[p].net].worst_arrival();
            // Time borrowing: if data arrives before closing edge, surplus
            // can be lent to the downstream path
            double surplus = info.closing_edge - data_arrival;
            if (surplus > 0) {
                double borrowed = std::min(surplus, info.borrow_limit);
                info.borrowed_time = borrowed;
                path.slack += borrowed;
            }
            path.latch_info = info;
        }

        paths.push_back(path);
    }

    return paths;
}

// === Single-Corner Analyze ===

StaResult StaEngine::analyze_corner(double clock_period, int num_paths, const CornerDerate& d) {
    derate_ = d;
    last_clock_period_ = clock_period;
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
    // Include latch endpoints
    if (latch_timing_enabled_) {
        for (auto gid : find_latches()) {
            auto& latch = nl_.gate(gid);
            if (!latch.inputs.empty()) check_endpoint(latch.inputs[0]);
        }
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
    if (ocv_mode_ == OcvMode::NONE) return;

    // Helper: trace clock path from a DFF's clk pin back through BUF/NOT
    // to the clock root. Returns the gate sequence (deepest first).
    auto trace_clock_path = [&](GateId dff_id) -> std::vector<GateId> {
        std::vector<GateId> clk_gates;
        auto& ff = nl_.gate(dff_id);
        if (ff.clk < 0) return clk_gates;

        NetId cur_net = ff.clk;
        std::unordered_set<int> visited;
        while (true) {
            if (visited.count(cur_net)) break;
            visited.insert(cur_net);
            GateId drv = nl_.net(cur_net).driver;
            if (drv < 0) break;
            auto& g = nl_.gate(drv);
            if (g.type == GateType::INPUT || g.type == GateType::DFF) break;
            if (g.type != GateType::BUF && g.type != GateType::NOT) break;
            clk_gates.push_back(drv);
            if (g.inputs.empty()) break;
            cur_net = g.inputs[0];
        }
        std::reverse(clk_gates.begin(), clk_gates.end());
        return clk_gates;
    };

    for (auto& path : paths) {
        // Identify launch and capture flip-flops
        GateId launch_ff = -1, capture_ff = -1;

        std::string cap_name = path.endpoint;
        auto slash_pos = cap_name.find('/');
        if (slash_pos != std::string::npos) cap_name = cap_name.substr(0, slash_pos);

        for (auto gid : nl_.flip_flops()) {
            auto& ff = nl_.gate(gid);
            if (launch_ff < 0 &&
                (ff.name == path.startpoint || ff.name + "/Q" == path.startpoint))
                launch_ff = gid;
            if (capture_ff < 0 && ff.name == cap_name)
                capture_ff = gid;
            if (launch_ff >= 0 && capture_ff >= 0) break;
        }

        if (launch_ff < 0 || capture_ff < 0) continue;

        // Trace clock tree paths for both FFs
        auto launch_clk = trace_clock_path(launch_ff);
        auto capture_clk = trace_clock_path(capture_ff);

        // Find deepest common prefix (gates shared by both clock paths)
        size_t common_len = 0;
        size_t min_len = std::min(launch_clk.size(), capture_clk.size());
        for (size_t i = 0; i < min_len; ++i) {
            if (launch_clk[i] == capture_clk[i])
                common_len = i + 1;
            else
                break;
        }

        double credit = 0;
        if (common_len > 0) {
            // Per-gate CPPR: sum |late_delay - early_delay| for each common gate
            for (size_t i = 0; i < common_len; ++i) {
                GateId gid = launch_clk[i];
                double in_slew = 0.01;
                int depth = 1;
                auto dit = gate_depth_.find(gid);
                if (dit != gate_depth_.end()) depth = std::max(1, dit->second);

                double late_derate = 1.0, early_derate = 1.0;
                if (ocv_mode_ == OcvMode::OCV) {
                    late_derate = ocv_late_cell_;
                    early_derate = ocv_early_cell_;
                } else if (ocv_mode_ == OcvMode::AOCV) {
                    std::string ctype = gate_type_str(nl_.gate(gid).type);
                    late_derate = aocv_table_.late_derate(depth, ctype);
                    early_derate = aocv_table_.early_derate(depth, ctype);
                } else if (ocv_mode_ == OcvMode::POCV) {
                    std::string ctype = gate_type_str(nl_.gate(gid).type);
                    double sigma = pocv_table_.get_sigma(ctype);
                    late_derate = 1.0 + sigma * pocv_table_.n_sigma;
                    early_derate = 1.0 - sigma * pocv_table_.n_sigma;
                }

                // Compute nominal gate delay (without OCV) for the common gate
                bool saved = analyzing_late_;
                analyzing_late_ = true;
                double base_delay = gate_delay(gid, in_slew);
                // Undo derate to get nominal
                double nom_delay = base_delay / late_derate;
                analyzing_late_ = saved;

                // Wire delay contribution on the common path
                auto& g = nl_.gate(gid);
                double wd_nom = 0;
                if (!g.inputs.empty() && g.output >= 0) {
                    double wd = wire_delay(g.inputs[0], g.output);
                    double wire_late = 1.0, wire_early = 1.0;
                    if (ocv_mode_ == OcvMode::OCV) {
                        wire_late = ocv_late_cell_;
                        wire_early = ocv_early_cell_;
                    }
                    wd_nom = wd / (analyzing_late_ ? wire_late : wire_early);
                    credit += wd_nom * std::abs(wire_late - wire_early);
                }

                credit += nom_delay * std::abs(late_derate - early_derate);
            }
        } else {
            // Fallback: use clock insertion delays for common-path estimate
            double launch_ins = 0, capture_ins = 0;
            auto lci = clock_insertion_.find(launch_ff);
            if (lci != clock_insertion_.end()) launch_ins = lci->second;
            auto cci = clock_insertion_.find(capture_ff);
            if (cci != clock_insertion_.end()) capture_ins = cci->second;

            double common_delay = std::min(launch_ins, capture_ins);
            if (common_delay > 0) {
                double late_f = 1.0, early_f = 1.0;
                if (ocv_mode_ == OcvMode::OCV) {
                    late_f = ocv_late_cell_;
                    early_f = ocv_early_cell_;
                } else if (ocv_mode_ == OcvMode::AOCV) {
                    late_f = aocv_table_.late_derate(1);
                    early_f = aocv_table_.early_derate(1);
                } else if (ocv_mode_ == OcvMode::POCV) {
                    late_f = 1.0 + pocv_table_.default_sigma_pct * pocv_table_.n_sigma;
                    early_f = 1.0 - pocv_table_.default_sigma_pct * pocv_table_.n_sigma;
                }
                credit = common_delay * std::abs(late_f - early_f);
            }
        }

        path.cppr_credit = credit;
        if (!path.is_hold)
            path.slack += credit;
        else
            path.slack -= credit;
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

    // Build a set of nets on each path for path-specific load computation
    for (auto& path : paths) {
        if (path.nets.size() < 2) continue;

        // Collect nets that are actually on this path for load filtering
        std::unordered_set<NetId> path_net_set(path.nets.begin(), path.nets.end());

        double path_delay = 0;
        double current_slew = 0.01;

        for (size_t i = 0; i < path.gates.size(); i++) {
            GateId gid = path.gates[i];
            auto& g = nl_.gate(gid);

            // Path-specific load: only count fanout gates whose input nets
            // are on this path, plus a small fixed off-path capacitance for
            // branches not on the critical path.
            double path_load = 0.001; // base wire cap
            if (g.output >= 0) {
                auto& out_net = nl_.net(g.output);
                for (auto fo_gid : out_net.fanout) {
                    auto& fo_gate = nl_.gate(fo_gid);
                    bool on_path = false;
                    // Check if this fanout gate is on the current path
                    if (i + 1 < path.gates.size() &&
                        fo_gid == path.gates[i + 1]) {
                        on_path = true;
                    }
                    // Also check if the fanout gate's output net is a path net
                    if (!on_path && fo_gate.output >= 0 &&
                        path_net_set.count(fo_gate.output)) {
                        on_path = true;
                    }

                    double pin_cap = 0.002; // default
                    if (lib_) {
                        std::string type_str = gate_type_str(fo_gate.type);
                        int num_in = (int)fo_gate.inputs.size();
                        std::string candidates[] = {
                            type_str + std::to_string(num_in), type_str,
                            type_str + "_X1"
                        };
                        for (auto& cand : candidates) {
                            if (auto* cell = lib_->find_cell(cand)) {
                                for (auto& pin : cell->pins) {
                                    if (pin.direction == "input" &&
                                        pin.capacitance > 0) {
                                        pin_cap = pin.capacitance;
                                        break;
                                    }
                                }
                                break;
                            }
                        }
                    }
                    if (on_path)
                        path_load += pin_cap;
                    else
                        path_load += pin_cap * 0.3; // off-path sees reduced effective cap
                }
            }

            // Recompute gate delay with exact per-path input slew
            double gd = gate_delay(gid, current_slew);

            NetId from_net = (i < path.nets.size()) ? path.nets[i] : -1;
            NetId to_net = (i + 1 < path.nets.size()) ? path.nets[i + 1] : g.output;
            double wd = (from_net >= 0 && to_net >= 0) ? wire_delay(from_net, to_net) : 0;

            double xt_delta = 0;
            if (xtalk_.enabled && to_net >= 0)
                xt_delta = compute_crosstalk_delta(to_net);

            path_delay += gd + wd + xt_delta;

            // Propagate slew using path-specific load
            current_slew = output_slew(gid, current_slew, path_load);
        }

        // POCV: apply path-level statistical adjustment
        if (ocv_mode_ == OcvMode::POCV) {
            double path_sigma = compute_path_pocv_sigma(path);
            path.path_sigma = path_sigma;
            if (analyzing_late_)
                path_delay += pocv_table_.n_sigma * path_sigma;
            else
                path_delay -= pocv_table_.n_sigma * path_sigma;
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
                if (pin_timing_.count(ep))
                    hold_req = pin_timing_.at(ep).hold_required_rise;
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

// === Multi-Clock Domain STA ===

MultiClockStaResult run_multi_clock_sta(
    const Netlist& nl,
    const LibertyLibrary* lib,
    const PhysicalDesign* pd,
    double clock_period,
    int num_paths,
    const SdcConstraints* sdc) {

    MultiClockStaResult result;

    // Step 1: Detect clock domains by tracing DFF clk pins back to sources
    // Map: clock source net → domain name
    std::unordered_map<int, std::string> clk_net_to_domain;
    // Map: DFF gate_id → domain name
    std::unordered_map<int, std::string> dff_domain;

    for (auto gid : nl.flip_flops()) {
        auto& ff = nl.gate(gid);
        if (ff.clk < 0) {
            dff_domain[gid] = "__no_clk__";
            continue;
        }

        // Trace clock net back to PI or buffer tree root
        int clk_net = ff.clk;
        std::unordered_set<int> visited;
        int root = clk_net;

        while (true) {
            if (visited.count(root)) break;
            visited.insert(root);
            auto& net = nl.net(root);
            if (net.driver < 0) break;
            auto& drv = nl.gate(net.driver);
            // Stop at PIs, DFFs (generated clock), or combinational with multiple inputs
            if (drv.type == GateType::INPUT || drv.type == GateType::DFF) break;
            if (drv.inputs.empty()) break;
            // Follow through BUF/NOT (clock tree buffers)
            if (drv.type == GateType::BUF || drv.type == GateType::NOT) {
                root = drv.inputs[0];
            } else {
                // Stop at complex logic
                break;
            }
        }

        auto it = clk_net_to_domain.find(root);
        if (it == clk_net_to_domain.end()) {
            std::string name = nl.net(root).name;
            if (name.empty()) name = "clk_" + std::to_string(root);
            clk_net_to_domain[root] = name;
        }
        dff_domain[gid] = clk_net_to_domain[root];
    }

    // Step 2: Detect generated clocks (divide-by-N)
    // A DFF whose output feeds back to clock inputs through inverter/logic
    // forms a divide-by-2 clock
    std::unordered_map<std::string, int> domain_divide_ratio;
    for (auto& [root_net, domain_name] : clk_net_to_domain) {
        int divide = 1;
        auto& net = nl.net(root_net);
        if (net.driver >= 0) {
            auto& drv = nl.gate(net.driver);
            if (drv.type == GateType::DFF) {
                // This clock is generated from a DFF output → divide-by-2
                divide = 2;
                // Check for chain: if the driving DFF's clk comes from another DFF → /4
                if (drv.clk >= 0 && nl.net(drv.clk).driver >= 0) {
                    auto& clk_drv = nl.gate(nl.net(drv.clk).driver);
                    if (clk_drv.type == GateType::DFF) divide = 4;
                }
            }
        }
        domain_divide_ratio[domain_name] = divide;
    }

    // Step 3: Group DFFs by clock domain and build domain info
    std::unordered_map<std::string, ClockDomainInfo> domain_map;
    for (auto& [gid, dom] : dff_domain) {
        if (dom == "__no_clk__") continue;
        auto& info = domain_map[dom];
        info.name = dom;
        info.num_flops++;
        auto dit = domain_divide_ratio.find(dom);
        if (dit != domain_divide_ratio.end() && dit->second > 1) {
            info.is_generated = true;
            info.divide_ratio = dit->second;
        }
    }

    // Overlay SDC generated clock definitions (more accurate than auto-detection)
    if (sdc) {
        for (auto& sdc_clk : sdc->clocks) {
            if (!sdc_clk.is_generated) continue;
            auto it = domain_map.find(sdc_clk.name);
            if (it != domain_map.end()) {
                it->second.is_generated = true;
                it->second.divide_ratio = sdc_clk.divide_by;
            }
        }
    }

    // Step 4: Run STA and partition paths by domain
    StaEngine sta(nl, lib, pd);
    if (sdc) sta.set_sdc_constraints(*sdc);
    auto sta_result = sta.analyze(clock_period, num_paths * 2);

    // For each timing path, determine its source and destination domains
    for (auto& path : sta_result.critical_paths) {
        std::string src_dom = "__unknown__";
        std::string dst_dom = "__unknown__";

        // Source domain: find the DFF that starts the path
        for (auto& [gid, dom] : dff_domain) {
            auto& ff = nl.gate(gid);
            if (ff.name == path.startpoint) {
                src_dom = dom;
                break;
            }
        }
        // Check if startpoint is a PI
        if (src_dom == "__unknown__") {
            for (auto pi : nl.primary_inputs()) {
                if (nl.net(pi).name == path.startpoint) {
                    src_dom = "__input__";
                    break;
                }
            }
        }

        // Destination domain: find the DFF at endpoint
        std::string ep = path.endpoint;
        auto slash = ep.find('/');
        if (slash != std::string::npos) ep = ep.substr(0, slash);

        for (auto& [gid, dom] : dff_domain) {
            auto& ff = nl.gate(gid);
            if (ff.name == ep) {
                dst_dom = dom;
                break;
            }
        }

        if (src_dom == dst_dom && src_dom != "__unknown__" && src_dom != "__input__") {
            // Intra-domain path — attribute to domain
            auto& info = domain_map[src_dom];
            if (path.slack < 0) {
                info.violations++;
                info.tns += path.slack;
                info.wns = std::min(info.wns, path.slack);
            }
        } else if (src_dom != "__unknown__" && dst_dom != "__unknown__") {
            // Inter-domain crossing
            InterClockPath icp;
            icp.src_domain = src_dom;
            icp.dst_domain = dst_dom;
            icp.slack = path.slack;
            // Check SDC clock groups for async/exclusive classification
            icp.is_async = true; // default to async unless SDC says otherwise
            if (sdc) {
                for (auto& grp : sdc->clock_groups) {
                    // If src and dst are in the same sub-group, they're synchronous
                    for (auto& sub : grp.groups) {
                        bool src_in = false, dst_in = false;
                        for (auto& gclk : sub) {
                            if (gclk == src_dom) src_in = true;
                            if (gclk == dst_dom) dst_in = true;
                        }
                        if (src_in && dst_in) {
                            icp.is_async = false;
                            break;
                        }
                    }
                    if (!icp.is_async) break;
                }
            }
            icp.endpoint = path.endpoint;
            result.inter_domain_paths.push_back(icp);
        }
    }

    // Step 5: Build final result
    for (auto& [name, info] : domain_map)
        result.domains.push_back(info);

    result.total_domains = (int)domain_map.size();
    result.async_crossings = (int)result.inter_domain_paths.size();

    result.worst_inter_domain_slack = 0;
    for (auto& icp : result.inter_domain_paths)
        result.worst_inter_domain_slack = std::min(result.worst_inter_domain_slack, icp.slack);

    result.message = "Multi-clock STA: " + std::to_string(result.total_domains) +
                     " domains, " + std::to_string(result.async_crossings) +
                     " inter-domain crossings";
    for (auto& d : result.domains) {
        if (d.is_generated)
            result.message += "\n  [" + d.name + "] generated (/" +
                              std::to_string(d.divide_ratio) + ")";
    }

    return result;
}

// ============================================================================
// RC Corner Support
// ============================================================================

void StaEngine::add_rc_corner(const RcCorner& corner) {
    rc_corners_.push_back(corner);
    if (active_corner_.empty())
        active_corner_ = corner.name;
}

void StaEngine::set_active_corner(const std::string& name) {
    for (auto& rc : rc_corners_) {
        if (rc.name == name) {
            active_corner_ = name;
            return;
        }
    }
}

// ============================================================================
// External Delay Annotation
// ============================================================================

void StaEngine::set_external_delays(const std::vector<ExternalDelay>& delays) {
    external_delays_ = delays;
}

// ============================================================================
// PBA Helpers
// ============================================================================

StaEngine::PbaResult::PathDetail StaEngine::trace_path(
    int endpoint, const std::vector<double>& arrival)
{
    PbaResult::PathDetail detail;

    // Back-trace from endpoint to startpoint through worst-arrival inputs
    std::vector<int> gate_seq;
    std::vector<NetId> net_seq;
    NetId curr = static_cast<NetId>(endpoint);
    net_seq.push_back(curr);
    for (int depth = 0; depth < 200; ++depth) {
        GateId drv = nl_.net(curr).driver;
        if (drv < 0) break;
        auto& g = nl_.gate(drv);
        if (g.type == GateType::DFF || g.type == GateType::INPUT) break;
        gate_seq.push_back(drv);

        // Follow worst-arrival input
        NetId worst_in = -1;
        double worst_arr = -1;
        for (auto ni : g.inputs) {
            double arr = (ni < static_cast<int>(arrival.size())) ? arrival[ni] : 0;
            if (arr > worst_arr) { worst_arr = arr; worst_in = ni; }
        }
        if (worst_in < 0) break;
        net_seq.push_back(worst_in);
        curr = worst_in;
    }
    std::reverse(gate_seq.begin(), gate_seq.end());
    std::reverse(net_seq.begin(), net_seq.end());
    detail.gates = gate_seq;

    // Build path net set for path-specific load filtering
    std::unordered_set<NetId> path_net_set(net_seq.begin(), net_seq.end());

    // Exact slew propagation through path gates with path-specific loads
    double current_slew = 0.01;
    double cumulative = 0;
    for (size_t idx = 0; idx < gate_seq.size(); ++idx) {
        auto gid = gate_seq[idx];
        auto& g = nl_.gate(gid);
        double gd = gate_delay(gid, current_slew);
        NetId from = g.inputs.empty() ? -1 : g.inputs[0];
        double wd = wire_delay(from, g.output);

        detail.cell_delays.push_back(gd);
        detail.wire_delays.push_back(wd);
        cumulative += gd + wd;
        detail.arrivals.push_back(cumulative);

        // Path-specific load: weight on-path fanout fully, off-path partially
        double path_load = 0.001;
        if (g.output >= 0) {
            auto& out_net = nl_.net(g.output);
            for (auto fo_gid : out_net.fanout) {
                auto& fo_g = nl_.gate(fo_gid);
                bool on_path = (idx + 1 < gate_seq.size() &&
                                fo_gid == gate_seq[idx + 1]);
                if (!on_path && fo_g.output >= 0 &&
                    path_net_set.count(fo_g.output))
                    on_path = true;

                double pin_cap = 0.002;
                if (lib_) {
                    std::string ts = gate_type_str(fo_g.type);
                    if (auto* cell = lib_->find_cell(ts)) {
                        for (auto& pin : cell->pins) {
                            if (pin.direction == "input" &&
                                pin.capacitance > 0) {
                                pin_cap = pin.capacitance;
                                break;
                            }
                        }
                    }
                }
                path_load += on_path ? pin_cap : pin_cap * 0.3;
            }
        }
        current_slew = output_slew(gid, current_slew, path_load);
    }

    detail.total_delay = cumulative;
    return detail;
}

double StaEngine::compute_path_delay_exact(const std::vector<int>& gates) {
    double current_slew = 0.01;
    double total = 0;

    // Build gate set for path-specific load determination
    std::unordered_set<int> gate_set(gates.begin(), gates.end());

    for (size_t idx = 0; idx < gates.size(); ++idx) {
        auto gid = gates[idx];
        auto& g = nl_.gate(gid);
        double gd = gate_delay(gid, current_slew);
        NetId from = g.inputs.empty() ? -1 : g.inputs[0];
        double wd = wire_delay(from, g.output);
        total += gd + wd;

        // Path-specific load
        double path_load = 0.001;
        if (g.output >= 0) {
            auto& out_net = nl_.net(g.output);
            for (auto fo_gid : out_net.fanout) {
                bool on_path = gate_set.count(fo_gid) > 0;
                double pin_cap = 0.002;
                if (lib_) {
                    auto& fo_g = nl_.gate(fo_gid);
                    std::string ts = gate_type_str(fo_g.type);
                    if (auto* cell = lib_->find_cell(ts)) {
                        for (auto& pin : cell->pins) {
                            if (pin.direction == "input" &&
                                pin.capacitance > 0) {
                                pin_cap = pin.capacitance;
                                break;
                            }
                        }
                    }
                }
                path_load += on_path ? pin_cap : pin_cap * 0.3;
            }
        }
        current_slew = output_slew(gid, current_slew, path_load);
    }
    return total;
}

// ============================================================================
// Path-Based Analysis (PBA)
// ============================================================================

StaEngine::PbaResult StaEngine::run_path_based(int num_paths) {
    PbaResult result;
    result.pba_wns = 0;
    result.pba_tns = 0;
    result.paths_improved = 0;

    // 1. Run standard graph-based STA to seed arrival times
    if (pin_timing_.empty()) {
        build_timing_graph();
        forward_propagation();
        hold_forward_propagation();
        if (last_clock_period_ > 0) {
            backward_propagation(last_clock_period_);
            hold_backward_propagation();
        }
        compute_slacks();
    }

    // Build arrival vector for trace_path back-tracking
    std::vector<double> arrival_vec(nl_.num_nets(), 0);
    for (auto& [nid, pt] : pin_timing_)
        arrival_vec[nid] = pt.worst_arrival();

    // 2. Collect endpoints sorted by worst graph-based slack
    struct EpInfo { NetId net; double slack; double graph_delay; std::string name; };
    std::vector<EpInfo> endpoints;
    for (auto po : nl_.primary_outputs()) {
        auto& pt = pin_timing_[po];
        endpoints.push_back({po, pt.worst_slack(), pt.worst_arrival(), nl_.net(po).name});
    }
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) {
            NetId d = ff.inputs[0];
            auto& pt = pin_timing_[d];
            endpoints.push_back({d, pt.worst_slack(), pt.worst_arrival(), ff.name + "/D"});
        }
    }
    std::sort(endpoints.begin(), endpoints.end(),
              [](auto& a, auto& b) { return a.slack < b.slack; });

    int count = std::min(num_paths, static_cast<int>(endpoints.size()));

    // 3. For each critical endpoint, trace path and recompute with exact slew
    for (int i = 0; i < count; ++i) {
        auto detail = trace_path(endpoints[i].net, arrival_vec);
        double req = last_clock_period_ > 0 ? last_clock_period_ : 1e18;
        NetId ep = endpoints[i].net;
        if (pin_timing_.count(ep)) {
            req = std::min(pin_timing_.at(ep).required_rise,
                           pin_timing_.at(ep).required_fall);
        }
        detail.slack = req - detail.total_delay;
        detail.is_critical = (detail.slack < 0);

        // Apply CPPR credit if enabled: find common clock path pessimism
        if (cppr_.enabled && ocv_mode_ != OcvMode::NONE) {
            // Build a temporary TimingPath for compute_cppr_credits
            // For PBA results, approximate CPPR credit proportionally
            // to the graph-based credit already computed
            double cppr_adjustment = 0;
            // Find startpoint by tracing to root
            NetId cur = ep;
            std::string startpoint;
            for (int depth = 0; depth < 200; ++depth) {
                GateId drv = nl_.net(cur).driver;
                if (drv < 0) break;
                auto& g = nl_.gate(drv);
                if (g.type == GateType::DFF || g.type == GateType::INPUT) {
                    startpoint = g.name;
                    break;
                }
                if (g.inputs.empty()) break;
                // Follow worst input
                NetId wi = -1; double wa = -1;
                for (auto ni : g.inputs) {
                    double a = (ni < (int)arrival_vec.size()) ? arrival_vec[ni] : 0;
                    if (a > wa) { wa = a; wi = ni; }
                }
                if (wi < 0) break;
                cur = wi;
            }

            if (!startpoint.empty()) {
                // Compute CPPR for this specific path
                TimingPath tmp_path;
                tmp_path.startpoint = startpoint;
                tmp_path.endpoint = endpoints[i].name;
                tmp_path.gates = std::vector<GateId>(detail.gates.begin(),
                                                     detail.gates.end());
                tmp_path.is_hold = false;
                tmp_path.slack = 0;
                std::vector<TimingPath> tmp_paths = {tmp_path};
                compute_cppr_credits(tmp_paths, last_clock_period_);
                cppr_adjustment = tmp_paths[0].cppr_credit;
            }
            detail.slack += cppr_adjustment;
        }

        // PBA typically recovers pessimism
        if (detail.total_delay < endpoints[i].graph_delay)
            result.paths_improved++;

        if (detail.slack < 0)
            result.pba_tns += detail.slack;
        result.pba_wns = std::min(result.pba_wns, detail.slack);
        result.paths.push_back(std::move(detail));
    }

    return result;
}

// ============================================================================
// SI Helpers
// ============================================================================

std::vector<int> StaEngine::find_aggressors(int net_idx) {
    std::vector<int> aggressors;
    if (!pd_ || pd_->wires.empty()) return aggressors;

    // Find wires belonging to victim net
    for (size_t i = 0; i < pd_->wires.size(); ++i) {
        auto& victim = pd_->wires[i];
        if (victim.net_id != net_idx) continue;

        for (size_t j = 0; j < pd_->wires.size(); ++j) {
            if (i == j) continue;
            auto& agg = pd_->wires[j];
            if (agg.layer != victim.layer) continue;
            if (agg.net_id == net_idx) continue;

            double dx = (agg.start.x + agg.end.x) / 2.0 -
                        (victim.start.x + victim.end.x) / 2.0;
            double dy = (agg.start.y + agg.end.y) / 2.0 -
                        (victim.start.y + victim.end.y) / 2.0;
            double spacing = std::sqrt(dx * dx + dy * dy);
            if (spacing <= xtalk_.max_coupling_distance_um && agg.net_id >= 0) {
                // Avoid duplicates
                bool found = false;
                for (auto a : aggressors)
                    if (a == agg.net_id) { found = true; break; }
                if (!found)
                    aggressors.push_back(agg.net_id);
            }
        }
        break;
    }
    return aggressors;
}

double StaEngine::compute_crosstalk_delay(int victim_net,
                                           const std::vector<int>& aggressors)
{
    if (!pd_ || pd_->wires.empty()) return 0;

    double victim_cap = net_load_cap(static_cast<NetId>(victim_net));
    double total_delta = 0;

    for (size_t i = 0; i < pd_->wires.size(); ++i) {
        auto& vw = pd_->wires[i];
        if (vw.net_id != victim_net) continue;
        double victim_len = vw.start.dist(vw.end);
        if (victim_len < 0.001) continue;

        for (int agg_net : aggressors) {
            for (size_t j = 0; j < pd_->wires.size(); ++j) {
                auto& aw = pd_->wires[j];
                if (aw.net_id != agg_net) continue;
                if (aw.layer != vw.layer) continue;

                double agg_len = aw.start.dist(aw.end);
                double prl = std::min(victim_len, agg_len);
                double coupling_cap = xtalk_.coupling_cap_per_um * prl;

                // SI delta = coupling_cap × aggressor_slew / victim_cap
                double si_delta = coupling_cap * xtalk_.aggressor_slew /
                                  std::max(0.001, victim_cap);
                total_delta += si_delta;
                break;
            }
        }
        break;
    }
    return total_delta;
}

// ============================================================================
// SI-Aware Delay Computation
// ============================================================================

std::vector<StaEngine::SiDelay> StaEngine::compute_si_delays() {
    std::vector<SiDelay> results;
    if (!si_enabled_ && !xtalk_.enabled) return results;

    for (size_t nid = 0; nid < nl_.num_nets(); ++nid) {
        auto& net = nl_.net(static_cast<NetId>(nid));
        if (net.driver < 0) continue;

        auto aggressors = find_aggressors(static_cast<int>(nid));
        if (aggressors.empty()) continue;

        SiDelay sid;
        sid.victim_net = static_cast<int>(nid);
        sid.aggressors = aggressors;

        // Nominal delay: wire delay without SI
        sid.nominal_delay = wire_delay(static_cast<NetId>(nid),
                                       static_cast<NetId>(nid));
        sid.si_delta = compute_crosstalk_delay(static_cast<int>(nid), aggressors);
        sid.total_delay = sid.nominal_delay + sid.si_delta;
        results.push_back(sid);
    }
    return results;
}

// ============================================================================
// Incremental STA
// ============================================================================

std::vector<int> StaEngine::find_affected_cone(const std::vector<int>& changed) {
    std::unordered_set<int> affected_set(changed.begin(), changed.end());
    std::queue<int> worklist;

    // Seed: all nets driven by changed gates
    for (int gid : changed) {
        auto& g = nl_.gate(static_cast<GateId>(gid));
        if (g.output >= 0) worklist.push(g.output);
    }

    // BFS forward through fanout cone
    while (!worklist.empty()) {
        int nid = worklist.front(); worklist.pop();
        auto& net = nl_.net(static_cast<NetId>(nid));
        for (auto fo_gid : net.fanout) {
            if (affected_set.count(fo_gid)) continue;
            affected_set.insert(fo_gid);
            auto& g = nl_.gate(fo_gid);
            if (g.output >= 0)
                worklist.push(g.output);
        }
    }

    // Return in topo order for correct propagation
    std::vector<int> result;
    for (auto gid : topo_) {
        if (affected_set.count(gid))
            result.push_back(gid);
    }
    return result;
}

StaEngine::IncrStaResult StaEngine::run_incremental(
    const std::vector<int>& changed_gates)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    IncrStaResult result;

    // Ensure timing graph and base timing exist
    if (topo_.empty()) build_timing_graph();
    if (pin_timing_.empty()) {
        forward_propagation();
        hold_forward_propagation();
        if (last_clock_period_ > 0) {
            backward_propagation(last_clock_period_);
            hold_backward_propagation();
        }
        compute_slacks();
    }

    // Find forward cone of affected gates
    auto cone = find_affected_cone(changed_gates);
    result.cones_updated = static_cast<int>(cone.size());

    // Re-propagate arrivals only through affected cone
    analyzing_late_ = true;
    double pi_slew = 0.01;
    for (int gid : cone) {
        auto& g = nl_.gate(static_cast<GateId>(gid));
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        double max_arr = 0;
        double worst_slew = pi_slew;
        for (auto ni : g.inputs) {
            double arr = pin_timing_[ni].worst_arrival();
            if (arr > max_arr) {
                max_arr = arr;
                worst_slew = std::max(pin_timing_[ni].slew_rise,
                                       pin_timing_[ni].slew_fall);
            }
        }

        double gd = gate_delay(static_cast<GateId>(gid), worst_slew);
        double wd = wire_delay(g.inputs.empty() ? -1 : g.inputs[0], g.output);
        double out_arr = max_arr + gd + wd;

        auto& pt = pin_timing_[g.output];
        pt.arrival_rise = out_arr;
        pt.arrival_fall = out_arr;

        double load = net_load_cap(g.output);
        double out_slew = output_slew(static_cast<GateId>(gid), worst_slew, load);
        pt.slew_rise = out_slew;
        pt.slew_fall = out_slew;
    }

    // Re-propagate required times backward through affected cone (reverse)
    if (last_clock_period_ > 0) {
        for (int idx = static_cast<int>(cone.size()) - 1; idx >= 0; --idx) {
            int gid = cone[idx];
            auto& g = nl_.gate(static_cast<GateId>(gid));
            if (g.type == GateType::DFF || g.type == GateType::INPUT ||
                g.type == GateType::OUTPUT || g.output < 0) continue;

            double out_req = std::min(pin_timing_[g.output].required_rise,
                                       pin_timing_[g.output].required_fall);
            double worst_slew_bp = 0.01;
            for (auto ni : g.inputs)
                worst_slew_bp = std::max(worst_slew_bp, pin_timing_[ni].slew_rise);
            double gd = gate_delay(static_cast<GateId>(gid), worst_slew_bp);
            for (auto ni : g.inputs) {
                double in_req = out_req - gd;
                pin_timing_[ni].required_rise = std::min(pin_timing_[ni].required_rise, in_req);
                pin_timing_[ni].required_fall = std::min(pin_timing_[ni].required_fall, in_req);
            }
        }
    }

    // Recompute slacks and gather WNS/TNS
    compute_slacks();
    result.wns = 0;
    result.tns = 0;
    for (auto po : nl_.primary_outputs()) {
        double slack = pin_timing_[po].worst_slack();
        if (slack < 0) { result.tns += slack; result.wns = std::min(result.wns, slack); }
    }
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (!ff.inputs.empty()) {
            double slack = pin_timing_[ff.inputs[0]].worst_slack();
            if (slack < 0) { result.tns += slack; result.wns = std::min(result.wns, slack); }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

// ============================================================================
// Bottleneck Analysis
// ============================================================================

std::vector<StaEngine::Bottleneck> StaEngine::analyze_bottlenecks(int top_n) {
    // Extract a large set of critical paths
    auto paths = extract_paths(std::max(top_n * 5, 100), false);

    // For each gate on critical paths, count how many pass through it and
    // accumulate the slack impact (gate_delay × num_paths_through).
    std::unordered_map<int, int> path_count;
    std::unordered_map<int, double> slack_impact;

    for (auto& path : paths) {
        if (path.slack >= 0) continue;  // only consider violating paths
        for (auto gid : path.gates) {
            path_count[gid]++;
            double gd = gate_delay(gid, 0.01);
            slack_impact[gid] += gd * std::abs(path.slack);
        }
    }

    // Build sorted bottleneck list
    std::vector<Bottleneck> bottlenecks;
    for (auto& [gid, count] : path_count) {
        Bottleneck bn;
        bn.gate_id = gid;
        bn.gate_name = nl_.gate(static_cast<GateId>(gid)).name;
        bn.num_critical_paths_through = count;
        bn.total_slack_impact = slack_impact[gid];
        bottlenecks.push_back(bn);
    }

    std::sort(bottlenecks.begin(), bottlenecks.end(),
              [](auto& a, auto& b) { return a.total_slack_impact > b.total_slack_impact; });

    if (static_cast<int>(bottlenecks.size()) > top_n)
        bottlenecks.resize(top_n);

    return bottlenecks;
}

// ============================================================================
// Latch Timing Helpers
// ============================================================================

std::vector<GateId> StaEngine::find_latches() const {
    std::vector<GateId> latches;
    for (size_t i = 0; i < nl_.num_gates(); ++i) {
        if (nl_.gate(static_cast<GateId>(i)).type == GateType::DLATCH)
            latches.push_back(static_cast<GateId>(i));
    }
    return latches;
}

LatchTimingInfo StaEngine::compute_latch_edges(GateId latch_id, double clock_period) const {
    // Check for user-provided latch timing first
    auto it = latch_info_.find(latch_id);
    if (it != latch_info_.end())
        return it->second;

    // Default: assume a 50% duty-cycle clock
    // Opening edge at 0 (rising edge → latch transparent)
    // Closing edge at clock_period/2 (falling edge → latch opaque)
    LatchTimingInfo info;
    info.opening_edge = 0.0;
    info.closing_edge = clock_period * 0.5;
    // Borrow limit = transparent window (closing - opening), capped at half period
    info.borrow_limit = info.closing_edge - info.opening_edge;
    info.borrowed_time = 0.0;
    return info;
}

// ============================================================================
// Multi-Supply-Domain Timing Helpers
// ============================================================================

double StaEngine::voltage_delay_scale(GateId gid) const {
    if (voltage_domains_.empty()) return 1.0;

    auto& g = nl_.gate(gid);
    auto cit = cell_domain_map_.find(g.name);
    if (cit == cell_domain_map_.end()) return 1.0;

    auto dit = voltage_domains_.find(cit->second);
    if (dit == voltage_domains_.end()) return 1.0;

    double v_domain = dit->second.nominal_voltage;
    if (v_domain <= 0) return 1.0;

    // delay_scaled = delay_nominal × (V_ref / V_domain)^alpha
    // V_ref = corner reference voltage (from derate_)
    double v_ref = derate_.ref_voltage;
    if (v_ref <= 0) v_ref = 1.0;

    return std::pow(v_ref / v_domain, voltage_scaling_alpha_);
}

double StaEngine::level_shifter_penalty(GateId from_gate, GateId to_gate) const {
    if (voltage_domains_.empty()) return 0.0;

    auto& gf = nl_.gate(from_gate);
    auto& gt = nl_.gate(to_gate);

    auto fit = cell_domain_map_.find(gf.name);
    auto tit = cell_domain_map_.find(gt.name);

    // Both must be mapped to a domain to check crossing
    if (fit == cell_domain_map_.end() || tit == cell_domain_map_.end()) return 0.0;
    if (fit->second == tit->second) return 0.0;

    // Crossing voltage domains — apply level-shifter delay
    return level_shifter_delay_;
}

} // namespace sf
