// SiliconForge — Power Analysis Implementation
// Activity-propagated power estimation with glitch, clock tree, multi-voltage.
#include "timing/power.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <cmath>
#include <unordered_map>

namespace sf {

double PowerAnalyzer::net_capacitance(NetId nid) const {
    auto& net = nl_.net(nid);
    double c_wire = 0.001; // 1 fF base wire cap
    if (lib_) {
        double c_load = 0;
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
        return c_wire + c_load;
    }
    return c_wire + net.fanout.size() * 0.002;
}

double PowerAnalyzer::cell_leakage(GateId gid) const {
    auto& g = nl_.gate(gid);
    if (lib_) {
        std::string type_str = gate_type_str(g.type);
        int num_in = (int)g.inputs.size();
        std::string candidates[] = {
            g.name, type_str + std::to_string(num_in),
            type_str, type_str + "_X1"
        };
        for (auto& cand : candidates) {
            if (auto* cell = lib_->find_cell(cand)) {
                if (cell->leakage_power > 0) return cell->leakage_power;
            }
        }
    }
    switch (g.type) {
        case GateType::BUF:  case GateType::NOT:  return 0.5;
        case GateType::AND:  case GateType::OR:   return 1.0 * g.inputs.size();
        case GateType::NAND: case GateType::NOR:  return 0.8 * g.inputs.size();
        case GateType::XOR:  case GateType::XNOR: return 1.5;
        case GateType::MUX:                       return 2.0;
        case GateType::DFF:                        return 5.0;
        default: return 1.0;
    }
}

double PowerAnalyzer::gate_vdd(GateId gid, double default_vdd) const {
    auto it = voltage_domains_.find(gid);
    return (it != voltage_domains_.end()) ? it->second : default_vdd;
}

double PowerAnalyzer::cell_dynamic(GateId gid, double freq, double vdd, double activity) const {
    auto& g = nl_.gate(gid);
    if (g.output < 0) return 0;
    double c_load = net_capacitance(g.output);
    double alpha = activity;
    auto it = activities_.find(g.output);
    if (it != activities_.end()) alpha = it->second;
    double cell_vdd = vdd;
    auto vit = voltage_domains_.find(gid);
    if (vit != voltage_domains_.end()) cell_vdd = vit->second;
    double p_pw = alpha * c_load * cell_vdd * cell_vdd * freq * 1e6;
    return p_pw * 1e-9; // pW → mW
}

// Glitch activity estimation: extra transitions from different-arrival-time inputs
double PowerAnalyzer::glitch_activity(GateId gid, double base_activity) const {
    auto& g = nl_.gate(gid);
    if (g.inputs.size() < 2) return 0;

    // Glitch happens when inputs arrive at different times at reconvergent gates.
    // Estimate: look at topo level spread of inputs — larger spread = more glitch
    int min_level = INT_MAX, max_level = 0;
    for (auto ni : g.inputs) {
        auto it = topo_levels_.find(ni);
        int lvl = (it != topo_levels_.end()) ? it->second : 0;
        min_level = std::min(min_level, lvl);
        max_level = std::max(max_level, lvl);
    }
    int level_diff = max_level - min_level;
    if (level_diff <= 1) return 0;

    // Glitch probability scales with level difference and input activity
    // Typical: 5-15% extra activity per level of skew
    double glitch_alpha = base_activity * 0.08 * level_diff;
    return std::min(glitch_alpha, 0.5); // cap at 50%
}

PowerResult PowerAnalyzer::analyze(double clock_freq_mhz, double supply_voltage,
                                     double default_activity) {
    auto t0 = std::chrono::high_resolution_clock::now();
    PowerResult result;
    result.clock_freq_mhz = clock_freq_mhz;
    result.supply_voltage = supply_voltage;

    auto topo = nl_.topo_order();

    // Compute topo levels for glitch estimation
    topo_levels_.clear();
    for (auto pi : nl_.primary_inputs())
        topo_levels_[pi] = 0;
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0) topo_levels_[ff.output] = 0;
    }
    for (auto gid : topo) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT ||
            g.type == GateType::DFF || g.output < 0) continue;
        int max_lvl = 0;
        for (auto ni : g.inputs) {
            auto it = topo_levels_.find(ni);
            max_lvl = std::max(max_lvl, (it != topo_levels_.end()) ? it->second : 0);
        }
        topo_levels_[g.output] = max_lvl + 1;
    }

    // === Activity Propagation ===
    for (auto pi : nl_.primary_inputs())
        activities_[pi] = default_activity;
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0) activities_[ff.output] = default_activity;
    }

    for (auto gid : topo) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT ||
            g.type == GateType::DFF || g.output < 0) continue;
        if (activities_.count(g.output)) continue;

        double alpha_out = default_activity;
        if (!g.inputs.empty()) {
            double avg_in = 0;
            for (auto ni : g.inputs) {
                auto it = activities_.find(ni);
                avg_in += (it != activities_.end()) ? it->second : default_activity;
            }
            avg_in /= g.inputs.size();

            switch (g.type) {
                case GateType::BUF:  alpha_out = avg_in; break;
                case GateType::NOT:  alpha_out = avg_in; break;
                case GateType::AND:
                case GateType::NAND: {
                    double prod = 1.0;
                    for (auto ni : g.inputs) {
                        auto it = activities_.find(ni);
                        prod *= (it != activities_.end()) ? it->second : default_activity;
                    }
                    alpha_out = prod;
                    break;
                }
                case GateType::OR:
                case GateType::NOR: {
                    double prod = 1.0;
                    for (auto ni : g.inputs) {
                        auto it = activities_.find(ni);
                        double a = (it != activities_.end()) ? it->second : default_activity;
                        prod *= (1.0 - a);
                    }
                    alpha_out = 1.0 - prod;
                    break;
                }
                case GateType::XOR:
                case GateType::XNOR: {
                    if (g.inputs.size() == 2) {
                        auto it_a = activities_.find(g.inputs[0]);
                        double a = (it_a != activities_.end()) ? it_a->second : default_activity;
                        alpha_out = 2.0 * a * (1.0 - a);
                    } else {
                        alpha_out = avg_in;
                    }
                    break;
                }
                case GateType::MUX: alpha_out = avg_in; break;
                default: alpha_out = avg_in; break;
            }
            alpha_out = std::max(0.0, std::min(1.0, alpha_out));
        }
        activities_[g.output] = alpha_out;
    }

    // === Per-cell power computation ===
    std::vector<PowerResult::CellPower> cell_powers;
    int num_dff = 0;

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;

        double cell_activity = default_activity;
        if (g.output >= 0) {
            auto it = activities_.find(g.output);
            if (it != activities_.end()) cell_activity = it->second;
        }

        double cell_vdd = gate_vdd(gid, supply_voltage);
        double leakage_mw = cell_leakage(gid) * 1e-6; // nW → mW
        // Leakage scales with V² for subthreshold
        leakage_mw *= (cell_vdd / supply_voltage) * (cell_vdd / supply_voltage);

        double dynamic_mw = cell_dynamic(gid, clock_freq_mhz, cell_vdd, cell_activity);
        double internal_mw = dynamic_mw * 0.15;

        // Glitch power
        double glitch_alpha = glitch_activity(gid, cell_activity);
        double glitch_mw = 0;
        if (glitch_alpha > 0 && g.output >= 0) {
            double c_load = net_capacitance(g.output);
            glitch_mw = glitch_alpha * c_load * cell_vdd * cell_vdd * clock_freq_mhz * 1e6 * 1e-9;
        }

        result.static_power_mw += leakage_mw;
        result.switching_power_mw += dynamic_mw;
        result.internal_power_mw += internal_mw;
        result.glitch_power_mw += glitch_mw;

        // Clock power: DFF toggle + clock buffer estimation
        if (g.type == GateType::DFF) {
            num_dff++;
            double clk_dyn = cell_dynamic(gid, clock_freq_mhz, cell_vdd, 1.0);
            result.clock_power_mw += clk_dyn;
        }

        cell_powers.push_back({
            g.name.empty() ? ("g" + std::to_string(gid)) : g.name,
            dynamic_mw + internal_mw + glitch_mw, leakage_mw,
            dynamic_mw + internal_mw + leakage_mw + glitch_mw
        });
        result.num_cells++;
    }

    // Clock tree buffer power: estimate ~1 buffer per 8 DFFs, each at full toggle
    int cts_buffers = (num_dff + 7) / 8;
    if (cts_buffers > 0) {
        // Each CTS buffer: C_buf * V² * f at activity=2.0 (both edges)
        double c_buf = 0.005; // 5fF per buffer
        double cts_buf_power = cts_buffers * 2.0 * c_buf * supply_voltage * supply_voltage
                               * clock_freq_mhz * 1e6 * 1e-9;
        result.clock_power_mw += cts_buf_power;
    }

    result.dynamic_power_mw = result.switching_power_mw + result.internal_power_mw +
                               result.glitch_power_mw;
    result.total_power_mw = result.dynamic_power_mw + result.static_power_mw +
                             result.clock_power_mw;

    // Top consumers
    std::sort(cell_powers.begin(), cell_powers.end(),
              [](auto& a, auto& b) { return a.total > b.total; });
    int top_n = std::min(10, (int)cell_powers.size());
    result.top_consumers.assign(cell_powers.begin(), cell_powers.begin() + top_n);

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = "Total: " + std::to_string(result.total_power_mw) + " mW " +
                     "(dynamic: " + std::to_string(result.dynamic_power_mw) +
                     ", static: " + std::to_string(result.static_power_mw) +
                     ", clock: " + std::to_string(result.clock_power_mw) +
                     ", glitch: " + std::to_string(result.glitch_power_mw) + ")";
    return result;
}

} // namespace sf
