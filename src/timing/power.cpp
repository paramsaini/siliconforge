// SiliconForge — Power Analysis Implementation
// Activity-propagated power estimation with glitch, clock tree, multi-voltage.
#include "timing/power.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <sstream>
#include <climits>
#include <queue>
#include <set>

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

// Short-circuit power: Isc during simultaneous PMOS/NMOS conduction on input transitions.
// Model: P_sc = K_sc * Vdd * (slew_time / rise_time) * frequency * width_factor * activity
double PowerAnalyzer::cell_short_circuit(GateId gid, double freq, double vdd, double activity) const {
    constexpr double K_sc = 0.1; // empirical short-circuit constant
    auto& g = nl_.gate(gid);
    if (g.output < 0 || g.inputs.empty()) return 0;

    // Width factor: scales with gate complexity (number of stacked transistors)
    double width_factor = 1.0;
    switch (g.type) {
        case GateType::BUF:  case GateType::NOT:  width_factor = 1.0; break;
        case GateType::AND:  case GateType::NAND:  width_factor = 1.0 + 0.2 * (g.inputs.size() - 1); break;
        case GateType::OR:   case GateType::NOR:   width_factor = 1.0 + 0.2 * (g.inputs.size() - 1); break;
        case GateType::XOR:  case GateType::XNOR:  width_factor = 1.8; break;
        case GateType::MUX:                         width_factor = 2.0; break;
        case GateType::DFF:                          width_factor = 0.5; break;
        default: width_factor = 1.0; break;
    }

    // Slew time and rise time from Liberty data (fall back to defaults)
    double slew_time = 0.1; // ns default
    double rise_time = 0.05; // ns default
    if (lib_) {
        std::string type_str = gate_type_str(g.type);
        int num_in = (int)g.inputs.size();
        std::string candidates[] = {
            type_str + std::to_string(num_in), type_str, type_str + "_X1"
        };
        for (auto& cand : candidates) {
            if (auto* cell = lib_->find_cell(cand)) {
                for (auto& t : cell->timings) {
                    if (t.rise_transition > 0) slew_time = t.rise_transition;
                    if (t.cell_rise > 0) rise_time = t.cell_rise;
                    break;
                }
                break;
            }
        }
    }
    if (rise_time <= 0) rise_time = 0.05;
    double slew_ratio = slew_time / rise_time;
    slew_ratio = std::min(slew_ratio, 5.0); // cap extreme ratios

    // P_sc = K_sc * Vdd * slew_ratio * freq_Hz * width_factor * activity  (in mW)
    double p_sc = K_sc * vdd * slew_ratio * (freq * 1e6) * width_factor * activity;
    return p_sc * 1e-9; // scale to mW (same convention as cell_dynamic)
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

    // Cache params for enhanced methods
    last_freq_mhz_ = clock_freq_mhz;
    last_vdd_ = supply_voltage;
    last_default_activity_ = default_activity;

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

        // Apply temperature scaling: P_leak(T) = P_leak(25) * 2^((T-25)/10)
        leakage_mw *= leakage_temp_factor_;

        // Apply voltage derating from IR drop feedback
        std::string cell_name = g.name.empty() ? ("g" + std::to_string(gid)) : g.name;
        auto vd_it = voltage_derating_.find(cell_name);
        if (vd_it != voltage_derating_.end()) {
            leakage_mw *= vd_it->second;
        }

        double dynamic_mw = cell_dynamic(gid, clock_freq_mhz, cell_vdd, cell_activity);
        double internal_mw = dynamic_mw * 0.15;

        // Glitch power
        double glitch_alpha = glitch_activity(gid, cell_activity);
        double glitch_mw = 0;
        if (glitch_alpha > 0 && g.output >= 0) {
            double c_load = net_capacitance(g.output);
            glitch_mw = glitch_alpha * c_load * cell_vdd * cell_vdd * clock_freq_mhz * 1e6 * 1e-9;
        }

        // Short-circuit power (Isc)
        double sc_mw = cell_short_circuit(gid, clock_freq_mhz, cell_vdd, cell_activity);

        result.static_power_mw += leakage_mw;
        result.switching_power_mw += dynamic_mw;
        result.internal_power_mw += internal_mw;
        result.glitch_power_mw += glitch_mw;
        result.short_circuit_power_mw += sc_mw;

        // Clock power: DFF toggle + clock buffer estimation
        if (g.type == GateType::DFF) {
            num_dff++;
            double clk_dyn = cell_dynamic(gid, clock_freq_mhz, cell_vdd, 1.0);
            result.clock_power_mw += clk_dyn;
        }

        cell_powers.push_back({
            g.name.empty() ? ("g" + std::to_string(gid)) : g.name,
            dynamic_mw + internal_mw + glitch_mw + sc_mw, leakage_mw,
            dynamic_mw + internal_mw + leakage_mw + glitch_mw + sc_mw
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
                               result.glitch_power_mw + result.short_circuit_power_mw;
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
                     ", glitch: " + std::to_string(result.glitch_power_mw) +
                     ", short_circuit: " + std::to_string(result.short_circuit_power_mw) + ")";
    return result;
}

bool PowerAnalyzer::load_vcd(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    return parse_vcd_string(content);
}

bool PowerAnalyzer::parse_vcd_string(const std::string& content) {
    // Parse IEEE 1364-2001 VCD format to extract toggle counts per signal
    // $var wire 1 ! clk $end
    // $var wire 1 " rst $end
    // #0 0! 0"
    // #10 1!
    
    std::unordered_map<std::string, std::string> code_to_name; // VCD code → net name
    std::unordered_map<std::string, int> toggles; // net name → toggle count
    std::unordered_map<std::string, char> last_val; // VCD code → last value
    
    std::istringstream iss(content);
    std::string line;
    uint64_t end_time = 0;
    bool in_defs = true;
    
    while (std::getline(iss, line)) {
        // Trim
        while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
            line.pop_back();
        
        if (line.find("$var") == 0) {
            // $var wire 1 ! clk $end
            std::istringstream ls(line);
            std::string tok, type, width, code, name;
            ls >> tok >> type >> width >> code >> name;
            if (!code.empty() && !name.empty()) {
                code_to_name[code] = name;
                last_val[code] = 'x';
                toggles[name] = 0;
            }
        } else if (line.find("$enddefinitions") == 0) {
            in_defs = false;
        } else if (!in_defs && !line.empty()) {
            if (line[0] == '#') {
                // Timestamp
                end_time = std::stoull(line.substr(1));
            } else if (line[0] == '0' || line[0] == '1' || line[0] == 'x' || line[0] == 'z') {
                // Value change: 0! or 1"
                char val = line[0];
                std::string code = line.substr(1);
                if (code_to_name.count(code)) {
                    if (last_val[code] != val && last_val[code] != 'x')
                        toggles[code_to_name[code]]++;
                    last_val[code] = val;
                }
            } else if (line[0] == 'b' || line[0] == 'B') {
                // Bus value change: b1010 <code>
                auto sp = line.find(' ');
                if (sp != std::string::npos) {
                    std::string code = line.substr(sp + 1);
                    if (code_to_name.count(code))
                        toggles[code_to_name[code]]++;
                }
            }
        }
    }
    
    if (end_time == 0) end_time = 1;
    
    // Convert toggle counts to activity factors and set on nets
    for (auto& [name, count] : toggles) {
        double activity = (double)count / (double)end_time;
        activity = std::min(activity, 1.0);
        
        // Find matching net by name
        for (size_t i = 0; i < nl_.num_nets(); i++) {
            if (nl_.net(i).name == name || nl_.net(i).name.find(name) != std::string::npos) {
                activities_[i] = activity;
                break;
            }
        }
    }
    
    return true;
}

bool PowerAnalyzer::load_saif(const std::string& filename) {
    // SAIF (Switching Activity Interchange Format) parser
    // Simplified: reads TC (toggle count), T0, T1 per signal
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    
    std::string line, current_signal;
    uint64_t total_time = 1;
    
    while (std::getline(f, line)) {
        std::istringstream ls(line);
        std::string tok;
        ls >> tok;
        
        if (tok == "(DURATION") {
            ls >> total_time;
        } else if (tok == "(INSTANCE" || tok == "(PORT") {
            // Next token might have the signal name
        } else if (line.find("(") != std::string::npos && line.find("TC") != std::string::npos) {
            // (signal_name (TC 42) (T0 58) (T1 42))
            auto paren = line.find('(');
            auto space = line.find(' ', paren + 1);
            if (space != std::string::npos) {
                current_signal = line.substr(paren + 1, space - paren - 1);
                auto tc_pos = line.find("TC");
                if (tc_pos != std::string::npos) {
                    int tc = 0;
                    std::sscanf(line.c_str() + tc_pos, "TC %d", &tc);
                    double activity = (double)tc / (double)total_time;
                    
                    for (size_t i = 0; i < nl_.num_nets(); i++) {
                        if (nl_.net(i).name == current_signal) {
                            activities_[i] = std::min(activity, 1.0);
                            break;
                        }
                    }
                }
            }
        }
    }
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Vectorless Toggle Rate Propagation
// Reference: Najm, "Transition Density: A Stochastic Measure of
//            Activity in Digital Circuits", DAC 1991
// ══════════════════════════════════════════════════════════════════════

// Propagate toggle rates through combinational logic in topological order.
// Uses the transition-density model:
//   AND:  TD(out) = sum_i( TD(i) * prod_{j!=i} P(j) )
//   OR:   TD(out) = sum_i( TD(i) * prod_{j!=i} (1 - P(j)) )
//   XOR:  TD(out) = sum( TD(i) )
//   NOT/BUF: TD(out) = TD(in)
//   DFF:  TD(out) = TD(d)  (synchronous capture)
//
// Static probability P(signal=1) is propagated alongside toggle density:
//   AND:  P(out) = prod( P(i) )
//   OR:   P(out) = 1 - prod( 1 - P(i) )
//   XOR:  P(out) = sum(P(i)) - 2*prod(P(i))  (for 2-input)
//   NOT:  P(out) = 1 - P(in)
//   BUF:  P(out) = P(in)
//   DFF:  P(out) = P(d)

static void propagate_toggle_rates(
    const Netlist& nl,
    std::unordered_map<NetId, double>& activities,
    const std::unordered_map<std::string, double>& user_static_probs,
    double default_activity)
{
    std::unordered_map<NetId, double> static_prob;

    // Initialize PIs
    for (auto pi : nl.primary_inputs()) {
        if (!activities.count(pi))
            activities[pi] = default_activity;
        // Look up user-specified static probability by net name
        double sp = 0.5;
        auto& name = nl.net(pi).name;
        if (!name.empty()) {
            auto it = user_static_probs.find(name);
            if (it != user_static_probs.end()) sp = it->second;
        }
        static_prob[pi] = sp;
    }

    // Initialize FF outputs
    for (auto gid : nl.flip_flops()) {
        auto& ff = nl.gate(gid);
        if (ff.output >= 0) {
            if (!activities.count(ff.output))
                activities[ff.output] = default_activity;
            // DFF output probability from D input, default 0.5
            double sp = 0.5;
            if (!ff.inputs.empty()) {
                auto it = static_prob.find(ff.inputs[0]);
                if (it != static_prob.end()) sp = it->second;
            }
            static_prob[ff.output] = sp;
        }
    }

    auto topo = nl.topo_order();

    for (auto gid : topo) {
        auto& g = nl.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT ||
            g.type == GateType::DFF || g.output < 0)
            continue;
        // Skip nets with user-supplied activity (from VCD/SAIF)
        if (activities.count(g.output)) {
            if (!static_prob.count(g.output))
                static_prob[g.output] = 0.5;
            continue;
        }

        if (g.inputs.empty()) {
            activities[g.output] = default_activity;
            static_prob[g.output] = 0.5;
            continue;
        }

        auto get_td = [&](NetId n) -> double {
            auto it = activities.find(n);
            return (it != activities.end()) ? it->second : default_activity;
        };
        auto get_sp = [&](NetId n) -> double {
            auto it = static_prob.find(n);
            return (it != static_prob.end()) ? it->second : 0.5;
        };

        double td_out = default_activity;
        double sp_out = 0.5;

        switch (g.type) {
            case GateType::BUF:
                td_out = get_td(g.inputs[0]);
                sp_out = get_sp(g.inputs[0]);
                break;

            case GateType::NOT:
                td_out = get_td(g.inputs[0]);
                sp_out = 1.0 - get_sp(g.inputs[0]);
                break;

            case GateType::AND:
            case GateType::NAND: {
                // TD(out) = sum_i( TD(i) * prod_{j!=i} P(j) )
                double sum_td = 0;
                double prob_all = 1.0;
                for (auto ni : g.inputs)
                    prob_all *= get_sp(ni);
                for (auto ni : g.inputs) {
                    double p_i = get_sp(ni);
                    double prod_others = (p_i > 0) ? prob_all / p_i : 0;
                    sum_td += get_td(ni) * prod_others;
                }
                td_out = sum_td;
                sp_out = prob_all;
                if (g.type == GateType::NAND) sp_out = 1.0 - sp_out;
                break;
            }

            case GateType::OR:
            case GateType::NOR: {
                // TD(out) = sum_i( TD(i) * prod_{j!=i} (1 - P(j)) )
                double comp_all = 1.0;
                for (auto ni : g.inputs)
                    comp_all *= (1.0 - get_sp(ni));
                double sum_td = 0;
                for (auto ni : g.inputs) {
                    double comp_i = 1.0 - get_sp(ni);
                    double prod_others = (comp_i > 0) ? comp_all / comp_i : 0;
                    sum_td += get_td(ni) * prod_others;
                }
                td_out = sum_td;
                sp_out = 1.0 - comp_all;
                if (g.type == GateType::NOR) sp_out = 1.0 - sp_out;
                break;
            }

            case GateType::XOR:
            case GateType::XNOR: {
                // TD(out) = sum( TD(i) )  (toggle rates add for XOR)
                double sum_td = 0;
                for (auto ni : g.inputs)
                    sum_td += get_td(ni);
                td_out = sum_td;
                // P(out) for 2-input XOR: P(a)(1-P(b)) + P(b)(1-P(a))
                if (g.inputs.size() == 2) {
                    double pa = get_sp(g.inputs[0]);
                    double pb = get_sp(g.inputs[1]);
                    sp_out = pa * (1.0 - pb) + pb * (1.0 - pa);
                } else {
                    sp_out = 0.5;
                }
                if (g.type == GateType::XNOR) sp_out = 1.0 - sp_out;
                break;
            }

            case GateType::MUX: {
                // MUX(sel, a, b): TD ≈ P(sel)*TD(a) + (1-P(sel))*TD(b) + |P(a)-P(b)|*TD(sel)
                if (g.inputs.size() >= 3) {
                    double td_s = get_td(g.inputs[0]);
                    double td_a = get_td(g.inputs[1]);
                    double td_b = get_td(g.inputs[2]);
                    double ps = get_sp(g.inputs[0]);
                    double pa = get_sp(g.inputs[1]);
                    double pb = get_sp(g.inputs[2]);
                    td_out = ps * td_a + (1.0 - ps) * td_b +
                             std::abs(pa - pb) * td_s;
                    sp_out = ps * pa + (1.0 - ps) * pb;
                } else {
                    double avg = 0;
                    for (auto ni : g.inputs) avg += get_td(ni);
                    td_out = avg / g.inputs.size();
                    sp_out = 0.5;
                }
                break;
            }

            default: {
                double avg = 0;
                for (auto ni : g.inputs) avg += get_td(ni);
                td_out = avg / g.inputs.size();
                sp_out = 0.5;
                break;
            }
        }

        td_out = std::max(0.0, std::min(1.0, td_out));
        sp_out = std::max(0.0, std::min(1.0, sp_out));
        activities[g.output] = td_out;
        static_prob[g.output] = sp_out;
    }
}

// ══════════════════════════════════════════════════════════════════════
// Enhanced Power Analysis
// ══════════════════════════════════════════════════════════════════════

// ── Set bulk activity data ───────────────────────────────────────────

void PowerAnalyzer::set_activity(const ActivityData& act) {
    activity_data_ = act;
    has_activity_data_ = true;

    // Map signal names to net IDs
    for (auto& [sig_name, toggle_rate] : act.toggle_rates) {
        for (size_t i = 0; i < nl_.num_nets(); ++i) {
            if (nl_.net(i).name == sig_name ||
                nl_.net(i).name.find(sig_name) != std::string::npos) {
                activities_[static_cast<NetId>(i)] = std::min(toggle_rate, 1.0);
                break;
            }
        }
    }
}

// ── RTL power estimation (early stage) ───────────────────────────────
// switching = 0.5 × α × Cload × V² × f for each gate
// Internal = from library cells. Leakage = sum of cell leakage.

PowerAnalyzer::RtlPowerResult PowerAnalyzer::estimate_rtl_power() {
    RtlPowerResult result{};
    double freq_hz = has_activity_data_ ? activity_data_.clock_freq_hz : 1e9;
    double freq_mhz = freq_hz / 1e6;
    double vdd = last_vdd_ > 0 ? last_vdd_ : 1.8;
    double default_alpha = last_default_activity_ > 0 ? last_default_activity_ : 0.1;

    // Run vectorless toggle rate propagation to fill in missing activities
    std::unordered_map<std::string, double> sp;
    if (has_activity_data_) sp = activity_data_.static_probs;
    propagate_toggle_rates(nl_, activities_, sp, default_alpha);

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;

        double alpha = default_alpha;
        if (g.output >= 0) {
            auto it = activities_.find(g.output);
            if (it != activities_.end()) alpha = it->second;
        }

        double cell_vdd = gate_vdd(static_cast<GateId>(gid), vdd);

        // Switching power: P_sw = 0.5 × α × C_load × V² × f
        double c_load = (g.output >= 0) ? net_capacitance(g.output) : 0.001;
        double p_sw = 0.5 * alpha * c_load * cell_vdd * cell_vdd * freq_mhz * 1e6 * 1e-9;
        result.switching_mw += p_sw;

        // Internal power: ~15% of switching (from short-circuit currents)
        double p_int = p_sw * 0.15;
        result.internal_mw += p_int;

        // Leakage
        double p_leak = cell_leakage(static_cast<GateId>(gid)) * 1e-6;
        p_leak *= (cell_vdd / vdd) * (cell_vdd / vdd);
        result.leakage_mw += p_leak;
    }

    result.total_mw = result.switching_mw + result.internal_mw + result.leakage_mw;
    result.message = "RTL power: " + std::to_string(result.total_mw) + " mW "
                     "(sw=" + std::to_string(result.switching_mw) +
                     " int=" + std::to_string(result.internal_mw) +
                     " leak=" + std::to_string(result.leakage_mw) + ")";
    return result;
}

// ── Clock tree power analysis ────────────────────────────────────────
// Identify clock network gates/buffers. Clock toggles at 2× (rise+fall).

PowerAnalyzer::ClockPowerResult PowerAnalyzer::analyze_clock_power() {
    ClockPowerResult result{};
    double freq_mhz = last_freq_mhz_ > 0 ? last_freq_mhz_ : 1000.0;
    double vdd = last_vdd_ > 0 ? last_vdd_ : 1.8;

    int num_dff = 0;
    double dff_clock_power = 0.0;

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF) {
            num_dff++;
            // DFF clock pin toggles every cycle (activity = 1.0 for clock input)
            double p_clk = cell_dynamic(static_cast<GateId>(gid), freq_mhz, vdd, 1.0);
            dff_clock_power += p_clk;
        }
    }

    // Clock tree buffer estimation: ~1 buffer per 8 DFFs
    int cts_buffers = (num_dff + 7) / 8;
    result.clock_buffers = cts_buffers;

    // Each CTS buffer: activity=2.0 (both edges), C_buf=5fF
    double c_buf = 0.005;
    double buf_power = cts_buffers * 2.0 * c_buf * vdd * vdd * freq_mhz * 1e6 * 1e-9;
    result.buffer_power_mw = buf_power;

    result.clock_network_mw = dff_clock_power + buf_power;

    // Compute total power for fraction calculation
    double total = 0.0;
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
        double alpha = 0.1;
        if (g.output >= 0) {
            auto it = activities_.find(g.output);
            if (it != activities_.end()) alpha = it->second;
        }
        total += cell_dynamic(static_cast<GateId>(gid), freq_mhz, vdd, alpha);
        total += cell_leakage(static_cast<GateId>(gid)) * 1e-6;
    }
    total += result.clock_network_mw;

    result.clock_fraction_pct = (total > 0) ? (result.clock_network_mw / total) * 100.0 : 0.0;
    return result;
}

// ── Memory power models ──────────────────────────────────────────────
// Identify DFF-based register files and estimate their power.

std::vector<PowerAnalyzer::MemoryPower> PowerAnalyzer::analyze_memory_power() {
    std::vector<MemoryPower> results;
    double freq_mhz = last_freq_mhz_ > 0 ? last_freq_mhz_ : 1000.0;
    double vdd = last_vdd_ > 0 ? last_vdd_ : 1.8;

    // Group DFFs by name prefix to identify register files
    std::unordered_map<std::string, std::vector<GateId>> reg_groups;
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type != GateType::DFF) continue;

        std::string prefix = g.name;
        // Strip trailing index: reg[0] → reg, mem_0 → mem
        auto bracket = prefix.find('[');
        if (bracket != std::string::npos) prefix = prefix.substr(0, bracket);
        auto underscore = prefix.rfind('_');
        if (underscore != std::string::npos) {
            std::string suffix = prefix.substr(underscore + 1);
            bool all_digits = !suffix.empty();
            for (char c : suffix) { if (!std::isdigit(c)) { all_digits = false; break; } }
            if (all_digits) prefix = prefix.substr(0, underscore);
        }
        if (prefix.empty()) prefix = "reg";
        reg_groups[prefix].push_back(static_cast<GateId>(gid));
    }

    // Each group with >1 DFF is a potential memory/register file
    for (auto& [name, dffs] : reg_groups) {
        if (dffs.size() < 2) continue;

        MemoryPower mp;
        mp.instance = name + "[" + std::to_string(dffs.size()) + "]";
        mp.read_power_mw = 0;
        mp.write_power_mw = 0;
        mp.leakage_mw = 0;

        for (auto gid : dffs) {
            double alpha = 0.1;
            auto& g = nl_.gate(gid);
            if (g.output >= 0) {
                auto it = activities_.find(g.output);
                if (it != activities_.end()) alpha = it->second;
            }
            double cell_vdd = gate_vdd(gid, vdd);
            // Read power: output switching
            mp.read_power_mw += cell_dynamic(gid, freq_mhz, cell_vdd, alpha) * 0.4;
            // Write power: input switching  
            mp.write_power_mw += cell_dynamic(gid, freq_mhz, cell_vdd, alpha) * 0.6;
            mp.leakage_mw += cell_leakage(gid) * 1e-6;
        }
        mp.total_mw = mp.read_power_mw + mp.write_power_mw + mp.leakage_mw;
        results.push_back(mp);
    }

    std::sort(results.begin(), results.end(),
              [](auto& a, auto& b) { return a.total_mw > b.total_mw; });
    return results;
}

// ── Per-instance power reporting ─────────────────────────────────────
// For each gate: compute switching + internal + leakage. Sort by total descending.

std::vector<PowerAnalyzer::InstancePower> PowerAnalyzer::report_instance_power() {
    std::vector<InstancePower> results;
    double freq_mhz = last_freq_mhz_ > 0 ? last_freq_mhz_ : 1000.0;
    double vdd = last_vdd_ > 0 ? last_vdd_ : 1.8;
    double default_alpha = last_default_activity_ > 0 ? last_default_activity_ : 0.1;

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;

        double alpha = default_alpha;
        if (g.output >= 0) {
            auto it = activities_.find(g.output);
            if (it != activities_.end()) alpha = it->second;
        }

        double cell_vdd = gate_vdd(static_cast<GateId>(gid), vdd);
        double leakage_mw = cell_leakage(static_cast<GateId>(gid)) * 1e-6;
        leakage_mw *= (cell_vdd / vdd) * (cell_vdd / vdd);

        double switching_mw = cell_dynamic(static_cast<GateId>(gid), freq_mhz, cell_vdd, alpha);
        double internal_mw = switching_mw * 0.15;

        InstancePower ip;
        ip.gate_id = static_cast<int>(gid);
        ip.name = g.name.empty() ? ("g" + std::to_string(gid)) : g.name;
        ip.cell_type = gate_type_str(g.type);
        ip.switching_mw = switching_mw;
        ip.internal_mw = internal_mw;
        ip.leakage_mw = leakage_mw;
        ip.total_mw = switching_mw + internal_mw + leakage_mw;
        results.push_back(ip);
    }

    std::sort(results.begin(), results.end(),
              [](auto& a, auto& b) { return a.total_mw > b.total_mw; });
    return results;
}

// ── Power state analysis (UPF-driven) ────────────────────────────────
// Compute power for each possible power state (all-on, each domain off, etc.)

std::vector<PowerAnalyzer::PowerState> PowerAnalyzer::analyze_power_states() {
    std::vector<PowerState> results;
    double freq_mhz = last_freq_mhz_ > 0 ? last_freq_mhz_ : 1000.0;
    double vdd = last_vdd_ > 0 ? last_vdd_ : 1.8;
    double default_alpha = last_default_activity_ > 0 ? last_default_activity_ : 0.1;

    // State 1: All domains active
    {
        PowerState ps;
        ps.state_name = "all_on";
        ps.total_power_mw = 0;
        for (auto& [name, info] : power_domains_) {
            ps.active_domains.push_back(name);
        }
        // Full power
        for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
            double alpha = default_alpha;
            if (g.output >= 0) {
                auto it = activities_.find(g.output);
                if (it != activities_.end()) alpha = it->second;
            }
            double cell_vdd = gate_vdd(static_cast<GateId>(gid), vdd);
            ps.total_power_mw += cell_dynamic(static_cast<GateId>(gid), freq_mhz, cell_vdd, alpha);
            ps.total_power_mw += cell_leakage(static_cast<GateId>(gid)) * 1e-6 *
                                 (cell_vdd / vdd) * (cell_vdd / vdd);
        }
        results.push_back(ps);
    }

    // State 2+: Each domain powered off individually
    for (auto& [off_domain, off_info] : power_domains_) {
        PowerState ps;
        ps.state_name = off_domain + "_off";
        ps.total_power_mw = 0;

        std::unordered_set<GateId> off_gates(off_info.gates.begin(), off_info.gates.end());
        for (auto& [name, info] : power_domains_) {
            if (name != off_domain) ps.active_domains.push_back(name);
        }

        for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
            if (off_gates.count(static_cast<GateId>(gid))) {
                // Powered-off domain: only retention leakage (~5% of normal)
                ps.total_power_mw += cell_leakage(static_cast<GateId>(gid)) * 1e-6 * 0.05;
                continue;
            }
            double alpha = default_alpha;
            if (g.output >= 0) {
                auto it = activities_.find(g.output);
                if (it != activities_.end()) alpha = it->second;
            }
            double cell_vdd = gate_vdd(static_cast<GateId>(gid), vdd);
            ps.total_power_mw += cell_dynamic(static_cast<GateId>(gid), freq_mhz, cell_vdd, alpha);
            ps.total_power_mw += cell_leakage(static_cast<GateId>(gid)) * 1e-6 *
                                 (cell_vdd / vdd) * (cell_vdd / vdd);
        }
        results.push_back(ps);
    }

    // If no domains defined, just report nominal state
    if (power_domains_.empty()) {
        PowerState ps;
        ps.state_name = "nominal";
        ps.active_domains.push_back("default");
        ps.total_power_mw = 0;
        for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
            double alpha = default_alpha;
            if (g.output >= 0) {
                auto it = activities_.find(g.output);
                if (it != activities_.end()) alpha = it->second;
            }
            ps.total_power_mw += cell_dynamic(static_cast<GateId>(gid), freq_mhz, vdd, alpha);
            ps.total_power_mw += cell_leakage(static_cast<GateId>(gid)) * 1e-6;
        }
        results.push_back(ps);
    }

    return results;
}

// ── Enhanced power run ───────────────────────────────────────────────

PowerResult PowerAnalyzer::run_enhanced() {
    double freq_mhz = last_freq_mhz_ > 0 ? last_freq_mhz_ : 1000.0;
    double vdd = last_vdd_ > 0 ? last_vdd_ : 1.8;
    double default_alpha = last_default_activity_ > 0 ? last_default_activity_ : 0.1;

    // 0. Vectorless toggle rate propagation (fills missing activities)
    std::unordered_map<std::string, double> sp;
    if (has_activity_data_) sp = activity_data_.static_probs;
    propagate_toggle_rates(nl_, activities_, sp, default_alpha);

    // 1. Run standard analysis (uses propagated activities)
    auto result = analyze(freq_mhz, vdd, default_alpha);

    // 2. Clock tree power (dedicated analysis)
    auto clock_result = analyze_clock_power();
    result.clock_power_mw = clock_result.clock_network_mw;

    // 3. RTL power estimation (early-stage cross-check)
    auto rtl = estimate_rtl_power();

    // 4. Memory power analysis
    auto mem = analyze_memory_power();
    double mem_total = 0;
    for (auto& m : mem) mem_total += m.total_mw;

    // 5. Power state analysis (UPF domains)
    auto states = analyze_power_states();

    // 6. Per-instance power reporting (for top-consumer refinement)
    auto instances = report_instance_power();
    if (!instances.empty()) {
        result.top_consumers.clear();
        int top_n = std::min(10, (int)instances.size());
        for (int i = 0; i < top_n; ++i) {
            result.top_consumers.push_back({
                instances[i].name,
                instances[i].switching_mw + instances[i].internal_mw,
                instances[i].leakage_mw,
                instances[i].total_mw
            });
        }
    }

    // Recalculate totals
    result.dynamic_power_mw = result.switching_power_mw + result.internal_power_mw +
                               result.glitch_power_mw + result.short_circuit_power_mw;
    result.total_power_mw = result.dynamic_power_mw + result.static_power_mw +
                             result.clock_power_mw;

    result.message = "Enhanced: " + std::to_string(result.total_power_mw) + " mW " +
                     "(dyn=" + std::to_string(result.dynamic_power_mw) +
                     " stat=" + std::to_string(result.static_power_mw) +
                     " clk=" + std::to_string(result.clock_power_mw) +
                     " glitch=" + std::to_string(result.glitch_power_mw) +
                     " short_circuit=" + std::to_string(result.short_circuit_power_mw) +
                     " mem=" + std::to_string(mem_total) +
                     " rtl_est=" + std::to_string(rtl.total_mw) +
                     " states=" + std::to_string(states.size()) + ")";
    return result;
}

// ══════════════════════════════════════════════════════════════════════
// Gate-Level Switching Activity Correlation
// ══════════════════════════════════════════════════════════════════════
// When multiple inputs of a gate share a common driver through reconvergent
// fanout, their switching activities are not independent.  The standard
// P(AB) = P(A)P(B) assumption overestimates toggle rate.  We perform a
// bounded BFS (depth <= 3 logic levels) from each gate input back through
// the driver graph to discover common ancestors, then compute a Pearson-
// style spatial correlation coefficient between each input pair.
//
// Power reduction model:
//   For each correlated pair (i,j) with coefficient rho_ij > threshold,
//   the effective joint toggle rate decreases by a factor of
//     (1 - rho_ij * min(alpha_i, alpha_j))
//   summed across all pairs and normalized by total switching power.
//
// Reference: Tsui, Pedram, Despain, "Efficient Estimation of Dynamic
//            Power Dissipation Under a Real Delay Model", ICCAD 1993

PowerAnalyzer::CorrelationResult
PowerAnalyzer::analyze_activity_correlation(double threshold) {
    CorrelationResult cr{};

    // Build reverse-driver map: for a given net, which gate drives it?
    // (Already available via Net::driver.)
    // For each gate, collect the set of ancestor nets reachable within
    // 3 logic levels through the driver chain.

    constexpr int MAX_BFS_DEPTH = 3;

    // Pre-compute ancestor sets lazily per net, caching results.
    // ancestor_map[net] = set of nets reachable by walking backwards
    // through drivers up to MAX_BFS_DEPTH levels.
    std::unordered_map<NetId, std::set<NetId>> ancestor_cache;

    auto collect_ancestors = [&](NetId seed) -> const std::set<NetId>& {
        auto cached = ancestor_cache.find(seed);
        if (cached != ancestor_cache.end()) return cached->second;

        std::set<NetId>& result = ancestor_cache[seed];
        // BFS backward through driver gates
        std::queue<std::pair<NetId, int>> frontier;
        frontier.push({seed, 0});
        while (!frontier.empty()) {
            auto [nid, depth] = frontier.front();
            frontier.pop();
            if (depth > MAX_BFS_DEPTH) continue;
            result.insert(nid);
            auto& n = nl_.net(nid);
            if (n.driver < 0) continue;
            auto& drv = nl_.gate(n.driver);
            if (drv.type == GateType::INPUT || drv.type == GateType::DFF)
                continue;
            for (auto inp : drv.inputs) {
                if (!result.count(inp))
                    frontier.push({inp, depth + 1});
            }
        }
        return result;
    };

    double total_rho = 0;
    int total_pairs = 0;
    double weighted_reduction = 0;
    double total_switching = 0;

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT ||
            g.type == GateType::DFF || g.inputs.size() < 2)
            continue;

        int n_in = static_cast<int>(g.inputs.size());
        bool gate_has_reconvergence = false;

        for (int i = 0; i < n_in; ++i) {
            const auto& anc_i = collect_ancestors(g.inputs[i]);
            double alpha_i = last_default_activity_;
            {
                auto it = activities_.find(g.inputs[i]);
                if (it != activities_.end()) alpha_i = it->second;
            }

            for (int j = i + 1; j < n_in; ++j) {
                const auto& anc_j = collect_ancestors(g.inputs[j]);
                double alpha_j = last_default_activity_;
                {
                    auto it = activities_.find(g.inputs[j]);
                    if (it != activities_.end()) alpha_j = it->second;
                }

                // Count shared ancestors (excluding the inputs themselves
                // if they happen to be the same net, which would be trivial).
                int shared = 0;
                int union_sz = 0;
                for (auto a : anc_i) {
                    if (anc_j.count(a)) ++shared;
                }
                union_sz = static_cast<int>(anc_i.size() + anc_j.size()) - shared;
                if (union_sz <= 0) continue;

                // Jaccard-style correlation: fraction of shared transitive
                // fanin within the BFS window.
                double rho = static_cast<double>(shared) / static_cast<double>(union_sz);

                total_rho += rho;
                ++total_pairs;

                if (rho > threshold) {
                    cr.correlated_pairs++;
                    if (!gate_has_reconvergence) {
                        cr.reconvergent_fanouts++;
                        gate_has_reconvergence = true;
                    }
                    // Power reduction contribution: the joint toggle rate
                    // overestimate from independence assumption is proportional
                    // to rho * min(alpha_i, alpha_j).
                    weighted_reduction += rho * std::min(alpha_i, alpha_j);
                }
            }
        }

        // Accumulate total switching for normalization.
        if (g.output >= 0) {
            double alpha = last_default_activity_;
            auto it = activities_.find(g.output);
            if (it != activities_.end()) alpha = it->second;
            total_switching += alpha;
        }
    }

    cr.avg_correlation = (total_pairs > 0)
        ? total_rho / static_cast<double>(total_pairs) : 0.0;
    cr.power_reduction_pct = (total_switching > 0)
        ? (weighted_reduction / total_switching) * 100.0 : 0.0;

    return cr;
}

// ══════════════════════════════════════════════════════════════════════
// SAF (Switching Activity File) I/O
// ══════════════════════════════════════════════════════════════════════
// Format is a simplified SAIF-like parenthesized structure.
// TC = toggle count over a canonical 1000-cycle simulation window.
// T0/T1 derived from static probability: T0 = (1-P(1))*1000, T1 = P(1)*1000.

bool PowerAnalyzer::write_saf(const std::string& filename) const {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) return false;

    constexpr int SIM_CYCLES = 1000;

    ofs << "(SAIF_VERSION \"2.0\")\n";
    ofs << "(DIRECTION \"backward\")\n";
    ofs << "(DESIGN (name \"top\")\n";

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT)
            continue;
        if (g.output < 0) continue;

        std::string gname = g.name.empty()
            ? ("gate_" + std::to_string(gid)) : g.name;

        double alpha = last_default_activity_;
        {
            auto it = activities_.find(g.output);
            if (it != activities_.end()) alpha = it->second;
        }

        // Static probability: approximate from activity.
        // For a signal with toggle rate alpha, P(1) ~ 0.5 when no
        // additional information is available (maximum entropy).  If
        // activity data carries explicit static probs, prefer those.
        double sp = 0.5;
        if (has_activity_data_) {
            auto& net_name = nl_.net(g.output).name;
            auto spit = activity_data_.static_probs.find(net_name);
            if (spit != activity_data_.static_probs.end())
                sp = spit->second;
        }

        int tc = static_cast<int>(alpha * SIM_CYCLES + 0.5);
        int t1 = static_cast<int>(sp * SIM_CYCLES + 0.5);
        int t0 = SIM_CYCLES - t1;
        int tx = 0;

        std::string nname = nl_.net(g.output).name.empty()
            ? ("net_" + std::to_string(g.output))
            : nl_.net(g.output).name;

        ofs << "  (INSTANCE (name \"" << gname << "\")\n";
        ofs << "    (NET (name \"" << nname << "\")\n";
        ofs << "      (T0 " << t0 << ") (T1 " << t1
            << ") (TX " << tx << ") (TC " << tc << ")\n";
        ofs << "    )\n";
        ofs << "  )\n";
    }

    ofs << ")\n";
    return ofs.good();
}

bool PowerAnalyzer::read_saf(const std::string& filename) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) return false;

    constexpr int SIM_CYCLES = 1000;
    std::string content((std::istreambuf_iterator<char>(ifs)),
                         std::istreambuf_iterator<char>());

    // Build name-to-net index for O(1) lookups.
    std::unordered_map<std::string, NetId> name_to_net;
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        auto& n = nl_.net(static_cast<NetId>(i));
        if (!n.name.empty())
            name_to_net[n.name] = static_cast<NetId>(i);
    }

    // Tokenize by scanning for INSTANCE/NET/T0/T1/TX/TC tokens inside
    // the parenthesized structure.  This is a hand-rolled recursive-descent
    // parser that tolerates whitespace and newline variations.
    size_t pos = 0;
    auto skip_ws = [&]() {
        while (pos < content.size() && (content[pos] == ' '  ||
               content[pos] == '\t' || content[pos] == '\n' ||
               content[pos] == '\r'))
            ++pos;
    };

    auto read_quoted = [&]() -> std::string {
        std::string result;
        skip_ws();
        if (pos < content.size() && content[pos] == '"') {
            ++pos;
            while (pos < content.size() && content[pos] != '"')
                result += content[pos++];
            if (pos < content.size()) ++pos; // consume closing quote
        }
        return result;
    };

    auto read_int = [&]() -> int {
        skip_ws();
        int val = 0;
        bool neg = false;
        if (pos < content.size() && content[pos] == '-') { neg = true; ++pos; }
        while (pos < content.size() && content[pos] >= '0' && content[pos] <= '9')
            val = val * 10 + (content[pos++] - '0');
        return neg ? -val : val;
    };

    auto find_token = [&](const char* tok) -> bool {
        size_t tlen = std::strlen(tok);
        size_t found = content.find(tok, pos);
        if (found == std::string::npos) return false;
        pos = found + tlen;
        return true;
    };

    // Scan for each INSTANCE block.
    while (find_token("(INSTANCE")) {
        // Expect (name "...")
        if (!find_token("(name")) continue;
        std::string inst_name = read_quoted();
        (void)inst_name; // instance name used for context, net name for mapping

        // Expect (NET (name "...") (T0 n) (T1 n) (TX n) (TC n))
        if (!find_token("(NET")) continue;
        if (!find_token("(name")) continue;
        std::string net_name = read_quoted();

        int t0 = 0, t1 = 0, tx = 0, tc = 0;
        // Parse T0, T1, TX, TC in any order within the NET block.
        // We look for the closing parentheses of NET to bound our search.
        size_t net_start = pos;
        // Find approximate end of this NET block (next INSTANCE or end).
        size_t net_end = content.find("(INSTANCE", pos);
        if (net_end == std::string::npos) net_end = content.size();
        std::string net_block = content.substr(net_start, net_end - net_start);

        auto extract_field = [&](const char* field) -> int {
            std::string tag = std::string("(") + field;
            size_t fp = net_block.find(tag);
            if (fp == std::string::npos) return 0;
            fp += tag.size();
            while (fp < net_block.size() && (net_block[fp] == ' ' || net_block[fp] == '\t'))
                ++fp;
            int v = 0;
            bool neg = false;
            if (fp < net_block.size() && net_block[fp] == '-') { neg = true; ++fp; }
            while (fp < net_block.size() && net_block[fp] >= '0' && net_block[fp] <= '9')
                v = v * 10 + (net_block[fp++] - '0');
            return neg ? -v : v;
        };

        t0 = extract_field("T0");
        t1 = extract_field("T1");
        tx = extract_field("TX");
        tc = extract_field("TC");

        // Convert TC to activity factor.
        double activity = (SIM_CYCLES > 0)
            ? static_cast<double>(tc) / static_cast<double>(SIM_CYCLES) : 0.0;
        activity = std::min(activity, 1.0);

        // Map to net by name.
        auto nit = name_to_net.find(net_name);
        if (nit != name_to_net.end()) {
            activities_[nit->second] = activity;
        } else {
            // Fallback: try stripping "net_" prefix and interpreting as NetId.
            if (net_name.size() > 4 && net_name.substr(0, 4) == "net_") {
                int nid = std::atoi(net_name.c_str() + 4);
                if (nid >= 0 && nid < static_cast<int>(nl_.num_nets()))
                    activities_[static_cast<NetId>(nid)] = activity;
            }
        }

        // Also populate static probability if activity data tracking is active.
        if (t0 + t1 > 0) {
            double sp = static_cast<double>(t1) / static_cast<double>(t0 + t1);
            auto& real_name = (nit != name_to_net.end())
                ? nl_.net(nit->second).name : net_name;
            if (!real_name.empty())
                activity_data_.static_probs[real_name] = sp;
        }

        pos = net_end;
    }

    has_activity_data_ = true;
    return true;
}

// ── Voltage derating for leakage power ───────────────────────────────
// Derate leakage based on IR-drop-reduced supply voltage per cell.
// P_leak_derated = P_leak * exp(-alpha * (Vdd - V_actual) / Vt)
// where alpha ≈ 1.0 (subthreshold slope factor), Vt ≈ 26mV at 25°C.

void PowerAnalyzer::apply_voltage_derating(const std::unordered_map<std::string, double>& cell_voltages) {
    constexpr double alpha = 1.0;  // subthreshold slope factor
    double vt = 0.026 * (temperature_c_ + 273.15) / 298.15;  // thermal voltage scaled by temperature

    voltage_derating_.clear();
    for (auto& [cell_name, v_actual] : cell_voltages) {
        double v_drop = last_vdd_ - v_actual;
        if (v_drop < 0) v_drop = 0;
        double factor = std::exp(-alpha * v_drop / vt);
        voltage_derating_[cell_name] = factor;
    }
}

// ── Temperature impact on leakage ────────────────────────────────────
// Leakage doubles roughly every 10°C: P_leak(T) = P_leak(25) * 2^((T-25)/10)

void PowerAnalyzer::set_temperature(double temp_celsius) {
    temperature_c_ = temp_celsius;
    leakage_temp_factor_ = std::pow(2.0, (temp_celsius - 25.0) / 10.0);
}

} // namespace sf
