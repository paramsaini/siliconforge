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

    // Run standard analysis
    auto result = analyze(freq_mhz, vdd, default_alpha);

    // Augment with clock tree analysis
    auto clock_result = analyze_clock_power();
    result.clock_power_mw = clock_result.clock_network_mw;

    // Recalculate totals
    result.dynamic_power_mw = result.switching_power_mw + result.internal_power_mw +
                               result.glitch_power_mw;
    result.total_power_mw = result.dynamic_power_mw + result.static_power_mw +
                             result.clock_power_mw;

    result.message = "Enhanced: " + std::to_string(result.total_power_mw) + " mW " +
                     "(dyn=" + std::to_string(result.dynamic_power_mw) +
                     " stat=" + std::to_string(result.static_power_mw) +
                     " clk=" + std::to_string(result.clock_power_mw) +
                     " glitch=" + std::to_string(result.glitch_power_mw) + ")";
    return result;
}

} // namespace sf
