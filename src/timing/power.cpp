// SiliconForge — Power Analysis Implementation
#include "timing/power.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <cmath>

namespace sf {

double PowerAnalyzer::net_capacitance(NetId nid) const {
    // Estimate capacitance from fanout
    // C_total = C_wire + sum(C_input)
    auto& net = nl_.net(nid);
    double c_wire = 0.001; // 1 fF base wire cap
    double c_pin = 0.002;  // 2 fF per input pin
    return c_wire + net.fanout.size() * c_pin;
}

double PowerAnalyzer::cell_leakage(GateId gid) const {
    auto& g = nl_.gate(gid);
    if (lib_) {
        for (auto& cell : lib_->cells) {
            if (cell.name == g.name) return cell.leakage_power;
        }
    }
    // Default leakage model (nW)
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

double PowerAnalyzer::cell_dynamic(GateId gid, double freq, double vdd, double activity) const {
    // P_dynamic = α × C_load × V² × f
    auto& g = nl_.gate(gid);
    if (g.output < 0) return 0;

    double c_load = net_capacitance(g.output);
    double alpha = activity;

    // Check if user-specified activity
    auto it = activities_.find(g.output);
    if (it != activities_.end()) alpha = it->second;

    // P = alpha * C * V² * f  (in pW if C in pF, V in V, f in Hz)
    double p_pw = alpha * c_load * vdd * vdd * freq * 1e6; // freq in MHz → Hz
    return p_pw * 1e-9; // convert pW → mW
}

PowerResult PowerAnalyzer::analyze(double clock_freq_mhz, double supply_voltage,
                                     double default_activity) {
    auto t0 = std::chrono::high_resolution_clock::now();
    PowerResult result;
    result.clock_freq_mhz = clock_freq_mhz;
    result.supply_voltage = supply_voltage;

    // Analyze each gate
    std::vector<PowerResult::CellPower> cell_powers;

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;

        double leakage_mw = cell_leakage(gid) * 1e-6; // nW → mW
        double dynamic_mw = cell_dynamic(gid, clock_freq_mhz, supply_voltage, default_activity);

        // Internal power (short-circuit during transitions)
        double internal_mw = dynamic_mw * 0.15; // ~15% of dynamic as internal

        result.static_power_mw += leakage_mw;
        result.switching_power_mw += dynamic_mw;
        result.internal_power_mw += internal_mw;

        // Clock network power (DFFs toggle every cycle)
        if (g.type == GateType::DFF) {
            double clk_dyn = cell_dynamic(gid, clock_freq_mhz, supply_voltage, 1.0); // α=1 for clock
            result.clock_power_mw += clk_dyn;
        }

        cell_powers.push_back({
            g.name.empty() ? ("g" + std::to_string(gid)) : g.name,
            dynamic_mw + internal_mw, leakage_mw,
            dynamic_mw + internal_mw + leakage_mw
        });
        result.num_cells++;
    }

    result.dynamic_power_mw = result.switching_power_mw + result.internal_power_mw;
    result.total_power_mw = result.dynamic_power_mw + result.static_power_mw;

    // Top consumers
    std::sort(cell_powers.begin(), cell_powers.end(),
              [](auto& a, auto& b) { return a.total > b.total; });
    int top_n = std::min(10, (int)cell_powers.size());
    result.top_consumers.assign(cell_powers.begin(), cell_powers.begin() + top_n);

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = "Total: " + std::to_string(result.total_power_mw) + " mW " +
                     "(dynamic: " + std::to_string(result.dynamic_power_mw) +
                     ", static: " + std::to_string(result.static_power_mw) + ")";
    return result;
}

} // namespace sf
