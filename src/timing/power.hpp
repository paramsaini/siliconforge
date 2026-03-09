#pragma once
// SiliconForge — Power Analysis Engine
// Computes static (leakage) and dynamic (switching) power.
// Reference: Rabaey, Chandrakasan, Nikolic, "Digital Integrated Circuits", 2003

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

struct PowerResult {
    double total_power_mw = 0;
    double dynamic_power_mw = 0;
    double static_power_mw = 0;    // leakage
    double switching_power_mw = 0;
    double internal_power_mw = 0;
    double clock_power_mw = 0;
    double clock_freq_mhz = 0;
    double supply_voltage = 1.8;
    int num_cells = 0;
    double time_ms = 0;

    struct CellPower {
        std::string name;
        double dynamic = 0, leakage = 0, total = 0;
    };
    std::vector<CellPower> top_consumers;  // sorted by power
    std::string message;
};

class PowerAnalyzer {
public:
    PowerAnalyzer(const Netlist& nl, const LibertyLibrary* lib = nullptr)
        : nl_(nl), lib_(lib) {}

    // Run power analysis
    // switching_activity: fraction of clock cycles where each net toggles (0.0-1.0)
    PowerResult analyze(double clock_freq_mhz, double supply_voltage = 1.8,
                        double default_activity = 0.1);

    // Set per-net switching activity
    void set_activity(NetId net, double activity) { activities_[net] = activity; }

private:
    const Netlist& nl_;
    const LibertyLibrary* lib_;
    std::unordered_map<NetId, double> activities_;

    double cell_leakage(GateId gid) const;
    double cell_dynamic(GateId gid, double freq, double vdd, double activity) const;
    double net_capacitance(NetId nid) const;
};

} // namespace sf
