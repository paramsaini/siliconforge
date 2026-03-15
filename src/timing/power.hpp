#pragma once
// SiliconForge — Power Analysis Engine
// Computes static (leakage), dynamic (switching), internal, glitch, and clock power.
// Reference: Rabaey, Chandrakasan, Nikolic, "Digital Integrated Circuits", 2003

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

namespace sf {

struct PowerResult {
    double total_power_mw = 0;
    double dynamic_power_mw = 0;
    double static_power_mw = 0;    // leakage
    double switching_power_mw = 0;
    double internal_power_mw = 0;
    double clock_power_mw = 0;
    double glitch_power_mw = 0;    // spurious transition power
    double short_circuit_power_mw = 0;  // mW — Isc during simultaneous PMOS/NMOS conduction
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
    PowerResult analyze(double clock_freq_mhz, double supply_voltage = 1.8,
                        double default_activity = 0.1);

    // Set per-net switching activity
    void set_activity(NetId net, double activity) { activities_[net] = activity; }

    // Set supply voltage domain for a gate
    void set_voltage_domain(GateId gid, double vdd) { voltage_domains_[gid] = vdd; }

    // Load VCD toggle counts for activity factors
    bool load_vcd(const std::string& filename);
    bool parse_vcd_string(const std::string& content);

    // Load SAIF activity factors
    bool load_saif(const std::string& filename);

    // UPF power intent
    void set_power_domain(const std::string& domain, double voltage, 
                          const std::vector<GateId>& gates) {
        power_domains_[domain] = {domain, voltage, gates};
        for (auto gid : gates) voltage_domains_[gid] = voltage;
    }

    // ── Enhanced power analysis features ─────────────────────────────

    // SAIF/VCD activity integration (bulk)
    struct ActivityData {
        std::unordered_map<std::string, double> toggle_rates;  // signal -> toggles/cycle
        std::unordered_map<std::string, double> static_probs;  // signal -> P(=1)
        double clock_freq_hz = 1e9;
    };
    void set_activity(const ActivityData& act);

    // RTL power estimation (early stage)
    struct RtlPowerResult {
        double total_mw;
        double switching_mw;
        double internal_mw;
        double leakage_mw;
        std::string message;
    };
    RtlPowerResult estimate_rtl_power();

    // Clock tree power
    struct ClockPowerResult {
        double clock_network_mw;
        double clock_fraction_pct;  // clock as % of total
        int clock_buffers;
        double buffer_power_mw;
    };
    ClockPowerResult analyze_clock_power();

    // Memory power models
    struct MemoryPower {
        std::string instance;
        double read_power_mw;
        double write_power_mw;
        double leakage_mw;
        double total_mw;
    };
    std::vector<MemoryPower> analyze_memory_power();

    // Per-instance power reporting
    struct InstancePower {
        int gate_id;
        std::string name;
        std::string cell_type;
        double switching_mw;
        double internal_mw;
        double leakage_mw;
        double total_mw;
    };
    std::vector<InstancePower> report_instance_power();

    // Power state (UPF-driven)
    struct PowerState {
        std::string state_name;
        std::vector<std::string> active_domains;
        double total_power_mw;
    };
    std::vector<PowerState> analyze_power_states();

    // Enhanced power run
    PowerResult run_enhanced();

    // Gate-level switching activity correlation
    // Tracks reconvergent fanout and input correlation to reduce pessimism
    struct CorrelationResult {
        int correlated_pairs = 0;       // pairs with correlation > threshold
        int reconvergent_fanouts = 0;   // reconvergent fanout points detected
        double avg_correlation = 0;     // average pairwise correlation
        double power_reduction_pct = 0; // estimated power reduction from correlation
    };
    CorrelationResult analyze_activity_correlation(double threshold = 0.3);

    // SAF (Switching Activity File) I/O
    bool write_saf(const std::string& filename) const;
    bool read_saf(const std::string& filename);

    // ── IR drop feedback to leakage power ────────────────────────────
    // Derate leakage based on per-cell actual voltage:
    // P_leak_derated = P_leak * exp(-alpha * (Vdd - V_actual) / Vt)
    void apply_voltage_derating(const std::unordered_map<std::string, double>& cell_voltages);

    // ── Temperature impact on leakage ────────────────────────────────
    // Leakage doubles roughly every 10°C: P_leak(T) = P_leak(25) * 2^((T-25)/10)
    void set_temperature(double temp_celsius);

private:
    const Netlist& nl_;
    const LibertyLibrary* lib_;
    std::unordered_map<NetId, double> activities_;
    std::unordered_map<GateId, double> voltage_domains_;
    std::unordered_map<NetId, int> topo_levels_; // for glitch estimation

    // Power domain tracking
    struct DomainInfo {
        std::string name;
        double voltage;
        std::vector<GateId> gates;
    };
    std::unordered_map<std::string, DomainInfo> power_domains_;

    // Activity data (enhanced)
    ActivityData activity_data_;
    bool has_activity_data_ = false;

    // Cached analysis parameters
    double last_freq_mhz_ = 0;
    double last_vdd_ = 1.8;
    double last_default_activity_ = 0.1;

    // Temperature and voltage derating state
    double temperature_c_ = 25.0;
    double leakage_temp_factor_ = 1.0;
    std::unordered_map<std::string, double> voltage_derating_;  // cell_name -> derating factor

    double cell_leakage(GateId gid) const;
    double cell_dynamic(GateId gid, double freq, double vdd, double activity) const;
    double net_capacitance(NetId nid) const;
    double glitch_activity(GateId gid, double base_activity) const;
    double gate_vdd(GateId gid, double default_vdd) const;
    double cell_short_circuit(GateId gid, double freq, double vdd, double activity) const;
};

} // namespace sf
