#pragma once
// SiliconForge — UPF 2.1 (IEEE 1801) Power Intent Parser
// Parses Unified Power Format for power domain specification, isolation,
// level shifting, retention, and power state management.

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <sstream>

namespace sf {

// ── Supply modeling ─────────────────────────────────────────────────────

struct SupplyNet {
    std::string name;
    double voltage = 0.0;
    bool is_resolved = false;
    std::string connected_supply_port;
};

struct SupplyPort {
    std::string name;
    enum Direction { IN, OUT } direction = IN;
    std::string domain;
};

// ── Power domain ────────────────────────────────────────────────────────

struct PowerDomain {
    std::string name;
    std::string parent_domain;
    std::vector<std::string> elements;
    std::string primary_power_net;
    std::string primary_ground_net;
    std::vector<std::string> supply_nets;
    bool is_top = false;
    double voltage = 1.0;
};

// ── Isolation ───────────────────────────────────────────────────────────

struct IsolationStrategy {
    std::string name;
    std::string domain;
    std::string isolation_power_net;
    std::string isolation_ground_net;
    enum IsolationType { CLAMP_0, CLAMP_1, LATCH, AND, OR } type = CLAMP_0;
    std::string applies_to;       // "inputs", "outputs", "both"
    std::string isolation_signal;
    enum Sense { HIGH, LOW } sense = HIGH;
    std::string location;         // "self", "parent", "fanout"
};

// ── Level shifter ───────────────────────────────────────────────────────

struct LevelShifter {
    std::string name;
    std::string domain;
    std::string applies_to;       // "inputs", "outputs", "both"
    std::string rule;             // "low_to_high", "high_to_low", "both"
    std::string location;
    double input_voltage = 0.0;
    double output_voltage = 0.0;
    std::string threshold;        // "HL" or "LH"
};

// ── Retention ───────────────────────────────────────────────────────────

struct RetentionStrategy {
    std::string name;
    std::string domain;
    std::string retention_power_net;
    std::string retention_ground_net;
    std::string save_signal;
    std::string restore_signal;
    enum SaveCondition { EDGE, LEVEL } save_condition = EDGE;
    std::vector<std::string> elements;
};

// ── Power state table ───────────────────────────────────────────────────

struct PowerState {
    std::string name;
    std::unordered_map<std::string, double> supply_voltages; // -1 → OFF
    std::string simstate; // "NORMAL", "CORRUPT", "RESET", "HOLD"
};

struct PowerStateTable {
    std::string name;
    std::vector<std::string> supply_names;
    std::vector<PowerState> states;
};

// ── Advanced isolation with multi-supply ─────────────────────────────────

struct UpfIsolationStrategy {
    std::string name;
    std::string domain;
    std::string isolation_signal;
    std::string clamp_value;       // "0", "1", "latch", "high_impedance"
    std::string location;          // "parent", "self", "fanout"
};

// ── Top-level UPF data model ────────────────────────────────────────────

struct UpfDesign {
    std::string version = "2.1";
    std::string scope;
    std::unordered_map<std::string, PowerDomain> domains;
    std::unordered_map<std::string, SupplyNet> supply_nets;
    std::unordered_map<std::string, SupplyPort> supply_ports;
    std::vector<IsolationStrategy> isolations;
    std::vector<LevelShifter> level_shifters;
    std::vector<RetentionStrategy> retentions;
    std::vector<PowerStateTable> state_tables;
    std::vector<UpfIsolationStrategy> advanced_isolations;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    const PowerDomain* find_domain(const std::string& name) const;
    const SupplyNet* find_supply(const std::string& name) const;
    std::vector<std::string> get_domain_hierarchy(const std::string& domain) const;
    bool validate() const;
    std::string to_upf() const;
    std::string report() const;
};

// ── Parser ──────────────────────────────────────────────────────────────

class UpfParser {
public:
    UpfParser() = default;

    UpfDesign parse_file(const std::string& filename);
    UpfDesign parse_string(const std::string& content);

private:
    struct Command {
        std::vector<std::string> args;
        int line_number = 0;
    };

    // TCL-like tokeniser (handles \-continuation, {braces}, "quotes")
    std::vector<Command> split_into_commands(const std::string& content);
    std::vector<std::string> tokenize(const std::string& line);

    // Flag helpers
    static std::string get_flag(const std::vector<std::string>& args,
                                const std::string& flag,
                                const std::string& default_val = "");
    static std::vector<std::string> get_flag_list(const std::vector<std::string>& args,
                                                  const std::string& flag);
    static bool has_flag(const std::vector<std::string>& args,
                         const std::string& flag);

    // Per-command parsers
    void parse_create_power_domain(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_create_supply_net(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_create_supply_port(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_connect_supply_net(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_set_domain_supply_net(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_set_isolation(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_set_level_shifter(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_set_retention(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_add_power_state(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_create_pst(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_add_pst_state(const std::vector<std::string>& args, UpfDesign& upf, int line);
    void parse_set_isolation_strategy(const std::vector<std::string>& args, UpfDesign& upf, int line);

    void error(UpfDesign& upf, int line, const std::string& msg);
    void warn(UpfDesign& upf, int line, const std::string& msg);
};

// ── Checker (DRC for power intent) ──────────────────────────────────────

class UpfChecker {
public:
    std::vector<std::string> check(const UpfDesign& upf, const Netlist& nl);

private:
    void check_domain_supplies(const UpfDesign& upf, std::vector<std::string>& issues);
    void check_isolation_refs(const UpfDesign& upf, std::vector<std::string>& issues);
    void check_level_shifters(const UpfDesign& upf, std::vector<std::string>& issues);
    void check_retention_coverage(const UpfDesign& upf, std::vector<std::string>& issues);
    void check_supply_consistency(const UpfDesign& upf, std::vector<std::string>& issues);
    void check_pst_validity(const UpfDesign& upf, std::vector<std::string>& issues);
    void check_netlist_coverage(const UpfDesign& upf, const Netlist& nl,
                                std::vector<std::string>& issues);
};

} // namespace sf
