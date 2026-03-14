#pragma once
// SiliconForge — SystemVerilog Elaboration Engine
// Parameterized modules, generate blocks, interfaces, modports at synthesis level.
// Reference: IEEE 1800-2017 §23 (Elaboration), §27 (Generate), §25 (Interfaces)

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

struct SvParameter {
    std::string name;
    int default_value = 0;
    std::string type;  // "integer", "string", "real"
    bool is_local = false;
};

struct SvGenerateBlock {
    enum Type { FOR_GENERATE, IF_GENERATE, CASE_GENERATE };
    Type type;
    std::string label;
    std::string condition;        // for if/case
    std::string iterator;         // for for-generate
    int start = 0, end = 0, step = 1;
    std::string body;             // module body to instantiate

    // Case-generate: condition value → body mapping
    std::unordered_map<int, std::string> case_items;
    std::string default_body;
};

struct SvInterface {
    std::string name;
    struct Signal {
        std::string name;
        int width;
        enum Dir { INPUT, OUTPUT, INOUT } dir;
    };
    std::vector<Signal> signals;
    struct Modport {
        std::string name;
        std::vector<std::pair<std::string, Signal::Dir>> port_dirs;
    };
    std::vector<Modport> modports;
};

struct SvModuleTemplate {
    std::string name;
    std::vector<SvParameter> parameters;
    std::vector<SvGenerateBlock> generates;
    std::vector<std::string> interface_ports;  // interface.modport port names
    std::string body;
};

struct ElaborateResult {
    bool success = false;
    int modules_elaborated = 0;
    int instances_generated = 0;
    int parameters_resolved = 0;
    int generate_blocks_expanded = 0;
    int interfaces_bound = 0;
    std::string error;
    std::string report;
};

class SvElaborator {
public:
    SvElaborator() = default;

    void add_module_template(const SvModuleTemplate& tmpl);
    void add_interface(const SvInterface& iface);
    void set_top_module(const std::string& name);
    void set_parameter(const std::string& module, const std::string& param, int value);

    ElaborateResult elaborate(Netlist& nl);

    // Parameter resolution: evaluate simple expressions with +, -, *, /, %, constants, param refs
    int resolve_parameter(const std::string& expr,
                          const std::unordered_map<std::string, int>& params);

    // Generate expansion
    int expand_for_generate(const SvGenerateBlock& gen, Netlist& nl,
                            const std::unordered_map<std::string, int>& params);
    int expand_if_generate(const SvGenerateBlock& gen, Netlist& nl,
                           const std::unordered_map<std::string, int>& params);
    int expand_case_generate(const SvGenerateBlock& gen, Netlist& nl,
                             const std::unordered_map<std::string, int>& params);

    // Full flow with detailed report
    ElaborateResult run_enhanced(Netlist& nl);

    // Accessors for testing
    const std::vector<SvModuleTemplate>& templates() const { return templates_; }
    const std::vector<SvInterface>& interfaces() const { return interfaces_; }

private:
    std::vector<SvModuleTemplate> templates_;
    std::vector<SvInterface> interfaces_;
    std::string top_module_;
    std::unordered_map<std::string, std::unordered_map<std::string, int>> param_overrides_;

    // Internal helpers
    void instantiate_body(const std::string& body, const std::string& prefix,
                          Netlist& nl, const std::unordered_map<std::string, int>& params);
    void bind_interfaces(const SvModuleTemplate& tmpl, Netlist& nl, ElaborateResult& res);
    void elaborate_recursive(const std::string& module_name, const std::string& prefix,
                             Netlist& nl, ElaborateResult& res,
                             const std::unordered_map<std::string, int>& param_ctx, int depth);
    const SvModuleTemplate* find_template(const std::string& name) const;
};

} // namespace sf
