#pragma once
// SiliconForge — Hierarchical Design Manager
// Manages multi-module designs with instantiation hierarchy,
// top-down partitioning, and bottom-up assembly.
// Supports: instance tree, block abstraction, parameterized modules, selective flattening.
// Reference: Keutzer et al., "System-Level Design: Orthogonalization of Concerns", IEEE 2000

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <variant>

namespace sf {

// ── Module port definition ──
struct ModulePort {
    std::string name;
    enum Direction { IN, OUT, INOUT } direction = IN;
    int width = 1;
    NetId net_id = -1; // internal net connected to this port
};

// ── Timing arc between ports (for block abstraction) ──
struct HierTimingArc {
    std::string from_port;
    std::string to_port;
    double delay_ps = 0;
    double slew_ps = 0;
    std::string timing_sense; // "positive_unate", "negative_unate", "non_unate"
};

// ── Block-level abstraction model ──
struct BlockModel {
    std::vector<ModulePort> ports;
    std::vector<HierTimingArc> timing_arcs;
    double area_um2 = 0;
    double static_power_mw = 0;
    double dynamic_power_mw_per_mhz = 0;
    bool valid = false;
};

// ── Parameter value (int, double, or string) ──
struct ParamValue {
    enum Type { INTEGER, REAL, STRING } type = INTEGER;
    int int_val = 0;
    double real_val = 0;
    std::string str_val;

    ParamValue() = default;
    explicit ParamValue(int v) : type(INTEGER), int_val(v) {}
    explicit ParamValue(double v) : type(REAL), real_val(v) {}
    explicit ParamValue(const std::string& v) : type(STRING), str_val(v) {}
};

// ── Module instance (an instantiation of a module within a parent) ──
struct ModuleInstance {
    std::string instance_name;  // unique within parent (e.g., "alu_0")
    std::string module_name;    // which module this instantiates
    std::unordered_map<std::string, ParamValue> params; // parameter overrides
    std::unordered_map<std::string, std::string> port_map; // port_name → net_name
    Rect placement_rect;        // physical location in parent
    bool is_flattened = false;  // has this instance been inlined?
};

// ── Design module ──
struct DesignModule {
    std::string name;
    Netlist netlist;
    PhysicalDesign physical;
    bool is_leaf = true;
    std::vector<std::string> sub_modules; // backward compat: child module names
    std::vector<ModuleInstance> instances; // proper instance tree
    std::vector<ModulePort> ports;        // module interface
    std::unordered_map<std::string, ParamValue> default_params; // defaults
    BlockModel block_model;               // timing abstraction
    bool use_abstract_timing = false;
    double area_um2 = 0;
    int gate_count = 0;
};

struct HierarchyResult {
    int total_modules = 0;
    int leaf_modules = 0;
    int hierarchy_depth = 0;
    double total_area = 0;
    int total_gates = 0;
    int total_instances = 0;
    double time_ms = 0;
    std::string message;
};

// ── Selective flattening options ──
enum class FlattenPolicy {
    FLATTEN_ALL,           // flatten everything (default, backward compat)
    DEPTH_LIMITED,         // flatten up to max_depth levels
    SELECTIVE              // preserve specific modules, flatten rest
};

struct FlattenOptions {
    FlattenPolicy policy = FlattenPolicy::FLATTEN_ALL;
    int max_depth = -1;
    std::unordered_set<std::string> preserve_modules;
    bool preserve_hierarchy_names = true;
};

class HierarchyManager {
public:
    // ── Module registration ──
    void add_module(const std::string& name, const Netlist& nl);
    void add_module(const std::string& name, const Netlist& nl, const PhysicalDesign& pd);

    // Define module interface (ports)
    void define_ports(const std::string& module_name, const std::vector<ModulePort>& ports);

    // Define default parameters
    void define_params(const std::string& module_name,
                       const std::unordered_map<std::string, ParamValue>& defaults);

    // ── Instantiation ──
    // Backward-compatible: adds to both sub_modules and instances
    void instantiate(const std::string& parent, const std::string& child,
                     const std::string& instance_name = "");

    // Parameterized instantiation
    ModuleInstance& instantiate_with_params(
        const std::string& parent, const std::string& child,
        const std::string& instance_name,
        const std::unordered_map<std::string, ParamValue>& params);

    // Connect instance port to parent net
    void connect_port(const std::string& parent, const std::string& instance_name,
                      const std::string& port_name, const std::string& net_name);

    // ── Block abstraction ──
    void add_timing_arc(const std::string& module_name,
                        const std::string& from_port, const std::string& to_port,
                        double delay_ps, double slew_ps = 0);
    void set_block_power(const std::string& module_name,
                         double static_mw, double dynamic_mw_per_mhz);
    BlockModel generate_block_model(const std::string& module_name) const;

    // ── Analysis ──
    HierarchyResult analyze();

    // ── Flattening ──
    Netlist flatten(const std::string& top_module);
    Netlist flatten(const std::string& top_module, const FlattenOptions& opts);

    // ── Queries ──
    DesignModule* get_module(const std::string& name);
    const DesignModule* get_module(const std::string& name) const;
    int instance_count(const std::string& module_name) const;
    std::vector<const ModuleInstance*> get_instances_of(const std::string& module_name) const;
    std::string hierarchy_tree(const std::string& top, int indent = 0) const;

private:
    std::unordered_map<std::string, DesignModule> modules_;
    int compute_depth(const std::string& mod, int current = 0) const;
    void flatten_recursive(const std::string& mod_name, const std::string& prefix,
                           int depth, const FlattenOptions& opts, Netlist& flat) const;
};

} // namespace sf
