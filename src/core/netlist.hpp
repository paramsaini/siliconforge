#pragma once
// SiliconForge — Gate-Level Netlist for Simulation
// Represents a flattened gate-level circuit with 4-state logic support.
// Used by the event-driven simulator, lint engine, and fault simulator.

#include "core/types.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>

namespace sf {

enum class GateType {
    INPUT, OUTPUT, BUF, NOT,
    AND, OR, NAND, NOR, XOR, XNOR,
    MUX, DFF, DLATCH, TRI, CONST0, CONST1,
    BUFIF0, BUFIF1, NOTIF0, NOTIF1
};

inline const char* gate_type_str(GateType t) {
    switch (t) {
        case GateType::INPUT:  return "INPUT";
        case GateType::OUTPUT: return "OUTPUT";
        case GateType::BUF:    return "BUF";
        case GateType::NOT:    return "NOT";
        case GateType::AND:    return "AND";
        case GateType::OR:     return "OR";
        case GateType::NAND:   return "NAND";
        case GateType::NOR:    return "NOR";
        case GateType::XOR:    return "XOR";
        case GateType::XNOR:   return "XNOR";
        case GateType::MUX:    return "MUX";
        case GateType::DFF:    return "DFF";
        case GateType::DLATCH: return "DLATCH";
        case GateType::TRI:    return "TRI";
        case GateType::CONST0: return "CONST0";
        case GateType::CONST1: return "CONST1";
        case GateType::BUFIF0: return "BUFIF0";
        case GateType::BUFIF1: return "BUFIF1";
        case GateType::NOTIF0: return "NOTIF0";
        case GateType::NOTIF1: return "NOTIF1";
    }
    return "?";
}

using NetId = int32_t;
using GateId = int32_t;

struct Net {
    NetId id;
    std::string name;
    Logic4 value = Logic4::X;
    Logic4 next_value = Logic4::X;
    std::vector<GateId> fanout; // gates driven by this net
    GateId driver = -1;         // gate that drives this net
};

struct Gate {
    GateId id;
    GateType type;
    std::string name;
    std::vector<NetId> inputs;
    NetId output = -1;
    // For DFF:
    NetId clk = -1;
    NetId reset = -1;
    Logic4 init_val = Logic4::ZERO;
};

class Netlist {
public:
    NetId add_net(const std::string& name = "");
    GateId add_gate(GateType type, const std::vector<NetId>& inputs, NetId output,
                    const std::string& name = "");
    GateId add_dff(NetId d, NetId clk, NetId q, NetId reset = -1,
                   const std::string& name = "");

    void mark_input(NetId id);
    void mark_output(NetId id);

    Net& net(NetId id) { return nets_[id]; }
    const Net& net(NetId id) const { return nets_[id]; }
    Gate& gate(GateId id) { return gates_[id]; }
    const Gate& gate(GateId id) const { return gates_[id]; }

    size_t num_nets() const { return nets_.size(); }
    size_t num_gates() const { return gates_.size(); }
    const std::vector<NetId>& primary_inputs() const { return pis_; }
    const std::vector<NetId>& primary_outputs() const { return pos_; }
    const std::vector<GateId>& flip_flops() const { return dffs_; }
    const std::vector<Net>& nets() const { return nets_; }
    const std::vector<Gate>& gates() const { return gates_; }

    // Compute topological order of combinational gates
    std::vector<GateId> topo_order() const;

    // Evaluate a single gate's output
    static Logic4 eval_gate(GateType type, const std::vector<Logic4>& inputs);

    void print_stats() const;
    void clear();

private:
    std::vector<Net> nets_;
    std::vector<Gate> gates_;
    std::vector<NetId> pis_, pos_;
    std::vector<GateId> dffs_;
};

// ============================================================================
// Hierarchical Netlist Support
// Enables block-level design with module definitions and instantiation.
// Reference: IEEE 1364/1800 module hierarchy semantics
// ============================================================================

// A port on a module boundary
struct ModulePort {
    std::string name;
    enum Direction { INPUT, OUTPUT, INOUT } direction = INPUT;
    int width = 1;  // bus width (1 = scalar)
    NetId net_id = -1; // internal net connected to this port
};

// A hierarchical module definition
struct ModuleDef {
    std::string name;
    std::vector<ModulePort> ports;
    Netlist internal_netlist;  // flat netlist inside this module

    // Sub-instances within this module
    struct Instance {
        std::string inst_name;
        std::string module_name;  // references another ModuleDef
        // Port connections: module_port_name → parent_net_name
        std::unordered_map<std::string, std::string> connections;
    };
    std::vector<Instance> instances;

    // Port lookup
    const ModulePort* find_port(const std::string& name) const;
    int port_index(const std::string& name) const;
};

// Hierarchical design database
class HierarchicalNetlist {
public:
    // Module management
    ModuleDef& add_module(const std::string& name);
    const ModuleDef* find_module(const std::string& name) const;
    ModuleDef* find_module_mut(const std::string& name);

    // Set top-level module
    void set_top(const std::string& name);
    const std::string& top_module() const { return top_module_; }

    // Add an instance of module_name inside parent_module
    void add_instance(const std::string& parent_module,
                      const std::string& inst_name,
                      const std::string& module_name,
                      const std::unordered_map<std::string, std::string>& connections);

    // Flatten hierarchy into a single Netlist
    // Prefixes all internal names with hierarchy path (e.g., "top/u_core/g0")
    Netlist flatten() const;

    // Elaborate: resolve parameters, generate blocks (future)
    bool elaborate();

    // Hierarchy queries
    int depth() const;  // maximum instantiation depth
    int total_instances() const;
    std::vector<std::string> module_names() const;

    // Hierarchy path for a net (given flat net name, return hierarchy path)
    std::string net_hierarchy_path(const std::string& flat_name) const;

    // Block abstraction: create a timing model for a module
    struct BlockAbstract {
        std::string module_name;
        std::vector<ModulePort> ports;
        // Port-to-port delay arcs
        struct DelayArc {
            std::string from_port;
            std::string to_port;
            double delay_rise = 0;
            double delay_fall = 0;
        };
        std::vector<DelayArc> arcs;
        double area = 0;
        double leakage_power = 0;
    };
    BlockAbstract create_abstract(const std::string& module_name) const;

    void print_stats() const;

private:
    std::vector<ModuleDef> modules_;
    std::unordered_map<std::string, int> module_map_;  // name → index
    std::string top_module_;

    // Recursive flatten helper
    void flatten_recursive(const std::string& prefix,
                          const ModuleDef& mod,
                          Netlist& result,
                          std::unordered_map<std::string, NetId>& net_map) const;

    int depth_recursive(const ModuleDef& mod, int current) const;
    int count_instances_recursive(const ModuleDef& mod) const;
};

} // namespace sf
