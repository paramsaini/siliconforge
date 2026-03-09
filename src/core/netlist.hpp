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
    MUX, DFF, DLATCH, TRI, CONST0, CONST1
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

private:
    std::vector<Net> nets_;
    std::vector<Gate> gates_;
    std::vector<NetId> pis_, pos_;
    std::vector<GateId> dffs_;
};

} // namespace sf
