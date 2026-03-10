// SiliconForge — Gate-Level Netlist Implementation
#include "core/netlist.hpp"
#include <queue>
#include <algorithm>
#include <iostream>
#include <cassert>

namespace sf {

NetId Netlist::add_net(const std::string& name) {
    NetId id = (NetId)nets_.size();
    nets_.push_back({id, name.empty() ? ("n" + std::to_string(id)) : name});
    return id;
}

GateId Netlist::add_gate(GateType type, const std::vector<NetId>& inputs,
                          NetId output, const std::string& name) {
    GateId id = (GateId)gates_.size();
    Gate g{id, type, name.empty() ? (std::string(gate_type_str(type)) + std::to_string(id)) : name,
           inputs, output};
    gates_.push_back(g);

    // Connect fanout
    for (auto ni : inputs)
        nets_[ni].fanout.push_back(id);
    if (output >= 0)
        nets_[output].driver = id;

    return id;
}

GateId Netlist::add_dff(NetId d, NetId clk, NetId q, NetId reset,
                         const std::string& name) {
    GateId id = (GateId)gates_.size();
    Gate g{id, GateType::DFF, name.empty() ? ("DFF" + std::to_string(id)) : name,
           {d}, q};
    g.clk = clk;
    g.reset = reset;
    gates_.push_back(g);
    nets_[d].fanout.push_back(id);
    nets_[clk].fanout.push_back(id);
    if (reset >= 0) nets_[reset].fanout.push_back(id);
    if (q >= 0) nets_[q].driver = id;
    dffs_.push_back(id);
    return id;
}

void Netlist::mark_input(NetId id) { pis_.push_back(id); }
void Netlist::mark_output(NetId id) { pos_.push_back(id); }

Logic4 Netlist::eval_gate(GateType type, const std::vector<Logic4>& in) {
    switch (type) {
        case GateType::BUF:   return in[0];
        case GateType::NOT:   return logic_not(in[0]);
        case GateType::AND:   { Logic4 r = Logic4::ONE; for (auto v : in) r = logic_and(r, v); return r; }
        case GateType::OR:    { Logic4 r = Logic4::ZERO; for (auto v : in) r = logic_or(r, v); return r; }
        case GateType::NAND:  { Logic4 r = Logic4::ONE; for (auto v : in) r = logic_and(r, v); return logic_not(r); }
        case GateType::NOR:   { Logic4 r = Logic4::ZERO; for (auto v : in) r = logic_or(r, v); return logic_not(r); }
        case GateType::XOR:   return logic_xor(in[0], in[1]);
        case GateType::XNOR:  return logic_not(logic_xor(in[0], in[1]));
        case GateType::MUX:   return (in[0] == Logic4::ONE) ? in[1] :
                               (in[0] == Logic4::ZERO) ? in[2] : Logic4::X;
        case GateType::CONST0: return Logic4::ZERO;
        case GateType::CONST1: return Logic4::ONE;
        default: return Logic4::X;
    }
}

std::vector<GateId> Netlist::topo_order() const {
    // Kahn's algorithm — only combinational gates
    std::vector<int> in_degree(gates_.size(), 0);
    for (auto& g : gates_) {
        if (g.type == GateType::DFF || g.type == GateType::INPUT) continue;
        for (auto ni : g.inputs) {
            if (nets_[ni].driver >= 0) {
                auto& drv = gates_[nets_[ni].driver];
                if (drv.type != GateType::DFF && drv.type != GateType::INPUT)
                    in_degree[g.id]++;
            }
        }
    }

    std::queue<GateId> q;
    for (auto& g : gates_) {
        if (g.type == GateType::DFF || g.type == GateType::INPUT) continue;
        if (in_degree[g.id] == 0) q.push(g.id);
    }

    std::vector<GateId> order;
    while (!q.empty()) {
        GateId gid = q.front(); q.pop();
        order.push_back(gid);
        auto& g = gates_[gid];
        if (g.output >= 0) {
            for (auto fo_gid : nets_[g.output].fanout) {
                if (gates_[fo_gid].type == GateType::DFF) continue;
                if (--in_degree[fo_gid] == 0) q.push(fo_gid);
            }
        }
    }
    return order;
}

void Netlist::print_stats() const {
    int comb = 0, seq = 0;
    for (auto& g : gates_) {
        if (g.type == GateType::DFF) seq++;
        else if (g.type != GateType::INPUT && g.type != GateType::OUTPUT) comb++;
    }
    std::cout << "Netlist: " << nets_.size() << " nets, " << comb << " comb gates, "
              << seq << " FFs, " << pis_.size() << " PIs, " << pos_.size() << " POs\n";
}

void Netlist::clear() {
    nets_.clear();
    gates_.clear();
    pis_.clear();
    pos_.clear();
    dffs_.clear();
}

} // namespace sf
