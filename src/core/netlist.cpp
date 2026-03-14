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
        // bufif1(data, enable): output = enable ? data : Z
        case GateType::BUFIF1: return (in.size() >= 2 && in[1] == Logic4::ONE) ? in[0] : Logic4::X;
        // bufif0(data, enable): output = !enable ? data : Z
        case GateType::BUFIF0: return (in.size() >= 2 && in[1] == Logic4::ZERO) ? in[0] : Logic4::X;
        // notif1(data, enable): output = enable ? ~data : Z
        case GateType::NOTIF1: return (in.size() >= 2 && in[1] == Logic4::ONE) ? logic_not(in[0]) : Logic4::X;
        // notif0(data, enable): output = !enable ? ~data : Z
        case GateType::NOTIF0: return (in.size() >= 2 && in[1] == Logic4::ZERO) ? logic_not(in[0]) : Logic4::X;
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

// ═══════════════════════════════════════════════════════════════════════════
// ModuleDef
// ═══════════════════════════════════════════════════════════════════════════

const ModulePort* ModuleDef::find_port(const std::string& name) const {
    for (auto& p : ports)
        if (p.name == name) return &p;
    return nullptr;
}

int ModuleDef::port_index(const std::string& name) const {
    for (int i = 0; i < (int)ports.size(); ++i)
        if (ports[i].name == name) return i;
    return -1;
}

// ═══════════════════════════════════════════════════════════════════════════
// HierarchicalNetlist — Module management
// ═══════════════════════════════════════════════════════════════════════════

ModuleDef& HierarchicalNetlist::add_module(const std::string& name) {
    int idx = (int)modules_.size();
    modules_.push_back({});
    modules_.back().name = name;
    module_map_[name] = idx;
    return modules_.back();
}

const ModuleDef* HierarchicalNetlist::find_module(const std::string& name) const {
    auto it = module_map_.find(name);
    if (it == module_map_.end()) return nullptr;
    return &modules_[it->second];
}

ModuleDef* HierarchicalNetlist::find_module_mut(const std::string& name) {
    auto it = module_map_.find(name);
    if (it == module_map_.end()) return nullptr;
    return &modules_[it->second];
}

void HierarchicalNetlist::set_top(const std::string& name) {
    top_module_ = name;
}

void HierarchicalNetlist::add_instance(const std::string& parent_module,
                                        const std::string& inst_name,
                                        const std::string& module_name,
                                        const std::unordered_map<std::string, std::string>& connections) {
    auto* parent = find_module_mut(parent_module);
    if (!parent) return;
    ModuleDef::Instance inst;
    inst.inst_name = inst_name;
    inst.module_name = module_name;
    inst.connections = connections;
    parent->instances.push_back(std::move(inst));
}

// ═══════════════════════════════════════════════════════════════════════════
// HierarchicalNetlist — Flatten
// ═══════════════════════════════════════════════════════════════════════════

Netlist HierarchicalNetlist::flatten() const {
    Netlist result;
    if (top_module_.empty()) return result;
    const ModuleDef* top = find_module(top_module_);
    if (!top) return result;

    std::unordered_map<std::string, NetId> net_map;
    flatten_recursive(top_module_, *top, result, net_map);
    return result;
}

void HierarchicalNetlist::flatten_recursive(
    const std::string& prefix,
    const ModuleDef& mod,
    Netlist& result,
    std::unordered_map<std::string, NetId>& net_map) const {

    const Netlist& nl = mod.internal_netlist;

    // Create nets for this module's internal netlist
    std::unordered_map<NetId, NetId> local_net_remap;
    for (size_t i = 0; i < nl.num_nets(); ++i) {
        const Net& n = nl.net((NetId)i);
        std::string flat_name = prefix + "/" + n.name;

        // Check if this net was already aliased by a parent's port connection
        auto alias_it = net_map.find(flat_name);
        if (alias_it != net_map.end()) {
            local_net_remap[(NetId)i] = alias_it->second;
        } else {
            NetId new_id = result.add_net(flat_name);
            net_map[flat_name] = new_id;
            local_net_remap[(NetId)i] = new_id;
        }
    }

    // Mark primary inputs/outputs
    for (auto pi : nl.primary_inputs()) {
        if (local_net_remap.count(pi))
            result.mark_input(local_net_remap[pi]);
    }
    for (auto po : nl.primary_outputs()) {
        if (local_net_remap.count(po))
            result.mark_output(local_net_remap[po]);
    }

    // Create gates (remap net IDs)
    for (size_t g = 0; g < nl.num_gates(); ++g) {
        const Gate& gate = nl.gate((GateId)g);
        if (gate.type == GateType::INPUT || gate.type == GateType::OUTPUT)
            continue;

        std::string flat_gname = prefix + "/" + gate.name;

        std::vector<NetId> new_inputs;
        for (auto inp : gate.inputs) {
            auto it = local_net_remap.find(inp);
            if (it != local_net_remap.end())
                new_inputs.push_back(it->second);
        }

        NetId new_out = -1;
        if (gate.output >= 0) {
            auto it = local_net_remap.find(gate.output);
            if (it != local_net_remap.end())
                new_out = it->second;
        }

        if (gate.type == GateType::DFF) {
            NetId new_d = new_inputs.empty() ? -1 : new_inputs[0];
            NetId new_clk = -1;
            if (gate.clk >= 0) {
                auto it = local_net_remap.find(gate.clk);
                if (it != local_net_remap.end()) new_clk = it->second;
            }
            NetId new_reset = -1;
            if (gate.reset >= 0) {
                auto it = local_net_remap.find(gate.reset);
                if (it != local_net_remap.end()) new_reset = it->second;
            }
            result.add_dff(new_d, new_clk, new_out, new_reset, flat_gname);
        } else {
            result.add_gate(gate.type, new_inputs, new_out, flat_gname);
        }
    }

    // Recurse into sub-instances
    for (auto& inst : mod.instances) {
        const ModuleDef* sub = find_module(inst.module_name);
        if (!sub) continue;

        std::string sub_prefix = prefix + "/" + inst.inst_name;

        // Before recursing, set up port aliases so that connected instance
        // nets map to the parent's nets.
        for (auto& [port_name, parent_net_name] : inst.connections) {
            const ModulePort* port = sub->find_port(port_name);
            if (!port || port->net_id < 0) continue;

            // The internal net name that corresponds to this port
            const Net& port_net = sub->internal_netlist.net(port->net_id);
            std::string sub_net_flat = sub_prefix + "/" + port_net.name;

            // The parent net's flat name
            std::string parent_flat = prefix + "/" + parent_net_name;

            // If the parent net exists in net_map, alias the sub net to it
            auto pit = net_map.find(parent_flat);
            if (pit != net_map.end()) {
                net_map[sub_net_flat] = pit->second;
            }
        }

        flatten_recursive(sub_prefix, *sub, result, net_map);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// HierarchicalNetlist — Elaborate
// ═══════════════════════════════════════════════════════════════════════════

bool HierarchicalNetlist::elaborate() {
    // Verify all referenced modules exist
    for (auto& mod : modules_) {
        for (auto& inst : mod.instances) {
            if (!find_module(inst.module_name))
                return false;
        }
    }
    // Verify top module is set and exists
    if (top_module_.empty() || !find_module(top_module_))
        return false;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// HierarchicalNetlist — Hierarchy queries
// ═══════════════════════════════════════════════════════════════════════════

int HierarchicalNetlist::depth() const {
    if (top_module_.empty()) return 0;
    const ModuleDef* top = find_module(top_module_);
    if (!top) return 0;
    return depth_recursive(*top, 0);
}

int HierarchicalNetlist::depth_recursive(const ModuleDef& mod, int current) const {
    if (mod.instances.empty()) return current;
    int max_d = current;
    for (auto& inst : mod.instances) {
        const ModuleDef* sub = find_module(inst.module_name);
        if (sub)
            max_d = std::max(max_d, depth_recursive(*sub, current + 1));
    }
    return max_d;
}

int HierarchicalNetlist::total_instances() const {
    if (top_module_.empty()) return 0;
    const ModuleDef* top = find_module(top_module_);
    if (!top) return 0;
    return count_instances_recursive(*top);
}

int HierarchicalNetlist::count_instances_recursive(const ModuleDef& mod) const {
    int count = (int)mod.instances.size();
    for (auto& inst : mod.instances) {
        const ModuleDef* sub = find_module(inst.module_name);
        if (sub)
            count += count_instances_recursive(*sub);
    }
    return count;
}

std::vector<std::string> HierarchicalNetlist::module_names() const {
    std::vector<std::string> names;
    names.reserve(modules_.size());
    for (auto& m : modules_)
        names.push_back(m.name);
    return names;
}

std::string HierarchicalNetlist::net_hierarchy_path(const std::string& flat_name) const {
    // Flat names use "/" as hierarchy separator (e.g., "top/u_core/n0")
    // Convert to dotted hierarchy path for display
    std::string path = flat_name;
    for (auto& c : path)
        if (c == '/') c = '.';
    return path;
}

// ═══════════════════════════════════════════════════════════════════════════
// HierarchicalNetlist — Block abstraction
// ═══════════════════════════════════════════════════════════════════════════

HierarchicalNetlist::BlockAbstract
HierarchicalNetlist::create_abstract(const std::string& module_name) const {
    BlockAbstract abs;
    const ModuleDef* mod = find_module(module_name);
    if (!mod) return abs;

    abs.module_name = module_name;
    abs.ports = mod->ports;

    // Estimate area from gate count
    abs.area = static_cast<double>(mod->internal_netlist.num_gates());

    // Estimate leakage from gate count (rough model: 1nW per gate)
    abs.leakage_power = abs.area * 0.001;

    // Generate port-to-port delay arcs: from each INPUT to each OUTPUT
    // Delay estimate: proportional to combinational depth
    auto topo = mod->internal_netlist.topo_order();
    double base_delay = topo.empty() ? 0.0 : static_cast<double>(topo.size()) * 0.05;

    for (auto& ip : mod->ports) {
        if (ip.direction != ModulePort::INPUT) continue;
        for (auto& op : mod->ports) {
            if (op.direction != ModulePort::OUTPUT) continue;
            BlockAbstract::DelayArc arc;
            arc.from_port = ip.name;
            arc.to_port = op.name;
            arc.delay_rise = base_delay;
            arc.delay_fall = base_delay * 1.1;  // fall slightly slower
            abs.arcs.push_back(std::move(arc));
        }
    }
    return abs;
}

// ═══════════════════════════════════════════════════════════════════════════
// HierarchicalNetlist — Statistics
// ═══════════════════════════════════════════════════════════════════════════

void HierarchicalNetlist::print_stats() const {
    int n_modules = (int)modules_.size();
    int n_instances = total_instances();
    int d = depth();

    std::cout << "HierarchicalNetlist: " << n_modules << " modules, "
              << n_instances << " instances, depth=" << d;
    if (!top_module_.empty())
        std::cout << ", top=" << top_module_;
    std::cout << "\n";

    for (auto& m : modules_) {
        std::cout << "  module " << m.name << ": "
                  << m.ports.size() << " ports, "
                  << m.internal_netlist.num_nets() << " nets, "
                  << m.internal_netlist.num_gates() << " gates, "
                  << m.instances.size() << " instances\n";
    }
}

} // namespace sf
