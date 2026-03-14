// SiliconForge — Hierarchy Manager Implementation
// Supports: instance tree, block abstraction, parameterized modules, selective flattening.
#include "core/hierarchy.hpp"
#include <chrono>
#include <algorithm>
#include <sstream>
#include <unordered_set>
#include <functional>

namespace sf {

// ═══════════════════════════════════════════════════════
// Module Registration
// ═══════════════════════════════════════════════════════

void HierarchyManager::add_module(const std::string& name, const Netlist& nl) {
    DesignModule mod;
    mod.name = name;
    mod.netlist = nl;
    mod.gate_count = (int)nl.num_gates();
    mod.is_leaf = true;
    modules_[name] = std::move(mod);
}

void HierarchyManager::add_module(const std::string& name, const Netlist& nl,
                                    const PhysicalDesign& pd) {
    DesignModule mod;
    mod.name = name;
    mod.netlist = nl;
    mod.physical = pd;
    mod.gate_count = (int)nl.num_gates();
    mod.area_um2 = pd.die_area.area();
    mod.is_leaf = true;
    modules_[name] = std::move(mod);
}

void HierarchyManager::define_ports(const std::string& module_name,
                                     const std::vector<ModulePort>& ports) {
    auto it = modules_.find(module_name);
    if (it == modules_.end()) return;
    it->second.ports = ports;
}

void HierarchyManager::define_params(const std::string& module_name,
                                      const std::unordered_map<std::string, ParamValue>& defaults) {
    auto it = modules_.find(module_name);
    if (it == modules_.end()) return;
    it->second.default_params = defaults;
}

// ═══════════════════════════════════════════════════════
// Instantiation
// ═══════════════════════════════════════════════════════

void HierarchyManager::instantiate(const std::string& parent, const std::string& child,
                                     const std::string& instance_name) {
    auto pit = modules_.find(parent);
    if (pit == modules_.end()) return;
    pit->second.sub_modules.push_back(child);
    pit->second.is_leaf = false;

    // Also create a proper ModuleInstance
    ModuleInstance inst;
    inst.module_name = child;
    inst.instance_name = instance_name.empty()
        ? (child + "_" + std::to_string(pit->second.instances.size()))
        : instance_name;
    pit->second.instances.push_back(std::move(inst));
}

ModuleInstance& HierarchyManager::instantiate_with_params(
    const std::string& parent, const std::string& child,
    const std::string& instance_name,
    const std::unordered_map<std::string, ParamValue>& params) {

    auto pit = modules_.find(parent);
    // Create instance with parameters
    ModuleInstance inst;
    inst.module_name = child;
    inst.instance_name = instance_name;
    inst.params = params;

    // Merge with module defaults: instance params override defaults
    auto cit = modules_.find(child);
    if (cit != modules_.end()) {
        for (auto& [k, v] : cit->second.default_params) {
            if (inst.params.find(k) == inst.params.end())
                inst.params[k] = v;
        }
    }

    if (pit != modules_.end()) {
        pit->second.sub_modules.push_back(child);
        pit->second.is_leaf = false;
        pit->second.instances.push_back(std::move(inst));
        return pit->second.instances.back();
    }

    // Fallback: create a temporary module for the parent
    static ModuleInstance dummy;
    return dummy;
}

void HierarchyManager::connect_port(const std::string& parent,
                                      const std::string& instance_name,
                                      const std::string& port_name,
                                      const std::string& net_name) {
    auto pit = modules_.find(parent);
    if (pit == modules_.end()) return;
    for (auto& inst : pit->second.instances) {
        if (inst.instance_name == instance_name) {
            inst.port_map[port_name] = net_name;
            return;
        }
    }
}

// ═══════════════════════════════════════════════════════
// Block Abstraction
// ═══════════════════════════════════════════════════════

void HierarchyManager::add_timing_arc(const std::string& module_name,
                                        const std::string& from_port,
                                        const std::string& to_port,
                                        double delay_ps, double slew_ps) {
    auto it = modules_.find(module_name);
    if (it == modules_.end()) return;
    HierTimingArc arc;
    arc.from_port = from_port;
    arc.to_port = to_port;
    arc.delay_ps = delay_ps;
    arc.slew_ps = slew_ps;
    it->second.block_model.timing_arcs.push_back(arc);
    it->second.block_model.valid = true;
}

void HierarchyManager::set_block_power(const std::string& module_name,
                                         double static_mw, double dynamic_mw_per_mhz) {
    auto it = modules_.find(module_name);
    if (it == modules_.end()) return;
    it->second.block_model.static_power_mw = static_mw;
    it->second.block_model.dynamic_power_mw_per_mhz = dynamic_mw_per_mhz;
    it->second.block_model.valid = true;
}

BlockModel HierarchyManager::generate_block_model(const std::string& module_name) const {
    auto it = modules_.find(module_name);
    if (it == modules_.end()) return {};

    BlockModel model;
    model.ports = it->second.ports;
    model.timing_arcs = it->second.block_model.timing_arcs;
    model.area_um2 = it->second.area_um2;
    model.static_power_mw = it->second.block_model.static_power_mw;
    model.dynamic_power_mw_per_mhz = it->second.block_model.dynamic_power_mw_per_mhz;

    // If no explicit timing arcs, derive from STA of the netlist
    // (simplified: estimate from gate count)
    if (model.timing_arcs.empty() && !it->second.ports.empty()) {
        for (auto& ip : model.ports) {
            if (ip.direction != ModulePort::INPUT) continue;
            for (auto& op : model.ports) {
                if (op.direction != ModulePort::OUTPUT) continue;
                HierTimingArc arc;
                arc.from_port = ip.name;
                arc.to_port = op.name;
                arc.delay_ps = 50.0 * it->second.gate_count; // rough estimate
                model.timing_arcs.push_back(arc);
            }
        }
    }

    model.valid = true;
    return model;
}

// ═══════════════════════════════════════════════════════
// Queries
// ═══════════════════════════════════════════════════════

DesignModule* HierarchyManager::get_module(const std::string& name) {
    auto it = modules_.find(name);
    return it != modules_.end() ? &it->second : nullptr;
}

const DesignModule* HierarchyManager::get_module(const std::string& name) const {
    auto it = modules_.find(name);
    return it != modules_.end() ? &it->second : nullptr;
}

int HierarchyManager::instance_count(const std::string& module_name) const {
    int count = 0;
    for (auto& [_, mod] : modules_) {
        for (auto& inst : mod.instances) {
            if (inst.module_name == module_name) count++;
        }
    }
    return count;
}

std::vector<const ModuleInstance*> HierarchyManager::get_instances_of(
    const std::string& module_name) const {
    std::vector<const ModuleInstance*> result;
    for (auto& [_, mod] : modules_) {
        for (auto& inst : mod.instances) {
            if (inst.module_name == module_name)
                result.push_back(&inst);
        }
    }
    return result;
}

int HierarchyManager::compute_depth(const std::string& mod, int current) const {
    auto it = modules_.find(mod);
    if (it == modules_.end()) return current;
    if (it->second.sub_modules.empty()) return current;
    int max_d = current;
    for (auto& sub : it->second.sub_modules)
        max_d = std::max(max_d, compute_depth(sub, current + 1));
    return max_d;
}

// ═══════════════════════════════════════════════════════
// Analysis
// ═══════════════════════════════════════════════════════

HierarchyResult HierarchyManager::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    HierarchyResult r;
    r.total_modules = (int)modules_.size();

    for (auto& [name, mod] : modules_) {
        if (mod.is_leaf) r.leaf_modules++;
        r.total_gates += mod.gate_count;
        r.total_area += mod.area_um2;
        r.total_instances += (int)mod.instances.size();
    }

    // Find top module (not instantiated by anyone else)
    std::unordered_set<std::string> children;
    for (auto& [name, mod] : modules_)
        for (auto& sub : mod.sub_modules) children.insert(sub);

    for (auto& [name, mod] : modules_) {
        if (children.find(name) == children.end()) {
            r.hierarchy_depth = compute_depth(name);
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.total_modules) + " modules (" +
                std::to_string(r.leaf_modules) + " leaf), depth=" +
                std::to_string(r.hierarchy_depth) + ", " +
                std::to_string(r.total_gates) + " gates, " +
                std::to_string(r.total_instances) + " instances";
    return r;
}

// ═══════════════════════════════════════════════════════
// Flattening
// ═══════════════════════════════════════════════════════

Netlist HierarchyManager::flatten(const std::string& top_module) {
    FlattenOptions opts;
    return flatten(top_module, opts);
}

Netlist HierarchyManager::flatten(const std::string& top_module, const FlattenOptions& opts) {
    Netlist flat;
    flatten_recursive(top_module, "", 0, opts, flat);
    return flat;
}

void HierarchyManager::flatten_recursive(const std::string& mod_name, const std::string& prefix,
                                           int depth, const FlattenOptions& opts,
                                           Netlist& flat) const {
    auto* mod = get_module(mod_name);
    if (!mod) return;

    // Check depth limit
    if (opts.policy == FlattenPolicy::DEPTH_LIMITED && opts.max_depth >= 0 && depth > opts.max_depth)
        return;

    // Check if this module should be preserved (not flattened)
    if (opts.policy == FlattenPolicy::SELECTIVE && opts.preserve_modules.count(mod_name))
        return;

    std::unordered_map<NetId, NetId> net_remap;
    // Copy nets
    for (size_t i = 0; i < mod->netlist.num_nets(); ++i) {
        auto& net = mod->netlist.net(i);
        std::string new_name = opts.preserve_hierarchy_names
            ? (prefix + net.name) : net.name;
        NetId new_nid = flat.add_net(new_name);
        net_remap[i] = new_nid;
    }
    // Copy gates
    for (size_t g = 0; g < mod->netlist.num_gates(); ++g) {
        auto& gate = mod->netlist.gate(g);
        if (gate.type == GateType::INPUT || gate.type == GateType::OUTPUT) continue;
        std::vector<NetId> new_inputs;
        for (auto inp : gate.inputs) {
            if (net_remap.count(inp)) new_inputs.push_back(net_remap[inp]);
        }
        NetId new_out = net_remap.count(gate.output) ? net_remap[gate.output] : -1;
        if (new_out >= 0) {
            std::string gname = opts.preserve_hierarchy_names
                ? (prefix + gate.name) : gate.name;
            flat.add_gate(gate.type, new_inputs, new_out, gname);
        }
    }

    // Recurse into children
    for (auto& sub : mod->sub_modules)
        flatten_recursive(sub, prefix + sub + "/", depth + 1, opts, flat);
}

// ═══════════════════════════════════════════════════════
// Visualization
// ═══════════════════════════════════════════════════════

std::string HierarchyManager::hierarchy_tree(const std::string& top, int indent) const {
    std::ostringstream ss;
    auto* mod = get_module(top);
    if (!mod) return "";

    for (int i = 0; i < indent; ++i) ss << "  ";
    ss << (mod->is_leaf ? "├─ " : "├─ ") << top;
    ss << " (" << mod->gate_count << " gates";
    if (mod->area_um2 > 0) ss << ", " << (int)mod->area_um2 << "um²";
    if (!mod->instances.empty()) ss << ", " << mod->instances.size() << " inst";
    if (mod->block_model.valid) ss << ", abstract";
    ss << ")\n";

    for (auto& sub : mod->sub_modules)
        ss << hierarchy_tree(sub, indent + 1);

    return ss.str();
}

} // namespace sf
