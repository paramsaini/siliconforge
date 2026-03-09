// SiliconForge — Hierarchy Manager Implementation
#include "core/hierarchy.hpp"
#include <chrono>
#include <algorithm>
#include <sstream>
#include <unordered_set>

namespace sf {

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

void HierarchyManager::instantiate(const std::string& parent, const std::string& child,
                                     const std::string& instance_name) {
    auto pit = modules_.find(parent);
    if (pit == modules_.end()) return;
    pit->second.sub_modules.push_back(child);
    pit->second.is_leaf = false;
}

DesignModule* HierarchyManager::get_module(const std::string& name) {
    auto it = modules_.find(name);
    return it != modules_.end() ? &it->second : nullptr;
}

const DesignModule* HierarchyManager::get_module(const std::string& name) const {
    auto it = modules_.find(name);
    return it != modules_.end() ? &it->second : nullptr;
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

HierarchyResult HierarchyManager::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    HierarchyResult r;
    r.total_modules = (int)modules_.size();

    for (auto& [name, mod] : modules_) {
        if (mod.is_leaf) r.leaf_modules++;
        r.total_gates += mod.gate_count;
        r.total_area += mod.area_um2;
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
                std::to_string(r.total_gates) + " gates";
    return r;
}

Netlist HierarchyManager::flatten(const std::string& top_module) {
    Netlist flat;
    // Copy all gates/nets from all reachable modules with prefixed names
    std::function<void(const std::string&, const std::string&)> merge;
    merge = [&](const std::string& mod_name, const std::string& prefix) {
        auto* mod = get_module(mod_name);
        if (!mod) return;

        std::unordered_map<NetId, NetId> net_remap;
        // Copy nets
        for (size_t i = 0; i < mod->netlist.num_nets(); ++i) {
            auto& net = mod->netlist.net(i);
            NetId new_nid = flat.add_net(prefix + net.name);
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
            if (new_out >= 0)
                flat.add_gate(gate.type, new_inputs, new_out, prefix + gate.name);
        }

        for (auto& sub : mod->sub_modules)
            merge(sub, prefix + sub + "/");
    };

    merge(top_module, "");
    return flat;
}

std::string HierarchyManager::hierarchy_tree(const std::string& top, int indent) const {
    std::ostringstream ss;
    auto* mod = get_module(top);
    if (!mod) return "";

    for (int i = 0; i < indent; ++i) ss << "  ";
    ss << (mod->is_leaf ? "├─ " : "├─ ") << top;
    ss << " (" << mod->gate_count << " gates";
    if (mod->area_um2 > 0) ss << ", " << (int)mod->area_um2 << "um²";
    ss << ")\n";

    for (auto& sub : mod->sub_modules)
        ss << hierarchy_tree(sub, indent + 1);

    return ss.str();
}

} // namespace sf
