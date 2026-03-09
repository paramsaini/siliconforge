#pragma once
// SiliconForge — Hierarchical Design Manager
// Manages multi-module designs with instantiation hierarchy,
// top-down partitioning, and bottom-up assembly.
// Reference: Keutzer et al., "System-Level Design: Orthogonalization of Concerns", IEEE 2000

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

namespace sf {

struct DesignModule {
    std::string name;
    Netlist netlist;
    PhysicalDesign physical;
    bool is_leaf = true;
    std::vector<std::string> sub_modules; // names of instantiated sub-modules
    double area_um2 = 0;
    int gate_count = 0;
};

struct HierarchyResult {
    int total_modules = 0;
    int leaf_modules = 0;
    int hierarchy_depth = 0;
    double total_area = 0;
    int total_gates = 0;
    double time_ms = 0;
    std::string message;
};

class HierarchyManager {
public:
    // Add a design module
    void add_module(const std::string& name, const Netlist& nl);
    void add_module(const std::string& name, const Netlist& nl, const PhysicalDesign& pd);

    // Instantiate sub-module within a parent
    void instantiate(const std::string& parent, const std::string& child,
                    const std::string& instance_name = "");

    // Analyze hierarchy
    HierarchyResult analyze();

    // Flatten: merge all modules into one netlist
    Netlist flatten(const std::string& top_module);

    // Get module
    DesignModule* get_module(const std::string& name);
    const DesignModule* get_module(const std::string& name) const;

    // Get hierarchy as text tree
    std::string hierarchy_tree(const std::string& top, int indent = 0) const;

private:
    std::unordered_map<std::string, DesignModule> modules_;
    int compute_depth(const std::string& mod, int current = 0) const;
};

} // namespace sf
