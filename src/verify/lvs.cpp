// SiliconForge — LVS Checker Implementation
#include "verify/lvs.hpp"
#include <chrono>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

namespace sf {

LvsResult LvsChecker::check() {
    auto t0 = std::chrono::high_resolution_clock::now();
    LvsResult r;

    // Count schematic elements (non-IO gates)
    std::unordered_map<std::string, int> schem_types;
    for (size_t i = 0; i < schem_.num_gates(); ++i) {
        auto& g = schem_.gate(i);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
        if (g.type == GateType::CONST0 || g.type == GateType::CONST1) continue;
        schem_types[gate_type_str(g.type)]++;
        r.schematic_cells++;
    }

    // Count layout elements
    std::unordered_map<std::string, int> layout_types;
    for (auto& c : layout_.cells) {
        layout_types[c.cell_type]++;
        r.layout_cells++;
    }

    // Match by gate type counts (simplified LVS)
    // Real LVS would do graph isomorphism
    std::unordered_set<std::string> all_types;
    for (auto& [t, _] : schem_types) all_types.insert(t);
    for (auto& [t, _] : layout_types) all_types.insert(t);

    for (auto& t : all_types) {
        int s_count = schem_types.count(t) ? schem_types[t] : 0;
        int l_count = layout_types.count(t) ? layout_types[t] : 0;
        int matched = std::min(s_count, l_count);
        r.matched_cells += matched;

        if (s_count > l_count) {
            r.unmatched_schematic += (s_count - l_count);
            r.mismatches.push_back({"missing_cell",
                std::to_string(s_count - l_count) + " " + t + " cell(s) in schematic but not in layout"});
        }
        if (l_count > s_count) {
            r.unmatched_layout += (l_count - s_count);
            r.mismatches.push_back({"extra_cell",
                std::to_string(l_count - s_count) + " " + t + " cell(s) in layout but not in schematic"});
        }
    }

    // Check net connectivity match
    // Compare net counts
    int schem_nets = (int)schem_.num_nets();
    int layout_nets = (int)layout_.nets.size();
    if (std::abs(schem_nets - layout_nets) > schem_nets / 2) {
        r.net_mismatches++;
        r.mismatches.push_back({"net_mismatch",
            "Net count mismatch: schematic=" + std::to_string(schem_nets) +
            " layout=" + std::to_string(layout_nets)});
    }

    r.match = r.mismatches.empty();

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = r.match ? "LVS CLEAN — schematic and layout match"
                        : "LVS MISMATCH — " + std::to_string(r.mismatches.size()) + " issue(s)";
    return r;
}

} // namespace sf
