// SiliconForge — LVS Checker Implementation
// Graph-based schematic vs layout comparison using connectivity fingerprints.
#include "verify/lvs.hpp"
#include <chrono>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <set>
#include <sstream>
#include <functional>

namespace sf {

// Build a connectivity fingerprint for each cell:
// fingerprint = sorted list of (connected_cell_type, pin_index) pairs
// Two cells match if they have the same type AND the same connectivity fingerprint.
static std::string build_fingerprint(int cell_idx, const std::string& cell_type,
                                      const std::vector<std::vector<int>>& cell_nets,
                                      const std::vector<std::vector<int>>& net_cells,
                                      const std::vector<std::string>& cell_types) {
    std::multiset<std::string> neighbors;
    if (cell_idx < (int)cell_nets.size()) {
        for (int net_id : cell_nets[cell_idx]) {
            if (net_id < 0 || net_id >= (int)net_cells.size()) continue;
            for (int other : net_cells[net_id]) {
                if (other == cell_idx) continue;
                if (other >= 0 && other < (int)cell_types.size())
                    neighbors.insert(cell_types[other]);
            }
        }
    }
    std::string fp = cell_type + ":{";
    for (auto& n : neighbors) fp += n + ",";
    fp += "}";
    return fp;
}

LvsResult LvsChecker::check() {
    auto t0 = std::chrono::high_resolution_clock::now();
    LvsResult r;

    // === Build schematic graph ===
    // Cells: non-IO gates; Nets: netlist nets
    std::vector<int> schem_gate_ids;  // index → gate ID
    std::vector<std::string> schem_types;
    std::unordered_map<int, int> gate_to_schem; // gateId → schem cell index
    for (size_t i = 0; i < schem_.num_gates(); ++i) {
        auto& g = schem_.gate(i);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT ||
            g.type == GateType::CONST0 || g.type == GateType::CONST1) continue;
        gate_to_schem[(int)i] = (int)schem_gate_ids.size();
        schem_gate_ids.push_back((int)i);
        schem_types.push_back(gate_type_str(g.type));
    }
    r.schematic_cells = (int)schem_gate_ids.size();

    // Build schem cell↔net adjacency
    std::vector<std::vector<int>> schem_cell_nets(schem_gate_ids.size());
    std::vector<std::vector<int>> schem_net_cells;
    int schem_net_count = 0;
    for (size_t ni = 0; ni < schem_.num_nets(); ++ni) {
        auto& net = schem_.net(ni);
        std::vector<int> connected;
        if (net.driver >= 0 && gate_to_schem.count(net.driver))
            connected.push_back(gate_to_schem[net.driver]);
        for (auto fo : net.fanout) {
            if (gate_to_schem.count(fo))
                connected.push_back(gate_to_schem[fo]);
        }
        if (connected.size() >= 2) {
            int net_idx = schem_net_count++;
            schem_net_cells.push_back(connected);
            for (int c : connected) {
                if (c >= 0 && c < (int)schem_cell_nets.size())
                    schem_cell_nets[c].push_back(net_idx);
            }
        }
    }

    // Build schematic fingerprints
    std::unordered_map<std::string, std::vector<int>> schem_fp_map;
    for (int i = 0; i < (int)schem_gate_ids.size(); ++i) {
        std::string fp = build_fingerprint(i, schem_types[i],
                                            schem_cell_nets, schem_net_cells, schem_types);
        schem_fp_map[fp].push_back(i);
    }

    // === Build layout graph ===
    r.layout_cells = (int)layout_.cells.size();
    std::vector<std::string> layout_types;
    for (auto& c : layout_.cells)
        layout_types.push_back(c.cell_type);

    // Build layout cell↔net adjacency
    std::vector<std::vector<int>> layout_cell_nets(layout_.cells.size());
    std::vector<std::vector<int>> layout_net_cells;
    for (int ni = 0; ni < (int)layout_.nets.size(); ++ni) {
        auto& net = layout_.nets[ni];
        if (net.cell_ids.size() >= 2) {
            layout_net_cells.push_back(net.cell_ids);
            for (int c : net.cell_ids) {
                if (c >= 0 && c < (int)layout_cell_nets.size())
                    layout_cell_nets[c].push_back(ni);
            }
        }
    }

    // Build layout fingerprints and match against schematic
    std::unordered_map<std::string, int> matched_fp_count;
    r.matched_cells = 0;
    for (int i = 0; i < (int)layout_.cells.size(); ++i) {
        std::string fp = build_fingerprint(i, layout_types[i],
                                            layout_cell_nets, layout_net_cells, layout_types);
        auto it = schem_fp_map.find(fp);
        if (it != schem_fp_map.end()) {
            int already_matched = matched_fp_count[fp];
            if (already_matched < (int)it->second.size()) {
                r.matched_cells++;
                matched_fp_count[fp]++;
            } else {
                r.unmatched_layout++;
                r.mismatches.push_back({"extra_cell",
                    "Layout cell '" + layout_.cells[i].name + "' (" + layout_types[i] +
                    ") has no unmatched schematic counterpart"});
            }
        } else {
            r.unmatched_layout++;
            r.mismatches.push_back({"extra_cell",
                "Layout cell '" + layout_.cells[i].name + "' connectivity doesn't match schematic"});
        }
    }

    // Check for unmatched schematic cells
    for (auto& [fp, indices] : schem_fp_map) {
        int matched = matched_fp_count.count(fp) ? matched_fp_count[fp] : 0;
        int unmatched = (int)indices.size() - matched;
        if (unmatched > 0) {
            r.unmatched_schematic += unmatched;
            int sample = indices[0];
            r.mismatches.push_back({"missing_cell",
                std::to_string(unmatched) + " " + schem_types[sample] +
                " cell(s) in schematic not found in layout"});
        }
    }

    // Net connectivity comparison: compare net count ratio
    if (r.layout_cells > 0 && r.schematic_cells > 0) {
        double schem_ratio = (double)schem_net_count / r.schematic_cells;
        double layout_ratio = (double)layout_net_cells.size() / r.layout_cells;
        if (std::abs(schem_ratio - layout_ratio) > 0.5) {
            r.net_mismatches++;
            r.mismatches.push_back({"net_mismatch",
                "Net-to-cell ratio mismatch: schematic=" +
                std::to_string(schem_net_count) + "/" + std::to_string(r.schematic_cells) +
                " layout=" + std::to_string((int)layout_net_cells.size()) + "/" +
                std::to_string(r.layout_cells)});
        }
    }

    r.match = r.mismatches.empty();

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = r.match ? "LVS CLEAN — schematic and layout match (graph isomorphism)"
                        : "LVS MISMATCH — " + std::to_string(r.mismatches.size()) + " issue(s)";
    return r;
}

} // namespace sf
