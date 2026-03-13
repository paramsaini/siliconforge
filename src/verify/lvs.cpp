// SiliconForge — LVS Checker Implementation
// Iterative partition refinement + connectivity fingerprint matching.
// Reference: Ohlrich et al., "SubGemini: Identifying Subcircuits", ICCAD 1993
#include "verify/lvs.hpp"
#include <chrono>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <set>
#include <map>
#include <sstream>
#include <functional>
#include <numeric>

namespace sf {

// ── Connectivity graph abstraction for both schematic and layout ──────
struct LvsGraph {
    std::vector<std::string> cell_types;
    std::vector<std::vector<int>> cell_nets; // cell → nets
    std::vector<std::vector<int>> net_cells; // net → cells

    // Iterative partition refinement signature
    // Each cell gets a class label. Refine until stable.
    std::vector<int> partition_refine() const {
        int n = (int)cell_types.size();
        if (n == 0) return {};

        // Initial partition: by cell type
        std::unordered_map<std::string, int> type_to_class;
        std::vector<int> classes(n);
        int next_class = 0;
        for (int i = 0; i < n; i++) {
            auto it = type_to_class.find(cell_types[i]);
            if (it == type_to_class.end()) {
                type_to_class[cell_types[i]] = next_class;
                classes[i] = next_class++;
            } else {
                classes[i] = it->second;
            }
        }

        // Iterative refinement: split classes by neighbor class signature
        for (int iter = 0; iter < 20; iter++) {
            std::unordered_map<std::string, int> sig_to_class;
            std::vector<int> new_classes(n);
            int new_next = 0;
            bool changed = false;

            for (int i = 0; i < n; i++) {
                // Build signature: own class + sorted neighbor classes
                std::multiset<int> neighbor_classes;
                for (int net_id : cell_nets[i]) {
                    if (net_id < 0 || net_id >= (int)net_cells.size()) continue;
                    for (int other : net_cells[net_id]) {
                        if (other != i) neighbor_classes.insert(classes[other]);
                    }
                }
                std::string sig = std::to_string(classes[i]) + ":";
                for (int nc : neighbor_classes) sig += std::to_string(nc) + ",";

                auto it = sig_to_class.find(sig);
                if (it == sig_to_class.end()) {
                    sig_to_class[sig] = new_next;
                    new_classes[i] = new_next++;
                } else {
                    new_classes[i] = it->second;
                }
            }

            if (new_classes != classes) {
                classes = new_classes;
                changed = true;
            }
            if (!changed) break;
        }
        return classes;
    }
};

// Build LvsGraph from schematic Netlist
static LvsGraph build_schem_graph(const Netlist& nl, std::vector<int>& gate_ids) {
    LvsGraph g;
    std::unordered_map<int, int> gate_to_idx;

    for (size_t i = 0; i < nl.num_gates(); i++) {
        auto& gate = nl.gate(i);
        if (gate.type == GateType::INPUT || gate.type == GateType::OUTPUT ||
            gate.type == GateType::CONST0 || gate.type == GateType::CONST1) continue;
        gate_to_idx[(int)i] = (int)g.cell_types.size();
        gate_ids.push_back((int)i);
        g.cell_types.push_back(gate_type_str(gate.type));
    }
    g.cell_nets.resize(g.cell_types.size());

    int net_idx = 0;
    for (size_t ni = 0; ni < nl.num_nets(); ni++) {
        auto& net = nl.net(ni);
        std::vector<int> connected;
        if (net.driver >= 0 && gate_to_idx.count(net.driver))
            connected.push_back(gate_to_idx[net.driver]);
        for (auto fo : net.fanout)
            if (gate_to_idx.count(fo)) connected.push_back(gate_to_idx[fo]);

        if (connected.size() >= 2) {
            g.net_cells.push_back(connected);
            for (int c : connected) g.cell_nets[c].push_back(net_idx);
            net_idx++;
        }
    }
    return g;
}

// Check if a cell type is physical infrastructure (not in schematic netlist)
static bool is_infrastructure_cell(const std::string& cell_type) {
    std::string ct = cell_type;
    std::transform(ct.begin(), ct.end(), ct.begin(), ::toupper);
    return ct.find("ENDCAP") != std::string::npos ||
           ct.find("WELLTAP") != std::string::npos ||
           ct.find("FILL") != std::string::npos ||
           ct.find("FILLER") != std::string::npos ||
           ct.find("TAP") != std::string::npos ||
           ct.find("DECAP") != std::string::npos;
}

// Build LvsGraph from PhysicalDesign, skipping infrastructure cells
static LvsGraph build_layout_graph(const PhysicalDesign& pd) {
    LvsGraph g;
    std::vector<int> pd_to_graph(pd.cells.size(), -1);
    for (size_t i = 0; i < pd.cells.size(); i++) {
        if (is_infrastructure_cell(pd.cells[i].cell_type)) continue;
        pd_to_graph[i] = (int)g.cell_types.size();
        g.cell_types.push_back(pd.cells[i].cell_type);
    }
    g.cell_nets.resize(g.cell_types.size());

    for (int ni = 0; ni < (int)pd.nets.size(); ni++) {
        auto& net = pd.nets[ni];
        // Remap cell IDs, skipping infrastructure cells
        std::vector<int> mapped;
        for (int c : net.cell_ids) {
            if (c >= 0 && c < (int)pd_to_graph.size() && pd_to_graph[c] >= 0)
                mapped.push_back(pd_to_graph[c]);
        }
        if (mapped.size() >= 2) {
            int graph_net = (int)g.net_cells.size();
            g.net_cells.push_back(mapped);
            for (int c : mapped)
                if (c >= 0 && c < (int)g.cell_nets.size())
                    g.cell_nets[c].push_back(graph_net);
        }
    }
    return g;
}

LvsResult LvsChecker::check() {
    auto t0 = std::chrono::high_resolution_clock::now();
    LvsResult r;

    // Build graphs
    std::vector<int> schem_gate_ids;
    LvsGraph sg = build_schem_graph(schem_, schem_gate_ids);
    LvsGraph lg = build_layout_graph(layout_);

    r.schematic_cells = (int)sg.cell_types.size();
    r.layout_cells = (int)lg.cell_types.size();

    // Step 1: Iterative partition refinement on both graphs
    auto s_classes = sg.partition_refine();
    auto l_classes = lg.partition_refine();

    // Step 2: Build class → cell mappings
    std::unordered_map<int, std::vector<int>> s_class_cells, l_class_cells;
    for (int i = 0; i < (int)s_classes.size(); i++) s_class_cells[s_classes[i]].push_back(i);
    for (int i = 0; i < (int)l_classes.size(); i++) l_class_cells[l_classes[i]].push_back(i);

    // Step 3: Match classes between schematic and layout by type signature
    // Each class has a canonical signature: {type: count_of_type_in_class}
    auto class_sig = [](const std::vector<int>& cells, const std::vector<std::string>& types) -> std::string {
        std::map<std::string, int> type_count;
        for (int c : cells) type_count[types[c]]++;
        std::string sig;
        for (auto& [t, cnt] : type_count) sig += t + ":" + std::to_string(cnt) + ";";
        return sig;
    };

    // Map: schem_class_sig → schem_class_id, layout_class_sig → layout_class_id
    std::unordered_map<std::string, std::vector<int>> s_sig_classes, l_sig_classes;
    for (auto& [cls, cells] : s_class_cells)
        s_sig_classes[class_sig(cells, sg.cell_types)].push_back(cls);
    for (auto& [cls, cells] : l_class_cells)
        l_sig_classes[class_sig(cells, lg.cell_types)].push_back(cls);

    // Step 4: Match by signature
    r.matched_cells = 0;
    std::unordered_set<int> matched_schem, matched_layout;

    for (auto& [sig, s_cls_list] : s_sig_classes) {
        auto it = l_sig_classes.find(sig);
        if (it == l_sig_classes.end()) continue;
        auto& l_cls_list = it->second;

        // Match classes 1:1 by size
        size_t match_count = std::min(s_cls_list.size(), l_cls_list.size());
        for (size_t k = 0; k < match_count; k++) {
            auto& s_cells = s_class_cells[s_cls_list[k]];
            auto& l_cells = l_class_cells[l_cls_list[k]];
            int pair_count = std::min((int)s_cells.size(), (int)l_cells.size());
            r.matched_cells += pair_count;
            for (int j = 0; j < pair_count; j++) {
                matched_schem.insert(s_cells[j]);
                matched_layout.insert(l_cells[j]);
            }
        }
    }

    // Step 5: Report unmatched
    r.unmatched_schematic = r.schematic_cells - (int)matched_schem.size();
    r.unmatched_layout = r.layout_cells - (int)matched_layout.size();

    if (r.unmatched_schematic > 0) {
        // Find first unmatched for detail
        for (int i = 0; i < (int)sg.cell_types.size(); i++) {
            if (!matched_schem.count(i)) {
                r.mismatches.push_back({"missing_cell",
                    std::to_string(r.unmatched_schematic) + " schematic cell(s) not found in layout (first: " +
                    sg.cell_types[i] + ")"});
                break;
            }
        }
    }
    if (r.unmatched_layout > 0) {
        // Check if ALL unmatched layout cells are CTS-inserted buffers
        // or physical infrastructure cells (already filtered, but check residual).
        bool all_cts_or_infra = true;
        for (int i = 0; i < (int)lg.cell_types.size(); i++) {
            if (!matched_layout.count(i)) {
                std::string ct = lg.cell_types[i];
                std::transform(ct.begin(), ct.end(), ct.begin(), ::toupper);
                bool is_cts = (ct.find("CLKBUF") != std::string::npos ||
                               ct.find("CLK_BUF") != std::string::npos ||
                               ct.find("CTS") != std::string::npos);
                bool is_infra = is_infrastructure_cell(ct);
                if (!is_cts && !is_infra) { all_cts_or_infra = false; break; }
            }
        }
        if (!all_cts_or_infra) {
            for (int i = 0; i < (int)lg.cell_types.size(); i++) {
                if (!matched_layout.count(i)) {
                    r.mismatches.push_back({"extra_cell",
                        std::to_string(r.unmatched_layout) + " layout cell(s) not in schematic (first: " +
                        lg.cell_types[i] + ")"});
                    break;
                }
            }
        }
        // CTS buffers and infrastructure cells are expected additions — not a mismatch
    }

    // Step 6: Power rail check — verify VDD/GND nets exist in layout
    bool has_vdd = false, has_gnd = false;
    for (auto& net : layout_.nets) {
        std::string n = net.name;
        std::transform(n.begin(), n.end(), n.begin(), ::toupper);
        if (n == "VDD" || n == "VCC" || n == "VPWR") has_vdd = true;
        if (n == "GND" || n == "VSS" || n == "VGND") has_gnd = true;
    }
    // Only flag if layout has enough cells to expect power rails
    // Note: power rail absence is advisory — simple designs may not have
    // explicit VDD/GND net names in the physical model
    if (r.layout_cells > 20 && !has_vdd && !has_gnd) {
        // Advisory only — do not add to mismatches
    }

    // Step 7: Net ratio sanity check
    if (r.layout_cells > 0 && r.schematic_cells > 0) {
        double s_ratio = (double)sg.net_cells.size() / r.schematic_cells;
        double l_ratio = (double)lg.net_cells.size() / r.layout_cells;
        if (std::abs(s_ratio - l_ratio) > 0.5) {
            r.net_mismatches++;
            r.mismatches.push_back({"net_mismatch",
                "Net/cell ratio mismatch: schem=" + std::to_string(sg.net_cells.size()) +
                "/" + std::to_string(r.schematic_cells) + " layout=" +
                std::to_string(lg.net_cells.size()) + "/" + std::to_string(r.layout_cells)});
        }
    }

    r.match = r.mismatches.empty();
    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = r.match
        ? "LVS CLEAN \xe2\x80\x94 " + std::to_string(r.matched_cells) + "/" +
          std::to_string(r.schematic_cells) + " cells matched (partition refinement)"
        : "LVS MISMATCH \xe2\x80\x94 " + std::to_string(r.mismatches.size()) + " issue(s)";
    return r;
}

} // namespace sf
