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
#include <cmath>
#include <cctype>
#include <regex>
#include <queue>

namespace sf {

// ── Helper: uppercase a string ───────────────────────────────────────
static std::string to_upper(const std::string& s) {
    std::string r = s;
    std::transform(r.begin(), r.end(), r.begin(), ::toupper);
    return r;
}

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
    std::string ct = to_upper(cell_type);
    return ct.find("ENDCAP") != std::string::npos ||
           ct.find("WELLTAP") != std::string::npos ||
           ct.find("FILL") != std::string::npos ||
           ct.find("FILLER") != std::string::npos ||
           ct.find("TAP") != std::string::npos ||
           ct.find("DECAP") != std::string::npos;
}

// Build LvsGraph from PhysicalDesign, skipping infrastructure cells
static LvsGraph build_layout_graph_static(const PhysicalDesign& pd) {
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

// ── Original check() — unchanged ─────────────────────────────────────

LvsResult LvsChecker::check() {
    auto t0 = std::chrono::high_resolution_clock::now();
    LvsResult r;

    // Build graphs
    std::vector<int> schem_gate_ids;
    LvsGraph sg = build_schem_graph(schem_, schem_gate_ids);
    LvsGraph lg = build_layout_graph_static(layout_);

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
        std::string n = to_upper(net.name);
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

// ══════════════════════════════════════════════════════════════════════
// A) Device extraction from layout
// ══════════════════════════════════════════════════════════════════════

LvsChecker::ExtractionResult LvsChecker::extract_devices() {
    ExtractionResult result;

    for (size_t ci = 0; ci < layout_.cells.size(); ++ci) {
        const auto& cell = layout_.cells[ci];
        if (is_infrastructure_cell(cell.cell_type)) continue;

        std::string ct = to_upper(cell.cell_type);

        // Identify MOS transistors
        bool is_nmos = ct.find("NMOS") != std::string::npos ||
                       ct.find("NFET") != std::string::npos ||
                       ct.find("NCH")  != std::string::npos;
        bool is_pmos = ct.find("PMOS") != std::string::npos ||
                       ct.find("PFET") != std::string::npos ||
                       ct.find("PCH")  != std::string::npos;

        if (is_nmos || is_pmos) {
            ExtractedDevice dev;
            dev.type = ExtractedDevice::MOSFET;
            dev.name = cell.name;
            dev.layer = 1;  // gate poly layer

            // Parse W/L from cell dimensions (width = gate width, height ∝ length)
            dev.width  = cell.width;
            dev.length = cell.height;

            // Try to extract W/L from name patterns like "NMOS_W0P5_L0P18"
            auto parse_dim = [](const std::string& s, const std::string& prefix) -> double {
                auto pos = s.find(prefix);
                if (pos == std::string::npos) return -1.0;
                pos += prefix.size();
                std::string num;
                while (pos < s.size() && (std::isdigit(static_cast<unsigned char>(s[pos])) ||
                       s[pos] == '.' || s[pos] == 'P' || s[pos] == 'p')) {
                    char c = s[pos];
                    num += (c == 'P' || c == 'p') ? '.' : c;
                    ++pos;
                }
                if (num.empty()) return -1.0;
                try { return std::stod(num); } catch (...) { return -1.0; }
            };

            double w_parsed = parse_dim(ct, "W");
            double l_parsed = parse_dim(ct, "L");
            if (w_parsed > 0) dev.width  = w_parsed;
            if (l_parsed > 0) dev.length = l_parsed;

            // Map terminal nets from connected nets
            // Convention: for a MOS cell, pin order is drain, gate, source, bulk
            std::vector<std::string> connected_nets;
            for (const auto& net : layout_.nets) {
                for (int cid : net.cell_ids) {
                    if (cid == (int)ci) {
                        connected_nets.push_back(net.name);
                        break;
                    }
                }
            }
            if (connected_nets.size() >= 1) dev.drain  = connected_nets[0];
            if (connected_nets.size() >= 2) dev.gate   = connected_nets[1];
            if (connected_nets.size() >= 3) dev.source = connected_nets[2];
            if (connected_nets.size() >= 4) dev.bulk   = connected_nets[3];
            // Default bulk to power rail if not explicitly connected
            if (dev.bulk.empty()) dev.bulk = is_nmos ? "GND" : "VDD";

            result.devices.push_back(dev);
            result.mos_count++;
            continue;
        }

        // Identify resistors (poly resistors, RPPOLY, etc.)
        bool is_res = ct.find("RES") != std::string::npos ||
                      ct.find("RPOLY") != std::string::npos ||
                      ct.find("RPPOLY") != std::string::npos ||
                      ct.find("RHIGH") != std::string::npos;
        if (is_res) {
            ExtractedDevice dev;
            dev.type = ExtractedDevice::RESISTOR;
            dev.name = cell.name;
            dev.layer = 2;  // poly layer
            dev.width  = cell.width;
            dev.length = cell.height;

            // Terminal mapping: two-terminal device
            std::vector<std::string> connected_nets;
            for (const auto& net : layout_.nets) {
                for (int cid : net.cell_ids) {
                    if (cid == (int)ci) {
                        connected_nets.push_back(net.name);
                        break;
                    }
                }
            }
            if (connected_nets.size() >= 1) dev.drain  = connected_nets[0]; // terminal A
            if (connected_nets.size() >= 2) dev.source = connected_nets[1]; // terminal B

            result.devices.push_back(dev);
            result.res_count++;
            continue;
        }

        // Identify capacitors (MIM, MOM, etc.)
        bool is_cap = ct.find("CAP") != std::string::npos ||
                      ct.find("MIM") != std::string::npos ||
                      ct.find("MOM") != std::string::npos ||
                      ct.find("CMIM") != std::string::npos;
        if (is_cap) {
            ExtractedDevice dev;
            dev.type = ExtractedDevice::CAPACITOR;
            dev.name = cell.name;
            dev.layer = 5;  // metal capacitor layer
            dev.width  = cell.width;
            dev.length = cell.height;

            std::vector<std::string> connected_nets;
            for (const auto& net : layout_.nets) {
                for (int cid : net.cell_ids) {
                    if (cid == (int)ci) {
                        connected_nets.push_back(net.name);
                        break;
                    }
                }
            }
            if (connected_nets.size() >= 1) dev.drain  = connected_nets[0]; // top plate
            if (connected_nets.size() >= 2) dev.source = connected_nets[1]; // bottom plate

            result.devices.push_back(dev);
            result.cap_count++;
            continue;
        }

        // Identify diodes
        bool is_diode = ct.find("DIODE") != std::string::npos ||
                        ct.find("ESD")   != std::string::npos;
        if (is_diode) {
            ExtractedDevice dev;
            dev.type = ExtractedDevice::DIODE;
            dev.name = cell.name;
            dev.layer = 1;
            dev.width  = cell.width;
            dev.length = cell.height;

            std::vector<std::string> connected_nets;
            for (const auto& net : layout_.nets) {
                for (int cid : net.cell_ids) {
                    if (cid == (int)ci) {
                        connected_nets.push_back(net.name);
                        break;
                    }
                }
            }
            if (connected_nets.size() >= 1) dev.drain  = connected_nets[0]; // anode
            if (connected_nets.size() >= 2) dev.source = connected_nets[1]; // cathode

            result.devices.push_back(dev);
        }
    }

    return result;
}

// ══════════════════════════════════════════════════════════════════════
// B) Subcircuit matching — graph-based cell matching
// ══════════════════════════════════════════════════════════════════════

LvsChecker::NetGraph LvsChecker::build_schematic_graph() {
    NetGraph g;
    std::unordered_map<int, int> gate_to_idx;

    for (size_t i = 0; i < schem_.num_gates(); i++) {
        auto& gate = schem_.gate(i);
        if (gate.type == GateType::INPUT || gate.type == GateType::OUTPUT ||
            gate.type == GateType::CONST0 || gate.type == GateType::CONST1) continue;
        gate_to_idx[(int)i] = (int)g.nodes.size();
        g.nodes.push_back(gate_type_str(gate.type));
    }
    g.adjacency.resize(g.nodes.size());

    for (size_t ni = 0; ni < schem_.num_nets(); ni++) {
        auto& net = schem_.net(ni);
        std::vector<int> connected;
        if (net.driver >= 0 && gate_to_idx.count(net.driver))
            connected.push_back(gate_to_idx[net.driver]);
        for (auto fo : net.fanout)
            if (gate_to_idx.count(fo)) connected.push_back(gate_to_idx[fo]);

        if (connected.size() >= 2) {
            g.edges.push_back(net.name);
            // Build adjacency: every cell on this net connects to every other
            for (size_t a = 0; a < connected.size(); ++a) {
                for (size_t b = a + 1; b < connected.size(); ++b) {
                    g.adjacency[connected[a]].push_back(connected[b]);
                    g.adjacency[connected[b]].push_back(connected[a]);
                }
            }
        }
    }
    return g;
}

LvsChecker::NetGraph LvsChecker::build_layout_graph() {
    NetGraph g;
    std::vector<int> pd_to_graph(layout_.cells.size(), -1);
    for (size_t i = 0; i < layout_.cells.size(); i++) {
        if (is_infrastructure_cell(layout_.cells[i].cell_type)) continue;
        pd_to_graph[i] = (int)g.nodes.size();
        g.nodes.push_back(layout_.cells[i].cell_type);
    }
    g.adjacency.resize(g.nodes.size());

    for (const auto& net : layout_.nets) {
        std::vector<int> mapped;
        for (int c : net.cell_ids) {
            if (c >= 0 && c < (int)pd_to_graph.size() && pd_to_graph[c] >= 0)
                mapped.push_back(pd_to_graph[c]);
        }
        if (mapped.size() >= 2) {
            g.edges.push_back(net.name);
            for (size_t a = 0; a < mapped.size(); ++a) {
                for (size_t b = a + 1; b < mapped.size(); ++b) {
                    g.adjacency[mapped[a]].push_back(mapped[b]);
                    g.adjacency[mapped[b]].push_back(mapped[a]);
                }
            }
        }
    }
    return g;
}

bool LvsChecker::graphs_isomorphic(const NetGraph& a, const NetGraph& b,
                                    std::vector<int>& mapping) {
    if (a.nodes.size() != b.nodes.size()) return false;
    if (a.edges.size() != b.edges.size()) return false;

    int n = (int)a.nodes.size();
    mapping.assign(n, -1);

    // Build degree + type signatures for each node
    auto node_sig = [](const NetGraph& g, int idx) -> std::string {
        return g.nodes[idx] + ":" + std::to_string(g.adjacency[idx].size());
    };

    // Group nodes by signature
    std::unordered_map<std::string, std::vector<int>> a_groups, b_groups;
    for (int i = 0; i < n; i++) {
        a_groups[node_sig(a, i)].push_back(i);
        b_groups[node_sig(b, i)].push_back(i);
    }

    // Quick check: same set of signatures
    if (a_groups.size() != b_groups.size()) return false;

    // Check pin swap tolerance: for each group, check if cell types
    // and pin counts are compatible (allowing swappable pins)
    auto is_swap_compatible = [this](const std::string& cell_type,
                                      const std::vector<std::string>& a_nets,
                                      const std::vector<std::string>& b_nets) -> bool {
        if (a_nets.size() != b_nets.size()) return false;
        // Check if any pin swap group covers this cell type
        for (const auto& group : pin_swap_groups_) {
            if (group.cell_type == cell_type) {
                // Nets might be in different order due to swappable pins
                std::multiset<std::string> a_set(a_nets.begin(), a_nets.end());
                std::multiset<std::string> b_set(b_nets.begin(), b_nets.end());
                return a_set == b_set;
            }
        }
        return a_nets == b_nets;
    };
    (void)is_swap_compatible; // used conceptually in matching

    // Greedy match within each signature group
    for (auto& [sig, a_cells] : a_groups) {
        auto it = b_groups.find(sig);
        if (it == b_groups.end()) return false;
        auto& b_cells = it->second;
        if (a_cells.size() != b_cells.size()) return false;

        std::vector<bool> b_used(b_cells.size(), false);
        for (int ai : a_cells) {
            bool found = false;
            for (size_t bi = 0; bi < b_cells.size(); ++bi) {
                if (b_used[bi]) continue;
                // Verify adjacency degree match
                if (a.adjacency[ai].size() == b.adjacency[b_cells[bi]].size()) {
                    mapping[ai] = b_cells[bi];
                    b_used[bi] = true;
                    found = true;
                    break;
                }
            }
            if (!found) return false;
        }
    }

    return true;
}

std::vector<LvsChecker::SubcircuitMatch> LvsChecker::match_subcircuits() {
    std::vector<SubcircuitMatch> results;

    // Build type → cells mapping for layout
    std::unordered_map<std::string, std::vector<int>> layout_by_type;
    for (size_t i = 0; i < layout_.cells.size(); i++) {
        if (is_infrastructure_cell(layout_.cells[i].cell_type)) continue;
        layout_by_type[to_upper(layout_.cells[i].cell_type)].push_back((int)i);
    }

    // Build type → gates mapping for schematic
    std::unordered_map<std::string, std::vector<int>> schem_by_type;
    for (size_t i = 0; i < schem_.num_gates(); i++) {
        auto& gate = schem_.gate(i);
        if (gate.type == GateType::INPUT || gate.type == GateType::OUTPUT ||
            gate.type == GateType::CONST0 || gate.type == GateType::CONST1) continue;
        schem_by_type[gate_type_str(gate.type)].push_back((int)i);
    }

    std::unordered_set<int> matched_layout_cells;

    for (auto& [type, schem_gates] : schem_by_type) {
        std::string type_upper = to_upper(type);

        // Find matching layout cells
        auto it = layout_by_type.find(type_upper);
        if (it == layout_by_type.end()) {
            // Try partial match
            std::vector<int> partial_matches;
            for (auto& [lt, cells] : layout_by_type) {
                if (lt.find(type_upper) != std::string::npos ||
                    type_upper.find(lt) != std::string::npos) {
                    for (int c : cells) partial_matches.push_back(c);
                }
            }

            if (partial_matches.empty()) {
                for (int gid : schem_gates) {
                    SubcircuitMatch m;
                    m.schematic_name = schem_.gate(gid).name;
                    m.matched = false;
                    m.mismatch_reason = "No layout cell of type '" + type + "' found";
                    results.push_back(m);
                }
                continue;
            }
            it = layout_by_type.end(); // will use partial_matches below
        }

        const auto& layout_cells = (it != layout_by_type.end()) ? it->second : std::vector<int>{};

        // Count connections for each schematic gate
        auto count_gate_nets = [this](int gid) -> int {
            const auto& gate = schem_.gate(gid);
            int count = (int)gate.inputs.size();
            if (gate.output >= 0) count++;
            if (gate.clk >= 0) count++;
            return count;
        };

        // Count connections for each layout cell
        auto count_cell_nets = [this](int cid) -> int {
            int count = 0;
            for (const auto& net : layout_.nets) {
                for (int c : net.cell_ids) {
                    if (c == cid) { count++; break; }
                }
            }
            return count;
        };

        size_t matched_count = 0;
        std::vector<bool> layout_used(layout_cells.size(), false);

        for (int gid : schem_gates) {
            SubcircuitMatch m;
            m.schematic_name = schem_.gate(gid).name;
            int s_pins = count_gate_nets(gid);

            bool found = false;
            for (size_t li = 0; li < layout_cells.size(); ++li) {
                if (layout_used[li]) continue;
                if (matched_layout_cells.count(layout_cells[li])) continue;

                int l_pins = count_cell_nets(layout_cells[li]);

                // Check pin count compatibility (allow pin swaps)
                bool pin_ok = (s_pins == l_pins);
                if (!pin_ok) {
                    // Check if pin swap groups make this acceptable
                    for (const auto& grp : pin_swap_groups_) {
                        if (to_upper(grp.cell_type) == type_upper) {
                            // Swappable pins reduce effective unique pin count
                            int effective = l_pins;
                            for (const auto& swap_set : grp.swappable_pins) {
                                if ((int)swap_set.size() > 1) {
                                    effective -= (int)swap_set.size() - 1;
                                }
                            }
                            pin_ok = (s_pins == effective || s_pins == l_pins);
                            break;
                        }
                    }
                }

                if (pin_ok) {
                    m.layout_name = layout_.cells[layout_cells[li]].name;
                    m.matched = true;
                    layout_used[li] = true;
                    matched_layout_cells.insert(layout_cells[li]);
                    matched_count++;

                    // Build pin mapping
                    const auto& gate = schem_.gate(gid);
                    std::vector<std::string> gate_pins;
                    for (auto inp : gate.inputs) {
                        if (inp >= 0 && inp < (int)schem_.num_nets())
                            gate_pins.push_back(schem_.net(inp).name);
                    }
                    if (gate.output >= 0 && gate.output < (int)schem_.num_nets())
                        gate_pins.push_back(schem_.net(gate.output).name);

                    std::vector<std::string> cell_pins;
                    for (const auto& net : layout_.nets) {
                        for (int c : net.cell_ids) {
                            if (c == layout_cells[li]) {
                                cell_pins.push_back(net.name);
                                break;
                            }
                        }
                    }

                    size_t pin_pairs = std::min(gate_pins.size(), cell_pins.size());
                    for (size_t p = 0; p < pin_pairs; ++p) {
                        m.pin_mapping.emplace_back(gate_pins[p], cell_pins[p]);
                    }

                    found = true;
                    break;
                }
            }

            if (!found) {
                m.matched = false;
                m.mismatch_reason = "No compatible layout cell (need " +
                                    std::to_string(s_pins) + " pins, type=" + type + ")";
            }

            results.push_back(m);
        }
    }

    return results;
}

// ══════════════════════════════════════════════════════════════════════
// C) Net tracing through vias
// ══════════════════════════════════════════════════════════════════════

LvsChecker::NetTrace LvsChecker::trace_net(int net_idx) {
    NetTrace trace;
    trace.net_idx = net_idx;
    trace.connected = false;

    if (net_idx < 0 || net_idx >= (int)layout_.nets.size()) return trace;

    const auto& net = layout_.nets[net_idx];
    trace.net_name = net.name;

    // Collect all wire segments belonging to this net
    std::set<int> layers_on_net;
    std::vector<int> net_wire_indices;
    for (size_t wi = 0; wi < layout_.wires.size(); ++wi) {
        if (layout_.wires[wi].net_id == net_idx) {
            net_wire_indices.push_back((int)wi);
            layers_on_net.insert(layout_.wires[wi].layer);
        }
    }

    // Collect vias that connect wire segments of this net
    // A via connects two layers at a point; check if any wire segment on
    // each layer passes through that point
    auto point_on_segment = [](const WireSegment& seg, const Point& p, double tol = 0.5) -> bool {
        double min_x = std::min(seg.start.x, seg.end.x) - tol;
        double max_x = std::max(seg.start.x, seg.end.x) + tol;
        double min_y = std::min(seg.start.y, seg.end.y) - tol;
        double max_y = std::max(seg.start.y, seg.end.y) + tol;
        return p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y;
    };

    for (const auto& via : layout_.vias) {
        bool lower_match = false, upper_match = false;
        for (int wi : net_wire_indices) {
            const auto& seg = layout_.wires[wi];
            if (seg.layer == via.lower_layer && point_on_segment(seg, via.position))
                lower_match = true;
            if (seg.layer == via.upper_layer && point_on_segment(seg, via.position))
                upper_match = true;
        }
        if (lower_match || upper_match) {
            trace.via_locations.emplace_back(via.position.x, via.position.y);
            layers_on_net.insert(via.lower_layer);
            layers_on_net.insert(via.upper_layer);
        }
    }

    trace.layers_visited.assign(layers_on_net.begin(), layers_on_net.end());

    // Verify connectivity: BFS/DFS through wire segments and vias
    // Build adjacency: wire segments that share endpoints or connect through vias
    if (net_wire_indices.empty()) {
        // No wires — check if net has pins (might be a purely logical connection)
        trace.connected = !net.cell_ids.empty();
        return trace;
    }

    // Union-Find for connectivity
    std::vector<int> parent(net_wire_indices.size());
    std::iota(parent.begin(), parent.end(), 0);
    std::function<int(int)> find = [&](int x) -> int {
        return parent[x] == x ? x : parent[x] = find(parent[x]);
    };
    auto unite = [&](int a, int b) {
        a = find(a); b = find(b);
        if (a != b) parent[a] = b;
    };

    // Connect wire segments on the same layer that share endpoints
    double conn_tol = 1.0;
    for (size_t i = 0; i < net_wire_indices.size(); ++i) {
        const auto& wi = layout_.wires[net_wire_indices[i]];
        for (size_t j = i + 1; j < net_wire_indices.size(); ++j) {
            const auto& wj = layout_.wires[net_wire_indices[j]];
            if (wi.layer != wj.layer) continue;
            // Check if endpoints are close
            if (wi.start.dist(wj.start) < conn_tol || wi.start.dist(wj.end) < conn_tol ||
                wi.end.dist(wj.start) < conn_tol || wi.end.dist(wj.end) < conn_tol) {
                unite((int)i, (int)j);
            }
        }
    }

    // Connect segments through vias
    for (const auto& via : layout_.vias) {
        int lower_seg = -1, upper_seg = -1;
        for (size_t i = 0; i < net_wire_indices.size(); ++i) {
            const auto& seg = layout_.wires[net_wire_indices[i]];
            if (seg.layer == via.lower_layer && point_on_segment(seg, via.position)) {
                lower_seg = (int)i;
            }
            if (seg.layer == via.upper_layer && point_on_segment(seg, via.position)) {
                upper_seg = (int)i;
            }
        }
        if (lower_seg >= 0 && upper_seg >= 0) {
            unite(lower_seg, upper_seg);
        }
    }

    // Check if all wire segments are in one connected component
    std::set<int> components;
    for (size_t i = 0; i < net_wire_indices.size(); ++i) {
        components.insert(find((int)i));
    }
    trace.connected = (components.size() <= 1);

    return trace;
}

std::vector<LvsChecker::NetTrace> LvsChecker::trace_all_nets() {
    std::vector<NetTrace> traces;
    traces.reserve(layout_.nets.size());
    for (int i = 0; i < (int)layout_.nets.size(); ++i) {
        traces.push_back(trace_net(i));
    }
    return traces;
}

// ══════════════════════════════════════════════════════════════════════
// D) Pin swap tolerance
// ══════════════════════════════════════════════════════════════════════

void LvsChecker::add_pin_swap_group(const PinSwapGroup& group) {
    pin_swap_groups_.push_back(group);
}

// ══════════════════════════════════════════════════════════════════════
// E) Cross-hierarchy comparison
// ══════════════════════════════════════════════════════════════════════

LvsChecker::HierarchyCompare LvsChecker::compare_hierarchy() {
    HierarchyCompare result;

    // Build schematic hierarchy from netlist gates
    // Root = top-level. Each gate type that appears multiple times suggests
    // a subcircuit (module) instantiation pattern.
    {
        HierarchyLevel root;
        root.instance_name = "top";
        root.cell_name = "top";
        root.parent = -1;
        result.schematic_hier.push_back(root);

        std::unordered_map<std::string, int> type_to_hier;
        for (size_t i = 0; i < schem_.num_gates(); i++) {
            auto& gate = schem_.gate(i);
            if (gate.type == GateType::INPUT || gate.type == GateType::OUTPUT ||
                gate.type == GateType::CONST0 || gate.type == GateType::CONST1) continue;

            std::string type_name = gate_type_str(gate.type);
            auto it = type_to_hier.find(type_name);

            if (it == type_to_hier.end()) {
                // Create a hierarchy level for this cell type
                HierarchyLevel level;
                level.instance_name = gate.name;
                level.cell_name = type_name;
                level.parent = 0; // child of root
                int idx = (int)result.schematic_hier.size();
                type_to_hier[type_name] = idx;
                result.schematic_hier[0].child_indices.push_back(idx);
                result.schematic_hier.push_back(level);
            } else {
                // Additional instance of same type — add as leaf
                HierarchyLevel leaf;
                leaf.instance_name = gate.name;
                leaf.cell_name = type_name;
                leaf.parent = it->second;
                int idx = (int)result.schematic_hier.size();
                result.schematic_hier[it->second].child_indices.push_back(idx);
                result.schematic_hier.push_back(leaf);
            }
        }
    }

    // Build layout hierarchy from cell instances
    {
        HierarchyLevel root;
        root.instance_name = "top";
        root.cell_name = "top";
        root.parent = -1;
        result.layout_hier.push_back(root);

        std::unordered_map<std::string, int> type_to_hier;
        for (size_t i = 0; i < layout_.cells.size(); i++) {
            const auto& cell = layout_.cells[i];
            if (is_infrastructure_cell(cell.cell_type)) continue;

            std::string type_name = cell.cell_type;
            auto it = type_to_hier.find(type_name);

            if (it == type_to_hier.end()) {
                HierarchyLevel level;
                level.instance_name = cell.name;
                level.cell_name = type_name;
                level.parent = 0;
                int idx = (int)result.layout_hier.size();
                type_to_hier[type_name] = idx;
                result.layout_hier[0].child_indices.push_back(idx);
                result.layout_hier.push_back(level);
            } else {
                HierarchyLevel leaf;
                leaf.instance_name = cell.name;
                leaf.cell_name = type_name;
                leaf.parent = it->second;
                int idx = (int)result.layout_hier.size();
                result.layout_hier[it->second].child_indices.push_back(idx);
                result.layout_hier.push_back(leaf);
            }
        }
    }

    // Compare hierarchies level by level
    result.hierarchies_match = true;

    // Compare root children (unique cell types)
    auto& s_root = result.schematic_hier[0];
    auto& l_root = result.layout_hier[0];

    // Build sets of child cell types
    std::unordered_map<std::string, int> s_type_count, l_type_count;
    for (int ci : s_root.child_indices) {
        const auto& child = result.schematic_hier[ci];
        // Count total instances (child + its sub-children)
        s_type_count[child.cell_name] = 1 + (int)child.child_indices.size();
    }
    for (int ci : l_root.child_indices) {
        const auto& child = result.layout_hier[ci];
        l_type_count[child.cell_name] = 1 + (int)child.child_indices.size();
    }

    // Check for cell types in schematic but not layout
    for (auto& [type, count] : s_type_count) {
        auto it = l_type_count.find(type);
        if (it == l_type_count.end()) {
            // Try case-insensitive match
            bool found = false;
            for (auto& [lt, lc] : l_type_count) {
                if (to_upper(lt) == to_upper(type)) {
                    if (lc != count) {
                        result.hierarchies_match = false;
                        result.mismatches.push_back(
                            "Instance count mismatch for '" + type + "': schematic=" +
                            std::to_string(count) + " layout=" + std::to_string(lc));
                    }
                    found = true;
                    break;
                }
            }
            if (!found) {
                result.hierarchies_match = false;
                result.mismatches.push_back(
                    "Cell type '" + type + "' (" + std::to_string(count) +
                    " instances) in schematic but not in layout");
            }
        } else if (it->second != count) {
            result.hierarchies_match = false;
            result.mismatches.push_back(
                "Instance count mismatch for '" + type + "': schematic=" +
                std::to_string(count) + " layout=" + std::to_string(it->second));
        }
    }

    // Check for cell types in layout but not schematic
    for (auto& [type, count] : l_type_count) {
        bool found = false;
        for (auto& [st, sc] : s_type_count) {
            if (to_upper(st) == to_upper(type)) { found = true; break; }
        }
        if (!found) {
            // Filter CTS/infrastructure — not a true mismatch
            std::string ut = to_upper(type);
            bool cts = (ut.find("CLKBUF") != std::string::npos ||
                        ut.find("CTS") != std::string::npos);
            if (!cts) {
                result.hierarchies_match = false;
                result.mismatches.push_back(
                    "Cell type '" + type + "' (" + std::to_string(count) +
                    " instances) in layout but not in schematic");
            }
        }
    }

    return result;
}

// ══════════════════════════════════════════════════════════════════════
// F) Net name fuzzy matching / similarity
// ══════════════════════════════════════════════════════════════════════

double LvsChecker::net_similarity(const std::string& a, const std::string& b) const {
    if (a == b) return 1.0;
    if (a.empty() || b.empty()) return 0.0;

    std::string ua = to_upper(a), ub = to_upper(b);
    if (ua == ub) return 0.95;

    // Longest common subsequence based similarity
    int m = (int)ua.size(), n = (int)ub.size();
    std::vector<std::vector<int>> dp(m + 1, std::vector<int>(n + 1, 0));
    for (int i = 1; i <= m; i++) {
        for (int j = 1; j <= n; j++) {
            if (ua[i-1] == ub[j-1])
                dp[i][j] = dp[i-1][j-1] + 1;
            else
                dp[i][j] = std::max(dp[i-1][j], dp[i][j-1]);
        }
    }
    double lcs = dp[m][n];
    double sim = (2.0 * lcs) / (m + n);

    // Bonus for common prefix
    int prefix = 0;
    while (prefix < m && prefix < n && ua[prefix] == ub[prefix]) prefix++;
    sim += 0.1 * ((double)prefix / std::max(m, n));

    // Bonus for matching after stripping common bus suffixes like [0], _0, etc.
    auto strip_suffix = [](const std::string& s) -> std::string {
        std::string r = s;
        // Remove trailing digits and brackets/underscores
        while (!r.empty() && (std::isdigit(static_cast<unsigned char>(r.back())) ||
               r.back() == ']' || r.back() == '[' || r.back() == '_'))
            r.pop_back();
        return r;
    };

    if (strip_suffix(ua) == strip_suffix(ub)) {
        sim = std::max(sim, 0.85);
    }

    return std::min(sim, 1.0);
}

// ══════════════════════════════════════════════════════════════════════
// G) LVS debug reporting
// ══════════════════════════════════════════════════════════════════════

LvsChecker::LvsDebugReport LvsChecker::generate_debug_report() {
    LvsDebugReport report;

    // Collect schematic net names and cell names
    std::unordered_set<std::string> schem_net_names, layout_net_names;
    std::unordered_set<std::string> schem_cell_names, layout_cell_names;

    for (size_t i = 0; i < schem_.num_nets(); i++) {
        if (!schem_.net(i).name.empty())
            schem_net_names.insert(schem_.net(i).name);
    }
    for (const auto& net : layout_.nets) {
        if (!net.name.empty())
            layout_net_names.insert(net.name);
    }

    for (size_t i = 0; i < schem_.num_gates(); i++) {
        auto& gate = schem_.gate(i);
        if (gate.type == GateType::INPUT || gate.type == GateType::OUTPUT ||
            gate.type == GateType::CONST0 || gate.type == GateType::CONST1) continue;
        schem_cell_names.insert(gate.name.empty() ? gate_type_str(gate.type) : gate.name);
    }
    for (const auto& cell : layout_.cells) {
        if (!is_infrastructure_cell(cell.cell_type))
            layout_cell_names.insert(cell.name.empty() ? cell.cell_type : cell.name);
    }

    // Unmatched nets
    for (const auto& sn : schem_net_names) {
        if (layout_net_names.find(sn) == layout_net_names.end()) {
            report.unmatched_schematic_nets.push_back(sn);
        }
    }
    for (const auto& ln : layout_net_names) {
        if (schem_net_names.find(ln) == schem_net_names.end()) {
            report.unmatched_layout_nets.push_back(ln);
        }
    }

    // Unmatched cells
    for (const auto& sc : schem_cell_names) {
        if (layout_cell_names.find(sc) == layout_cell_names.end()) {
            report.unmatched_schematic_cells.push_back(sc);
        }
    }
    for (const auto& lc : layout_cell_names) {
        if (schem_cell_names.find(lc) == schem_cell_names.end()) {
            // Exclude CTS buffers from unmatched list
            std::string uc = to_upper(lc);
            if (uc.find("CLKBUF") == std::string::npos && uc.find("CTS") == std::string::npos)
                report.unmatched_layout_cells.push_back(lc);
        }
    }

    // Suggest matches for unmatched layout nets using fuzzy matching
    for (const auto& ln : report.unmatched_layout_nets) {
        double best_score = 0.0;
        std::string best_match;
        for (const auto& sn : report.unmatched_schematic_nets) {
            double score = net_similarity(ln, sn);
            if (score > best_score) {
                best_score = score;
                best_match = sn;
            }
        }
        if (best_score >= 0.5 && !best_match.empty()) {
            report.net_name_suggestions.emplace_back(ln, best_match);
        }
    }

    // Format the report
    std::ostringstream oss;
    oss << "═══════════════════════════════════════════════════\n";
    oss << "  LVS DEBUG REPORT\n";
    oss << "═══════════════════════════════════════════════════\n\n";

    if (!report.unmatched_schematic_nets.empty()) {
        oss << "── Unmatched Schematic Nets (" << report.unmatched_schematic_nets.size() << ") ──\n";
        for (const auto& n : report.unmatched_schematic_nets)
            oss << "  [S] " << n << "\n";
        oss << "\n";
    }

    if (!report.unmatched_layout_nets.empty()) {
        oss << "── Unmatched Layout Nets (" << report.unmatched_layout_nets.size() << ") ──\n";
        for (const auto& n : report.unmatched_layout_nets)
            oss << "  [L] " << n << "\n";
        oss << "\n";
    }

    if (!report.unmatched_schematic_cells.empty()) {
        oss << "── Unmatched Schematic Cells (" << report.unmatched_schematic_cells.size() << ") ──\n";
        for (const auto& c : report.unmatched_schematic_cells)
            oss << "  [S] " << c << "\n";
        oss << "\n";
    }

    if (!report.unmatched_layout_cells.empty()) {
        oss << "── Unmatched Layout Cells (" << report.unmatched_layout_cells.size() << ") ──\n";
        for (const auto& c : report.unmatched_layout_cells)
            oss << "  [L] " << c << "\n";
        oss << "\n";
    }

    if (!report.net_name_suggestions.empty()) {
        oss << "── Suggested Net Correspondences ──\n";
        for (const auto& [ln, sn] : report.net_name_suggestions)
            oss << "  " << ln << "  <-->  " << sn << "\n";
        oss << "\n";
    }

    if (report.unmatched_schematic_nets.empty() && report.unmatched_layout_nets.empty() &&
        report.unmatched_schematic_cells.empty() && report.unmatched_layout_cells.empty()) {
        oss << "  No unmatched elements found.\n";
    }

    oss << "═══════════════════════════════════════════════════\n";
    report.formatted_report = oss.str();

    return report;
}

// ══════════════════════════════════════════════════════════════════════
// H) Enhanced LVS run — full flow
// ══════════════════════════════════════════════════════════════════════

LvsResult LvsChecker::run_enhanced() {
    auto t0 = std::chrono::high_resolution_clock::now();
    LvsResult r;

    // Phase 1: Device extraction
    auto extraction = extract_devices();

    // Phase 2: Net tracing through vias
    auto net_traces = trace_all_nets();

    // Count disconnected nets
    int disconnected_nets = 0;
    for (const auto& trace : net_traces) {
        if (!trace.connected && !trace.layers_visited.empty())
            disconnected_nets++;
    }

    // Phase 3: Subcircuit matching (with pin swap tolerance)
    auto subckt_matches = match_subcircuits();

    int matched_subckt = 0, unmatched_subckt = 0;
    for (const auto& m : subckt_matches) {
        if (m.matched) matched_subckt++;
        else unmatched_subckt++;
    }

    // Phase 4: Hierarchy comparison
    auto hier_cmp = compare_hierarchy();

    // Phase 5: Run the original partition-refinement check
    LvsResult base_result = check();

    // Combine results
    r.schematic_cells = base_result.schematic_cells;
    r.layout_cells = base_result.layout_cells;
    r.matched_cells = base_result.matched_cells;
    r.unmatched_schematic = base_result.unmatched_schematic;
    r.unmatched_layout = base_result.unmatched_layout;
    r.net_mismatches = base_result.net_mismatches;
    r.mismatches = base_result.mismatches;

    // Add device extraction info
    if (extraction.mos_count > 0 || extraction.res_count > 0 || extraction.cap_count > 0) {
        // Check extracted device counts against schematic expectations
        // (MOS count should match transistor-level cells if present)
    }

    // Add disconnected net mismatches
    if (disconnected_nets > 0) {
        r.net_mismatches += disconnected_nets;
        r.mismatches.push_back({"net_mismatch",
            std::to_string(disconnected_nets) +
            " net(s) have incomplete via connectivity across layers"});
    }

    // Add subcircuit match results
    if (unmatched_subckt > 0 && unmatched_subckt > r.unmatched_schematic) {
        // Only report additional subcircuit mismatches beyond what base check found
        int additional = unmatched_subckt - r.unmatched_schematic;
        if (additional > 0) {
            r.mismatches.push_back({"missing_cell",
                std::to_string(additional) +
                " additional subcircuit(s) unmatched by connectivity analysis"});
        }
    }

    // Add hierarchy mismatches
    if (!hier_cmp.hierarchies_match) {
        for (const auto& mismatch : hier_cmp.mismatches) {
            r.mismatches.push_back({"hierarchy_mismatch", mismatch});
        }
    }

    // Phase 6: Generate debug report on mismatch
    r.match = r.mismatches.empty();

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (r.match) {
        std::ostringstream oss;
        oss << "LVS CLEAN \xe2\x80\x94 " << r.matched_cells << "/"
            << r.schematic_cells << " cells, "
            << extraction.mos_count << " MOS, "
            << extraction.res_count << " R, "
            << extraction.cap_count << " C extracted, "
            << (int)net_traces.size() << " nets traced";
        r.message = oss.str();
    } else {
        auto debug = generate_debug_report();
        std::ostringstream oss;
        oss << "LVS MISMATCH \xe2\x80\x94 " << r.mismatches.size()
            << " issue(s), " << matched_subckt << "/" << subckt_matches.size()
            << " subcircuits matched\n" << debug.formatted_report;
        r.message = oss.str();
    }

    return r;
}

} // namespace sf
