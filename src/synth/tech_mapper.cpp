// SiliconForge — Technology Mapper Implementation
// Truth-table-based cell matching with cut enumeration
#include "synth/tech_mapper.hpp"
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <functional>
#include <numeric>

namespace sf {

// Compute truth table for AIG subgraph from root to leaves
static uint16_t compute_node_tt(const AigGraph& aig, uint32_t root,
                                 const std::vector<uint32_t>& leaves) {
    uint16_t tt = 0;
    int n = (int)leaves.size();
    if (n > 4 || n == 0) return 0;

    for (int pattern = 0; pattern < (1 << n); ++pattern) {
        std::unordered_map<uint32_t, bool> vals;
        vals[0] = false;
        for (int i = 0; i < n; ++i)
            vals[leaves[i]] = (pattern >> i) & 1;

        std::function<bool(AigLit)> eval = [&](AigLit lit) -> bool {
            uint32_t v = aig_var(lit);
            if (vals.count(v)) return vals[v] ^ aig_sign(lit);
            if (!aig.is_and(v)) return aig_sign(lit);
            const auto& nd = aig.and_node(v);
            bool v0 = eval(nd.fanin0);
            bool v1 = eval(nd.fanin1);
            vals[v] = v0 && v1;
            return vals[v] ^ aig_sign(lit);
        };

        if (eval(aig_make(root))) tt |= (1 << pattern);
    }
    return tt;
}

// Normalize a Boolean function string to a truth table
static uint16_t func_to_tt(const std::string& func, int num_inputs) {
    // Parse simple Boolean expressions: A & B, !(A | B), A ^ B, etc.
    // Input names: A=0, B=1, C=2, D=3
    auto char_to_idx = [](char c) -> int {
        if (c >= 'A' && c <= 'D') return c - 'A';
        if (c >= 'a' && c <= 'd') return c - 'a';
        return -1;
    };

    uint16_t tt = 0;
    for (int pattern = 0; pattern < (1 << num_inputs); ++pattern) {
        // Evaluate the function string for this pattern
        bool inputs[4] = {false};
        for (int i = 0; i < num_inputs; ++i)
            inputs[i] = (pattern >> i) & 1;

        // Simple recursive evaluator
        std::function<bool(const std::string&, size_t&)> eval_expr;
        std::function<bool(const std::string&, size_t&)> eval_term;
        std::function<bool(const std::string&, size_t&)> eval_primary;

        eval_primary = [&](const std::string& s, size_t& pos) -> bool {
            while (pos < s.size() && s[pos] == ' ') pos++;
            if (pos >= s.size()) return false;

            if (s[pos] == '!') {
                pos++;
                return !eval_primary(s, pos);
            }
            if (s[pos] == '(') {
                pos++;
                bool val = eval_expr(s, pos);
                if (pos < s.size() && s[pos] == ')') pos++;
                return val;
            }
            int idx = char_to_idx(s[pos]);
            if (idx >= 0 && idx < num_inputs) {
                pos++;
                return inputs[idx];
            }
            if (s[pos] == '1') { pos++; return true; }
            if (s[pos] == '0') { pos++; return false; }
            pos++;
            return false;
        };

        eval_term = [&](const std::string& s, size_t& pos) -> bool {
            bool val = eval_primary(s, pos);
            while (pos < s.size()) {
                while (pos < s.size() && s[pos] == ' ') pos++;
                if (pos >= s.size()) break;
                if (s[pos] == '&') { pos++; val = val & eval_primary(s, pos); }
                else if (s[pos] == '*') { pos++; val = val & eval_primary(s, pos); }
                else break;
            }
            return val;
        };

        eval_expr = [&](const std::string& s, size_t& pos) -> bool {
            bool val = eval_term(s, pos);
            while (pos < s.size()) {
                while (pos < s.size() && s[pos] == ' ') pos++;
                if (pos >= s.size()) break;
                if (s[pos] == '|') { pos++; val = val | eval_term(s, pos); }
                else if (s[pos] == '+') { pos++; val = val | eval_term(s, pos); }
                else if (s[pos] == '^') { pos++; val = val ^ eval_term(s, pos); }
                else break;
            }
            return val;
        };

        size_t pos = 0;
        if (eval_expr(func, pos))
            tt |= (1 << pattern);
    }
    return tt;
}

// Check if two truth tables match under input permutation
static bool tt_match_permuted(uint16_t cell_tt, uint16_t node_tt, int n,
                               std::vector<int>& perm) {
    // Try all permutations of n inputs
    std::vector<int> p(n);
    std::iota(p.begin(), p.end(), 0);

    do {
        // Apply permutation to node_tt
        uint16_t permuted = 0;
        for (int pattern = 0; pattern < (1 << n); ++pattern) {
            int new_pattern = 0;
            for (int i = 0; i < n; ++i) {
                if ((pattern >> p[i]) & 1)
                    new_pattern |= (1 << i);
            }
            if ((node_tt >> pattern) & 1)
                permuted |= (1 << new_pattern);
        }
        if (permuted == cell_tt) {
            perm = p;
            return true;
        }
    } while (std::next_permutation(p.begin(), p.end()));

    return false;
}

TechMapper::CellMatch TechMapper::match_node(uint32_t var) {
    if (!aig_.is_and(var)) return {nullptr, {}, AIG_FALSE};

    const auto& nd = aig_.and_node(var);
    CellMatch best{nullptr, {}, aig_make(var)};

    // Enumerate small cuts for this node
    std::vector<std::vector<uint32_t>> cuts;

    // 2-input cut: direct fanins
    uint32_t v0 = aig_var(nd.fanin0), v1 = aig_var(nd.fanin1);
    if (v0 != 0 && v1 != 0 && v0 != v1)
        cuts.push_back({v0, v1});
    else if (v0 != 0)
        cuts.push_back({v0});

    // 3-input cuts: expand one fanin
    if (aig_.is_and(v0)) {
        const auto& n0 = aig_.and_node(v0);
        uint32_t a = aig_var(n0.fanin0), b = aig_var(n0.fanin1);
        std::vector<uint32_t> c3 = {a, b, v1};
        // Remove duplicates and zeros
        c3.erase(std::remove(c3.begin(), c3.end(), 0u), c3.end());
        std::sort(c3.begin(), c3.end());
        c3.erase(std::unique(c3.begin(), c3.end()), c3.end());
        if (c3.size() == 3) cuts.push_back(c3);
    }
    if (aig_.is_and(v1)) {
        const auto& n1 = aig_.and_node(v1);
        uint32_t a = aig_var(n1.fanin0), b = aig_var(n1.fanin1);
        std::vector<uint32_t> c3 = {v0, a, b};
        c3.erase(std::remove(c3.begin(), c3.end(), 0u), c3.end());
        std::sort(c3.begin(), c3.end());
        c3.erase(std::unique(c3.begin(), c3.end()), c3.end());
        if (c3.size() == 3) cuts.push_back(c3);
    }

    // Try each cut against each library cell
    double best_area = 1e9;

    for (auto& cut_leaves : cuts) {
        int n = (int)cut_leaves.size();
        if (n > 2) continue; // Only match 2-input cells (3+ input causes misclassification)
        uint16_t node_tt = compute_node_tt(aig_, var, cut_leaves);

        for (auto& cell : lib_.cells) {
            int cell_inputs = cell.num_inputs();
            if (cell_inputs != n) continue;

            // Get cell truth table
            std::string func = cell.output_function();
            if (func.empty()) continue;

            uint16_t cell_tt = func_to_tt(func, cell_inputs);

            // Direct match
            if (cell_tt == node_tt && cell.area < best_area) {
                best.cell = &cell;
                best.inputs.clear();
                for (auto l : cut_leaves) best.inputs.push_back(aig_make(l));
                best.output = aig_make(var); // non-inverted
                best_area = cell.area;
                continue;
            }

            // Match with complement (output inverted)
            uint16_t mask = (1 << (1 << n)) - 1;
            uint16_t inv_node_tt = (~node_tt) & mask;
            if (cell_tt == inv_node_tt && cell.area < best_area) {
                best.cell = &cell;
                best.inputs.clear();
                for (auto l : cut_leaves) best.inputs.push_back(aig_make(l));
                best.output = aig_not(aig_make(var)); // mark as inverted
                best_area = cell.area;
                continue;
            }

            // Match with input permutation
            std::vector<int> perm;
            if (tt_match_permuted(cell_tt, node_tt, n, perm) && cell.area < best_area) {
                best.cell = &cell;
                best.inputs.clear();
                for (int i = 0; i < n; ++i)
                    best.inputs.push_back(aig_make(cut_leaves[perm[i]]));
                best.output = aig_make(var);
                best_area = cell.area;
                continue;
            }
        }
    }

    // Fallback: simple AND2 decomposition
    if (!best.cell) {
        // Find AND2 cell
        for (auto& c : lib_.cells) {
            if (c.num_inputs() == 2) {
                std::string func = c.output_function();
                uint16_t tt = func_to_tt(func, 2);
                if (tt == 0x8) { // A & B
                    best.cell = &c;
                    best.inputs = {nd.fanin0, nd.fanin1};
                    best.output = aig_make(var);
                    break;
                }
            }
        }
        // If still nothing, use NAND2
        if (!best.cell) {
            for (auto& c : lib_.cells) {
                if (c.num_inputs() == 2) {
                    std::string func = c.output_function();
                    uint16_t tt = func_to_tt(func, 2);
                    if (tt == 0x7) { // NAND2
                        best.cell = &c;
                        best.inputs = {nd.fanin0, nd.fanin1};
                        best.output = aig_not(aig_make(var)); // inverted
                        break;
                    }
                }
            }
        }
        // Last resort: any 2-input cell
        if (!best.cell) {
            for (auto& c : lib_.cells) {
                if (c.num_inputs() == 2) {
                    best.cell = &c;
                    best.inputs = {nd.fanin0, nd.fanin1};
                    best.output = aig_make(var);
                    break;
                }
            }
        }
    }

    return best;
}

Netlist TechMapper::build_netlist(const std::vector<CellMatch>& matches) {
    Netlist nl;
    std::unordered_map<uint32_t, NetId> var_to_net;
    std::unordered_map<uint32_t, bool> var_inverted; // track inverted outputs

    // Create nets for inputs
    for (size_t i = 0; i < aig_.num_inputs(); ++i) {
        uint32_t v = aig_.inputs()[i];
        std::string iname = i < aig_.input_names().size() ? aig_.input_names()[i] : ("i" + std::to_string(i));
        NetId n = nl.add_net(iname);
        nl.mark_input(n);
        var_to_net[v] = n;
    }

    var_to_net[0] = nl.add_net("GND");
    nl.add_gate(GateType::CONST0, {}, var_to_net[0], "CONST0");

    // Track which nodes need their output inverted
    for (auto& m : matches) {
        if (m.output != AIG_FALSE && aig_sign(m.output)) {
            var_inverted[aig_var(m.output)] = true;
        }
    }

    auto get_net = [&](AigLit lit) -> NetId {
        uint32_t v = aig_var(lit);
        bool need_inv = aig_sign(lit);

        // If the cell already produces inverted output, cancel
        if (var_inverted.count(v) && var_inverted[v])
            need_inv = !need_inv;

        if (!var_to_net.count(v))
            var_to_net[v] = nl.add_net("n" + std::to_string(v));

        NetId base = var_to_net[v];
        if (need_inv) {
            std::string inv_name = "inv_n" + std::to_string(v) + (aig_sign(lit) ? "_c" : "");
            NetId inv_out = nl.add_net(inv_name);
            nl.add_gate(GateType::NOT, {base}, inv_out, "INV_" + std::to_string(v));
            return inv_out;
        }
        return base;
    };

    // Create gates from matches
    for (auto& m : matches) {
        if (!m.cell) continue;

        uint32_t out_var = aig_var(m.output);
        if (!var_to_net.count(out_var))
            var_to_net[out_var] = nl.add_net("n" + std::to_string(out_var));

        std::vector<NetId> in_nets;
        for (auto lit : m.inputs) {
            in_nets.push_back(get_net(lit));
        }

        // Determine gate type from cell function truth table
        std::string func = m.cell->output_function();
        int ni = m.cell->num_inputs();
        uint16_t tt = func_to_tt(func, ni);

        GateType gtype = GateType::AND;
        if (ni == 1) {
            gtype = (tt == 0x1) ? GateType::NOT : GateType::BUF;
        } else if (ni == 2) {
            switch (tt) {
                case 0x8: gtype = GateType::AND; break;   // A & B
                case 0x7: gtype = GateType::NAND; break;  // !(A & B)
                case 0xE: gtype = GateType::OR; break;    // A | B
                case 0x1: gtype = GateType::NOR; break;   // !(A | B)
                case 0x6: gtype = GateType::XOR; break;   // A ^ B
                case 0x9: gtype = GateType::XNOR; break;  // !(A ^ B)
                default:  gtype = GateType::AND; break;
            }
        } else if (ni == 3) {
            // Complex cells
            if (func.find("!(") == 0 && func.find('&') != std::string::npos && func.find('|') != std::string::npos) {
                gtype = GateType::NAND; // AOI or OAI
            } else if (func.find('&') != std::string::npos) {
                gtype = GateType::AND;
            } else if (func.find('|') != std::string::npos) {
                gtype = GateType::OR;
            }
        }

        nl.add_gate(gtype, in_nets, var_to_net[out_var], m.cell->name + "_i" + std::to_string(out_var));
        stats_.total_area += m.cell->area;
        stats_.num_cells++;
    }

    // Mark outputs — preserve original AIG output names
    // Track which var_to_net entries are already used as a named output
    std::unordered_set<uint32_t> named_output_vars;
    for (size_t i = 0; i < aig_.num_outputs(); ++i) {
        AigLit olit = aig_.outputs()[i];
        uint32_t v = aig_var(olit);
        std::string oname = i < aig_.output_names().size() ? aig_.output_names()[i] : "out_" + std::to_string(i);

        if (!var_to_net.count(v))
            var_to_net[v] = nl.add_net(oname.empty() ? "o" + std::to_string(i) : oname);

        NetId out_net = var_to_net[v];
        bool need_inv = aig_sign(olit);
        if (var_inverted.count(v) && var_inverted[v]) need_inv = !need_inv;

        // If this var was already used as a different output, add a BUF/NOT to create unique net
        bool shared_var = named_output_vars.count(v) > 0;
        named_output_vars.insert(v);

        if (need_inv) {
            NetId inv_out = nl.add_net(oname.empty() ? "out_" + std::to_string(i) : oname);
            nl.add_gate(GateType::NOT, {out_net}, inv_out, "OUT_INV_" + std::to_string(i));
            nl.mark_output(inv_out);
            stats_.num_cells++;
        } else if (shared_var) {
            // Multiple outputs alias same AIG node — add BUF to separate names
            NetId buf_out = nl.add_net(oname.empty() ? "out_" + std::to_string(i) : oname);
            nl.add_gate(GateType::BUF, {out_net}, buf_out, "OUT_BUF_" + std::to_string(i));
            nl.mark_output(buf_out);
        } else {
            if (!oname.empty() && nl.net(out_net).name != oname) {
                nl.net(out_net).name = oname;
            }
            nl.mark_output(out_net);
        }
    }

    return nl;
}

Netlist TechMapper::map(bool optimize_area) {
    auto t0 = std::chrono::high_resolution_clock::now();
    stats_ = {};

    // Match each AND gate to a cell
    std::vector<CellMatch> matches;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v)) continue;
        auto m = match_node(v);
        if (m.cell) matches.push_back(m);
    }

    auto nl = build_netlist(matches);

    stats_.depth = 0;
    auto topo = nl.topo_order();
    stats_.depth = topo.size();

    auto t1 = std::chrono::high_resolution_clock::now();
    stats_.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Post-mapping: insert buffers for high-fanout nets
    insert_buffers(nl);

    return nl;
}

void TechMapper::insert_buffers(Netlist& nl, int max_fanout) {
    // For each net with fanout > max_fanout, insert a buffer tree
    // This reduces load capacitance on the driver and improves timing
    int bufs_inserted = 0;
    
    for (size_t ni = 0; ni < nl.num_nets(); ni++) {
        auto& net = nl.net(ni);
        int fo = (int)net.fanout.size();
        if (fo <= max_fanout) continue;
        
        // Need ceil(fo / max_fanout) buffers
        int num_bufs = (fo + max_fanout - 1) / max_fanout;
        if (num_bufs <= 1) continue;
        
        // Create buffer gates that split the fanout
        std::vector<GateId> original_fanout(net.fanout.begin(), net.fanout.end());
        net.fanout.clear();
        
        for (int b = 0; b < num_bufs; b++) {
            // Create intermediate net
            NetId buf_out = nl.add_net("buf_" + net.name + "_" + std::to_string(b));
            // Create buffer gate
            GateId buf = nl.add_gate(GateType::BUF, {(int)ni}, buf_out,
                                     "BUF_X2_" + std::to_string(ni) + "_" + std::to_string(b));
            
            // Assign portion of original fanout to this buffer's output
            int start = b * max_fanout;
            int end = std::min(start + max_fanout, fo);
            for (int i = start; i < end; i++) {
                // Reconnect: change input of fanout gate from original net to buf_out
                auto& fg = nl.gate(original_fanout[i]);
                for (auto& inp : fg.inputs) {
                    if (inp == (int)ni) { inp = buf_out; break; }
                }
                nl.net(buf_out).fanout.push_back(original_fanout[i]);
            }
            
            net.fanout.push_back(buf); // Original net drives the buffer
            bufs_inserted++;
        }
    }
    
    if (bufs_inserted > 0)
        std::cout << "  [Buffer] Inserted " << bufs_inserted << " buffers for high-fanout nets\n";
}

} // namespace sf
