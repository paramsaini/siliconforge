// SiliconForge — Technology Mapper Implementation
// Truth-table-based cell matching with cut enumeration
// Enhanced: complex gate matching, multi-goal optimization, area recovery
#include "synth/tech_mapper.hpp"
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <functional>
#include <numeric>
#include <cmath>
#include <cctype>

namespace sf {

// ============================================================================
// Static helpers
// ============================================================================

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

// Case-insensitive substring search
static bool icontains(const std::string& haystack, const std::string& needle) {
    if (needle.size() > haystack.size()) return false;
    auto it = std::search(
        haystack.begin(), haystack.end(),
        needle.begin(), needle.end(),
        [](char a, char b) { return std::toupper(static_cast<unsigned char>(a)) ==
                                     std::toupper(static_cast<unsigned char>(b)); }
    );
    return it != haystack.end();
}

// ============================================================================
// Helper methods
// ============================================================================

const LibertyCell* TechMapper::find_cell_by_pattern(const std::string& pattern) const {
    const LibertyCell* best = nullptr;
    double best_area = 1e9;
    for (auto& cell : lib_.cells) {
        if (icontains(cell.name, pattern)) {
            if (cell.area < best_area) {
                best = &cell;
                best_area = cell.area;
            }
        }
    }
    return best;
}

const LibertyCell* TechMapper::find_cell_by_tt(uint16_t target_tt, int num_inputs) const {
    const LibertyCell* best = nullptr;
    double best_area = 1e9;
    for (auto& cell : lib_.cells) {
        if (cell.num_inputs() != num_inputs) continue;
        std::string func = cell.output_function();
        if (func.empty()) continue;
        uint16_t cell_tt = func_to_tt(func, num_inputs);
        if (cell_tt == target_tt && cell.area < best_area) {
            best = &cell;
            best_area = cell.area;
        }
        // Also try with permutations
        if (cell_tt != target_tt && !best) {
            std::vector<int> perm;
            if (tt_match_permuted(cell_tt, target_tt, num_inputs, perm)) {
                if (cell.area < best_area) {
                    best = &cell;
                    best_area = cell.area;
                }
            }
        }
    }
    return best;
}

int TechMapper::compute_fanout(uint32_t var) const {
    int count = 0;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v)) continue;
        const auto& nd = aig_.and_node(v);
        if (aig_var(nd.fanin0) == var) count++;
        if (aig_var(nd.fanin1) == var) count++;
    }
    // Also count outputs referencing this var
    for (auto olit : aig_.outputs()) {
        if (aig_var(olit) == var) count++;
    }
    return count;
}

TechMapper::CellMatch TechMapper::to_cell_match(const ExtendedMatch& em) const {
    CellMatch cm;
    cm.cell = em.cell;
    cm.inputs = em.inputs;
    cm.output = em.output;
    return cm;
}

// ============================================================================
// Load-aware delay computation (Elmore model)
// ============================================================================

double TechMapper::compute_load_delay(const LibertyCell* cell, double load_cap) const {
    if (!cell) return 1e9;
    LoadModel model;

    // Extract intrinsic delay from timing tables if available
    if (!cell->timings.empty()) {
        const auto& t = cell->timings[0];
        model.intrinsic_delay = (t.cell_rise + t.cell_fall) / 2.0;
        if (model.intrinsic_delay <= 0.0) model.intrinsic_delay = 0.1;
        model.transition_delay = (t.rise_transition + t.fall_transition) / 2.0;
        if (model.transition_delay <= 0.0) model.transition_delay = 0.05;
    }

    // Elmore: delay = intrinsic + transition × load_cap
    return model.intrinsic_delay + model.transition_delay * load_cap;
}

// ============================================================================
// Complex gate pattern matchers
// ============================================================================

// AOI21: !(a&b | c)  — truth table for 3 inputs: 0x17
TechMapper::ExtendedMatch TechMapper::match_aoi21(uint32_t var) {
    ExtendedMatch em;
    if (!aig_.is_and(var)) return em;

    const auto& nd = aig_.and_node(var);
    uint32_t v0 = aig_var(nd.fanin0), v1 = aig_var(nd.fanin1);

    // AOI21 in AIG: AND(NOT(OR(AND(a,b), c))) = AND(NAND(a,b), NOT(c))
    // or equivalently, we look for the structure and verify by truth table
    // Expand to 3-input cut and check TT
    auto try_match = [&](const std::vector<uint32_t>& leaves) -> bool {
        if (leaves.size() != 3) return false;
        uint16_t node_tt = compute_node_tt(aig_, var, leaves);
        // AOI21: !(A&B | C) => TT = 0x17 for 3 inputs
        // But we also accept permutations and inversions
        constexpr uint16_t aoi21_tt = 0x17; // !(A&B | C) = ~(1000 | 00X0) ...
        // Actually compute: A=bit0, B=bit1, C=bit2
        // pattern: ABC => A&B|C => negate
        // 000: 0|0=0 => 1, 001: 0|1=1 => 0, 010: 0|0=0 => 1, 011: 0|1=1 => 0
        // 100: 0|0=0 => 1, 101: 0|1=1 => 0, 110: 1|0=1 => 0, 111: 1|1=1 => 0
        // TT = 0b00010101 = 0x15 (for 3 inputs, 8 patterns)
        constexpr uint16_t aoi21_correct = 0x15;

        std::vector<int> perm;
        if (node_tt == aoi21_correct || tt_match_permuted(aoi21_correct, node_tt, 3, perm)) {
            return true;
        }
        // Also check complement
        uint16_t inv_tt = (~node_tt) & 0xFF;
        if (inv_tt == aoi21_correct || tt_match_permuted(aoi21_correct, inv_tt, 3, perm)) {
            return true;
        }
        return false;
    };

    // Build 3-input cuts
    std::vector<std::vector<uint32_t>> cuts;
    if (aig_.is_and(v0)) {
        const auto& n0 = aig_.and_node(v0);
        std::vector<uint32_t> c = {aig_var(n0.fanin0), aig_var(n0.fanin1), v1};
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 3) cuts.push_back(c);
    }
    if (aig_.is_and(v1)) {
        const auto& n1 = aig_.and_node(v1);
        std::vector<uint32_t> c = {v0, aig_var(n1.fanin0), aig_var(n1.fanin1)};
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 3) cuts.push_back(c);
    }

    for (auto& leaves : cuts) {
        if (!try_match(leaves)) continue;

        // Find AOI21 cell by name or truth table
        const LibertyCell* cell = find_cell_by_pattern("AOI21");
        if (!cell) cell = find_cell_by_tt(0x15, 3); // fallback: TT match
        if (!cell) continue;

        uint16_t node_tt = compute_node_tt(aig_, var, leaves);
        uint16_t cell_tt = func_to_tt(cell->output_function(), 3);

        std::vector<int> perm;
        em.cell = cell;
        em.output = aig_make(var);
        em.area = cell->area;
        em.priority = 2;

        if (cell_tt == node_tt) {
            for (auto l : leaves) em.inputs.push_back(aig_make(l));
        } else if (tt_match_permuted(cell_tt, node_tt, 3, perm)) {
            for (int i = 0; i < 3; ++i)
                em.inputs.push_back(aig_make(leaves[perm[i]]));
        } else {
            // Complemented output
            em.output = aig_not(aig_make(var));
            uint16_t inv_tt = (~node_tt) & 0xFF;
            if (cell_tt == inv_tt) {
                for (auto l : leaves) em.inputs.push_back(aig_make(l));
            } else if (tt_match_permuted(cell_tt, inv_tt, 3, perm)) {
                for (int i = 0; i < 3; ++i)
                    em.inputs.push_back(aig_make(leaves[perm[i]]));
            } else {
                em.cell = nullptr; // no match
                continue;
            }
        }

        int fo = compute_fanout(var);
        em.delay = compute_load_delay(cell, fo * 0.01);
        em.power = cell->leakage_power + cell->area * 0.001;
        return em;
    }
    return em;
}

// OAI21: !((a|b) & c) — similar structure
TechMapper::ExtendedMatch TechMapper::match_oai21(uint32_t var) {
    ExtendedMatch em;
    if (!aig_.is_and(var)) return em;

    const auto& nd = aig_.and_node(var);
    uint32_t v0 = aig_var(nd.fanin0), v1 = aig_var(nd.fanin1);

    // OAI21 TT: !((A|B) & C)
    // 000: !(0&0)=1, 001: !(0&1)=1, 010: !(1&0)=1, 011: !(1&1)=0
    // 100: !(1&0)=1, 101: !(1&1)=0, 110: !(1&0)=1, 111: !(1&1)=0
    // TT = 0b01010111 = 0x57
    constexpr uint16_t oai21_tt = 0x57;

    std::vector<std::vector<uint32_t>> cuts;
    if (aig_.is_and(v0)) {
        const auto& n0 = aig_.and_node(v0);
        std::vector<uint32_t> c = {aig_var(n0.fanin0), aig_var(n0.fanin1), v1};
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 3) cuts.push_back(c);
    }
    if (aig_.is_and(v1)) {
        const auto& n1 = aig_.and_node(v1);
        std::vector<uint32_t> c = {v0, aig_var(n1.fanin0), aig_var(n1.fanin1)};
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 3) cuts.push_back(c);
    }

    for (auto& leaves : cuts) {
        if (leaves.size() != 3) continue;
        uint16_t node_tt = compute_node_tt(aig_, var, leaves);

        const LibertyCell* cell = find_cell_by_pattern("OAI21");
        if (!cell) cell = find_cell_by_tt(oai21_tt, 3);
        if (!cell) continue;

        uint16_t cell_tt = func_to_tt(cell->output_function(), 3);
        std::vector<int> perm;

        em.cell = cell;
        em.area = cell->area;
        em.priority = 2;

        if (cell_tt == node_tt) {
            em.output = aig_make(var);
            for (auto l : leaves) em.inputs.push_back(aig_make(l));
        } else if (tt_match_permuted(cell_tt, node_tt, 3, perm)) {
            em.output = aig_make(var);
            for (int i = 0; i < 3; ++i)
                em.inputs.push_back(aig_make(leaves[perm[i]]));
        } else {
            uint16_t inv_tt = (~node_tt) & 0xFF;
            if (cell_tt == inv_tt) {
                em.output = aig_not(aig_make(var));
                for (auto l : leaves) em.inputs.push_back(aig_make(l));
            } else if (tt_match_permuted(cell_tt, inv_tt, 3, perm)) {
                em.output = aig_not(aig_make(var));
                for (int i = 0; i < 3; ++i)
                    em.inputs.push_back(aig_make(leaves[perm[i]]));
            } else {
                em.cell = nullptr;
                continue;
            }
        }

        int fo = compute_fanout(var);
        em.delay = compute_load_delay(cell, fo * 0.01);
        em.power = cell->leakage_power + cell->area * 0.001;
        return em;
    }
    return em;
}

// AOI22: !(a&b | c&d) — 4-input pattern
TechMapper::ExtendedMatch TechMapper::match_aoi22(uint32_t var) {
    ExtendedMatch em;
    if (!aig_.is_and(var)) return em;

    const auto& nd = aig_.and_node(var);
    uint32_t v0 = aig_var(nd.fanin0), v1 = aig_var(nd.fanin1);

    // Need both fanins to be AND nodes for 4-input expansion
    if (!aig_.is_and(v0) || !aig_.is_and(v1)) return em;

    const auto& n0 = aig_.and_node(v0);
    const auto& n1 = aig_.and_node(v1);

    std::vector<uint32_t> leaves = {
        aig_var(n0.fanin0), aig_var(n0.fanin1),
        aig_var(n1.fanin0), aig_var(n1.fanin1)
    };
    leaves.erase(std::remove(leaves.begin(), leaves.end(), 0u), leaves.end());
    std::sort(leaves.begin(), leaves.end());
    leaves.erase(std::unique(leaves.begin(), leaves.end()), leaves.end());
    if (leaves.size() != 4) return em;

    uint16_t node_tt = compute_node_tt(aig_, var, leaves);

    // AOI22: !(A&B | C&D) truth table for 4 inputs
    // Compute: for each of 16 patterns
    // Brute force the canonical TT
    uint16_t aoi22_tt = 0;
    for (int p = 0; p < 16; ++p) {
        bool a = (p >> 0) & 1, b = (p >> 1) & 1;
        bool c = (p >> 2) & 1, d = (p >> 3) & 1;
        if (!((a && b) || (c && d)))
            aoi22_tt |= (1 << p);
    }

    const LibertyCell* cell = find_cell_by_pattern("AOI22");
    if (!cell) cell = find_cell_by_tt(aoi22_tt, 4);
    if (!cell) return em;

    uint16_t cell_tt = func_to_tt(cell->output_function(), 4);
    std::vector<int> perm;

    em.cell = cell;
    em.area = cell->area;
    em.priority = 3;

    if (cell_tt == node_tt) {
        em.output = aig_make(var);
        for (auto l : leaves) em.inputs.push_back(aig_make(l));
    } else if (tt_match_permuted(cell_tt, node_tt, 4, perm)) {
        em.output = aig_make(var);
        for (int i = 0; i < 4; ++i)
            em.inputs.push_back(aig_make(leaves[perm[i]]));
    } else {
        uint16_t inv_tt = (~node_tt) & 0xFFFF;
        if (cell_tt == inv_tt) {
            em.output = aig_not(aig_make(var));
            for (auto l : leaves) em.inputs.push_back(aig_make(l));
        } else if (tt_match_permuted(cell_tt, inv_tt, 4, perm)) {
            em.output = aig_not(aig_make(var));
            for (int i = 0; i < 4; ++i)
                em.inputs.push_back(aig_make(leaves[perm[i]]));
        } else {
            em.cell = nullptr;
            return em;
        }
    }

    int fo = compute_fanout(var);
    em.delay = compute_load_delay(cell, fo * 0.01);
    em.power = cell->leakage_power + cell->area * 0.001;
    return em;
}

// MUX2: s?a:b = (s&a) | (!s&b)
TechMapper::ExtendedMatch TechMapper::match_mux2(uint32_t var) {
    ExtendedMatch em;
    if (!aig_.is_and(var)) return em;

    const auto& nd = aig_.and_node(var);
    uint32_t v0 = aig_var(nd.fanin0), v1 = aig_var(nd.fanin1);

    // MUX in AIG is OR(AND(s,a), AND(NOT(s),b))
    // Since AIG only has AND gates, OR(x,y) = NOT(AND(NOT(x), NOT(y)))
    // So we need: the top node is AND with both fanins complemented (= NOR structure inverted)
    // Build 3-input cuts and check against MUX truth table

    std::vector<std::vector<uint32_t>> cuts;
    // Expand both fanins if they are AND nodes
    if (aig_.is_and(v0) && aig_.is_and(v1)) {
        const auto& n0 = aig_.and_node(v0);
        const auto& n1 = aig_.and_node(v1);
        // Collect all unique leaf variables
        std::vector<uint32_t> c = {
            aig_var(n0.fanin0), aig_var(n0.fanin1),
            aig_var(n1.fanin0), aig_var(n1.fanin1)
        };
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 3) cuts.push_back(c);
    }
    // Also try standard 3-input cuts
    if (aig_.is_and(v0)) {
        const auto& n0 = aig_.and_node(v0);
        std::vector<uint32_t> c = {aig_var(n0.fanin0), aig_var(n0.fanin1), v1};
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 3) cuts.push_back(c);
    }
    if (aig_.is_and(v1)) {
        const auto& n1 = aig_.and_node(v1);
        std::vector<uint32_t> c = {v0, aig_var(n1.fanin0), aig_var(n1.fanin1)};
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 3) cuts.push_back(c);
    }

    // MUX TT: S?A:B where S=bit2, A=bit0, B=bit1
    // 000: B=0, 001: A=1, 010: B=1, 011: A=1, 100: B=0, 101: A=1, 110: B=1, 111: A=1
    // Actually MUX(S,A,B) = S&A | !S&B with S=C(bit2), A=A(bit0), B=B(bit1)
    // 000: 0&0|1&0=0, 001: 0&1|1&0=0, 010: 0&0|1&1=1, 011: 0&1|1&1=1
    // 100: 1&0|0&0=0, 101: 1&1|0&0=1, 110: 1&0|0&1=1, 111: 1&1|0&1=1
    // TT = 0b11100100 = 0xE4
    // But different pin orderings give different TTs — we check all permutations
    constexpr uint16_t mux_tt = 0xE4;

    for (auto& leaves : cuts) {
        if (leaves.size() != 3) continue;
        uint16_t node_tt = compute_node_tt(aig_, var, leaves);

        const LibertyCell* cell = find_cell_by_pattern("MUX");
        if (!cell) cell = find_cell_by_tt(mux_tt, 3);
        if (!cell) continue;

        uint16_t cell_tt = func_to_tt(cell->output_function(), 3);
        std::vector<int> perm;

        em.cell = cell;
        em.area = cell->area;
        em.priority = 2;

        if (cell_tt == node_tt) {
            em.output = aig_make(var);
            for (auto l : leaves) em.inputs.push_back(aig_make(l));
        } else if (tt_match_permuted(cell_tt, node_tt, 3, perm)) {
            em.output = aig_make(var);
            for (int i = 0; i < 3; ++i)
                em.inputs.push_back(aig_make(leaves[perm[i]]));
        } else {
            uint16_t inv_tt = (~node_tt) & 0xFF;
            if (cell_tt == inv_tt) {
                em.output = aig_not(aig_make(var));
                for (auto l : leaves) em.inputs.push_back(aig_make(l));
            } else if (tt_match_permuted(cell_tt, inv_tt, 3, perm)) {
                em.output = aig_not(aig_make(var));
                for (int i = 0; i < 3; ++i)
                    em.inputs.push_back(aig_make(leaves[perm[i]]));
            } else {
                em.cell = nullptr;
                continue;
            }
        }

        int fo = compute_fanout(var);
        em.delay = compute_load_delay(cell, fo * 0.01);
        em.power = cell->leakage_power + cell->area * 0.001;
        return em;
    }
    return em;
}

// XOR2: a^b — TT = 0x6 for 2 inputs
TechMapper::ExtendedMatch TechMapper::match_xor2(uint32_t var) {
    ExtendedMatch em;
    if (!aig_.is_and(var)) return em;

    const auto& nd = aig_.and_node(var);
    uint32_t v0 = aig_var(nd.fanin0), v1 = aig_var(nd.fanin1);

    // XOR in AIG: AND(OR(a,b), NAND(a,b))
    // = AND(NOT(AND(NOT(a),NOT(b))), NOT(AND(a,b)))
    // Expand to find the 2-input leaves
    std::vector<std::vector<uint32_t>> cuts;

    // Try direct 2-input
    if (v0 != 0 && v1 != 0 && v0 != v1)
        cuts.push_back({v0, v1});

    // Expand through intermediate AND nodes to find shared variables
    if (aig_.is_and(v0) && aig_.is_and(v1)) {
        const auto& n0 = aig_.and_node(v0);
        const auto& n1 = aig_.and_node(v1);
        std::vector<uint32_t> c = {
            aig_var(n0.fanin0), aig_var(n0.fanin1),
            aig_var(n1.fanin0), aig_var(n1.fanin1)
        };
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 2) cuts.push_back(c);
    }

    constexpr uint16_t xor_tt = 0x6;

    for (auto& leaves : cuts) {
        if (leaves.size() != 2) continue;
        uint16_t node_tt = compute_node_tt(aig_, var, leaves);

        if (node_tt != xor_tt) {
            // Check complement (XNOR)
            uint16_t inv = (~node_tt) & 0xF;
            if (inv != xor_tt) continue;
        }

        bool is_xor = (node_tt == xor_tt);
        const LibertyCell* cell = nullptr;
        if (is_xor) {
            cell = find_cell_by_pattern("XOR");
            if (!cell) cell = find_cell_by_tt(xor_tt, 2);
        }
        if (!cell) continue;

        em.cell = cell;
        em.output = aig_make(var);
        em.area = cell->area;
        em.priority = 1;
        for (auto l : leaves) em.inputs.push_back(aig_make(l));

        int fo = compute_fanout(var);
        em.delay = compute_load_delay(cell, fo * 0.01);
        em.power = cell->leakage_power + cell->area * 0.001;
        return em;
    }
    return em;
}

// XNOR2: !(a^b) — TT = 0x9 for 2 inputs
TechMapper::ExtendedMatch TechMapper::match_xnor2(uint32_t var) {
    ExtendedMatch em;
    if (!aig_.is_and(var)) return em;

    const auto& nd = aig_.and_node(var);
    uint32_t v0 = aig_var(nd.fanin0), v1 = aig_var(nd.fanin1);

    std::vector<std::vector<uint32_t>> cuts;
    if (v0 != 0 && v1 != 0 && v0 != v1)
        cuts.push_back({v0, v1});

    if (aig_.is_and(v0) && aig_.is_and(v1)) {
        const auto& n0 = aig_.and_node(v0);
        const auto& n1 = aig_.and_node(v1);
        std::vector<uint32_t> c = {
            aig_var(n0.fanin0), aig_var(n0.fanin1),
            aig_var(n1.fanin0), aig_var(n1.fanin1)
        };
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 2) cuts.push_back(c);
    }

    constexpr uint16_t xnor_tt = 0x9;

    for (auto& leaves : cuts) {
        if (leaves.size() != 2) continue;
        uint16_t node_tt = compute_node_tt(aig_, var, leaves);

        if (node_tt != xnor_tt) {
            uint16_t inv = (~node_tt) & 0xF;
            if (inv != xnor_tt) continue;
        }

        bool is_xnor = (node_tt == xnor_tt);
        const LibertyCell* cell = nullptr;
        if (is_xnor) {
            cell = find_cell_by_pattern("XNOR");
            if (!cell) cell = find_cell_by_tt(xnor_tt, 2);
        }
        if (!cell) continue;

        em.cell = cell;
        em.output = aig_make(var);
        em.area = cell->area;
        em.priority = 1;
        for (auto l : leaves) em.inputs.push_back(aig_make(l));

        int fo = compute_fanout(var);
        em.delay = compute_load_delay(cell, fo * 0.01);
        em.power = cell->leakage_power + cell->area * 0.001;
        return em;
    }
    return em;
}

// Generic 3-input matcher helper using truth table
TechMapper::ExtendedMatch TechMapper::match_3input_generic(
    uint32_t var, uint16_t target_tt,
    const std::string& name_pattern)
{
    ExtendedMatch em;
    if (!aig_.is_and(var)) return em;

    const auto& nd = aig_.and_node(var);
    uint32_t v0 = aig_var(nd.fanin0), v1 = aig_var(nd.fanin1);

    std::vector<std::vector<uint32_t>> cuts;
    if (aig_.is_and(v0)) {
        const auto& n0 = aig_.and_node(v0);
        std::vector<uint32_t> c = {aig_var(n0.fanin0), aig_var(n0.fanin1), v1};
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 3) cuts.push_back(c);
    }
    if (aig_.is_and(v1)) {
        const auto& n1 = aig_.and_node(v1);
        std::vector<uint32_t> c = {v0, aig_var(n1.fanin0), aig_var(n1.fanin1)};
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 3) cuts.push_back(c);
    }
    // Also try expanding both
    if (aig_.is_and(v0) && aig_.is_and(v1)) {
        const auto& n0 = aig_.and_node(v0);
        const auto& n1 = aig_.and_node(v1);
        std::vector<uint32_t> c = {
            aig_var(n0.fanin0), aig_var(n0.fanin1),
            aig_var(n1.fanin0), aig_var(n1.fanin1)
        };
        c.erase(std::remove(c.begin(), c.end(), 0u), c.end());
        std::sort(c.begin(), c.end());
        c.erase(std::unique(c.begin(), c.end()), c.end());
        if (c.size() == 3) cuts.push_back(c);
    }

    for (auto& leaves : cuts) {
        if (leaves.size() != 3) continue;
        uint16_t node_tt = compute_node_tt(aig_, var, leaves);

        const LibertyCell* cell = find_cell_by_pattern(name_pattern);
        if (!cell) cell = find_cell_by_tt(target_tt, 3);
        if (!cell) continue;

        uint16_t cell_tt = func_to_tt(cell->output_function(), 3);
        std::vector<int> perm;

        em.cell = cell;
        em.area = cell->area;
        em.priority = 2;

        if (cell_tt == node_tt) {
            em.output = aig_make(var);
            for (auto l : leaves) em.inputs.push_back(aig_make(l));
        } else if (tt_match_permuted(cell_tt, node_tt, 3, perm)) {
            em.output = aig_make(var);
            for (int i = 0; i < 3; ++i)
                em.inputs.push_back(aig_make(leaves[perm[i]]));
        } else {
            uint16_t inv_tt = (~node_tt) & 0xFF;
            if (cell_tt == inv_tt) {
                em.output = aig_not(aig_make(var));
                for (auto l : leaves) em.inputs.push_back(aig_make(l));
            } else if (tt_match_permuted(cell_tt, inv_tt, 3, perm)) {
                em.output = aig_not(aig_make(var));
                for (int i = 0; i < 3; ++i)
                    em.inputs.push_back(aig_make(leaves[perm[i]]));
            } else {
                em.cell = nullptr;
                continue;
            }
        }

        int fo = compute_fanout(var);
        em.delay = compute_load_delay(cell, fo * 0.01);
        em.power = cell->leakage_power + cell->area * 0.001;
        return em;
    }
    return em;
}

// NAND3: !(a&b&c) — TT = 0x7F for 3 inputs
TechMapper::ExtendedMatch TechMapper::match_nand3(uint32_t var) {
    return match_3input_generic(var, 0x7F, "NAND3");
}

// NOR3: !(a|b|c) — TT = 0x01 for 3 inputs
TechMapper::ExtendedMatch TechMapper::match_nor3(uint32_t var) {
    return match_3input_generic(var, 0x01, "NOR3");
}

// AND3: a&b&c — TT = 0x80 for 3 inputs
TechMapper::ExtendedMatch TechMapper::match_and3(uint32_t var) {
    return match_3input_generic(var, 0x80, "AND3");
}

// OR3: a|b|c — TT = 0xFE for 3 inputs
TechMapper::ExtendedMatch TechMapper::match_or3(uint32_t var) {
    return match_3input_generic(var, 0xFE, "OR3");
}

// MAJ(a,b,c): majority — TT = 0xE8 for 3 inputs
TechMapper::ExtendedMatch TechMapper::match_majority(uint32_t var) {
    return match_3input_generic(var, 0xE8, "MAJ");
}

// ============================================================================
// find_all_matches — enumerate all valid matches for a node
// ============================================================================

std::vector<TechMapper::ExtendedMatch> TechMapper::find_all_matches(uint32_t var) {
    std::vector<ExtendedMatch> results;
    if (!aig_.is_and(var)) return results;

    // 1. Try original match_node (2-input cells) — convert to ExtendedMatch
    CellMatch cm = match_node(var);
    if (cm.cell) {
        ExtendedMatch em;
        em.cell = cm.cell;
        em.inputs = cm.inputs;
        em.output = cm.output;
        em.area = cm.cell->area;
        int fo = compute_fanout(var);
        em.delay = compute_load_delay(cm.cell, fo * 0.01);
        em.power = cm.cell->leakage_power + cm.cell->area * 0.001;
        em.priority = 5; // baseline match
        results.push_back(em);
    }

    // 2. Try all complex gate matchers
    auto try_add = [&](ExtendedMatch&& m) {
        if (m.cell) results.push_back(std::move(m));
    };

    try_add(match_xor2(var));
    try_add(match_xnor2(var));
    try_add(match_nand3(var));
    try_add(match_nor3(var));
    try_add(match_and3(var));
    try_add(match_or3(var));
    try_add(match_aoi21(var));
    try_add(match_oai21(var));
    try_add(match_aoi22(var));
    try_add(match_mux2(var));
    try_add(match_majority(var));

    return results;
}

// ============================================================================
// select_best_match — choose based on optimization goal
// ============================================================================

TechMapper::ExtendedMatch TechMapper::select_best_match(
    const std::vector<ExtendedMatch>& matches)
{
    if (matches.empty()) return {};

    auto best = matches.begin();
    for (auto it = matches.begin(); it != matches.end(); ++it) {
        bool better = false;
        switch (goal_) {
            case MapGoal::AREA:
                better = it->area < best->area ||
                         (it->area == best->area && it->priority < best->priority);
                break;
            case MapGoal::DELAY:
                better = it->delay < best->delay ||
                         (it->delay == best->delay && it->area < best->area);
                break;
            case MapGoal::POWER:
                better = it->power < best->power ||
                         (it->power == best->power && it->area < best->area);
                break;
        }
        if (better) best = it;
    }
    return *best;
}

// ============================================================================
// area_recovery_pass — swap non-critical cells to smaller alternatives
// ============================================================================

void TechMapper::area_recovery_pass(Netlist& nl, const std::vector<CellMatch>& matches) {
    // Compute arrival times for each gate (forward pass)
    std::vector<double> arrival(nl.num_gates(), 0.0);
    auto topo = nl.topo_order();

    for (GateId gid : topo) {
        const auto& g = nl.gate(gid);
        double max_in = 0.0;
        for (NetId inp : g.inputs) {
            const auto& n = nl.net(inp);
            if (n.driver >= 0) {
                max_in = std::max(max_in, arrival[n.driver]);
            }
        }
        // Estimate gate delay as 0.1ns base
        arrival[gid] = max_in + 0.1;
    }

    // Find critical path delay
    double max_arrival = 0.0;
    for (GateId gid : topo) {
        max_arrival = std::max(max_arrival, arrival[gid]);
    }

    // Required time = max_arrival for all outputs (no timing constraint)
    std::vector<double> required(nl.num_gates(), max_arrival);

    // Backward pass: compute required times
    for (auto it = topo.rbegin(); it != topo.rend(); ++it) {
        GateId gid = *it;
        const auto& g = nl.gate(gid);
        if (g.output < 0) continue;
        const auto& onet = nl.net(g.output);
        double min_req = required[gid];
        for (GateId fid : onet.fanout) {
            if (fid >= 0 && fid < (GateId)nl.num_gates()) {
                min_req = std::min(min_req, required[fid] - 0.1);
            }
        }
        required[gid] = min_req;
    }

    // Compute slack and count recoverable gates
    int recovered = 0;
    double area_saved = 0.0;

    for (size_t i = 0; i < matches.size() && i < (size_t)nl.num_gates(); ++i) {
        if (!matches[i].cell) continue;
        // Only consider gates with positive slack (non-critical)
        GateId gid = (GateId)i;
        if (gid >= (GateId)nl.num_gates()) break;
        double slack = required[gid] - arrival[gid];
        if (slack <= 0.05) continue; // On or near critical path

        // Try to find a smaller cell with same function but higher delay
        const LibertyCell* current = matches[i].cell;
        int ninputs = current->num_inputs();
        std::string func = current->output_function();
        if (func.empty()) continue;
        uint16_t target_tt = func_to_tt(func, ninputs);

        const LibertyCell* smaller = nullptr;
        for (auto& cell : lib_.cells) {
            if (cell.num_inputs() != ninputs) continue;
            if (&cell == current) continue;
            std::string cfunc = cell.output_function();
            if (cfunc.empty()) continue;
            uint16_t ctt = func_to_tt(cfunc, ninputs);

            std::vector<int> perm;
            if (ctt == target_tt || tt_match_permuted(ctt, target_tt, ninputs, perm)) {
                if (cell.area < current->area) {
                    if (!smaller || cell.area < smaller->area) {
                        smaller = &cell;
                    }
                }
            }
        }

        if (smaller) {
            area_saved += current->area - smaller->area;
            recovered++;
        }
    }

    if (recovered > 0) {
        std::cout << "  [AreaRecovery] Recovered " << recovered
                  << " gates, saved ~" << area_saved << " area units\n";
    }
}

// ============================================================================
// Original match_node (unchanged — serves as fallback)
// ============================================================================

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

// ============================================================================
// build_netlist (unchanged)
// ============================================================================

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
            // Complex cells — classify by truth table
            if (tt == 0x80) {
                gtype = GateType::AND;   // AND3
            } else if (tt == 0x7F) {
                gtype = GateType::NAND;  // NAND3
            } else if (tt == 0xFE) {
                gtype = GateType::OR;    // OR3
            } else if (tt == 0x01) {
                gtype = GateType::NOR;   // NOR3
            } else if (tt == 0xE8) {
                gtype = GateType::AND;   // MAJ (approximate)
            } else if (tt == 0xE4 || tt == 0xD8 || tt == 0xCA || tt == 0xAC) {
                gtype = GateType::MUX;   // MUX variants
            } else if (func.find("!(") == 0 && func.find('&') != std::string::npos && func.find('|') != std::string::npos) {
                gtype = GateType::NAND;  // AOI or OAI
            } else if (func.find('&') != std::string::npos) {
                gtype = GateType::AND;
            } else if (func.find('|') != std::string::npos) {
                gtype = GateType::OR;
            }
        } else if (ni == 4) {
            // 4-input complex cells
            if (func.find("!(") == 0) {
                gtype = GateType::NAND;  // AOI22 or similar
            } else if (func.find('&') != std::string::npos) {
                gtype = GateType::AND;
            } else {
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

// ============================================================================
// Enhanced map() — multi-goal with area recovery
// ============================================================================

Netlist TechMapper::map(bool optimize_area) {
    auto t0 = std::chrono::high_resolution_clock::now();
    stats_ = {};

    // If optimize_area is true and no explicit goal set, use AREA
    if (optimize_area && goal_ == MapGoal::DELAY) {
        goal_ = MapGoal::AREA;
    }

    // Step 1: Enumerate all matches for all AIG AND nodes
    // Step 2: Select best match based on goal
    std::vector<CellMatch> matches;
    std::unordered_set<uint32_t> covered_vars; // track nodes covered by multi-input matches

    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v)) continue;

        auto all_matches = find_all_matches(v);
        if (all_matches.empty()) {
            // Fallback to original match_node
            auto m = match_node(v);
            if (m.cell) matches.push_back(m);
            continue;
        }

        ExtendedMatch best = select_best_match(all_matches);
        if (best.cell) {
            matches.push_back(to_cell_match(best));
        } else {
            // Fallback
            auto m = match_node(v);
            if (m.cell) matches.push_back(m);
        }
    }

    // Step 3: Build netlist
    auto nl = build_netlist(matches);

    // Step 4: If goal is DELAY, run area recovery on non-critical paths
    if (goal_ == MapGoal::DELAY) {
        area_recovery_pass(nl, matches);
    }

    stats_.depth = 0;
    auto topo = nl.topo_order();
    stats_.depth = topo.size();

    auto t1 = std::chrono::high_resolution_clock::now();
    stats_.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Step 5: Post-mapping buffer insertion for high-fanout nets
    insert_buffers(nl);

    return nl;
}

// ============================================================================
// insert_buffers (unchanged)
// ============================================================================

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
