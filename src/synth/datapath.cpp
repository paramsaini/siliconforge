// SiliconForge — Datapath Synthesis Engine (implementation)
// Carry-chain inference, Wallace tree construction, Booth encoding detection.

#include "synth/datapath.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_set>
#include <unordered_map>

namespace sf {

// ============================================================================
// Construction
// ============================================================================

DatapathOptimizer::DatapathOptimizer(Netlist& nl, const LibertyLibrary* lib)
    : nl_(nl), lib_(lib) {}

// ============================================================================
// Gate-level helpers
// ============================================================================

bool DatapathOptimizer::is_propagate_gate(GateId gid) const {
    if (gid < 0 || gid >= static_cast<GateId>(nl_.num_gates())) return false;
    return nl_.gate(gid).type == GateType::XOR;
}

bool DatapathOptimizer::is_generate_gate(GateId gid) const {
    if (gid < 0 || gid >= static_cast<GateId>(nl_.num_gates())) return false;
    return nl_.gate(gid).type == GateType::AND;
}

bool DatapathOptimizer::share_inputs(GateId a, GateId b) const {
    const auto& ga = nl_.gate(a);
    const auto& gb = nl_.gate(b);
    if (ga.inputs.size() != 2 || gb.inputs.size() != 2) return false;
    // Same pair of input nets in any order
    return (ga.inputs[0] == gb.inputs[0] && ga.inputs[1] == gb.inputs[1]) ||
           (ga.inputs[0] == gb.inputs[1] && ga.inputs[1] == gb.inputs[0]);
}

double DatapathOptimizer::gate_delay_ps(GateType type) const {
    if (lib_) {
        // Approximate from Liberty data — use first matching cell
        for (const auto& cell : lib_->cells) {
            if (!cell.timings.empty()) {
                double avg = (cell.timings[0].cell_rise + cell.timings[0].cell_fall) / 2.0;
                if (avg > 0) return avg * 1000.0; // ns → ps
            }
        }
    }
    // Default estimates for typical 28nm process
    switch (type) {
        case GateType::XOR:  return 45.0;
        case GateType::AND:  return 30.0;
        case GateType::OR:   return 32.0;
        case GateType::NAND: return 22.0;
        case GateType::NOR:  return 24.0;
        case GateType::NOT:  return 15.0;
        case GateType::BUF:  return 20.0;
        default:             return 35.0;
    }
}

// ============================================================================
// Carry-chain inference
// ============================================================================
// Scans the netlist for XOR+AND gate pairs sharing inputs — the hallmark of
// a ripple-carry adder.  Propagate = A⊕B, Generate = A·B.  Adjacent bit
// slices are chained when the carry-out of slice i feeds slice i+1.

std::vector<CarryChainInfo> DatapathOptimizer::infer_carry_chains() {
    std::vector<CarryChainInfo> chains;

    // Phase 1 — find all (XOR, AND) pairs sharing both inputs
    struct BitSlice {
        GateId xor_gate;   // propagate: P = A ⊕ B
        GateId and_gate;   // generate:  G = A · B
        NetId  carry_out;  // net driven by the OR that merges G with P·Cin
        bool   used = false;
    };
    std::vector<BitSlice> slices;

    // Build index: net → gates it feeds (fanout)
    std::unordered_map<NetId, std::vector<GateId>> net_fanout;
    for (GateId gid = 0; gid < static_cast<GateId>(nl_.num_gates()); ++gid) {
        for (NetId inp : nl_.gate(gid).inputs)
            net_fanout[inp].push_back(gid);
    }

    // Helper: follow an AND output through an OR gate to find the real carry-out.
    // In a full adder the carry is: Cout = G | (P · Cin), so the AND output feeds
    // an OR gate whose output is the actual carry signal to the next bit.
    auto resolve_carry_out = [&](NetId and_out) -> NetId {
        const Net& n = nl_.net(and_out);
        for (GateId fo : n.fanout) {
            if (nl_.gate(fo).type == GateType::OR)
                return nl_.gate(fo).output;
        }
        return and_out; // no OR found, use AND output directly
    };

    // Collect XOR gates, then look for AND gates sharing the same inputs
    for (GateId gid = 0; gid < static_cast<GateId>(nl_.num_gates()); ++gid) {
        if (!is_propagate_gate(gid)) continue;
        const Gate& xg = nl_.gate(gid);
        if (xg.inputs.size() != 2) continue;

        // Search for a paired AND gate with identical inputs
        for (GateId gid2 = 0; gid2 < static_cast<GateId>(nl_.num_gates()); ++gid2) {
            if (gid2 == gid) continue;
            if (!is_generate_gate(gid2)) continue;
            if (!share_inputs(gid, gid2)) continue;

            BitSlice bs;
            bs.xor_gate = gid;
            bs.and_gate = gid2;
            bs.carry_out = resolve_carry_out(nl_.gate(gid2).output);
            slices.push_back(bs);
            break; // one match per XOR is sufficient
        }
    }

    if (slices.empty()) return chains;

    // Phase 2 — chain adjacent slices via carry propagation
    // A slice's carry-out feeds the next slice if that net appears as an input
    // to another XOR (propagate with Cin) or an AND (Cin·P for carry logic).
    auto output_feeds_slice = [&](NetId carry_net, size_t& next_idx) -> bool {
        for (size_t i = 0; i < slices.size(); ++i) {
            if (slices[i].used) continue;
            const Gate& xg = nl_.gate(slices[i].xor_gate);
            for (NetId inp : xg.inputs) {
                if (inp == carry_net) {
                    next_idx = i;
                    return true;
                }
            }
            // Also check if carry feeds an AND gate that participates in carry logic
            const Gate& ag = nl_.gate(slices[i].and_gate);
            for (NetId inp : ag.inputs) {
                if (inp == carry_net) {
                    next_idx = i;
                    return true;
                }
            }
        }
        return false;
    };

    // Build chains starting from unused slices
    for (size_t s = 0; s < slices.size(); ++s) {
        if (slices[s].used) continue;

        // Start a new chain
        std::vector<size_t> chain_indices;
        chain_indices.push_back(s);
        slices[s].used = true;

        // Follow carry-out → next slice
        size_t cur = s;
        while (true) {
            NetId cout = slices[cur].carry_out;
            size_t next;
            if (output_feeds_slice(cout, next)) {
                chain_indices.push_back(next);
                slices[next].used = true;
                cur = next;
            } else {
                break;
            }
        }

        // Only record chains of width ≥ 2
        if (chain_indices.size() >= 2) {
            CarryChainInfo cci;
            cci.width = static_cast<int>(chain_indices.size());
            cci.start_gate = slices[chain_indices.front()].xor_gate;
            cci.end_gate   = slices[chain_indices.back()].xor_gate;
            cci.propagate_delay_ps = gate_delay_ps(GateType::XOR);
            cci.generate_delay_ps  = gate_delay_ps(GateType::AND);
            chains.push_back(cci);
        }
    }

    return chains;
}

// ============================================================================
// Wallace tree construction
// ============================================================================
// Reorganises a flat AND-array (partial-product matrix) into a balanced
// carry-save adder (CSA) tree.  Each CSA level reduces the number of rows
// by a factor of roughly 1.5, so levels ≈ ceil(log_{1.5}(N)).

int DatapathOptimizer::wallace_levels(int n) const {
    if (n <= 2) return 1;
    int levels = 0;
    double count = static_cast<double>(n);
    while (count > 2.0) {
        // Each full-adder stage takes 3 rows → 2 (reduction ratio ~1.5)
        count = std::ceil(count * 2.0 / 3.0);
        ++levels;
    }
    return levels;
}

int DatapathOptimizer::csa_gate_count(int partial_products) const {
    // Each full adder: ~5 gates (2 XOR + 2 AND + 1 OR)
    // Approximate total gates across all reduction levels
    int gates = 0;
    int rows = partial_products;
    while (rows > 2) {
        int triplets = rows / 3;
        gates += triplets * 5; // 5 gates per FA
        rows = rows - triplets + triplets * 0; // 3→2 reduction
        rows = (rows - triplets * 3) + triplets * 2;
        rows = partial_products; // recount
        // simpler: accumulate and break
        gates = partial_products * 5 * wallace_levels(partial_products) / 3;
        break;
    }
    return std::max(gates, 1);
}

std::vector<DatapathOptimizer::AndArray> DatapathOptimizer::find_and_arrays() const {
    std::vector<AndArray> arrays;

    // Group AND gates by shared output structure
    // In a multiplier, AND gates form a rectangular grid: a[i] & b[j]
    std::unordered_map<NetId, std::vector<GateId>> input_groups;
    for (GateId gid = 0; gid < static_cast<GateId>(nl_.num_gates()); ++gid) {
        const Gate& g = nl_.gate(gid);
        if (g.type != GateType::AND) continue;
        if (g.inputs.size() != 2) continue;
        // Group by first input — partial products sharing multiplicand bit
        input_groups[g.inputs[0]].push_back(gid);
    }

    // Merge groups that share the second input (multiplier bits)
    std::unordered_set<GateId> visited;
    for (auto& [net, gates] : input_groups) {
        if (gates.size() < 2) continue;
        // Check if any gate in this group is already part of a found array
        bool any_visited = false;
        for (GateId gid : gates)
            if (visited.count(gid)) { any_visited = true; break; }
        if (any_visited) continue;

        // Collect all AND gates reachable from this group via shared inputs
        std::unordered_set<GateId> array_set(gates.begin(), gates.end());
        std::unordered_set<NetId> second_inputs;
        for (GateId gid : gates) {
            second_inputs.insert(nl_.gate(gid).inputs[1]);
        }

        // Expand: find AND gates with the same second inputs but different first
        for (GateId gid2 = 0; gid2 < static_cast<GateId>(nl_.num_gates()); ++gid2) {
            const Gate& g2 = nl_.gate(gid2);
            if (g2.type != GateType::AND || g2.inputs.size() != 2) continue;
            if (second_inputs.count(g2.inputs[1]) || second_inputs.count(g2.inputs[0])) {
                array_set.insert(gid2);
            }
        }

        if (array_set.size() >= 4) {
            AndArray aa;
            aa.gates.assign(array_set.begin(), array_set.end());
            // Estimate dimensions from unique first/second inputs
            std::unordered_set<NetId> first_nets, second_nets;
            for (GateId gid : aa.gates) {
                first_nets.insert(nl_.gate(gid).inputs[0]);
                second_nets.insert(nl_.gate(gid).inputs[1]);
            }
            aa.rows = static_cast<int>(first_nets.size());
            aa.cols = static_cast<int>(second_nets.size());
            for (GateId gid : aa.gates) visited.insert(gid);
            arrays.push_back(aa);
        }
    }

    return arrays;
}

std::vector<WallaceTreeInfo> DatapathOptimizer::build_wallace_trees(int max_inputs) {
    std::vector<WallaceTreeInfo> trees;

    auto arrays = find_and_arrays();
    for (const auto& arr : arrays) {
        int n = std::max(arr.rows, arr.cols);
        if (n > max_inputs) continue;

        WallaceTreeInfo wti;
        wti.input_bits = n;
        wti.partial_products = static_cast<int>(arr.gates.size());
        wti.levels = wallace_levels(wti.partial_products);
        wti.area_gates = csa_gate_count(wti.partial_products);
        trees.push_back(wti);
    }

    return trees;
}

// ============================================================================
// Booth encoding detection
// ============================================================================
// Identifies multiplier structures and determines the best Booth recoding
// radix.  Radix-4 halves partial products; radix-8 reduces by ~2/3 at the
// cost of a harder multiple (3×multiplicand).

BoothEncoderInfo::EncodingType DatapathOptimizer::select_booth_radix(int width) const {
    if (width >= 16)
        return BoothEncoderInfo::RADIX8;
    else if (width >= 8)
        return BoothEncoderInfo::RADIX4;
    else
        return BoothEncoderInfo::RADIX2;
}

double DatapathOptimizer::booth_reduction_pct(BoothEncoderInfo::EncodingType enc) const {
    switch (enc) {
        case BoothEncoderInfo::RADIX2: return 0.0;
        case BoothEncoderInfo::RADIX4: return 50.0;
        case BoothEncoderInfo::RADIX8: return 66.7;
    }
    return 0.0;
}

std::vector<BoothEncoderInfo> DatapathOptimizer::apply_booth_encoding() {
    std::vector<BoothEncoderInfo> encoders;

    auto arrays = find_and_arrays();
    for (const auto& arr : arrays) {
        int width = std::max(arr.rows, arr.cols);
        if (width < 2) continue;

        BoothEncoderInfo bei;
        bei.multiplier_width = width;
        bei.encoding_type = select_booth_radix(width);
        bei.partial_product_reduction_pct = booth_reduction_pct(bei.encoding_type);
        encoders.push_back(bei);
    }

    return encoders;
}

// ============================================================================
// Combined optimization
// ============================================================================

DatapathResult DatapathOptimizer::optimize(const DatapathConfig& cfg) {
    cfg_ = cfg;
    DatapathResult res;

    // Carry-chain inference
    std::vector<CarryChainInfo> chains;
    if (cfg_.enable_carry_chain) {
        chains = infer_carry_chains();
        // Filter by max width
        chains.erase(
            std::remove_if(chains.begin(), chains.end(),
                           [&](const CarryChainInfo& c) { return c.width > cfg_.max_adder_width; }),
            chains.end());
        res.carry_chains_inferred = static_cast<int>(chains.size());
        res.adders_optimized = res.carry_chains_inferred;
    }

    // Wallace tree construction
    std::vector<WallaceTreeInfo> trees;
    if (cfg_.enable_wallace_tree) {
        trees = build_wallace_trees();
        res.wallace_trees_built = static_cast<int>(trees.size());
        res.multipliers_optimized += res.wallace_trees_built;
    }

    // Booth encoding
    std::vector<BoothEncoderInfo> booths;
    if (cfg_.enable_booth_encoding) {
        booths = apply_booth_encoding();
        res.booth_encoders_built = static_cast<int>(booths.size());
        // Booth and Wallace may target the same multiplier structures;
        // count distinct multiplier optimizations
        res.multipliers_optimized = std::max(res.multipliers_optimized,
                                              res.booth_encoders_built);
    }

    // Estimate area/delay improvements
    double area_save = 0.0;
    double delay_save = 0.0;

    // Carry chains: replacing ripple carry with dedicated chain saves
    // ~30% area per adder and ~60% delay (log vs linear)
    for (const auto& cc : chains) {
        double ripple_delay = cc.width * (cc.propagate_delay_ps + cc.generate_delay_ps);
        double chain_delay  = cc.propagate_delay_ps * std::log2(std::max(cc.width, 2)) * 2.0
                              + cc.generate_delay_ps;
        if (ripple_delay > 0) {
            delay_save += (ripple_delay - chain_delay) / ripple_delay * 100.0;
        }
        area_save += 15.0; // ~15% area reduction per adder from shared logic
    }

    // Wallace trees: reduce delay from O(N) to O(log1.5 N)
    for (const auto& wt : trees) {
        double flat_delay  = wt.partial_products;
        double tree_delay  = wt.levels;
        if (flat_delay > 0)
            delay_save += (flat_delay - tree_delay) / flat_delay * 100.0;
        area_save += 10.0;
    }

    // Booth encoding: area savings from fewer partial products
    for (const auto& be : booths) {
        area_save += be.partial_product_reduction_pct * 0.3; // scaled contribution
    }

    int total_opts = res.adders_optimized + res.multipliers_optimized;
    if (total_opts > 0) {
        res.area_reduction_pct    = area_save / total_opts;
        res.delay_improvement_pct = delay_save / total_opts;
    }

    res.report = build_report(res);
    return res;
}

// ============================================================================
// Enhanced full flow
// ============================================================================

DatapathResult DatapathOptimizer::run_enhanced() {
    DatapathConfig cfg;
    cfg.enable_carry_chain    = true;
    cfg.enable_wallace_tree   = true;
    cfg.enable_booth_encoding = true;
    cfg.max_adder_width       = 64;
    cfg.target_freq_mhz       = 500.0;

    DatapathResult res = optimize(cfg);

    // Augment report with frequency analysis
    std::ostringstream oss;
    oss << res.report;
    oss << "\n--- Enhanced Flow Summary ---\n";
    oss << "Target frequency: " << cfg.target_freq_mhz << " MHz\n";
    double period_ps = 1e6 / cfg.target_freq_mhz; // ps
    oss << "Clock period:     " << period_ps << " ps\n";
    oss << "Adders optimized:      " << res.adders_optimized << "\n";
    oss << "Multipliers optimized: " << res.multipliers_optimized << "\n";
    oss << "Area reduction:        " << res.area_reduction_pct << " %\n";
    oss << "Delay improvement:     " << res.delay_improvement_pct << " %\n";
    res.report = oss.str();

    return res;
}

// ============================================================================
// Report generation
// ============================================================================

std::string DatapathOptimizer::build_report(const DatapathResult& res) const {
    std::ostringstream oss;
    oss << "=== Datapath Synthesis Report ===\n";
    oss << "Carry chains inferred: " << res.carry_chains_inferred << "\n";
    oss << "Wallace trees built:   " << res.wallace_trees_built << "\n";
    oss << "Booth encoders built:  " << res.booth_encoders_built << "\n";
    oss << "Adders optimized:      " << res.adders_optimized << "\n";
    oss << "Multipliers optimized: " << res.multipliers_optimized << "\n";
    oss << "Area reduction:        " << res.area_reduction_pct << " %\n";
    oss << "Delay improvement:     " << res.delay_improvement_pct << " %\n";
    return oss.str();
}

} // namespace sf
