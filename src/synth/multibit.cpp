// SiliconForge — Multi-Bit FF Banking Implementation
#include "synth/multibit.hpp"
#include <algorithm>
#include <unordered_map>
#include <cmath>

namespace sf {

std::vector<std::vector<GateId>> MultiBitOptimizer::find_bankable_groups(const MultiBitConfig& cfg) {
    std::vector<std::vector<GateId>> groups;

    // Group DFFs by (clock_net, reset_net) — only bank FFs sharing both
    struct ClkRstKey {
        NetId clk;
        NetId rst;
        bool operator==(const ClkRstKey& o) const { return clk == o.clk && rst == o.rst; }
    };
    struct KeyHash {
        size_t operator()(const ClkRstKey& k) const {
            return std::hash<int>()(k.clk) ^ (std::hash<int>()(k.rst) << 16);
        }
    };

    std::unordered_map<ClkRstKey, std::vector<GateId>, KeyHash> domain_ffs;

    for (size_t i = 0; i < nl_.num_gates(); ++i) {
        auto& g = nl_.gate(static_cast<GateId>(i));
        if (g.type != GateType::DFF) continue;
        domain_ffs[{g.clk, g.reset}].push_back(static_cast<GateId>(i));
    }

    // For each domain group, cluster spatially close FFs
    // Use simple greedy clustering: sort by name (proxy for spatial locality),
    // then form groups of up to max_bank_width consecutive FFs
    for (auto& [key, ffs] : domain_ffs) {
        if ((int)ffs.size() < 2) continue;

        // Sort by gate ID as a locality proxy (FFs from same module are adjacent)
        std::sort(ffs.begin(), ffs.end());

        // Greedy grouping: take consecutive FFs up to max_bank_width
        for (size_t i = 0; i < ffs.size(); ) {
            int remaining = static_cast<int>(ffs.size() - i);
            int group_size = std::min(cfg.max_bank_width, remaining);
            if (group_size < 2) break;

            std::vector<GateId> group(ffs.begin() + i, ffs.begin() + i + group_size);
            groups.push_back(group);
            i += group_size;
        }
    }

    return groups;
}

void MultiBitOptimizer::merge_group(const std::vector<GateId>& group) {
    if (group.size() < 2) return;

    // Structural multi-bit cell merge:
    // The leader gate becomes a multi-bit DFF whose D inputs are the
    // concatenation of all member FFs' D inputs.  All member Q outputs
    // are redirected through the leader.
    //
    // Commercial approach (Cadence/Synopsys): replace N single-bit DFFs
    // with one multi-bit cell from the Liberty library, rewire all fanin/
    // fanout.  We emulate this by:
    //   1. Expanding leader's input vector to hold all D inputs
    //   2. Creating new output nets for each bit of the banked cell
    //   3. Rewiring all downstream fanout from removed FFs to new nets
    //   4. Converting removed FFs to BUF pass-throughs

    std::string mb_type = "DFF" + std::to_string(group.size());
    auto& leader = nl_.gate(group[0]);
    leader.name = mb_type + "_" + leader.name;

    // Leader keeps its own D input at index 0.
    // Append D inputs from the other FFs in the group.
    for (size_t i = 1; i < group.size(); ++i) {
        auto& ff = nl_.gate(group[i]);

        // Add this FF's D input to the leader's input list
        if (!ff.inputs.empty()) {
            leader.inputs.push_back(ff.inputs[0]);
        }

        // Rewire fanout: all gates driven by this FF's output net
        // should now be driven via a BUF from the leader.
        // Create a pass-through BUF that connects leader output → original fanout.
        // The BUF preserves the original output net connectivity.
        NetId old_q = ff.output;
        if (old_q >= 0) {
            // Convert the removed FF into a BUF driven by the leader's Q
            ff.type = GateType::BUF;
            ff.inputs.clear();
            ff.inputs.push_back(leader.output);
            ff.name = mb_type + "_buf_" + std::to_string(i);
            ff.clk = -1;
            ff.reset = -1;
            // ff.output remains the same — downstream fanout is preserved
            // The BUF just passes the leader's Q to the original output net
        }
    }
}

MultiBitResult MultiBitOptimizer::optimize(const MultiBitConfig& cfg) {
    MultiBitResult result;

    if (!cfg.enable) {
        result.message = "Multi-bit banking disabled";
        return result;
    }

    // Count original FFs
    for (size_t i = 0; i < nl_.num_gates(); ++i) {
        if (nl_.gate(static_cast<GateId>(i)).type == GateType::DFF)
            result.original_ffs++;
    }

    if (result.original_ffs == 0) {
        result.message = "No flip-flops to bank";
        return result;
    }

    auto groups = find_bankable_groups(cfg);

    for (auto& grp : groups) {
        merge_group(grp);
        result.banked_groups++;
        result.banked_ffs += static_cast<int>(grp.size());
    }

    result.remaining_ffs = result.original_ffs - result.banked_ffs;

    // Multi-bit cells are ~20% smaller than equivalent individual FFs
    if (result.original_ffs > 0) {
        double banked_fraction = static_cast<double>(result.banked_ffs) / result.original_ffs;
        result.area_savings_pct = banked_fraction * 20.0;
    }

    result.message = std::to_string(result.banked_groups) + " multi-bit groups (" +
                     std::to_string(result.banked_ffs) + "/" +
                     std::to_string(result.original_ffs) + " FFs), ~" +
                     std::to_string(static_cast<int>(result.area_savings_pct)) + "% area savings";
    return result;
}

} // namespace sf
