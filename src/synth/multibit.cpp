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
    if (group.empty()) return;

    // Rename the first FF in the group to represent the multi-bit cell
    // E.g., DFF4 for a group of 4
    std::string mb_type = "DFF" + std::to_string(group.size());
    auto& leader = nl_.gate(group[0]);
    leader.name = mb_type + "_" + leader.name;

    // Mark other FFs as merged (rename to indicate banked status)
    for (size_t i = 1; i < group.size(); ++i) {
        auto& ff = nl_.gate(group[i]);
        ff.name = mb_type + "_bank_" + std::to_string(i) + "_" + ff.name;
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
