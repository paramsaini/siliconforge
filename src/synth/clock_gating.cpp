// SiliconForge — Clock Gating Implementation
#include "synth/clock_gating.hpp"
#include <chrono>
#include <algorithm>
#include <unordered_map>

namespace sf {

std::vector<ClockGatingEngine::FfGroup> ClockGatingEngine::find_groups() {
    std::vector<FfGroup> groups;
    // Group FFs by their clock net
    std::unordered_map<NetId, std::vector<GateId>> clk_groups;

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type != GateType::DFF) continue;
        // DFF stores clock in g.clk, data in inputs[0]
        if (g.clk >= 0) {
            clk_groups[g.clk].push_back(gid);
        }
    }

    for (auto& [clk, ffs] : clk_groups) {
        // Try to find a common enable signal
        // Look at the data input of each FF: if it feeds from a MUX
        // with the FF output as one input (feedback), the select is the enable
        NetId enable = -1;

        for (auto fid : ffs) {
            auto& ff = nl_.gate(fid);
            if (ff.inputs.empty()) continue;
            NetId data = ff.inputs[0];
            auto& data_net = nl_.net(data);
            if (data_net.driver >= 0) {
                auto& drv = nl_.gate(data_net.driver);
                // Simple pattern: AND gate feeding data → potential enable
                if (drv.type == GateType::AND && drv.inputs.size() >= 2) {
                    enable = drv.inputs[1]; // second input as enable
                    break;
                }
            }
        }

        FfGroup grp;
        grp.clock = clk;
        grp.enable = enable;
        grp.ffs = ffs;
        groups.push_back(grp);
    }
    return groups;
}

ClockGatingResult ClockGatingEngine::insert() {
    auto t0 = std::chrono::high_resolution_clock::now();
    ClockGatingResult r;

    auto groups = find_groups();

    for (auto& grp : groups) {
        r.total_ffs += (int)grp.ffs.size();

        if ((int)grp.ffs.size() < min_group_) continue;

        // Create ICG cell: gated_clk = CLK AND enable
        std::string icg_name = "ICG_" + std::to_string(r.icg_cells_inserted);
        NetId gated_clk = nl_.add_net(icg_name + "_gclk");

        NetId enable_net;
        if (grp.enable >= 0) {
            enable_net = grp.enable;
        } else {
            // Create a tied-high enable (always gatable — optimization opportunity)
            enable_net = nl_.add_net(icg_name + "_en");
            nl_.mark_input(enable_net); // external enable control
        }

        // ICG = AND(CLK, EN) — latch-based in real impl, AND gate for model
        nl_.add_gate(GateType::AND, {grp.clock, enable_net}, gated_clk, icg_name);

        // Reconnect all FFs in the group to use gated clock
        for (auto fid : grp.ffs) {
            auto& ff = nl_.gate(fid);
            ff.clk = gated_clk; // reconnect clock
            r.gated_ffs++;
        }

        ClockGatingResult::IcgCell cell;
        cell.name = icg_name;
        cell.clk_in = grp.clock;
        cell.enable = enable_net;
        cell.clk_out = gated_clk;
        cell.gated_ffs = grp.ffs;
        r.cells.push_back(cell);
        r.icg_cells_inserted++;
    }

    // Estimate power reduction: gated FFs save switching power ∝ (1 - activity)
    if (r.total_ffs > 0) {
        double frac_gated = (double)r.gated_ffs / r.total_ffs;
        r.power_reduction_pct = frac_gated * (1.0 - activity_) * 100;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.icg_cells_inserted) + " ICG cells, " +
                std::to_string(r.gated_ffs) + "/" + std::to_string(r.total_ffs) +
                " FFs gated, ~" + std::to_string((int)r.power_reduction_pct) + "% power saved";
    return r;
}

} // namespace sf
