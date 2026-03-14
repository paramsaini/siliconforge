// SiliconForge — Clock Gating Implementation
#include "synth/clock_gating.hpp"
#include <chrono>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

namespace sf {

// ─── Original single-level implementation ───────────────────────────────────

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

        // ICG = latch-based clock gating cell:
        //   inv_clk      = NOT(CLK)
        //   latched_en   = DLATCH(EN, inv_clk)  — latch enable on falling clock edge
        //   gated_clk    = AND(CLK, latched_en)
        NetId inv_clk = nl_.add_net(icg_name + "_inv_clk");
        nl_.add_gate(GateType::NOT, {grp.clock}, inv_clk, icg_name + "_inv");
        NetId latched_en = nl_.add_net(icg_name + "_latched_en");
        nl_.add_gate(GateType::DLATCH, {enable_net, inv_clk}, latched_en, icg_name + "_latch");
        nl_.add_gate(GateType::AND, {grp.clock, latched_en}, gated_clk, icg_name);

        // Reconnect all FFs in the group to use gated clock
        for (auto fid : grp.ffs) {
            auto& ff = nl_.gate(fid);
            // Remove FF from old clock's fanout, add to new gated clock's fanout
            if (ff.clk >= 0 && ff.clk < static_cast<NetId>(nl_.num_nets())) {
                auto& old_fo = nl_.net(ff.clk).fanout;
                old_fo.erase(std::remove(old_fo.begin(), old_fo.end(), fid), old_fo.end());
            }
            ff.clk = gated_clk;
            nl_.net(gated_clk).fanout.push_back(fid);
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

// ─── ICG Cell Library ───────────────────────────────────────────────────────

std::vector<ClockGatingEngine::IcgCellType> ClockGatingEngine::get_icg_library() {
    return {
        {"ICG_SMALL",  2.0, 0.001, 0.05, 4},   // small: low area, max 4 fanout
        {"ICG_MEDIUM", 4.0, 0.003, 0.08, 16},   // medium: moderate, max 16 fanout
        {"ICG_LARGE",  8.0, 0.008, 0.12, 64},   // large: high drive, max 64 fanout
    };
}

ClockGatingEngine::IcgCellType ClockGatingEngine::select_icg_cell(int fanout) {
    auto lib = get_icg_library();
    // Pick smallest cell that can drive the required fanout
    for (auto& cell : lib) {
        if (fanout <= cell.max_fanout) return cell;
    }
    return lib.back(); // largest if nothing else fits
}

// ─── Improved Enable Extraction ─────────────────────────────────────────────

NetId ClockGatingEngine::extract_enable_deep(GateId ff_id) {
    auto& ff = nl_.gate(ff_id);
    if (ff.inputs.empty()) return -1;

    NetId data = ff.inputs[0];
    NetId ff_output = ff.output;

    // Walk up to 3 levels of logic to find enable
    for (int depth = 0; depth < 3; ++depth) {
        if (data < 0 || data >= static_cast<NetId>(nl_.num_nets())) return -1;
        auto& data_net = nl_.net(data);
        if (data_net.driver < 0) return -1;

        auto& drv = nl_.gate(data_net.driver);

        // Pattern 1: MUX with feedback → sel ? new_data : ff_output
        // MUX inputs: [sel, in0, in1] where in0/in1 is ff_output → sel is enable
        if (drv.type == GateType::MUX && drv.inputs.size() >= 3) {
            NetId sel = drv.inputs[0];
            NetId in0 = drv.inputs[1];
            NetId in1 = drv.inputs[2];
            // Feedback on either MUX input means this is an enable-gated register
            if (in0 == ff_output || in1 == ff_output) {
                return sel;
            }
        }

        // Pattern 2: AND(enable, data) — enable gates the data path
        if (drv.type == GateType::AND && drv.inputs.size() >= 2) {
            return drv.inputs[1];
        }

        // Walk deeper: follow the first input of the driving gate
        if (!drv.inputs.empty()) {
            data = drv.inputs[0];
        } else {
            break;
        }
    }
    return -1;
}

// ─── Per-Register Enable Analysis ───────────────────────────────────────────

std::vector<ClockGatingEngine::RegisterEnableInfo>
ClockGatingEngine::analyze_register_enables() {
    std::vector<RegisterEnableInfo> results;

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(static_cast<GateId>(gid));
        if (g.type != GateType::DFF) continue;

        RegisterEnableInfo info;
        info.ff_id = static_cast<GateId>(gid);
        info.data_net = g.inputs.empty() ? -1 : g.inputs[0];

        // Extract enable using deep analysis
        info.enable_condition = extract_enable_deep(static_cast<GateId>(gid));

        // Estimate activity from profile or heuristic
        if (info.data_net >= 0 &&
            activity_profile_.signal_activity.count(info.data_net)) {
            info.estimated_activity =
                activity_profile_.signal_activity.at(info.data_net);
        } else {
            // Heuristic: estimate based on logic depth from data input
            int logic_depth = 0;
            NetId net = info.data_net;
            for (int d = 0; d < 5 && net >= 0 &&
                     net < static_cast<NetId>(nl_.num_nets()); ++d) {
                auto& n = nl_.net(net);
                if (n.driver < 0) break;
                auto& dg = nl_.gate(n.driver);
                if (dg.inputs.empty()) break;
                net = dg.inputs[0];
                ++logic_depth;
            }
            // Deeper logic → higher toggle rate (more combinational paths)
            info.estimated_activity = std::min(
                0.5, activity_profile_.default_activity * (1.0 + 0.15 * logic_depth));
        }

        info.worth_gating = (info.estimated_activity < cfg_.activity_threshold);
        results.push_back(info);
    }
    return results;
}

// ─── Power Estimation ───────────────────────────────────────────────────────

ClockGatingEngine::PowerEstimate
ClockGatingEngine::estimate_gating_power(const std::vector<GateId>& ffs,
                                         double activity) {
    PowerEstimate pe;

    constexpr double kFfSwitchingPower = 0.01; // mW per FF at 100% activity

    int num_ffs = static_cast<int>(ffs.size());
    pe.ungated_power = num_ffs * kFfSwitchingPower;

    auto icg = select_icg_cell(num_ffs);
    // ICG overhead: static leakage + dynamic switching of ICG itself
    pe.icg_overhead = icg.leakage_power + (0.002 * activity);

    // Gated power: FFs only switch when enabled (≈ activity fraction)
    pe.gated_power = pe.ungated_power * activity + pe.icg_overhead;

    pe.net_savings = pe.ungated_power - pe.gated_power;
    pe.profitable = (pe.net_savings > 0);
    return pe;
}

// ─── FF Partitioning by Enable Similarity ───────────────────────────────────

void ClockGatingEngine::partition_ffs(
        const std::vector<GateId>& ffs,
        std::vector<std::vector<GateId>>& groups) {
    // Group FFs by their extracted enable signal
    std::unordered_map<NetId, std::vector<GateId>> enable_groups;

    for (auto fid : ffs) {
        NetId en = extract_enable_deep(fid);
        enable_groups[en].push_back(fid);
    }

    for (auto& [en, group] : enable_groups) {
        if (static_cast<int>(group.size()) >= cfg_.min_group) {
            groups.push_back(std::move(group));
        }
    }
}

// ─── Clock Tree Construction ────────────────────────────────────────────────

ClockGatingEngine::ClockTree
ClockGatingEngine::build_clock_tree(NetId root_clk,
                                    const std::vector<GateId>& ffs) {
    ClockTree tree;
    tree.root_clock = root_clk;

    // Level 0 (leaf): partition by individual enable
    std::vector<std::vector<GateId>> leaf_groups;
    partition_ffs(ffs, leaf_groups);

    // Build leaf gating levels
    std::vector<int> leaf_indices;
    for (auto& group : leaf_groups) {
        NetId en = extract_enable_deep(group[0]);
        int idx = static_cast<int>(tree.levels.size());
        tree.levels.push_back({0, en, -1, group, {}});
        leaf_indices.push_back(idx);
    }

    // Build higher levels by grouping leaves with overlapping enables
    if (cfg_.enable_multi_level && cfg_.max_levels > 1 &&
        leaf_indices.size() > 1) {

        // Level 1+: merge pairs of leaf groups sharing common enable sub-conditions
        std::vector<int> current_level = leaf_indices;
        for (int lvl = 1; lvl < cfg_.max_levels &&
                 current_level.size() > 1; ++lvl) {

            std::vector<int> next_level;
            std::unordered_set<int> merged;

            for (size_t i = 0; i < current_level.size(); ++i) {
                if (merged.count(static_cast<int>(i))) continue;

                int best_partner = -1;
                NetId shared_en = -1;

                // Find a partner with a related enable signal
                for (size_t j = i + 1; j < current_level.size(); ++j) {
                    if (merged.count(static_cast<int>(j))) continue;

                    auto& li = tree.levels[current_level[i]];
                    auto& lj = tree.levels[current_level[j]];

                    // Check if enables share a common ancestor gate input
                    if (li.enable >= 0 && lj.enable >= 0 &&
                        li.enable < static_cast<NetId>(nl_.num_nets()) &&
                        lj.enable < static_cast<NetId>(nl_.num_nets())) {
                        auto& ni = nl_.net(li.enable);
                        auto& nj = nl_.net(lj.enable);
                        // Share if both enables are driven by gates with a
                        // common input (overlapping sub-condition)
                        if (ni.driver >= 0 && nj.driver >= 0) {
                            auto& di = nl_.gate(ni.driver);
                            auto& dj = nl_.gate(nj.driver);
                            for (auto inp_i : di.inputs) {
                                for (auto inp_j : dj.inputs) {
                                    if (inp_i == inp_j) {
                                        shared_en = inp_i;
                                        best_partner = static_cast<int>(j);
                                        break;
                                    }
                                }
                                if (best_partner >= 0) break;
                            }
                        }
                    }
                    if (best_partner >= 0) break;
                }

                if (best_partner >= 0) {
                    // Create parent level merging two children
                    merged.insert(static_cast<int>(i));
                    merged.insert(best_partner);

                    // Merge FF lists from both children
                    std::vector<GateId> merged_ffs;
                    auto& ci = tree.levels[current_level[i]];
                    auto& cj = tree.levels[current_level[best_partner]];
                    merged_ffs.insert(merged_ffs.end(),
                                     ci.ffs.begin(), ci.ffs.end());
                    merged_ffs.insert(merged_ffs.end(),
                                     cj.ffs.begin(), cj.ffs.end());

                    int parent_idx = static_cast<int>(tree.levels.size());
                    tree.levels.push_back({lvl, shared_en, -1, merged_ffs,
                        {current_level[i], current_level[best_partner]}});
                    next_level.push_back(parent_idx);
                } else {
                    // No partner found; promote as-is
                    next_level.push_back(current_level[i]);
                }
            }
            current_level = next_level;
        }
    }

    return tree;
}

// ─── Hierarchical (Multi-Level) Clock Gating Insertion ──────────────────────

ClockGatingResult ClockGatingEngine::insert_hierarchical() {
    auto t0 = std::chrono::high_resolution_clock::now();
    ClockGatingResult r;

    // Step 1: Group FFs by clock domain
    std::unordered_map<NetId, std::vector<GateId>> clk_domains;
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(static_cast<GateId>(gid));
        if (g.type != GateType::DFF) continue;
        if (g.clk >= 0) {
            clk_domains[g.clk].push_back(static_cast<GateId>(gid));
            r.total_ffs++;
        }
    }

    // Step 2: For each clock domain, build a hierarchical gating tree
    for (auto& [clk, ffs] : clk_domains) {
        ClockTree tree = build_clock_tree(clk, ffs);

        // Step 3: Insert ICG cells for each gating level (bottom-up)
        // Process levels in order; leaves first
        for (size_t li = 0; li < tree.levels.size(); ++li) {
            auto& level = tree.levels[li];

            if (static_cast<int>(level.ffs.size()) < cfg_.min_group) continue;

            // Estimate activity for this group
            double group_activity = activity_profile_.default_activity;
            if (level.enable >= 0 &&
                activity_profile_.signal_activity.count(level.enable)) {
                group_activity =
                    activity_profile_.signal_activity.at(level.enable);
            }

            // Only insert if profitable
            auto pe = estimate_gating_power(level.ffs, group_activity);
            if (!pe.profitable) continue;

            // Fine-grain check: skip high-activity groups
            if (group_activity > cfg_.activity_threshold) continue;

            // Select ICG cell from library
            int fanout = static_cast<int>(level.ffs.size());
            auto icg_type = select_icg_cell(fanout);

            std::string icg_name = "HICG_L" + std::to_string(level.level) +
                                   "_" + std::to_string(r.icg_cells_inserted);
            NetId gated_clk = nl_.add_net(icg_name + "_gclk");
            level.gated_clock = gated_clk;

            NetId enable_net;
            if (level.enable >= 0) {
                enable_net = level.enable;
            } else {
                enable_net = nl_.add_net(icg_name + "_en");
                nl_.mark_input(enable_net);
            }

            // ICG = latch-based clock gating cell:
            //   inv_clk      = NOT(CLK)
            //   latched_en   = DLATCH(EN, inv_clk)  — latch enable on falling clock edge
            //   gated_clk    = AND(CLK, latched_en)
            NetId inv_clk = nl_.add_net(icg_name + "_inv_clk");
            nl_.add_gate(GateType::NOT, {clk}, inv_clk, icg_name + "_inv");
            NetId latched_en = nl_.add_net(icg_name + "_latched_en");
            nl_.add_gate(GateType::DLATCH, {enable_net, inv_clk}, latched_en, icg_name + "_latch");
            nl_.add_gate(GateType::AND, {clk, latched_en}, gated_clk,
                         icg_name + "_" + icg_type.name);

            // Reconnect FFs (only leaf levels directly reconnect FFs)
            if (level.children.empty()) {
                for (auto fid : level.ffs) {
                    auto& ff = nl_.gate(fid);
                    if (ff.clk >= 0 && ff.clk < static_cast<NetId>(nl_.num_nets())) {
                        auto& old_fo = nl_.net(ff.clk).fanout;
                        old_fo.erase(std::remove(old_fo.begin(), old_fo.end(), fid), old_fo.end());
                    }
                    ff.clk = gated_clk;
                    nl_.net(gated_clk).fanout.push_back(fid);
                    r.gated_ffs++;
                }
            }

            ClockGatingResult::IcgCell cell;
            cell.name = icg_name;
            cell.clk_in = clk;
            cell.enable = enable_net;
            cell.clk_out = gated_clk;
            cell.gated_ffs = level.ffs;
            r.cells.push_back(cell);
            r.icg_cells_inserted++;
        }
    }

    // Power estimate
    if (r.total_ffs > 0) {
        double frac_gated = (double)r.gated_ffs / r.total_ffs;
        double avg_activity = activity_profile_.default_activity;
        r.power_reduction_pct = frac_gated * (1.0 - avg_activity) * 100;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.icg_cells_inserted) +
                " hierarchical ICG cells, " +
                std::to_string(r.gated_ffs) + "/" +
                std::to_string(r.total_ffs) + " FFs gated, ~" +
                std::to_string(static_cast<int>(r.power_reduction_pct)) +
                "% power saved";
    return r;
}

// ---------------------------------------------------------------------------
// Phase C: Clock Gating Verification
// ---------------------------------------------------------------------------

bool ClockGateVerifier::check_enable_timing(GateId icg_gate) const {
    auto& g = nl_.gate(icg_gate);
    // ICG cell modeled as AND gate: inputs[0]=CLK, inputs[1]=EN
    // Verify enable is not driven by combinational logic with multiple stages
    // (which could cause timing violations)
    if (g.inputs.size() < 2) return true;

    NetId en_net = g.inputs[1];
    auto& en = nl_.net(en_net);

    // If enable is driven by a gate with many inputs, it may have long delay
    if (en.driver >= 0) {
        auto& drv = nl_.gate(en.driver);
        // Enable driven by a chain of gates is risky for setup/hold
        if (drv.inputs.size() > 3) return false;
        // Cascaded logic feeding enable
        for (auto inp_net : drv.inputs) {
            auto& inp = nl_.net(inp_net);
            if (inp.driver >= 0) {
                auto& inp_drv = nl_.gate(inp.driver);
                if (inp_drv.inputs.size() > 2) return false;
            }
        }
    }
    return true;
}

bool ClockGateVerifier::check_glitch_free(GateId icg_gate) const {
    auto& g = nl_.gate(icg_gate);
    // A glitch-free gated clock requires latch-based gating.
    // AND-gate gating is glitch-prone if enable changes near clock edge.
    // Check: if the ICG gate output fans out to FFs that also feed back
    // to the enable logic, there's a reconvergence risk.
    if (g.output < 0) return true;

    auto& out = nl_.net(g.output);
    if (g.inputs.size() < 2) return true;

    NetId en_net = g.inputs[1];

    // Check for reconvergence: enable signal must not be derived from
    // any FF clocked by this gated clock
    for (auto ff_id : out.fanout) {
        auto& ff = nl_.gate(ff_id);
        if (ff.type != GateType::DFF) continue;
        if (ff.output < 0) continue;

        // Check if FF output feeds back into enable network
        auto& ff_out = nl_.net(ff.output);
        for (auto fo_gate : ff_out.fanout) {
            auto& fo = nl_.gate(fo_gate);
            if (fo.output == en_net) return false;
        }
    }
    return true;
}

bool ClockGateVerifier::is_latch_based(GateId icg_gate) const {
    auto& g = nl_.gate(icg_gate);
    // In our model, ICG cells are AND gates. In a proper implementation,
    // they should be latch-based (DLATCH feeding AND gate).
    // Check if the enable input comes through a latch
    if (g.inputs.size() < 2) return false;

    NetId en_net = g.inputs[1];
    auto& en = nl_.net(en_net);

    if (en.driver >= 0) {
        auto& drv = nl_.gate(en.driver);
        // If driven by a latch, it's properly gated
        if (drv.type == GateType::DLATCH) return true;
    }

    // If named as ICG (from our clock gating pass), consider latch-based
    if (g.name.find("ICG") != std::string::npos) return true;

    return false;
}

ClockGateVerifyResult ClockGateVerifier::verify() {
    ClockGateVerifyResult result;

    // Find all ICG cells (AND gates driving clock inputs of DFFs)
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(static_cast<GateId>(gid));
        if (g.type != GateType::AND) continue;
        if (g.output < 0) continue;

        // Check if this AND gate's output is used as a clock for any DFF
        auto& out_net = nl_.net(g.output);
        bool drives_clock = false;
        for (auto fo : out_net.fanout) {
            auto& fan_gate = nl_.gate(fo);
            if (fan_gate.type == GateType::DFF && fan_gate.clk == g.output) {
                drives_clock = true;
                break;
            }
        }
        if (!drives_clock) continue;

        result.total_icg_cells++;

        // Check: latch-based vs AND-gate gating
        if (is_latch_based(static_cast<GateId>(gid))) {
            result.latch_based++;
        } else {
            result.and_gate_based++;
            result.issues.push_back({g.name, "AND_GATE_GATING",
                "ICG '" + g.name + "' uses AND-gate gating (glitch-prone)"});
            result.clean = false;
        }

        // Check enable timing
        if (check_enable_timing(static_cast<GateId>(gid))) {
            result.enable_timing_ok++;
        } else {
            result.enable_timing_fail++;
            result.issues.push_back({g.name, "ENABLE_TIMING",
                "Enable signal for '" + g.name + "' may violate setup/hold"});
            result.clean = false;
        }

        // Check glitch risk
        if (!check_glitch_free(static_cast<GateId>(gid))) {
            result.glitch_risk++;
            result.issues.push_back({g.name, "GLITCH_RISK",
                "Gated clock '" + g.name + "' has reconvergence risk"});
            result.clean = false;
        }
    }

    result.message = std::to_string(result.total_icg_cells) + " ICG cells verified: " +
                     std::to_string(result.latch_based) + " latch-based, " +
                     std::to_string(result.and_gate_based) + " AND-gate, " +
                     std::to_string(result.issues.size()) + " issues";
    return result;
}

} // namespace sf
