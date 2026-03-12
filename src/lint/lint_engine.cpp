// SiliconForge — Lint Engine Implementation
// Industrial: 30+ rules across 8 categories.
// Reference: Synopsys SpyGlass / Cadence HAL methodology
#include "lint/lint_engine.hpp"
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <iostream>
#include <algorithm>
#include <functional>

namespace sf {

// ============================================================================
// Summary generator
// ============================================================================
LintSummary LintEngine::summarize(const std::vector<LintViolation>& violations) const {
    LintSummary s;
    s.total_violations = (int)violations.size();
    for (auto& v : violations) {
        if (v.severity == LintViolation::ERROR) s.errors++;
        else if (v.severity == LintViolation::WARNING) s.warnings++;
        else s.infos++;
        s.violations_by_rule[v.rule]++;
        s.violations_by_category[v.category]++;
    }
    return s;
}

// ============================================================================
// run_all: execute all enabled rules
// ============================================================================
std::vector<LintViolation> LintEngine::run_all() {
    std::vector<LintViolation> all;
    auto append = [&](std::vector<LintViolation>& v) {
        all.insert(all.end(), v.begin(), v.end());
    };

    // Connectivity (original 6)
    if (config_.check_connectivity) {
        if (is_rule_enabled("LINT-001")) { auto r = check_undriven_nets(); append(r); }
        if (is_rule_enabled("LINT-002")) { auto r = check_multi_driven_nets(); append(r); }
        if (is_rule_enabled("LINT-003")) { auto r = check_floating_outputs(); append(r); }
        if (is_rule_enabled("LINT-004")) { auto r = check_combinational_loops(); append(r); }
        if (is_rule_enabled("LINT-005")) { auto r = check_unconnected_inputs(); append(r); }
        if (is_rule_enabled("LINT-006")) { auto r = check_constant_outputs(); append(r); }
    }

    // Clock
    if (config_.check_clock) {
        if (is_rule_enabled("LINT-C01")) { auto r = check_multiple_clocks(); append(r); }
        if (is_rule_enabled("LINT-C02")) { auto r = check_gated_clocks(); append(r); }
        if (is_rule_enabled("LINT-C03")) { auto r = check_async_reset_usage(); append(r); }
        if (is_rule_enabled("LINT-C04")) { auto r = check_clock_as_data(); append(r); }
        if (is_rule_enabled("LINT-C05")) { auto r = check_data_as_clock(); append(r); }
    }

    // Timing
    if (config_.check_timing) {
        if (is_rule_enabled("LINT-T01")) { auto r = check_reconvergent_fanout(); append(r); }
        if (is_rule_enabled("LINT-T02")) { auto r = check_combo_depth(); append(r); }
        if (is_rule_enabled("LINT-T03")) { auto r = check_latch_inference(); append(r); }
    }

    // Naming
    if (config_.check_naming) {
        if (is_rule_enabled("LINT-N01")) { auto r = check_short_names(); append(r); }
        if (is_rule_enabled("LINT-N02")) { auto r = check_naming_conventions(); append(r); }
        if (is_rule_enabled("LINT-N03")) { auto r = check_duplicate_names(); append(r); }
    }

    // Structure
    if (config_.check_structure) {
        if (is_rule_enabled("LINT-S01")) { auto r = check_high_fanout(); append(r); }
        if (is_rule_enabled("LINT-S02")) { auto r = check_high_fanin(); append(r); }
        if (is_rule_enabled("LINT-S03")) { auto r = check_tristate_bus(); append(r); }
        if (is_rule_enabled("LINT-S04")) { auto r = check_dangling_nets(); append(r); }
    }

    // Redundancy
    if (config_.check_redundancy) {
        if (is_rule_enabled("LINT-R01")) { auto r = check_dead_logic(); append(r); }
        if (is_rule_enabled("LINT-R02")) { auto r = check_redundant_inverters(); append(r); }
        if (is_rule_enabled("LINT-R03")) { auto r = check_constant_propagation(); append(r); }
    }

    // Power
    if (config_.check_power) {
        if (is_rule_enabled("LINT-P01")) { auto r = check_clock_gating_opportunity(); append(r); }
        if (is_rule_enabled("LINT-P02")) { auto r = check_unnecessary_buffers(); append(r); }
    }

    // DFT
    if (config_.check_dft) {
        if (is_rule_enabled("LINT-D01")) { auto r = check_non_scannable_ffs(); append(r); }
        if (is_rule_enabled("LINT-D02")) { auto r = check_dff_reset_missing(); append(r); }
    }

    return all;
}

// ============================================================================
// CONNECTIVITY rules (original, preserved)
// ============================================================================
std::vector<LintViolation> LintEngine::check_undriven_nets() {
    std::vector<LintViolation> v;
    std::unordered_set<NetId> pi_set(nl_.primary_inputs().begin(), nl_.primary_inputs().end());
    for (auto& net : nl_.nets()) {
        if (net.driver < 0 && !pi_set.count(net.id)) {
            bool is_ff_ctrl = false;
            for (auto gid : net.fanout) {
                auto& g = nl_.gate(gid);
                if (g.clk == net.id || g.reset == net.id) is_ff_ctrl = true;
            }
            if (!is_ff_ctrl && !net.fanout.empty()) {
                v.push_back({LintViolation::ERROR, "LINT-001",
                    "Undriven net '" + net.name + "' has " +
                    std::to_string(net.fanout.size()) + " fanout(s)", net.id, -1,
                    LintCategory::CONNECTIVITY});
            }
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_multi_driven_nets() {
    std::vector<LintViolation> v;
    std::vector<int> driver_count(nl_.num_nets(), 0);
    for (auto& g : nl_.gates()) {
        if (g.output >= 0) driver_count[g.output]++;
    }
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        if (driver_count[i] > 1) {
            v.push_back({LintViolation::ERROR, "LINT-002",
                "Multi-driven net '" + nl_.net(i).name + "' has " +
                std::to_string(driver_count[i]) + " drivers", (NetId)i, -1,
                LintCategory::CONNECTIVITY});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_floating_outputs() {
    std::vector<LintViolation> v;
    for (auto po : nl_.primary_outputs()) {
        if (nl_.net(po).driver < 0) {
            v.push_back({LintViolation::WARNING, "LINT-003",
                "Primary output '" + nl_.net(po).name + "' is undriven", po, -1,
                LintCategory::CONNECTIVITY});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_combinational_loops() {
    std::vector<LintViolation> v;
    size_t ng = nl_.num_gates();
    std::vector<int> color(ng, 0);
    bool has_loop = false;

    std::function<void(GateId)> dfs = [&](GateId gid) {
        if (has_loop) return;
        color[gid] = 1;
        auto& g = nl_.gate(gid);
        if (g.output >= 0) {
            for (auto fo_gid : nl_.net(g.output).fanout) {
                if (nl_.gate(fo_gid).type == GateType::DFF) continue;
                if (color[fo_gid] == 1) {
                    has_loop = true;
                    v.push_back({LintViolation::ERROR, "LINT-004",
                        "Combinational loop detected involving gate '" +
                        nl_.gate(fo_gid).name + "'", -1, fo_gid,
                        LintCategory::TIMING});
                    return;
                }
                if (color[fo_gid] == 0) dfs(fo_gid);
            }
        }
        color[gid] = 2;
    };

    for (size_t i = 0; i < ng; ++i) {
        auto& g = nl_.gate(i);
        if (g.type == GateType::DFF || g.type == GateType::INPUT) continue;
        if (color[i] == 0) dfs(i);
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_unconnected_inputs() {
    std::vector<LintViolation> v;
    for (auto pi : nl_.primary_inputs()) {
        if (nl_.net(pi).fanout.empty()) {
            v.push_back({LintViolation::WARNING, "LINT-005",
                "Primary input '" + nl_.net(pi).name + "' has no fanout", pi, -1,
                LintCategory::CONNECTIVITY});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_constant_outputs() {
    std::vector<LintViolation> v;
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::AND && g.output >= 0) {
            for (auto ni : g.inputs) {
                if (nl_.net(ni).driver >= 0 && nl_.gate(nl_.net(ni).driver).type == GateType::CONST0) {
                    v.push_back({LintViolation::INFO, "LINT-006",
                        "Gate '" + g.name + "' has constant-0 input — output always 0",
                        g.output, g.id, LintCategory::REDUNDANCY});
                }
            }
        }
    }
    return v;
}

// ============================================================================
// CLOCK rules
// ============================================================================
std::vector<LintViolation> LintEngine::check_multiple_clocks() {
    std::vector<LintViolation> v;
    // Identify all clock nets (nets driving DFF.clk)
    std::unordered_set<NetId> clock_nets;
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::DFF && g.clk >= 0) clock_nets.insert(g.clk);
    }
    if (clock_nets.size() > 1) {
        v.push_back({LintViolation::WARNING, "LINT-C01",
            "Multiple clock domains detected: " + std::to_string(clock_nets.size()) +
            " distinct clock nets", -1, -1, LintCategory::CLOCK});
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_gated_clocks() {
    std::vector<LintViolation> v;
    // Check if any clock net is driven by a combinational gate (AND/OR = clock gating)
    for (auto& g : nl_.gates()) {
        if (g.type != GateType::DFF) continue;
        if (g.clk < 0) continue;
        auto& clk_net = nl_.net(g.clk);
        if (clk_net.driver < 0) continue;
        auto& drv = nl_.gate(clk_net.driver);
        if (drv.type == GateType::AND || drv.type == GateType::OR ||
            drv.type == GateType::NAND || drv.type == GateType::NOR) {
            v.push_back({LintViolation::WARNING, "LINT-C02",
                "Gated clock on DFF '" + g.name + "': clock driven by " + drv.name,
                g.clk, g.id, LintCategory::CLOCK});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_async_reset_usage() {
    std::vector<LintViolation> v;
    // Check DFFs with reset: flag if some DFFs have reset and some don't
    int with_reset = 0, without_reset = 0;
    for (auto& g : nl_.gates()) {
        if (g.type != GateType::DFF) continue;
        if (g.reset >= 0) with_reset++;
        else without_reset++;
    }
    if (with_reset > 0 && without_reset > 0) {
        v.push_back({LintViolation::WARNING, "LINT-C03",
            "Inconsistent reset: " + std::to_string(with_reset) + " DFFs with reset, " +
            std::to_string(without_reset) + " without", -1, -1, LintCategory::CLOCK});
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_clock_as_data() {
    std::vector<LintViolation> v;
    // Find clock nets used as data inputs to gates
    std::unordered_set<NetId> clock_nets;
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::DFF && g.clk >= 0) clock_nets.insert(g.clk);
    }
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::DFF) continue;
        for (auto ni : g.inputs) {
            if (clock_nets.count(ni)) {
                v.push_back({LintViolation::WARNING, "LINT-C04",
                    "Clock net '" + nl_.net(ni).name + "' used as data input to '" + g.name + "'",
                    ni, g.id, LintCategory::CLOCK});
            }
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_data_as_clock() {
    std::vector<LintViolation> v;
    // Find nets used both as data input and as clock
    std::unordered_set<NetId> data_nets;
    for (auto& g : nl_.gates()) {
        for (auto ni : g.inputs) data_nets.insert(ni);
    }
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::DFF && g.clk >= 0) {
            if (data_nets.count(g.clk)) {
                // Check if the clock net is also driven by combinational logic
                auto& clk_net = nl_.net(g.clk);
                if (clk_net.driver >= 0) {
                    auto& drv = nl_.gate(clk_net.driver);
                    if (drv.type != GateType::INPUT && drv.type != GateType::DFF) {
                        v.push_back({LintViolation::WARNING, "LINT-C05",
                            "DFF '" + g.name + "' clocked by derived signal '" + clk_net.name + "'",
                            g.clk, g.id, LintCategory::CLOCK});
                    }
                }
            }
        }
    }
    return v;
}

// ============================================================================
// TIMING rules
// ============================================================================
std::vector<LintViolation> LintEngine::check_reconvergent_fanout() {
    std::vector<LintViolation> v;
    // For each gate, check if any two of its inputs share a common source
    for (auto& g : nl_.gates()) {
        if (g.inputs.size() < 2) continue;
        std::unordered_set<GateId> sources;
        bool reconvergent = false;
        for (auto ni : g.inputs) {
            auto& net = nl_.net(ni);
            if (net.driver >= 0) {
                if (sources.count(net.driver)) {
                    reconvergent = true;
                    break;
                }
                sources.insert(net.driver);
            }
        }
        if (reconvergent) {
            v.push_back({LintViolation::INFO, "LINT-T01",
                "Reconvergent fanout at gate '" + g.name + "'",
                -1, g.id, LintCategory::TIMING});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_combo_depth() {
    std::vector<LintViolation> v;
    // BFS from DFF outputs / primary inputs, count combinational depth to next DFF/PO
    size_t ng = nl_.num_gates();
    std::vector<int> depth(ng, 0);

    // Topological forward pass
    for (size_t i = 0; i < ng; i++) {
        auto& g = nl_.gate(i);
        if (g.type == GateType::INPUT || g.type == GateType::DFF ||
            g.type == GateType::CONST0 || g.type == GateType::CONST1) {
            depth[i] = 0;
            continue;
        }
        int max_in = 0;
        for (auto ni : g.inputs) {
            auto& net = nl_.net(ni);
            if (net.driver >= 0 && nl_.gate(net.driver).type != GateType::DFF) {
                max_in = std::max(max_in, depth[net.driver]);
            }
        }
        depth[i] = max_in + 1;
        if (depth[i] > config_.max_combo_depth) {
            v.push_back({LintViolation::WARNING, "LINT-T02",
                "Combinational depth " + std::to_string(depth[i]) + " at gate '" + g.name +
                "' exceeds limit " + std::to_string(config_.max_combo_depth),
                -1, g.id, LintCategory::TIMING});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_latch_inference() {
    std::vector<LintViolation> v;
    // Check for gates that might indicate latch behavior:
    // MUX with output feeding back to input = transparent latch
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::MUX && g.output >= 0) {
            for (auto ni : g.inputs) {
                if (ni == g.output) {
                    v.push_back({LintViolation::WARNING, "LINT-T03",
                        "Possible latch inference: MUX '" + g.name +
                        "' has output feeding back to input",
                        g.output, g.id, LintCategory::TIMING});
                    break;
                }
            }
        }
    }
    return v;
}

// ============================================================================
// NAMING rules
// ============================================================================
std::vector<LintViolation> LintEngine::check_short_names() {
    std::vector<LintViolation> v;
    int min_len = config_.min_name_length;
    for (auto& g : nl_.gates()) {
        if (!g.name.empty() && (int)g.name.length() < min_len &&
            g.type != GateType::INPUT && g.type != GateType::OUTPUT &&
            g.type != GateType::CONST0 && g.type != GateType::CONST1) {
            v.push_back({LintViolation::INFO, "LINT-N01",
                "Gate name '" + g.name + "' is shorter than " + std::to_string(min_len) + " chars",
                -1, g.id, LintCategory::NAMING});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_naming_conventions() {
    std::vector<LintViolation> v;
    // Check clock/reset signals follow naming convention
    for (auto& g : nl_.gates()) {
        if (g.type != GateType::DFF) continue;
        if (g.clk >= 0) {
            auto& name = nl_.net(g.clk).name;
            if (!name.empty() && name.find(config_.clock_pattern) == std::string::npos) {
                v.push_back({LintViolation::INFO, "LINT-N02",
                    "Clock net '" + name + "' does not contain '" + config_.clock_pattern + "'",
                    g.clk, g.id, LintCategory::NAMING});
            }
        }
        if (g.reset >= 0) {
            auto& name = nl_.net(g.reset).name;
            if (!name.empty() && name.find(config_.reset_pattern) == std::string::npos) {
                v.push_back({LintViolation::INFO, "LINT-N02",
                    "Reset net '" + name + "' does not contain '" + config_.reset_pattern + "'",
                    g.reset, g.id, LintCategory::NAMING});
            }
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_duplicate_names() {
    std::vector<LintViolation> v;
    std::unordered_map<std::string, int> name_count;
    for (auto& g : nl_.gates()) {
        if (!g.name.empty()) name_count[g.name]++;
    }
    for (auto& [name, count] : name_count) {
        if (count > 1) {
            v.push_back({LintViolation::WARNING, "LINT-N03",
                "Duplicate gate name '" + name + "' appears " + std::to_string(count) + " times",
                -1, -1, LintCategory::NAMING});
        }
    }
    return v;
}

// ============================================================================
// STRUCTURE rules
// ============================================================================
std::vector<LintViolation> LintEngine::check_high_fanout() {
    std::vector<LintViolation> v;
    for (auto& net : nl_.nets()) {
        if ((int)net.fanout.size() > config_.max_fanout) {
            v.push_back({LintViolation::WARNING, "LINT-S01",
                "Net '" + net.name + "' has fanout " + std::to_string(net.fanout.size()) +
                " (limit: " + std::to_string(config_.max_fanout) + ")",
                net.id, -1, LintCategory::STRUCTURE});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_high_fanin() {
    std::vector<LintViolation> v;
    for (auto& g : nl_.gates()) {
        if ((int)g.inputs.size() > config_.max_fanin) {
            v.push_back({LintViolation::WARNING, "LINT-S02",
                "Gate '" + g.name + "' has fanin " + std::to_string(g.inputs.size()) +
                " (limit: " + std::to_string(config_.max_fanin) + ")",
                -1, g.id, LintCategory::STRUCTURE});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_tristate_bus() {
    std::vector<LintViolation> v;
    // Check nets driven by multiple tristate gates
    std::unordered_map<NetId, int> tri_drivers;
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::TRI && g.output >= 0) {
            tri_drivers[g.output]++;
        }
    }
    for (auto& [nid, count] : tri_drivers) {
        if (count > 1) {
            v.push_back({LintViolation::INFO, "LINT-S03",
                "Tristate bus '" + nl_.net(nid).name + "' has " +
                std::to_string(count) + " drivers — ensure mutual exclusion",
                nid, -1, LintCategory::STRUCTURE});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_dangling_nets() {
    std::vector<LintViolation> v;
    std::unordered_set<NetId> pi_set(nl_.primary_inputs().begin(), nl_.primary_inputs().end());
    std::unordered_set<NetId> po_set(nl_.primary_outputs().begin(), nl_.primary_outputs().end());
    for (auto& net : nl_.nets()) {
        if (net.driver < 0 && net.fanout.empty() && !pi_set.count(net.id) && !po_set.count(net.id)) {
            v.push_back({LintViolation::INFO, "LINT-S04",
                "Dangling net '" + net.name + "' has no driver and no fanout",
                net.id, -1, LintCategory::STRUCTURE});
        }
    }
    return v;
}

// ============================================================================
// REDUNDANCY rules
// ============================================================================
std::vector<LintViolation> LintEngine::check_dead_logic() {
    std::vector<LintViolation> v;
    // BFS backward from POs and DFF inputs — any gate not reached is dead
    std::unordered_set<GateId> reachable;
    std::queue<GateId> q;

    // Seed: gates driving primary outputs
    for (auto po : nl_.primary_outputs()) {
        if (nl_.net(po).driver >= 0) q.push(nl_.net(po).driver);
    }
    // Seed: gates driving DFF data/clock/reset inputs
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::DFF) {
            for (auto ni : g.inputs) {
                if (nl_.net(ni).driver >= 0) q.push(nl_.net(ni).driver);
            }
            if (g.clk >= 0 && nl_.net(g.clk).driver >= 0) q.push(nl_.net(g.clk).driver);
        }
    }

    while (!q.empty()) {
        GateId gid = q.front(); q.pop();
        if (reachable.count(gid)) continue;
        reachable.insert(gid);
        auto& g = nl_.gate(gid);
        for (auto ni : g.inputs) {
            if (nl_.net(ni).driver >= 0) q.push(nl_.net(ni).driver);
        }
    }

    for (auto& g : nl_.gates()) {
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT ||
            g.type == GateType::CONST0 || g.type == GateType::CONST1) continue;
        if (!reachable.count(g.id)) {
            v.push_back({LintViolation::WARNING, "LINT-R01",
                "Dead logic: gate '" + g.name + "' has no path to any output",
                -1, g.id, LintCategory::REDUNDANCY});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_redundant_inverters() {
    std::vector<LintViolation> v;
    // NOT(NOT(x)) = x — redundant double inversion
    for (auto& g : nl_.gates()) {
        if (g.type != GateType::NOT) continue;
        if (g.inputs.empty()) continue;
        auto& in_net = nl_.net(g.inputs[0]);
        if (in_net.driver >= 0) {
            auto& drv = nl_.gate(in_net.driver);
            if (drv.type == GateType::NOT) {
                v.push_back({LintViolation::INFO, "LINT-R02",
                    "Double inversion: '" + drv.name + "' → '" + g.name + "' can be simplified",
                    -1, g.id, LintCategory::REDUNDANCY});
            }
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_constant_propagation() {
    std::vector<LintViolation> v;
    // Gates where ALL inputs are constants
    for (auto& g : nl_.gates()) {
        if (g.inputs.empty()) continue;
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT ||
            g.type == GateType::DFF || g.type == GateType::CONST0 ||
            g.type == GateType::CONST1) continue;
        bool all_const = true;
        for (auto ni : g.inputs) {
            auto& net = nl_.net(ni);
            if (net.driver < 0) { all_const = false; break; }
            auto t = nl_.gate(net.driver).type;
            if (t != GateType::CONST0 && t != GateType::CONST1) {
                all_const = false; break;
            }
        }
        if (all_const) {
            v.push_back({LintViolation::INFO, "LINT-R03",
                "Gate '" + g.name + "' has all constant inputs — can be folded",
                -1, g.id, LintCategory::REDUNDANCY});
        }
    }
    return v;
}

// ============================================================================
// POWER rules
// ============================================================================
std::vector<LintViolation> LintEngine::check_clock_gating_opportunity() {
    std::vector<LintViolation> v;
    // DFFs with an enable signal (MUX on data input selecting between old Q and new D)
    // that are NOT clock-gated → opportunity
    for (auto& g : nl_.gates()) {
        if (g.type != GateType::DFF) continue;
        // Check if data input comes from a MUX (common enable pattern)
        if (g.inputs.empty()) continue;
        auto& data_net = nl_.net(g.inputs[0]);
        if (data_net.driver >= 0 && nl_.gate(data_net.driver).type == GateType::MUX) {
            // Check if clock is NOT gated
            bool clock_gated = false;
            if (g.clk >= 0 && nl_.net(g.clk).driver >= 0) {
                auto ct = nl_.gate(nl_.net(g.clk).driver).type;
                if (ct == GateType::AND || ct == GateType::OR) clock_gated = true;
            }
            if (!clock_gated) {
                v.push_back({LintViolation::INFO, "LINT-P01",
                    "Clock gating opportunity: DFF '" + g.name +
                    "' has MUX-based enable but no clock gating",
                    -1, g.id, LintCategory::POWER});
            }
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_unnecessary_buffers() {
    std::vector<LintViolation> v;
    // BUF gates with fanout == 1 can be removed
    for (auto& g : nl_.gates()) {
        if (g.type != GateType::BUF) continue;
        if (g.output >= 0 && nl_.net(g.output).fanout.size() <= 1) {
            v.push_back({LintViolation::INFO, "LINT-P02",
                "Unnecessary buffer: '" + g.name + "' drives only " +
                std::to_string(nl_.net(g.output).fanout.size()) + " gate(s)",
                g.output, g.id, LintCategory::POWER});
        }
    }
    return v;
}

// ============================================================================
// DFT rules
// ============================================================================
std::vector<LintViolation> LintEngine::check_non_scannable_ffs() {
    std::vector<LintViolation> v;
    // DFFs without a MUX on data input (no scan mux)
    for (auto& g : nl_.gates()) {
        if (g.type != GateType::DFF) continue;
        bool has_scan_mux = false;
        if (!g.inputs.empty()) {
            auto& data_net = nl_.net(g.inputs[0]);
            if (data_net.driver >= 0 && nl_.gate(data_net.driver).type == GateType::MUX) {
                has_scan_mux = true;
            }
        }
        if (!has_scan_mux) {
            v.push_back({LintViolation::INFO, "LINT-D01",
                "DFF '" + g.name + "' has no scan MUX — may not be scannable",
                -1, g.id, LintCategory::DFT});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_dff_reset_missing() {
    std::vector<LintViolation> v;
    for (auto& g : nl_.gates()) {
        if (g.type != GateType::DFF) continue;
        if (g.reset < 0) {
            v.push_back({LintViolation::INFO, "LINT-D02",
                "DFF '" + g.name + "' has no reset — initialization may be undefined",
                -1, g.id, LintCategory::DFT});
        }
    }
    return v;
}

} // namespace sf
