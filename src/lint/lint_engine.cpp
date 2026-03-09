// SiliconForge — Lint Engine Implementation
#include "lint/lint_engine.hpp"
#include <unordered_set>
#include <queue>
#include <iostream>

namespace sf {

std::vector<LintViolation> LintEngine::run_all() {
    std::vector<LintViolation> all;
    auto append = [&](std::vector<LintViolation>& v) {
        all.insert(all.end(), v.begin(), v.end());
    };
    auto r1 = check_undriven_nets();       append(r1);
    auto r2 = check_multi_driven_nets();   append(r2);
    auto r3 = check_floating_outputs();    append(r3);
    auto r4 = check_combinational_loops(); append(r4);
    auto r5 = check_unconnected_inputs();  append(r5);
    auto r6 = check_constant_outputs();    append(r6);
    return all;
}

std::vector<LintViolation> LintEngine::check_undriven_nets() {
    std::vector<LintViolation> v;
    std::unordered_set<NetId> pi_set(nl_.primary_inputs().begin(), nl_.primary_inputs().end());
    for (auto& net : nl_.nets()) {
        if (net.driver < 0 && !pi_set.count(net.id)) {
            // Check if it's a clock or reset (driven externally)
            bool is_ff_ctrl = false;
            for (auto gid : net.fanout) {
                auto& g = nl_.gate(gid);
                if (g.clk == net.id || g.reset == net.id) is_ff_ctrl = true;
            }
            if (!is_ff_ctrl && !net.fanout.empty()) {
                v.push_back({LintViolation::ERROR, "LINT-001",
                    "Undriven net '" + net.name + "' has " +
                    std::to_string(net.fanout.size()) + " fanout(s)", net.id});
            }
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_multi_driven_nets() {
    std::vector<LintViolation> v;
    // Count drivers per net
    std::vector<int> driver_count(nl_.num_nets(), 0);
    for (auto& g : nl_.gates()) {
        if (g.output >= 0) driver_count[g.output]++;
    }
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        if (driver_count[i] > 1) {
            v.push_back({LintViolation::ERROR, "LINT-002",
                "Multi-driven net '" + nl_.net(i).name + "' has " +
                std::to_string(driver_count[i]) + " drivers", (NetId)i});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_floating_outputs() {
    std::vector<LintViolation> v;
    for (auto po : nl_.primary_outputs()) {
        if (nl_.net(po).driver < 0) {
            v.push_back({LintViolation::WARNING, "LINT-003",
                "Primary output '" + nl_.net(po).name + "' is undriven", po});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_combinational_loops() {
    std::vector<LintViolation> v;
    // DFS on combinational gates to find back edges
    size_t ng = nl_.num_gates();
    std::vector<int> color(ng, 0); // 0=white, 1=gray, 2=black
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
                        nl_.gate(fo_gid).name + "'", -1, fo_gid});
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
                "Primary input '" + nl_.net(pi).name + "' has no fanout", pi});
        }
    }
    return v;
}

std::vector<LintViolation> LintEngine::check_constant_outputs() {
    std::vector<LintViolation> v;
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::AND && g.output >= 0) {
            // Check if any input is tied to constant 0
            for (auto ni : g.inputs) {
                if (nl_.net(ni).driver >= 0 && nl_.gate(nl_.net(ni).driver).type == GateType::CONST0) {
                    v.push_back({LintViolation::INFO, "LINT-006",
                        "Gate '" + g.name + "' has constant-0 input — output always 0",
                        g.output, g.id});
                }
            }
        }
    }
    return v;
}

} // namespace sf
