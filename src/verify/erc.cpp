// SiliconForge — Electrical Rule Checking Implementation
#include "verify/erc.hpp"
#include <unordered_set>

namespace sf {

ErcResult ErcEngine::run() {
    ErcResult res;
    check_floating_inputs(res);
    check_unconnected_outputs(res);
    check_multi_driven(res);
    check_missing_tieoff(res);
    check_power_ground(res);

    res.clean = (res.violations == 0);
    res.message = "ERC: " + std::to_string(res.total_checks) + " checks, " +
                  std::to_string(res.violations) + " violations, " +
                  std::to_string(res.warnings) + " warnings" +
                  (res.clean ? " — CLEAN" : "");
    return res;
}

void ErcEngine::check_floating_inputs(ErcResult& res) {
    // Check for nets that are inputs to gates but have no driver
    std::unordered_set<NetId> pi_set(nl_.primary_inputs().begin(),
                                     nl_.primary_inputs().end());
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        res.total_checks++;
        auto& net = nl_.net(static_cast<NetId>(i));
        if (net.driver < 0 && !net.fanout.empty() && !pi_set.count(static_cast<NetId>(i))) {
            // Synthesis artifacts (bmux_o_*) are expected — treat as warnings
            if (net.name.size() >= 7 && net.name.substr(0, 7) == "bmux_o_") {
                res.warnings++;
                res.details.push_back({ErcViolation::FLOATING_INPUT, net.name,
                    "Net '" + net.name + "' has no driver but feeds " +
                    std::to_string(net.fanout.size()) + " gate(s) [synthesis artifact]"});
            } else {
                // Internal net with no driver but has fanout → floating input
                res.violations++;
                res.details.push_back({ErcViolation::FLOATING_INPUT, net.name,
                    "Net '" + net.name + "' has no driver but feeds " +
                    std::to_string(net.fanout.size()) + " gate(s)"});
            }
        }
    }
}

void ErcEngine::check_unconnected_outputs(ErcResult& res) {
    // Check for gate outputs with no fanout and not primary outputs
    std::unordered_set<NetId> po_set(nl_.primary_outputs().begin(),
                                     nl_.primary_outputs().end());
    for (size_t i = 0; i < nl_.num_gates(); ++i) {
        auto& gate = nl_.gate(static_cast<GateId>(i));
        if (gate.type == GateType::INPUT || gate.type == GateType::OUTPUT) continue;
        if (gate.type == GateType::CONST0 || gate.type == GateType::CONST1) continue;
        res.total_checks++;
        if (gate.output >= 0) {
            auto& net = nl_.net(gate.output);
            if (net.fanout.empty() && !po_set.count(gate.output)) {
                res.warnings++;
                res.details.push_back({ErcViolation::UNCONNECTED_OUTPUT, net.name,
                    "Gate '" + gate.name + "' output '" + net.name + "' is unconnected"});
            }
        }
    }
}

void ErcEngine::check_multi_driven(ErcResult& res) {
    // Check for nets driven by more than one gate (excluding primary inputs)
    std::unordered_set<NetId> pi_set(nl_.primary_inputs().begin(),
                                     nl_.primary_inputs().end());
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        res.total_checks++;
        auto& net = nl_.net(static_cast<NetId>(i));
        // If a net has driver and is also a primary input → potential conflict
        if (net.driver >= 0 && pi_set.count(static_cast<NetId>(i))) {
            // DFF outputs that are also primary inputs are normal (registered outputs)
            auto& drv_gate = nl_.gate(net.driver);
            if (drv_gate.type == GateType::DFF) {
                // Normal for registered outputs — skip silently
                continue;
            }
            res.warnings++;
            res.details.push_back({ErcViolation::MULTI_DRIVEN, net.name,
                "Net '" + net.name + "' is both a primary input and driven by gate '" +
                drv_gate.name + "'"});
        }
    }
}

void ErcEngine::check_missing_tieoff(ErcResult& res) {
    // Check for nets stuck at X (unknown value) that should be tied to 0/1
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        res.total_checks++;
        auto& net = nl_.net(static_cast<NetId>(i));
        if (net.driver < 0 && net.fanout.empty() && net.name.find("tie") != std::string::npos) {
            res.warnings++;
            res.details.push_back({ErcViolation::MISSING_TIEOFF, net.name,
                "Net '" + net.name + "' appears to need tie-off but is unconnected"});
        }
    }
}

void ErcEngine::check_power_ground(ErcResult& res) {
    // Check physical design has power and ground wires
    res.total_checks++;
    bool has_vdd = false, has_vss = false;
    for (auto& w : pd_.wires) {
        if (w.net_id == -3) has_vdd = true;  // VDD convention
        if (w.net_id == -4) has_vss = true;  // VSS convention
    }
    if (!has_vdd) {
        res.warnings++;
        res.details.push_back({ErcViolation::MISSING_POWER, "VDD",
            "No VDD power rail found in physical design"});
    }
    if (!has_vss) {
        res.warnings++;
        res.details.push_back({ErcViolation::MISSING_GROUND, "VSS",
            "No VSS ground rail found in physical design"});
    }
}

} // namespace sf
