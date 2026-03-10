// SiliconForge — PODEM ATPG Implementation
#include "dft/podem.hpp"
#include <algorithm>
#include <iostream>
#include <cassert>
#include <unordered_set>

namespace sf {

std::string Fault::str() const {
    return "net_" + std::to_string(net) + " SA" + (stuck_at == Logic4::ZERO ? "0" : "1");
}

PodemAtpg::PodemAtpg(Netlist& nl) : nl_(nl) {
    topo_ = nl_.topo_order();
}

std::vector<Fault> PodemAtpg::enumerate_faults() const {
    std::vector<Fault> faults;
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        faults.push_back({(NetId)i, Logic4::ZERO});
        faults.push_back({(NetId)i, Logic4::ONE});
    }
    return faults;
}

void PodemAtpg::init_values() {
    net_values_.assign(nl_.num_nets(), DLogic::X);
}

PodemAtpg::DLogic PodemAtpg::eval_gate_d(GateType type, const std::vector<DLogic>& in) {
    // 5-value logic evaluation for D-algorithm
    auto to4 = [](DLogic d) -> Logic4 {
        switch (d) {
            case DLogic::ZERO: return Logic4::ZERO;
            case DLogic::ONE: return Logic4::ONE;
            default: return Logic4::X;
        }
    };

    // For simple gates, use standard 4-state then handle D/D'
    if (type == GateType::NOT) {
        if (in[0] == DLogic::D) return DLogic::DBAR;
        if (in[0] == DLogic::DBAR) return DLogic::D;
        return in[0] == DLogic::ONE ? DLogic::ZERO :
               in[0] == DLogic::ZERO ? DLogic::ONE : DLogic::X;
    }
    if (type == GateType::BUF) return in[0];

    if (type == GateType::AND || type == GateType::NAND) {
        // AND: if any input is 0, output is 0
        bool has_d = false, has_dbar = false, has_x = false;
        for (auto v : in) {
            if (v == DLogic::ZERO) return type == GateType::AND ? DLogic::ZERO : DLogic::ONE;
            if (v == DLogic::D) has_d = true;
            if (v == DLogic::DBAR) has_dbar = true;
            if (v == DLogic::X) has_x = true;
        }
        if (has_x) return DLogic::X;
        DLogic result;
        if (has_d && has_dbar) result = DLogic::ZERO; // D AND D' = 0
        else if (has_d) result = DLogic::D;
        else if (has_dbar) result = DLogic::DBAR;
        else result = DLogic::ONE; // all ones
        return type == GateType::NAND ?
            (result == DLogic::ZERO ? DLogic::ONE :
             result == DLogic::ONE ? DLogic::ZERO :
             result == DLogic::D ? DLogic::DBAR : DLogic::D) : result;
    }

    if (type == GateType::OR || type == GateType::NOR) {
        bool has_d = false, has_dbar = false, has_x = false;
        for (auto v : in) {
            if (v == DLogic::ONE) return type == GateType::OR ? DLogic::ONE : DLogic::ZERO;
            if (v == DLogic::D) has_d = true;
            if (v == DLogic::DBAR) has_dbar = true;
            if (v == DLogic::X) has_x = true;
        }
        if (has_x) return DLogic::X;
        DLogic result;
        if (has_d && has_dbar) result = DLogic::ONE;
        else if (has_d) result = DLogic::D;
        else if (has_dbar) result = DLogic::DBAR;
        else result = DLogic::ZERO;
        return type == GateType::NOR ?
            (result == DLogic::ZERO ? DLogic::ONE :
             result == DLogic::ONE ? DLogic::ZERO :
             result == DLogic::D ? DLogic::DBAR : DLogic::D) : result;
    }

    if (type == GateType::XOR) {
        if (in[0] == DLogic::X || in[1] == DLogic::X) return DLogic::X;
        auto xor_d = [](DLogic a, DLogic b) -> DLogic {
            if (a == DLogic::ZERO) return b;
            if (b == DLogic::ZERO) return a;
            if (a == DLogic::ONE) {
                if (b == DLogic::D) return DLogic::DBAR;
                if (b == DLogic::DBAR) return DLogic::D;
                return DLogic::ZERO;
            }
            if (b == DLogic::ONE) {
                if (a == DLogic::D) return DLogic::DBAR;
                if (a == DLogic::DBAR) return DLogic::D;
                return DLogic::ZERO;
            }
            if (a == b) return DLogic::ZERO;
            return DLogic::ONE;
        };
        return xor_d(in[0], in[1]);
    }

    return DLogic::X;
}

bool PodemAtpg::forward_imply() {
    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0) continue;
        std::vector<DLogic> in_vals;
        for (auto ni : g.inputs) in_vals.push_back(net_values_[ni]);
        net_values_[g.output] = eval_gate_d(g.type, in_vals);
    }
    return true;
}

bool PodemAtpg::fault_propagated(const Fault& fault) {
    // Check if any PO has D or D'
    for (auto po : nl_.primary_outputs()) {
        if (net_values_[po] == DLogic::D || net_values_[po] == DLogic::DBAR)
            return true;
    }
    return false;
}

bool PodemAtpg::objective(const Fault& fault, NetId& obj_net, DLogic& obj_val) {
    // If fault not yet activated, activate it
    DLogic fault_val = (fault.stuck_at == Logic4::ZERO) ? DLogic::ZERO : DLogic::ONE;
    DLogic good_val = (fault.stuck_at == Logic4::ZERO) ? DLogic::ONE : DLogic::ZERO;

    if (net_values_[fault.net] == DLogic::X) {
        obj_net = fault.net;
        obj_val = good_val; // Set to opposite of stuck value to activate
        return true;
    }

    // Fault activated — find a D-frontier gate to propagate through
    for (auto& g : nl_.gates()) {
        if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0) continue;
        if (net_values_[g.output] != DLogic::X) continue;

        bool has_d = false;
        for (auto ni : g.inputs) {
            if (net_values_[ni] == DLogic::D || net_values_[ni] == DLogic::DBAR)
                has_d = true;
        }
        if (!has_d) continue;

        // Found D-frontier gate — set an X input to non-controlling value
        DLogic non_ctrl = (g.type == GateType::AND || g.type == GateType::NAND)
                          ? DLogic::ONE : DLogic::ZERO;
        for (auto ni : g.inputs) {
            if (net_values_[ni] == DLogic::X) {
                obj_net = ni;
                obj_val = non_ctrl;
                return true;
            }
        }
    }
    return false; // No objective found
}

bool PodemAtpg::backtrace(NetId obj_net, DLogic obj_val, NetId& pi, Logic4& pi_val) {
    // Trace back from objective to a primary input
    NetId current = obj_net;
    DLogic required = obj_val;
    std::unordered_set<NetId> visited;

    while (true) {
        if (visited.count(current)) return false;
        visited.insert(current);

        // Check if current net is a PI
        for (auto p : nl_.primary_inputs()) {
            if (p == current) {
                pi = current;
                pi_val = (required == DLogic::ONE) ? Logic4::ONE : Logic4::ZERO;
                return true;
            }
        }

        // Trace through the driver gate
        GateId drv = nl_.net(current).driver;
        if (drv < 0) return false;

        auto& g = nl_.gate(drv);
        // Choose an unassigned input
        bool inversion = (g.type == GateType::NOT || g.type == GateType::NAND || g.type == GateType::NOR);
        if (inversion) {
            required = (required == DLogic::ONE) ? DLogic::ZERO : DLogic::ONE;
        }
        // For multi-input gates, pick the first X input
        bool found = false;
        for (auto ni : g.inputs) {
            if (net_values_[ni] == DLogic::X) {
                current = ni;
                found = true;
                break;
            }
        }
        if (!found) return false;
    }
}

bool PodemAtpg::podem_recursive(const Fault& fault, int depth) {
    if (depth > MAX_RECURSION_DEPTH || backtrack_count_ > MAX_BACKTRACKS) return false;
    backtrack_count_++;

    // Inject fault value
    DLogic fault_d = (fault.stuck_at == Logic4::ZERO) ? DLogic::D : DLogic::DBAR;
    auto saved = net_values_;

    if (net_values_[fault.net] != DLogic::X) {
        // Modify fault site
        DLogic good = net_values_[fault.net];
        DLogic stuck = (fault.stuck_at == Logic4::ZERO) ? DLogic::ZERO : DLogic::ONE;
        if (good != stuck) {
            net_values_[fault.net] = fault_d;
        }
    }

    forward_imply();

    if (fault_propagated(fault)) return true;

    // Get objective
    NetId obj_net;
    DLogic obj_val;
    if (!objective(fault, obj_net, obj_val)) {
        net_values_ = saved;
        return false;
    }

    // Backtrace to PI
    NetId pi;
    Logic4 pi_val;
    if (!backtrace(obj_net, obj_val, pi, pi_val)) {
        net_values_ = saved;
        return false;
    }

    // Try assigning PI
    for (int attempt = 0; attempt < 2; ++attempt) {
        auto branch_saved = net_values_;
        net_values_[pi] = (pi_val == Logic4::ONE) ? DLogic::ONE : DLogic::ZERO;

        if (net_values_[fault.net] != DLogic::X &&
            net_values_[fault.net] != DLogic::D && net_values_[fault.net] != DLogic::DBAR) {
            DLogic good = net_values_[fault.net];
            DLogic stuck = (fault.stuck_at == Logic4::ZERO) ? DLogic::ZERO : DLogic::ONE;
            if (good != stuck) net_values_[fault.net] = fault_d;
        }

        forward_imply();
        if (fault_propagated(fault)) return true;
        if (podem_recursive(fault, depth + 1)) return true;

        // Backtrack
        net_values_ = branch_saved;
        pi_val = (pi_val == Logic4::ONE) ? Logic4::ZERO : Logic4::ONE;
    }

    net_values_ = saved;
    return false;
}

AtpgResult PodemAtpg::generate_test(const Fault& fault) {
    init_values();
    backtrack_count_ = 0;

    if (podem_recursive(fault)) {
        AtpgResult r;
        r.detected = true;
        r.message = "Test found for " + fault.str();
        for (auto pi : nl_.primary_inputs()) {
            Logic4 val = Logic4::X;
            if (net_values_[pi] == DLogic::ZERO) val = Logic4::ZERO;
            else if (net_values_[pi] == DLogic::ONE) val = Logic4::ONE;
            r.test_vector.push_back({pi, val});
        }
        return r;
    }
    return {false, {}, "No test found for " + fault.str()};
}

FaultCoverage PodemAtpg::run_full_atpg() {
    auto faults = enumerate_faults();
    FaultCoverage fc;
    fc.total_faults = faults.size();

    for (auto& fault : faults) {
        auto result = generate_test(fault);
        if (result.detected) {
            fc.detected++;
            fc.tests.push_back({fault, result.test_vector});
        } else {
            fc.undetectable++;
        }
    }
    return fc;
}

} // namespace sf
