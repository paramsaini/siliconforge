// SiliconForge — Production PODEM ATPG Implementation
// Full recursive backtracking with D-frontier heuristics and fault dropping
//
// Key fix: fault injection happens INLINE during forward implication.
// After evaluating the driver of the fault net, the fault transform is applied
// immediately, so downstream gates in the same pass see D/DBAR values.
#include "dft/podem.hpp"
#include <algorithm>
#include <iostream>
#include <cassert>
#include <unordered_set>
#include <unordered_map>

namespace sf {

std::string Fault::str() const {
    return "net_" + std::to_string(net) + " SA" + (stuck_at == Logic4::ZERO ? "0" : "1");
}

PodemAtpg::PodemAtpg(Netlist& nl) : nl_(nl) {
    topo_ = nl_.topo_order();
    // Real PIs
    for (auto pi : nl_.primary_inputs()) pi_set_.insert(pi);
    // Real POs
    for (auto po : nl_.primary_outputs()) po_set_.insert(po);

    // Scan-equivalent: DFF outputs become pseudo-PIs, DFF D-inputs become pseudo-POs
    for (size_t gid = 0; gid < nl_.gates().size(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF) {
            // DFF output (Q) → pseudo-PI (controllable in scan mode)
            if (g.output >= 0) pi_set_.insert(g.output);
            // DFF data input (D) → pseudo-PO (observable in scan mode)
            if (!g.inputs.empty() && g.inputs[0] >= 0) po_set_.insert(g.inputs[0]);
        }
    }
}

std::vector<Fault> PodemAtpg::enumerate_faults() const {
    std::vector<Fault> faults;
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        auto& net = nl_.net(i);
        bool has_driver = net.driver >= 0;
        bool is_ctrl_point = pi_set_.count((NetId)i) > 0; // includes pseudo-PIs
        if (has_driver || is_ctrl_point) {
            faults.push_back({(NetId)i, Logic4::ZERO});
            faults.push_back({(NetId)i, Logic4::ONE});
        }
    }
    return faults;
}

void PodemAtpg::init_values() {
    net_values_.assign(nl_.num_nets(), DLogic::X);
}

PodemAtpg::DLogic PodemAtpg::eval_gate_d(GateType type, const std::vector<DLogic>& in) {
    // Constants have no inputs — check these first
    if (type == GateType::CONST0) return DLogic::ZERO;
    if (type == GateType::CONST1) return DLogic::ONE;

    if (in.empty()) return DLogic::X;

    auto dinv = [](DLogic v) -> DLogic {
        switch (v) {
            case DLogic::ZERO: return DLogic::ONE;
            case DLogic::ONE: return DLogic::ZERO;
            case DLogic::D: return DLogic::DBAR;
            case DLogic::DBAR: return DLogic::D;
            default: return DLogic::X;
        }
    };

    if (type == GateType::NOT) return dinv(in[0]);
    if (type == GateType::BUF) return in[0];

    if (type == GateType::AND || type == GateType::NAND) {
        bool has_d = false, has_dbar = false, has_x = false;
        for (auto v : in) {
            if (v == DLogic::ZERO)
                return type == GateType::AND ? DLogic::ZERO : DLogic::ONE;
            if (v == DLogic::D) has_d = true;
            if (v == DLogic::DBAR) has_dbar = true;
            if (v == DLogic::X) has_x = true;
        }
        if (has_x) return DLogic::X;
        DLogic result;
        if (has_d && has_dbar) result = DLogic::ZERO;
        else if (has_d) result = DLogic::D;
        else if (has_dbar) result = DLogic::DBAR;
        else result = DLogic::ONE;
        return type == GateType::NAND ? dinv(result) : result;
    }

    if (type == GateType::OR || type == GateType::NOR) {
        bool has_d = false, has_dbar = false, has_x = false;
        for (auto v : in) {
            if (v == DLogic::ONE)
                return type == GateType::OR ? DLogic::ONE : DLogic::ZERO;
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
        return type == GateType::NOR ? dinv(result) : result;
    }

    if (type == GateType::XOR || type == GateType::XNOR) {
        if (in.size() < 2) return in.empty() ? DLogic::X : in[0];
        DLogic a = in[0], b = in[1];
        if (a == DLogic::X || b == DLogic::X) return DLogic::X;
        DLogic result;
        if (a == DLogic::ZERO) result = b;
        else if (b == DLogic::ZERO) result = a;
        else if (a == DLogic::ONE) result = dinv(b);
        else if (b == DLogic::ONE) result = dinv(a);
        else if (a == b) result = DLogic::ZERO;
        else result = DLogic::ONE;
        return type == GateType::XNOR ? dinv(result) : result;
    }

    if (type == GateType::MUX) {
        if (in.size() < 3) return DLogic::X;
        DLogic sel = in[0], t = in[1], e = in[2];
        if (sel == DLogic::ONE) return t;
        if (sel == DLogic::ZERO) return e;
        if (t == e) return t;
        return DLogic::X;
    }

    return DLogic::X;
}

// Forward implication with INLINE fault injection.
// After evaluating the driver of the fault net, we apply the fault transform
// so downstream gates see D/DBAR in the same pass.
void PodemAtpg::forward_imply_with_fault(const Fault& fault) {
    DLogic fault_d = (fault.stuck_at == Logic4::ZERO) ? DLogic::D : DLogic::DBAR;
    DLogic stuck_val = (fault.stuck_at == Logic4::ZERO) ? DLogic::ZERO : DLogic::ONE;

    for (int pass = 0; pass < 5; ++pass) {
        bool changed = false;
        for (auto gid : topo_) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0) continue;

            std::vector<DLogic> in_vals;
            for (auto ni : g.inputs) {
                if (ni >= 0 && ni < (NetId)net_values_.size())
                    in_vals.push_back(net_values_[ni]);
                else
                    in_vals.push_back(DLogic::X);
            }
            DLogic new_val = eval_gate_d(g.type, in_vals);

            // If this gate drives the fault net, apply fault transform
            if ((NetId)g.output == fault.net) {
                if (new_val != DLogic::X && new_val != stuck_val &&
                    new_val != DLogic::D && new_val != DLogic::DBAR) {
                    new_val = fault_d;
                } else if (new_val == stuck_val) {
                    new_val = stuck_val;
                }
            }

            if (new_val != net_values_[g.output]) {
                net_values_[g.output] = new_val;
                changed = true;
            }
        }

        // Handle faults on PIs or pseudo-PIs (DFF outputs in scan mode)
        if (pi_set_.count(fault.net)) {
            DLogic pv = net_values_[fault.net];
            if (pv != DLogic::X && pv != stuck_val && pv != DLogic::D && pv != DLogic::DBAR) {
                net_values_[fault.net] = fault_d;
                changed = true;
            }
        }

        if (!changed) break;
    }
}

bool PodemAtpg::fault_propagated(const Fault& fault) {
    // Check both real POs and pseudo-POs (DFF D-inputs in scan mode)
    for (auto po : po_set_) {
        if (po >= 0 && po < (NetId)net_values_.size()) {
            if (net_values_[po] == DLogic::D || net_values_[po] == DLogic::DBAR)
                return true;
        }
    }
    return false;
}

bool PodemAtpg::objective(const Fault& fault, NetId& obj_net, DLogic& obj_val) {
    DLogic good_val = (fault.stuck_at == Logic4::ZERO) ? DLogic::ONE : DLogic::ZERO;

    // Step 1: If fault site not activated, drive it to good value
    DLogic fsite = net_values_[fault.net];
    if (fsite != DLogic::D && fsite != DLogic::DBAR) {
        obj_net = fault.net;
        obj_val = good_val;
        return true;
    }

    // Step 2: D-frontier — gates with D/D' on an input but X on output
    struct DfGate { GateId gid; NetId x_input; int level; };
    std::vector<DfGate> d_frontier;

    std::unordered_map<GateId, int> gate_level;
    int lvl = 0;
    for (auto gid : topo_) gate_level[gid] = lvl++;

    for (size_t gid = 0; gid < nl_.gates().size(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0) continue;
        if (net_values_[g.output] != DLogic::X) continue;

        bool has_d = false;
        NetId first_x = -1;
        for (auto ni : g.inputs) {
            if (ni >= 0 && ni < (NetId)net_values_.size()) {
                if (net_values_[ni] == DLogic::D || net_values_[ni] == DLogic::DBAR)
                    has_d = true;
                if (net_values_[ni] == DLogic::X && first_x < 0)
                    first_x = ni;
            }
        }
        if (has_d && first_x >= 0) {
            d_frontier.push_back({(GateId)gid, first_x, gate_level.count(gid) ? gate_level[gid] : 0});
        }
    }

    if (d_frontier.empty()) return false;

    // Pick gate nearest to output (highest topo level)
    std::sort(d_frontier.begin(), d_frontier.end(),
              [](const DfGate& a, const DfGate& b) { return a.level > b.level; });

    auto& chosen = d_frontier[0];
    auto& g = nl_.gate(chosen.gid);

    // Non-controlling value for gate type
    DLogic non_ctrl;
    switch (g.type) {
        case GateType::AND: case GateType::NAND:
            non_ctrl = DLogic::ONE; break;
        case GateType::OR: case GateType::NOR:
            non_ctrl = DLogic::ZERO; break;
        default:
            non_ctrl = DLogic::ZERO; break;
    }

    obj_net = chosen.x_input;
    obj_val = non_ctrl;
    return true;
}

bool PodemAtpg::backtrace(NetId obj_net, DLogic obj_val, NetId& pi, Logic4& pi_val) {
    NetId current = obj_net;
    DLogic required = obj_val;
    std::unordered_set<NetId> visited;
    int max_depth = 500;

    while (max_depth-- > 0) {
        if (visited.count(current)) return false;
        visited.insert(current);

        if (pi_set_.count(current)) {
            pi = current;
            pi_val = (required == DLogic::ONE) ? Logic4::ONE : Logic4::ZERO;
            return true;
        }

        GateId drv = nl_.net(current).driver;
        if (drv < 0) return false;

        auto& g = nl_.gate(drv);
        if (g.inputs.empty()) return false;

        // Track inversion through gate
        bool inverts = (g.type == GateType::NOT || g.type == GateType::NAND ||
                       g.type == GateType::NOR || g.type == GateType::XNOR);
        if (inverts) {
            required = (required == DLogic::ONE) ? DLogic::ZERO : DLogic::ONE;
        }

        // For multi-input gates, prefer unassigned (X) input
        bool found = false;
        for (auto ni : g.inputs) {
            if (ni >= 0 && ni < (NetId)net_values_.size() && net_values_[ni] == DLogic::X) {
                current = ni;
                found = true;
                break;
            }
        }

        if (!found) {
            current = g.inputs[0];
        }
    }
    return false;
}

bool PodemAtpg::podem_recursive(const Fault& fault, int depth) {
    if (depth > MAX_RECURSION_DEPTH || backtrack_count_ > MAX_BACKTRACKS) return false;

    forward_imply_with_fault(fault);

    if (fault_propagated(fault)) return true;

    // Check if D-frontier is empty AND fault is activated → FAILURE
    DLogic fsite = net_values_[fault.net];
    if (fsite == DLogic::D || fsite == DLogic::DBAR) {
        // Also check if any pseudo-PO already has D/DBAR (might have been missed by fault_propagated)
        bool has_d_frontier = false;
        for (size_t gid = 0; gid < nl_.gates().size(); ++gid) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0) continue;
            if (net_values_[g.output] != DLogic::X) continue;
            for (auto ni : g.inputs) {
                if (ni >= 0 && ni < (NetId)net_values_.size()) {
                    if (net_values_[ni] == DLogic::D || net_values_[ni] == DLogic::DBAR) {
                        has_d_frontier = true;
                        break;
                    }
                }
            }
            if (has_d_frontier) break;
        }
        if (!has_d_frontier) return false;
    }

    // Get objective
    NetId obj_net;
    DLogic obj_val;
    if (!objective(fault, obj_net, obj_val)) return false;

    NetId pi;
    Logic4 pi_val;
    if (!backtrace(obj_net, obj_val, pi, pi_val)) return false;

    // Try both PI assignments: recommended value first, then complement
    for (int attempt = 0; attempt < 2; ++attempt) {
        backtrack_count_++;
        if (backtrack_count_ > MAX_BACKTRACKS) return false;

        auto saved = net_values_;
        net_values_[pi] = (pi_val == Logic4::ONE) ? DLogic::ONE : DLogic::ZERO;

        if (podem_recursive(fault, depth + 1)) return true;

        // Backtrack
        net_values_ = saved;
        pi_val = (pi_val == Logic4::ONE) ? Logic4::ZERO : Logic4::ONE;
    }

    return false;
}

AtpgResult PodemAtpg::generate_test(const Fault& fault) {
    init_values();
    backtrack_count_ = 0;

    if (podem_recursive(fault)) {
        AtpgResult r;
        r.detected = true;
        r.message = "Test found for " + fault.str();
        // Include all controllable points (real PIs + pseudo-PIs from scan)
        for (auto pi : pi_set_) {
            if (pi >= 0 && pi < (NetId)net_values_.size()) {
                Logic4 val = Logic4::ZERO;
                if (net_values_[pi] == DLogic::ONE || net_values_[pi] == DLogic::D)
                    val = Logic4::ONE;
                r.test_vector.push_back({pi, val});
            }
        }
        return r;
    }
    return {false, {}, "No test found for " + fault.str()};
}

FaultCoverage PodemAtpg::run_full_atpg() {
    auto faults = enumerate_faults();
    FaultCoverage fc;
    fc.total_faults = faults.size();

    std::unordered_set<size_t> detected_faults;

    for (size_t fi = 0; fi < faults.size(); ++fi) {
        if (detected_faults.count(fi)) {
            fc.detected++;
            continue;
        }

        auto result = generate_test(faults[fi]);
        if (result.detected) {
            fc.detected++;
            detected_faults.insert(fi);
            fc.tests.push_back({faults[fi], result.test_vector});

            // Fault dropping: simulate this vector against remaining faults
            for (size_t fj = fi + 1; fj < faults.size(); ++fj) {
                if (detected_faults.count(fj)) continue;

                init_values();
                // Set PI values from test vector
                for (auto& [pi, val] : result.test_vector) {
                    if (pi >= 0 && pi < (NetId)net_values_.size()) {
                        net_values_[pi] = (val == Logic4::ONE) ? DLogic::ONE : DLogic::ZERO;
                    }
                }

                // Forward imply with fault fj injected inline
                forward_imply_with_fault(faults[fj]);

                if (fault_propagated(faults[fj])) {
                    detected_faults.insert(fj);
                }
            }
        } else {
            fc.undetectable++;
        }
    }

    return fc;
}

} // namespace sf
