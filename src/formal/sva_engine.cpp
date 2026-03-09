// SiliconForge — SVA Formal Engine
#include "formal/sva_engine.hpp"
#include <stdexcept>
#include <iostream>

namespace sf {

NetId SvaEngine::find_net_by_name(const Netlist& nl, const std::string& name) {
    for (const auto& net : nl.nets()) {
        if (net.name == name) return net.id;
    }
    throw std::runtime_error("SVA Engine: Cannot find net '" + name + "' in netlist.");
}

void SvaEngine::synthesize_assertions(Netlist& nl, const std::vector<SvaProperty>& props) {
    for (const auto& prop : props) {
        if (!prop.expr) continue;
        
        // Find clock
        NetId clk = -1;
        if (!prop.clock_domain.empty()) {
            clk = find_net_by_name(nl, prop.clock_domain);
        }

        auto root = prop.expr;
        if (root->op != SvaOp::PROP_OVERLAPPING && root->op != SvaOp::PROP_NON_OVERLAPPING) {
            std::cerr << "SVA Engine: Unsupported root operation for property " << prop.name << "\n";
            continue;
        }
        
        if (!root->left || !root->right || root->left->op != SvaOp::LITERAL || root->right->op != SvaOp::LITERAL) {
            std::cerr << "SVA Engine: Properties must currently be simple A |-> B format.\n";
            continue;
        }

        NetId ant_net = find_net_by_name(nl, root->left->literal);
        NetId cons_net = find_net_by_name(nl, root->right->literal);

        NetId check_ant = ant_net;

        if (root->op == SvaOp::PROP_NON_OVERLAPPING) {
            // delay antecedent by 1 cycle
            if (clk == -1) throw std::runtime_error("SVA Engine: Clock required for |=>");
            NetId delayed_ant = nl.add_net(prop.name + "_delayed_ant");
            nl.add_dff(ant_net, clk, delayed_ant, -1, "DFF_" + prop.name);
            check_ant = delayed_ant;
        }

        // We want to flag a violation if (check_ant == 1) and (cons_net == 0)
        // fail = check_ant AND (NOT cons_net)
        NetId not_cons = nl.add_net(prop.name + "_not_cons");
        nl.add_gate(GateType::NOT, {cons_net}, not_cons, "NOT_" + prop.name);

        NetId fail_net = nl.add_net("sva_fail_" + prop.name);
        nl.add_gate(GateType::AND, {check_ant, not_cons}, fail_net, "AND_" + prop.name);
        
        nl.mark_output(fail_net);
    }
}

} // namespace sf
