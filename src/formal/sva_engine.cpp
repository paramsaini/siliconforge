// SiliconForge — SVA Formal Engine
#include "formal/sva_engine.hpp"
#include <stdexcept>
#include <iostream>
#include <sstream>

namespace sf {

// ── Helpers ─────────────────────────────────────────────────────────────────

NetId SvaEngine::find_net_by_name(const Netlist& nl, const std::string& name) {
    for (const auto& net : nl.nets()) {
        if (net.name == name) return net.id;
    }
    throw std::runtime_error("SVA Engine: Cannot find net '" + name + "' in netlist.");
}

// ── Recursive node synthesizer ──────────────────────────────────────────────

NetId SvaEngine::synthesize_node(Netlist& nl, const SvaNode* node, NetId clk,
                                  const std::string& prefix, int& counter) {
    if (!node)
        throw std::runtime_error("SVA Engine: null SvaNode in expression tree.");

    auto tag = [&](const std::string& suffix) {
        return prefix + "_" + suffix + std::to_string(counter++);
    };

    switch (node->op) {

    case SvaOp::LITERAL:
        return find_net_by_name(nl, node->literal);

    case SvaOp::NOT: {
        NetId child = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        NetId out = nl.add_net(tag("not"));
        nl.add_gate(GateType::NOT, {child}, out, tag("NOT"));
        return out;
    }

    case SvaOp::AND: {
        NetId l = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        NetId r = synthesize_node(nl, node->right.get(), clk, prefix, counter);
        NetId out = nl.add_net(tag("and"));
        nl.add_gate(GateType::AND, {l, r}, out, tag("AND"));
        return out;
    }

    case SvaOp::OR: {
        NetId l = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        NetId r = synthesize_node(nl, node->right.get(), clk, prefix, counter);
        NetId out = nl.add_net(tag("or"));
        nl.add_gate(GateType::OR, {l, r}, out, tag("OR"));
        return out;
    }

    case SvaOp::NEXT:
    case SvaOp::DELAY: {
        NetId cur = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        int cycles = node->delay_cycles;
        if (cycles < 1) cycles = 1;
        if (clk < 0)
            throw std::runtime_error("SVA Engine: Clock required for DELAY/NEXT.");
        for (int i = 0; i < cycles; ++i) {
            NetId q = nl.add_net(tag("dly"));
            nl.add_dff(cur, clk, q, -1, tag("DFF"));
            cur = q;
        }
        return cur;
    }

    case SvaOp::GLOBALLY: {
        // Monitor: fail on any cycle where property is false → NOT(signal)
        NetId child = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        NetId inv = nl.add_net(tag("glob_inv"));
        nl.add_gate(GateType::NOT, {child}, inv, tag("GNOT"));
        // Double invert so output == child (monitor flag is the inverted version
        // captured externally); return the raw signal for upstream composition.
        return child;
    }

    case SvaOp::EVENTUALLY: {
        // seen |= current  →  OR(current, DFF(seen))
        NetId child = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        if (clk < 0)
            throw std::runtime_error("SVA Engine: Clock required for EVENTUALLY.");
        NetId fb = nl.add_net(tag("ev_fb"));
        NetId seen = nl.add_net(tag("ev_seen"));
        nl.add_gate(GateType::OR, {child, fb}, seen, tag("EVOR"));
        nl.add_dff(seen, clk, fb, -1, tag("EVDFF"));
        return seen;
    }

    case SvaOp::UNTIL: {
        // a holds until b arrives: monitor = a_held & b_arrived
        NetId a = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        NetId b = synthesize_node(nl, node->right.get(), clk, prefix, counter);
        if (clk < 0)
            throw std::runtime_error("SVA Engine: Clock required for UNTIL.");
        // a_held: feedback DFF, AND(a, prev_held), initialised to 1 (first cycle OK)
        NetId fb = nl.add_net(tag("u_fb"));
        NetId held = nl.add_net(tag("u_held"));
        nl.add_gate(GateType::AND, {a, fb}, held, tag("UAND"));
        nl.add_dff(held, clk, fb, -1, tag("UDFF"));
        NetId result = nl.add_net(tag("u_res"));
        nl.add_gate(GateType::AND, {held, b}, result, tag("URES"));
        return result;
    }

    case SvaOp::REPEAT: {
        // N consecutive copies with DFF chain
        NetId child = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        int n = node->delay_cycles;
        if (n < 1) n = 1;
        if (clk < 0)
            throw std::runtime_error("SVA Engine: Clock required for REPEAT.");
        NetId cur = child;
        for (int i = 1; i < n; ++i) {
            NetId delayed = nl.add_net(tag("rep_d"));
            nl.add_dff(cur, clk, delayed, -1, tag("REPDFF"));
            NetId anded = nl.add_net(tag("rep_a"));
            nl.add_gate(GateType::AND, {child, delayed}, anded, tag("REPAND"));
            cur = anded;
        }
        return cur;
    }

    case SvaOp::SEQUENCE_CONCAT: {
        // Chain left then right with DFF delay between them
        NetId l = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        if (clk < 0)
            throw std::runtime_error("SVA Engine: Clock required for SEQUENCE_CONCAT.");
        NetId delayed_l = nl.add_net(tag("seq_d"));
        nl.add_dff(l, clk, delayed_l, -1, tag("SEQDFF"));
        NetId r = synthesize_node(nl, node->right.get(), clk, prefix, counter);
        NetId out = nl.add_net(tag("seq_o"));
        nl.add_gate(GateType::AND, {delayed_l, r}, out, tag("SEQAND"));
        return out;
    }

    case SvaOp::PROP_OVERLAPPING: {
        NetId ant = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        NetId cons = synthesize_node(nl, node->right.get(), clk, prefix, counter);
        NetId out = nl.add_net(tag("ovl"));
        nl.add_gate(GateType::AND, {ant, cons}, out, tag("OVLAND"));
        return out;
    }

    case SvaOp::PROP_NON_OVERLAPPING: {
        NetId ant = synthesize_node(nl, node->left.get(), clk, prefix, counter);
        if (clk < 0)
            throw std::runtime_error("SVA Engine: Clock required for |=>.");
        NetId delayed_ant = nl.add_net(tag("novl_d"));
        nl.add_dff(ant, clk, delayed_ant, -1, tag("NOVLDFF"));
        NetId cons = synthesize_node(nl, node->right.get(), clk, prefix, counter);
        NetId out = nl.add_net(tag("novl"));
        nl.add_gate(GateType::AND, {delayed_ant, cons}, out, tag("NOVLAND"));
        return out;
    }

    } // switch

    throw std::runtime_error("SVA Engine: Unhandled SvaOp " +
                             std::to_string(static_cast<int>(node->op)));
}

// ── Enhanced synthesizer ────────────────────────────────────────────────────

SvaMonitorResult SvaEngine::synthesize_enhanced(Netlist& nl,
                                                 const std::vector<SvaProperty>& props) {
    SvaMonitorResult res;
    size_t nets_before = nl.num_nets();
    size_t gates_before = nl.num_gates();
    size_t dffs_before = nl.flip_flops().size();

    for (const auto& prop : props) {
        if (!prop.expr) continue;

        NetId clk = -1;
        if (!prop.clock_domain.empty())
            clk = find_net_by_name(nl, prop.clock_domain);

        int counter = 0;
        std::string prefix = "sva_" + prop.name;

        // Synthesize the full expression tree
        NetId expr_net = synthesize_node(nl, prop.expr.get(), clk, prefix, counter);

        if (prop.is_cover) {
            // Cover: hit when expression is true
            std::string oname = "sva_cover_" + prop.name;
            NetId cover_out = nl.add_net(oname);
            nl.add_gate(GateType::BUF, {expr_net}, cover_out, "BUF_cover_" + prop.name);
            nl.mark_output(cover_out);
            res.cover_monitors++;
            res.monitor_outputs.push_back(oname);
        } else if (prop.is_assume) {
            // Assume: constraint output (solver should hold this true)
            std::string oname = "sva_assume_" + prop.name;
            NetId assume_out = nl.add_net(oname);
            nl.add_gate(GateType::BUF, {expr_net}, assume_out, "BUF_assume_" + prop.name);
            nl.mark_output(assume_out);
            res.assume_monitors++;
            res.monitor_outputs.push_back(oname);
        } else {
            // Assert (default): fail = antecedent & !consequent
            // For property-level nodes the expr_net already represents ant&cons;
            // the fail signal fires when ant is true but cons is false.
            // For simple implication roots we invert the consequent.
            auto root = prop.expr;
            if (root->op == SvaOp::PROP_OVERLAPPING ||
                root->op == SvaOp::PROP_NON_OVERLAPPING) {
                // Re-synthesize antecedent/consequent individually for fail logic
                int c2 = 0;
                NetId ant = synthesize_node(nl, root->left.get(), clk, prefix + "_f", c2);
                NetId cons;
                if (root->op == SvaOp::PROP_NON_OVERLAPPING) {
                    NetId ant_d = nl.add_net(prefix + "_fant_d");
                    nl.add_dff(ant, clk, ant_d, -1, "DFF_fant_" + prop.name);
                    ant = ant_d;
                }
                cons = synthesize_node(nl, root->right.get(), clk, prefix + "_f", c2);
                NetId not_cons = nl.add_net(prefix + "_fnot");
                nl.add_gate(GateType::NOT, {cons}, not_cons, "NOT_f_" + prop.name);
                std::string oname = "sva_fail_" + prop.name;
                NetId fail_net = nl.add_net(oname);
                nl.add_gate(GateType::AND, {ant, not_cons}, fail_net, "AND_f_" + prop.name);
                nl.mark_output(fail_net);
                res.monitor_outputs.push_back(oname);
            } else {
                // Non-implication assert: fail when expression is false
                NetId inv = nl.add_net(prefix + "_fail_inv");
                nl.add_gate(GateType::NOT, {expr_net}, inv, "NOT_fail_" + prop.name);
                std::string oname = "sva_fail_" + prop.name;
                NetId fail_net = nl.add_net(oname);
                nl.add_gate(GateType::BUF, {inv}, fail_net, "BUF_fail_" + prop.name);
                nl.mark_output(fail_net);
                res.monitor_outputs.push_back(oname);
            }
            res.assert_monitors++;
        }
        res.properties_synthesized++;
    }

    res.delay_registers = static_cast<int>(nl.flip_flops().size() - dffs_before);
    res.logic_gates     = static_cast<int>((nl.num_gates() - gates_before)
                                           - static_cast<size_t>(res.delay_registers));

    std::ostringstream rpt;
    rpt << "SVA synthesis: " << res.properties_synthesized << " properties ("
        << res.assert_monitors << " assert, " << res.cover_monitors << " cover, "
        << res.assume_monitors << " assume), "
        << res.delay_registers << " DFFs, " << res.logic_gates << " gates";
    res.report = rpt.str();
    return res;
}

// ── Legacy entry point ──────────────────────────────────────────────────────

void SvaEngine::synthesize_assertions(Netlist& nl,
                                       const std::vector<SvaProperty>& props) {
    synthesize_enhanced(nl, props);
}

} // namespace sf
