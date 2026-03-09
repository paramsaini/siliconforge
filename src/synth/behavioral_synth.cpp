// SiliconForge — Behavioral Verilog Synthesis Engine Implementation
#include "synth/behavioral_synth.hpp"
#include <iostream>

namespace sf {

bool BehavioralSynthesizer::synthesize(const BehavioralAST& ast, Netlist& nl) {
    if (!ast.root || ast.root->type != AstNodeType::MODULE) return false;
    synth_node(ast.root, nl);
    return true;
}

NetId BehavioralSynthesizer::get_net(Netlist& nl, const std::string& name) {
    // Check if net already exists in the structural scope
    for (size_t i = 0; i < nl.num_nets(); ++i) {
        if (nl.net(i).name == name) return i;
    }
    return nl.add_net(name);
}

void BehavioralSynthesizer::synth_node(std::shared_ptr<AstNode> node, Netlist& nl) {
    if (!node) return;

    if (node->type == AstNodeType::MODULE) {
        for (auto& child : node->children) {
            synth_node(child, nl);
        }
    }
    // Behavioral Synthesis Specific: Lowering 'always' blocks
    else if (node->type == AstNodeType::ALWAYS_POS_CLK) {
        synth_always(node, nl);
    }
}

void BehavioralSynthesizer::synth_always(std::shared_ptr<AstNode> node, Netlist& nl) {
    // Node structure: Always -> Block(Begin/End) -> statements
    // We assume statements inside always@(posedge clk) infer DFFs
    BlockState state;
    
    // For simplicity in this engine, assume there is a global clk in the nl
    state.clock_net = get_net(nl, "clk");
    
    std::map<std::string, NetId> next_state;
    
    for (auto& child : node->children) {
        if (child->type == AstNodeType::BLOCK_BEGIN_END) {
            for (auto& stmt : child->children) {
                synth_statement(stmt, nl, state, next_state);
            }
        } else {
            synth_statement(child, nl, state, next_state);
        }
    }
    
    // Infer DFFs for all assigned variables
    for (auto& pair : next_state) {
        std::string var = pair.first;
        NetId d_net = pair.second;
        NetId q_net = get_net(nl, var);
        nl.add_dff(d_net, state.clock_net, q_net, -1, "reg_" + var);
    }
}

void BehavioralSynthesizer::synth_statement(std::shared_ptr<AstNode> stmt, Netlist& nl, const BlockState& state, std::map<std::string, NetId>& next_state) {
    if (!stmt) return;

    if (stmt->type == AstNodeType::NONBLOCK_ASSIGN) {
        // q <= expr;
        std::string target_name = stmt->value;
        if (stmt->children.size() > 0) {
            NetId d_net = synth_expr(stmt->children[0], nl);
            next_state[target_name] = d_net;
        }
    }
    else if (stmt->type == AstNodeType::BLOCK_BEGIN_END) {
        for (auto& child : stmt->children) {
            synth_statement(child, nl, state, next_state);
        }
    }
    else if (stmt->type == AstNodeType::IF_ELSE) {
        if (stmt->children.empty()) return;
        
        NetId cond_net = synth_expr(stmt->children[0], nl);
        
        // True branch
        std::map<std::string, NetId> true_state = next_state;
        if (stmt->children.size() > 1) {
            synth_statement(stmt->children[1], nl, state, true_state);
        }
        
        // False branch
        std::map<std::string, NetId> false_state = next_state;
        if (stmt->children.size() > 2) {
            synth_statement(stmt->children[2], nl, state, false_state);
        }
        
        // Merge states: any variable modified in either branch needs a MUX
        std::set<std::string> modified_vars;
        for (auto& pair : true_state) {
            if (next_state.find(pair.first) == next_state.end() || next_state[pair.first] != pair.second)
                modified_vars.insert(pair.first);
        }
        for (auto& pair : false_state) {
            if (next_state.find(pair.first) == next_state.end() || next_state[pair.first] != pair.second)
                modified_vars.insert(pair.first);
        }
        
        for (const auto& var : modified_vars) {
            NetId true_val = true_state.count(var) ? true_state[var] : get_net(nl, var);
            NetId false_val = false_state.count(var) ? false_state[var] : get_net(nl, var);
            next_state[var] = build_mux2(cond_net, true_val, false_val, nl);
        }
    }
}

NetId BehavioralSynthesizer::synth_expr(std::shared_ptr<AstNode> expr, Netlist& nl) {
    if (!expr) return -1;

    // Is it a literal or a direct variable?
    if (expr->type == AstNodeType::WIRE_DECL || expr->children.empty()) {
        return get_net(nl, expr->value);
    }
    
    // Binary Operator Lowering
    if (expr->type == AstNodeType::BIN_OP) {
        NetId lhs = synth_expr(expr->children[0], nl);
        NetId rhs = synth_expr(expr->children[1], nl);

        if (expr->value == "+") {
            return build_adder(lhs, rhs, nl);
        } else if (expr->value == "^") {
            NetId out = nl.add_net("xor_out_" + std::to_string(nl.num_nets()));
            nl.add_gate(GateType::XOR, {lhs, rhs}, out, "synth_xor");
            return out;
        } else if (expr->value == "&") {
            NetId out = nl.add_net("and_out_" + std::to_string(nl.num_nets()));
            nl.add_gate(GateType::AND, {lhs, rhs}, out, "synth_and");
            return out;
        } else if (expr->value == "==") {
            NetId xor_out = nl.add_net("eq_xor_" + std::to_string(nl.num_nets()));
            nl.add_gate(GateType::XOR, {lhs, rhs}, xor_out, "synth_eq_xor");
            NetId not_out = nl.add_net("eq_not_" + std::to_string(nl.num_nets()));
            nl.add_gate(GateType::NOT, {xor_out}, not_out, "synth_eq_not");
            return not_out;
        }
    }

    return -1;
}

// Lowers A + B into structural Half-Adder / Full-Adder logic
NetId BehavioralSynthesizer::build_adder(NetId a, NetId b, Netlist& nl) {
    // For single-bit inputs, an adder is a Half-Adder:
    // Sum = A ^ B
    // Carry = A & B (ignored for 1-bit output)
    NetId sum_out = nl.add_net("sum_" + std::to_string(a) + "_" + std::to_string(b));
    nl.add_gate(GateType::XOR, {a, b}, sum_out, "ha_sum");
    return sum_out;
}

NetId BehavioralSynthesizer::build_mux2(NetId sel, NetId in1, NetId in0, Netlist& nl) {
    // out = (sel & in1) | (~sel & in0)
    NetId not_sel = nl.add_net("not_sel_" + std::to_string(sel));
    nl.add_gate(GateType::NOT, {sel}, not_sel, "mux_inv");
    
    NetId and1 = nl.add_net("mux_and1");
    nl.add_gate(GateType::AND, {sel, in1}, and1, "mux_and1");

    NetId and0 = nl.add_net("mux_and0");
    nl.add_gate(GateType::AND, {not_sel, in0}, and0, "mux_and0");

    NetId out = nl.add_net("mux_out");
    nl.add_gate(GateType::OR, {and1, and0}, out, "mux_or");
    
    return out;
}

} // namespace sf
