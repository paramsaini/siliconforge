// SiliconForge — Algorithmic FSM Extraction & Encoding Implementation
#include "synth/fsm_extract.hpp"
#include <iostream>

namespace sf {

std::vector<StateMachine> FsmExtractor::extract_fsms(std::shared_ptr<AstNode> root) {
    std::vector<StateMachine> fsms;
    find_state_machines(root, fsms);
    return fsms;
}

void FsmExtractor::find_state_machines(std::shared_ptr<AstNode> node, std::vector<StateMachine>& fsms) {
    if (!node) return;

    // A state machine is typically modeled inside an ALWAYS_POS_CLK block
    if (node->type == AstNodeType::ALWAYS_POS_CLK) {
        // Look for a switch/case on a state variable, or large if/else tree
        // For phase 16 MVP, we'll extract explicit IF_ELSE structures assigning to the same variable
        StateMachine fsm;
        extract_transitions(node, fsm);
        
        if (fsm.state_names.size() > 1) { // Found a valid state machine with at least 2 states
            std::cout << "[FSM] Extracted State Machine for variable '" << fsm.state_var_name << "' with " << fsm.num_states << " states.\n";
            fsms.push_back(fsm);
        }
    } else {
        for (auto& child : node->children) {
            find_state_machines(child, fsms);
        }
    }
}

// Simple heuristic: Walk the AST of the always block. If an IF_ELSE checks a variable against a literal,
// and assigns functionally to that same variable, infer it as a State Register.
// E.g., if (state == 0) state <= 1;
void FsmExtractor::extract_transitions(std::shared_ptr<AstNode> node, StateMachine& fsm) {
    if (!node) return;

    if (node->type == AstNodeType::IF_ELSE && node->children.size() > 0) {
        auto cond = node->children[0];
        if (cond->type == AstNodeType::BIN_OP && cond->value == "==") {
            auto lhs = cond->children[0];
            auto rhs = cond->children[1];
            
            // Assuming lhs is the variable and rhs is the literal state value
            if (lhs->children.empty() && rhs->children.empty()) { // Simple wire and literal
                if (fsm.state_var_name.empty()) {
                    fsm.state_var_name = lhs->value;
                }
                
                if (lhs->value == fsm.state_var_name) {
                    std::string from_state = rhs->value;
                    if (std::find(fsm.state_names.begin(), fsm.state_names.end(), from_state) == fsm.state_names.end()) {
                        fsm.state_names.push_back(from_state);
                        fsm.num_states++;
                    }
                }
            }
        }
    }
    
    // Only recurse into control flow blocks, not individual expressions to avoid infinite loops
    if (node->type == AstNodeType::ALWAYS_POS_CLK || 
        node->type == AstNodeType::BLOCK_BEGIN_END || 
        node->type == AstNodeType::IF_ELSE) {
        for (auto& child : node->children) {
            extract_transitions(child, fsm);
        }
    }
}

// Apply Mathematical State Encodings
void FsmExtractor::apply_encoding(StateMachine& fsm, FsmEncoding encoding) {
    fsm.encoded_values.clear();
    
    for (int i = 0; i < fsm.num_states; ++i) {
        int encoded_val = 0;
        
        if (encoding == FsmEncoding::BINARY) {
            encoded_val = i;
        } else if (encoding == FsmEncoding::ONE_HOT) {
            encoded_val = 1 << i;
        } else if (encoding == FsmEncoding::GRAY_CODE) {
            encoded_val = i ^ (i >> 1);
        }
        
        fsm.encoded_values[fsm.state_names[i]] = encoded_val;
    }
    
    std::cout << "[FSM] Re-encoded '" << fsm.state_var_name << "' elements.\n";
}

void FsmExtractor::optimize_fsms(std::shared_ptr<AstNode> root, const std::vector<StateMachine>& fsms, FsmEncoding encoding) {
    for (auto fsm : fsms) {
        apply_encoding(fsm, encoding);
        rewrite_ast(root, fsm);
        std::cout << "[FSM] Optimized AST for State Machine '" << fsm.state_var_name << "' using advanced encoding.\n";
    }
}

// Rewrites AST conditional == checks and <= assignments with the new mathematically optimal encoding
void FsmExtractor::rewrite_ast(std::shared_ptr<AstNode> node, const StateMachine& fsm) {
    if (!node) return;
    
    // Replace literals in comparisons (if state == 0)
    if (node->type == AstNodeType::BIN_OP && node->value == "==") {
        if (!node->children.empty() && node->children[0]->value == fsm.state_var_name) {
            std::string old_val = node->children[1]->value;
            if (fsm.encoded_values.count(old_val)) {
                node->children[1]->value = std::to_string(fsm.encoded_values.at(old_val));
            }
        }
    }
    
    // Replace literals in assignments (state <= 1)
    if (node->type == AstNodeType::NONBLOCK_ASSIGN && node->value == fsm.state_var_name) {
        if (!node->children.empty() && node->children[0]->children.empty()) { // Simple wire/literal assignment
            std::string old_val = node->children[0]->value;
            if (fsm.encoded_values.count(old_val)) {
                node->children[0]->value = std::to_string(fsm.encoded_values.at(old_val));
            }
        }
    }
    
    for (auto& child : node->children) {
        rewrite_ast(child, fsm);
    }
}

} // namespace sf
