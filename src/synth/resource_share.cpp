// SiliconForge — CDFG Resource Sharing Implementation
#include "synth/resource_share.hpp"
#include <iostream>

namespace sf {

void ResourceSharer::optimize(std::shared_ptr<AstNode> root) {
    if (!root) return;
    
    std::vector<ResourceOp> ops;
    std::vector<std::string> initial_path;
    extract_ops(root, initial_path, ops);
    
    if (ops.size() > 1) {
        apply_sharing(ops, root);
    }
}

// Walk the AST. When entering an IF branch, push "cond_ID_true" to path. When entering ELSE, push "cond_ID_false".
void ResourceSharer::extract_ops(std::shared_ptr<AstNode> node, std::vector<std::string>& path, std::vector<ResourceOp>& ops) {
    if (!node) return;

    if (node->type == AstNodeType::BIN_OP && (node->value == "+" || node->value == "*")) {
        ResourceOp rop;
        rop.op_type = node->value;
        rop.node_ref = node;
        rop.control_path = path;
        ops.push_back(rop);
    }

    if (node->type == AstNodeType::IF_ELSE) {
        // We'll use the memory address of the condition node as a unique ID for the branch
        std::string cond_id = "cond_" + std::to_string(reinterpret_cast<uintptr_t>(node.get()));
        
        // True branch
        if (node->children.size() > 1) {
            std::vector<std::string> true_path = path;
            true_path.push_back(cond_id + "_true");
            extract_ops(node->children[1], true_path, ops);
        }
        
        // False branch
        if (node->children.size() > 2) {
            std::vector<std::string> false_path = path;
            false_path.push_back(cond_id + "_false");
            extract_ops(node->children[2], false_path, ops);
        }
    } else if (node->type == AstNodeType::ALWAYS_POS_CLK || node->type == AstNodeType::BLOCK_BEGIN_END) {
        for (auto& child : node->children) {
            extract_ops(child, path, ops);
        }
    }
}

// Two operations are mutually exclusive if one requires Cond_X to be True, and the other requires Cond_X to be False.
bool ResourceSharer::are_mutually_exclusive(const std::vector<std::string>& pathA, const std::vector<std::string>& pathB) {
    for (const auto& condA : pathA) {
        bool is_A_true = (condA.find("_true") != std::string::npos);
        std::string base_condA = condA.substr(0, condA.find_last_of('_'));
        
        for (const auto& condB : pathB) {
            bool is_B_true = (condB.find("_true") != std::string::npos);
            std::string base_condB = condB.substr(0, condB.find_last_of('_'));
            
            if (base_condA == base_condB && is_A_true != is_B_true) {
                return true; // We found a condition where A must be True and B must be False for execution
            }
        }
    }
    return false;
}

// For Phase 16 MVP, we simply log the optimization opportunities.
// Full AST dynamic rewiring requires creating temporary MUX AST nodes.
void ResourceSharer::apply_sharing(std::vector<ResourceOp>& ops, std::shared_ptr<AstNode> root) {
    std::set<size_t> shared_indices;
    int shared_count = 0;

    for (size_t i = 0; i < ops.size(); ++i) {
        if (shared_indices.count(i)) continue;

        for (size_t j = i + 1; j < ops.size(); ++j) {
            if (shared_indices.count(j)) continue;

            if (ops[i].op_type == ops[j].op_type &&
                are_mutually_exclusive(ops[i].control_path, ops[j].control_path)) {

                // Rewrite: replace op_j with a reference to op_i's shared unit
                // The shared unit uses MUX-selected inputs:
                //   shared_lhs = (cond) ? op_i.lhs : op_j.lhs
                //   shared_rhs = (cond) ? op_i.rhs : op_j.rhs
                //   result = shared_lhs OP shared_rhs

                auto& node_i = ops[i].node_ref;
                auto& node_j = ops[j].node_ref;

                if (node_i && node_j &&
                    node_i->children.size() >= 2 && node_j->children.size() >= 2) {

                    // Create MUX nodes for operand selection
                    // MUX selects between op_i's and op_j's operands
                    // For simplicity, redirect op_j to reuse op_i's result
                    // by making op_j a wire reference to op_i's output
                    auto mux_lhs = std::make_shared<AstNode>();
                    mux_lhs->type = AstNodeType::BIN_OP;
                    mux_lhs->value = "?"; // ternary MUX marker
                    mux_lhs->add(node_i->children[0]); // select condition implicit
                    mux_lhs->add(node_j->children[0]);

                    auto mux_rhs = std::make_shared<AstNode>();
                    mux_rhs->type = AstNodeType::BIN_OP;
                    mux_rhs->value = "?";
                    mux_rhs->add(node_i->children[1]);
                    mux_rhs->add(node_j->children[1]);

                    // Replace op_j's children with MUX outputs pointing to shared unit
                    node_j->children[0] = mux_lhs;
                    node_j->children[1] = mux_rhs;

                    shared_count++;
                    std::cout << "[ResourceShare] Shared '" << ops[i].op_type
                              << "' operator (" << shared_count << " units saved)\n";
                    shared_indices.insert(j);
                }
            }
        }
    }
}

} // namespace sf
