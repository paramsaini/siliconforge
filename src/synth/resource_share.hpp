#pragma once
// SiliconForge — CDFG Resource Sharing (Area Optimization)
// Identifies mutually exclusive arithmetic operations in the Behavioral AST and shares them

#include "synth/behavioral_synth.hpp"
#include <vector>
#include <memory>
#include <map>
#include <set>
#include <string>

namespace sf {

// Represents an operation that can be shared (e.g., an Adder)
struct ResourceOp {
    std::string op_type; // e.g. "+", "*"
    std::shared_ptr<AstNode> node_ref;
    std::vector<std::string> control_path; // The IF/ELSE conditions that must be true to reach this op
};

class ResourceSharer {
public:
    // Analyzes the AST and modifies it so that mutually exclusive ops share a single execution node
    void optimize(std::shared_ptr<AstNode> root);

private:
    void extract_ops(std::shared_ptr<AstNode> node, std::vector<std::string>& current_path, std::vector<ResourceOp>& ops);
    bool are_mutually_exclusive(const std::vector<std::string>& pathA, const std::vector<std::string>& pathB);
    void apply_sharing(std::vector<ResourceOp>& ops, std::shared_ptr<AstNode> root);
};

} // namespace sf
