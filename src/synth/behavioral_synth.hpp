#pragma once
// SiliconForge — Behavioral Verilog Synthesis Engine
// Converts Behavioral AST nodes from the Verilog Parser into Structural Netlist Elements

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <set>

namespace sf {

// Basic AST elements parsed from structural and behavioral Verilog
enum class AstNodeType {
    MODULE, PORT_DECL, WIRE_DECL,
    ASSIGN,         // assign a = b & c;
    GATE_INST,      // and u1 (out, in1, in2);
    ALWAYS_POS_CLK, // always @(posedge clk)
    BLOCK_BEGIN_END,// begin ... end
    IF_ELSE,        // if (c) ... else ...
    NONBLOCK_ASSIGN,// a <= b;
    BIN_OP          // a + b
};

struct AstNode {
    AstNodeType type;
    std::string value; // e.g. "a", "+", "module_name"
    std::vector<std::shared_ptr<AstNode>> children;
    
    // Helper to add child
    void add(std::shared_ptr<AstNode> child) {
        children.push_back(child);
    }
};

class BehavioralAST {
public:
    std::string module_name;
    std::shared_ptr<AstNode> root;
    
    BehavioralAST() {
        root = std::make_shared<AstNode>();
        root->type = AstNodeType::MODULE;
    }
};

class BehavioralSynthesizer {
public:
    // Transforms an abstract behavioral AST into structural gates inside Netlist nl
    bool synthesize(const BehavioralAST& ast, Netlist& nl);

private:
    NetId get_net(Netlist& nl, const std::string& name);
    
    // Synthesis visitors
    void synth_node(std::shared_ptr<AstNode> node, Netlist& nl);
    NetId synth_expr(std::shared_ptr<AstNode> expr, Netlist& nl);
    
    // Basic Arithmetic Generators
    NetId build_adder(NetId a, NetId b, Netlist& nl);
    NetId build_subtractor(NetId a, NetId b, Netlist& nl);
    NetId build_mux2(NetId sel, NetId in1, NetId in0, Netlist& nl);
    
    // For handling state within always blocks
    struct BlockState {
        NetId clock_net = -1;
        NetId condition_net = -1; // -1 means true
    };
    
    void synth_always(std::shared_ptr<AstNode> node, Netlist& nl);
    void synth_statement(std::shared_ptr<AstNode> stmt, Netlist& nl, const BlockState& state, std::map<std::string, NetId>& next_state);
};

} // namespace sf
