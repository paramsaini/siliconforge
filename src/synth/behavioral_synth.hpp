#pragma once
// SiliconForge — Behavioral Verilog Synthesis Engine
// Converts Behavioral AST nodes from the Verilog Parser into Structural Netlist Elements

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <set>
#include <unordered_map>

namespace sf {

// AST node types for structural and behavioral Verilog
enum class AstNodeType {
    MODULE, PORT_DECL, WIRE_DECL, NUMBER_LITERAL,
    ASSIGN,           // assign a = b & c;
    GATE_INST,        // and u1 (out, in1, in2);
    ALWAYS_POS_CLK,   // always @(posedge clk)
    ALWAYS_NEG_CLK,   // always @(negedge clk)
    ALWAYS_COMB,      // always @(*) — combinational, no DFFs
    BLOCK_BEGIN_END,  // begin ... end
    IF_ELSE,          // if (c) ... else ...
    NONBLOCK_ASSIGN,  // a <= b;
    BLOCK_ASSIGN,     // a = b; (blocking in always)
    BIN_OP,           // a + b, a - b, a * b, a << b, a < b, etc.
    UNARY_OP,         // ~a, !a, -a, &a, |a, ^a (reduction)
    TERNARY_OP,       // cond ? true_val : false_val
    CONCAT,           // {a, b, c}
    REPLICATE,        // {N{expr}}
    CASE_STMT,        // case(sel) ... endcase
    CASE_ITEM,        // value: statement
    FOR_LOOP,         // for(init; cond; incr) body
    MEM_READ,         // mem[addr] — value=mem_name, child[0]=addr expr
    MEM_WRITE         // mem[addr] <= data — value=mem_name, child[0]=addr, child[1]=data
};

struct AstNode {
    AstNodeType type;
    std::string value; // e.g. "a", "+", "module_name"
    std::vector<std::shared_ptr<AstNode>> children;
    int int_val = 0;   // for NUMBER_LITERAL

    void add(std::shared_ptr<AstNode> child) {
        children.push_back(child);
    }

    static std::shared_ptr<AstNode> make(AstNodeType t, const std::string& v = "") {
        auto n = std::make_shared<AstNode>();
        n->type = t; n->value = v;
        return n;
    }
};

class BehavioralAST {
public:
    std::string module_name;
    std::shared_ptr<AstNode> root;
    // Bus width info populated by parser — enables multi-bit behavioral synthesis
    std::unordered_map<std::string, std::pair<int,int>> bus_ranges;
    std::unordered_map<std::string, NetId> net_map;
    // Memory arrays: name -> {word_width, depth}
    std::unordered_map<std::string, std::pair<int,int>> memory_arrays;
    // Signed signal set
    std::set<std::string> signed_signals;

    BehavioralAST() {
        root = std::make_shared<AstNode>();
        root->type = AstNodeType::MODULE;
    }
};

class BehavioralSynthesizer {
public:
    bool synthesize(const BehavioralAST& ast, Netlist& nl);

private:
    // Bus info from AST
    std::unordered_map<std::string, std::pair<int,int>> bus_ranges_;
    std::unordered_map<std::string, NetId> net_map_ref_;
    // Memory arrays: name -> {word_width, depth}
    std::unordered_map<std::string, std::pair<int,int>> memory_arrays_;
    // Signed signals: auto-propagate signedness to comparisons
    std::set<std::string> signed_signals_;
    int uid_counter_ = 0;
    std::string uid();

    // Width helpers
    int get_bus_width(const std::string& name) const;
    int get_bus_lo(const std::string& name) const;
    int infer_expr_width(std::shared_ptr<AstNode> expr) const;

    // Net helpers
    NetId get_net(Netlist& nl, const std::string& name);
    std::vector<NetId> get_bus_nets(Netlist& nl, const std::string& name,
                                     const std::map<std::string, std::vector<NetId>>& next_state);
    NetId make_const(int bit, Netlist& nl);
    std::vector<NetId> const_to_bits(const std::string& val_str, int width, Netlist& nl);
    std::vector<NetId> pad_to_width(std::vector<NetId> bits, int width, Netlist& nl);
    uint64_t parse_number(const std::string& s) const;
    int get_const_value(std::shared_ptr<AstNode> expr) const;

    // Synthesis visitors
    void synth_node(std::shared_ptr<AstNode> node, Netlist& nl);

    // Expression synthesis — bus-aware
    NetId synth_expr_single(std::shared_ptr<AstNode> expr, Netlist& nl,
                            const std::map<std::string, std::vector<NetId>>& next_state);
    std::vector<NetId> synth_expr_bus(std::shared_ptr<AstNode> expr, Netlist& nl,
                                       const std::map<std::string, std::vector<NetId>>& next_state,
                                       int target_w);

    // Reduction and bitwise
    NetId reduce(const std::vector<NetId>& bits, const std::string& op, Netlist& nl);
    std::vector<NetId> bitwise_op(const std::vector<NetId>& a, const std::vector<NetId>& b, GateType gt, Netlist& nl);

    // Single-bit arithmetic builders
    NetId build_adder(NetId a, NetId b, Netlist& nl);
    NetId build_subtractor(NetId a, NetId b, Netlist& nl);
    NetId build_mux2(NetId sel, NetId in1, NetId in0, Netlist& nl);
    NetId build_comparator_lt(NetId a, NetId b, Netlist& nl);
    NetId build_comparator_gt(NetId a, NetId b, Netlist& nl);

    // Multi-bit arithmetic builders
    std::vector<NetId> build_bus_adder(const std::vector<NetId>& a, const std::vector<NetId>& b, Netlist& nl);
    std::vector<NetId> build_bus_subtractor(const std::vector<NetId>& a, const std::vector<NetId>& b, Netlist& nl);
    std::vector<NetId> build_bus_multiplier(const std::vector<NetId>& a, const std::vector<NetId>& b, Netlist& nl);
    std::vector<NetId> build_bus_shift_left(const std::vector<NetId>& data, int amount, Netlist& nl);
    std::vector<NetId> build_bus_shift_right(const std::vector<NetId>& data, int amount, Netlist& nl);
    std::vector<NetId> build_barrel_shift_left(const std::vector<NetId>& data, const std::vector<NetId>& shamt, Netlist& nl);
    std::vector<NetId> build_barrel_shift_right(const std::vector<NetId>& data, const std::vector<NetId>& shamt, Netlist& nl);
    NetId build_bus_lt(const std::vector<NetId>& a, const std::vector<NetId>& b, Netlist& nl);
    NetId build_bus_lt_signed(const std::vector<NetId>& a, const std::vector<NetId>& b, Netlist& nl);
    NetId build_bus_eq(const std::vector<NetId>& a, const std::vector<NetId>& b, Netlist& nl);
    std::vector<NetId> build_bus_mux(NetId sel, const std::vector<NetId>& t_val, const std::vector<NetId>& f_val, Netlist& nl);

    // Arithmetic right shift builders
    std::vector<NetId> build_arith_shift_right(const std::vector<NetId>& data, int amt, Netlist& nl);
    std::vector<NetId> build_barrel_arith_shift_right(const std::vector<NetId>& data, const std::vector<NetId>& shamt, Netlist& nl);

    // Division/modulo: returns {quotient, remainder}
    std::pair<std::vector<NetId>, std::vector<NetId>> build_divider(const std::vector<NetId>& a, const std::vector<NetId>& b, Netlist& nl);

    // Memory array synthesis
    std::vector<NetId> synth_mem_read(const std::string& mem_name, std::shared_ptr<AstNode> addr_expr,
        Netlist& nl, const std::map<std::string, std::vector<NetId>>& next_state, int target_w);
    void synth_mem_write(const std::string& mem_name, std::shared_ptr<AstNode> addr_expr,
        std::shared_ptr<AstNode> data_expr, Netlist& nl, std::map<std::string, std::vector<NetId>>& next_state);

    // For handling state within always blocks
    struct BlockState {
        NetId clock_net = -1;
        bool is_combinational = false;
    };

    void synth_always(std::shared_ptr<AstNode> node, Netlist& nl);
    void synth_always_comb(std::shared_ptr<AstNode> node, Netlist& nl);
    void synth_statement(std::shared_ptr<AstNode> stmt, Netlist& nl, const BlockState& state,
                         std::map<std::string, std::vector<NetId>>& next_state);
};

// Structural expression synthesizer — used for continuous assigns
class StructuralSynthesizer {
public:
    NetId synth_expr(std::shared_ptr<AstNode> expr, Netlist& nl,
                     std::unordered_map<std::string, NetId>& net_map);
    std::vector<NetId> synth_expr_bus(std::shared_ptr<AstNode> expr, Netlist& nl,
                                       std::unordered_map<std::string, NetId>& net_map,
                                       const std::unordered_map<std::string, std::pair<int,int>>& bus_ranges,
                                       int target_width);
private:
    NetId get_or_create(Netlist& nl, std::unordered_map<std::string, NetId>& net_map, const std::string& name);
    int infer_width(std::shared_ptr<AstNode> expr,
                    const std::unordered_map<std::string, std::pair<int,int>>& bus_ranges);
    int uid_ = 0;
    std::string uid() { return std::to_string(uid_++); }
};

} // namespace sf
