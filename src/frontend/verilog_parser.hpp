#pragma once
// SiliconForge — Structural Verilog Parser
// Parses gate-level and structural Verilog netlists into internal Netlist.
// Supports: module, input, output, wire, assign, gate primitives, always @(posedge).

#include "core/netlist.hpp"
#include "synth/behavioral_synth.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

struct VerilogParseResult {
    bool success = false;
    std::string module_name;
    int num_inputs = 0;
    int num_outputs = 0;
    int num_wires = 0;
    int num_gates = 0;
    int num_ffs = 0;
    std::string error;
};

class VerilogParser {
public:
    // Returns true if parser detected behavioral blocks (always) requiring synthesis
    bool has_behavioral_blocks = false;
    BehavioralAST ast;

    VerilogParseResult parse_string(const std::string& src, Netlist& nl);
    VerilogParseResult parse_file(const std::string& filename, Netlist& nl);

    // Export netlist back to Verilog
    static std::string to_verilog(const Netlist& nl, const std::string& module_name = "top");

private:
    struct Token {
        enum Type { IDENT, NUMBER, LPAREN, RPAREN, LBRACKET, RBRACKET,
                    COMMA, SEMI, DOT, ASSIGN, TILDE, AMP, PIPE, CARET,
                    LBRACE, RBRACE, AT, HASH, COLON, QUESTION,
                    // Behavioral extensions
                    ALWAYS, IF, ELSE, BEGIN_KW, END_KW, POSEDGE, NEGEDGE,
                    LEQ, PLUS, MINUS, STAR, EQEQ, END };
        Type type;
        std::string value;
        int line = 0;
    };

    std::vector<Token> tokenize(const std::string& src);
    VerilogParseResult parse_tokens(const std::vector<Token>& tokens, Netlist& nl);

    // Parse helpers
    size_t parse_module(const std::vector<Token>& t, size_t pos, Netlist& nl, VerilogParseResult& r);
    size_t parse_port_decl(const std::vector<Token>& t, size_t pos, Netlist& nl,
                           VerilogParseResult& r, const std::string& dir);
    size_t parse_gate_inst(const std::vector<Token>& t, size_t pos, Netlist& nl,
                           VerilogParseResult& r, GateType gtype);
    size_t parse_assign(const std::vector<Token>& t, size_t pos, Netlist& nl, VerilogParseResult& r);
    
    // Behavioral block parsers
    size_t parse_always(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent);
    size_t parse_statement_block(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> block);
    size_t parse_statement(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent);
    std::shared_ptr<AstNode> parse_expression(const std::vector<Token>& t, size_t& pos);

    NetId get_or_create_net(Netlist& nl, const std::string& name);

    std::unordered_map<std::string, NetId> net_map_;
};

} // namespace sf
