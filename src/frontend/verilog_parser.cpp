// SiliconForge — Structural Verilog Parser Implementation
#include "frontend/verilog_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>

namespace sf {

NetId VerilogParser::get_or_create_net(Netlist& nl, const std::string& name) {
    auto it = net_map_.find(name);
    if (it != net_map_.end()) return it->second;
    NetId id = nl.add_net(name);
    net_map_[name] = id;
    return id;
}

std::vector<VerilogParser::Token> VerilogParser::tokenize(const std::string& src) {
    std::vector<Token> tokens;
    int line = 1;
    size_t i = 0;
    while (i < src.size()) {
        char c = src[i];
        if (c == '\n') { line++; i++; continue; }
        if (std::isspace(c)) { i++; continue; }

        // Line comments
        if (c == '/' && i+1 < src.size() && src[i+1] == '/') {
            while (i < src.size() && src[i] != '\n') i++;
            continue;
        }
        // Block comments
        if (c == '/' && i+1 < src.size() && src[i+1] == '*') {
            i += 2;
            while (i+1 < src.size() && !(src[i]=='*' && src[i+1]=='/')) {
                if (src[i]=='\n') line++;
                i++;
            }
            i += 2; continue;
        }

        if (c == '(') { tokens.push_back({Token::LPAREN, "(", line}); i++; }
        else if (c == ')') { tokens.push_back({Token::RPAREN, ")", line}); i++; }
        else if (c == '[') { tokens.push_back({Token::LBRACKET, "[", line}); i++; }
        else if (c == ']') { tokens.push_back({Token::RBRACKET, "]", line}); i++; }
        else if (c == '{') { tokens.push_back({Token::LBRACE, "{", line}); i++; }
        else if (c == '}') { tokens.push_back({Token::RBRACE, "}", line}); i++; }
        else if (c == ',') { tokens.push_back({Token::COMMA, ",", line}); i++; }
        else if (c == ';') { tokens.push_back({Token::SEMI, ";", line}); i++; }
        else if (c == '.') { tokens.push_back({Token::DOT, ".", line}); i++; }
        else if (c == '=') {
            if (i + 1 < src.size() && src[i + 1] == '=') {
                tokens.push_back({Token::EQEQ, "==", line}); i += 2;
            } else {
                tokens.push_back({Token::ASSIGN, "=", line}); i++;
            }
        }
        else if (c == '<') {
            if (i + 1 < src.size() && src[i + 1] == '=') {
                tokens.push_back({Token::LEQ, "<=", line}); i += 2;
            } else i++;
        }
        else if (c == '+') { tokens.push_back({Token::PLUS, "+", line}); i++; }
        else if (c == '-') { tokens.push_back({Token::MINUS, "-", line}); i++; }
        else if (c == '*') { tokens.push_back({Token::STAR, "*", line}); i++; }
        else if (c == '~') { tokens.push_back({Token::TILDE, "~", line}); i++; }
        else if (c == '&') { tokens.push_back({Token::AMP, "&", line}); i++; }
        else if (c == '|') { tokens.push_back({Token::PIPE, "|", line}); i++; }
        else if (c == '^') { tokens.push_back({Token::CARET, "^", line}); i++; }
        else if (c == '@') { tokens.push_back({Token::AT, "@", line}); i++; }
        else if (c == '#') { tokens.push_back({Token::HASH, "#", line}); i++; }
        else if (c == ':') { tokens.push_back({Token::COLON, ":", line}); i++; }
        else if (c == '?') { tokens.push_back({Token::QUESTION, "?", line}); i++; }
        else if (std::isdigit(c) || (c == '\'' && i+1 < src.size())) {
            std::string num;
            // Handle Verilog literals: 1'b0, 8'hFF, etc.
            while (i < src.size() && (std::isalnum(src[i]) || src[i]=='\'' || src[i]=='_'))
                num += src[i++];
            tokens.push_back({Token::NUMBER, num, line});
        }
        else if (std::isalpha(c) || c == '_' || c == '\\' || c == '$') {
            std::string ident;
            if (c == '\\') {
                i++;
                while (i < src.size() && !std::isspace(src[i])) ident += src[i++];
            } else {
                while (i < src.size() && (std::isalnum(src[i]) || src[i]=='_' || src[i]=='$'))
                    ident += src[i++];
            }
            // Check behavioral keywords
            if (ident == "always") tokens.push_back({Token::ALWAYS, ident, line});
            else if (ident == "if") tokens.push_back({Token::IF, ident, line});
            else if (ident == "else") tokens.push_back({Token::ELSE, ident, line});
            else if (ident == "begin") tokens.push_back({Token::BEGIN_KW, ident, line});
            else if (ident == "end") tokens.push_back({Token::END_KW, ident, line});
            else if (ident == "posedge") tokens.push_back({Token::POSEDGE, ident, line});
            else if (ident == "negedge") tokens.push_back({Token::NEGEDGE, ident, line});
            else tokens.push_back({Token::IDENT, ident, line});
        }
        else i++;
    }
    tokens.push_back({Token::END, "", line});
    return tokens;
}

size_t VerilogParser::parse_port_decl(const std::vector<Token>& t, size_t pos, Netlist& nl,
                                       VerilogParseResult& r, const std::string& dir) {
    pos++; // skip input/output/wire keyword
    // Handle optional [MSB:LSB] bus range
    // For now, treat as single-bit
    if (pos < t.size() && t[pos].type == Token::LBRACKET) {
        while (pos < t.size() && t[pos].type != Token::RBRACKET) pos++;
        pos++; // skip ]
    }

    while (pos < t.size() && t[pos].type != Token::SEMI) {
        if (t[pos].type == Token::IDENT) {
            NetId nid = get_or_create_net(nl, t[pos].value);
            if (dir == "input") { nl.mark_input(nid); r.num_inputs++; }
            else if (dir == "output") { nl.mark_output(nid); r.num_outputs++; }
            else { r.num_wires++; }
        }
        pos++;
    }
    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
    return pos;
}

size_t VerilogParser::parse_gate_inst(const std::vector<Token>& t, size_t pos, Netlist& nl,
                                       VerilogParseResult& r, GateType gtype) {
    pos++; // skip gate keyword
    // Optional instance name
    std::string inst_name;
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        inst_name = t[pos].value;
        pos++;
    }

    // Collect port connections ( out, in1, in2, ... )
    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        pos++;
        std::vector<std::string> ports;
        while (pos < t.size() && t[pos].type != Token::RPAREN) {
            if (t[pos].type == Token::IDENT)
                ports.push_back(t[pos].value);
            pos++;
        }
        if (pos < t.size()) pos++; // skip )
        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;

        // First port = output, rest = inputs
        if (!ports.empty()) {
            NetId out = get_or_create_net(nl, ports[0]);
            std::vector<NetId> ins;
            for (size_t i = 1; i < ports.size(); ++i)
                ins.push_back(get_or_create_net(nl, ports[i]));
            nl.add_gate(gtype, ins, out, inst_name);
            r.num_gates++;
        }
    }
    return pos;
}

size_t VerilogParser::parse_assign(const std::vector<Token>& t, size_t pos, Netlist& nl,
                                    VerilogParseResult& r) {
    pos++; // skip 'assign'
    // Simple: assign out = in; or assign out = a & b;
    std::string lhs;
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        lhs = t[pos].value; pos++;
    }
    if (pos < t.size() && t[pos].type == Token::ASSIGN) pos++;

    // Parse RHS expression (simplified: supports single ops)
    if (pos < t.size() && t[pos].type == Token::TILDE) {
        pos++;
        if (pos < t.size() && t[pos].type == Token::IDENT) {
            NetId out = get_or_create_net(nl, lhs);
            NetId in = get_or_create_net(nl, t[pos].value);
            nl.add_gate(GateType::NOT, {in}, out, "assign_" + lhs);
            r.num_gates++; pos++;
        }
    } else if (pos < t.size() && t[pos].type == Token::IDENT) {
        std::string op1 = t[pos].value; pos++;
        if (pos < t.size() && (t[pos].type == Token::AMP || t[pos].type == Token::PIPE ||
                                t[pos].type == Token::CARET || t[pos].type == Token::PLUS)) {
            Token::Type op = t[pos].type; pos++;
            if (pos < t.size() && t[pos].type == Token::IDENT) {
                std::string op2 = t[pos].value; pos++;
                NetId out = get_or_create_net(nl, lhs);
                NetId in1 = get_or_create_net(nl, op1);
                NetId in2 = get_or_create_net(nl, op2);
                GateType gt = (op == Token::AMP) ? GateType::AND :
                              (op == Token::PIPE) ? GateType::OR :
                              (op == Token::PLUS) ? GateType::XOR : GateType::XOR;
                nl.add_gate(gt, {in1, in2}, out, "assign_" + lhs);
                r.num_gates++;
            }
        } else {
            // Simple buffer: assign out = in;
            NetId out = get_or_create_net(nl, lhs);
            NetId in = get_or_create_net(nl, op1);
            nl.add_gate(GateType::BUF, {in}, out, "assign_" + lhs);
            r.num_gates++;
        }
    }

    // Skip to semicolon
    while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
    if (pos < t.size()) pos++;
    return pos;
}

size_t VerilogParser::parse_module(const std::vector<Token>& t, size_t pos, Netlist& nl,
                                    VerilogParseResult& r) {
    pos++; // skip 'module'
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        r.module_name = t[pos].value; pos++;
    }

    // Parse ANSI-style port list in parens
    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        pos++;
        std::string current_dir = "";
        while (pos < t.size() && t[pos].type != Token::RPAREN) {
            if (t[pos].type == Token::IDENT && (t[pos].value == "input" || t[pos].value == "output" || t[pos].value == "inout")) {
                current_dir = t[pos].value;
                pos++;
                // Skip [msb:lsb]
                if (pos < t.size() && t[pos].type == Token::LBRACKET) {
                    while (pos < t.size() && t[pos].type != Token::RBRACKET) pos++;
                    if (pos < t.size()) pos++;
                }
            } else if (t[pos].type == Token::IDENT && (t[pos].value == "wire" || t[pos].value == "reg" || t[pos].value == "logic")) {
                pos++;
            } else if (t[pos].type == Token::IDENT) {
                if (current_dir != "") {
                    NetId nid = get_or_create_net(nl, t[pos].value);
                    if (current_dir == "input") { nl.mark_input(nid); r.num_inputs++; }
                    else if (current_dir == "output" || current_dir == "inout") { nl.mark_output(nid); r.num_outputs++; }
                } else {
                    // Just names without direction (Verilog-1995), will be parsed in body
                    get_or_create_net(nl, t[pos].value);
                }
                pos++;
            } else {
                pos++;
            }
        }
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
    }
    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;

    // Parse module body
    while (pos < t.size() && !(t[pos].type == Token::IDENT && t[pos].value == "endmodule")) {
        if (t[pos].type != Token::IDENT && t[pos].type != Token::ALWAYS) { pos++; continue; }

        const std::string& kw = t[pos].value;
        if (kw == "input")    pos = parse_port_decl(t, pos, nl, r, "input");
        else if (kw == "output") pos = parse_port_decl(t, pos, nl, r, "output");
        else if (kw == "wire" || kw == "reg") pos = parse_port_decl(t, pos, nl, r, "wire");
        else if (kw == "assign") pos = parse_assign(t, pos, nl, r);
        else if (kw == "and")   pos = parse_gate_inst(t, pos, nl, r, GateType::AND);
        else if (kw == "or")    pos = parse_gate_inst(t, pos, nl, r, GateType::OR);
        else if (kw == "not" || kw == "inv") pos = parse_gate_inst(t, pos, nl, r, GateType::NOT);
        else if (kw == "nand")  pos = parse_gate_inst(t, pos, nl, r, GateType::NAND);
        else if (kw == "nor")   pos = parse_gate_inst(t, pos, nl, r, GateType::NOR);
        else if (kw == "xor")   pos = parse_gate_inst(t, pos, nl, r, GateType::XOR);
        else if (kw == "xnor")  pos = parse_gate_inst(t, pos, nl, r, GateType::XNOR);
        else if (kw == "buf")   pos = parse_gate_inst(t, pos, nl, r, GateType::BUF);
        else if (kw == "always") pos = parse_always(t, pos, ast.root);
        else pos++;
    }
    if (pos < t.size()) pos++; // skip endmodule
    return pos;
}

VerilogParseResult VerilogParser::parse_tokens(const std::vector<Token>& t, Netlist& nl) {
    VerilogParseResult r;
    r.success = true;
    net_map_.clear();

    size_t pos = 0;
    while (pos < t.size() && t[pos].type != Token::END) {
        if (t[pos].type == Token::IDENT && t[pos].value == "module") {
            pos = parse_module(t, pos, nl, r);
        } else pos++;
    }
    return r;
}

VerilogParseResult VerilogParser::parse_string(const std::string& src, Netlist& nl) {
    auto tokens = tokenize(src);
    return parse_tokens(tokens, nl);
}

VerilogParseResult VerilogParser::parse_file(const std::string& filename, Netlist& nl) {
    std::ifstream f(filename);
    if (!f.is_open()) return {false, "", 0, 0, 0, 0, 0, "Cannot open file: " + filename};
    std::string src((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return parse_string(src, nl);
}

std::string VerilogParser::to_verilog(const Netlist& nl, const std::string& module_name) {
    std::ostringstream v;
    v << "// Generated by SiliconForge\n";
    v << "module " << module_name << " (";

    // Port list
    bool first = true;
    for (auto pi : nl.primary_inputs()) {
        if (!first) v << ", "; first = false;
        v << nl.net(pi).name;
    }
    for (auto po : nl.primary_outputs()) {
        if (!first) v << ", "; first = false;
        v << nl.net(po).name;
    }
    v << ");\n\n";

    // Declarations
    for (auto pi : nl.primary_inputs()) v << "  input " << nl.net(pi).name << ";\n";
    for (auto po : nl.primary_outputs()) v << "  output " << nl.net(po).name << ";\n";

    // Wires
    for (size_t i = 0; i < nl.num_nets(); ++i) {
        bool is_io = false;
        for (auto pi : nl.primary_inputs()) if (pi == (NetId)i) is_io = true;
        for (auto po : nl.primary_outputs()) if (po == (NetId)i) is_io = true;
        if (!is_io) v << "  wire " << nl.net(i).name << ";\n";
    }
    v << "\n";

    // Gates
    for (size_t gid = 0; gid < nl.num_gates(); ++gid) {
        auto& g = nl.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
        if (g.type == GateType::DFF) {
            v << "  // DFF: " << g.name << "\n";
            continue;
        }
        std::string gt = gate_type_str(g.type);
        std::transform(gt.begin(), gt.end(), gt.begin(), ::tolower);
        v << "  " << gt << " " << g.name << " (";
        if (g.output >= 0) v << nl.net(g.output).name;
        for (auto ni : g.inputs) v << ", " << nl.net(ni).name;
        v << ");\n";
    }

    v << "\nendmodule\n";
    return v.str();
}

// ----------------------------------------------------
// Behavioral Parser Extensions
// ----------------------------------------------------

std::shared_ptr<AstNode> VerilogParser::parse_expression(const std::vector<Token>& t, size_t& pos) {
    auto expr = std::make_shared<AstNode>();
    
    // Simplistic expression parser for A OP B or just A
    std::string lhs = t[pos].value; pos++;
    
    if (pos < t.size() && (t[pos].type == Token::PLUS || t[pos].type == Token::MINUS || t[pos].type == Token::STAR ||
                           t[pos].type == Token::EQEQ || t[pos].type == Token::AMP || t[pos].type == Token::PIPE || 
                           t[pos].type == Token::CARET)) {
        expr->type = AstNodeType::BIN_OP;
        expr->value = t[pos].value;
        pos++;
        
        auto left = std::make_shared<AstNode>();
        left->type = AstNodeType::WIRE_DECL; // treating basic idents as wires for now
        left->value = lhs;
        expr->add(left);
        
        auto right = parse_expression(t, pos);
        expr->add(right);
    } else {
        expr->type = AstNodeType::WIRE_DECL;
        expr->value = lhs;
    }
    
    return expr;
}

size_t VerilogParser::parse_statement(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent) {
    if (t[pos].type == Token::IF) {
        auto if_node = std::make_shared<AstNode>();
        if_node->type = AstNodeType::IF_ELSE;
        pos++; // skip if
        if (t[pos].type == Token::LPAREN) pos++;
        if_node->add(parse_expression(t, pos));
        if (t[pos].type == Token::RPAREN) pos++;
        
        pos = parse_statement(t, pos, if_node);
        
        if (pos < t.size() && t[pos].type == Token::ELSE) {
            pos++; // skip else
            pos = parse_statement(t, pos, if_node);
        }
        parent->add(if_node);
        return pos;
    }
    else if (t[pos].type == Token::BEGIN_KW) {
        return parse_statement_block(t, pos, parent);
    }
    else if (t[pos].type == Token::IDENT) {
        // Assume non-blocking assign: a <= b;
        auto assign = std::make_shared<AstNode>();
        assign->type = AstNodeType::NONBLOCK_ASSIGN;
        assign->value = t[pos].value; pos++;
        
        if (t[pos].type == Token::LEQ || t[pos].type == Token::ASSIGN) pos++;
        
        assign->add(parse_expression(t, pos));
        
        if (t[pos].type == Token::SEMI) pos++;
        parent->add(assign);
        return pos;
    }
    
    pos++; // fallback skip
    return pos;
}

size_t VerilogParser::parse_statement_block(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent) {
    auto block = std::make_shared<AstNode>();
    block->type = AstNodeType::BLOCK_BEGIN_END;
    pos++; // skip begin
    
    while (pos < t.size() && t[pos].type != Token::END_KW) {
        pos = parse_statement(t, pos, block);
    }
    if (pos < t.size() && t[pos].type == Token::END_KW) pos++;
    
    parent->add(block);
    return pos;
}

size_t VerilogParser::parse_always(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent) {
    auto always = std::make_shared<AstNode>();
    pos++; // skip always
    
    // Parse sensitivity list @(posedge clk)
    if (t[pos].type == Token::AT) pos++;
    if (t[pos].type == Token::LPAREN) pos++;
    
    if (t[pos].type == Token::POSEDGE) {
        always->type = AstNodeType::ALWAYS_POS_CLK;
        pos++;
        if (t[pos].type == Token::IDENT) { always->value = t[pos].value; pos++; }
    }
    
    while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
    if (t[pos].type == Token::RPAREN) pos++;
    
    // Parse the body
    pos = parse_statement(t, pos, always);
    
    parent->add(always);
    has_behavioral_blocks = true;
    return pos;
}

} // namespace sf
