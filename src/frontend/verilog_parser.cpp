// SiliconForge — Complete Verilog Parser Implementation
// Supports: all operators, always @(*), for loops, parameters, concatenation, ternary
#include "frontend/verilog_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cassert>

namespace sf {

// Forward declarations
static uint64_t parse_verilog_number(const std::string& s);

// ============================================================
// Utility helpers
// ============================================================

NetId VerilogParser::get_or_create_net(Netlist& nl, const std::string& name) {
    auto it = net_map_.find(name);
    if (it != net_map_.end()) return it->second;
    NetId id = nl.add_net(name);
    net_map_[name] = id;
    return id;
}

std::pair<int,int> VerilogParser::parse_bus_range(const std::vector<Token>& t, size_t& pos) {
    if (pos >= t.size() || t[pos].type != Token::LBRACKET) return {-1, -1};
    pos++;
    int msb = eval_const_expr(t, pos);
    if (pos < t.size() && t[pos].type == Token::COLON) {
        pos++;
        int lsb = eval_const_expr(t, pos);
        if (pos < t.size() && t[pos].type == Token::RBRACKET) pos++;
        return {msb, lsb};
    }
    if (pos < t.size() && t[pos].type == Token::RBRACKET) pos++;
    return {msb, msb};
}

// Evaluate compile-time constant expression (for bus ranges, parameters)
// Handles: numbers, parameter names, +, -, *, /, %
int VerilogParser::eval_const_expr(const std::vector<Token>& t, size_t& pos) {
    // Simple recursive descent: additive > multiplicative > primary
    auto eval_primary = [&]() -> int {
        if (pos >= t.size()) return 0;
        if (t[pos].type == Token::NUMBER) {
            int v = (int)parse_verilog_number(t[pos].value); pos++; return v;
        }
        if (t[pos].type == Token::IDENT) {
            std::string name = t[pos].value;
            // $clog2(expr) — ceiling log base 2
            if (name == "$clog2" && pos+1 < t.size() && t[pos+1].type == Token::LPAREN) {
                pos += 2; // skip $clog2(
                int arg = eval_const_expr(t, pos);
                if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
                if (arg <= 1) return (arg <= 0) ? 0 : 1;
                int result = 0; arg--;
                while (arg > 0) { result++; arg >>= 1; }
                return result;
            }
            // $bits(signal) — return bit width
            if (name == "$bits" && pos+1 < t.size() && t[pos+1].type == Token::LPAREN) {
                pos += 2; // skip $bits(
                if (pos < t.size() && t[pos].type == Token::IDENT) {
                    std::string sig = t[pos].value; pos++;
                    if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
                    auto it = bus_ranges_.find(sig);
                    if (it != bus_ranges_.end())
                        return std::abs(it->second.first - it->second.second) + 1;
                    return 1;
                }
                if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
                return 1;
            }
            auto it = params_.find(name);
            int v = (it != params_.end()) ? it->second : 0;
            pos++; return v;
        }
        if (t[pos].type == Token::LPAREN) {
            pos++;
            int v = eval_const_expr(t, pos);
            if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
            return v;
        }
        if (t[pos].type == Token::MINUS) {
            pos++; return -eval_const_expr(t, pos);
        }
        return 0;
    };
    auto eval_mult = [&]() -> int {
        int left = eval_primary();
        while (pos < t.size()) {
            if (t[pos].type == Token::STAR) { pos++; left *= eval_primary(); }
            else if (t[pos].type == Token::SLASH) { pos++; int r = eval_primary(); left = r ? left/r : 0; }
            else if (t[pos].type == Token::PERCENT) { pos++; int r = eval_primary(); left = r ? left%r : 0; }
            else if (t[pos].type == Token::POWER_OP) {
                pos++; int exp = eval_primary();
                int result = 1; for (int i = 0; i < exp; i++) result *= left;
                left = result;
            }
            else break;
        }
        return left;
    };
    int left = eval_mult();
    while (pos < t.size()) {
        if (t[pos].type == Token::PLUS) { pos++; left += eval_mult(); }
        else if (t[pos].type == Token::MINUS) { pos++; left -= eval_mult(); }
        else break;
    }
    return left;
}

std::vector<NetId> VerilogParser::get_or_create_bus(Netlist& nl, const std::string& name, int msb, int lsb) {
    std::vector<NetId> bits;
    int lo = std::min(msb, lsb);
    int hi = std::max(msb, lsb);
    for (int i = lo; i <= hi; i++) {
        std::string bit_name = name + "[" + std::to_string(i) + "]";
        bits.push_back(get_or_create_net(nl, bit_name));
    }
    bus_ranges_[name] = {msb, lsb};
    return bits;
}

int VerilogParser::bus_width(const std::string& name) const {
    auto it = bus_ranges_.find(name);
    if (it == bus_ranges_.end()) return 1;
    return std::abs(it->second.first - it->second.second) + 1;
}

static uint64_t parse_verilog_number(const std::string& s) {
    auto tick = s.find('\'');
    if (tick != std::string::npos) {
        char base = (tick + 1 < s.size()) ? s[tick + 1] : 'd';
        std::string digits = s.substr(tick + 2);
        digits.erase(std::remove(digits.begin(), digits.end(), '_'), digits.end());
        if (digits.empty()) return 0;
        // Replace x/X/z/Z/? with 0 for numeric conversion (don't-care → 0)
        for (auto& c : digits) {
            if (c == 'x' || c == 'X' || c == 'z' || c == 'Z' || c == '?') c = '0';
        }
        try {
            if (base == 'h' || base == 'H') return std::stoull(digits, nullptr, 16);
            if (base == 'b' || base == 'B') return std::stoull(digits, nullptr, 2);
            if (base == 'o' || base == 'O') return std::stoull(digits, nullptr, 8);
            return std::stoull(digits, nullptr, 10);
        } catch (...) { return 0; }
    }
    if (s.empty()) return 0;
    try { return std::stoull(s, nullptr, 10); } catch (...) { return 0; }
}


// ============================================================
// TOKENIZER — full Verilog operator set
// ============================================================

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
            if (i+1 < src.size()) i += 2;
            continue;
        }

        // Two-character and three-character operators first
        if (c == '=' && i+2 < src.size() && src[i+1] == '=' && src[i+2] == '=') {
            tokens.push_back({Token::CASE_EQ, "===", line}); i += 3;
        } else if (c == '!' && i+2 < src.size() && src[i+1] == '=' && src[i+2] == '=') {
            tokens.push_back({Token::CASE_NEQ, "!==", line}); i += 3;
        } else if (c == '*' && i+1 < src.size() && src[i+1] == '*') {
            tokens.push_back({Token::POWER_OP, "**", line}); i += 2;
        } else if (c == '~' && i+1 < src.size() && src[i+1] == '^') {
            tokens.push_back({Token::XNOR_OP, "~^", line}); i += 2;
        } else if (c == '^' && i+1 < src.size() && src[i+1] == '~') {
            tokens.push_back({Token::XNOR_OP, "^~", line}); i += 2;
        } else if (c == '+' && i+1 < src.size() && src[i+1] == ':') {
            tokens.push_back({Token::PLUS_COLON, "+:", line}); i += 2;
        } else if (c == '-' && i+1 < src.size() && src[i+1] == ':') {
            tokens.push_back({Token::MINUS_COLON, "-:", line}); i += 2;
        } else if (c == '<' && i+1 < src.size() && src[i+1] == '<') {
            tokens.push_back({Token::LSHIFT, "<<", line}); i += 2;
        } else if (c == '>' && i+2 < src.size() && src[i+1] == '>' && src[i+2] == '>') {
            tokens.push_back({Token::ARSHIFT, ">>>", line}); i += 3;
        } else if (c == '>' && i+1 < src.size() && src[i+1] == '>') {
            tokens.push_back({Token::RSHIFT, ">>", line}); i += 2;
        } else if (c == '<' && i+1 < src.size() && src[i+1] == '=') {
            tokens.push_back({Token::LEQ, "<=", line}); i += 2;
        } else if (c == '>' && i+1 < src.size() && src[i+1] == '=') {
            tokens.push_back({Token::GTE, ">=", line}); i += 2;
        } else if (c == '=' && i+1 < src.size() && src[i+1] == '=') {
            tokens.push_back({Token::EQEQ, "==", line}); i += 2;
        } else if (c == '!' && i+1 < src.size() && src[i+1] == '=') {
            tokens.push_back({Token::NEQ, "!=", line}); i += 2;
        } else if (c == '&' && i+1 < src.size() && src[i+1] == '&') {
            tokens.push_back({Token::LAND, "&&", line}); i += 2;
        } else if (c == '|' && i+1 < src.size() && src[i+1] == '|') {
            tokens.push_back({Token::LOR, "||", line}); i += 2;
        }
        // Single-character operators
        else if (c == '(') { tokens.push_back({Token::LPAREN, "(", line}); i++; }
        else if (c == ')') { tokens.push_back({Token::RPAREN, ")", line}); i++; }
        else if (c == '[') { tokens.push_back({Token::LBRACKET, "[", line}); i++; }
        else if (c == ']') { tokens.push_back({Token::RBRACKET, "]", line}); i++; }
        else if (c == '{') { tokens.push_back({Token::LBRACE, "{", line}); i++; }
        else if (c == '}') { tokens.push_back({Token::RBRACE, "}", line}); i++; }
        else if (c == ',') { tokens.push_back({Token::COMMA, ",", line}); i++; }
        else if (c == ';') { tokens.push_back({Token::SEMI, ";", line}); i++; }
        else if (c == '.') { tokens.push_back({Token::DOT, ".", line}); i++; }
        else if (c == '=') { tokens.push_back({Token::ASSIGN, "=", line}); i++; }
        else if (c == '!') { tokens.push_back({Token::BANG, "!", line}); i++; }
        else if (c == '<') { tokens.push_back({Token::LT, "<", line}); i++; }
        else if (c == '>') { tokens.push_back({Token::GT, ">", line}); i++; }
        else if (c == '+') { tokens.push_back({Token::PLUS, "+", line}); i++; }
        else if (c == '-') { tokens.push_back({Token::MINUS, "-", line}); i++; }
        else if (c == '*') { tokens.push_back({Token::STAR, "*", line}); i++; }
        else if (c == '/') { tokens.push_back({Token::SLASH, "/", line}); i++; }
        else if (c == '%') { tokens.push_back({Token::PERCENT, "%", line}); i++; }
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
            if (ident == "always") tokens.push_back({Token::ALWAYS, ident, line});
            else if (ident == "if") tokens.push_back({Token::IF, ident, line});
            else if (ident == "else") tokens.push_back({Token::ELSE, ident, line});
            else if (ident == "begin") tokens.push_back({Token::BEGIN_KW, ident, line});
            else if (ident == "end") tokens.push_back({Token::END_KW, ident, line});
            else if (ident == "posedge") tokens.push_back({Token::POSEDGE, ident, line});
            else if (ident == "negedge") tokens.push_back({Token::NEGEDGE, ident, line});
            else if (ident == "case" || ident == "casex" || ident == "casez")
                tokens.push_back({Token::CASE_KW, ident, line});
            else if (ident == "endcase") tokens.push_back({Token::ENDCASE_KW, ident, line});
            else if (ident == "default") tokens.push_back({Token::DEFAULT_KW, ident, line});
            else if (ident == "for") tokens.push_back({Token::FOR_KW, ident, line});
            else if (ident == "parameter" || ident == "localparam")
                tokens.push_back({Token::PARAMETER_KW, ident, line});
            else if (ident == "signed") tokens.push_back({Token::SIGNED_KW, ident, line});
            else if (ident == "integer") tokens.push_back({Token::INTEGER_KW, ident, line});
            else if (ident == "generate") tokens.push_back({Token::GENERATE_KW, ident, line});
            else if (ident == "endgenerate") tokens.push_back({Token::ENDGENERATE_KW, ident, line});
            else if (ident == "genvar") tokens.push_back({Token::GENVAR_KW, ident, line});
            else if (ident == "function") tokens.push_back({Token::FUNCTION_KW, ident, line});
            else if (ident == "endfunction") tokens.push_back({Token::ENDFUNCTION_KW, ident, line});
            else if (ident == "task") tokens.push_back({Token::TASK_KW, ident, line});
            else if (ident == "endtask") tokens.push_back({Token::ENDTASK_KW, ident, line});
            else if (ident == "while") tokens.push_back({Token::WHILE_KW, ident, line});
            else if (ident == "repeat") tokens.push_back({Token::REPEAT_KW, ident, line});
            else if (ident == "forever") tokens.push_back({Token::FOREVER_KW, ident, line});
            else if (ident == "initial") tokens.push_back({Token::INITIAL_KW, ident, line});
            else if (ident == "specify") tokens.push_back({Token::SPECIFY_KW, ident, line});
            else if (ident == "endspecify") tokens.push_back({Token::ENDSPECIFY_KW, ident, line});
            else if (ident == "disable") tokens.push_back({Token::DISABLE_KW, ident, line});
            else if (ident == "defparam") tokens.push_back({Token::DEFPARAM_KW, ident, line});
            else if (ident == "automatic") tokens.push_back({Token::AUTOMATIC_KW, ident, line});
            else tokens.push_back({Token::IDENT, ident, line});
        }
        else i++;
    }
    tokens.push_back({Token::END, "", line});
    return tokens;
}


// ============================================================
// EXPRESSION PARSER — Recursive descent with Verilog precedence
// Precedence (low to high):
//   ternary ?: > || > && > | > ^ > & > == != > < > <= >= > << >> > + - > * / % > unary
// ============================================================

std::shared_ptr<AstNode> VerilogParser::parse_expression(const std::vector<Token>& t, size_t& pos) {
    return parse_ternary(t, pos);
}

std::shared_ptr<AstNode> VerilogParser::parse_ternary(const std::vector<Token>& t, size_t& pos) {
    auto cond = parse_logor(t, pos);
    if (pos < t.size() && t[pos].type == Token::QUESTION) {
        pos++; // skip ?
        auto true_expr = parse_expression(t, pos);
        if (pos < t.size() && t[pos].type == Token::COLON) pos++; // skip :
        auto false_expr = parse_expression(t, pos);
        auto node = AstNode::make(AstNodeType::TERNARY_OP, "?:");
        node->add(cond);
        node->add(true_expr);
        node->add(false_expr);
        return node;
    }
    return cond;
}

std::shared_ptr<AstNode> VerilogParser::parse_logor(const std::vector<Token>& t, size_t& pos) {
    auto left = parse_logand(t, pos);
    while (pos < t.size() && t[pos].type == Token::LOR) {
        pos++;
        auto right = parse_logand(t, pos);
        auto node = AstNode::make(AstNodeType::BIN_OP, "||");
        node->add(left); node->add(right);
        left = node;
    }
    return left;
}

std::shared_ptr<AstNode> VerilogParser::parse_logand(const std::vector<Token>& t, size_t& pos) {
    auto left = parse_bitor(t, pos);
    while (pos < t.size() && t[pos].type == Token::LAND) {
        pos++;
        auto right = parse_bitor(t, pos);
        auto node = AstNode::make(AstNodeType::BIN_OP, "&&");
        node->add(left); node->add(right);
        left = node;
    }
    return left;
}

std::shared_ptr<AstNode> VerilogParser::parse_bitor(const std::vector<Token>& t, size_t& pos) {
    auto left = parse_bitxor(t, pos);
    while (pos < t.size() && t[pos].type == Token::PIPE) {
        pos++;
        auto right = parse_bitxor(t, pos);
        auto node = AstNode::make(AstNodeType::BIN_OP, "|");
        node->add(left); node->add(right);
        left = node;
    }
    return left;
}

std::shared_ptr<AstNode> VerilogParser::parse_bitxor(const std::vector<Token>& t, size_t& pos) {
    auto left = parse_bitand(t, pos);
    while (pos < t.size() && (t[pos].type == Token::CARET || t[pos].type == Token::XNOR_OP)) {
        std::string op = (t[pos].type == Token::XNOR_OP) ? "~^" : "^";
        pos++;
        auto right = parse_bitand(t, pos);
        auto node = AstNode::make(AstNodeType::BIN_OP, op);
        node->add(left); node->add(right);
        left = node;
    }
    return left;
}

std::shared_ptr<AstNode> VerilogParser::parse_bitand(const std::vector<Token>& t, size_t& pos) {
    auto left = parse_equality(t, pos);
    while (pos < t.size() && t[pos].type == Token::AMP) {
        pos++;
        auto right = parse_equality(t, pos);
        auto node = AstNode::make(AstNodeType::BIN_OP, "&");
        node->add(left); node->add(right);
        left = node;
    }
    return left;
}

std::shared_ptr<AstNode> VerilogParser::parse_equality(const std::vector<Token>& t, size_t& pos) {
    auto left = parse_relational(t, pos);
    while (pos < t.size() && (t[pos].type == Token::EQEQ || t[pos].type == Token::NEQ ||
                               t[pos].type == Token::CASE_EQ || t[pos].type == Token::CASE_NEQ)) {
        // Map === to == and !== to != for synthesis (4-value irrelevant in gate-level)
        std::string op;
        if (t[pos].type == Token::CASE_EQ) op = "==";
        else if (t[pos].type == Token::CASE_NEQ) op = "!=";
        else op = t[pos].value;
        pos++;
        auto right = parse_relational(t, pos);
        auto node = AstNode::make(AstNodeType::BIN_OP, op);
        node->add(left); node->add(right);
        left = node;
    }
    return left;
}

std::shared_ptr<AstNode> VerilogParser::parse_relational(const std::vector<Token>& t, size_t& pos) {
    auto left = parse_shift(t, pos);
    while (pos < t.size() && (t[pos].type == Token::LT || t[pos].type == Token::GT ||
                               t[pos].type == Token::LEQ || t[pos].type == Token::GTE)) {
        std::string op = t[pos].value; pos++;
        auto right = parse_shift(t, pos);
        auto node = AstNode::make(AstNodeType::BIN_OP, op);
        node->add(left); node->add(right);
        left = node;
    }
    return left;
}

std::shared_ptr<AstNode> VerilogParser::parse_shift(const std::vector<Token>& t, size_t& pos) {
    auto left = parse_additive(t, pos);
    while (pos < t.size() && (t[pos].type == Token::LSHIFT || t[pos].type == Token::RSHIFT || t[pos].type == Token::ARSHIFT)) {
        std::string op = t[pos].value; pos++;
        auto right = parse_additive(t, pos);
        auto node = AstNode::make(AstNodeType::BIN_OP, op);
        node->add(left); node->add(right);
        left = node;
    }
    return left;
}

std::shared_ptr<AstNode> VerilogParser::parse_additive(const std::vector<Token>& t, size_t& pos) {
    auto left = parse_multiplicative(t, pos);
    while (pos < t.size() && (t[pos].type == Token::PLUS || t[pos].type == Token::MINUS)) {
        std::string op = t[pos].value; pos++;
        auto right = parse_multiplicative(t, pos);
        auto node = AstNode::make(AstNodeType::BIN_OP, op);
        node->add(left); node->add(right);
        left = node;
    }
    return left;
}

std::shared_ptr<AstNode> VerilogParser::parse_multiplicative(const std::vector<Token>& t, size_t& pos) {
    auto left = parse_unary(t, pos);
    while (pos < t.size() && (t[pos].type == Token::STAR || t[pos].type == Token::SLASH ||
                               t[pos].type == Token::PERCENT || t[pos].type == Token::POWER_OP)) {
        std::string op = t[pos].value; pos++;
        auto right = parse_unary(t, pos);
        auto node = AstNode::make(AstNodeType::BIN_OP, op);
        node->add(left); node->add(right);
        left = node;
    }
    return left;
}

std::shared_ptr<AstNode> VerilogParser::parse_unary(const std::vector<Token>& t, size_t& pos) {
    if (pos >= t.size()) return AstNode::make(AstNodeType::NUMBER_LITERAL, "0");

    // Bitwise NOT ~ (check for reduction ~&, ~|, ~^ first)
    if (t[pos].type == Token::TILDE) {
        // Check for reduction NAND ~&, NOR ~|, XNOR ~^
        if (pos + 1 < t.size() &&
            (t[pos+1].type == Token::AMP || t[pos+1].type == Token::PIPE || t[pos+1].type == Token::CARET)) {
            std::string red_op = t[pos+1].value; // &, |, ^
            pos += 2; // skip ~ and &/|/^
            auto operand = parse_unary(t, pos);
            auto node = AstNode::make(AstNodeType::UNARY_OP, "red_n" + red_op); // red_n&, red_n|, red_n^
            node->add(operand);
            return node;
        }
        pos++;
        auto operand = parse_unary(t, pos);
        auto node = AstNode::make(AstNodeType::UNARY_OP, "~");
        node->add(operand);
        return node;
    }
    // XNOR_OP (~^ as single token) used as reduction
    if (t[pos].type == Token::XNOR_OP &&
        pos + 1 < t.size() &&
        (t[pos+1].type == Token::IDENT || t[pos+1].type == Token::LPAREN || t[pos+1].type == Token::NUMBER)) {
        pos++;
        auto operand = parse_unary(t, pos);
        auto node = AstNode::make(AstNodeType::UNARY_OP, "red_n^");
        node->add(operand);
        return node;
    }
    // Logical NOT !
    if (t[pos].type == Token::BANG) {
        pos++;
        auto operand = parse_unary(t, pos);
        auto node = AstNode::make(AstNodeType::UNARY_OP, "!");
        node->add(operand);
        return node;
    }
    // Unary minus -
    if (t[pos].type == Token::MINUS) {
        pos++;
        auto operand = parse_unary(t, pos);
        auto node = AstNode::make(AstNodeType::UNARY_OP, "-");
        node->add(operand);
        return node;
    }
    // Unary plus + (identity, no-op)
    if (t[pos].type == Token::PLUS) {
        pos++;
        return parse_unary(t, pos); // just return operand, no wrapper node
    }
    // Reduction operators: &x, |x, ^x (prefix form before identifier/parens)
    if ((t[pos].type == Token::AMP || t[pos].type == Token::PIPE || t[pos].type == Token::CARET) &&
        pos + 1 < t.size() &&
        (t[pos+1].type == Token::IDENT || t[pos+1].type == Token::LPAREN || t[pos+1].type == Token::NUMBER)) {
        std::string op = t[pos].value;
        pos++;
        auto operand = parse_unary(t, pos);
        auto node = AstNode::make(AstNodeType::UNARY_OP, "red_" + op);
        node->add(operand);
        return node;
    }

    return parse_primary(t, pos);
}

std::shared_ptr<AstNode> VerilogParser::parse_primary(const std::vector<Token>& t, size_t& pos) {
    if (pos >= t.size()) return AstNode::make(AstNodeType::NUMBER_LITERAL, "0");

    // Parenthesized expression
    if (t[pos].type == Token::LPAREN) {
        pos++; // skip (
        auto expr = parse_expression(t, pos);
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
        return expr;
    }

    // Concatenation { expr, expr, ... }
    if (t[pos].type == Token::LBRACE) {
        pos++; // skip {
        // Check for replication: {N{expr}}
        if (pos + 2 < t.size() && t[pos].type == Token::NUMBER && t[pos+1].type == Token::LBRACE) {
            int count = std::stoi(t[pos].value);
            pos++; // skip N
            pos++; // skip {
            auto expr = parse_expression(t, pos);
            if (pos < t.size() && t[pos].type == Token::RBRACE) pos++;
            if (pos < t.size() && t[pos].type == Token::RBRACE) pos++;
            auto node = AstNode::make(AstNodeType::REPLICATE, std::to_string(count));
            node->add(expr);
            return node;
        }
        // Regular concatenation
        auto node = AstNode::make(AstNodeType::CONCAT, "{}");
        while (pos < t.size() && t[pos].type != Token::RBRACE) {
            node->add(parse_expression(t, pos));
            if (pos < t.size() && t[pos].type == Token::COMMA) pos++;
        }
        if (pos < t.size() && t[pos].type == Token::RBRACE) pos++;
        return node;
    }

    // Number literal
    if (t[pos].type == Token::NUMBER) {
        auto node = AstNode::make(AstNodeType::NUMBER_LITERAL, t[pos].value);
        node->int_val = (int)parse_verilog_number(t[pos].value);
        pos++;
        return node;
    }

    // Identifier (possibly with bit-select, function call, or $signed/$unsigned)
    if (t[pos].type == Token::IDENT) {
        std::string name = t[pos].value;
        pos++;

        // $signed(expr) / $unsigned(expr) — system functions
        if ((name == "$signed" || name == "$unsigned") && pos < t.size() && t[pos].type == Token::LPAREN) {
            pos++; // skip (
            auto inner = parse_expression(t, pos);
            if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
            if (name == "$signed") {
                auto node = AstNode::make(AstNodeType::UNARY_OP, "signed");
                node->add(inner);
                return node;
            }
            return inner; // $unsigned just returns the expression
        }

        // $clog2(expr) — compile-time ceiling log2
        if (name == "$clog2" && pos < t.size() && t[pos].type == Token::LPAREN) {
            pos++; // skip (
            auto inner = parse_expression(t, pos);
            if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
            int val = inner->int_val;
            if (val <= 1) val = (val <= 0) ? 0 : 1;
            else { int r = 0; val--; while (val > 0) { r++; val >>= 1; } val = r; }
            auto node = AstNode::make(AstNodeType::NUMBER_LITERAL, std::to_string(val));
            node->int_val = val;
            return node;
        }

        // $bits(signal) — bit width query
        if (name == "$bits" && pos < t.size() && t[pos].type == Token::LPAREN) {
            pos++; // skip (
            int w = 1;
            if (pos < t.size() && t[pos].type == Token::IDENT) {
                auto it = bus_ranges_.find(t[pos].value);
                if (it != bus_ranges_.end())
                    w = std::abs(it->second.first - it->second.second) + 1;
                pos++;
            }
            if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
            auto node = AstNode::make(AstNodeType::NUMBER_LITERAL, std::to_string(w));
            node->int_val = w;
            return node;
        }

        // System tasks: $display, $finish, $stop, $write, $monitor, $readmemh, $readmemb, etc.
        // Skip call including parenthesized arguments
        if (name.size() > 1 && name[0] == '$') {
            if (pos < t.size() && t[pos].type == Token::LPAREN) {
                int depth = 1; pos++;
                while (pos < t.size() && depth > 0) {
                    if (t[pos].type == Token::LPAREN) depth++;
                    if (t[pos].type == Token::RPAREN) depth--;
                    pos++;
                }
            }
            // Return a zero — system tasks have no synthesis value
            auto node = AstNode::make(AstNodeType::NUMBER_LITERAL, "0");
            node->int_val = 0;
            return node;
        }

        // Function call: name(args)
        if (pos < t.size() && t[pos].type == Token::LPAREN && functions_.count(name)) {
            pos++; // skip (
            auto& func = functions_[name];
            // Collect arguments
            std::vector<std::shared_ptr<AstNode>> args;
            while (pos < t.size() && t[pos].type != Token::RPAREN) {
                args.push_back(parse_expression(t, pos));
                if (pos < t.size() && t[pos].type == Token::COMMA) pos++;
            }
            if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
            // Inline: substitute params and parse body tokens
            auto saved_params = params_;
            for (size_t ai = 0; ai < std::min(args.size(), func.param_names.size()); ai++) {
                if (args[ai]->type == AstNodeType::NUMBER_LITERAL)
                    params_[func.param_names[ai]] = args[ai]->int_val;
            }
            // Parse function body to get the expression
            if (!func.body_tokens.empty()) {
                size_t fpos = 0;
                auto result = parse_expression(func.body_tokens, fpos);
                params_ = saved_params;
                return result;
            }
            params_ = saved_params;
            return AstNode::make(AstNodeType::WIRE_DECL, name);
        }

        // Check for memory array read: mem[addr_expr]
        if (pos < t.size() && t[pos].type == Token::LBRACKET) {
            auto mit = memory_arrays_.find(name);
            if (mit != memory_arrays_.end()) {
                // Memory array reference — parse index as expression
                pos++; // skip [
                auto addr_expr = parse_expression(t, pos);
                if (pos < t.size() && t[pos].type == Token::RBRACKET) pos++;
                auto node = AstNode::make(AstNodeType::MEM_READ, name);
                node->add(addr_expr);
                return node;
            }

            // Normal bit-select [idx], part-select [msb:lsb], or indexed part-select [base +: width] / [base -: width]
            size_t saved = pos;

            // Try indexed part-select first: peek for +: or -:
            // Parse: [ expr +: const_width ] or [ expr -: const_width ]
            {
                size_t peek = pos + 1; // after [
                // Scan for +: or -: before ]
                int bracket_depth = 1;
                bool found_plus_colon = false, found_minus_colon = false;
                size_t op_pos = 0;
                for (size_t p = peek; p < t.size() && bracket_depth > 0; p++) {
                    if (t[p].type == Token::LBRACKET) bracket_depth++;
                    else if (t[p].type == Token::RBRACKET) bracket_depth--;
                    else if (t[p].type == Token::PLUS_COLON && bracket_depth == 1) { found_plus_colon = true; op_pos = p; break; }
                    else if (t[p].type == Token::MINUS_COLON && bracket_depth == 1) { found_minus_colon = true; op_pos = p; break; }
                }
                if (found_plus_colon || found_minus_colon) {
                    pos++; // skip [
                    auto base_expr = parse_expression(t, pos);
                    pos++; // skip +: or -:
                    // Width must be a constant
                    int width = 1;
                    if (pos < t.size() && t[pos].type == Token::NUMBER) {
                        width = (int)parse_verilog_number(t[pos].value); pos++;
                    } else if (pos < t.size() && t[pos].type == Token::IDENT) {
                        auto pit2 = params_.find(t[pos].value);
                        if (pit2 != params_.end()) width = pit2->second;
                        pos++;
                    }
                    if (pos < t.size() && t[pos].type == Token::RBRACKET) pos++;

                    // Evaluate base if it's a constant
                    int base_val = -1;
                    if (base_expr->type == AstNodeType::NUMBER_LITERAL) {
                        try { base_val = std::stoi(base_expr->value); } catch (...) {}
                    }

                    if (base_val >= 0) {
                        // Constant base: expand to concatenation of individual bits
                        auto node = AstNode::make(AstNodeType::CONCAT, "{}");
                        int lo, hi;
                        if (found_plus_colon) { lo = base_val; hi = base_val + width - 1; }
                        else { hi = base_val; lo = base_val - width + 1; if (lo < 0) lo = 0; }
                        for (int b = lo; b <= hi; b++) {
                            node->add(AstNode::make(AstNodeType::WIRE_DECL, name + "[" + std::to_string(b) + "]"));
                        }
                        return node;
                    } else {
                        // Variable base: generate MUX-based selection at synthesis time
                        // For now, create a special CONCAT with MEM_READ-like pattern
                        // We model data[i*8 +: 8] as: select 8 bits starting at base
                        // This creates a shifted extraction: (signal >> base)[width-1:0]
                        auto full_signal = AstNode::make(AstNodeType::WIRE_DECL, name);
                        auto shift_node = AstNode::make(AstNodeType::BIN_OP, found_plus_colon ? ">>" : ">>");
                        if (found_minus_colon) {
                            // base -: width → bits [base : base-width+1], same as >> (base-width+1)
                            auto offset = AstNode::make(AstNodeType::BIN_OP, "-");
                            offset->add(base_expr);
                            auto wm1 = AstNode::make(AstNodeType::NUMBER_LITERAL, std::to_string(width - 1));
                            wm1->int_val = width - 1;
                            offset->add(wm1);
                            shift_node->add(full_signal);
                            shift_node->add(offset);
                        } else {
                            shift_node->add(full_signal);
                            shift_node->add(base_expr);
                        }
                        // Mask to width bits — synthesizer handles via target_w
                        return shift_node;
                    }
                }
            }

            auto sel = parse_bus_range(t, pos);
            if (sel.first >= 0 && sel.first == sel.second) {
                name = name + "[" + std::to_string(sel.first) + "]";
            } else if (sel.first >= 0) {
                // Part-select — return concatenation of individual bits
                auto node = AstNode::make(AstNodeType::CONCAT, "{}");
                int lo = std::min(sel.first, sel.second);
                int hi = std::max(sel.first, sel.second);
                for (int b = lo; b <= hi; b++) {
                    node->add(AstNode::make(AstNodeType::WIRE_DECL, name + "[" + std::to_string(b) + "]"));
                }
                return node;
            }
        }
        // Substitute parameters
        auto pit = params_.find(name);
        if (pit != params_.end()) {
            auto node = AstNode::make(AstNodeType::NUMBER_LITERAL, std::to_string(pit->second));
            node->int_val = pit->second;
            return node;
        }
        return AstNode::make(AstNodeType::WIRE_DECL, name);
    }

    // Fallback
    pos++;
    return AstNode::make(AstNodeType::NUMBER_LITERAL, "0");
}


// ============================================================
// PORT / WIRE / GATE declarations (mostly preserved from original)
// ============================================================

size_t VerilogParser::parse_port_decl(const std::vector<Token>& t, size_t pos, Netlist& nl,
                                       VerilogParseResult& r, const std::string& dir) {
    pos++; // skip keyword
    // skip optional signed keyword
    if (pos < t.size() && t[pos].type == Token::SIGNED_KW) pos++;
    auto range = parse_bus_range(t, pos);

    while (pos < t.size() && t[pos].type != Token::SEMI) {
        if (t[pos].type == Token::IDENT) {
            std::string name = t[pos].value;
            if (range.first >= 0 && range.first != range.second) {
                auto bits = get_or_create_bus(nl, name, range.first, range.second);
                for (auto nid : bits) {
                    if (dir == "input") { nl.mark_input(nid); r.num_inputs++; }
                    else if (dir == "output") { nl.mark_output(nid); r.num_outputs++; }
                    else { r.num_wires++; }
                }
            } else {
                NetId nid = get_or_create_net(nl, name);
                if (dir == "input") { nl.mark_input(nid); r.num_inputs++; }
                else if (dir == "output") { nl.mark_output(nid); r.num_outputs++; }
                else { r.num_wires++; }
            }
        }
        pos++;
    }
    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
    return pos;
}

size_t VerilogParser::parse_gate_inst(const std::vector<Token>& t, size_t pos, Netlist& nl,
                                       VerilogParseResult& r, GateType gtype) {
    pos++; // skip gate keyword
    std::string inst_name;
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        inst_name = t[pos].value; pos++;
    }
    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        pos++;
        std::vector<std::string> ports;
        while (pos < t.size() && t[pos].type != Token::RPAREN) {
            if (t[pos].type == Token::IDENT) ports.push_back(t[pos].value);
            pos++;
        }
        if (pos < t.size()) pos++;
        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
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

// ============================================================
// PARAMETER PARSING
// ============================================================

size_t VerilogParser::parse_parameter(const std::vector<Token>& t, size_t pos) {
    pos++; // skip parameter/localparam
    // Skip optional type/range
    if (pos < t.size() && t[pos].type == Token::SIGNED_KW) pos++;
    if (pos < t.size() && t[pos].type == Token::INTEGER_KW) pos++;
    if (pos < t.size() && t[pos].type == Token::LBRACKET) {
        while (pos < t.size() && t[pos].type != Token::RBRACKET) pos++;
        if (pos < t.size()) pos++;
    }
    // name = value
    while (pos < t.size() && t[pos].type != Token::SEMI) {
        if (t[pos].type == Token::IDENT) {
            std::string name = t[pos].value; pos++;
            if (pos < t.size() && t[pos].type == Token::ASSIGN) {
                pos++;
                if (pos < t.size() && t[pos].type == Token::NUMBER) {
                    params_[name] = (int)parse_verilog_number(t[pos].value);
                    pos++;
                }
            }
        } else {
            pos++;
        }
    }
    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
    return pos;
}

// ============================================================
// ASSIGN — now uses expression parser + StructuralSynthesizer
// ============================================================

size_t VerilogParser::parse_assign(const std::vector<Token>& t, size_t pos, Netlist& nl,
                                    VerilogParseResult& r) {
    pos++; // skip 'assign'

    // Parse LHS name + optional bit-select
    std::string lhs;
    int lhs_bit = -1;
    std::pair<int,int> lhs_part = {-1,-1};
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        lhs = t[pos].value; pos++;
        if (pos < t.size() && t[pos].type == Token::LBRACKET) {
            auto sel = parse_bus_range(t, pos);
            if (sel.first == sel.second) lhs_bit = sel.first;
            else lhs_part = sel;
        }
    }
    if (pos < t.size() && t[pos].type == Token::ASSIGN) pos++;

    int lhs_w = bus_width(lhs);
    bool lhs_is_bus = (lhs_bit < 0 && lhs_part.first < 0 && lhs_w > 1);

    // Parse RHS expression using the full expression parser
    auto rhs_expr = parse_expression(t, pos);

    // Skip to semicolon
    while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
    if (pos < t.size()) pos++;

    // Synthesize: use StructuralSynthesizer for expression -> gates
    if (lhs_is_bus || (lhs_part.first >= 0)) {
        int target_w = lhs_is_bus ? lhs_w : (std::abs(lhs_part.first - lhs_part.second) + 1);
        auto bits = struct_synth_.synth_expr_bus(rhs_expr, nl, net_map_, bus_ranges_, target_w);
        // Connect to LHS bits
        auto it = bus_ranges_.find(lhs);
        int lo = 0;
        if (lhs_part.first >= 0) {
            lo = std::min(lhs_part.first, lhs_part.second);
        } else if (it != bus_ranges_.end()) {
            lo = std::min(it->second.first, it->second.second);
        }
        for (int i = 0; i < target_w && i < (int)bits.size(); i++) {
            std::string bit_name = lhs + "[" + std::to_string(lo + i) + "]";
            NetId out = get_or_create_net(nl, bit_name);
            // Create BUF from synthesized bit to output bit
            if (bits[i] != out) {
                nl.add_gate(GateType::BUF, {bits[i]}, out, "assign_" + bit_name);
                r.num_gates++;
            }
        }
    } else {
        // Single-bit LHS
        std::string lhs_name = (lhs_bit >= 0) ? lhs + "[" + std::to_string(lhs_bit) + "]" : lhs;
        NetId out = get_or_create_net(nl, lhs_name);
        NetId result = struct_synth_.synth_expr(rhs_expr, nl, net_map_);
        if (result >= 0 && result != out) {
            nl.add_gate(GateType::BUF, {result}, out, "assign_" + lhs_name);
            r.num_gates++;
        }
    }

    return pos;
}


// ============================================================
// MODULE INSTANTIATION (preserved)
// ============================================================

size_t VerilogParser::parse_module_inst(const std::vector<Token>& t, size_t pos, Netlist& nl,
                                         VerilogParseResult& r, const std::string& mod_type) {
    std::string inst_name;
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        inst_name = t[pos].value; pos++;
    }

    // Parse parameter overrides #(.P(V)) if present at this point
    std::unordered_map<std::string, int> param_overrides;

    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        pos++;
        std::vector<std::pair<std::string, std::string>> connections;
        while (pos < t.size() && t[pos].type != Token::RPAREN) {
            if (t[pos].type == Token::DOT) {
                pos++;
                std::string port;
                if (pos < t.size() && t[pos].type == Token::IDENT) { port = t[pos].value; pos++; }
                if (pos < t.size() && t[pos].type == Token::LPAREN) {
                    pos++;
                    std::string sig;
                    if (pos < t.size() && t[pos].type == Token::IDENT) { sig = t[pos].value; pos++; }
                    if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
                    connections.push_back({port, sig});
                }
            } else if (t[pos].type == Token::IDENT) {
                connections.push_back({"", t[pos].value}); pos++;
            }
            if (pos < t.size() && t[pos].type == Token::COMMA) pos++;
        }
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;

        // Try hierarchical elaboration if module definition is available
        auto it = module_defs_.find(mod_type);
        if (it != module_defs_.end() && hierarchy_depth_ < MAX_HIERARCHY_DEPTH) {
            elaborate_instance(mod_type, inst_name, connections, param_overrides, nl, r);
        } else {
            // Fallback: BUF-based port connections (no submodule logic)
            for (size_t ci = 0; ci < connections.size(); ci++) {
                auto& conn = connections[ci];
                std::string port_net = inst_name + "/" + (conn.first.empty() ? ("p" + std::to_string(ci)) : conn.first);
                std::string sig_net = conn.second;
                if (!sig_net.empty()) {
                    NetId pn = get_or_create_net(nl, port_net);
                    int sw = bus_width(sig_net);
                    if (sw > 1) {
                        auto bits = get_or_create_bus(nl, port_net, sw-1, 0);
                        for (int bi = 0; bi < sw; bi++) {
                            auto bit_it = bus_ranges_.find(sig_net);
                            int lo = (bit_it != bus_ranges_.end()) ? std::min(bit_it->second.first, bit_it->second.second) : 0;
                            std::string bit_name = sig_net + "[" + std::to_string(lo + bi) + "]";
                            NetId sn = get_or_create_net(nl, bit_name);
                            nl.add_gate(GateType::BUF, {sn}, bits[bi], inst_name + "_conn_" + std::to_string(ci) + "_" + std::to_string(bi));
                            r.num_gates++;
                        }
                    } else {
                        NetId sn = get_or_create_net(nl, sig_net);
                        nl.add_gate(GateType::BUF, {sn}, pn, inst_name + "_conn_" + std::to_string(ci));
                        r.num_gates++;
                    }
                }
            }
        }
    }
    return pos;
}

// Hierarchical elaboration: parse submodule into temp netlist, copy gates with prefix
void VerilogParser::elaborate_instance(const std::string& mod_type, const std::string& inst_name,
                                        const std::vector<std::pair<std::string,std::string>>& connections,
                                        const std::unordered_map<std::string,int>& param_overrides,
                                        Netlist& nl, VerilogParseResult& r) {
    auto& mdef = module_defs_[mod_type];
    hierarchy_depth_++;

    // Save parent parser state
    auto saved_net_map = net_map_;
    auto saved_bus_ranges = bus_ranges_;
    auto saved_params = params_;
    auto saved_functions = functions_;
    auto saved_signed = signed_signals_;
    auto saved_memory = memory_arrays_;
    auto saved_tasks = tasks_;
    auto saved_multidim = multidim_arrays_;

    // Fresh state for submodule parsing
    net_map_.clear();
    bus_ranges_.clear();
    // Apply parameter overrides
    params_ = param_overrides;
    functions_.clear();
    signed_signals_.clear();
    memory_arrays_.clear();
    tasks_.clear();
    multidim_arrays_.clear();

    // Parse submodule into temporary netlist
    Netlist sub_nl;
    VerilogParseResult sub_r;
    sub_r.success = true;

    if (!mdef.tokens.empty()) {
        size_t sub_pos = 0;
        if (mdef.tokens[sub_pos].type == Token::IDENT && mdef.tokens[sub_pos].value == "module")
            parse_module(mdef.tokens, sub_pos, sub_nl, sub_r);
    }

    // Build port name → submodule net ID mapping
    std::unordered_map<std::string, NetId> sub_port_nets;
    for (auto& [name, nid] : net_map_) {
        // Only store simple port names (not bit-level names)
        sub_port_nets[name] = nid;
    }

    // Copy submodule gates into parent netlist with instance-prefixed signal names
    // Create mapping: submodule net ID → parent net ID
    std::unordered_map<NetId, NetId> net_remap;

    // First, create parent nets for all submodule internal nets
    for (size_t ni = 0; ni < sub_nl.num_nets(); ni++) {
        auto& sn = sub_nl.net(ni);
        std::string prefixed = inst_name + "/" + sn.name;
        // Check if this is a port connected to parent
        bool is_port = false;
        for (auto& conn : connections) {
            std::string port_name = conn.first;
            if (port_name.empty()) continue;
            if (sn.name == port_name || sn.name.find(port_name + "[") == 0) {
                // This submodule port maps to parent signal
                std::string parent_sig = conn.second;
                if (!parent_sig.empty()) {
                    // Check if bus bit
                    if (sn.name.find('[') != std::string::npos && parent_sig.find('[') == std::string::npos) {
                        // Submodule has bit-level, parent signal might be bus
                        // Extract bit index from submodule name
                        std::string bit_suffix = sn.name.substr(sn.name.find('['));
                        std::string parent_bit = parent_sig + bit_suffix;
                        auto pit = saved_net_map.find(parent_bit);
                        if (pit != saved_net_map.end()) {
                            net_remap[ni] = pit->second;
                            is_port = true;
                            break;
                        }
                    }
                    auto pit = saved_net_map.find(parent_sig);
                    if (pit != saved_net_map.end()) {
                        net_remap[ni] = pit->second;
                        is_port = true;
                        break;
                    }
                }
            }
        }
        if (!is_port) {
            // Internal net — create new net in parent with instance prefix
            NetId parent_nid = nl.add_net(prefixed);
            net_remap[ni] = parent_nid;
        }
    }

    // Copy all non-INPUT/OUTPUT gates from submodule to parent
    for (size_t gi = 0; gi < sub_nl.num_gates(); gi++) {
        auto& sg = sub_nl.gate(gi);
        if (sg.type == GateType::INPUT || sg.type == GateType::OUTPUT) continue;

        // Remap inputs
        std::vector<NetId> remapped_inputs;
        for (auto in_nid : sg.inputs) {
            auto it = net_remap.find(in_nid);
            if (it != net_remap.end()) remapped_inputs.push_back(it->second);
            else remapped_inputs.push_back(in_nid); // shouldn't happen
        }

        // Remap output
        NetId remapped_out = -1;
        if (sg.output >= 0) {
            auto it = net_remap.find(sg.output);
            if (it != net_remap.end()) remapped_out = it->second;
        }

        std::string gate_name = inst_name + "/" + (sg.name.empty() ? ("g" + std::to_string(gi)) : sg.name);
        GateId gid = nl.add_gate(sg.type, remapped_inputs, remapped_out, gate_name);
        r.num_gates++;

        // Handle DFF special fields
        if (sg.type == GateType::DFF && sg.clk >= 0) {
            auto cit = net_remap.find(sg.clk);
            if (cit != net_remap.end()) nl.gate(gid).clk = cit->second;
        }
    }

    // Restore parent parser state
    net_map_ = saved_net_map;
    bus_ranges_ = saved_bus_ranges;
    params_ = saved_params;
    functions_ = saved_functions;
    signed_signals_ = saved_signed;
    memory_arrays_ = saved_memory;
    tasks_ = saved_tasks;
    multidim_arrays_ = saved_multidim;

    hierarchy_depth_--;
}

// ============================================================
// MODULE body + top-level parsing
// ============================================================

size_t VerilogParser::parse_module(const std::vector<Token>& t, size_t pos, Netlist& nl,
                                    VerilogParseResult& r) {
    pos++; // skip 'module'
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        r.module_name = t[pos].value;
        known_modules_.push_back(r.module_name);
        pos++;
    }

    // Optional #(parameters)
    if (pos < t.size() && t[pos].type == Token::HASH) {
        pos++;
        if (pos < t.size() && t[pos].type == Token::LPAREN) {
            pos++;
            // Parse parameter declarations inside #(...)
            while (pos < t.size() && t[pos].type != Token::RPAREN) {
                if (t[pos].type == Token::PARAMETER_KW) pos++; // skip keyword
                if (pos < t.size() && t[pos].type == Token::SIGNED_KW) pos++;
                if (pos < t.size() && t[pos].type == Token::INTEGER_KW) pos++;
                // Skip optional range [MSB:LSB]
                if (pos < t.size() && t[pos].type == Token::LBRACKET) {
                    while (pos < t.size() && t[pos].type != Token::RBRACKET) pos++;
                    if (pos < t.size()) pos++;
                }
                if (pos < t.size() && t[pos].type == Token::IDENT) {
                    std::string pname = t[pos].value; pos++;
                    if (pos < t.size() && t[pos].type == Token::ASSIGN) {
                        pos++;
                        if (pos < t.size() && t[pos].type == Token::NUMBER) {
                            params_[pname] = (int)parse_verilog_number(t[pos].value);
                            pos++;
                        }
                    }
                }
                if (pos < t.size() && t[pos].type == Token::COMMA) pos++;
            }
            if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
        }
    }

    // Parse ANSI-style port list
    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        pos++;
        std::string current_dir = "";
        while (pos < t.size() && t[pos].type != Token::RPAREN) {
            if (t[pos].type == Token::IDENT && (t[pos].value == "input" || t[pos].value == "output" || t[pos].value == "inout")) {
                current_dir = t[pos].value; pos++;
                if (pos < t.size() && t[pos].type == Token::IDENT &&
                    (t[pos].value == "wire" || t[pos].value == "reg" || t[pos].value == "logic")) {
                    pos++;
                }
                if (pos < t.size() && t[pos].type == Token::SIGNED_KW) pos++;
                auto range = parse_bus_range(t, pos);
                while (pos < t.size() && t[pos].type == Token::IDENT) {
                    std::string name = t[pos].value;
                    if (range.first >= 0 && range.first != range.second) {
                        auto bits = get_or_create_bus(nl, name, range.first, range.second);
                        for (auto nid : bits) {
                            if (current_dir == "input") { nl.mark_input(nid); r.num_inputs++; }
                            else if (current_dir == "output" || current_dir == "inout") { nl.mark_output(nid); r.num_outputs++; }
                        }
                    } else {
                        NetId nid = get_or_create_net(nl, name);
                        if (current_dir == "input") { nl.mark_input(nid); r.num_inputs++; }
                        else if (current_dir == "output" || current_dir == "inout") { nl.mark_output(nid); r.num_outputs++; }
                    }
                    pos++;
                    if (pos < t.size() && t[pos].type == Token::COMMA) { pos++; break; }
                }
            } else if (t[pos].type == Token::IDENT && (t[pos].value == "wire" || t[pos].value == "reg" || t[pos].value == "logic")) {
                pos++;
            } else if (t[pos].type == Token::SIGNED_KW) {
                pos++;
            } else if (t[pos].type == Token::IDENT) {
                if (current_dir != "") {
                    NetId nid = get_or_create_net(nl, t[pos].value);
                    if (current_dir == "input") { nl.mark_input(nid); r.num_inputs++; }
                    else if (current_dir == "output" || current_dir == "inout") { nl.mark_output(nid); r.num_outputs++; }
                } else {
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
        // Skip non-keyword tokens
        if (t[pos].type != Token::IDENT && t[pos].type != Token::ALWAYS &&
            t[pos].type != Token::PARAMETER_KW && t[pos].type != Token::INTEGER_KW &&
            t[pos].type != Token::GENERATE_KW && t[pos].type != Token::GENVAR_KW &&
            t[pos].type != Token::FUNCTION_KW && t[pos].type != Token::TASK_KW &&
            t[pos].type != Token::INITIAL_KW && t[pos].type != Token::SPECIFY_KW &&
            t[pos].type != Token::DEFPARAM_KW && t[pos].type != Token::DISABLE_KW) {
            pos++; continue;
        }

        const std::string& kw = t[pos].value;
        if (kw == "input")    pos = parse_port_decl(t, pos, nl, r, "input");
        else if (kw == "output") pos = parse_port_decl(t, pos, nl, r, "output");
        else if (kw == "inout") pos = parse_port_decl(t, pos, nl, r, "output"); // treat inout as output for synthesis
        else if (kw == "wire") pos = parse_port_decl(t, pos, nl, r, "wire");
        // Tri-state and special net types — treat as wire for synthesis
        else if (kw == "tri" || kw == "wand" || kw == "wor" ||
                 kw == "tri0" || kw == "tri1" ||
                 kw == "supply0" || kw == "supply1")
            pos = parse_port_decl(t, pos, nl, r, "wire");
        else if (kw == "reg") {
            // Check for memory array: reg [7:0] mem [0:255]
            size_t saved_pos = pos;
            pos++; // skip reg
            // Skip optional signed
            if (pos < t.size() && t[pos].type == Token::SIGNED_KW) {
                signed_signals_.insert("__next_reg__"); pos++;
            }
            std::pair<int,int> word_range = {-1, -1};
            if (pos < t.size() && t[pos].type == Token::LBRACKET) {
                word_range = parse_bus_range(t, pos);
            }
            if (pos < t.size() && t[pos].type == Token::IDENT) {
                std::string reg_name = t[pos].value;
                pos++;
                // Check for memory dimension: [0:N] after name
                if (pos < t.size() && t[pos].type == Token::LBRACKET) {
                    auto mem_range = parse_bus_range(t, pos);
                    if (mem_range.first >= 0) {
                        int word_w = (word_range.first >= 0) ?
                            std::abs(word_range.first - word_range.second) + 1 : 1;
                        int depth = std::abs(mem_range.first - mem_range.second) + 1;

                        // Check for second dimension: [0:M]
                        if (pos < t.size() && t[pos].type == Token::LBRACKET) {
                            auto dim2_range = parse_bus_range(t, pos);
                            if (dim2_range.first >= 0) {
                                int depth2 = std::abs(dim2_range.first - dim2_range.second) + 1;
                                // Flatten 2D to 1D: total_depth = dim1 * dim2
                                multidim_arrays_[reg_name] = {word_w, depth, depth2};
                                memory_arrays_[reg_name] = {word_w, depth * depth2};
                                if (word_range.first >= 0) bus_ranges_[reg_name] = word_range;
                                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                                continue;
                            }
                        }

                        memory_arrays_[reg_name] = {word_w, depth};
                        if (word_range.first >= 0) bus_ranges_[reg_name] = word_range;
                        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                        continue;
                    }
                }
                // Not a memory, go back and use normal port decl
                pos = saved_pos;
            } else {
                pos = saved_pos;
            }
            pos = parse_port_decl(t, pos, nl, r, "wire");
        }
        else if (kw == "assign") pos = parse_assign(t, pos, nl, r);
        else if (kw == "and")   pos = parse_gate_inst(t, pos, nl, r, GateType::AND);
        else if (kw == "or")    pos = parse_gate_inst(t, pos, nl, r, GateType::OR);
        else if (kw == "not" || kw == "inv") pos = parse_gate_inst(t, pos, nl, r, GateType::NOT);
        else if (kw == "nand")  pos = parse_gate_inst(t, pos, nl, r, GateType::NAND);
        else if (kw == "nor")   pos = parse_gate_inst(t, pos, nl, r, GateType::NOR);
        else if (kw == "xor")   pos = parse_gate_inst(t, pos, nl, r, GateType::XOR);
        else if (kw == "xnor")  pos = parse_gate_inst(t, pos, nl, r, GateType::XNOR);
        else if (kw == "buf")   pos = parse_gate_inst(t, pos, nl, r, GateType::BUF);
        else if (kw == "bufif0") pos = parse_gate_inst(t, pos, nl, r, GateType::BUFIF0);
        else if (kw == "bufif1") pos = parse_gate_inst(t, pos, nl, r, GateType::BUFIF1);
        else if (kw == "notif0") pos = parse_gate_inst(t, pos, nl, r, GateType::NOTIF0);
        else if (kw == "notif1") pos = parse_gate_inst(t, pos, nl, r, GateType::NOTIF1);
        else if (kw == "always") pos = parse_always(t, pos, ast.root);
        else if (t[pos].type == Token::PARAMETER_KW) pos = parse_parameter(t, pos);
        else if (t[pos].type == Token::INTEGER_KW) {
            pos++;
            while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
            if (pos < t.size()) pos++;
        }
        else if (t[pos].type == Token::GENVAR_KW) {
            // genvar i; — just skip, loop var handled in generate-for
            pos++;
            while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
            if (pos < t.size()) pos++;
        }
        else if (t[pos].type == Token::GENERATE_KW) {
            pos++; // skip 'generate'
            while (pos < t.size() && t[pos].type != Token::ENDGENERATE_KW) {
                if (t[pos].type == Token::FOR_KW) {
                    pos = parse_generate_for(t, pos, nl, r);
                } else if (t[pos].type == Token::IF) {
                    pos = parse_generate_if(t, pos, nl, r);
                } else if (t[pos].type == Token::CASE_KW) {
                    pos = parse_generate_case(t, pos, nl, r);
                } else if (t[pos].type == Token::GENVAR_KW) {
                    pos++;
                    while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
                    if (pos < t.size()) pos++;
                } else {
                    pos++;
                }
            }
            if (pos < t.size() && t[pos].type == Token::ENDGENERATE_KW) pos++;
        }
        else if (t[pos].type == Token::FUNCTION_KW) {
            pos = parse_function_def(t, pos);
        }
        else if (t[pos].type == Token::TASK_KW) {
            pos = parse_task_def(t, pos);
        }
        else if (t[pos].type == Token::INITIAL_KW) {
            // Skip initial blocks (not synthesizable)
            pos++;
            if (pos < t.size() && t[pos].type == Token::BEGIN_KW) {
                int depth = 1; pos++;
                while (pos < t.size() && depth > 0) {
                    if (t[pos].type == Token::BEGIN_KW) depth++;
                    if (t[pos].type == Token::END_KW) depth--;
                    pos++;
                }
            } else {
                while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
                if (pos < t.size()) pos++;
            }
        }
        else if (t[pos].type == Token::SPECIFY_KW) {
            // Skip entire specify...endspecify block
            pos++;
            while (pos < t.size() && t[pos].type != Token::ENDSPECIFY_KW) pos++;
            if (pos < t.size()) pos++;
        }
        else if (t[pos].type == Token::DEFPARAM_KW) {
            // defparam instance.param = value;
            pos++; // skip defparam
            // Parse hierarchical name
            std::string hier_name;
            while (pos < t.size() && t[pos].type != Token::ASSIGN && t[pos].type != Token::SEMI) {
                hier_name += t[pos].value;
                pos++;
            }
            if (pos < t.size() && t[pos].type == Token::ASSIGN) {
                pos++;
                size_t ep = pos;
                int val = eval_const_expr(t, ep);
                pos = ep;
                // Extract the parameter name (after last dot)
                auto dotpos = hier_name.rfind('.');
                if (dotpos != std::string::npos) {
                    params_[hier_name.substr(dotpos+1)] = val;
                }
            }
            if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
        }
        else if (t[pos].type == Token::DISABLE_KW) {
            // disable block_name; — skip
            pos++;
            while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
            if (pos < t.size()) pos++;
        }
        else {
            // Module instantiation: ident [#(...)] ident (...)
            // Also handle system tasks at module level ($display, etc.)
            if (t[pos].type == Token::IDENT && t[pos].value.size() > 1 && t[pos].value[0] == '$') {
                // System task at module level — skip to semicolon
                pos++;
                if (pos < t.size() && t[pos].type == Token::LPAREN) {
                    int depth = 1; pos++;
                    while (pos < t.size() && depth > 0) {
                        if (t[pos].type == Token::LPAREN) depth++;
                        if (t[pos].type == Token::RPAREN) depth--;
                        pos++;
                    }
                }
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            }
            // Module instantiation with parameter override: mod #(.P(V)) inst (...)
            else if (pos + 2 < t.size() && t[pos].type == Token::IDENT &&
                     t[pos+1].type == Token::HASH) {
                std::string mod_type = t[pos].value;
                pos++; // skip module type
                pos++; // skip #
                // Skip parameter override #(...)
                if (pos < t.size() && t[pos].type == Token::LPAREN) {
                    int depth = 1; pos++;
                    while (pos < t.size() && depth > 0) {
                        if (t[pos].type == Token::LPAREN) depth++;
                        if (t[pos].type == Token::RPAREN) depth--;
                        pos++;
                    }
                }
                // Now parse instance: inst_name (...)
                pos = parse_module_inst(t, pos, nl, r, mod_type);
            }
            // Standard module instantiation: mod inst (...)
            else if (pos + 2 < t.size() && t[pos].type == Token::IDENT &&
                t[pos+1].type == Token::IDENT && t[pos+2].type == Token::LPAREN) {
                std::string mod_type = t[pos].value;
                pos++;
                pos = parse_module_inst(t, pos, nl, r, mod_type);
            } else {
                pos++;
            }
        }
    }
    // Bridge bus width info to AST for behavioral synthesizer
    ast.bus_ranges = bus_ranges_;
    ast.net_map = net_map_;
    ast.memory_arrays = memory_arrays_;
    ast.signed_signals = signed_signals_;
    if (pos < t.size()) pos++; // skip endmodule
    return pos;
}

VerilogParseResult VerilogParser::parse_tokens(const std::vector<Token>& t, Netlist& nl) {
    VerilogParseResult r;
    r.success = true;
    net_map_.clear();
    bus_ranges_.clear();
    params_.clear();
    functions_.clear();
    signed_signals_.clear();
    memory_arrays_.clear();
    tasks_.clear();
    multidim_arrays_.clear();
    module_defs_.clear();
    hierarchy_depth_ = 0;

    // === Pass 1: Collect all module definitions (token ranges) ===
    {
        size_t pos = 0;
        while (pos < t.size() && t[pos].type != Token::END) {
            if (t[pos].type == Token::IDENT && t[pos].value == "module") {
                size_t mod_start = pos;
                pos++; // skip 'module'
                std::string mod_name;
                if (pos < t.size() && t[pos].type == Token::IDENT)
                    mod_name = t[pos].value;
                // Find endmodule
                int depth = 0;
                while (pos < t.size() && t[pos].type != Token::END) {
                    if (t[pos].type == Token::IDENT && t[pos].value == "module" && pos != mod_start)
                        depth++;
                    if (t[pos].type == Token::IDENT && t[pos].value == "endmodule") {
                        if (depth == 0) { pos++; break; }
                        depth--;
                    }
                    pos++;
                }
                // Store token range for this module
                if (!mod_name.empty()) {
                    ModuleDef md;
                    md.name = mod_name;
                    md.tokens.assign(t.begin() + mod_start, t.begin() + pos);
                    module_defs_[mod_name] = std::move(md);
                }
            } else pos++;
        }
    }

    // === Pass 2: Parse only the top module (last in file) ===
    // Earlier modules are available in module_defs_ for hierarchical elaboration.
    size_t pos = 0;
    // Find the LAST 'module' keyword — that's the top module
    size_t last_module_pos = 0;
    bool found_module = false;
    {
        size_t scan = 0;
        while (scan < t.size() && t[scan].type != Token::END) {
            if (t[scan].type == Token::IDENT && t[scan].value == "module") {
                last_module_pos = scan;
                found_module = true;
            }
            scan++;
        }
    }
    if (found_module) {
        pos = last_module_pos;
        parse_module(t, pos, nl, r);
    }
    return r;
}

VerilogParseResult VerilogParser::parse_string(const std::string& src, Netlist& nl) {
    auto processed = preprocess(src);
    auto tokens = tokenize(processed);
    return parse_tokens(tokens, nl);
}

VerilogParseResult VerilogParser::parse_file(const std::string& filename, Netlist& nl) {
    std::ifstream f(filename);
    if (!f.is_open()) return {false, "", 0, 0, 0, 0, 0, "Cannot open file: " + filename};
    std::string src((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return parse_string(src, nl);
}

// ============================================================
// PREPROCESSOR — `define, `ifdef, `ifndef, `else, `endif, `include
// ============================================================
std::string VerilogParser::preprocess(const std::string& src) {
    std::istringstream stream(src);
    std::string result;
    std::string line;
    std::vector<bool> cond_stack;  // true = active section
    cond_stack.push_back(true);    // top-level always active

    while (std::getline(stream, line)) {
        // Trim leading whitespace for directive detection
        size_t first = line.find_first_not_of(" \t");
        std::string trimmed = (first != std::string::npos) ? line.substr(first) : "";

        if (trimmed.size() > 0 && trimmed[0] == '`') {
            std::string directive;
            size_t dp = 1;
            while (dp < trimmed.size() && (std::isalnum(trimmed[dp]) || trimmed[dp] == '_'))
                directive += trimmed[dp++];

            if (directive == "define") {
                if (cond_stack.back()) {
                    while (dp < trimmed.size() && std::isspace(trimmed[dp])) dp++;
                    std::string name;
                    while (dp < trimmed.size() && (std::isalnum(trimmed[dp]) || trimmed[dp] == '_'))
                        name += trimmed[dp++];
                    while (dp < trimmed.size() && std::isspace(trimmed[dp])) dp++;
                    std::string value = trimmed.substr(dp);
                    // Handle line continuation
                    while (!value.empty() && value.back() == '\\') {
                        value.pop_back();
                        std::string next_line;
                        if (std::getline(stream, next_line)) value += next_line;
                    }
                    defines_[name] = value;
                }
            } else if (directive == "ifdef") {
                while (dp < trimmed.size() && std::isspace(trimmed[dp])) dp++;
                std::string name;
                while (dp < trimmed.size() && (std::isalnum(trimmed[dp]) || trimmed[dp] == '_'))
                    name += trimmed[dp++];
                cond_stack.push_back(cond_stack.back() && defines_.count(name));
            } else if (directive == "ifndef") {
                while (dp < trimmed.size() && std::isspace(trimmed[dp])) dp++;
                std::string name;
                while (dp < trimmed.size() && (std::isalnum(trimmed[dp]) || trimmed[dp] == '_'))
                    name += trimmed[dp++];
                cond_stack.push_back(cond_stack.back() && !defines_.count(name));
            } else if (directive == "else") {
                if (cond_stack.size() > 1) {
                    bool parent = (cond_stack.size() > 2) ? cond_stack[cond_stack.size()-2] : true;
                    cond_stack.back() = parent && !cond_stack.back();
                }
            } else if (directive == "endif") {
                if (cond_stack.size() > 1) cond_stack.pop_back();
            } else if (directive == "elsif") {
                // `elsif condition — flip current, check new condition
                while (dp < trimmed.size() && std::isspace(trimmed[dp])) dp++;
                std::string name;
                while (dp < trimmed.size() && (std::isalnum(trimmed[dp]) || trimmed[dp] == '_'))
                    name += trimmed[dp++];
                if (cond_stack.size() > 1) {
                    bool parent = (cond_stack.size() > 2) ? cond_stack[cond_stack.size()-2] : true;
                    // Only activate if parent is active AND previous branch was NOT taken
                    if (!cond_stack.back() && parent) {
                        cond_stack.back() = defines_.count(name) > 0;
                    } else {
                        cond_stack.back() = false; // previous branch was taken, skip
                    }
                }
            } else if (directive == "undef") {
                while (dp < trimmed.size() && std::isspace(trimmed[dp])) dp++;
                std::string name;
                while (dp < trimmed.size() && (std::isalnum(trimmed[dp]) || trimmed[dp] == '_'))
                    name += trimmed[dp++];
                defines_.erase(name);
            } else if (directive == "timescale" || directive == "celldefine" || directive == "endcelldefine" ||
                       directive == "default_nettype" || directive == "resetall" || directive == "pragma") {
                // Skip these directives entirely — they don't affect synthesis
            } else if (directive == "include") {
                if (cond_stack.back()) {
                    auto q1 = trimmed.find('"');
                    auto q2 = trimmed.rfind('"');
                    if (q1 != std::string::npos && q2 != q1) {
                        std::string inc_file = trimmed.substr(q1+1, q2-q1-1);
                        std::ifstream inc(inc_file);
                        if (inc.is_open()) {
                            std::string inc_src((std::istreambuf_iterator<char>(inc)), std::istreambuf_iterator<char>());
                            result += preprocess(inc_src);
                        }
                    }
                }
            }
            result += "\n"; // keep line numbers consistent
            continue;
        }

        if (cond_stack.back()) {
            // Substitute macro references: `NAME → value
            std::string processed_line;
            for (size_t i = 0; i < line.size(); i++) {
                if (line[i] == '`' && i+1 < line.size() && (std::isalpha(line[i+1]) || line[i+1] == '_')) {
                    i++;
                    std::string mname;
                    while (i < line.size() && (std::isalnum(line[i]) || line[i] == '_'))
                        mname += line[i++];
                    i--; // back up for loop increment
                    auto it = defines_.find(mname);
                    if (it != defines_.end()) processed_line += it->second;
                    else processed_line += "`" + mname; // leave unknown macros
                } else {
                    processed_line += line[i];
                }
            }
            result += processed_line + "\n";
        } else {
            result += "\n";
        }
    }
    return result;
}


// ============================================================
// to_verilog — export netlist (preserved)
// ============================================================

std::string VerilogParser::to_verilog(const Netlist& nl, const std::string& module_name) {
    std::ostringstream v;
    v << "// Generated by SiliconForge\n";
    v << "module " << module_name << " (";
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
    for (auto pi : nl.primary_inputs()) v << "  input " << nl.net(pi).name << ";\n";
    for (auto po : nl.primary_outputs()) v << "  output " << nl.net(po).name << ";\n";
    for (size_t i = 0; i < nl.num_nets(); ++i) {
        bool is_io = false;
        for (auto pi : nl.primary_inputs()) if (pi == (NetId)i) is_io = true;
        for (auto po : nl.primary_outputs()) if (po == (NetId)i) is_io = true;
        if (!is_io) v << "  wire " << nl.net(i).name << ";\n";
    }
    v << "\n";
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

// ============================================================
// BEHAVIORAL PARSER — always, statements, for, case
// ============================================================

size_t VerilogParser::parse_always(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent) {
    auto always = std::make_shared<AstNode>();
    pos++; // skip always

    if (pos < t.size() && t[pos].type == Token::AT) pos++;
    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        pos++;
        // Check for always @(*)
        if (pos < t.size() && t[pos].type == Token::STAR) {
            always->type = AstNodeType::ALWAYS_COMB;
            always->value = "*";
            pos++; // skip *
        } else if (pos < t.size() && t[pos].type == Token::POSEDGE) {
            always->type = AstNodeType::ALWAYS_POS_CLK;
            pos++;
            if (pos < t.size() && t[pos].type == Token::IDENT) { always->value = t[pos].value; pos++; }
        } else if (pos < t.size() && t[pos].type == Token::NEGEDGE) {
            always->type = AstNodeType::ALWAYS_NEG_CLK;
            pos++;
            if (pos < t.size() && t[pos].type == Token::IDENT) { always->value = t[pos].value; pos++; }
        } else {
            // always @(signal_list) — treat as combinational
            always->type = AstNodeType::ALWAYS_COMB;
            always->value = "*";
        }
        // Skip to closing paren (handles "or" in sensitivity lists)
        while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
    }

    // Parse the body
    if (pos < t.size()) {
        pos = parse_statement(t, pos, always);
    }

    parent->add(always);
    has_behavioral_blocks = true;
    return pos;
}

size_t VerilogParser::parse_statement_block(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent) {
    auto block = std::make_shared<AstNode>();
    block->type = AstNodeType::BLOCK_BEGIN_END;
    pos++; // skip begin

    // Skip optional block label: begin : label
    if (pos < t.size() && t[pos].type == Token::COLON) {
        pos++;
        if (pos < t.size() && t[pos].type == Token::IDENT) pos++;
    }

    while (pos < t.size() && t[pos].type != Token::END_KW) {
        // Parse local reg/wire declarations with bus range support
        if (t[pos].type == Token::IDENT && (t[pos].value == "reg" || t[pos].value == "wire")) {
            pos++; // skip reg/wire
            if (pos < t.size() && t[pos].type == Token::SIGNED_KW) pos++;
            auto range = parse_bus_range(t, pos);
            while (pos < t.size() && t[pos].type != Token::SEMI) {
                if (t[pos].type == Token::IDENT) {
                    std::string name = t[pos].value;
                    if (range.first >= 0 && range.first != range.second)
                        bus_ranges_[name] = {range.first, range.second};
                }
                pos++;
            }
            if (pos < t.size()) pos++;
            continue;
        }
        if (t[pos].type == Token::INTEGER_KW) {
            while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
            if (pos < t.size()) pos++;
            continue;
        }
        pos = parse_statement(t, pos, block);
    }
    if (pos < t.size() && t[pos].type == Token::END_KW) pos++;
    parent->add(block);
    return pos;
}

size_t VerilogParser::parse_statement(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent) {
    if (pos >= t.size()) return pos;

    if (t[pos].type == Token::IF) {
        auto if_node = AstNode::make(AstNodeType::IF_ELSE);
        pos++; // skip if
        if (pos < t.size() && t[pos].type == Token::LPAREN) pos++;
        if_node->add(parse_expression(t, pos));
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
        pos = parse_statement(t, pos, if_node);
        if (pos < t.size() && t[pos].type == Token::ELSE) {
            pos++;
            pos = parse_statement(t, pos, if_node);
        }
        parent->add(if_node);
        return pos;
    }
    else if (t[pos].type == Token::CASE_KW) {
        return parse_case(t, pos, parent);
    }
    else if (t[pos].type == Token::FOR_KW) {
        return parse_for_loop(t, pos, parent);
    }
    else if (t[pos].type == Token::WHILE_KW) {
        // while (cond) body — unroll if condition is compile-time evaluable
        pos++; // skip while
        if (pos < t.size() && t[pos].type == Token::LPAREN) pos++;
        // Try to evaluate condition as constant
        size_t cond_start = pos;
        auto cond_expr = parse_expression(t, pos);
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
        // Find body extent
        size_t body_start = pos;
        size_t body_end = pos;
        if (pos < t.size() && t[pos].type == Token::BEGIN_KW) {
            int depth = 1; body_end = pos + 1;
            while (body_end < t.size() && depth > 0) {
                if (t[body_end].type == Token::BEGIN_KW) depth++;
                if (t[body_end].type == Token::END_KW) depth--;
                body_end++;
            }
        } else {
            body_end = pos;
            while (body_end < t.size() && t[body_end].type != Token::SEMI) body_end++;
            if (body_end < t.size()) body_end++;
        }
        // Unroll up to 1024 iterations (compile-time condition only)
        if (cond_expr && cond_expr->type == AstNodeType::NUMBER_LITERAL) {
            for (int iter = 0; iter < 1024 && cond_expr->int_val; iter++) {
                size_t bp = body_start;
                bp = parse_statement(t, bp, parent);
                // Re-evaluate condition
                size_t cp = cond_start;
                cond_expr = parse_expression(t, cp);
            }
        }
        return body_end;
    }
    else if (t[pos].type == Token::REPEAT_KW) {
        // repeat (N) body — unroll N times
        pos++; // skip repeat
        int count = 0;
        if (pos < t.size() && t[pos].type == Token::LPAREN) pos++;
        if (pos < t.size()) {
            size_t ep = pos;
            count = eval_const_expr(t, ep);
            pos = ep;
        }
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
        size_t body_start = pos;
        size_t body_end = pos;
        if (pos < t.size() && t[pos].type == Token::BEGIN_KW) {
            int depth = 1; body_end = pos + 1;
            while (body_end < t.size() && depth > 0) {
                if (t[body_end].type == Token::BEGIN_KW) depth++;
                if (t[body_end].type == Token::END_KW) depth--;
                body_end++;
            }
        } else {
            while (body_end < t.size() && t[body_end].type != Token::SEMI) body_end++;
            if (body_end < t.size()) body_end++;
        }
        if (count > 1024) count = 1024;
        for (int i = 0; i < count; i++) {
            size_t bp = body_start;
            bp = parse_statement(t, bp, parent);
        }
        return body_end;
    }
    else if (t[pos].type == Token::FOREVER_KW) {
        // forever — skip for synthesis (not synthesizable without break)
        pos++;
        if (pos < t.size() && t[pos].type == Token::BEGIN_KW) {
            int depth = 1; pos++;
            while (pos < t.size() && depth > 0) {
                if (t[pos].type == Token::BEGIN_KW) depth++;
                if (t[pos].type == Token::END_KW) depth--;
                pos++;
            }
        } else {
            while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
            if (pos < t.size()) pos++;
        }
        return pos;
    }
    else if (t[pos].type == Token::BEGIN_KW) {
        return parse_statement_block(t, pos, parent);
    }
    else if (t[pos].type == Token::DISABLE_KW) {
        // disable block_name; — skip for synthesis
        pos++;
        while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
        if (pos < t.size()) pos++;
        return pos;
    }
    else if (t[pos].type == Token::IDENT) {
        std::string peeked = t[pos].value;
        // System tasks: $display, $finish, $stop, $write, $monitor, etc. — skip
        if (peeked.size() > 1 && peeked[0] == '$') {
            pos++;
            if (pos < t.size() && t[pos].type == Token::LPAREN) {
                int depth = 1; pos++;
                while (pos < t.size() && depth > 0) {
                    if (t[pos].type == Token::LPAREN) depth++;
                    if (t[pos].type == Token::RPAREN) depth--;
                    pos++;
                }
            }
            if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            return pos;
        }
        // wait(expr) statement — skip for synthesis
        if (peeked == "wait") {
            pos++;
            if (pos < t.size() && t[pos].type == Token::LPAREN) {
                int depth = 1; pos++;
                while (pos < t.size() && depth > 0) {
                    if (t[pos].type == Token::LPAREN) depth++;
                    if (t[pos].type == Token::RPAREN) depth--;
                    pos++;
                }
            }
            // Skip the body statement
            pos = parse_statement(t, pos, parent);
            return pos;
        }
        // force/release/deassign — skip to semicolon
        if (peeked == "force" || peeked == "release" || peeked == "deassign") {
            pos++;
            while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
            if (pos < t.size()) pos++;
            return pos;
        }
        // Task call: task_name(args); — inline the body
        if (pos + 1 < t.size() && t[pos+1].type == Token::LPAREN && tasks_.count(peeked)) {
            auto& task = tasks_[peeked];
            pos++; // skip task name
            pos++; // skip (
            // Parse arguments
            std::vector<std::shared_ptr<AstNode>> args;
            while (pos < t.size() && t[pos].type != Token::RPAREN) {
                args.push_back(parse_expression(t, pos));
                if (pos < t.size() && t[pos].type == Token::COMMA) pos++;
            }
            if (pos < t.size()) pos++; // skip )
            if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            // Inline: substitute params and parse body tokens as statements
            auto saved_params = params_;
            for (size_t ai = 0; ai < std::min(args.size(), task.param_names.size()); ai++) {
                if (args[ai]->type == AstNodeType::NUMBER_LITERAL)
                    params_[task.param_names[ai]] = args[ai]->int_val;
            }
            if (!task.body_tokens.empty()) {
                size_t tpos = 0;
                while (tpos < task.body_tokens.size() && task.body_tokens[tpos].type != Token::END) {
                    tpos = parse_statement(task.body_tokens, tpos, parent);
                }
            }
            params_ = saved_params;
            return pos;
        }
        // Assignment: target [<= or =] expr;
        auto assign = std::make_shared<AstNode>();
        std::string target = t[pos].value; pos++;

        // Handle memory array write: mem[addr_expr] <= data
        bool is_mem_write = false;
        std::shared_ptr<AstNode> mem_addr_expr;
        if (pos < t.size() && t[pos].type == Token::LBRACKET && memory_arrays_.count(target)) {
            is_mem_write = true;
            pos++; // skip [
            mem_addr_expr = parse_expression(t, pos);
            if (pos < t.size() && t[pos].type == Token::RBRACKET) pos++;
        }

        // Handle bit-select, part-select, or indexed part-select on LHS
        bool is_part_select = false;
        bool is_indexed_partsel = false;
        int part_hi = -1, part_lo = -1;
        int ips_width = 0;
        bool ips_ascending = false;
        std::shared_ptr<AstNode> ips_base_expr;
        if (!is_mem_write && pos < t.size() && t[pos].type == Token::LBRACKET) {
            // Peek for +: or -: (indexed part-select)
            size_t peek = pos + 1;
            int bd = 1;
            bool has_pc = false, has_mc = false;
            for (size_t p = peek; p < t.size() && bd > 0; p++) {
                if (t[p].type == Token::LBRACKET) bd++;
                else if (t[p].type == Token::RBRACKET) bd--;
                else if (t[p].type == Token::PLUS_COLON && bd == 1) { has_pc = true; break; }
                else if (t[p].type == Token::MINUS_COLON && bd == 1) { has_mc = true; break; }
            }
            if (has_pc || has_mc) {
                is_indexed_partsel = true;
                ips_ascending = has_pc;
                pos++; // skip [
                ips_base_expr = parse_expression(t, pos);
                pos++; // skip +: or -:
                if (pos < t.size() && t[pos].type == Token::NUMBER) {
                    ips_width = (int)parse_verilog_number(t[pos].value); pos++;
                } else if (pos < t.size() && t[pos].type == Token::IDENT) {
                    auto pit3 = params_.find(t[pos].value);
                    if (pit3 != params_.end()) ips_width = pit3->second;
                    pos++;
                }
                if (pos < t.size() && t[pos].type == Token::RBRACKET) pos++;
            } else {
                auto sel = parse_bus_range(t, pos);
                if (sel.first >= 0 && sel.first == sel.second) {
                    target = target + "[" + std::to_string(sel.first) + "]";
                } else if (sel.first >= 0) {
                    is_part_select = true;
                    part_hi = std::max(sel.first, sel.second);
                    part_lo = std::min(sel.first, sel.second);
                }
            }
        }

        if (pos < t.size() && t[pos].type == Token::LEQ) {
            assign->type = AstNodeType::NONBLOCK_ASSIGN;
            pos++;
        } else if (pos < t.size() && t[pos].type == Token::ASSIGN) {
            assign->type = AstNodeType::BLOCK_ASSIGN;
            pos++;
        } else {
            // Unknown, skip
            while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
            if (pos < t.size()) pos++;
            return pos;
        }

        auto rhs = parse_expression(t, pos);
        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;

        if (is_mem_write) {
            // Memory write: mem[addr] <= data → MEM_WRITE node
            auto mw = AstNode::make(AstNodeType::MEM_WRITE, target);
            mw->add(mem_addr_expr);
            mw->add(rhs);
            parent->add(mw);
        } else if (is_indexed_partsel) {
            // Indexed part-select: target[base +: width] <= rhs
            // If base is constant, expand to per-bit assignments
            int base_val = -1;
            if (ips_base_expr && ips_base_expr->type == AstNodeType::NUMBER_LITERAL) {
                try { base_val = std::stoi(ips_base_expr->value); } catch (...) {}
            }
            if (base_val >= 0 && ips_width > 0) {
                int lo, hi;
                if (ips_ascending) { lo = base_val; hi = base_val + ips_width - 1; }
                else { hi = base_val; lo = base_val - ips_width + 1; if (lo < 0) lo = 0; }
                int pw = hi - lo + 1;
                for (int b = lo; b <= hi; b++) {
                    auto bit_assign = std::make_shared<AstNode>();
                    bit_assign->type = assign->type;
                    bit_assign->value = target + "[" + std::to_string(b) + "]";
                    int bit_idx = b - lo;
                    if (rhs->type == AstNodeType::CONCAT && (int)rhs->children.size() == pw) {
                        bit_assign->add(rhs->children[bit_idx]);
                    } else if (rhs->type == AstNodeType::WIRE_DECL) {
                        bit_assign->add(AstNode::make(AstNodeType::WIRE_DECL,
                            rhs->value + "[" + std::to_string(bit_idx) + "]"));
                    } else {
                        bit_assign->add(rhs);
                    }
                    parent->add(bit_assign);
                }
            } else {
                // Variable base — model as full-bus write (best effort)
                assign->value = target;
                assign->add(rhs);
                parent->add(assign);
            }
        } else if (is_part_select) {
            // Expand part-select into per-bit assignments
            int part_width = part_hi - part_lo + 1;
            for (int b = part_lo; b <= part_hi; b++) {
                auto bit_assign = std::make_shared<AstNode>();
                bit_assign->type = assign->type;
                bit_assign->value = target + "[" + std::to_string(b) + "]";
                int bit_idx = b - part_lo;
                // If RHS is a CONCAT (from part-select like a[3:0]), pick individual children
                if (rhs->type == AstNodeType::CONCAT && (int)rhs->children.size() == part_width) {
                    bit_assign->add(rhs->children[bit_idx]);
                } else if (rhs->type == AstNodeType::WIRE_DECL) {
                    // Simple signal: reference individual bit
                    auto bit_ref = AstNode::make(AstNodeType::WIRE_DECL,
                        rhs->value + "[" + std::to_string(bit_idx) + "]");
                    bit_assign->add(bit_ref);
                } else {
                    // Complex expression: pass through, synth will handle
                    bit_assign->add(rhs);
                }
                parent->add(bit_assign);
            }
        } else {
            assign->value = target;
            assign->add(rhs);
            parent->add(assign);
        }
        return pos;
    }

    pos++; // fallback skip
    return pos;
}

size_t VerilogParser::parse_case(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent) {
    auto case_node = AstNode::make(AstNodeType::CASE_STMT);
    case_node->value = t[pos].value; // "case", "casex", or "casez"
    pos++; // skip case/casex/casez
    if (pos < t.size() && t[pos].type == Token::LPAREN) pos++;
    case_node->add(parse_expression(t, pos));
    if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;

    while (pos < t.size() && t[pos].type != Token::ENDCASE_KW) {
        auto item = AstNode::make(AstNodeType::CASE_ITEM);
        if (t[pos].type == Token::DEFAULT_KW) {
            item->value = "default"; pos++;
            if (pos < t.size() && t[pos].type == Token::COLON) pos++;
        } else if (t[pos].type == Token::NUMBER || t[pos].type == Token::IDENT) {
            item->value = t[pos].value; pos++;
            if (pos < t.size() && t[pos].type == Token::COLON) pos++;
        } else {
            pos++; continue;
        }
        if (pos < t.size() && t[pos].type != Token::ENDCASE_KW) {
            pos = parse_statement(t, pos, item);
        }
        case_node->add(item);
    }
    if (pos < t.size() && t[pos].type == Token::ENDCASE_KW) pos++;
    parent->add(case_node);
    has_behavioral_blocks = true;
    return pos;
}

// for(i=0; i<N; i=i+1) body — unrolled at parse time
size_t VerilogParser::parse_for_loop(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent) {
    pos++; // skip for
    if (pos < t.size() && t[pos].type == Token::LPAREN) pos++;

    // Parse init: ident = expr
    std::string var;
    int init_val = 0;
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        var = t[pos].value; pos++;
        if (pos < t.size() && t[pos].type == Token::ASSIGN) { pos++;
            if (pos < t.size() && t[pos].type == Token::NUMBER) {
                init_val = (int)parse_verilog_number(t[pos].value); pos++;
            }
        }
    }
    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;

    // Parse condition: ident < N or ident <= N
    int limit = 0;
    bool less_eq = false;
    if (pos < t.size() && t[pos].type == Token::IDENT) pos++;
    if (pos < t.size() && (t[pos].type == Token::LT || t[pos].type == Token::LEQ)) {
        less_eq = (t[pos].type == Token::LEQ); pos++;
        if (pos < t.size() && t[pos].type == Token::NUMBER) {
            limit = (int)parse_verilog_number(t[pos].value); pos++;
        }
    }
    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;

    // Parse increment: ident = ident + 1 (skip to closing paren)
    int step = 1;
    while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
    if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;

    // Save body start position
    size_t body_start = pos;
    // Find body end to know its extent
    size_t body_end = pos;
    if (pos < t.size() && t[pos].type == Token::BEGIN_KW) {
        int depth = 1;
        body_end = pos + 1;
        while (body_end < t.size() && depth > 0) {
            if (t[body_end].type == Token::BEGIN_KW) depth++;
            if (t[body_end].type == Token::END_KW) depth--;
            body_end++;
        }
    } else {
        // Single statement body — find semicolon
        body_end = pos;
        while (body_end < t.size() && t[body_end].type != Token::SEMI) body_end++;
        if (body_end < t.size()) body_end++;
    }

    // Unroll: for each iteration, re-parse body with substituted loop var
    int end_val = less_eq ? limit + 1 : limit;
    if (end_val > 1024) end_val = 1024; // safety limit
    for (int iter = init_val; iter < end_val; iter += step) {
        params_[var] = iter;
        size_t bp = body_start;
        bp = parse_statement(t, bp, parent);
    }
    params_.erase(var);

    return body_end;
}


// ============================================================
// GENERATE-FOR — unroll at module level, creating structural elements
// ============================================================
size_t VerilogParser::parse_generate_for(const std::vector<Token>& t, size_t pos, Netlist& nl, VerilogParseResult& r) {
    pos++; // skip for
    if (pos < t.size() && t[pos].type == Token::LPAREN) pos++;

    // Parse init: ident = expr
    std::string var;
    int init_val = 0;
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        var = t[pos].value; pos++;
        if (pos < t.size() && t[pos].type == Token::ASSIGN) { pos++;
            size_t ep = pos;
            init_val = eval_const_expr(t, ep);
            pos = ep;
        }
    }
    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;

    // Parse condition
    int limit = 0;
    bool less_eq = false;
    if (pos < t.size() && t[pos].type == Token::IDENT) pos++;
    if (pos < t.size() && (t[pos].type == Token::LT || t[pos].type == Token::LEQ)) {
        less_eq = (t[pos].type == Token::LEQ); pos++;
        size_t ep = pos;
        limit = eval_const_expr(t, ep);
        pos = ep;
    }
    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;

    // Skip increment to closing paren
    while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
    if (pos < t.size()) pos++;

    // Find body (begin...end block)
    size_t body_start = pos;
    size_t body_end = pos;
    if (pos < t.size() && t[pos].type == Token::BEGIN_KW) {
        int depth = 1; body_end = pos + 1;
        while (body_end < t.size() && depth > 0) {
            if (t[body_end].type == Token::BEGIN_KW) depth++;
            if (t[body_end].type == Token::END_KW) depth--;
            body_end++;
        }
    }

    // Unroll: for each iteration, re-parse body as module-level constructs
    int end_val = less_eq ? limit + 1 : limit;
    if (end_val > 1024) end_val = 1024;
    for (int iter = init_val; iter < end_val; iter++) {
        params_[var] = iter;
        size_t bp = body_start;
        if (bp < t.size() && t[bp].type == Token::BEGIN_KW) bp++; // skip begin
        // Skip optional block label
        if (bp < t.size() && t[bp].type == Token::COLON) {
            bp++;
            if (bp < t.size() && t[bp].type == Token::IDENT) bp++;
        }
        // Parse module-body items inside the generate block
        while (bp < body_end - 1 && bp < t.size() && t[bp].type != Token::END_KW) {
            if (t[bp].type == Token::IDENT) {
                const std::string& gkw = t[bp].value;
                if (gkw == "assign") { bp = parse_assign(t, bp, nl, r); }
                else if (gkw == "wire" || gkw == "reg" ||
                         gkw == "tri" || gkw == "wand" || gkw == "wor" ||
                         gkw == "tri0" || gkw == "tri1" ||
                         gkw == "supply0" || gkw == "supply1") { bp = parse_port_decl(t, bp, nl, r, "wire"); }
                else if (gkw == "and") { bp = parse_gate_inst(t, bp, nl, r, GateType::AND); }
                else if (gkw == "or") { bp = parse_gate_inst(t, bp, nl, r, GateType::OR); }
                else if (gkw == "not") { bp = parse_gate_inst(t, bp, nl, r, GateType::NOT); }
                else if (gkw == "xor") { bp = parse_gate_inst(t, bp, nl, r, GateType::XOR); }
                else if (gkw == "buf") { bp = parse_gate_inst(t, bp, nl, r, GateType::BUF); }
                else { bp++; }
            } else if (t[bp].type == Token::ALWAYS) {
                bp = parse_always(t, bp, ast.root);
            } else { bp++; }
        }
    }
    params_.erase(var);
    return body_end;
}

// ============================================================
// GENERATE-IF — conditional generate
// ============================================================
size_t VerilogParser::parse_generate_if(const std::vector<Token>& t, size_t pos, Netlist& nl, VerilogParseResult& r) {
    pos++; // skip if
    if (pos < t.size() && t[pos].type == Token::LPAREN) pos++;
    size_t ep = pos;
    int cond_val = eval_const_expr(t, ep);
    pos = ep;
    if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;

    // Find true body
    size_t true_start = pos, true_end = pos;
    if (pos < t.size() && t[pos].type == Token::BEGIN_KW) {
        int depth = 1; true_end = pos + 1;
        while (true_end < t.size() && depth > 0) {
            if (t[true_end].type == Token::BEGIN_KW) depth++;
            if (t[true_end].type == Token::END_KW) depth--;
            true_end++;
        }
    } else {
        while (true_end < t.size() && t[true_end].type != Token::SEMI) true_end++;
        if (true_end < t.size()) true_end++;
    }

    // Check for else
    size_t false_start = true_end, false_end = true_end;
    if (true_end < t.size() && t[true_end].type == Token::ELSE) {
        false_start = true_end + 1;
        false_end = false_start;
        if (false_start < t.size() && t[false_start].type == Token::BEGIN_KW) {
            int depth = 1; false_end = false_start + 1;
            while (false_end < t.size() && depth > 0) {
                if (t[false_end].type == Token::BEGIN_KW) depth++;
                if (t[false_end].type == Token::END_KW) depth--;
                false_end++;
            }
        } else {
            while (false_end < t.size() && t[false_end].type != Token::SEMI) false_end++;
            if (false_end < t.size()) false_end++;
        }
    }

    // Only parse the active branch
    if (cond_val) {
        // Parse true body
        size_t bp = true_start;
        while (bp < true_end) {
            if (t[bp].type == Token::IDENT && t[bp].value == "assign") bp = parse_assign(t, bp, nl, r);
            else if (t[bp].type == Token::ALWAYS) bp = parse_always(t, bp, ast.root);
            else bp++;
        }
    } else if (false_end > false_start) {
        size_t bp = false_start;
        while (bp < false_end) {
            if (t[bp].type == Token::IDENT && t[bp].value == "assign") bp = parse_assign(t, bp, nl, r);
            else if (t[bp].type == Token::ALWAYS) bp = parse_always(t, bp, ast.root);
            else bp++;
        }
    }
    return false_end > true_end ? false_end : true_end;
}

// ============================================================
// FUNCTION DEFINITION — store for inlining
// ============================================================
size_t VerilogParser::parse_function_def(const std::vector<Token>& t, size_t pos) {
    pos++; // skip function
    FuncDef func;

    // Optional: 'automatic' keyword
    if (pos < t.size() && t[pos].type == Token::AUTOMATIC_KW) pos++;

    // Optional return type: [signed] [range]
    if (pos < t.size() && t[pos].type == Token::SIGNED_KW) pos++;
    if (pos < t.size() && t[pos].type == Token::LBRACKET) {
        func.return_range = parse_bus_range(t, pos);
    }
    if (pos < t.size() && t[pos].type == Token::INTEGER_KW) pos++;

    // Function name
    std::string fname;
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        fname = t[pos].value; pos++;
    }
    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;

    // Parse inputs and body until endfunction
    size_t body_start = 0;
    while (pos < t.size() && t[pos].type != Token::ENDFUNCTION_KW) {
        if (t[pos].type == Token::IDENT && t[pos].value == "input") {
            pos++; // skip input
            if (pos < t.size() && t[pos].type == Token::SIGNED_KW) pos++;
            if (pos < t.size() && t[pos].type == Token::LBRACKET) {
                parse_bus_range(t, pos); // skip range
            }
            while (pos < t.size() && t[pos].type != Token::SEMI) {
                if (t[pos].type == Token::IDENT)
                    func.param_names.push_back(t[pos].value);
                pos++;
            }
            if (pos < t.size()) pos++;
        } else if (t[pos].type == Token::IDENT && (t[pos].value == "reg" || t[pos].value == "wire")) {
            while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
            if (pos < t.size()) pos++;
        } else if (t[pos].type == Token::BEGIN_KW) {
            // Capture body tokens: find matching end, store assign expression
            pos++; // skip begin
            body_start = pos;
            // Find the assignment expression in the function body
            while (pos < t.size() && t[pos].type != Token::END_KW) {
                if (t[pos].type == Token::IDENT && t[pos].value == fname) {
                    // fname = expr; — this is the return value assignment
                    pos++; // skip fname
                    if (pos < t.size() && t[pos].type == Token::ASSIGN) {
                        pos++; // skip =
                        // Capture tokens until semicolon as the body expression
                        while (pos < t.size() && t[pos].type != Token::SEMI) {
                            func.body_tokens.push_back(t[pos]);
                            pos++;
                        }
                        func.body_tokens.push_back({Token::END, "", 0});
                        if (pos < t.size()) pos++;
                    }
                } else {
                    pos++;
                }
            }
            if (pos < t.size()) pos++; // skip end
        } else {
            // Single-line function body: fname = expr;
            if (t[pos].type == Token::IDENT && t[pos].value == fname) {
                pos++;
                if (pos < t.size() && t[pos].type == Token::ASSIGN) {
                    pos++;
                    while (pos < t.size() && t[pos].type != Token::SEMI) {
                        func.body_tokens.push_back(t[pos]);
                        pos++;
                    }
                    func.body_tokens.push_back({Token::END, "", 0});
                    if (pos < t.size()) pos++;
                }
            } else {
                pos++;
            }
        }
    }
    if (pos < t.size() && t[pos].type == Token::ENDFUNCTION_KW) pos++;

    if (!fname.empty()) functions_[fname] = func;
    return pos;
}

// ============================================================
// TASK DEFINITION — store for inlining (like functions but with I/O ports)
// ============================================================
size_t VerilogParser::parse_task_def(const std::vector<Token>& t, size_t pos) {
    pos++; // skip task
    // Skip optional 'automatic'
    if (pos < t.size() && t[pos].type == Token::AUTOMATIC_KW) pos++;

    FuncDef task_def;
    std::string tname;
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        tname = t[pos].value; pos++;
    }
    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;

    // Parse I/O declarations and collect body tokens
    bool in_body = false;
    while (pos < t.size() && t[pos].type != Token::ENDTASK_KW) {
        if (!in_body && t[pos].type == Token::IDENT &&
            (t[pos].value == "input" || t[pos].value == "output" || t[pos].value == "inout")) {
            pos++; // skip input/output/inout
            if (pos < t.size() && t[pos].type == Token::SIGNED_KW) pos++;
            if (pos < t.size() && t[pos].type == Token::LBRACKET) parse_bus_range(t, pos);
            while (pos < t.size() && t[pos].type != Token::SEMI) {
                if (t[pos].type == Token::IDENT)
                    task_def.param_names.push_back(t[pos].value);
                pos++;
            }
            if (pos < t.size()) pos++;
        } else if (!in_body && t[pos].type == Token::IDENT &&
                   (t[pos].value == "reg" || t[pos].value == "wire" || t[pos].value == "integer")) {
            while (pos < t.size() && t[pos].type != Token::SEMI) pos++;
            if (pos < t.size()) pos++;
        } else {
            // Body tokens
            in_body = true;
            task_def.body_tokens.push_back(t[pos]);
            pos++;
        }
    }
    task_def.body_tokens.push_back({Token::END, "", 0});
    if (pos < t.size()) pos++; // skip endtask

    if (!tname.empty()) tasks_[tname] = task_def;
    return pos;
}

// ============================================================
// GENERATE CASE — evaluate case expression, parse matching branch
// ============================================================
size_t VerilogParser::parse_generate_case(const std::vector<Token>& t, size_t pos, Netlist& nl, VerilogParseResult& r) {
    pos++; // skip case
    if (pos < t.size() && t[pos].type == Token::LPAREN) pos++;
    size_t ep = pos;
    int case_val = eval_const_expr(t, ep);
    pos = ep;
    if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;

    // Find matching case item
    bool matched = false;
    size_t default_start = 0, default_end = 0;
    while (pos < t.size() && t[pos].type != Token::ENDCASE_KW) {
        if (t[pos].type == Token::DEFAULT_KW) {
            pos++;
            if (pos < t.size() && t[pos].type == Token::COLON) pos++;
            default_start = pos;
            // Find end of default body
            if (pos < t.size() && t[pos].type == Token::BEGIN_KW) {
                int depth = 1; pos++;
                while (pos < t.size() && depth > 0) {
                    if (t[pos].type == Token::BEGIN_KW) depth++;
                    if (t[pos].type == Token::END_KW) depth--;
                    pos++;
                }
            } else {
                while (pos < t.size() && t[pos].type != Token::SEMI &&
                       t[pos].type != Token::ENDCASE_KW) pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            }
            default_end = pos;
        } else {
            // Case item value
            size_t vp = pos;
            int item_val = eval_const_expr(t, vp);
            pos = vp;
            if (pos < t.size() && t[pos].type == Token::COLON) pos++;

            size_t item_start = pos;
            // Find end of item body
            if (pos < t.size() && t[pos].type == Token::BEGIN_KW) {
                int depth = 1; pos++;
                while (pos < t.size() && depth > 0) {
                    if (t[pos].type == Token::BEGIN_KW) depth++;
                    if (t[pos].type == Token::END_KW) depth--;
                    pos++;
                }
            } else {
                while (pos < t.size() && t[pos].type != Token::SEMI &&
                       t[pos].type != Token::ENDCASE_KW) pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            }
            size_t item_end = pos;

            if (!matched && item_val == case_val) {
                matched = true;
                // Parse this branch
                size_t bp = item_start;
                while (bp < item_end) {
                    if (t[bp].type == Token::IDENT && t[bp].value == "assign")
                        bp = parse_assign(t, bp, nl, r);
                    else if (t[bp].type == Token::ALWAYS)
                        bp = parse_always(t, bp, ast.root);
                    else bp++;
                }
            }
        }
    }
    // If no match, parse default
    if (!matched && default_end > default_start) {
        size_t bp = default_start;
        while (bp < default_end) {
            if (t[bp].type == Token::IDENT && t[bp].value == "assign")
                bp = parse_assign(t, bp, nl, r);
            else if (t[bp].type == Token::ALWAYS)
                bp = parse_always(t, bp, ast.root);
            else bp++;
        }
    }
    if (pos < t.size() && t[pos].type == Token::ENDCASE_KW) pos++;
    return pos;
}

NetId StructuralSynthesizer::get_or_create(Netlist& nl, std::unordered_map<std::string, NetId>& net_map, const std::string& name) {
    auto it = net_map.find(name);
    if (it != net_map.end()) return it->second;
    NetId id = nl.add_net(name);
    net_map[name] = id;
    return id;
}

int StructuralSynthesizer::infer_width(std::shared_ptr<AstNode> expr,
                                        const std::unordered_map<std::string, std::pair<int,int>>& bus_ranges) {
    if (!expr) return 1;
    if (expr->type == AstNodeType::WIRE_DECL) {
        // Strip any bit-select suffix
        std::string name = expr->value;
        auto bracket = name.find('[');
        if (bracket != std::string::npos) return 1; // already bit-selected
        auto it = bus_ranges.find(name);
        if (it != bus_ranges.end()) return std::abs(it->second.first - it->second.second) + 1;
        return 1;
    }
    if (expr->type == AstNodeType::NUMBER_LITERAL) {
        // Check for width specifier: N'bXXX
        auto tick = expr->value.find('\'');
        if (tick != std::string::npos && tick > 0) {
            return std::stoi(expr->value.substr(0, tick));
        }
        return 1;
    }
    if (expr->type == AstNodeType::CONCAT) {
        int w = 0;
        for (auto& ch : expr->children) w += infer_width(ch, bus_ranges);
        return w;
    }
    if (expr->type == AstNodeType::REPLICATE) {
        int n = std::stoi(expr->value);
        if (!expr->children.empty()) return n * infer_width(expr->children[0], bus_ranges);
        return 1;
    }
    if (expr->type == AstNodeType::TERNARY_OP && expr->children.size() >= 3) {
        return std::max(infer_width(expr->children[1], bus_ranges), infer_width(expr->children[2], bus_ranges));
    }
    if (expr->type == AstNodeType::BIN_OP && expr->children.size() >= 2) {
        std::string op = expr->value;
        if (op == "<" || op == ">" || op == "<=" || op == ">=" || op == "==" || op == "!=" ||
            op == "&&" || op == "||") return 1;
        return std::max(infer_width(expr->children[0], bus_ranges), infer_width(expr->children[1], bus_ranges));
    }
    if (expr->type == AstNodeType::UNARY_OP) {
        std::string op = expr->value;
        if (op == "!" || op.substr(0,4) == "red_") return 1;
        if (!expr->children.empty()) return infer_width(expr->children[0], bus_ranges);
    }
    return 1;
}

// Single-bit expression synthesis
NetId StructuralSynthesizer::synth_expr(std::shared_ptr<AstNode> expr, Netlist& nl,
                                         std::unordered_map<std::string, NetId>& net_map) {
    if (!expr) return -1;

    if (expr->type == AstNodeType::WIRE_DECL) {
        return get_or_create(nl, net_map, expr->value);
    }
    if (expr->type == AstNodeType::NUMBER_LITERAL) {
        uint64_t val = parse_verilog_number(expr->value);
        GateType ct = (val & 1) ? GateType::CONST1 : GateType::CONST0;
        NetId out = nl.add_net("const_" + uid());
        nl.add_gate(ct, {}, out, "const_" + uid());
        return out;
    }
    if (expr->type == AstNodeType::UNARY_OP) {
        if (expr->children.empty()) return -1;
        NetId operand = synth_expr(expr->children[0], nl, net_map);
        if (expr->value == "~") {
            NetId out = nl.add_net("not_" + uid());
            nl.add_gate(GateType::NOT, {operand}, out, "not_" + uid());
            return out;
        }
        if (expr->value == "!") {
            // Logical NOT: same as bitwise NOT for 1-bit
            NetId out = nl.add_net("lnot_" + uid());
            nl.add_gate(GateType::NOT, {operand}, out, "lnot_" + uid());
            return out;
        }
        if (expr->value == "-") {
            // Unary minus: ~a + 1 (2's complement, single bit: just NOT)
            NetId out = nl.add_net("neg_" + uid());
            nl.add_gate(GateType::NOT, {operand}, out, "neg_" + uid());
            return out;
        }
        // Reduction operators just return the operand for 1-bit
        return operand;
    }
    if (expr->type == AstNodeType::TERNARY_OP && expr->children.size() >= 3) {
        NetId sel = synth_expr(expr->children[0], nl, net_map);
        NetId t_val = synth_expr(expr->children[1], nl, net_map);
        NetId f_val = synth_expr(expr->children[2], nl, net_map);
        // MUX: (sel & t) | (~sel & f)
        NetId ns = nl.add_net("mux_nsel_" + uid());
        nl.add_gate(GateType::NOT, {sel}, ns, "mux_inv_" + uid());
        NetId a1 = nl.add_net("mux_a1_" + uid());
        nl.add_gate(GateType::AND, {sel, t_val}, a1, "mux_and1_" + uid());
        NetId a0 = nl.add_net("mux_a0_" + uid());
        nl.add_gate(GateType::AND, {ns, f_val}, a0, "mux_and0_" + uid());
        NetId out = nl.add_net("mux_out_" + uid());
        nl.add_gate(GateType::OR, {a1, a0}, out, "mux_or_" + uid());
        return out;
    }
    if (expr->type == AstNodeType::BIN_OP && expr->children.size() >= 2) {
        NetId lhs = synth_expr(expr->children[0], nl, net_map);
        NetId rhs = synth_expr(expr->children[1], nl, net_map);
        std::string op = expr->value;
        if (op == "&") {
            NetId out = nl.add_net("and_" + uid());
            nl.add_gate(GateType::AND, {lhs, rhs}, out, "and_" + uid());
            return out;
        }
        if (op == "|") {
            NetId out = nl.add_net("or_" + uid());
            nl.add_gate(GateType::OR, {lhs, rhs}, out, "or_" + uid());
            return out;
        }
        if (op == "^") {
            NetId out = nl.add_net("xor_" + uid());
            nl.add_gate(GateType::XOR, {lhs, rhs}, out, "xor_" + uid());
            return out;
        }
        if (op == "+") {
            // Half adder sum
            NetId out = nl.add_net("sum_" + uid());
            nl.add_gate(GateType::XOR, {lhs, rhs}, out, "ha_sum_" + uid());
            return out;
        }
        if (op == "-") {
            // a - b (1-bit): a XOR b (same as +)
            NetId out = nl.add_net("sub_" + uid());
            nl.add_gate(GateType::XOR, {lhs, rhs}, out, "sub_" + uid());
            return out;
        }
        if (op == "*") {
            // 1-bit multiply = AND
            NetId out = nl.add_net("mul_" + uid());
            nl.add_gate(GateType::AND, {lhs, rhs}, out, "mul_" + uid());
            return out;
        }
        if (op == "==" ) {
            // XNOR
            NetId x = nl.add_net("eq_xor_" + uid());
            nl.add_gate(GateType::XOR, {lhs, rhs}, x, "eq_xor_" + uid());
            NetId out = nl.add_net("eq_" + uid());
            nl.add_gate(GateType::NOT, {x}, out, "eq_not_" + uid());
            return out;
        }
        if (op == "!=") {
            NetId out = nl.add_net("neq_" + uid());
            nl.add_gate(GateType::XOR, {lhs, rhs}, out, "neq_" + uid());
            return out;
        }
        if (op == "<") {
            // a < b (1-bit): ~a & b
            NetId na = nl.add_net("lt_na_" + uid());
            nl.add_gate(GateType::NOT, {lhs}, na, "lt_inv_" + uid());
            NetId out = nl.add_net("lt_" + uid());
            nl.add_gate(GateType::AND, {na, rhs}, out, "lt_and_" + uid());
            return out;
        }
        if (op == ">") {
            // a > b (1-bit): a & ~b
            NetId nb = nl.add_net("gt_nb_" + uid());
            nl.add_gate(GateType::NOT, {rhs}, nb, "gt_inv_" + uid());
            NetId out = nl.add_net("gt_" + uid());
            nl.add_gate(GateType::AND, {lhs, nb}, out, "gt_and_" + uid());
            return out;
        }
        if (op == "<=" || op == ">=") {
            // a <= b: ~(a > b) = ~(a & ~b)
            // a >= b: ~(a < b) = ~(~a & b)
            NetId inv_arg = (op == "<=") ? rhs : lhs;
            NetId other_arg = (op == "<=") ? lhs : rhs;
            NetId ni = nl.add_net("cmp_ni_" + uid());
            nl.add_gate(GateType::NOT, {inv_arg}, ni, "cmp_inv_" + uid());
            NetId lt = nl.add_net("cmp_lt_" + uid());
            nl.add_gate(GateType::AND, {ni, other_arg}, lt, "cmp_and_" + uid());
            NetId out = nl.add_net("cmp_" + uid());
            nl.add_gate(GateType::NOT, {lt}, out, "cmp_not_" + uid());
            return out;
        }
        if (op == "&&") {
            NetId out = nl.add_net("land_" + uid());
            nl.add_gate(GateType::AND, {lhs, rhs}, out, "land_" + uid());
            return out;
        }
        if (op == "||") {
            NetId out = nl.add_net("lor_" + uid());
            nl.add_gate(GateType::OR, {lhs, rhs}, out, "lor_" + uid());
            return out;
        }
        // Shifts, division etc. — for 1-bit, shift by 0 = identity, else 0
        // Fall through to buffer
        return lhs;
    }
    if (expr->type == AstNodeType::CONCAT) {
        // For single-bit output, just return first child
        if (!expr->children.empty()) return synth_expr(expr->children[0], nl, net_map);
        return -1;
    }
    return -1;
}


// Multi-bit bus expression synthesis
std::vector<NetId> StructuralSynthesizer::synth_expr_bus(
    std::shared_ptr<AstNode> expr, Netlist& nl,
    std::unordered_map<std::string, NetId>& net_map,
    const std::unordered_map<std::string, std::pair<int,int>>& bus_ranges,
    int target_width) {

    if (!expr) return {};
    int w = infer_width(expr, bus_ranges);
    if (w <= 1 && target_width <= 1) {
        return {synth_expr(expr, nl, net_map)};
    }
    int out_w = std::max(w, target_width);

    // Wire/identifier
    if (expr->type == AstNodeType::WIRE_DECL) {
        std::string name = expr->value;
        auto it = bus_ranges.find(name);
        if (it != bus_ranges.end()) {
            int lo = std::min(it->second.first, it->second.second);
            int hi = std::max(it->second.first, it->second.second);
            std::vector<NetId> bits;
            for (int i = lo; i <= hi; i++) {
                bits.push_back(get_or_create(nl, net_map, name + "[" + std::to_string(i) + "]"));
            }
            // Zero-extend
            while ((int)bits.size() < out_w) {
                NetId z = nl.add_net("zext_" + uid());
                nl.add_gate(GateType::CONST0, {}, z, "zext_" + uid());
                bits.push_back(z);
            }
            return bits;
        }
        // Single-bit signal
        std::vector<NetId> bits = {get_or_create(nl, net_map, name)};
        while ((int)bits.size() < out_w) {
            NetId z = nl.add_net("zext_" + uid());
            nl.add_gate(GateType::CONST0, {}, z, "zext_" + uid());
            bits.push_back(z);
        }
        return bits;
    }

    // Number literal
    if (expr->type == AstNodeType::NUMBER_LITERAL) {
        uint64_t val = parse_verilog_number(expr->value);
        std::vector<NetId> bits;
        for (int i = 0; i < out_w; i++) {
            GateType ct = ((val >> i) & 1) ? GateType::CONST1 : GateType::CONST0;
            NetId b = nl.add_net("const_" + uid());
            nl.add_gate(ct, {}, b, "const_" + uid());
            bits.push_back(b);
        }
        return bits;
    }

    // Concatenation
    if (expr->type == AstNodeType::CONCAT) {
        // Verilog concat: {MSB, ..., LSB} — first child is MSB
        std::vector<std::vector<NetId>> parts;
        for (auto& ch : expr->children) {
            int cw = infer_width(ch, bus_ranges);
            parts.push_back(synth_expr_bus(ch, nl, net_map, bus_ranges, cw));
        }
        // Assemble LSB first: reverse order (last child = LSB)
        std::vector<NetId> result;
        for (int pi = (int)parts.size()-1; pi >= 0; pi--) {
            for (auto nid : parts[pi]) result.push_back(nid);
        }
        while ((int)result.size() < out_w) {
            NetId z = nl.add_net("zext_" + uid());
            nl.add_gate(GateType::CONST0, {}, z, "zext_" + uid());
            result.push_back(z);
        }
        return result;
    }

    // Replication
    if (expr->type == AstNodeType::REPLICATE && !expr->children.empty()) {
        int n = std::stoi(expr->value);
        int cw = infer_width(expr->children[0], bus_ranges);
        auto part = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, cw);
        std::vector<NetId> result;
        for (int rep = 0; rep < n; rep++) {
            for (auto nid : part) result.push_back(nid);
        }
        return result;
    }

    // Ternary
    if (expr->type == AstNodeType::TERNARY_OP && expr->children.size() >= 3) {
        NetId sel = synth_expr(expr->children[0], nl, net_map);
        auto t_bits = synth_expr_bus(expr->children[1], nl, net_map, bus_ranges, out_w);
        auto f_bits = synth_expr_bus(expr->children[2], nl, net_map, bus_ranges, out_w);
        std::vector<NetId> result;
        NetId nsel = nl.add_net("mux_nsel_" + uid());
        nl.add_gate(GateType::NOT, {sel}, nsel, "mux_inv_" + uid());
        for (int i = 0; i < out_w; i++) {
            NetId t_bit = (i < (int)t_bits.size()) ? t_bits[i] : nl.add_net("z_" + uid());
            NetId f_bit = (i < (int)f_bits.size()) ? f_bits[i] : nl.add_net("z_" + uid());
            NetId a1 = nl.add_net("mux_a1_" + uid());
            nl.add_gate(GateType::AND, {sel, t_bit}, a1, "mux_and1_" + uid());
            NetId a0 = nl.add_net("mux_a0_" + uid());
            nl.add_gate(GateType::AND, {nsel, f_bit}, a0, "mux_and0_" + uid());
            NetId out = nl.add_net("mux_" + uid());
            nl.add_gate(GateType::OR, {a1, a0}, out, "mux_or_" + uid());
            result.push_back(out);
        }
        return result;
    }

    // Unary
    if (expr->type == AstNodeType::UNARY_OP && !expr->children.empty()) {
        if (expr->value == "~") {
            auto operand = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, out_w);
            std::vector<NetId> result;
            for (int i = 0; i < out_w && i < (int)operand.size(); i++) {
                NetId out = nl.add_net("not_" + uid());
                nl.add_gate(GateType::NOT, {operand[i]}, out, "not_" + uid());
                result.push_back(out);
            }
            return result;
        }
        if (expr->value == "-") {
            // 2's complement: ~x + 1
            auto operand = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, out_w);
            // Invert all bits
            std::vector<NetId> inv;
            for (int i = 0; i < out_w && i < (int)operand.size(); i++) {
                NetId n = nl.add_net("neg_inv_" + uid());
                nl.add_gate(GateType::NOT, {operand[i]}, n, "neg_inv_" + uid());
                inv.push_back(n);
            }
            // Add 1 (ripple carry)
            NetId one = nl.add_net("neg_one_" + uid());
            nl.add_gate(GateType::CONST1, {}, one, "neg_one_" + uid());
            // Ripple carry add with constant 1
            std::vector<NetId> result;
            NetId carry = one;
            for (int i = 0; i < (int)inv.size(); i++) {
                NetId s = nl.add_net("neg_s_" + uid());
                nl.add_gate(GateType::XOR, {inv[i], carry}, s, "neg_xor_" + uid());
                result.push_back(s);
                NetId c = nl.add_net("neg_c_" + uid());
                nl.add_gate(GateType::AND, {inv[i], carry}, c, "neg_and_" + uid());
                carry = c;
            }
            return result;
        }
        // Reduction operators -> single bit result
        if (expr->value.substr(0,4) == "red_") {
            auto operand = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges,
                                           infer_width(expr->children[0], bus_ranges));
            if (operand.empty()) return {synth_expr(expr, nl, net_map)};
            std::string rop = expr->value.substr(4);
            GateType gt = (rop == "&") ? GateType::AND : (rop == "|") ? GateType::OR : GateType::XOR;
            NetId result = operand[0];
            for (int i = 1; i < (int)operand.size(); i++) {
                NetId out = nl.add_net("red_" + uid());
                nl.add_gate(gt, {result, operand[i]}, out, "red_" + uid());
                result = out;
            }
            // Return 1-bit result, zero-extend
            std::vector<NetId> res = {result};
            while ((int)res.size() < out_w) {
                NetId z = nl.add_net("zext_" + uid());
                nl.add_gate(GateType::CONST0, {}, z, "zext_" + uid());
                res.push_back(z);
            }
            return res;
        }
        // Logical NOT
        if (expr->value == "!") {
            auto operand = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges,
                                           infer_width(expr->children[0], bus_ranges));
            // OR all bits, then NOT
            if (operand.empty()) return {};
            NetId any = operand[0];
            for (int i = 1; i < (int)operand.size(); i++) {
                NetId out = nl.add_net("lor_" + uid());
                nl.add_gate(GateType::OR, {any, operand[i]}, out, "lor_" + uid());
                any = out;
            }
            NetId result = nl.add_net("lnot_" + uid());
            nl.add_gate(GateType::NOT, {any}, result, "lnot_" + uid());
            std::vector<NetId> res = {result};
            while ((int)res.size() < out_w) {
                NetId z = nl.add_net("zext_" + uid());
                nl.add_gate(GateType::CONST0, {}, z, "zext_" + uid());
                res.push_back(z);
            }
            return res;
        }
    }

    // Binary operations on buses
    if (expr->type == AstNodeType::BIN_OP && expr->children.size() >= 2) {
        std::string op = expr->value;
        int lw = infer_width(expr->children[0], bus_ranges);
        int rw = infer_width(expr->children[1], bus_ranges);
        int max_w = std::max(lw, rw);
        if (max_w < out_w) max_w = out_w;

        // Bitwise ops: &, |, ^
        if (op == "&" || op == "|" || op == "^") {
            auto l_bits = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, max_w);
            auto r_bits = synth_expr_bus(expr->children[1], nl, net_map, bus_ranges, max_w);
            GateType gt = (op == "&") ? GateType::AND : (op == "|") ? GateType::OR : GateType::XOR;
            std::vector<NetId> result;
            for (int i = 0; i < out_w; i++) {
                NetId a = (i < (int)l_bits.size()) ? l_bits[i] : nl.add_net("z_" + uid());
                NetId b = (i < (int)r_bits.size()) ? r_bits[i] : nl.add_net("z_" + uid());
                NetId out = nl.add_net(op + "_" + uid());
                nl.add_gate(gt, {a, b}, out, op + "_" + uid());
                result.push_back(out);
            }
            return result;
        }

        // Addition: ripple carry adder
        if (op == "+") {
            auto l_bits = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, max_w);
            auto r_bits = synth_expr_bus(expr->children[1], nl, net_map, bus_ranges, max_w);
            std::vector<NetId> result;
            NetId carry = -1;
            for (int i = 0; i < out_w; i++) {
                NetId a = (i < (int)l_bits.size()) ? l_bits[i] : nl.add_net("z_" + uid());
                NetId b = (i < (int)r_bits.size()) ? r_bits[i] : nl.add_net("z_" + uid());
                if (carry < 0) {
                    NetId s = nl.add_net("ha_s_" + uid());
                    nl.add_gate(GateType::XOR, {a, b}, s, "ha_xor_" + uid());
                    result.push_back(s);
                    carry = nl.add_net("ha_c_" + uid());
                    nl.add_gate(GateType::AND, {a, b}, carry, "ha_and_" + uid());
                } else {
                    NetId x1 = nl.add_net("fa_x1_" + uid());
                    nl.add_gate(GateType::XOR, {a, b}, x1, "fa_xor1_" + uid());
                    NetId s = nl.add_net("fa_s_" + uid());
                    nl.add_gate(GateType::XOR, {x1, carry}, s, "fa_xor2_" + uid());
                    result.push_back(s);
                    NetId a1 = nl.add_net("fa_a1_" + uid());
                    nl.add_gate(GateType::AND, {a, b}, a1, "fa_and1_" + uid());
                    NetId a2 = nl.add_net("fa_a2_" + uid());
                    nl.add_gate(GateType::AND, {x1, carry}, a2, "fa_and2_" + uid());
                    carry = nl.add_net("fa_c_" + uid());
                    nl.add_gate(GateType::OR, {a1, a2}, carry, "fa_or_" + uid());
                }
            }
            return result;
        }

        // Subtraction: a + (~b) + 1 via ripple carry
        if (op == "-") {
            auto l_bits = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, max_w);
            auto r_bits = synth_expr_bus(expr->children[1], nl, net_map, bus_ranges, max_w);
            // Invert b
            std::vector<NetId> r_inv;
            for (int i = 0; i < max_w && i < (int)r_bits.size(); i++) {
                NetId n = nl.add_net("sub_inv_" + uid());
                nl.add_gate(GateType::NOT, {r_bits[i]}, n, "sub_inv_" + uid());
                r_inv.push_back(n);
            }
            while ((int)r_inv.size() < max_w) {
                // ~0 = 1 for zero-extended bits
                NetId n = nl.add_net("sub_one_" + uid());
                nl.add_gate(GateType::CONST1, {}, n, "sub_one_" + uid());
                r_inv.push_back(n);
            }
            // Ripple carry add with initial carry-in = 1
            NetId one = nl.add_net("sub_cin_" + uid());
            nl.add_gate(GateType::CONST1, {}, one, "sub_cin_" + uid());
            std::vector<NetId> result;
            NetId carry = one;
            for (int i = 0; i < out_w; i++) {
                NetId a = (i < (int)l_bits.size()) ? l_bits[i] : nl.add_net("z_" + uid());
                NetId b = (i < (int)r_inv.size()) ? r_inv[i] : nl.add_net("z_" + uid());
                NetId x1 = nl.add_net("sub_x1_" + uid());
                nl.add_gate(GateType::XOR, {a, b}, x1, "sub_xor1_" + uid());
                NetId s = nl.add_net("sub_s_" + uid());
                nl.add_gate(GateType::XOR, {x1, carry}, s, "sub_xor2_" + uid());
                result.push_back(s);
                NetId a1 = nl.add_net("sub_a1_" + uid());
                nl.add_gate(GateType::AND, {a, b}, a1, "sub_and1_" + uid());
                NetId a2 = nl.add_net("sub_a2_" + uid());
                nl.add_gate(GateType::AND, {x1, carry}, a2, "sub_and2_" + uid());
                carry = nl.add_net("sub_c_" + uid());
                nl.add_gate(GateType::OR, {a1, a2}, carry, "sub_or_" + uid());
            }
            return result;
        }

        // Multiplication: shift-add
        if (op == "*") {
            auto l_bits = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, out_w);
            auto r_bits = synth_expr_bus(expr->children[1], nl, net_map, bus_ranges, out_w);
            // Partial products with ripple carry addition
            std::vector<NetId> accum;
            for (int i = 0; i < out_w; i++) {
                NetId z = nl.add_net("mul_z_" + uid());
                nl.add_gate(GateType::CONST0, {}, z, "mul_z_" + uid());
                accum.push_back(z);
            }
            for (int j = 0; j < std::min((int)r_bits.size(), out_w); j++) {
                // Generate partial product: l_bits[i] & r_bits[j], shifted by j
                std::vector<NetId> pp;
                for (int i = 0; i < out_w; i++) {
                    int src_bit = i - j;
                    if (src_bit >= 0 && src_bit < (int)l_bits.size()) {
                        NetId p = nl.add_net("pp_" + uid());
                        nl.add_gate(GateType::AND, {l_bits[src_bit], r_bits[j]}, p, "pp_and_" + uid());
                        pp.push_back(p);
                    } else {
                        NetId z = nl.add_net("pp_z_" + uid());
                        nl.add_gate(GateType::CONST0, {}, z, "pp_z_" + uid());
                        pp.push_back(z);
                    }
                }
                // Add pp to accum
                std::vector<NetId> new_accum;
                NetId carry = -1;
                for (int i = 0; i < out_w; i++) {
                    if (carry < 0) {
                        NetId s = nl.add_net("mul_s_" + uid());
                        nl.add_gate(GateType::XOR, {accum[i], pp[i]}, s, "mul_xor_" + uid());
                        new_accum.push_back(s);
                        carry = nl.add_net("mul_c_" + uid());
                        nl.add_gate(GateType::AND, {accum[i], pp[i]}, carry, "mul_and_" + uid());
                    } else {
                        NetId x1 = nl.add_net("mul_x1_" + uid());
                        nl.add_gate(GateType::XOR, {accum[i], pp[i]}, x1, "mul_xor1_" + uid());
                        NetId s = nl.add_net("mul_s_" + uid());
                        nl.add_gate(GateType::XOR, {x1, carry}, s, "mul_xor2_" + uid());
                        new_accum.push_back(s);
                        NetId a1 = nl.add_net("mul_a1_" + uid());
                        nl.add_gate(GateType::AND, {accum[i], pp[i]}, a1, "mul_and1_" + uid());
                        NetId a2 = nl.add_net("mul_a2_" + uid());
                        nl.add_gate(GateType::AND, {x1, carry}, a2, "mul_and2_" + uid());
                        carry = nl.add_net("mul_c_" + uid());
                        nl.add_gate(GateType::OR, {a1, a2}, carry, "mul_or_" + uid());
                    }
                }
                accum = new_accum;
            }
            return accum;
        }

        // Left shift by constant
        if (op == "<<") {
            auto l_bits = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, out_w);
            int amount = 0;
            if (expr->children[1]->type == AstNodeType::NUMBER_LITERAL) {
                amount = expr->children[1]->int_val;
            }
            std::vector<NetId> result;
            for (int i = 0; i < out_w; i++) {
                int src = i - amount;
                if (src >= 0 && src < (int)l_bits.size()) {
                    result.push_back(l_bits[src]);
                } else {
                    NetId z = nl.add_net("shl_z_" + uid());
                    nl.add_gate(GateType::CONST0, {}, z, "shl_z_" + uid());
                    result.push_back(z);
                }
            }
            return result;
        }

        // Right shift by constant
        if (op == ">>") {
            auto l_bits = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, out_w);
            int amount = 0;
            if (expr->children[1]->type == AstNodeType::NUMBER_LITERAL) {
                amount = expr->children[1]->int_val;
            }
            std::vector<NetId> result;
            for (int i = 0; i < out_w; i++) {
                int src = i + amount;
                if (src < (int)l_bits.size()) {
                    result.push_back(l_bits[src]);
                } else {
                    NetId z = nl.add_net("shr_z_" + uid());
                    nl.add_gate(GateType::CONST0, {}, z, "shr_z_" + uid());
                    result.push_back(z);
                }
            }
            return result;
        }

        // Comparison operators — result is 1-bit, zero-extended
        if (op == "==" || op == "!=" || op == "<" || op == ">" || op == "<=" || op == ">=") {
            int cmp_w = std::max(lw, rw);
            auto l_bits = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, cmp_w);
            auto r_bits = synth_expr_bus(expr->children[1], nl, net_map, bus_ranges, cmp_w);

            NetId result = -1;
            if (op == "==" || op == "!=") {
                // XOR each pair, then NOR all
                NetId any_diff = -1;
                for (int i = 0; i < cmp_w; i++) {
                    NetId x = nl.add_net("cmp_x_" + uid());
                    nl.add_gate(GateType::XOR, {l_bits[i], r_bits[i]}, x, "cmp_xor_" + uid());
                    if (any_diff < 0) any_diff = x;
                    else {
                        NetId o = nl.add_net("cmp_or_" + uid());
                        nl.add_gate(GateType::OR, {any_diff, x}, o, "cmp_or_" + uid());
                        any_diff = o;
                    }
                }
                if (op == "==") {
                    result = nl.add_net("eq_" + uid());
                    nl.add_gate(GateType::NOT, {any_diff}, result, "eq_not_" + uid());
                } else {
                    result = any_diff;
                }
            } else {
                // < > <= >= : subtract and check sign bit
                // a < b: compute a - b, check if MSB (borrow out) is set
                // Using subtraction chain from b
                std::vector<NetId> r_inv;
                for (int i = 0; i < cmp_w; i++) {
                    NetId n = nl.add_net("cmp_inv_" + uid());
                    nl.add_gate(GateType::NOT, {r_bits[i]}, n, "cmp_inv_" + uid());
                    r_inv.push_back(n);
                }
                NetId cin = nl.add_net("cmp_cin_" + uid());
                nl.add_gate(GateType::CONST1, {}, cin, "cmp_cin_" + uid());
                NetId carry = cin;
                NetId last_sum = -1;
                for (int i = 0; i < cmp_w; i++) {
                    NetId x1 = nl.add_net("cmp_x1_" + uid());
                    nl.add_gate(GateType::XOR, {l_bits[i], r_inv[i]}, x1, "cmp_xor1_" + uid());
                    last_sum = nl.add_net("cmp_s_" + uid());
                    nl.add_gate(GateType::XOR, {x1, carry}, last_sum, "cmp_xor2_" + uid());
                    NetId a1 = nl.add_net("cmp_a1_" + uid());
                    nl.add_gate(GateType::AND, {l_bits[i], r_inv[i]}, a1, "cmp_and1_" + uid());
                    NetId a2 = nl.add_net("cmp_a2_" + uid());
                    nl.add_gate(GateType::AND, {x1, carry}, a2, "cmp_and2_" + uid());
                    carry = nl.add_net("cmp_c_" + uid());
                    nl.add_gate(GateType::OR, {a1, a2}, carry, "cmp_or_" + uid());
                }
                // For unsigned: a < b iff borrow (carry out NOT set after a - b) 
                // carry = 1 means no borrow, carry = 0 means borrow
                NetId not_carry = nl.add_net("cmp_nc_" + uid());
                nl.add_gate(GateType::NOT, {carry}, not_carry, "cmp_nc_" + uid());

                if (op == "<") result = not_carry;
                else if (op == ">=") result = carry;
                else if (op == ">") {
                    // a > b iff b < a: swap operands conceptually
                    // carry is from a-b, not_carry means a < b
                    // a > b means carry=1 AND result != 0
                    // Actually: a > b = ~(a <= b) = ~(a < b | a == b)
                    // Simpler: a > b iff carry=1 AND (a != b)
                    NetId any_diff = -1;
                    for (int i = 0; i < cmp_w; i++) {
                        NetId x = nl.add_net("gt_x_" + uid());
                        nl.add_gate(GateType::XOR, {l_bits[i], r_bits[i]}, x, "gt_xor_" + uid());
                        if (any_diff < 0) any_diff = x;
                        else {
                            NetId o = nl.add_net("gt_or_" + uid());
                            nl.add_gate(GateType::OR, {any_diff, x}, o, "gt_or_" + uid());
                            any_diff = o;
                        }
                    }
                    result = nl.add_net("gt_" + uid());
                    nl.add_gate(GateType::AND, {carry, any_diff}, result, "gt_and_" + uid());
                }
                else if (op == "<=") {
                    // a <= b = ~(a > b)
                    NetId any_diff = -1;
                    for (int i = 0; i < cmp_w; i++) {
                        NetId x = nl.add_net("le_x_" + uid());
                        nl.add_gate(GateType::XOR, {l_bits[i], r_bits[i]}, x, "le_xor_" + uid());
                        if (any_diff < 0) any_diff = x;
                        else {
                            NetId o = nl.add_net("le_or_" + uid());
                            nl.add_gate(GateType::OR, {any_diff, x}, o, "le_or_" + uid());
                            any_diff = o;
                        }
                    }
                    NetId gt = nl.add_net("le_gt_" + uid());
                    nl.add_gate(GateType::AND, {carry, any_diff}, gt, "le_and_" + uid());
                    result = nl.add_net("le_" + uid());
                    nl.add_gate(GateType::NOT, {gt}, result, "le_not_" + uid());
                }
            }

            // Zero-extend to out_w
            std::vector<NetId> res = {result};
            while ((int)res.size() < out_w) {
                NetId z = nl.add_net("zext_" + uid());
                nl.add_gate(GateType::CONST0, {}, z, "zext_" + uid());
                res.push_back(z);
            }
            return res;
        }

        // Logical AND/OR: 1-bit result
        if (op == "&&" || op == "||") {
            auto l_bits = synth_expr_bus(expr->children[0], nl, net_map, bus_ranges, lw);
            auto r_bits = synth_expr_bus(expr->children[1], nl, net_map, bus_ranges, rw);
            // Reduce each to 1-bit (OR of all bits)
            auto reduce_or = [&](const std::vector<NetId>& bits) -> NetId {
                if (bits.empty()) return -1;
                NetId r = bits[0];
                for (int i = 1; i < (int)bits.size(); i++) {
                    NetId o = nl.add_net("lor_" + uid());
                    nl.add_gate(GateType::OR, {r, bits[i]}, o, "lor_" + uid());
                    r = o;
                }
                return r;
            };
            NetId la = reduce_or(l_bits);
            NetId ra = reduce_or(r_bits);
            GateType gt = (op == "&&") ? GateType::AND : GateType::OR;
            NetId result = nl.add_net("log_" + uid());
            nl.add_gate(gt, {la, ra}, result, "log_" + uid());
            std::vector<NetId> res = {result};
            while ((int)res.size() < out_w) {
                NetId z = nl.add_net("zext_" + uid());
                nl.add_gate(GateType::CONST0, {}, z, "zext_" + uid());
                res.push_back(z);
            }
            return res;
        }
    }

    // Fallback: single-bit synthesis with zero-extension
    NetId single = synth_expr(expr, nl, net_map);
    std::vector<NetId> res;
    if (single >= 0) res.push_back(single);
    while ((int)res.size() < out_w) {
        NetId z = nl.add_net("zext_" + uid());
        nl.add_gate(GateType::CONST0, {}, z, "zext_" + uid());
        res.push_back(z);
    }
    return res;
}

} // namespace sf
