// SiliconForge — SystemVerilog Assertions (SVA) Parser
// Supports: |-> |=> ##N always eventually until && || ! [*N]
// Also: assert property, assume property, cover property
#include "frontend/sva_parser.hpp"
#include <sstream>
#include <iostream>
#include <cctype>

namespace sf {

std::vector<std::string> SvaParser::tokenize(const std::string& src) const {
    std::vector<std::string> tokens;
    std::string cur;
    for (size_t i = 0; i < src.length(); ++i) {
        char c = src[i];
        if (std::isspace(c)) {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            continue;
        }
        if (c == '(' || c == ')' || c == ';' || c == '@' || c == '!') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            tokens.push_back(std::string(1, c));
        } else if (c == '|' && i + 2 < src.length() && src[i+1] == '-' && src[i+2] == '>') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            tokens.push_back("|->"); i += 2;
        } else if (c == '|' && i + 2 < src.length() && src[i+1] == '=' && src[i+2] == '>') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            tokens.push_back("|=>"); i += 2;
        } else if (c == '|' && i + 1 < src.length() && src[i+1] == '|') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            tokens.push_back("||"); i += 1;
        } else if (c == '&' && i + 1 < src.length() && src[i+1] == '&') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            tokens.push_back("&&"); i += 1;
        } else if (c == '#' && i + 1 < src.length() && src[i+1] == '#') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            // ##N cycle delay
            i += 2;
            std::string num;
            while (i < src.length() && std::isdigit(src[i])) num += src[i++];
            tokens.push_back("##" + (num.empty() ? "1" : num));
            i--; // back up for loop increment
        } else if (c == '[' && i + 1 < src.length() && src[i+1] == '*') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            i += 2;
            std::string num;
            while (i < src.length() && std::isdigit(src[i])) num += src[i++];
            if (i < src.length() && src[i] == ']') i++;
            tokens.push_back("[*" + (num.empty() ? "1" : num) + "]");
            i--;
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) tokens.push_back(cur);
    return tokens;
}

std::shared_ptr<SvaNode> SvaParser::parse_expr(const std::vector<std::string>& tokens, size_t& pos) {
    if (pos >= tokens.size()) return nullptr;

    // Handle NOT
    if (tokens[pos] == "!") {
        pos++;
        auto node = std::make_shared<SvaNode>();
        node->op = SvaOp::NOT;
        node->left = parse_expr(tokens, pos);
        return node;
    }

    // Handle always/eventually
    if (tokens[pos] == "always" || tokens[pos] == "G") {
        pos++;
        auto node = std::make_shared<SvaNode>();
        node->op = SvaOp::GLOBALLY;
        node->left = parse_expr(tokens, pos);
        return node;
    }
    if (tokens[pos] == "eventually" || tokens[pos] == "F" || tokens[pos] == "s_eventually") {
        pos++;
        auto node = std::make_shared<SvaNode>();
        node->op = SvaOp::EVENTUALLY;
        node->left = parse_expr(tokens, pos);
        return node;
    }
    if (tokens[pos] == "nexttime" || tokens[pos] == "X") {
        pos++;
        auto node = std::make_shared<SvaNode>();
        node->op = SvaOp::NEXT;
        node->delay_cycles = 1;
        node->left = parse_expr(tokens, pos);
        return node;
    }

    // Handle parenthesized expression
    if (tokens[pos] == "(") {
        pos++;
        auto inner = parse_expr(tokens, pos);
        if (pos < tokens.size() && tokens[pos] == ")") pos++;
        // Check for binary ops after closing paren
        if (pos < tokens.size()) {
            return parse_binary(tokens, pos, inner);
        }
        return inner;
    }

    // Literal
    auto lit = std::make_shared<SvaNode>();
    lit->op = SvaOp::LITERAL;
    lit->literal = tokens[pos++];

    // Check for binary operator following
    if (pos < tokens.size()) {
        return parse_binary(tokens, pos, lit);
    }
    return lit;
}

std::shared_ptr<SvaNode> SvaParser::parse_binary(
    const std::vector<std::string>& tokens, size_t& pos,
    std::shared_ptr<SvaNode> left) {

    if (pos >= tokens.size()) return left;
    const std::string& tok = tokens[pos];

    SvaOp op;
    if (tok == "|->")      op = SvaOp::PROP_OVERLAPPING;
    else if (tok == "|=>") op = SvaOp::PROP_NON_OVERLAPPING;
    else if (tok == "&&")  op = SvaOp::AND;
    else if (tok == "||")  op = SvaOp::OR;
    else if (tok == "until") op = SvaOp::UNTIL;
    else if (tok.substr(0, 2) == "##") {
        pos++;
        auto node = std::make_shared<SvaNode>();
        node->op = SvaOp::DELAY;
        node->delay_cycles = std::stoi(tok.substr(2));
        node->left = left;
        node->right = parse_expr(tokens, pos);
        return node;
    } else if (tok.size() > 2 && tok[0] == '[' && tok[1] == '*') {
        pos++;
        auto node = std::make_shared<SvaNode>();
        node->op = SvaOp::REPEAT;
        node->delay_cycles = std::stoi(tok.substr(2, tok.size() - 3));
        node->left = left;
        return node;
    } else {
        return left; // not a binary op we recognize
    }

    pos++;
    auto node = std::make_shared<SvaNode>();
    node->op = op;
    node->left = left;
    node->right = parse_expr(tokens, pos);
    return node;
}

std::vector<SvaProperty> SvaParser::parse(const std::string& source) {
    std::vector<SvaProperty> props;
    sequences_.clear();
    auto tokens = tokenize(source);

    size_t i = 0;
    while (i < tokens.size()) {
        bool is_assert = false, is_assume = false, is_cover = false;

        // Recognize sequence declarations
        if (tokens[i] == "sequence" && i + 1 < tokens.size()) {
            parse_sequence_decl(tokens, i);
            continue;
        }

        if (tokens[i] == "assert" && i + 1 < tokens.size() && tokens[i+1] == "property") {
            is_assert = true; i += 2;
        } else if (tokens[i] == "assume" && i + 1 < tokens.size() && tokens[i+1] == "property") {
            is_assume = true; i += 2;
        } else if (tokens[i] == "cover" && i + 1 < tokens.size() && tokens[i+1] == "property") {
            is_cover = true; i += 2;
        } else {
            i++; continue;
        }

        SvaProperty prop;
        prop.name = "P_" + std::to_string(props.size());
        prop.is_assert = is_assert;
        prop.is_assume = is_assume;
        prop.is_cover  = is_cover;

        // Expected: ( @ ( posedge clk ) expr ) ;
        if (i < tokens.size() && tokens[i] == "(") i++;
        if (i < tokens.size() && tokens[i] == "@") {
            i++;
            if (i < tokens.size() && tokens[i] == "(") i++;
            if (i < tokens.size() && tokens[i] == "posedge") i++;
            if (i < tokens.size()) {
                prop.clock_domain = tokens[i++];
            }
            if (i < tokens.size() && tokens[i] == ")") i++;
        }

        // Parse expression up to closing ) or ;
        size_t start_expr = i;
        int depth = 0;
        size_t end_expr = i;
        while (end_expr < tokens.size()) {
            if (tokens[end_expr] == "(") depth++;
            if (tokens[end_expr] == ")") {
                if (depth == 0) break;
                depth--;
            }
            if (tokens[end_expr] == ";") break;
            end_expr++;
        }

        size_t p = start_expr;
        prop.expr = parse_expr(tokens, p);

        i = end_expr;
        while (i < tokens.size() && tokens[i] != ";") i++;
        if (i < tokens.size() && tokens[i] == ";") i++;

        if (prop.expr) props.push_back(prop);
    }
    return props;
}

void SvaParser::parse_sequence_decl(const std::vector<std::string>& tokens, size_t& pos) {
    // sequence name (args) ; body ; endsequence
    pos++; // skip 'sequence'
    SvaSequence seq;
    if (pos < tokens.size()) seq.name = tokens[pos++];

    // Parse optional formal arguments in parentheses
    if (pos < tokens.size() && tokens[pos] == "(") {
        pos++;
        while (pos < tokens.size() && tokens[pos] != ")") {
            if (tokens[pos] != ",") seq.arguments.push_back(tokens[pos]);
            pos++;
        }
        if (pos < tokens.size()) pos++; // skip ')'
    }
    if (pos < tokens.size() && tokens[pos] == ";") pos++;

    // Collect expression body until endsequence
    std::string expr;
    while (pos < tokens.size() && tokens[pos] != "endsequence") {
        if (tokens[pos] == ";") { pos++; continue; }
        if (!expr.empty()) expr += " ";
        expr += tokens[pos++];
    }
    if (pos < tokens.size()) pos++; // skip 'endsequence'

    seq.expression = expr;
    sequences_.push_back(std::move(seq));
}

} // namespace sf
