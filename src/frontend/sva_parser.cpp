// SiliconForge — SystemVerilog Assertions (SVA) Parser
#include "frontend/sva_parser.hpp"
#include <sstream>
#include <iostream>

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
        if (c == '(' || c == ')' || c == ';' || c == '@') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            tokens.push_back(std::string(1, c));
        } else if (c == '|' && i + 2 < src.length() && src[i+1] == '-' && src[i+2] == '>') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            tokens.push_back("|->"); i += 2;
        } else if (c == '|' && i + 2 < src.length() && src[i+1] == '=' && src[i+2] == '>') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            tokens.push_back("|=>"); i += 2;
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) tokens.push_back(cur);
    return tokens;
}

std::shared_ptr<SvaNode> SvaParser::parse_expr(const std::vector<std::string>& tokens, size_t& pos) {
    if (pos >= tokens.size()) return nullptr;
    
    // Very simplified: expects A |-> B or A |=> B
    std::string antecedent = tokens[pos++];
    if (pos >= tokens.size()) return nullptr;
    
    std::string op_tok = tokens[pos++];
    SvaOp op = SvaOp::LITERAL;
    if (op_tok == "|->") op = SvaOp::PROP_OVERLAPPING;
    else if (op_tok == "|=>") op = SvaOp::PROP_NON_OVERLAPPING;
    else throw std::runtime_error("SVA Parser: Expected |-> or |=>, got " + op_tok);
    
    if (pos >= tokens.size()) return nullptr;
    std::string consequent = tokens[pos++];
    
    auto root = std::make_shared<SvaNode>();
    root->op = op;
    
    root->left = std::make_shared<SvaNode>();
    root->left->op = SvaOp::LITERAL;
    root->left->literal = antecedent;
    
    root->right = std::make_shared<SvaNode>();
    root->right->op = SvaOp::LITERAL;
    root->right->literal = consequent;
    
    return root;
}

std::vector<SvaProperty> SvaParser::parse(const std::string& source) {
    std::vector<SvaProperty> props;
    auto tokens = tokenize(source);
    
    size_t i = 0;
    while (i < tokens.size()) {
        if (tokens[i] == "assert" && i + 1 < tokens.size() && tokens[i+1] == "property") {
            i += 2; // skip assert property
            SvaProperty prop;
            prop.name = "P_" + std::to_string(props.size());
            
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
            
            // Re-sync basic AST parsing bounds for simplicity
            size_t start_expr = i;
            while (i < tokens.size() && tokens[i] != ")" && tokens[i] != ";") {
                i++;
            }
            
            size_t p = start_expr;
            prop.expr = parse_expr(tokens, p);
            
            while (i < tokens.size() && tokens[i] != ";") i++;
            if (i < tokens.size() && tokens[i] == ";") i++;
            
            if (prop.expr) props.push_back(prop);
        } else {
            i++;
        }
    }
    return props;
}

} // namespace sf
