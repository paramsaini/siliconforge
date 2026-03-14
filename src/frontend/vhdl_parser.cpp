// SiliconForge — VHDL 2008 Parser Implementation
// Converts VHDL entity/architecture/process into gate-level Netlist.

#include "frontend/vhdl_parser.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace sf {

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------

std::string VhdlParser::to_lower(const std::string& s) const {
    std::string r = s;
    std::transform(r.begin(), r.end(), r.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return r;
}

bool VhdlParser::is_keyword(const Token& tok, const std::string& kw) const {
    if (tok.type != Token::IDENT) return false;
    return to_lower(tok.text) == kw;
}

NetId VhdlParser::lookup_or_create(const std::string& name, Netlist& nl) {
    std::string low = to_lower(name);
    auto it = name_map_.find(low);
    if (it != name_map_.end()) return it->second;
    NetId id = nl.add_net(low);
    name_map_[low] = id;
    return id;
}

void VhdlParser::skip_until_semi(const std::vector<Token>& t, size_t& pos) {
    while (pos < t.size() && t[pos].type != Token::SEMI)
        ++pos;
    if (pos < t.size()) ++pos; // consume ';'
}

// ---------------------------------------------------------------------------
// Tokenizer — VHDL is case-insensitive
// ---------------------------------------------------------------------------

std::vector<VhdlParser::Token> VhdlParser::tokenize(const std::string& src) {
    std::vector<Token> tokens;
    int line = 1;
    size_t i = 0;
    while (i < src.size()) {
        char c = src[i];

        // Newlines
        if (c == '\n') { ++line; ++i; continue; }
        if (std::isspace(static_cast<unsigned char>(c))) { ++i; continue; }

        // Single-line comment: --
        if (c == '-' && i + 1 < src.size() && src[i + 1] == '-') {
            while (i < src.size() && src[i] != '\n') ++i;
            continue;
        }

        // String literal
        if (c == '"') {
            size_t start = i++;
            while (i < src.size() && src[i] != '"') ++i;
            if (i < src.size()) ++i;
            tokens.push_back({Token::STRING_LIT,
                               src.substr(start, i - start), line});
            continue;
        }

        // Character literal: 'X'
        if (c == '\'' && i + 2 < src.size() && src[i + 2] == '\'') {
            tokens.push_back({Token::STRING_LIT,
                               src.substr(i, 3), line});
            i += 3;
            continue;
        }

        // Signal assignment <=
        if (c == '<' && i + 1 < src.size() && src[i + 1] == '=') {
            tokens.push_back({Token::ASSIGN_SIG, "<=", line});
            i += 2;
            continue;
        }

        // Variable assignment :=
        if (c == ':' && i + 1 < src.size() && src[i + 1] == '=') {
            tokens.push_back({Token::ASSIGN_VAR, ":=", line});
            i += 2;
            continue;
        }

        // Arrow =>
        if (c == '=' && i + 1 < src.size() && src[i + 1] == '>') {
            tokens.push_back({Token::ARROW, "=>", line});
            i += 2;
            continue;
        }

        // Single-character tokens
        switch (c) {
            case '(': tokens.push_back({Token::LPAREN, "(", line}); ++i; continue;
            case ')': tokens.push_back({Token::RPAREN, ")", line}); ++i; continue;
            case ';': tokens.push_back({Token::SEMI, ";", line}); ++i; continue;
            case ':': tokens.push_back({Token::COLON, ":", line}); ++i; continue;
            case ',': tokens.push_back({Token::COMMA, ",", line}); ++i; continue;
            case '<': tokens.push_back({Token::LT, "<", line}); ++i; continue;
            case '>': tokens.push_back({Token::GT, ">", line}); ++i; continue;
            case '&': tokens.push_back({Token::AMP, "&", line}); ++i; continue;
            case '|': tokens.push_back({Token::BAR, "|", line}); ++i; continue;
            case '.': tokens.push_back({Token::DOT, ".", line}); ++i; continue;
            case '\'': tokens.push_back({Token::TICK, "'", line}); ++i; continue;
            case '=': tokens.push_back({Token::EQ, "=", line}); ++i; continue;
            case '#': tokens.push_back({Token::HASH, "#", line}); ++i; continue;
            default: break;
        }

        // Numbers
        if (std::isdigit(static_cast<unsigned char>(c))) {
            size_t start = i;
            while (i < src.size() &&
                   (std::isdigit(static_cast<unsigned char>(src[i])) || src[i] == '_'))
                ++i;
            tokens.push_back({Token::NUMBER, src.substr(start, i - start), line});
            continue;
        }

        // Identifiers / keywords
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            size_t start = i;
            while (i < src.size() &&
                   (std::isalnum(static_cast<unsigned char>(src[i])) || src[i] == '_'))
                ++i;
            tokens.push_back({Token::IDENT, src.substr(start, i - start), line});
            continue;
        }

        // Skip unknown characters
        ++i;
    }
    tokens.push_back({Token::EOF_TOK, "", line});
    return tokens;
}

// ---------------------------------------------------------------------------
// parse_string / parse_file
// ---------------------------------------------------------------------------

VhdlParseResult VhdlParser::parse_string(const std::string& src, Netlist& nl) {
    VhdlParseResult r;
    ports_.clear();
    signals_.clear();
    name_map_.clear();
    gate_counter_ = 0;
    dff_counter_ = 0;
    nl_ = &nl;

    try {
        auto tokens = tokenize(src);
        size_t pos = 0;
        while (pos < tokens.size() && tokens[pos].type != Token::EOF_TOK) {
            // Skip library/use declarations
            if (is_keyword(tokens[pos], "library") ||
                is_keyword(tokens[pos], "use")) {
                skip_until_semi(tokens, pos);
                continue;
            }
            if (is_keyword(tokens[pos], "entity")) {
                parse_entity(tokens, pos, nl, r);
                continue;
            }
            if (is_keyword(tokens[pos], "architecture")) {
                parse_architecture(tokens, pos, nl, r);
                continue;
            }
            ++pos;
        }
        r.num_gates = static_cast<int>(nl.num_gates());
        r.success = true;
    } catch (const std::exception& e) {
        r.success = false;
        r.error = e.what();
    }
    return r;
}

VhdlParseResult VhdlParser::parse_file(const std::string& filename, Netlist& nl) {
    std::ifstream f(filename);
    if (!f.is_open()) {
        VhdlParseResult r;
        r.error = "Cannot open file: " + filename;
        return r;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return parse_string(ss.str(), nl);
}

// ---------------------------------------------------------------------------
// parse_entity
// ---------------------------------------------------------------------------

void VhdlParser::parse_entity(const std::vector<Token>& t, size_t& pos,
                               Netlist& nl, VhdlParseResult& r) {
    ++pos; // skip 'entity'
    if (pos >= t.size()) return;
    r.entity_name = t[pos].text;
    ++pos;

    // expect 'is'
    if (pos < t.size() && is_keyword(t[pos], "is")) ++pos;

    // Look for 'port'
    while (pos < t.size() && !is_keyword(t[pos], "port") &&
           !is_keyword(t[pos], "end")) {
        ++pos;
    }

    if (pos < t.size() && is_keyword(t[pos], "port")) {
        ++pos; // skip 'port'
        if (pos < t.size() && t[pos].type == Token::LPAREN) ++pos;

        // Parse port list: name {, name} : direction type ;
        while (pos < t.size() && t[pos].type != Token::RPAREN &&
               t[pos].type != Token::EOF_TOK) {

            // Collect port names
            std::vector<std::string> names;
            while (pos < t.size() && t[pos].type == Token::IDENT &&
                   !is_keyword(t[pos], "in") && !is_keyword(t[pos], "out") &&
                   !is_keyword(t[pos], "inout") && !is_keyword(t[pos], "buffer")) {
                names.push_back(t[pos].text);
                ++pos;
                if (pos < t.size() && t[pos].type == Token::COMMA) ++pos;
                if (pos < t.size() && t[pos].type == Token::COLON) { ++pos; break; }
            }

            if (names.empty()) { ++pos; continue; }

            // Direction
            VhdlPort::Direction dir = VhdlPort::IN;
            if (pos < t.size()) {
                std::string d = to_lower(t[pos].text);
                if (d == "in") { dir = VhdlPort::IN; ++pos; }
                else if (d == "out") { dir = VhdlPort::OUT; ++pos; }
                else if (d == "inout") { dir = VhdlPort::INOUT; ++pos; }
                else if (d == "buffer") { dir = VhdlPort::BUFFER; ++pos; }
            }

            // Type — std_logic or std_logic_vector(N downto 0)
            std::string type_name;
            int width = 1;
            if (pos < t.size() && t[pos].type == Token::IDENT) {
                type_name = to_lower(t[pos].text);
                ++pos;

                // Check for _vector suffix: std_logic_vector
                if (pos < t.size() && t[pos].type == Token::IDENT) {
                    // Handle case where tokenizer splits std_logic_vector into parts
                    // (won't happen since underscores are part of IDENT)
                }

                // Parse (N downto 0) or (0 to N)
                if (pos < t.size() && t[pos].type == Token::LPAREN) {
                    ++pos;
                    int hi = 0, lo = 0;
                    if (pos < t.size() && t[pos].type == Token::NUMBER) {
                        hi = std::stoi(t[pos].text);
                        ++pos;
                    }
                    // downto or to
                    if (pos < t.size() && t[pos].type == Token::IDENT) ++pos;
                    if (pos < t.size() && t[pos].type == Token::NUMBER) {
                        lo = std::stoi(t[pos].text);
                        ++pos;
                    }
                    if (pos < t.size() && t[pos].type == Token::RPAREN) ++pos;
                    width = (hi >= lo) ? (hi - lo + 1) : (lo - hi + 1);
                }
            }

            // Create ports and nets
            for (auto& name : names) {
                VhdlPort port{name, dir, width, type_name};
                ports_.push_back(port);

                NetId id = lookup_or_create(name, nl);
                if (dir == VhdlPort::IN) {
                    nl.mark_input(id);
                    r.num_inputs++;
                } else if (dir == VhdlPort::OUT || dir == VhdlPort::BUFFER) {
                    nl.mark_output(id);
                    r.num_outputs++;
                } else if (dir == VhdlPort::INOUT) {
                    nl.mark_input(id);
                    nl.mark_output(id);
                    r.num_inputs++;
                    r.num_outputs++;
                }
            }

            // Skip to semicolon or next port
            if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
        }

        // Skip closing )
        if (pos < t.size() && t[pos].type == Token::RPAREN) ++pos;
        // Skip ;
        if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
    }

    // Skip to 'end'
    while (pos < t.size() && !is_keyword(t[pos], "end")) ++pos;
    if (pos < t.size()) ++pos; // skip 'end'
    // Skip optional 'entity' and entity name
    while (pos < t.size() && t[pos].type != Token::SEMI) ++pos;
    if (pos < t.size()) ++pos; // skip ';'
}

// ---------------------------------------------------------------------------
// parse_architecture
// ---------------------------------------------------------------------------

void VhdlParser::parse_architecture(const std::vector<Token>& t, size_t& pos,
                                     Netlist& nl, VhdlParseResult& r) {
    ++pos; // skip 'architecture'
    if (pos >= t.size()) return;
    ++pos; // skip arch name

    // skip 'of ENTITY is'
    while (pos < t.size() && !is_keyword(t[pos], "is")) ++pos;
    if (pos < t.size()) ++pos; // skip 'is'

    // Declarative region: signal declarations, component declarations
    while (pos < t.size() && !is_keyword(t[pos], "begin") &&
           !is_keyword(t[pos], "end")) {
        if (is_keyword(t[pos], "signal")) {
            parse_signal_decl(t, pos, nl, r);
        } else if (is_keyword(t[pos], "component")) {
            // Skip component declarations
            while (pos < t.size() && !is_keyword(t[pos], "end")) ++pos;
            if (pos < t.size()) ++pos; // 'end'
            if (pos < t.size() && is_keyword(t[pos], "component")) ++pos;
            if (pos < t.size() && t[pos].type == Token::IDENT) ++pos;
            if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
        } else {
            ++pos;
        }
    }

    if (pos < t.size() && is_keyword(t[pos], "begin")) ++pos;

    // Statement region: concurrent statements
    while (pos < t.size() && !is_keyword(t[pos], "end")) {
        // Process block
        if (is_keyword(t[pos], "process")) {
            parse_process(t, pos, nl, r);
            continue;
        }

        // Look ahead for label : or signal assignment
        // label : component_name port map (...)
        // sig <= expr ;
        if (t[pos].type == Token::IDENT) {
            // Check for label pattern: IDENT : IDENT port map
            if (pos + 1 < t.size() && t[pos + 1].type == Token::COLON) {
                std::string label = t[pos].text;
                pos += 2; // skip label and ':'

                if (is_keyword(t[pos], "process")) {
                    parse_process(t, pos, nl, r);
                    continue;
                }

                // Component instantiation: label : comp_name port map (...)
                if (pos < t.size() && t[pos].type == Token::IDENT &&
                    !is_keyword(t[pos], "process")) {
                    parse_component_inst(t, pos, nl, r, label);
                    continue;
                }
            }

            // Signal assignment: name <= ...
            if (pos + 1 < t.size() && t[pos + 1].type == Token::ASSIGN_SIG) {
                parse_signal_assignment(t, pos, nl, r);
                continue;
            }
        }
        ++pos;
    }

    // Skip 'end [architecture] [name] ;'
    if (pos < t.size()) ++pos; // 'end'
    while (pos < t.size() && t[pos].type != Token::SEMI) ++pos;
    if (pos < t.size()) ++pos; // ';'
}

// ---------------------------------------------------------------------------
// parse_signal_decl: signal name [, name] : type ;
// ---------------------------------------------------------------------------

void VhdlParser::parse_signal_decl(const std::vector<Token>& t, size_t& pos,
                                    Netlist& nl, VhdlParseResult& r) {
    ++pos; // skip 'signal'

    std::vector<std::string> names;
    while (pos < t.size() && t[pos].type == Token::IDENT) {
        names.push_back(t[pos].text);
        ++pos;
        if (pos < t.size() && t[pos].type == Token::COMMA) ++pos;
        if (pos < t.size() && t[pos].type == Token::COLON) { ++pos; break; }
    }

    std::string type_name;
    int width = 1;
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        type_name = to_lower(t[pos].text);
        ++pos;
        if (pos < t.size() && t[pos].type == Token::LPAREN) {
            ++pos;
            int hi = 0, lo = 0;
            if (pos < t.size() && t[pos].type == Token::NUMBER) {
                hi = std::stoi(t[pos].text);
                ++pos;
            }
            if (pos < t.size() && t[pos].type == Token::IDENT) ++pos;
            if (pos < t.size() && t[pos].type == Token::NUMBER) {
                lo = std::stoi(t[pos].text);
                ++pos;
            }
            if (pos < t.size() && t[pos].type == Token::RPAREN) ++pos;
            width = (hi >= lo) ? (hi - lo + 1) : (lo - hi + 1);
        }
    }

    for (auto& name : names) {
        VhdlSignal sig{name, width, type_name};
        signals_.push_back(sig);
        lookup_or_create(name, nl);
        r.num_signals++;
    }

    if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
}

// ---------------------------------------------------------------------------
// parse_signal_assignment: target <= expr ;
//   Handles: a and b, a or b, not a, a xor b, etc.
//   Conditional: z <= a when cond else b ;
// ---------------------------------------------------------------------------

void VhdlParser::parse_signal_assignment(const std::vector<Token>& t, size_t& pos,
                                          Netlist& nl, VhdlParseResult& r) {
    std::string target = t[pos].text;
    NetId out = lookup_or_create(target, nl);
    pos += 2; // skip name and '<='

    // Collect RHS tokens until ';'
    std::vector<Token> rhs;
    while (pos < t.size() && t[pos].type != Token::SEMI) {
        rhs.push_back(t[pos]);
        ++pos;
    }
    if (pos < t.size()) ++pos; // skip ';'

    if (rhs.empty()) return;

    // Check for conditional assignment: ... when ... = ... else ...
    int when_pos = -1;
    for (size_t i = 0; i < rhs.size(); ++i) {
        if (is_keyword(rhs[i], "when")) { when_pos = (int)i; break; }
    }

    if (when_pos >= 0) {
        // Conditional: val_true when sel = 'X' else val_false
        // Collect true value operands
        std::vector<NetId> true_nets;
        for (int i = 0; i < when_pos; ++i) {
            if (rhs[i].type == Token::IDENT)
                true_nets.push_back(lookup_or_create(rhs[i].text, nl));
        }

        // Find the selector
        NetId sel = -1;
        size_t wi = when_pos + 1;
        if (wi < rhs.size() && rhs[wi].type == Token::IDENT) {
            sel = lookup_or_create(rhs[wi].text, nl);
        }

        // Find 'else' and get false value
        int else_pos = -1;
        for (size_t i = when_pos; i < rhs.size(); ++i) {
            if (is_keyword(rhs[i], "else")) { else_pos = (int)i; break; }
        }

        std::vector<NetId> false_nets;
        if (else_pos >= 0) {
            for (size_t i = else_pos + 1; i < rhs.size(); ++i) {
                if (rhs[i].type == Token::IDENT)
                    false_nets.push_back(lookup_or_create(rhs[i].text, nl));
            }
        }

        NetId true_val = true_nets.empty() ? -1 : true_nets[0];
        NetId false_val = false_nets.empty() ? -1 : false_nets[0];

        if (sel >= 0 && true_val >= 0 && false_val >= 0) {
            std::string name = "mux_" + std::to_string(gate_counter_++);
            nl.add_gate(GateType::MUX, {sel, true_val, false_val}, out, name);
        }
        return;
    }

    // Check for concatenation: a & b
    bool has_concat = false;
    for (auto& tok : rhs) {
        if (tok.type == Token::AMP) { has_concat = true; break; }
    }

    if (has_concat) {
        // Treat as BUF of first operand for flat netlist
        if (rhs[0].type == Token::IDENT) {
            NetId in = lookup_or_create(rhs[0].text, nl);
            std::string name = "buf_" + std::to_string(gate_counter_++);
            nl.add_gate(GateType::BUF, {in}, out, name);
        }
        return;
    }

    // Simple unary: not a
    if (rhs.size() >= 2 && is_keyword(rhs[0], "not")) {
        NetId in = lookup_or_create(rhs[1].text, nl);
        std::string name = "not_" + std::to_string(gate_counter_++);
        nl.add_gate(GateType::NOT, {in}, out, name);
        return;
    }

    // Binary operator: a OP b [OP c ...]
    // Collect operands and operators
    std::vector<NetId> operands;
    GateType gtype = GateType::BUF;
    bool has_op = false;

    for (size_t i = 0; i < rhs.size(); ++i) {
        if (rhs[i].type == Token::IDENT) {
            std::string low = to_lower(rhs[i].text);
            if (low == "and") { gtype = GateType::AND; has_op = true; }
            else if (low == "or") { gtype = GateType::OR; has_op = true; }
            else if (low == "xor") { gtype = GateType::XOR; has_op = true; }
            else if (low == "nand") { gtype = GateType::NAND; has_op = true; }
            else if (low == "nor") { gtype = GateType::NOR; has_op = true; }
            else if (low == "xnor") { gtype = GateType::XNOR; has_op = true; }
            else if (low == "not") { /* handled above */ }
            else {
                operands.push_back(lookup_or_create(rhs[i].text, nl));
            }
        }
    }

    if (has_op && operands.size() >= 2) {
        std::string name = std::string(gate_type_str(gtype)) + "_" +
                           std::to_string(gate_counter_++);
        std::transform(name.begin(), name.end(), name.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        nl.add_gate(gtype, operands, out, name);
    } else if (operands.size() == 1) {
        std::string name = "buf_" + std::to_string(gate_counter_++);
        nl.add_gate(GateType::BUF, operands, out, name);
    }
}

// ---------------------------------------------------------------------------
// parse_process: process (sensitivity_list) begin ... end process;
// ---------------------------------------------------------------------------

void VhdlParser::parse_process(const std::vector<Token>& t, size_t& pos,
                                Netlist& nl, VhdlParseResult& r) {
    ++pos; // skip 'process'
    r.num_processes++;

    // Parse optional sensitivity list
    std::vector<std::string> sensitivity;
    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        ++pos;
        while (pos < t.size() && t[pos].type != Token::RPAREN) {
            if (t[pos].type == Token::IDENT)
                sensitivity.push_back(to_lower(t[pos].text));
            ++pos;
        }
        if (pos < t.size()) ++pos; // skip ')'
    }

    // Skip optional 'is'
    if (pos < t.size() && is_keyword(t[pos], "is")) ++pos;

    // Skip variable declarations
    while (pos < t.size() && !is_keyword(t[pos], "begin") &&
           !is_keyword(t[pos], "end")) {
        if (is_keyword(t[pos], "variable")) {
            skip_until_semi(t, pos);
        } else {
            ++pos;
        }
    }

    if (pos < t.size() && is_keyword(t[pos], "begin")) ++pos;

    // Process body: parse if/case statements
    int depth = 1;
    while (pos < t.size() && depth > 0) {
        if (is_keyword(t[pos], "end")) {
            // Check for 'end process' or 'end if' or 'end case'
            if (pos + 1 < t.size() && is_keyword(t[pos + 1], "process")) {
                pos += 2; // skip 'end process'
                if (pos < t.size() && t[pos].type == Token::IDENT) ++pos;
                if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
                depth = 0;
                break;
            }
            if (pos + 1 < t.size() && is_keyword(t[pos + 1], "if")) {
                pos += 2;
                if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
                continue;
            }
            if (pos + 1 < t.size() && is_keyword(t[pos + 1], "case")) {
                pos += 2;
                if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
                continue;
            }
            // Plain 'end ;'
            ++pos;
            if (pos < t.size() && t[pos].type == Token::SEMI) { ++pos; depth = 0; }
            break;
        }

        // if rising_edge(clk) then Q <= D; end if;
        if (is_keyword(t[pos], "if")) {
            ++pos;

            // Check for rising_edge / falling_edge → DFF
            if (pos < t.size() && (is_keyword(t[pos], "rising_edge") ||
                                    is_keyword(t[pos], "falling_edge"))) {
                std::string edge_fn = to_lower(t[pos].text);
                ++pos;
                // Parse (clk)
                NetId clk_net = -1;
                if (pos < t.size() && t[pos].type == Token::LPAREN) {
                    ++pos;
                    if (pos < t.size() && t[pos].type == Token::IDENT) {
                        clk_net = lookup_or_create(t[pos].text, nl);
                        ++pos;
                    }
                    if (pos < t.size() && t[pos].type == Token::RPAREN) ++pos;
                }

                // skip 'then'
                if (pos < t.size() && is_keyword(t[pos], "then")) ++pos;

                // Parse assignments until 'end if' or 'elsif' or 'else'
                while (pos < t.size() && !is_keyword(t[pos], "end") &&
                       !is_keyword(t[pos], "elsif") && !is_keyword(t[pos], "else")) {
                    if (t[pos].type == Token::IDENT &&
                        pos + 1 < t.size() && t[pos + 1].type == Token::ASSIGN_SIG) {
                        std::string q_name = t[pos].text;
                        NetId q = lookup_or_create(q_name, nl);
                        pos += 2; // skip name and '<='
                        if (pos < t.size() && t[pos].type == Token::IDENT) {
                            NetId d = lookup_or_create(t[pos].text, nl);
                            std::string name = "dff_" + std::to_string(dff_counter_++);
                            nl.add_dff(d, clk_net, q, -1, name);
                            ++pos;
                        }
                        if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
                    } else {
                        ++pos;
                    }
                }
                continue;
            }

            // Regular if/else → MUX
            // Collect condition signal
            NetId cond = -1;
            if (pos < t.size() && t[pos].type == Token::IDENT) {
                cond = lookup_or_create(t[pos].text, nl);
            }

            // Skip to 'then'
            while (pos < t.size() && !is_keyword(t[pos], "then")) ++pos;
            if (pos < t.size()) ++pos; // skip 'then'

            // Parse true branch assignment
            NetId true_val = -1;
            NetId mux_out = -1;
            while (pos < t.size() && !is_keyword(t[pos], "else") &&
                   !is_keyword(t[pos], "elsif") && !is_keyword(t[pos], "end")) {
                if (t[pos].type == Token::IDENT &&
                    pos + 1 < t.size() && t[pos + 1].type == Token::ASSIGN_SIG) {
                    mux_out = lookup_or_create(t[pos].text, nl);
                    pos += 2;
                    if (pos < t.size() && t[pos].type == Token::IDENT) {
                        true_val = lookup_or_create(t[pos].text, nl);
                        ++pos;
                    }
                    if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
                } else {
                    ++pos;
                }
            }

            // Parse else branch
            NetId false_val = -1;
            if (pos < t.size() && is_keyword(t[pos], "else")) {
                ++pos;
                while (pos < t.size() && !is_keyword(t[pos], "end")) {
                    if (t[pos].type == Token::IDENT &&
                        pos + 1 < t.size() && t[pos + 1].type == Token::ASSIGN_SIG) {
                        pos += 2;
                        if (pos < t.size() && t[pos].type == Token::IDENT) {
                            false_val = lookup_or_create(t[pos].text, nl);
                            ++pos;
                        }
                        if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
                    } else {
                        ++pos;
                    }
                }
            }

            if (cond >= 0 && true_val >= 0 && false_val >= 0 && mux_out >= 0) {
                std::string name = "mux_" + std::to_string(gate_counter_++);
                nl.add_gate(GateType::MUX, {cond, true_val, false_val}, mux_out, name);
            }
            continue;
        }

        // case sel is when "00" => z <= a; when "01" => z <= b; ...
        if (is_keyword(t[pos], "case")) {
            ++pos;
            NetId sel = -1;
            if (pos < t.size() && t[pos].type == Token::IDENT) {
                sel = lookup_or_create(t[pos].text, nl);
                ++pos;
            }
            // skip 'is'
            if (pos < t.size() && is_keyword(t[pos], "is")) ++pos;

            // Collect case arms: when value => target <= source;
            NetId case_out = -1;
            std::vector<NetId> case_vals;
            while (pos < t.size() && !is_keyword(t[pos], "end")) {
                if (is_keyword(t[pos], "when")) {
                    ++pos;
                    // Skip the match value (string literal, "others", etc.)
                    if (pos < t.size()) ++pos;
                    // skip '=>'
                    if (pos < t.size() && t[pos].type == Token::ARROW) ++pos;

                    // Parse assignment
                    if (pos < t.size() && t[pos].type == Token::IDENT &&
                        pos + 1 < t.size() && t[pos + 1].type == Token::ASSIGN_SIG) {
                        if (case_out < 0)
                            case_out = lookup_or_create(t[pos].text, nl);
                        pos += 2;
                        if (pos < t.size() && t[pos].type == Token::IDENT) {
                            case_vals.push_back(lookup_or_create(t[pos].text, nl));
                            ++pos;
                        }
                        if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
                    }
                } else {
                    ++pos;
                }
            }

            // Build MUX tree from case values
            if (sel >= 0 && case_out >= 0 && case_vals.size() >= 2) {
                // Pair-wise MUX tree
                NetId current = case_vals.back();
                for (int i = (int)case_vals.size() - 2; i >= 0; --i) {
                    NetId tmp = (i == 0) ? case_out : nl.add_net("case_tmp_" +
                                std::to_string(gate_counter_));
                    std::string name = "mux_" + std::to_string(gate_counter_++);
                    nl.add_gate(GateType::MUX, {sel, case_vals[i], current}, tmp, name);
                    current = tmp;
                }
            }

            // Skip 'end case ;'
            if (pos < t.size() && is_keyword(t[pos], "end")) {
                ++pos;
                if (pos < t.size() && is_keyword(t[pos], "case")) ++pos;
                if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
            }
            continue;
        }

        // Simple signal assignment within process
        if (t[pos].type == Token::IDENT &&
            pos + 1 < t.size() && t[pos + 1].type == Token::ASSIGN_SIG) {
            // Reuse concurrent assignment parser logic
            std::string target = t[pos].text;
            NetId out = lookup_or_create(target, nl);
            pos += 2;
            if (pos < t.size() && t[pos].type == Token::IDENT) {
                NetId in = lookup_or_create(t[pos].text, nl);
                std::string name = "buf_" + std::to_string(gate_counter_++);
                nl.add_gate(GateType::BUF, {in}, out, name);
                ++pos;
            }
            if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
            continue;
        }

        ++pos;
    }
}

// ---------------------------------------------------------------------------
// parse_component_inst: comp_name port map ( port => signal, ... );
// ---------------------------------------------------------------------------

void VhdlParser::parse_component_inst(const std::vector<Token>& t, size_t& pos,
                                       Netlist& nl, VhdlParseResult& r,
                                       const std::string& label) {
    std::string comp_name;
    if (pos < t.size() && t[pos].type == Token::IDENT) {
        comp_name = t[pos].text;
        ++pos;
    }

    // Skip to 'port'
    while (pos < t.size() && !is_keyword(t[pos], "port") &&
           t[pos].type != Token::SEMI) {
        ++pos;
    }

    if (pos < t.size() && is_keyword(t[pos], "port")) {
        ++pos; // skip 'port'
        if (pos < t.size() && is_keyword(t[pos], "map")) ++pos;

        if (pos < t.size() && t[pos].type == Token::LPAREN) {
            ++pos;

            // Parse port connections: formal => actual
            std::vector<std::pair<std::string, std::string>> connections;
            while (pos < t.size() && t[pos].type != Token::RPAREN) {
                if (t[pos].type == Token::IDENT) {
                    std::string formal = t[pos].text;
                    ++pos;
                    if (pos < t.size() && t[pos].type == Token::ARROW) {
                        ++pos;
                        if (pos < t.size() && t[pos].type == Token::IDENT) {
                            connections.push_back({formal, t[pos].text});
                            ++pos;
                        }
                    }
                }
                if (pos < t.size() && t[pos].type == Token::COMMA) ++pos;
                else if (t[pos].type != Token::RPAREN) ++pos;
            }
            if (pos < t.size()) ++pos; // skip ')'

            // Create a gate representing the component instance
            std::vector<NetId> inputs;
            NetId output = -1;
            for (auto& [formal, actual] : connections) {
                NetId id = lookup_or_create(actual, nl);
                // Heuristic: last connection is typically output
                inputs.push_back(id);
            }
            if (!inputs.empty()) {
                output = inputs.back();
                inputs.pop_back();
            }
            if (output >= 0) {
                nl.add_gate(GateType::AND, inputs, output, label);
            }
        }
    }

    if (pos < t.size() && t[pos].type == Token::SEMI) ++pos;
}

// ---------------------------------------------------------------------------
// to_vhdl: Generate VHDL from Netlist
// ---------------------------------------------------------------------------

std::string VhdlParser::to_vhdl(const Netlist& nl, const std::string& entity_name) {
    std::ostringstream out;
    out << "library ieee;\nuse ieee.std_logic_1164.all;\n\n";

    // Entity declaration
    out << "entity " << entity_name << " is\n";

    auto& pis = nl.primary_inputs();
    auto& pos_vec = nl.primary_outputs();

    if (!pis.empty() || !pos_vec.empty()) {
        out << "  port (\n";
        bool first = true;
        for (auto id : pis) {
            if (!first) out << ";\n";
            out << "    " << nl.net(id).name << " : in std_logic";
            first = false;
        }
        for (auto id : pos_vec) {
            if (!first) out << ";\n";
            out << "    " << nl.net(id).name << " : out std_logic";
            first = false;
        }
        out << "\n  );\n";
    }
    out << "end entity " << entity_name << ";\n\n";

    // Architecture
    out << "architecture rtl of " << entity_name << " is\n";

    // Signal declarations for internal nets
    std::unordered_map<NetId, bool> is_port;
    for (auto id : pis) is_port[id] = true;
    for (auto id : pos_vec) is_port[id] = true;

    for (auto& net : nl.nets()) {
        if (is_port.find(net.id) == is_port.end()) {
            out << "  signal " << net.name << " : std_logic;\n";
        }
    }

    out << "begin\n";

    // Gate instantiations as signal assignments
    for (auto& g : nl.gates()) {
        if (g.type == GateType::DFF) {
            NetId clk = g.clk;
            std::string clk_name = (clk >= 0) ? nl.net(clk).name : "clk";
            out << "  -- " << g.name << ": DFF\n";
            out << "  process (" << clk_name << ")\n";
            out << "  begin\n";
            out << "    if rising_edge(" << clk_name << ") then\n";
            out << "      " << nl.net(g.output).name << " <= "
                << nl.net(g.inputs[0]).name << ";\n";
            out << "    end if;\n";
            out << "  end process;\n";
            continue;
        }

        if (g.output < 0) continue;
        std::string oname = nl.net(g.output).name;

        switch (g.type) {
            case GateType::BUF:
                out << "  " << oname << " <= " << nl.net(g.inputs[0]).name << ";\n";
                break;
            case GateType::NOT:
                out << "  " << oname << " <= not " << nl.net(g.inputs[0]).name << ";\n";
                break;
            case GateType::AND:
            case GateType::OR:
            case GateType::XOR:
            case GateType::NAND:
            case GateType::NOR:
            case GateType::XNOR: {
                std::string op;
                switch (g.type) {
                    case GateType::AND:  op = " and "; break;
                    case GateType::OR:   op = " or "; break;
                    case GateType::XOR:  op = " xor "; break;
                    case GateType::NAND: op = " nand "; break;
                    case GateType::NOR:  op = " nor "; break;
                    case GateType::XNOR: op = " xnor "; break;
                    default: break;
                }
                out << "  " << oname << " <= ";
                for (size_t i = 0; i < g.inputs.size(); ++i) {
                    if (i > 0) out << op;
                    out << nl.net(g.inputs[i]).name;
                }
                out << ";\n";
                break;
            }
            case GateType::MUX:
                if (g.inputs.size() >= 3) {
                    out << "  " << oname << " <= " << nl.net(g.inputs[1]).name
                        << " when " << nl.net(g.inputs[0]).name
                        << " = '1' else " << nl.net(g.inputs[2]).name << ";\n";
                }
                break;
            default:
                out << "  -- " << g.name << ": unsupported gate type\n";
                break;
        }
    }

    out << "end architecture rtl;\n";
    return out.str();
}

} // namespace sf
