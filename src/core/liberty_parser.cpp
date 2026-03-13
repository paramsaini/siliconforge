// SiliconForge — Liberty (.lib) Parser Implementation
#include "core/liberty_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>

namespace sf {

const LibertyPin* LibertyCell::find_pin(const std::string& n) const {
    for (auto& p : pins) if (p.name == n) return &p;
    return nullptr;
}

std::string LibertyCell::output_function() const {
    for (auto& p : pins) if (p.direction == "output" && !p.function.empty()) return p.function;
    return "";
}

int LibertyCell::num_inputs() const {
    int n = 0;
    for (auto& p : pins) if (p.direction == "input") n++;
    return n;
}

double LibertyCell::internal_power_at(double input_slew, double output_load) const {
    for (auto& pw : internal_powers) {
        if (pw.power_rise_table.valid() && pw.power_fall_table.valid()) {
            double pr = pw.power_rise_table.interpolate(input_slew, output_load);
            double pf = pw.power_fall_table.interpolate(input_slew, output_load);
            return (pr + pf) / 2.0;
        }
        if (pw.rise_power > 0 || pw.fall_power > 0)
            return (pw.rise_power + pw.fall_power) / 2.0;
    }
    return 0.0;
}

const LibertyCell* LibertyLibrary::find_cell(const std::string& n) const {
    for (auto& c : cells) if (c.name == n) return &c;
    return nullptr;
}

std::vector<const LibertyCell*> LibertyLibrary::cells_by_function(const std::string& func) const {
    std::vector<const LibertyCell*> result;
    for (auto& c : cells) if (c.output_function() == func) result.push_back(&c);
    return result;
}

void LibertyLibrary::print_stats() const {
    std::cout << "Liberty Library: " << name << "\n"
              << "  Technology: " << technology << "\n"
              << "  Cells: " << cells.size() << "\n"
              << "  Voltage: " << nom_voltage << "V\n";
}

// Tokenizer
std::vector<LibertyLibrary::Token> LibertyLibrary::tokenize(const std::string& content) {
    std::vector<Token> tokens;
    size_t i = 0;
    while (i < content.size()) {
        char c = content[i];
        if (std::isspace(c)) { i++; continue; }

        // Skip line comments
        if (c == '/' && i + 1 < content.size() && content[i+1] == '*') {
            i += 2;
            while (i + 1 < content.size() && !(content[i] == '*' && content[i+1] == '/')) i++;
            i += 2; continue;
        }
        if (c == '/' && i + 1 < content.size() && content[i+1] == '/') {
            while (i < content.size() && content[i] != '\n') i++;
            continue;
        }

        if (c == '{') { tokens.push_back({Token::LBRACE, "{"}); i++; }
        else if (c == '}') { tokens.push_back({Token::RBRACE, "}"}); i++; }
        else if (c == '(') { tokens.push_back({Token::LPAREN, "("}); i++; }
        else if (c == ')') { tokens.push_back({Token::RPAREN, ")"}); i++; }
        else if (c == ':') { tokens.push_back({Token::COLON, ":"}); i++; }
        else if (c == ';') { tokens.push_back({Token::SEMI, ";"}); i++; }
        else if (c == ',') { tokens.push_back({Token::COMMA, ","}); i++; }
        else if (c == '"') {
            i++;
            std::string s;
            while (i < content.size() && content[i] != '"') s += content[i++];
            i++; // skip closing "
            tokens.push_back({Token::STRING, s});
        }
        else if (std::isdigit(c) || c == '-' || c == '.') {
            std::string num;
            while (i < content.size() && (std::isdigit(content[i]) || content[i] == '.' ||
                   content[i] == 'e' || content[i] == 'E' || content[i] == '-' || content[i] == '+'))
                num += content[i++];
            tokens.push_back({Token::NUMBER, num});
        }
        else if (std::isalpha(c) || c == '_' || c == '\\' || c == '!' || c == '&' || c == '|' || c == '^') {
            std::string ident;
            while (i < content.size() && (std::isalnum(content[i]) || content[i] == '_' ||
                   content[i] == '!' || content[i] == '&' || content[i] == '|' ||
                   content[i] == '^' || content[i] == ' ' || content[i] == '\\'))
            {
                if (content[i] == ' ') {
                    // Only keep spaces in function expressions
                    if (!ident.empty() && (ident.back() == '&' || ident.back() == '|' || ident.back() == '^'))
                        { ident += content[i]; i++; continue; }
                    // Check if next non-space is an operator
                    size_t j = i + 1;
                    while (j < content.size() && content[j] == ' ') j++;
                    if (j < content.size() && (content[j] == '&' || content[j] == '|' || content[j] == '^'))
                        { ident += content[i]; i++; continue; }
                    break;
                }
                ident += content[i++];
            }
            tokens.push_back({Token::IDENT, ident});
        }
        else { i++; } // skip unknown
    }
    tokens.push_back({Token::END, ""});
    return tokens;
}

size_t LibertyLibrary::skip_group(const std::vector<Token>& t, size_t pos) {
    if (pos < t.size() && t[pos].type == Token::LBRACE) {
        pos++;
        int depth = 1;
        while (pos < t.size() && depth > 0) {
            if (t[pos].type == Token::LBRACE) depth++;
            else if (t[pos].type == Token::RBRACE) depth--;
            pos++;
        }
    }
    return pos;
}

size_t LibertyLibrary::parse_pin(const std::vector<Token>& t, size_t pos, LibertyCell& cell) {
    // pin(name) { ... }
    LibertyPin pin;
    if (t[pos].type == Token::LPAREN) {
        pos++;
        if (pos < t.size()) { pin.name = t[pos].value; pos++; }
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
    }
    std::string pin_name = pin.name;
    if (pos < t.size() && t[pos].type == Token::LBRACE) {
        pos++;
        while (pos < t.size() && t[pos].type != Token::RBRACE) {
            if (t[pos].value == "direction" && pos + 2 < t.size()) {
                pos += 2; // skip : 
                pin.direction = t[pos].value; pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "function" && pos + 2 < t.size()) {
                pos += 2;
                pin.function = t[pos].value; pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "capacitance" && pos + 2 < t.size()) {
                pos += 2;
                try { pin.capacitance = std::stod(t[pos].value); } catch(...) {}
                pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "max_transition" && pos + 2 < t.size()) {
                pos += 2;
                try { pin.max_transition = std::stod(t[pos].value); } catch(...) {}
                pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "timing") {
                pos++;
                pos = parse_timing(t, pos, cell, pin_name);
            } else if (t[pos].value == "internal_power") {
                pos++;
                pos = parse_internal_power(t, pos, cell);
            } else {
                // Skip unknown attribute/group
                if (t[pos].type == Token::IDENT) {
                    pos++;
                    if (pos < t.size() && t[pos].type == Token::LPAREN) {
                        while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
                        pos++;
                        pos = skip_group(t, pos);
                    } else if (pos < t.size() && t[pos].type == Token::COLON) {
                        pos++;
                        if (pos < t.size()) pos++;
                        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                    } else if (pos < t.size() && t[pos].type == Token::LBRACE) {
                        pos = skip_group(t, pos);
                    }
                } else pos++;
            }
        }
        if (pos < t.size() && t[pos].type == Token::RBRACE) pos++;
    }
    cell.pins.push_back(pin);
    return pos;
}

size_t LibertyLibrary::parse_cell(const std::vector<Token>& t, size_t pos) {
    LibertyCell cell;
    if (t[pos].type == Token::LPAREN) {
        pos++;
        if (pos < t.size()) { cell.name = t[pos].value; pos++; }
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
    }
    if (pos < t.size() && t[pos].type == Token::LBRACE) {
        pos++;
        while (pos < t.size() && t[pos].type != Token::RBRACE) {
            if (t[pos].value == "area" && pos + 2 < t.size()) {
                pos += 2;
                try { cell.area = std::stod(t[pos].value); } catch(...) {}
                pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "cell_leakage_power" && pos + 2 < t.size()) {
                pos += 2;
                try { cell.leakage_power = std::stod(t[pos].value); } catch(...) {}
                pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "leakage_power") {
                // leakage_power() { when : "..."; value : ...; }
                pos++;
                LibertyLeakage lk;
                if (pos < t.size() && t[pos].type == Token::LPAREN) {
                    while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
                    pos++;
                }
                if (pos < t.size() && t[pos].type == Token::LBRACE) {
                    pos++;
                    while (pos < t.size() && t[pos].type != Token::RBRACE) {
                        if (t[pos].value == "when" && pos + 2 < t.size()) {
                            pos += 2;
                            lk.when = t[pos].value; pos++;
                            if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                        } else if (t[pos].value == "value" && pos + 2 < t.size()) {
                            pos += 2;
                            try { lk.value = std::stod(t[pos].value); } catch(...) {}
                            pos++;
                            if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                        } else { pos++; }
                    }
                    if (pos < t.size() && t[pos].type == Token::RBRACE) pos++;
                }
                cell.leakage_powers.push_back(lk);
            } else if (t[pos].value == "pin") {
                pos++;
                pos = parse_pin(t, pos, cell);
            } else {
                if (t[pos].type == Token::IDENT) {
                    pos++;
                    if (pos < t.size() && t[pos].type == Token::LPAREN) {
                        while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
                        pos++;
                        if (pos < t.size() && t[pos].type == Token::LBRACE)
                            pos = skip_group(t, pos);
                    } else if (pos < t.size() && t[pos].type == Token::COLON) {
                        pos++;
                        if (pos < t.size()) pos++;
                        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                    } else if (pos < t.size() && t[pos].type == Token::LBRACE) {
                        pos = skip_group(t, pos);
                    }
                } else pos++;
            }
        }
        if (pos < t.size() && t[pos].type == Token::RBRACE) pos++;
    }
    cells.push_back(cell);
    return pos;
}

bool LibertyLibrary::parse_string(const std::string& content) {
    auto tokens = tokenize(content);
    return parse_tokens(tokens);
}

bool LibertyLibrary::parse(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return parse_string(content);
}

bool LibertyLibrary::parse_tokens(const std::vector<Token>& t) {
    size_t pos = 0;
    // Expect: library(name) { ... }
    while (pos < t.size() && t[pos].type != Token::END) {
        if (t[pos].value == "library") {
            pos++;
            if (pos < t.size() && t[pos].type == Token::LPAREN) {
                pos++;
                if (pos < t.size()) { name = t[pos].value; pos++; }
                if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
            }
            if (pos < t.size() && t[pos].type == Token::LBRACE) {
                pos++;
                while (pos < t.size() && t[pos].type != Token::RBRACE) {
                    if (t[pos].value == "cell") {
                        pos++;
                        pos = parse_cell(t, pos);
                    } else if (t[pos].value == "lu_table_template") {
                        pos++;
                        pos = parse_lu_table_template(t, pos);
                    } else if (t[pos].value == "nom_voltage" && pos + 2 < t.size()) {
                        pos += 2;
                        try { nom_voltage = std::stod(t[pos].value); } catch(...) {}
                        pos++;
                        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                    } else if (t[pos].value == "technology") {
                        pos += 2;
                        if (pos < t.size()) { technology = t[pos].value; pos++; }
                        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                    } else {
                        if (t[pos].type == Token::IDENT) {
                            pos++;
                            if (pos < t.size() && t[pos].type == Token::LPAREN) {
                                while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
                                pos++;
                                if (pos < t.size() && t[pos].type == Token::LBRACE)
                                    pos = skip_group(t, pos);
                            } else if (pos < t.size() && t[pos].type == Token::COLON) {
                                pos++;
                                if (pos < t.size()) pos++;
                                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                            } else if (pos < t.size() && t[pos].type == Token::LBRACE) {
                                pos = skip_group(t, pos);
                            }
                        } else pos++;
                    }
                }
                if (pos < t.size()) pos++; // closing }
            }
        } else pos++;
    }
    return !cells.empty() || !name.empty();
}

} // namespace sf

// Helper: parse comma-separated numbers from a string like "0.01, 0.02, 0.05"
std::vector<double> sf::LibertyLibrary::parse_number_list(const std::string& s) {
    std::vector<double> result;
    std::istringstream iss(s);
    std::string tok;
    while (std::getline(iss, tok, ',')) {
        // Trim whitespace
        size_t start = tok.find_first_not_of(" \t\n\r\\\"");
        size_t end = tok.find_last_not_of(" \t\n\r\\\"");
        if (start == std::string::npos) continue;
        std::string val = tok.substr(start, end - start + 1);
        if (val.empty()) continue;
        try { result.push_back(std::stod(val)); } catch (...) {}
    }
    return result;
}

size_t sf::LibertyLibrary::parse_lu_table_template(const std::vector<Token>& t, size_t pos) {
    // lu_table_template(name) { variable_1 : ...; index_1("..."); variable_2 : ...; index_2("..."); }
    std::string tmpl_name;
    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        pos++;
        if (pos < t.size()) { tmpl_name = t[pos].value; pos++; }
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
    }
    TableTemplate tmpl;
    if (pos < t.size() && t[pos].type == Token::LBRACE) {
        pos++;
        while (pos < t.size() && t[pos].type != Token::RBRACE) {
            if (t[pos].value == "index_1" || t[pos].value == "index_2") {
                bool is_idx1 = (t[pos].value == "index_1");
                pos++;
                // index_1("0.01, 0.02, ...") or index_1 : "0.01, ..."
                std::string vals_str;
                if (pos < t.size() && t[pos].type == Token::LPAREN) {
                    pos++;
                    while (pos < t.size() && t[pos].type != Token::RPAREN) {
                        if (!vals_str.empty()) vals_str += ",";
                        vals_str += t[pos].value;
                        pos++;
                    }
                    if (pos < t.size()) pos++; // RPAREN
                } else if (pos < t.size() && t[pos].type == Token::COLON) {
                    pos++;
                    if (pos < t.size()) { vals_str = t[pos].value; pos++; }
                }
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                auto nums = parse_number_list(vals_str);
                if (is_idx1) tmpl.index_1 = nums;
                else tmpl.index_2 = nums;
            } else {
                // skip variable_1, variable_2, etc.
                pos++;
                if (pos < t.size() && t[pos].type == Token::COLON) {
                    pos++;
                    if (pos < t.size()) pos++;
                    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                }
            }
        }
        if (pos < t.size() && t[pos].type == Token::RBRACE) pos++;
    }
    if (!tmpl_name.empty())
        table_templates[tmpl_name] = tmpl;
    return pos;
}

size_t sf::LibertyLibrary::parse_nldm_table(const std::vector<Token>& t, size_t pos,
                                              LibertyTiming::NldmTable& table) {
    // table_name(template) { index_1("..."); index_2("..."); values("...", "...", ...); }
    std::string tmpl_name;
    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        pos++;
        if (pos < t.size()) { tmpl_name = t[pos].value; pos++; }
        if (pos < t.size() && t[pos].type == Token::RPAREN) pos++;
    }

    // Pre-fill from template if available
    if (!tmpl_name.empty()) {
        auto it = table_templates.find(tmpl_name);
        if (it != table_templates.end()) {
            table.index_1 = it->second.index_1;
            table.index_2 = it->second.index_2;
        }
    }

    if (pos < t.size() && t[pos].type == Token::LBRACE) {
        pos++;
        while (pos < t.size() && t[pos].type != Token::RBRACE) {
            if (t[pos].value == "index_1" || t[pos].value == "index_2") {
                bool is_idx1 = (t[pos].value == "index_1");
                pos++;
                std::string vals_str;
                if (pos < t.size() && t[pos].type == Token::LPAREN) {
                    pos++;
                    while (pos < t.size() && t[pos].type != Token::RPAREN) {
                        if (!vals_str.empty()) vals_str += ",";
                        vals_str += t[pos].value;
                        pos++;
                    }
                    if (pos < t.size()) pos++;
                } else if (pos < t.size() && t[pos].type == Token::COLON) {
                    pos++;
                    if (pos < t.size()) { vals_str = t[pos].value; pos++; }
                }
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                auto nums = parse_number_list(vals_str);
                if (is_idx1) table.index_1 = nums;
                else table.index_2 = nums;
            } else if (t[pos].value == "values") {
                pos++;
                // values("row0", "row1", ...) or values("row0", \n "row1", ...)
                if (pos < t.size() && t[pos].type == Token::LPAREN) {
                    pos++;
                    std::vector<std::vector<double>> rows;
                    while (pos < t.size() && t[pos].type != Token::RPAREN) {
                        if (t[pos].type == Token::STRING || t[pos].type == Token::NUMBER ||
                            t[pos].type == Token::IDENT) {
                            auto row = parse_number_list(t[pos].value);
                            if (!row.empty()) rows.push_back(row);
                        }
                        pos++;
                        // skip commas between rows
                        if (pos < t.size() && t[pos].type == Token::COMMA) pos++;
                    }
                    if (pos < t.size()) pos++; // RPAREN
                    table.values = rows;
                }
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else {
                pos++;
                if (pos < t.size() && t[pos].type == Token::COLON) {
                    pos++;
                    if (pos < t.size()) pos++;
                    if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                }
            }
        }
        if (pos < t.size() && t[pos].type == Token::RBRACE) pos++;
    }
    return pos;
}

size_t sf::LibertyLibrary::parse_timing(const std::vector<Token>& t, size_t pos,
                                         LibertyCell& cell, const std::string& pin_name) {
    // timing() { related_pin : "A"; timing_type : "..."; cell_rise(tmpl) { ... }; ... }
    LibertyTiming timing;
    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
        pos++;
    }
    if (pos < t.size() && t[pos].type == Token::LBRACE) {
        pos++;
        while (pos < t.size() && t[pos].type != Token::RBRACE) {
            if (t[pos].value == "related_pin" && pos + 2 < t.size()) {
                pos += 2;
                timing.related_pin = t[pos].value; pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "timing_type" && pos + 2 < t.size()) {
                pos += 2;
                timing.timing_type = t[pos].value; pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "timing_sense" && pos + 2 < t.size()) {
                pos += 2;
                pos++; // skip value
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "cell_rise") {
                pos++;
                pos = parse_nldm_table(t, pos, timing.nldm_rise);
            } else if (t[pos].value == "cell_fall") {
                pos++;
                pos = parse_nldm_table(t, pos, timing.nldm_fall);
            } else if (t[pos].value == "rise_transition") {
                pos++;
                pos = parse_nldm_table(t, pos, timing.nldm_rise_tr);
            } else if (t[pos].value == "fall_transition") {
                pos++;
                pos = parse_nldm_table(t, pos, timing.nldm_fall_tr);
            } else if ((t[pos].value == "intrinsic_rise" || t[pos].value == "cell_rise") &&
                       pos + 2 < t.size() && t[pos+1].type == Token::COLON) {
                // Scalar form: cell_rise : 0.05 ;
                bool is_rise = (t[pos].value == "intrinsic_rise" || t[pos].value == "cell_rise");
                pos += 2;
                try {
                    double v = std::stod(t[pos].value);
                    if (is_rise) timing.cell_rise = v; else timing.cell_fall = v;
                } catch (...) {}
                pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if ((t[pos].value == "intrinsic_fall" || t[pos].value == "cell_fall") &&
                       pos + 2 < t.size() && t[pos+1].type == Token::COLON) {
                pos += 2;
                try { timing.cell_fall = std::stod(t[pos].value); } catch (...) {}
                pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "rise_transition" && pos + 2 < t.size() &&
                       t[pos+1].type == Token::COLON) {
                pos += 2;
                try { timing.rise_transition = std::stod(t[pos].value); } catch (...) {}
                pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "fall_transition" && pos + 2 < t.size() &&
                       t[pos+1].type == Token::COLON) {
                pos += 2;
                try { timing.fall_transition = std::stod(t[pos].value); } catch (...) {}
                pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else {
                // Skip unknown attributes or sub-groups
                if (t[pos].type == Token::IDENT) {
                    pos++;
                    if (pos < t.size() && t[pos].type == Token::LPAREN) {
                        while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
                        pos++;
                        if (pos < t.size() && t[pos].type == Token::LBRACE)
                            pos = skip_group(t, pos);
                    } else if (pos < t.size() && t[pos].type == Token::COLON) {
                        pos++;
                        if (pos < t.size()) pos++;
                        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                    } else if (pos < t.size() && t[pos].type == Token::LBRACE) {
                        pos = skip_group(t, pos);
                    }
                } else pos++;
            }
        }
        if (pos < t.size() && t[pos].type == Token::RBRACE) pos++;
    }
    cell.timings.push_back(timing);
    return pos;
}

size_t sf::LibertyLibrary::parse_internal_power(const std::vector<Token>& t, size_t pos,
                                                  LibertyCell& cell) {
    // internal_power() { related_pin : "A"; rise_power(tmpl) {...}; fall_power(tmpl) {...}; }
    LibertyPower pw;
    if (pos < t.size() && t[pos].type == Token::LPAREN) {
        while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
        pos++;
    }
    if (pos < t.size() && t[pos].type == Token::LBRACE) {
        pos++;
        while (pos < t.size() && t[pos].type != Token::RBRACE) {
            if (t[pos].value == "related_pin" && pos + 2 < t.size()) {
                pos += 2;
                pw.related_pin = t[pos].value; pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "when" && pos + 2 < t.size()) {
                pos += 2;
                pw.when = t[pos].value; pos++;
                if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
            } else if (t[pos].value == "rise_power") {
                pos++;
                pos = parse_nldm_table(t, pos, pw.power_rise_table);
            } else if (t[pos].value == "fall_power") {
                pos++;
                pos = parse_nldm_table(t, pos, pw.power_fall_table);
            } else {
                if (t[pos].type == Token::IDENT) {
                    pos++;
                    if (pos < t.size() && t[pos].type == Token::LPAREN) {
                        while (pos < t.size() && t[pos].type != Token::RPAREN) pos++;
                        pos++;
                        if (pos < t.size() && t[pos].type == Token::LBRACE)
                            pos = skip_group(t, pos);
                    } else if (pos < t.size() && t[pos].type == Token::COLON) {
                        pos++;
                        if (pos < t.size()) pos++;
                        if (pos < t.size() && t[pos].type == Token::SEMI) pos++;
                    } else if (pos < t.size() && t[pos].type == Token::LBRACE) {
                        pos = skip_group(t, pos);
                    }
                } else pos++;
            }
        }
        if (pos < t.size() && t[pos].type == Token::RBRACE) pos++;
    }
    cell.internal_powers.push_back(pw);
    return pos;
}

// NLDM 2D bilinear interpolation
double sf::LibertyTiming::NldmTable::interpolate(double slew, double load) const {
    if (!valid()) return 0;
    
    auto find_bracket = [](const std::vector<double>& v, double x) -> std::pair<int,int> {
        if (x <= v.front()) return {0, 0};
        if (x >= v.back()) return {(int)v.size()-1, (int)v.size()-1};
        for (int i = 0; i + 1 < (int)v.size(); i++)
            if (x >= v[i] && x <= v[i+1]) return {i, i+1};
        return {0, 0};
    };
    
    auto [i0, i1] = find_bracket(index_1, slew);
    auto [j0, j1] = find_bracket(index_2, load);
    
    if (i0 >= (int)values.size() || i1 >= (int)values.size()) return values[0][0];
    if (j0 >= (int)values[i0].size() || j1 >= (int)values[i0].size()) return values[0][0];
    
    // Bilinear interpolation
    double t_s = (i0 == i1) ? 0 : (slew - index_1[i0]) / (index_1[i1] - index_1[i0]);
    double t_l = (j0 == j1) ? 0 : (load - index_2[j0]) / (index_2[j1] - index_2[j0]);
    
    double v00 = values[i0][j0], v01 = values[i0][j1];
    double v10 = values[i1][j0], v11 = values[i1][j1];
    
    double v0 = v00 + (v01 - v00) * t_l;
    double v1 = v10 + (v11 - v10) * t_l;
    return v0 + (v1 - v0) * t_s;
}
