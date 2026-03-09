// SiliconForge — DEF Parser Implementation
#include "core/def_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

namespace sf {

std::vector<std::string> DefParser::tokenize(const std::string& content) {
    std::vector<std::string> tokens;
    std::istringstream ss(content);
    std::string tok;
    while (ss >> tok) {
        // Skip comments
        if (tok.size() >= 1 && tok[0] == '#') {
            std::string line;
            std::getline(ss, line);
            continue;
        }
        // Split around semicolons and parens
        std::string cur;
        for (char c : tok) {
            if (c == ';' || c == '(' || c == ')') {
                if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
                tokens.push_back(std::string(1, c));
            } else {
                cur += c;
            }
        }
        if (!cur.empty()) tokens.push_back(cur);
    }
    return tokens;
}

bool DefParser::parse_tokens(const std::vector<std::string>& t, PhysicalDesign& pd) {
    size_t pos = 0;
    while (pos < t.size()) {
        if (t[pos] == "DIEAREA") {
            pos++;
            // ( x0 y0 ) ( x1 y1 ) ;
            if (pos < t.size() && t[pos] == "(") pos++;
            double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
            if (pos < t.size()) { x0 = std::stod(t[pos]) / 1000.0; pos++; }
            if (pos < t.size()) { y0 = std::stod(t[pos]) / 1000.0; pos++; }
            if (pos < t.size() && t[pos] == ")") pos++;
            if (pos < t.size() && t[pos] == "(") pos++;
            if (pos < t.size()) { x1 = std::stod(t[pos]) / 1000.0; pos++; }
            if (pos < t.size()) { y1 = std::stod(t[pos]) / 1000.0; pos++; }
            if (pos < t.size() && t[pos] == ")") pos++;
            if (pos < t.size() && t[pos] == ";") pos++;
            pd.die_area = Rect(x0, y0, x1, y1);
        }
        else if (t[pos] == "COMPONENTS") {
            pos++; // skip count
            if (pos < t.size()) pos++;
            if (pos < t.size() && t[pos] == ";") pos++;

            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    std::string name, type;
                    if (pos < t.size()) { name = t[pos]; pos++; }
                    if (pos < t.size()) { type = t[pos]; pos++; }

                    double px = 0, py = 0;
                    while (pos < t.size() && t[pos] != ";") {
                        if (t[pos] == "PLACED" || t[pos] == "FIXED") {
                            pos++;
                            if (pos < t.size() && t[pos] == "(") pos++;
                            if (pos < t.size()) { px = std::stod(t[pos]) / 1000.0; pos++; }
                            if (pos < t.size()) { py = std::stod(t[pos]) / 1000.0; pos++; }
                            if (pos < t.size() && t[pos] == ")") pos++;
                        }
                        else pos++;
                    }
                    if (pos < t.size() && t[pos] == ";") pos++;

                    int cid = pd.add_cell(name, type, 3.0, pd.row_height);
                    pd.cells[cid].position = {px, py};
                    pd.cells[cid].placed = true;
                }
                else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; pos++; } // END COMPONENTS
        }
        else if (t[pos] == "NETS") {
            pos++;
            if (pos < t.size()) pos++;
            if (pos < t.size() && t[pos] == ";") pos++;

            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    std::string net_name;
                    if (pos < t.size()) { net_name = t[pos]; pos++; }

                    std::vector<int> cell_ids;
                    while (pos < t.size() && t[pos] != ";") {
                        if (t[pos] == "(") {
                            pos++;
                            if (pos < t.size()) {
                                std::string cname = t[pos]; pos++;
                                // Find cell by name
                                for (size_t ci = 0; ci < pd.cells.size(); ++ci) {
                                    if (pd.cells[ci].name == cname) {
                                        cell_ids.push_back(ci);
                                        break;
                                    }
                                }
                            }
                            while (pos < t.size() && t[pos] != ")") pos++;
                            if (pos < t.size()) pos++; // skip )
                        }
                        else pos++;
                    }
                    if (pos < t.size() && t[pos] == ";") pos++;
                    if (cell_ids.size() >= 2)
                        pd.add_net(net_name, cell_ids);
                }
                else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; pos++; }
        }
        else pos++;
    }
    return true;
}

bool DefParser::parse_string(const std::string& content, PhysicalDesign& pd) {
    auto tokens = tokenize(content);
    return parse_tokens(tokens, pd);
}

bool DefParser::parse_file(const std::string& filename, PhysicalDesign& pd) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return parse_string(content, pd);
}

std::string DefParser::export_def(const PhysicalDesign& pd) {
    std::ostringstream def;
    def << "VERSION 5.8 ;\n"
        << "DESIGN SiliconForge ;\n"
        << "UNITS DISTANCE MICRONS 1000 ;\n"
        << "DIEAREA ( " << (int)(pd.die_area.x0*1000) << " "
        << (int)(pd.die_area.y0*1000) << " ) ( "
        << (int)(pd.die_area.x1*1000) << " "
        << (int)(pd.die_area.y1*1000) << " ) ;\n\n";

    def << "COMPONENTS " << pd.cells.size() << " ;\n";
    for (auto& c : pd.cells) {
        def << "  - " << c.name << " " << c.cell_type
            << " + PLACED ( " << (int)(c.position.x*1000) << " "
            << (int)(c.position.y*1000) << " ) N ;\n";
    }
    def << "END COMPONENTS\n\n";

    def << "NETS " << pd.nets.size() << " ;\n";
    for (auto& n : pd.nets) {
        def << "  - " << n.name;
        for (auto cid : n.cell_ids)
            def << " ( " << pd.cells[cid].name << " Z )";
        def << " ;\n";
    }
    def << "END NETS\n\n";
    def << "END DESIGN\n";
    return def.str();
}

} // namespace sf
