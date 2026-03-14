// SiliconForge — SPEF/DSPF Parasitic File Parser Implementation
#include "core/spef_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>

namespace sf {

// ── Utility ──────────────────────────────────────────────────────────────────

std::string SpefParser::strip(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) a++;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) b--;
    return s.substr(a, b - a);
}

std::vector<std::string> SpefParser::split_tokens(const std::string& line) {
    std::vector<std::string> toks;
    std::istringstream iss(line);
    std::string tok;
    while (iss >> tok) toks.push_back(tok);
    return toks;
}

std::vector<std::string> SpefParser::read_lines(const std::string& content) {
    std::vector<std::string> lines;
    std::istringstream iss(content);
    std::string line;
    while (std::getline(iss, line)) {
        // Strip block comments (simple single-line handling)
        auto bc = line.find("/*");
        if (bc != std::string::npos) {
            auto ec = line.find("*/", bc + 2);
            if (ec != std::string::npos)
                line = line.substr(0, bc) + line.substr(ec + 2);
            else
                line = line.substr(0, bc);
        }
        // Strip line comments
        auto lc = line.find("//");
        if (lc != std::string::npos) line = line.substr(0, lc);

        std::string trimmed = strip(line);
        if (!trimmed.empty()) lines.push_back(trimmed);
    }
    return lines;
}

std::string SpefParser::resolve_name(const std::string& token) const {
    if (!token.empty() && token[0] == '*') {
        std::string key = token.substr(1);
        // Only resolve purely numeric map keys
        if (!key.empty() && std::all_of(key.begin(), key.end(),
                [](unsigned char c) { return std::isdigit(c); })) {
            auto it = name_map_.find(key);
            if (it != name_map_.end()) return it->second;
        }
    }
    return token;
}

double SpefParser::parse_unit_scale(const std::string& amount, const std::string& unit,
                                    double /*base_ns*/, double /*base_pf*/,
                                    double /*base_ohm*/, double /*base_nh*/) {
    double val = std::stod(amount);
    std::string u = unit;
    std::transform(u.begin(), u.end(), u.begin(),
                   [](unsigned char c) { return std::toupper(c); });

    if (u == "NS")    return val * 1e-9;
    if (u == "PS")    return val * 1e-12;
    if (u == "US")    return val * 1e-6;
    if (u == "PF")    return val * 1e-12;
    if (u == "FF")    return val * 1e-15;
    if (u == "OHM")   return val * 1.0;
    if (u == "KOHM")  return val * 1e3;
    if (u == "NH")    return val * 1e-9;
    if (u == "PH")    return val * 1e-12;
    return val;
}

// ── Public interface ─────────────────────────────────────────────────────────

bool SpefParser::parse(const std::string& filename) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "[SPEF] Cannot open file: " << filename << "\n";
        return false;
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();
    return parse_string(ss.str());
}

bool SpefParser::parse_string(const std::string& content) {
    data_ = SpefData{};
    name_map_.clear();

    auto lines = read_lines(content);
    if (lines.empty()) return false;

    size_t idx = 0;
    parse_header(lines, idx);
    parse_name_map(lines, idx);

    while (idx < lines.size()) {
        const auto& line = lines[idx];
        if (line.rfind("*D_NET", 0) == 0 || line.rfind("*R_NET", 0) == 0 ||
            line.rfind("*L_NET", 0) == 0) {
            parse_net(lines, idx);
        } else if (line.rfind("*K_NET", 0) == 0 || line.rfind("*MUTUAL", 0) == 0) {
            parse_mutual_inductance(lines, idx);
        } else {
            idx++;
        }
    }

    // Build net_map
    for (int i = 0; i < static_cast<int>(data_.nets.size()); i++)
        data_.net_map[data_.nets[i].name] = i;

    return true;
}

void SpefParser::print_stats() const {
    size_t total_res = 0, total_caps = 0, total_pins = 0, total_inds = 0;
    for (auto& n : data_.nets) {
        total_res  += n.resistors.size();
        total_caps += n.caps.size();
        total_pins += n.pins.size();
        total_inds += n.inductors.size();
    }
    std::cout << "SPEF Design: " << data_.design_name << "\n"
              << "  Nets:       " << data_.nets.size() << "\n"
              << "  Pins:       " << total_pins << "\n"
              << "  Resistors:  " << total_res << "\n"
              << "  Capacitors: " << total_caps << "\n"
              << "  Inductors:  " << total_inds << "\n"
              << "  Mutual L:   " << data_.mutual_inductances.size() << "\n";
}

// ── Header parsing ───────────────────────────────────────────────────────────

void SpefParser::parse_header(const std::vector<std::string>& lines, size_t& idx) {
    while (idx < lines.size()) {
        const auto& line = lines[idx];

        // Stop at name map or first net section
        if (line.rfind("*NAME_MAP", 0) == 0 ||
            line.rfind("*D_NET", 0) == 0 ||
            line.rfind("*R_NET", 0) == 0 ||
            line.rfind("*L_NET", 0) == 0)
            return;

        auto toks = split_tokens(line);
        if (toks.empty()) { idx++; continue; }

        if (toks[0] == "*DESIGN" && toks.size() >= 2) {
            // *DESIGN "my_design" or *DESIGN my_design
            std::string name = toks[1];
            if (name.size() >= 2 && name.front() == '"' && name.back() == '"')
                name = name.substr(1, name.size() - 2);
            data_.design_name = name;
        }
        else if (toks[0] == "*T_UNIT" && toks.size() >= 3)
            data_.units.time_scale = parse_unit_scale(toks[1], toks[2], 0, 0, 0, 0);
        else if (toks[0] == "*C_UNIT" && toks.size() >= 3)
            data_.units.cap_scale = parse_unit_scale(toks[1], toks[2], 0, 0, 0, 0);
        else if (toks[0] == "*R_UNIT" && toks.size() >= 3)
            data_.units.res_scale = parse_unit_scale(toks[1], toks[2], 0, 0, 0, 0);
        else if (toks[0] == "*L_UNIT" && toks.size() >= 3)
            data_.units.inductance_scale = parse_unit_scale(toks[1], toks[2], 0, 0, 0, 0);

        idx++;
    }
}

// ── Name map parsing ─────────────────────────────────────────────────────────

void SpefParser::parse_name_map(const std::vector<std::string>& lines, size_t& idx) {
    if (idx >= lines.size() || lines[idx].rfind("*NAME_MAP", 0) != 0) return;
    idx++; // skip *NAME_MAP line

    while (idx < lines.size()) {
        const auto& line = lines[idx];
        // End of name map: next keyword section
        if (line.rfind("*D_NET", 0) == 0 || line.rfind("*R_NET", 0) == 0 ||
            line.rfind("*L_NET", 0) == 0 ||
            line.rfind("*PORTS", 0) == 0 || line.rfind("*POWER_NETS", 0) == 0 ||
            line.rfind("*GROUND_NETS", 0) == 0)
            return;

        // Name map entry: *<number> <name>
        if (line[0] == '*') {
            auto toks = split_tokens(line);
            if (toks.size() >= 2) {
                std::string key = toks[0].substr(1); // strip leading *
                if (!key.empty() && std::isdigit(static_cast<unsigned char>(key[0])))
                    name_map_[key] = toks[1];
            }
        }
        idx++;
    }
}

// ── Net parsing ──────────────────────────────────────────────────────────────

void SpefParser::parse_net(const std::vector<std::string>& lines, size_t& idx) {
    // *D_NET net_name total_cap  or  *R_NET net_name total_cap
    auto header_toks = split_tokens(lines[idx]);
    if (header_toks.size() < 3) { idx++; return; }

    SpefNet net;
    net.name      = resolve_name(header_toks[1]);
    net.total_cap = std::stod(header_toks[2]);
    idx++;

    enum Section { NONE, CONN, CAP, RES, INDUC };
    Section section = NONE;

    while (idx < lines.size()) {
        const auto& line = lines[idx];

        // End of this net
        if (line == "*END") { idx++; break; }

        // Start of a different net (missing *END)
        if (line.rfind("*D_NET", 0) == 0 || line.rfind("*R_NET", 0) == 0 ||
            line.rfind("*L_NET", 0) == 0) break;

        // Section headers
        if (line.rfind("*CONN", 0) == 0)   { section = CONN;  idx++; continue; }
        if (line.rfind("*CAP", 0) == 0)     { section = CAP;   idx++; continue; }
        if (line.rfind("*RES", 0) == 0)     { section = RES;   idx++; continue; }
        if (line.rfind("*INDUC", 0) == 0)   { section = INDUC; idx++; continue; }

        auto toks = split_tokens(line);
        if (toks.empty()) { idx++; continue; }

        switch (section) {
        case CONN: {
            // *P pin_name dir [cap] [*C x y] or *I inst:pin dir [cap] [*C x y]
            if (toks[0] == "*P" || toks[0] == "*I") {
                SpefPinParasitic pin;
                pin.pin_name = (toks.size() >= 2) ? resolve_name(toks[1]) : "";
                // Look for cap value (numeric token after direction)
                if (toks.size() >= 4) {
                    try { pin.cap = std::stod(toks[3]); } catch (...) {}
                }
                // Look for *C x y coordinates
                for (size_t t = 0; t + 2 < toks.size(); t++) {
                    if (toks[t] == "*C") {
                        try { pin.x = std::stod(toks[t + 1]); } catch (...) {}
                        try { pin.y = std::stod(toks[t + 2]); } catch (...) {}
                        break;
                    }
                }
                net.pins.push_back(pin);
            }
            break;
        }
        case CAP: {
            // id node1 value              (ground cap)
            // id node1 node2 value        (coupling cap)
            if (toks.size() >= 3) {
                SpefCapacitor cap;
                cap.id = std::stoi(toks[0]);
                // Determine ground vs coupling by trying to parse last token as double
                bool coupling = false;
                if (toks.size() >= 4) {
                    // If toks[2] does NOT parse as a number, it's a node name → coupling
                    try {
                        std::stod(toks[2]);
                    } catch (...) {
                        coupling = true;
                    }
                    // Also check: if toks.size()==4, treat as coupling
                    if (toks.size() == 4) coupling = true;
                }

                if (coupling && toks.size() >= 4) {
                    cap.node1 = resolve_name(toks[1]);
                    cap.node2 = resolve_name(toks[2]);
                    cap.value = std::stod(toks[3]);
                } else {
                    cap.node1 = resolve_name(toks[1]);
                    cap.value = std::stod(toks[2]);
                }
                net.caps.push_back(cap);
            }
            break;
        }
        case RES: {
            // id node1 node2 value
            if (toks.size() >= 4) {
                SpefResistor res;
                res.id    = std::stoi(toks[0]);
                res.node1 = resolve_name(toks[1]);
                res.node2 = resolve_name(toks[2]);
                res.value = std::stod(toks[3]);
                net.resistors.push_back(res);
            }
            break;
        }
        case INDUC: {
            // id node1 node2 value  (inductance in scaled units)
            if (toks.size() >= 4) {
                SpefInductor ind;
                ind.id    = std::stoi(toks[0]);
                ind.node1 = resolve_name(toks[1]);
                ind.node2 = resolve_name(toks[2]);
                ind.value = std::stod(toks[3]);
                net.inductors.push_back(ind);
                net.total_ind += ind.value;
            }
            break;
        }
        default:
            break;
        }
        idx++;
    }

    data_.nets.push_back(std::move(net));
}

// ── Mutual inductance parsing ────────────────────────────────────────────

void SpefParser::parse_mutual_inductance(const std::vector<std::string>& lines, size_t& idx) {
    // *K_NET net1 net2 coupling_value  OR  *MUTUAL net1 net2 coupling_value
    auto toks = split_tokens(lines[idx]);
    if (toks.size() >= 4) {
        MutualInductance mi;
        mi.net1      = resolve_name(toks[1]);
        mi.net2      = resolve_name(toks[2]);
        mi.coupling_l = std::stod(toks[3]);
        data_.mutual_inductances.push_back(mi);
    }
    idx++;
}

// ── SpefData: temperature/voltage scaling ────────────────────────────────

void SpefData::scale_parasitics(double temp_factor, double voltage_factor,
                                double tcr, double vcc) {
    // Resistance scales with temperature: R_scaled = R * (1 + TCR * (T - T_ref))
    // where temp_factor encodes (T - T_ref) as a ratio: temp_factor = T/T_ref
    double r_scale = 1.0 + tcr * (temp_factor - 1.0);

    // Capacitance scales mildly with voltage (depletion cap effect)
    double c_scale = 1.0 + vcc * (voltage_factor - 1.0);

    // Inductance has mild temperature dependence (skin effect changes)
    double l_scale = 1.0 + 0.0005 * (temp_factor - 1.0);

    for (auto& net : nets) {
        for (auto& r : net.resistors)
            r.value *= r_scale;
        for (auto& c : net.caps)
            c.value *= c_scale;
        net.total_cap *= c_scale;
        for (auto& l : net.inductors)
            l.value *= l_scale;
        net.total_ind *= l_scale;
    }
    for (auto& mi : mutual_inductances)
        mi.coupling_l *= l_scale;
}

// ── SpefData: process-corner derating ────────────────────────────────────

void SpefData::apply_corner(const ExtractionCorner& corner) {
    for (auto& net : nets) {
        for (auto& r : net.resistors)
            r.value *= corner.r_factor;
        for (auto& c : net.caps)
            c.value *= corner.c_factor;
        net.total_cap *= corner.c_factor;
        for (auto& l : net.inductors)
            l.value *= corner.l_factor;
        net.total_ind *= corner.l_factor;
    }
    for (auto& mi : mutual_inductances)
        mi.coupling_l *= corner.l_factor;
}

} // namespace sf
