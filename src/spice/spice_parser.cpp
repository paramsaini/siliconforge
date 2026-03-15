// SPICE Netlist Parser — implementation
#include "spice/spice_parser.hpp"
#include <iostream>
#include <set>
#include <cmath>

namespace sf {

// ════════════════════════════════════════════════════════════════════════
// Utility helpers
// ════════════════════════════════════════════════════════════════════════

static std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return s;
}

static std::string trim(const std::string& s) {
    auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

// ════════════════════════════════════════════════════════════════════════
// Tokenizer
// ════════════════════════════════════════════════════════════════════════

std::vector<std::string> SpiceParser::tokenize(const std::string& line) {
    std::vector<std::string> tokens;
    std::istringstream iss(line);
    std::string tok;
    while (iss >> tok) tokens.push_back(tok);
    return tokens;
}

// ════════════════════════════════════════════════════════════════════════
// Engineering notation parser
// ════════════════════════════════════════════════════════════════════════

double SpiceParser::parse_value(const std::string& s) {
    if (s.empty()) return 0.0;

    // Try direct parse first
    char* end = nullptr;
    double val = std::strtod(s.c_str(), &end);

    if (end == s.c_str()) {
        // Could not parse any number
        return 0.0;
    }

    std::string suffix(end);
    suffix = to_lower(trim(suffix));

    if (suffix.empty()) return val;

    // SPICE engineering suffixes
    if (suffix[0] == 't')       return val * 1e12;
    if (suffix[0] == 'g')       return val * 1e9;
    if (suffix.substr(0,3) == "meg") return val * 1e6;
    if (suffix[0] == 'k')       return val * 1e3;
    if (suffix[0] == 'm' && suffix.substr(0,3) != "meg")
                                return val * 1e-3;
    if (suffix[0] == 'u')       return val * 1e-6;
    if (suffix[0] == 'n')       return val * 1e-9;
    if (suffix[0] == 'p')       return val * 1e-12;
    if (suffix[0] == 'f')       return val * 1e-15;
    if (suffix[0] == 'a')       return val * 1e-18;

    return val;
}

// ════════════════════════════════════════════════════════════════════════
// Key=Value parameter extraction
// ════════════════════════════════════════════════════════════════════════

std::map<std::string, double> SpiceParser::parse_kv_params(
        const std::vector<std::string>& tokens, size_t start_idx) {
    std::map<std::string, double> params;
    for (size_t i = start_idx; i < tokens.size(); ++i) {
        auto eq = tokens[i].find('=');
        if (eq != std::string::npos) {
            std::string key = to_lower(tokens[i].substr(0, eq));
            std::string val_str = tokens[i].substr(eq + 1);
            if (val_str.empty() && i + 1 < tokens.size()) {
                val_str = tokens[++i];
            }
            params[key] = parse_value(val_str);
        }
    }
    return params;
}

// ════════════════════════════════════════════════════════════════════════
// Device type inference
// ════════════════════════════════════════════════════════════════════════

SpiceDeviceType SpiceParser::device_type_from_name(const std::string& name) {
    if (name.empty()) return SpiceDeviceType::UNKNOWN;
    switch (std::toupper(static_cast<unsigned char>(name[0]))) {
        case 'M': return SpiceDeviceType::MOSFET;
        case 'R': return SpiceDeviceType::RESISTOR;
        case 'C': return SpiceDeviceType::CAPACITOR;
        case 'V': return SpiceDeviceType::VSOURCE;
        case 'I': return SpiceDeviceType::ISOURCE;
        case 'L': return SpiceDeviceType::INDUCTOR;
        case 'D': return SpiceDeviceType::DIODE;
        default:  return SpiceDeviceType::UNKNOWN;
    }
}

// ════════════════════════════════════════════════════════════════════════
// Device instance parser
// ════════════════════════════════════════════════════════════════════════

SpiceDevice SpiceParser::parse_device(const std::vector<std::string>& tokens) {
    SpiceDevice dev;
    dev.name = tokens[0];
    dev.type = device_type_from_name(dev.name);

    switch (dev.type) {
        case SpiceDeviceType::MOSFET: {
            // Mxxx drain gate source bulk model_name [params...]
            if (tokens.size() < 6) break;
            dev.terminals = {tokens[1], tokens[2], tokens[3], tokens[4]};
            dev.model_name = tokens[5];
            dev.params = parse_kv_params(tokens, 6);
            break;
        }
        case SpiceDeviceType::RESISTOR: {
            // Rxxx n+ n- value  OR  Rxxx n+ n- R=value
            if (tokens.size() < 4) break;
            dev.terminals = {tokens[1], tokens[2]};
            if (tokens[3].find('=') != std::string::npos) {
                dev.params = parse_kv_params(tokens, 3);
            } else {
                dev.params["r"] = parse_value(tokens[3]);
                auto extra = parse_kv_params(tokens, 4);
                dev.params.insert(extra.begin(), extra.end());
            }
            break;
        }
        case SpiceDeviceType::CAPACITOR: {
            // Cxxx n+ n- value
            if (tokens.size() < 4) break;
            dev.terminals = {tokens[1], tokens[2]};
            if (tokens[3].find('=') != std::string::npos) {
                dev.params = parse_kv_params(tokens, 3);
            } else {
                dev.params["c"] = parse_value(tokens[3]);
                auto extra = parse_kv_params(tokens, 4);
                dev.params.insert(extra.begin(), extra.end());
            }
            break;
        }
        case SpiceDeviceType::VSOURCE:
        case SpiceDeviceType::ISOURCE: {
            // Vxxx n+ n- DC value  OR  Vxxx n+ n- value  OR  Vxxx n+ n- PWL(...)
            if (tokens.size() < 3) break;
            dev.terminals = {tokens[1], tokens[2]};
            // Check for DC keyword
            size_t val_idx = 3;
            if (val_idx < tokens.size()) {
                std::string kw = to_lower(tokens[val_idx]);
                if (kw == "dc" || kw == "ac") {
                    val_idx++;
                }
                if (kw == "pwl" || (val_idx < tokens.size() && to_lower(tokens[val_idx]) == "pwl")) {
                    // Parse PWL points — simplified: store as pwl_t0, pwl_v0, ...
                    dev.params["pwl"] = 1.0;
                    // Collect time-value pairs from remaining tokens
                    size_t pwl_start = (kw == "pwl") ? val_idx + 1 : val_idx + 1;
                    // Remove parentheses
                    int pair_idx = 0;
                    for (size_t k = pwl_start; k + 1 < tokens.size(); k += 2) {
                        std::string t_str = tokens[k];
                        std::string v_str = tokens[k+1];
                        // Strip parens
                        if (!t_str.empty() && t_str[0] == '(') t_str = t_str.substr(1);
                        if (!v_str.empty() && v_str.back() == ')') v_str = v_str.substr(0, v_str.size()-1);
                        dev.params["pwl_t" + std::to_string(pair_idx)] = parse_value(t_str);
                        dev.params["pwl_v" + std::to_string(pair_idx)] = parse_value(v_str);
                        pair_idx++;
                    }
                    dev.params["pwl_n"] = pair_idx;
                } else if (val_idx < tokens.size()) {
                    dev.params["dc"] = parse_value(tokens[val_idx]);
                }
            }
            break;
        }
        default: {
            // Generic: assume 2-terminal device
            if (tokens.size() >= 3) {
                dev.terminals = {tokens[1], tokens[2]};
                if (tokens.size() > 3)
                    dev.params = parse_kv_params(tokens, 3);
            }
            break;
        }
    }
    return dev;
}

// ════════════════════════════════════════════════════════════════════════
// .model parser
// ════════════════════════════════════════════════════════════════════════

SpiceModel SpiceParser::parse_model(const std::string& combined_line) {
    SpiceModel model;
    // Remove parentheses
    std::string line = combined_line;
    std::replace(line.begin(), line.end(), '(', ' ');
    std::replace(line.begin(), line.end(), ')', ' ');

    auto tokens = tokenize(line);
    // .model name type [params...]
    if (tokens.size() < 3) return model;

    model.name = tokens[1];
    model.type = to_lower(tokens[2]);

    auto kv = parse_kv_params(tokens, 3);
    model.params = kv;

    if (model.params.count("level"))
        model.level = static_cast<int>(model.params["level"]);

    return model;
}

// ════════════════════════════════════════════════════════════════════════
// Internal net resolution
// ════════════════════════════════════════════════════════════════════════

void SpiceParser::resolve_internal_nets(SpiceSubckt& sub) {
    std::set<std::string> port_set(sub.ports.begin(), sub.ports.end());
    std::set<std::string> all_nets;
    // Ground is always external
    port_set.insert("0");
    port_set.insert("gnd");
    port_set.insert("GND");

    for (auto& dev : sub.devices) {
        for (auto& t : dev.terminals) {
            if (port_set.find(t) == port_set.end())
                all_nets.insert(t);
        }
    }
    sub.internal_nets.assign(all_nets.begin(), all_nets.end());
}

// ════════════════════════════════════════════════════════════════════════
// Main parser — line-by-line state machine
// ════════════════════════════════════════════════════════════════════════

SpiceCircuit SpiceParser::parse(const std::string& netlist) {
    SpiceCircuit ckt;

    // Pre-process: handle line continuations (+ at start of line)
    std::vector<std::string> lines;
    {
        std::istringstream iss(netlist);
        std::string line;
        while (std::getline(iss, line)) {
            line = trim(line);
            if (!line.empty() && line[0] == '+') {
                // Continuation of previous line
                if (!lines.empty()) {
                    lines.back() += " " + line.substr(1);
                }
            } else {
                lines.push_back(line);
            }
        }
    }

    // First line is the title
    if (!lines.empty()) {
        ckt.title = lines[0];
    }

    bool in_subckt = false;
    SpiceSubckt current_sub;

    for (size_t i = 1; i < lines.size(); ++i) {
        const std::string& line = lines[i];
        if (line.empty() || line[0] == '*') continue;  // comment or blank

        auto tokens = tokenize(line);
        if (tokens.empty()) continue;

        std::string first = to_lower(tokens[0]);

        // ── Directives ──
        if (first == ".subckt") {
            if (tokens.size() < 2) continue;
            in_subckt = true;
            current_sub = SpiceSubckt{};
            current_sub.name = tokens[1];
            for (size_t j = 2; j < tokens.size(); ++j) {
                // Stop at parameter assignments
                if (tokens[j].find('=') != std::string::npos) break;
                current_sub.ports.push_back(tokens[j]);
            }
        }
        else if (first == ".ends") {
            if (in_subckt) {
                resolve_internal_nets(current_sub);
                ckt.subcircuits.push_back(std::move(current_sub));
                in_subckt = false;
            }
        }
        else if (first == ".model") {
            auto model = parse_model(line);
            ckt.models[model.name] = model;
        }
        else if (first == ".param") {
            // .param name=value ...
            auto kv = parse_kv_params(tokens, 1);
            for (auto& p : kv) ckt.params[p.first] = p.second;
        }
        else if (first == ".end") {
            break;  // end of netlist
        }
        else if (first[0] == '.') {
            // Other directives — skip for now
            continue;
        }
        else {
            // Device instance line
            auto dev = parse_device(tokens);
            if (in_subckt) {
                current_sub.devices.push_back(std::move(dev));
            } else {
                ckt.instances.push_back(std::move(dev));
            }
        }
    }

    return ckt;
}

} // namespace sf
