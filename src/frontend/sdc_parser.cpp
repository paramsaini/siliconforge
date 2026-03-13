// SiliconForge — SDC Parser Implementation
#include "frontend/sdc_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>

namespace sf {

const SdcClock* SdcConstraints::find_clock(const std::string& name) const {
    for (auto& c : clocks)
        if (c.name == name) return &c;
    return nullptr;
}

double SdcConstraints::get_clock_period(const std::string& name) const {
    auto* c = find_clock(name);
    return c ? c->period_ns : 0;
}

std::vector<SdcParser::Token> SdcParser::tokenize(const std::string& src) {
    std::vector<Token> tokens;
    int line = 1;
    std::istringstream ss(src);
    std::string raw_line;
    while (std::getline(ss, raw_line)) {
        // Strip comments
        auto hash = raw_line.find('#');
        if (hash != std::string::npos) raw_line = raw_line.substr(0, hash);

        // Handle line continuation
        while (!raw_line.empty() && raw_line.back() == '\\') {
            raw_line.pop_back();
            std::string next;
            if (std::getline(ss, next)) {
                auto h = next.find('#');
                if (h != std::string::npos) next = next.substr(0, h);
                raw_line += " " + next;
                line++;
            }
        }

        // Tokenize the line
        std::istringstream ls(raw_line);
        std::string tok;
        while (ls >> tok) {
            // Handle brackets and braces
            std::string cur;
            for (char c : tok) {
                if (c == '[' || c == ']' || c == '{' || c == '}') {
                    if (!cur.empty()) { tokens.push_back({cur, line}); cur.clear(); }
                    tokens.push_back({std::string(1,c), line});
                } else cur += c;
            }
            if (!cur.empty()) tokens.push_back({cur, line});
        }
        line++;
    }
    return tokens;
}

size_t SdcParser::parse_command(const std::vector<Token>& t, size_t pos,
                                 SdcConstraints& sdc, SdcParseResult& r) {
    const std::string& cmd = t[pos].value;

    if (cmd == "create_clock") {
        pos++;
        SdcClock clk;
        while (pos < t.size()) {
            if (t[pos].value == "-name" && pos+1 < t.size()) { clk.name = t[++pos].value; pos++; }
            else if (t[pos].value == "-period" && pos+1 < t.size()) {
                clk.period_ns = std::stod(t[++pos].value);
                clk.waveform_fall = clk.period_ns / 2;
                pos++;
            }
            else if (t[pos].value == "-waveform") {
                pos++;
                if (pos < t.size() && t[pos].value == "{") pos++;
                if (pos < t.size()) { clk.waveform_rise = std::stod(t[pos].value); pos++; }
                if (pos < t.size()) { clk.waveform_fall = std::stod(t[pos].value); pos++; }
                if (pos < t.size() && t[pos].value == "}") pos++;
            }
            else if (t[pos].value[0] == '-') { pos += 2; } // skip unknown flags
            else if (t[pos].value[0] == '[') { // [get_ports xxx] expression
                pos++; // skip '['
                // Try to extract port from get_ports/get_pins expression
                if (pos < t.size() && (t[pos].value == "get_ports" || t[pos].value == "get_pins" ||
                    t[pos].value == "get_clocks" || t[pos].value == "get_nets")) {
                    pos++; // skip command name
                    if (pos < t.size() && t[pos].value != "]") {
                        if (clk.port.empty()) clk.port = t[pos].value;
                        if (clk.name.empty()) clk.name = t[pos].value;
                    }
                }
                while (pos < t.size() && t[pos].value != "]") pos++;
                if (pos < t.size()) pos++; // skip ']'
            }
            else if (t[pos].value == "create_clock" || t[pos].value == "create_generated_clock" ||
                     t[pos].value == "set_false_path" || t[pos].value == "set_multicycle_path" ||
                     t[pos].value == "set_input_delay" || t[pos].value == "set_output_delay" ||
                     t[pos].value == "set_max_delay" || t[pos].value == "set_min_delay" ||
                     t[pos].value == "set_clock_groups" || t[pos].value == "set_clock_latency") {
                break; // next command — don't consume
            }
            else { // port name
                if (clk.port.empty()) clk.port = t[pos].value;
                if (clk.name.empty()) clk.name = t[pos].value;
                pos++;
                break;
            }
        }
        sdc.clocks.push_back(clk);
        r.num_constraints++;
    }
    else if (cmd == "create_generated_clock") {
        pos++;
        SdcClock clk;
        clk.is_generated = true;
        while (pos < t.size()) {
            if (t[pos].value == "-name" && pos+1 < t.size()) { clk.name = t[++pos].value; pos++; }
            else if (t[pos].value == "-source" && pos+1 < t.size()) {
                pos++;
                if (t[pos].value == "[") {
                    pos++; // skip '['
                    if (pos < t.size() && (t[pos].value == "get_ports" || t[pos].value == "get_pins" ||
                        t[pos].value == "get_clocks")) {
                        pos++; // skip command
                        if (pos < t.size() && t[pos].value != "]")
                            clk.source = t[pos].value;
                    }
                    while (pos < t.size() && t[pos].value != "]") pos++;
                    if (pos < t.size()) pos++; // skip ']'
                } else {
                    clk.source = t[pos].value;
                    pos++;
                }
            }
            else if (t[pos].value == "-divide_by" && pos+1 < t.size()) {
                try { clk.divide_by = std::stoi(t[++pos].value); } catch (...) {}
                pos++;
            }
            else if (t[pos].value == "-multiply_by" && pos+1 < t.size()) {
                try { clk.multiply_by = std::stoi(t[++pos].value); } catch (...) {}
                pos++;
            }
            else if (t[pos].value == "-duty_cycle" && pos+1 < t.size()) {
                try { clk.duty_cycle = std::stod(t[++pos].value); } catch (...) {}
                pos++;
            }
            else if (t[pos].value == "-invert") { clk.invert = true; pos++; }
            else if (t[pos].value == "-edges") {
                pos++;
                if (pos < t.size() && t[pos].value == "{") pos++;
                while (pos < t.size() && t[pos].value != "}") {
                    try { clk.edges.push_back(std::stoi(t[pos].value)); } catch (...) {}
                    pos++;
                }
                if (pos < t.size()) pos++;
            }
            else if (t[pos].value[0] == '[') { while (pos < t.size() && t[pos].value != "]") pos++; pos++; }
            else if (t[pos].value[0] == '-') { pos += 2; }
            else {
                if (clk.port.empty()) clk.port = t[pos].value;
                if (clk.name.empty()) clk.name = t[pos].value;
                pos++;
                break;
            }
        }
        // Compute period from source clock
        if (!clk.source.empty()) {
            auto* src = sdc.find_clock(clk.source);
            if (src) {
                clk.period_ns = src->period_ns * clk.divide_by / clk.multiply_by;
                clk.waveform_fall = clk.period_ns * clk.duty_cycle / 100.0;
            }
        }
        sdc.clocks.push_back(clk);
        r.num_constraints++;
    }
    else if (cmd == "set_input_delay") {
        pos++;
        SdcInputDelay id;
        while (pos < t.size()) {
            if (t[pos].value == "-clock" && pos+1 < t.size()) { id.clock = t[++pos].value; pos++; }
            else if (t[pos].value == "-max") { id.is_max = true; pos++; }
            else if (t[pos].value == "-min") { id.is_max = false; pos++; }
            else if (t[pos].value[0] == '-') { pos += 2; }
            else if (t[pos].value[0] == '[') { while (pos < t.size() && t[pos].value != "]") pos++; pos++; }
            else {
                try { id.delay_ns = std::stod(t[pos].value); pos++; }
                catch (...) { id.port = t[pos].value; pos++; break; }
            }
        }
        sdc.input_delays.push_back(id);
        r.num_constraints++;
    }
    else if (cmd == "set_output_delay") {
        pos++;
        SdcOutputDelay od;
        while (pos < t.size()) {
            if (t[pos].value == "-clock" && pos+1 < t.size()) { od.clock = t[++pos].value; pos++; }
            else if (t[pos].value == "-max") { od.is_max = true; pos++; }
            else if (t[pos].value == "-min") { od.is_max = false; pos++; }
            else if (t[pos].value[0] == '-') { pos += 2; }
            else if (t[pos].value[0] == '[') { while (pos < t.size() && t[pos].value != "]") pos++; pos++; }
            else {
                try { od.delay_ns = std::stod(t[pos].value); pos++; }
                catch (...) { od.port = t[pos].value; pos++; break; }
            }
        }
        sdc.output_delays.push_back(od);
        r.num_constraints++;
    }
    else if (cmd == "set_false_path") {
        pos++;
        SdcException ex{SdcException::FALSE_PATH, "", "", 0, 0, "", true};
        while (pos < t.size()) {
            if (t[pos].value == "-from" && pos+1 < t.size()) { ex.from = t[++pos].value; pos++; }
            else if (t[pos].value == "-to" && pos+1 < t.size()) { ex.to = t[++pos].value; pos++; }
            else if (t[pos].value == "-through" && pos+1 < t.size()) { ex.through = t[++pos].value; pos++; }
            else if (t[pos].value[0] == '[') { while (pos < t.size() && t[pos].value != "]") pos++; pos++; }
            else if (t[pos].value[0] == '-') { pos += 2; }
            else { pos++; break; }
        }
        sdc.exceptions.push_back(ex);
        r.num_constraints++;
    }
    else if (cmd == "set_multicycle_path") {
        pos++;
        SdcException ex{SdcException::MULTICYCLE_PATH, "", "", 0, 2, "", true};
        while (pos < t.size()) {
            if (t[pos].value == "-from" && pos+1 < t.size()) { ex.from = t[++pos].value; pos++; }
            else if (t[pos].value == "-to" && pos+1 < t.size()) { ex.to = t[++pos].value; pos++; }
            else if (t[pos].value == "-through" && pos+1 < t.size()) { ex.through = t[++pos].value; pos++; }
            else if (t[pos].value == "-setup") { ex.is_setup = true; pos++; }
            else if (t[pos].value == "-hold") { ex.is_setup = false; pos++; }
            else if (t[pos].value[0] == '-') { pos += 2; }
            else {
                try { ex.multiplier = std::stoi(t[pos].value); pos++; }
                catch (...) { pos++; break; }
            }
        }
        sdc.exceptions.push_back(ex);
        r.num_constraints++;
    }
    else if (cmd == "set_max_delay") {
        pos++;
        SdcException ex{SdcException::MAX_DELAY, "", "", 0, 1, "", true};
        while (pos < t.size()) {
            if (t[pos].value == "-from" && pos+1 < t.size()) { ex.from = t[++pos].value; pos++; }
            else if (t[pos].value == "-to" && pos+1 < t.size()) { ex.to = t[++pos].value; pos++; }
            else if (t[pos].value == "-through" && pos+1 < t.size()) { ex.through = t[++pos].value; pos++; }
            else if (t[pos].value[0] == '-') { pos += 2; }
            else {
                try { ex.value = std::stod(t[pos].value); pos++; }
                catch (...) { pos++; break; }
            }
        }
        sdc.exceptions.push_back(ex);
        r.num_constraints++;
    }
    else if (cmd == "set_min_delay") {
        pos++;
        SdcException ex{SdcException::MIN_DELAY, "", "", 0, 1, "", true};
        while (pos < t.size()) {
            if (t[pos].value == "-from" && pos+1 < t.size()) { ex.from = t[++pos].value; pos++; }
            else if (t[pos].value == "-to" && pos+1 < t.size()) { ex.to = t[++pos].value; pos++; }
            else if (t[pos].value == "-through" && pos+1 < t.size()) { ex.through = t[++pos].value; pos++; }
            else if (t[pos].value[0] == '-') { pos += 2; }
            else {
                try { ex.value = std::stod(t[pos].value); pos++; }
                catch (...) { pos++; break; }
            }
        }
        sdc.exceptions.push_back(ex);
        r.num_constraints++;
    }
    else if (cmd == "set_clock_groups") {
        pos++;
        SdcClockGroup cg;
        while (pos < t.size()) {
            if (t[pos].value == "-name" && pos+1 < t.size()) { cg.name = t[++pos].value; pos++; }
            else if (t[pos].value == "-asynchronous") { cg.relation = SdcClockGroup::ASYNC; pos++; }
            else if (t[pos].value == "-exclusive") { cg.relation = SdcClockGroup::EXCLUSIVE; pos++; }
            else if (t[pos].value == "-physically_exclusive") { cg.relation = SdcClockGroup::PHYSICALLY_EXCLUSIVE; pos++; }
            else if (t[pos].value == "-group") {
                pos++;
                std::vector<std::string> group;
                if (pos < t.size() && t[pos].value == "{") {
                    pos++;
                    while (pos < t.size() && t[pos].value != "}") {
                        group.push_back(t[pos].value); pos++;
                    }
                    if (pos < t.size()) pos++; // skip }
                } else if (pos < t.size()) {
                    group.push_back(t[pos].value); pos++;
                }
                cg.groups.push_back(group);
            }
            else if (t[pos].value[0] == '-') { pos += 2; }
            else { pos++; break; }
        }
        sdc.clock_groups.push_back(cg);
        r.num_constraints++;
    }
    else if (cmd == "set_max_fanout") {
        pos++;
        double val = 0;
        std::string target;
        while (pos < t.size()) {
            if (t[pos].value[0] == '-') { pos += 2; }
            else {
                try { val = std::stod(t[pos].value); pos++; }
                catch (...) { target = t[pos].value; pos++; break; }
            }
        }
        if (!target.empty()) sdc.max_fanout[target] = val;
        r.num_constraints++;
    }
    else if (cmd == "set_clock_uncertainty") {
        pos++;
        double val = 0;
        std::string clk_name;
        while (pos < t.size()) {
            if (t[pos].value[0] == '-') { pos += 2; }
            else {
                try { val = std::stod(t[pos].value); pos++; }
                catch (...) { clk_name = t[pos].value; pos++; break; }
            }
        }
        for (auto& c : sdc.clocks) {
            if (c.name == clk_name) c.uncertainty = val;
        }
        r.num_constraints++;
    }
    else { pos++; } // skip unknown commands

    return pos;
}

SdcParseResult SdcParser::parse_string(const std::string& src, SdcConstraints& sdc) {
    auto tokens = tokenize(src);
    SdcParseResult r;
    r.success = true;

    size_t pos = 0;
    while (pos < tokens.size()) {
        pos = parse_command(tokens, pos, sdc, r);
    }
    return r;
}

SdcParseResult SdcParser::parse_file(const std::string& filename, SdcConstraints& sdc) {
    std::ifstream f(filename);
    if (!f.is_open()) return {false, 0, "Cannot open: " + filename};
    std::string src((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return parse_string(src, sdc);
}

std::string SdcParser::to_sdc(const SdcConstraints& sdc) {
    std::ostringstream ss;
    ss << "# SiliconForge SDC output\n\n";
    for (auto& c : sdc.clocks) {
        ss << "create_clock -name " << c.name << " -period " << c.period_ns;
        if (!c.port.empty()) ss << " [get_ports " << c.port << "]";
        ss << "\n";
        if (c.uncertainty > 0)
            ss << "set_clock_uncertainty " << c.uncertainty << " " << c.name << "\n";
    }
    ss << "\n";
    for (auto& d : sdc.input_delays)
        ss << "set_input_delay -clock " << d.clock << " " << d.delay_ns << " " << d.port << "\n";
    for (auto& d : sdc.output_delays)
        ss << "set_output_delay -clock " << d.clock << " " << d.delay_ns << " " << d.port << "\n";
    ss << "\n";
    for (auto& e : sdc.exceptions) {
        if (e.type == SdcException::FALSE_PATH)
            ss << "set_false_path -from " << e.from << " -to " << e.to << "\n";
        else if (e.type == SdcException::MULTICYCLE_PATH)
            ss << "set_multicycle_path " << e.multiplier << " -from " << e.from << " -to " << e.to << "\n";
        else if (e.type == SdcException::MAX_DELAY)
            ss << "set_max_delay " << e.value << " -from " << e.from << " -to " << e.to << "\n";
    }
    return ss.str();
}

} // namespace sf
