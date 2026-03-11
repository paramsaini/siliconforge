// SiliconForge — Built-in TCL-compatible script interpreter
#include "shell/tcl_interp.hpp"
#include "flow/engine.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace sf {

// ── Exception for control flow ───────────────────────────────────────
struct TclReturn : std::runtime_error {
    std::string value;
    TclReturn(const std::string& v) : std::runtime_error("return"), value(v) {}
};

// ── Constructor ──────────────────────────────────────────────────────
TclInterp::TclInterp() {
    register_builtins();
}

void TclInterp::register_command(const std::string& name, CmdFunc func) {
    commands_[name] = std::move(func);
}

void TclInterp::set_var(const std::string& name, const std::string& value) {
    vars_[name] = value;
}

std::string TclInterp::get_var(const std::string& name) const {
    auto it = vars_.find(name);
    return (it != vars_.end()) ? it->second : "";
}

bool TclInterp::has_var(const std::string& name) const {
    return vars_.count(name) > 0;
}

// ── Tokenize a single command line ───────────────────────────────────
std::vector<std::string> TclInterp::tokenize(const std::string& cmd) {
    std::vector<std::string> tokens;
    size_t i = 0;
    while (i < cmd.size()) {
        // Skip whitespace
        while (i < cmd.size() && (cmd[i] == ' ' || cmd[i] == '\t')) i++;
        if (i >= cmd.size()) break;

        std::string tok;
        if (cmd[i] == '"') {
            // Quoted string — substitution applies inside
            i++;
            while (i < cmd.size() && cmd[i] != '"') {
                if (cmd[i] == '\\' && i + 1 < cmd.size()) {
                    i++;
                    if (cmd[i] == 'n') tok += '\n';
                    else if (cmd[i] == 't') tok += '\t';
                    else tok += cmd[i];
                } else {
                    tok += cmd[i];
                }
                i++;
            }
            if (i < cmd.size()) i++; // skip closing "
            tok = substitute(tok);
        } else if (cmd[i] == '{') {
            // Braced string — NO substitution
            i++;
            int depth = 1;
            while (i < cmd.size() && depth > 0) {
                if (cmd[i] == '{') depth++;
                else if (cmd[i] == '}') { depth--; if (depth == 0) break; }
                tok += cmd[i];
                i++;
            }
            if (i < cmd.size()) i++; // skip closing }
        } else {
            // Bare word — handle [...] command substitution as unit
            while (i < cmd.size() && cmd[i] != ' ' && cmd[i] != '\t') {
                if (cmd[i] == '[') {
                    // Capture entire [...] including spaces
                    tok += cmd[i]; i++;
                    int bd = 1;
                    while (i < cmd.size() && bd > 0) {
                        if (cmd[i] == '[') bd++;
                        else if (cmd[i] == ']') bd--;
                        tok += cmd[i]; i++;
                    }
                } else {
                    tok += cmd[i]; i++;
                }
            }
            tok = substitute(tok);
        }
        tokens.push_back(tok);
    }
    return tokens;
}

// ── Variable and command substitution ────────────────────────────────
std::string TclInterp::substitute(const std::string& str) {
    std::string result;
    for (size_t i = 0; i < str.size(); i++) {
        if (str[i] == '$' && i + 1 < str.size()) {
            i++;
            std::string varname;
            if (str[i] == '{') {
                i++;
                while (i < str.size() && str[i] != '}') varname += str[i++];
                if (i < str.size()) i++; // skip }
                i--; // outer loop will i++
            } else {
                while (i < str.size() && (isalnum(str[i]) || str[i] == '_'))
                    varname += str[i++];
                i--; // outer loop will i++
            }
            result += get_var(varname);
        } else if (str[i] == '[') {
            // Command substitution
            i++;
            int depth = 1;
            std::string subcmd;
            while (i < str.size() && depth > 0) {
                if (str[i] == '[') depth++;
                else if (str[i] == ']') { depth--; if (depth == 0) break; }
                subcmd += str[i];
                i++;
            }
            result += eval_command(subcmd);
        } else if (str[i] == '\\' && i + 1 < str.size()) {
            i++;
            if (str[i] == 'n') result += '\n';
            else if (str[i] == 't') result += '\t';
            else result += str[i];
        } else {
            result += str[i];
        }
    }
    return result;
}

// ── Evaluate a single command ────────────────────────────────────────
std::string TclInterp::eval_command(const std::string& cmd) {
    auto tokens = tokenize(cmd);
    if (tokens.empty()) return "";

    std::string name = tokens[0];
    std::vector<std::string> args(tokens.begin() + 1, tokens.end());

    // Check user-defined procs first
    auto pit = procs_.find(name);
    if (pit != procs_.end()) {
        auto& proc = pit->second;
        // Save vars, set args
        auto saved = vars_;
        for (size_t i = 0; i < proc.args.size() && i < args.size(); i++)
            vars_[proc.args[i]] = args[i];
        std::string result;
        try {
            result = eval(proc.body);
        } catch (TclReturn& tr) {
            result = tr.value;
        }
        vars_ = saved;
        return result;
    }

    // Built-in commands
    auto cit = commands_.find(name);
    if (cit != commands_.end()) {
        return cit->second(args);
    }

    std::cerr << "TCL Error: unknown command '" << name << "'\n";
    return "";
}

// ── Evaluate a multi-line script ─────────────────────────────────────
std::string TclInterp::eval(const std::string& script) {
    std::string result;
    std::string current_cmd;
    int brace_depth = 0;
    bool in_quote = false;

    std::istringstream stream(script);
    std::string line;
    while (std::getline(stream, line)) {
        // Strip comments (only at start of non-continuation line)
        if (current_cmd.empty()) {
            size_t p = 0;
            while (p < line.size() && (line[p] == ' ' || line[p] == '\t')) p++;
            if (p < line.size() && (line[p] == '#' || (p + 1 < line.size() && line[p] == '/' && line[p+1] == '/')))
                continue;
        }

        // Handle line continuation
        if (!line.empty() && line.back() == '\\') {
            current_cmd += line.substr(0, line.size() - 1) + " ";
            continue;
        }

        current_cmd += line;

        // Count braces/quotes to determine if command is complete
        brace_depth = 0;
        in_quote = false;
        for (char c : current_cmd) {
            if (c == '"' && brace_depth == 0) in_quote = !in_quote;
            if (!in_quote) {
                if (c == '{') brace_depth++;
                if (c == '}') brace_depth--;
            }
        }

        if (brace_depth <= 0 && !in_quote) {
            // Split on semicolons for multiple commands per line
            std::string segment;
            int seg_brace = 0;
            bool seg_quote = false;
            for (size_t i = 0; i < current_cmd.size(); i++) {
                char c = current_cmd[i];
                if (c == '"' && seg_brace == 0) seg_quote = !seg_quote;
                if (!seg_quote) {
                    if (c == '{') seg_brace++;
                    if (c == '}') seg_brace--;
                }
                if (c == ';' && seg_brace == 0 && !seg_quote) {
                    std::string trimmed = segment;
                    while (!trimmed.empty() && (trimmed.front() == ' ' || trimmed.front() == '\t'))
                        trimmed.erase(trimmed.begin());
                    while (!trimmed.empty() && (trimmed.back() == ' ' || trimmed.back() == '\t'))
                        trimmed.pop_back();
                    if (!trimmed.empty()) result = eval_command(trimmed);
                    segment.clear();
                } else {
                    segment += c;
                }
            }
            // Process remaining segment
            std::string trimmed = segment;
            while (!trimmed.empty() && (trimmed.front() == ' ' || trimmed.front() == '\t'))
                trimmed.erase(trimmed.begin());
            while (!trimmed.empty() && (trimmed.back() == ' ' || trimmed.back() == '\t'))
                trimmed.pop_back();
            if (!trimmed.empty()) result = eval_command(trimmed);

            current_cmd.clear();
            brace_depth = 0;
            in_quote = false;
        } else {
            current_cmd += "\n";
        }
    }
    return result;
}

// ── Source a file ────────────────────────────────────────────────────
std::string TclInterp::source_file(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "TCL Error: cannot open file '" << path << "'\n";
        return "";
    }
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    return eval(content);
}

// ── Expression evaluator (simple arithmetic + comparisons) ───────────
std::string TclInterp::eval_expr(const std::string& expr) {
    // Substitute variables first
    std::string e = substitute(expr);

    // Simple tokenizer for math expressions
    // Supports: + - * / % < > <= >= == != && || ! ( )
    // Returns numeric result as string
    auto trim = [](const std::string& s) {
        size_t a = s.find_first_not_of(" \t");
        size_t b = s.find_last_not_of(" \t");
        return (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
    };
    std::string te = trim(e);

    // Try to evaluate as a simple numeric expression
    try {
        // Handle comparison operators
        for (auto& op : {"<=", ">=", "==", "!=", "&&", "||", "<", ">"}) {
            std::string sop(op);
            size_t pos = te.rfind(sop);
            if (pos != std::string::npos && pos > 0) {
                std::string lhs = eval_expr(te.substr(0, pos));
                std::string rhs = eval_expr(te.substr(pos + sop.size()));
                double l = std::stod(lhs), r = std::stod(rhs);
                bool res = false;
                if (sop == "<=") res = l <= r;
                else if (sop == ">=") res = l >= r;
                else if (sop == "==") res = l == r;
                else if (sop == "!=") res = l != r;
                else if (sop == "&&") res = l && r;
                else if (sop == "||") res = l || r;
                else if (sop == "<") res = l < r;
                else if (sop == ">") res = l > r;
                return res ? "1" : "0";
            }
        }
        // Handle + and - (left to right, lowest precedence after comparisons)
        for (int i = (int)te.size() - 1; i > 0; i--) {
            if ((te[i] == '+' || te[i] == '-') && te[i-1] != 'e' && te[i-1] != 'E') {
                // Check not inside parens
                int pdepth = 0;
                for (int j = 0; j < i; j++) {
                    if (te[j] == '(') pdepth++;
                    if (te[j] == ')') pdepth--;
                }
                if (pdepth == 0) {
                    double l = std::stod(eval_expr(te.substr(0, i)));
                    double r = std::stod(eval_expr(te.substr(i + 1)));
                    double res = (te[i] == '+') ? l + r : l - r;
                    if (res == (int)res) return std::to_string((int)res);
                    return std::to_string(res);
                }
            }
        }
        // Handle * / %
        for (int i = (int)te.size() - 1; i > 0; i--) {
            if (te[i] == '*' || te[i] == '/' || te[i] == '%') {
                int pdepth = 0;
                for (int j = 0; j < i; j++) {
                    if (te[j] == '(') pdepth++;
                    if (te[j] == ')') pdepth--;
                }
                if (pdepth == 0) {
                    double l = std::stod(eval_expr(te.substr(0, i)));
                    double r = std::stod(eval_expr(te.substr(i + 1)));
                    double res = 0;
                    if (te[i] == '*') res = l * r;
                    else if (te[i] == '/') res = (r != 0) ? l / r : 0;
                    else res = (r != 0) ? std::fmod(l, r) : 0;
                    if (res == (int)res) return std::to_string((int)res);
                    return std::to_string(res);
                }
            }
        }
        // Handle parens
        if (!te.empty() && te.front() == '(' && te.back() == ')')
            return eval_expr(te.substr(1, te.size() - 2));
        // Handle unary !
        if (!te.empty() && te.front() == '!')
            return (std::stod(eval_expr(te.substr(1))) == 0) ? "1" : "0";
        // Literal number
        return std::to_string(std::stod(te));
    } catch (...) {
        return te; // Return as string if not numeric
    }
}

// ── Built-in command implementations ─────────────────────────────────
void TclInterp::register_builtins() {
    register_command("set", [this](auto& a) { return cmd_set(a); });
    register_command("puts", [this](auto& a) { return cmd_puts(a); });
    register_command("expr", [this](auto& a) { return cmd_expr(a); });
    register_command("if", [this](auto& a) { return cmd_if(a); });
    register_command("for", [this](auto& a) { return cmd_for(a); });
    register_command("foreach", [this](auto& a) { return cmd_foreach(a); });
    register_command("while", [this](auto& a) { return cmd_while(a); });
    register_command("proc", [this](auto& a) { return cmd_proc(a); });
    register_command("source", [this](auto& a) { return cmd_source(a); });
    register_command("return", [this](auto& a) { return cmd_return(a); });
    register_command("incr", [this](auto& a) { return cmd_incr(a); });
    register_command("list", [this](auto& a) { return cmd_list(a); });
    register_command("llength", [this](auto& a) { return cmd_llength(a); });
    register_command("lindex", [this](auto& a) { return cmd_lindex(a); });
    register_command("string", [this](auto& a) { return cmd_string(a); });
    register_command("info", [this](auto& a) { return cmd_info(a); });
    register_command("catch", [this](auto& a) -> std::string {
        if (a.empty()) return "1";
        try { eval(a[0]); return "0"; } catch (...) { return "1"; }
    });
    register_command("error", [](auto& a) -> std::string {
        std::string msg = a.empty() ? "error" : a[0];
        throw std::runtime_error(msg);
    });
}

std::string TclInterp::cmd_set(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    if (args.size() == 1) return get_var(args[0]);
    vars_[args[0]] = args[1];
    return args[1];
}

std::string TclInterp::cmd_puts(const std::vector<std::string>& args) {
    if (args.empty()) { std::cout << "\n"; return ""; }
    bool nonewline = false;
    size_t idx = 0;
    if (args.size() > 1 && args[0] == "-nonewline") { nonewline = true; idx = 1; }
    std::string msg = (idx < args.size()) ? args[idx] : "";
    std::cout << msg;
    if (!nonewline) std::cout << "\n";
    return "";
}

std::string TclInterp::cmd_expr(const std::vector<std::string>& args) {
    std::string combined;
    for (auto& a : args) combined += a + " ";
    return eval_expr(combined);
}

std::string TclInterp::cmd_if(const std::vector<std::string>& args) {
    // if {cond} {body} ?elseif {cond} {body}? ?else {body}?
    size_t i = 0;
    while (i < args.size()) {
        if (i == 0 || (args[i] == "elseif")) {
            size_t ci = (args[i] == "elseif") ? i + 1 : i;
            if (ci + 1 >= args.size()) break;
            std::string cond_val = eval_expr(args[ci]);
            try {
                if (std::stod(cond_val) != 0) return eval(args[ci + 1]);
            } catch (...) {
                if (!cond_val.empty() && cond_val != "0" && cond_val != "false")
                    return eval(args[ci + 1]);
            }
            i = ci + 2;
        } else if (args[i] == "then") {
            i++; // skip 'then' keyword
        } else if (args[i] == "else") {
            if (i + 1 < args.size()) return eval(args[i + 1]);
            break;
        } else {
            i++;
        }
    }
    return "";
}

std::string TclInterp::cmd_for(const std::vector<std::string>& args) {
    // for {init} {cond} {incr} {body}
    if (args.size() < 4) return "";
    eval(args[0]); // init
    std::string result;
    while (true) {
        std::string cond = eval_expr(args[1]);
        try { if (std::stod(cond) == 0) break; } catch (...) { break; }
        result = eval(args[3]); // body
        eval(args[2]); // increment
    }
    return result;
}

std::string TclInterp::cmd_foreach(const std::vector<std::string>& args) {
    // foreach var list body
    if (args.size() < 3) return "";
    std::string varname = args[0];
    // Parse list
    auto items = tokenize(args[1]);
    std::string result;
    for (auto& item : items) {
        vars_[varname] = item;
        result = eval(args[2]);
    }
    return result;
}

std::string TclInterp::cmd_while(const std::vector<std::string>& args) {
    if (args.size() < 2) return "";
    std::string result;
    while (true) {
        std::string cond = eval_expr(args[0]);
        try { if (std::stod(cond) == 0) break; } catch (...) { break; }
        result = eval(args[1]);
    }
    return result;
}

std::string TclInterp::cmd_proc(const std::vector<std::string>& args) {
    // proc name {args} {body}
    if (args.size() < 3) return "";
    Proc p;
    p.args = tokenize(args[1]);
    p.body = args[2];
    procs_[args[0]] = p;
    return "";
}

std::string TclInterp::cmd_source(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    return source_file(args[0]);
}

std::string TclInterp::cmd_return(const std::vector<std::string>& args) {
    throw TclReturn(args.empty() ? "" : args[0]);
}

std::string TclInterp::cmd_incr(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    int val = 0;
    try { val = std::stoi(get_var(args[0])); } catch (...) {}
    int delta = 1;
    if (args.size() > 1) try { delta = std::stoi(args[1]); } catch (...) {}
    val += delta;
    vars_[args[0]] = std::to_string(val);
    return std::to_string(val);
}

std::string TclInterp::cmd_list(const std::vector<std::string>& args) {
    std::string result;
    for (size_t i = 0; i < args.size(); i++) {
        if (i > 0) result += " ";
        result += args[i];
    }
    return result;
}

std::string TclInterp::cmd_llength(const std::vector<std::string>& args) {
    if (args.empty()) return "0";
    return std::to_string(tokenize(args[0]).size());
}

std::string TclInterp::cmd_lindex(const std::vector<std::string>& args) {
    if (args.size() < 2) return "";
    auto items = tokenize(args[0]);
    int idx = 0;
    try { idx = std::stoi(args[1]); } catch (...) {}
    if (idx < 0 || idx >= (int)items.size()) return "";
    return items[idx];
}

std::string TclInterp::cmd_string(const std::vector<std::string>& args) {
    if (args.size() < 2) return "";
    if (args[0] == "length") return std::to_string(args[1].size());
    if (args[0] == "equal" && args.size() >= 3) return (args[1] == args[2]) ? "1" : "0";
    if (args[0] == "compare" && args.size() >= 3)
        return std::to_string(args[1].compare(args[2]));
    if (args[0] == "toupper") {
        std::string r = args[1];
        std::transform(r.begin(), r.end(), r.begin(), ::toupper);
        return r;
    }
    if (args[0] == "tolower") {
        std::string r = args[1];
        std::transform(r.begin(), r.end(), r.begin(), ::tolower);
        return r;
    }
    if (args[0] == "trim" && args.size() >= 2) {
        std::string r = args[1];
        while (!r.empty() && (r.front() == ' ' || r.front() == '\t')) r.erase(r.begin());
        while (!r.empty() && (r.back() == ' ' || r.back() == '\t')) r.pop_back();
        return r;
    }
    return "";
}

std::string TclInterp::cmd_info(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    if (args[0] == "commands") {
        std::string result;
        for (auto& kv : commands_) result += kv.first + " ";
        for (auto& kv : procs_) result += kv.first + " ";
        return result;
    }
    if (args[0] == "exists" && args.size() >= 2)
        return has_var(args[1]) ? "1" : "0";
    if (args[0] == "procs") {
        std::string result;
        for (auto& kv : procs_) result += kv.first + " ";
        return result;
    }
    return "";
}

// ── Register SiliconForge engine commands as TCL procedures ──────────
void TclInterp::register_sf_commands(SiliconForge& engine) {
    register_command("read_verilog", [&engine](auto& args) -> std::string {
        if (args.empty()) { std::cerr << "Usage: read_verilog <file>\n"; return "0"; }
        return engine.read_verilog(args[0]) ? "1" : "0";
    });
    register_command("read_sva", [&engine](auto& args) -> std::string {
        if (args.empty()) return "0";
        return engine.read_sva(args[0]) ? "1" : "0";
    });
    register_command("synth", [&engine](auto&) -> std::string {
        return engine.synthesize() ? "1" : "0";
    });
    register_command("simulate", [&engine](auto&) -> std::string {
        return engine.run_simulation() ? "1" : "0";
    });
    register_command("formal_bmc", [&engine](auto& args) -> std::string {
        int depth = 10;
        if (!args.empty()) try { depth = std::stoi(args[0]); } catch (...) {}
        return engine.run_formal_bmc(depth) ? "1" : "0";
    });
    register_command("run_dft", [&engine](auto&) -> std::string {
        return engine.run_dft() ? "1" : "0";
    });
    register_command("floorplan", [&engine](auto& args) -> std::string {
        double w = 100, h = 100, rh = 10;
        if (args.size() >= 1) try { w = std::stod(args[0]); } catch (...) {}
        if (args.size() >= 2) try { h = std::stod(args[1]); } catch (...) {}
        if (args.size() >= 3) try { rh = std::stod(args[2]); } catch (...) {}
        return engine.initialize_floorplan(w, h, rh) ? "1" : "0";
    });
    register_command("place", [&engine](auto&) -> std::string {
        return engine.place() ? "1" : "0";
    });
    register_command("route", [&engine](auto&) -> std::string {
        return engine.route() ? "1" : "0";
    });
    register_command("drc", [&engine](auto&) -> std::string {
        return engine.run_drc() ? "1" : "0";
    });
    register_command("lvs", [&engine](auto&) -> std::string {
        return engine.run_lvs() ? "1" : "0";
    });
    register_command("sta", [&engine](auto&) -> std::string {
        return engine.run_sta() ? "1" : "0";
    });
    register_command("power", [&engine](auto&) -> std::string {
        return engine.run_power() ? "1" : "0";
    });
    register_command("cdc", [&engine](auto&) -> std::string {
        return engine.run_cdc() ? "1" : "0";
    });
    register_command("cts", [&engine](auto&) -> std::string {
        return engine.run_cts() ? "1" : "0";
    });
    register_command("reliability", [&engine](auto&) -> std::string {
        return engine.run_reliability() ? "1" : "0";
    });
    register_command("ir_drop", [&engine](auto&) -> std::string {
        return engine.run_reliability() ? "1" : "0";
    });
    register_command("lec", [&engine](auto&) -> std::string {
        return engine.run_lec() ? "1" : "0";
    });
    register_command("ai_tune", [&engine](auto&) -> std::string {
        engine.optimize_pnr_with_ai();
        return "1";
    });
    register_command("run_all", [&engine](auto& args) -> std::string {
        double w = 200, h = 200;
        if (args.size() >= 1) try { w = std::stod(args[0]); } catch (...) {}
        if (args.size() >= 2) try { h = std::stod(args[1]); } catch (...) {}
        return engine.run_all(w, h) ? "1" : "0";
    });
    register_command("write_gds", [&engine](auto& args) -> std::string {
        if (args.empty()) { std::cerr << "Usage: write_gds <file>\n"; return "0"; }
        return engine.write_gds(args[0]) ? "1" : "0";
    });
    register_command("dashboard", [&engine](auto& args) -> std::string {
        if (args.empty()) { std::cerr << "Usage: dashboard <file>\n"; return "0"; }
        return engine.generate_dashboard(args[0]) ? "1" : "0";
    });
    register_command("export_json", [&engine](auto& args) -> std::string {
        if (args.empty()) { std::cerr << "Usage: export_json <file>\n"; return "0"; }
        return engine.write_json(args[0]) ? "1" : "0";
    });
    register_command("export_full", [&engine](auto& args) -> std::string {
        if (args.empty()) { std::cerr << "Usage: export_full <file>\n"; return "0"; }
        return engine.write_full_json(args[0]) ? "1" : "0";
    });
    register_command("reset", [&engine](auto&) -> std::string {
        engine.reset();
        return "1";
    });
    register_command("exit", [](auto&) -> std::string { return "exit"; });
    register_command("quit", [](auto&) -> std::string { return "exit"; });
}

} // namespace sf
