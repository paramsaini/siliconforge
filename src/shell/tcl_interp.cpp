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
                result += get_var(varname);
            } else {
                while (i < str.size() && (isalnum(str[i]) || str[i] == '_' || str[i] == ':'))
                    varname += str[i++];
                // Check for array reference: $arr(key)
                if (i < str.size() && str[i] == '(') {
                    i++; // skip (
                    std::string key;
                    while (i < str.size() && str[i] != ')') key += str[i++];
                    // Substitute variables in key
                    key = substitute(key);
                    result += get_array(varname, key);
                } else {
                    i--; // outer loop will i++
                    result += get_var(varname);
                }
            }
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
    // Industrial: new commands
    register_command("array", [this](auto& a) { return cmd_array(a); });
    register_command("lappend", [this](auto& a) { return cmd_lappend(a); });
    register_command("lsort", [this](auto& a) { return cmd_lsort(a); });
    register_command("lsearch", [this](auto& a) { return cmd_lsearch(a); });
    register_command("lrange", [this](auto& a) { return cmd_lrange(a); });
    register_command("lreplace", [this](auto& a) { return cmd_lreplace(a); });
    register_command("join", [this](auto& a) { return cmd_join(a); });
    register_command("split", [this](auto& a) { return cmd_split(a); });
    register_command("append", [this](auto& a) { return cmd_append(a); });
    register_command("format", [this](auto& a) { return cmd_format(a); });
    register_command("regexp", [this](auto& a) { return cmd_regexp(a); });
    register_command("regsub", [this](auto& a) { return cmd_regsub(a); });
    register_command("catch", [this](auto& a) { return cmd_catch(a); });
    register_command("error", [this](auto& a) { return cmd_error(a); });
    register_command("switch", [this](auto& a) { return cmd_switch(a); });
    register_command("namespace", [this](auto& a) { return cmd_namespace(a); });
    register_command("global", [this](auto& a) { return cmd_global(a); });
    register_command("upvar", [this](auto& a) { return cmd_upvar(a); });
    register_command("unset", [this](auto& a) { return cmd_unset(a); });
    register_command("concat", [this](auto& a) { return cmd_concat(a); });
    register_command("eval", [this](auto& a) { return cmd_eval_cmd(a); });
    register_command("lmap", [this](auto& a) { return cmd_lmap(a); });
    register_command("break", [](auto&) -> std::string { throw std::string("break"); });
    register_command("continue", [](auto&) -> std::string { throw std::string("continue"); });
}

std::string TclInterp::cmd_set(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    // Handle array syntax: set arr(key) value
    std::string name = args[0];
    auto paren = name.find('(');
    if (paren != std::string::npos && name.back() == ')') {
        std::string arrname = name.substr(0, paren);
        std::string key = name.substr(paren + 1, name.size() - paren - 2);
        if (args.size() == 1) return get_array(arrname, key);
        set_array(arrname, key, args[1]);
        return args[1];
    }
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
        try {
            result = eval(args[3]); // body
        } catch (const std::string& ctrl) {
            if (ctrl == "break") break;
            if (ctrl == "continue") { eval(args[2]); continue; }
            throw;
        }
        eval(args[2]); // increment
    }
    return result;
}

std::string TclInterp::cmd_foreach(const std::vector<std::string>& args) {
    // foreach var list body
    if (args.size() < 3) return "";
    std::string varname = args[0];
    auto items = tokenize(args[1]);
    std::string result;
    for (auto& item : items) {
        vars_[varname] = item;
        try {
            result = eval(args[2]);
        } catch (const std::string& ctrl) {
            if (ctrl == "break") break;
            if (ctrl == "continue") continue;
            throw;
        }
    }
    return result;
}

std::string TclInterp::cmd_while(const std::vector<std::string>& args) {
    if (args.size() < 2) return "";
    std::string result;
    while (true) {
        std::string cond = eval_expr(args[0]);
        try { if (std::stod(cond) == 0) break; } catch (...) { break; }
        try {
            result = eval(args[1]);
        } catch (const std::string& ctrl) {
            if (ctrl == "break") break;
            if (ctrl == "continue") continue;
            throw;
        }
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
    if (args[0] == "trimleft" && args.size() >= 2) {
        std::string r = args[1];
        while (!r.empty() && (r.front() == ' ' || r.front() == '\t')) r.erase(r.begin());
        return r;
    }
    if (args[0] == "trimright" && args.size() >= 2) {
        std::string r = args[1];
        while (!r.empty() && (r.back() == ' ' || r.back() == '\t')) r.pop_back();
        return r;
    }
    if (args[0] == "index" && args.size() >= 3) {
        int idx = 0;
        try { idx = std::stoi(args[2]); } catch (...) {}
        if (idx >= 0 && idx < (int)args[1].size()) return std::string(1, args[1][idx]);
        return "";
    }
    if (args[0] == "range" && args.size() >= 4) {
        int first = 0, last = (int)args[1].size() - 1;
        try { first = std::stoi(args[2]); } catch (...) {}
        if (args[3] == "end") last = (int)args[1].size() - 1;
        else try { last = std::stoi(args[3]); } catch (...) {}
        if (first < 0) first = 0;
        if (last >= (int)args[1].size()) last = (int)args[1].size() - 1;
        if (first > last) return "";
        return args[1].substr(first, last - first + 1);
    }
    if (args[0] == "first" && args.size() >= 3) {
        size_t pos = args[2].find(args[1]);
        return (pos != std::string::npos) ? std::to_string(pos) : "-1";
    }
    if (args[0] == "last" && args.size() >= 3) {
        size_t pos = args[2].rfind(args[1]);
        return (pos != std::string::npos) ? std::to_string(pos) : "-1";
    }
    if (args[0] == "repeat" && args.size() >= 3) {
        std::string result;
        int count = 0;
        try { count = std::stoi(args[2]); } catch (...) {}
        for (int i = 0; i < count; i++) result += args[1];
        return result;
    }
    if (args[0] == "replace" && args.size() >= 4) {
        int first = 0, last = 0;
        try { first = std::stoi(args[2]); } catch (...) {}
        try { last = std::stoi(args[3]); } catch (...) {}
        std::string rep = (args.size() >= 5) ? args[4] : "";
        std::string s = args[1];
        if (first >= 0 && last < (int)s.size() && first <= last)
            return s.substr(0, first) + rep + s.substr(last + 1);
        return s;
    }
    if (args[0] == "match" && args.size() >= 3) {
        // Glob-style match (simplified)
        return (args[2].find(args[1]) != std::string::npos) ? "1" : "0";
    }
    if (args[0] == "is" && args.size() >= 3) {
        if (args[1] == "integer") {
            try { std::stoi(args[2]); return "1"; } catch (...) { return "0"; }
        }
        if (args[1] == "double") {
            try { std::stod(args[2]); return "1"; } catch (...) { return "0"; }
        }
        if (args[1] == "alpha") {
            for (char c : args[2]) if (!std::isalpha(c)) return "0";
            return "1";
        }
        return "0";
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
    if (args[0] == "vars") {
        std::string result;
        for (auto& kv : vars_) result += kv.first + " ";
        return result;
    }
    if (args[0] == "globals") {
        std::string result;
        for (auto& kv : vars_) result += kv.first + " ";
        return result;
    }
    return "";
}

// ── Array access methods ────────────────────────────────────────────
void TclInterp::unset_var(const std::string& name) {
    vars_.erase(name);
}

void TclInterp::set_array(const std::string& name, const std::string& key, const std::string& value) {
    arrays_[name][key] = value;
}

std::string TclInterp::get_array(const std::string& name, const std::string& key) const {
    auto ait = arrays_.find(name);
    if (ait == arrays_.end()) return "";
    auto kit = ait->second.find(key);
    return (kit != ait->second.end()) ? kit->second : "";
}

bool TclInterp::has_array(const std::string& name) const {
    return arrays_.count(name) > 0;
}

std::vector<std::string> TclInterp::array_names(const std::string& name) const {
    std::vector<std::string> result;
    auto ait = arrays_.find(name);
    if (ait != arrays_.end()) {
        for (auto& kv : ait->second) result.push_back(kv.first);
    }
    return result;
}

size_t TclInterp::array_size(const std::string& name) const {
    auto ait = arrays_.find(name);
    return (ait != arrays_.end()) ? ait->second.size() : 0;
}

// ── Industrial TCL commands ─────────────────────────────────────────

std::string TclInterp::cmd_array(const std::vector<std::string>& args) {
    if (args.size() < 2) return "";
    std::string subcmd = args[0];
    std::string name = args[1];
    if (subcmd == "set") {
        if (args.size() < 3) return "";
        auto items = tokenize(args[2]);
        for (size_t i = 0; i + 1 < items.size(); i += 2)
            arrays_[name][items[i]] = items[i + 1];
        return "";
    }
    if (subcmd == "get") {
        std::string result;
        auto ait = arrays_.find(name);
        if (ait != arrays_.end()) {
            for (auto& kv : ait->second)
                result += kv.first + " " + kv.second + " ";
        }
        if (!result.empty()) result.pop_back();
        return result;
    }
    if (subcmd == "names") {
        std::string result;
        auto ait = arrays_.find(name);
        if (ait != arrays_.end()) {
            for (auto& kv : ait->second) result += kv.first + " ";
        }
        if (!result.empty()) result.pop_back();
        return result;
    }
    if (subcmd == "size") {
        return std::to_string(array_size(name));
    }
    if (subcmd == "exists") {
        return has_array(name) ? "1" : "0";
    }
    if (subcmd == "unset") {
        arrays_.erase(name);
        return "";
    }
    return "";
}

std::string TclInterp::cmd_lappend(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    std::string& val = vars_[args[0]];
    for (size_t i = 1; i < args.size(); i++) {
        if (!val.empty()) val += " ";
        val += args[i];
    }
    return val;
}

std::string TclInterp::cmd_lsort(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    bool decreasing = false;
    bool integer_sort = false;
    bool unique = false;
    std::string list_str;
    for (size_t i = 0; i < args.size(); i++) {
        if (args[i] == "-decreasing") decreasing = true;
        else if (args[i] == "-increasing") decreasing = false;
        else if (args[i] == "-integer") integer_sort = true;
        else if (args[i] == "-unique") unique = true;
        else list_str = args[i];
    }
    auto items = tokenize(list_str);
    if (integer_sort) {
        std::sort(items.begin(), items.end(), [&](const std::string& a, const std::string& b) {
            try {
                int ia = std::stoi(a), ib = std::stoi(b);
                return decreasing ? ia > ib : ia < ib;
            } catch (...) { return a < b; }
        });
    } else {
        std::sort(items.begin(), items.end());
        if (decreasing) std::reverse(items.begin(), items.end());
    }
    if (unique) {
        items.erase(std::unique(items.begin(), items.end()), items.end());
    }
    std::string result;
    for (size_t i = 0; i < items.size(); i++) {
        if (i > 0) result += " ";
        result += items[i];
    }
    return result;
}

std::string TclInterp::cmd_lsearch(const std::vector<std::string>& args) {
    if (args.size() < 2) return "-1";
    auto items = tokenize(args[0]);
    std::string pattern = args[1];
    for (size_t i = 0; i < items.size(); i++) {
        if (items[i] == pattern) return std::to_string(i);
    }
    return "-1";
}

std::string TclInterp::cmd_lrange(const std::vector<std::string>& args) {
    if (args.size() < 3) return "";
    auto items = tokenize(args[0]);
    int first = 0, last = (int)items.size() - 1;
    try { first = std::stoi(args[1]); } catch (...) {}
    if (args[2] == "end") last = (int)items.size() - 1;
    else try { last = std::stoi(args[2]); } catch (...) {}
    if (first < 0) first = 0;
    if (last >= (int)items.size()) last = (int)items.size() - 1;
    std::string result;
    for (int i = first; i <= last; i++) {
        if (!result.empty()) result += " ";
        result += items[i];
    }
    return result;
}

std::string TclInterp::cmd_lreplace(const std::vector<std::string>& args) {
    if (args.size() < 3) return "";
    auto items = tokenize(args[0]);
    int first = 0, last = 0;
    try { first = std::stoi(args[1]); } catch (...) {}
    try { last = std::stoi(args[2]); } catch (...) {}
    if (first < 0) first = 0;
    if (last >= (int)items.size()) last = (int)items.size() - 1;
    std::vector<std::string> result;
    for (int i = 0; i < first; i++) result.push_back(items[i]);
    for (size_t i = 3; i < args.size(); i++) result.push_back(args[i]);
    for (int i = last + 1; i < (int)items.size(); i++) result.push_back(items[i]);
    std::string out;
    for (size_t i = 0; i < result.size(); i++) {
        if (i > 0) out += " ";
        out += result[i];
    }
    return out;
}

std::string TclInterp::cmd_join(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    auto items = tokenize(args[0]);
    std::string sep = " ";
    if (args.size() >= 2) sep = args[1];
    std::string result;
    for (size_t i = 0; i < items.size(); i++) {
        if (i > 0) result += sep;
        result += items[i];
    }
    return result;
}

std::string TclInterp::cmd_split(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    std::string str = args[0];
    std::string sep = " ";
    if (args.size() >= 2) sep = args[1];
    std::string result;
    size_t pos = 0;
    while (pos < str.size()) {
        size_t found = str.find(sep, pos);
        if (found == std::string::npos) {
            if (!result.empty()) result += " ";
            result += str.substr(pos);
            break;
        }
        if (!result.empty()) result += " ";
        result += str.substr(pos, found - pos);
        pos = found + sep.size();
    }
    return result;
}

std::string TclInterp::cmd_append(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    std::string& val = vars_[args[0]];
    for (size_t i = 1; i < args.size(); i++) val += args[i];
    return val;
}

std::string TclInterp::cmd_format(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    std::string fmt = args[0];
    std::string result;
    size_t ai = 1;
    for (size_t i = 0; i < fmt.size(); i++) {
        if (fmt[i] == '%' && i + 1 < fmt.size()) {
            i++;
            if (fmt[i] == 's' && ai < args.size()) {
                result += args[ai++];
            } else if (fmt[i] == 'd' && ai < args.size()) {
                try { result += std::to_string(std::stoi(args[ai++])); }
                catch (...) { result += args[ai - 1]; }
            } else if (fmt[i] == 'f' && ai < args.size()) {
                try { result += std::to_string(std::stod(args[ai++])); }
                catch (...) { result += args[ai - 1]; }
            } else if (fmt[i] == '%') {
                result += '%';
            } else {
                result += '%';
                result += fmt[i];
            }
        } else {
            result += fmt[i];
        }
    }
    return result;
}

std::string TclInterp::cmd_regexp(const std::vector<std::string>& args) {
    if (args.size() < 2) return "0";
    try {
        std::regex re(args[0]);
        std::smatch match;
        std::string str = args[1];
        bool found = std::regex_search(str, match, re);
        // Store match results in variables if provided
        for (size_t i = 2; i < args.size() && i - 2 < match.size(); i++) {
            vars_[args[i]] = match[i - 2].str();
        }
        return found ? "1" : "0";
    } catch (...) {
        return "0";
    }
}

std::string TclInterp::cmd_regsub(const std::vector<std::string>& args) {
    // regsub pattern string replacement ?varname?
    if (args.size() < 3) return "";
    bool all = false;
    size_t idx = 0;
    if (args[0] == "-all") { all = true; idx = 1; }
    if (idx + 2 >= args.size()) return "";
    try {
        std::regex re(args[idx]);
        std::string result;
        if (all) result = std::regex_replace(args[idx + 1], re, args[idx + 2]);
        else result = std::regex_replace(args[idx + 1], re, args[idx + 2],
                                          std::regex_constants::format_first_only);
        if (idx + 3 < args.size()) {
            vars_[args[idx + 3]] = result;
        }
        return result;
    } catch (...) {
        return args[1];
    }
}

std::string TclInterp::cmd_catch(const std::vector<std::string>& args) {
    if (args.empty()) return "1";
    try {
        std::string result = eval(args[0]);
        if (args.size() >= 2) vars_[args[1]] = result;
        return "0";
    } catch (const TclReturn& tr) {
        if (args.size() >= 2) vars_[args[1]] = tr.value;
        return "0";
    } catch (const std::exception& e) {
        if (args.size() >= 2) vars_[args[1]] = e.what();
        return "1";
    } catch (...) {
        if (args.size() >= 2) vars_[args[1]] = "unknown error";
        return "1";
    }
}

std::string TclInterp::cmd_error(const std::vector<std::string>& args) {
    std::string msg = args.empty() ? "error" : args[0];
    throw std::runtime_error(msg);
}

std::string TclInterp::cmd_switch(const std::vector<std::string>& args) {
    if (args.size() < 2) return "";
    std::string value = args[0];
    // switch value {pattern body pattern body ...}
    if (args.size() == 2) {
        auto items = tokenize(args[1]);
        for (size_t i = 0; i + 1 < items.size(); i += 2) {
            if (items[i] == value || items[i] == "default") {
                return eval(items[i + 1]);
            }
        }
    } else {
        // switch value pattern body pattern body ...
        for (size_t i = 1; i + 1 < args.size(); i += 2) {
            if (args[i] == value || args[i] == "default") {
                return eval(args[i + 1]);
            }
        }
    }
    return "";
}

std::string TclInterp::cmd_namespace(const std::vector<std::string>& args) {
    if (args.empty()) return "";
    if (args[0] == "eval" && args.size() >= 3) {
        std::string old_ns = current_ns_;
        current_ns_ = args[1];
        if (current_ns_.substr(0, 2) != "::") current_ns_ = "::" + current_ns_;
        std::string result = eval(args[2]);
        current_ns_ = old_ns;
        return result;
    }
    if (args[0] == "current") return current_ns_;
    if (args[0] == "exists" && args.size() >= 2) {
        return (ns_vars_.count(args[1]) > 0) ? "1" : "0";
    }
    return "";
}

std::string TclInterp::cmd_global(const std::vector<std::string>& args) {
    // In our simple model, all variables are global already
    // This is a no-op for compatibility
    return "";
}

std::string TclInterp::cmd_upvar(const std::vector<std::string>& args) {
    // upvar ?level? otherVar myVar
    // Simple implementation: create alias in current scope
    if (args.size() < 2) return "";
    size_t idx = 0;
    if (args.size() >= 3 && (args[0] == "1" || args[0] == "#0")) idx = 1;
    std::string other = args[idx];
    std::string mine = args[idx + 1];
    // Copy the value (simplified — real TCL uses reference)
    vars_[mine] = get_var(other);
    return "";
}

std::string TclInterp::cmd_unset(const std::vector<std::string>& args) {
    for (auto& a : args) {
        vars_.erase(a);
        arrays_.erase(a);
    }
    return "";
}

std::string TclInterp::cmd_concat(const std::vector<std::string>& args) {
    std::string result;
    for (size_t i = 0; i < args.size(); i++) {
        if (i > 0) result += " ";
        result += args[i];
    }
    return result;
}

std::string TclInterp::cmd_eval_cmd(const std::vector<std::string>& args) {
    std::string script;
    for (auto& a : args) {
        if (!script.empty()) script += " ";
        script += a;
    }
    return eval(script);
}

std::string TclInterp::cmd_lmap(const std::vector<std::string>& args) {
    // lmap var list body — like foreach but collects results
    if (args.size() < 3) return "";
    std::string varname = args[0];
    auto items = tokenize(args[1]);
    std::string result;
    for (auto& item : items) {
        vars_[varname] = item;
        std::string r = eval(args[2]);
        if (!result.empty()) result += " ";
        result += r;
    }
    return result;
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
