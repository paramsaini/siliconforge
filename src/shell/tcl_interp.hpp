#pragma once
// SiliconForge — Built-in TCL-compatible script interpreter
// Supports: set, puts, expr, if/elseif/else, for, foreach, while, proc, source,
//           variable substitution ($var, ${var}), command substitution [cmd],
//           quoting (", {}), comments (#), and all SiliconForge EDA commands.
//
// Industry EDA tools (Synopsys DC, Cadence Innovus, etc.) use TCL as their
// scripting language. This interpreter provides TCL-compatible syntax so
// scripts written for SiliconForge can use standard TCL idioms.

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>

namespace sf {

class SiliconForge;

class TclInterp {
public:
    using CmdFunc = std::function<std::string(const std::vector<std::string>&)>;

    TclInterp();

    // Register a native command
    void register_command(const std::string& name, CmdFunc func);

    // Evaluate a TCL script string (may contain multiple commands)
    std::string eval(const std::string& script);

    // Source a TCL file
    std::string source_file(const std::string& path);

    // Variable access
    void set_var(const std::string& name, const std::string& value);
    std::string get_var(const std::string& name) const;
    bool has_var(const std::string& name) const;

    // Register all SiliconForge engine commands
    void register_sf_commands(SiliconForge& engine);

private:
    std::unordered_map<std::string, std::string> vars_;
    std::unordered_map<std::string, CmdFunc> commands_;
    // User-defined procs: name → {args_list, body}
    struct Proc { std::vector<std::string> args; std::string body; };
    std::unordered_map<std::string, Proc> procs_;

    // Parsing helpers
    std::string eval_command(const std::string& cmd);
    std::vector<std::string> tokenize(const std::string& cmd);
    std::string substitute(const std::string& str);
    std::string eval_expr(const std::string& expr);

    // Built-in commands
    void register_builtins();
    std::string cmd_set(const std::vector<std::string>& args);
    std::string cmd_puts(const std::vector<std::string>& args);
    std::string cmd_expr(const std::vector<std::string>& args);
    std::string cmd_if(const std::vector<std::string>& args);
    std::string cmd_for(const std::vector<std::string>& args);
    std::string cmd_foreach(const std::vector<std::string>& args);
    std::string cmd_while(const std::vector<std::string>& args);
    std::string cmd_proc(const std::vector<std::string>& args);
    std::string cmd_source(const std::vector<std::string>& args);
    std::string cmd_return(const std::vector<std::string>& args);
    std::string cmd_incr(const std::vector<std::string>& args);
    std::string cmd_list(const std::vector<std::string>& args);
    std::string cmd_llength(const std::vector<std::string>& args);
    std::string cmd_lindex(const std::vector<std::string>& args);
    std::string cmd_string(const std::vector<std::string>& args);
    std::string cmd_info(const std::vector<std::string>& args);
};

} // namespace sf
