#pragma once
// SiliconForge — Built-in TCL-compatible script interpreter
// Industrial: arrays, namespaces, regex, full list ops, catch/error,
//   upvar/uplevel, global, append, split, join, format, switch, break/continue, unset
//
// Reference: Synopsys DC Shell / Cadence Innovus TCL compatibility
// Industry EDA tools use TCL as their scripting language.

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <regex>

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
    void unset_var(const std::string& name);

    // Array access (industrial)
    void set_array(const std::string& name, const std::string& key, const std::string& value);
    std::string get_array(const std::string& name, const std::string& key) const;
    bool has_array(const std::string& name) const;
    std::vector<std::string> array_names(const std::string& name) const;
    size_t array_size(const std::string& name) const;

    // Namespace access (industrial)
    std::string current_namespace() const { return current_ns_; }

    // Register all SiliconForge engine commands
    void register_sf_commands(SiliconForge& engine);

private:
    std::unordered_map<std::string, std::string> vars_;
    std::unordered_map<std::string, CmdFunc> commands_;
    // User-defined procs: name → {args_list, body}
    struct Proc { std::vector<std::string> args; std::string body; };
    std::unordered_map<std::string, Proc> procs_;

    // Industrial: arrays (name → {key → value})
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> arrays_;

    // TCL dict storage
    struct TclDict {
        std::unordered_map<std::string, std::string> entries;
    };
    std::unordered_map<std::string, TclDict> dicts_;

    // Industrial: namespaces
    std::string current_ns_ = "::";
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> ns_vars_;

    // Industrial: call stack for upvar/uplevel
    struct Frame {
        std::unordered_map<std::string, std::string>* vars;
        std::string ns;
    };
    std::vector<Frame> call_stack_;

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

    // Industrial: new commands
    std::string cmd_array(const std::vector<std::string>& args);
    std::string cmd_lappend(const std::vector<std::string>& args);
    std::string cmd_lsort(const std::vector<std::string>& args);
    std::string cmd_lsearch(const std::vector<std::string>& args);
    std::string cmd_lrange(const std::vector<std::string>& args);
    std::string cmd_lreplace(const std::vector<std::string>& args);
    std::string cmd_join(const std::vector<std::string>& args);
    std::string cmd_split(const std::vector<std::string>& args);
    std::string cmd_append(const std::vector<std::string>& args);
    std::string cmd_format(const std::vector<std::string>& args);
    std::string cmd_regexp(const std::vector<std::string>& args);
    std::string cmd_regsub(const std::vector<std::string>& args);
    std::string cmd_catch(const std::vector<std::string>& args);
    std::string cmd_error(const std::vector<std::string>& args);
    std::string cmd_switch(const std::vector<std::string>& args);
    std::string cmd_namespace(const std::vector<std::string>& args);
    std::string cmd_global(const std::vector<std::string>& args);
    std::string cmd_upvar(const std::vector<std::string>& args);
    std::string cmd_uplevel(const std::vector<std::string>& args);
    std::string cmd_dict(const std::vector<std::string>& args);
    std::string cmd_unset(const std::vector<std::string>& args);
    std::string cmd_concat(const std::vector<std::string>& args);
    std::string cmd_eval_cmd(const std::vector<std::string>& args);
    std::string cmd_lmap(const std::vector<std::string>& args);
};

} // namespace sf
