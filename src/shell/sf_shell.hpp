#pragma once
// SiliconForge — Interactive Shell (REPL)
// Parses text commands and drives the SiliconForge unified engine.
// Supports both native commands and TCL scripting via built-in interpreter.

#include "flow/engine.hpp"
#include "shell/tcl_interp.hpp"
#include <string>

namespace sf {

class SiliconForgeShell {
public:
    SiliconForgeShell();

    // Start parsing interactive commands from stdin.
    void run_interactive();

    // Run a single command string
    bool execute_command(const std::string& cmd_line);

    // Run a script file (auto-detects .tcl for TCL mode)
    bool run_script(const std::string& filename);

    // Access to TCL interpreter for external use
    TclInterp& tcl() { return tcl_; }

private:
    SiliconForge engine_;
    TclInterp tcl_;
    bool tcl_mode_ = false;  // when true, all commands go through TCL interp
    void print_help() const;
};

} // namespace sf
