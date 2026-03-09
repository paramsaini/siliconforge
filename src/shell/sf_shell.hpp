#pragma once
// SiliconForge — Interactive Shell (REPL)
// Parses text commands and drives the SiliconForge unified engine.

#include "flow/engine.hpp"
#include <string>

namespace sf {

class SiliconForgeShell {
public:
    SiliconForgeShell();

    // Start parsing interactive commands from stdin.
    // Returns when 'exit' or 'quit' is typed.
    void run_interactive();

    // Run a single command string
    bool execute_command(const std::string& cmd_line);

private:
    SiliconForge engine_;
    void print_help() const;
};

} // namespace sf
