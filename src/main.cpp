// SiliconForge — Main CLI Entry Point
// Phase 14: The Grand Finale

#include "shell/sf_shell.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace sf;

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════╗
 ║   _____ _ _ _                 _____                   ║
 ║  / ____(_) (_)               |  ___|                  ║
 ║ | (___  _| |_  ___ ___  _ __ | |_ ___  _ __ __ _  ___ ║
 ║  \___ \| | | |/ __/ _ \| '_ \|  _/ _ \| '__/ _` |/ _ \║
 ║  ____) | | | | (_| (_) | | | | || (_) | | | (_| |  __/║
 ║ |_____/|_|_|_|\___\___/|_| |_\_| \___/|_|  \__, |\___|║
 ║                                              __/ |    ║
 ║  RTL-to-GDSII Tool Suite v1.0.0             |___/     ║
 ║  Phase 14: The Grand Finale                           ║
 ╚═══════════════════════════════════════════════════════╝
)" << std::endl;
}

// Execute a script file: uses shell's run_script for TCL support
static bool run_script_file(SiliconForgeShell& shell, const std::string& filename) {
    // .tcl files go through TCL interpreter automatically
    bool is_tcl = (filename.size() > 4 &&
                   filename.substr(filename.size() - 4) == ".tcl");
    if (is_tcl) {
        return shell.run_script(filename);
    }

    // Legacy native script mode
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "Error: cannot open script file: " << filename << "\n";
        return false;
    }
    std::cout << "Executing script: " << filename << "\n";
    std::string line;
    int line_num = 0;
    while (std::getline(ifs, line)) {
        ++line_num;
        if (line.empty()) continue;
        size_t first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        if (line[first] == '#') continue;
        if (line.size() > first + 1 && line[first] == '/' && line[first + 1] == '/') continue;

        std::cout << "sf[" << line_num << "]> " << line << "\n";
        if (!shell.execute_command(line)) {
            std::cout << "Script terminated at line " << line_num << "\n";
            return true;
        }
    }
    std::cout << "Script completed: " << line_num << " lines processed.\n";
    return true;
}

int main(int argc, char** argv) {
    print_banner();
    
    SiliconForgeShell shell;
    
    if (argc > 1) {
        // Run script file(s) provided as arguments
        for (int i = 1; i < argc; ++i) {
            run_script_file(shell, argv[i]);
        }
    } else {
        // Start REPL
        shell.run_interactive();
    }
    
    std::cout << "Exiting SiliconForge.\n";
    return 0;
}
