// SiliconForge — Main CLI Entry Point
// Phase 14: The Grand Finale

#include "shell/sf_shell.hpp"
#include <iostream>

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

int main(int argc, char** argv) {
    print_banner();
    
    SiliconForgeShell shell;
    
    if (argc > 1) {
        // Run a script if provided, e.g. `./build/siliconforge script.tcl`
        // For now just pass it to interactive mode or build out script runner later
        std::cout << "Loading script: " << argv[1] << "\n";
        // Not fully implemented script reading here, falling back to interactive
    }
    
    // Start REPL
    shell.run_interactive();
    
    std::cout << "Exiting SiliconForge.\n";
    return 0;
}
