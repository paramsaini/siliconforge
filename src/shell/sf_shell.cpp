// SiliconForge — Interactive Shell (REPL)
#include "shell/sf_shell.hpp"
#include <iostream>
#include <sstream>

namespace sf {

SiliconForgeShell::SiliconForgeShell() {
}

void SiliconForgeShell::print_help() const {
    std::cout << "\nSiliconForge Interactive Shell Commands:\n";
    std::cout << "  read_verilog <file>      : Parse Verilog RTL into Netlist\n";
    std::cout << "  read_sva <props>         : Parse and synthesize SVA into Netlist\n";
    std::cout << "  synth                    : Map Netlist to standard cells\n";
    std::cout << "  simulate                 : Run event-driven simulation\n";
    std::cout << "  formal_bmc <depth>       : Run Bounded Model Checking\n";
    std::cout << "  run_dft                  : Run PODEM ATPG + Fault Simulation\n";
    std::cout << "  floorplan <w> <h> <rh>   : Initialize floorplan (width, height, row_height)\n";
    std::cout << "  place                    : Analytical Global + Detailed Placement\n";
    std::cout << "  route                    : Multi-Threaded Detailed Routing\n";
    std::cout << "  drc                      : Run Design Rule Check\n";
    std::cout << "  lvs                      : Run Layout vs Schematic check\n";
    std::cout << "  sta                      : Static Timing Analysis\n";
    std::cout << "  power                    : Run Power Analysis\n";
    std::cout << "  ai_tune                  : Run AI PnR optimization\n";
    std::cout << "  run_all [w] [h]          : Run full RTL-to-GDSII flow\n";
    std::cout << "  write_gds <file>         : Export GDSII binary\n";
    std::cout << "  dashboard <file>         : Generate Ultimate HTML Dashboard\n";
    std::cout << "  export_json <file>       : Dump basic state to JSON (legacy)\n";
    std::cout << "  export_full <file>       : Dump FULL EDA state to JSON (for frontend)\n";
    std::cout << "  help                     : Print this menu\n";
    std::cout << "  exit / quit              : Exit shell\n\n";
}

bool SiliconForgeShell::execute_command(const std::string& cmd_line) {
    if (cmd_line.empty()) return true;

    std::istringstream iss(cmd_line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "help") {
        print_help();
    } else if (cmd == "read_verilog") {
        std::string filename;
        if (iss >> filename) engine_.read_verilog(filename);
        else std::cerr << "Usage: read_verilog <file>\n";
    } else if (cmd == "read_sva") {
        std::string props;
        std::getline(iss, props); // get rest of line
        if (!props.empty()) engine_.read_sva(props);
        else std::cerr << "Usage: read_sva <properties>\n";
    } else if (cmd == "synth") {
        engine_.synthesize();
    } else if (cmd == "simulate") {
        engine_.run_simulation();
    } else if (cmd == "formal_bmc") {
        int depth = 10;
        iss >> depth;
        engine_.run_formal_bmc(depth);
    } else if (cmd == "run_dft") {
        engine_.run_dft();
    } else if (cmd == "floorplan") {
        double w = 100, h = 100, rh = 10;
        iss >> w >> h >> rh;
        engine_.initialize_floorplan(w, h, rh);
    } else if (cmd == "place") {
        engine_.place();
    } else if (cmd == "route") {
        engine_.route();
    } else if (cmd == "drc") {
        engine_.run_drc();
    } else if (cmd == "lvs") {
        engine_.run_lvs();
    } else if (cmd == "sta") {
        engine_.run_sta();
    } else if (cmd == "power") {
        engine_.run_power();
    } else if (cmd == "ai_tune") {
        engine_.optimize_pnr_with_ai();
    } else if (cmd == "run_all") {
        double w = 200, h = 200;
        iss >> w >> h;
        engine_.run_all(w, h);
    } else if (cmd == "write_gds") {
        std::string file;
        if (iss >> file) engine_.write_gds(file);
        else std::cerr << "Usage: write_gds <file>\n";
    } else if (cmd == "dashboard") {
        std::string file;
        if (iss >> file) engine_.generate_dashboard(file);
        else std::cerr << "Usage: dashboard <file>\n";
    } else if (cmd == "export_json") {
        std::string file;
        if (iss >> file) engine_.write_json(file);
        else std::cerr << "Usage: export_json <file>\n";
    } else if (cmd == "export_full") {
        std::string file;
        if (iss >> file) engine_.write_full_json(file);
        else std::cerr << "Usage: export_full <file>\n";
    } else if (cmd == "exit" || cmd == "quit") {
        return false;
    } else {
        std::cerr << "Unknown command: " << cmd << "\n";
    }
    return true;
}

void SiliconForgeShell::run_interactive() {
    std::cout << "\n";
    std::cout << "==============================================================\n";
    std::cout << "  SiliconForge EDA Suite v15.0 (Frontend Integration)\n";
    std::cout << "==============================================================\n";
    std::cout << "Type 'help' for commands, or 'exit' to quit.\n\n";

    std::string line;
    while (true) {
        std::cout << "sf> ";
        if (!std::getline(std::cin, line)) break;
        if (!execute_command(line)) break;
    }
}

} // namespace sf
