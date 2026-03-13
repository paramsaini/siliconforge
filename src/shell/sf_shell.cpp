// SiliconForge — Interactive Shell (REPL)
#include "shell/sf_shell.hpp"
#include <iostream>
#include <fstream>
#include <sstream>

namespace sf {

SiliconForgeShell::SiliconForgeShell() {
    tcl_.register_sf_commands(engine_);

    // Add shell-specific TCL commands
    tcl_.register_command("help", [this](auto&) -> std::string {
        print_help();
        return "";
    });
    tcl_.register_command("tcl_mode", [this](auto& args) -> std::string {
        tcl_mode_ = args.empty() || args[0] != "off";
        std::cout << "TCL mode " << (tcl_mode_ ? "ON" : "OFF") << "\n";
        return tcl_mode_ ? "1" : "0";
    });
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
    std::cout << "  cdc                      : Run Clock Domain Crossing Analysis\n";
    std::cout << "  cts                      : Run Clock Tree Synthesis\n";
    std::cout << "  reliability              : Run Reliability & IR-Drop Analysis\n";
    std::cout << "  lec                      : Run Logic Equivalence Checking\n";
    std::cout << "  ai_tune                  : Run AI PnR + ML optimization\n";
    std::cout << "  mcmm                     : Run Multi-Corner Multi-Mode analysis\n";
    std::cout << "  ssta                     : Run Statistical STA (Monte Carlo)\n";
    std::cout << "  ir_drop                  : Run IR Drop analysis\n";
    std::cout << "  pdn                      : Run PDN Impedance analysis\n";
    std::cout << "  si                       : Run Signal Integrity analysis\n";
    std::cout << "  thermal                  : Run Thermal analysis\n";
    std::cout << "  em                       : Run Electromigration analysis\n";
    std::cout << "  noise                    : Run Noise/SSN analysis\n";
    std::cout << "  post_route_opt           : Run Post-Route Optimization\n";
    std::cout << "  chip_assemble            : Run Chip Assembly (I/O pads, bumps)\n";
    std::cout << "  adv_formal               : Run Advanced Formal (IC3/LTL/CEGAR)\n";
    std::cout << "  run_all [w] [h]          : Run full RTL-to-GDSII flow\n";
    std::cout << "  write_gds <file>         : Export GDSII binary\n";
    std::cout << "  write_oasis <file>       : Export OASIS format\n";
    std::cout << "  write_lef <file>         : Export LEF format\n";
    std::cout << "  dashboard <file>         : Generate Ultimate HTML Dashboard\n";
    std::cout << "  export_json <file>       : Dump basic state to JSON (legacy)\n";
    std::cout << "  export_full <file>       : Dump FULL EDA state to JSON (for frontend)\n";
    std::cout << "  source <file>            : Execute a TCL script file\n";
    std::cout << "  tcl_mode [on|off]        : Toggle TCL scripting mode\n";
    std::cout << "  help                     : Print this menu\n";
    std::cout << "  exit / quit              : Exit shell\n\n";
}

bool SiliconForgeShell::execute_command(const std::string& cmd_line) {
    if (cmd_line.empty()) return true;

    // In TCL mode, send everything through the TCL interpreter
    if (tcl_mode_) {
        std::string result = tcl_.eval(cmd_line);
        if (result == "exit") return false;
        return true;
    }

    std::istringstream iss(cmd_line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "help") {
        print_help();
    } else if (cmd == "reset") {
        engine_.reset();
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
    } else if (cmd == "cdc") {
        engine_.run_cdc();
    } else if (cmd == "cts") {
        engine_.run_cts();
    } else if (cmd == "reliability") {
        engine_.run_reliability();
    } else if (cmd == "lec") {
        engine_.run_lec();
    } else if (cmd == "ai_tune") {
        engine_.optimize_pnr_with_ai();
    } else if (cmd == "mcmm") {
        engine_.run_mcmm();
    } else if (cmd == "ssta") {
        engine_.run_ssta();
    } else if (cmd == "ir_drop") {
        engine_.run_ir_drop();
    } else if (cmd == "pdn") {
        engine_.run_pdn();
    } else if (cmd == "si") {
        engine_.run_signal_integrity();
    } else if (cmd == "thermal") {
        engine_.run_thermal();
    } else if (cmd == "em") {
        engine_.run_em();
    } else if (cmd == "noise") {
        engine_.run_noise();
    } else if (cmd == "post_route_opt") {
        engine_.run_post_route_opt();
    } else if (cmd == "chip_assemble") {
        engine_.run_chip_assemble();
    } else if (cmd == "adv_formal") {
        engine_.run_adv_formal();
    } else if (cmd == "run_all") {
        double w = 200, h = 200;
        iss >> w >> h;
        engine_.run_all(w, h);
    } else if (cmd == "write_gds") {
        std::string file;
        if (iss >> file) engine_.write_gds(file);
        else std::cerr << "Usage: write_gds <file>\n";
    } else if (cmd == "write_oasis") {
        std::string file;
        if (iss >> file) engine_.write_oasis(file);
        else std::cerr << "Usage: write_oasis <file>\n";
    } else if (cmd == "write_lef") {
        std::string file;
        if (iss >> file) engine_.write_lef(file);
        else std::cerr << "Usage: write_lef <file>\n";
    } else if (cmd == "run_eco") {
        int mode = 0;
        iss >> mode;
        engine_.run_eco(mode);
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
    } else if (cmd == "source") {
        std::string file;
        if (iss >> file) run_script(file);
        else std::cerr << "Usage: source <file.tcl>\n";
    } else if (cmd == "tcl_mode") {
        std::string arg;
        iss >> arg;
        tcl_mode_ = (arg != "off");
        std::cout << "TCL mode " << (tcl_mode_ ? "ON" : "OFF") << "\n";
    } else {
        // In TCL mode, pass unknown commands to TCL interpreter
        if (tcl_mode_) {
            std::string result = tcl_.eval(cmd_line);
            if (result == "exit") return false;
        } else {
            std::cerr << "Unknown command: " << cmd << "\n";
        }
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

bool SiliconForgeShell::run_script(const std::string& filename) {
    // .tcl files are run through TCL interpreter; others use line-by-line native commands
    bool is_tcl = (filename.size() > 4 &&
                   filename.substr(filename.size() - 4) == ".tcl");
    if (is_tcl) {
        std::cout << "[SiliconForge] Sourcing TCL script: " << filename << "\n";
        std::string result = tcl_.source_file(filename);
        return result != "exit";
    }

    // Native script mode
    std::ifstream f(filename);
    if (!f.is_open()) {
        std::cerr << "Error: cannot open script '" << filename << "'\n";
        return false;
    }
    std::string line;
    while (std::getline(f, line)) {
        // Strip comments
        size_t p = line.find_first_not_of(" \t");
        if (p == std::string::npos) continue;
        if (line[p] == '#' || (p + 1 < line.size() && line[p] == '/' && line[p+1] == '/'))
            continue;
        if (!execute_command(line)) return false;
    }
    return true;
}

} // namespace sf
