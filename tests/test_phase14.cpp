// SiliconForge — Phase 14 Test Suite
// End-To-End Master Flow Test

#include "flow/engine.hpp"
#include <iostream>
#include <fstream>
#include <cassert>

using namespace sf;

static int passed = 0, failed = 0;
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<"\n";failed++;return false;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)

bool test_e2e_flow() {
    std::cout << "Running: End-to-End Flow Validation\n";
    
    // Create a dummy verilog file
    std::ofstream out("test_design.v");
    out << "module test_design ( a, b, c, clk, y );\n";
    out << "  input a, b, c, clk;\n";
    out << "  output y;\n";
    out << "  wire w1, w2;\n";
    out << "  assign w1 = a & b;\n";
    out << "  assign w2 = w1 | c;\n";
    out << "  assign y = w2;\n";
    out << "  // assert property (@(posedge clk) a |=> y);\n";
    out << "endmodule\n";
    out.close();

    SiliconForge engine;
    
    CHECK(engine.read_verilog("test_design.v"), "Engine parsed Verilog");
    CHECK(engine.read_sva("assert property (@(posedge clk) a |=> y);"), "Engine parsed SVA");
    
    CHECK(engine.synthesize(), "Synthesis successful");
    
    CHECK(engine.initialize_floorplan(100.0, 100.0, 10.0), "Floorplan initialized");
    CHECK(engine.place(), "Placement successful");
    CHECK(engine.optimize_pnr_with_ai(), "AI Tuner executed");
    CHECK(engine.route(), "Routing successful");
    
    // Verification
    CHECK(engine.run_drc(), "DRC clean");
    CHECK(engine.run_lvs(), "LVS clean");
    CHECK(engine.run_sta(), "STA passed");
    
    // Export
    CHECK(engine.write_gds("sf_final_layout.gds"), "GDSII generated");
    CHECK(engine.generate_dashboard("siliconforge.html"), "HTML Dashboard generated");
    
    PASS("End-to-End Flow Compilation");
    return true;
}

int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 14 — The Grand Finale        ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    test_e2e_flow();

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";

    return failed > 0 ? 1 : 0;
}
