// SiliconForge — Phase 12 Test Suite
// Generates RTL, Simulation, and Logic Verification Visualizations
// to standalone HTML files for user inspection.

#include "viz/rtl_viz.hpp"
#include "viz/sim_viz.hpp"
#include "viz/logic_viz.hpp"
#include "core/netlist.hpp"
#include "sim/simulator.hpp"
#include <iostream>
#include <vector>
#include <cassert>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// Make a simple 4-bit synchronous counter
static Netlist make_counter() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId rst = nl.add_net("rst"); nl.mark_input(rst);
    
    std::vector<NetId> q(4), d(4);
    for (int i=0; i<4; ++i) {
        q[i] = nl.add_net("q[" + std::to_string(i) + "]");
        nl.mark_output(q[i]);
        d[i] = nl.add_net("d[" + std::to_string(i) + "]");
    }
    
    // Half adders for toggle logic (very naive netlist just to get nodes)
    // q0 toggles
    NetId t0 = nl.add_gate(GateType::NOT, {q[0]}, -1, "NOT0");
    // rst mux
    nl.add_gate(GateType::MUX, {t0, nl.add_net("ZERO"), rst}, d[0], "MUX0");
    
    for (int i=1; i<4; ++i) {
        NetId carry = q[i-1];
        if (i > 1) carry = nl.add_gate(GateType::AND, {q[i-1], q[i-2]}, -1, "AND" + std::to_string(i));
        NetId sum = nl.add_gate(GateType::XOR, {q[i], carry}, -1, "XOR" + std::to_string(i));
        nl.add_gate(GateType::MUX, {sum, nl.add_net("ZERO"), rst}, d[i], "MUX" + std::to_string(i));
    }
    
    for (int i=0; i<4; ++i) {
        nl.add_dff(d[i], clk, q[i], -1, "FF" + std::to_string(i));
    }
    return nl;
}

TEST(rtl_viz) {
    Netlist nl = make_counter();
    RtlVisualizer viz(nl);
    
    bool ok = viz.export_to_file("rtl_out.html");
    CHECK(ok, "wrote rtl_out.html");
    PASS("rtl_viz");
}

TEST(sim_viz) {
    Netlist nl = make_counter();
    EventSimulator sim(nl);
    
    // Create clock and reset vectors
    std::vector<TestVector> vectors;
    for (int t = 0; t < 200; t += 10) {
        TestVector tv;
        tv.time = t;
        tv.assignments.push_back({0 /*clk*/, (t % 20 < 10) ? Logic4::ONE : Logic4::ZERO});
        tv.assignments.push_back({1 /*rst*/, (t < 30) ? Logic4::ONE : Logic4::ZERO});
        vectors.push_back(tv);
    }
    
    sim.run(vectors, 200);
    
    SimVisualizer viz(nl, sim.trace());
    bool ok = viz.export_to_file("sim_out.html");
    CHECK(ok, "wrote sim_out.html");
    PASS("sim_viz");
}

TEST(logic_viz) {
    // Fake some metrics to show the dashboard
    FormalMetrics f;
    f.max_depth_unrolled = 25;
    f.properties_proved = 4;
    f.properties_failed = 1;
    f.cex_cycle = 12;
    
    SynthesisMetrics s;
    s.initial_nodes = 450;
    s.final_nodes = 210;
    s.node_count_history = {450, 410, 395, 360, 320, 315, 290, 240, 220, 210};
    s.pass_names = {"init", "b1", "rw", "rf", "rw", "b2", "rw", "rf", "rw", "b3"};
    
    DftMetrics d;
    d.total_faults = 1500;
    d.detected_faults = 1480;
    d.coverage_pct = 98.6;
    d.scan_chain_length = 150;
    
    LogicVisualizer viz;
    viz.set_formal(f);
    viz.set_synth(s);
    viz.set_dft(d);
    
    bool ok = viz.export_to_file("logic_out.html");
    CHECK(ok, "wrote logic_out.html");
    PASS("logic_viz");
}

int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 12 — Visualizer Test Suite   ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    RUN(rtl_viz);
    RUN(sim_viz);
    RUN(logic_viz);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";

    return failed > 0 ? 1 : 0;
}
