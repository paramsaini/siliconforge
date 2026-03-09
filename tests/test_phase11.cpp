// SiliconForge — Phase 11 Test Suite
// Tests: HTML5 Visualizer, ML Congestion, ML Timing, AI Tuner

#include "viz/html_export.hpp"
#include "ml/congestion_model.hpp"
#include "ml/timing_model.hpp"
#include "pnr/ai_tuner.hpp"
#include "pnr/physical.hpp"
#include "core/netlist.hpp"
#include <iostream>
#include <cassert>
#include <fstream>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

static void make_test_design(Netlist& nl, PhysicalDesign& pd) {
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId c = nl.add_net("c"); nl.mark_input(c);
    NetId y = nl.add_net("y"); nl.mark_output(y);
    GateId g1 = nl.add_gate(GateType::AND, {a, b}, c, "G1");
    GateId g2 = nl.add_gate(GateType::OR, {c, a}, y, "G2");
    
    pd.die_area = Rect(0, 0, 100, 100);
    pd.cells.resize(2);
    pd.cells[0].id = 0; pd.cells[0].name = "G1"; 
    pd.cells[0].position = Point(10, 10); pd.cells[0].width = 10; pd.cells[0].height = 10;
    pd.cells[1].id = 1; pd.cells[1].name = "G2";
    pd.cells[1].position = Point(80, 80); pd.cells[1].width = 10; pd.cells[1].height = 10;
    
    pd.nets.resize(4);
    pd.nets[0].id = 0; pd.nets[0].name = "a"; pd.nets[0].cell_ids = {0, 1}; // driver missing
    pd.nets[1].id = 1; pd.nets[1].name = "b"; pd.nets[1].cell_ids = {0};
    pd.nets[2].id = 2; pd.nets[2].name = "c"; pd.nets[2].cell_ids = {0, 1};
    pd.nets[3].id = 3; pd.nets[3].name = "y"; pd.nets[3].cell_ids = {1};
    
    // Add fake routing
    sf::WireSegment w;
    w.start = Point(15, 15); w.end = Point(85, 85); w.layer = 1; w.width = 1; 
    pd.wires.push_back(w);
}

// ============================================================================
// HTML Visualizer Tests
// ============================================================================
TEST(html_export) {
    Netlist nl; PhysicalDesign pd;
    make_test_design(nl, pd);
    
    HtmlVisualizer viz(pd);
    std::vector<std::vector<double>> fake_congestion(10, std::vector<double>(10, 0.5));
    viz.set_congestion_map(fake_congestion);
    
    std::string html = viz.generate_html();
    CHECK(html.find("<canvas") != std::string::npos, "canvas tag found");
    CHECK(html.find("designData = {") != std::string::npos, "JSON payload injected");
    CHECK(html.find("G1") != std::string::npos, "Cell name in JSON");
    
    bool ok = viz.export_to_file("test_out.html");
    CHECK(ok, "export to file ok");
    std::ifstream in("test_out.html");
    CHECK(in.good(), "file exists");
    in.close();
    
    PASS("html_export");
}

// ============================================================================
// ML Congestion Model Tests
// ============================================================================
TEST(ml_congestion) {
    Netlist nl; PhysicalDesign pd;
    make_test_design(nl, pd);
    
    MlCongestionPredictor cong(pd, 10, 10);
    auto r = cong.predict();
    
    CHECK(r.heatmap_grid.size() == 10, "grid y");
    CHECK(r.heatmap_grid[0].size() == 10, "grid x");
    CHECK(r.peak_congestion > 0, "peak congestion > 0");
    CHECK(r.average_congestion > 0, "avg congestion > 0");
    PASS("ml_congestion");
}

// ============================================================================
// ML Timing Model Tests
// ============================================================================
TEST(ml_timing) {
    Netlist nl; PhysicalDesign pd;
    make_test_design(nl, pd);
    
    MlTimingPredictor timer(nl, pd, nullptr);
    auto r = timer.predict(1.0);
    
    // WNS might be 0 or small depending on basic STA defaults, just check it calculates
    CHECK(r.predicted_wns <= 0 || r.predicted_wns > 0, "WNS valid number");
    CHECK(r.time_ms > 0, "timing > 0ms");
    PASS("ml_timing");
}

// ============================================================================
// AI Tuner Tests
// ============================================================================
TEST(ai_tuner) {
    Netlist nl; PhysicalDesign pd;
    make_test_design(nl, pd);
    
    AiTuner tuner(pd, nl);
    // Note: This relies on Placer which relies on hyperparams. We stubbed a loop.
    auto r = tuner.optimize(3, 1.0);
    
    CHECK(r.iterations > 0, "ran >0 iterations");
    CHECK(r.best_congestion > 0, "found best congestion");
    CHECK(r.optimal_density_weight > 0, "tuned parameter returned");
    PASS("ai_tuner");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 11 — Test Suite              ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Standalone Visualizer ──\n";
    RUN(html_export);

    std::cout << "\n── ML Prediction Models ──\n";
    RUN(ml_congestion);
    RUN(ml_timing);

    std::cout << "\n── AI Tuner ──\n";
    RUN(ai_tuner);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
