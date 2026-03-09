// SiliconForge — Phase 13 Test Suite
// Validates SVA, HLS, 3D Packaging, and Advanced Routing Engine.

#include "frontend/sva_parser.hpp"
#include "formal/sva_engine.hpp"
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

TEST(sva_parsing) {
    SvaParser parser;
    auto props = parser.parse("assert property (@(posedge clk) req |=> ack);");
    CHECK(props.size() == 1, "parsed one property");
    CHECK(props[0].clock_domain == "clk", "clock parsed");
    CHECK(props[0].expr->op == SvaOp::PROP_NON_OVERLAPPING, "operator parsed");
    CHECK(props[0].expr->left->literal == "req", "left side");
    CHECK(props[0].expr->right->literal == "ack", "right side");
    PASS("sva_parsing");
}

TEST(sva_engine_sim) {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId req = nl.add_net("req"); nl.mark_input(req);
    NetId ack = nl.add_net("ack"); nl.mark_input(ack);

    // Some dummy logic just so it's not totally empty (not required though)
    NetId y = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {req, ack}, y, "U_AND");

    SvaParser parser;
    auto props = parser.parse("assert property (@(posedge clk) req |=> ack);");
    
    // Synthesize the property into the netlist
    // It will track: if req is 1, in the next cycle, ack must be 1.
    SvaEngine::synthesize_assertions(nl, props);
    
    NetId fail_output = -1;
    for (auto po : nl.primary_outputs()) {
        if (nl.net(po).name.find("sva_fail_") == 0) {
            fail_output = po;
            break;
        }
    }
    CHECK(fail_output != -1, "sva output created");

    EventSimulator sim(nl);
    
    // Cycle 1: req=1, ack=0. 
    // Cycle 2: req=0, ack=0 (violation! ack should have been 1)
    // Change inputs slightly before clock edges to avoid race conditions
    std::vector<TestVector> vectors = {
        {0, {{clk, Logic4::ZERO}, {req, Logic4::ZERO}, {ack, Logic4::ZERO}}},
        {5, {{req, Logic4::ONE},  {ack, Logic4::ZERO}}},
        {10, {{clk, Logic4::ONE}}}, // Posedge 1 (req=1 captured)
        {15, {{clk, Logic4::ZERO}, {req, Logic4::ZERO}}}, // Negedge
        {25, {{req, Logic4::ZERO}, {ack, Logic4::ZERO}}},
        {30, {{clk, Logic4::ONE}}}, // Posedge 2 (req=0 captured, checking previous req=1 -> ack must be 1) Fail!
        {35, {{clk, Logic4::ZERO}}},
    };

    sim.run(vectors, 50);

    auto& trace = sim.trace();
    auto& fail_trace = trace.traces.at(fail_output);
    
    std::cout << "\nTrace Table:\n";
    for (size_t i = 0; i < trace.times.size(); ++i) {
        std::cout << "t=" << trace.times[i] << "  req=" << (int)trace.traces.at(req)[i] 
                  << "  ack=" << (int)trace.traces.at(ack)[i]
                  << "  fail=" << (int)fail_trace[i] << "\n";
    }

    bool failed_assertion = false;
    for (size_t i = 0; i < trace.times.size(); ++i) {
        if (trace.times[i] >= 20 && fail_trace[i] == Logic4::ONE) {
            failed_assertion = true;
        }
    }
    CHECK(failed_assertion, "violation caught by SVA synthesized monitor");
    PASS("sva_engine_sim");
}

#include "hls/c_parser.hpp"

TEST(hls_parser_scheduler) {
    CParser parser;
    auto cdfg = parser.parse("y = a + b ; z = y * c ;");
    
    CHECK(cdfg.size() == 1, "One basic block");
    CHECK(cdfg[0].datapath.size() > 5, "Datapath built");
    
    HlsScheduler::schedule_asap(cdfg);
    
    int max_cycle = 0;
    for (const auto& node : cdfg[0].datapath) {
        if (node.cycle > max_cycle) max_cycle = node.cycle;
        if (node.op == HlsOp::ADD) CHECK(node.cycle >= 1, "ADD scheduled");
        if (node.op == HlsOp::MUL) CHECK(node.cycle >= 2, "MUL scheduled after ADD");
    }
    CHECK(max_cycle >= 3, "Total cycles at least 3");
    
    Netlist nl;
    HlsSynthesizer::synthesize_to_netlist(cdfg, nl);
    CHECK(nl.num_nets() > 5, "Netlist FSM generated");
    
    PASS("hls_parser_scheduler");
}

#include "core/die_to_die.hpp"
#include "pnr/tsv_mgr.hpp"
#include "pnr/physical.hpp"

TEST(tsv_insertion) {
    PackageDesign pkg;
    int die1_id = pkg.add_die("LogicDie", 0);
    int die2_id = pkg.add_die("MemoryDie", 1);
    
    PhysicalDesign d1, d2;
    // Net 10 is shared
    int c1 = d1.add_cell("BUF1", "BUF", 1, 1);
    int c2 = d1.add_cell("BUF2", "BUF", 1, 1);
    d1.cells[c1].position = {10.0f, 10.0f};
    d1.cells[c2].position = {90.0f, 10.0f};
    int n1 = d1.add_net("shared_net", {c1, c2});
    d1.nets[n1].id = 10; // Force ID

    int c3 = d2.add_cell("BUF3", "BUF", 1, 1);
    d2.cells[c3].position = {50.0f, 90.0f};
    int n2 = d2.add_net("shared_net_d2", {c3});
    d2.nets[n2].id = 10; // Force ID
    
    TsvManager::insert_tsvs(pkg, d1, d2, 0, 1);
    
    CHECK(pkg.tsvs().size() == 1, "1 TSV inserted for shared net 10");
    
    const auto& tsv = pkg.tsvs()[0];
    CHECK(tsv.net_id == 10, "TSV connects net 10");
    CHECK(tsv.from_die_id == die1_id, "TSV from die 0");
    CHECK(tsv.to_die_id == die2_id, "TSV to die 1");
    // Center of mass of (10,10), (90,10) on d1 and (50,90) on d2
    // (10+90+50)/3 = 50 
    // (10+10+90)/3 = 36.66
    CHECK(tsv.x > 49.0f && tsv.x < 51.0f, "TSV X is around 50");
    CHECK(tsv.y > 36.0f && tsv.y < 37.0f, "TSV Y is around 36.6");
    
    PASS("tsv_insertion");
}

#include "pnr/detailed_router_v2.hpp"

TEST(detailed_router_v2) {
    PhysicalDesign pd;
    
    // Add some cells
    int c1 = pd.add_cell("U1", "INV", 1, 1);
    int c2 = pd.add_cell("U2", "INV", 1, 1);
    int c3 = pd.add_cell("U3", "INV", 1, 1);
    int c4 = pd.add_cell("U4", "INV", 1, 1);
    
    pd.cells[c1].position = {0, 0};
    pd.cells[c2].position = {10, 10};
    pd.cells[c3].position = {20, 0};
    pd.cells[c4].position = {30, 20};
    
    // Add two nets
    int n1 = pd.add_net("net_1", {c1, c2});
    int n2 = pd.add_net("net_2", {c3, c4});
    
    DetailedRouterV2 router(pd);
    router.route(2); // Route with 2 threads
    
    // Both nets should be routed. 
    // Net 1 is c1 to c2 -> needs at least 2 segments (Manhattan L-shape)
    // Net 2 is c3 to c4 -> needs at least 2 segments
    CHECK(pd.wires.size() >= 4, "Router generated wires");
    
    PASS("detailed_router_v2");
}

int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 13 — Advanced Next-Gen Flow  ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    RUN(sva_parsing);
    RUN(sva_engine_sim);
    RUN(hls_parser_scheduler);
    RUN(tsv_insertion);
    RUN(detailed_router_v2);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";

    return failed > 0 ? 1 : 0;
}
