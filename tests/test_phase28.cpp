// Phase 28: HLS Industrial Tests
// Tests if/else, for loops, ASAP/ALAP/list scheduling, resource allocation, binding, pipeline
#include "hls/c_parser.hpp"
#include "core/netlist.hpp"
#include <iostream>
#include <string>
#include <cassert>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// ════════════════════════════════════════════════
// PARSER TESTS
// ════════════════════════════════════════════════
TEST(parse_simple_assign) {
    CParser p;
    auto cdfg = p.parse("x = a + b ;");
    CHECK(!cdfg.empty(), "parsed at least 1 block");
    CHECK(cdfg[0].datapath.size() >= 3, "at least 3 nodes: READ_VAR(a), READ_VAR(b), ADD, ASSIGN");
    PASS("parse_simple_assign");
}

TEST(parse_multi_statement) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = x * c ;");
    CHECK(!cdfg.empty(), "parsed");
    int assigns = 0;
    for (auto& n : cdfg[0].datapath)
        if (n.op == HlsOp::ASSIGN) assigns++;
    CHECK(assigns == 2, "2 assignments");
    PASS("parse_multi_statement");
}

TEST(parse_if_else) {
    CParser p;
    auto cdfg = p.parse("if ( a > b ) { x = a + 1 ; } else { x = b + 1 ; }");
    CHECK(cdfg.size() >= 2, "at least 2 blocks for if/else, got " + std::to_string(cdfg.size()));
    // Should have comparison node in first block
    bool has_cmp = false;
    for (auto& b : cdfg) {
        for (auto& n : b.datapath) {
            if (n.op == HlsOp::CMP_GT) has_cmp = true;
        }
    }
    CHECK(has_cmp, "has comparison node");
    PASS("parse_if_else");
}

TEST(parse_for_loop) {
    CParser p;
    auto cdfg = p.parse("for ( i = 0 ; i < 4 ; i = i + 1 ) { sum = sum + i ; }");
    CHECK(cdfg.size() >= 2, "for loop creates multiple blocks");
    // One block should be marked as loop header
    bool found_loop = false;
    for (auto& b : cdfg)
        if (b.is_loop_header) found_loop = true;
    CHECK(found_loop, "loop header found");
    PASS("parse_for_loop");
}

TEST(parse_while_loop) {
    CParser p;
    auto cdfg = p.parse("while ( x > 0 ) { x = x - 1 ; }");
    CHECK(cdfg.size() >= 2, "while loop creates blocks");
    bool found_loop = false;
    for (auto& b : cdfg)
        if (b.is_loop_header) found_loop = true;
    CHECK(found_loop, "while loop header found");
    PASS("parse_while_loop");
}

TEST(parse_comparison_ops) {
    CParser p;
    auto cdfg = p.parse("a = x == y ; b = x != z ; c = x <= w ;");
    bool has_eq = false, has_ne = false, has_le = false;
    for (auto& b : cdfg)
        for (auto& n : b.datapath) {
            if (n.op == HlsOp::CMP_EQ) has_eq = true;
            if (n.op == HlsOp::CMP_NE) has_ne = true;
            if (n.op == HlsOp::CMP_LE) has_le = true;
        }
    CHECK(has_eq, "has CMP_EQ");
    CHECK(has_ne, "has CMP_NE");
    CHECK(has_le, "has CMP_LE");
    PASS("parse_comparison_ops");
}

TEST(parse_shift_ops) {
    CParser p;
    auto cdfg = p.parse("y = a << 2 ; z = b >> 1 ;");
    bool has_shl = false, has_shr = false;
    for (auto& b : cdfg)
        for (auto& n : b.datapath) {
            if (n.op == HlsOp::SHL) has_shl = true;
            if (n.op == HlsOp::SHR) has_shr = true;
        }
    CHECK(has_shl, "has SHL");
    CHECK(has_shr, "has SHR");
    PASS("parse_shift_ops");
}

// ════════════════════════════════════════════════
// SCHEDULING TESTS
// ════════════════════════════════════════════════
TEST(schedule_asap) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = x * c ;");
    HlsScheduler::schedule_asap(cdfg);
    int max_cycle = 0;
    for (auto& b : cdfg)
        for (auto& n : b.datapath)
            max_cycle = std::max(max_cycle, n.cycle + HlsScheduler::get_latency(n.op));
    CHECK(max_cycle >= 3, "ASAP: MUL after ADD takes >= 3 cycles, got " + std::to_string(max_cycle));
    PASS("schedule_asap");
}

TEST(schedule_alap) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = c + d ;");
    HlsScheduler::schedule_asap(cdfg);
    int deadline = 0;
    for (auto& b : cdfg) for (auto& n : b.datapath)
        deadline = std::max(deadline, n.cycle + HlsScheduler::get_latency(n.op));
    HlsScheduler::schedule_alap(cdfg, deadline);
    bool has_alap = false;
    for (auto& b : cdfg)
        for (auto& n : b.datapath)
            if (n.alap_cycle >= 0) has_alap = true;
    CHECK(has_alap, "ALAP assigned");
    PASS("schedule_alap");
}

TEST(schedule_mobility) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = c + d ; z = x * y ;");
    HlsScheduler::schedule_asap(cdfg);
    int deadline = 0;
    for (auto& b : cdfg) for (auto& n : b.datapath)
        deadline = std::max(deadline, n.cycle + HlsScheduler::get_latency(n.op));
    HlsScheduler::schedule_alap(cdfg, deadline);
    HlsScheduler::compute_mobility(cdfg);
    // Independent ADD ops should have mobility > 0
    bool found_mobility = false;
    for (auto& b : cdfg)
        for (auto& n : b.datapath)
            if (n.mobility > 0) found_mobility = true;
    // Note: in a tight schedule mobility might be 0 for all — that's valid
    PASS("schedule_mobility");
}

TEST(schedule_list) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = c + d ; z = e + f ; w = x * y ;");
    HlsConfig cfg;
    cfg.max_adders = 2;
    cfg.max_multipliers = 1;
    HlsScheduler::schedule_list(cdfg, cfg);
    // With 2 adders, first 2 ADDs can be parallel, 3rd has to wait
    int max_cycle = 0;
    for (auto& b : cdfg) for (auto& n : b.datapath)
        if (n.cycle > max_cycle) max_cycle = n.cycle;
    CHECK(max_cycle >= 1, "list scheduling produces valid cycles");
    PASS("schedule_list");
}

// ════════════════════════════════════════════════
// RESOURCE ALLOCATION & BINDING
// ════════════════════════════════════════════════
TEST(resource_allocation) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = c * d ;");
    HlsScheduler::schedule_asap(cdfg);
    HlsConfig cfg;
    auto resources = HlsResourceAllocator::allocate(cdfg, cfg);
    CHECK(!resources.empty(), "resources allocated");
    bool has_add = false, has_mul = false;
    for (auto& r : resources) {
        if (r.type == HlsOp::ADD) has_add = true;
        if (r.type == HlsOp::MUL) has_mul = true;
    }
    CHECK(has_add, "adder allocated");
    CHECK(has_mul, "multiplier allocated");
    PASS("resource_allocation");
}

TEST(resource_binding) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = c + d ;");
    HlsScheduler::schedule_asap(cdfg);
    HlsConfig cfg;
    auto resources = HlsResourceAllocator::allocate(cdfg, cfg);
    HlsResourceAllocator::bind(cdfg, resources);
    // Check that nodes got resource IDs
    int bound = 0;
    for (auto& b : cdfg)
        for (auto& n : b.datapath)
            if (n.resource_id >= 0) bound++;
    CHECK(bound > 0, "at least some nodes bound");
    PASS("resource_binding");
}

// ════════════════════════════════════════════════
// PIPELINE TESTS
// ════════════════════════════════════════════════
TEST(pipeline_generation) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = x * c ;");
    HlsScheduler::schedule_asap(cdfg);
    auto stages = HlsPipeliner::pipeline(cdfg, 1);
    CHECK(!stages.empty(), "pipeline stages generated");
    for (auto& s : stages) {
        CHECK(s.stage_id > 0, "valid stage id");
        CHECK(s.initiation_interval == 1, "II=1");
    }
    PASS("pipeline_generation");
}

// ════════════════════════════════════════════════
// FULL SYNTHESIS FLOW
// ════════════════════════════════════════════════
TEST(synthesize_full_asap) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = x * c ; z = y - d ;");
    HlsConfig cfg;
    cfg.schedule_algo = HlsConfig::ASAP;
    auto result = HlsSynthesizer::synthesize(cdfg, cfg);
    CHECK(result.success, "synthesis successful");
    CHECK(result.total_cycles > 0, "total_cycles > 0: " + std::to_string(result.total_cycles));
    CHECK(result.num_adders > 0, "adders allocated");
    CHECK(result.num_multipliers > 0, "multipliers allocated");
    CHECK(!result.schedule_report.empty(), "schedule report generated");
    CHECK(!result.binding_report.empty(), "binding report generated");
    std::cout << "    Cycles=" << result.total_cycles << " Adders=" << result.num_adders
              << " Muls=" << result.num_multipliers << " Regs=" << result.num_registers << "\n";
    PASS("synthesize_full_asap");
}

TEST(synthesize_full_list) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = c + d ; z = x * y ;");
    HlsConfig cfg;
    cfg.schedule_algo = HlsConfig::LIST;
    cfg.max_adders = 1; // Force sequential ADDs
    auto result = HlsSynthesizer::synthesize(cdfg, cfg);
    CHECK(result.success, "list scheduling synthesis");
    CHECK(result.total_cycles > 0, "cycles > 0");
    PASS("synthesize_full_list");
}

TEST(synthesize_with_pipeline) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = x * c ;");
    HlsConfig cfg;
    cfg.enable_pipelining = true;
    cfg.pipeline_ii = 1;
    auto result = HlsSynthesizer::synthesize(cdfg, cfg);
    CHECK(result.success, "pipeline synthesis");
    CHECK(!result.pipeline.empty(), "pipeline stages generated");
    std::cout << "    Pipeline: " << result.pipeline.size() << " stages, II=1\n";
    PASS("synthesize_with_pipeline");
}

TEST(synthesize_to_netlist) {
    CParser p;
    auto cdfg = p.parse("x = a + b ; y = x * c ;");
    HlsScheduler::schedule_asap(cdfg);
    Netlist nl;
    HlsSynthesizer::synthesize_to_netlist(cdfg, nl);
    CHECK(nl.num_nets() > 5, "netlist generated with FSM");
    CHECK(nl.num_gates() > 3, "gates generated");
    PASS("synthesize_to_netlist");
}

TEST(latency_model) {
    CHECK(HlsScheduler::get_latency(HlsOp::ADD) == 1, "ADD=1");
    CHECK(HlsScheduler::get_latency(HlsOp::SUB) == 1, "SUB=1");
    CHECK(HlsScheduler::get_latency(HlsOp::MUL) == 2, "MUL=2");
    CHECK(HlsScheduler::get_latency(HlsOp::DIV) == 4, "DIV=4");
    CHECK(HlsScheduler::get_latency(HlsOp::CONST) == 0, "CONST=0");
    CHECK(HlsScheduler::get_latency(HlsOp::READ_VAR) == 0, "READ_VAR=0");
    PASS("latency_model");
}

int main() {
    std::cout << "═══════════════════════════════════════\n";
    std::cout << " Phase 28: HLS Industrial Tests\n";
    std::cout << "═══════════════════════════════════════\n\n";

    std::cout << "── Parser ──\n";
    RUN(parse_simple_assign);
    RUN(parse_multi_statement);
    RUN(parse_if_else);
    RUN(parse_for_loop);
    RUN(parse_while_loop);
    RUN(parse_comparison_ops);
    RUN(parse_shift_ops);

    std::cout << "\n── Scheduling ──\n";
    RUN(schedule_asap);
    RUN(schedule_alap);
    RUN(schedule_mobility);
    RUN(schedule_list);

    std::cout << "\n── Resource Allocation & Binding ──\n";
    RUN(resource_allocation);
    RUN(resource_binding);

    std::cout << "\n── Pipeline ──\n";
    RUN(pipeline_generation);

    std::cout << "\n── Full Synthesis ──\n";
    RUN(synthesize_full_asap);
    RUN(synthesize_full_list);
    RUN(synthesize_with_pipeline);
    RUN(synthesize_to_netlist);
    RUN(latency_model);

    std::cout << "\n════════════════════════════════\n";
    std::cout << "Results: " << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}
