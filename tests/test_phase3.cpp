// SiliconForge — Phase 3 Test Suite
// Tests: Liberty parser, AIG optimizer, tech mapper, scan insertion

#include "core/types.hpp"
#include "core/aig.hpp"
#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "synth/aig_opt.hpp"
#include "synth/tech_mapper.hpp"
#include "dft/scan_insert.hpp"
#include "formal/equiv_checker.hpp"
#include <iostream>
#include <cassert>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

static const char* MINI_LIBERTY = R"LIB(
library(testlib) {
  technology(cmos);
  nom_voltage : 1.8;
  cell(INV_X1) {
    area : 1.0;
    pin(A) { direction : input; }
    pin(Y) { direction : output; function : "!A"; }
  }
  cell(AND2_X1) {
    area : 2.0;
    pin(A) { direction : input; }
    pin(B) { direction : input; }
    pin(Y) { direction : output; function : "A&B"; }
  }
  cell(OR2_X1) {
    area : 2.0;
    pin(A) { direction : input; }
    pin(B) { direction : input; }
    pin(Y) { direction : output; function : "A|B"; }
  }
  cell(NAND2_X1) {
    area : 1.5;
    pin(A) { direction : input; }
    pin(B) { direction : input; }
    pin(Y) { direction : output; function : "!(A&B)"; }
  }
  cell(XOR2_X1) {
    area : 3.0;
    pin(A) { direction : input; }
    pin(B) { direction : input; }
    pin(Y) { direction : output; function : "A^B"; }
  }
  cell(DFF_X1) {
    area : 5.0;
    pin(D) { direction : input; }
    pin(CK) { direction : input; }
    pin(Q) { direction : output; function : "D"; }
  }
}
)LIB";

// ============================================================================
// Test: Liberty Parser
// ============================================================================
TEST(liberty_parser) {
    LibertyLibrary lib;
    bool ok = lib.parse_string(MINI_LIBERTY);
    CHECK(ok, "Liberty parse succeeded");
    CHECK(lib.name == "testlib", "library name");
    CHECK(lib.cells.size() == 6, "6 cells parsed");

    auto* inv = lib.find_cell("INV_X1");
    CHECK(inv != nullptr, "INV_X1 found");
    CHECK(inv->area == 1.0, "INV area");
    CHECK(inv->output_function() == "!A", "INV function");
    CHECK(inv->num_inputs() == 1, "INV has 1 input");

    auto* and2 = lib.find_cell("AND2_X1");
    CHECK(and2 != nullptr, "AND2_X1 found");
    CHECK(and2->area == 2.0, "AND2 area");
    CHECK(and2->num_inputs() == 2, "AND2 has 2 inputs");
    PASS("liberty_parser");
}

TEST(liberty_voltage) {
    LibertyLibrary lib;
    lib.parse_string(MINI_LIBERTY);
    CHECK(lib.nom_voltage == 1.8, "nominal voltage");
    PASS("liberty_voltage");
}

// ============================================================================
// Test: AIG Optimizer
// ============================================================================
TEST(aig_opt_depth) {
    // Build a chain of 8 ANDs (linear): depth = 8
    AigGraph g;
    AigLit prev = g.create_input("x0");
    for (int i = 1; i <= 8; ++i) {
        AigLit inp = g.create_input("x" + std::to_string(i));
        prev = g.create_and(prev, inp);
    }
    g.add_output(prev, "y");

    uint32_t d = AigOptimizer::compute_depth(g);
    CHECK(d == 8, "linear chain depth = 8");
    PASS("aig_opt_depth");
}

TEST(aig_opt_balance) {
    // Build linear chain, then balance
    AigGraph g;
    AigLit prev = g.create_input("x0");
    for (int i = 1; i <= 7; ++i) {
        AigLit inp = g.create_input("x" + std::to_string(i));
        prev = g.create_and(prev, inp);
    }
    g.add_output(prev, "y");

    uint32_t d_before = AigOptimizer::compute_depth(g);
    AigOptimizer opt(g);
    opt.balance();
    uint32_t d_after = AigOptimizer::compute_depth(g);

    CHECK(d_after <= d_before, "balancing reduces or maintains depth");
    CHECK(d_after <= 4, "8 inputs → balanced depth ≤ 4 (log2(8))");
    PASS("aig_opt_balance");
}

TEST(aig_opt_sweep) {
    // Build circuit with dead logic
    AigGraph g;
    AigLit a = g.create_input("a");
    AigLit b = g.create_input("b");
    AigLit c = g.create_input("c");
    AigLit ab = g.create_and(a, b);
    AigLit ac = g.create_and(a, c); // dead — not connected to output
    g.add_output(ab, "y");

    uint32_t before = g.num_ands();
    AigOptimizer opt(g);
    opt.sweep();
    uint32_t after = g.num_ands();

    CHECK(after <= before, "sweep reduces gate count");
    CHECK(after == 1, "only 1 AND gate should remain");
    PASS("aig_opt_sweep");
}

TEST(aig_opt_full) {
    // Full optimization pass
    AigGraph g;
    AigLit prev = g.create_input("x0");
    for (int i = 1; i <= 15; ++i) {
        AigLit inp = g.create_input("x" + std::to_string(i));
        prev = g.create_and(prev, inp);
    }
    g.add_output(prev, "y");

    AigOptimizer opt(g);
    auto stats = opt.optimize(2);
    CHECK(stats.final_depth <= stats.initial_depth, "depth reduced or same");
    CHECK(stats.passes == 2, "2 passes done");
    PASS("aig_opt_full");
}

// ============================================================================
// Test: Technology Mapper
// ============================================================================
TEST(tech_map_simple) {
    AigGraph g;
    AigLit a = g.create_input("a");
    AigLit b = g.create_input("b");
    AigLit y = g.create_and(a, b);
    g.add_output(y, "y");

    LibertyLibrary lib;
    lib.parse_string(MINI_LIBERTY);

    TechMapper mapper(g, lib);
    Netlist nl = mapper.map();

    CHECK(nl.primary_inputs().size() == 2, "2 PIs in mapped netlist");
    CHECK(nl.primary_outputs().size() == 1, "1 PO in mapped netlist");
    CHECK(mapper.stats().num_cells > 0, "cells used");
    PASS("tech_map_simple");
}

TEST(tech_map_multi) {
    AigGraph g;
    AigLit a = g.create_input("a");
    AigLit b = g.create_input("b");
    AigLit c = g.create_input("c");
    AigLit ab = g.create_and(a, b);
    AigLit abc = g.create_and(ab, c);
    g.add_output(abc, "y");

    LibertyLibrary lib;
    lib.parse_string(MINI_LIBERTY);

    TechMapper mapper(g, lib);
    Netlist nl = mapper.map();
    CHECK(mapper.stats().num_cells >= 2, "at least 2 cells for 3-input AND");
    PASS("tech_map_multi");
}

// ============================================================================
// Test: Scan Chain Insertion
// ============================================================================
TEST(scan_insert_basic) {
    // Build a simple circuit with 2 FFs
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId d0 = nl.add_net("d0"); nl.mark_input(d0);
    NetId d1 = nl.add_net("d1"); nl.mark_input(d1);
    NetId q0 = nl.add_net("q0"); nl.mark_output(q0);
    NetId q1 = nl.add_net("q1"); nl.mark_output(q1);
    nl.add_dff(d0, clk, q0, -1, "FF0");
    nl.add_dff(d1, clk, q1, -1, "FF1");

    ScanInserter inserter(nl);
    auto result = inserter.insert({.max_chain_length = 10});

    CHECK(result.num_chains == 1, "1 scan chain");
    CHECK(result.total_ffs == 2, "2 FFs scanned");
    CHECK(result.chain_lengths[0] == 2, "chain length = 2");
    PASS("scan_insert_basic");
}

TEST(scan_insert_multi_chain) {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    for (int i = 0; i < 5; ++i) {
        NetId d = nl.add_net("d" + std::to_string(i)); nl.mark_input(d);
        NetId q = nl.add_net("q" + std::to_string(i)); nl.mark_output(q);
        nl.add_dff(d, clk, q, -1, "FF" + std::to_string(i));
    }

    ScanInserter inserter(nl);
    auto result = inserter.insert({.max_chain_length = 2});

    CHECK(result.num_chains == 3, "3 chains for 5 FFs with max_len=2");
    CHECK(result.total_ffs == 5, "5 FFs total");
    PASS("scan_insert_multi_chain");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 3 — Test Suite               ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Liberty Parser ──\n";
    RUN(liberty_parser);
    RUN(liberty_voltage);

    std::cout << "\n── AIG Optimizer ──\n";
    RUN(aig_opt_depth);
    RUN(aig_opt_balance);
    RUN(aig_opt_sweep);
    RUN(aig_opt_full);

    std::cout << "\n── Technology Mapper ──\n";
    RUN(tech_map_simple);
    RUN(tech_map_multi);

    std::cout << "\n── Scan Chain Insertion ──\n";
    RUN(scan_insert_basic);
    RUN(scan_insert_multi_chain);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
