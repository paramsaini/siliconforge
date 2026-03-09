// SiliconForge — Phase 1 Test Suite
// Tests: 4-state logic, AIG, SAT solver, Tseitin, equivalence checker, BMC

#include "core/types.hpp"
#include "core/aig.hpp"
#include "sat/cnf.hpp"
#include "sat/cdcl_solver.hpp"
#include "formal/tseitin.hpp"
#include "formal/equiv_checker.hpp"
#include "formal/bmc.hpp"
#include <iostream>
#include <vector>
#include <cassert>
#include <string>

using namespace sf;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) static void test_##name()
#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        std::cerr << "  [FAIL] " << msg << " (line " << __LINE__ << ")\n"; \
        tests_failed++; return; \
    } \
} while(0)
#define PASS(name) do { std::cout << "  [PASS] " << name << "\n"; tests_passed++; } while(0)
#define RUN(name) do { std::cout << "Running: " #name "\n"; test_##name(); } while(0)

// ============================================================================
// Test: 4-State Logic
// ============================================================================
TEST(logic4_truth_tables) {
    // NOT
    CHECK(logic_not(Logic4::ZERO) == Logic4::ONE, "NOT 0 = 1");
    CHECK(logic_not(Logic4::ONE) == Logic4::ZERO, "NOT 1 = 0");
    CHECK(logic_not(Logic4::X) == Logic4::X, "NOT X = X");
    CHECK(logic_not(Logic4::Z) == Logic4::X, "NOT Z = X");

    // AND: 0 dominates
    CHECK(logic_and(Logic4::ZERO, Logic4::ONE) == Logic4::ZERO, "0 AND 1 = 0");
    CHECK(logic_and(Logic4::ONE, Logic4::ONE) == Logic4::ONE, "1 AND 1 = 1");
    CHECK(logic_and(Logic4::ONE, Logic4::X) == Logic4::X, "1 AND X = X");
    CHECK(logic_and(Logic4::ZERO, Logic4::X) == Logic4::ZERO, "0 AND X = 0");

    // OR: 1 dominates
    CHECK(logic_or(Logic4::ONE, Logic4::ZERO) == Logic4::ONE, "1 OR 0 = 1");
    CHECK(logic_or(Logic4::ZERO, Logic4::ZERO) == Logic4::ZERO, "0 OR 0 = 0");
    CHECK(logic_or(Logic4::ZERO, Logic4::X) == Logic4::X, "0 OR X = X");
    CHECK(logic_or(Logic4::ONE, Logic4::X) == Logic4::ONE, "1 OR X = 1");

    // XOR
    CHECK(logic_xor(Logic4::ZERO, Logic4::ONE) == Logic4::ONE, "0 XOR 1 = 1");
    CHECK(logic_xor(Logic4::ONE, Logic4::ONE) == Logic4::ZERO, "1 XOR 1 = 0");
    CHECK(logic_xor(Logic4::ONE, Logic4::X) == Logic4::X, "1 XOR X = X");

    PASS("logic4_truth_tables");
}

TEST(bitvector_basic) {
    auto bv = BitVector::from_uint(8, 0xA5);
    CHECK(bv.width() == 8, "width = 8");
    CHECK(bv.is_known(), "fully known");
    CHECK(bv.to_uint() == 0xA5, "value = 0xA5");
    CHECK(bv.to_string() == "10100101", "string representation");

    BitVector bv2(4, Logic4::X);
    CHECK(!bv2.is_known(), "X vector not known");

    PASS("bitvector_basic");
}

// ============================================================================
// Test: AIG
// ============================================================================
TEST(aig_construction) {
    AigGraph g;
    AigLit a = g.create_input("a");
    AigLit b = g.create_input("b");

    CHECK(g.num_inputs() == 2, "2 inputs");
    CHECK(a == aig_make(1), "first input var=1");
    CHECK(b == aig_make(2), "second input var=2");

    AigLit ab = g.create_and(a, b);
    CHECK(g.num_ands() == 1, "1 AND gate");

    // Structural hashing: same AND again should reuse
    AigLit ab2 = g.create_and(a, b);
    CHECK(ab == ab2, "structural hash hit");
    CHECK(g.num_ands() == 1, "still 1 AND gate");

    // Commutative hash
    AigLit ba = g.create_and(b, a);
    CHECK(ab == ba, "commutative hash");

    PASS("aig_construction");
}

TEST(aig_simplification) {
    AigGraph g;
    AigLit a = g.create_input("a");

    CHECK(g.create_and(a, AIG_FALSE) == AIG_FALSE, "AND(x, 0) = 0");
    CHECK(g.create_and(a, AIG_TRUE) == a, "AND(x, 1) = x");
    CHECK(g.create_and(a, a) == a, "AND(x, x) = x");
    CHECK(g.create_and(a, aig_not(a)) == AIG_FALSE, "AND(x, ~x) = 0");
    CHECK(g.num_ands() == 0, "no AND gates created for trivial cases");

    PASS("aig_simplification");
}

TEST(aig_evaluation) {
    AigGraph g;
    AigLit a = g.create_input("a");
    AigLit b = g.create_input("b");
    AigLit y = g.create_and(a, b);
    g.add_output(y, "y");

    // Test all input combinations
    struct { bool a, b, expected; } cases[] = {
        {false, false, false},
        {false, true,  false},
        {true,  false, false},
        {true,  true,  true},
    };
    for (auto& tc : cases) {
        auto vals = g.evaluate({tc.a, tc.b});
        bool result = g.eval_lit(g.outputs()[0], vals);
        CHECK(result == tc.expected, "AND evaluation");
    }

    PASS("aig_evaluation");
}

TEST(aig_xor_mux) {
    AigGraph g;
    AigLit a = g.create_input("a");
    AigLit b = g.create_input("b");
    AigLit x = g.create_xor(a, b);
    g.add_output(x, "xor");

    // Exhaustive XOR check
    for (int i = 0; i < 4; ++i) {
        bool va = i & 1, vb = (i >> 1) & 1;
        auto vals = g.evaluate({va, vb});
        CHECK(g.eval_lit(x, vals) == (va ^ vb), "XOR evaluation");
    }

    // MUX check
    AigGraph g2;
    AigLit s = g2.create_input("s");
    AigLit d0 = g2.create_input("d0");
    AigLit d1 = g2.create_input("d1");
    AigLit m = g2.create_mux(s, d1, d0);
    g2.add_output(m, "mux");

    for (int i = 0; i < 8; ++i) {
        bool vs = i & 1, vd0 = (i >> 1) & 1, vd1 = (i >> 2) & 1;
        auto vals = g2.evaluate({vs, vd0, vd1});
        bool expected = vs ? vd1 : vd0;
        CHECK(g2.eval_lit(m, vals) == expected, "MUX evaluation");
    }

    PASS("aig_xor_mux");
}

TEST(aigvec_adder) {
    AigGraph g;
    auto a = AigVec::create_input(g, 4, "a");
    auto b = AigVec::create_input(g, 4, "b");
    auto sum = AigVec::add(g, a, b);
    for (size_t i = 0; i < 4; ++i)
        g.add_output(sum[i], "s" + std::to_string(i));

    // Test: 3 + 5 = 8, but we only have 4 bits so result = 8 mod 16 = 8
    // 3 = 0011, 5 = 0101, sum = 1000 = 8
    std::vector<bool> inputs(8);
    // a = 3 (bits: 1,1,0,0)
    inputs[0] = 1; inputs[1] = 1; inputs[2] = 0; inputs[3] = 0;
    // b = 5 (bits: 1,0,1,0)
    inputs[4] = 1; inputs[5] = 0; inputs[6] = 1; inputs[7] = 0;

    auto vals = g.evaluate(inputs);
    uint32_t result = 0;
    for (size_t i = 0; i < 4; ++i)
        if (g.eval_lit(g.outputs()[i], vals)) result |= (1 << i);
    CHECK(result == 8, "3 + 5 = 8");

    // Test: 7 + 7 = 14
    inputs[0]=1; inputs[1]=1; inputs[2]=1; inputs[3]=0;
    inputs[4]=1; inputs[5]=1; inputs[6]=1; inputs[7]=0;
    vals = g.evaluate(inputs);
    result = 0;
    for (size_t i = 0; i < 4; ++i)
        if (g.eval_lit(g.outputs()[i], vals)) result |= (1 << i);
    CHECK(result == 14, "7 + 7 = 14");

    PASS("aigvec_adder");
}

// ============================================================================
// Test: SAT Solver
// ============================================================================
TEST(sat_trivial) {
    // SAT: (x1 ∨ x2) ∧ (¬x1 ∨ x2)
    CnfFormula f;
    f.new_var(); f.new_var();
    f.add_clause({1, 2});
    f.add_clause({-1, 2});
    CdclSolver s(f);
    CHECK(s.solve() == SatResult::SAT, "trivial SAT");
    // x2 must be true
    CHECK(s.model_value(2) == true, "x2 forced true");
    PASS("sat_trivial");
}

TEST(sat_unsat) {
    // UNSAT: (x1) ∧ (¬x1)
    CnfFormula f;
    f.new_var();
    f.add_clause({1});
    f.add_clause({-1});
    CdclSolver s(f);
    CHECK(s.solve() == SatResult::UNSAT, "trivial UNSAT");
    PASS("sat_unsat");
}

TEST(sat_3vars) {
    // (x1 ∨ x2 ∨ x3) ∧ (¬x1 ∨ ¬x2) ∧ (¬x2 ∨ ¬x3) ∧ (¬x1 ∨ ¬x3)
    CnfFormula f;
    f.new_var(); f.new_var(); f.new_var();
    f.add_clause({1, 2, 3});
    f.add_clause({-1, -2});
    f.add_clause({-2, -3});
    f.add_clause({-1, -3});
    CdclSolver s(f);
    auto r = s.solve();
    CHECK(r == SatResult::SAT, "3-var SAT");
    // Verify model
    bool x1 = s.model_value(1), x2 = s.model_value(2), x3 = s.model_value(3);
    CHECK((x1 || x2 || x3), "clause 1");
    CHECK((!x1 || !x2), "clause 2");
    CHECK((!x2 || !x3), "clause 3");
    CHECK((!x1 || !x3), "clause 4");
    PASS("sat_3vars");
}

TEST(sat_pigeonhole_2_1) {
    // 2 pigeons, 1 hole — UNSAT
    // p11 = pigeon 1 in hole 1, p21 = pigeon 2 in hole 1
    CnfFormula f;
    f.new_var(); f.new_var(); // p11=1, p21=2
    f.add_clause({1});    // pigeon 1 must go somewhere
    f.add_clause({2});    // pigeon 2 must go somewhere
    f.add_clause({-1, -2}); // at most 1 pigeon per hole
    CdclSolver s(f);
    CHECK(s.solve() == SatResult::UNSAT, "pigeonhole 2→1 UNSAT");
    PASS("sat_pigeonhole_2_1");
}

// ============================================================================
// Test: Tseitin Encoding
// ============================================================================
TEST(tseitin_and_gate) {
    AigGraph g;
    AigLit a = g.create_input("a");
    AigLit b = g.create_input("b");
    AigLit y = g.create_and(a, b);
    g.add_output(y, "y");

    TseitinEncoder enc;
    CnfFormula cnf = enc.encode(g);

    // For each input combination, force inputs and check output
    for (int ia = 0; ia <= 1; ++ia) {
        for (int ib = 0; ib <= 1; ++ib) {
            CnfFormula test_cnf = cnf;
            CnfLit la = enc.aig_lit_to_cnf(a);
            CnfLit lb = enc.aig_lit_to_cnf(b);
            test_cnf.add_unit(ia ? la : -la);
            test_cnf.add_unit(ib ? lb : -lb);

            CdclSolver solver(test_cnf);
            CHECK(solver.solve() == SatResult::SAT, "Tseitin AND satisfiable");
            bool output = solver.model_value(enc.aig_to_cnf(aig_var(y)));
            bool expected = ia && ib;
            CHECK(output == expected, "Tseitin AND correct");
        }
    }
    PASS("tseitin_and_gate");
}

// ============================================================================
// Test: Equivalence Checker
// ============================================================================
TEST(equiv_identical) {
    AigGraph g;
    AigLit a = g.create_input("a");
    AigLit b = g.create_input("b");
    AigLit y1 = g.create_and(a, b);
    AigLit y2 = g.create_and(a, b); // same due to structural hashing
    g.add_output(y1, "y1");
    g.add_output(y2, "y2");

    auto r = EquivChecker::check_outputs(g, y1, y2);
    CHECK(r.equivalent, "identical outputs equivalent");
    PASS("equiv_identical");
}

TEST(equiv_demorgan) {
    // Verify De Morgan's: NOT(a AND b) == (NOT a) OR (NOT b)
    AigGraph g1, g2;
    AigLit a1 = g1.create_input("a"); AigLit b1 = g1.create_input("b");
    g1.add_output(g1.create_nand(a1, b1), "y");

    AigLit a2 = g2.create_input("a"); AigLit b2 = g2.create_input("b");
    g2.add_output(g2.create_or(aig_not(a2), aig_not(b2)), "y");

    auto r = EquivChecker::check(g1, g2);
    CHECK(r.equivalent, "De Morgan NAND == OR(NOT,NOT)");
    PASS("equiv_demorgan");
}

TEST(equiv_not_equal) {
    // AND vs OR — should NOT be equivalent
    AigGraph g1, g2;
    AigLit a1 = g1.create_input("a"); AigLit b1 = g1.create_input("b");
    g1.add_output(g1.create_and(a1, b1), "y");

    AigLit a2 = g2.create_input("a"); AigLit b2 = g2.create_input("b");
    g2.add_output(g2.create_or(a2, b2), "y");

    auto r = EquivChecker::check(g1, g2);
    CHECK(!r.equivalent, "AND != OR");
    CHECK(!r.counterexample.empty(), "counterexample provided");
    PASS("equiv_not_equal");
}

TEST(equiv_8bit_mux) {
    // Two implementations of 8-bit MUX
    AigGraph ma, mb;
    AigLit sa = ma.create_input("s");
    auto a0 = AigVec::create_input(ma, 8, "a");
    auto a1 = AigVec::create_input(ma, 8, "b");
    auto oa = AigVec::mux(ma, sa, a1, a0);
    for (size_t i = 0; i < 8; ++i) ma.add_output(oa[i]);

    AigLit sb = mb.create_input("s");
    auto b0 = AigVec::create_input(mb, 8, "a");
    auto b1 = AigVec::create_input(mb, 8, "b");
    for (size_t i = 0; i < 8; ++i) {
        AigLit y = mb.create_or(mb.create_and(sb, b1[i]), mb.create_and(aig_not(sb), b0[i]));
        mb.add_output(y);
    }

    auto r = EquivChecker::check(ma, mb);
    CHECK(r.equivalent, "8-bit MUX implementations equivalent");
    PASS("equiv_8bit_mux");
}

// ============================================================================
// Test: BMC
// ============================================================================
TEST(bmc_counter_overflow) {
    // 2-bit counter, property: never reaches 3
    // Should find violation at cycle 3 (0→1→2→3)
    AigGraph ctr;
    AigLit clk = ctr.create_input("clk");
    AigLit b0 = ctr.create_input("b0");
    AigLit b1 = ctr.create_input("b1");

    AigLit next0 = aig_not(b0);
    AigLit next1 = ctr.create_xor(b1, b0);
    ctr.add_latch(next0, AIG_FALSE, "b0");
    ctr.add_latch(next1, AIG_FALSE, "b1");

    // Bad state: both bits = 1 (counter = 3)
    AigLit bad = ctr.create_and(b0, b1);
    ctr.add_output(bad, "bad");

    auto r = BmcEngine::check(ctr, 10);
    CHECK(!r.safe, "counter overflow detected");
    CHECK(r.depth <= 5, "found within reasonable depth");
    PASS("bmc_counter_overflow");
}

// ============================================================================
// Main
// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 1 — Test Suite               ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    // 4-state logic
    std::cout << "── 4-State Logic ──\n";
    RUN(logic4_truth_tables);
    RUN(bitvector_basic);

    // AIG
    std::cout << "\n── And-Inverter Graph ──\n";
    RUN(aig_construction);
    RUN(aig_simplification);
    RUN(aig_evaluation);
    RUN(aig_xor_mux);
    RUN(aigvec_adder);

    // SAT
    std::cout << "\n── CDCL SAT Solver ──\n";
    RUN(sat_trivial);
    RUN(sat_unsat);
    RUN(sat_3vars);
    RUN(sat_pigeonhole_2_1);

    // Tseitin
    std::cout << "\n── Tseitin Encoding ──\n";
    RUN(tseitin_and_gate);

    // Equivalence
    std::cout << "\n── Equivalence Checker ──\n";
    RUN(equiv_identical);
    RUN(equiv_demorgan);
    RUN(equiv_not_equal);
    RUN(equiv_8bit_mux);

    // BMC
    std::cout << "\n── Bounded Model Checking ──\n";
    RUN(bmc_counter_overflow);

    // Summary
    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << tests_passed << " passed, " << tests_failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";

    return tests_failed > 0 ? 1 : 0;
}
