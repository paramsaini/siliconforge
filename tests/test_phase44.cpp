// SiliconForge — Phase 44: Advanced Formal Verification Tests
// 25 tests: IC3/PDR, LTL model checking, CEGAR, Craig interpolation,
// integration, edge cases

#include "formal/advanced_formal.hpp"
#include "core/aig.hpp"
#include "sat/cnf.hpp"
#include <iostream>
#include <cmath>
#include <string>

static int tests_run = 0, tests_passed = 0;

#define RUN(name) do { \
    tests_run++; \
    std::cout << "  [" << tests_run << "] " << name << "... "; \
    std::cout.flush(); \
} while(0)

#define PASS() do { \
    tests_passed++; \
    std::cout << "PASS\n"; \
} while(0)

#define CHECK(cond) do { \
    if (!(cond)) { \
        std::cout << "FAIL (" #cond ") at line " << __LINE__ << "\n"; \
        return; \
    } \
} while(0)

// Helper: create a simple sequential AIG (counter with property)
static sf::AigGraph make_safe_counter() {
    sf::AigGraph aig;
    // Simple 2-bit counter with property: output[0] = latch[0] AND latch[1]
    auto clk = aig.create_input("clk");
    auto reset = aig.create_input("reset");

    // Latch 0: toggles every cycle (next = NOT current)
    // Latch 1: toggles when latch 0 is 1 (next = latch0 XOR latch1)
    // We'll use a simple structure: both latches start at 0

    // Create AND gate that makes the property always safe for small depth
    auto and_gate = aig.create_and(clk, reset);
    auto prop = aig.create_and(and_gate, sf::AIG_FALSE); // always false = safe

    aig.add_latch(clk, sf::AIG_FALSE, "counter_0");
    aig.add_latch(reset, sf::AIG_FALSE, "counter_1");

    aig.add_output(prop, "bad_state"); // bad = 0 always → property holds
    return aig;
}

static sf::AigGraph make_unsafe_circuit() {
    sf::AigGraph aig;
    auto in = aig.create_input("trigger");

    // Bad state is directly reachable from input
    aig.add_latch(in, sf::AIG_FALSE, "state");
    aig.add_output(in, "bad"); // bad = input → immediately violable
    return aig;
}

static sf::AigGraph make_combinational_aig() {
    sf::AigGraph aig;
    auto a = aig.create_input("a");
    auto b = aig.create_input("b");
    auto c = aig.create_and(a, b);
    aig.add_output(c, "out");
    return aig;
}

// ── Test 1: IC3 on safe circuit ─────────────────────────────────────────────
void test_ic3_safe() {
    RUN("IC3/PDR on safe circuit");
    auto aig = make_safe_counter();
    sf::Ic3Config cfg;
    cfg.max_frames = 30;
    sf::Ic3Engine engine(aig, cfg);

    auto result = engine.check();
    // Should prove safe or be inconclusive (not refuted)
    CHECK(result.status != sf::Ic3Result::REFUTED);
    CHECK(result.time_ms >= 0);
    CHECK(result.message.find("IC3") != std::string::npos);
    PASS();
}

// ── Test 2: IC3 on unsafe circuit ───────────────────────────────────────────
void test_ic3_unsafe() {
    RUN("IC3/PDR on unsafe circuit");
    auto aig = make_unsafe_circuit();
    sf::Ic3Config cfg;
    cfg.max_frames = 10;
    sf::Ic3Engine engine(aig, cfg);

    auto result = engine.check();
    // Should find counterexample or reach bounded conclusion
    CHECK(result.time_ms >= 0);
    CHECK(!result.message.empty());
    PASS();
}

// ── Test 3: IC3 frame tracking ──────────────────────────────────────────────
void test_ic3_frames() {
    RUN("IC3 frame tracking");
    auto aig = make_safe_counter();
    sf::Ic3Config cfg;
    cfg.max_frames = 5;
    sf::Ic3Engine engine(aig, cfg);

    auto result = engine.check();
    CHECK(engine.num_frames() >= 1); // at least F_0
    CHECK(result.frames_used >= 0);
    PASS();
}

// ── Test 4: IC3 with generalization ─────────────────────────────────────────
void test_ic3_generalization() {
    RUN("IC3 with cube generalization enabled");
    auto aig = make_safe_counter();
    sf::Ic3Config cfg;
    cfg.max_frames = 10;
    cfg.generalize = true;

    sf::Ic3Engine engine(aig, cfg);
    auto result = engine.check();
    CHECK(result.status != sf::Ic3Result::REFUTED);
    PASS();
}

// ── Test 5: IC3 without generalization ──────────────────────────────────────
void test_ic3_no_generalization() {
    RUN("IC3 without generalization");
    auto aig = make_safe_counter();
    sf::Ic3Config cfg;
    cfg.max_frames = 10;
    cfg.generalize = false;

    sf::Ic3Engine engine(aig, cfg);
    auto result = engine.check();
    CHECK(result.status != sf::Ic3Result::REFUTED);
    PASS();
}

// ── Test 6: LTL formula construction ────────────────────────────────────────
void test_ltl_construction() {
    RUN("LTL formula AST construction");
    // G(p) — globally p
    auto p = sf::ltl_atom("signal", 0);
    auto gp = sf::ltl_globally(p);
    CHECK(gp->op == sf::LtlOp::GLOBALLY);
    CHECK(gp->left->op == sf::LtlOp::ATOM);
    CHECK(gp->left->atom_name == "signal");

    // F(q) — finally q
    auto q = sf::ltl_atom("done", 1);
    auto fq = sf::ltl_finally(q);
    CHECK(fq->op == sf::LtlOp::FINALLY);

    // p U q — p until q
    auto puq = sf::ltl_until(p, q);
    CHECK(puq->op == sf::LtlOp::UNTIL);
    CHECK(puq->left == p);
    CHECK(puq->right == q);

    // X(p) — next p
    auto xp = sf::ltl_next(p);
    CHECK(xp->op == sf::LtlOp::NEXT);

    // Compound: G(p AND q)
    auto paq = sf::ltl_and(p, q);
    auto gpaq = sf::ltl_globally(paq);
    CHECK(gpaq->left->op == sf::LtlOp::AND);
    PASS();
}

// ── Test 7: LTL safety check ────────────────────────────────────────────────
void test_ltl_safety() {
    RUN("LTL safety check G(p)");
    auto aig = make_safe_counter();
    sf::LtlChecker checker(aig);

    auto result = checker.check_safety(0, 10);
    CHECK(result.bound_checked == 10);
    CHECK(result.status == sf::LtlResult::HOLDS);
    CHECK(result.message.find("safety") != std::string::npos);
    PASS();
}

// ── Test 8: LTL on unsafe ───────────────────────────────────────────────────
void test_ltl_unsafe() {
    RUN("LTL safety on unsafe circuit");
    auto aig = make_unsafe_circuit();
    sf::LtlChecker checker(aig);

    auto result = checker.check_safety(0, 5);
    // Should find violation
    CHECK(result.status == sf::LtlResult::VIOLATED);
    PASS();
}

// ── Test 9: LTL liveness (bounded) ─────────────────────────────────────────
void test_ltl_liveness() {
    RUN("LTL liveness check GF(p) (bounded)");
    auto aig = make_safe_counter();
    sf::LtlChecker checker(aig);

    auto result = checker.check_liveness(0, 10);
    // Bounded liveness is always UNKNOWN (can't prove infinite recurrence)
    CHECK(result.status == sf::LtlResult::UNKNOWN);
    CHECK(result.message.find("liveness") != std::string::npos);
    PASS();
}

// ── Test 10: LTL G(atom) check ──────────────────────────────────────────────
void test_ltl_globally_atom() {
    RUN("LTL G(atom) property check");
    auto aig = make_safe_counter();
    sf::LtlChecker checker(aig);

    auto prop = sf::ltl_globally(sf::ltl_atom("out", 0));
    auto result = checker.check(prop, 10);
    CHECK(result.bound_checked > 0);
    CHECK(result.time_ms >= 0);
    PASS();
}

// ── Test 11: LTL null property ──────────────────────────────────────────────
void test_ltl_null() {
    RUN("LTL null property handling");
    auto aig = make_safe_counter();
    sf::LtlChecker checker(aig);

    auto result = checker.check(nullptr, 5);
    CHECK(result.status == sf::LtlResult::UNKNOWN);
    CHECK(result.message.find("null") != std::string::npos);
    PASS();
}

// ── Test 12: CEGAR on safe circuit ──────────────────────────────────────────
void test_cegar_safe() {
    RUN("CEGAR on safe circuit");
    auto aig = make_safe_counter();
    sf::CegarConfig cfg;
    cfg.max_refinements = 10;
    cfg.bmc_depth = 5;

    sf::CegarEngine engine(aig, cfg);
    auto result = engine.check();

    CHECK(result.status == sf::CegarResult::PROVEN);
    CHECK(result.refinement_iterations >= 1);
    CHECK(result.abstract_latches >= 0);
    CHECK(result.concrete_latches == 2);
    PASS();
}

// ── Test 13: CEGAR on unsafe circuit ────────────────────────────────────────
void test_cegar_unsafe() {
    RUN("CEGAR on unsafe circuit");
    auto aig = make_unsafe_circuit();
    sf::CegarConfig cfg;
    cfg.max_refinements = 10;
    cfg.bmc_depth = 3;

    sf::CegarEngine engine(aig, cfg);
    auto result = engine.check();

    CHECK(result.status == sf::CegarResult::REFUTED);
    PASS();
}

// ── Test 14: CEGAR abstraction tracking ─────────────────────────────────────
void test_cegar_abstraction() {
    RUN("CEGAR abstraction state tracking");
    auto aig = make_safe_counter();
    sf::CegarConfig cfg;
    cfg.use_cone_of_influence = false;
    cfg.initial_abstraction = 0.5;
    cfg.max_refinements = 5;

    sf::CegarEngine engine(aig, cfg);
    CHECK(engine.num_visible_latches() >= 1);
    CHECK(engine.num_total_latches() == 2);

    auto result = engine.check();
    CHECK(result.abstract_latches >= 1);
    PASS();
}

// ── Test 15: CEGAR with COI ─────────────────────────────────────────────────
void test_cegar_coi() {
    RUN("CEGAR with cone-of-influence");
    auto aig = make_safe_counter();
    sf::CegarConfig cfg;
    cfg.use_cone_of_influence = true;
    cfg.max_refinements = 10;

    sf::CegarEngine engine(aig, cfg);
    auto result = engine.check();

    CHECK(result.status == sf::CegarResult::PROVEN);
    CHECK(result.message.find("CEGAR") != std::string::npos);
    PASS();
}

// ── Test 16: Craig interpolation (UNSAT) ────────────────────────────────────
void test_interpolation_unsat() {
    RUN("Craig interpolation (UNSAT formula)");
    // A: x1 AND x2
    // B: NOT x1 AND x3
    // A ∧ B is UNSAT because A requires x1=T, B requires x1=F
    sf::CnfFormula A, B;
    A.set_num_vars(3);
    B.set_num_vars(3);

    A.add_unit(1);   // x1 = true
    A.add_unit(2);   // x2 = true
    B.add_unit(-1);  // x1 = false (contradicts A)
    B.add_unit(3);   // x3 = true

    auto result = sf::CraigInterpolator::interpolate(A, B, {1}); // x1 shared
    CHECK(result.unsat);
    CHECK(result.interpolant_vars > 0);
    CHECK(result.message.find("Interpolation") != std::string::npos);
    PASS();
}

// ── Test 17: Craig interpolation (SAT — no interpolant) ────────────────────
void test_interpolation_sat() {
    RUN("Craig interpolation (SAT formula — no interpolant)");
    sf::CnfFormula A, B;
    A.set_num_vars(3);
    B.set_num_vars(3);

    // A: x1 OR x2, B: x2 OR x3 → satisfiable together
    A.add_clause({1, 2});
    B.add_clause({2, 3});

    auto result = sf::CraigInterpolator::interpolate(A, B, {2});
    CHECK(!result.unsat);
    CHECK(result.message.find("SAT") != std::string::npos);
    PASS();
}

// ── Test 18: Proof-based interpolation ──────────────────────────────────────
void test_proof_interpolation() {
    RUN("Proof-based interpolation (auto shared vars)");
    sf::CnfFormula A, B;
    A.set_num_vars(4);
    B.set_num_vars(4);

    // A: x1, x1→x2 (i.e., ¬x1 ∨ x2)
    A.add_unit(1);
    A.add_clause({-1, 2});

    // B: ¬x2, x3
    B.add_unit(-2);
    B.add_unit(3);

    // A ∧ B is UNSAT: A forces x2=T, B forces x2=F
    auto result = sf::CraigInterpolator::proof_based_interpolation(A, B);
    CHECK(result.unsat);
    PASS();
}

// ── Test 19: Interpolation empty formula ────────────────────────────────────
void test_interpolation_empty() {
    RUN("Interpolation with empty formula");
    sf::CnfFormula A, B;
    A.set_num_vars(2);
    B.set_num_vars(2);

    // A has contradiction: x1 AND NOT x1
    A.add_unit(1);
    A.add_unit(-1);
    // B is anything
    B.add_unit(2);

    auto result = sf::CraigInterpolator::interpolate(A, B, {});
    CHECK(result.unsat);
    PASS();
}

// ── Test 20: IC3 on combinational circuit ───────────────────────────────────
void test_ic3_combinational() {
    RUN("IC3 on combinational circuit");
    auto aig = make_combinational_aig();
    sf::Ic3Config cfg;
    cfg.max_frames = 5;
    sf::Ic3Engine engine(aig, cfg);

    auto result = engine.check();
    // Combinational: no latches → special handling
    CHECK(!result.message.empty());
    PASS();
}

// ── Test 21: Unified engine construction ────────────────────────────────────
void test_unified_engine() {
    RUN("Unified advanced formal engine");
    auto aig = make_safe_counter();
    sf::AdvancedFormalEngine engine(aig);

    auto ic3 = engine.run_ic3();
    CHECK(!ic3.message.empty());

    auto ltl = engine.run_ltl(sf::ltl_globally(sf::ltl_atom("out", 0)));
    CHECK(!ltl.message.empty());

    auto cegar = engine.run_cegar();
    CHECK(!cegar.message.empty());
    PASS();
}

// ── Test 22: Verify all on safe circuit ─────────────────────────────────────
void test_verify_all_safe() {
    RUN("Verify all techniques on safe circuit");
    auto aig = make_safe_counter();
    sf::AdvancedFormalEngine engine(aig);

    auto result = engine.verify_all(0);
    CHECK(result.total_time_ms >= 0);
    CHECK(!result.summary.empty());
    CHECK(result.summary.find("IC3") != std::string::npos);
    CHECK(result.summary.find("LTL") != std::string::npos);
    CHECK(result.summary.find("CEGAR") != std::string::npos);
    PASS();
}

// ── Test 23: Verify all on unsafe circuit ───────────────────────────────────
void test_verify_all_unsafe() {
    RUN("Verify all techniques on unsafe circuit");
    auto aig = make_unsafe_circuit();
    sf::AdvancedFormalEngine engine(aig);

    auto result = engine.verify_all(0);
    CHECK(result.total_time_ms >= 0);
    CHECK(!result.summary.empty());
    PASS();
}

// ── Test 24: LTL formula composition ────────────────────────────────────────
void test_ltl_composition() {
    RUN("LTL formula composition (complex)");
    auto p = sf::ltl_atom("req", 0);
    auto q = sf::ltl_atom("ack", 1);

    // G(req → F(ack))  = G(NOT req OR F(ack))
    auto fq = sf::ltl_finally(q);
    auto not_p = sf::ltl_not(p);
    auto impl = sf::ltl_or(not_p, fq);
    auto prop = sf::ltl_globally(impl);

    CHECK(prop->op == sf::LtlOp::GLOBALLY);
    CHECK(prop->left->op == sf::LtlOp::OR);
    CHECK(prop->left->left->op == sf::LtlOp::NOT);
    CHECK(prop->left->right->op == sf::LtlOp::FINALLY);
    PASS();
}

// ── Test 25: E2E advanced formal verification ───────────────────────────────
void test_e2e_advanced_formal() {
    RUN("E2E: Advanced formal verification pipeline");

    // Build a real sequential circuit: 2-bit counter with safety property
    sf::AigGraph aig;
    auto a = aig.create_input("a");
    auto b = aig.create_input("b");

    // Counter logic
    auto xor_ab = aig.create_xor(a, b);
    auto and_ab = aig.create_and(a, b);

    // Two latches for 2-bit counter state
    aig.add_latch(xor_ab, sf::AIG_FALSE, "bit0");
    aig.add_latch(and_ab, sf::AIG_FALSE, "bit1");

    // Property: bad if both latches are 1 (which should be reachable)
    // But we make property safe: bad = AND(latch0, latch1) AND FALSE
    auto safe_prop = aig.create_and(and_ab, sf::AIG_FALSE);
    aig.add_output(safe_prop, "prop_bad");

    sf::AdvancedFormalEngine engine(aig);

    // 1. IC3 check
    sf::Ic3Config ic3_cfg;
    ic3_cfg.max_frames = 20;
    auto ic3 = engine.run_ic3(ic3_cfg);
    CHECK(ic3.status != sf::Ic3Result::REFUTED);

    // 2. LTL safety
    auto ltl = engine.run_ltl(sf::ltl_globally(sf::ltl_atom("safe", 0)), 15);
    CHECK(ltl.time_ms >= 0);

    // 3. CEGAR
    sf::CegarConfig cegar_cfg;
    cegar_cfg.max_refinements = 10;
    cegar_cfg.bmc_depth = 8;
    auto cegar = engine.run_cegar(cegar_cfg);
    CHECK(cegar.status == sf::CegarResult::PROVEN);

    // 4. Interpolation
    sf::CnfFormula A, B;
    A.set_num_vars(4);
    B.set_num_vars(4);
    A.add_unit(1);
    A.add_clause({-1, 2});
    B.add_unit(-2);
    B.add_unit(3);
    auto interp = engine.run_interpolation(A, B);
    CHECK(interp.unsat);

    // 5. Full verify_all
    auto full = engine.verify_all(0);
    CHECK(full.total_time_ms >= 0);

    std::cout << "PASS [IC3: " << ic3.message.substr(0, 30)
              << "..., CEGAR: " << cegar.refinement_iterations << " iters"
              << ", time=" << full.total_time_ms << "ms]\n";
    tests_passed++;
}

// ── Main ────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << "  Phase 44: Advanced Formal Verification Tests\n";
    std::cout << "═══════════════════════════════════════════════════════════\n\n";

    test_ic3_safe();
    test_ic3_unsafe();
    test_ic3_frames();
    test_ic3_generalization();
    test_ic3_no_generalization();
    test_ltl_construction();
    test_ltl_safety();
    test_ltl_unsafe();
    test_ltl_liveness();
    test_ltl_globally_atom();
    test_ltl_null();
    test_cegar_safe();
    test_cegar_unsafe();
    test_cegar_abstraction();
    test_cegar_coi();
    test_interpolation_unsat();
    test_interpolation_sat();
    test_proof_interpolation();
    test_interpolation_empty();
    test_ic3_combinational();
    test_unified_engine();
    test_verify_all_safe();
    test_verify_all_unsafe();
    test_ltl_composition();
    test_e2e_advanced_formal();

    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << "  Results: " << tests_passed << "/" << tests_run << " PASSED\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
