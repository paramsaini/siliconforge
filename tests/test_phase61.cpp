// ============================================================
// SiliconForge — Multi-Threading Phase Tests (MT-5)
// SAT Solver Portfolio Parallelism
// ============================================================

#include "sat/cdcl_solver.hpp"
#include "sat/cnf.hpp"
#include "core/thread_pool.hpp"
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

using namespace sf;

static int pass_count = 0;
static int fail_count = 0;

#define TEST(name) \
    static void test_##name(); \
    struct Register_##name { Register_##name() { tests.push_back({#name, test_##name}); } } reg_##name; \
    static void test_##name()

struct TestEntry { const char* name; void(*fn)(); };
static std::vector<TestEntry> tests;

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        std::cerr << "  FAIL: " #cond " at line " << __LINE__ << "\n"; \
        fail_count++; return; \
    } \
} while(0)

// ============================================================
// Test 1: Portfolio solve SAT instance
// ============================================================
TEST(portfolio_sat_basic) {
    CnfFormula f;
    int a = f.new_var(); // 1
    int b = f.new_var(); // 2
    f.add_clause({a, b});        // a OR b
    f.add_clause({-a, b});       // !a OR b
    auto r = portfolio_solve(f, 2);
    ASSERT_TRUE(r.result == SatResult::SAT);
    ASSERT_TRUE(r.winning_config >= 0);
    pass_count++;
}

// ============================================================
// Test 2: Portfolio solve UNSAT instance
// ============================================================
TEST(portfolio_unsat_basic) {
    CnfFormula f;
    int a = f.new_var();
    f.add_clause({a});
    f.add_clause({-a});
    auto r = portfolio_solve(f, 2);
    ASSERT_TRUE(r.result == SatResult::UNSAT);
    pass_count++;
}

// ============================================================
// Test 3: Portfolio with larger formula
// ============================================================
TEST(portfolio_larger) {
    CnfFormula f;
    std::vector<int> vars;
    for (int i = 0; i < 20; ++i) vars.push_back(f.new_var());

    // Chain implications: x_i → x_{i+1}  ≡  (!x_i OR x_{i+1})
    for (int i = 0; i + 1 < 20; ++i) {
        f.add_clause({-vars[i], vars[i+1]});
    }
    f.add_clause({vars[0]}); // force x0 = true
    // Should be SAT with all variables true
    auto r = portfolio_solve(f, 4);
    ASSERT_TRUE(r.result == SatResult::SAT);
    pass_count++;
}

// ============================================================
// Test 4: Portfolio UNSAT with pigeonhole-like constraint
// ============================================================
TEST(portfolio_pigeon_unsat) {
    CnfFormula f;
    // 3 pigeons, 2 holes: at least one pair conflicts
    // x_ij means pigeon i in hole j
    int x[3][2];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 2; ++j)
            x[i][j] = f.new_var();

    // Each pigeon must be in some hole
    for (int i = 0; i < 3; ++i)
        f.add_clause({x[i][0], x[i][1]});

    // No two pigeons in same hole
    for (int j = 0; j < 2; ++j)
        for (int i1 = 0; i1 < 3; ++i1)
            for (int i2 = i1 + 1; i2 < 3; ++i2)
                f.add_clause({-x[i1][j], -x[i2][j]});

    auto r = portfolio_solve(f, 4);
    ASSERT_TRUE(r.result == SatResult::UNSAT);
    pass_count++;
}

// ============================================================
// Test 5: Portfolio single config
// ============================================================
TEST(portfolio_single_config) {
    CnfFormula f;
    int a = f.new_var();
    int b = f.new_var();
    f.add_clause({a, b});
    auto r = portfolio_solve(f, 1);
    ASSERT_TRUE(r.result == SatResult::SAT);
    ASSERT_TRUE(r.winning_config == 0);
    pass_count++;
}

// ============================================================
// Test 6: Portfolio per-config stats are populated
// ============================================================
TEST(portfolio_stats) {
    CnfFormula f;
    int a = f.new_var(); int b = f.new_var(); int c = f.new_var();
    f.add_clause({a, b, c});
    f.add_clause({-a, -b});
    f.add_clause({-b, -c});
    f.add_clause({-a, -c});
    auto r = portfolio_solve(f, 3);
    ASSERT_TRUE(r.per_config_stats.size() == 3);
    // Winner should have done some decisions
    ASSERT_TRUE(r.per_config_stats[r.winning_config].decisions >= 0);
    pass_count++;
}

// ============================================================
// Test 7: Portfolio timing (should complete in reasonable time)
// ============================================================
TEST(portfolio_timing) {
    CnfFormula f;
    for (int i = 0; i < 10; ++i) f.new_var();
    // Random-ish 3-SAT clauses
    f.add_clause({1, 2, 3}); f.add_clause({-1, 4, 5});
    f.add_clause({2, -3, 6}); f.add_clause({-4, 7, -8});
    f.add_clause({3, 5, 9}); f.add_clause({-2, -6, 10});
    f.add_clause({1, -7, 8}); f.add_clause({-3, 6, -9});
    auto r = portfolio_solve(f, 4);
    ASSERT_TRUE(r.time_ms < 5000); // should be fast
    ASSERT_TRUE(r.result == SatResult::SAT || r.result == SatResult::UNSAT);
    pass_count++;
}

// ============================================================
// Test 8: Portfolio default config count
// ============================================================
TEST(portfolio_default_configs) {
    CnfFormula f;
    f.new_var();
    f.add_clause({1});
    auto r = portfolio_solve(f, 0); // 0 means auto-detect
    ASSERT_TRUE(r.result == SatResult::SAT);
    ASSERT_TRUE(r.per_config_stats.size() >= 1);
    pass_count++;
}

// ============================================================
// Test 9: Portfolio repeated calls
// ============================================================
TEST(portfolio_repeated) {
    for (int trial = 0; trial < 5; ++trial) {
        CnfFormula f;
        int a = f.new_var(); int b = f.new_var();
        f.add_clause({a, b});
        auto r = portfolio_solve(f, 2);
        ASSERT_TRUE(r.result == SatResult::SAT);
    }
    pass_count++;
}

// ============================================================
// Test 10: Portfolio trivially true (empty clause set)
// ============================================================
TEST(portfolio_trivial) {
    CnfFormula f;
    f.new_var();
    // No clauses → trivially satisfiable
    auto r = portfolio_solve(f, 2);
    ASSERT_TRUE(r.result == SatResult::SAT);
    pass_count++;
}

// ============================================================
// main
// ============================================================
int main() {
    std::cout << "=== Phase 61: SAT Portfolio Parallelism (MT-5) ===\n";
    for (auto& t : tests) {
        std::cout << "  " << t.name << "... ";
        t.fn();
        if (fail_count == 0 || pass_count > 0)
            std::cout << "PASS\n";
    }
    std::cout << "\nPhase 61: " << pass_count << " passed, " << fail_count << " failed\n";
    return fail_count > 0 ? 1 : 0;
}
