// SiliconForge — Phase 96 Tests
// Power/ground planning with IR drop feedback loop, EM-aware sizing,
// and adaptive pitch computation.

#include "pnr/power_plan.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <string>
#include <vector>

static int g_pass = 0, g_fail = 0;

static void check(bool cond, const std::string& name) {
    if (cond) { ++g_pass; std::cout << "  PASS  " << name << "\n"; }
    else      { ++g_fail; std::cout << "**FAIL  " << name << "\n"; }
}

// ── Helper: create a 200x200 design with 20 placed cells ──────────────

static sf::PhysicalDesign make_power_design() {
    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 200, 200};
    pd.row_height = 10.0;
    pd.site_width = 1.0;

    for (int i = 0; i < 20; ++i) {
        sf::CellInstance c;
        c.id = i;
        c.name = "cell_" + std::to_string(i);
        c.cell_type = "INV_X1";
        c.width = 5.0;
        c.height = 10.0;
        c.position = {10.0 + (i % 10) * 18.0, 20.0 + (i / 10) * 15.0};
        c.placed = true;
        pd.cells.push_back(c);
    }
    return pd;
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 1: Basic power plan
// ═══════════════════════════════════════════════════════════════════════════

static void test_basic_power_plan() {
    std::cout << "\n=== test_basic_power_plan ===\n";
    auto pd = make_power_design();
    sf::PowerPlanner planner(pd);
    auto res = planner.plan();

    check(res.rings > 0,            "rings created");
    check(res.stripes > 0,          "stripes created");
    check(res.rails > 0,            "rails created");
    check(res.vias > 0,             "vias created");
    check(res.total_wire_length > 0, "positive wire length");
    check(!res.message.empty(),     "message populated");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 2: Ring creation
// ═══════════════════════════════════════════════════════════════════════════

static void test_ring_creation() {
    std::cout << "\n=== test_ring_creation ===\n";
    auto pd = make_power_design();
    sf::PowerPlanner planner(pd);
    sf::PowerPlanConfig cfg;
    auto res = planner.plan(cfg);

    // Count wires on the ring layer
    int ring_wires = 0;
    for (const auto& w : pd.wires) {
        if (w.layer == cfg.ring_layer) ++ring_wires;
    }
    check(ring_wires >= 4,  "at least 4 ring wires (VDD top/bot/left/right)");
    check(ring_wires >= 8,  "VDD + VSS rings = 8 ring wires");
    check(res.rings >= 2,   "at least 2 rings (VDD + VSS)");
    check(res.total_wire_length > 0, "ring contributes to wire length");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 3: Stripe creation
// ═══════════════════════════════════════════════════════════════════════════

static void test_stripe_creation() {
    std::cout << "\n=== test_stripe_creation ===\n";
    auto pd = make_power_design();
    sf::PowerPlanner planner(pd);
    sf::PowerPlanConfig cfg;
    auto res = planner.plan(cfg);

    int stripe_wires = 0;
    for (const auto& w : pd.wires) {
        if (w.layer == cfg.stripe_layer) ++stripe_wires;
    }
    check(stripe_wires > 0, "stripe wires exist on stripe layer");
    check(res.stripes > 0,  "stripes count > 0");
    check(res.stripes == stripe_wires, "stripe count matches wire count on stripe layer");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 4: Rail creation
// ═══════════════════════════════════════════════════════════════════════════

static void test_rail_creation() {
    std::cout << "\n=== test_rail_creation ===\n";
    auto pd = make_power_design();
    sf::PowerPlanner planner(pd);
    sf::PowerPlanConfig cfg;
    auto res = planner.plan(cfg);

    int rail_wires = 0;
    for (const auto& w : pd.wires) {
        if (w.layer == cfg.rail_layer) ++rail_wires;
    }
    check(rail_wires > 0,  "rail wires exist on M1");
    check(res.rails > 0,   "rails count > 0");
    check(res.rails == rail_wires, "rail count matches wire count on rail layer");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 5: Via creation
// ═══════════════════════════════════════════════════════════════════════════

static void test_via_creation() {
    std::cout << "\n=== test_via_creation ===\n";
    auto pd = make_power_design();
    sf::PowerPlanner planner(pd);
    auto res = planner.plan();

    check(res.vias > 0,       "vias created");
    check(!pd.vias.empty(),   "vias stored in physical design");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 6: IR feedback basic
// ═══════════════════════════════════════════════════════════════════════════

static void test_ir_feedback_basic() {
    std::cout << "\n=== test_ir_feedback_basic ===\n";
    auto pd = make_power_design();
    sf::PowerPlanner planner(pd);
    sf::PowerPlanConfig cfg;
    cfg.total_current_ma = 100.0;
    auto res = planner.plan_with_ir_feedback(cfg);

    check(res.ir_feedback.iterations >= 1,           "at least 1 iteration");
    check(res.ir_feedback.initial_worst_ir_mv > 0,   "initial IR > 0");
    check(!res.ir_feedback.ir_per_iteration.empty(),  "ir_per_iteration tracked");
    check(!res.ir_feedback.message.empty(),           "feedback message populated");
    check(res.stripes > 0,                            "stripes exist after feedback");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 7: IR convergence
// ═══════════════════════════════════════════════════════════════════════════

static void test_ir_convergence() {
    std::cout << "\n=== test_ir_convergence ===\n";

    // Easy target: should converge immediately
    {
        auto pd = make_power_design();
        sf::PowerPlanner planner(pd);
        sf::PowerPlanConfig cfg;
        cfg.ir_target_mv = 10000.0; // very relaxed
        cfg.total_current_ma = 1.0; // tiny current
        auto res = planner.plan_with_ir_feedback(cfg);
        check(res.ir_feedback.converged, "converged with easy target");
    }

    // Impossible target: should not converge
    {
        auto pd = make_power_design();
        sf::PowerPlanner planner(pd);
        sf::PowerPlanConfig cfg;
        cfg.ir_target_mv = 0.001;         // impossibly tight
        cfg.ir_max_iterations = 3;
        cfg.total_current_ma = 500.0;     // very high current
        auto res = planner.plan_with_ir_feedback(cfg);
        check(!res.ir_feedback.converged, "not converged with impossible target");
    }

    // Check iteration cap
    {
        auto pd = make_power_design();
        sf::PowerPlanner planner(pd);
        sf::PowerPlanConfig cfg;
        cfg.ir_target_mv = 0.001;
        cfg.ir_max_iterations = 2;
        cfg.total_current_ma = 500.0;
        auto res = planner.plan_with_ir_feedback(cfg);
        check(res.ir_feedback.iterations <= 2, "respects max iteration cap");
    }

    // Converged flag correct
    {
        auto pd = make_power_design();
        sf::PowerPlanner planner(pd);
        sf::PowerPlanConfig cfg;
        cfg.ir_target_mv = 10000.0;
        cfg.total_current_ma = 0.1;
        auto res = planner.plan_with_ir_feedback(cfg);
        check(res.ir_feedback.final_worst_ir_mv <= cfg.ir_target_mv,
              "final IR within target when converged");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 8: IR improvement
// ═══════════════════════════════════════════════════════════════════════════

static void test_ir_improvement() {
    std::cout << "\n=== test_ir_improvement ===\n";
    auto pd = make_power_design();
    sf::PowerPlanner planner(pd);
    sf::PowerPlanConfig cfg;
    cfg.ir_target_mv = 1.0;        // tight target forces iterations
    cfg.total_current_ma = 200.0;
    cfg.ir_max_iterations = 5;
    auto res = planner.plan_with_ir_feedback(cfg);

    check(res.ir_feedback.final_worst_ir_mv <= res.ir_feedback.initial_worst_ir_mv,
          "final IR <= initial IR");
    check(res.ir_feedback.iterations >= 1, "ran at least 1 iteration for improvement");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 9: Extra stripes added
// ═══════════════════════════════════════════════════════════════════════════

static void test_extra_stripes_added() {
    std::cout << "\n=== test_extra_stripes_added ===\n";
    auto pd = make_power_design();
    sf::PowerPlanner planner(pd);
    sf::PowerPlanConfig cfg;
    cfg.ir_target_mv = 0.5;        // tight enough to force extra stripes
    cfg.total_current_ma = 300.0;
    cfg.ir_max_iterations = 5;
    auto res = planner.plan_with_ir_feedback(cfg);

    check(res.ir_feedback.extra_stripes_added > 0, "extra stripes were added");
    check(res.ir_feedback.iterations >= 1,         "iterations ran to add stripes");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 10: EM-aware width
// ═══════════════════════════════════════════════════════════════════════════

static void test_em_aware_width() {
    std::cout << "\n=== test_em_aware_width ===\n";
    auto pd = make_power_design();
    sf::PowerPlanner planner(pd);
    sf::PowerPlanConfig cfg;
    cfg.enable_em_sizing = true;
    cfg.total_current_ma = 500.0;   // high current
    cfg.j_max_em_ma_per_um = 0.1;   // very low EM limit -> forces widening
    auto res = planner.plan_with_ir_feedback(cfg);

    check(res.em_widened_stripes > 0, "EM widened some stripes");

    // Verify widths were actually increased
    bool found_wide = false;
    for (const auto& w : pd.wires) {
        if (w.layer == cfg.stripe_layer && w.width > cfg.stripe_width) {
            found_wide = true;
            break;
        }
    }
    check(found_wide, "stripe widths increased beyond default");
    check(res.stripes > 0, "stripes still present after EM sizing");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 11: compute_em_width unit test
// ═══════════════════════════════════════════════════════════════════════════

static void test_compute_em_width() {
    std::cout << "\n=== test_compute_em_width ===\n";

    // 100 mA / 1.0 mA/um = 100 um required, min 1.5 -> 100
    double w1 = sf::PowerPlanner::compute_em_width(100.0, 1.0, 1.5);
    check(std::abs(w1 - 100.0) < 0.01, "em_width: 100mA/1.0 -> 100 um");

    // 0.5 mA / 1.0 mA/um = 0.5 um required, min 1.5 -> 1.5
    double w2 = sf::PowerPlanner::compute_em_width(0.5, 1.0, 1.5);
    check(std::abs(w2 - 1.5) < 0.01, "em_width: 0.5mA/1.0 -> min 1.5 um");

    // Edge case: j_max = 0 -> return min_width
    double w3 = sf::PowerPlanner::compute_em_width(50.0, 0.0, 2.0);
    check(std::abs(w3 - 2.0) < 0.01, "em_width: j_max=0 -> min_width");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 12: Adaptive pitches
// ═══════════════════════════════════════════════════════════════════════════

static void test_adaptive_pitches() {
    std::cout << "\n=== test_adaptive_pitches ===\n";

    std::vector<double> currents = {2.0, 1.0, 0.3};
    auto pitches = sf::PowerPlanner::compute_adaptive_pitches(3, currents, 20.0, 80.0);

    check(pitches.size() == 3, "3 pitches returned for 3 regions");
    check(pitches[0] < pitches[2],
          "high-current region gets smaller pitch than low-current");
    check(pitches[0] >= 20.0 && pitches[0] <= 80.0, "pitch[0] within range");
    check(pitches[2] >= 20.0 && pitches[2] <= 80.0, "pitch[2] within range");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 13: IR per-iteration tracking
// ═══════════════════════════════════════════════════════════════════════════

static void test_ir_per_iteration_tracking() {
    std::cout << "\n=== test_ir_per_iteration_tracking ===\n";
    auto pd = make_power_design();
    sf::PowerPlanner planner(pd);
    sf::PowerPlanConfig cfg;
    cfg.ir_target_mv = 0.01;
    cfg.total_current_ma = 200.0;
    cfg.ir_max_iterations = 4;
    auto res = planner.plan_with_ir_feedback(cfg);

    check(res.ir_feedback.ir_per_iteration.size() >= 2,
          "at least 2 IR data points (initial + 1 iteration)");
    // Each iteration should have reduced or equal IR
    bool monotone = true;
    for (size_t i = 1; i < res.ir_feedback.ir_per_iteration.size(); ++i) {
        if (res.ir_feedback.ir_per_iteration[i] > res.ir_feedback.ir_per_iteration[i-1] + 0.01) {
            monotone = false;
        }
    }
    check(monotone, "IR values non-increasing across iterations");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 14: Empty design (no cells but valid die)
// ═══════════════════════════════════════════════════════════════════════════

static void test_empty_design() {
    std::cout << "\n=== test_empty_design ===\n";
    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 200, 200};
    pd.row_height = 10.0;

    sf::PowerPlanner planner(pd);
    auto res = planner.plan();

    check(res.rings > 0,   "rings created even with no cells");
    check(res.stripes > 0, "stripes created even with no cells");
}

// ═══════════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "SiliconForge Phase 96 — Power/Ground Planning with IR Drop Feedback\n";

    test_basic_power_plan();
    test_ring_creation();
    test_stripe_creation();
    test_rail_creation();
    test_via_creation();
    test_ir_feedback_basic();
    test_ir_convergence();
    test_ir_improvement();
    test_extra_stripes_added();
    test_em_aware_width();
    test_compute_em_width();
    test_adaptive_pitches();
    test_ir_per_iteration_tracking();
    test_empty_design();

    std::cout << "\n════════════════════════════════════════\n"
              << "Phase 96: " << g_pass << " passed, " << g_fail << " failed\n"
              << "════════════════════════════════════════\n";
    return g_fail ? 1 : 0;
}
