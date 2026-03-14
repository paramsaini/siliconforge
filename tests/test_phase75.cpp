// SiliconForge — Phase 75: SVA Engine & ECO Enhancement Tests
#include "../src/formal/sva_engine.hpp"
#include "../src/frontend/sva_parser.hpp"
#include "../src/synth/eco.hpp"
#include "../src/core/netlist.hpp"
#include <cassert>
#include <iostream>
using namespace sf;

static int tests_run = 0, tests_passed = 0;
#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { std::cerr << "FAIL: " << msg << " [" << __LINE__ << "]\n"; } \
    else { tests_passed++; } \
} while(0)

// Helper: build a minimal netlist with clk, a, b as primary inputs
static Netlist make_base_nl() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId a   = nl.add_net("a");   nl.mark_input(a);
    NetId b   = nl.add_net("b");   nl.mark_input(b);
    return nl;
}

// Helper: build simple SvaProperty with an implication root
static SvaProperty make_impl_prop(const std::string& name, SvaOp impl_op,
                                   bool is_assert = true, bool is_cover = false,
                                   bool is_assume = false) {
    SvaProperty prop;
    prop.name = name;
    prop.clock_domain = "clk";
    prop.is_assert = is_assert;
    prop.is_cover  = is_cover;
    prop.is_assume = is_assume;

    auto left  = std::make_shared<SvaNode>();
    left->op   = SvaOp::LITERAL;
    left->literal = "a";

    auto right = std::make_shared<SvaNode>();
    right->op  = SvaOp::LITERAL;
    right->literal = "b";

    prop.expr  = std::make_shared<SvaNode>();
    prop.expr->op = impl_op;
    prop.expr->left  = left;
    prop.expr->right = right;
    return prop;
}

// ── 1. SVA simple overlapping implication (|->) ──────────────────────────────
static void test_sva_overlapping() {
    Netlist nl = make_base_nl();
    auto prop = make_impl_prop("p1", SvaOp::PROP_OVERLAPPING);
    SvaEngine::synthesize_assertions(nl, {prop});
    bool found = false;
    for (auto& n : nl.nets()) if (n.name == "sva_fail_p1") found = true;
    CHECK(found, "overlapping implication creates sva_fail output");
}

// ── 2. SVA non-overlapping implication (|=>) ─────────────────────────────────
static void test_sva_non_overlapping() {
    Netlist nl = make_base_nl();
    auto prop = make_impl_prop("p2", SvaOp::PROP_NON_OVERLAPPING);
    size_t dffs_before = nl.flip_flops().size();
    SvaEngine::synthesize_assertions(nl, {prop});
    bool found = false;
    for (auto& n : nl.nets()) if (n.name == "sva_fail_p2") found = true;
    CHECK(found, "non-overlapping implication creates sva_fail output");
    CHECK(nl.flip_flops().size() > dffs_before, "|=> adds DFFs for delay");
}

// ── 3. SVA with DELAY ##2 ───────────────────────────────────────────────────
static void test_sva_delay() {
    Netlist nl = make_base_nl();
    SvaProperty prop;
    prop.name = "p3";
    prop.clock_domain = "clk";
    prop.is_assert = true;

    auto delay_node = std::make_shared<SvaNode>();
    delay_node->op = SvaOp::DELAY;
    delay_node->delay_cycles = 2;
    auto lit_a = std::make_shared<SvaNode>();
    lit_a->op = SvaOp::LITERAL;
    lit_a->literal = "a";
    delay_node->left = lit_a;

    auto lit_b = std::make_shared<SvaNode>();
    lit_b->op = SvaOp::LITERAL;
    lit_b->literal = "b";

    prop.expr = std::make_shared<SvaNode>();
    prop.expr->op = SvaOp::PROP_OVERLAPPING;
    prop.expr->left  = delay_node;
    prop.expr->right = lit_b;

    size_t dffs_before = nl.flip_flops().size();
    auto res = SvaEngine::synthesize_enhanced(nl, {prop});
    CHECK(res.delay_registers >= 2, "DELAY ##2 adds at least 2 DFFs");
}

// ── 4. SVA with NOT operator ─────────────────────────────────────────────────
static void test_sva_not() {
    Netlist nl = make_base_nl();
    SvaProperty prop;
    prop.name = "p4";
    prop.clock_domain = "clk";
    prop.is_assert = true;

    auto not_node = std::make_shared<SvaNode>();
    not_node->op = SvaOp::NOT;
    auto lit_a = std::make_shared<SvaNode>();
    lit_a->op = SvaOp::LITERAL;
    lit_a->literal = "a";
    not_node->left = lit_a;

    auto lit_b = std::make_shared<SvaNode>();
    lit_b->op = SvaOp::LITERAL;
    lit_b->literal = "b";

    prop.expr = std::make_shared<SvaNode>();
    prop.expr->op = SvaOp::PROP_OVERLAPPING;
    prop.expr->left  = not_node;
    prop.expr->right = lit_b;

    auto res = SvaEngine::synthesize_enhanced(nl, {prop});
    CHECK(res.logic_gates > 0, "NOT operator adds logic gates");
    CHECK(res.properties_synthesized == 1, "NOT property synthesized");
}

// ── 5. SVA with AND/OR logic ─────────────────────────────────────────────────
static void test_sva_and_or() {
    Netlist nl = make_base_nl();
    SvaProperty prop;
    prop.name = "p5";
    prop.clock_domain = "clk";
    prop.is_assert = true;

    auto lit_a = std::make_shared<SvaNode>();
    lit_a->op = SvaOp::LITERAL; lit_a->literal = "a";
    auto lit_b = std::make_shared<SvaNode>();
    lit_b->op = SvaOp::LITERAL; lit_b->literal = "b";

    auto and_node = std::make_shared<SvaNode>();
    and_node->op = SvaOp::AND;
    and_node->left = lit_a;
    and_node->right = lit_b;

    auto or_node = std::make_shared<SvaNode>();
    or_node->op = SvaOp::OR;
    or_node->left = lit_a;
    or_node->right = lit_b;

    prop.expr = std::make_shared<SvaNode>();
    prop.expr->op = SvaOp::PROP_OVERLAPPING;
    prop.expr->left  = and_node;
    prop.expr->right = or_node;

    auto res = SvaEngine::synthesize_enhanced(nl, {prop});
    CHECK(res.logic_gates >= 2, "AND/OR adds multiple logic gates");
}

// ── 6. SVA cover property ────────────────────────────────────────────────────
static void test_sva_cover() {
    Netlist nl = make_base_nl();
    auto prop = make_impl_prop("cov1", SvaOp::PROP_OVERLAPPING,
                               false, true, false);
    auto res = SvaEngine::synthesize_enhanced(nl, {prop});
    CHECK(res.cover_monitors == 1, "cover property counted");
    bool found = false;
    for (auto& n : nl.nets()) if (n.name == "sva_cover_cov1") found = true;
    CHECK(found, "cover property creates sva_cover_ output");
}

// ── 7. SVA assume property ───────────────────────────────────────────────────
static void test_sva_assume() {
    Netlist nl = make_base_nl();
    auto prop = make_impl_prop("asm1", SvaOp::PROP_OVERLAPPING,
                               false, false, true);
    auto res = SvaEngine::synthesize_enhanced(nl, {prop});
    CHECK(res.assume_monitors == 1, "assume property counted");
    bool found = false;
    for (auto& n : nl.nets()) if (n.name == "sva_assume_asm1") found = true;
    CHECK(found, "assume property creates sva_assume_ output");
}

// ── 8. SVA synthesize_enhanced() aggregated counts ──────────────────────────
static void test_sva_enhanced_counts() {
    Netlist nl = make_base_nl();
    std::vector<SvaProperty> props;
    props.push_back(make_impl_prop("ea", SvaOp::PROP_OVERLAPPING));
    props.push_back(make_impl_prop("eb", SvaOp::PROP_NON_OVERLAPPING));
    auto cp = make_impl_prop("ec", SvaOp::PROP_OVERLAPPING, false, true, false);
    props.push_back(cp);

    auto res = SvaEngine::synthesize_enhanced(nl, props);
    CHECK(res.properties_synthesized == 3, "3 properties synthesized");
    CHECK(res.assert_monitors == 2, "2 assert monitors");
    CHECK(res.cover_monitors == 1, "1 cover monitor");
    CHECK(res.monitor_outputs.size() == 3, "3 monitor outputs");
    CHECK(!res.report.empty(), "report is non-empty");
}

// ── 9. ECO insert buffer ────────────────────────────────────────────────────
static void test_eco_insert_buffer() {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b");
    nl.add_gate(GateType::BUF, {a}, b, "buf0");
    size_t gates_before = nl.num_gates();

    EcoEngine eco(nl);
    GateId buf = eco.insert_buffer(a, "eco_buf_test");
    CHECK(buf >= 0, "insert_buffer returns valid gate id");
    CHECK(nl.num_gates() > gates_before, "insert_buffer adds gate");
}

// ── 10. ECO fix timing ─────────────────────────────────────────────────────
static void test_eco_fix_timing() {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_output(b);
    nl.add_gate(GateType::BUF, {a}, b, "drv");

    EcoEngine eco(nl);
    auto res = eco.fix_timing({a});
    CHECK(res.success, "fix_timing succeeds");
    CHECK(res.buffers_inserted >= 1, "fix_timing inserts buffers");
}

// ── 11. ECO fix fanout ──────────────────────────────────────────────────────
static void test_eco_fix_fanout() {
    Netlist nl;
    NetId src = nl.add_net("src"); nl.mark_input(src);
    // Build fanout > 2
    for (int i = 0; i < 4; ++i) {
        NetId o = nl.add_net("fo_" + std::to_string(i));
        nl.add_gate(GateType::BUF, {src}, o, "g_" + std::to_string(i));
    }

    EcoEngine eco(nl);
    auto res = eco.fix_fanout(2);
    CHECK(res.success, "fix_fanout succeeds");
    CHECK(res.nets_modified >= 1, "fix_fanout modifies high-fanout nets");
}

// ── 12. ECO full enhanced flow ──────────────────────────────────────────────
static void test_eco_full_flow() {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_output(b);
    nl.add_gate(GateType::BUF, {a}, b, "drv0");

    FullEcoEngine full(nl);
    auto res = full.run_enhanced();
    CHECK(!res.message.empty(), "run_enhanced returns a report");
}

// ── main ────────────────────────────────────────────────────────────────────
int main() {
    test_sva_overlapping();
    test_sva_non_overlapping();
    test_sva_delay();
    test_sva_not();
    test_sva_and_or();
    test_sva_cover();
    test_sva_assume();
    test_sva_enhanced_counts();
    test_eco_insert_buffer();
    test_eco_fix_timing();
    test_eco_fix_fanout();
    test_eco_full_flow();

    std::cout << "\n=== Phase 75 Results: " << tests_passed << "/" << tests_run
              << " passed ===\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
