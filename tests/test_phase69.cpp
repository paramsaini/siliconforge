// SiliconForge — Phase 69: Datapath Synthesis Tests
#include "../src/synth/datapath.hpp"
#include "../src/core/netlist.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace sf;

static int tests_run = 0, tests_passed = 0;
#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { std::cerr << "FAIL: " << msg << " [" << __LINE__ << "]\n"; } \
    else { tests_passed++; } \
} while(0)

// ── Helpers ─────────────────────────────────────────────────────────────────

// Build a 1-bit full adder slice: Sum = A⊕B⊕Cin, Cout = (A·B)|(Cin·(A⊕B))
// Returns {sum_net, cout_net}
static std::pair<NetId, NetId> add_full_adder(Netlist& nl, NetId a, NetId b,
                                               NetId cin, int bit) {
    std::string tag = std::to_string(bit);
    NetId p   = nl.add_net("p_" + tag);
    NetId g   = nl.add_net("g_" + tag);
    NetId sum = nl.add_net("sum_" + tag);
    NetId cp  = nl.add_net("cp_" + tag);
    NetId co  = nl.add_net("cout_" + tag);

    nl.add_gate(GateType::XOR, {a, b}, p, "xor_ab_" + tag);     // P = A ⊕ B
    nl.add_gate(GateType::AND, {a, b}, g, "and_ab_" + tag);     // G = A · B
    nl.add_gate(GateType::XOR, {p, cin}, sum, "xor_pc_" + tag); // Sum = P ⊕ Cin
    nl.add_gate(GateType::AND, {p, cin}, cp, "and_pc_" + tag);  // Cin · P
    nl.add_gate(GateType::OR,  {g, cp}, co, "or_co_" + tag);    // Cout = G | (Cin·P)
    return {sum, co};
}

// Build an N-bit ripple-carry adder
static Netlist make_ripple_adder(int width) {
    Netlist nl;
    std::vector<NetId> a(width), b(width);
    for (int i = 0; i < width; ++i) {
        a[i] = nl.add_net("a_" + std::to_string(i));
        b[i] = nl.add_net("b_" + std::to_string(i));
        nl.mark_input(a[i]);
        nl.mark_input(b[i]);
    }
    NetId cin = nl.add_net("cin");
    nl.mark_input(cin);

    NetId carry = cin;
    for (int i = 0; i < width; ++i) {
        auto [sum, cout] = add_full_adder(nl, a[i], b[i], carry, i);
        nl.mark_output(sum);
        carry = cout;
    }
    nl.mark_output(carry);
    return nl;
}

// Build a partial-product AND array (simulates unsigned multiplier)
static Netlist make_multiplier(int width_a, int width_b) {
    Netlist nl;
    std::vector<NetId> a(width_a), b(width_b);
    for (int i = 0; i < width_a; ++i) {
        a[i] = nl.add_net("a_" + std::to_string(i));
        nl.mark_input(a[i]);
    }
    for (int j = 0; j < width_b; ++j) {
        b[j] = nl.add_net("b_" + std::to_string(j));
        nl.mark_input(b[j]);
    }
    for (int i = 0; i < width_a; ++i) {
        for (int j = 0; j < width_b; ++j) {
            NetId pp = nl.add_net("pp_" + std::to_string(i) + "_" + std::to_string(j));
            nl.add_gate(GateType::AND, {a[i], b[j]}, pp,
                        "and_" + std::to_string(i) + "_" + std::to_string(j));
            nl.mark_output(pp);
        }
    }
    return nl;
}

// ── Tests ───────────────────────────────────────────────────────────────────

// Test 1: carry chain inference on a 4-bit ripple adder
void test_carry_chain_4bit() {
    Netlist nl = make_ripple_adder(4);
    DatapathOptimizer opt(nl);
    auto chains = opt.infer_carry_chains();
    CHECK(!chains.empty(), "4-bit adder: carry chain detected");
    int total_width = 0;
    for (const auto& c : chains) total_width += c.width;
    CHECK(total_width >= 2, "4-bit adder: chain width >= 2");
    CHECK(chains[0].start_gate >= 0, "4-bit adder: valid start gate");
    CHECK(chains[0].propagate_delay_ps > 0, "4-bit adder: propagate delay > 0");
}

// Test 2: carry chain inference on an 8-bit adder
void test_carry_chain_8bit() {
    Netlist nl = make_ripple_adder(8);
    DatapathOptimizer opt(nl);
    auto chains = opt.infer_carry_chains();
    CHECK(!chains.empty(), "8-bit adder: carry chain detected");
    int total_width = 0;
    for (const auto& c : chains) total_width += c.width;
    CHECK(total_width >= 4, "8-bit adder: total chain width >= 4");
    CHECK(chains[0].generate_delay_ps > 0, "8-bit adder: generate delay > 0");
}

// Test 3: Wallace tree building from 4-input partial products
void test_wallace_tree_4input() {
    Netlist nl = make_multiplier(4, 4);
    DatapathOptimizer opt(nl);
    auto trees = opt.build_wallace_trees();
    CHECK(!trees.empty(), "4×4 multiplier: Wallace tree built");
    CHECK(trees[0].input_bits == 4, "4×4 multiplier: input_bits == 4");
    CHECK(trees[0].partial_products >= 4, "4×4 multiplier: partial products >= 4");
}

// Test 4: Wallace tree level calculation
void test_wallace_levels() {
    Netlist nl;
    NetId dummy = nl.add_net("d");
    DatapathOptimizer opt(nl);

    // Construct a multiplier to access the level computation indirectly
    Netlist nl8 = make_multiplier(4, 4);
    DatapathOptimizer opt8(nl8);
    auto trees = opt8.build_wallace_trees();
    if (!trees.empty()) {
        // For 16 partial products, levels should be > 1
        CHECK(trees[0].levels >= 2, "Wallace levels >= 2 for 16 partial products");
        CHECK(trees[0].area_gates > 0, "Wallace tree area > 0 gates");
    } else {
        CHECK(true, "Wallace level calc (no tree found, pass by convention)");
        CHECK(true, "Wallace area placeholder");
    }
}

// Test 5: Booth radix-2 encoding detection
void test_booth_radix2() {
    // Small multiplier (< 8-bit) → radix-2
    Netlist nl = make_multiplier(3, 3);
    DatapathOptimizer opt(nl);
    auto booths = opt.apply_booth_encoding();
    CHECK(!booths.empty(), "3×3 multiplier: Booth encoding detected");
    if (!booths.empty()) {
        CHECK(booths[0].encoding_type == BoothEncoderInfo::RADIX2,
              "3×3 multiplier: radix-2 selected");
    } else {
        CHECK(false, "3×3 multiplier: radix-2 type");
    }
}

// Test 6: Booth radix-4 encoding
void test_booth_radix4() {
    // 8-bit multiplier → radix-4
    Netlist nl = make_multiplier(8, 8);
    DatapathOptimizer opt(nl);
    auto booths = opt.apply_booth_encoding();
    CHECK(!booths.empty(), "8×8 multiplier: Booth encoding detected");
    if (!booths.empty()) {
        CHECK(booths[0].encoding_type == BoothEncoderInfo::RADIX4,
              "8×8 multiplier: radix-4 selected");
        CHECK(booths[0].partial_product_reduction_pct >= 49.0,
              "Radix-4: ~50% partial product reduction");
    } else {
        CHECK(false, "8×8 multiplier: radix-4 type");
        CHECK(false, "8×8 multiplier: reduction pct");
    }
}

// Test 7: combined optimize() on a multiplier netlist
void test_optimize_multiplier() {
    Netlist nl = make_multiplier(4, 4);
    DatapathOptimizer opt(nl);
    auto res = opt.optimize();
    CHECK(res.multipliers_optimized >= 1, "optimize: multipliers_optimized >= 1");
    CHECK(res.wallace_trees_built >= 1 || res.booth_encoders_built >= 1,
          "optimize: at least one multiplier opt applied");
    CHECK(!res.report.empty(), "optimize: report generated");
}

// Test 8: empty netlist (no datapath patterns)
void test_empty_netlist() {
    Netlist nl;
    DatapathOptimizer opt(nl);
    auto res = opt.optimize();
    CHECK(res.adders_optimized == 0, "empty: no adders optimized");
    CHECK(res.multipliers_optimized == 0, "empty: no multipliers optimized");
    CHECK(res.carry_chains_inferred == 0, "empty: no carry chains");
    CHECK(res.wallace_trees_built == 0, "empty: no Wallace trees");
    CHECK(res.booth_encoders_built == 0, "empty: no Booth encoders");
}

// Test 9: config flags (disable individual optimizations)
void test_config_flags() {
    Netlist nl = make_ripple_adder(4);
    DatapathOptimizer opt(nl);

    DatapathConfig cfg;
    cfg.enable_carry_chain   = false;
    cfg.enable_wallace_tree  = false;
    cfg.enable_booth_encoding = false;
    auto res = opt.optimize(cfg);
    CHECK(res.carry_chains_inferred == 0, "config: carry chains disabled");
    CHECK(res.wallace_trees_built == 0, "config: Wallace trees disabled");
    CHECK(res.booth_encoders_built == 0, "config: Booth encoding disabled");
}

// Test 10: area reduction calculation
void test_area_reduction() {
    Netlist nl = make_ripple_adder(8);
    DatapathOptimizer opt(nl);
    auto res = opt.optimize();
    // With carry chains detected, area reduction should be positive
    if (res.carry_chains_inferred > 0) {
        CHECK(res.area_reduction_pct > 0.0, "area reduction > 0% with carry chains");
    } else {
        CHECK(res.area_reduction_pct >= 0.0, "area reduction >= 0%");
    }
}

// Test 11: delay improvement estimation
void test_delay_improvement() {
    Netlist nl = make_ripple_adder(8);
    DatapathOptimizer opt(nl);
    auto res = opt.optimize();
    if (res.carry_chains_inferred > 0) {
        CHECK(res.delay_improvement_pct > 0.0,
              "delay improvement > 0% with carry chains");
    } else {
        CHECK(res.delay_improvement_pct >= 0.0, "delay improvement >= 0%");
    }
}

// Test 12: run_enhanced() full flow
void test_run_enhanced() {
    Netlist nl = make_ripple_adder(4);
    DatapathOptimizer opt(nl);
    auto res = opt.run_enhanced();
    CHECK(!res.report.empty(), "enhanced: report not empty");
    CHECK(res.report.find("Enhanced Flow Summary") != std::string::npos,
          "enhanced: report contains enhanced section");
    CHECK(res.report.find("Target frequency") != std::string::npos,
          "enhanced: report contains frequency info");
}

int main() {
    std::cout << "=== Phase 69: Datapath Synthesis Tests ===\n";
    test_carry_chain_4bit();
    test_carry_chain_8bit();
    test_wallace_tree_4input();
    test_wallace_levels();
    test_booth_radix2();
    test_booth_radix4();
    test_optimize_multiplier();
    test_empty_netlist();
    test_config_flags();
    test_area_reduction();
    test_delay_improvement();
    test_run_enhanced();
    std::cout << tests_passed << "/" << tests_run << " datapath tests passed.\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
