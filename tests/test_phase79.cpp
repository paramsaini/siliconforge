// SiliconForge — Phase 79: Hierarchical LVS & Rule Deck Tests
#include "../src/verify/lvs.hpp"
#include "../src/verify/drc.hpp"
#include "../src/core/netlist.hpp"
#include "../src/pnr/physical.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace sf;

static int tests_run = 0, tests_passed = 0;
#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { \
        std::cerr << "  [FAIL] " << msg << " (line " << __LINE__ << ")\n"; \
        return; \
    } \
    tests_passed++; \
} while(0)

#define TEST(name) static void test_##name()
#define RUN(name) do { \
    std::cout << "Running: " #name "\n"; \
    test_##name(); \
    std::cout << "  [PASS] " #name "\n"; \
} while(0)

// ── Helper: build a simple design with known cell types ──────────────

static void build_simple_design(Netlist& nl, PhysicalDesign& pd) {
    // Schematic: 2 AND gates, 1 OR gate
    auto n0 = nl.add_net("a");
    auto n1 = nl.add_net("b");
    auto n2 = nl.add_net("c");
    auto n3 = nl.add_net("w0");
    auto n4 = nl.add_net("w1");
    auto n5 = nl.add_net("out");

    nl.add_gate(GateType::INPUT, {}, n0, "in_a");
    nl.add_gate(GateType::INPUT, {}, n1, "in_b");
    nl.add_gate(GateType::INPUT, {}, n2, "in_c");
    nl.add_gate(GateType::AND, {n0, n1}, n3, "and0");
    nl.add_gate(GateType::AND, {n1, n2}, n4, "and1");
    nl.add_gate(GateType::OR, {n3, n4}, n5, "or0");
    nl.add_gate(GateType::OUTPUT, {n5}, -1, "out0");

    // Layout: matching cell instances
    pd.die_area = Rect(0, 0, 100, 100);
    pd.add_cell("and0", "AND", 4, 2);
    pd.add_cell("and1", "AND", 4, 2);
    pd.add_cell("or0",  "OR",  4, 2);

    // Nets connecting cells
    pd.add_net("w0", {0, 2});
    pd.add_net("w1", {1, 2});
}

// ── Test 1: Hierarchy extraction from simple design ──────────────────

TEST(hierarchy_extraction) {
    Netlist nl;
    PhysicalDesign pd;
    build_simple_design(nl, pd);

    LvsChecker checker(nl, pd);
    auto blocks = checker.extract_hierarchy();

    CHECK(!blocks.empty(), "Should extract at least one block");
    // We have 2 cell types: AND (2 instances) and OR (1 instance)
    CHECK(blocks.size() == 2, "Should have 2 unique block types");

    // Find the AND block
    auto it = std::find_if(blocks.begin(), blocks.end(),
        [](const LvsChecker::HierBlock& b) { return b.name == "AND"; });
    CHECK(it != blocks.end(), "Should find AND block");
    CHECK(it->instance_count == 2, "AND block should have 2 instances");
    CHECK(it->devices.size() == 2, "AND block should have 2 devices");
}

// ── Test 2: Hierarchical LVS — matching blocks ──────────────────────

TEST(hierarchical_lvs_match) {
    Netlist nl;
    PhysicalDesign pd;
    build_simple_design(nl, pd);

    LvsChecker checker(nl, pd);
    auto result = checker.check_hierarchical();

    CHECK(result.blocks_compared > 0, "Should compare blocks");
    CHECK(result.blocks_matched > 0, "Should have matched blocks");
    CHECK(result.time_ms >= 0, "Time should be non-negative");
    CHECK(!result.report.empty(), "Report should not be empty");
}

// ── Test 3: Hierarchical LVS — mismatched block ─────────────────────

TEST(hierarchical_lvs_mismatch) {
    Netlist nl;
    PhysicalDesign pd;

    // Schematic: 2 AND gates
    auto n0 = nl.add_net("a");
    auto n1 = nl.add_net("b");
    auto n2 = nl.add_net("w0");
    nl.add_gate(GateType::INPUT, {}, n0, "in_a");
    nl.add_gate(GateType::INPUT, {}, n1, "in_b");
    nl.add_gate(GateType::AND, {n0, n1}, n2, "and0");
    nl.add_gate(GateType::AND, {n0, n1}, n2, "and1");

    // Layout: 3 AND instances (mismatch!)
    pd.die_area = Rect(0, 0, 100, 100);
    pd.add_cell("and0", "AND", 4, 2);
    pd.add_cell("and1", "AND", 4, 2);
    pd.add_cell("and2", "AND", 4, 2);

    LvsChecker checker(nl, pd);
    auto result = checker.check_hierarchical();

    CHECK(result.blocks_compared > 0, "Should compare blocks");
    // Mismatch: schematic has 2 AND, layout has 3
    bool has_mismatch = false;
    for (auto& br : result.block_results) {
        if (!br.matched) has_mismatch = true;
    }
    CHECK(has_mismatch, "Should detect device count mismatch");
}

// ── Test 4: Multi-level hierarchy comparison ─────────────────────────

TEST(multi_level_hierarchy) {
    Netlist nl;
    PhysicalDesign pd;

    // Schematic: AND + NAND + OR (3 different types)
    auto n0 = nl.add_net("a");
    auto n1 = nl.add_net("b");
    auto n2 = nl.add_net("c");
    auto n3 = nl.add_net("w0");
    auto n4 = nl.add_net("w1");
    auto n5 = nl.add_net("out");
    nl.add_gate(GateType::INPUT, {}, n0, "in_a");
    nl.add_gate(GateType::INPUT, {}, n1, "in_b");
    nl.add_gate(GateType::INPUT, {}, n2, "in_c");
    nl.add_gate(GateType::AND, {n0, n1}, n3, "g_and");
    nl.add_gate(GateType::NAND, {n1, n2}, n4, "g_nand");
    nl.add_gate(GateType::OR, {n3, n4}, n5, "g_or");

    // Layout: matching
    pd.die_area = Rect(0, 0, 200, 200);
    pd.add_cell("g_and", "AND", 4, 2);
    pd.add_cell("g_nand", "NAND", 4, 2);
    pd.add_cell("g_or", "OR", 4, 2);
    pd.add_net("w0", {0, 2});
    pd.add_net("w1", {1, 2});

    LvsChecker checker(nl, pd);
    auto result = checker.check_hierarchical();

    CHECK(result.blocks_compared == 3, "Should compare 3 blocks");
    CHECK(result.blocks_matched >= 1, "Should match at least 1 block");
    CHECK(result.report.find("block") != std::string::npos ||
          result.report.find("Block") != std::string::npos ||
          result.report.find("compared") != std::string::npos,
          "Report should contain comparison info");
}

// ── Test 5: Block device count verification ──────────────────────────

TEST(block_device_count) {
    Netlist nl;
    PhysicalDesign pd;
    build_simple_design(nl, pd);

    LvsChecker checker(nl, pd);
    auto blocks = checker.extract_hierarchy();

    int total_devices = 0;
    for (const auto& b : blocks) {
        total_devices += static_cast<int>(b.devices.size());
    }
    // Total devices = 3 (and0, and1, or0)
    CHECK(total_devices == 3, "Total device count should be 3");

    // Each device should have a name
    for (const auto& b : blocks) {
        for (const auto& d : b.devices) {
            CHECK(!d.name.empty(), "Device should have a name");
        }
    }
}

// ── Test 6: Block port identification ────────────────────────────────

TEST(block_port_identification) {
    Netlist nl;
    PhysicalDesign pd;
    build_simple_design(nl, pd);

    LvsChecker checker(nl, pd);
    auto blocks = checker.extract_hierarchy();

    // AND block connects to OR block via nets w0, w1 — these should be ports
    auto it = std::find_if(blocks.begin(), blocks.end(),
        [](const LvsChecker::HierBlock& b) { return b.name == "AND"; });
    CHECK(it != blocks.end(), "Should find AND block");
    CHECK(!it->ports.empty(), "AND block should have ports (connects to OR)");

    // OR block should also have ports (connects to AND block)
    auto itor = std::find_if(blocks.begin(), blocks.end(),
        [](const LvsChecker::HierBlock& b) { return b.name == "OR"; });
    CHECK(itor != blocks.end(), "Should find OR block");
    CHECK(!itor->ports.empty(), "OR block should have ports");
}

// ── Test 7: DRC DENSITY rule parsing ─────────────────────────────────

TEST(drc_density_parse) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);

    DrcEngine drc(pd);
    int loaded = drc.load_rule_deck_string("DENSITY M1 0.2 0.8 50.0 10.0\n");
    CHECK(loaded == 1, "Should load 1 DENSITY rule");
}

// ── Test 8: DRC OFFGRID rule parsing ─────────────────────────────────

TEST(drc_offgrid_parse) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);

    DrcEngine drc(pd);
    int loaded = drc.load_rule_deck_string("OFFGRID M1 0.005 0.005\n");
    CHECK(loaded == 1, "Should load 1 OFFGRID rule");
}

// ── Test 9: DRC ANTENNAAREA rule parsing ─────────────────────────────

TEST(drc_antennaarea_parse) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);

    DrcEngine drc(pd);
    int loaded = drc.load_rule_deck_string("ANTENNAAREA M1 500.0 1000.0\n");
    CHECK(loaded == 1, "Should load 1 ANTENNAAREA rule");

    // Verify the rule was stored
    bool found_antenna = false;
    for (const auto& r : drc.rules()) {
        if (r.name.find("AA.DECK") != std::string::npos) {
            found_antenna = true;
            CHECK(std::abs(r.value - 500.0) < 0.01, "Antenna ratio should be 500");
            CHECK(std::abs(r.aux_value - 1000.0) < 0.01, "Cumulative ratio should be 1000");
        }
    }
    CHECK(found_antenna, "Should find antenna area rule in rules list");
}

// ── Test 10: DRC VARIABLE directive ──────────────────────────────────

TEST(drc_variable_parse) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);

    DrcEngine drc(pd);
    int loaded = drc.load_rule_deck_string(
        "VARIABLE min_space 0.065\n"
        "VARIABLE min_width 0.040\n"
    );
    CHECK(loaded == 2, "Should load 2 VARIABLE directives");

    // Variables stored as disabled INFO rules with VAR. prefix
    int var_count = 0;
    for (const auto& r : drc.rules()) {
        if (r.name.find("VAR.") == 0) {
            var_count++;
            CHECK(r.severity == DrcRule::INFO, "Variable should be INFO severity");
            CHECK(!r.enabled, "Variable rule should be disabled");
        }
    }
    CHECK(var_count == 2, "Should have 2 variable entries");
}

// ── Test 11: DRC RULECHECK directive ─────────────────────────────────

TEST(drc_rulecheck_parse) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);

    DrcEngine drc(pd);
    int loaded = drc.load_rule_deck_string("RULECHECK metal_spacing_group\n");
    CHECK(loaded == 1, "Should load 1 RULECHECK directive");

    bool found = false;
    for (const auto& r : drc.rules()) {
        if (r.name == "RULECHECK.METAL_SPACING_GROUP") {
            found = true;
            CHECK(r.severity == DrcRule::INFO, "RULECHECK should be INFO severity");
            CHECK(r.description.find("Rule check group") != std::string::npos,
                  "Description should mention rule check group");
        }
    }
    CHECK(found, "Should find RULECHECK entry in rules list");
}

// ── Test 12: Combined hierarchical LVS + DRC flow ───────────────────

TEST(combined_hier_lvs_drc) {
    Netlist nl;
    PhysicalDesign pd;
    build_simple_design(nl, pd);

    // Add a wire for DRC to check
    WireSegment w;
    w.layer = 14; // MET1
    w.start = Point(10, 10);
    w.end = Point(50, 10);
    w.width = 0.1;
    w.net_id = 0;
    pd.wires.push_back(w);

    // Run hierarchical LVS
    LvsChecker lvs(nl, pd);
    auto lvs_result = lvs.check_hierarchical();
    CHECK(lvs_result.blocks_compared > 0, "LVS should compare blocks");

    // Run DRC with rule deck
    DrcEngine drc(pd);
    int loaded = drc.load_rule_deck_string(
        "# Combined rule deck\n"
        "WIDTH M1 0.04\n"
        "SPACING M1 0.065\n"
        "DENSITY M1 0.2 0.8 50.0 10.0\n"
        "OFFGRID M1 0.005 0.005\n"
        "ANTENNAAREA M1 400.0\n"
        "VARIABLE tech_node 0.130\n"
        "RULECHECK metal_checks\n"
    );
    CHECK(loaded == 7, "Should load 7 rules/directives from combined deck");

    // Verify both engines produced results
    CHECK(!lvs_result.report.empty(), "LVS report should exist");
    CHECK(drc.rule_count() >= 5, "DRC should have rules loaded");
}

// ══════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "=== Phase 79: Hierarchical LVS & Rule Deck Tests ===\n";

    RUN(hierarchy_extraction);
    RUN(hierarchical_lvs_match);
    RUN(hierarchical_lvs_mismatch);
    RUN(multi_level_hierarchy);
    RUN(block_device_count);
    RUN(block_port_identification);
    RUN(drc_density_parse);
    RUN(drc_offgrid_parse);
    RUN(drc_antennaarea_parse);
    RUN(drc_variable_parse);
    RUN(drc_rulecheck_parse);
    RUN(combined_hier_lvs_drc);

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_run
              << " checks passed ===\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
