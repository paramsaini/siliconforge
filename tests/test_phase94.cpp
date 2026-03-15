// SiliconForge — Phase 94 Tests
// Standard cell characterization engine, cell-level DRC, and cell-level LVS.

#include "stdcell/cell_drc.hpp"
#include "stdcell/cell_lvs.hpp"
#include "stdcell/characterizer.hpp"
#include "stdcell/cell_layout.hpp"
#include "verify/drc.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <string>

static int g_pass = 0, g_fail = 0;

static void check(bool cond, const std::string& name) {
    if (cond) { ++g_pass; std::cout << "  PASS  " << name << "\n"; }
    else      { ++g_fail; std::cout << "**FAIL  " << name << "\n"; }
}

// ── Helper: build a simple inverter cell layout (DRC clean) ──────────────
// Sky130-style: NWELL on top half, DIFF+POLY forming NMOS (bottom) and
// PMOS (top), LICON contacts, implant layers.

static sf::CellLayout make_inv_layout_clean() {
    sf::CellLayout cl;
    cl.cell_name = "INV_X1";
    cl.width  = 1.38;
    cl.height = 2.72;

    // NWELL covering upper half (PMOS region)
    cl.rects.emplace_back(sf::DrcLayer::NWELL, 0.0, 1.36, 1.38, 2.72);

    // PMOS diffusion (in NWELL)
    cl.rects.emplace_back(sf::DrcLayer::DIFF, 0.26, 1.60, 1.12, 2.18);

    // NMOS diffusion (in PWELL, lower half)
    cl.rects.emplace_back(sf::DrcLayer::DIFF, 0.26, 0.54, 1.12, 1.12);

    // Poly gate crossing both diffusions with proper endcap
    // Poly extends 0.14 um past diff on each side (> 0.13 endcap rule)
    cl.rects.emplace_back(sf::DrcLayer::POLY, 0.56, 0.40, 0.82, 2.32);

    // LICON contacts (0.17 x 0.17) with proper spacing
    cl.rects.emplace_back(sf::DrcLayer::LICON, 0.30, 0.70, 0.47, 0.87);  // NMOS source
    cl.rects.emplace_back(sf::DrcLayer::LICON, 0.91, 0.70, 1.08, 0.87);  // NMOS drain
    cl.rects.emplace_back(sf::DrcLayer::LICON, 0.30, 1.76, 0.47, 1.93); // PMOS source
    cl.rects.emplace_back(sf::DrcLayer::LICON, 0.91, 1.76, 1.08, 1.93); // PMOS drain

    // NSDM implant enclosing NMOS diffusion
    cl.rects.emplace_back(sf::DrcLayer::NSDM, 0.13, 0.41, 1.25, 1.25);

    // PSDM implant enclosing PMOS diffusion
    cl.rects.emplace_back(sf::DrcLayer::PSDM, 0.13, 1.47, 1.25, 2.31);

    // Pins
    cl.pins.emplace_back("A",   sf::DrcLayer::POLY, sf::Rect(0.56, 0.40, 0.82, 2.32), "INPUT");
    cl.pins.emplace_back("Y",   sf::DrcLayer::DIFF, sf::Rect(0.91, 0.70, 1.08, 0.87), "OUTPUT");
    cl.pins.emplace_back("VDD", sf::DrcLayer::DIFF, sf::Rect(0.30, 1.76, 0.47, 1.93), "POWER");
    cl.pins.emplace_back("VSS", sf::DrcLayer::DIFF, sf::Rect(0.30, 0.70, 0.47, 0.87), "GROUND");

    return cl;
}

// ── Helper: build a layout with intentional DRC violation ────────────────

static sf::CellLayout make_inv_layout_violation() {
    auto cl = make_inv_layout_clean();

    // Add a narrow poly rect (width 0.10 < 0.15 min)
    cl.rects.emplace_back(sf::DrcLayer::POLY, 0.20, 0.40, 0.30, 2.32);

    return cl;
}

// ── Test: Cell DRC Clean ─────────────────────────────────────────────────

static void test_cell_drc_clean() {
    std::cout << "[test_cell_drc_clean]\n";

    auto layout = make_inv_layout_clean();
    sf::CellDrcChecker checker(layout);
    checker.load_sky130_rules();

    auto result = checker.check();
    check(result.clean, "clean layout passes DRC");
    check(result.violations_count == 0, "zero violations");
    check(result.time_ms >= 0.0, "timing recorded");
}

// ── Test: Cell DRC Violation ─────────────────────────────────────────────

static void test_cell_drc_violation() {
    std::cout << "[test_cell_drc_violation]\n";

    auto layout = make_inv_layout_violation();
    sf::CellDrcChecker checker(layout);
    checker.load_sky130_rules();

    auto result = checker.check();
    check(!result.clean, "violated layout fails DRC");
    check(result.violations_count > 0, "violations detected");

    // Should catch the narrow poly
    bool found_poly_width = false;
    for (const auto& v : result.violations) {
        if (v.rule_name.find("poly") != std::string::npos) {
            found_poly_width = true;
            break;
        }
    }
    check(found_poly_width, "poly min-width violation caught");
}

// ── Test: Cell DRC Rules Loaded ──────────────────────────────────────────

static void test_cell_drc_rules() {
    std::cout << "[test_cell_drc_rules]\n";

    sf::CellLayout dummy;
    sf::CellDrcChecker checker(dummy);
    checker.load_sky130_rules();

    const auto& rules = checker.rules();
    check(rules.size() >= 10, "at least 10 sky130 rules loaded");

    // Verify specific rules exist
    bool has_poly_width = false, has_diff_spacing = false, has_licon = false;
    for (const auto& r : rules) {
        if (r.name == "poly.1") has_poly_width = true;
        if (r.name == "diff.2") has_diff_spacing = true;
        if (r.name == "licon.1") has_licon = true;
    }
    check(has_poly_width, "poly.1 rule present");
    check(has_diff_spacing, "diff.2 rule present");
    check(has_licon, "licon.1 rule present");
}

// ── Test: Cell LVS Match ─────────────────────────────────────────────────

static void test_cell_lvs_match() {
    std::cout << "[test_cell_lvs_match]\n";

    auto layout = make_inv_layout_clean();

    // Schematic: one NMOS + one PMOS matching layout dimensions
    // NMOS: poly over lower diff, W = diff height = 0.58, L = poly width = 0.26
    // PMOS: poly over upper diff, W = diff height = 0.58, L = poly width = 0.26
    std::vector<sf::CellLvsDevice> schematic;
    schematic.push_back({sf::CellLvsDevice::NMOS, "A", "Y", "VSS", "VSS",
                         0.58, 0.26});
    schematic.push_back({sf::CellLvsDevice::PMOS, "A", "Y", "VDD", "VDD",
                         0.58, 0.26});

    sf::CellLvsChecker checker(layout, schematic);
    auto result = checker.compare();

    check(result.match, "INV layout matches schematic");
    check(result.layout_devices == 2, "2 layout devices extracted");
    check(result.schematic_devices == 2, "2 schematic devices");
    check(result.matched == 2, "all devices matched");
    check(result.mismatches.empty(), "no mismatches");
}

// ── Test: Cell LVS Mismatch ─────────────────────────────────────────────

static void test_cell_lvs_mismatch() {
    std::cout << "[test_cell_lvs_mismatch]\n";

    auto layout = make_inv_layout_clean();

    // Schematic with an extra device
    std::vector<sf::CellLvsDevice> schematic;
    schematic.push_back({sf::CellLvsDevice::NMOS, "A", "Y", "VSS", "VSS",
                         0.58, 0.26});
    schematic.push_back({sf::CellLvsDevice::PMOS, "A", "Y", "VDD", "VDD",
                         0.58, 0.26});
    schematic.push_back({sf::CellLvsDevice::NMOS, "B", "Z", "VSS", "VSS",
                         0.42, 0.15});

    sf::CellLvsChecker checker(layout, schematic);
    auto result = checker.compare();

    check(!result.match, "mismatch detected");
    check(result.schematic_devices == 3, "3 schematic devices");
    check(!result.mismatches.empty(), "mismatch reasons reported");
}

// ── Test: Characterize Inverter ──────────────────────────────────────────

static void test_char_inverter() {
    std::cout << "[test_char_inverter]\n";

    sf::CharConfig cfg;
    sf::CellCharacterizer charzer(cfg);

    auto result = charzer.characterize_combinational(
        "INV_X1", {"A"}, "Y", "!A", 0.76);

    check(result.cell_name == "INV_X1", "cell name");
    check(std::fabs(result.area - 0.76) < 1e-6, "area preserved");
    check(result.pins.size() == 2, "2 pins (A, Y)");
    check(result.timings.size() == 1, "1 timing arc (A->Y)");

    // Verify NLDM table dimensions (7x7)
    const auto& t = result.timings[0];
    check(t.nldm_rise.valid(), "rise NLDM valid");
    check(t.nldm_fall.valid(), "fall NLDM valid");
    check(t.nldm_rise.index_1.size() == 7, "7 slew breakpoints");
    check(t.nldm_rise.index_2.size() == 7, "7 load breakpoints");
    check(t.nldm_rise.values.size() == 7, "7x7 rise delay table");
    check(t.nldm_rise.values[0].size() == 7, "7 columns in rise delay");

    // Verify physically realistic delay range
    double min_delay = t.nldm_rise.values[0][0];
    double max_delay = t.nldm_rise.values[6][6];
    check(min_delay > 0.005 && min_delay < 0.1,
          "min delay in [5ps, 100ps] range");
    check(max_delay > 0.05 && max_delay < 5.0,
          "max delay in [50ps, 5ns] range");

    // Leakage in realistic range
    check(result.leakage_power > 1e-12 && result.leakage_power < 1e-6,
          "leakage in [1pW, 1uW] range");
}

// ── Test: Characterize NAND2 ─────────────────────────────────────────────

static void test_char_nand2() {
    std::cout << "[test_char_nand2]\n";

    sf::CharConfig cfg;
    sf::CellCharacterizer charzer(cfg);

    auto result = charzer.characterize_combinational(
        "NAND2_X1", {"A", "B"}, "Y", "!(A & B)", 1.14);

    check(result.cell_name == "NAND2_X1", "cell name");
    check(result.pins.size() == 3, "3 pins (A, B, Y)");
    check(result.timings.size() == 2, "2 timing arcs (A->Y, B->Y)");

    // NAND2 should be slower than INV on fall (series NMOS)
    // Just verify tables are populated and reasonable
    for (const auto& t : result.timings) {
        check(t.nldm_rise.valid(), "rise table valid for " + t.related_pin);
        check(t.nldm_fall.valid(), "fall table valid for " + t.related_pin);
    }
}

// ── Test: Characterize DFF ───────────────────────────────────────────────

static void test_char_dff() {
    std::cout << "[test_char_dff]\n";

    sf::CharConfig cfg;
    sf::CellCharacterizer charzer(cfg);

    auto result = charzer.characterize_sequential(
        "DFF_X1", "D", "CLK", "Q", 4.56);

    check(result.cell_name == "DFF_X1", "cell name");
    check(result.pins.size() == 3, "3 pins (D, CLK, Q)");

    // Should have CLK->Q delay, setup, and hold timing arcs
    check(result.timings.size() == 3, "3 timing arcs (clk->q, setup, hold)");

    bool has_rising_edge = false, has_setup = false, has_hold = false;
    for (const auto& t : result.timings) {
        if (t.timing_type == "rising_edge") has_rising_edge = true;
        if (t.timing_type == "setup_rising") has_setup = true;
        if (t.timing_type == "hold_rising") has_hold = true;
    }
    check(has_rising_edge, "CLK->Q rising_edge timing");
    check(has_setup, "setup_rising timing");
    check(has_hold, "hold_rising timing");
}

// ── Test: NLDM Monotonicity ─────────────────────────────────────────────
// Delay must increase with output load (fundamental physics).

static void test_char_nldm_monotonic() {
    std::cout << "[test_char_nldm_monotonic]\n";

    sf::CharConfig cfg;
    sf::CellCharacterizer charzer(cfg);

    auto result = charzer.characterize_combinational(
        "INV_X1", {"A"}, "Y", "!A", 0.76);

    const auto& t = result.timings[0];

    // Check monotonicity along load axis (each row)
    bool rise_monotonic = true, fall_monotonic = true;
    for (size_t si = 0; si < t.nldm_rise.values.size(); ++si) {
        for (size_t li = 1; li < t.nldm_rise.values[si].size(); ++li) {
            if (t.nldm_rise.values[si][li] < t.nldm_rise.values[si][li-1] - 1e-9)
                rise_monotonic = false;
            if (t.nldm_fall.values[si][li] < t.nldm_fall.values[si][li-1] - 1e-9)
                fall_monotonic = false;
        }
    }
    check(rise_monotonic, "rise delay monotonic with load");
    check(fall_monotonic, "fall delay monotonic with load");

    // Check monotonicity along slew axis (each column)
    bool slew_rise_mono = true, slew_fall_mono = true;
    for (size_t li = 0; li < t.nldm_rise.values[0].size(); ++li) {
        for (size_t si = 1; si < t.nldm_rise.values.size(); ++si) {
            if (t.nldm_rise.values[si][li] < t.nldm_rise.values[si-1][li] - 1e-9)
                slew_rise_mono = false;
            if (t.nldm_fall.values[si][li] < t.nldm_fall.values[si-1][li] - 1e-9)
                slew_fall_mono = false;
        }
    }
    check(slew_rise_mono, "rise delay monotonic with input slew");
    check(slew_fall_mono, "fall delay monotonic with input slew");
}

// ── Test: CharResult to LibertyCell Round-Trip ───────────────────────────

static void test_char_to_liberty() {
    std::cout << "[test_char_to_liberty]\n";

    sf::CharConfig cfg;
    sf::CellCharacterizer charzer(cfg);

    auto result = charzer.characterize_combinational(
        "INV_X1", {"A"}, "Y", "!A", 0.76);

    auto cell = charzer.to_liberty_cell(result);

    check(cell.name == "INV_X1", "cell name preserved");
    check(std::fabs(cell.area - 0.76) < 1e-6, "area preserved");
    check(cell.pins.size() == 2, "pins preserved");
    check(cell.timings.size() == 1, "timings preserved");
    check(cell.leakage_power > 0, "leakage power set");

    // Verify pin details survived
    bool found_input = false, found_output = false;
    for (const auto& p : cell.pins) {
        if (p.name == "A" && p.direction == "input") found_input = true;
        if (p.name == "Y" && p.direction == "output") found_output = true;
    }
    check(found_input, "input pin A present");
    check(found_output, "output pin Y present");
}

// ── Test: Generate Library ───────────────────────────────────────────────

static void test_generate_library() {
    std::cout << "[test_generate_library]\n";

    sf::CharConfig cfg;
    sf::CellCharacterizer charzer(cfg);

    auto inv = charzer.characterize_combinational(
        "INV_X1", {"A"}, "Y", "!A", 0.76);
    auto nand = charzer.characterize_combinational(
        "NAND2_X1", {"A", "B"}, "Y", "!(A & B)", 1.14);
    auto dff = charzer.characterize_sequential(
        "DFF_X1", "D", "CLK", "Q", 4.56);

    auto lib = charzer.generate_library(
        "sky130_fd_sc_hd__tt_025C_1v80",
        {inv, nand, dff}, 1.8, 25.0);

    check(lib.name == "sky130_fd_sc_hd__tt_025C_1v80", "library name");
    check(std::fabs(lib.nom_voltage - 1.8) < 1e-6, "nom voltage");
    check(std::fabs(lib.nom_temperature - 25.0) < 1e-6, "nom temperature");
    check(lib.cells.size() == 3, "3 cells in library");
    check(lib.time_unit == "1ns", "time unit");
    check(lib.cap_unit == "1pF", "cap unit");

    // Verify table template stored
    check(lib.table_templates.count("delay_template_7x7") > 0,
          "delay template stored");

    // Verify each cell has correct name
    check(lib.cells[0].name == "INV_X1", "INV cell");
    check(lib.cells[1].name == "NAND2_X1", "NAND2 cell");
    check(lib.cells[2].name == "DFF_X1", "DFF cell");
}

// ── Main ─────────────────────────────────────────────────────────────────

int main() {
    std::cout << "=== Phase 94: Standard Cell Characterization & Cell DRC/LVS ===\n\n";

    test_cell_drc_clean();
    test_cell_drc_violation();
    test_cell_drc_rules();
    test_cell_lvs_match();
    test_cell_lvs_mismatch();
    test_char_inverter();
    test_char_nand2();
    test_char_dff();
    test_char_nldm_monotonic();
    test_char_to_liberty();
    test_generate_library();

    std::cout << "\n=== Phase 94 Summary: " << g_pass << " passed, "
              << g_fail << " failed ===\n";
    return g_fail ? 1 : 0;
}
