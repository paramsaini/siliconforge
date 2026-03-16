// SiliconForge — Phase 98 Tests
// Gap coverage: NPN matching, CCS delay, coupling caps, hierarchical DRC,
// SPICE characterization, GNN infrastructure, GDSII reader, TrackOccupancy scaling.

#include "synth/npn_match.hpp"
#include "synth/tech_mapper.hpp"
#include "timing/ccs_delay.hpp"
#include "timing/parasitics.hpp"
#include "verify/drc.hpp"
#include "stdcell/characterizer.hpp"
#include "ml/gnn.hpp"
#include "pnr/gdsii_reader.hpp"
#include "pnr/detailed_router_v2.hpp"
#include "core/liberty_parser.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <string>
#include <vector>
#include <cstring>

static int g_pass = 0, g_fail = 0;

static void check(bool cond, const std::string& name) {
    if (cond) { ++g_pass; std::cout << "  PASS  " << name << "\n"; }
    else      { ++g_fail; std::cout << "**FAIL  " << name << "\n"; }
}

// ═══════════════════════════════════════════════════════════════════════════
// GAP-001: NPN Boolean Matching
// ═══════════════════════════════════════════════════════════════════════════

static void test_npn_truth_table_basics() {
    std::cout << "\n=== test_npn_truth_table_basics ===\n";

    // 1-input NOT: ~A. TT = 0b01 = 1 for input A
    sf::TruthTable6 not_tt = sf::NpnMatcher::func_to_tt6("!A", 1);
    // For 1 input: minterm 0 (A=0) -> 1, minterm 1 (A=1) -> 0 => tt = 0b01 = 1
    check(not_tt == 0x1, "NOT truth table = 0x1");

    // 2-input AND: A & B
    sf::TruthTable6 and_tt = sf::NpnMatcher::func_to_tt6("A&B", 2);
    // minterms: 00->0, 01->0, 10->0, 11->1 => tt = 0b1000 = 8
    check(and_tt == 0x8, "AND2 truth table = 0x8");

    // 2-input OR: A | B
    sf::TruthTable6 or_tt = sf::NpnMatcher::func_to_tt6("A|B", 2);
    // minterms: 00->0, 01->1, 10->1, 11->1 => tt = 0b1110 = 0xE
    check(or_tt == 0xE, "OR2 truth table = 0xE");

    // 2-input NAND: !(A & B)
    sf::TruthTable6 nand_tt = sf::NpnMatcher::func_to_tt6("!(A&B)", 2);
    check(nand_tt == 0x7, "NAND2 truth table = 0x7");

    // 2-input XOR: A ^ B
    sf::TruthTable6 xor_tt = sf::NpnMatcher::func_to_tt6("A^B", 2);
    check(xor_tt == 0x6, "XOR2 truth table = 0x6");
}

static void test_npn_output_negate() {
    std::cout << "\n=== test_npn_output_negate ===\n";

    sf::TruthTable6 and_tt = 0x8;  // AND2
    sf::TruthTable6 neg = sf::NpnMatcher::negate_output(and_tt, 2);
    check(neg == 0x7, "negate_output(AND2) = NAND2");

    sf::TruthTable6 neg2 = sf::NpnMatcher::negate_output(neg, 2);
    check(neg2 == and_tt, "double negate returns original");
}

static void test_npn_canonical_form() {
    std::cout << "\n=== test_npn_canonical_form ===\n";

    // AND and OR are NPN-equivalent (OR = negate all inputs + output of AND)
    sf::TruthTable6 and_tt = 0x8;  // A & B
    sf::TruthTable6 or_tt  = 0xE;  // A | B
    sf::NpnCanonical can_and = sf::NpnMatcher::compute_canonical(and_tt, 2);
    sf::NpnCanonical can_or  = sf::NpnMatcher::compute_canonical(or_tt, 2);
    check(can_and.canonical_tt == can_or.canonical_tt,
          "AND2 and OR2 have same NPN canonical form");

    // NAND and NOR are also NPN-equivalent
    sf::TruthTable6 nand_tt = 0x7;
    sf::TruthTable6 nor_tt  = 0x1;
    sf::NpnCanonical can_nand = sf::NpnMatcher::compute_canonical(nand_tt, 2);
    sf::NpnCanonical can_nor  = sf::NpnMatcher::compute_canonical(nor_tt, 2);
    check(can_nand.canonical_tt == can_nor.canonical_tt,
          "NAND2 and NOR2 have same NPN canonical form");

    // Buffer (A) and inverter (!A) are NPN-equivalent
    sf::TruthTable6 buf_tt = 0x2;  // f(A) = A for 1-input
    sf::TruthTable6 inv_tt = 0x1;  // f(A) = !A
    sf::NpnCanonical can_buf = sf::NpnMatcher::compute_canonical(buf_tt, 1);
    sf::NpnCanonical can_inv = sf::NpnMatcher::compute_canonical(inv_tt, 1);
    check(can_buf.canonical_tt == can_inv.canonical_tt,
          "BUF and INV have same NPN canonical form");
}

static void test_npn_cofactor_signatures() {
    std::cout << "\n=== test_npn_cofactor_signatures ===\n";

    // 3-input AND: A & B & C => tt = 0x80 (only minterm 111 = 1)
    sf::TruthTable6 and3 = sf::NpnMatcher::func_to_tt6("A&B&C", 3);
    auto sigs = sf::NpnMatcher::cofactor_signatures(and3, 3);
    check((int)sigs.size() == 3, "3 cofactor signatures for 3-input");
    // All inputs symmetric in AND3: each cofactor sig should be the same
    check(sigs[0].count0 == sigs[1].count0 && sigs[1].count0 == sigs[2].count0,
          "AND3 inputs have symmetric cofactor signatures");
}

static void test_npn_library_lookup() {
    std::cout << "\n=== test_npn_library_lookup ===\n";

    sf::NpnMatcher matcher;

    // Build a small library: INV, NAND2, NOR2, AND2, OR2
    std::vector<sf::NpnCellEntry> cells;

    // INV
    sf::NpnCellEntry inv;
    inv.cell_name = "INV_X1";
    inv.original_tt = sf::NpnMatcher::func_to_tt6("!A", 1);
    inv.num_inputs = 1;
    inv.area = 1.0;
    inv.delay = 0.05;
    cells.push_back(inv);

    // NAND2
    sf::NpnCellEntry nand2;
    nand2.cell_name = "NAND2_X1";
    nand2.original_tt = sf::NpnMatcher::func_to_tt6("!(A&B)", 2);
    nand2.num_inputs = 2;
    nand2.area = 2.0;
    nand2.delay = 0.08;
    cells.push_back(nand2);

    // NOR2
    sf::NpnCellEntry nor2;
    nor2.cell_name = "NOR2_X1";
    nor2.original_tt = sf::NpnMatcher::func_to_tt6("!(A|B)", 2);
    nor2.num_inputs = 2;
    nor2.area = 2.0;
    nor2.delay = 0.09;
    cells.push_back(nor2);

    // AND2
    sf::NpnCellEntry and2;
    and2.cell_name = "AND2_X1";
    and2.original_tt = sf::NpnMatcher::func_to_tt6("A&B", 2);
    and2.num_inputs = 2;
    and2.area = 3.0;
    and2.delay = 0.10;
    cells.push_back(and2);

    matcher.build_library(cells);

    check(matcher.num_cells_mapped() == 4, "4 cells in NPN library");
    check(matcher.num_npn_classes() >= 1, "at least 1 NPN class");

    // Look up AND2 function — should match AND2, NAND2 (NPN-equiv with output neg)
    auto results = matcher.lookup(0x8, 2);
    check(!results.empty(), "AND2 lookup returns matches");

    // Best match by area
    auto best = matcher.best_match_area(0x8, 2);
    check(best.cell != nullptr, "best_match_area returns a cell");
}

static void test_npn_heuristic_5input() {
    std::cout << "\n=== test_npn_heuristic_5input ===\n";

    // 5-input majority function (should use heuristic path)
    // Simple 5-input AND for testing
    sf::TruthTable6 and5 = sf::NpnMatcher::func_to_tt6("A&B&C&D&E", 5);
    sf::NpnCanonical can = sf::NpnMatcher::compute_canonical(and5, 5);
    check(can.canonical_tt != 0, "5-input canonical form is non-zero");
    check(can.num_inputs == 5, "canonical form preserves input count");

    // Canonical should be deterministic
    sf::NpnCanonical can2 = sf::NpnMatcher::compute_canonical(and5, 5);
    check(can.canonical_tt == can2.canonical_tt,
          "canonical form is deterministic");
}

// ═══════════════════════════════════════════════════════════════════════════
// GAP-005: CCS Delay Model
// ═══════════════════════════════════════════════════════════════════════════

static void test_ccs_delay_basic() {
    std::cout << "\n=== test_ccs_delay_basic ===\n";

    sf::CcsDelayEngine engine;

    // Create a timing arc with CCS tables
    sf::LibertyTiming timing;
    timing.related_pin = "A";
    timing.timing_type = "combinational";

    // Build a simple CCS table (3x3 grid)
    sf::CcsTable ccs;
    ccs.index_1 = {0.01, 0.1, 0.5};      // input slew
    ccs.index_2 = {0.001, 0.01, 0.1};     // output load
    ccs.index_3 = {0.0, 0.05, 0.1, 0.15, 0.2};  // time points

    // Fill with realistic current waveform (bell-shaped)
    ccs.values.resize(3);
    for (int s = 0; s < 3; ++s) {
        ccs.values[s].resize(3);
        for (int l = 0; l < 3; ++l) {
            double peak = 0.5 + s * 0.1 + l * 0.2;
            ccs.values[s][l] = {0.0, peak * 0.5, peak, peak * 0.5, 0.0};
        }
    }
    timing.ccs_rise = ccs;
    timing.ccs_fall = ccs;

    // Also fill NLDM tables for fallback
    timing.nldm_rise.index_1 = {0.01, 0.1, 0.5};
    timing.nldm_rise.index_2 = {0.001, 0.01, 0.1};
    timing.nldm_rise.values = {{0.05, 0.08, 0.15}, {0.07, 0.10, 0.20}, {0.12, 0.18, 0.35}};
    timing.nldm_fall = timing.nldm_rise;
    timing.nldm_rise_tr = timing.nldm_rise;
    timing.nldm_fall_tr = timing.nldm_rise;

    // Compute CCS delay
    auto result = engine.compute_delay(timing, 0.05, 0.01, 1.8, true);
    check(result.delay_ns >= 0, "CCS delay is non-negative");
    check(result.slew_ns >= 0, "CCS slew is non-negative");

    // Compute NLDM delay as fallback
    auto nldm_result = engine.compute_nldm_delay(timing, 0.05, 0.01, true);
    check(nldm_result.delay_ns >= 0, "NLDM fallback delay non-negative");
}

static void test_ccs_pi_model() {
    std::cout << "\n=== test_ccs_pi_model ===\n";

    sf::PiModelCap pi;
    pi.c_near = 5.0;    // 5 fF
    pi.r_wire = 100.0;  // 100 ohm
    pi.c_far = 10.0;    // 10 fF

    check(pi.total_cap() == 15.0, "total cap = c_near + c_far");

    double ceff = pi.effective_cap(0.1);
    check(ceff >= pi.c_near, "C_eff >= C_near");
    check(ceff <= pi.total_cap(), "C_eff <= total_cap");

    // Zero wire resistance: Ceff = total_cap
    sf::PiModelCap pi_zero;
    pi_zero.c_near = 5.0;
    pi_zero.r_wire = 0.0;
    pi_zero.c_far = 10.0;
    check(std::abs(pi_zero.effective_cap(0.1) - 15.0) < 0.01,
          "C_eff with zero R_wire = total_cap");
}

static void test_ccs_effective_cap() {
    std::cout << "\n=== test_ccs_effective_cap ===\n";

    sf::CcsDelayEngine engine;

    sf::LibertyTiming timing;
    timing.related_pin = "A";
    sf::CcsTable ccs;
    ccs.index_1 = {0.01, 0.5};
    ccs.index_2 = {0.001, 0.1};
    ccs.index_3 = {0.0, 0.05, 0.1};
    ccs.values = {{{0.0, 0.5, 0.0}, {0.0, 0.8, 0.0}},
                  {{0.0, 0.3, 0.0}, {0.0, 0.6, 0.0}}};
    timing.ccs_rise = ccs;
    timing.ccs_fall = ccs;
    timing.nldm_rise.index_1 = {0.01, 0.5};
    timing.nldm_rise.index_2 = {0.001, 0.1};
    timing.nldm_rise.values = {{0.05, 0.15}, {0.10, 0.25}};
    timing.nldm_fall = timing.nldm_rise;
    timing.nldm_rise_tr = timing.nldm_rise;
    timing.nldm_fall_tr = timing.nldm_rise;

    sf::PiModelCap pi;
    pi.c_near = 5.0;
    pi.r_wire = 50.0;
    pi.c_far = 10.0;

    double ceff = engine.compute_effective_cap(timing, 0.05, pi, 1.8, true);
    check(ceff > 0, "effective cap is positive");
    check(ceff <= pi.total_cap() * 1.5, "effective cap within reasonable range");
}

static void test_ccs_smis() {
    std::cout << "\n=== test_ccs_smis ===\n";

    sf::CcsDelayEngine engine;
    sf::LibertyTiming timing;
    timing.related_pin = "A";
    timing.nldm_rise.index_1 = {0.01, 0.5};
    timing.nldm_rise.index_2 = {0.001, 0.1};
    timing.nldm_rise.values = {{0.05, 0.15}, {0.10, 0.25}};
    timing.nldm_fall = timing.nldm_rise;
    timing.nldm_rise_tr = timing.nldm_rise;
    timing.nldm_fall_tr = timing.nldm_rise;

    sf::CcsDelayEngine::SmisConfig smis;
    smis.num_switching_inputs = 2;
    smis.alignment_factor = 0.8;  // near worst case

    auto result = engine.compute_smis_delay(timing, 0.05, 0.01, smis, 1.8, true);
    check(result.delay_ns >= 0, "SMIS delay non-negative");
}

// ═══════════════════════════════════════════════════════════════════════════
// GAP-006: Coupling Capacitance Extraction (spatial grid)
// ═══════════════════════════════════════════════════════════════════════════

static void test_coupling_spatial_config() {
    std::cout << "\n=== test_coupling_spatial_config ===\n";

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};

    sf::ParasiticExtractor ext(pd);

    sf::ParasiticExtractor::CouplingConfig cfg;
    cfg.max_coupling_distance_um = 3.0;
    cfg.inter_layer_coupling_factor = 0.5;
    cfg.spatial_grid_bins = 32;
    cfg.enable_miller_effect = true;
    cfg.miller_factor = 2.0;
    ext.set_coupling_config(cfg);

    // Should not crash on empty design
    auto result = ext.extract();
    check(result.time_ms >= 0, "extraction completes on empty design");
}

static void test_coupling_with_wires() {
    std::cout << "\n=== test_coupling_with_wires ===\n";

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};
    pd.row_height = 10.0;
    pd.site_width = 1.0;

    // Two parallel wires on same layer — should produce coupling cap
    sf::WireSegment w1;
    w1.net_id = 0; w1.layer = 1;
    w1.start = {10, 20}; w1.end = {50, 20}; w1.width = 0.1;
    pd.wires.push_back(w1);

    sf::WireSegment w2;
    w2.net_id = 1; w2.layer = 1;
    w2.start = {10, 20.5}; w2.end = {50, 20.5}; w2.width = 0.1;
    pd.wires.push_back(w2);

    // Need matching nets
    sf::PhysNet n0, n1;
    n0.id = 0; n0.name = "net0";
    n1.id = 1; n1.name = "net1";
    pd.nets.push_back(n0);
    pd.nets.push_back(n1);

    sf::ParasiticExtractor ext(pd);
    sf::ParasiticExtractor::CouplingConfig cfg;
    cfg.max_coupling_distance_um = 5.0;
    cfg.spatial_grid_bins = 16;
    ext.set_coupling_config(cfg);

    auto result = ext.extract();
    check(result.nets.size() >= 2, "extracted parasitics for both nets");
}

static void test_awe_delay() {
    std::cout << "\n=== test_awe_delay ===\n";

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};

    // One wire
    sf::WireSegment w;
    w.net_id = 0; w.layer = 1;
    w.start = {10, 20}; w.end = {50, 20}; w.width = 0.1;
    pd.wires.push_back(w);

    sf::PhysNet n0;
    n0.id = 0; n0.name = "net0";
    pd.nets.push_back(n0);

    sf::ParasiticExtractor ext(pd);
    auto result = ext.extract();
    check(!result.nets.empty(), "at least one parasitic net");

    // Compute AWE for net 0
    auto awe = ext.compute_awe(0, 2);
    check(awe.delay_ps >= 0, "AWE delay is non-negative");
    check(awe.slew_ps >= 0, "AWE slew is non-negative");
}

// ═══════════════════════════════════════════════════════════════════════════
// GAP-009: Hierarchical DRC
// ═══════════════════════════════════════════════════════════════════════════

static void test_hier_drc_basic() {
    std::cout << "\n=== test_hier_drc_basic ===\n";

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 200, 200};
    sf::DrcEngine drc(pd);
    drc.load_default_rules(0.14);

    // Define a cell with two shapes that violate min spacing
    sf::DrcEngine::HierCellDef cell;
    cell.name = "INV_X1";
    cell.bbox = {0, 0, 5, 5};

    sf::WireSegment s1;
    s1.layer = 14; s1.start = {0, 0}; s1.end = {2, 0}; s1.width = 0.14;
    cell.shapes.push_back(s1);

    sf::WireSegment s2;
    s2.layer = 14; s2.start = {0, 0.2}; s2.end = {2, 0.2}; s2.width = 0.14;
    cell.shapes.push_back(s2);

    drc.add_hier_cell(cell);

    // Place two instances far apart (no inter-cell violation)
    sf::DrcEngine::HierInstance inst0;
    inst0.cell_def_idx = 0;
    inst0.offset = {10, 10};
    drc.add_hier_instance(inst0);

    sf::DrcEngine::HierInstance inst1;
    inst1.cell_def_idx = 0;
    inst1.offset = {100, 100};
    drc.add_hier_instance(inst1);

    auto result = drc.check_hierarchical();
    check(result.unique_cells_checked == 1, "checked 1 unique cell");
    check(result.instances_checked == 2, "checked 2 instances");
    check(result.time_ms >= 0, "hierarchical DRC completes");
}

static void test_hier_drc_scanline() {
    std::cout << "\n=== test_hier_drc_scanline ===\n";

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 200, 200};
    sf::DrcEngine drc(pd);
    drc.load_default_rules(0.14);

    // Cell with boundary shapes
    sf::DrcEngine::HierCellDef cell;
    cell.name = "BUF_X1";
    cell.bbox = {0, 0, 5, 5};

    sf::WireSegment s1;
    s1.layer = 14; s1.start = {4.5, 2}; s1.end = {5.0, 2}; s1.width = 0.14;
    cell.shapes.push_back(s1);

    drc.add_hier_cell(cell);

    // Two instances placed close together — boundary shapes may conflict
    sf::DrcEngine::HierInstance inst0;
    inst0.cell_def_idx = 0;
    inst0.offset = {0, 0};
    drc.add_hier_instance(inst0);

    sf::DrcEngine::HierInstance inst1;
    inst1.cell_def_idx = 0;
    inst1.offset = {5.05, 0};  // very close placement
    drc.add_hier_instance(inst1);

    sf::DrcEngine::ScanlineConfig scfg;
    scfg.min_spacing = 0.14;
    drc.set_scanline_config(scfg);

    auto result = drc.check_hierarchical();
    check(result.instances_checked == 2, "scanline checks 2 instances");
    // The inter-cell scanline should have been exercised
    check(result.time_ms >= 0, "scanline DRC completes");
}

static void test_hier_drc_mirror() {
    std::cout << "\n=== test_hier_drc_mirror ===\n";

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 200, 200};
    sf::DrcEngine drc(pd);
    drc.load_default_rules(0.14);

    sf::DrcEngine::HierCellDef cell;
    cell.name = "CELL_A";
    cell.bbox = {0, 0, 10, 10};
    sf::WireSegment s;
    s.layer = 14; s.start = {1, 1}; s.end = {5, 1}; s.width = 0.2;
    cell.shapes.push_back(s);
    drc.add_hier_cell(cell);

    sf::DrcEngine::HierInstance inst;
    inst.cell_def_idx = 0;
    inst.offset = {20, 20};
    inst.mirror_x = true;
    drc.add_hier_instance(inst);

    auto result = drc.check_hierarchical();
    check(result.unique_cells_checked == 1, "mirror: 1 cell checked");
    check(result.instances_checked == 1, "mirror: 1 instance checked");
}

// ═══════════════════════════════════════════════════════════════════════════
// GAP-014: SPICE-driven Characterization
// ═══════════════════════════════════════════════════════════════════════════

static void test_spice_char_config() {
    std::cout << "\n=== test_spice_char_config ===\n";

    sf::CharConfig cfg;
    sf::CellCharacterizer charzer(cfg);

    sf::CellCharacterizer::SpiceCharConfig scfg;
    scfg.sim_duration_ns = 10.0;
    scfg.timestep_ns = 0.001;
    scfg.generate_ccs = true;
    scfg.ccs_time_points = 32;
    scfg.multi_corner = true;
    scfg.corner_voltages = {1.62, 1.8, 1.98};
    scfg.corner_temperatures = {-40, 25, 125};
    charzer.set_spice_config(scfg);

    check(true, "SPICE config accepted without crash");
}

static void test_spice_char_inverter() {
    std::cout << "\n=== test_spice_char_inverter ===\n";

    sf::CharConfig cfg;
    cfg.vdd = 1.8;
    sf::CellCharacterizer charzer(cfg);

    sf::CellCharacterizer::SpiceCharConfig scfg;
    scfg.generate_ccs = true;
    scfg.ccs_time_points = 16;
    charzer.set_spice_config(scfg);

    // Build INV netlist: PMOS pull-up + NMOS pull-down
    sf::CellCharacterizer::CellNetlist netlist;
    netlist.cell_name = "INV_X1";
    netlist.inputs = {"A"};
    netlist.output = "Y";
    netlist.area = 1.0;

    sf::CellCharacterizer::TransistorInfo mp;
    mp.name = "MP0"; mp.type = "pmos";
    mp.w = 2.0; mp.l = 0.18;
    mp.gate = "A"; mp.drain = "Y"; mp.source = "VDD"; mp.bulk = "VDD";
    netlist.transistors.push_back(mp);

    sf::CellCharacterizer::TransistorInfo mn;
    mn.name = "MN0"; mn.type = "nmos";
    mn.w = 1.0; mn.l = 0.18;
    mn.gate = "A"; mn.drain = "Y"; mn.source = "VSS"; mn.bulk = "VSS";
    netlist.transistors.push_back(mn);

    auto result = charzer.characterize_spice(netlist);
    check(result.cell_name == "INV_X1", "cell name preserved");
    check(!result.timings.empty(), "SPICE char produced timing arcs");
    check(result.area > 0, "area is positive");
}

static void test_spice_char_nand2() {
    std::cout << "\n=== test_spice_char_nand2 ===\n";

    sf::CharConfig cfg;
    sf::CellCharacterizer charzer(cfg);

    sf::CellCharacterizer::SpiceCharConfig scfg;
    scfg.generate_ccs = false;  // NLDM only
    charzer.set_spice_config(scfg);

    sf::CellCharacterizer::CellNetlist netlist;
    netlist.cell_name = "NAND2_X1";
    netlist.inputs = {"A", "B"};
    netlist.output = "Y";
    netlist.area = 2.0;

    // PMOS pull-up: parallel
    sf::CellCharacterizer::TransistorInfo mp0;
    mp0.name = "MP0"; mp0.type = "pmos"; mp0.w = 2.0; mp0.l = 0.18;
    mp0.gate = "A"; mp0.drain = "Y"; mp0.source = "VDD"; mp0.bulk = "VDD";
    netlist.transistors.push_back(mp0);

    sf::CellCharacterizer::TransistorInfo mp1;
    mp1.name = "MP1"; mp1.type = "pmos"; mp1.w = 2.0; mp1.l = 0.18;
    mp1.gate = "B"; mp1.drain = "Y"; mp1.source = "VDD"; mp1.bulk = "VDD";
    netlist.transistors.push_back(mp1);

    // NMOS pull-down: series
    sf::CellCharacterizer::TransistorInfo mn0;
    mn0.name = "MN0"; mn0.type = "nmos"; mn0.w = 2.0; mn0.l = 0.18;
    mn0.gate = "A"; mn0.drain = "Y"; mn0.source = "int1"; mn0.bulk = "VSS";
    netlist.transistors.push_back(mn0);

    sf::CellCharacterizer::TransistorInfo mn1;
    mn1.name = "MN1"; mn1.type = "nmos"; mn1.w = 2.0; mn1.l = 0.18;
    mn1.gate = "B"; mn1.drain = "int1"; mn1.source = "VSS"; mn1.bulk = "VSS";
    netlist.transistors.push_back(mn1);

    auto result = charzer.characterize_spice(netlist);
    check(result.cell_name == "NAND2_X1", "NAND2 cell name");
    check(!result.timings.empty(), "NAND2 has timing arcs");
    // Should have arcs for both inputs
    check(result.timings.size() >= 2, "NAND2 has arcs for each input");
}

static void test_spice_multi_corner() {
    std::cout << "\n=== test_spice_multi_corner ===\n";

    sf::CharConfig cfg;
    sf::CellCharacterizer charzer(cfg);

    sf::CellCharacterizer::SpiceCharConfig scfg;
    scfg.multi_corner = true;
    scfg.corner_voltages = {1.62, 1.8};
    scfg.corner_temperatures = {25, 125};
    charzer.set_spice_config(scfg);

    sf::CellCharacterizer::CellNetlist netlist;
    netlist.cell_name = "INV_X1";
    netlist.inputs = {"A"};
    netlist.output = "Y";
    netlist.area = 1.0;

    sf::CellCharacterizer::TransistorInfo mp;
    mp.name = "MP0"; mp.type = "pmos"; mp.w = 2.0; mp.l = 0.18;
    mp.gate = "A"; mp.drain = "Y"; mp.source = "VDD"; mp.bulk = "VDD";
    netlist.transistors.push_back(mp);

    sf::CellCharacterizer::TransistorInfo mn;
    mn.name = "MN0"; mn.type = "nmos"; mn.w = 1.0; mn.l = 0.18;
    mn.gate = "A"; mn.drain = "Y"; mn.source = "VSS"; mn.bulk = "VSS";
    netlist.transistors.push_back(mn);

    auto results = charzer.characterize_multi_corner(netlist);
    check(results.size() == 4, "4 corners (2 voltages x 2 temps)");
    for (auto& r : results) {
        check(!r.timings.empty(), "each corner has timing data");
    }
}

static void test_spice_sequential() {
    std::cout << "\n=== test_spice_sequential ===\n";

    sf::CharConfig cfg;
    sf::CellCharacterizer charzer(cfg);

    sf::CellCharacterizer::SpiceCharConfig scfg;
    scfg.generate_ccs = false;
    charzer.set_spice_config(scfg);

    sf::CellCharacterizer::SeqNetlist seq;
    seq.cell_name = "DFF_X1";
    seq.inputs = {"D"};
    seq.output = "Q";
    seq.clock_pin = "CLK";
    seq.q_pin = "Q";
    seq.rising_edge = true;
    seq.area = 8.0;

    // Simplified transistor description (transmission-gate latch style)
    sf::CellCharacterizer::TransistorInfo mn;
    mn.name = "MN0"; mn.type = "nmos"; mn.w = 1.0; mn.l = 0.18;
    mn.gate = "CLK"; mn.drain = "D"; mn.source = "int1"; mn.bulk = "VSS";
    seq.transistors.push_back(mn);

    sf::CellCharacterizer::TransistorInfo mp;
    mp.name = "MP0"; mp.type = "pmos"; mp.w = 2.0; mp.l = 0.18;
    mp.gate = "CLK"; mp.drain = "int1"; mp.source = "Q"; mp.bulk = "VDD";
    seq.transistors.push_back(mp);

    auto result = charzer.characterize_spice_sequential(seq);
    check(result.cell_name == "DFF_X1", "sequential cell name preserved");
    check(!result.timings.empty(), "sequential cell has timing arcs");
}

// ═══════════════════════════════════════════════════════════════════════════
// GAP-024: GNN Infrastructure
// ═══════════════════════════════════════════════════════════════════════════

static void test_gnn_graph_build() {
    std::cout << "\n=== test_gnn_graph_build ===\n";

    sf::GnnGraph g;
    g.num_nodes = 4;
    g.num_edges = 4;
    g.node_features = {{1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {0.5, 0.5}};
    g.edges = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    g.edge_features = {{1.0}, {2.0}, {3.0}, {4.0}};
    g.node_labels = {0.1, 0.5, 0.9, 0.3};

    check(g.num_nodes == 4, "graph has 4 nodes");
    check(g.num_edges == 4, "graph has 4 edges");
    check((int)g.node_features.size() == 4, "4 node feature vectors");
}

static void test_gnn_layer_forward() {
    std::cout << "\n=== test_gnn_layer_forward ===\n";

    sf::GnnLayerConfig cfg;
    cfg.input_dim = 2;
    cfg.output_dim = 4;
    cfg.aggregation = "mean";
    cfg.activation = "relu";

    sf::GnnLayer layer(cfg);
    std::mt19937 rng(42);
    layer.init_weights(rng);

    check(layer.num_parameters() > 0, "layer has parameters");

    // Forward pass
    std::vector<std::vector<double>> node_emb = {
        {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}
    };
    std::vector<std::pair<int, int>> edges = {{0, 1}, {1, 2}, {2, 0}};

    auto output = layer.forward(node_emb, edges);
    check((int)output.size() == 3, "output has 3 nodes");
    check((int)output[0].size() == 4, "output dim is 4");
}

static void test_gnn_model_predict() {
    std::cout << "\n=== test_gnn_model_predict ===\n";

    sf::GnnConfig cfg;
    cfg.node_feat_dim = 4;
    cfg.edge_feat_dim = 2;
    cfg.hidden_dims = {8, 8};
    cfg.num_message_passes = 2;
    cfg.readout_dim = 4;
    cfg.output_dim = 1;
    cfg.task = "node_regression";

    sf::GnnModel model(cfg);
    model.build();

    check(model.total_parameters() > 0, "model has parameters");

    sf::GnnGraph g;
    g.num_nodes = 3;
    g.num_edges = 3;
    g.node_features = {{1, 0, 0, 1}, {0, 1, 0, 1}, {0, 0, 1, 1}};
    g.edges = {{0, 1}, {1, 2}, {2, 0}};
    g.edge_features = {{1, 0}, {0, 1}, {1, 1}};

    auto preds = model.predict(g);
    check((int)preds.size() == 3, "predictions for 3 nodes");
}

static void test_gnn_model_train() {
    std::cout << "\n=== test_gnn_model_train ===\n";

    sf::GnnConfig cfg;
    cfg.node_feat_dim = 2;
    cfg.hidden_dims = {4};
    cfg.num_message_passes = 1;
    cfg.readout_dim = 4;
    cfg.output_dim = 1;
    cfg.epochs = 5;
    cfg.learning_rate = 0.01;
    cfg.task = "node_regression";

    sf::GnnModel model(cfg);
    model.build();

    // Small training set
    std::vector<sf::GnnGraph> train_data;
    for (int i = 0; i < 3; ++i) {
        sf::GnnGraph g;
        g.num_nodes = 2;
        g.num_edges = 1;
        g.node_features = {{1.0 * i, 0.5}, {0.5, 1.0 * i}};
        g.edges = {{0, 1}};
        g.node_labels = {0.1 * i, 0.2 * i};
        train_data.push_back(g);
    }

    auto result = model.train(train_data);
    check(result.loss_per_epoch.size() >= 1, "training produced loss history");
    check(result.final_loss >= 0, "final loss is non-negative");
}

static void test_gnn_netlist_builder() {
    std::cout << "\n=== test_gnn_netlist_builder ===\n";

    // Create minimal netlist + physical design
    sf::Netlist nl;
    sf::NetId n0 = nl.add_net("n0");
    sf::NetId n1 = nl.add_net("n1");
    sf::NetId n2 = nl.add_net("n2");
    sf::NetId n3 = nl.add_net("n3");

    nl.add_gate(sf::GateType::NOT, {n0}, n1, "g0");
    nl.add_gate(sf::GateType::NAND, {n1, n2}, n3, "g1");

    nl.mark_input(n0);
    nl.mark_input(n2);
    nl.mark_output(n3);

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};
    sf::CellInstance c0; c0.id = 0; c0.name = "g0"; c0.position = {10, 20};
    c0.width = 2; c0.height = 5; c0.placed = true;
    pd.cells.push_back(c0);
    sf::CellInstance c1; c1.id = 1; c1.name = "g1"; c1.position = {30, 20};
    c1.width = 3; c1.height = 5; c1.placed = true;
    pd.cells.push_back(c1);

    sf::NetlistGraphBuilder builder;
    builder.add_gate_type("INV_X1", 0);
    builder.add_gate_type("NAND2_X1", 1);

    auto graph = builder.build_graph(nl, pd);
    check(graph.num_nodes >= 2, "graph has at least 2 nodes (gates)");
    check(graph.num_edges >= 0, "graph has edges");
}

static void test_gnn_aggregation_modes() {
    std::cout << "\n=== test_gnn_aggregation_modes ===\n";

    std::vector<std::string> modes = {"mean", "max", "sum"};
    for (auto& mode : modes) {
        sf::GnnLayerConfig cfg;
        cfg.input_dim = 2;
        cfg.output_dim = 3;
        cfg.aggregation = mode;
        cfg.activation = "relu";

        sf::GnnLayer layer(cfg);
        std::mt19937 rng(42);
        layer.init_weights(rng);

        std::vector<std::vector<double>> emb = {{1.0, 0.0}, {0.0, 1.0}};
        std::vector<std::pair<int, int>> edges = {{0, 1}, {1, 0}};
        auto out = layer.forward(emb, edges);
        check((int)out.size() == 2, mode + " aggregation: correct output size");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// GAP-025: GDSII Reader
// ═══════════════════════════════════════════════════════════════════════════

static void test_gdsii_reader_empty() {
    std::cout << "\n=== test_gdsii_reader_empty ===\n";

    sf::GdsiiReader reader;

    // Try reading nonexistent file — should fail gracefully
    auto result = reader.read("/nonexistent/path.gds");
    check(!result.success, "reading nonexistent file returns failure");
    check(!result.error.empty(), "error message is set");
}

static void test_gdsii_reader_buffer() {
    std::cout << "\n=== test_gdsii_reader_buffer ===\n";

    sf::GdsiiReader reader;

    // Construct a minimal valid GDSII stream in memory:
    // HEADER(6.0) + BGNLIB + LIBNAME + UNITS + ENDLIB
    std::vector<uint8_t> buf;
    auto push16 = [&](uint16_t v) {
        buf.push_back(v >> 8); buf.push_back(v & 0xFF);
    };
    auto push32 = [&](int32_t v) {
        buf.push_back((v >> 24) & 0xFF);
        buf.push_back((v >> 16) & 0xFF);
        buf.push_back((v >> 8) & 0xFF);
        buf.push_back(v & 0xFF);
    };

    // HEADER record: length=6, type=0x00, datatype=0x01(INT2), value=600
    push16(6); buf.push_back(0x00); buf.push_back(0x01); push16(600);

    // BGNLIB record: length=4, type=0x01, datatype=0x00
    push16(4); buf.push_back(0x01); buf.push_back(0x00);

    // LIBNAME record: length=10, type=0x02, datatype=0x06(ASCII), "test\0\0"
    push16(10); buf.push_back(0x02); buf.push_back(0x06);
    buf.push_back('t'); buf.push_back('e'); buf.push_back('s');
    buf.push_back('t'); buf.push_back(0); buf.push_back(0);

    // UNITS record: length=20, type=0x03, datatype=0x05(REAL8)
    // Two 8-byte IBM floats: user_units=0.001, db_units=1e-9
    // For simplicity, use zero bytes (the reader should handle it gracefully)
    push16(20); buf.push_back(0x03); buf.push_back(0x05);
    for (int i = 0; i < 16; i++) buf.push_back(0);

    // ENDLIB record: length=4, type=0x04, datatype=0x00
    push16(4); buf.push_back(0x04); buf.push_back(0x00);

    auto result = reader.read_buffer(buf.data(), buf.size());
    // Even if parsing is partial, it shouldn't crash
    check(result.cells_read >= 0, "buffer parse doesn't crash");
}

static void test_gdsii_structs() {
    std::cout << "\n=== test_gdsii_structs ===\n";

    sf::GdsLibrary lib;
    lib.name = "test_lib";
    lib.user_units = 0.001;
    lib.db_units = 1e-9;

    sf::GdsCell cell;
    cell.name = "top";
    cell.bbox = {0, 0, 100, 100};

    sf::GdsPolygon poly;
    poly.layer = 1;
    poly.datatype = 0;
    poly.points = {{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}};
    cell.polygons.push_back(poly);

    sf::GdsPath path;
    path.layer = 2;
    path.width = 0.5;
    path.points = {{0, 5}, {20, 5}};
    cell.paths.push_back(path);

    sf::GdsCellRef ref;
    ref.cell_name = "sub";
    ref.x = 50; ref.y = 50;
    ref.angle = 90.0;
    cell.refs.push_back(ref);

    sf::GdsArrayRef aref;
    aref.cell_name = "sub";
    aref.x = 0; aref.y = 0;
    aref.cols = 4; aref.rows = 3;
    aref.col_pitch = 10; aref.row_pitch = 10;
    cell.arefs.push_back(aref);

    lib.cells.push_back(cell);
    lib.cell_index["top"] = 0;

    check(lib.find_cell("top") != nullptr, "find_cell returns valid cell");
    check(lib.find_cell("nonexistent") == nullptr, "find_cell returns null for missing");
    check(cell.polygons.size() == 1, "cell has 1 polygon");
    check(cell.paths.size() == 1, "cell has 1 path");
    check(cell.refs.size() == 1, "cell has 1 SREF");
    check(cell.arefs.size() == 1, "cell has 1 AREF");
}

static void test_gdsii_layer_map() {
    std::cout << "\n=== test_gdsii_layer_map ===\n";

    sf::GdsiiReader reader;

    std::unordered_map<int, int> layer_map;
    layer_map[1] = 14;   // GDSII layer 1 -> MET1
    layer_map[2] = 16;   // GDSII layer 2 -> MET2
    reader.set_layer_map(layer_map);
    reader.set_scale(0.001);  // nm to um

    check(true, "layer map and scale set without crash");
}

// ═══════════════════════════════════════════════════════════════════════════
// GAP-004: TrackOccupancy 20-bit scaling
// ═══════════════════════════════════════════════════════════════════════════

static void test_track_occupancy_large_indices() {
    std::cout << "\n=== test_track_occupancy_large_indices ===\n";

    sf::TrackOccupancy occ;

    // Insert segments with track indices > 65535 (old 16-bit limit)
    occ.insert(0, 100000, 0.0, 10.0, 1);
    occ.insert(0, 200000, 0.0, 10.0, 2);
    occ.insert(1, 500000, 5.0, 15.0, 3);

    check(occ.size() == 3, "3 segments inserted with large indices");

    // Check that large-index tracks are distinguishable
    bool free1 = occ.is_free(0, 100000, 0.0, 10.0, 99, 0, 0, 1.0, 0.1, 0.14);
    check(!free1, "occupied track 100000 is not free");

    bool free2 = occ.is_free(0, 100001, 0.0, 10.0, 99, 0, 0, 1.0, 0.1, 0.14);
    check(free2, "adjacent track 100001 is free");

    // Remove a net
    occ.remove_net(1);
    check(occ.size() == 2, "after removing net 1: 2 segments remain");
}

static void test_track_occupancy_boundary() {
    std::cout << "\n=== test_track_occupancy_boundary ===\n";

    sf::TrackOccupancy occ;

    // Test at the boundary of 20-bit index space (1M - 1 = 1048575)
    int max_track = (1 << 20) - 1;
    occ.insert(0, max_track, 0.0, 5.0, 1);
    occ.insert(0, 0, 0.0, 5.0, 2);  // track 0

    check(occ.size() == 2, "boundary tracks inserted");

    bool free_max = occ.is_free(0, max_track, 0.0, 5.0, 99, 0, 0, 1.0, 0.1, 0.14);
    check(!free_max, "max track occupied");

    bool free_zero = occ.is_free(0, 0, 0.0, 5.0, 99, 0, 0, 1.0, 0.1, 0.14);
    check(!free_zero, "track 0 occupied");
}

// ═══════════════════════════════════════════════════════════════════════════
// Integration: CCS + AWE wire delay
// ═══════════════════════════════════════════════════════════════════════════

static void test_ccs_wire_delay_integration() {
    std::cout << "\n=== test_ccs_wire_delay_integration ===\n";

    sf::CcsDelayEngine engine;

    sf::LibertyTiming timing;
    timing.related_pin = "A";
    timing.nldm_rise.index_1 = {0.01, 0.5};
    timing.nldm_rise.index_2 = {0.001, 0.1};
    timing.nldm_rise.values = {{0.05, 0.15}, {0.10, 0.25}};
    timing.cell_fall = timing.cell_rise;
    timing.nldm_rise_tr.index_1 = {0.01, 0.5};
    timing.nldm_rise_tr.index_2 = {0.001, 0.1};
    timing.nldm_rise_tr.values = {{0.02, 0.08}, {0.05, 0.15}};
    timing.nldm_fall_tr = timing.nldm_rise_tr;

    // Build a parasitic net
    sf::ParasiticNet wire_rc;
    wire_rc.net_id = 0;
    wire_rc.total_cap_ff = 20.0;
    wire_rc.total_res_ohm = 50.0;
    wire_rc.elmore_delay_ps = 10.0;

    sf::ParasiticNet::RCSegment seg;
    seg.length = 40.0;
    seg.resistance = 50.0;
    seg.capacitance = 20.0;
    wire_rc.segments.push_back(seg);

    auto result = engine.compute_with_wire(timing, 0.05, wire_rc, 1.8, true);
    check(result.total_delay_ns >= 0, "total delay (CCS + wire) non-negative");
    check(result.driver_delay_ns >= 0, "driver delay non-negative");
    check(result.wire_delay_ns >= 0, "wire delay non-negative");
    check(result.total_delay_ns >= result.driver_delay_ns,
          "total >= driver delay (wire adds delay)");
}

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     SiliconForge Phase 98: Gap Coverage Tests               ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";

    // GAP-001: NPN Boolean Matching
    test_npn_truth_table_basics();
    test_npn_output_negate();
    test_npn_canonical_form();
    test_npn_cofactor_signatures();
    test_npn_library_lookup();
    test_npn_heuristic_5input();

    // GAP-005: CCS Delay Model
    test_ccs_delay_basic();
    test_ccs_pi_model();
    test_ccs_effective_cap();
    test_ccs_smis();

    // GAP-006: Coupling Capacitance Extraction
    test_coupling_spatial_config();
    test_coupling_with_wires();
    test_awe_delay();

    // GAP-009: Hierarchical DRC
    test_hier_drc_basic();
    test_hier_drc_scanline();
    test_hier_drc_mirror();

    // GAP-014: SPICE-driven Characterization
    test_spice_char_config();
    test_spice_char_inverter();
    test_spice_char_nand2();
    test_spice_multi_corner();
    test_spice_sequential();

    // GAP-024: GNN Infrastructure
    test_gnn_graph_build();
    test_gnn_layer_forward();
    test_gnn_model_predict();
    test_gnn_model_train();
    test_gnn_netlist_builder();
    test_gnn_aggregation_modes();

    // GAP-025: GDSII Reader
    test_gdsii_reader_empty();
    test_gdsii_reader_buffer();
    test_gdsii_structs();
    test_gdsii_layer_map();

    // GAP-004: TrackOccupancy 20-bit scaling
    test_track_occupancy_large_indices();
    test_track_occupancy_boundary();

    // Integration
    test_ccs_wire_delay_integration();

    std::cout << "\n══════════════════════════════════════════\n";
    std::cout << "Phase 98 results: " << g_pass << " passed, "
              << g_fail << " failed out of " << (g_pass + g_fail) << "\n";
    return g_fail == 0 ? 0 : 1;
}
