// SiliconForge — Phase 45: Tier 0 Foundation Tests
// 35 tests: Liberty NLDM tables, bilinear interpolation, internal power,
// leakage power, DEF sections (PINS, BLOCKAGES, TRACKS, VIAS, REGIONS,
// SPECIALNETS, GROUPS), cell_name_map, hierarchy (instance tree, block
// abstraction, parameterized modules, selective flattening)

#include "core/liberty_parser.hpp"
#include "core/def_parser.hpp"
#include "core/hierarchy.hpp"
#include "pnr/physical.hpp"
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>

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

// ── Minimal Liberty content with NLDM tables ──
static const char* LIBERTY_NLDM = R"(
library (test_lib) {
  technology (cmos);
  nom_voltage : 1.1;
  nom_temperature : 25.0;
  time_unit : "1ns";
  capacitive_load_unit (1, pf);

  lu_table_template (delay_2x2) {
    variable_1 : input_net_transition;
    variable_2 : total_output_net_capacitance;
    index_1 ("0.01, 0.1");
    index_2 ("0.005, 0.05");
  }

  cell (INV_X1) {
    area : 1.5;
    cell_leakage_power : 0.003;

    leakage_power () {
      when : "A";
      value : 0.002;
    }
    leakage_power () {
      when : "!A";
      value : 0.004;
    }

    pin (A) {
      direction : input;
      capacitance : 0.002;
      max_transition : 0.5;
    }
    pin (Y) {
      direction : output;
      function : "!A";
      timing () {
        related_pin : "A";
        timing_type : combinational;
        cell_rise (delay_2x2) {
          values ("0.05, 0.15", \
                  "0.08, 0.22");
        }
        cell_fall (delay_2x2) {
          values ("0.04, 0.12", \
                  "0.07, 0.19");
        }
        rise_transition (delay_2x2) {
          values ("0.02, 0.06", \
                  "0.03, 0.09");
        }
        fall_transition (delay_2x2) {
          values ("0.015, 0.05", \
                  "0.025, 0.08");
        }
      }
      internal_power () {
        related_pin : "A";
        rise_power (delay_2x2) {
          values ("0.001, 0.003", \
                  "0.002, 0.005");
        }
        fall_power (delay_2x2) {
          values ("0.0008, 0.0025", \
                  "0.0015, 0.004");
        }
      }
    }
  }

  cell (AND2_X1) {
    area : 2.0;
    pin (A) {
      direction : input;
      capacitance : 0.001;
    }
    pin (B) {
      direction : input;
      capacitance : 0.001;
    }
    pin (Y) {
      direction : output;
      function : "A & B";
      timing () {
        related_pin : "A";
        timing_type : combinational;
        cell_rise (delay_2x2) {
          values ("0.06, 0.18", \
                  "0.10, 0.28");
        }
        cell_fall (delay_2x2) {
          values ("0.05, 0.14", \
                  "0.08, 0.22");
        }
        rise_transition (delay_2x2) {
          values ("0.025, 0.07", \
                  "0.04, 0.11");
        }
        fall_transition (delay_2x2) {
          values ("0.02, 0.06", \
                  "0.035, 0.09");
        }
      }
    }
  }
}
)";

// ── Minimal DEF with all new sections ──
static const char* DEF_FULL = R"(
VERSION 5.8 ;
DESIGN test_chip ;
UNITS DISTANCE MICRONS 2000 ;
DIEAREA ( 0 0 ) ( 200000 200000 ) ;

TRACKS X 0 DO 100 STEP 2000 LAYER metal1 ;
TRACKS Y 0 DO 100 STEP 2000 LAYER metal2 ;

VIAS 1 ;
  - via1_std
    + RECT metal1 ( -50 -50 ) ( 50 50 )
    + RECT via1 ( -30 -30 ) ( 30 30 )
    + RECT metal2 ( -50 -50 ) ( 50 50 )
  ;
END VIAS

COMPONENTS 3 ;
  - u0 INV_X1 + PLACED ( 10000 20000 ) N ;
  - u1 AND2_X1 + PLACED ( 30000 20000 ) N ;
  - u2 INV_X1 + PLACED ( 50000 20000 ) S ;
END COMPONENTS

PINS 2 ;
  - clk + NET clk_net + DIRECTION INPUT
    + LAYER metal1 ( 0 0 ) ( 100 100 )
    + PLACED ( 0 50000 ) N ;
  - data_out + NET out_net + DIRECTION OUTPUT
    + PLACED ( 200000 50000 ) N ;
END PINS

BLOCKAGES 2 ;
  - LAYER metal1
    RECT ( 80000 80000 ) ( 120000 120000 ) ;
  - PLACEMENT
    RECT ( 140000 0 ) ( 160000 40000 ) ;
END BLOCKAGES

SPECIALNETS 2 ;
  - VDD ( * vdd ) + USE POWER
    + ROUTED metal1 2000 ( 0 0 ) ( 200000 0 )
  ;
  - VSS ( * vss ) + USE GROUND
    + ROUTED metal1 2000 ( 0 200000 ) ( 200000 200000 )
  ;
END SPECIALNETS

NETS 2 ;
  - net0 ( u0 Y ) ( u1 A ) ;
  - net1 ( u1 Y ) ( u2 A ) ;
END NETS

REGIONS 1 ;
  - core_region ( 10000 10000 ) ( 190000 190000 ) + TYPE FENCE ;
END REGIONS

GROUPS 1 ;
  - logic_group u0 u1 u2 + REGION core_region ;
END GROUPS

END DESIGN
)";

// ═══════════════════════════════════════════════════════
// Liberty NLDM Tests
// ═══════════════════════════════════════════════════════

void test_liberty_parse_cells() {
    RUN("Liberty: parse cells");
    sf::LibertyLibrary lib;
    CHECK(lib.parse_string(LIBERTY_NLDM));
    CHECK(lib.cells.size() == 2);
    CHECK(lib.find_cell("INV_X1") != nullptr);
    CHECK(lib.find_cell("AND2_X1") != nullptr);
    CHECK(lib.find_cell("NONEXIST") == nullptr);
    PASS();
}

void test_liberty_cell_properties() {
    RUN("Liberty: cell area and leakage");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto* inv = lib.find_cell("INV_X1");
    CHECK(inv != nullptr);
    CHECK(std::abs(inv->area - 1.5) < 1e-6);
    CHECK(std::abs(inv->leakage_power - 0.003) < 1e-6);
    auto* and2 = lib.find_cell("AND2_X1");
    CHECK(and2 != nullptr);
    CHECK(std::abs(and2->area - 2.0) < 1e-6);
    PASS();
}

void test_liberty_pins() {
    RUN("Liberty: pin parsing");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto* inv = lib.find_cell("INV_X1");
    CHECK(inv->pins.size() == 2);
    auto* pinA = inv->find_pin("A");
    CHECK(pinA != nullptr);
    CHECK(pinA->direction == "input");
    CHECK(std::abs(pinA->capacitance - 0.002) < 1e-6);
    auto* pinY = inv->find_pin("Y");
    CHECK(pinY != nullptr);
    CHECK(pinY->direction == "output");
    CHECK(pinY->function == "!A");
    PASS();
}

void test_liberty_num_inputs() {
    RUN("Liberty: num_inputs()");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    CHECK(lib.find_cell("INV_X1")->num_inputs() == 1);
    CHECK(lib.find_cell("AND2_X1")->num_inputs() == 2);
    PASS();
}

void test_liberty_lu_table_template() {
    RUN("Liberty: lu_table_template");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    CHECK(lib.table_templates.count("delay_2x2") == 1);
    auto& tmpl = lib.table_templates["delay_2x2"];
    CHECK(tmpl.index_1.size() == 2);
    CHECK(tmpl.index_2.size() == 2);
    CHECK(std::abs(tmpl.index_1[0] - 0.01) < 1e-6);
    CHECK(std::abs(tmpl.index_1[1] - 0.1) < 1e-6);
    CHECK(std::abs(tmpl.index_2[0] - 0.005) < 1e-6);
    CHECK(std::abs(tmpl.index_2[1] - 0.05) < 1e-6);
    PASS();
}

void test_liberty_nldm_tables() {
    RUN("Liberty: NLDM timing tables");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto* inv = lib.find_cell("INV_X1");
    CHECK(!inv->timings.empty());
    auto& t = inv->timings[0];
    CHECK(t.related_pin == "A");
    CHECK(t.timing_type == "combinational");
    // cell_rise table
    CHECK(t.nldm_rise.valid());
    CHECK(t.nldm_rise.index_1.size() == 2);
    CHECK(t.nldm_rise.index_2.size() == 2);
    CHECK(t.nldm_rise.values.size() == 2);
    CHECK(t.nldm_rise.values[0].size() == 2);
    CHECK(std::abs(t.nldm_rise.values[0][0] - 0.05) < 1e-6);
    CHECK(std::abs(t.nldm_rise.values[1][1] - 0.22) < 1e-6);
    // cell_fall table
    CHECK(t.nldm_fall.valid());
    CHECK(std::abs(t.nldm_fall.values[0][0] - 0.04) < 1e-6);
    CHECK(std::abs(t.nldm_fall.values[1][1] - 0.19) < 1e-6);
    // rise_transition
    CHECK(t.nldm_rise_tr.valid());
    CHECK(std::abs(t.nldm_rise_tr.values[0][0] - 0.02) < 1e-6);
    // fall_transition
    CHECK(t.nldm_fall_tr.valid());
    CHECK(std::abs(t.nldm_fall_tr.values[0][0] - 0.015) < 1e-6);
    PASS();
}

void test_liberty_nldm_interpolation_corner() {
    RUN("Liberty: NLDM interpolation at corners");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto& t = lib.find_cell("INV_X1")->timings[0];
    // Exact corner: slew=0.01, load=0.005 → 0.05
    double v = t.nldm_rise.interpolate(0.01, 0.005);
    CHECK(std::abs(v - 0.05) < 1e-6);
    // Exact corner: slew=0.1, load=0.05 → 0.22
    v = t.nldm_rise.interpolate(0.1, 0.05);
    CHECK(std::abs(v - 0.22) < 1e-6);
    PASS();
}

void test_liberty_nldm_interpolation_midpoint() {
    RUN("Liberty: NLDM bilinear interpolation midpoint");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto& t = lib.find_cell("INV_X1")->timings[0];
    // Midpoint: slew=0.055, load=0.0275 → bilinear average of 4 corners
    // Expected: (0.05 + 0.15 + 0.08 + 0.22) / 4 = 0.125
    double v = t.nldm_rise.interpolate(0.055, 0.0275);
    CHECK(std::abs(v - 0.125) < 1e-3);
    PASS();
}

void test_liberty_nldm_interpolation_clamp() {
    RUN("Liberty: NLDM interpolation clamp (out of range)");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto& t = lib.find_cell("INV_X1")->timings[0];
    // Below range clamps to corner
    double v = t.nldm_rise.interpolate(0.001, 0.001);
    CHECK(std::abs(v - 0.05) < 1e-3);
    // Above range clamps to opposite corner
    v = t.nldm_rise.interpolate(1.0, 1.0);
    CHECK(std::abs(v - 0.22) < 1e-3);
    PASS();
}

void test_liberty_internal_power() {
    RUN("Liberty: internal_power parsing");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto* inv = lib.find_cell("INV_X1");
    CHECK(!inv->internal_powers.empty());
    auto& pw = inv->internal_powers[0];
    CHECK(pw.related_pin == "A");
    CHECK(pw.power_rise_table.valid());
    CHECK(pw.power_fall_table.valid());
    // Corner value for rise_power
    CHECK(std::abs(pw.power_rise_table.values[0][0] - 0.001) < 1e-6);
    PASS();
}

void test_liberty_internal_power_at() {
    RUN("Liberty: internal_power_at() lookup");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto* inv = lib.find_cell("INV_X1");
    // At corner (0.01, 0.005): rise=0.001, fall=0.0008, avg=0.0009
    double p = inv->internal_power_at(0.01, 0.005);
    CHECK(std::abs(p - 0.0009) < 1e-4);
    PASS();
}

void test_liberty_leakage_power() {
    RUN("Liberty: leakage_power per-vector");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto* inv = lib.find_cell("INV_X1");
    CHECK(inv->leakage_powers.size() == 2);
    CHECK(inv->leakage_powers[0].when == "A");
    CHECK(std::abs(inv->leakage_powers[0].value - 0.002) < 1e-6);
    CHECK(inv->leakage_powers[1].when == "!A");
    CHECK(std::abs(inv->leakage_powers[1].value - 0.004) < 1e-6);
    PASS();
}

void test_liberty_and2_timing() {
    RUN("Liberty: AND2 timing tables");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto* and2 = lib.find_cell("AND2_X1");
    CHECK(!and2->timings.empty());
    auto& t = and2->timings[0];
    CHECK(t.nldm_rise.valid());
    CHECK(std::abs(t.nldm_rise.values[0][0] - 0.06) < 1e-6);
    CHECK(std::abs(t.nldm_rise.values[1][1] - 0.28) < 1e-6);
    PASS();
}

void test_liberty_cells_by_function() {
    RUN("Liberty: cells_by_function");
    sf::LibertyLibrary lib;
    lib.parse_string(LIBERTY_NLDM);
    auto inv_cells = lib.cells_by_function("!A");
    CHECK(inv_cells.size() == 1);
    CHECK(inv_cells[0]->name == "INV_X1");
    auto and_cells = lib.cells_by_function("A & B");
    CHECK(and_cells.size() == 1);
    CHECK(and_cells[0]->name == "AND2_X1");
    PASS();
}

// ═══════════════════════════════════════════════════════
// DEF Parser Tests
// ═══════════════════════════════════════════════════════

void test_def_parse_basic() {
    RUN("DEF: basic parse and die area");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    CHECK(parser.parse_string(DEF_FULL, pd));
    // DBU = 2000, die = (200000,200000) → (100, 100) µm
    CHECK(std::abs(pd.dbu_per_micron - 2000.0) < 1e-6);
    CHECK(std::abs(pd.die_area.x1 - 100.0) < 0.1);
    CHECK(std::abs(pd.die_area.y1 - 100.0) < 0.1);
    PASS();
}

void test_def_components() {
    RUN("DEF: COMPONENTS parsing");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    parser.parse_string(DEF_FULL, pd);
    CHECK(pd.cells.size() == 3);
    CHECK(pd.cells[0].name == "u0");
    CHECK(pd.cells[0].cell_type == "INV_X1");
    CHECK(pd.cells[0].placed);
    // u0 at (10000/2000, 20000/2000) = (5, 10)
    CHECK(std::abs(pd.cells[0].position.x - 5.0) < 0.1);
    CHECK(std::abs(pd.cells[0].position.y - 10.0) < 0.1);
    CHECK(pd.cells[2].name == "u2");
    CHECK(pd.cells[2].orientation == 1); // S
    PASS();
}

void test_def_cell_name_map() {
    RUN("DEF: cell_name_map O(1) lookup");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    parser.parse_string(DEF_FULL, pd);
    CHECK(pd.cell_name_map.count("u0") == 1);
    CHECK(pd.cell_name_map.count("u1") == 1);
    CHECK(pd.cell_name_map.count("u2") == 1);
    int idx = pd.cell_name_map["u1"];
    CHECK(pd.cells[idx].name == "u1");
    CHECK(pd.cells[idx].cell_type == "AND2_X1");
    PASS();
}

void test_def_nets() {
    RUN("DEF: NETS parsing with PIN refs");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    parser.parse_string(DEF_FULL, pd);
    CHECK(pd.nets.size() >= 2);
    // net0 connects u0 and u1
    CHECK(pd.nets[0].name == "net0");
    CHECK(pd.nets[0].cell_ids.size() == 2);
    PASS();
}

void test_def_pins() {
    RUN("DEF: PINS section");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    parser.parse_string(DEF_FULL, pd);
    CHECK(pd.io_pins.size() == 2);
    CHECK(pd.io_pins[0].name == "clk");
    CHECK(pd.io_pins[0].direction == "INPUT");
    CHECK(pd.io_pins[0].net_name == "clk_net");
    CHECK(pd.io_pins[0].placed);
    CHECK(pd.io_pins[1].name == "data_out");
    CHECK(pd.io_pins[1].direction == "OUTPUT");
    PASS();
}

void test_def_blockages() {
    RUN("DEF: BLOCKAGES section");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    parser.parse_string(DEF_FULL, pd);
    CHECK(pd.blockages.size() == 2);
    // First: routing blockage on metal1
    CHECK(pd.blockages[0].type == "ROUTING");
    // Second: placement blockage
    CHECK(pd.blockages[1].type == "PLACEMENT");
    PASS();
}

void test_def_tracks() {
    RUN("DEF: TRACKS section");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    parser.parse_string(DEF_FULL, pd);
    CHECK(pd.tracks.size() == 2);
    CHECK(pd.tracks[0].direction == "X");
    CHECK(pd.tracks[0].count == 100);
    CHECK(pd.tracks[0].layer_names.size() >= 1);
    CHECK(pd.tracks[0].layer_names[0] == "metal1");
    CHECK(pd.tracks[1].direction == "Y");
    CHECK(pd.tracks[1].layer_names[0] == "metal2");
    PASS();
}

void test_def_vias() {
    RUN("DEF: VIAS section");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    parser.parse_string(DEF_FULL, pd);
    CHECK(pd.via_defs.size() == 1);
    CHECK(pd.via_defs[0].name == "via1_std");
    CHECK(pd.via_defs[0].layers.size() == 3);
    PASS();
}

void test_def_special_nets() {
    RUN("DEF: SPECIALNETS section");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    parser.parse_string(DEF_FULL, pd);
    CHECK(pd.special_nets.size() == 2);
    CHECK(pd.special_nets[0].name == "VDD");
    CHECK(pd.special_nets[0].use == "POWER");
    CHECK(pd.special_nets[1].name == "VSS");
    CHECK(pd.special_nets[1].use == "GROUND");
    PASS();
}

void test_def_regions() {
    RUN("DEF: REGIONS section");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    parser.parse_string(DEF_FULL, pd);
    CHECK(pd.regions.size() == 1);
    CHECK(pd.regions[0].name == "core_region");
    CHECK(pd.regions[0].type == "FENCE");
    CHECK(!pd.regions[0].rects.empty());
    PASS();
}

void test_def_groups() {
    RUN("DEF: GROUPS section");
    sf::PhysicalDesign pd;
    sf::DefParser parser;
    parser.parse_string(DEF_FULL, pd);
    CHECK(pd.groups.size() == 1);
    CHECK(pd.groups[0].name == "logic_group");
    CHECK(pd.groups[0].region_name == "core_region");
    CHECK(pd.groups[0].members.size() >= 3);
    PASS();
}

// ═══════════════════════════════════════════════════════
// Hierarchy Tests
// ═══════════════════════════════════════════════════════

// Helper: make a small netlist with some gates
static sf::Netlist make_small_netlist(int num_gates = 4) {
    sf::Netlist nl;
    sf::NetId a = nl.add_net("a"); nl.mark_input(a);
    sf::NetId b = nl.add_net("b"); nl.mark_input(b);
    sf::NetId w1 = nl.add_net("w1");
    sf::NetId w2 = nl.add_net("w2");
    sf::NetId out = nl.add_net("out"); nl.mark_output(out);
    nl.add_gate(sf::GateType::AND, {a, b}, w1, "g0");
    nl.add_gate(sf::GateType::NOT, {w1}, w2, "g1");
    if (num_gates > 2) nl.add_gate(sf::GateType::OR, {w1, w2}, out, "g2");
    return nl;
}

void test_hier_instance_tree() {
    RUN("Hierarchy: instance tree tracking");
    sf::HierarchyManager hm;
    auto nl_alu = make_small_netlist();
    auto nl_reg = make_small_netlist(2);
    sf::Netlist nl_top;
    nl_top.add_net("clk"); nl_top.mark_input(0);

    hm.add_module("alu", nl_alu);
    hm.add_module("regfile", nl_reg);
    hm.add_module("top", nl_top);

    hm.instantiate("top", "alu", "alu_inst0");
    hm.instantiate("top", "alu", "alu_inst1");
    hm.instantiate("top", "regfile", "reg_inst");

    auto* top = hm.get_module("top");
    CHECK(top->instances.size() == 3);
    CHECK(top->instances[0].instance_name == "alu_inst0");
    CHECK(top->instances[0].module_name == "alu");
    CHECK(top->instances[1].instance_name == "alu_inst1");
    CHECK(top->instances[2].instance_name == "reg_inst");
    PASS();
}

void test_hier_instance_count() {
    RUN("Hierarchy: instance_count query");
    sf::HierarchyManager hm;
    auto nl = make_small_netlist();
    hm.add_module("adder", nl);
    hm.add_module("top", nl);
    hm.instantiate("top", "adder", "add0");
    hm.instantiate("top", "adder", "add1");
    hm.instantiate("top", "adder", "add2");

    CHECK(hm.instance_count("adder") == 3);
    auto insts = hm.get_instances_of("adder");
    CHECK(insts.size() == 3);
    PASS();
}

void test_hier_ports() {
    RUN("Hierarchy: module port definitions");
    sf::HierarchyManager hm;
    auto nl = make_small_netlist();
    hm.add_module("alu", nl);

    std::vector<sf::ModulePort> ports = {
        {"clk", sf::ModulePort::IN, 1, -1},
        {"a", sf::ModulePort::IN, 8, -1},
        {"b", sf::ModulePort::IN, 8, -1},
        {"result", sf::ModulePort::OUT, 8, -1}
    };
    hm.define_ports("alu", ports);

    auto* mod = hm.get_module("alu");
    CHECK(mod->ports.size() == 4);
    CHECK(mod->ports[0].name == "clk");
    CHECK(mod->ports[0].direction == sf::ModulePort::IN);
    CHECK(mod->ports[3].name == "result");
    CHECK(mod->ports[3].width == 8);
    PASS();
}

void test_hier_block_model() {
    RUN("Hierarchy: block abstraction timing arcs");
    sf::HierarchyManager hm;
    auto nl = make_small_netlist();
    hm.add_module("mac", nl);

    std::vector<sf::ModulePort> ports = {
        {"a", sf::ModulePort::IN, 16, -1},
        {"b", sf::ModulePort::IN, 16, -1},
        {"out", sf::ModulePort::OUT, 32, -1}
    };
    hm.define_ports("mac", ports);
    hm.add_timing_arc("mac", "a", "out", 500.0, 50.0);
    hm.add_timing_arc("mac", "b", "out", 450.0, 45.0);
    hm.set_block_power("mac", 0.5, 0.01);

    auto model = hm.generate_block_model("mac");
    CHECK(model.valid);
    CHECK(model.timing_arcs.size() == 2);
    CHECK(std::abs(model.timing_arcs[0].delay_ps - 500.0) < 1e-6);
    CHECK(std::abs(model.timing_arcs[1].delay_ps - 450.0) < 1e-6);
    CHECK(std::abs(model.static_power_mw - 0.5) < 1e-6);
    PASS();
}

void test_hier_parameterized() {
    RUN("Hierarchy: parameterized instantiation");
    sf::HierarchyManager hm;
    auto nl = make_small_netlist();
    hm.add_module("sram", nl);
    hm.define_params("sram", {
        {"WIDTH", sf::ParamValue(32)},
        {"DEPTH", sf::ParamValue(256)}
    });
    hm.add_module("top", nl);

    auto& inst = hm.instantiate_with_params("top", "sram", "ram_4k", {
        {"DEPTH", sf::ParamValue(4096)}  // override depth
    });

    CHECK(inst.instance_name == "ram_4k");
    CHECK(inst.module_name == "sram");
    // DEPTH overridden to 4096
    CHECK(inst.params["DEPTH"].int_val == 4096);
    // WIDTH inherited from default
    CHECK(inst.params["WIDTH"].int_val == 32);
    PASS();
}

void test_hier_port_connections() {
    RUN("Hierarchy: port connections");
    sf::HierarchyManager hm;
    auto nl = make_small_netlist();
    hm.add_module("sub", nl);
    hm.add_module("top", nl);
    hm.instantiate("top", "sub", "sub_inst");

    hm.connect_port("top", "sub_inst", "a", "top_a");
    hm.connect_port("top", "sub_inst", "out", "top_out");

    auto* top = hm.get_module("top");
    CHECK(top->instances[0].port_map.count("a") == 1);
    CHECK(top->instances[0].port_map["a"] == "top_a");
    CHECK(top->instances[0].port_map["out"] == "top_out");
    PASS();
}

void test_hier_analyze_instances() {
    RUN("Hierarchy: analyze with instance counting");
    sf::HierarchyManager hm;
    auto nl = make_small_netlist();
    hm.add_module("leaf", nl);
    hm.add_module("mid", nl);
    hm.add_module("top", nl);
    hm.instantiate("top", "mid", "m0");
    hm.instantiate("mid", "leaf", "l0");
    hm.instantiate("mid", "leaf", "l1");

    auto r = hm.analyze();
    CHECK(r.total_modules == 3);
    CHECK(r.leaf_modules == 1);
    CHECK(r.hierarchy_depth >= 2);
    CHECK(r.total_instances == 3); // m0, l0, l1
    PASS();
}

void test_hier_selective_flatten() {
    RUN("Hierarchy: selective flattening");
    sf::HierarchyManager hm;
    auto nl = make_small_netlist();
    hm.add_module("leaf_a", nl);
    hm.add_module("leaf_b", nl);
    hm.add_module("mid", nl);
    hm.add_module("top", nl);
    hm.instantiate("top", "mid");
    hm.instantiate("top", "leaf_a");
    hm.instantiate("mid", "leaf_b");

    // Flatten with depth limit = 1 (only flatten top's immediate children)
    sf::FlattenOptions opts;
    opts.policy = sf::FlattenPolicy::DEPTH_LIMITED;
    opts.max_depth = 1;
    auto flat = hm.flatten("top", opts);
    CHECK(flat.num_gates() > 0);
    PASS();
}

void test_hier_preserve_modules() {
    RUN("Hierarchy: preserve modules during flatten");
    sf::HierarchyManager hm;
    auto nl = make_small_netlist();
    hm.add_module("hardmacro", nl);
    hm.add_module("top", nl);
    hm.instantiate("top", "hardmacro");

    sf::FlattenOptions opts;
    opts.policy = sf::FlattenPolicy::SELECTIVE;
    opts.preserve_modules = {"hardmacro"};
    auto flat = hm.flatten("top", opts);
    // Flattened should only contain top's gates, not hardmacro's
    CHECK(flat.num_gates() >= 2); // top's own gates
    PASS();
}

void test_hier_tree_visualization() {
    RUN("Hierarchy: tree visualization");
    sf::HierarchyManager hm;
    auto nl = make_small_netlist();
    hm.add_module("leaf", nl);
    hm.add_module("mid", nl);
    hm.add_module("top", nl);
    hm.instantiate("top", "mid");
    hm.instantiate("mid", "leaf");

    std::string tree = hm.hierarchy_tree("top");
    CHECK(!tree.empty());
    CHECK(tree.find("top") != std::string::npos);
    CHECK(tree.find("mid") != std::string::npos);
    CHECK(tree.find("leaf") != std::string::npos);
    PASS();
}

// ═══════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════

int main() {
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << "  SiliconForge — Phase 45: Tier 0 Foundation Tests\n";
    std::cout << "═══════════════════════════════════════════════════════════\n\n";

    // Liberty NLDM tests
    std::cout << "── Liberty Parser / NLDM Tables ──\n";
    test_liberty_parse_cells();
    test_liberty_cell_properties();
    test_liberty_pins();
    test_liberty_num_inputs();
    test_liberty_lu_table_template();
    test_liberty_nldm_tables();
    test_liberty_nldm_interpolation_corner();
    test_liberty_nldm_interpolation_midpoint();
    test_liberty_nldm_interpolation_clamp();
    test_liberty_internal_power();
    test_liberty_internal_power_at();
    test_liberty_leakage_power();
    test_liberty_and2_timing();
    test_liberty_cells_by_function();

    // DEF parser tests
    std::cout << "\n── DEF Parser / New Sections ──\n";
    test_def_parse_basic();
    test_def_components();
    test_def_cell_name_map();
    test_def_nets();
    test_def_pins();
    test_def_blockages();
    test_def_tracks();
    test_def_vias();
    test_def_special_nets();
    test_def_regions();
    test_def_groups();

    // Hierarchy tests
    std::cout << "\n── Hierarchical Design ──\n";
    test_hier_instance_tree();
    test_hier_instance_count();
    test_hier_ports();
    test_hier_block_model();
    test_hier_parameterized();
    test_hier_port_connections();
    test_hier_analyze_instances();
    test_hier_selective_flatten();
    test_hier_preserve_modules();
    test_hier_tree_visualization();

    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << "  Results: " << tests_passed << "/" << tests_run << " PASSED\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
