// Phase 93: Transistor-level standard cell layout generator.
// Validates technology rules, inverter/NAND/NOR/DFF/BUF/FILL/TIE cell
// layouts for correct layer coverage, pin presence, dimensional
// consistency, and drive-strength scaling.

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <cassert>

#include "stdcell/layout_rules.hpp"
#include "stdcell/cell_layout.hpp"
#include "verify/drc.hpp"

static int total = 0, passed = 0;
#define CHECK(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  [PASS] %s\n", msg); } \
    else { printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while(0)

// ═══════════════════════════════════════════════════════════════════════════
// TEST 1: SKY130 technology rules sanity
// ═══════════════════════════════════════════════════════════════════════════
static void test_tech_rules_sky130() {
    printf("\n-- SKY130 technology rules --\n");
    auto r = sf::TechRules::sky130_rules();

    CHECK(r.process_name == "SKY130", "process name");
    CHECK(r.gate_length > 0.1 && r.gate_length < 0.2,
          "gate length in 130nm range");
    CHECK(r.row_height > 2.0 && r.row_height < 4.0,
          "row height reasonable (2-4 um)");
    CHECK(r.site_width > 0.3 && r.site_width < 1.0,
          "site width reasonable");
    CHECK(r.vdd_voltage > 1.5 && r.vdd_voltage < 2.0,
          "VDD voltage ~1.8V");
    CHECK(r.min_poly_spacing > r.gate_length,
          "poly spacing > gate length");
    CHECK(r.contact_size > 0.0, "contact size positive");
    CHECK(r.nwell_enclosure > 0.0, "NWELL enclosure positive");
    CHECK(r.m1_width > 0.0, "M1 width positive");
    CHECK(r.power_rail_width > r.m1_width,
          "power rail wider than min M1");

    // ASAP7 quick sanity
    auto a7 = sf::TechRules::asap7_rules();
    CHECK(a7.process_name == "ASAP7", "ASAP7 process name");
    CHECK(a7.gate_length < 0.01, "ASAP7 gate length < 10nm");
    CHECK(a7.vdd_voltage < 1.0, "ASAP7 Vdd < 1V");
    CHECK(a7.row_height < 0.5, "ASAP7 row height < 0.5um");
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 2: INV_X1 layout completeness
// ═══════════════════════════════════════════════════════════════════════════
static void test_inv_layout() {
    printf("\n-- INV_X1 layout --\n");
    auto rules = sf::TechRules::sky130_rules();
    sf::CellLayoutGenerator gen(rules);
    auto inv = gen.generate_inv(1);

    CHECK(inv.cell_name == "INV_X1", "cell name");
    CHECK(inv.width > 0, "width > 0");
    CHECK(std::abs(inv.height - rules.row_height) < 1e-6,
          "height == row_height");

    // Layer coverage
    CHECK(inv.has_layer(sf::DrcLayer::NWELL), "has NWELL");
    CHECK(inv.has_layer(sf::DrcLayer::DIFF),  "has DIFF");
    CHECK(inv.has_layer(sf::DrcLayer::POLY),  "has POLY");
    CHECK(inv.has_layer(sf::DrcLayer::LICON), "has LICON");
    CHECK(inv.has_layer(sf::DrcLayer::LI1),   "has LI1");
    CHECK(inv.has_layer(sf::DrcLayer::MET1),  "has MET1");
    CHECK(inv.has_layer(sf::DrcLayer::NSDM),  "has NSDM");
    CHECK(inv.has_layer(sf::DrcLayer::PSDM),  "has PSDM");

    // Pin presence
    CHECK(inv.find_pin("A")   != nullptr, "has input pin A");
    CHECK(inv.find_pin("Y")   != nullptr, "has output pin Y");
    CHECK(inv.find_pin("VDD") != nullptr, "has VDD pin");
    CHECK(inv.find_pin("VSS") != nullptr, "has VSS pin");

    // Pin directions
    CHECK(inv.find_pin("A")->direction   == "INPUT",  "A is INPUT");
    CHECK(inv.find_pin("Y")->direction   == "OUTPUT", "Y is OUTPUT");
    CHECK(inv.find_pin("VDD")->direction == "POWER",  "VDD is POWER");
    CHECK(inv.find_pin("VSS")->direction == "GROUND", "VSS is GROUND");

    // NWELL covers upper half
    bool nwell_top = false;
    for (auto& r : inv.rects) {
        if (r.layer == sf::DrcLayer::NWELL) {
            nwell_top = r.bounds.y1 >= inv.height;
            break;
        }
    }
    CHECK(nwell_top, "NWELL extends to top of cell");

    // Poly crosses both active regions (at least 2 DIFF rects)
    CHECK(inv.count_layer(sf::DrcLayer::DIFF) >= 2,
          "at least 2 DIFF rects (NMOS + PMOS)");

    // Width snapped to site grid
    double sites = inv.width / rules.site_width;
    CHECK(std::abs(sites - std::round(sites)) < 1e-6,
          "width is integer multiple of site_width");

    printf("  INV_X1: %.3f x %.3f um, %zu rects, %zu pins\n",
           inv.width, inv.height, inv.rects.size(), inv.pins.size());
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 3: Drive strength scaling
// ═══════════════════════════════════════════════════════════════════════════
static void test_inv_drive_strengths() {
    printf("\n-- INV drive strength scaling --\n");
    auto rules = sf::TechRules::sky130_rules();
    sf::CellLayoutGenerator gen(rules);

    auto inv1 = gen.generate_inv(1);
    auto inv2 = gen.generate_inv(2);
    auto inv4 = gen.generate_inv(4);

    CHECK(inv2.width > inv1.width,
          "INV_X2 wider than INV_X1");
    CHECK(inv4.width > inv2.width,
          "INV_X4 wider than INV_X2");

    printf("  X1=%.3f  X2=%.3f  X4=%.3f um\n",
           inv1.width, inv2.width, inv4.width);
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 4: NAND2 layout
// ═══════════════════════════════════════════════════════════════════════════
static void test_nand2_layout() {
    printf("\n-- NAND2_X1 layout --\n");
    auto rules = sf::TechRules::sky130_rules();
    sf::CellLayoutGenerator gen(rules);
    auto nand = gen.generate_nand2(1);

    CHECK(nand.cell_name == "NAND2_X1", "cell name");
    CHECK(nand.width > 0, "width > 0");
    CHECK(std::abs(nand.height - rules.row_height) < 1e-6,
          "height == row_height");

    CHECK(nand.find_pin("A") != nullptr, "has pin A");
    CHECK(nand.find_pin("B") != nullptr, "has pin B");
    CHECK(nand.find_pin("Y") != nullptr, "has pin Y");
    CHECK(nand.find_pin("VDD") != nullptr, "has VDD");
    CHECK(nand.find_pin("VSS") != nullptr, "has VSS");

    CHECK(nand.has_layer(sf::DrcLayer::NWELL), "has NWELL");
    CHECK(nand.has_layer(sf::DrcLayer::DIFF),  "has DIFF");
    CHECK(nand.has_layer(sf::DrcLayer::POLY),  "has POLY");

    // NAND2 should be wider than INV (more transistors)
    auto inv = gen.generate_inv(1);
    CHECK(nand.width >= inv.width,
          "NAND2 width >= INV width");

    printf("  NAND2_X1: %.3f x %.3f um, %zu rects\n",
           nand.width, nand.height, nand.rects.size());
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 5: NOR2 layout
// ═══════════════════════════════════════════════════════════════════════════
static void test_nor2_layout() {
    printf("\n-- NOR2_X1 layout --\n");
    auto rules = sf::TechRules::sky130_rules();
    sf::CellLayoutGenerator gen(rules);
    auto nor = gen.generate_nor2(1);

    CHECK(nor.cell_name == "NOR2_X1", "cell name");
    CHECK(nor.width > 0, "width > 0");
    CHECK(std::abs(nor.height - rules.row_height) < 1e-6,
          "height == row_height");

    CHECK(nor.find_pin("A") != nullptr, "has pin A");
    CHECK(nor.find_pin("B") != nullptr, "has pin B");
    CHECK(nor.find_pin("Y") != nullptr, "has pin Y");

    CHECK(nor.has_layer(sf::DrcLayer::NWELL), "has NWELL");
    CHECK(nor.has_layer(sf::DrcLayer::POLY),  "has POLY");
    CHECK(nor.has_layer(sf::DrcLayer::DIFF),  "has DIFF");
    CHECK(nor.has_layer(sf::DrcLayer::NSDM),  "has NSDM");
    CHECK(nor.has_layer(sf::DrcLayer::PSDM),  "has PSDM");

    printf("  NOR2_X1: %.3f x %.3f um, %zu rects\n",
           nor.width, nor.height, nor.rects.size());
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 6: DFF layout
// ═══════════════════════════════════════════════════════════════════════════
static void test_dff_layout() {
    printf("\n-- DFF_X1 layout --\n");
    auto rules = sf::TechRules::sky130_rules();
    sf::CellLayoutGenerator gen(rules);
    auto dff = gen.generate_dff();

    CHECK(dff.cell_name == "DFF_X1", "cell name");
    CHECK(dff.width > 0, "width > 0");
    CHECK(std::abs(dff.height - rules.row_height) < 1e-6,
          "height == row_height");

    CHECK(dff.find_pin("D")   != nullptr, "has pin D");
    CHECK(dff.find_pin("CLK") != nullptr, "has pin CLK");
    CHECK(dff.find_pin("Q")   != nullptr, "has pin Q");
    CHECK(dff.find_pin("VDD") != nullptr, "has VDD");
    CHECK(dff.find_pin("VSS") != nullptr, "has VSS");

    // DFF should be significantly wider than INV (many more transistors)
    auto inv = gen.generate_inv(1);
    CHECK(dff.width > inv.width * 2.0,
          "DFF much wider than INV");

    printf("  DFF_X1: %.3f x %.3f um, %zu rects\n",
           dff.width, dff.height, dff.rects.size());
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 7: Fill cell
// ═══════════════════════════════════════════════════════════════════════════
static void test_fill_layout() {
    printf("\n-- FILL cell --\n");
    auto rules = sf::TechRules::sky130_rules();
    sf::CellLayoutGenerator gen(rules);

    auto fill1 = gen.generate_fill(1);
    CHECK(fill1.cell_name == "FILL_1", "fill name");
    CHECK(std::abs(fill1.width - rules.site_width) < 1e-6,
          "FILL_1 width == site_width");
    CHECK(std::abs(fill1.height - rules.row_height) < 1e-6,
          "height == row_height");
    CHECK(fill1.has_layer(sf::DrcLayer::NWELL), "has NWELL");
    CHECK(fill1.has_layer(sf::DrcLayer::MET1),  "has MET1 (power rails)");
    CHECK(fill1.find_pin("VDD") != nullptr, "has VDD");
    CHECK(fill1.find_pin("VSS") != nullptr, "has VSS");

    auto fill4 = gen.generate_fill(4);
    CHECK(std::abs(fill4.width - 4.0 * rules.site_width) < 1e-6,
          "FILL_4 width == 4 * site_width");
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 8: BUF = approximately 2x INV width
// ═══════════════════════════════════════════════════════════════════════════
static void test_buf_layout() {
    printf("\n-- BUF_X1 layout --\n");
    auto rules = sf::TechRules::sky130_rules();
    sf::CellLayoutGenerator gen(rules);

    auto buf = gen.generate_buf(1);
    auto inv = gen.generate_inv(1);

    CHECK(buf.cell_name == "BUF_X1", "cell name");
    CHECK(buf.width > 0, "width > 0");
    CHECK(std::abs(buf.height - rules.row_height) < 1e-6,
          "height == row_height");

    CHECK(buf.find_pin("A") != nullptr, "has input pin A");
    CHECK(buf.find_pin("Y") != nullptr, "has output pin Y");
    CHECK(buf.find_pin("VDD") != nullptr, "has VDD");
    CHECK(buf.find_pin("VSS") != nullptr, "has VSS");

    // BUF should be roughly 2x INV width (two inverter stages)
    double ratio = buf.width / inv.width;
    CHECK(ratio > 1.5 && ratio < 3.0,
          "BUF width ~ 2x INV width");

    printf("  BUF=%.3f  INV=%.3f  ratio=%.2f\n",
           buf.width, inv.width, ratio);
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 9: Tie cells
// ═══════════════════════════════════════════════════════════════════════════
static void test_tie_cells() {
    printf("\n-- TIEH / TIEL cells --\n");
    auto rules = sf::TechRules::sky130_rules();
    sf::CellLayoutGenerator gen(rules);

    auto tieh = gen.generate_tieh();
    CHECK(tieh.cell_name == "TIEH", "TIEH name");
    CHECK(tieh.find_pin("Y") != nullptr, "TIEH has output Y");
    CHECK(tieh.find_pin("VDD") != nullptr, "TIEH has VDD");
    CHECK(tieh.find_pin("VSS") != nullptr, "TIEH has VSS");
    CHECK(tieh.width > 0, "TIEH width > 0");
    CHECK(std::abs(tieh.height - rules.row_height) < 1e-6,
          "TIEH height == row_height");

    auto tiel = gen.generate_tiel();
    CHECK(tiel.cell_name == "TIEL", "TIEL name");
    CHECK(tiel.find_pin("Y") != nullptr, "TIEL has output Y");
    CHECK(tiel.find_pin("VDD") != nullptr, "TIEL has VDD");
    CHECK(tiel.find_pin("VSS") != nullptr, "TIEL has VSS");
    CHECK(tiel.width > 0, "TIEL width > 0");
}

// ═══════════════════════════════════════════════════════════════════════════
int main() {
    printf("=== Phase 93: Standard Cell Layout Generator ===\n");

    test_tech_rules_sky130();
    test_inv_layout();
    test_inv_drive_strengths();
    test_nand2_layout();
    test_nor2_layout();
    test_dff_layout();
    test_fill_layout();
    test_buf_layout();
    test_tie_cells();

    printf("\n=== Phase 93 result: %d / %d passed ===\n", passed, total);
    return (passed == total) ? 0 : 1;
}
