// SiliconForge — Phase 31: LEF Parser Industrial Tests
#include "core/lef_parser.hpp"
#include <iostream>
#include <sstream>

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

using namespace sf;

static const char* BASIC_LEF = R"(
VERSION 5.8 ;
UNITS
  DATABASE MICRONS 1000 ;
END UNITS

LAYER metal1
  TYPE ROUTING ;
  DIRECTION HORIZONTAL ;
  PITCH 0.28 ;
  WIDTH 0.14 ;
  SPACING 0.14 ;
  RESISTANCE RPERSQ 12.8 ;
  CAPACITANCE CPERSQDIST 0.000115 ;
  THICKNESS 0.13 ;
END metal1

LAYER via1
  TYPE CUT ;
END via1

LAYER metal2
  TYPE ROUTING ;
  DIRECTION VERTICAL ;
  PITCH 0.28 ;
  WIDTH 0.14 ;
  SPACING 0.14 ;
END metal2

VIA via1_default DEFAULT
  LAYER metal1 ;
    RECT -0.06 -0.06 0.06 0.06 ;
  LAYER via1 ;
    RECT -0.06 -0.06 0.06 0.06 ;
  LAYER metal2 ;
    RECT -0.065 -0.065 0.065 0.065 ;
END via1_default

SITE unit
  CLASS CORE ;
  SIZE 0.46 BY 2.72 ;
END unit

MACRO INV_X1
  CLASS CORE ;
  SIZE 0.46 BY 2.72 ;
  ORIGIN 0 0 ;
  SYMMETRY X Y ;
  SITE unit ;
  PIN A
    DIRECTION INPUT ;
    USE SIGNAL ;
    PORT
      LAYER metal1 ;
        RECT 0.06 1.09 0.16 1.63 ;
    END
  END A
  PIN Y
    DIRECTION OUTPUT ;
    USE SIGNAL ;
    PORT
      LAYER metal1 ;
        RECT 0.30 1.09 0.40 1.63 ;
    END
  END Y
  PIN VDD
    DIRECTION INOUT ;
    USE POWER ;
    PORT
      LAYER metal1 ;
        RECT 0.0 2.62 0.46 2.72 ;
    END
  END VDD
  PIN VSS
    DIRECTION INOUT ;
    USE GROUND ;
    PORT
      LAYER metal1 ;
        RECT 0.0 0.0 0.46 0.10 ;
    END
  END VSS
  OBS
    LAYER metal1 ;
      RECT 0.0 0.10 0.46 2.62 ;
  END
END INV_X1

MACRO BUF_X2
  CLASS CORE ;
  SIZE 0.92 BY 2.72 ;
  PIN A
    DIRECTION INPUT ;
    USE SIGNAL ;
    PORT
      LAYER metal1 ;
        RECT 0.06 1.09 0.16 1.63 ;
    END
  END A
  PIN Y
    DIRECTION OUTPUT ;
    USE SIGNAL ;
    PORT
      LAYER metal1 ;
        RECT 0.76 1.09 0.86 1.63 ;
    END
  END Y
END BUF_X2

END LIBRARY
)";

TEST(parse_basic_lef) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    CHECK(lib.version == "5.8", "Version should be 5.8");
    PASS("parse_basic_lef");
}

TEST(units_parsing) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    CHECK(lib.units.database_microns == 1000, "DB microns should be 1000");
    PASS("units_parsing");
}

TEST(layer_routing) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    auto* m1 = lib.find_layer("metal1");
    CHECK(m1 != nullptr, "metal1 should exist");
    CHECK(m1->width > 0, "metal1 width should be positive");
    CHECK(m1->pitch > 0, "metal1 pitch should be positive");
    CHECK(m1->spacing > 0, "metal1 spacing should be positive");
    PASS("layer_routing");
}

TEST(layer_cut) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    auto* v1 = lib.find_layer("via1");
    CHECK(v1 != nullptr, "via1 should exist");
    PASS("layer_cut");
}

TEST(layer_count) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    CHECK(lib.num_routing_layers() == 2, "Should have 2 routing layers");
    CHECK(lib.num_cut_layers() == 1, "Should have 1 cut layer");
    PASS("layer_count");
}

TEST(via_parsing) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    auto* v = lib.find_via("via1_default");
    CHECK(v != nullptr, "via1_default should exist");
    CHECK(!v->layers.empty(), "Via should have layers");
    PASS("via_parsing");
}

TEST(site_parsing) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    auto* s = lib.find_site("unit");
    CHECK(s != nullptr, "unit site should exist");
    CHECK(s->width > 0, "Site width positive");
    CHECK(s->height > 0, "Site height positive");
    PASS("site_parsing");
}

TEST(macro_inv) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    auto* m = lib.find_macro("INV_X1");
    CHECK(m != nullptr, "INV_X1 macro should exist");
    CHECK(m->width > 0, "Macro width positive");
    CHECK(m->height > 0, "Macro height positive");
    CHECK(m->pins.size() >= 3, "INV should have at least 3 pins (A, Y, VDD/VSS)");
    PASS("macro_inv");
}

TEST(macro_pin_direction) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    auto* m = lib.find_macro("INV_X1");
    CHECK(m != nullptr, "INV_X1 should exist");
    bool found_input = false, found_output = false;
    for (auto& p : m->pins) {
        if (p.name == "A") { CHECK(p.direction == LefPin::INPUT, "A should be INPUT"); found_input = true; }
        if (p.name == "Y") { CHECK(p.direction == LefPin::OUTPUT, "Y should be OUTPUT"); found_output = true; }
    }
    CHECK(found_input, "Should find input pin");
    CHECK(found_output, "Should find output pin");
    PASS("macro_pin_direction");
}

TEST(macro_pin_port_geometry) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    auto* m = lib.find_macro("INV_X1");
    CHECK(m != nullptr, "INV_X1 exists");
    bool found_port = false;
    for (auto& p : m->pins) {
        if (p.name == "A" && !p.ports.empty()) {
            found_port = true;
            CHECK(!p.ports[0].rects.empty(), "Port should have geometry");
        }
    }
    CHECK(found_port, "Should find port with geometry");
    PASS("macro_pin_port_geometry");
}

TEST(macro_obs) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    auto* m = lib.find_macro("INV_X1");
    CHECK(m != nullptr, "INV_X1 exists");
    CHECK(!m->obs.layers.empty(), "Should have obstructions");
    PASS("macro_obs");
}

TEST(multiple_macros) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    CHECK(lib.macros.size() >= 2, "Should have at least 2 macros");
    CHECK(lib.find_macro("BUF_X2") != nullptr, "BUF_X2 should exist");
    PASS("multiple_macros");
}

TEST(find_nonexistent) {
    LefParser parser;
    auto lib = parser.parse_string(BASIC_LEF);
    CHECK(lib.find_macro("NAND_X99") == nullptr, "Nonexistent macro should return null");
    CHECK(lib.find_layer("metal99") == nullptr, "Nonexistent layer should return null");
    PASS("find_nonexistent");
}

TEST(empty_input) {
    LefParser parser;
    auto lib = parser.parse_string("");
    CHECK(lib.macros.empty(), "Empty input should produce empty library");
    PASS("empty_input");
}

int main() {
    std::cout << "═══════════════════════════════════════════════════════\n"
              << " Phase 31: LEF Parser Industrial Tests\n"
              << "═══════════════════════════════════════════════════════\n\n";
    RUN(parse_basic_lef);
    RUN(units_parsing);
    RUN(layer_routing);
    RUN(layer_cut);
    RUN(layer_count);
    RUN(via_parsing);
    RUN(site_parsing);
    RUN(macro_inv);
    RUN(macro_pin_direction);
    RUN(macro_pin_port_geometry);
    RUN(macro_obs);
    RUN(multiple_macros);
    RUN(find_nonexistent);
    RUN(empty_input);
    std::cout << "\n═══════════════════════════════════════════════════════\n"
              << " Results: " << passed << " passed, " << failed << " failed\n"
              << "═══════════════════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
