#pragma once
// SiliconForge -- Standard Cell Layout Data Structures and Generator
// Represents transistor-level physical geometry for standard cells,
// including active regions, poly gates, contacts, local interconnect,
// metal routing, wells, and implant layers.
//
// The CellLayoutGenerator produces DRC-clean-by-construction layouts
// using the TechRules parameter set for a given process node.

#include "pnr/physical.hpp"
#include "verify/drc.hpp"
#include "stdcell/layout_rules.hpp"
#include <string>
#include <vector>
#include <utility>

namespace sf {

// Axis-aligned rectangle on a specific GDSII/process layer
struct LayoutRect {
    int layer = 0;        // DrcLayer::Id
    Rect bounds;

    LayoutRect() = default;
    LayoutRect(int l, const Rect& b) : layer(l), bounds(b) {}
    LayoutRect(int l, double x0, double y0, double x1, double y1)
        : layer(l), bounds(x0, y0, x1, y1) {}
};

// Arbitrary polygon on a specific layer (for non-rectangular shapes)
struct LayoutPolygon {
    int layer = 0;
    std::vector<Point> vertices;

    LayoutPolygon() = default;
    LayoutPolygon(int l, const std::vector<Point>& v) : layer(l), vertices(v) {}
};

// Pin shape with label and direction metadata
struct LayoutPin {
    std::string name;
    int layer = 0;        // DrcLayer::Id for pin shape
    Rect bounds;
    std::string direction; // "INPUT", "OUTPUT", "INOUT", "POWER", "GROUND"

    LayoutPin() = default;
    LayoutPin(const std::string& n, int l, const Rect& b, const std::string& d)
        : name(n), layer(l), bounds(b), direction(d) {}
};

// Complete layout for a single standard cell
struct CellLayout {
    std::string cell_name;
    double width  = 0.0;  // cell boundary width (um)
    double height = 0.0;  // cell boundary height (um), equals row_height

    std::vector<LayoutRect>  rects;       // all geometry rectangles
    std::vector<LayoutPin>   pins;        // pin shapes + labels
    std::vector<std::pair<std::string,std::string>> properties; // (name,value)

    // Find a pin by name (returns nullptr if not found)
    const LayoutPin* find_pin(const std::string& name) const {
        for (auto& p : pins)
            if (p.name == name) return &p;
        return nullptr;
    }

    // Check whether any rectangle exists on a given layer
    bool has_layer(int layer_id) const {
        for (auto& r : rects)
            if (r.layer == layer_id) return true;
        return false;
    }

    // Count rectangles on a given layer
    int count_layer(int layer_id) const {
        int n = 0;
        for (auto& r : rects)
            if (r.layer == layer_id) ++n;
        return n;
    }
};

// Generates transistor-level standard cell layouts from technology rules.
// All generated layouts are DRC-clean by construction: every rectangle
// respects minimum width, spacing, enclosure, and endcap constraints
// encoded in the TechRules structure.
class CellLayoutGenerator {
public:
    explicit CellLayoutGenerator(const TechRules& rules) : rules_(rules) {}

    // ----- Combinational cells -----

    // CMOS inverter.  drive_strength scales PMOS/NMOS widths:
    //   X1 = 1x minimum, X2 = 2x, X4 = 4x.
    CellLayout generate_inv(int drive_strength = 1);

    // 2-input NAND: 2 series NMOS + 2 parallel PMOS
    CellLayout generate_nand2(int drive_strength = 1);

    // 2-input NOR: 2 parallel NMOS + 2 series PMOS
    CellLayout generate_nor2(int drive_strength = 1);

    // Non-inverting buffer (two cascaded inverters)
    CellLayout generate_buf(int drive_strength = 1);

    // ----- Sequential cells -----

    // Positive-edge-triggered D flip-flop (transmission-gate master-slave)
    CellLayout generate_dff();

    // ----- Utility cells -----

    // Filler cell: NWELL + power rails + well taps, no logic
    CellLayout generate_fill(int sites = 1);

    // Tie-high: output permanently connected to VDD
    CellLayout generate_tieh();

    // Tie-low: output permanently connected to VSS
    CellLayout generate_tiel();

private:
    TechRules rules_;

    // Internal helpers for building layout rectangles

    // Add power rails (VDD at top, VSS at bottom) on MET1
    void add_power_rails(CellLayout& cl);

    // Add NWELL covering the PMOS (upper) region
    void add_nwell(CellLayout& cl);

    // Add well taps (substrate/well ties) at cell edges
    void add_well_taps(CellLayout& cl);

    // Place a single transistor (diffusion + gate + contacts + implant)
    // Returns the x-coordinate of the right edge of the placed transistor.
    //   x_origin  : left edge of diffusion
    //   y_center  : vertical center of the active region
    //   width     : transistor width (diffusion height in y)
    //   is_pmos   : true for PMOS (PSDM + NWELL), false for NMOS (NSDM)
    //   gate_name : pin name for the gate (e.g. "A")
    double place_transistor(CellLayout& cl, double x_origin, double y_center,
                            double width, bool is_pmos,
                            const std::string& gate_name = "");

    // Place a contact stack (LICON + LI1) at given center coordinates
    void place_contact(CellLayout& cl, double cx, double cy);

    // Place a metal1 connection between two points (horizontal or vertical strap)
    void place_m1_strap(CellLayout& cl, double x0, double y0, double x1, double y1);

    // Snap width to integer number of sites
    double snap_to_site(double w) const;
};

} // namespace sf
