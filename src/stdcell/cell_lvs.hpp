#pragma once
// SiliconForge — Cell-Level Layout vs Schematic (LVS)
// Extracts MOSFET devices from cell layout geometry and compares
// against a schematic device list for transistor-level verification.
//
// References:
//   - Weste & Harris, "CMOS VLSI Design", 4th ed., Ch. 3 (Layout)
//   - Calibre nmLVS User Manual concepts (device recognition)
//   - BSIM4 device geometry model (W/L extraction)

#include "stdcell/cell_layout.hpp"
#include <string>
#include <vector>

namespace sf {

// ── Extracted / Schematic Device ─────────────────────────────────────────

struct CellLvsDevice {
    enum Type { NMOS, PMOS, RES, CAP } type;
    std::string gate;
    std::string drain;
    std::string source;
    std::string bulk;
    double w = 0.0;   // gate width  (um)
    double l = 0.0;   // gate length (um)
};

// ── Cell LVS Result ──────────────────────────────────────────────────────

struct CellLvsResult {
    bool match = false;
    int  layout_devices    = 0;
    int  schematic_devices = 0;
    int  matched           = 0;
    std::vector<std::string> mismatches;
    double time_ms = 0.0;
};

// ── Cell LVS Checker ────────────────────────────────────────────────────

class CellLvsChecker {
public:
    /// Construct from layout and expected (schematic) device list.
    CellLvsChecker(const CellLayout& layout,
                   const std::vector<CellLvsDevice>& schematic);

    /// Extract MOSFET devices from layout geometry.
    /// Finds poly-over-diffusion intersections, classifies NMOS vs PMOS
    /// by NWELL presence, and computes W/L from overlap dimensions.
    std::vector<CellLvsDevice> extract_devices() const;

    /// Compare extracted layout devices against schematic.
    CellLvsResult compare() const;

private:
    const CellLayout& layout_;
    std::vector<CellLvsDevice> schematic_;

    // Determine the net name for a pin at a given location
    std::string net_at(int layer, const Rect& region) const;

    // Check if a point lies inside any rect on a given layer
    bool point_in_layer(double x, double y, int layer) const;
};

} // namespace sf
