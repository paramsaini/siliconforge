#pragma once
// SiliconForge — Filler, Endcap, and Well-Tap Cell Insertion
// Inserts physical-only cells after placement for manufacturing compliance.
// Reference: Synopsys IC Compiler insert_pad_filler, Cadence Innovus addFiller

#include "pnr/physical.hpp"
#include <string>

namespace sf {

struct CellInsertConfig {
    double tap_spacing   = 20.0;                     // max distance between well-taps (um)
    double filler_widths[4] = {0.5, 1.0, 2.0, 4.0};  // FILL1/FILL2/FILL4/FILL8
    double endcap_width  = 1.0;
    double tap_width     = 2.0;
};

struct CellInsertResult {
    int fillers   = 0;
    int endcaps   = 0;
    int well_taps = 0;
    std::string message;
};

class CellInserter {
public:
    explicit CellInserter(PhysicalDesign& pd) : pd_(pd) {}
    CellInsertResult insert(const CellInsertConfig& cfg = {});

    // Insert decoupling capacitor cells near IR-drop hotspots or at regular intervals
    int insert_decaps(const std::vector<Point>& hotspots, double decap_width = 2.0);

private:
    PhysicalDesign& pd_;
    int insert_endcaps(const CellInsertConfig& cfg);
    int insert_well_taps(const CellInsertConfig& cfg);
    int insert_fillers(const CellInsertConfig& cfg);
};

} // namespace sf
