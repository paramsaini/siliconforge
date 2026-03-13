#pragma once
// SiliconForge — Filler, Endcap, and Well-Tap Cell Insertion
// Inserts physical-only cells after placement for manufacturing compliance.
// Reference: Synopsys IC Compiler insert_pad_filler, Cadence Innovus addFiller

#include "pnr/physical.hpp"
#include <string>
#include <map>
#include <vector>

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

    // ── Tier 3: Spare Cell Placement/Management ─────────────────────────
    struct SpareCellConfig {
        struct SpareType {
            std::string type;          // "NAND2", "NOR2", "INV", "BUF", "TIE"
            int count_per_region = 4;
            double width = 2.0;
            double height = 1.4;
        };
        std::vector<SpareType> spare_types;
        double region_pitch_x = 80.0;     // um, spacing between spare clusters
        double region_pitch_y = 80.0;
        bool near_critical_nets = true;    // prefer placement near timing-critical nets
    };

    struct SpareCellResult {
        int total_spares = 0;
        int spare_regions = 0;
        std::map<std::string, int> spares_by_type;
        std::string message;
    };

    SpareCellResult insert_spare_cells(const SpareCellConfig& cfg);

    // ── Tier 3: Filler Cell Keepout Zones ───────────────────────────────
    struct KeepoutZone {
        Rect region;
        bool no_filler = true;
        bool no_decap = true;
        bool no_tap = false;
        std::string reason;                // "analog", "critical_timing", "macro_halo"
    };

    struct DensityTarget {
        Rect region;
        double min_density = 0.20;
        double max_density = 0.80;
    };

    struct KeepoutInsertResult {
        int fillers_inserted = 0;
        int fillers_blocked = 0;
        int decaps_inserted = 0;
        int decaps_blocked = 0;
        int density_violations = 0;
        std::string message;
    };

    void add_keepout(const KeepoutZone& kz) { keepouts_.push_back(kz); }
    void add_density_target(const DensityTarget& dt) { density_targets_.push_back(dt); }
    KeepoutInsertResult insert_with_keepouts(const CellInsertConfig& cfg);
    int insert_decaps_grid(double pitch = 40.0, double decap_width = 2.0);

private:
    PhysicalDesign& pd_;
    std::vector<KeepoutZone> keepouts_;
    std::vector<DensityTarget> density_targets_;

    int insert_endcaps(const CellInsertConfig& cfg);
    int insert_well_taps(const CellInsertConfig& cfg);
    int insert_fillers(const CellInsertConfig& cfg);
    bool in_keepout(const Point& p, double w, double h) const;
};

} // namespace sf
