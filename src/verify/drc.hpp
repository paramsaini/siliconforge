#pragma once
// SiliconForge — Design Rule Check (DRC) Engine
// Comprehensive manufacturing rule validation with spatial indexing.
// Reference: "DRC Rule Deck", Foundry PDK documentation
//            TSMC/Samsung/GF design rule manuals for 45nm-7nm nodes

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

struct DrcRule {
    std::string name;
    std::string description;
    double value;          // rule value in um
    enum Type {
        MIN_WIDTH, MIN_SPACING, MIN_AREA, MIN_ENCLOSURE,
        MAX_WIDTH, DENSITY_MIN, DENSITY_MAX,
        // Extended rule types
        MIN_VIA_SPACING, VIA_ENCLOSURE, VIA_ARRAY_SPACING,
        END_OF_LINE_SPACING, WIDE_WIRE_SPACING,
        MIN_HOLE_WIDTH, MIN_HOLE_SPACING,
        ANTENNA_RATIO, ANTENNA_AREA,
        CUT_SPACING, CUT_ENCLOSURE,
        MIN_STEP, JOG_LENGTH, JOG_SPACING,
        NOTCH_FILL, CORNER_FILL,
        FATFINGER_WIDTH, FATFINGER_SPACING,
        PARALLEL_RUN_LENGTH_SPACING, // spacing depends on parallel run length
        EOL_SPACING_WITHIN, // end-of-line spacing within distance
        MIN_CUT_AREA
    } type;
    int layer;
    double aux_value = 0;  // secondary parameter (e.g., width threshold for wide-wire rules)
    double aux_value2 = 0; // tertiary parameter (e.g., parallel run length)
};

struct DrcViolation {
    std::string rule_name;
    std::string message;
    Rect bbox;
    double actual_value;
    double required_value;
    enum Severity { ERROR, WARNING, INFO } severity;
};

struct DrcResult {
    int total_rules = 0;
    int violations = 0;
    int errors = 0;
    int warnings = 0;
    std::vector<DrcViolation> details;
    double time_ms = 0;
    std::string message;
};

class DrcEngine {
public:
    DrcEngine(const PhysicalDesign& pd) : pd_(pd) {}

    void add_rule(const DrcRule& rule) { rules_.push_back(rule); }
    void load_default_rules(double min_feature_um = 0.13);
    void load_advanced_rules(double min_feature_um = 0.13, int num_metal_layers = 6);

    DrcResult check();

private:
    const PhysicalDesign& pd_;
    std::vector<DrcRule> rules_;

    // Spatial index for O(n log n) neighbor queries
    struct GridCell { std::vector<int> wire_ids; std::vector<int> cell_ids; std::vector<int> via_ids; };
    std::vector<std::vector<GridCell>> spatial_grid_;
    int sgrid_nx_ = 0, sgrid_ny_ = 0;
    double sgrid_cw_ = 0, sgrid_ch_ = 0;
    void build_spatial_index();
    std::vector<int> query_nearby_wires(double x, double y, double radius, int layer) const;
    std::vector<int> query_nearby_cells(double x, double y, double radius) const;

    // Check categories
    void check_min_width(DrcResult& r);
    void check_min_spacing(DrcResult& r);
    void check_min_area(DrcResult& r);
    void check_boundary(DrcResult& r);
    void check_density(DrcResult& r);
    void check_via_rules(DrcResult& r);
    void check_wide_wire_spacing(DrcResult& r);
    void check_end_of_line(DrcResult& r);
    void check_antenna(DrcResult& r);
    void check_min_step(DrcResult& r);
    void check_jog(DrcResult& r);
    void check_parallel_run(DrcResult& r);
    void check_cut_rules(DrcResult& r);
    void check_notch_fill(DrcResult& r);
    void check_enclosure(DrcResult& r);
    void check_max_width(DrcResult& r);
    void check_min_hole(DrcResult& r);

    // Wire geometry helpers
    double wire_length(const WireSegment& w) const;
    double wire_area(const WireSegment& w) const;
    double wires_spacing(const WireSegment& a, const WireSegment& b) const;
    double parallel_run_length(const WireSegment& a, const WireSegment& b) const;
    bool is_eol(const WireSegment& w, double eol_width) const;
};

} // namespace sf
