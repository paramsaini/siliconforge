#pragma once
// SiliconForge — Design Rule Check (DRC) Engine (Industrial Grade)
// Comprehensive manufacturing rule validation with spatial indexing.
// Supports external PDK rule decks (JSON), conditional rules, and
// built-in SKY130 open-source PDK rule set.
//
// References:
//   - SkyWater SKY130 PDK Design Rules (skywater-pdk.readthedocs.io)
//   - Calibre DRC rule deck format (Siemens EDA)
//   - IC Validator / PVS rule syntax (Synopsys / Cadence)

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

// Layer identifiers for front-end and back-end process layers
// Matches SKY130 layer stack: front-end → back-end
struct DrcLayer {
    enum Id {
        // Front-end layers
        NWELL = 0, PWELL = 1, DNW = 2,         // wells
        DIFF = 3, TAP = 4,                       // active regions
        POLY = 5, NPC = 6,                       // gate/poly
        NSDM = 7, PSDM = 8,                      // implants
        HVI = 9, HVNTM = 10,                     // high-voltage
        // Contact/local interconnect
        LICON = 11, LI1 = 12, MCON = 13,
        // Back-end metal stack
        MET1 = 14, VIA1 = 15,
        MET2 = 16, VIA2 = 17,
        MET3 = 18, VIA3 = 19,
        MET4 = 20, VIA4 = 21,
        MET5 = 22,
        // Special
        PAD = 23, RDL = 24,
        CELL = -1,  // abstract cell layer (for standard cell checks)
        ANY = -2     // any layer (boundary, density)
    };
};

struct DrcRule {
    std::string name;
    std::string description;
    double value;          // rule value in um (or ratio for antenna/density)

    enum Type {
        // === Geometry rules ===
        MIN_WIDTH, MIN_SPACING, MIN_AREA, MIN_ENCLOSURE,
        MAX_WIDTH, MAX_LENGTH,
        // === Density rules ===
        DENSITY_MIN, DENSITY_MAX,
        // === Via/contact rules ===
        MIN_VIA_SPACING, VIA_ENCLOSURE, VIA_ARRAY_SPACING,
        MIN_VIA_WIDTH,
        // === Advanced spacing rules ===
        END_OF_LINE_SPACING,        // spacing for wire ends
        WIDE_WIRE_SPACING,          // width-dependent spacing
        PARALLEL_RUN_LENGTH_SPACING,// PRL-dependent spacing
        EOL_SPACING_WITHIN,         // EOL within distance
        SAME_NET_SPACING,           // min spacing same net
        DIFFERENT_NET_SPACING,      // min spacing different nets
        // === Hole rules ===
        MIN_HOLE_WIDTH, MIN_HOLE_SPACING, MIN_ENCLOSED_AREA,
        // === Antenna rules ===
        ANTENNA_RATIO, ANTENNA_AREA, ANTENNA_CUMULATIVE_RATIO,
        // === Cut/via rules ===
        CUT_SPACING, CUT_ENCLOSURE, MIN_CUT_AREA,
        // === Step/jog rules ===
        MIN_STEP, JOG_LENGTH, JOG_SPACING,
        // === Fill/corner rules ===
        NOTCH_FILL, CORNER_FILL,
        // === Fat wire rules ===
        FATFINGER_WIDTH, FATFINGER_SPACING,
        // === Front-end specific ===
        POLY_ENDCAP,          // poly extension past diffusion
        POLY_GATE_MIN_WIDTH,  // min poly width over active (gate length)
        GATE_SPACING,         // poly-to-poly over active
        DIFF_ENCLOSURE,       // diffusion enclosure of contact
        WELL_ENCLOSURE,       // well enclosure of active
        WELL_SPACING,         // well-to-well spacing
        IMPLANT_ENCLOSURE,    // implant enclosure of active
        IMPLANT_SPACING,      // implant-to-implant spacing
        LATCHUP_SPACING,      // tap-to-source spacing (latchup)
        // === Extension rules ===
        MIN_EXTENSION,        // min extension of one layer past another
        MIN_OVERLAP,          // min overlap between layers
        // === Conditional rules (width-dependent, context-dependent) ===
        CONDITIONAL_SPACING,  // spacing = f(width, run_length)
        CONDITIONAL_ENCLOSURE, // enclosure = f(width)
        // === Advanced reliability / manufacturing rules ===
        EM_MIN_WIDTH,              // Electromigration-aware minimum wire width
        MULTI_PATTERNING_COLOR,    // Multi-patterning color assignment check
        STRESS_VOIDING,            // Metal stress voiding (via-to-via on wide wires)
        ESD_SPACING,               // ESD protection spacing rules
        GUARD_RING_SPACING,        // Latchup guard ring width/spacing
        RELIABILITY_WIDTH,         // Reliability-driven minimum width (lifetime-aware)
        TEMP_VARIANT_SPACING       // Temperature-dependent spacing rules
    } type;

    int layer;                  // primary layer (DrcLayer::Id or int from JSON)
    int layer2 = -1;            // secondary layer (for inter-layer rules)
    double aux_value = 0;       // secondary parameter (width threshold, etc.)
    double aux_value2 = 0;      // tertiary parameter (parallel run length, etc.)

    // Conditional rule support: if wire_width >= condition_min_width, use value
    double condition_min_width = 0;   // width condition threshold
    double condition_min_length = 0;  // length/PRL condition threshold

    // Severity override
    enum Severity { ERROR, WARNING, INFO } severity = ERROR;

    // Rule enabled/disabled flag
    bool enabled = true;
};

struct DrcConfig {
    double temperature_c = 25.0;          // operating temperature
    double em_current_limit_ma = 1.0;     // default EM current limit (mA)
    int multi_patterning_colors = 2;      // 2 = double patterning, 3 = triple
    bool enable_reliability_rules = true;
    bool enable_em_rules = true;
    double target_lifetime_years = 10.0;  // for reliability rules
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

    // Per-category violation counts (industrial reporting)
    int width_violations = 0;
    int spacing_violations = 0;
    int enclosure_violations = 0;
    int area_violations = 0;
    int antenna_violations = 0;
    int density_violations = 0;
    int via_violations = 0;
    int other_violations = 0;

    // Advanced rule violation counts
    int em_width_violations = 0;
    int multi_patterning_violations = 0;
    int stress_violations = 0;
    int esd_violations = 0;
    int guard_ring_violations = 0;
    int reliability_violations = 0;
    int temp_violations = 0;
};

class DrcEngine {
public:
    DrcEngine(const PhysicalDesign& pd) : pd_(pd) {}

    void set_config(const DrcConfig& cfg) { config_ = cfg; }
    const DrcConfig& config() const { return config_; }

    void add_rule(const DrcRule& rule) { rules_.push_back(rule); }
    void load_default_rules(double min_feature_um = 0.13);
    void load_advanced_rules(double min_feature_um = 0.13, int num_metal_layers = 6);

    // === Industrial PDK rule loading ===

    // Load external PDK rule deck from JSON file
    int load_rules_from_file(const std::string& filename, bool append = false);

    // Write current rules to JSON file (for export/round-tripping)
    bool write_rules_to_file(const std::string& filename) const;

    // Built-in SKY130 open-source PDK rule deck (500+ rules)
    // Based on SkyWater SKY130 PDK documentation (skywater-pdk.readthedocs.io)
    void load_sky130_rules();

    // Access loaded rules
    const std::vector<DrcRule>& rules() const { return rules_; }
    int rule_count() const { return static_cast<int>(rules_.size()); }
    void clear_rules() { rules_.clear(); }
    void enable_rule(const std::string& name, bool enabled = true);
    void disable_rule(const std::string& name) { enable_rule(name, false); }

    // Advanced node rule loading (28nm and below)
    void load_advanced_node_rules(double min_feature_um);

    // Auto-generate EM rules from current density limits
    void generate_em_rules(int num_metal_layers = 6);

    DrcResult check();

private:
    const PhysicalDesign& pd_;
    std::vector<DrcRule> rules_;
    DrcConfig config_;

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
    void check_conditional_spacing(DrcResult& r);
    void check_conditional_enclosure(DrcResult& r);

    // Advanced rule checks
    void check_em_width(DrcResult& r);
    void check_multi_patterning(DrcResult& r);
    void check_stress_voiding(DrcResult& r);
    void check_esd_spacing(DrcResult& r);
    void check_guard_ring(DrcResult& r);
    void check_reliability_width(DrcResult& r);
    void check_temp_variant(DrcResult& r);

    // Wire geometry helpers
    double wire_length(const WireSegment& w) const;
    double wire_area(const WireSegment& w) const;
    double wires_spacing(const WireSegment& a, const WireSegment& b) const;
    double parallel_run_length(const WireSegment& a, const WireSegment& b) const;
    bool is_eol(const WireSegment& w, double eol_width) const;
};

} // namespace sf
