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

    // Enhanced check violation counts
    int via_enclosure_violations = 0;
    int density_uniformity_violations = 0;
    int off_grid_violations = 0;
    int antenna_per_layer_violations = 0;
    int min_area_enhanced_violations = 0;
    int waived_violations = 0;

    // Context-dependent and inter-layer rule violation counts
    int context_rule_violations = 0;
    int inter_layer_violations = 0;

    // 3D IC rule violation counts
    int tsv_keepout_violations = 0;
    int inter_die_alignment_violations = 0;
    int bump_pitch_violations = 0;
    int thermal_via_density_violations = 0;

    // Lithography violation counts
    int litho_resolution_violations = 0;
    int litho_line_end_violations = 0;
    int litho_corner_rounding_violations = 0;
};

// ── Context-dependent rules ──────────────────────────────────────────────

struct ContextRule {
    std::string name;
    std::string layer;
    double spacing = 0.0;
    double min_width = 0.0;
    std::string context;        // "parallel_run", "end_of_line", "corner"
    double context_distance = 0.0;
};

// ── Layer-to-layer interaction rules ─────────────────────────────────────

struct InterLayerRule {
    std::string layer1;
    std::string layer2;
    double min_spacing = 0.0;
    double min_enclosure = 0.0;
    std::string type;           // "spacing", "enclosure", "extension"
};

// ── 3D IC Design Rule Configuration ─────────────────────────────────────
// Rules for 2.5D/3D stacked die: through-silicon vias (TSVs), micro-bumps,
// inter-die alignment, and thermal via density for vertical heat extraction.
//
// References:
//   - ITRS 3D integration roadmap (keep-out zone scaling)
//   - Lau, "3D IC Integration and Packaging", McGraw-Hill
//   - JEDEC JC-14 bump pitch standards

struct Drc3dConfig {
    double tsv_keepout_um = 5.0;         // minimum exclusion zone around each TSV
    double bump_pitch_um = 40.0;         // micro-bump center-to-center pitch
    double thermal_via_density = 0.05;   // minimum thermal via density (fraction)
    double tsv_diameter_um = 5.0;        // TSV barrel diameter
    double inter_die_alignment_um = 1.0; // maximum tolerated die-to-die misalignment
    double min_bump_diameter_um = 20.0;  // minimum micro-bump pad diameter
    int num_dies = 2;                    // number of stacked dies
};

// ── Lithography Simulation Configuration ────────────────────────────────
// Simplified aerial image simulation for printability analysis using
// Rayleigh diffraction limit: resolution = k1 * lambda / NA.
//
// References:
//   - Mack, "Fundamental Principles of Optical Lithography", Wiley
//   - Wong, "Resolution Enhancement Techniques in Optical Lithography"
//   - Rayleigh criterion for minimum resolvable feature

struct LithoConfig {
    double wavelength_nm = 193.0;        // exposure wavelength (193nm ArF immersion)
    double NA = 1.35;                    // numerical aperture (immersion scanner)
    double resist_threshold = 0.3;       // aerial image threshold (normalized)
    double defocus_nm = 30.0;            // defocus budget
    double k1_factor = 0.28;             // process k1 factor (0.25 = aggressive EUV, 0.5 = relaxed)
    double sigma_outer = 0.9;            // illumination outer sigma (annular/dipole)
    double sigma_inner = 0.6;            // illumination inner sigma
};

// DFM Yield Prediction based on DRC violation density
// Uses Poisson yield model: Y = exp(-D * A_crit)
// where D = defect density (defects/cm^2) and A_crit = critical area (cm^2)
// Critical area computed from minimum spacing violations and wire density.
//
// Reference: Stapper, "Modeling of Defects in IC Photolithographic Patterns", IBM J. Res. 1984
// Reference: Koren & Koren, "Defect Tolerant VLSI Circuits", IEEE Design & Test 1998
// Reference: Papadopoulou et al., "Critical Area Computation via Voronoi Diagrams", TCAD 2000
struct YieldPrediction {
    double critical_area_um2 = 0.0;     // total critical area susceptible to defects
    double defect_density = 0.5;        // defects per cm^2 (fab-specific, typical 0.1-2.0)
    double yield_estimate = 1.0;        // predicted parametric yield [0, 1]
    int killer_defect_count = 0;        // estimated killer defects from violation clustering

    // Per-layer critical area breakdown
    struct LayerCritArea {
        int layer = 0;
        double spacing_crit_area = 0.0;   // critical area from min-spacing near-violations
        double width_crit_area = 0.0;     // critical area from narrow width segments
        double via_crit_area = 0.0;       // critical area around via clusters
    };
    std::vector<LayerCritArea> layer_breakdown;

    // Yield loss contributors
    double systematic_yield_loss = 0.0; // from DRC violations (deterministic)
    double random_yield_loss = 0.0;     // from defect density (stochastic)
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

    // ── Via enclosure checking ───────────────────────────────────────
    struct ViaEnclosureRule {
        int via_layer;
        int metal_above;
        int metal_below;
        double enclosure_above;  // min enclosure on upper metal
        double enclosure_below;  // min enclosure on lower metal
    };
    void add_via_enclosure_rule(const ViaEnclosureRule& rule);
    int check_via_enclosure();

    // ── Density uniformity (window-based) ────────────────────────────
    struct DensityRule {
        int layer;
        double min_density;     // e.g., 0.2 (20%)
        double max_density;     // e.g., 0.8 (80%)
        double window_size;     // um, sliding window size
        double window_step;     // um, step between windows
    };
    void add_density_rule(const DensityRule& rule);
    int check_density_uniformity();

    // ── Off-grid checking ────────────────────────────────────────────
    struct GridRule {
        int layer;
        double grid_x;          // manufacturing grid X pitch
        double grid_y;          // manufacturing grid Y pitch
    };
    void add_grid_rule(const GridRule& rule);
    int check_off_grid();

    // ── Per-layer antenna ratio ──────────────────────────────────────
    struct AntennaRule {
        int layer;
        double max_ratio;       // max metal_area / gate_area ratio
        double max_cum_ratio;   // cumulative across layers
    };
    void add_antenna_rule(const AntennaRule& rule);
    int check_antenna_per_layer();

    // ── Minimum area rules ───────────────────────────────────────────
    struct MinAreaRule {
        int layer;
        double min_area;        // um^2
    };
    void add_min_area_rule(const MinAreaRule& rule);
    int check_min_area_enhanced();

    // ── Soft DRC waiver system ───────────────────────────────────────
    struct DrcWaiver {
        std::string rule_name;
        int layer;
        double x_lo, y_lo, x_hi, y_hi;  // waiver region
        std::string reason;
    };
    void add_waiver(const DrcWaiver& waiver);
    bool is_waived(const std::string& rule, int layer, double x, double y) const;

    // ── Rule deck parser (simplified Calibre-like format) ────────────
    // Format: "SPACING M1 0.065" / "WIDTH M1 0.04" / etc.
    int load_rule_deck(const std::string& filename);
    int load_rule_deck_string(const std::string& deck);

    // ── Enhanced DRC run with all new checks ─────────────────────────
    DrcResult run_enhanced();

    // ── Context-dependent rules ──────────────────────────────────────
    std::vector<ContextRule> context_rules;
    void add_context_rule(const ContextRule& rule);
    int check_context_rules(DrcResult& r);

    // ── Layer-to-layer interaction rules ─────────────────────────────
    std::vector<InterLayerRule> inter_layer_rules;
    void add_inter_layer_rule(const InterLayerRule& rule);
    int check_inter_layer_rules(DrcResult& r);

    // ── 3D IC design rule checks ────────────────────────────────────
    // Validates 3D-specific manufacturing constraints for TSV-based
    // stacked die integration, micro-bump interconnect, and thermal
    // via placement density.
    void set_3d_config(const Drc3dConfig& cfg) { drc3d_cfg_ = cfg; }
    const Drc3dConfig& drc3d_config() const { return drc3d_cfg_; }
    DrcResult check_3d_rules();

    // ── DFM Yield Prediction ─────────────────────────────────────────
    // Estimates parametric yield from critical area analysis and DRC
    // violation density using the Poisson yield model.
    // defect_density: defects/cm^2 (typical fab range: 0.1 for mature, 2.0 for early)
    YieldPrediction predict_yield(double defect_density = 0.5);

    // ── Lithography printability analysis ────────────────────────────
    // Identifies features below the optical resolution limit (Rayleigh
    // criterion), line-end shortening candidates, and corner rounding
    // regions.  Uses k1*lambda/NA as the minimum resolvable feature.
    void set_litho_config(const LithoConfig& cfg) { litho_cfg_ = cfg; }
    const LithoConfig& litho_config() const { return litho_cfg_; }
    DrcResult litho_check();

private:
    std::vector<ViaEnclosureRule> via_enclosure_rules_;
    std::vector<DensityRule> density_rules_;
    std::vector<GridRule> grid_rules_;
    std::vector<AntennaRule> antenna_rules_;
    std::vector<MinAreaRule> min_area_rules_;
    std::vector<DrcWaiver> waivers_;

    const PhysicalDesign& pd_;
    std::vector<DrcRule> rules_;
    DrcConfig config_;
    Drc3dConfig drc3d_cfg_;
    LithoConfig litho_cfg_;

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

public:
    // ── Phase 98: Hierarchical DRC ──────────────────────────────────────
    // Check each unique cell definition once, then propagate violations
    // through instance placements. Reduces O(N^2) flat DRC to O(U*M^2 + I)
    // where U = unique cells, M = max shapes per cell, I = instances.
    //
    // Inter-cell spacing checks still run at the flat level but only
    // between boundary shapes of adjacent instances (scanline sweep).

    struct HierCellDef {
        std::string name;
        std::vector<WireSegment> shapes;   // local-coordinate shapes
        std::vector<Via> local_vias;
        Rect bbox;                         // bounding box in local coords
    };

    struct HierInstance {
        int cell_def_idx;
        Point offset;                      // placement origin
        bool mirror_x = false;
        bool mirror_y = false;
    };

    struct HierDrcResult {
        int intra_cell_violations = 0;     // violations inside cell defs
        int inter_cell_violations = 0;     // violations between instances
        int unique_cells_checked = 0;
        int instances_checked = 0;
        double time_ms = 0;
        std::vector<DrcViolation> all_violations;
        std::string summary;
    };

    // Register cell definitions and instances
    void add_hier_cell(const HierCellDef& cell) { hier_cells_.push_back(cell); }
    void add_hier_instance(const HierInstance& inst) { hier_instances_.push_back(inst); }
    void clear_hierarchy() { hier_cells_.clear(); hier_instances_.clear(); }

    // Run hierarchical DRC
    HierDrcResult check_hierarchical();

    // ── Scanline-based inter-cell spacing ────────────────────────────────
    // Sweeps a vertical line across the design, maintaining an active set
    // of wire segments. Only checks spacing between segments from different
    // instances that overlap in the Y dimension. O(N log N) vs O(N^2).

    struct ScanlineConfig {
        double min_spacing = 0.14;        // global minimum spacing
        int max_events_per_sweep = 100000; // memory limit
    };
    void set_scanline_config(const ScanlineConfig& cfg) { scanline_cfg_ = cfg; }
    int check_inter_cell_scanline(HierDrcResult& result);

private:
    std::vector<HierCellDef> hier_cells_;
    std::vector<HierInstance> hier_instances_;
    ScanlineConfig scanline_cfg_;

    // Intra-cell DRC: check one cell definition
    int check_intra_cell(const HierCellDef& cell, HierDrcResult& result);

    // Transform local shape to global coordinates
    WireSegment transform_shape(const WireSegment& local, const HierInstance& inst) const;
};

} // namespace sf
