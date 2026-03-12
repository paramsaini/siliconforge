#pragma once
// SiliconForge — LEF 5.8 Parser
// Reads Library Exchange Format files for standard cell physical definitions.
// Supports: UNITS, LAYER, VIA, VIARULE, SITE, MACRO (PIN, OBS, PORT), SPACING, PROPERTY
// Reference: LEF/DEF Language Reference 5.8, Cadence

#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <functional>
#include <cmath>
#include <utility>

namespace sf {

// ── Unit conversion factors ──────────────────────────────────────────────────

struct LefUnits {
    double database_microns = 100;
    double capacitance_pf   = 1.0;
    double resistance_kohm  = 1.0;
    double time_ns          = 1.0;
    double power_mw         = 1.0;
};

// ── Technology layer ─────────────────────────────────────────────────────────

struct LefLayer {
    std::string name;

    enum Type { ROUTING, CUT, MASTERSLICE, OVERLAP, IMPLANT } type = ROUTING;

    int layer_num = 0;

    double width     = 0;
    double min_width = 0;
    double max_width = 0;
    double pitch     = 0;
    double offset    = 0;
    double spacing   = 0;

    double resistance_per_sq   = 0;
    double capacitance_per_um  = 0;
    double edge_capacitance    = 0;
    double thickness           = 0;
    double height              = 0;
    double area                = 0;

    enum Direction { HORIZONTAL, VERTICAL, NONE } direction = NONE;

    // Wide-wire spacing table: (width, spacing) pairs
    std::vector<std::pair<double, double>> spacing_table;
};

// ── Via definition ───────────────────────────────────────────────────────────

struct LefVia {
    std::string name;

    struct ViaLayer {
        std::string layer_name;
        std::vector<std::array<double, 4>> rects; // {x0, y0, x1, y1}
    };

    std::vector<ViaLayer> layers;
    double resistance = 0;
};

// ── Via rule (generated vias) ────────────────────────────────────────────────

struct LefViaRule {
    std::string name;
    bool is_generate = false;

    struct RuleLayer {
        std::string layer_name;
        double width_lo  = 0, width_hi  = 0;
        double overhang1 = 0, overhang2 = 0;
        double spacing_x = 0, spacing_y = 0;
        double enc_x     = 0, enc_y     = 0;
    };

    std::vector<RuleLayer> layers;
};

// ── Placement site ───────────────────────────────────────────────────────────

struct LefSite {
    std::string name;

    enum Class { CORE, PAD, IO } site_class = CORE;
    enum Symmetry { X, Y, R90, NONE } symmetry = NONE;

    double width  = 0;
    double height = 0;
};

// ── Macro pin ────────────────────────────────────────────────────────────────

struct LefPin {
    std::string name;

    enum Direction { INPUT, OUTPUT, INOUT, FEEDTHRU } direction = INOUT;
    enum Use { SIGNAL, POWER, GROUND, CLOCK, ANALOG } use = SIGNAL;

    std::string shape; // ABUTMENT, RING, etc.

    struct Port {
        std::string layer;
        std::vector<std::array<double, 4>> rects; // {x0, y0, x1, y1}
    };
    std::vector<Port> ports;

    double capacitance = 0;
    double max_fanout  = 0;
    bool   is_analog   = false;
};

// ── Obstruction ──────────────────────────────────────────────────────────────

struct LefObs {
    struct ObsLayer {
        std::string layer;
        std::vector<std::array<double, 4>> rects;
    };
    std::vector<ObsLayer> layers;
};

// ── Macro (cell) definition ──────────────────────────────────────────────────

struct LefMacro {
    std::string name;

    enum Class {
        CORE, CORE_TIEHIGH, CORE_TIELOW, CORE_ANTENNACELL, CORE_WELLTAP,
        PAD, PAD_INPUT, PAD_OUTPUT, PAD_INOUT, PAD_POWER, PAD_SPACER, PAD_AREAIO,
        BLOCK, ENDCAP_PRE, ENDCAP_POST, RING, COVER
    } macro_class = CORE;

    double width    = 0;
    double height   = 0;
    double origin_x = 0;
    double origin_y = 0;

    std::string symmetry; // "X Y R90" etc.
    LefSite     site_ref;
    std::string source;   // USER, GENERATE, BLOCK

    std::vector<LefPin> pins;
    LefObs              obs;

    bool is_filler = false;
    std::unordered_map<std::string, std::string> properties;
};

// ── Inter-layer spacing rule ─────────────────────────────────────────────────

struct LefSpacing {
    std::string layer1;
    std::string layer2;
    double      min_spacing = 0;
    bool        stack       = false;
};

// ── Top-level library container ──────────────────────────────────────────────

struct LefLibrary {
    std::string version       = "5.8";
    std::string bus_bit_chars = "[]";
    std::string divider_char  = "/";

    LefUnits                units;
    std::vector<LefLayer>   layers;
    std::vector<LefVia>     vias;
    std::vector<LefViaRule> via_rules;
    std::vector<LefSite>    sites;
    std::vector<LefMacro>   macros;
    std::vector<LefSpacing> spacings;
    std::unordered_map<std::string, std::string> properties;

    const LefMacro* find_macro(const std::string& name) const;
    const LefLayer* find_layer(const std::string& name) const;
    const LefVia*   find_via(const std::string& name) const;
    const LefSite*  find_site(const std::string& name) const;
    int  num_routing_layers() const;
    int  num_cut_layers() const;
};

// ── Parser ───────────────────────────────────────────────────────────────────

class LefParser {
public:
    LefLibrary parse_file(const std::string& filename);
    LefLibrary parse_string(const std::string& content);

private:
    struct Token {
        std::string value;
        int         line = 0;
    };

    std::vector<Token> tokens_;
    size_t             pos_ = 0;

    // Tokenizer
    std::vector<Token> tokenize(const std::string& content);

    // Helpers
    const std::string& peek() const;
    const std::string& advance();
    bool               match(const std::string& expected);
    void               expect(const std::string& expected);
    double             next_double();
    int                next_int();
    void               skip_to_semicolon();
    void               skip_past_end(const std::string& block_name);
    [[noreturn]] void  error(const std::string& msg) const;

    // Section parsers
    void parse_units(LefLibrary& lib);
    void parse_layer(LefLibrary& lib);
    void parse_via(LefLibrary& lib);
    void parse_via_rule(LefLibrary& lib);
    void parse_site(LefLibrary& lib);
    void parse_macro(LefLibrary& lib);
    void parse_macro_pin(LefMacro& macro);
    void parse_macro_obs(LefMacro& macro);
    void parse_spacing(LefLibrary& lib);
    void parse_property(LefLibrary& lib);
};

} // namespace sf
