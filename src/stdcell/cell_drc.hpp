#pragma once
// SiliconForge — Cell-Level Design Rule Check (DRC)
// Front-end DRC for standard cell layout validation against
// process design rules (poly, diffusion, contact, well, implant).
//
// References:
//   - SkyWater SKY130 PDK Design Rules (skywater-pdk.readthedocs.io)
//   - Weste & Harris, "CMOS VLSI Design", 4th ed., Ch. 3 (Layout)
//   - Mead & Conway, "Introduction to VLSI Systems" (design rule concepts)

#include "stdcell/cell_layout.hpp"
#include <string>
#include <vector>

namespace sf {

// ── Cell DRC Rule ────────────────────────────────────────────────────────

struct CellDrcRule {
    std::string name;
    int layer   = 0;       // primary layer
    int layer2  = -1;      // secondary layer (enclosure rules)
    double value = 0.0;    // rule value in um

    enum Type {
        MIN_WIDTH,
        MIN_SPACING,
        MIN_ENCLOSURE,
        POLY_ENDCAP,
        GATE_MIN_WIDTH,
        CONTACT_SIZE,
        CONTACT_SPACING,
        CONTACT_ENCLOSURE,
        WELL_ENCLOSURE,
        IMPLANT_ENCLOSURE,
        TAP_SPACING
    } type;
};

// ── Cell DRC Violation ───────────────────────────────────────────────────

struct CellDrcViolation {
    std::string rule_name;
    std::string description;
    Rect        location;
};

// ── Cell DRC Result ──────────────────────────────────────────────────────

struct CellDrcResult {
    bool clean = true;
    int  violations_count = 0;
    std::vector<CellDrcViolation> violations;
    double time_ms = 0.0;
};

// ── Cell DRC Checker ─────────────────────────────────────────────────────

class CellDrcChecker {
public:
    explicit CellDrcChecker(const CellLayout& layout);

    /// Load SKY130 front-end design rules (poly, diff, contact, well, implant).
    void load_sky130_rules();

    /// Add a custom rule.
    void add_rule(const CellDrcRule& rule) { rules_.push_back(rule); }

    /// Access loaded rules.
    const std::vector<CellDrcRule>& rules() const { return rules_; }

    /// Run all loaded rules against the cell layout.
    CellDrcResult check() const;

private:
    const CellLayout& layout_;
    std::vector<CellDrcRule> rules_;

    // Individual rule checkers
    void check_min_width(const CellDrcRule& rule, CellDrcResult& r) const;
    void check_min_spacing(const CellDrcRule& rule, CellDrcResult& r) const;
    void check_min_enclosure(const CellDrcRule& rule, CellDrcResult& r) const;
    void check_poly_endcap(const CellDrcRule& rule, CellDrcResult& r) const;
    void check_contact_size(const CellDrcRule& rule, CellDrcResult& r) const;
    void check_contact_spacing(const CellDrcRule& rule, CellDrcResult& r) const;

    // Geometry helpers
    static double rect_spacing(const Rect& a, const Rect& b);
    static bool   rects_overlap(const Rect& a, const Rect& b);
    static double enclosure(const Rect& inner, const Rect& outer);
};

} // namespace sf
