#pragma once
// SiliconForge — Design Rule Check (DRC) Engine
// Validates physical layout against manufacturing rules.
// Reference: "DRC Rule Deck", Foundry PDK documentation

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct DrcRule {
    std::string name;
    std::string description;
    double value;          // rule value in um
    enum Type { MIN_WIDTH, MIN_SPACING, MIN_AREA, MIN_ENCLOSURE,
                MAX_WIDTH, DENSITY_MIN, DENSITY_MAX } type;
    int layer;
};

struct DrcViolation {
    std::string rule_name;
    std::string message;
    Rect bbox;             // bounding box of violation
    double actual_value;
    double required_value;
    enum Severity { ERROR, WARNING } severity;
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

    // Add design rules
    void add_rule(const DrcRule& rule) { rules_.push_back(rule); }
    void load_default_rules(double min_feature_um = 0.13);

    // Run all DRC checks
    DrcResult check();

private:
    const PhysicalDesign& pd_;
    std::vector<DrcRule> rules_;

    void check_min_width(DrcResult& r);
    void check_min_spacing(DrcResult& r);
    void check_min_area(DrcResult& r);
    void check_boundary(DrcResult& r);
    void check_density(DrcResult& r);
};

} // namespace sf
