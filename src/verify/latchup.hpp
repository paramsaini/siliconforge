#pragma once
// SiliconForge — Latchup / Well-Tap Verification
// Ensures all active cells have sufficient well-tap coverage to prevent latchup.
// Reference: Troutman, "Latchup in CMOS Technology", Springer 1986

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct LatchupViolation {
    enum Type { TAP_SPACING, MISSING_GUARD_RING, SUBSTRATE_CONTACT };
    Type type;
    double x, y;
    double actual_distance;
    double max_allowed;
    std::string message;
};

struct LatchupResult {
    int total_cells = 0;
    int cells_covered = 0;
    int violations = 0;
    int warnings = 0;
    std::vector<LatchupViolation> details;
    bool clean = false;
    double worst_tap_distance = 0;
    std::string message;
};

class LatchupChecker {
public:
    LatchupChecker(const PhysicalDesign& pd) : pd_(pd) {}
    LatchupResult check(double max_tap_distance = 20.0);

private:
    const PhysicalDesign& pd_;
};

} // namespace sf
