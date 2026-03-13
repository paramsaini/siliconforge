// SiliconForge — Latchup / Well-Tap Verification Implementation
#include "verify/latchup.hpp"
#include <cmath>
#include <algorithm>

namespace sf {

LatchupResult LatchupChecker::check(double max_tap_distance) {
    LatchupResult res;

    // Collect well-tap positions
    std::vector<Point> taps;
    for (auto& cell : pd_.cells) {
        if (cell.cell_type == "WELLTAP" && cell.placed) {
            taps.push_back({cell.position.x + cell.width / 2.0,
                            cell.position.y + cell.height / 2.0});
        }
    }

    // Check each non-infrastructure cell
    for (auto& cell : pd_.cells) {
        if (!cell.placed) continue;
        // Skip infrastructure cells
        if (cell.cell_type == "FILL" || cell.cell_type == "ENDCAP" ||
            cell.cell_type == "WELLTAP" || cell.cell_type == "DECAP") continue;

        res.total_cells++;

        double cx = cell.position.x + cell.width / 2.0;
        double cy = cell.position.y + cell.height / 2.0;

        // Find distance to nearest well-tap
        double min_dist = 1e18;
        for (auto& tap : taps) {
            double d = std::abs(cx - tap.x) + std::abs(cy - tap.y);
            min_dist = std::min(min_dist, d);
        }

        res.worst_tap_distance = std::max(res.worst_tap_distance, min_dist);

        if (min_dist <= max_tap_distance) {
            res.cells_covered++;
        } else if (min_dist <= max_tap_distance * 1.5) {
            // Warning zone — between max and 1.5× max
            res.warnings++;
            res.details.push_back({
                LatchupViolation::TAP_SPACING,
                cx, cy,
                min_dist,
                max_tap_distance,
                "Cell '" + cell.name + "' at (" + std::to_string(cx) + "," +
                std::to_string(cy) + ") tap distance " +
                std::to_string(min_dist) + "um > " +
                std::to_string(max_tap_distance) + "um (warning)"
            });
            res.cells_covered++; // Still marginally covered
        } else {
            res.violations++;
            res.details.push_back({
                LatchupViolation::TAP_SPACING,
                cx, cy,
                min_dist,
                max_tap_distance,
                "Cell '" + cell.name + "' at (" + std::to_string(cx) + "," +
                std::to_string(cy) + ") tap distance " +
                std::to_string(min_dist) + "um > " +
                std::to_string(max_tap_distance) + "um — LATCHUP RISK"
            });
        }
    }

    // If no taps at all, every active cell is a violation
    if (taps.empty() && res.total_cells > 0) {
        res.details.push_back({
            LatchupViolation::SUBSTRATE_CONTACT,
            0, 0, 0, max_tap_distance,
            "No WELLTAP cells found — all " + std::to_string(res.total_cells) +
            " active cells lack substrate contact"
        });
    }

    res.clean = (res.violations == 0);
    double coverage_pct = res.total_cells > 0 ?
        100.0 * res.cells_covered / res.total_cells : 100.0;
    res.message = "Latchup: " + std::to_string(res.total_cells) + " cells, " +
                  std::to_string(res.cells_covered) + " covered (" +
                  std::to_string((int)coverage_pct) + "%), " +
                  std::to_string(res.violations) + " violations, " +
                  "worst tap dist: " + std::to_string(res.worst_tap_distance) + "um";
    return res;
}

} // namespace sf
