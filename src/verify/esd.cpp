// SiliconForge — ESD Verification Implementation
// Heuristic checks for IO pad protection in the absence of explicit ESD cells.
#include "verify/esd.hpp"
#include <cmath>
#include <algorithm>

namespace sf {

// Maximum distance from die boundary to count as IO-proximate (um)
static constexpr double IO_BOUNDARY_MARGIN = 10.0;
// Maximum distance from IO pad to nearest clamp/buffer cell (um)
static constexpr double MAX_CLAMP_DISTANCE = 50.0;

EsdResult EsdChecker::check() {
    EsdResult res;
    check_io_clamps(res);
    check_power_clamps(res);
    check_discharge_paths(res);

    res.clean = (res.violations == 0);
    res.message = "ESD: " + std::to_string(res.total_pads) + " pads, " +
                  std::to_string(res.protected_pads) + " protected, " +
                  std::to_string(res.violations) + " violations, " +
                  std::to_string(res.warnings) + " warnings";
    return res;
}

void EsdChecker::check_io_clamps(EsdResult& res) {
    // Identify IO pads from primary inputs/outputs
    // Each PI/PO net corresponds to an IO pad
    std::vector<std::string> io_names;
    for (auto pi : nl_.primary_inputs())
        io_names.push_back(nl_.net(pi).name);
    for (auto po : nl_.primary_outputs())
        io_names.push_back(nl_.net(po).name);

    res.total_pads = (int)io_names.size();

    // For each IO net, check that there is at least one buffer/gate cell
    // within IO_BOUNDARY_MARGIN of the die boundary
    double bx0 = pd_.die_area.x0, by0 = pd_.die_area.y0;
    double bx1 = pd_.die_area.x1, by1 = pd_.die_area.y1;

    for (auto& io_name : io_names) {
        bool has_nearby_gate = false;
        double min_dist_to_boundary = 1e18;

        for (auto& cell : pd_.cells) {
            if (!cell.placed) continue;
            // Skip infrastructure cells
            if (cell.cell_type == "FILL" || cell.cell_type == "ENDCAP" ||
                cell.cell_type == "WELLTAP") continue;

            double cx = cell.position.x + cell.width / 2.0;
            double cy = cell.position.y + cell.height / 2.0;

            // Distance to nearest die boundary
            double dist_boundary = std::min({cx - bx0, bx1 - cx, cy - by0, by1 - cy});

            if (dist_boundary <= IO_BOUNDARY_MARGIN) {
                has_nearby_gate = true;
                min_dist_to_boundary = std::min(min_dist_to_boundary, dist_boundary);
            }
        }

        if (has_nearby_gate) {
            res.protected_pads++;
        } else {
            // Check if there is ANY cell within MAX_CLAMP_DISTANCE of boundary
            bool has_any_near = false;
            for (auto& cell : pd_.cells) {
                if (!cell.placed) continue;
                if (cell.cell_type == "FILL" || cell.cell_type == "ENDCAP") continue;
                double cx = cell.position.x + cell.width / 2.0;
                double cy = cell.position.y + cell.height / 2.0;
                double dist_boundary = std::min({cx - bx0, bx1 - cx, cy - by0, by1 - cy});
                if (dist_boundary <= MAX_CLAMP_DISTANCE) {
                    has_any_near = true;
                    break;
                }
            }

            if (has_any_near) {
                // Warning — there's something nearby but not close enough
                res.warnings++;
                res.details.push_back({
                    EsdViolation::MISSING_CLAMP,
                    io_name,
                    "IO pad '" + io_name + "' has no clamp cell within " +
                    std::to_string(IO_BOUNDARY_MARGIN) + "um of die boundary (warning)"
                });
            } else {
                res.violations++;
                res.details.push_back({
                    EsdViolation::MISSING_CLAMP,
                    io_name,
                    "IO pad '" + io_name + "' has no ESD clamp cell within " +
                    std::to_string(MAX_CLAMP_DISTANCE) + "um of die boundary"
                });
            }
        }
    }
}

void EsdChecker::check_power_clamps(EsdResult& res) {
    // Check that power-to-ground clamp paths exist
    // Heuristic: at least one WELLTAP cell must exist (acts as substrate contact)
    bool has_welltap = false;
    for (auto& cell : pd_.cells) {
        if (cell.cell_type == "WELLTAP") {
            has_welltap = true;
            break;
        }
    }

    if (!has_welltap && !pd_.cells.empty()) {
        res.violations++;
        res.details.push_back({
            EsdViolation::NO_POWER_CLAMP,
            "VDD/VSS",
            "No well-tap cells found — missing power-to-ground ESD clamp path"
        });
    }
}

void EsdChecker::check_discharge_paths(EsdResult& res) {
    // Compute worst-case discharge resistance heuristic
    // Model: sheet resistance of metal grid × distance from farthest IO to nearest tap
    if (pd_.cells.empty()) return;

    double sheet_r = 0.05; // ohm per um (simplified)
    double worst_r = 0;

    double bx0 = pd_.die_area.x0, by0 = pd_.die_area.y0;
    double bx1 = pd_.die_area.x1, by1 = pd_.die_area.y1;

    // Find all welltap positions
    std::vector<Point> taps;
    for (auto& cell : pd_.cells) {
        if (cell.cell_type == "WELLTAP" && cell.placed) {
            taps.push_back({cell.position.x + cell.width / 2.0,
                            cell.position.y + cell.height / 2.0});
        }
    }

    if (taps.empty()) {
        // No taps — infinite resistance
        worst_r = 1e6;
        res.worst_resistance_ohm = worst_r;
        res.violations++;
        res.details.push_back({
            EsdViolation::NO_DISCHARGE_PATH,
            "global",
            "No discharge path — no well-tap cells for substrate contact"
        });
        return;
    }

    // Check corner points of die (farthest potential IO locations)
    std::vector<Point> corners = {
        {bx0, by0}, {bx1, by0}, {bx0, by1}, {bx1, by1}
    };

    for (auto& corner : corners) {
        double min_dist = 1e18;
        for (auto& tap : taps) {
            double d = std::abs(corner.x - tap.x) + std::abs(corner.y - tap.y);
            min_dist = std::min(min_dist, d);
        }
        double r = min_dist * sheet_r;
        worst_r = std::max(worst_r, r);
    }

    res.worst_resistance_ohm = worst_r;

    // Flag if worst-case resistance is too high (>5 ohm)
    if (worst_r > 5.0) {
        res.warnings++;
        res.details.push_back({
            EsdViolation::HIGH_RESISTANCE_PATH,
            "global",
            "High discharge resistance: " + std::to_string(worst_r) +
            " ohm (threshold: 5.0 ohm)"
        });
    }
}

} // namespace sf
