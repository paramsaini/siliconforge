// SiliconForge — DRC Engine Implementation
#include "verify/drc.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <algorithm>

namespace sf {

void DrcEngine::load_default_rules(double min_feature) {
    rules_.clear();
    // Standard CMOS DRC rules scaled by min_feature
    rules_.push_back({"M1.W.1", "Metal1 minimum width", min_feature, DrcRule::MIN_WIDTH, 1});
    rules_.push_back({"M1.S.1", "Metal1 minimum spacing", min_feature * 1.2, DrcRule::MIN_SPACING, 1});
    rules_.push_back({"M1.A.1", "Metal1 minimum area", min_feature * min_feature * 4, DrcRule::MIN_AREA, 1});
    rules_.push_back({"CELL.W.1", "Cell minimum width", min_feature * 2, DrcRule::MIN_WIDTH, 0});
    rules_.push_back({"CELL.S.1", "Cell minimum spacing", min_feature * 0.5, DrcRule::MIN_SPACING, 0});
    rules_.push_back({"DIE.B.1", "Cells within die boundary", 0, DrcRule::MIN_ENCLOSURE, 0});
    rules_.push_back({"DENS.MIN", "Minimum metal density 20%", 0.20, DrcRule::DENSITY_MIN, 1});
    rules_.push_back({"DENS.MAX", "Maximum metal density 80%", 0.80, DrcRule::DENSITY_MAX, 1});
}

void DrcEngine::check_min_width(DrcResult& r) {
    for (auto& rule : rules_) {
        if (rule.type != DrcRule::MIN_WIDTH) continue;
        r.total_rules++;

        if (rule.layer == 0) { // Cell layer
            for (auto& c : pd_.cells) {
                if (!c.placed) continue;
                if (c.width < rule.value) {
                    r.violations++;
                    r.errors++;
                    Rect bbox(c.position.x, c.position.y,
                             c.position.x + c.width, c.position.y + c.height);
                    r.details.push_back({rule.name,
                        "Cell '" + c.name + "' width " + std::to_string(c.width) +
                        " < min " + std::to_string(rule.value),
                        bbox, c.width, rule.value, DrcViolation::ERROR});
                }
            }
        } else { // Wire layer
            for (auto& w : pd_.wires) {
                if (w.layer != rule.layer) continue;
                if (w.width < rule.value) {
                    r.violations++;
                    r.errors++;
                    Rect bbox(std::min(w.start.x, w.end.x), std::min(w.start.y, w.end.y),
                             std::max(w.start.x, w.end.x), std::max(w.start.y, w.end.y));
                    r.details.push_back({rule.name,
                        "Wire width " + std::to_string(w.width) + " < min " + std::to_string(rule.value),
                        bbox, w.width, rule.value, DrcViolation::ERROR});
                }
            }
        }
    }
}

void DrcEngine::check_min_spacing(DrcResult& r) {
    for (auto& rule : rules_) {
        if (rule.type != DrcRule::MIN_SPACING) continue;
        r.total_rules++;

        if (rule.layer == 0) { // Cell spacing
            for (size_t i = 0; i < pd_.cells.size(); ++i) {
                if (!pd_.cells[i].placed) continue;
                for (size_t j = i+1; j < pd_.cells.size(); ++j) {
                    if (!pd_.cells[j].placed) continue;
                    auto& a = pd_.cells[i]; auto& b = pd_.cells[j];
                    // Check if on same row
                    if (std::abs(a.position.y - b.position.y) < 0.01) {
                        double gap;
                        if (a.position.x < b.position.x)
                            gap = b.position.x - (a.position.x + a.width);
                        else
                            gap = a.position.x - (b.position.x + b.width);

                        if (gap > 0 && gap < rule.value) {
                            r.violations++;
                            r.warnings++;
                            r.details.push_back({rule.name,
                                "Spacing " + std::to_string(gap) + " between '" +
                                a.name + "' and '" + b.name + "' < min " + std::to_string(rule.value),
                                {}, gap, rule.value, DrcViolation::WARNING});
                        }
                    }
                }
            }
        }
    }
}

void DrcEngine::check_min_area(DrcResult& r) {
    for (auto& rule : rules_) {
        if (rule.type != DrcRule::MIN_AREA) continue;
        r.total_rules++;
        for (auto& c : pd_.cells) {
            double area = c.width * c.height;
            if (area < rule.value) {
                r.violations++;
                r.warnings++;
                r.details.push_back({rule.name,
                    "Cell '" + c.name + "' area " + std::to_string(area) +
                    " < min " + std::to_string(rule.value),
                    {}, area, rule.value, DrcViolation::WARNING});
            }
        }
    }
}

void DrcEngine::check_boundary(DrcResult& r) {
    for (auto& rule : rules_) {
        if (rule.type != DrcRule::MIN_ENCLOSURE) continue;
        r.total_rules++;
        for (auto& c : pd_.cells) {
            if (!c.placed) continue;
            Rect cr(c.position.x, c.position.y,
                   c.position.x + c.width, c.position.y + c.height);
            if (cr.x0 < pd_.die_area.x0 || cr.y0 < pd_.die_area.y0 ||
                cr.x1 > pd_.die_area.x1 || cr.y1 > pd_.die_area.y1) {
                r.violations++;
                r.errors++;
                r.details.push_back({rule.name,
                    "Cell '" + c.name + "' extends outside die boundary",
                    cr, 0, 0, DrcViolation::ERROR});
            }
        }
    }
}

void DrcEngine::check_density(DrcResult& r) {
    for (auto& rule : rules_) {
        if (rule.type != DrcRule::DENSITY_MIN && rule.type != DrcRule::DENSITY_MAX) continue;
        r.total_rules++;
        double util = pd_.utilization();
        if (rule.type == DrcRule::DENSITY_MIN && util < rule.value) {
            r.violations++;
            r.warnings++;
            r.details.push_back({rule.name,
                "Utilization " + std::to_string(util*100) + "% < min " +
                std::to_string(rule.value*100) + "%",
                pd_.die_area, util, rule.value, DrcViolation::WARNING});
        }
        if (rule.type == DrcRule::DENSITY_MAX && util > rule.value) {
            r.violations++;
            r.errors++;
            r.details.push_back({rule.name,
                "Utilization " + std::to_string(util*100) + "% > max " +
                std::to_string(rule.value*100) + "%",
                pd_.die_area, util, rule.value, DrcViolation::ERROR});
        }
    }
}

DrcResult DrcEngine::check() {
    auto t0 = std::chrono::high_resolution_clock::now();
    DrcResult r;

    check_min_width(r);
    check_min_spacing(r);
    check_min_area(r);
    check_boundary(r);
    check_density(r);

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = r.violations == 0
        ? "DRC CLEAN — " + std::to_string(r.total_rules) + " rules checked"
        : std::to_string(r.violations) + " violations (" +
          std::to_string(r.errors) + " errors, " +
          std::to_string(r.warnings) + " warnings)";
    return r;
}

} // namespace sf
