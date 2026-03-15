// SiliconForge — Cell-Level DRC Implementation
// Validates standard cell layouts against process design rules.

#include "stdcell/cell_drc.hpp"
#include "verify/drc.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace sf {

// ── Constructor ──────────────────────────────────────────────────────────

CellDrcChecker::CellDrcChecker(const CellLayout& layout)
    : layout_(layout) {}

// ── SKY130 Front-End Rules ───────────────────────────────────────────────
// Values derived from SkyWater SKY130 PDK design rule manual.
// Layer IDs follow DrcLayer::Id encoding.

void CellDrcChecker::load_sky130_rules() {
    rules_.clear();

    // Poly rules
    rules_.push_back({"poly.1",  DrcLayer::POLY, -1, 0.15,
                       CellDrcRule::MIN_WIDTH});
    rules_.push_back({"poly.2",  DrcLayer::POLY, -1, 0.21,
                       CellDrcRule::MIN_SPACING});
    rules_.push_back({"poly.8",  DrcLayer::POLY, DrcLayer::DIFF, 0.13,
                       CellDrcRule::POLY_ENDCAP});

    // Diffusion rules
    rules_.push_back({"diff.1",  DrcLayer::DIFF, -1, 0.15,
                       CellDrcRule::MIN_WIDTH});
    rules_.push_back({"diff.2",  DrcLayer::DIFF, -1, 0.27,
                       CellDrcRule::MIN_SPACING});

    // Gate (poly over diffusion) minimum width (channel length)
    rules_.push_back({"poly.1a", DrcLayer::POLY, -1, 0.15,
                       CellDrcRule::GATE_MIN_WIDTH});

    // LICON (local interconnect contact) rules
    rules_.push_back({"licon.1", DrcLayer::LICON, -1, 0.17,
                       CellDrcRule::CONTACT_SIZE});
    rules_.push_back({"licon.2", DrcLayer::LICON, -1, 0.17,
                       CellDrcRule::CONTACT_SPACING});
    rules_.push_back({"licon.5", DrcLayer::LICON, DrcLayer::DIFF, 0.04,
                       CellDrcRule::CONTACT_ENCLOSURE});

    // NWELL enclosure of DIFF (PMOS active)
    rules_.push_back({"nwell.5", DrcLayer::DIFF, DrcLayer::NWELL, 0.18,
                       CellDrcRule::WELL_ENCLOSURE});

    // NWELL minimum width
    rules_.push_back({"nwell.1", DrcLayer::NWELL, -1, 0.84,
                       CellDrcRule::MIN_WIDTH});

    // Implant enclosure
    rules_.push_back({"nsdm.1",  DrcLayer::DIFF, DrcLayer::NSDM, 0.125,
                       CellDrcRule::IMPLANT_ENCLOSURE});
    rules_.push_back({"psdm.1",  DrcLayer::DIFF, DrcLayer::PSDM, 0.125,
                       CellDrcRule::IMPLANT_ENCLOSURE});

    // TAP spacing (latchup prevention)
    rules_.push_back({"tap.1",   DrcLayer::TAP, -1, 15.0,
                       CellDrcRule::TAP_SPACING});
}

// ── Main Check Entry Point ───────────────────────────────────────────────

CellDrcResult CellDrcChecker::check() const {
    auto t0 = std::chrono::high_resolution_clock::now();
    CellDrcResult result;
    result.clean = true;

    for (const auto& rule : rules_) {
        switch (rule.type) {
        case CellDrcRule::MIN_WIDTH:
        case CellDrcRule::GATE_MIN_WIDTH:
            check_min_width(rule, result);
            break;
        case CellDrcRule::MIN_SPACING:
            check_min_spacing(rule, result);
            break;
        case CellDrcRule::MIN_ENCLOSURE:
        case CellDrcRule::WELL_ENCLOSURE:
        case CellDrcRule::IMPLANT_ENCLOSURE:
        case CellDrcRule::CONTACT_ENCLOSURE:
            check_min_enclosure(rule, result);
            break;
        case CellDrcRule::POLY_ENDCAP:
            check_poly_endcap(rule, result);
            break;
        case CellDrcRule::CONTACT_SIZE:
            check_contact_size(rule, result);
            break;
        case CellDrcRule::CONTACT_SPACING:
            check_contact_spacing(rule, result);
            break;
        case CellDrcRule::TAP_SPACING:
            // TAP spacing is a global rule; cell-level check is informational
            break;
        }
    }

    result.violations_count = static_cast<int>(result.violations.size());
    result.clean = (result.violations_count == 0);

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

// ── MIN_WIDTH / GATE_MIN_WIDTH ───────────────────────────────────────────

void CellDrcChecker::check_min_width(const CellDrcRule& rule,
                                      CellDrcResult& r) const {
    for (const auto& lr : layout_.rects) {
        if (lr.layer != rule.layer) continue;
        double w = lr.bounds.width();
        double h = lr.bounds.height();
        double min_dim = std::min(w, h);
        if (min_dim < rule.value - 1e-6) {
            std::ostringstream oss;
            oss << rule.name << ": width " << min_dim
                << " um < " << rule.value << " um";
            r.violations.push_back({rule.name, oss.str(), lr.bounds});
        }
    }
}

// ── MIN_SPACING ──────────────────────────────────────────────────────────

void CellDrcChecker::check_min_spacing(const CellDrcRule& rule,
                                        CellDrcResult& r) const {
    // Collect rects on this layer
    std::vector<const Rect*> layer_rects;
    for (const auto& lr : layout_.rects) {
        if (lr.layer == rule.layer) layer_rects.push_back(&lr.bounds);
    }

    for (size_t i = 0; i < layer_rects.size(); ++i) {
        for (size_t j = i + 1; j < layer_rects.size(); ++j) {
            double sp = rect_spacing(*layer_rects[i], *layer_rects[j]);
            // Only flag if rects are separate (sp > 0) but too close
            if (sp > 0.0 && sp < rule.value - 1e-6) {
                std::ostringstream oss;
                oss << rule.name << ": spacing " << sp
                    << " um < " << rule.value << " um";
                Rect viol_loc(
                    std::min(layer_rects[i]->x0, layer_rects[j]->x0),
                    std::min(layer_rects[i]->y0, layer_rects[j]->y0),
                    std::max(layer_rects[i]->x1, layer_rects[j]->x1),
                    std::max(layer_rects[i]->y1, layer_rects[j]->y1));
                r.violations.push_back({rule.name, oss.str(), viol_loc});
            }
        }
    }
}

// ── MIN_ENCLOSURE / WELL_ENCLOSURE / IMPLANT_ENCLOSURE ───────────────────

void CellDrcChecker::check_min_enclosure(const CellDrcRule& rule,
                                          CellDrcResult& r) const {
    // For each rect on layer, verify it is enclosed by some rect on layer2
    // by at least rule.value on all sides.
    //
    // For WELL_ENCLOSURE / IMPLANT_ENCLOSURE: only check inner rects that
    // actually overlap with the enclosing layer — e.g. NMOS DIFF does NOT
    // need NWELL enclosure, and PMOS DIFF does NOT need NSDM enclosure.
    bool selective = (rule.type == CellDrcRule::WELL_ENCLOSURE ||
                      rule.type == CellDrcRule::IMPLANT_ENCLOSURE);

    for (const auto& inner_lr : layout_.rects) {
        if (inner_lr.layer != rule.layer) continue;

        // Collect outer rects on layer2
        bool has_overlap = false;
        bool enclosed = false;
        double best_enc = -1e30;

        for (const auto& outer_lr : layout_.rects) {
            if (outer_lr.layer != rule.layer2) continue;
            double enc = enclosure(inner_lr.bounds, outer_lr.bounds);
            best_enc = std::max(best_enc, enc);

            // Check if inner rect overlaps with (or is inside) this outer rect
            if (rects_overlap(inner_lr.bounds, outer_lr.bounds))
                has_overlap = true;

            if (enc >= rule.value - 1e-6) {
                enclosed = true;
                break;
            }
        }

        // For selective rules, skip inner rects that don't overlap with
        // the enclosing layer at all (they belong to a different domain).
        if (selective && !has_overlap) continue;

        if (!enclosed && best_enc > -1e29) {
            std::ostringstream oss;
            oss << rule.name << ": enclosure " << best_enc
                << " um < " << rule.value << " um";
            r.violations.push_back({rule.name, oss.str(), inner_lr.bounds});
        }
    }
}

// ── POLY_ENDCAP ──────────────────────────────────────────────────────────
// Poly must extend past diffusion by at least rule.value.

void CellDrcChecker::check_poly_endcap(const CellDrcRule& rule,
                                        CellDrcResult& r) const {
    for (const auto& poly_lr : layout_.rects) {
        if (poly_lr.layer != DrcLayer::POLY) continue;

        for (const auto& diff_lr : layout_.rects) {
            if (diff_lr.layer != DrcLayer::DIFF) continue;

            // Check if poly crosses diffusion (forms a gate)
            if (!rects_overlap(poly_lr.bounds, diff_lr.bounds)) continue;

            // Poly must extend past diff top and bottom
            double ext_top    = poly_lr.bounds.y1 - diff_lr.bounds.y1;
            double ext_bottom = diff_lr.bounds.y0 - poly_lr.bounds.y0;

            if (ext_top < rule.value - 1e-6) {
                std::ostringstream oss;
                oss << rule.name << ": poly endcap (top) " << ext_top
                    << " um < " << rule.value << " um";
                r.violations.push_back({rule.name, oss.str(), poly_lr.bounds});
            }
            if (ext_bottom < rule.value - 1e-6) {
                std::ostringstream oss;
                oss << rule.name << ": poly endcap (bottom) " << ext_bottom
                    << " um < " << rule.value << " um";
                r.violations.push_back({rule.name, oss.str(), poly_lr.bounds});
            }
        }
    }
}

// ── CONTACT_SIZE ─────────────────────────────────────────────────────────

void CellDrcChecker::check_contact_size(const CellDrcRule& rule,
                                         CellDrcResult& r) const {
    for (const auto& lr : layout_.rects) {
        if (lr.layer != rule.layer) continue;
        double w = lr.bounds.width();
        double h = lr.bounds.height();
        if (w < rule.value - 1e-6 || h < rule.value - 1e-6) {
            std::ostringstream oss;
            oss << rule.name << ": contact size " << w << "x" << h
                << " um < " << rule.value << " um";
            r.violations.push_back({rule.name, oss.str(), lr.bounds});
        }
    }
}

// ── CONTACT_SPACING ──────────────────────────────────────────────────────

void CellDrcChecker::check_contact_spacing(const CellDrcRule& rule,
                                            CellDrcResult& r) const {
    std::vector<const Rect*> contacts;
    for (const auto& lr : layout_.rects) {
        if (lr.layer == rule.layer) contacts.push_back(&lr.bounds);
    }

    for (size_t i = 0; i < contacts.size(); ++i) {
        for (size_t j = i + 1; j < contacts.size(); ++j) {
            double sp = rect_spacing(*contacts[i], *contacts[j]);
            if (sp > 0.0 && sp < rule.value - 1e-6) {
                std::ostringstream oss;
                oss << rule.name << ": contact spacing " << sp
                    << " um < " << rule.value << " um";
                Rect viol_loc(
                    std::min(contacts[i]->x0, contacts[j]->x0),
                    std::min(contacts[i]->y0, contacts[j]->y0),
                    std::max(contacts[i]->x1, contacts[j]->x1),
                    std::max(contacts[i]->y1, contacts[j]->y1));
                r.violations.push_back({rule.name, oss.str(), viol_loc});
            }
        }
    }
}

// ── Geometry Helpers ─────────────────────────────────────────────────────

double CellDrcChecker::rect_spacing(const Rect& a, const Rect& b) {
    // Manhattan distance between two rectangles (0 if overlapping/touching)
    double dx = std::max(0.0, std::max(a.x0 - b.x1, b.x0 - a.x1));
    double dy = std::max(0.0, std::max(a.y0 - b.y1, b.y0 - a.y1));
    return std::max(dx, dy);
}

bool CellDrcChecker::rects_overlap(const Rect& a, const Rect& b) {
    return a.x0 < b.x1 && a.x1 > b.x0 && a.y0 < b.y1 && a.y1 > b.y0;
}

double CellDrcChecker::enclosure(const Rect& inner, const Rect& outer) {
    // Minimum enclosure of inner by outer on all four sides.
    // Negative means inner protrudes beyond outer.
    double left   = inner.x0 - outer.x0;
    double bottom = inner.y0 - outer.y0;
    double right  = outer.x1 - inner.x1;
    double top    = outer.y1 - inner.y1;
    return std::min({left, bottom, right, top});
}

} // namespace sf
