// SiliconForge — Cell-Level LVS Implementation
// Extracts MOSFETs from poly-over-diffusion geometry and compares
// against schematic device lists.

#include "stdcell/cell_lvs.hpp"
#include "verify/drc.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace sf {

// ── Constructor ──────────────────────────────────────────────────────────

CellLvsChecker::CellLvsChecker(const CellLayout& layout,
                                const std::vector<CellLvsDevice>& schematic)
    : layout_(layout), schematic_(schematic) {}

// ── Device Extraction ────────────────────────────────────────────────────
// MOSFET recognition: a transistor exists wherever poly crosses diffusion.
// Gate length = poly width at intersection.
// Gate width  = diffusion height at intersection.
// PMOS if the intersection centroid lies inside an NWELL rectangle;
// NMOS otherwise (PWELL or no explicit well).

std::vector<CellLvsDevice> CellLvsChecker::extract_devices() const {
    std::vector<CellLvsDevice> devices;

    for (const auto& poly_lr : layout_.rects) {
        if (poly_lr.layer != DrcLayer::POLY) continue;

        for (const auto& diff_lr : layout_.rects) {
            if (diff_lr.layer != DrcLayer::DIFF) continue;

            // Compute intersection
            double ix0 = std::max(poly_lr.bounds.x0, diff_lr.bounds.x0);
            double iy0 = std::max(poly_lr.bounds.y0, diff_lr.bounds.y0);
            double ix1 = std::min(poly_lr.bounds.x1, diff_lr.bounds.x1);
            double iy1 = std::min(poly_lr.bounds.y1, diff_lr.bounds.y1);

            if (ix1 <= ix0 || iy1 <= iy0) continue; // no overlap

            double gate_length = ix1 - ix0; // poly width through active
            double gate_width  = iy1 - iy0; // active height through poly

            // Centroid for well classification
            double cx = (ix0 + ix1) / 2.0;
            double cy = (iy0 + iy1) / 2.0;

            bool in_nwell = point_in_layer(cx, cy, DrcLayer::NWELL);

            CellLvsDevice dev;
            dev.type = in_nwell ? CellLvsDevice::PMOS : CellLvsDevice::NMOS;
            dev.w = gate_width;
            dev.l = gate_length;

            // Terminal net assignment from layout pins
            Rect gate_region(ix0, iy0, ix1, iy1);
            dev.gate = net_at(DrcLayer::POLY, gate_region);

            // Source/drain: diffusion regions on either side of gate
            // Left side of gate = one terminal, right side = other
            Rect left_sd(diff_lr.bounds.x0, iy0, ix0, iy1);
            Rect right_sd(ix1, iy0, diff_lr.bounds.x1, iy1);

            dev.source = net_at(DrcLayer::DIFF, left_sd);
            dev.drain  = net_at(DrcLayer::DIFF, right_sd);
            dev.bulk   = in_nwell ? "VDD" : "VSS";

            devices.push_back(dev);
        }
    }

    return devices;
}

// ── Compare ──────────────────────────────────────────────────────────────

CellLvsResult CellLvsChecker::compare() const {
    auto t0 = std::chrono::high_resolution_clock::now();

    auto extracted = extract_devices();
    CellLvsResult result;
    result.layout_devices    = static_cast<int>(extracted.size());
    result.schematic_devices = static_cast<int>(schematic_.size());

    // Match by type and W/L (with tolerance)
    std::vector<bool> sch_matched(schematic_.size(), false);
    std::vector<bool> ext_matched(extracted.size(), false);

    for (size_t i = 0; i < extracted.size(); ++i) {
        for (size_t j = 0; j < schematic_.size(); ++j) {
            if (sch_matched[j]) continue;
            if (extracted[i].type != schematic_[j].type) continue;

            // W/L match with 10% tolerance
            double w_err = std::fabs(extracted[i].w - schematic_[j].w);
            double l_err = std::fabs(extracted[i].l - schematic_[j].l);
            double w_tol = 0.1 * std::max(extracted[i].w, schematic_[j].w);
            double l_tol = 0.1 * std::max(extracted[i].l, schematic_[j].l);

            if (w_tol < 1e-6) w_tol = 0.01; // absolute minimum tolerance
            if (l_tol < 1e-6) l_tol = 0.01;

            if (w_err <= w_tol && l_err <= l_tol) {
                sch_matched[j] = true;
                ext_matched[i] = true;
                result.matched++;
                break;
            }
        }
    }

    // Report mismatches
    for (size_t i = 0; i < extracted.size(); ++i) {
        if (!ext_matched[i]) {
            std::ostringstream oss;
            oss << "Extra layout device: "
                << (extracted[i].type == CellLvsDevice::PMOS ? "PMOS" : "NMOS")
                << " W=" << extracted[i].w << " L=" << extracted[i].l;
            result.mismatches.push_back(oss.str());
        }
    }
    for (size_t j = 0; j < schematic_.size(); ++j) {
        if (!sch_matched[j]) {
            std::ostringstream oss;
            oss << "Missing layout device: "
                << (schematic_[j].type == CellLvsDevice::PMOS ? "PMOS" : "NMOS")
                << " W=" << schematic_[j].w << " L=" << schematic_[j].l;
            result.mismatches.push_back(oss.str());
        }
    }

    result.match = (result.matched == result.schematic_devices &&
                    result.matched == result.layout_devices);

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

// ── Helpers ──────────────────────────────────────────────────────────────

std::string CellLvsChecker::net_at(int layer, const Rect& region) const {
    double cx = (region.x0 + region.x1) / 2.0;
    double cy = (region.y0 + region.y1) / 2.0;

    // Search pins for one that overlaps the region centroid
    for (const auto& pin : layout_.pins) {
        if (pin.layer != layer) continue;
        if (cx >= pin.bounds.x0 && cx <= pin.bounds.x1 &&
            cy >= pin.bounds.y0 && cy <= pin.bounds.y1) {
            return pin.name;
        }
    }

    // Fallback: generate a synthetic net name from coordinates
    std::ostringstream oss;
    oss << "net_" << static_cast<int>(cx * 100) << "_"
        << static_cast<int>(cy * 100);
    return oss.str();
}

bool CellLvsChecker::point_in_layer(double x, double y, int layer) const {
    for (const auto& lr : layout_.rects) {
        if (lr.layer != layer) continue;
        if (x >= lr.bounds.x0 && x <= lr.bounds.x1 &&
            y >= lr.bounds.y0 && y <= lr.bounds.y1) {
            return true;
        }
    }
    return false;
}

} // namespace sf
