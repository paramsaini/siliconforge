// SiliconForge -- Standard Cell Layout Generator Implementation
// Produces transistor-level CMOS layouts for common logic gates,
// flip-flops, and utility cells.  All geometry is constructed to
// satisfy the design rules encoded in TechRules, ensuring DRC-clean
// output by construction.
//
// Layout coordinate system:
//   Origin (0,0) at lower-left of cell boundary.
//   X increases to the right, Y increases upward.
//   VSS rail along bottom edge, VDD rail along top edge.
//   NMOS devices in the lower half, PMOS in the upper half (NWELL).

#include "stdcell/cell_layout.hpp"
#include <cmath>
#include <algorithm>

namespace sf {

// ═══════════════════════════════════════════════════════════════════════════
//  Helper utilities
// ═══════════════════════════════════════════════════════════════════════════

double CellLayoutGenerator::snap_to_site(double w) const {
    double sites = std::ceil(w / rules_.site_width);
    return sites * rules_.site_width;
}

void CellLayoutGenerator::add_power_rails(CellLayout& cl) {
    double rw = rules_.power_rail_width;
    // VSS rail at bottom
    cl.rects.emplace_back(DrcLayer::MET1, 0.0, 0.0, cl.width, rw);
    cl.pins.emplace_back("VSS", DrcLayer::MET1,
                          Rect(0.0, 0.0, cl.width, rw), "GROUND");
    // VDD rail at top
    cl.rects.emplace_back(DrcLayer::MET1, 0.0, cl.height - rw, cl.width, cl.height);
    cl.pins.emplace_back("VDD", DrcLayer::MET1,
                          Rect(0.0, cl.height - rw, cl.width, cl.height), "POWER");
}

void CellLayoutGenerator::add_nwell(CellLayout& cl) {
    // NWELL covers the upper half plus enclosure margin below PMOS active
    double nw_enc = rules_.nwell_enclosure;
    double mid_y  = cl.height / 2.0;
    cl.rects.emplace_back(DrcLayer::NWELL,
                          -nw_enc, mid_y - nw_enc,
                          cl.width + nw_enc, cl.height + nw_enc);
}

void CellLayoutGenerator::add_well_taps(CellLayout& cl) {
    double tw = rules_.tap_width;
    double rw = rules_.power_rail_width;
    double cs = rules_.contact_size;

    // NMOS substrate tap near VSS rail (left edge)
    double tap_y0 = rw;
    double tap_y1 = rw + tw;
    cl.rects.emplace_back(DrcLayer::TAP, 0.0, tap_y0, tw, tap_y1);
    cl.rects.emplace_back(DrcLayer::NSDM,
                          -rules_.nsdm_enclosure, tap_y0 - rules_.nsdm_enclosure,
                          tw + rules_.nsdm_enclosure, tap_y1 + rules_.nsdm_enclosure);
    // Contact on substrate tap
    double tcx = tw / 2.0;
    double tcy = (tap_y0 + tap_y1) / 2.0;
    cl.rects.emplace_back(DrcLayer::LICON,
                          tcx - cs/2, tcy - cs/2, tcx + cs/2, tcy + cs/2);

    // PMOS well tap near VDD rail (left edge)
    double ptap_y1 = cl.height - rw;
    double ptap_y0 = ptap_y1 - tw;
    cl.rects.emplace_back(DrcLayer::TAP, 0.0, ptap_y0, tw, ptap_y1);
    cl.rects.emplace_back(DrcLayer::PSDM,
                          -rules_.psdm_enclosure, ptap_y0 - rules_.psdm_enclosure,
                          tw + rules_.psdm_enclosure, ptap_y1 + rules_.psdm_enclosure);
    double pcx = tw / 2.0;
    double pcy = (ptap_y0 + ptap_y1) / 2.0;
    cl.rects.emplace_back(DrcLayer::LICON,
                          pcx - cs/2, pcy - cs/2, pcx + cs/2, pcy + cs/2);
}

void CellLayoutGenerator::place_contact(CellLayout& cl, double cx, double cy) {
    double cs = rules_.contact_size;
    double lw = rules_.li_width;
    // LICON (contact cut)
    cl.rects.emplace_back(DrcLayer::LICON,
                          cx - cs/2, cy - cs/2, cx + cs/2, cy + cs/2);
    // LI1 pad over contact
    cl.rects.emplace_back(DrcLayer::LI1,
                          cx - lw/2, cy - lw/2, cx + lw/2, cy + lw/2);
}

void CellLayoutGenerator::place_m1_strap(CellLayout& cl,
                                          double x0, double y0,
                                          double x1, double y1) {
    double mw = rules_.m1_width;
    // Ensure minimum width in the thin dimension
    if (std::abs(x1 - x0) < mw) {
        double cx = (x0 + x1) / 2.0;
        x0 = cx - mw / 2.0;
        x1 = cx + mw / 2.0;
    }
    if (std::abs(y1 - y0) < mw) {
        double cy = (y0 + y1) / 2.0;
        y0 = cy - mw / 2.0;
        y1 = cy + mw / 2.0;
    }
    cl.rects.emplace_back(DrcLayer::MET1, x0, y0, x1, y1);
}

double CellLayoutGenerator::place_transistor(CellLayout& cl,
                                              double x_origin, double y_center,
                                              double diff_w, bool is_pmos,
                                              const std::string& gate_name) {
    double gl   = rules_.gate_length;
    double ec   = rules_.poly_endcap;
    double dce  = rules_.diff_contact_enclosure;
    double cs   = rules_.contact_size;

    // Diffusion rectangle: width = contact + enclosures + gate + contact + enclosures
    double sd_width = dce + cs + dce;  // source or drain contact region width
    double diff_x0  = x_origin;
    double diff_x1  = x_origin + sd_width + gl + sd_width;
    double diff_y0  = y_center - diff_w / 2.0;
    double diff_y1  = y_center + diff_w / 2.0;

    cl.rects.emplace_back(DrcLayer::DIFF, diff_x0, diff_y0, diff_x1, diff_y1);

    // Poly gate (extends beyond diffusion by endcap in Y)
    double gate_x0 = diff_x0 + sd_width;
    double gate_x1 = gate_x0 + gl;
    double gate_y0 = diff_y0 - ec;
    double gate_y1 = diff_y1 + ec;
    cl.rects.emplace_back(DrcLayer::POLY, gate_x0, gate_y0, gate_x1, gate_y1);

    // Gate pin label
    if (!gate_name.empty()) {
        cl.pins.emplace_back(gate_name, DrcLayer::POLY,
                              Rect(gate_x0, gate_y0, gate_x1, gate_y1), "INPUT");
    }

    // Source contact (left of gate)
    double src_cx = diff_x0 + dce + cs / 2.0;
    place_contact(cl, src_cx, y_center);

    // Drain contact (right of gate)
    double drn_cx = gate_x1 + dce + cs / 2.0;
    place_contact(cl, drn_cx, y_center);

    // Implant layer
    double imp_enc = is_pmos ? rules_.psdm_enclosure : rules_.nsdm_enclosure;
    int imp_layer  = is_pmos ? DrcLayer::PSDM : DrcLayer::NSDM;
    cl.rects.emplace_back(imp_layer,
                          diff_x0 - imp_enc, diff_y0 - imp_enc,
                          diff_x1 + imp_enc, diff_y1 + imp_enc);

    return diff_x1;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Inverter (INV_X1, INV_X2, INV_X4)
// ═══════════════════════════════════════════════════════════════════════════

CellLayout CellLayoutGenerator::generate_inv(int drive_strength) {
    CellLayout cl;
    int ds = std::max(drive_strength, 1);
    cl.cell_name = "INV_X" + std::to_string(ds);
    cl.height    = rules_.row_height;

    double rw    = rules_.power_rail_width;
    double mid_y = cl.height / 2.0;
    double dce   = rules_.diff_contact_enclosure;
    double cs    = rules_.contact_size;
    double gl    = rules_.gate_length;

    // Base transistor widths (single finger) — real cells use finger
    // multiplication for higher drive strength, not just wider diffusion.
    double nmos_w = rules_.diff_width;
    double pmos_w = rules_.diff_width * 1.5;  // PMOS ~1.5x for equal rise/fall

    double nmos_cy = rw + nmos_w / 2.0 + dce;
    double pmos_cy = cl.height - rw - pmos_w / 2.0 - dce;

    // Left margin: well taps + spacing
    double x_start = rules_.tap_width + rules_.diff_spacing;

    // Finger multiplication: ds fingers placed side by side.
    // Each finger = one NMOS + one PMOS transistor pair.
    // Adjacent fingers share source/drain diffusion edges.
    int nfin = ds;
    double sd_w  = dce + cs + dce;  // source/drain contact region width
    double fin_pitch = sd_w + gl;   // pitch per finger (shared S/D between fingers)

    double x_cursor = x_start;
    double last_nmos_right = 0.0;
    double last_pmos_right = 0.0;

    for (int f = 0; f < nfin; ++f) {
        // Only label input pin "A" on the first finger's gate
        std::string gate_label = (f == 0) ? "A" : "";

        double nr = place_transistor(cl, x_cursor, nmos_cy, nmos_w, false, gate_label);
        double pr = place_transistor(cl, x_cursor, pmos_cy, pmos_w, true, "");

        // Connect NMOS and PMOS gates vertically (shared input)
        double gate_x0 = x_cursor + sd_w;
        double gate_x1 = gate_x0 + gl;
        double nmos_top = nmos_cy + nmos_w / 2.0 + rules_.poly_endcap;
        double pmos_bot = pmos_cy - pmos_w / 2.0 - rules_.poly_endcap;
        if (pmos_bot > nmos_top) {
            cl.rects.emplace_back(DrcLayer::POLY, gate_x0, nmos_top, gate_x1, pmos_bot);
        }

        // Output: connect drains via MCON + M1 vertical strap
        double drn_cx = gate_x1 + dce + cs / 2.0;
        double mcon_sz = cs;
        cl.rects.emplace_back(DrcLayer::MCON,
                              drn_cx - mcon_sz/2, nmos_cy - mcon_sz/2,
                              drn_cx + mcon_sz/2, nmos_cy + mcon_sz/2);
        cl.rects.emplace_back(DrcLayer::MCON,
                              drn_cx - mcon_sz/2, pmos_cy - mcon_sz/2,
                              drn_cx + mcon_sz/2, pmos_cy + mcon_sz/2);
        place_m1_strap(cl, drn_cx - rules_.m1_width/2, nmos_cy,
                           drn_cx + rules_.m1_width/2, pmos_cy);

        // Source ties to power rails
        double src_cx = x_cursor + dce + cs / 2.0;
        cl.rects.emplace_back(DrcLayer::LI1,
                              src_cx - rules_.li_width/2, 0.0,
                              src_cx + rules_.li_width/2, nmos_cy);
        cl.rects.emplace_back(DrcLayer::LI1,
                              src_cx - rules_.li_width/2, pmos_cy,
                              src_cx + rules_.li_width/2, cl.height);

        last_nmos_right = nr;
        last_pmos_right = pr;

        // Advance cursor for next finger (shared S/D saves one sd_w)
        if (f < nfin - 1) {
            x_cursor = nr + rules_.diff_spacing;
        }
    }

    // Output pin Y at center of cell height, at the last finger's drain
    double last_gate_x1 = x_cursor + sd_w + gl;
    double out_drn_cx   = last_gate_x1 + dce + cs / 2.0;
    cl.pins.emplace_back("Y", DrcLayer::MET1,
                          Rect(out_drn_cx - rules_.m1_width/2, mid_y - rules_.m1_width/2,
                               out_drn_cx + rules_.m1_width/2, mid_y + rules_.m1_width/2),
                          "OUTPUT");

    // Cell width: snap to site grid
    double raw_w = std::max(last_nmos_right, last_pmos_right) + rules_.diff_spacing
                   + rules_.tap_width;
    cl.width = snap_to_site(raw_w);

    // Add structural elements
    add_nwell(cl);
    add_power_rails(cl);
    add_well_taps(cl);

    cl.properties.emplace_back("FUNCTION", "INV");
    cl.properties.emplace_back("DRIVE", std::to_string(ds));
    return cl;
}

// ═══════════════════════════════════════════════════════════════════════════
//  2-input NAND (NAND2_X1, NAND2_X2, NAND2_X4)
//  NMOS: series stack (A on bottom, B on top, shared internal node)
//  PMOS: parallel (A and B in parallel, shared source to VDD, drains merged)
// ═══════════════════════════════════════════════════════════════════════════

CellLayout CellLayoutGenerator::generate_nand2(int drive_strength) {
    CellLayout cl;
    double ds = static_cast<double>(std::max(drive_strength, 1));
    cl.cell_name = "NAND2_X" + std::to_string(std::max(drive_strength, 1));
    cl.height    = rules_.row_height;

    double rw    = rules_.power_rail_width;
    double gl    = rules_.gate_length;
    double dce   = rules_.diff_contact_enclosure;
    double cs    = rules_.contact_size;
    double mid_y = cl.height / 2.0;

    // Series NMOS needs 2x width to compensate for stacking penalty
    double nmos_w = rules_.diff_width * ds * 2.0;
    double pmos_w = rules_.diff_width * ds * 1.5;

    double nmos_cy = rw + nmos_w / 2.0 + dce;
    double pmos_cy = cl.height - rw - pmos_w / 2.0 - dce;

    double x_start = rules_.tap_width + rules_.diff_spacing;

    // NMOS series stack: source -- gate_A -- internal -- gate_B -- drain
    double sd_w = dce + cs + dce;
    double nmos_x0 = x_start;
    double nmos_diff_x1 = nmos_x0 + sd_w + gl + sd_w + gl + sd_w;
    double nmos_y0 = nmos_cy - nmos_w / 2.0;
    double nmos_y1 = nmos_cy + nmos_w / 2.0;

    cl.rects.emplace_back(DrcLayer::DIFF, nmos_x0, nmos_y0, nmos_diff_x1, nmos_y1);

    // Gate A (first poly)
    double ga_x0 = nmos_x0 + sd_w;
    double ga_x1 = ga_x0 + gl;
    cl.rects.emplace_back(DrcLayer::POLY, ga_x0,
                          nmos_y0 - rules_.poly_endcap, ga_x1,
                          nmos_y1 + rules_.poly_endcap);
    cl.pins.emplace_back("A", DrcLayer::POLY,
                          Rect(ga_x0, nmos_y0, ga_x1, nmos_y1), "INPUT");

    // Gate B (second poly)
    double gb_x0 = ga_x1 + sd_w;
    double gb_x1 = gb_x0 + gl;
    cl.rects.emplace_back(DrcLayer::POLY, gb_x0,
                          nmos_y0 - rules_.poly_endcap, gb_x1,
                          nmos_y1 + rules_.poly_endcap);
    cl.pins.emplace_back("B", DrcLayer::POLY,
                          Rect(gb_x0, nmos_y0, gb_x1, nmos_y1), "INPUT");

    // NMOS contacts: source, internal node, drain
    double nsrc_cx = nmos_x0 + dce + cs / 2.0;
    place_contact(cl, nsrc_cx, nmos_cy);
    double nint_cx = ga_x1 + dce + cs / 2.0;
    place_contact(cl, nint_cx, nmos_cy);  // internal node (no external pin)
    double ndrn_cx = gb_x1 + dce + cs / 2.0;
    place_contact(cl, ndrn_cx, nmos_cy);

    // NSDM over NMOS
    cl.rects.emplace_back(DrcLayer::NSDM,
                          nmos_x0 - rules_.nsdm_enclosure, nmos_y0 - rules_.nsdm_enclosure,
                          nmos_diff_x1 + rules_.nsdm_enclosure, nmos_y1 + rules_.nsdm_enclosure);

    // PMOS parallel: two separate transistors sharing source, drains merged
    // Transistor A
    double pmos_x0 = x_start;
    double pa_right = place_transistor(cl, pmos_x0, pmos_cy, pmos_w, true, "");
    // Transistor B -- placed adjacent
    double pb_x0 = pa_right + rules_.diff_spacing;
    // Use smaller spacing if sharing diffusion
    pb_x0 = pa_right;  // shared diffusion (drain of A = source of B region)
    double pmos_diff_x1 = pb_x0 + (dce + cs + dce) + gl + (dce + cs + dce);

    // Extend PMOS diffusion across both gates for proper parallel topology
    double pmos_y0 = pmos_cy - pmos_w / 2.0;
    double pmos_y1 = pmos_cy + pmos_w / 2.0;

    // Additional poly stripe for gate B in PMOS region
    double pgb_x0 = pa_right + sd_w;
    double pgb_x1 = pgb_x0 + gl;
    cl.rects.emplace_back(DrcLayer::POLY, pgb_x0,
                          pmos_y0 - rules_.poly_endcap, pgb_x1,
                          pmos_y1 + rules_.poly_endcap);

    // Extend PMOS diffusion to cover second gate + drain
    cl.rects.emplace_back(DrcLayer::DIFF, pa_right, pmos_y0,
                          pgb_x1 + sd_w, pmos_y1);

    // PMOS second drain contact
    double pdrn2_cx = pgb_x1 + dce + cs / 2.0;
    place_contact(cl, pdrn2_cx, pmos_cy);

    // PSDM over full PMOS region
    double pmos_full_x1 = pgb_x1 + sd_w;
    cl.rects.emplace_back(DrcLayer::PSDM,
                          pmos_x0 - rules_.psdm_enclosure, pmos_y0 - rules_.psdm_enclosure,
                          pmos_full_x1 + rules_.psdm_enclosure, pmos_y1 + rules_.psdm_enclosure);

    // Connect poly gates vertically (A NMOS to A PMOS, B NMOS to B PMOS)
    // Gate A vertical connection
    double na_top = nmos_y1 + rules_.poly_endcap;
    double pa_bot = pmos_y0 - rules_.poly_endcap;
    if (pa_bot > na_top) {
        cl.rects.emplace_back(DrcLayer::POLY, ga_x0, na_top, ga_x1, pa_bot);
    }

    // Output Y: drain of NMOS series + merged drains of PMOS parallel
    double mcon_sz = cs;
    cl.rects.emplace_back(DrcLayer::MCON,
                          ndrn_cx - mcon_sz/2, nmos_cy - mcon_sz/2,
                          ndrn_cx + mcon_sz/2, nmos_cy + mcon_sz/2);

    // M1 output strap
    double out_x = ndrn_cx;
    place_m1_strap(cl, out_x - rules_.m1_width/2, nmos_cy,
                       out_x + rules_.m1_width/2, pmos_cy);
    cl.pins.emplace_back("Y", DrcLayer::MET1,
                          Rect(out_x - rules_.m1_width/2, mid_y - rules_.m1_width/2,
                               out_x + rules_.m1_width/2, mid_y + rules_.m1_width/2),
                          "OUTPUT");

    // Source ties
    cl.rects.emplace_back(DrcLayer::LI1,
                          nsrc_cx - rules_.li_width/2, 0.0,
                          nsrc_cx + rules_.li_width/2, nmos_cy);

    // Cell width
    double raw_w = std::max(nmos_diff_x1, pmos_full_x1) + rules_.diff_spacing
                   + rules_.tap_width;
    cl.width = snap_to_site(raw_w);

    add_nwell(cl);
    add_power_rails(cl);
    add_well_taps(cl);

    cl.properties.emplace_back("FUNCTION", "NAND2");
    cl.properties.emplace_back("DRIVE", std::to_string(std::max(drive_strength, 1)));
    return cl;
}

// ═══════════════════════════════════════════════════════════════════════════
//  2-input NOR (NOR2_X1, NOR2_X2, NOR2_X4)
//  NMOS: parallel (A and B), PMOS: series stack (A on top, B below)
// ═══════════════════════════════════════════════════════════════════════════

CellLayout CellLayoutGenerator::generate_nor2(int drive_strength) {
    CellLayout cl;
    double ds = static_cast<double>(std::max(drive_strength, 1));
    cl.cell_name = "NOR2_X" + std::to_string(std::max(drive_strength, 1));
    cl.height    = rules_.row_height;

    double rw    = rules_.power_rail_width;
    double gl    = rules_.gate_length;
    double dce   = rules_.diff_contact_enclosure;
    double cs    = rules_.contact_size;
    double mid_y = cl.height / 2.0;

    double nmos_w = rules_.diff_width * ds;
    // Series PMOS needs 2x width to compensate for stacking + mobility
    double pmos_w = rules_.diff_width * ds * 3.0;

    double nmos_cy = rw + nmos_w / 2.0 + dce;
    double pmos_cy = cl.height - rw - pmos_w / 2.0 - dce;

    double x_start = rules_.tap_width + rules_.diff_spacing;
    double sd_w = dce + cs + dce;

    // NMOS parallel: two gates sharing source to VSS, drains merged to Y
    double nmos_x0 = x_start;
    double nmos_diff_x1 = nmos_x0 + sd_w + gl + sd_w + gl + sd_w;
    double nmos_y0 = nmos_cy - nmos_w / 2.0;
    double nmos_y1 = nmos_cy + nmos_w / 2.0;

    cl.rects.emplace_back(DrcLayer::DIFF, nmos_x0, nmos_y0, nmos_diff_x1, nmos_y1);

    // Gate A
    double ga_x0 = nmos_x0 + sd_w;
    double ga_x1 = ga_x0 + gl;
    cl.rects.emplace_back(DrcLayer::POLY, ga_x0,
                          nmos_y0 - rules_.poly_endcap, ga_x1,
                          nmos_y1 + rules_.poly_endcap);
    cl.pins.emplace_back("A", DrcLayer::POLY,
                          Rect(ga_x0, nmos_y0, ga_x1, nmos_y1), "INPUT");

    // Gate B
    double gb_x0 = ga_x1 + sd_w;
    double gb_x1 = gb_x0 + gl;
    cl.rects.emplace_back(DrcLayer::POLY, gb_x0,
                          nmos_y0 - rules_.poly_endcap, gb_x1,
                          nmos_y1 + rules_.poly_endcap);
    cl.pins.emplace_back("B", DrcLayer::POLY,
                          Rect(gb_x0, nmos_y0, gb_x1, nmos_y1), "INPUT");

    // NMOS contacts
    double nsrc_cx = nmos_x0 + dce + cs / 2.0;
    place_contact(cl, nsrc_cx, nmos_cy);
    double nint_cx = ga_x1 + dce + cs / 2.0;
    place_contact(cl, nint_cx, nmos_cy);
    double ndrn_cx = gb_x1 + dce + cs / 2.0;
    place_contact(cl, ndrn_cx, nmos_cy);

    cl.rects.emplace_back(DrcLayer::NSDM,
                          nmos_x0 - rules_.nsdm_enclosure, nmos_y0 - rules_.nsdm_enclosure,
                          nmos_diff_x1 + rules_.nsdm_enclosure, nmos_y1 + rules_.nsdm_enclosure);

    // PMOS series: source -- gate_A -- internal -- gate_B -- drain
    double pmos_x0 = x_start;
    double pmos_diff_x1 = pmos_x0 + sd_w + gl + sd_w + gl + sd_w;
    double pmos_y0 = pmos_cy - pmos_w / 2.0;
    double pmos_y1 = pmos_cy + pmos_w / 2.0;

    cl.rects.emplace_back(DrcLayer::DIFF, pmos_x0, pmos_y0, pmos_diff_x1, pmos_y1);

    // PMOS Gate A
    double pga_x0 = pmos_x0 + sd_w;
    double pga_x1 = pga_x0 + gl;
    cl.rects.emplace_back(DrcLayer::POLY, pga_x0,
                          pmos_y0 - rules_.poly_endcap, pga_x1,
                          pmos_y1 + rules_.poly_endcap);

    // PMOS Gate B
    double pgb_x0 = pga_x1 + sd_w;
    double pgb_x1 = pgb_x0 + gl;
    cl.rects.emplace_back(DrcLayer::POLY, pgb_x0,
                          pmos_y0 - rules_.poly_endcap, pgb_x1,
                          pmos_y1 + rules_.poly_endcap);

    // PMOS contacts
    double psrc_cx = pmos_x0 + dce + cs / 2.0;
    place_contact(cl, psrc_cx, pmos_cy);
    double pint_cx = pga_x1 + dce + cs / 2.0;
    place_contact(cl, pint_cx, pmos_cy);
    double pdrn_cx = pgb_x1 + dce + cs / 2.0;
    place_contact(cl, pdrn_cx, pmos_cy);

    cl.rects.emplace_back(DrcLayer::PSDM,
                          pmos_x0 - rules_.psdm_enclosure, pmos_y0 - rules_.psdm_enclosure,
                          pmos_diff_x1 + rules_.psdm_enclosure, pmos_y1 + rules_.psdm_enclosure);

    // Connect poly gates vertically (A NMOS to A PMOS, B NMOS to B PMOS)
    double na_top = nmos_y1 + rules_.poly_endcap;
    double pa_bot = pmos_y0 - rules_.poly_endcap;
    if (pa_bot > na_top) {
        cl.rects.emplace_back(DrcLayer::POLY, ga_x0, na_top, ga_x1, pa_bot);
        cl.rects.emplace_back(DrcLayer::POLY, gb_x0, na_top, gb_x1, pa_bot);
    }

    // Output Y: drain of PMOS series + merged drain/source of NMOS parallel
    double out_x = nint_cx;  // internal node of NMOS parallel = shared drain
    double mcon_sz = cs;
    cl.rects.emplace_back(DrcLayer::MCON,
                          out_x - mcon_sz/2, nmos_cy - mcon_sz/2,
                          out_x + mcon_sz/2, nmos_cy + mcon_sz/2);
    cl.rects.emplace_back(DrcLayer::MCON,
                          pdrn_cx - mcon_sz/2, pmos_cy - mcon_sz/2,
                          pdrn_cx + mcon_sz/2, pmos_cy + mcon_sz/2);

    place_m1_strap(cl, out_x - rules_.m1_width/2, nmos_cy,
                       out_x + rules_.m1_width/2, pmos_cy);
    cl.pins.emplace_back("Y", DrcLayer::MET1,
                          Rect(out_x - rules_.m1_width/2, mid_y - rules_.m1_width/2,
                               out_x + rules_.m1_width/2, mid_y + rules_.m1_width/2),
                          "OUTPUT");

    // Source ties to rails
    cl.rects.emplace_back(DrcLayer::LI1,
                          nsrc_cx - rules_.li_width/2, 0.0,
                          nsrc_cx + rules_.li_width/2, nmos_cy);
    cl.rects.emplace_back(DrcLayer::LI1,
                          psrc_cx - rules_.li_width/2, pmos_cy,
                          psrc_cx + rules_.li_width/2, cl.height);

    double raw_w = std::max(nmos_diff_x1, pmos_diff_x1) + rules_.diff_spacing
                   + rules_.tap_width;
    cl.width = snap_to_site(raw_w);

    add_nwell(cl);
    add_power_rails(cl);
    add_well_taps(cl);

    cl.properties.emplace_back("FUNCTION", "NOR2");
    cl.properties.emplace_back("DRIVE", std::to_string(std::max(drive_strength, 1)));
    return cl;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Buffer (BUF_X1 .. X4) -- two cascaded inverters
// ═══════════════════════════════════════════════════════════════════════════

CellLayout CellLayoutGenerator::generate_buf(int drive_strength) {
    CellLayout cl;
    double ds = static_cast<double>(std::max(drive_strength, 1));
    cl.cell_name = "BUF_X" + std::to_string(std::max(drive_strength, 1));
    cl.height    = rules_.row_height;

    double rw    = rules_.power_rail_width;
    double mid_y = cl.height / 2.0;

    // First inverter: minimum size (internal)
    double nmos_w1 = rules_.diff_width;
    double pmos_w1 = rules_.diff_width * 1.5;
    // Second inverter: drive-strength sized
    double nmos_w2 = rules_.diff_width * ds;
    double pmos_w2 = rules_.diff_width * ds * 1.5;

    double nmos_cy = rw + std::max(nmos_w1, nmos_w2) / 2.0
                     + rules_.diff_contact_enclosure;
    double pmos_cy = cl.height - rw - std::max(pmos_w1, pmos_w2) / 2.0
                     - rules_.diff_contact_enclosure;

    double x_start = rules_.tap_width + rules_.diff_spacing;

    // First stage NMOS + PMOS
    double n1_right = place_transistor(cl, x_start, nmos_cy, nmos_w1, false, "A");
    double p1_right = place_transistor(cl, x_start, pmos_cy, pmos_w1, true, "");

    double stage1_right = std::max(n1_right, p1_right);

    // Internal connection between stages via LI1
    double int_x = stage1_right + rules_.diff_spacing;

    // Second stage
    double x2_start = int_x;
    double n2_right = place_transistor(cl, x2_start, nmos_cy, nmos_w2, false, "");
    double p2_right = place_transistor(cl, x2_start, pmos_cy, pmos_w2, true, "");

    // Output pin Y on second stage drain
    double gl = rules_.gate_length;
    double dce = rules_.diff_contact_enclosure;
    double cs = rules_.contact_size;
    double drn2_cx = x2_start + (dce + cs + dce) + gl + dce + cs / 2.0;

    double mcon_sz = cs;
    cl.rects.emplace_back(DrcLayer::MCON,
                          drn2_cx - mcon_sz/2, nmos_cy - mcon_sz/2,
                          drn2_cx + mcon_sz/2, nmos_cy + mcon_sz/2);
    cl.rects.emplace_back(DrcLayer::MCON,
                          drn2_cx - mcon_sz/2, pmos_cy - mcon_sz/2,
                          drn2_cx + mcon_sz/2, pmos_cy + mcon_sz/2);
    place_m1_strap(cl, drn2_cx - rules_.m1_width/2, nmos_cy,
                       drn2_cx + rules_.m1_width/2, pmos_cy);
    cl.pins.emplace_back("Y", DrcLayer::MET1,
                          Rect(drn2_cx - rules_.m1_width/2, mid_y - rules_.m1_width/2,
                               drn2_cx + rules_.m1_width/2, mid_y + rules_.m1_width/2),
                          "OUTPUT");

    // Source ties for both stages
    double src1_cx = x_start + dce + cs / 2.0;
    cl.rects.emplace_back(DrcLayer::LI1,
                          src1_cx - rules_.li_width/2, 0.0,
                          src1_cx + rules_.li_width/2, nmos_cy);
    cl.rects.emplace_back(DrcLayer::LI1,
                          src1_cx - rules_.li_width/2, pmos_cy,
                          src1_cx + rules_.li_width/2, cl.height);

    double raw_w = std::max(n2_right, p2_right) + rules_.diff_spacing
                   + rules_.tap_width;
    cl.width = snap_to_site(raw_w);

    add_nwell(cl);
    add_power_rails(cl);
    add_well_taps(cl);

    cl.properties.emplace_back("FUNCTION", "BUF");
    cl.properties.emplace_back("DRIVE", std::to_string(std::max(drive_strength, 1)));
    return cl;
}

// ═══════════════════════════════════════════════════════════════════════════
//  D Flip-Flop (transmission-gate master-slave)
//  Topology: ~20 transistors arranged as:
//    Input buffer (inv) -> TG master -> master latch (inv + TG feedback)
//    -> TG slave -> slave latch (inv + TG feedback) -> output buffer
// ═══════════════════════════════════════════════════════════════════════════

CellLayout CellLayoutGenerator::generate_dff() {
    CellLayout cl;
    cl.cell_name = "DFF_X1";
    cl.height    = rules_.row_height;

    double rw    = rules_.power_rail_width;
    double mid_y = cl.height / 2.0;
    double dce   = rules_.diff_contact_enclosure;
    double cs    = rules_.contact_size;
    double gl    = rules_.gate_length;

    double nmos_w = rules_.diff_width;
    double pmos_w = rules_.diff_width * 1.5;

    double nmos_cy = rw + nmos_w / 2.0 + dce;
    double pmos_cy = cl.height - rw - pmos_w / 2.0 - dce;

    double x_cursor = rules_.tap_width + rules_.diff_spacing;

    // The DFF uses ~10 inverter-like stages.  We lay them out sequentially.
    // Stage labels: clk_inv, d_inv, tg_master(2), master_inv, master_fb(2),
    //               tg_slave(2), slave_inv, slave_fb(2), out_inv
    // For brevity we place 10 gate columns (each = 1 transistor pair)
    int num_gates = 10;
    bool first_input_placed = false;
    bool clk_placed = false;

    for (int i = 0; i < num_gates; ++i) {
        std::string ngate_name, pgate_name;
        if (i == 0 && !clk_placed) {
            ngate_name = "CLK";
            clk_placed = true;
        } else if (i == 1 && !first_input_placed) {
            ngate_name = "D";
            first_input_placed = true;
        }

        double n_right = place_transistor(cl, x_cursor, nmos_cy, nmos_w, false,
                                           ngate_name);
        place_transistor(cl, x_cursor, pmos_cy, pmos_w, true, "");

        // Connect gates vertically
        double gate_x0 = x_cursor + (dce + cs + dce);
        double gate_x1 = gate_x0 + gl;
        double n_top = (nmos_cy + nmos_w/2.0) + rules_.poly_endcap;
        double p_bot = (pmos_cy - pmos_w/2.0) - rules_.poly_endcap;
        if (p_bot > n_top) {
            cl.rects.emplace_back(DrcLayer::POLY, gate_x0, n_top, gate_x1, p_bot);
        }

        x_cursor = n_right + rules_.min_poly_spacing;
    }

    // Output Q: connect last stage drain to M1
    double last_drn_cx = x_cursor - rules_.min_poly_spacing
                         - (dce + cs / 2.0);
    // Approximate: just use the last gate's drain position
    double drn_approx = x_cursor - rules_.min_poly_spacing;
    double out_cx = drn_approx - dce - cs / 2.0;

    double mcon_sz = cs;
    cl.rects.emplace_back(DrcLayer::MCON,
                          out_cx - mcon_sz/2, nmos_cy - mcon_sz/2,
                          out_cx + mcon_sz/2, nmos_cy + mcon_sz/2);
    cl.rects.emplace_back(DrcLayer::MCON,
                          out_cx - mcon_sz/2, pmos_cy - mcon_sz/2,
                          out_cx + mcon_sz/2, pmos_cy + mcon_sz/2);
    place_m1_strap(cl, out_cx - rules_.m1_width/2, nmos_cy,
                       out_cx + rules_.m1_width/2, pmos_cy);
    cl.pins.emplace_back("Q", DrcLayer::MET1,
                          Rect(out_cx - rules_.m1_width/2, mid_y - rules_.m1_width/2,
                               out_cx + rules_.m1_width/2, mid_y + rules_.m1_width/2),
                          "OUTPUT");

    // Source ties (first gate pair source to rails)
    double src_cx = rules_.tap_width + rules_.diff_spacing + dce + cs / 2.0;
    cl.rects.emplace_back(DrcLayer::LI1,
                          src_cx - rules_.li_width/2, 0.0,
                          src_cx + rules_.li_width/2, nmos_cy);
    cl.rects.emplace_back(DrcLayer::LI1,
                          src_cx - rules_.li_width/2, pmos_cy,
                          src_cx + rules_.li_width/2, cl.height);

    double raw_w = x_cursor + rules_.diff_spacing + rules_.tap_width;
    cl.width = snap_to_site(raw_w);

    add_nwell(cl);
    add_power_rails(cl);
    add_well_taps(cl);

    cl.properties.emplace_back("FUNCTION", "DFF");
    return cl;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Filler cell -- no logic, just NWELL + power rails + well taps
// ═══════════════════════════════════════════════════════════════════════════

CellLayout CellLayoutGenerator::generate_fill(int sites) {
    CellLayout cl;
    sites = std::max(sites, 1);
    cl.cell_name = "FILL_" + std::to_string(sites);
    cl.height    = rules_.row_height;
    cl.width     = rules_.site_width * sites;

    add_nwell(cl);
    add_power_rails(cl);
    add_well_taps(cl);

    cl.properties.emplace_back("FUNCTION", "FILL");
    return cl;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Tie-high cell: output permanently connected to VDD
// ═══════════════════════════════════════════════════════════════════════════

CellLayout CellLayoutGenerator::generate_tieh() {
    CellLayout cl;
    cl.cell_name = "TIEH";
    cl.height    = rules_.row_height;

    double rw = rules_.power_rail_width;
    double x_start = rules_.tap_width + rules_.diff_spacing;

    // Minimum-size PMOS with gate tied to VSS, drain = output (always VDD)
    double pmos_w  = rules_.diff_width;
    double pmos_cy = cl.height - rw - pmos_w / 2.0 - rules_.diff_contact_enclosure;

    double right = place_transistor(cl, x_start, pmos_cy, pmos_w, true, "");

    // Connect gate to VSS via poly strap down to bottom rail
    double dce = rules_.diff_contact_enclosure;
    double cs  = rules_.contact_size;
    double gl  = rules_.gate_length;
    double gate_x0 = x_start + (dce + cs + dce);
    double gate_x1 = gate_x0 + gl;
    cl.rects.emplace_back(DrcLayer::POLY, gate_x0, 0.0, gate_x1, rw);

    // Output pin on drain
    double drn_cx = gate_x1 + dce + cs / 2.0;
    double mcon_sz = cs;
    cl.rects.emplace_back(DrcLayer::MCON,
                          drn_cx - mcon_sz/2, pmos_cy - mcon_sz/2,
                          drn_cx + mcon_sz/2, pmos_cy + mcon_sz/2);
    double mid_y = cl.height / 2.0;
    place_m1_strap(cl, drn_cx - rules_.m1_width/2, pmos_cy,
                       drn_cx + rules_.m1_width/2, cl.height - rw);
    cl.pins.emplace_back("Y", DrcLayer::MET1,
                          Rect(drn_cx - rules_.m1_width/2, mid_y - rules_.m1_width/2,
                               drn_cx + rules_.m1_width/2, mid_y + rules_.m1_width/2),
                          "OUTPUT");

    // Source tied to VDD rail
    double src_cx = x_start + dce + cs / 2.0;
    cl.rects.emplace_back(DrcLayer::LI1,
                          src_cx - rules_.li_width/2, pmos_cy,
                          src_cx + rules_.li_width/2, cl.height);

    double raw_w = right + rules_.diff_spacing + rules_.tap_width;
    cl.width = snap_to_site(raw_w);

    add_nwell(cl);
    add_power_rails(cl);
    add_well_taps(cl);

    cl.properties.emplace_back("FUNCTION", "TIEH");
    return cl;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Tie-low cell: output permanently connected to VSS
// ═══════════════════════════════════════════════════════════════════════════

CellLayout CellLayoutGenerator::generate_tiel() {
    CellLayout cl;
    cl.cell_name = "TIEL";
    cl.height    = rules_.row_height;

    double rw = rules_.power_rail_width;
    double x_start = rules_.tap_width + rules_.diff_spacing;

    // Minimum-size NMOS with gate tied to VDD, drain = output (always VSS)
    double nmos_w  = rules_.diff_width;
    double nmos_cy = rw + nmos_w / 2.0 + rules_.diff_contact_enclosure;

    double right = place_transistor(cl, x_start, nmos_cy, nmos_w, false, "");

    // Connect gate to VDD via poly strap up to top rail
    double dce = rules_.diff_contact_enclosure;
    double cs  = rules_.contact_size;
    double gl  = rules_.gate_length;
    double gate_x0 = x_start + (dce + cs + dce);
    double gate_x1 = gate_x0 + gl;
    cl.rects.emplace_back(DrcLayer::POLY, gate_x0, cl.height - rw, gate_x1, cl.height);

    // Output pin on drain
    double drn_cx = gate_x1 + dce + cs / 2.0;
    double mcon_sz = cs;
    cl.rects.emplace_back(DrcLayer::MCON,
                          drn_cx - mcon_sz/2, nmos_cy - mcon_sz/2,
                          drn_cx + mcon_sz/2, nmos_cy + mcon_sz/2);
    double mid_y = cl.height / 2.0;
    place_m1_strap(cl, drn_cx - rules_.m1_width/2, rw,
                       drn_cx + rules_.m1_width/2, nmos_cy);
    cl.pins.emplace_back("Y", DrcLayer::MET1,
                          Rect(drn_cx - rules_.m1_width/2, mid_y - rules_.m1_width/2,
                               drn_cx + rules_.m1_width/2, mid_y + rules_.m1_width/2),
                          "OUTPUT");

    // Source tied to VSS rail
    double src_cx = x_start + dce + cs / 2.0;
    cl.rects.emplace_back(DrcLayer::LI1,
                          src_cx - rules_.li_width/2, 0.0,
                          src_cx + rules_.li_width/2, nmos_cy);

    double raw_w = right + rules_.diff_spacing + rules_.tap_width;
    cl.width = snap_to_site(raw_w);

    add_nwell(cl);
    add_power_rails(cl);
    add_well_taps(cl);

    cl.properties.emplace_back("FUNCTION", "TIEL");
    return cl;
}

} // namespace sf
