// SiliconForge — Chip Assembler Implementation
#include "pnr/chip_assembler.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace sf {

void ChipAssembler::add_pad(const std::string& name, const std::string& signal,
                              IoPad::Type type, IoPad::Side side) {
    IoPad pad;
    pad.name = name;
    pad.signal = signal;
    pad.type = type;
    pad.side = side;
    pads_.push_back(pad);
}

void ChipAssembler::auto_power_pads(int vdd_count, int vss_count) {
    IoPad::Side sides[] = {IoPad::NORTH, IoPad::SOUTH, IoPad::EAST, IoPad::WEST};
    for (int i = 0; i < vdd_count; ++i) {
        add_pad("VDD_" + std::to_string(i), "VDD", IoPad::POWER_VDD, sides[i % 4]);
    }
    for (int i = 0; i < vss_count; ++i) {
        add_pad("VSS_" + std::to_string(i), "VSS", IoPad::POWER_VSS, sides[i % 4]);
    }
}

void ChipAssembler::place_pads() {
    // Compute chip dimensions
    chip_w_ = core_.die_area.width() + 2 * cfg_.core_margin;
    chip_h_ = core_.die_area.height() + 2 * cfg_.core_margin;

    // Count pads per side
    int n_count = 0, s_count = 0, e_count = 0, w_count = 0;
    for (auto& p : pads_) {
        switch (p.side) {
            case IoPad::NORTH: n_count++; break;
            case IoPad::SOUTH: s_count++; break;
            case IoPad::EAST:  e_count++; break;
            case IoPad::WEST:  w_count++; break;
        }
    }

    // Ensure chip is wide enough
    int max_h = std::max(n_count, s_count);
    int max_v = std::max(e_count, w_count);
    chip_w_ = std::max(chip_w_, max_h * cfg_.pad_pitch + 2 * cfg_.seal_ring_width);
    chip_h_ = std::max(chip_h_, max_v * cfg_.pad_pitch + 2 * cfg_.seal_ring_width);

    // Place pads along each side
    int ni = 0, si = 0, ei = 0, wi = 0;
    for (auto& p : pads_) {
        double pos;
        switch (p.side) {
            case IoPad::NORTH:
                pos = cfg_.seal_ring_width + ni * cfg_.pad_pitch + cfg_.pad_pitch/2;
                p.offset = pos;
                p.bump_position = {pos, chip_h_ - p.height/2};
                ni++;
                break;
            case IoPad::SOUTH:
                pos = cfg_.seal_ring_width + si * cfg_.pad_pitch + cfg_.pad_pitch/2;
                p.offset = pos;
                p.bump_position = {pos, p.height/2};
                si++;
                break;
            case IoPad::EAST:
                pos = cfg_.seal_ring_width + ei * cfg_.pad_pitch + cfg_.pad_pitch/2;
                p.offset = pos;
                p.bump_position = {chip_w_ - p.width/2, pos};
                ei++;
                break;
            case IoPad::WEST:
                pos = cfg_.seal_ring_width + wi * cfg_.pad_pitch + cfg_.pad_pitch/2;
                p.offset = pos;
                p.bump_position = {p.width/2, pos};
                wi++;
                break;
        }
    }
}

void ChipAssembler::generate_bump_map() {
    int bump_rows = (int)(chip_h_ / cfg_.bump_pitch);
    int bump_cols = (int)(chip_w_ / cfg_.bump_pitch);

    for (auto& p : pads_) {
        // Snap bump to nearest grid position
        int bx = std::clamp((int)(p.bump_position.x / cfg_.bump_pitch), 0, bump_cols-1);
        int by = std::clamp((int)(p.bump_position.y / cfg_.bump_pitch), 0, bump_rows-1);
        p.bump_position = {(bx + 0.5) * cfg_.bump_pitch, (by + 0.5) * cfg_.bump_pitch};
    }
}

void ChipAssembler::generate_seal_ring(PhysicalDesign& pd) const {
    double sw = cfg_.seal_ring_width;
    // Four rectangles forming the seal ring
    // (represented as cells in the physical design)
    pd.add_cell("seal_north", "SEAL_RING", chip_w_, sw);
    pd.cells.back().position = {0, chip_h_ - sw};
    pd.cells.back().placed = true;

    pd.add_cell("seal_south", "SEAL_RING", chip_w_, sw);
    pd.cells.back().position = {0, 0};
    pd.cells.back().placed = true;

    pd.add_cell("seal_east", "SEAL_RING", sw, chip_h_);
    pd.cells.back().position = {chip_w_ - sw, 0};
    pd.cells.back().placed = true;

    pd.add_cell("seal_west", "SEAL_RING", sw, chip_h_);
    pd.cells.back().position = {0, 0};
    pd.cells.back().placed = true;
}

ChipResult ChipAssembler::assemble() {
    auto t0 = std::chrono::high_resolution_clock::now();
    ChipResult r;

    place_pads();
    generate_bump_map();

    r.chip_width = chip_w_;
    r.chip_height = chip_h_;
    r.chip_area_mm2 = (chip_w_ * chip_h_) / 1e6; // um² → mm²
    r.total_pads = (int)pads_.size();
    r.pads = pads_;

    for (auto& p : pads_) {
        if (p.type == IoPad::POWER_VDD || p.type == IoPad::POWER_VSS)
            r.power_pads++;
        else
            r.signal_pads++;
    }
    r.total_bumps = r.total_pads;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Chip: " + std::to_string((int)chip_w_) + "×" +
                std::to_string((int)chip_h_) + "um, " +
                std::to_string(r.total_pads) + " pads (" +
                std::to_string(r.signal_pads) + " signal, " +
                std::to_string(r.power_pads) + " power)";
    return r;
}

PhysicalDesign ChipAssembler::to_physical_design() const {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, chip_w_, chip_h_);
    pd.row_height = core_.row_height;

    // Copy core cells with offset
    double ox = cfg_.core_margin;
    double oy = cfg_.core_margin;
    for (auto& c : core_.cells) {
        int cid = pd.add_cell(c.name, c.cell_type, c.width, c.height);
        pd.cells[cid].position = {c.position.x + ox, c.position.y + oy};
        pd.cells[cid].placed = c.placed;
    }

    // Add pad cells
    for (auto& p : pads_) {
        int cid = pd.add_cell(p.name, "IO_PAD", p.width, p.height);
        pd.cells[cid].position = p.bump_position;
        pd.cells[cid].placed = true;
    }

    // Add seal ring
    generate_seal_ring(pd);

    return pd;
}

// ═══════════════════════════════════════════════════════════════════════════
// Tier 3: Detailed IO Pad / Bump Assignment
// ═══════════════════════════════════════════════════════════════════════════

std::string ChipAssembler::ball_name(int row, int col) const {
    std::string name;
    name += (char)('A' + row);
    name += std::to_string(col + 1);
    return name;
}

ChipAssembler::IoAssignResult ChipAssembler::assign_bumps_detailed() {
    IoAssignResult res;
    bumps_.clear();

    if (chip_w_ <= 0 || chip_h_ <= 0) {
        double margin = cfg_.core_margin;
        chip_w_ = core_.die_area.width() + 2 * margin;
        chip_h_ = core_.die_area.height() + 2 * margin;
    }

    double bp = cfg_.bump_pitch;
    if (bp <= 0) bp = 200.0; // default 200um bump pitch
    int cols = std::max(1, (int)(chip_w_ / bp));
    int rows = std::max(1, (int)(chip_h_ / bp));

    // Create bump grid
    double x_offset = (chip_w_ - (cols - 1) * bp) / 2.0;
    double y_offset = (chip_h_ - (rows - 1) * bp) / 2.0;

    // Assign signal pads to peripheral bumps, power to interior
    int pad_idx = 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            BumpAssignment ba;
            ba.ball_name = ball_name(r, c);
            ba.x = x_offset + c * bp;
            ba.y = y_offset + r * bp;

            bool is_peripheral = (r == 0 || r == rows-1 || c == 0 || c == cols-1);
            if (is_peripheral && pad_idx < (int)pads_.size()) {
                ba.signal = pads_[pad_idx].signal;
                ba.is_power = (pads_[pad_idx].type == IoPad::POWER_VDD ||
                               pads_[pad_idx].type == IoPad::POWER_VSS);
                ba.pad_index = pad_idx;
                pad_idx++;
            } else {
                // Interior bumps: alternate VDD/VSS for power delivery
                ba.is_power = true;
                ba.signal = ((r + c) % 2 == 0) ? "VDD" : "VSS";
                ba.pad_index = -1;
            }
            bumps_.push_back(ba);
            res.bumps_assigned++;
        }
    }

    res.message = "Assigned " + std::to_string(res.bumps_assigned) + " bumps (" +
                  std::to_string(cols) + "x" + std::to_string(rows) + " grid)";
    return res;
}

std::vector<ChipAssembler::EscapeRoute> ChipAssembler::route_escapes(int rdl_layer) {
    escape_routes_.clear();

    for (size_t bi = 0; bi < bumps_.size(); bi++) {
        auto& bump = bumps_[bi];
        if (bump.pad_index < 0 || bump.is_power) continue;

        EscapeRoute er;
        er.bump_index = (int)bi;
        er.pad_index = bump.pad_index;
        er.layer = rdl_layer;

        // Route from bump to pad — simple L-shape
        auto& pad = pads_[bump.pad_index];
        Point bump_pt = {bump.x, bump.y};
        Point pad_pt = {pad.offset, 0};

        // Determine pad location based on side
        double margin = cfg_.core_margin;
        switch (pad.side) {
            case IoPad::NORTH: pad_pt = {pad.offset, chip_h_ - margin / 2}; break;
            case IoPad::SOUTH: pad_pt = {pad.offset, margin / 2}; break;
            case IoPad::EAST:  pad_pt = {chip_w_ - margin / 2, pad.offset}; break;
            case IoPad::WEST:  pad_pt = {margin / 2, pad.offset}; break;
        }

        er.path.push_back(bump_pt);
        er.path.push_back({pad_pt.x, bump_pt.y}); // horizontal
        er.path.push_back(pad_pt);                  // vertical
        er.length = std::abs(pad_pt.x - bump_pt.x) + std::abs(pad_pt.y - bump_pt.y);
        escape_routes_.push_back(er);
    }

    return escape_routes_;
}

double ChipAssembler::estimate_io_ir_drop(double current_per_pad) const {
    // Simple IR drop model: R = ρL/A for RDL traces
    double rdl_rho = 0.02;  // ohm/um for RDL
    double total_drop = 0;
    int power_bumps = 0;
    for (auto& b : bumps_) {
        if (b.is_power) {
            power_bumps++;
            double dist_to_core = std::max(
                std::abs(b.x - chip_w_ / 2),
                std::abs(b.y - chip_h_ / 2));
            total_drop += current_per_pad * rdl_rho * dist_to_core;
        }
    }
    return power_bumps > 0 ? total_drop / power_bumps : 0;
}

} // namespace sf
