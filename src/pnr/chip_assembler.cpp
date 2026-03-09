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

} // namespace sf
