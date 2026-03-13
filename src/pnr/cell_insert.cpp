// SiliconForge — Filler, Endcap, and Well-Tap Cell Insertion Implementation
#include "pnr/cell_insert.hpp"
#include <algorithm>
#include <vector>
#include <cmath>

namespace sf {

// Minimum spacing between adjacent infrastructure cells (um)
static constexpr double MIN_CELL_SPACING = 0.07;

CellInsertResult CellInserter::insert(const CellInsertConfig& cfg) {
    CellInsertResult res;
    res.endcaps   = insert_endcaps(cfg);
    res.well_taps = insert_well_taps(cfg);
    res.fillers   = insert_fillers(cfg);
    res.message = "Cell insert: " + std::to_string(res.endcaps) + " endcaps, " +
                  std::to_string(res.well_taps) + " taps, " +
                  std::to_string(res.fillers) + " fillers";
    return res;
}

int CellInserter::insert_endcaps(const CellInsertConfig& cfg) {
    double row_h = pd_.row_height;
    if (row_h <= 0) row_h = 10.0;
    double die_x0 = pd_.die_area.x0;
    double die_x1 = pd_.die_area.x1;
    double die_y0 = pd_.die_area.y0;
    double die_y1 = pd_.die_area.y1;

    int count = 0;
    for (double y = die_y0; y + row_h <= die_y1; y += row_h) {
        // Check for existing cells too close to endcap positions
        auto would_conflict = [&](double ex0, double ex1) {
            for (auto& c : pd_.cells) {
                if (!c.placed) continue;
                if (c.position.y < y || c.position.y >= y + row_h) continue;
                double gap_right = c.position.x - ex1;
                double gap_left = ex0 - (c.position.x + c.width);
                if ((gap_right > 0 && gap_right < MIN_CELL_SPACING) ||
                    (gap_left > 0 && gap_left < MIN_CELL_SPACING))
                    return true;
            }
            return false;
        };

        // Left endcap
        if (!would_conflict(die_x0, die_x0 + cfg.endcap_width)) {
            int lid = pd_.add_cell("ENDCAP_L_r" + std::to_string(count), "ENDCAP",
                                   cfg.endcap_width, row_h);
            pd_.cells[lid].position = {die_x0, y};
            pd_.cells[lid].placed = true;
            count++;
        }

        // Right endcap
        double rx0 = die_x1 - cfg.endcap_width;
        if (!would_conflict(rx0, die_x1)) {
            int rid = pd_.add_cell("ENDCAP_R_r" + std::to_string(count), "ENDCAP",
                                   cfg.endcap_width, row_h);
            pd_.cells[rid].position = {rx0, y};
            pd_.cells[rid].placed = true;
            count++;
        }
    }
    return count;
}

int CellInserter::insert_well_taps(const CellInsertConfig& cfg) {
    double row_h = pd_.row_height;
    if (row_h <= 0) row_h = 10.0;
    double die_x0 = pd_.die_area.x0;
    double die_x1 = pd_.die_area.x1;
    double die_y0 = pd_.die_area.y0;
    double die_y1 = pd_.die_area.y1;

    int count = 0;
    for (double y = die_y0; y + row_h <= die_y1; y += row_h) {
        // Collect occupied X intervals in this row for spacing checks
        std::vector<std::pair<double, double>> occupied;
        for (auto& c : pd_.cells) {
            if (!c.placed) continue;
            if (c.position.y >= y && c.position.y < y + row_h) {
                occupied.push_back({c.position.x, c.position.x + c.width});
            }
        }
        std::sort(occupied.begin(), occupied.end());

        // Insert taps at regular intervals across the row
        for (double x = die_x0 + cfg.endcap_width + cfg.tap_spacing;
             x + cfg.tap_width <= die_x1 - cfg.endcap_width;
             x += cfg.tap_spacing) {
            // Check if this position would create a gap < MIN_CELL_SPACING
            double tap_x0 = x;
            double tap_x1 = x + cfg.tap_width;
            bool too_close = false;
            for (auto& [ox0, ox1] : occupied) {
                double gap_left = tap_x0 - ox1;   // gap between existing cell end and tap start
                double gap_right = ox0 - tap_x1;  // gap between tap end and existing cell start
                if ((gap_left > 0 && gap_left < MIN_CELL_SPACING) ||
                    (gap_right > 0 && gap_right < MIN_CELL_SPACING)) {
                    too_close = true;
                    break;
                }
            }
            if (too_close) continue;

            int tid = pd_.add_cell("WELLTAP_" + std::to_string(count), "WELLTAP",
                                   cfg.tap_width, row_h);
            pd_.cells[tid].position = {x, y};
            pd_.cells[tid].placed = true;
            count++;
        }
    }
    return count;
}

int CellInserter::insert_fillers(const CellInsertConfig& cfg) {
    double row_h = pd_.row_height;
    if (row_h <= 0) row_h = 10.0;
    double die_x0 = pd_.die_area.x0;
    double die_x1 = pd_.die_area.x1;
    double die_y0 = pd_.die_area.y0;
    double die_y1 = pd_.die_area.y1;
    double site_w = pd_.site_width;
    if (site_w <= 0) site_w = 1.0;

    // Sorted filler widths largest first for greedy fill
    double sorted_widths[4];
    for (int i = 0; i < 4; ++i) sorted_widths[i] = cfg.filler_widths[i];
    std::sort(sorted_widths, sorted_widths + 4, std::greater<double>());

    int count = 0;
    for (double y = die_y0; y + row_h <= die_y1; y += row_h) {
        // Collect occupied X intervals in this row
        std::vector<std::pair<double, double>> occupied;
        for (auto& c : pd_.cells) {
            if (!c.placed) continue;
            if (c.position.y >= y && c.position.y < y + row_h) {
                occupied.push_back({c.position.x, c.position.x + c.width});
            }
        }
        std::sort(occupied.begin(), occupied.end());

        // Find gaps between occupied intervals
        double cursor = die_x0;
        for (auto& [ox0, ox1] : occupied) {
            double gap = ox0 - cursor;
            // Fill the gap with largest-first filler cells, respecting min spacing
            double fill_x = cursor;
            while (gap >= sorted_widths[3] - 1e-9) { // smallest filler
                bool placed_one = false;
                for (int i = 0; i < 4; ++i) {
                    if (gap >= sorted_widths[i] - 1e-9) {
                        double remaining = gap - sorted_widths[i];
                        // Only place if it fills exactly or leaves enough spacing
                        if (remaining > 1e-9 && remaining < MIN_CELL_SPACING)
                            continue;
                        std::string fname = "FILLER_" + std::to_string(count);
                        int fid = pd_.add_cell(fname, "FILL", sorted_widths[i], row_h);
                        pd_.cells[fid].position = {fill_x, y};
                        pd_.cells[fid].placed = true;
                        fill_x += sorted_widths[i];
                        gap -= sorted_widths[i];
                        count++;
                        placed_one = true;
                        break;
                    }
                }
                if (!placed_one) break;
            }
            cursor = std::max(cursor, ox1);
        }
        // Fill remaining gap to row end
        double gap = die_x1 - cursor;
        double fill_x = cursor;
        while (gap >= sorted_widths[3] - 1e-9) {
            bool placed_one = false;
            for (int i = 0; i < 4; ++i) {
                if (gap >= sorted_widths[i] - 1e-9) {
                    double remaining = gap - sorted_widths[i];
                    // Only place if it fills exactly or leaves enough spacing
                    if (remaining > 1e-9 && remaining < MIN_CELL_SPACING)
                        continue;
                    std::string fname = "FILLER_" + std::to_string(count);
                    int fid = pd_.add_cell(fname, "FILL", sorted_widths[i], row_h);
                    pd_.cells[fid].position = {fill_x, y};
                    pd_.cells[fid].placed = true;
                    fill_x += sorted_widths[i];
                    gap -= sorted_widths[i];
                    count++;
                    placed_one = true;
                    break;
                }
            }
            if (!placed_one) break;
        }
    }
    return count;
}

int CellInserter::insert_decaps(const std::vector<Point>& hotspots, double decap_width) {
    double row_h = pd_.row_height;
    if (row_h <= 0) row_h = 10.0;
    double die_x0 = pd_.die_area.x0;
    double die_x1 = pd_.die_area.x1;
    double die_y0 = pd_.die_area.y0;
    double die_y1 = pd_.die_area.y1;

    int count = 0;

    if (hotspots.empty()) {
        // No hotspots — insert DECAPs at regular intervals in each row
        double interval = 40.0; // every 40um
        for (double y = die_y0; y + row_h <= die_y1; y += row_h) {
            // Collect occupied intervals
            std::vector<std::pair<double, double>> occupied;
            for (auto& c : pd_.cells) {
                if (!c.placed) continue;
                if (c.position.y >= y && c.position.y < y + row_h)
                    occupied.push_back({c.position.x, c.position.x + c.width});
            }
            std::sort(occupied.begin(), occupied.end());

            for (double x = die_x0 + interval; x + decap_width <= die_x1; x += interval) {
                // Check if position is clear
                bool blocked = false;
                for (auto& [ox0, ox1] : occupied) {
                    if (x < ox1 && x + decap_width > ox0) {
                        blocked = true;
                        break;
                    }
                }
                if (blocked) continue;

                // Check minimum spacing
                bool too_close = false;
                for (auto& [ox0, ox1] : occupied) {
                    double gap_left = x - ox1;
                    double gap_right = ox0 - (x + decap_width);
                    if ((gap_left > 0 && gap_left < MIN_CELL_SPACING) ||
                        (gap_right > 0 && gap_right < MIN_CELL_SPACING)) {
                        too_close = true;
                        break;
                    }
                }
                if (too_close) continue;

                int did = pd_.add_cell("DECAP_" + std::to_string(count), "DECAP",
                                       decap_width, row_h);
                pd_.cells[did].position = {x, y};
                pd_.cells[did].placed = true;
                occupied.push_back({x, x + decap_width});
                std::sort(occupied.begin(), occupied.end());
                count++;
            }
        }
    } else {
        // Insert DECAPs near each hotspot
        double radius = 30.0; // search within 30um of hotspot
        for (auto& hs : hotspots) {
            // Find the nearest row
            int best_row = -1;
            double best_dist = 1e18;
            for (double y = die_y0; y + row_h <= die_y1; y += row_h) {
                double d = std::abs(hs.y - (y + row_h / 2.0));
                if (d < best_dist) { best_dist = d; best_row = (int)((y - die_y0) / row_h); }
            }
            if (best_row < 0) continue;

            double row_y = die_y0 + best_row * row_h;

            // Collect occupied in this row
            std::vector<std::pair<double, double>> occupied;
            for (auto& c : pd_.cells) {
                if (!c.placed) continue;
                if (c.position.y >= row_y && c.position.y < row_y + row_h)
                    occupied.push_back({c.position.x, c.position.x + c.width});
            }
            std::sort(occupied.begin(), occupied.end());

            // Try to place near hotspot x
            double try_x = std::max(die_x0, hs.x - decap_width / 2.0);
            try_x = std::min(try_x, die_x1 - decap_width);

            // Search outward from hotspot
            for (double offset = 0; offset < radius; offset += decap_width) {
                for (double sign : {0.0, 1.0, -1.0}) {
                    double x = try_x + sign * offset;
                    if (x < die_x0 || x + decap_width > die_x1) continue;

                    bool blocked = false;
                    for (auto& [ox0, ox1] : occupied) {
                        if (x < ox1 && x + decap_width > ox0) {
                            blocked = true;
                            break;
                        }
                    }
                    if (blocked) continue;

                    bool too_close = false;
                    for (auto& [ox0, ox1] : occupied) {
                        double gap_left = x - ox1;
                        double gap_right = ox0 - (x + decap_width);
                        if ((gap_left > 0 && gap_left < MIN_CELL_SPACING) ||
                            (gap_right > 0 && gap_right < MIN_CELL_SPACING)) {
                            too_close = true;
                            break;
                        }
                    }
                    if (too_close) continue;

                    int did = pd_.add_cell("DECAP_" + std::to_string(count), "DECAP",
                                           decap_width, row_h);
                    pd_.cells[did].position = {x, row_y};
                    pd_.cells[did].placed = true;
                    count++;
                    goto next_hotspot;
                }
            }
            next_hotspot:;
        }
    }

    return count;
}

} // namespace sf
