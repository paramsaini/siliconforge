// SiliconForge — Physical Design Implementation
#include "pnr/physical.hpp"
#include <iostream>
#include <cmath>

namespace sf {

int PhysicalDesign::add_cell(const std::string& name, const std::string& type,
                              double w, double h) {
    int id = (int)cells.size();
    cells.push_back({id, name, type, w, h, {0,0}, false});
    cell_name_map[name] = id;
    return id;
}

int PhysicalDesign::add_net(const std::string& name, const std::vector<int>& cell_ids) {
    int id = (int)nets.size();
    PhysNet net{id, name, cell_ids, {}};
    // Default pin offsets: center of cell
    for (auto cid : cell_ids) {
        net.pin_offsets.push_back({cells[cid].width / 2, cells[cid].height / 2});
    }
    nets.push_back(net);
    return id;
}

double PhysicalDesign::total_wirelength() const {
    // Half-Perimeter Wirelength (HPWL)
    double total = 0;
    for (auto& net : nets) {
        if (net.cell_ids.size() < 2) continue;
        double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
        for (size_t i = 0; i < net.cell_ids.size(); ++i) {
            auto& c = cells[net.cell_ids[i]];
            double px = c.position.x + (i < net.pin_offsets.size() ? net.pin_offsets[i].x : 0);
            double py = c.position.y + (i < net.pin_offsets.size() ? net.pin_offsets[i].y : 0);
            min_x = std::min(min_x, px); max_x = std::max(max_x, px);
            min_y = std::min(min_y, py); max_y = std::max(max_y, py);
        }
        total += (max_x - min_x) + (max_y - min_y);
    }
    return total;
}

double PhysicalDesign::utilization() const {
    double cell_area = 0;
    for (auto& c : cells) cell_area += c.width * c.height;
    double die = die_area.area();
    return die > 0 ? cell_area / die : 0;
}

bool PhysicalDesign::has_overlaps() const {
    for (size_t i = 0; i < cells.size(); ++i) {
        if (!cells[i].placed) continue;
        Rect ri(cells[i].position.x, cells[i].position.y,
                cells[i].position.x + cells[i].width,
                cells[i].position.y + cells[i].height);
        for (size_t j = i + 1; j < cells.size(); ++j) {
            if (!cells[j].placed) continue;
            Rect rj(cells[j].position.x, cells[j].position.y,
                    cells[j].position.x + cells[j].width,
                    cells[j].position.y + cells[j].height);
            if (ri.overlaps(rj)) return true;
        }
    }
    return false;
}

// ── SpatialIndex implementation ──────────────────────────────────────

int SpatialIndex::grid_key(int col, int row) const {
    return row * grid_cols_ + col;
}

std::pair<int,int> SpatialIndex::to_cell(double x, double y) const {
    int col = (cell_w_ > 0) ? (int)((x - min_x_) / cell_w_) : 0;
    int row = (cell_h_ > 0) ? (int)((y - min_y_) / cell_h_) : 0;
    col = std::clamp(col, 0, std::max(0, grid_cols_ - 1));
    row = std::clamp(row, 0, std::max(0, grid_rows_ - 1));
    return {col, row};
}

void SpatialIndex::compute_grid_params() {
    if (entries_.empty()) {
        grid_cols_ = grid_rows_ = 0;
        min_x_ = min_y_ = 0;
        cell_w_ = cell_h_ = 100.0;
        return;
    }
    double mx0 = 1e18, my0 = 1e18, mx1 = -1e18, my1 = -1e18;
    for (auto& e : entries_) {
        mx0 = std::min(mx0, e.bbox.x0);
        my0 = std::min(my0, e.bbox.y0);
        mx1 = std::max(mx1, e.bbox.x1);
        my1 = std::max(my1, e.bbox.y1);
    }
    min_x_ = mx0;
    min_y_ = my0;
    // Target ~sqrt(N) cells per side for O(sqrt(N)) query cost
    int side = std::max(1, (int)std::sqrt((double)entries_.size()));
    side = std::min(side, 4096);  // cap to avoid excessive memory
    double span_x = mx1 - mx0, span_y = my1 - my0;
    if (span_x <= 0) span_x = 1.0;
    if (span_y <= 0) span_y = 1.0;
    grid_cols_ = side;
    grid_rows_ = side;
    cell_w_ = span_x / grid_cols_;
    cell_h_ = span_y / grid_rows_;
    if (cell_w_ <= 0) cell_w_ = 1.0;
    if (cell_h_ <= 0) cell_h_ = 1.0;
}

void SpatialIndex::insert_into_grid(size_t entry_idx) {
    if (grid_cols_ <= 0 || grid_rows_ <= 0) return;
    auto& e = entries_[entry_idx];
    auto [c0, r0] = to_cell(e.bbox.x0, e.bbox.y0);
    auto [c1, r1] = to_cell(e.bbox.x1, e.bbox.y1);
    for (int r = r0; r <= r1; r++)
        for (int c = c0; c <= c1; c++)
            grid_[grid_key(c, r)].push_back(entry_idx);
}

void SpatialIndex::insert(int id, const Rect& bbox) {
    // If grid hasn't been built yet, just accumulate and let rebuild() handle it
    size_t idx = entries_.size();
    entries_.push_back({id, bbox});
    id_to_index_[id] = idx;
    if (grid_cols_ > 0 && grid_rows_ > 0) {
        insert_into_grid(idx);
    }
}

void SpatialIndex::remove(int id) {
    auto it = id_to_index_.find(id);
    if (it == id_to_index_.end()) return;
    size_t idx = it->second;
    // Swap-and-pop in entries_
    size_t last = entries_.size() - 1;
    if (idx != last) {
        entries_[idx] = entries_[last];
        id_to_index_[entries_[idx].id] = idx;
    }
    entries_.pop_back();
    id_to_index_.erase(it);
    // Full grid rebuild is simplest; for hot-path remove, a lazy-deletion
    // scheme would be better, but rebuild() is fine for typical EDA usage.
    rebuild();
}

std::vector<int> SpatialIndex::query(const Rect& region) const {
    std::vector<int> result;
    if (grid_cols_ <= 0 || grid_rows_ <= 0) {
        // Fallback linear scan (grid not built yet)
        for (auto& e : entries_)
            if (e.bbox.overlaps(region))
                result.push_back(e.id);
        return result;
    }
    auto [c0, r0] = to_cell(region.x0, region.y0);
    auto [c1, r1] = to_cell(region.x1, region.y1);
    // Use a seen-set to avoid duplicates from entries spanning multiple cells
    std::unordered_map<int, bool> seen;
    for (int r = r0; r <= r1; r++) {
        for (int c = c0; c <= c1; c++) {
            int key = grid_key(c, r);
            auto git = grid_.find(key);
            if (git == grid_.end()) continue;
            for (size_t idx : git->second) {
                if (idx >= entries_.size()) continue;
                auto& e = entries_[idx];
                if (!seen[e.id] && e.bbox.overlaps(region)) {
                    seen[e.id] = true;
                    result.push_back(e.id);
                }
            }
        }
    }
    return result;
}

std::vector<int> SpatialIndex::query_point(double x, double y) const {
    return query(Rect(x, y, x, y));
}

void SpatialIndex::clear() {
    entries_.clear();
    id_to_index_.clear();
    grid_.clear();
    grid_cols_ = grid_rows_ = 0;
}

size_t SpatialIndex::size() const {
    return entries_.size();
}

void SpatialIndex::rebuild() {
    grid_.clear();
    compute_grid_params();
    for (size_t i = 0; i < entries_.size(); i++)
        insert_into_grid(i);
}

void PhysicalDesign::print_stats() const {
    std::cout << "Physical Design:\n"
              << "  Die: " << die_area.width() << " x " << die_area.height() << "\n"
              << "  Cells: " << cells.size() << "\n"
              << "  Nets: " << nets.size() << "\n"
              << "  HPWL: " << total_wirelength() << "\n"
              << "  Utilization: " << utilization() * 100 << "%\n";
}

} // namespace sf
