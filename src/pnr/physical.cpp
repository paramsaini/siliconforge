// SiliconForge — Physical Design Implementation
#include "pnr/physical.hpp"
#include <iostream>
#include <cmath>

namespace sf {

int PhysicalDesign::add_cell(const std::string& name, const std::string& type,
                              double w, double h) {
    int id = (int)cells.size();
    cells.push_back({id, name, type, w, h, {0,0}, false});
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

void PhysicalDesign::print_stats() const {
    std::cout << "Physical Design:\n"
              << "  Die: " << die_area.width() << " x " << die_area.height() << "\n"
              << "  Cells: " << cells.size() << "\n"
              << "  Nets: " << nets.size() << "\n"
              << "  HPWL: " << total_wirelength() << "\n"
              << "  Utilization: " << utilization() * 100 << "%\n";
}

} // namespace sf
