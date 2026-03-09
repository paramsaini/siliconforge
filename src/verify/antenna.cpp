// SiliconForge — Antenna Checker Implementation
#include "verify/antenna.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

namespace sf {

double AntennaChecker::wire_area_for_net(int net_id, int layer) const {
    double area = 0;
    // Sum wire area on specified layer connected to this net
    // Simplified: estimate from wire segments near net cells
    auto& net = pd_.nets[net_id];
    for (auto& w : pd_.wires) {
        if (w.layer != layer) continue;
        // Check if wire endpoints are near any cell in this net
        for (auto cid : net.cell_ids) {
            auto& c = pd_.cells[cid];
            double cx = c.position.x + c.width/2;
            double cy = c.position.y + c.height/2;
            double d = std::min(w.start.dist({cx, cy}), w.end.dist({cx, cy}));
            if (d < 30) { // within proximity
                double length = w.start.dist(w.end);
                area += length * w.width;
                break;
            }
        }
    }
    return area;
}

double AntennaChecker::gate_area_for_net(int net_id) const {
    auto& net = pd_.nets[net_id];
    double area = 0;
    for (auto cid : net.cell_ids) {
        // Gate area ≈ small fraction of cell area
        area += pd_.cells[cid].width * pd_.cells[cid].height * 0.05;
    }
    return std::max(area, 0.01); // prevent division by zero
}

AntennaResult AntennaChecker::check() {
    auto t0 = std::chrono::high_resolution_clock::now();
    AntennaResult r;

    // Determine number of layers
    int max_layer = 0;
    for (auto& w : pd_.wires)
        max_layer = std::max(max_layer, w.layer);

    for (int ni = 0; ni < (int)pd_.nets.size(); ++ni) {
        r.nets_checked++;
        double gate_area = gate_area_for_net(ni);
        double cum_ratio = 0;

        for (int l = 0; l <= max_layer; ++l) {
            double wire_area = wire_area_for_net(ni, l);
            if (wire_area == 0) continue;

            double ratio = wire_area / gate_area;
            cum_ratio += ratio;

            if (ratio > rules_.max_ratio) {
                AntennaViolation v;
                v.net_id = ni;
                v.net_name = pd_.nets[ni].name;
                v.layer = l;
                v.wire_area = wire_area;
                v.gate_area = gate_area;
                v.ratio = ratio;
                v.max_ratio = rules_.max_ratio;
                v.fix_suggestion = ratio > rules_.diode_threshold
                    ? "Insert antenna diode"
                    : "Add jumper to higher metal layer";
                r.details.push_back(v);
                r.violations++;
                r.worst_ratio = std::max(r.worst_ratio, ratio);
            }
        }

        // Check cumulative ratio
        if (cum_ratio > rules_.max_cum_ratio) {
            AntennaViolation v;
            v.net_id = ni;
            v.net_name = pd_.nets[ni].name;
            v.layer = -1; // cumulative
            v.wire_area = 0;
            v.gate_area = gate_area;
            v.ratio = cum_ratio;
            v.max_ratio = rules_.max_cum_ratio;
            v.fix_suggestion = "Reduce wire length or add antenna diodes";
            r.details.push_back(v);
            r.violations++;
            r.worst_ratio = std::max(r.worst_ratio, cum_ratio);
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.nets_checked) + " nets, " +
                std::to_string(r.violations) + " antenna violations, " +
                "worst ratio: " + std::to_string((int)r.worst_ratio);
    return r;
}

} // namespace sf
