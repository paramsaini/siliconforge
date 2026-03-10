// SiliconForge — Detailed Router Implementation
// Supports dynamic layer count — distributes nets across available metal layers.
#include "pnr/detailed_router.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

namespace sf {

int DetailedRouter::coord_to_track(double coord) const {
    return std::max(0, (int)(coord / track_pitch_));
}

void DetailedRouter::build_track_grid() {
    int num_tracks = (int)(std::max(pd_.die_area.width(), pd_.die_area.height()) / track_pitch_) + 1;
    tracks_.resize(num_layers_);
    for (int l = 0; l < num_layers_; ++l)
        tracks_[l].resize(num_tracks);
}

bool DetailedRouter::is_available(int layer, int track, double start, double end) const {
    if (layer < 0 || layer >= (int)tracks_.size()) return false;
    if (track < 0 || track >= (int)tracks_[layer].size()) return false;
    for (auto& iv : tracks_[layer][track]) {
        if (!(end <= iv.start || start >= iv.end)) return false; // overlap
    }
    return true;
}

bool DetailedRouter::assign_track(int net_idx, int layer, int track, double start, double end) {
    if (!is_available(layer, track, start, end)) return false;
    tracks_[layer][track].push_back({start, end, net_idx});
    return true;
}

bool DetailedRouter::route_two_pin(int net_idx, Point p0, Point p1) {
    // Dynamic layer pair assignment:
    // Distribute nets across available layer pairs to reduce congestion.
    // Even layers = horizontal, odd layers = vertical.
    int num_pairs = std::max(1, num_layers_ / 2);
    int pair_index = net_idx % num_pairs;
    int h_layer = pair_index * 2;
    int v_layer = pair_index * 2 + 1;

    // Clamp to valid range
    if (h_layer >= num_layers_) h_layer = num_layers_ - 2;
    if (v_layer >= num_layers_) v_layer = num_layers_ - 1;
    if (h_layer < 0) h_layer = 0;
    if (v_layer < 0) v_layer = (num_layers_ > 1) ? 1 : 0;

    double x0 = std::min(p0.x, p1.x), x1 = std::max(p0.x, p1.x);
    double y0 = std::min(p0.y, p1.y), y1 = std::max(p0.y, p1.y);

    // Horizontal segment
    int h_track = coord_to_track(p0.y);
    bool h_ok = false;
    for (int dt = 0; dt < 20; ++dt) {
        int t = h_track + (dt % 2 == 0 ? dt/2 : -(dt+1)/2);
        if (t >= 0 && is_available(h_layer, t, x0, x1)) {
            assign_track(net_idx, h_layer, t, x0, x1);
            double ty = t * track_pitch_;
            pd_.wires.push_back({h_layer, {x0, ty}, {x1, ty}, track_pitch_ * 0.3});
            h_ok = true;
            break;
        }
    }

    // Vertical segment
    int v_track = coord_to_track(p0.x);
    bool v_ok = false;
    for (int dt = 0; dt < 20; ++dt) {
        int t = v_track + (dt % 2 == 0 ? dt/2 : -(dt+1)/2);
        if (t >= 0 && is_available(v_layer, t, y0, y1)) {
            assign_track(net_idx, v_layer, t, y0, y1);
            double tx = t * track_pitch_;
            pd_.wires.push_back({v_layer, {tx, y0}, {tx, y1}, track_pitch_ * 0.3});
            v_ok = true;
            break;
        }
    }

    // Add via at junction between the two layers
    if (h_ok && v_ok && h_layer != v_layer) {
        int lower = std::min(h_layer, v_layer);
        int upper = std::max(h_layer, v_layer);
        pd_.vias.push_back({{(p0.x + p1.x)/2, (p0.y + p1.y)/2}, lower, upper});
        return true;
    }
    return h_ok || v_ok;
}

DetailedRouteResult DetailedRouter::route() {
    auto t0 = std::chrono::high_resolution_clock::now();
    build_track_grid();

    DetailedRouteResult r;

    // Sort nets by bounding box (smallest first)
    std::vector<int> order(pd_.nets.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        auto bb = [&](int n) {
            if (pd_.nets[n].cell_ids.size() < 2) return 0.0;
            double xmin=1e18, xmax=-1e18, ymin=1e18, ymax=-1e18;
            for (auto c : pd_.nets[n].cell_ids) {
                xmin = std::min(xmin, pd_.cells[c].position.x);
                xmax = std::max(xmax, pd_.cells[c].position.x);
                ymin = std::min(ymin, pd_.cells[c].position.y);
                ymax = std::max(ymax, pd_.cells[c].position.y);
            }
            return (xmax-xmin)+(ymax-ymin);
        };
        return bb(a) < bb(b);
    });

    for (int ni : order) {
        auto& net = pd_.nets[ni];
        if (net.cell_ids.size() < 2) { r.routed_nets++; continue; }

        bool success = true;
        for (size_t i = 0; i + 1 < net.cell_ids.size(); ++i) {
            auto& c0 = pd_.cells[net.cell_ids[i]];
            auto& c1 = pd_.cells[net.cell_ids[i+1]];
            size_t pi0 = i < net.pin_offsets.size() ? i : 0;
            size_t pi1 = (i+1) < net.pin_offsets.size() ? (i+1) : 0;
            Point p0(c0.position.x + net.pin_offsets[pi0].x,
                     c0.position.y + net.pin_offsets[pi0].y);
            Point p1(c1.position.x + net.pin_offsets[pi1].x,
                     c1.position.y + net.pin_offsets[pi1].y);

            if (!route_two_pin(ni, p0, p1)) {
                success = false;
            } else {
                r.total_vias++;
            }
        }
        if (success) r.routed_nets++;
        else r.failed_nets++;
    }

    // Compute metrics
    r.total_wirelength = 0;
    for (auto& w : pd_.wires)
        r.total_wirelength += w.start.dist(w.end);

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.routed_nets) + "/" +
                std::to_string(r.routed_nets + r.failed_nets) + " nets, " +
                std::to_string(r.total_vias) + " vias, WL=" +
                std::to_string((int)r.total_wirelength);
    return r;
}

} // namespace sf
