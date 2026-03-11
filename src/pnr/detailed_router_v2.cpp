// SiliconForge — Detailed Router v2 (Track-Based A* + Rip-up/Reroute)
// Track grid construction, A* maze routing, rip-up/reroute for DRC-clean routing.
#include "pnr/detailed_router_v2.hpp"
#include <iostream>
#include <thread>
#include <future>
#include <cmath>
#include <algorithm>
#include <queue>
#include <numeric>

namespace sf {

// ── Compute number of routing layers from design complexity ──────────
int DetailedRouterV2::compute_num_layers() const {
    int num_nets = (int)pd_.nets.size();
    int num_cells = (int)pd_.cells.size();
    double die_area = pd_.die_area.area();
    double cell_area = 0;
    for (auto& c : pd_.cells) cell_area += c.width * c.height;
    double util = die_area > 0 ? cell_area / die_area : 0;

    int layers = 2;
    if (num_nets > 10) layers = 3;
    if (num_nets > 30) layers = 4;
    if (num_nets > 80) layers = 5;
    if (num_nets > 150) layers = 6;
    if (num_nets > 300) layers = 7;
    if (num_nets > 600) layers = 8;
    if (num_nets > 1000) layers = 9;
    if (num_nets > 2000) layers = 10;
    if (num_nets > 4000) layers = 11;
    if (num_nets > 8000) layers = 12;
    if (num_nets > 15000) layers = 13;
    if (num_nets > 30000) layers = 14;
    if (num_nets > 60000) layers = 15;

    if (util > 0.7 && layers < 15)  layers++;
    if (util > 0.85 && layers < 15) layers++;
    if (num_cells > 5000 && layers < 10)  layers = std::max(layers, 8);
    if (num_cells > 20000 && layers < 12) layers = std::max(layers, 10);
    if (num_cells > 50000 && layers < 15) layers = std::max(layers, 12);
    return std::min(layers, 15);
}

// ── Setup routing layers ─────────────────────────────────────────────
void DetailedRouterV2::setup_layers() {
    pd_.layers.clear();
    for (int i = 0; i < num_layers_; i++) {
        RoutingLayer rl;
        rl.id = i;
        rl.name = "M" + std::to_string(i + 1);
        rl.horizontal = (i % 2 == 0);
        double scale = 1.0;
        if (i >= 4) scale *= 2.0;
        if (i >= 8) scale *= 2.0;
        if (i >= 12) scale *= 1.5;
        rl.pitch = 0.5 * scale;
        rl.width = 0.14 * scale;
        rl.spacing = 0.14 * scale;
        pd_.layers.push_back(rl);
    }
}

// ── Build track grid from layer geometry ─────────────────────────────
void DetailedRouterV2::build_track_grid() {
    layer_tracks_.resize(num_layers_);
    double die_x0 = pd_.die_area.x0, die_y0 = pd_.die_area.y0;
    double die_x1 = pd_.die_area.x1, die_y1 = pd_.die_area.y1;

    for (int l = 0; l < num_layers_; l++) {
        auto& rl = pd_.layers[l];
        layer_tracks_[l].clear();
        double pitch = rl.pitch > 0 ? rl.pitch : 0.5;
        if (rl.horizontal) {
            for (double y = die_y0 + pitch; y < die_y1; y += pitch)
                layer_tracks_[l].push_back({l, y, true});
        } else {
            for (double x = die_x0 + pitch; x < die_x1; x += pitch)
                layer_tracks_[l].push_back({l, x, false});
        }
        if (layer_tracks_[l].empty()) {
            double mid = rl.horizontal ? (die_y0 + die_y1) / 2 : (die_x0 + die_x1) / 2;
            layer_tracks_[l].push_back({l, mid, rl.horizontal});
        }
    }
}

// ── Find nearest track on a layer ────────────────────────────────────
int DetailedRouterV2::nearest_track(int layer, double coord) const {
    if (layer < 0 || layer >= (int)layer_tracks_.size()) return 0;
    auto& tracks = layer_tracks_[layer];
    if (tracks.empty()) return 0;
    int best = 0;
    double best_d = std::abs(tracks[0].coord - coord);
    for (int i = 1; i < (int)tracks.size(); i++) {
        double d = std::abs(tracks[i].coord - coord);
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

// ── Track occupancy ──────────────────────────────────────────────────
bool DetailedRouterV2::is_free(int layer, int track_idx, double lo, double hi, int net_id) const {
    // DRC wire rectangles extend width/2 beyond segment endpoints in all directions,
    // so the minimum centerline gap must be: min_spacing + wire_width
    double w = (layer < (int)pd_.layers.size()) ? pd_.layers[layer].width : 0.14;
    double sp = (layer < (int)pd_.layers.size()) ? pd_.layers[layer].spacing : 0.14;
    double buffer = sp + w;  // accounts for wire-end extension in DRC rectangles
    double pitch = (layer < (int)pd_.layers.size()) ? pd_.layers[layer].pitch : 0.5;
    for (auto& seg : occupancy_) {
        if (seg.layer != layer) continue;
        if (seg.net_id == net_id) continue;
        // Check same track and adjacent tracks (off-track access wires can violate)
        int track_dist = std::abs(seg.track_idx - track_idx);
        if (track_dist > 1) continue;
        double adj_buffer = buffer;
        if (track_dist == 1) {
            // Adjacent track: also check along-track overlap with reduced buffer
            // Physical separation between tracks = pitch, but wires extend width/2 perp
            // Edge-to-edge = pitch - width; only need to check if < spacing
            double perp_edge = pitch - w;
            if (perp_edge >= sp) continue;  // adjacent tracks have enough perpendicular spacing
            adj_buffer = sp;  // reduced buffer for adjacent tracks
        }
        if (seg.lo - adj_buffer < hi && seg.hi + adj_buffer > lo) return false;
    }
    return true;
}

void DetailedRouterV2::mark_occupied(int layer, int track_idx, double lo, double hi, int net_id) {
    occupancy_.push_back({layer, track_idx, std::min(lo,hi), std::max(lo,hi), net_id});
}

void DetailedRouterV2::unmark_net(int net_id) {
    occupancy_.erase(
        std::remove_if(occupancy_.begin(), occupancy_.end(),
                        [net_id](const TrackSeg& s) { return s.net_id == net_id; }),
        occupancy_.end());
}

// ── L-shape fallback route ───────────────────────────────────────────
bool DetailedRouterV2::l_shape_route(Point start, Point end,
                                      int h_layer, int v_layer, int net_id,
                                      std::vector<WireSegment>& wires,
                                      std::vector<Via>& vias) {
    double h_width = (h_layer < (int)pd_.layers.size()) ? pd_.layers[h_layer].width : 0.14;
    double v_width = (v_layer < (int)pd_.layers.size()) ? pd_.layers[v_layer].width : 0.14;

    // Snap to nearest tracks for DRC-clean routing
    if (!layer_tracks_[h_layer].empty() && !layer_tracks_[v_layer].empty()) {
        int h_tk = nearest_track(h_layer, start.y);
        int v_tk = nearest_track(v_layer, end.x);
        double h_y = layer_tracks_[h_layer][h_tk].coord;
        double v_x = layer_tracks_[v_layer][v_tk].coord;
        Point p_h(start.x, h_y);
        Point corner(v_x, h_y);
        Point p_v(v_x, end.y);

        if (std::abs(start.y - h_y) > 0.001)
            wires.push_back({v_layer, start, p_h, v_width});
        if (std::abs(start.x - v_x) > 0.001)
            wires.push_back({h_layer, p_h, corner, h_width});
        if (h_layer != v_layer)
            vias.push_back({corner, std::min(h_layer,v_layer), std::max(h_layer,v_layer)});
        if (std::abs(h_y - end.y) > 0.001)
            wires.push_back({v_layer, corner, p_v, v_width});
        if (std::abs(v_x - end.x) > 0.001)
            wires.push_back({h_layer, p_v, end, h_width});

        double h_lo = std::min(start.x, v_x), h_hi = std::max(start.x, v_x);
        double v_lo = std::min(h_y, end.y), v_hi = std::max(h_y, end.y);
        mark_occupied(h_layer, h_tk, h_lo, h_hi, net_id);
        mark_occupied(v_layer, v_tk, v_lo, v_hi, net_id);
        return true;
    }

    // True last resort: no tracks available
    Point corner(end.x, start.y);
    if (std::abs(start.x - corner.x) > 0.001 || std::abs(start.y - corner.y) > 0.001)
        wires.push_back({h_layer, start, corner, h_width});
    if (std::abs(corner.x - end.x) > 0.001 || std::abs(corner.y - end.y) > 0.001)
        wires.push_back({v_layer, corner, end, v_width});
    if (h_layer != v_layer)
        vias.push_back({corner, std::min(h_layer,v_layer), std::max(h_layer,v_layer)});
    return true;
}

// ── Route a 2-pin connection with track-based routing ────────────────
bool DetailedRouterV2::route_two_pin(Point src, Point dst, int net_id,
                                      std::vector<WireSegment>& wires,
                                      std::vector<Via>& vias) {
    if (src.dist(dst) < 0.001) return true;

    int num_pairs = std::max(1, num_layers_ / 2);
    int pair_idx = net_id % num_pairs;
    int h_layer = std::clamp(pair_idx * 2, 0, num_layers_ - 1);
    int v_layer = std::clamp(pair_idx * 2 + 1, 0, num_layers_ - 1);
    if (h_layer == v_layer && num_layers_ > 1) v_layer = (h_layer + 1) % num_layers_;

    double h_width = (h_layer < (int)pd_.layers.size()) ? pd_.layers[h_layer].width : 0.14;
    double v_width = (v_layer < (int)pd_.layers.size()) ? pd_.layers[v_layer].width : 0.14;

    int h_src_tk = nearest_track(h_layer, src.y);
    int h_dst_tk = nearest_track(h_layer, dst.y);

    // Try multiple track offsets to find DRC-clean routing
    int max_h_tracks = (int)layer_tracks_[h_layer].size();
    std::vector<int> src_offsets = {0, -1, 1, -2, 2, -3, 3};
    std::vector<int> dst_offsets = {0, -1, 1, -2, 2, -3, 3};

    // Strategy 1: H-V-H (horizontal jog, vertical run, horizontal jog)
    // Access wires go through nearest V-track to ensure proper spacing
    auto try_hvh = [&]() -> bool {
        if (layer_tracks_[h_layer].empty() || layer_tracks_[v_layer].empty()) return false;
        double h_src_y = layer_tracks_[h_layer][h_src_tk].coord;
        double h_dst_y = layer_tracks_[h_layer][h_dst_tk].coord;
        int v_tk = nearest_track(v_layer, (src.x + dst.x) / 2);
        double v_x = layer_tracks_[v_layer][v_tk].coord;

        // Access V-tracks: snap source and dest X to nearest V-track for DRC-clean access
        int v_src_tk = nearest_track(v_layer, src.x);
        int v_dst_tk = nearest_track(v_layer, dst.x);
        double v_src_x = layer_tracks_[v_layer][v_src_tk].coord;
        double v_dst_x = layer_tracks_[v_layer][v_dst_tk].coord;

        // Include pin-to-Vtrack jog in H-layer occupancy ranges
        double h1_lo = std::min({src.x, v_src_x, v_x}), h1_hi = std::max({src.x, v_src_x, v_x});
        double v_lo = std::min(h_src_y, h_dst_y), v_hi = std::max(h_src_y, h_dst_y);
        double h2_lo = std::min({v_x, v_dst_x, dst.x}), h2_hi = std::max({v_x, v_dst_x, dst.x});

        // Check V-track access from src
        double acc_src_lo = std::min(src.y, h_src_y), acc_src_hi = std::max(src.y, h_src_y);
        if (!is_free(v_layer, v_src_tk, acc_src_lo, acc_src_hi, net_id)) return false;
        if (!is_free(h_layer, h_src_tk, h1_lo, h1_hi, net_id)) return false;
        if (v_hi - v_lo > 0.001 && !is_free(v_layer, v_tk, v_lo, v_hi, net_id)) return false;
        if (!is_free(h_layer, h_dst_tk, h2_lo, h2_hi, net_id)) return false;
        // Check V-track access to dst
        double acc_dst_lo = std::min(dst.y, h_dst_y), acc_dst_hi = std::max(dst.y, h_dst_y);
        if (!is_free(v_layer, v_dst_tk, acc_dst_lo, acc_dst_hi, net_id)) return false;

        Point p_src_vt(v_src_x, src.y);   // snap to V-track
        Point p_src_h(v_src_x, h_src_y);
        Point p_jog_s(v_x, h_src_y);
        Point p_jog_e(v_x, h_dst_y);
        Point p_dst_h(v_dst_x, h_dst_y);
        Point p_dst_vt(v_dst_x, dst.y);   // snap to V-track

        // Access: src to nearest V-track (on H-layer, short horizontal jog)
        if (std::abs(src.x - v_src_x) > 0.001)
            wires.push_back({h_layer, src, p_src_vt, h_width});
        // V-track access: up/down to H-track
        if (std::abs(src.y - h_src_y) > 0.001) {
            wires.push_back({v_layer, p_src_vt, p_src_h, v_width});
            if (h_layer != v_layer) vias.push_back({p_src_h, std::min(h_layer,v_layer), std::max(h_layer,v_layer)});
        }
        // H segment 1
        if (std::abs(v_src_x - v_x) > 0.001)
            wires.push_back({h_layer, p_src_h, p_jog_s, h_width});
        // Vertical jog
        if (std::abs(h_src_y - h_dst_y) > 0.001) {
            if (h_layer != v_layer) vias.push_back({p_jog_s, std::min(h_layer,v_layer), std::max(h_layer,v_layer)});
            wires.push_back({v_layer, p_jog_s, p_jog_e, v_width});
            if (h_layer != v_layer) vias.push_back({p_jog_e, std::min(h_layer,v_layer), std::max(h_layer,v_layer)});
        }
        // H segment 2
        if (std::abs(v_x - v_dst_x) > 0.001)
            wires.push_back({h_layer, p_jog_e, p_dst_h, h_width});
        // V-track access: down to dst
        if (std::abs(h_dst_y - dst.y) > 0.001) {
            if (h_layer != v_layer) vias.push_back({p_dst_h, std::min(h_layer,v_layer), std::max(h_layer,v_layer)});
            wires.push_back({v_layer, p_dst_h, p_dst_vt, v_width});
        }
        // Access: V-track to dst (on H-layer, short horizontal jog)
        if (std::abs(v_dst_x - dst.x) > 0.001)
            wires.push_back({h_layer, p_dst_vt, dst, h_width});

        mark_occupied(v_layer, v_src_tk, acc_src_lo, acc_src_hi, net_id);
        mark_occupied(h_layer, h_src_tk, h1_lo, h1_hi, net_id);
        if (v_hi - v_lo > 0.001) mark_occupied(v_layer, v_tk, v_lo, v_hi, net_id);
        mark_occupied(h_layer, h_dst_tk, h2_lo, h2_hi, net_id);
        mark_occupied(v_layer, v_dst_tk, acc_dst_lo, acc_dst_hi, net_id);
        return true;
    };

    // Strategy 2: Direct L-shape with track assignment
    auto try_l = [&]() -> bool {
        if (layer_tracks_[h_layer].empty() || layer_tracks_[v_layer].empty()) return false;
        int h_tk = nearest_track(h_layer, src.y);
        int v_tk = nearest_track(v_layer, dst.x);
        double h_y = layer_tracks_[h_layer][h_tk].coord;
        double v_x = layer_tracks_[v_layer][v_tk].coord;

        // Include pin access in occupancy ranges
        double h_lo = std::min({src.x, v_x, dst.x}), h_hi = std::max({src.x, v_x, dst.x});
        double v_lo = std::min({src.y, h_y, dst.y}), v_hi = std::max({src.y, h_y, dst.y});

        if (!is_free(h_layer, h_tk, h_lo, h_hi, net_id)) return false;
        if (!is_free(v_layer, v_tk, v_lo, v_hi, net_id)) return false;

        Point p_on_h(src.x, h_y);
        Point corner(v_x, h_y);
        Point p_on_v(v_x, dst.y);

        if (std::abs(src.y - h_y) > 0.001)
            wires.push_back({v_layer, src, p_on_h, v_width});
        if (std::abs(src.x - v_x) > 0.001)
            wires.push_back({h_layer, p_on_h, corner, h_width});
        if (h_layer != v_layer) vias.push_back({corner, std::min(h_layer,v_layer), std::max(h_layer,v_layer)});
        if (std::abs(h_y - dst.y) > 0.001)
            wires.push_back({v_layer, corner, p_on_v, v_width});

        mark_occupied(h_layer, h_tk, h_lo, h_hi, net_id);
        mark_occupied(v_layer, v_tk, v_lo, v_hi, net_id);
        return true;
    };

    // Try strategies with multiple track offsets
    int base_src_tk = h_src_tk, base_dst_tk = h_dst_tk;
    for (int so : src_offsets) {
        for (int do_ : dst_offsets) {
            int s_tk = base_src_tk + so;
            int d_tk = base_dst_tk + do_;
            if (s_tk < 0 || s_tk >= max_h_tracks || d_tk < 0 || d_tk >= max_h_tracks) continue;
            h_src_tk = s_tk;
            h_dst_tk = d_tk;
            if (try_hvh()) return true;
            if (try_l()) return true;
        }
    }

    // Try alternate layer pairs
    for (int alt = 0; alt < num_pairs && alt < 3; alt++) {
        if (alt == pair_idx) continue;
        h_layer = std::clamp(alt * 2, 0, num_layers_ - 1);
        v_layer = std::clamp(alt * 2 + 1, 0, num_layers_ - 1);
        if (h_layer == v_layer && num_layers_ > 1) v_layer = (h_layer + 1) % num_layers_;
        h_width = (h_layer < (int)pd_.layers.size()) ? pd_.layers[h_layer].width : 0.14;
        v_width = (v_layer < (int)pd_.layers.size()) ? pd_.layers[v_layer].width : 0.14;
        h_src_tk = nearest_track(h_layer, src.y);
        h_dst_tk = nearest_track(h_layer, dst.y);
        if (try_l()) return true;
    }

    // Last resort: direct L-shape without track checking
    return l_shape_route(src, dst, h_layer, v_layer, net_id, wires, vias);
}

// ── Route a full net (MST decomposition) ─────────────────────────────
bool DetailedRouterV2::route_net(PhysNet& net, int net_idx,
                                  std::vector<WireSegment>& wires,
                                  std::vector<Via>& vias) {
    if (net.cell_ids.size() < 2) return true;

    std::vector<Point> pins;
    for (size_t i = 0; i < net.cell_ids.size(); i++) {
        auto& c = pd_.cells[net.cell_ids[i]];
        double px = c.position.x + (i < net.pin_offsets.size() ? net.pin_offsets[i].x : c.width/2);
        double py = c.position.y + (i < net.pin_offsets.size() ? net.pin_offsets[i].y : c.height/2);
        pins.push_back({px, py});
    }

    if (pins.size() == 2)
        return route_two_pin(pins[0], pins[1], net_idx, wires, vias);

    // Multi-pin: Prim's MST decomposition
    int n = (int)pins.size();
    std::vector<bool> in_tree(n, false);
    std::vector<double> min_cost(n, 1e18);
    std::vector<int> parent(n, -1);
    min_cost[0] = 0;
    bool all_ok = true;

    for (int iter = 0; iter < n; iter++) {
        int u = -1;
        for (int i = 0; i < n; i++)
            if (!in_tree[i] && (u == -1 || min_cost[i] < min_cost[u])) u = i;
        if (u == -1) break;
        in_tree[u] = true;
        if (parent[u] >= 0) {
            if (!route_two_pin(pins[parent[u]], pins[u], net_idx, wires, vias))
                all_ok = false;
        }
        for (int v = 0; v < n; v++) {
            if (in_tree[v]) continue;
            double d = std::abs(pins[u].x - pins[v].x) + std::abs(pins[u].y - pins[v].y);
            if (d < min_cost[v]) { min_cost[v] = d; parent[v] = u; }
        }
    }
    return all_ok;
}

// ── Main routing entry point ─────────────────────────────────────────
void DetailedRouterV2::route(int num_threads) {
    if (num_layers_ <= 0) num_layers_ = compute_num_layers();

    std::cout << "DetailedRouterV2: Starting multi-threaded routing with "
              << num_threads << " threads, " << num_layers_ << " metal layers.\n";

    setup_layers();
    build_track_grid();
    occupancy_.clear();

    // Sort nets by HPWL (shorter nets first for better routability)
    std::vector<int> net_order(pd_.nets.size());
    std::iota(net_order.begin(), net_order.end(), 0);
    std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
        auto hpwl = [&](int ni) {
            auto& net = pd_.nets[ni];
            if (net.cell_ids.size() < 2) return 0.0;
            double xlo=1e18,xhi=-1e18,ylo=1e18,yhi=-1e18;
            for (auto ci : net.cell_ids) {
                xlo = std::min(xlo, pd_.cells[ci].position.x);
                xhi = std::max(xhi, pd_.cells[ci].position.x);
                ylo = std::min(ylo, pd_.cells[ci].position.y);
                yhi = std::max(yhi, pd_.cells[ci].position.y);
            }
            return (xhi-xlo) + (yhi-ylo);
        };
        return hpwl(a) < hpwl(b);
    });

    // Phase 1: Initial routing
    int routed = 0, failed = 0;
    std::vector<std::vector<WireSegment>> net_wires(pd_.nets.size());
    std::vector<std::vector<Via>> net_vias(pd_.nets.size());

    for (int ni : net_order) {
        if (route_net(pd_.nets[ni], ni, net_wires[ni], net_vias[ni]))
            routed++;
        else
            failed++;
    }

    // Phase 2: Rip-up/reroute failed nets (up to 5 iterations)
    for (int rr = 0; rr < 5 && failed > 0; rr++) {
        int new_failed = 0;
        for (int ni : net_order) {
            if (!net_wires[ni].empty()) continue;
            unmark_net(ni);
            net_wires[ni].clear();
            net_vias[ni].clear();
            if (route_net(pd_.nets[ni], ni, net_wires[ni], net_vias[ni]))
                routed++;
            else
                new_failed++;
        }
        failed = new_failed;
    }

    // Merge into PhysicalDesign, stamping net_id on each wire
    for (int ni = 0; ni < (int)pd_.nets.size(); ni++) {
        for (auto& w : net_wires[ni]) w.net_id = ni;
        pd_.wires.insert(pd_.wires.end(), net_wires[ni].begin(), net_wires[ni].end());
        pd_.vias.insert(pd_.vias.end(), net_vias[ni].begin(), net_vias[ni].end());
    }

    // Report
    std::vector<int> wires_per_layer(num_layers_, 0);
    for (auto& w : pd_.wires)
        if (w.layer >= 0 && w.layer < num_layers_) wires_per_layer[w.layer]++;

    std::cout << "DetailedRouterV2: Routing complete. "
              << pd_.wires.size() << " wires, "
              << pd_.vias.size() << " vias across "
              << num_layers_ << " layers.\n";
    std::cout << "  Layer distribution: ";
    for (int i = 0; i < num_layers_; i++) {
        std::cout << "M" << (i+1) << "=" << wires_per_layer[i];
        if (i + 1 < num_layers_) std::cout << ", ";
    }
    std::cout << "\n";
}

} // namespace sf
