// SiliconForge — Detailed Router v2 (Track-Based A* + Rip-up/Reroute)
// Industrial: timing-driven net ordering, via minimization, antenna tracking.
#include "pnr/detailed_router_v2.hpp"
#include "core/thread_pool.hpp"
#include <iostream>
#include <thread>
#include <future>
#include <cmath>
#include <algorithm>
#include <queue>
#include <numeric>
#include <chrono>
#include <atomic>

namespace sf {

// ── TrackOccupancy — sorted-interval occupancy for O(log k) queries ──

void TrackOccupancy::clear() {
    tracks_.clear();
}

void TrackOccupancy::insert(int layer, int track_idx, double lo, double hi, int net_id) {
    double a = std::min(lo, hi);
    double b = std::max(lo, hi);
    int key = make_key(layer, track_idx);
    auto& vec = tracks_[key];
    // Insert maintaining sort by lo using binary search
    Interval iv{a, b, net_id};
    auto it = std::lower_bound(vec.begin(), vec.end(), iv,
        [](const Interval& x, const Interval& y) { return x.lo < y.lo; });
    vec.insert(it, iv);
}

bool TrackOccupancy::has_overlap(const std::vector<Interval>& intervals, double lo, double hi,
                                  double buffer, int exclude_net) const {
    if (intervals.empty()) return false;
    // We need to find intervals where: iv.lo - buffer < hi  AND  iv.hi + buffer > lo
    // Since intervals are sorted by iv.lo, find the first where iv.lo >= lo - buffer_max
    // (conservative start point). We use iv.hi + buffer > lo as a post-filter.
    //
    // Binary search for first interval where iv.lo >= lo - buffer - max_possible_width.
    // Conservative: start from first interval whose lo could possibly satisfy overlap.
    // An interval overlaps if:  iv.lo - buffer < hi  AND  iv.hi + buffer > lo
    // Rearranged: iv.lo < hi + buffer.  So find first iv with iv.lo >= lo - buffer
    // (but iv.hi could extend rightward), so we need to start scanning earlier.
    //
    // Strategy: binary search for first iv where iv.lo >= lo - buffer, but we must also
    // catch intervals that start before lo-buffer but extend past it (iv.hi + buffer > lo).
    // Those intervals have iv.lo < lo - buffer but iv.hi > lo - buffer.
    // To handle that: also scan backward from the found position.

    // Find first interval with lo >= (query_lo - buffer)
    // But intervals starting before that point could still overlap if their hi extends far enough.
    // We handle this by also checking a few intervals before the lower_bound position.

    Interval sentinel{lo - buffer, 0.0, 0};
    auto it = std::lower_bound(intervals.begin(), intervals.end(), sentinel,
        [](const Interval& x, const Interval& y) { return x.lo < y.lo; });

    // Scan backward to catch intervals that start before lo-buffer but whose hi extends into range.
    // These intervals satisfy: iv.lo < lo - buffer AND iv.hi + buffer > lo.
    // Since intervals are sorted by lo, we scan backward until iv.lo is too small for iv.hi to matter.
    // In practice, we check all intervals from it backward while iv.hi + buffer > lo.
    if (it != intervals.begin()) {
        auto back = it;
        do {
            --back;
            if (back->net_id != exclude_net) {
                if (back->lo - buffer < hi && back->hi + buffer > lo) return true;
            }
            // Once hi + buffer <= lo, no earlier interval can overlap either
            if (back->hi + buffer <= lo) break;
        } while (back != intervals.begin());
    }

    // Scan forward: check intervals while iv.lo - buffer < hi (could still overlap)
    for (auto fwd = it; fwd != intervals.end(); ++fwd) {
        if (fwd->lo - buffer >= hi) break;  // sorted by lo, no further overlap possible
        if (fwd->net_id == exclude_net) continue;
        if (fwd->hi + buffer > lo) return true;
    }
    return false;
}

bool TrackOccupancy::is_free(int layer, int track_idx, double lo, double hi, int net_id,
                              double buffer, double adj_buffer, double pitch,
                              double wire_width, double spacing) const {
    // Same track check
    int key = make_key(layer, track_idx);
    auto it = tracks_.find(key);
    if (it != tracks_.end()) {
        if (has_overlap(it->second, lo, hi, buffer, net_id)) return false;
    }
    // Adjacent track checks (track_idx - 1, track_idx + 1)
    double perp_edge = pitch - wire_width;
    if (perp_edge < spacing) {
        for (int delta : {-1, 1}) {
            int adj_key = make_key(layer, track_idx + delta);
            auto adj_it = tracks_.find(adj_key);
            if (adj_it != tracks_.end()) {
                if (has_overlap(adj_it->second, lo, hi, adj_buffer, net_id)) return false;
            }
        }
    }
    return true;
}

bool TrackOccupancy::is_free_with_adj(int layer, int track_idx, double lo, double hi, int net_id,
                                       double wire_width, double spacing, double pitch) const {
    double buffer = spacing + wire_width;
    double adj_buffer = spacing;
    return is_free(layer, track_idx, lo, hi, net_id, buffer, adj_buffer, pitch, wire_width, spacing);
}

void TrackOccupancy::remove_net(int net_id) {
    for (auto& kv : tracks_) {
        auto& vec = kv.second;
        vec.erase(std::remove_if(vec.begin(), vec.end(),
            [net_id](const Interval& iv) { return iv.net_id == net_id; }),
            vec.end());
    }
}

size_t TrackOccupancy::size() const {
    size_t total = 0;
    for (auto& kv : tracks_) total += kv.second.size();
    return total;
}

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
    // DRC parameters for this layer
    double w = (layer < (int)pd_.layers.size()) ? pd_.layers[layer].width : 0.14;
    double sp = (layer < (int)pd_.layers.size()) ? pd_.layers[layer].spacing : 0.14;
    double pitch = (layer < (int)pd_.layers.size()) ? pd_.layers[layer].pitch : 0.5;
    // Delegate to interval-tree occupancy for O(log k) per-track query
    return occ_.is_free_with_adj(layer, track_idx, lo, hi, net_id, w, sp, pitch);
}

void DetailedRouterV2::mark_occupied(int layer, int track_idx, double lo, double hi, int net_id) {
    occupancy_.push_back({layer, track_idx, std::min(lo,hi), std::max(lo,hi), net_id});
    occ_.insert(layer, track_idx, lo, hi, net_id);
}

void DetailedRouterV2::unmark_net(int net_id) {
    occupancy_.erase(
        std::remove_if(occupancy_.begin(), occupancy_.end(),
                        [net_id](const TrackSeg& s) { return s.net_id == net_id; }),
        occupancy_.end());
    occ_.remove_net(net_id);
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

// ── Pattern routing fast-path: L/Z shapes before A* ──────────────────
// ~80% of two-pin connections can be routed with simple geometric patterns,
// avoiding the O(N log N) overhead of A* maze expansion.  This enables
// embarrassingly-parallel first-pass routing for the majority of nets.
bool DetailedRouterV2::try_pattern_route(Point src, Point dst, int net_id,
                                          std::vector<WireSegment>& wires,
                                          std::vector<Via>& vias) {
    if (src.dist(dst) < 0.001) return true;
    if (num_layers_ < 2 || layer_tracks_.empty()) return false;

    int num_pairs = std::max(1, num_layers_ / 2);
    int pair_idx = net_id % num_pairs;
    int h_layer = std::clamp(pair_idx * 2, 0, num_layers_ - 1);
    int v_layer = std::clamp(pair_idx * 2 + 1, 0, num_layers_ - 1);
    if (h_layer == v_layer && num_layers_ > 1)
        v_layer = (h_layer + 1) % num_layers_;

    if (h_layer >= (int)layer_tracks_.size() || v_layer >= (int)layer_tracks_.size())
        return false;
    if (layer_tracks_[h_layer].empty() || layer_tracks_[v_layer].empty())
        return false;

    double h_width = (h_layer < (int)pd_.layers.size()) ? pd_.layers[h_layer].width : 0.14;
    double v_width = (v_layer < (int)pd_.layers.size()) ? pd_.layers[v_layer].width : 0.14;

    // ── Pattern 1: L-shape horizontal-first ─────────────────────────
    // src -> (dst.x, src.y) -> dst
    {
        int h_tk = nearest_track(h_layer, src.y);
        int v_tk = nearest_track(v_layer, dst.x);
        double h_y = layer_tracks_[h_layer][h_tk].coord;
        double v_x = layer_tracks_[v_layer][v_tk].coord;

        double h_lo = std::min(src.x, v_x), h_hi = std::max(src.x, v_x);
        double v_lo = std::min(h_y, dst.y), v_hi = std::max(h_y, dst.y);

        if (is_free(h_layer, h_tk, h_lo, h_hi, net_id) &&
            is_free(v_layer, v_tk, v_lo, v_hi, net_id)) {
            Point on_h(src.x, h_y);
            Point bend(v_x, h_y);
            Point on_v(v_x, dst.y);

            if (std::abs(src.y - h_y) > 0.001)
                wires.push_back({v_layer, src, on_h, v_width});
            if (std::abs(src.x - v_x) > 0.001)
                wires.push_back({h_layer, on_h, bend, h_width});
            if (h_layer != v_layer)
                vias.push_back({bend, std::min(h_layer, v_layer), std::max(h_layer, v_layer)});
            if (std::abs(h_y - dst.y) > 0.001)
                wires.push_back({v_layer, bend, on_v, v_width});
            if (std::abs(v_x - dst.x) > 0.001)
                wires.push_back({h_layer, on_v, dst, h_width});

            mark_occupied(h_layer, h_tk, h_lo, h_hi, net_id);
            mark_occupied(v_layer, v_tk, v_lo, v_hi, net_id);
            return true;
        }
    }

    // ── Pattern 2: L-shape vertical-first ───────────────────────────
    // src -> (src.x, dst.y) -> dst
    {
        int v_tk = nearest_track(v_layer, src.x);
        int h_tk = nearest_track(h_layer, dst.y);
        double v_x = layer_tracks_[v_layer][v_tk].coord;
        double h_y = layer_tracks_[h_layer][h_tk].coord;

        double v_lo = std::min(src.y, h_y), v_hi = std::max(src.y, h_y);
        double h_lo = std::min(v_x, dst.x), h_hi = std::max(v_x, dst.x);

        if (is_free(v_layer, v_tk, v_lo, v_hi, net_id) &&
            is_free(h_layer, h_tk, h_lo, h_hi, net_id)) {
            Point on_v(v_x, src.y);
            Point bend(v_x, h_y);
            Point on_h(dst.x, h_y);

            if (std::abs(src.x - v_x) > 0.001)
                wires.push_back({h_layer, src, on_v, h_width});
            if (std::abs(src.y - h_y) > 0.001)
                wires.push_back({v_layer, on_v, bend, v_width});
            if (h_layer != v_layer)
                vias.push_back({bend, std::min(h_layer, v_layer), std::max(h_layer, v_layer)});
            if (std::abs(v_x - dst.x) > 0.001)
                wires.push_back({h_layer, bend, on_h, h_width});
            if (std::abs(h_y - dst.y) > 0.001)
                wires.push_back({v_layer, on_h, dst, v_width});

            mark_occupied(v_layer, v_tk, v_lo, v_hi, net_id);
            mark_occupied(h_layer, h_tk, h_lo, h_hi, net_id);
            return true;
        }
    }

    // ── Pattern 3: Z-shape HVH ──────────────────────────────────────
    // src -> (mid_x, src.y) -> (mid_x, dst.y) -> dst
    // where mid_x = (src.x + dst.x) / 2
    {
        double mid_x = (src.x + dst.x) / 2.0;
        int h_src_tk = nearest_track(h_layer, src.y);
        int h_dst_tk = nearest_track(h_layer, dst.y);
        int v_mid_tk = nearest_track(v_layer, mid_x);
        double h_src_y = layer_tracks_[h_layer][h_src_tk].coord;
        double h_dst_y = layer_tracks_[h_layer][h_dst_tk].coord;
        double v_mid_x = layer_tracks_[v_layer][v_mid_tk].coord;

        double h1_lo = std::min(src.x, v_mid_x), h1_hi = std::max(src.x, v_mid_x);
        double v_lo  = std::min(h_src_y, h_dst_y), v_hi = std::max(h_src_y, h_dst_y);
        double h2_lo = std::min(v_mid_x, dst.x), h2_hi = std::max(v_mid_x, dst.x);

        if (is_free(h_layer, h_src_tk, h1_lo, h1_hi, net_id) &&
            (v_hi - v_lo < 0.001 || is_free(v_layer, v_mid_tk, v_lo, v_hi, net_id)) &&
            is_free(h_layer, h_dst_tk, h2_lo, h2_hi, net_id)) {

            Point p_src_h(src.x, h_src_y);
            Point jog_s(v_mid_x, h_src_y);
            Point jog_e(v_mid_x, h_dst_y);
            Point p_dst_h(dst.x, h_dst_y);

            // Access: src to H-track
            if (std::abs(src.y - h_src_y) > 0.001)
                wires.push_back({v_layer, src, p_src_h, v_width});
            // H segment 1
            if (std::abs(src.x - v_mid_x) > 0.001)
                wires.push_back({h_layer, p_src_h, jog_s, h_width});
            // Via at first bend
            if (h_layer != v_layer && v_hi - v_lo > 0.001)
                vias.push_back({jog_s, std::min(h_layer, v_layer), std::max(h_layer, v_layer)});
            // V segment (the jog)
            if (std::abs(h_src_y - h_dst_y) > 0.001)
                wires.push_back({v_layer, jog_s, jog_e, v_width});
            // Via at second bend
            if (h_layer != v_layer && v_hi - v_lo > 0.001)
                vias.push_back({jog_e, std::min(h_layer, v_layer), std::max(h_layer, v_layer)});
            // H segment 2
            if (std::abs(v_mid_x - dst.x) > 0.001)
                wires.push_back({h_layer, jog_e, p_dst_h, h_width});
            // Access: H-track to dst
            if (std::abs(h_dst_y - dst.y) > 0.001)
                wires.push_back({v_layer, p_dst_h, dst, v_width});

            mark_occupied(h_layer, h_src_tk, h1_lo, h1_hi, net_id);
            if (v_hi - v_lo > 0.001)
                mark_occupied(v_layer, v_mid_tk, v_lo, v_hi, net_id);
            mark_occupied(h_layer, h_dst_tk, h2_lo, h2_hi, net_id);
            return true;
        }
    }

    // All patterns failed -- caller should fall back to full maze routing
    return false;
}

// ── Route a 2-pin connection with track-based routing ────────────────
bool DetailedRouterV2::route_two_pin(Point src, Point dst, int net_id,
                                      std::vector<WireSegment>& wires,
                                      std::vector<Via>& vias) {
    if (src.dist(dst) < 0.001) return true;

    // Fast-path: try simple geometric patterns before expensive maze routing
    if (try_pattern_route(src, dst, net_id, wires, vias))
        return true;

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

// ============================================================================
// Industrial: net criticality accessor
// ============================================================================
double DetailedRouterV2::get_criticality(int net_id) const {
    auto it = net_timing_.find(net_id);
    if (it == net_timing_.end()) return 0.0;
    return it->second.criticality;
}

// ============================================================================
// Industrial: count vias for a net
// ============================================================================
int DetailedRouterV2::count_net_vias(const std::vector<Via>& vias) const {
    return (int)vias.size();
}

// ============================================================================
// Industrial: antenna ratio check per net
// ============================================================================
AntennaViolation DetailedRouterV2::check_antenna(
    int net_id, const std::vector<WireSegment>& wires) const
{
    AntennaViolation av;
    av.net_id = net_id;
    av.net_name = net_id >= 0 && net_id < (int)pd_.nets.size() ? pd_.nets[net_id].name : "";
    double wire_area = 0;
    for (auto& w : wires) {
        wire_area += w.start.dist(w.end) * w.width;
    }
    double gate_area = 0;
    if (net_id >= 0 && net_id < (int)pd_.nets.size()) {
        for (auto ci : pd_.nets[net_id].cell_ids) {
            if (ci >= 0 && ci < (int)pd_.cells.size())
                gate_area += pd_.cells[ci].width * 0.05;
        }
    }
    av.wire_area = wire_area;
    av.gate_area = gate_area;
    av.ratio = gate_area > 0 ? wire_area / gate_area : 0;
    av.max_ratio = config_.max_antenna_ratio;
    av.layer = -1;
    if (av.ratio > config_.max_antenna_ratio)
        av.fix_suggestion = "Insert diode or break wire";
    return av;
}

// ── Main routing entry point (industrial: returns RouteResult) ───────
RouteResult DetailedRouterV2::route(int num_threads) {
    auto t0 = std::chrono::high_resolution_clock::now();
    RouteResult result;

    if (num_layers_ <= 0) num_layers_ = compute_num_layers();

    std::cout << "DetailedRouterV2: Starting routing with "
              << num_layers_ << " metal layers.\n";

    setup_layers();
    build_track_grid();
    occupancy_.clear();
    occ_.clear();

    // HPWL computation for net ordering
    auto compute_hpwl = [&](int ni) -> double {
        auto& net = pd_.nets[ni];
        if (net.cell_ids.size() < 2) return 0.0;
        double xlo = 1e18, xhi = -1e18, ylo = 1e18, yhi = -1e18;
        for (auto ci : net.cell_ids) {
            xlo = std::min(xlo, pd_.cells[ci].position.x);
            xhi = std::max(xhi, pd_.cells[ci].position.x);
            ylo = std::min(ylo, pd_.cells[ci].position.y);
            yhi = std::max(yhi, pd_.cells[ci].position.y);
        }
        return (xhi - xlo) + (yhi - ylo);
    };

    // Net ordering: sort by HPWL descending (longest nets first) to reduce
    // congestion from late-arriving long nets.  Critical nets override.
    std::vector<int> net_order(pd_.nets.size());
    std::iota(net_order.begin(), net_order.end(), 0);

    if (config_.enable_timing_driven && !net_timing_.empty()) {
        std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
            double ca = get_criticality(a);
            double cb = get_criticality(b);
            if (std::abs(ca - cb) > 0.01) return ca > cb;
            return compute_hpwl(a) > compute_hpwl(b);
        });
    } else {
        std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
            return compute_hpwl(a) > compute_hpwl(b);
        });
    }

    int routed = 0, failed = 0;
    std::vector<std::vector<WireSegment>> net_wires(pd_.nets.size());
    std::vector<std::vector<Via>> net_vias(pd_.nets.size());

    // Phase 1a: Parallel pattern routing pass
    // Pattern routes (L/Z shapes) are fast and can be tried for all 2-pin
    // nets simultaneously.  Each net gets private wire/via vectors; successful
    // results are merged under lock.  Nets that fail pattern routing are
    // collected for sequential A* in Phase 1b.
    int effective_threads = std::max(1, num_threads);
    ThreadPool pool(effective_threads);
    int N = (int)net_order.size();

    // Per-net pattern routing results (thread-local storage per net)
    std::vector<std::vector<WireSegment>> pat_wires(pd_.nets.size());
    std::vector<std::vector<Via>> pat_vias(pd_.nets.size());
    std::vector<std::atomic<int>> pat_ok(pd_.nets.size());
    for (auto& a : pat_ok) a.store(0, std::memory_order_relaxed);

    // Extract 2-pin net info for pattern routing (read-only access to pd_)
    struct TwoPinInfo {
        int net_idx;
        Point src, dst;
    };
    std::vector<TwoPinInfo> two_pin_nets;
    std::vector<int> multi_pin_nets;
    two_pin_nets.reserve(N);

    for (int ni : net_order) {
        auto& net = pd_.nets[ni];
        if (net.cell_ids.size() == 2) {
            auto& c0 = pd_.cells[net.cell_ids[0]];
            auto& c1 = pd_.cells[net.cell_ids[1]];
            double px0 = c0.position.x + (net.pin_offsets.size() > 0 ? net.pin_offsets[0].x : c0.width / 2);
            double py0 = c0.position.y + (net.pin_offsets.size() > 0 ? net.pin_offsets[0].y : c0.height / 2);
            double px1 = c1.position.x + (net.pin_offsets.size() > 1 ? net.pin_offsets[1].x : c1.width / 2);
            double py1 = c1.position.y + (net.pin_offsets.size() > 1 ? net.pin_offsets[1].y : c1.height / 2);
            two_pin_nets.push_back({ni, {px0, py0}, {px1, py1}});
        } else {
            multi_pin_nets.push_back(ni);
        }
    }

    // Try pattern routing in parallel for all 2-pin nets
    // Each thread writes to its own net's wire/via vectors (no contention),
    // but is_free() reads from shared occupancy (safe for read-only snapshot
    // since occupancy is empty at this point -- no writes yet).
    // After parallel pass, successful routes commit occupancy sequentially.
    if (!two_pin_nets.empty()) {
        pool.parallel_for(0, (int)two_pin_nets.size(), [&](int i) {
            auto& tp = two_pin_nets[i];
            int ni = tp.net_idx;
            // Pattern route into thread-local vectors (no occupancy writes)
            std::vector<WireSegment> local_wires;
            std::vector<Via> local_vias;

            // Inline pattern route check without marking occupancy
            if (tp.src.dist(tp.dst) < 0.001) {
                pat_ok[ni].store(1, std::memory_order_relaxed);
                return;
            }
            if (num_layers_ < 2 || layer_tracks_.empty()) return;

            int num_pairs = std::max(1, num_layers_ / 2);
            int pair_idx = ni % num_pairs;
            int h_layer = std::clamp(pair_idx * 2, 0, num_layers_ - 1);
            int v_layer = std::clamp(pair_idx * 2 + 1, 0, num_layers_ - 1);
            if (h_layer == v_layer && num_layers_ > 1)
                v_layer = (h_layer + 1) % num_layers_;

            if (h_layer >= (int)layer_tracks_.size() || v_layer >= (int)layer_tracks_.size()) return;
            if (layer_tracks_[h_layer].empty() || layer_tracks_[v_layer].empty()) return;

            double h_width = (h_layer < (int)pd_.layers.size()) ? pd_.layers[h_layer].width : 0.14;
            double v_width = (v_layer < (int)pd_.layers.size()) ? pd_.layers[v_layer].width : 0.14;

            Point src = tp.src, dst = tp.dst;
            bool found = false;

            // L-shape horizontal-first
            if (!found) {
                int h_tk = nearest_track(h_layer, src.y);
                int v_tk = nearest_track(v_layer, dst.x);
                double h_y = layer_tracks_[h_layer][h_tk].coord;
                double v_x = layer_tracks_[v_layer][v_tk].coord;
                double h_lo = std::min(src.x, v_x), h_hi = std::max(src.x, v_x);
                double v_lo = std::min(h_y, dst.y), v_hi = std::max(h_y, dst.y);

                if (is_free(h_layer, h_tk, h_lo, h_hi, ni) &&
                    is_free(v_layer, v_tk, v_lo, v_hi, ni)) {
                    Point on_h(src.x, h_y), bend(v_x, h_y), on_v(v_x, dst.y);
                    if (std::abs(src.y - h_y) > 0.001)
                        local_wires.push_back({v_layer, src, on_h, v_width});
                    if (std::abs(src.x - v_x) > 0.001)
                        local_wires.push_back({h_layer, on_h, bend, h_width});
                    if (h_layer != v_layer)
                        local_vias.push_back({bend, std::min(h_layer, v_layer), std::max(h_layer, v_layer)});
                    if (std::abs(h_y - dst.y) > 0.001)
                        local_wires.push_back({v_layer, bend, on_v, v_width});
                    if (std::abs(v_x - dst.x) > 0.001)
                        local_wires.push_back({h_layer, on_v, dst, h_width});
                    found = true;
                }
            }

            // L-shape vertical-first
            if (!found) {
                int v_tk = nearest_track(v_layer, src.x);
                int h_tk = nearest_track(h_layer, dst.y);
                double v_x = layer_tracks_[v_layer][v_tk].coord;
                double h_y = layer_tracks_[h_layer][h_tk].coord;
                double v_lo = std::min(src.y, h_y), v_hi = std::max(src.y, h_y);
                double h_lo = std::min(v_x, dst.x), h_hi = std::max(v_x, dst.x);

                if (is_free(v_layer, v_tk, v_lo, v_hi, ni) &&
                    is_free(h_layer, h_tk, h_lo, h_hi, ni)) {
                    Point on_v(v_x, src.y), bend(v_x, h_y), on_h(dst.x, h_y);
                    if (std::abs(src.x - v_x) > 0.001)
                        local_wires.push_back({h_layer, src, on_v, h_width});
                    if (std::abs(src.y - h_y) > 0.001)
                        local_wires.push_back({v_layer, on_v, bend, v_width});
                    if (h_layer != v_layer)
                        local_vias.push_back({bend, std::min(h_layer, v_layer), std::max(h_layer, v_layer)});
                    if (std::abs(v_x - dst.x) > 0.001)
                        local_wires.push_back({h_layer, bend, on_h, h_width});
                    if (std::abs(h_y - dst.y) > 0.001)
                        local_wires.push_back({v_layer, on_h, dst, v_width});
                    found = true;
                }
            }

            if (found) {
                pat_wires[ni] = std::move(local_wires);
                pat_vias[ni] = std::move(local_vias);
                pat_ok[ni].store(1, std::memory_order_relaxed);
            }
        });
    }

    // Commit successful pattern routes sequentially (occupancy updates)
    for (auto& tp : two_pin_nets) {
        int ni = tp.net_idx;
        if (!pat_ok[ni].load(std::memory_order_relaxed)) continue;

        // Commit occupancy for this net's pattern route
        for (auto& w : pat_wires[ni]) {
            int tk = nearest_track(w.layer, w.start.y);
            double lo = std::min(w.start.x, w.end.x);
            double hi = std::max(w.start.x, w.end.x);
            // For vertical wires, use y-coordinates for occupancy range
            if (w.layer < (int)pd_.layers.size() && !pd_.layers[w.layer].horizontal) {
                tk = nearest_track(w.layer, w.start.x);
                lo = std::min(w.start.y, w.end.y);
                hi = std::max(w.start.y, w.end.y);
            }
            mark_occupied(w.layer, tk, lo, hi, ni);
        }
        net_wires[ni] = std::move(pat_wires[ni]);
        net_vias[ni] = std::move(pat_vias[ni]);
        routed++;
    }

    // Phase 1b: Sequential A* routing for pattern-failed 2-pin nets
    // and all multi-pin nets (these need Steiner/MST decomposition with
    // occupancy coordination that cannot be parallelized safely).
    for (auto& tp : two_pin_nets) {
        int ni = tp.net_idx;
        if (pat_ok[ni].load(std::memory_order_relaxed)) continue;
        if (route_net(pd_.nets[ni], ni, net_wires[ni], net_vias[ni]))
            routed++;
        else
            failed++;
    }
    for (int ni : multi_pin_nets) {
        if (route_net(pd_.nets[ni], ni, net_wires[ni], net_vias[ni]))
            routed++;
        else
            failed++;
    }

    // Phase 2: Rip-up/reroute failed nets
    int max_rr = config_.detailed_reroute_iterations;
    for (int rr = 0; rr < max_rr && failed > 0; rr++) {
        result.reroute_iterations++;
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

    // Industrial: via minimization pass
    // For each net with >2 vias, try rerouting on a single layer pair
    if (config_.enable_via_minimization) {
        for (int ni : net_order) {
            int vc = count_net_vias(net_vias[ni]);
            if (vc <= 2) continue;
            // Save current routing
            auto saved_wires = net_wires[ni];
            auto saved_vias = net_vias[ni];
            // Try rerouting
            unmark_net(ni);
            net_wires[ni].clear();
            net_vias[ni].clear();
            if (route_net(pd_.nets[ni], ni, net_wires[ni], net_vias[ni])) {
                int new_vc = count_net_vias(net_vias[ni]);
                if (new_vc >= vc) {
                    // No improvement — restore
                    unmark_net(ni);
                    net_wires[ni] = saved_wires;
                    net_vias[ni] = saved_vias;
                    for (auto& seg : saved_wires) {
                        int tk = nearest_track(seg.layer, seg.start.y);
                        mark_occupied(seg.layer, tk,
                                      std::min(seg.start.x, seg.end.x),
                                      std::max(seg.start.x, seg.end.x), ni);
                    }
                }
            } else {
                // Failed — restore
                net_wires[ni] = saved_wires;
                net_vias[ni] = saved_vias;
            }
        }
    }

    // Merge into PhysicalDesign
    for (int ni = 0; ni < (int)pd_.nets.size(); ni++) {
        for (auto& w : net_wires[ni]) w.net_id = ni;
        pd_.wires.insert(pd_.wires.end(), net_wires[ni].begin(), net_wires[ni].end());
        pd_.vias.insert(pd_.vias.end(), net_vias[ni].begin(), net_vias[ni].end());
    }

    // ── Compute industrial metrics ───────────────────────────────────
    result.routed_nets = routed;
    result.failed_nets = failed;
    result.total_wires = (int)pd_.wires.size();
    result.total_vias = (int)pd_.vias.size();

    result.total_wirelength = 0;
    for (auto& w : pd_.wires)
        result.total_wirelength += w.start.dist(w.end);

    // Per-layer distribution
    result.wires_per_layer.resize(num_layers_, 0);
    result.vias_per_layer.resize(std::max(1, num_layers_ - 1), 0);
    for (auto& w : pd_.wires)
        if (w.layer >= 0 && w.layer < num_layers_) result.wires_per_layer[w.layer]++;
    for (auto& v : pd_.vias) {
        int low = std::min(v.lower_layer, v.upper_layer);
        if (low >= 0 && low < (int)result.vias_per_layer.size())
            result.vias_per_layer[low]++;
    }

    // Critical net wirelength
    if (config_.enable_timing_driven) {
        for (int ni : net_order) {
            if (get_criticality(ni) > 0.5) {
                for (auto& w : net_wires[ni])
                    result.critical_net_wirelength += w.start.dist(w.end);
            }
        }
    }

    // Antenna violations
    if (config_.enable_antenna_check) {
        for (int ni : net_order) {
            auto av = check_antenna(ni, net_wires[ni]);
            if (av.ratio > config_.max_antenna_ratio) {
                result.antenna_violations++;
                result.antenna_details.push_back(av);
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "DetailedRouterV2: Routing complete. "
              << result.total_wires << " wires, "
              << result.total_vias << " vias across "
              << num_layers_ << " layers.\n";
    std::cout << "  Layer distribution: ";
    for (int i = 0; i < num_layers_; i++) {
        std::cout << "M" << (i+1) << "=" << result.wires_per_layer[i];
        if (i + 1 < num_layers_) std::cout << ", ";
    }
    std::cout << "\n";

    result.message = std::to_string(result.routed_nets) + "/" +
                     std::to_string(result.routed_nets + result.failed_nets) +
                     " nets, " + std::to_string(result.total_vias) + " vias, " +
                     std::to_string(result.reroute_iterations) + " reroute passes";
    if (config_.enable_timing_driven) result.message += ", timing-driven";
    if (config_.enable_via_minimization) result.message += ", via-min";
    if (result.antenna_violations > 0)
        result.message += ", " + std::to_string(result.antenna_violations) + " antenna";

    return result;
}

// ============================================================================
// TrackResource: check if an interval is available
// ============================================================================
bool DetailedRouterV2::TrackResource::is_available(double start, double end, double spacing) const {
    double lo = std::min(start, end);
    double hi = std::max(start, end);
    for (auto& [occ_lo, occ_hi] : occupied) {
        if (occ_lo - spacing < hi && occ_hi + spacing > lo)
            return false;
    }
    return true;
}

// ============================================================================
// Initialize track resources for multi-layer assignment
// ============================================================================
void DetailedRouterV2::init_track_resources(int num_layers) {
    track_resources_.clear();
    track_resources_.resize(num_layers);

    double die_x0 = pd_.die_area.x0, die_y0 = pd_.die_area.y0;
    double die_x1 = pd_.die_area.x1, die_y1 = pd_.die_area.y1;

    for (int l = 0; l < num_layers; l++) {
        double pitch = 0.5;
        double spacing = 0.14;

        // Use layer_rules_ if available for this layer
        for (auto& lr : layer_rules_) {
            if (lr.layer == l) {
                pitch = lr.pitch > 0 ? lr.pitch : pitch;
                spacing = lr.min_spacing > 0 ? lr.min_spacing : spacing;
                break;
            }
        }
        // Fall back to pd_.layers if available
        if (l < (int)pd_.layers.size()) {
            if (pitch <= 0) pitch = pd_.layers[l].pitch > 0 ? pd_.layers[l].pitch : 0.5;
        }

        bool horizontal = (l % 2 == 0);
        if (l < (int)pd_.layers.size()) horizontal = pd_.layers[l].horizontal;
        for (auto& lr : layer_rules_) {
            if (lr.layer == l) { horizontal = lr.prefer_horizontal; break; }
        }

        int track_idx = 0;
        if (horizontal) {
            for (double y = die_y0 + pitch; y < die_y1; y += pitch) {
                track_resources_[l].push_back({l, track_idx++, {}});
            }
        } else {
            for (double x = die_x0 + pitch; x < die_x1; x += pitch) {
                track_resources_[l].push_back({l, track_idx++, {}});
            }
        }
        // Ensure at least one track
        if (track_resources_[l].empty()) {
            track_resources_[l].push_back({l, 0, {}});
        }
    }
}

// ============================================================================
// Find the best available track on a layer for a given interval
// ============================================================================
int DetailedRouterV2::find_best_track(int net_idx, int layer, double start, double end) {
    if (layer < 0 || layer >= (int)track_resources_.size()) return -1;
    auto& tracks = track_resources_[layer];
    if (tracks.empty()) return -1;

    double spacing = drc_constraints_.min_spacing;
    if (layer < (int)pd_.layers.size())
        spacing = std::max(spacing, pd_.layers[layer].spacing);

    // Prefer the track closest to the midpoint of the interval
    double mid = (start + end) / 2.0;
    int best = -1;
    double best_dist = 1e18;

    for (int i = 0; i < (int)tracks.size(); i++) {
        if (!tracks[i].is_available(start, end, spacing)) continue;

        // Use track coordinate from layer_tracks_ if available, else use index*pitch
        double coord = 0;
        if (layer < (int)layer_tracks_.size() && i < (int)layer_tracks_[layer].size())
            coord = layer_tracks_[layer][i].coord;
        else
            coord = static_cast<double>(i);

        double d = std::abs(coord - mid);
        if (d < best_dist) { best_dist = d; best = i; }
    }
    return best;
}

// ============================================================================
// A) Multi-layer track assignment
// ============================================================================
std::vector<DetailedRouterV2::TrackAssignResult>
DetailedRouterV2::assign_tracks_multilayer(int num_layers) {
    std::vector<TrackAssignResult> results;

    if (num_layers <= 0) num_layers = num_layers_ > 0 ? num_layers_ : 4;

    // Ensure layers/tracks are set up
    if (pd_.layers.empty()) {
        int saved = num_layers_;
        num_layers_ = num_layers;
        setup_layers();
        build_track_grid();
        num_layers_ = saved;
    }
    init_track_resources(num_layers);

    // Sort nets by priority: critical first, then short HPWL first
    std::vector<int> net_order(pd_.nets.size());
    std::iota(net_order.begin(), net_order.end(), 0);
    std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
        double ca = get_criticality(a), cb = get_criticality(b);
        if (std::abs(ca - cb) > 0.01) return ca > cb;
        auto hpwl = [&](int ni) -> double {
            auto& net = pd_.nets[ni];
            if (net.cell_ids.size() < 2) return 0.0;
            double xlo = 1e18, xhi = -1e18, ylo = 1e18, yhi = -1e18;
            for (auto ci : net.cell_ids) {
                xlo = std::min(xlo, pd_.cells[ci].position.x);
                xhi = std::max(xhi, pd_.cells[ci].position.x);
                ylo = std::min(ylo, pd_.cells[ci].position.y);
                yhi = std::max(yhi, pd_.cells[ci].position.y);
            }
            return (xhi - xlo) + (yhi - ylo);
        };
        return hpwl(a) < hpwl(b);
    });

    for (int ni : net_order) {
        auto& net = pd_.nets[ni];
        if (net.cell_ids.size() < 2) continue;

        // Compute bounding box
        double xlo = 1e18, xhi = -1e18, ylo = 1e18, yhi = -1e18;
        for (size_t i = 0; i < net.cell_ids.size(); i++) {
            auto& c = pd_.cells[net.cell_ids[i]];
            double px = c.position.x + (i < net.pin_offsets.size() ? net.pin_offsets[i].x : c.width / 2);
            double py = c.position.y + (i < net.pin_offsets.size() ? net.pin_offsets[i].y : c.height / 2);
            xlo = std::min(xlo, px); xhi = std::max(xhi, px);
            ylo = std::min(ylo, py); yhi = std::max(yhi, py);
        }

        double dx = xhi - xlo, dy = yhi - ylo;
        bool prefer_h = (dx >= dy);

        // Find the best layer with the matching preferred direction
        int best_layer = -1;
        int best_track = -1;
        int via_count = 0;

        for (int l = 0; l < num_layers; l++) {
            bool layer_h = (l % 2 == 0);
            if (l < (int)pd_.layers.size()) layer_h = pd_.layers[l].horizontal;
            for (auto& lr : layer_rules_) {
                if (lr.layer == l) { layer_h = lr.prefer_horizontal; break; }
            }

            if (layer_h != prefer_h) continue;

            double seg_start = prefer_h ? xlo : ylo;
            double seg_end = prefer_h ? xhi : yhi;
            int tk = find_best_track(ni, l, seg_start, seg_end);
            if (tk >= 0) {
                best_layer = l;
                best_track = tk;
                break;
            }
        }

        // If no preferred-direction layer found, try any layer
        if (best_layer < 0) {
            for (int l = 0; l < num_layers; l++) {
                double seg_start = prefer_h ? xlo : ylo;
                double seg_end = prefer_h ? xhi : yhi;
                int tk = find_best_track(ni, l, seg_start, seg_end);
                if (tk >= 0) {
                    best_layer = l;
                    best_track = tk;
                    via_count = 1;  // layer transition needed
                    break;
                }
            }
        }

        if (best_layer < 0 || best_track < 0) continue;

        // Mark track interval as occupied
        double seg_start = prefer_h ? xlo : ylo;
        double seg_end = prefer_h ? xhi : yhi;
        if (best_layer < (int)track_resources_.size() &&
            best_track < (int)track_resources_[best_layer].size()) {
            track_resources_[best_layer][best_track].occupied.emplace_back(seg_start, seg_end);
        }

        // Insert vias at layer transitions if net spans multiple pins at different layers
        if (best_layer > 0) via_count = std::max(via_count, 1);

        TrackAssignResult tar;
        tar.net_idx = ni;
        tar.layer = best_layer;
        tar.track_idx = best_track;
        tar.start_x = seg_start;
        tar.end_x = seg_end;
        tar.via_count = via_count;
        results.push_back(tar);
    }

    return results;
}

// ============================================================================
// B) Via pillar optimization
// ============================================================================
int DetailedRouterV2::optimize_via_pillars(std::vector<ViaPillar>& vias) {
    if (vias.empty()) return 0;

    int removed = 0;
    const double pos_tol = 0.01;

    // Step 1: Identify via stacks at same (x,y) location
    // Group vias by approximate position
    struct PosKey {
        int ix, iy;
        bool operator==(const PosKey& o) const { return ix == o.ix && iy == o.iy; }
    };
    struct PosHash {
        size_t operator()(const PosKey& k) const {
            return std::hash<int>()(k.ix) ^ (std::hash<int>()(k.iy) << 16);
        }
    };

    std::unordered_map<PosKey, std::vector<size_t>, PosHash> pos_groups;
    for (size_t i = 0; i < vias.size(); i++) {
        int ix = static_cast<int>(std::round(vias[i].x / pos_tol));
        int iy = static_cast<int>(std::round(vias[i].y / pos_tol));
        pos_groups[{ix, iy}].push_back(i);
    }

    // Step 2: For single-via stacks at same location, try to consolidate
    // If multiple vias at same position cover consecutive layers, merge into a stacked via
    std::vector<bool> merged(vias.size(), false);

    for (auto& [pos, indices] : pos_groups) {
        if (indices.size() < 2) continue;

        // Sort by bottom_layer
        std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
            return vias[a].bottom_layer < vias[b].bottom_layer;
        });

        // Try to merge consecutive vias into a stacked pillar
        size_t i = 0;
        while (i < indices.size()) {
            size_t j = i + 1;
            int top = vias[indices[i]].top_layer;

            // Extend the stack as far as possible
            while (j < indices.size()) {
                int next_bot = vias[indices[j]].bottom_layer;
                if (next_bot <= top + 1) {
                    top = std::max(top, vias[indices[j]].top_layer);
                    j++;
                } else {
                    break;
                }
            }

            if (j - i >= 2) {
                // Merge: keep the first via, expand its range, mark rest as merged
                vias[indices[i]].bottom_layer = vias[indices[i]].bottom_layer;
                vias[indices[i]].top_layer = top;
                vias[indices[i]].is_stacked = true;

                for (size_t k = i + 1; k < j; k++) {
                    merged[indices[k]] = true;
                    removed++;
                }
            }
            i = j;
        }
    }

    // Step 3: Remove merged vias
    std::vector<ViaPillar> compacted;
    compacted.reserve(vias.size() - removed);
    for (size_t i = 0; i < vias.size(); i++) {
        if (!merged[i]) compacted.push_back(vias[i]);
    }
    vias = std::move(compacted);

    return removed;
}

// ============================================================================
// F) DRC check for a single wire segment
// ============================================================================
bool DetailedRouterV2::drc_check_segment(double x1, double y1, double x2, double y2,
                                          int layer, int net_idx) const {
    double dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
    double length = dx + dy;

    // Check min_width
    double width = drc_constraints_.min_width;
    if (layer < (int)pd_.layers.size())
        width = std::max(width, pd_.layers[layer].width);
    for (auto& lr : layer_rules_) {
        if (lr.layer == layer) { width = std::max(width, lr.min_width); break; }
    }
    // A zero-length segment is degenerate but not a width violation
    if (length > 0.001 && width < drc_constraints_.min_width)
        return false;

    // Check spacing to all other segments on the same layer
    double min_sp = drc_constraints_.min_spacing;
    if (layer < (int)pd_.layers.size())
        min_sp = std::max(min_sp, pd_.layers[layer].spacing);
    for (auto& lr : layer_rules_) {
        if (lr.layer == layer) { min_sp = std::max(min_sp, lr.min_spacing); break; }
    }

    double seg_xlo = std::min(x1, x2) - width / 2.0;
    double seg_xhi = std::max(x1, x2) + width / 2.0;
    double seg_ylo = std::min(y1, y2) - width / 2.0;
    double seg_yhi = std::max(y1, y2) + width / 2.0;

    // Search window: only check nearby wires
    double search_margin = min_sp + width;
    for (auto& w : pd_.wires) {
        if (w.layer != layer) continue;
        if (w.net_id == net_idx) continue;

        double w_xlo = std::min(w.start.x, w.end.x) - w.width / 2.0;
        double w_xhi = std::max(w.start.x, w.end.x) + w.width / 2.0;
        double w_ylo = std::min(w.start.y, w.end.y) - w.width / 2.0;
        double w_yhi = std::max(w.start.y, w.end.y) + w.width / 2.0;

        // Quick bounding-box reject
        if (w_xhi + search_margin < seg_xlo || w_xlo - search_margin > seg_xhi) continue;
        if (w_yhi + search_margin < seg_ylo || w_ylo - search_margin > seg_yhi) continue;

        // Compute edge-to-edge distance between rectangles
        double gap_x = 0, gap_y = 0;
        if (seg_xhi < w_xlo) gap_x = w_xlo - seg_xhi;
        else if (w_xhi < seg_xlo) gap_x = seg_xlo - w_xhi;
        if (seg_yhi < w_ylo) gap_y = w_ylo - seg_yhi;
        else if (w_yhi < seg_ylo) gap_y = seg_ylo - w_yhi;

        double edge_dist = std::max(gap_x, gap_y);
        if (gap_x > 0 && gap_y > 0)
            edge_dist = std::sqrt(gap_x * gap_x + gap_y * gap_y);

        if (edge_dist < min_sp) return false;
    }

    // Check end-of-line spacing at segment endpoints
    double eol_sp = drc_constraints_.end_of_line_spacing;
    if (eol_sp > min_sp && length > 0.001) {
        // Check endpoints against nearby wires
        Point ends[2] = {{x1, y1}, {x2, y2}};
        for (auto& ep : ends) {
            for (auto& w : pd_.wires) {
                if (w.layer != layer || w.net_id == net_idx) continue;
                double d = std::abs(ep.x - w.start.x) + std::abs(ep.y - w.start.y);
                double d2 = std::abs(ep.x - w.end.x) + std::abs(ep.y - w.end.y);
                if (std::min(d, d2) < eol_sp) return false;
            }
        }
    }

    // Check via enclosure at via locations
    double via_enc = drc_constraints_.via_enclosure;
    for (auto& lr : layer_rules_) {
        if (lr.layer == layer) { via_enc = std::max(via_enc, lr.min_enclosure); break; }
    }
    for (auto& v : pd_.vias) {
        if (v.lower_layer != layer && v.upper_layer != layer) continue;
        double vx = v.position.x, vy = v.position.y;
        // Check if via is on this segment (approximately)
        bool on_seg = (vx >= std::min(x1, x2) - 0.01 && vx <= std::max(x1, x2) + 0.01 &&
                       vy >= std::min(y1, y2) - 0.01 && vy <= std::max(y1, y2) + 0.01);
        if (!on_seg) continue;
        // Via must be enclosed by at least via_enc on each side
        double ext_x = std::min(std::abs(vx - std::min(x1, x2)), std::abs(vx - std::max(x1, x2)));
        double ext_y = std::min(std::abs(vy - std::min(y1, y2)), std::abs(vy - std::max(y1, y2)));
        if (ext_x < via_enc && ext_y < via_enc && length > via_enc * 2) {
            // Enclosure violated: via is too close to segment edge on both axes
            return false;
        }
    }

    return true;
}

// ============================================================================
// Maze cost calculation for DRC-aware routing
// ============================================================================
double DetailedRouterV2::maze_cost(const MazeNode& from, const MazeNode& to) const {
    double base_cost = std::abs(to.x - from.x) + std::abs(to.y - from.y);

    // Via penalty for layer change
    double via_penalty = 0;
    if (from.layer != to.layer) {
        via_penalty = config_.via_cost > 0 ? config_.via_cost : 1.5;
        via_penalty *= std::abs(to.layer - from.layer);
    }

    // Congestion penalty: check how many occupied segments are near the target
    double congestion = 0;
    double search_r = drc_constraints_.min_spacing * 3.0;
    for (auto& seg : occupancy_) {
        if (seg.layer != to.layer) continue;
        // Manhattan distance from target to occupied segment center
        double seg_mid = (seg.lo + seg.hi) / 2.0;
        double d = std::abs(static_cast<double>(to.x) - seg_mid) +
                   std::abs(static_cast<double>(to.y) - seg_mid);
        if (d < search_r) {
            congestion += (search_r - d) / search_r;
        }
    }

    // DRC proximity penalty: wires near other wires cost more
    double proximity_penalty = congestion * congestion_penalty_ * 0.5;

    return base_cost + via_penalty + proximity_penalty;
}

// ============================================================================
// C) DRC-aware maze routing for a single net
// ============================================================================
bool DetailedRouterV2::route_net_drc_aware(int net_idx) {
    if (net_idx < 0 || net_idx >= (int)pd_.nets.size()) return false;
    auto& net = pd_.nets[net_idx];
    if (net.cell_ids.size() < 2) return true;

    // Gather pin locations
    std::vector<Point> pins;
    for (size_t i = 0; i < net.cell_ids.size(); i++) {
        auto& c = pd_.cells[net.cell_ids[i]];
        double px = c.position.x + (i < net.pin_offsets.size() ? net.pin_offsets[i].x : c.width / 2);
        double py = c.position.y + (i < net.pin_offsets.size() ? net.pin_offsets[i].y : c.height / 2);
        pins.push_back({px, py});
    }

    // Grid resolution based on minimum pitch
    double grid_res = 0.5;
    if (!pd_.layers.empty()) {
        grid_res = pd_.layers[0].pitch > 0 ? pd_.layers[0].pitch : 0.5;
    }

    int nl = num_layers_ > 0 ? num_layers_ : (int)pd_.layers.size();
    if (nl <= 0) nl = 2;

    // For each two-pin connection (MST decomposition)
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
            Point src = pins[parent[u]], dst = pins[u];
            if (src.dist(dst) < 0.001) continue;

            // 3D A* maze routing with DRC checking
            // Convert to grid coordinates
            int sx = static_cast<int>(std::round(src.x / grid_res));
            int sy = static_cast<int>(std::round(src.y / grid_res));
            int dx = static_cast<int>(std::round(dst.x / grid_res));
            int dy = static_cast<int>(std::round(dst.y / grid_res));
            int sl = 0;  // start on layer 0

            // A* priority queue
            std::priority_queue<MazeNode, std::vector<MazeNode>, std::greater<MazeNode>> pq;
            // Encode node as (layer * grid_size^2 + y * grid_size + x)
            int grid_range = std::max({std::abs(dx - sx), std::abs(dy - sy), 1}) + 10;
            int ox = std::min(sx, dx) - 5;
            int oy = std::min(sy, dy) - 5;
            int gw = grid_range + 10;
            int gh = grid_range + 10;
            int total_nodes = nl * gw * gh;

            auto encode = [&](int x, int y, int l) -> int {
                return l * gw * gh + (y - oy) * gw + (x - ox);
            };

            std::vector<double> dist(total_nodes, 1e18);
            std::vector<int> prev(total_nodes, -1);
            std::vector<bool> visited(total_nodes, false);

            // Bounds check
            auto in_bounds = [&](int x, int y, int l) -> bool {
                return x >= ox && x < ox + gw && y >= oy && y < oy + gh && l >= 0 && l < nl;
            };

            if (!in_bounds(sx, sy, sl) || !in_bounds(dx, dy, 0)) {
                all_ok = false;
                continue;
            }

            int start_id = encode(sx, sy, sl);
            dist[start_id] = 0;
            pq.push({sx, sy, sl, 0, -1});

            bool found = false;
            int goal_id = -1;
            int max_expansions = std::min(total_nodes, 50000);
            int expansions = 0;

            while (!pq.empty() && expansions < max_expansions) {
                auto cur = pq.top(); pq.pop();
                int cid = encode(cur.x, cur.y, cur.layer);
                if (cid < 0 || cid >= total_nodes) continue;
                if (visited[cid]) continue;
                visited[cid] = true;
                expansions++;

                // Check if we reached any target layer at destination
                if (cur.x == dx && cur.y == dy) {
                    found = true;
                    goal_id = cid;
                    break;
                }

                // Expand neighbors: 4 directions + layer up/down
                int moves[][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
                for (auto& m : moves) {
                    int nx = cur.x + m[0], ny = cur.y + m[1], nlay = cur.layer + m[2];
                    if (!in_bounds(nx, ny, nlay)) continue;
                    int nid = encode(nx, ny, nlay);
                    if (nid < 0 || nid >= total_nodes || visited[nid]) continue;

                    // DRC check: verify the segment doesn't violate spacing
                    double rx1 = cur.x * grid_res, ry1 = cur.y * grid_res;
                    double rx2 = nx * grid_res, ry2 = ny * grid_res;

                    if (m[2] == 0) {  // same-layer move
                        if (!drc_check_segment(rx1, ry1, rx2, ry2, nlay, net_idx))
                            continue;
                    }

                    MazeNode next_node = {nx, ny, nlay, 0, cid};
                    MazeNode cur_node = {cur.x, cur.y, cur.layer, 0, 0};
                    double step = maze_cost(cur_node, next_node);
                    double new_dist = dist[cid] + step;

                    if (new_dist < dist[nid]) {
                        dist[nid] = new_dist;
                        prev[nid] = cid;
                        // A* heuristic: Manhattan distance to goal
                        double h = (std::abs(nx - dx) + std::abs(ny - dy)) * grid_res;
                        pq.push({nx, ny, nlay, new_dist + h, cid});
                    }
                }
            }

            if (found && goal_id >= 0) {
                // Backtrace path and create wire segments
                std::vector<int> path;
                int cur_id = goal_id;
                while (cur_id >= 0 && cur_id < total_nodes) {
                    path.push_back(cur_id);
                    cur_id = prev[cur_id];
                }
                std::reverse(path.begin(), path.end());

                // Convert path nodes to wire segments
                for (size_t p = 1; p < path.size(); p++) {
                    int pid = path[p - 1], cid2 = path[p];
                    int pl = pid / (gw * gh), cl = cid2 / (gw * gh);
                    int py2 = (pid % (gw * gh)) / gw + oy;
                    int px2 = (pid % (gw * gh)) % gw + ox;
                    int cy2 = (cid2 % (gw * gh)) / gw + oy;
                    int cx2 = (cid2 % (gw * gh)) % gw + ox;

                    Point p1(px2 * grid_res, py2 * grid_res);
                    Point p2(cx2 * grid_res, cy2 * grid_res);

                    if (pl != cl) {
                        // Layer change → via
                        pd_.vias.push_back({p1, std::min(pl, cl), std::max(pl, cl)});
                    } else if (p1.dist(p2) > 0.001) {
                        double w = (cl < (int)pd_.layers.size()) ? pd_.layers[cl].width : 0.14;
                        WireSegment ws = {cl, p1, p2, w, net_idx};
                        pd_.wires.push_back(ws);
                        // Mark occupancy
                        int tk = nearest_track(cl, p1.y);
                        mark_occupied(cl, tk,
                                      std::min(p1.x, p2.x), std::max(p1.x, p2.x), net_idx);
                    }
                }
            } else {
                // Fallback to regular routing
                std::vector<WireSegment> fw;
                std::vector<Via> fv;
                if (!route_two_pin(src, dst, net_idx, fw, fv))
                    all_ok = false;
                for (auto& w : fw) { w.net_id = net_idx; pd_.wires.push_back(w); }
                for (auto& v : fv) pd_.vias.push_back(v);
            }
        }

        for (int v = 0; v < n; v++) {
            if (in_tree[v]) continue;
            double d = std::abs(pins[u].x - pins[v].x) + std::abs(pins[u].y - pins[v].y);
            if (d < min_cost[v]) { min_cost[v] = d; parent[v] = u; }
        }
    }
    return all_ok;
}

// ============================================================================
// Identify nets with DRC violations (for convergence loop)
// ============================================================================
std::vector<int> DetailedRouterV2::identify_violating_nets() {
    std::vector<int> violating;

    // Check each net's wires for DRC violations
    // Group wires by net_id
    std::unordered_map<int, int> violation_count;

    for (size_t i = 0; i < pd_.wires.size(); i++) {
        auto& w = pd_.wires[i];
        if (w.net_id < 0) continue;

        if (!drc_check_segment(w.start.x, w.start.y, w.end.x, w.end.y, w.layer, w.net_id)) {
            violation_count[w.net_id]++;
        }
    }

    // Sort by violation count (worst first)
    std::vector<std::pair<int, int>> sorted_violations(violation_count.begin(), violation_count.end());
    std::sort(sorted_violations.begin(), sorted_violations.end(),
              [](auto& a, auto& b) { return a.second > b.second; });

    for (auto& [net_id, count] : sorted_violations) {
        violating.push_back(net_id);
    }

    return violating;
}

// ============================================================================
// Escalate congestion costs
// ============================================================================
void DetailedRouterV2::escalate_costs(double factor) {
    congestion_penalty_ *= factor;
}

// ============================================================================
// D) Convergence-based global rip-up reroute
// ============================================================================
RouteResult DetailedRouterV2::route_with_convergence() {
    auto t0 = std::chrono::high_resolution_clock::now();
    RouteResult result;

    if (num_layers_ <= 0) num_layers_ = compute_num_layers();

    std::cout << "DetailedRouterV2: Starting convergence routing with "
              << num_layers_ << " metal layers.\n";

    setup_layers();
    build_track_grid();
    occupancy_.clear();
    occ_.clear();
    congestion_penalty_ = 1.0;

    int total_nets = (int)pd_.nets.size();
    conv_net_wires_.assign(total_nets, {});
    conv_net_vias_.assign(total_nets, {});

    // Timing-driven net ordering
    std::vector<int> net_order(total_nets);
    std::iota(net_order.begin(), net_order.end(), 0);

    if (config_.enable_timing_driven && !net_timing_.empty()) {
        std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
            double ca = get_criticality(a), cb = get_criticality(b);
            if (std::abs(ca - cb) > 0.01) return ca > cb;
            auto hpwl = [&](int ni) -> double {
                auto& net = pd_.nets[ni];
                if (net.cell_ids.size() < 2) return 0.0;
                double xlo = 1e18, xhi = -1e18, ylo = 1e18, yhi = -1e18;
                for (auto ci : net.cell_ids) {
                    xlo = std::min(xlo, pd_.cells[ci].position.x);
                    xhi = std::max(xhi, pd_.cells[ci].position.x);
                    ylo = std::min(ylo, pd_.cells[ci].position.y);
                    yhi = std::max(yhi, pd_.cells[ci].position.y);
                }
                return (xhi - xlo) + (yhi - ylo);
            };
            return hpwl(a) < hpwl(b);
        });
    }

    // Phase 1: Initial routing of all nets
    int routed = 0, failed = 0;
    for (int ni : net_order) {
        if (route_net(pd_.nets[ni], ni, conv_net_wires_[ni], conv_net_vias_[ni]))
            routed++;
        else
            failed++;
    }

    // Phase 2: Iterative convergence loop
    int max_iter = conv_cfg_.max_iterations;
    int nets_per_iter = conv_cfg_.nets_per_iteration;

    for (int iteration = 0; iteration < max_iter; iteration++) {
        result.reroute_iterations++;

        auto violating = identify_violating_nets();
        if (violating.empty() && failed == 0) {
            std::cout << "  Convergence reached at iteration " << iteration << "\n";
            break;
        }

        // Rip up the worst nets
        int to_ripup = std::min(nets_per_iter, (int)violating.size());
        for (int r = 0; r < to_ripup; r++) {
            int ni = violating[r];
            unmark_net(ni);

            // Remove old wires/vias from pd_
            pd_.wires.erase(
                std::remove_if(pd_.wires.begin(), pd_.wires.end(),
                               [ni](const WireSegment& w) { return w.net_id == ni; }),
                pd_.wires.end());

            conv_net_wires_[ni].clear();
            conv_net_vias_[ni].clear();
        }

        // Escalate congestion costs
        escalate_costs(conv_cfg_.cost_escalation);

        // Reroute with DRC awareness
        int new_failed = 0;
        for (int r = 0; r < to_ripup; r++) {
            int ni = violating[r];
            if (route_net_drc_aware(ni)) {
                // Collect wires for this net
                for (auto& w : pd_.wires) {
                    if (w.net_id == ni) conv_net_wires_[ni].push_back(w);
                }
                for (auto& v : pd_.vias) {
                    // Attribute vias near this net's wires
                }
            } else {
                new_failed++;
            }
        }

        if (new_failed == 0 && violating.empty()) break;

        // Also reroute any previously failed nets
        for (int ni : net_order) {
            if (!conv_net_wires_[ni].empty()) continue;
            if (route_net(pd_.nets[ni], ni, conv_net_wires_[ni], conv_net_vias_[ni]))
                routed++;
            else
                failed++;
        }
    }

    // Phase 3: Multi-layer track assignment
    auto track_results = assign_tracks_multilayer(num_layers_);

    // Phase 4: Via pillar optimization
    std::vector<ViaPillar> via_pillars;
    for (auto& v : pd_.vias) {
        ViaPillar vp;
        vp.x = v.position.x;
        vp.y = v.position.y;
        vp.bottom_layer = v.lower_layer;
        vp.top_layer = v.upper_layer;
        vp.is_stacked = false;
        via_pillars.push_back(vp);
    }
    int vias_removed = optimize_via_pillars(via_pillars);
    std::cout << "  Via pillar optimization: removed " << vias_removed << " redundant vias\n";

    // Merge conv_net wires into PhysicalDesign (for nets not already added by DRC-aware routing)
    for (int ni = 0; ni < total_nets; ni++) {
        for (auto& w : conv_net_wires_[ni]) {
            w.net_id = ni;
            // Avoid duplicates: only add if not already in pd_.wires
            bool dup = false;
            for (auto& ew : pd_.wires) {
                if (ew.net_id == ni && ew.layer == w.layer &&
                    std::abs(ew.start.x - w.start.x) < 0.001 &&
                    std::abs(ew.start.y - w.start.y) < 0.001 &&
                    std::abs(ew.end.x - w.end.x) < 0.001 &&
                    std::abs(ew.end.y - w.end.y) < 0.001) {
                    dup = true; break;
                }
            }
            if (!dup) pd_.wires.push_back(w);
        }
        for (auto& v : conv_net_vias_[ni]) {
            pd_.vias.push_back(v);
        }
    }

    // ── Compute metrics ──────────────────────────────────────────────
    result.routed_nets = routed;
    result.failed_nets = failed;
    result.total_wires = (int)pd_.wires.size();
    result.total_vias = (int)pd_.vias.size();
    result.total_wirelength = 0;
    for (auto& w : pd_.wires) result.total_wirelength += w.start.dist(w.end);

    result.wires_per_layer.resize(num_layers_, 0);
    result.vias_per_layer.resize(std::max(1, num_layers_ - 1), 0);
    for (auto& w : pd_.wires)
        if (w.layer >= 0 && w.layer < num_layers_) result.wires_per_layer[w.layer]++;
    for (auto& v : pd_.vias) {
        int low = std::min(v.lower_layer, v.upper_layer);
        if (low >= 0 && low < (int)result.vias_per_layer.size())
            result.vias_per_layer[low]++;
    }

    if (config_.enable_timing_driven) {
        for (int ni : net_order) {
            if (get_criticality(ni) > 0.5) {
                for (auto& w : conv_net_wires_[ni])
                    result.critical_net_wirelength += w.start.dist(w.end);
            }
        }
    }

    if (config_.enable_antenna_check) {
        for (int ni : net_order) {
            auto av = check_antenna(ni, conv_net_wires_[ni]);
            if (av.ratio > config_.max_antenna_ratio) {
                result.antenna_violations++;
                result.antenna_details.push_back(av);
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "DetailedRouterV2: Convergence routing complete. "
              << result.total_wires << " wires, " << result.total_vias << " vias, "
              << result.reroute_iterations << " iterations.\n";

    result.message = std::to_string(result.routed_nets) + "/" +
                     std::to_string(result.routed_nets + result.failed_nets) +
                     " nets, " + std::to_string(result.total_vias) + " vias, " +
                     std::to_string(result.reroute_iterations) + " convergence iters";
    if (vias_removed > 0)
        result.message += ", " + std::to_string(vias_removed) + " vias optimized";

    return result;
}

// ============================================================================
// E) Double patterning conflict check
// ============================================================================
bool DetailedRouterV2::check_dp_conflict(int track1, int track2, int layer) const {
    if (!dp_cfg_.enable) return false;
    if (layer < 0 || layer >= (int)layer_tracks_.size()) return false;
    auto& tracks = layer_tracks_[layer];
    if (track1 < 0 || track1 >= (int)tracks.size()) return false;
    if (track2 < 0 || track2 >= (int)tracks.size()) return false;
    if (track1 == track2) return false;

    // Physical distance between tracks
    double dist = std::abs(tracks[track1].coord - tracks[track2].coord);

    // If tracks are within min_dp_spacing, they must be on different masks
    if (dist >= dp_cfg_.min_dp_spacing) return false;

    // Simple greedy coloring: assign mask based on track index parity
    // Track coloring: even index → mask 0, odd index → mask 1
    int mask1 = track1 % dp_cfg_.num_masks;
    int mask2 = track2 % dp_cfg_.num_masks;

    // Conflict if both need the same mask but are too close
    if (mask1 == mask2 && dist < dp_cfg_.min_dp_spacing)
        return true;

    // Check for occupied segments that overlap
    // Two tracks with overlapping occupied intervals on same mask → conflict
    for (auto& seg1 : occupancy_) {
        if (seg1.layer != layer || seg1.track_idx != track1) continue;
        for (auto& seg2 : occupancy_) {
            if (seg2.layer != layer || seg2.track_idx != track2) continue;
            // Check interval overlap along the track
            if (seg1.lo < seg2.hi && seg1.hi > seg2.lo) {
                // Overlapping intervals + same mask + too close = conflict
                if (mask1 == mask2) return true;
            }
        }
    }

    return false;
}

} // namespace sf
