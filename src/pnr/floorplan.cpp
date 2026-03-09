// SiliconForge — Floorplanner Implementation (Simulated Annealing)
#include "pnr/floorplan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

namespace sf {

void Floorplanner::add_macro(const std::string& name, double w, double h, bool fixed) {
    int id = (int)macros_.size();
    macros_.push_back({id, name, w, h, fixed, {0,0}, false});
}

void Floorplanner::add_connection(int a, int b, double weight) {
    connections_.push_back({a, b, weight});
}

void Floorplanner::random_swap(std::mt19937& rng) {
    if (macros_.size() < 2) return;
    std::uniform_int_distribution<int> dist(0, (int)macros_.size()-1);
    int a = dist(rng), b = dist(rng);
    while (b == a || macros_[a].fixed || macros_[b].fixed) {
        a = dist(rng); b = dist(rng);
    }
    std::swap(macros_[a].position, macros_[b].position);
}

void Floorplanner::random_rotate(std::mt19937& rng) {
    std::uniform_int_distribution<int> dist(0, (int)macros_.size()-1);
    int idx = dist(rng);
    int tries = 0;
    while (macros_[idx].fixed && tries++ < 20) idx = dist(rng);
    if (macros_[idx].fixed) return;
    macros_[idx].rotated = !macros_[idx].rotated;
    std::swap(macros_[idx].width, macros_[idx].height);
}

void Floorplanner::random_move(std::mt19937& rng, double range) {
    std::uniform_int_distribution<int> idx_dist(0, (int)macros_.size()-1);
    std::uniform_real_distribution<double> delta(-range, range);
    int idx = idx_dist(rng);
    int tries = 0;
    while (macros_[idx].fixed && tries++ < 20) idx = idx_dist(rng);
    if (macros_[idx].fixed) return;
    macros_[idx].position.x += delta(rng);
    macros_[idx].position.y += delta(rng);
    macros_[idx].position.x = std::max(0.0, macros_[idx].position.x);
    macros_[idx].position.y = std::max(0.0, macros_[idx].position.y);
}

void Floorplanner::pack() {
    // Sort by area (largest first) then pack left-bottom
    std::vector<int> order(macros_.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return macros_[a].width * macros_[a].height > macros_[b].width * macros_[b].height;
    });

    // Simple skyline packing
    double max_width = 0;
    for (auto& m : macros_) max_width += m.width;
    max_width = std::sqrt(max_width * max_width / macros_.size()) * 2; // estimate width

    std::vector<double> skyline((int)max_width + 1, 0);
    for (int idx : order) {
        auto& m = macros_[idx];
        if (m.fixed) continue;
        // Find lowest position where macro fits
        double best_y = 1e18;
        int best_x = 0;
        int mw = std::max(1, (int)m.width);
        for (int x = 0; x + mw <= (int)skyline.size(); ++x) {
            double max_y = 0;
            for (int dx = 0; dx < mw; ++dx)
                max_y = std::max(max_y, skyline[x + dx]);
            if (max_y < best_y) {
                best_y = max_y;
                best_x = x;
            }
        }
        m.position.x = best_x;
        m.position.y = best_y;
        for (int dx = 0; dx < mw && best_x + dx < (int)skyline.size(); ++dx)
            skyline[best_x + dx] = best_y + m.height;
    }
}

double Floorplanner::compute_wirelength() const {
    double wl = 0;
    for (auto& c : connections_) {
        auto& ma = macros_[c.a]; auto& mb = macros_[c.b];
        double cx_a = ma.position.x + ma.width/2;
        double cy_a = ma.position.y + ma.height/2;
        double cx_b = mb.position.x + mb.width/2;
        double cy_b = mb.position.y + mb.height/2;
        wl += c.weight * (std::abs(cx_a - cx_b) + std::abs(cy_a - cy_b));
    }
    return wl;
}

double Floorplanner::bounding_area() const {
    double max_x = 0, max_y = 0;
    for (auto& m : macros_) {
        max_x = std::max(max_x, m.position.x + m.width);
        max_y = std::max(max_y, m.position.y + m.height);
    }
    return max_x * max_y;
}

double Floorplanner::compute_cost() const {
    double area = bounding_area();
    double wl = compute_wirelength();
    // Check aspect ratio
    double max_x = 0, max_y = 0;
    for (auto& m : macros_) {
        max_x = std::max(max_x, m.position.x + m.width);
        max_y = std::max(max_y, m.position.y + m.height);
    }
    double ar = (max_y > 0) ? max_x / max_y : 1.0;
    double ar_penalty = 0;
    if (ar < min_ar_) ar_penalty = (min_ar_ - ar) * 1000;
    if (ar > max_ar_) ar_penalty = (ar - max_ar_) * 1000;

    return area + wl * 10 + ar_penalty;
}

FloorplanResult Floorplanner::solve(int max_iters, double init_temp, double cool_rate) {
    auto t0 = std::chrono::high_resolution_clock::now();

    // Initial packing
    pack();

    std::mt19937 rng(42);
    double temp = init_temp;
    double best_cost = compute_cost();
    auto best_state = macros_;
    double current_cost = best_cost;
    double move_range = 20.0;

    for (int iter = 0; iter < max_iters; ++iter) {
        auto saved = macros_;

        // Random perturbation
        std::uniform_int_distribution<int> move_type(0, 2);
        int mt = move_type(rng);
        if (mt == 0) random_swap(rng);
        else if (mt == 1) random_rotate(rng);
        else random_move(rng, move_range);

        pack();
        double new_cost = compute_cost();
        double delta = new_cost - current_cost;

        // Metropolis criterion
        std::uniform_real_distribution<double> unif(0, 1);
        if (delta < 0 || unif(rng) < std::exp(-delta / temp)) {
            current_cost = new_cost;
            if (new_cost < best_cost) {
                best_cost = new_cost;
                best_state = macros_;
            }
        } else {
            macros_ = saved; // reject move
        }

        temp *= cool_rate;
        move_range *= 0.999;
    }

    macros_ = best_state;
    pack(); // final pack

    auto t1 = std::chrono::high_resolution_clock::now();
    FloorplanResult r;
    double max_x = 0, max_y = 0;
    double macro_area = 0;
    for (auto& m : macros_) {
        max_x = std::max(max_x, m.position.x + m.width);
        max_y = std::max(max_y, m.position.y + m.height);
        macro_area += m.width * m.height;
    }
    r.die_width = max_x; r.die_height = max_y;
    r.total_area = max_x * max_y;
    r.dead_space_pct = r.total_area > 0 ? (1.0 - macro_area / r.total_area) * 100 : 0;
    r.wirelength = compute_wirelength();
    r.iterations = max_iters;
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Floorplan: " + std::to_string((int)r.die_width) + "×" +
                std::to_string((int)r.die_height) + ", dead space: " +
                std::to_string((int)r.dead_space_pct) + "%";
    return r;
}

PhysicalDesign Floorplanner::to_physical_design() const {
    PhysicalDesign pd;
    double max_x = 0, max_y = 0;
    for (auto& m : macros_) {
        max_x = std::max(max_x, m.position.x + m.width);
        max_y = std::max(max_y, m.position.y + m.height);
    }
    pd.die_area = Rect(0, 0, max_x, max_y);
    for (auto& m : macros_) {
        int cid = pd.add_cell(m.name, "MACRO", m.width, m.height);
        pd.cells[cid].position = m.position;
        pd.cells[cid].placed = true;
    }
    for (auto& c : connections_)
        pd.add_net("fn" + std::to_string(c.a) + "_" + std::to_string(c.b), {c.a, c.b});
    return pd;
}

} // namespace sf
