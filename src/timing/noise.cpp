// SiliconForge — Noise / PSIJ Analyzer Implementation
#include "timing/noise.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

namespace sf {

double NoiseAnalyzer::estimate_supply_noise(double x, double y) const {
    // Model: noise increases with distance from power pads and cell density
    // Assume pads at corners
    double die_w = pd_.die_area.width();
    double die_h = pd_.die_area.height();
    double dx = std::min(x - pd_.die_area.x0, pd_.die_area.x1 - x);
    double dy = std::min(y - pd_.die_area.y0, pd_.die_area.y1 - y);
    double dist_to_edge = std::min(dx, dy);
    double max_dist = std::min(die_w, die_h) / 2;

    // Base noise: proportional to distance from edge
    double base_noise = vdd_ * 1000 * 0.03 * (dist_to_edge / max_dist); // up to 3% VDD

    // Add contribution from nearby switching cells
    double cell_noise = 0;
    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        double cx = c.position.x + c.width/2;
        double cy = c.position.y + c.height/2;
        double d = std::sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy));
        if (d > 0 && d < 20) {
            cell_noise += c.width * c.height * 0.1 / d; // nearby large cells → more noise
        }
    }

    return base_noise + cell_noise;
}

double NoiseAnalyzer::noise_to_jitter(double noise_mv, double vdd_mv) const {
    // Simple model: jitter ∝ noise / slew_rate
    // Assume slew = VDD / 100ps typical
    double slew = vdd_mv / 100; // mV/ps
    return (slew > 0) ? noise_mv / slew : 0; // ps
}

NoiseResult NoiseAnalyzer::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    NoiseResult r;
    double noise_margin = vdd_ * 1000 * noise_margin_pct_ / 100.0;
    r.noise_margin_mv = noise_margin;

    double total_noise_sq = 0;
    int count = 0;

    for (int ni = 0; ni < (int)pd_.nets.size(); ++ni) {
        auto& net = pd_.nets[ni];
        if (net.cell_ids.empty()) continue;

        // Estimate noise at each pin of the net
        double max_noise = 0;
        for (auto cid : net.cell_ids) {
            auto& c = pd_.cells[cid];
            double noise = estimate_supply_noise(
                c.position.x + c.width/2, c.position.y + c.height/2);
            max_noise = std::max(max_noise, noise);
        }

        double jitter = noise_to_jitter(max_noise, vdd_ * 1000);
        bool violates = max_noise > noise_margin;

        r.details.push_back({ni, net.name, max_noise, jitter, violates});
        r.peak_noise_mv = std::max(r.peak_noise_mv, max_noise);
        r.psij_ps = std::max(r.psij_ps, jitter);
        total_noise_sq += max_noise * max_noise;
        if (violates) r.noise_violations++;
        count++;
    }

    r.rms_noise_mv = count > 0 ? std::sqrt(total_noise_sq / count) : 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Peak noise: " + std::to_string((int)r.peak_noise_mv) + "mV, " +
                "PSIJ: " + std::to_string((int)r.psij_ps) + "ps, " +
                std::to_string(r.noise_violations) + " violations";
    return r;
}

} // namespace sf
