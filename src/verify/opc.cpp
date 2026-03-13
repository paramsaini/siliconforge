// SiliconForge — OPC / Lithography Simulator Implementation
#include "verify/opc.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace sf {

double OpcEngine::aerial_image(double x, double y) const {
    // Simplified aerial image: sum of rect contributions using sinc-based model
    // I(x,y) = |∑ sinc(π·NA·(x-xi)/λ) · sinc(π·NA·(y-yi)/λ)|²
    double intensity = 0;
    double k = M_PI * params_.numerical_aperture / params_.wavelength_nm;

    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        // Cell rectangle contribution
        double cx = (c.position.x + c.width/2) * 1000; // um → nm
        double cy = (c.position.y + c.height/2) * 1000;
        double hw = c.width * 500; // half-width in nm
        double hh = c.height * 500;

        // Simplified: Gaussian approximation of diffraction pattern
        double dx = (x - cx); double dy = (y - cy);
        double sigma_x = hw / 2.0;
        double sigma_y = hh / 2.0;
        if (sigma_x > 0 && sigma_y > 0) {
            double contrib = std::exp(-(dx*dx)/(2*sigma_x*sigma_x) - (dy*dy)/(2*sigma_y*sigma_y));
            intensity += contrib;
        }
    }
    return std::min(1.0, intensity);
}

double OpcEngine::compute_bias(double feature_width) const {
    // Bias depends on feature size relative to resolution limit
    double res_limit = resolution_limit();
    double ratio = feature_width / res_limit;

    if (ratio > 2.0) return 0; // No correction needed for large features
    if (ratio < 0.5) return feature_width * 0.3; // Large bias for sub-resolution

    // Linear interpolation: smaller features get more bias
    return feature_width * (1.0 - ratio) * 0.2;
}

Rect OpcEngine::apply_correction(const Rect& feature) const {
    double w = feature.width() * 1000; // um → nm
    double h = feature.height() * 1000;

    double bias_w = compute_bias(w);
    double bias_h = compute_bias(h);

    // Expand feature by bias amount
    return Rect(
        feature.x0 - bias_w / 2000.0, // nm → um, half on each side
        feature.y0 - bias_h / 2000.0,
        feature.x1 + bias_w / 2000.0,
        feature.y1 + bias_h / 2000.0
    );
}

OpcResult OpcEngine::apply_opc() {
    auto t0 = std::chrono::high_resolution_clock::now();
    OpcResult r;
    double res_limit = resolution_limit();

    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        r.features_analyzed++;

        Rect original(c.position.x, c.position.y,
                     c.position.x + c.width, c.position.y + c.height);

        double w_nm = c.width * 1000;
        double h_nm = c.height * 1000;

        // Check if near resolution limit
        if (w_nm < res_limit * 2.0 || h_nm < res_limit * 2.0) {
            Rect corrected = apply_correction(original);
            double bias = compute_bias(std::min(w_nm, h_nm));

            std::string corr_type = "bias";
            if (w_nm < res_limit) corr_type = "hammerhead";  // very small features
            else if (w_nm < res_limit * 1.5) corr_type = "serif"; // medium features

            r.corrections.push_back({original, corrected, bias, corr_type});
            r.corrections_applied++;
            r.avg_bias_nm += bias;
            r.max_edge_placement_error_nm = std::max(r.max_edge_placement_error_nm, bias);
        }
    }

    // Also check wire features
    for (auto& w : pd_.wires) {
        r.features_analyzed++;
        double wire_w_nm = w.width * 1000;
        if (wire_w_nm < res_limit * 2.0) {
            double bias = compute_bias(wire_w_nm);
            Rect orig(std::min(w.start.x, w.end.x), std::min(w.start.y, w.end.y),
                     std::max(w.start.x, w.end.x), std::max(w.start.y, w.end.y));
            Rect corrected = apply_correction(orig);
            r.corrections.push_back({orig, corrected, bias, "wire_bias"});
            r.corrections_applied++;
            r.avg_bias_nm += bias;
        }
    }

    if (r.corrections_applied > 0)
        r.avg_bias_nm /= r.corrections_applied;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.corrections_applied) + "/" +
                std::to_string(r.features_analyzed) + " features corrected, " +
                "avg bias: " + std::to_string((int)r.avg_bias_nm) + "nm, " +
                "resolution limit: " + std::to_string((int)res_limit) + "nm";
    return r;
}

// ── Tier 2: SRAF insertion ───────────────────────────────────────────
OpcEngine::SrafResult OpcEngine::insert_sraf(const SrafConfig& cfg) {
    SrafResult result;
    double res_limit = resolution_limit();

    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        result.features_checked++;

        double w_nm = c.width * 1000;
        double h_nm = c.height * 1000;

        // Only insert SRAF for features near resolution limit
        if (w_nm > res_limit * 3.0 && h_nm > res_limit * 3.0) continue;

        double cx = c.position.x;
        double cy = c.position.y;

        // Insert assist features on each side of the narrow dimension
        for (int side = 0; side < cfg.max_sraf_per_side; side++) {
            double offset_nm = cfg.sraf_spacing_nm + side * (cfg.sraf_width_nm + cfg.min_pitch_nm);
            double offset_um = offset_nm / 1000.0;
            double sraf_w_um = cfg.sraf_width_nm / 1000.0;

            if (w_nm <= res_limit * 2.0) {
                // Add SRAF left and right of feature
                result.assist_features.push_back(Rect(
                    cx - offset_um - sraf_w_um, cy,
                    cx - offset_um, cy + c.height));
                result.assist_features.push_back(Rect(
                    cx + c.width + offset_um, cy,
                    cx + c.width + offset_um + sraf_w_um, cy + c.height));
                result.sraf_inserted += 2;
            }
            if (h_nm <= res_limit * 2.0) {
                // Add SRAF above and below
                result.assist_features.push_back(Rect(
                    cx, cy - offset_um - sraf_w_um,
                    cx + c.width, cy - offset_um));
                result.assist_features.push_back(Rect(
                    cx, cy + c.height + offset_um,
                    cx + c.width, cy + c.height + offset_um + sraf_w_um));
                result.sraf_inserted += 2;
            }
        }
    }

    result.message = std::to_string(result.sraf_inserted) + " SRAFs inserted for " +
                     std::to_string(result.features_checked) + " features";
    return result;
}

double OpcEngine::estimate_psm_improvement() const {
    // Alternating PSM doubles resolution: k1 drops from 0.5 to ~0.25
    // Process window improvement is proportional to NA²
    double k1_standard = 0.5;
    double k1_psm = 0.25;
    return k1_standard / k1_psm;  // ~2× improvement factor
}

} // namespace sf
