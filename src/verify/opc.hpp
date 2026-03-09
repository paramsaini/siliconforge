#pragma once
// SiliconForge — OPC (Optical Proximity Correction) / Lithography Simulator
// Models the image formation in photolithography and applies corrections.
// Reference: Wong, "Resolution Enhancement Techniques in Optical Lithography", SPIE 2001

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <cmath>

namespace sf {

struct OpcResult {
    int features_analyzed = 0;
    int corrections_applied = 0;
    double avg_bias_nm = 0;
    double max_edge_placement_error_nm = 0;
    double time_ms = 0;
    std::string message;

    struct Correction {
        Rect original;
        Rect corrected;
        double bias_nm;
        std::string type; // "serif", "hammerhead", "bias"
    };
    std::vector<Correction> corrections;
};

struct LithoParams {
    double wavelength_nm = 193;     // ArF excimer laser
    double numerical_aperture = 0.85;
    double sigma = 0.75;            // partial coherence
    double resist_threshold = 0.3;
    double min_feature_nm = 45;
};

class OpcEngine {
public:
    OpcEngine(const PhysicalDesign& pd, LithoParams params = {})
        : pd_(pd), params_(params) {}

    OpcResult apply_opc();

    // Simulate aerial image intensity at a point
    double aerial_image(double x, double y) const;

    // Get the critical dimension
    double resolution_limit() const {
        return 0.5 * params_.wavelength_nm / params_.numerical_aperture;
    }

private:
    const PhysicalDesign& pd_;
    LithoParams params_;

    // Compute edge bias for a feature
    double compute_bias(double feature_width) const;
    // Apply correction to a rectangle
    Rect apply_correction(const Rect& feature) const;
};

} // namespace sf
