#pragma once
// SiliconForge — Standalone HTML5/Canvas Visualizer
// Generates an interactive, zero-dependency HTML file to visualize
// the physical design (placement, routing, congestion, power maps).
// Serves as the high-performance visualization backend.

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

class HtmlVisualizer {
public:
    explicit HtmlVisualizer(const PhysicalDesign& pd) : pd_(pd) {}

    // Add optional data overlays
    void set_congestion_map(const std::vector<std::vector<double>>& map);
    void set_power_map(const std::vector<std::vector<double>>& map);
    void highlight_net(const std::string& net_name);

    // Generate the complete HTML string
    std::string generate_html() const;

    // Export to file
    bool export_to_file(const std::string& filename) const;

private:
    const PhysicalDesign& pd_;
    std::vector<std::vector<double>> congestion_map_;
    std::vector<std::vector<double>> power_map_;
    std::string highlight_net_;

    // Helpers to generate JSON data embedded in HTML
    std::string generate_json_data() const;
    std::string generate_js_logic() const;
    std::string generate_css() const;
};

} // namespace sf
