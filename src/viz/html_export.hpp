#pragma once
// SiliconForge — Production-Grade HTML5/Canvas Layout Visualizer
// Interactive zero-dependency viewer: per-layer visibility, net highlighting,
// cell inspection, DRC markers, timing path overlay, Viridis heatmaps,
// search, measurement, minimap, placement animation playback.

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct VizTimingPath {
    std::string name;
    double slack = 0;
    std::vector<int> cell_ids;
    std::vector<int> net_ids;
};

struct VizDrcMarker {
    double x = 0, y = 0;
    std::string type;
    std::string message;
    int severity = 1; // 0=warning, 1=error
};

struct PlacementSnapshot {
    int iteration = 0;
    double cost = 0;
    std::vector<Point> positions;
};

class HtmlVisualizer {
public:
    explicit HtmlVisualizer(const PhysicalDesign& pd) : pd_(pd) {}

    void set_congestion_map(const std::vector<std::vector<double>>& map);
    void set_power_map(const std::vector<std::vector<double>>& map);
    void set_timing_paths(const std::vector<VizTimingPath>& paths);
    void set_drc_markers(const std::vector<VizDrcMarker>& markers);
    void set_placement_history(const std::vector<PlacementSnapshot>& history);
    void highlight_net(const std::string& net_name);

    std::string generate_html() const;
    bool export_to_file(const std::string& filename) const;

private:
    const PhysicalDesign& pd_;
    std::vector<std::vector<double>> congestion_map_;
    std::vector<std::vector<double>> power_map_;
    std::vector<VizTimingPath> timing_paths_;
    std::vector<VizDrcMarker> drc_markers_;
    std::vector<PlacementSnapshot> placement_history_;
    std::string highlight_net_;

    std::string generate_json_data() const;
    std::string generate_js_logic() const;
    std::string generate_css() const;
};

} // namespace sf
