#pragma once
// SiliconForge — Simulation Waveform Visualizer
// Exports EventSimulator traces into an interactive HTML5 waveform canvas.

#include "sim/simulator.hpp"
#include <string>

namespace sf {

class SimVisualizer {
public:
    explicit SimVisualizer(const Netlist& nl, const SimTrace& trace) 
        : nl_(nl), trace_(trace) {}

    std::string generate_html() const;
    bool export_to_file(const std::string& filename) const;

private:
    const Netlist& nl_;
    const SimTrace& trace_;

    std::string generate_css() const;
    std::string generate_js_logic() const;
    std::string generate_json_data() const;
};

} // namespace sf
