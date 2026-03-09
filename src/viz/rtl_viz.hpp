#pragma once
// SiliconForge — RTL Circuit Graph Visualizer
// Exports a Netlist to an interactive, force-directed graph in HTML5.

#include "core/netlist.hpp"
#include <string>

namespace sf {

class RtlVisualizer {
public:
    explicit RtlVisualizer(const Netlist& nl) : nl_(nl) {}

    // Generates a standalone HTML string containing the circuit graph UI
    std::string generate_html() const;
    
    // Writes the HTML string to a file
    bool export_to_file(const std::string& filename) const;

private:
    const Netlist& nl_;

    std::string generate_css() const;
    std::string generate_js_logic() const;
    std::string generate_json_data() const;
};

} // namespace sf
