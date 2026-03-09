#pragma once
// SiliconForge — Ultimate GUI Dashboard
// Generates a complete end-to-end interactive dashboard

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include <string>

namespace sf {

class DashboardVisualizer {
public:
    DashboardVisualizer(const Netlist& nl, const PhysicalDesign& pd);

    // Generates the final full-stack HTML dashboard
    std::string generate_html() const;

private:
    const Netlist& nl_;
    const PhysicalDesign& pd_;

    std::string generate_css() const;
    std::string generate_js() const;
    
    // Different views
    std::string render_nav() const;
    std::string render_rtl_view() const;
    std::string render_layout_view() const;
    std::string render_metrics_view() const;
};

} // namespace sf
