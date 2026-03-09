#pragma once
// SiliconForge — Post-Route Optimizer
// Performs incremental optimization after routing: buffer sizing,
// wire spreading, via doubling, timing-driven optimization.
// Reference: Cong et al., "Post-Route Gate Sizing", ICCAD 2005

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct PostRouteResult {
    double wns_before = 0, wns_after = 0;
    double tns_before = 0, tns_after = 0;
    int buffers_resized = 0;
    int vias_doubled = 0;
    int wires_widened = 0;
    double wirelength_change_pct = 0;
    double time_ms = 0;
    std::string message;
};

class PostRouteOptimizer {
public:
    PostRouteOptimizer(Netlist& nl, PhysicalDesign& pd)
        : nl_(nl), pd_(pd) {}

    PostRouteResult optimize(double target_wns = 0);

private:
    Netlist& nl_;
    PhysicalDesign& pd_;

    // Optimization passes
    int via_doubling();
    int wire_widening(double factor = 1.5);
    int buffer_sizing();
    double estimate_wns() const;
    double estimate_tns() const;
};

} // namespace sf
