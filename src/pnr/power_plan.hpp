#pragma once
// SiliconForge — Power Grid Planning
// Creates VDD/VSS rings, power stripes, standard cell rails, and via stacks.
// Reference: Cadence Innovus power planning methodology

#include "pnr/physical.hpp"
#include <string>

namespace sf {

struct PowerPlanConfig {
    double ring_width  = 2.0;     // power ring width (um)
    double ring_offset = 1.0;     // offset from die boundary
    double stripe_width = 1.5;    // vertical stripe width
    double stripe_pitch = 40.0;   // stripe center-to-center spacing
    int stripe_layer = 4;         // metal layer for vertical stripes (M5)
    int ring_layer   = 3;         // metal layer for rings (M4)
    double rail_width = 0.5;      // std cell rail width
    int rail_layer   = 0;         // M1
};

struct PowerPlanResult {
    int rings  = 0;
    int stripes = 0;
    int rails  = 0;
    int vias   = 0;
    double total_wire_length = 0;
    std::string message;
};

class PowerPlanner {
public:
    explicit PowerPlanner(PhysicalDesign& pd) : pd_(pd) {}
    PowerPlanResult plan(const PowerPlanConfig& cfg = {});

    // Tier 3: Multi-Vdd power grid
    struct DomainGrid {
        std::string domain_name;
        double voltage = 1.0;
        Rect region;
        std::string power_net = "VDD";
        std::string ground_net = "VSS";
    };
    void add_domain_grid(const DomainGrid& dg) { domain_grids_.push_back(dg); }
    PowerPlanResult plan_multi_vdd(const PowerPlanConfig& cfg = {});

private:
    PhysicalDesign& pd_;
    std::vector<DomainGrid> domain_grids_;
    void create_rings(const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_stripes(const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_rails(const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_vias(const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_domain_rings(const DomainGrid& dg, const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_domain_stripes(const DomainGrid& dg, const PowerPlanConfig& cfg, PowerPlanResult& res);
};

} // namespace sf
