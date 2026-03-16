#pragma once
// SiliconForge — Power Grid Planning
// Creates VDD/VSS rings, power stripes, standard cell rails, and via stacks.
// Reference: Cadence Innovus power planning methodology

#include "pnr/physical.hpp"
#include <string>
#include <vector>

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

    // IR feedback loop
    double ir_target_mv = 50.0;            // max acceptable IR drop (mV)
    int ir_max_iterations = 5;             // max feedback iterations
    double ir_convergence_threshold = 1.0; // stop if improvement < this (mV)
    double vdd = 1.8;                      // supply voltage for IR analysis
    double total_current_ma = 100.0;       // estimated total current (mA)

    // EM-aware sizing
    bool enable_em_sizing = false;
    double j_max_em_ma_per_um = 1.0;       // EM current density limit

    // Adaptive pitch
    bool enable_adaptive_pitch = false;
    double min_stripe_pitch = 20.0;
    double max_stripe_pitch = 80.0;
};

struct IrFeedbackResult {
    int iterations = 0;
    double initial_worst_ir_mv = 0;
    double final_worst_ir_mv = 0;
    bool converged = false;
    int extra_stripes_added = 0;
    std::vector<double> ir_per_iteration;
    std::string message;
};

struct PowerPlanResult {
    int rings  = 0;
    int stripes = 0;
    int rails  = 0;
    int vias   = 0;
    double total_wire_length = 0;
    std::string message;
    IrFeedbackResult ir_feedback;
    int em_widened_stripes = 0;
};

class PowerPlanner {
public:
    explicit PowerPlanner(PhysicalDesign& pd) : pd_(pd) {}
    PowerPlanResult plan(const PowerPlanConfig& cfg = {});

    // Closed-loop IR-feedback power planning
    PowerPlanResult plan_with_ir_feedback(const PowerPlanConfig& cfg = {});

    // EM-aware stripe width
    static double compute_em_width(double peak_current_ma, double j_max_em, double min_width);

    // Adaptive pitch computation
    static std::vector<double> compute_adaptive_pitches(
        int num_regions, const std::vector<double>& region_currents,
        double min_pitch, double max_pitch);

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
    void add_stripes_at_positions(const std::vector<double>& x_positions,
                                   const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_rings(const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_stripes(const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_rails(const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_vias(const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_domain_rings(const DomainGrid& dg, const PowerPlanConfig& cfg, PowerPlanResult& res);
    void create_domain_stripes(const DomainGrid& dg, const PowerPlanConfig& cfg, PowerPlanResult& res);
};

} // namespace sf
