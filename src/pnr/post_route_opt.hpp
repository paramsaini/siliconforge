#pragma once
// SiliconForge — Post-Route Optimizer
// STA-integrated incremental optimization after routing: timing-driven gate
// sizing, critical-net via doubling, slack-aware wire widening, iterative
// convergence loop with real timing closure verification.
// Reference: Cong et al., "Post-Route Gate Sizing", ICCAD 2005
// Reference: Hu et al., "Fast Algorithms for Slew-Constrained Minimum-Cost
//            Buffering and Sizing", IEEE TCAD 2007

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include "timing/sta.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace sf {

// Voltage-threshold variants for multi-Vt optimization
// Reference: Synopsys IC Compiler, Cadence Innovus — Vt swap is the #1 timing closure knob
enum class VtType { ULVT, LVT, SVT, HVT };

inline const char* vt_type_str(VtType vt) {
    switch(vt) {
        case VtType::ULVT: return "ULVT";
        case VtType::LVT:  return "LVT";
        case VtType::SVT:  return "SVT";
        case VtType::HVT:  return "HVT";
        default: return "SVT";
    }
}

struct PostRouteResult {
    double wns_before = 0, wns_after = 0;
    double tns_before = 0, tns_after = 0;
    double hold_wns_before = 0, hold_wns_after = 0;
    int gates_resized = 0;
    int vias_doubled = 0;
    int wires_widened = 0;
    int iterations = 0;
    double wirelength_change_pct = 0;
    double time_ms = 0;
    bool sta_driven = false;
    int buffers_resized = 0;   // backward compatibility alias for gates_resized
    std::string message;

    // Industrial optimization counters
    int cells_vt_swapped = 0;       // cells swapped to different Vt
    int buffers_inserted = 0;       // new buffers inserted for slew/timing
    int cells_cloned = 0;           // high-fanout cells cloned
    int hold_buffers_inserted = 0;  // delay cells for hold fix
    int useful_skew_adjustments = 0; // clock insertion adjusted for borrowing
    int leakage_recovered = 0;      // cells swapped back to HVT after closure
    
    // Power/area metrics
    double leakage_before_uw = 0;   // leakage power before (microwatts)
    double leakage_after_uw = 0;    // leakage power after
    double area_change_pct = 0;     // area change from sizing/cloning
    
    // Detailed timing
    int hold_fixes = 0;             // total hold violations fixed
    int setup_fixes = 0;            // total setup violations fixed
};

// Industrial post-route optimization configuration
struct PostRouteConfig {
    // Feature enables
    bool enable_vt_swap = true;           // Vt library swap (HVT/SVT/LVT/ULVT)
    bool enable_cell_cloning = true;      // clone high-fanout drivers
    bool enable_buffer_insertion = true;  // insert buffers for slew/load
    bool enable_hold_fix = true;          // fix hold violations with delay cells
    bool enable_useful_skew = false;      // borrow time via CTS adjustment (careful!)
    bool enable_leakage_recovery = true;  // swap back to HVT where slack permits
    
    // Thresholds
    int fanout_clone_threshold = 8;       // clone if fanout > this
    double max_slew_ns = 0.5;            // insert buffer if slew exceeds this
    double hold_margin_ns = 0.05;         // required hold margin
    double leakage_recovery_slack = 0.1;  // min slack to allow HVT swap (ns)
    double useful_skew_max_ns = 0.2;      // max clock adjustment for useful skew
    
    // Vt delay/power model (relative to SVT)
    double ulvt_delay_factor = 0.75;      // 25% faster than SVT
    double lvt_delay_factor = 0.85;       // 15% faster than SVT
    double svt_delay_factor = 1.00;       // baseline
    double hvt_delay_factor = 1.20;       // 20% slower than SVT
    double ulvt_leakage_factor = 8.0;     // 8× leakage vs SVT
    double lvt_leakage_factor = 4.0;      // 4× leakage vs SVT
    double svt_leakage_factor = 1.0;      // baseline
    double hvt_leakage_factor = 0.25;     // 4× lower leakage than SVT
};

class PostRouteOptimizer {
public:
    PostRouteOptimizer(Netlist& nl, PhysicalDesign& pd)
        : nl_(nl), pd_(pd), lib_(nullptr) {}

    PostRouteOptimizer(Netlist& nl, PhysicalDesign& pd, const LibertyLibrary* lib)
        : nl_(nl), pd_(pd), lib_(lib) {}

    void set_clock_period(double period) { clock_period_ = period; }
    void set_max_iterations(int n) { max_iterations_ = n; }
    void set_slack_threshold(double t) { slack_threshold_ = t; }

    PostRouteResult optimize(double target_wns = 0);

    // Industrial configuration
    void set_config(const PostRouteConfig& cfg) { config_ = cfg; }
    PostRouteConfig& config() { return config_; }
    
    // Set Vt for a specific gate (manual override)
    void set_gate_vt(GateId gid, VtType vt) { gate_vt_[gid] = vt; }
    VtType get_gate_vt(GateId gid) const {
        auto it = gate_vt_.find(gid);
        return (it != gate_vt_.end()) ? it->second : VtType::SVT;
    }

private:
    Netlist& nl_;
    PhysicalDesign& pd_;
    const LibertyLibrary* lib_ = nullptr;
    double clock_period_ = 0;       // 0 = auto-estimate
    int max_iterations_ = 5;
    double slack_threshold_ = 0;    // nets with slack below this are "critical"

    PostRouteConfig config_;
    std::unordered_map<GateId, VtType> gate_vt_; // per-gate Vt assignment

    // --- STA-driven analysis ---
    bool can_run_sta() const;
    StaResult run_sta() const;
    double auto_clock_period() const;

    // Net correlation between Netlist and PhysicalDesign
    std::unordered_map<std::string, int> build_phys_net_map() const;

    // Critical element identification from STA results
    std::unordered_set<GateId> find_critical_gates(const StaResult& sta) const;
    std::unordered_set<int>    find_critical_phys_nets(
        const StaResult& sta,
        const std::unordered_map<std::string, int>& net_map) const;

    // --- STA-driven optimization passes ---
    int timing_gate_sizing(const StaResult& sta);
    int timing_via_doubling(const std::unordered_set<int>& crit_nets);
    int timing_wire_widening(const std::unordered_set<int>& crit_nets,
                             double max_factor = 1.5);

    // --- Heuristic (legacy) fallback ---
    PostRouteResult optimize_legacy(double target_wns);
    int via_doubling_heuristic();
    int wire_widening_heuristic(double factor = 1.5);
    int buffer_sizing_heuristic();
    double estimate_wns_hpwl() const;
    double estimate_tns_hpwl() const;

    // --- Industrial optimization passes ---
    int vt_swap_for_timing(const StaResult& sta);
    int clone_high_fanout(const StaResult& sta);
    int insert_buffers_for_slew(const StaResult& sta);
    int fix_hold_violations(const StaResult& sta);
    int useful_skew_optimization(const StaResult& sta);
    int leakage_recovery(const StaResult& sta);
    
    // Helper: estimate gate leakage based on Vt
    double gate_leakage(GateId gid) const;
    double total_leakage() const;
    
    // Helper: get delay factor for a gate based on its Vt
    double vt_delay_factor(GateId gid) const;
};

} // namespace sf
