#pragma once
// SiliconForge — Clock Tree Synthesis (CTS)
// Zero-skew DME algorithm + buffer insertion.
// Reference: T.H. Chao et al., "Zero Skew Clock Routing with Minimum Wirelength", IEEE TCAD 1992

#include "pnr/physical.hpp"
#include <functional>
#include <string>
#include <vector>

namespace sf {

struct ClkBufType {
    std::string name;
    double drive_strength; // output resistance, Ohm
    double input_cap;      // fF
    double intrinsic_delay; // ps
    double area;           // um²
};

struct CtsResult {
    double skew = 0;
    double wirelength = 0;
    int buffers_inserted = 0;
    double max_latency_ps = 0;
    double min_latency_ps = 0;
    double time_ms = 0;
    std::string message;
};

// --- Multi-clock domain structs ---

struct ClockDomain {
    std::string name;           // e.g., "clk_sys", "clk_io"
    Point source;               // clock root location
    double period_ps = 1000.0;  // clock period
    double uncertainty_ps = 50; // jitter/uncertainty
    std::vector<int> sink_cells; // FF cell IDs in this domain
    double target_skew_ps = 0;  // 0 = zero-skew, >0 = useful skew target
    bool is_generated = false;  // true if derived from another clock
    std::string parent_clock;   // source clock name if generated
    int divide_by = 1;          // clock divider ratio
};

struct ClockGroup {
    enum Relationship { SYNCHRONOUS, ASYNCHRONOUS, EXCLUSIVE } type = ASYNCHRONOUS;
    std::vector<std::string> clock_names;
};

struct CtsConfig {
    std::vector<ClockDomain> domains;
    std::vector<ClockGroup> groups;
    double max_skew_ps = 100.0;        // global max skew constraint
    double max_insertion_delay_ps = 500; // max latency
    int max_buffer_levels = 10;
    double max_transition_ps = 100;     // max buffer output transition
    bool enable_useful_skew = false;
    bool enable_gate_cloning = false;   // clone gates for better buffering
    bool power_aware = false;           // minimize clock power
    double clock_gating_threshold = 0.3; // ICG insertion threshold
};

struct MultiCtsResult {
    std::vector<CtsResult> domain_results;  // per-domain results
    double total_skew = 0;                  // worst-case across all domains
    double total_wirelength = 0;
    int total_buffers = 0;
    double inter_domain_skew = 0;           // max skew between sync clocks
    double total_clock_power_uw = 0;        // estimated clock tree power
    std::string report;                     // detailed multi-clock report
};

class CtsEngine {
public:
    explicit CtsEngine(PhysicalDesign& pd) : pd_(pd) {}

    // Build clock tree for the given sink points
    CtsResult build_clock_tree(const Point& source, const std::vector<int>& sink_cells);

    // Multi-clock CTS — builds trees for all domains
    MultiCtsResult build_multi_clock(const CtsConfig& cfg);

    // Useful skew — adjust insertion delays per sink for timing optimization
    CtsResult build_useful_skew_tree(const ClockDomain& domain,
                                      const std::vector<double>& sink_slack_ps);

    // Power-aware CTS — minimize switching power
    CtsResult build_power_aware_tree(const ClockDomain& domain);

private:
    PhysicalDesign& pd_;

    struct TreeNode {
        Point position;
        int cell_id = -1;    // -1 if Steiner point
        int left = -1, right = -1;
        double delay = 0;
    };

    std::vector<TreeNode> tree_;

    // DME: build balanced merge tree
    int dme_merge(const std::vector<int>& sinks);

    // Buffer sizing
    std::vector<ClkBufType> buf_lib_;
    void init_buffer_library();
    ClkBufType select_buffer(double load_cap, double wire_length);
    double compute_subtree_cap(int node);

    // Group sinks by clock domain for independent tree construction
    void partition_sinks(const CtsConfig& cfg);

    // Adjust delays for useful skew (non-zero target skew)
    void apply_useful_skew(int root, const std::vector<double>& target_delays,
                           const std::vector<int>& sinks);

    // Estimate clock tree power
    double estimate_clock_power(int root, double freq_ghz, double vdd);

    // Inter-domain skew calculation for synchronous groups
    double compute_inter_domain_skew(const std::vector<CtsResult>& results,
                                     const std::vector<ClockGroup>& groups,
                                     const CtsConfig& cfg);

    // Generate multi-clock report
    void generate_multi_cts_report(MultiCtsResult& result, const CtsConfig& cfg);
};

} // namespace sf
