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
    double useful_skew_max_ps = 200.0;  // max allowed useful skew per path
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

// ── H-tree topology configuration ───────────────────────────────────────────
struct HtreeConfig {
    int levels = 4;            // branching levels (2^levels leaf branches)
    double wire_width_um = 0.2; // wider for lower resistance
    bool alternate_xy = true;  // alternate X/Y branching per level
};

// ── Clock shielding configuration ───────────────────────────────────────────
struct ClkShieldConfig {
    double shield_spacing_um = 0.5;  // gap between clock wire and shield
    double shield_width_um = 0.2;
    bool shield_vdd = true;          // VDD shield on one side
    bool shield_gnd = true;          // GND shield on other side
    double coupling_cap_ff_per_um = 0.02; // without shielding
    double shielded_cap_ff_per_um = 0.005; // with shielding (75% reduction)
};

// ── Slew-driven optimization configuration ──────────────────────────────────
struct SlewConfig {
    double max_slew_ps = 100.0;        // max allowed transition time
    double slew_per_load_ps_per_ff = 5.0; // slew degradation per fF load
    double slew_per_wire_ps_per_um = 0.01; // slew degradation per um wire
    bool bottom_up = true;             // bottom-up slew propagation
};

// ── ECO (Engineering Change Order) CTS ──────────────────────────────────────
struct EcoCtsRequest {
    std::vector<int> add_sinks;       // new FF cell IDs to add
    std::vector<int> remove_sinks;    // FF cell IDs to remove
    std::string domain_name;          // which clock domain
};

struct EcoCtsResult {
    int sinks_added = 0;
    int sinks_removed = 0;
    int buffers_added = 0;
    int buffers_removed = 0;
    double skew_before = 0;
    double skew_after = 0;
    double wirelength_delta = 0;
    std::string message;
};

// ── Derived clock result ────────────────────────────────────────────────────
struct DerivedClockResult {
    std::string name;
    double effective_period_ps = 0;
    double cumulative_uncertainty_ps = 0;
    int divider_cells_inserted = 0;
    double phase_shift_ps = 0;
    CtsResult tree_result;
};

class CtsEngine {
public:
    explicit CtsEngine(PhysicalDesign& pd) : pd_(pd) {}

    // ── Original APIs (preserved) ───────────────────────────────────────────
    CtsResult build_clock_tree(const Point& source, const std::vector<int>& sink_cells);
    MultiCtsResult build_multi_clock(const CtsConfig& cfg);
    CtsResult build_useful_skew_tree(const ClockDomain& domain,
                                      const std::vector<double>& sink_slack_ps);
    CtsResult build_power_aware_tree(const ClockDomain& domain);

    // ── Phase 43: H-tree topology ───────────────────────────────────────────
    CtsResult build_htree(const Point& source, const std::vector<int>& sink_cells,
                          const HtreeConfig& cfg = {});

    // ── Phase 43: Clock shielding ───────────────────────────────────────────
    int apply_clock_shielding(const ClkShieldConfig& cfg = {});
    double coupling_reduction_factor(const ClkShieldConfig& cfg) const;

    // ── Phase 43: Slew-driven optimization ──────────────────────────────────
    CtsResult build_slew_driven_tree(const Point& source,
                                      const std::vector<int>& sink_cells,
                                      const SlewConfig& cfg = {});
    double compute_slew_at_node(int node, const SlewConfig& cfg) const;

    // ── Phase 43: Incremental ECO CTS ───────────────────────────────────────
    EcoCtsResult apply_eco(const EcoCtsRequest& req, const CtsConfig& cts_cfg);

    // ── Phase 43: Derived/generated clocks ──────────────────────────────────
    DerivedClockResult build_derived_clock(const ClockDomain& derived,
                                            const ClockDomain& parent);

    // ── Phase C: Useful skew optimization ──────────────────────────────────
    struct UsefulSkewResult {
        int paths_improved = 0;
        double slack_improvement = 0;
        double max_applied_skew = 0;
        std::string message;
    };

    UsefulSkewResult apply_useful_skew_opt(
        const std::vector<int>& sink_cells,
        const std::vector<double>& sink_slack_ps,
        const CtsConfig& cfg = {});

    // ── Accessors ───────────────────────────────────────────────────────────
    int tree_size() const { return (int)tree_.size(); }
    int clock_wire_count() const { return clock_wire_count_; }

private:
    PhysicalDesign& pd_;

    struct TreeNode {
        Point position;
        int cell_id = -1;    // -1 if Steiner point
        int left = -1, right = -1;
        double delay = 0;
        double slew = 0;     // Phase 43: transition time at this node
        bool shielded = false; // Phase 43: whether wire to parent is shielded
    };

    std::vector<TreeNode> tree_;
    int clock_wire_count_ = 0; // track clock wires for shielding

    // DME: build balanced merge tree
    int dme_merge(const std::vector<int>& sinks);

    // Buffer sizing
    std::vector<ClkBufType> buf_lib_;
    void init_buffer_library();
    ClkBufType select_buffer(double load_cap, double wire_length);
    double compute_subtree_cap(int node);

    void partition_sinks(const CtsConfig& cfg);
    void apply_useful_skew(int root, const std::vector<double>& target_delays,
                           const std::vector<int>& sinks);
    double estimate_clock_power(int root, double freq_ghz, double vdd);
    double compute_inter_domain_skew(const std::vector<CtsResult>& results,
                                     const std::vector<ClockGroup>& groups,
                                     const CtsConfig& cfg);
    void generate_multi_cts_report(MultiCtsResult& result, const CtsConfig& cfg);

    // Phase 43: H-tree recursive branching
    int htree_branch(const Point& center, double span_x, double span_y,
                     int level, int max_levels, bool horizontal,
                     const std::vector<int>& nearby_sinks);

    // Phase 43: Slew propagation (bottom-up)
    double propagate_slew(int node, const SlewConfig& cfg);

    // Phase 43: Find nearest tree node to a point
    int find_nearest_tree_node(const Point& p, bool internal_only) const;
};

} // namespace sf
