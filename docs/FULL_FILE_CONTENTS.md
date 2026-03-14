# SiliconForge Full File Contents

## FILE 1: src/timing/sta.hpp (COMPLETE)

```cpp
#pragma once
// SiliconForge — Static Timing Analysis (STA)
// Graph-based path analysis with slew propagation, hold checks, multi-corner,
// OCV (On-Chip Variation) and AOCV (Advanced OCV) derating.
// Reference: Sapatnekar, "Timing", Springer 2004
// Reference: AOCV: Sirichotiyakul et al., "Statistical SSTA with OCV", ICCAD 2008

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <limits>
#include <cmath>

namespace sf {

// OCV analysis mode
enum class OcvMode { NONE, OCV, AOCV, POCV };

// AOCV depth-dependent derating table
// Uses √N statistical averaging: deeper paths accumulate less variation.
struct AocvTable {
    double late_variation  = 0.10;  // 10% base late variation (setup pessimism)
    double early_variation = 0.10;  // 10% base early variation (hold pessimism)
    int    min_depth       = 1;     // minimum depth clamp

    // Late derate: multiplier > 1 (makes data path slower for setup worst-case)
    double late_derate(int depth) const {
        depth = std::max(depth, min_depth);
        return 1.0 + late_variation / std::sqrt(static_cast<double>(depth));
    }
    // Early derate: multiplier < 1 (makes data path faster for hold worst-case)
    double early_derate(int depth) const {
        depth = std::max(depth, min_depth);
        return 1.0 - early_variation / std::sqrt(static_cast<double>(depth));
    }
};

// POCV: Parametric OCV with per-cell sigma-based statistical derating
// Reference: Synopsys POCV Application Note
// Each cell has a sigma (std deviation) of its delay variation.
// Path-level variation = RSS(σ_1, σ_2, ..., σ_N) = √(Σ σ_i²)
// Final derate = mean ± N_sigma × path_sigma
struct PocvTable {
    double n_sigma = 3.0;                   // sigma multiplier (3-sigma = 99.7% coverage)
    double default_sigma_pct = 0.05;        // default 5% sigma for cells without explicit data
    std::unordered_map<std::string, double> cell_sigma; // per-cell-type sigma as fraction of delay
    
    // Get sigma fraction for a cell type
    double get_sigma(const std::string& cell_type) const {
        auto it = cell_sigma.find(cell_type);
        return (it != cell_sigma.end()) ? it->second : default_sigma_pct;
    }
    
    // Set sigma for a specific cell type
    void set_cell_sigma(const std::string& type, double sigma_pct) {
        cell_sigma[type] = sigma_pct;
    }
};

// CPPR: Common Path Pessimism Removal
// Removes artificial pessimism from shared launch/capture clock paths
// Reference: DAC'21, Huang et al., "A Provably Good Algorithm for CPPR"
struct CpprConfig {
    bool enabled = false;
    // In full implementation, we'd track clock tree topology.
    // With CTS insertion delays only, we approximate:
    // common_path = min(launch_insertion, capture_insertion)
    // credit = common_delay × |late_derate - early_derate|
};

// Crosstalk delta-delay estimation
// Models coupling capacitance between adjacent wires
// Reference: Devgan, "Efficient Coupled Noise Estimation", ICCAD 1997
struct CrosstalkConfig {
    bool enabled = false;
    double coupling_cap_per_um = 0.00015;   // fF/um coupling cap
    double miller_factor = 1.5;             // 0=same-dir, 2=opposite-dir, 1.5=pessimistic default
    double aggressor_slew = 0.05;           // default aggressor slew (ns)
    double min_spacing_um = 0.14;           // below this, coupling is max
    double max_coupling_distance_um = 1.0;  // beyond this, ignore coupling
};

// Multi-corner derating factors
struct CornerDerate {
    std::string name = "typical";
    double cell_derate = 1.0;   // multiply cell delay (LATE/setup data path)
    double wire_derate = 1.0;   // multiply wire delay (LATE/setup data path)
    double early_cell  = 1.0;   // for hold (best-case cell, EARLY data path)
    double early_wire  = 1.0;   // for hold (best-case wire, EARLY data path)

    // OCV: separate clock path derating (for capture clock)
    // Setup: capture clock uses early derate (clock arrives early = pessimistic)
    // Hold:  capture clock uses late derate (clock arrives late = pessimistic)
    double clock_late_cell  = 1.0;
    double clock_late_wire  = 1.0;
    double clock_early_cell = 1.0;
    double clock_early_wire = 1.0;
};

struct TimingArc {
    NetId from;
    NetId to;
    double delay = 0;
    double slew = 0;
    GateId gate = -1;
};

struct PinTiming {
    double arrival_rise = 0;
    double arrival_fall = 0;
    double required_rise = std::numeric_limits<double>::max();
    double required_fall = std::numeric_limits<double>::max();
    double slack_rise = std::numeric_limits<double>::max();
    double slack_fall = std::numeric_limits<double>::max();
    double slew_rise = 0;
    double slew_fall = 0;
    // Hold analysis
    double hold_arrival_rise = 0;
    double hold_arrival_fall = 0;
    double hold_required_rise = 0;
    double hold_required_fall = 0;
    double hold_slack_rise = std::numeric_limits<double>::max();
    double hold_slack_fall = std::numeric_limits<double>::max();

    double worst_arrival() const { return std::max(arrival_rise, arrival_fall); }
    double worst_slack() const { return std::min(slack_rise, slack_fall); }
    double worst_hold_slack() const { return std::min(hold_slack_rise, hold_slack_fall); }
    double best_arrival() const { return std::min(hold_arrival_rise, hold_arrival_fall); }
};

struct TimingPath {
    std::vector<NetId> nets;
    std::vector<GateId> gates;
    double delay = 0;
    double slack = 0;
    std::string startpoint;
    std::string endpoint;
    bool is_hold = false;
    int depth = 0;              // logic depth (for AOCV reporting)

    // Industrial additions
    double cppr_credit = 0;     // CPPR pessimism removal credit (ns)
    double pba_slack = 0;       // Path-Based Analysis adjusted slack
    double pba_delay = 0;       // Path-Based Analysis delay
    double crosstalk_delta = 0; // Crosstalk-induced delay change (ns)
    double path_sigma = 0;      // POCV: RSS path sigma (ns)
    bool pba_valid = false;     // Whether PBA was computed for this path
};

struct StaResult {
    double wns = 0;         // Worst Negative Slack (setup)
    double tns = 0;         // Total Negative Slack (setup)
    double hold_wns = 0;    // Worst Negative Slack (hold)
    double hold_tns = 0;    // Total Negative Slack (hold)
    double clock_period = 0;
    int num_violations = 0;
    int hold_violations = 0;
    int num_endpoints = 0;
    std::vector<TimingPath> critical_paths; // top N worst (setup + hold)
    std::string corner_name = "typical";
    OcvMode ocv_mode = OcvMode::NONE;
    double time_ms = 0;
    std::string message;

    // Industrial additions
    double cppr_total_credit = 0;   // sum of CPPR credits across critical paths
    double pba_wns = 0;             // PBA-adjusted WNS
    double pba_tns = 0;             // PBA-adjusted TNS
    int pba_violations = 0;         // PBA-adjusted violation count
    double max_crosstalk_delta = 0; // largest crosstalk delay delta
    bool cppr_enabled = false;
    bool pba_enabled = false;
    bool crosstalk_enabled = false;
};

class StaEngine {
public:
    StaEngine(const Netlist& nl, const LibertyLibrary* lib = nullptr,
              const PhysicalDesign* pd = nullptr)
        : nl_(nl), lib_(lib), pd_(pd) {}

    // Run full STA with given clock period (single corner)
    StaResult analyze(double clock_period, int num_paths = 5);

    // Run multi-corner STA — returns per-corner results + merged worst
    std::vector<StaResult> analyze_multicorner(double clock_period, int num_paths = 5);

    // Set clock uncertainty (jitter + skew)
    void set_clock_uncertainty(double setup_unc, double hold_unc) {
        setup_uncertainty_ = setup_unc; hold_uncertainty_ = hold_unc;
    }

    // Set per-DFF clock insertion delay (from CTS)
    void set_clock_insertion(GateId dff_id, double delay) {
        clock_insertion_[dff_id] = delay;
    }

    // --- OCV/AOCV configuration ---
    void set_ocv_mode(OcvMode mode) { ocv_mode_ = mode; }
    void set_aocv_table(const AocvTable& table) { aocv_table_ = table; }

    // Convenience: enable OCV with foundry-standard derating
    void enable_ocv(double late_cell = 1.15, double early_cell = 0.85) {
        ocv_mode_ = OcvMode::OCV;
        ocv_late_cell_ = late_cell;
        ocv_early_cell_ = early_cell;
    }

    // Convenience: enable AOCV with variation percentages
    void enable_aocv(double late_variation = 0.10, double early_variation = 0.10) {
        ocv_mode_ = OcvMode::AOCV;
        aocv_table_.late_variation = late_variation;
        aocv_table_.early_variation = early_variation;
    }

    // --- CPPR Configuration ---
    void enable_cppr(bool enable = true) { cppr_.enabled = enable; }
    
    // --- POCV Configuration ---
    void set_pocv_table(const PocvTable& table) { pocv_table_ = table; }
    void enable_pocv(double n_sigma = 3.0, double default_sigma_pct = 0.05) {
        ocv_mode_ = OcvMode::POCV;
        pocv_table_.n_sigma = n_sigma;
        pocv_table_.default_sigma_pct = default_sigma_pct;
    }
    
    // --- PBA Configuration ---
    void enable_pba(bool enable = true) { pba_enabled_ = enable; }
    
    // --- Crosstalk Configuration ---
    void enable_crosstalk(const CrosstalkConfig& config) { xtalk_ = config; xtalk_.enabled = true; }
    void enable_crosstalk(double coupling_cap = 0.00015, double miller = 1.5) {
        xtalk_.enabled = true;
        xtalk_.coupling_cap_per_um = coupling_cap;
        xtalk_.miller_factor = miller;
    }

    // Get timing for a specific net
    const PinTiming& timing(NetId net) const { return pin_timing_.at(net); }

    // Run STA for a specific corner derate (used by MCMM)
    StaResult analyze_corner(double clock_period, int num_paths, const CornerDerate& d);

private:
    const Netlist& nl_;
    const LibertyLibrary* lib_;
    const PhysicalDesign* pd_;
    std::unordered_map<NetId, PinTiming> pin_timing_;
    std::vector<TimingArc> arcs_;
    std::vector<GateId> topo_;
    CornerDerate derate_;
    double setup_uncertainty_ = 0;
    double hold_uncertainty_ = 0;
    std::unordered_map<GateId, double> clock_insertion_; // per-DFF CTS delay

    // OCV/AOCV state
    OcvMode ocv_mode_ = OcvMode::NONE;
    AocvTable aocv_table_;
    double ocv_late_cell_ = 1.15;   // flat OCV late derate
    double ocv_early_cell_ = 0.85;  // flat OCV early derate
    std::unordered_map<GateId, int> gate_depth_;  // logic depth per gate (for AOCV)
    bool analyzing_late_ = true;    // true = late/setup analysis, false = early/hold

    // POCV state
    PocvTable pocv_table_;
    
    // CPPR state  
    CpprConfig cppr_;
    
    // PBA state
    bool pba_enabled_ = false;
    
    // Crosstalk state
    CrosstalkConfig xtalk_;

    // Core STA steps
    void build_timing_graph();
    void compute_gate_depths();     // compute logic depth for AOCV
    void forward_propagation(double input_arrival = 0);
    void hold_forward_propagation(double input_arrival = 0);
    void backward_propagation(double clock_period);
    void hold_backward_propagation();
    void compute_slacks();
    std::vector<TimingPath> extract_paths(int count, bool include_hold = true);

    // Delay calculation with slew + OCV/AOCV awareness
    double gate_delay(GateId gid, double input_slew = 0.01) const;
    double output_slew(GateId gid, double input_slew, double load_cap) const;
    double wire_delay(NetId from, NetId to) const;
    double net_load_cap(NetId nid) const;

    // OCV-aware derate factor for a gate
    double effective_cell_derate(GateId gid) const;
    double effective_wire_derate(GateId gid) const;

    // Industrial analysis passes
    void compute_cppr_credits(std::vector<TimingPath>& paths, double clock_period);
    void pba_reanalyze(std::vector<TimingPath>& paths, double clock_period);
    double compute_path_pocv_sigma(const TimingPath& path) const;
    double compute_crosstalk_delta(NetId net) const;
};

} // namespace sf
```

---

## FILE 2: src/timing/sta.cpp (EXCERPT - Key Functions)

See full file at `/home/paramsaini/Desktop/siliconforge/src/timing/sta.cpp`

Key sections:
- Lines 1-99: Effective derate functions (OCV/AOCV/POCV)
- Lines 101-228: Delay and slew calculation
- Lines 271-296: Timing graph construction
- Lines 300-360: Forward propagation (setup)
- Lines 364-415: Forward propagation (hold)
- Lines 419-468: Backward propagation (setup)
- Lines 472-498: Backward propagation (hold)
- Lines 513-588: Path extraction
- Lines 592-712: Single corner analysis
- Lines 728-759: Multi-corner analysis
- Lines 763-816: CPPR computation
- Lines 843-906: PBA re-analysis
- Lines 910-950: Crosstalk delta computation

---

## FILE 3: src/CMakeLists.txt (COMPLETE)

Location: `/home/paramsaini/Desktop/siliconforge/src/CMakeLists.txt`

To add ssta.cpp, insert this line after timing/sta.cpp at line 74:
```cmake
timing/ssta.cpp
```

Complete library definition (103 lines) organizes 70+ source files by subsystem.

---

## FILE 4: tests/CMakeLists.txt (PATTERN)

Location: `/home/paramsaini/Desktop/siliconforge/tests/CMakeLists.txt`

Repeats this pattern 38 times (test_phase1 through test_phase38):
```cmake
add_executable(test_phaseN test_phaseN.cpp)
target_link_libraries(test_phaseN PRIVATE siliconforge_lib)
add_test(NAME PhaseNTests COMMAND test_phaseN)
```

To add test_phase39, add after line 151:
```cmake
add_executable(test_phase39 test_phase39.cpp)
target_link_libraries(test_phase39 PRIVATE siliconforge_lib)
add_test(NAME Phase39Tests COMMAND test_phase39)
```

---

## FILE 5: tests/test_phase38.cpp (EXCERPT)

Location: `/home/paramsaini/Desktop/siliconforge/tests/test_phase38.cpp` (525 lines)

Test pattern with macros:
```cpp
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<...failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

TEST(config_defaults) {
    DrcConfig cfg;
    CHECK(cfg.temperature_c == 25.0, "default temp");
    PASS("config_defaults");
}

int main() {
    RUN(config_defaults);
    RUN(config_custom);
    // ... more tests
    std::cout << "\nPassed: " << passed << "\nFailed: " << failed << "\n";
    return (failed > 0) ? 1 : 0;
}
```

---

## FILE 6: src/core/netlist.hpp (KEY DEFINITIONS)

Location: `/home/paramsaini/Desktop/siliconforge/src/core/netlist.hpp`

Gate types:
```cpp
enum class GateType {
    INPUT, OUTPUT, BUF, NOT,
    AND, OR, NAND, NOR, XOR, XNOR,
    MUX, DFF, DLATCH, TRI, CONST0, CONST1,
    BUFIF0, BUFIF1, NOTIF0, NOTIF1
};

using NetId = int32_t;
using GateId = int32_t;
```

Netlist structures:
```cpp
struct Net {
    NetId id;
    std::string name;
    Logic4 value = Logic4::X;
    Logic4 next_value = Logic4::X;
    std::vector<GateId> fanout;  // gates driven by this net
    GateId driver = -1;          // gate that drives this net
};

struct Gate {
    GateId id;
    GateType type;
    std::string name;
    std::vector<NetId> inputs;   // input nets
    NetId output = -1;           // output net
    NetId clk = -1;              // for DFF: clock
    NetId reset = -1;            // for DFF: reset
    Logic4 init_val = Logic4::ZERO;
};

class Netlist {
public:
    NetId add_net(const std::string& name = "");
    GateId add_gate(GateType type, const std::vector<NetId>& inputs, 
                    NetId output, const std::string& name = "");
    GateId add_dff(NetId d, NetId clk, NetId q, NetId reset = -1,
                   const std::string& name = "");
    void mark_input(NetId id);
    void mark_output(NetId id);
    
    Net& net(NetId id) { return nets_[id]; }
    const Net& net(NetId id) const { return nets_[id]; }
    Gate& gate(GateId id) { return gates_[id]; }
    const Gate& gate(GateId id) const { return gates_[id]; }
    
    size_t num_nets() const { return nets_.size(); }
    size_t num_gates() const { return gates_.size(); }
    const std::vector<NetId>& primary_inputs() const { return pis_; }
    const std::vector<NetId>& primary_outputs() const { return pos_; }
    const std::vector<GateId>& flip_flops() const { return dffs_; }
    const std::vector<Net>& nets() const { return nets_; }
    const std::vector<Gate>& gates() const { return gates_; }
    
    std::vector<GateId> topo_order() const;
    static Logic4 eval_gate(GateType type, const std::vector<Logic4>& inputs);
    void print_stats() const;
    void clear();

private:
    std::vector<Net> nets_;
    std::vector<Gate> gates_;
    std::vector<NetId> pis_, pos_;
    std::vector<GateId> dffs_;
};
```

---

## FILE 7: src/core/liberty_parser.hpp (KEY DEFINITIONS)

Location: `/home/paramsaini/Desktop/siliconforge/src/core/liberty_parser.hpp`

Liberty structures:
```cpp
struct LibertyPin {
    std::string name;
    std::string direction;      // "input", "output", "inout"
    std::string function;       // Boolean function: "A & B", "!A"
    double capacitance = 0.0;   // input pin capacitance
    double max_transition = 0.0;
};

struct LibertyTiming {
    std::string related_pin;
    std::string timing_type;    // "combinational", "rising_edge", etc.
    double cell_rise = 0.0;
    double cell_fall = 0.0;
    double rise_transition = 0.0;
    double fall_transition = 0.0;
    
    struct NldmTable {
        std::vector<double> index_1;  // slew breakpoints
        std::vector<double> index_2;  // load breakpoints
        std::vector<std::vector<double>> values; // 2D [slew][load]
        bool valid() const;
        double interpolate(double slew, double load) const;
    };
    NldmTable nldm_rise;
    NldmTable nldm_fall;
    NldmTable nldm_rise_tr;
    NldmTable nldm_fall_tr;
};

struct LibertyCell {
    std::string name;
    double area = 0.0;
    double leakage_power = 0.0;
    std::vector<LibertyPin> pins;
    std::vector<LibertyTiming> timings;
    
    const LibertyPin* find_pin(const std::string& name) const;
    std::string output_function() const;
    int num_inputs() const;
};

class LibertyLibrary {
public:
    std::string name;
    std::string technology;
    double nom_voltage = 1.8;
    double nom_temperature = 25.0;
    std::string time_unit = "1ns";
    std::string cap_unit = "1pF";
    
    std::vector<LibertyCell> cells;
    
    bool parse(const std::string& filename);
    bool parse_string(const std::string& content);
    const LibertyCell* find_cell(const std::string& name) const;
    std::vector<const LibertyCell*> cells_by_function(const std::string& func) const;
    void print_stats() const;

private:
    // Parser internals
    struct Token { enum Type { ... }; };
    std::vector<Token> tokenize(const std::string& content);
    bool parse_tokens(const std::vector<Token>& tokens);
    // ... more parsing methods
};
```

