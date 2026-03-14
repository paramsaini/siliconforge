# SiliconForge Project Analysis

## 1. DIRECTORY STRUCTURE

```
siliconforge/
├── CMakeLists.txt (root)
├── src/
│   ├── CMakeLists.txt
│   ├── core/        (netlist, types, design_db, parsers)
│   ├── frontend/    (Verilog, SDC, SVA, UPF parsers)
│   ├── synth/       (synthesis optimizations)
│   ├── formal/      (BMC, LEC, SVA, equivalence checking)
│   ├── sim/         (event-driven simulator)
│   ├── lint/        (lint engine)
│   ├── dft/         (DFT: scan, JTAG, fault sim, PODEM)
│   ├── hls/         (C-to-RTL)
│   ├── macro/       (memory compiler)
│   ├── pnr/         (place & route: floorplan, place, route, CTS, P/R opt)
│   ├── timing/      (STA, power, parasitics, PDN, noise, signal integrity)
│   ├── verify/      (DRC, LVS, CDC, antenna, OPC, reliability)
│   ├── ml/          (timing/congestion models)
│   ├── viz/         (visualization: dashboard, RTL, logic)
│   ├── flow/        (main flow engine)
│   ├── shell/       (Tcl shell)
│   └── sat/         (SAT solver: CDCL)
└── tests/
    ├── CMakeLists.txt
    └── test_phase{1..38}.cpp
```

## 2. ROOT CMakeLists.txt

**Location:** `/home/paramsaini/Desktop/siliconforge/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20)
project(SiliconForge VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

add_subdirectory(src)

enable_testing()
add_subdirectory(tests)
```

## 3. src/CMakeLists.txt

**Location:** `/home/paramsaini/Desktop/siliconforge/src/CMakeLists.txt`

Creates a static library `siliconforge_lib` with ALL source files organized by category.

**To add ssta.cpp:** Add this line in the "Timing & Power" section (after timing/sta.cpp):
```cmake
timing/ssta.cpp    # ← Add this line
```

Full "Timing & Power" section (lines 73-82):
```cmake
# ── Timing & Power ────────────────────────────────────────────
timing/sta.cpp
timing/ssta.cpp              # ← ADD THIS LINE
timing/power.cpp
timing/parasitics.cpp
timing/ir_drop.cpp
timing/pdn.cpp
timing/noise.cpp
timing/signal_integrity.cpp
timing/electromigration.cpp
timing/mcmm.cpp
```

**Full library definition:**
```cmake
add_library(siliconforge_lib STATIC
    # ── Core ──────────────────────────────────────────────────────────
    core/aig.cpp
    core/netlist.cpp
    core/design_db.cpp
    core/hierarchy.cpp
    core/die_to_die.cpp
    core/liberty_parser.cpp
    core/def_parser.cpp
    core/lef_parser.cpp
    core/report_gen.cpp

    # ── Frontend Parsers ──────────────────────────────────────────────
    frontend/verilog_parser.cpp
    frontend/sdc_parser.cpp
    frontend/sva_parser.cpp
    frontend/upf_parser.cpp

    # ── Synthesis ─────────────────────────────────────────────────────
    synth/aig_opt.cpp
    synth/behavioral_synth.cpp
    synth/clock_gating.cpp
    synth/eco.cpp
    synth/fsm_extract.cpp
    synth/multi_vt.cpp
    synth/resource_share.cpp
    synth/retiming.cpp
    synth/tech_mapper.cpp

    # ── SAT Solver ────────────────────────────────────────────────────
    sat/cdcl_solver.cpp

    # ── Formal Verification ───────────────────────────────────────────
    formal/tseitin.cpp
    formal/equiv_checker.cpp
    formal/bmc.cpp
    formal/k_induction.cpp
    formal/lec.cpp
    formal/sva_engine.cpp

    # ── Simulation ────────────────────────────────────────────────────
    sim/simulator.cpp

    # ── Lint ──────────────────────────────────────────────────────────
    lint/lint_engine.cpp

    # ── DFT ───────────────────────────────────────────────────────────
    dft/podem.cpp
    dft/fault_sim.cpp
    dft/scan_insert.cpp
    dft/jtag_bist.cpp

    # ── HLS ───────────────────────────────────────────────────────────
    hls/c_parser.cpp

    # ── Macro Generation ──────────────────────────────────────────────
    macro/memory_compiler.cpp

    # ── Place & Route ─────────────────────────────────────────────────
    pnr/physical.cpp
    pnr/floorplan.cpp
    pnr/placer.cpp
    pnr/global_router.cpp
    pnr/detailed_router.cpp
    pnr/detailed_router_v2.cpp
    pnr/cts.cpp
    pnr/post_route_opt.cpp
    pnr/gdsii_writer.cpp
    pnr/chip_assembler.cpp
    pnr/tsv_mgr.cpp
    pnr/ai_tuner.cpp

    # ── Timing & Power ────────────────────────────────────────────────
    timing/sta.cpp
    timing/ssta.cpp              # ← ADD THIS LINE
    timing/power.cpp
    timing/parasitics.cpp
    timing/ir_drop.cpp
    timing/pdn.cpp
    timing/noise.cpp
    timing/signal_integrity.cpp
    timing/electromigration.cpp
    timing/mcmm.cpp

    # ── Physical Verification ─────────────────────────────────────────
    verify/drc.cpp
    verify/lvs.cpp
    verify/cdc.cpp
    verify/antenna.cpp
    verify/opc.cpp
    verify/reliability.cpp

    # ── ML Models ─────────────────────────────────────────────────────
    ml/congestion_model.cpp
    ml/timing_model.cpp

    # ── Visualization ─────────────────────────────────────────────────
    viz/dashboard.cpp
    viz/html_export.cpp
    viz/logic_viz.cpp
    viz/rtl_viz.cpp
    viz/sim_viz.cpp

    # ── Flow Engine & Shell ───────────────────────────────────────────
    flow/engine.cpp
    shell/sf_shell.cpp
    shell/tcl_interp.cpp
)

target_include_directories(siliconforge_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(siliconforge main.cpp)
target_link_libraries(siliconforge PRIVATE siliconforge_lib)
```

## 4. tests/CMakeLists.txt

**Location:** `/home/paramsaini/Desktop/siliconforge/tests/CMakeLists.txt`

**Pattern for adding test_phase39:**
Add these 3 lines after the test_phase38 entry (lines 149-151):
```cmake
add_executable(test_phase39 test_phase39.cpp)
target_link_libraries(test_phase39 PRIVATE siliconforge_lib)
add_test(NAME Phase39Tests COMMAND test_phase39)
```

The pattern is:
- `add_executable(test_phaseN test_phaseN.cpp)` - creates executable
- `target_link_libraries(test_phaseN PRIVATE siliconforge_lib)` - links with library
- `add_test(NAME PhaseNTests COMMAND test_phaseN)` - registers test with CMake

Each test from test_phase1 through test_phase38 follows this identical pattern.

## 5. src/timing/sta.hpp — FULL CONTENTS

**Location:** `/home/paramsaini/Desktop/siliconforge/src/timing/sta.hpp`

See the complete file for:
- **Enums:** `OcvMode` (NONE, OCV, AOCV, POCV)
- **Structs:**
  - `AocvTable` - AOCV depth-dependent derating (uses √N statistical averaging)
  - `PocvTable` - Parametric OCV with per-cell sigma
  - `CpprConfig` - Common Path Pessimism Removal configuration
  - `CrosstalkConfig` - Coupling capacitance modeling
  - `CornerDerate` - Multi-corner derating factors (late/early for cell/wire)
  - `TimingArc` - Gate-level timing arc (from net, to net, delay, slew, gate)
  - `PinTiming` - Complete timing info (arrival, required, slack, slew, hold)
  - `TimingPath` - Critical path representation
  - `StaResult` - Analysis output (WNS, TNS, violations, critical paths)

- **StaEngine class:**
  - **Constructor:** `StaEngine(const Netlist& nl, const LibertyLibrary* lib = nullptr, const PhysicalDesign* pd = nullptr)`
  - **Main analysis methods:**
    - `StaResult analyze(double clock_period, int num_paths = 5)` - single corner
    - `std::vector<StaResult> analyze_multicorner(double clock_period, int num_paths = 5)` - 3 corners
    - `StaResult analyze_corner(double clock_period, int num_paths, const CornerDerate& d)` - configurable corner
  - **Configuration methods:**
    - `set_clock_uncertainty(double setup_unc, double hold_unc)`
    - `set_clock_insertion(GateId dff_id, double delay)`
    - `set_ocv_mode(OcvMode mode)` / `enable_ocv()` / `enable_aocv()` / `enable_pocv()`
    - `enable_cppr()` / `set_pocv_table()` / `enable_pba()` / `enable_crosstalk()`
  - **Accessor:** `const PinTiming& timing(NetId net) const`

- **Namespaces:** `sf`
- **Key includes:**
  - `core/netlist.hpp` (gate-level circuit)
  - `core/liberty_parser.hpp` (cell timing/power models)
  - `pnr/physical.hpp` (wire routing and physical info)
  - Standard library: `<vector>`, `<unordered_map>`, `<limits>`, `<cmath>`

---

## 6. src/timing/sta.cpp — KEY SECTIONS

**Location:** `/home/paramsaini/Desktop/siliconforge/src/timing/sta.cpp` (852 lines)

### Core Private Methods:

1. **`effective_cell_derate(GateId gid)` (lines 15-39)**
   - Returns OCV/AOCV/POCV-aware derate factor for late/early analysis
   - AOCV: uses √N depth scaling
   - OCV: flat 1.15 (late) / 0.85 (early)
   - POCV: base 1.0, path-level adjustment in PBA

2. **`effective_wire_derate(GateId gid)` (lines 41-64)**
   - Depth-aware wire derate (AOCV) or flat (OCV)

3. **`compute_gate_depths()` (lines 68-97)**
   - Computes logic depth from PIs/DFFs for AOCV/POCV
   - Uses BFS through topological order

4. **`gate_delay(GateId gid, double input_slew)` (lines 101-150)**
   - Slew-dependent gate delay lookup from Liberty
   - Falls back to gate-type defaults if no Liberty model
   - Applies effective_cell_derate

5. **`output_slew(GateId gid, double input_slew, double load_cap)` (lines 152-195)**
   - Computes output slew from input slew and load capacitance

6. **`net_load_cap(NetId nid)` (lines 197-228)**
   - Sums fanout gate capacitances (from Liberty or default)

7. **`wire_delay(NetId from, NetId to)` (lines 230-267)**
   - Calculates wire RC delay from physical design (if available)
   - Uses Elmore delay model: R·C·L²/2

8. **`build_timing_graph()` (lines 271-296)**
   - Creates timing arcs for all gates
   - Calls compute_gate_depths() for AOCV

9. **`forward_propagation(double input_arrival)` (lines 300-360)**
   - LATE path propagation (for setup analysis)
   - Initializes PI and DFF outputs
   - Propagates maximum arrival times through combinational logic
   - Computes slew at each net

10. **`hold_forward_propagation(double input_arrival)` (lines 364-415)**
    - EARLY path propagation (for hold analysis)
    - Uses analyzing_late_ = false to switch to early derates
    - Propagates minimum arrival times

11. **`backward_propagation(double clock_period)` (lines 419-468)**
    - Setup analysis: requires at clock_period
    - Uses capture clock's EARLY derate (pessimistic for setup)
    - Back-propagates requirements through combinational logic

12. **`hold_backward_propagation()` (lines 472-498)**
    - Hold analysis: requires at hold_time + hold_uncertainty_
    - Uses capture clock's LATE derate (pessimistic for hold)

13. **`compute_slacks()` (lines 502-509)**
    - `slack = required - arrival`

14. **`extract_paths(int count, bool include_hold)` (lines 513-588)**
    - Extracts worst N critical paths (setup + hold)
    - Backtracks from endpoints to startpoints
    - Records path delay, slack, depth

15. **`analyze_corner(...)` (lines 592-712)** - Main per-corner analysis:
    - Builds timing graph
    - Forward/backward propagation
    - Slack computation
    - Path extraction
    - Industrial STA passes: CPPR, PBA, Crosstalk
    - Returns StaResult

16. **`analyze(double clock_period, int num_paths)` (lines 716-724)**
    - Simple wrapper for typical corner analysis

17. **`analyze_multicorner(double clock_period, int num_paths)` (lines 728-759)**
    - Analyzes 3 corners: best (0.85x), typical (1.0x), worst (1.25x)
    - Returns per-corner results

### Industrial STA Passes:

18. **`compute_cppr_credits(...)` (lines 763-816)**
    - Removes common clock path pessimism
    - Credit = common_delay × |late_derate - early_derate|
    - Only when OCV/AOCV/POCV enabled

19. **`compute_path_pocv_sigma(const TimingPath&)` (lines 820-839)**
    - POCV: Computes path-level σ using RSS: √(Σ σ_i²)

20. **`pba_reanalyze(...)` (lines 843-906)**
    - Path-Based Analysis: re-analyzes critical paths with specific slew
    - Adds POCV path sigma to delay
    - Computes PBA-adjusted slack

21. **`compute_crosstalk_delta(NetId net)` (lines 910-950)**
    - Coupling-capacitance based delay delta
    - Uses miller_factor for same/opposite direction
    - Spatial coupling model

---

## 7. src/core/netlist.hpp — KEY DEFINITIONS

**Location:** `/home/paramsaini/Desktop/siliconforge/src/core/netlist.hpp`

### Enums:
```cpp
enum class GateType {
    INPUT, OUTPUT, BUF, NOT,
    AND, OR, NAND, NOR, XOR, XNOR,
    MUX, DFF, DLATCH, TRI, CONST0, CONST1,
    BUFIF0, BUFIF1, NOTIF0, NOTIF1
};
```

### Type Aliases:
```cpp
using NetId = int32_t;
using GateId = int32_t;
```

### Structures:
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
    // For DFF:
    NetId clk = -1;
    NetId reset = -1;
    Logic4 init_val = Logic4::ZERO;
};
```

### Netlist Class (Public Interface):
```cpp
class Netlist {
public:
    // Construction
    NetId add_net(const std::string& name = "");
    GateId add_gate(GateType type, const std::vector<NetId>& inputs, 
                    NetId output, const std::string& name = "");
    GateId add_dff(NetId d, NetId clk, NetId q, NetId reset = -1,
                   const std::string& name = "");
    
    // Marking
    void mark_input(NetId id);
    void mark_output(NetId id);
    
    // Access (O(1) lookup)
    Net& net(NetId id) { return nets_[id]; }
    const Net& net(NetId id) const { return nets_[id]; }
    Gate& gate(GateId id) { return gates_[id]; }
    const Gate& gate(GateId id) const { return gates_[id]; }
    
    // Collections
    size_t num_nets() const { return nets_.size(); }
    size_t num_gates() const { return gates_.size(); }
    const std::vector<NetId>& primary_inputs() const { return pis_; }
    const std::vector<NetId>& primary_outputs() const { return pos_; }
    const std::vector<GateId>& flip_flops() const { return dffs_; }
    const std::vector<Net>& nets() const { return nets_; }
    const std::vector<Gate>& gates() const { return gates_; }
    
    // Analysis
    std::vector<GateId> topo_order() const;  // topological sort
    
    // Evaluation
    static Logic4 eval_gate(GateType type, const std::vector<Logic4>& inputs);
    
    // Utilities
    void print_stats() const;
    void clear();

private:
    std::vector<Net> nets_;
    std::vector<Gate> gates_;
    std::vector<NetId> pis_, pos_;
    std::vector<GateId> dffs_;
};
```

**Key methods for STA:**
- `net(NetId id).driver` — gate driving this net
- `net(NetId id).fanout` — gates driven by this net
- `gate(GateId id).inputs` — input nets to gate
- `gate(GateId id).output` — output net of gate
- `gate(GateId id).type` — gate type (AND, DFF, etc.)
- `gate(GateId id).clk/reset` — clock/reset for DFFs
- `primary_inputs()` / `primary_outputs()` / `flip_flops()` — collections
- `topo_order()` — combinational gate topological order

---

## 8. src/core/liberty_parser.hpp — KEY DEFINITIONS

**Location:** `/home/paramsaini/Desktop/siliconforge/src/core/liberty_parser.hpp`

### Structures:
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
    
    // NLDM 2D lookup tables: [slew][load]
    struct NldmTable {
        std::vector<double> index_1;  // slew breakpoints
        std::vector<double> index_2;  // load breakpoints
        std::vector<std::vector<double>> values; // 2D table
        bool valid() const;
        double interpolate(double slew, double load) const;
    };
    NldmTable nldm_rise;        // cell_rise NLDM table
    NldmTable nldm_fall;        // cell_fall NLDM table
    NldmTable nldm_rise_tr;     // rise_transition NLDM table
    NldmTable nldm_fall_tr;     // fall_transition NLDM table
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
```

### LibertyLibrary Class:
```cpp
class LibertyLibrary {
public:
    std::string name;
    std::string technology;
    double nom_voltage = 1.8;
    double nom_temperature = 25.0;
    std::string time_unit = "1ns";
    std::string cap_unit = "1pF";
    
    std::vector<LibertyCell> cells;
    
    // Parsing
    bool parse(const std::string& filename);
    bool parse_string(const std::string& content);
    
    // Lookup
    const LibertyCell* find_cell(const std::string& name) const;
    std::vector<const LibertyCell*> cells_by_function(const std::string& func) const;
    
    void print_stats() const;
    
private:
    // Token types and parser
    struct Token { enum Type { ... }; Type type; std::string value; };
    std::vector<Token> tokenize(const std::string& content);
    bool parse_tokens(const std::vector<Token>& tokens);
    // ... more parsing internals
};
```

**Key methods for STA:**
- `find_cell(const std::string& name)` — lookup cell by name (AND2, BUF, AND2_X1, etc.)
- `cell->timings` — vector of timing arcs
- `timing.nldm_rise.interpolate(slew, load)` — 2D table lookup
- `cell->pins[i].capacitance` — input pin load capacitance

---

## 9. tests/test_phase38.cpp — PATTERN EXAMPLE

**Location:** `/home/paramsaini/Desktop/siliconforge/tests/test_phase38.cpp` (525 lines)

### Header & Utilities:
```cpp
#include "core/types.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "verify/drc.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)
```

### Test Structure:
```cpp
// Helper function to build test data
static PhysicalDesign build_wire_design() {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    return pd;
}

// Individual test (uses TEST macro)
TEST(config_defaults) {
    DrcConfig cfg;
    CHECK(cfg.temperature_c == 25.0, "default temp");
    CHECK(cfg.em_current_limit_ma == 1.0, "default EM current");
    PASS("config_defaults");
}

TEST(em_width_clean) {
    PhysicalDesign pd = build_wire_design();
    pd.wires.push_back({0, {10,10}, {50,10}, 0.5, 0});
    DrcEngine eng(pd);
    DrcRule rule;
    rule.name = "M1.EM.W";
    rule.description = "EM min width";
    rule.value = 0.3;
    rule.type = DrcRule::EM_MIN_WIDTH;
    rule.layer = 0;
    rule.aux_value = 1.0;
    eng.add_rule(rule);
    auto r = eng.check();
    CHECK(r.em_width_violations == 0, "no EM violations for wide wire");
    CHECK(r.violations == 0, "clean");
    PASS("em_width_clean");
}

// Main function
int main() {
    RUN(config_defaults);
    RUN(config_custom);
    RUN(em_width_clean);
    RUN(em_width_violation);
    // ... more tests
    std::cout << "\n=== Summary ===\n"
              << "Passed: " << passed << "\n"
              << "Failed: " << failed << "\n";
    return (failed > 0) ? 1 : 0;
}
```

### Test Pattern:
1. Create test data structure (Netlist, PhysicalDesign, etc.)
2. Create engine/analyzer object
3. Configure engine with rules/options
4. Run analysis/check
5. Use CHECK() macro to validate results
6. Call PASS() on success or return early on failure

---

## 10. SUMMARY FOR ADDING ssta.cpp

### What You Need to Create:

**File:** `/home/paramsaini/Desktop/siliconforge/src/timing/ssta.hpp`
- Class declaration: `class SstaEngine`
- Should inherit from or use `StaEngine`
- Add statistical/parametric timing methods
- Include POCV/corner distribution support

**File:** `/home/paramsaini/Desktop/siliconforge/src/timing/ssta.cpp`
- Implement statistical STA methods
- Extend corner analysis with distribution
- Add path-level sigma computation using POCV tables
- Implement confidence interval computation

### CMakeLists.txt Updates:

**1. src/CMakeLists.txt (line 74):**
```cmake
# ── Timing & Power ────────────────────────────────────────────
timing/sta.cpp
timing/ssta.cpp         # ← ADD THIS LINE
timing/power.cpp
```

**2. tests/CMakeLists.txt (after line 151):**
```cmake
add_executable(test_phase39 test_phase39.cpp)
target_link_libraries(test_phase39 PRIVATE siliconforge_lib)
add_test(NAME Phase39Tests COMMAND test_phase39)
```

### StaEngine Interface You'll Use:

```cpp
// From sta.hpp
StaEngine sta(netlist, liberty_lib, physical_design);
sta.enable_pocv(n_sigma, default_sigma_pct);
sta.set_ocv_mode(OcvMode::POCV);
StaResult result = sta.analyze(clock_period, num_paths);
```

### Pattern for test_phase39.cpp:

Follow test_phase38.cpp pattern:
- Include headers for STA/SSTA
- Use TEST/CHECK/PASS macros
- Build small netlist with DFFs and combinational logic
- Instantiate SstaEngine
- Configure statistical parameters
- Run analysis and validate results
- Return pass/fail count from main

---

## 11. KEY DESIGN PATTERNS IN SiliconForge

### Netlist Connectivity Model:
```
Gate → (inputs) → [Net] → (fanout) → [Gate]
       (output) → [Net] → (driver) ← [Gate]
```

### Timing Analysis Flow:
1. **Build timing graph:** Gates → timing arcs
2. **Forward pass (LATE):** Compute maximum arrival times, slew
3. **Forward pass (EARLY):** Compute minimum arrival times
4. **Backward pass (setup):** Required times from clock_period
5. **Backward pass (hold):** Required times from hold margin
6. **Slack:** required - arrival
7. **Path extraction:** Worst N paths from endpoints to startpoints

### OCV Derating Strategy:
- **OCV:** Flat multiplier (1.15 late, 0.85 early)
- **AOCV:** Depth-dependent: `1.0 ± variation/√depth`
- **POCV:** Per-cell sigma, path-level RSS, ±N_sigma adjustment
- **Clock path:** Separate derate for capture clock in setup/hold

### Key Parameters:
- `clock_insertion_[dff_id]` — CTS delay to DFF clock
- `gate_depth_[gid]` — Logic levels from PI/DFF (for AOCV)
- `setup_uncertainty_`, `hold_uncertainty_` — clock jitter + skew
- `pin_timing_[net]` — arrival, required, slack, slew for each net

