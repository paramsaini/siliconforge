# SiliconForge STA/SSTA Quick Reference

## Adding ssta.cpp in 3 Steps

### Step 1: Create Header File
Create `/home/paramsaini/Desktop/siliconforge/src/timing/ssta.hpp`

Must include:
```cpp
#pragma once
#include "timing/sta.hpp"
#include <vector>

namespace sf {

class SstaEngine {
public:
    SstaEngine(const Netlist& nl, const LibertyLibrary* lib = nullptr,
               const PhysicalDesign* pd = nullptr);
    
    // Statistical timing analysis methods
    StaResult analyze_statistical(double clock_period, int num_paths = 5);
    std::vector<StaResult> analyze_corners_with_distribution(double clock_period);
    
private:
    StaEngine sta_;  // Use existing StaEngine internally
    // ... statistical computation methods
};

} // namespace sf
```

### Step 2: Update src/CMakeLists.txt

Add this line at line 74 (in "Timing & Power" section):
```cmake
timing/ssta.cpp
```

Result:
```cmake
# ── Timing & Power ────────────────────────────────────────────
timing/sta.cpp
timing/ssta.cpp              # ← NEW LINE
timing/power.cpp
timing/parasitics.cpp
...
```

### Step 3: Add Test in tests/CMakeLists.txt

Add these 3 lines after line 151:
```cmake
add_executable(test_phase39 test_phase39.cpp)
target_link_libraries(test_phase39 PRIVATE siliconforge_lib)
add_test(NAME Phase39Tests COMMAND test_phase39)
```

---

## StaEngine Key Methods

### Constructor
```cpp
StaEngine sta(netlist, liberty_lib, physical_design);
// all parameters required, lib and pd can be nullptr
```

### Analysis
```cpp
// Single corner
StaResult result = sta.analyze(1.0);  // 1.0 ns clock period

// Multi-corner (best/typical/worst)
auto results = sta.analyze_multicorner(1.0);  // returns vector<3>
```

### Configuration
```cpp
// Clock uncertainty (jitter + skew)
sta.set_clock_uncertainty(0.05, 0.02);  // setup=0.05ns, hold=0.02ns

// Per-DFF clock insertion delay (from CTS)
sta.set_clock_insertion(dff_gate_id, 0.1);

// OCV modes
sta.enable_ocv(1.15, 0.85);  // flat: 15% late, 15% early
sta.enable_aocv(0.10, 0.10);  // depth-dependent: ±10% variation
sta.enable_pocv(3.0, 0.05);   // parametric: 3-sigma, 5% default

// Industrial passes
sta.enable_cppr();           // Common Path Pessimism Removal
sta.enable_pba();            // Path-Based Analysis
sta.enable_crosstalk();      // Coupling capacitance
```

### Results
```cpp
StaResult res = sta.analyze(1.0, 10);

// Timing metrics
res.wns  // Worst Negative Slack (ns) — setup
res.tns  // Total Negative Slack (ns)
res.hold_wns  // Hold WNS (ns)
res.num_violations  // setup violations
res.hold_violations  // hold violations

// Critical paths
for (auto& path : res.critical_paths) {
    path.slack          // path slack (ns)
    path.delay          // path delay (ns)
    path.startpoint     // source gate name
    path.endpoint       // destination gate/port name
    path.depth          // logic levels
    path.is_hold        // true if hold path
    path.nets           // NetId[] along path
    path.gates          // GateId[] along path
    path.cppr_credit    // CPPR pessimism removal (ns)
    path.pba_slack      // PBA-adjusted slack
    path.crosstalk_delta  // timing impact of coupling
    path.path_sigma     // POCV path standard deviation
}

// Industrial analysis results
res.message           // Status message
res.time_ms          // Analysis runtime
res.corner_name      // "typical", "best", "worst"
res.ocv_mode         // OcvMode enum
res.cppr_total_credit  // total CPPR credit across paths
res.pba_wns          // PBA-adjusted WNS
res.max_crosstalk_delta  // largest crosstalk effect
```

---

## Netlist Interface (from core/netlist.hpp)

### Build Netlist
```cpp
Netlist nl;

// Create nets
NetId in1 = nl.add_net("in1");
NetId in2 = nl.add_net("in2");
NetId out = nl.add_net("out");

// Mark I/O
nl.mark_input(in1);
nl.mark_input(in2);
nl.mark_output(out);

// Add gates
GateId and_gate = nl.add_gate(GateType::AND, {in1, in2}, out, "U1");
GateId dff = nl.add_dff(out, clk_net, q_net, reset_net, "ff1");
```

### Query Netlist
```cpp
// Access nets/gates (O(1))
Net& n = nl.net(net_id);
Gate& g = nl.gate(gate_id);

// Collections
const auto& pis = nl.primary_inputs();    // vector<NetId>
const auto& pos = nl.primary_outputs();   // vector<NetId>
const auto& dffs = nl.flip_flops();       // vector<GateId>

// Gate connectivity
g.inputs      // vector<NetId> — input nets
g.output      // NetId — output net (-1 if none)
n.fanout      // vector<GateId> — driven gates
n.driver      // GateId — driving gate (-1 if PI)

// DFF-specific
g.clk    // NetId — clock input
g.reset  // NetId — reset input
g.init_val  // Logic4 — initial value

// Topology
auto topo = nl.topo_order();  // vector<GateId> in topological order
```

---

## LibertyLibrary Interface (from core/liberty_parser.hpp)

### Parse & Lookup
```cpp
LibertyLibrary lib;
lib.parse("mylib.lib");  // Read from file
lib.parse_string(lib_content);  // From string

// Cell lookup
const LibertyCell* cell = lib.find_cell("AND2_X1");
if (!cell) cell = lib.find_cell("AND2");  // Try generic name

// Function lookup
auto cells = lib.cells_by_function("A & B");  // Find AND cells
```

### Cell Timing Data
```cpp
// Pins
for (auto& pin : cell->pins) {
    pin.name;        // "A", "B", "Z"
    pin.direction;   // "input" or "output"
    pin.capacitance; // fF (input pin capacitance)
    pin.function;    // "A & B" for output
}

// Timing arcs (combinational delays)
for (auto& timing : cell->timings) {
    timing.related_pin;    // "A" or "B" (input driving this arc)
    timing.timing_type;    // "combinational"
    timing.cell_rise;      // ns at nominal corner
    timing.cell_fall;      // ns
    timing.rise_transition;  // ns
    timing.fall_transition;  // ns
    
    // NLDM 2D tables [slew][load]
    if (timing.nldm_rise.valid()) {
        double delay = timing.nldm_rise.interpolate(0.01, 0.005);
        // delay = f(input_slew=0.01ns, output_load=0.005pF)
    }
}

// Cell properties
cell->name;          // "AND2_X1"
cell->area;          // um²
cell->leakage_power; // mW @ nom corner
```

---

## OCV Modes Explained

### NONE (default)
- No derating
- All corners use base 1.0x delay

### OCV (On-Chip Variation)
- Flat derating: 1.15x late (setup), 0.85x early (hold)
- Simple model, commonly used
- Usage: `sta.enable_ocv(1.15, 0.85);`

### AOCV (Advanced OCV)
- Depth-dependent: `1.0 ± variation/√depth`
- √N averaging: deeper paths = less variation
- More realistic than flat OCV
- Usage: `sta.enable_aocv(0.10, 0.10);`  // ±10% variation

### POCV (Parametric OCV)
- Per-cell sigma model
- Path-level RSS: σ_path = √(Σ σ_cell²)
- Fine-grained statistical analysis
- Usage: `sta.enable_pocv(3.0, 0.05);`  // 3-sigma, 5% default

---

## Test Pattern (test_phase39.cpp template)

```cpp
#include "core/netlist.hpp"
#include "timing/sta.hpp"
#include <iostream>
#include <cassert>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) \
    do { if(!(c)) { \
        std::cerr << "  [FAIL] " << m << " (line " << __LINE__ << ")\n"; \
        failed++; return; \
    }} while(0)
#define PASS(n) do { std::cout << "  [PASS] " << n << "\n"; passed++; } while(0)
#define RUN(n) do { std::cout << "Running: " #n "\n"; test_##n(); } while(0)

TEST(build_simple_netlist) {
    Netlist nl;
    
    // Create nets
    NetId in = nl.add_net("in");
    NetId out = nl.add_net("out");
    nl.mark_input(in);
    nl.mark_output(out);
    
    // Add buffer
    GateId buf = nl.add_gate(GateType::BUF, {in}, out, "U1");
    
    CHECK(nl.num_nets() == 2, "2 nets");
    CHECK(nl.num_gates() == 1, "1 gate");
    CHECK(nl.gate(buf).type == GateType::BUF, "BUF gate");
    PASS("build_simple_netlist");
}

TEST(sta_basic) {
    Netlist nl;
    NetId in = nl.add_net("in");
    NetId out = nl.add_net("out");
    nl.mark_input(in);
    nl.mark_output(out);
    nl.add_gate(GateType::BUF, {in}, out, "U1");
    
    StaEngine sta(nl);
    StaResult res = sta.analyze(1.0);  // 1.0 ns clock period
    
    CHECK(res.clock_period == 1.0, "clock period set");
    CHECK(res.corner_name == "typical", "typical corner");
    PASS("sta_basic");
}

TEST(sta_with_ocv) {
    Netlist nl;
    NetId in = nl.add_net("in");
    NetId out = nl.add_net("out");
    nl.mark_input(in);
    nl.mark_output(out);
    nl.add_gate(GateType::BUF, {in}, out, "U1");
    
    StaEngine sta(nl);
    sta.enable_ocv(1.15, 0.85);
    StaResult res = sta.analyze(1.0);
    
    CHECK(res.ocv_mode == OcvMode::OCV, "OCV enabled");
    PASS("sta_with_ocv");
}

int main() {
    RUN(build_simple_netlist);
    RUN(sta_basic);
    RUN(sta_with_ocv);
    
    std::cout << "\n=== Summary ===\n"
              << "Passed: " << passed << "\n"
              << "Failed: " << failed << "\n";
    return (failed > 0) ? 1 : 0;
}
```

---

## Build & Test

```bash
cd ~/Desktop/siliconforge
mkdir -p build
cd build
cmake ..
make

# Run specific test
./tests/test_phase39

# Run all tests
ctest

# Build with debug symbols
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

---

## Key Files Summary

| File | Purpose | Lines |
|------|---------|-------|
| `src/timing/sta.hpp` | STA engine header | 307 |
| `src/timing/sta.cpp` | STA implementation | 852 |
| `src/core/netlist.hpp` | Gate-level netlist | 111 |
| `src/core/liberty_parser.hpp` | Liberty cell library | 98 |
| `src/CMakeLists.txt` | Library build config | 113 |
| `tests/CMakeLists.txt` | Test build config | 152 |
| `tests/test_phase38.cpp` | Example test | 525 |

