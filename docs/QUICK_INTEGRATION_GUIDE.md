# SiliconForge DFT Integration Guide

**Location**: `/home/paramsaini/Desktop/siliconforge/`

## 1. Core Data Structures (netlist.hpp)

```cpp
// Create a circuit
Netlist nl;

// Create signals
NetId sig = nl.add_net("signal_name");

// Add gates (combinational)
nl.add_gate(GateType::AND, {in1, in2}, out, "AND_0");
nl.add_gate(GateType::MUX, {select, data1, data0}, mux_out);  // MUX

// Add flip-flops (sequential)
nl.add_dff(d_input, clock, q_output, reset_net, "DFF_0");

// Mark I/O
nl.mark_input(sig);
nl.mark_output(sig);

// Query
auto& dffs = nl.flip_flops();           // std::vector<GateId>
auto& pis = nl.primary_inputs();        // std::vector<NetId>
auto& gates = nl.gates();               // std::vector<Gate>
Gate& g = nl.gate(gate_id);
std::vector<GateId> order = nl.topo_order();  // Topological sort
```

## 2. Creating New DFT Tools

### Header Template (src/dft/my_tool.hpp)

```cpp
#pragma once
// SiliconForge — My DFT Tool
// Brief description

#include "core/netlist.hpp"
#include <vector>
#include <string>

namespace sf {

struct MyToolConfig {
    int param1 = 100;
    std::string param2 = "default";
};

struct MyToolResult {
    int count = 0;
    std::string message;
};

class MyTool {
public:
    explicit MyTool(Netlist& nl) : nl_(nl) {}
    
    MyToolResult run(const MyToolConfig& config = {});

private:
    Netlist& nl_;
};

} // namespace sf
```

### Implementation Template (src/dft/my_tool.cpp)

```cpp
#include "dft/my_tool.hpp"
#include <algorithm>
#include <iostream>

namespace sf {

MyToolResult MyTool::run(const MyToolConfig& config) {
    MyToolResult result;
    
    // Access flip-flops
    for (GateId dff_id : nl_.flip_flops()) {
        Gate& g = nl_.gate(dff_id);
        NetId d_in = g.inputs[0];
        NetId q_out = g.output;
        NetId clk = g.clk;
        // ... process DFF ...
    }
    
    // Iterate all gates
    for (const auto& g : nl_.gates()) {
        if (g.type == GateType::DFF) {
            // Handle DFF
        }
    }
    
    // Add new gates
    NetId new_sig = nl_.add_net("new_signal");
    nl_.add_gate(GateType::BUF, {new_sig}, nl_.add_net(), "BUF_NEW");
    
    result.count = nl_.num_gates();
    result.message = "Success";
    return result;
}

} // namespace sf
```

### Update CMakeLists.txt

In `src/CMakeLists.txt`, add to the DFT section:

```cmake
# ── DFT ───────────────────────────────────────────────────────
dft/podem.cpp
dft/fault_sim.cpp
dft/scan_insert.cpp
dft/my_tool.cpp                         # <-- Add this
```

### Rebuild

```bash
cd ~/Desktop/siliconforge/build
cmake ..
make
```

## 3. Essential APIs

### Scan Insertion (dft/scan_insert.hpp)

```cpp
#include "dft/scan_insert.hpp"

Netlist nl;
// ... build circuit ...

ScanConfig cfg;
cfg.max_chain_length = 100;

ScanInserter inserter(nl);
ScanResult result = inserter.insert(cfg);

std::cout << "Chains: " << result.num_chains << std::endl;
std::cout << "FFs: " << result.total_ffs << std::endl;
```

### Fault Simulation (dft/fault_sim.hpp)

```cpp
#include "dft/fault_sim.hpp"

FaultSimulator sim(nl);

std::vector<std::vector<Logic4>> tests = {
    {Logic4::ONE, Logic4::ZERO, ...},
    {Logic4::ZERO, Logic4::ONE, ...},
};

FaultSimResult result = sim.simulate(tests);
std::cout << "Coverage: " << result.coverage_pct() << "%" << std::endl;
```

### ATPG (dft/podem.hpp)

```cpp
#include "dft/podem.hpp"

PodemAtpg atpg(nl);

// Single fault test
Fault f{net_id, Logic4::ZERO};  // Stuck-at-0
AtpgResult test = atpg.generate_test(f);
if (test.detected) {
    for (auto [net, val] : test.test_vector) {
        std::cout << "Set net " << net << " to " << (int)val << std::endl;
    }
}

// Full ATPG
FaultCoverage coverage = atpg.run_full_atpg();
std::cout << "Detected: " << coverage.detected << "/" 
          << coverage.total_faults << std::endl;
```

## 4. Gate Types & Logic Values

### GateType Enum

```cpp
enum class GateType {
    INPUT, OUTPUT,           // I/O markers
    BUF, NOT,                // Basic
    AND, OR, NAND, NOR,      // Multi-input
    XOR, XNOR,               // 2-input
    MUX,                     // 3-input: {select, data1, data0}
    DFF, DLATCH,             // Sequential
    TRI, CONST0, CONST1,     // Special
    BUFIF0, BUFIF1,          // Tristate
    NOTIF0, NOTIF1
};
```

### Logic4 (4-State Logic)

```cpp
enum class Logic4 : uint8_t {
    ZERO = 0,   // Logic 0
    ONE  = 1,   // Logic 1
    X    = 2,   // Unknown
    Z    = 3    // High-Z
};

// Operations
Logic4 result = logic_and(Logic4::ONE, Logic4::ZERO);    // → ZERO
Logic4 result = logic_or(Logic4::ZERO, Logic4::X);       // → X
Logic4 result = logic_xor(Logic4::ONE, Logic4::ONE);     // → ZERO
```

## 5. Common Patterns

### Iterate Flip-Flops (Safe Pattern)

```cpp
// Cache IDs before modification (vectors may grow!)
std::vector<GateId> ff_ids(nl.flip_flops().begin(), nl.flip_flops().end());

for (GateId gid : ff_ids) {
    Gate& g = nl.gate(gid);
    
    // Safe to add gates/nets here
    nl.add_net("new_sig");
    nl.add_gate(...);  // Vector may grow, but ff_ids is cached
    
    // Re-fetch after add
    Gate& g2 = nl.gate(gid);  // Still valid
}
```

### Add MUX Gate

```cpp
// MUX: select between data1 (select=1) and data0 (select=0)
NetId select = /* ... */;
NetId data1 = /* ... */;
NetId data0 = /* ... */;
NetId output = nl.add_net("mux_out");

GateId mux = nl.add_gate(GateType::MUX, 
    {select, data1, data0},  // Order matters!
    output,
    "MUX_0");
```

### Topological Sort

```cpp
std::vector<GateId> order = nl.topo_order();
for (GateId gid : order) {
    Gate& g = nl.gate(gid);
    // Gates in evaluation order (excludes DFFs)
}
```

## 6. Directory Structure

```
src/
  core/              netlist.hpp, types.hpp, aig, hierarchy, ...
  dft/               scan_insert, fault_sim, podem
  frontend/          verilog_parser, sdc_parser, sva_parser
  synth/             behavioral_synth, tech_mapper, aig_opt, ...
  formal/            equiv_checker, lec, bmc, k_induction, ...
  sim/               event-driven simulator
  sat/               CDCL SAT solver
  timing/            sta, power, parasitics, mcmm, ...
  pnr/               placer, router, floorplan, gdsii_writer, ...
  verify/            drc, lvs, cdc, antenna, ...
  ...                (26 modules total)

tests/             test_phase1.cpp to test_phase28.cpp
studio/            Verilog test designs
```

## 7. Build Commands

```bash
# Build project
cd ~/Desktop/siliconforge/build
cmake ..
make

# Build specific test
make test_phase1

# Run test
./tests/test_phase1

# View available targets
make help | grep test
```

## 8. File Locations Reference

| Component | Path |
|-----------|------|
| Netlist API | `src/core/netlist.hpp` (111 lines) |
| Logic Types | `src/core/types.hpp` (113 lines) |
| Scan Insertion | `src/dft/scan_insert.hpp` (37 lines) |
| Fault Simulator | `src/dft/fault_sim.hpp` (42 lines) |
| ATPG | `src/dft/podem.hpp` (79 lines) |
| Build Config | `src/CMakeLists.txt` (113 lines) |
| Tests | `tests/test_phase*.cpp` (28 tests) |

## 9. Key Macros & Constants

### PODEM Constants
```cpp
static constexpr int MAX_RECURSION_DEPTH = 200;
static constexpr int MAX_BACKTRACKS = 5000;
```

### Scan Insertion Defaults
```cpp
struct ScanConfig {
    int max_chain_length = 100;
    std::string scan_in_prefix = "scan_in";
    std::string scan_out_prefix = "scan_out";
    std::string scan_enable_name = "scan_enable";
};
```

## 10. Memory Management Rule

⚠️ **Always cache vector contents before modifying netlist:**

```cpp
// Cache before iteration
std::vector<GateId> ids = std::vector<GateId>(
    nl.flip_flops().begin(), 
    nl.flip_flops().end()
);

// Now safe to modify
for (GateId id : ids) {
    nl.add_gate(...);  // Won't invalidate ids
}
```

**Why**: C++ vectors reallocate on growth, invalidating references/iterators.

---

**For complete documentation**, see:
- `/home/paramsaini/Desktop/SILICONFORGE_ANALYSIS_SUMMARY.txt` (651 lines)
- Project source: `/home/paramsaini/Desktop/siliconforge/src/`

