================================================================================
                 SILICONFORGE PROJECT ANALYSIS - FILE INDEX
================================================================================

COMPREHENSIVE PROJECT ANALYSIS COMPLETED
Project Location: /home/paramsaini/Desktop/siliconforge/

This analysis provides complete API documentation, code patterns, and integration
guides for the SiliconForge codebase, with emphasis on DFT (Design-for-Test) tools.

================================================================================
GENERATED DOCUMENTATION FILES (on your Desktop)
================================================================================

1. **SILICONFORGE_ANALYSIS_SUMMARY.txt** (21 KB, 651 lines) ⭐ MAIN REFERENCE
   ├─ QUICK START section with all essential APIs
   ├─ DIRECTORY STRUCTURE - Complete file organization
   ├─ NETLIST API - Full Netlist class specification
   ├─ SCAN INSERTION API - ScanInserter, ScanConfig, ScanResult
   ├─ FAULT SIMULATOR API - FaultSimulator, FaultSimResult
   ├─ PODEM ATPG API - PodemAtpg, Fault, AtpgResult, FaultCoverage
   ├─ LOGIC4 & TYPES - 4-state logic, BitVector class
   ├─ BUILD CONFIGURATION - CMakeLists.txt breakdown
   ├─ INCLUDE PATTERNS - How to structure new files
   ├─ COMPLETE EXAMPLE - Creating new DFT tool
   ├─ TESTING TEMPLATE - How to write tests
   ├─ MEMORY MANAGEMENT - Vector growth patterns
   ├─ GATE EVALUATION - Truth tables and semantics
   └─ INTEGRATION CHECKLIST - Step-by-step guide

2. **QUICK_INTEGRATION_GUIDE.md** (8 KB)
   ├─ Core data structures with code examples
   ├─ Creating new DFT tools (header + implementation templates)
   ├─ CMakeLists.txt update instructions
   ├─ Essential APIs with usage examples
   ├─ Gate types and logic values
   ├─ Common patterns (iteration, MUX, topological sort)
   ├─ Directory structure overview
   ├─ Build commands
   ├─ File locations reference table
   └─ Memory management rules

3. **README_ANALYSIS.txt** (This file)
   └─ Index and navigation guide

================================================================================
KEY FINDINGS: PROJECT STRUCTURE
================================================================================

PROJECT ROOT: /home/paramsaini/Desktop/siliconforge/

DIRECTORY STRUCTURE:
  src/
    ├── core/           (8 files)  → netlist, types, aig, hierarchy
    ├── dft/            (3 files)  → scan_insert, fault_sim, podem ⭐
    ├── frontend/       (3 files)  → verilog_parser, sdc_parser, sva_parser
    ├── synth/          (9 files)  → behavioral_synth, tech_mapper, etc.
    ├── formal/         (7 files)  → equiv_checker, lec, bmc, k_induction
    ├── sim/            (1 file)   → event-driven simulator
    ├── sat/            (1 file)   → CDCL SAT solver
    ├── timing/         (8 files)  → sta, power, parasitics, mcmm
    ├── pnr/            (12 files) → placer, router, floorplan, gdsii
    ├── verify/         (6 files)  → drc, lvs, cdc, antenna
    ├── macro/          (1 file)   → memory_compiler
    ├── ml/             (2 files)  → congestion_model, timing_model
    ├── hls/            (1 file)   → c_parser
    ├── viz/            (5 files)  → visualization, dashboard
    ├── flow/           (1 file)   → flow engine
    └── shell/          (2 files)  → tcl_interp, sf_shell

  tests/                (28 test files) → test_phase1.cpp to test_phase28.cpp
  studio/               (Verilog examples)
  CMakeLists.txt        (Root build config)

BUILD OUTPUT:
  - Library: libsiliconforge_lib.a (static, 105+ source files)
  - Executable: siliconforge (main.cpp)

================================================================================
KEY APIS (Quick Reference)
================================================================================

NETLIST (core/netlist.hpp) - 111 lines
  Classes: Netlist
  Structs: Net, Gate
  Enums: GateType (18 values: INPUT, OUTPUT, BUF, NOT, AND, OR, XOR, MUX, DFF, ...)
  Key Methods:
    - add_net(), add_gate(), add_dff()
    - mark_input(), mark_output()
    - flip_flops(), primary_inputs(), primary_outputs()
    - gate(), net(), topo_order()
    - eval_gate() [static]

LOGIC TYPES (core/types.hpp) - 113 lines
  Enum: Logic4 (ZERO, ONE, X, Z)
  Functions: logic_not(), logic_and(), logic_or(), logic_xor()
  Class: BitVector

SCAN INSERTION (dft/scan_insert.hpp) - 37 lines
  Classes: ScanInserter
  Structs: ScanConfig (max_chain_length, scan_in/out prefixes, scan_enable_name)
  Structs: ScanResult (num_chains, total_ffs, chain_lengths, message)

FAULT SIMULATOR (dft/fault_sim.hpp) - 42 lines
  Classes: FaultSimulator
  Structs: FaultSimResult (total_faults, detected, coverage_pct())

PODEM ATPG (dft/podem.hpp) - 79 lines
  Classes: PodemAtpg
  Structs: Fault, AtpgResult, FaultCoverage
  Algorithms: Recursive PODEM with D-frontier heuristics

================================================================================
FILE SPECIFICATIONS
================================================================================

BUILD CONFIG: src/CMakeLists.txt (113 lines)
  - Creates static library: siliconforge_lib
  - Lists 105+ source .cpp files (NO glob patterns, explicit listing)
  - All files organized by module (core, dft, frontend, synth, etc.)
  - Include path: src/ (allows #include "dft/scan_insert.hpp")
  - Executable: siliconforge linked against library

DFT MODULE STRUCTURE: src/dft/
  ├── scan_insert.hpp/cpp      (37 + 71 lines)
  │   └─ Scan chain insertion with MUX insertion and chaining
  ├── fault_sim.hpp/cpp        (42 + 50+ lines)
  │   └─ Bit-parallel fault simulation, 64 patterns at once
  └── podem.hpp/cpp            (79 + 50+ lines)
      └─ PODEM ATPG with D-algorithm, max 200 recursion depth

HEADER PATTERNS:
  1. #pragma once
  2. // SiliconForge — Component description
  3. #include "core/netlist.hpp" (required dependency)
  4. #include <vector>, <string>, etc. (STL headers)
  5. namespace sf { ... }

IMPLEMENTATION PATTERNS:
  1. #include "dft/component.hpp" (own header first)
  2. #include <algorithm>, <iostream>, etc.
  3. namespace sf { ... }
  4. All code in sf namespace

================================================================================
CENTRAL DATA STRUCTURES
================================================================================

NETLIST CLASS:
  Contains:
    - std::vector<Net> nets_              (all signals, indexed by NetId)
    - std::vector<Gate> gates_            (all gates, indexed by GateId)
    - std::vector<NetId> pis_             (primary inputs)
    - std::vector<NetId> pos_             (primary outputs)
    - std::vector<GateId> dffs_           (flip-flop gates)

NET STRUCT:
    - NetId id                           (unique ID)
    - std::string name                   (signal name)
    - Logic4 value, next_value           (4-state logic)
    - std::vector<GateId> fanout         (driven gates)
    - GateId driver                      (driving gate, -1 if PI)

GATE STRUCT:
    - GateId id                          (unique ID)
    - GateType type                      (AND, OR, DFF, MUX, etc.)
    - std::string name                   (instance name)
    - std::vector<NetId> inputs          (input nets, order matters!)
    - NetId output                       (output net)
    - NetId clk, reset                   (DFF-specific)
    - Logic4 init_val                    (DFF-specific)

MUX INPUT ORDER:
    inputs[0] = select signal
    inputs[1] = data when select=1
    inputs[2] = data when select=0

DFF STRUCTURE:
    inputs[0] = D input
    output = Q output
    clk = clock net
    reset = reset net (-1 if unused)

================================================================================
INTEGRATION STEPS FOR NEW DFT TOOLS
================================================================================

STEP 1: Create Header (src/dft/my_tool.hpp)
  ✓ #pragma once
  ✓ Comments with description
  ✓ #include "core/netlist.hpp"
  ✓ MyToolConfig struct with defaults
  ✓ MyToolResult struct
  ✓ MyTool class with:
    - explicit MyTool(Netlist& nl)
    - Non-owning reference member: Netlist& nl_
    - Public run/insert method

STEP 2: Create Implementation (src/dft/my_tool.cpp)
  ✓ #include "dft/my_tool.hpp" (own header first)
  ✓ Standard library includes
  ✓ namespace sf { ... }
  ✓ Access netlist: nl_.flip_flops(), nl_.gates()
  ✓ Cache vector contents before modifications
  ✓ Use nl_.add_net(), nl_.add_gate() for creation

STEP 3: Update Build Config (src/CMakeLists.txt)
  ✓ Add "dft/my_tool.cpp" to DFT section

STEP 4: Rebuild
  ✓ cd ~/Desktop/siliconforge/build
  ✓ cmake ..
  ✓ make

STEP 5: Test (optional)
  ✓ Create tests/test_my_tool.cpp
  ✓ Build: make test_my_tool
  ✓ Run: ./tests/test_my_tool

================================================================================
CRITICAL PATTERN: VECTOR SAFETY
================================================================================

⚠️ C++ vectors invalidate references when reallocating!

WRONG:
    for (auto& gid : nl.flip_flops()) {
        nl.add_gate(...);  // Modifies gates_ vector!
        Gate& g = nl.gate(gid);  // DANGLING REFERENCE if vector grew!
    }

CORRECT:
    // Cache IDs before modification
    std::vector<GateId> ff_ids(nl.flip_flops().begin(), nl.flip_flops().end());
    
    for (GateId gid : ff_ids) {
        // Safe to modify netlist
        nl.add_gate(...);
        
        // Re-fetch after modification
        Gate& g = nl.gate(gid);  // Still valid
    }

This pattern is used in scan_insert.cpp (lines 22-23) and shown in documentation.

================================================================================
ESSENTIAL CODE PATTERNS
================================================================================

1. ITERATE FLIP-FLOPS:
   for (GateId dff_id : nl.flip_flops()) {
       Gate& g = nl.gate(dff_id);
       NetId d = g.inputs[0];
       NetId q = g.output;
       NetId clk = g.clk;
   }

2. ITERATE ALL GATES:
   for (size_t i = 0; i < nl.num_gates(); ++i) {
       Gate& g = nl.gate(i);
       if (g.type == GateType::DFF) { ... }
   }

3. ADD GATE:
   nl.add_gate(GateType::AND, {in1, in2}, out, "AND_0");

4. ADD MUX (order matters!):
   nl.add_gate(GateType::MUX, {select, data1, data0}, out, "MUX_0");

5. ADD DFF:
   nl.add_dff(d, clk, q, reset, "DFF_0");

6. TOPOLOGICAL SORT:
   auto order = nl.topo_order();  // Returns std::vector<GateId>

7. EVALUATE GATE:
   Logic4 result = Netlist::eval_gate(GateType::AND, {ONE, ZERO});

================================================================================
TESTING
================================================================================

Test Template Pattern (tests/test_phase_X.cpp):

#include "core/netlist.hpp"
#include "dft/scan_insert.hpp"
#include <cassert>

int main() {
    sf::Netlist nl;
    
    // Build test circuit
    auto in = nl.add_net("in");
    auto clk = nl.add_net("clk");
    auto q = nl.add_net("q");
    nl.mark_input(in);
    nl.mark_input(clk);
    nl.mark_output(q);
    nl.add_dff(in, clk, q);
    
    // Test tool
    sf::ScanInserter si(nl);
    auto result = si.insert();
    
    // Verify
    assert(result.num_chains > 0);
    return 0;
}

Build & Run:
  cd build && cmake .. && make
  ./tests/test_phase1

Existing Tests: 28 phases (test_phase1.cpp through test_phase28.cpp)

================================================================================
GATE TYPES & SEMANTICS
================================================================================

COMBINATIONAL GATES (18 types):

INPUT, OUTPUT             I/O markers (not real gates)
BUF                       Pass-through
NOT                       Inversion
AND, OR, NAND, NOR        Multi-input (fanin varies)
XOR, XNOR                 2-input logic
MUX                       3-input multiplexer (select, data1, data0)
TRI                       Tri-state buffer
CONST0, CONST1            Constant values
BUFIF0, BUFIF1            Conditional buffers
NOTIF0, NOTIF1            Conditional inverters

SEQUENTIAL GATES:
DFF                       D flip-flop (positive-edge triggered)
DLATCH                    D latch (level-triggered)

LOGIC VALUES (Logic4):
ZERO (0)                  Logic 0
ONE (1)                   Logic 1
X (2)                     Unknown
Z (3)                     High-impedance

TRUTH TABLES (IEEE 1364-2005):
AND:   0&X=0, 1&1=1, X&X=X (0-dominance)
OR:    1|X=1, 0|0=0, X|X=X (1-dominance)
XOR:   All X with unknown inputs
MUX:   sel=1→data1, sel=0→data0, sel=X→X

================================================================================
HOW TO USE THESE DOCUMENTS
================================================================================

Quick Reference:
  → Read: QUICK_INTEGRATION_GUIDE.md (5 minutes)
  → Start creating new DFT tools immediately

Detailed Learning:
  → Read: SILICONFORGE_ANALYSIS_SUMMARY.txt (30 minutes)
  → Understand all APIs, patterns, and conventions

During Development:
  → Refer to QUICK_INTEGRATION_GUIDE.md for code templates
  → Check SILICONFORGE_ANALYSIS_SUMMARY.txt for detailed APIs
  → Look at src/dft/*.hpp for actual class/struct definitions

Creating New Tools:
  1. Copy template from QUICK_INTEGRATION_GUIDE.md section 2
  2. Implement using patterns from SILICONFORGE_ANALYSIS_SUMMARY.txt
  3. Update CMakeLists.txt (see section 4 in summary)
  4. Build and test

================================================================================
SUMMARY STATISTICS
================================================================================

Project Size:
  - 26 modules in src/
  - 105+ source files (.cpp)
  - 100+ header files (.hpp)
  - 28 test files
  - ~100,000+ lines of C++ code

DFT Module (Focus):
  - 3 files: scan_insert, fault_sim, podem
  - 266 lines of headers
  - 150+ lines of implementation (visible in this analysis)

Documentation Generated:
  - SILICONFORGE_ANALYSIS_SUMMARY.txt: 651 lines, 21 KB
  - QUICK_INTEGRATION_GUIDE.md: ~300 lines, 8 KB
  - README_ANALYSIS.txt: ~400 lines (this file)

================================================================================
NEXT STEPS
================================================================================

1. Read QUICK_INTEGRATION_GUIDE.md to understand basic concepts (5 min)
2. Review SILICONFORGE_ANALYSIS_SUMMARY.txt for deep understanding (30 min)
3. Look at existing DFT tools:
   - src/dft/scan_insert.hpp/cpp
   - src/dft/fault_sim.hpp/cpp
   - src/dft/podem.hpp/cpp
4. Create new tool following templates
5. Add .cpp to src/CMakeLists.txt
6. Rebuild: cd build && cmake .. && make
7. Write tests following templates
8. Verify integration

================================================================================
CONTACT & REFERENCE
================================================================================

Project Root: /home/paramsaini/Desktop/siliconforge/
Documentation: ~/Desktop/ (current directory with analysis files)
Build Directory: ~/Desktop/siliconforge/build/
Source Root: ~/Desktop/siliconforge/src/

Files Created:
  ✓ SILICONFORGE_ANALYSIS_SUMMARY.txt (Main Reference)
  ✓ QUICK_INTEGRATION_GUIDE.md (Quick Start)
  ✓ README_ANALYSIS.txt (This Index)

All code: namespace sf { ... }
All headers: #pragma once

================================================================================
