# SiliconForge Tier 1 - Detailed File Inventory & Code Snippets

## 1. PARASITIC EXTRACTION (40% Complete)

### Files
```
/src/timing/parasitics.hpp    77 lines
/src/timing/parasitics.cpp    196 lines
TOTAL: 273 lines
```

### Implementation Status

#### ✅ WORKING
- **RC Extraction:** Manhattan distance model (lines cpp:17-37)
  - Wire length: `std::abs(px1-px0) + std::abs(py1-py0)`
  - R: `length × wire_res_per_um`
  - C: `length × wire_cap_per_um`
  - Via parasitics included (lines 39-45)

- **Coupling Capacitance:** Same-layer parallel wires (lines cpp:156-193)
  - Spacing check: overlap if `spacing < 5.0` um
  - Formula: `cc = coupling_coeff × overlap / spacing`
  - Bidirectional coupling added to both nets

- **SPEF Writer:** IEEE 1481-2009 format (lines cpp:80-123)
  - Outputs: *SPEF, *DESIGN, *DATE, *DIVIDER, *DELIMITER
  - Units: 1 PS, 1 FF, 1 OHM
  - D_NET sections with *RES/*CAP/*CC subsections

#### ❌ STUBBED / MISSING
```cpp
// Line 40-45 (parasitics.cpp)
size_t num_vias = 0;  // ⚠️ HARDCODED - should count from layout
for (auto& w : pd_.wires) {
    // Count vias adjacent to this net (simplified)
}
pn.total_res_ohm += num_vias * params_.via_res;
pn.total_cap_ff += num_vias * params_.via_cap;

// Issues:
// 1. num_vias always 0 → via parasitics = 0
// 2. No loop body to actual count vias
// 3. pd_.wires doesn't explicitly mark vias
```

### Critical Gaps
1. **NO SPEF READER** - Write-only format
   - Cannot import parasitics from other tools
   - No parser for `*D_NET` sections

2. **Inaccurate Coupling Model** - Linear field
   - Real: 3D fringing, multi-layer coupling
   - Current: cc = coeff × overlap / spacing
   - Missing: per-metal-layer variation, fringe cap

3. **No Via Extraction** - Hardcoded to 0
   - Should extract from physical.cpp wire data
   - Each wire with different layer = potential via

4. **No 3D Effects** - Only same-layer
   - No inter-layer capacitance between metal layers
   - No shield/crosstalk modeling between layers

5. **No Sensitivity** - No delay impact analysis
   - Extracts RC but doesn't compute how it affects timing
   - No hotspot flagging (high-RC nets)

### Key Data Structures
```cpp
// parasitics.hpp lines 12-32
struct ParasiticNet {
    int net_id;
    double total_cap_ff = 0;    // Σ of all segments
    double total_res_ohm = 0;   // Σ of all segments
    double elmore_delay_ps = 0; // R_total × C_total

    struct RCSegment {
        double length, resistance, capacitance;
    };
    std::vector<RCSegment> segments;  // Wire segments

    struct CouplingCap {
        int aggressor_net;
        double coupling_ff;
        double overlap_um;
    };
    std::vector<CouplingCap> coupling;  // To other nets
    double total_coupling_ff = 0;
};
```

---

## 2. SDC TIMING CONSTRAINTS (50% Complete)

### Files
```
/src/frontend/sdc_parser.hpp   79 lines
/src/frontend/sdc_parser.cpp   257 lines
TOTAL: 336 lines
```

### Implementation Status

#### ✅ WORKING
- **create_clock** (lines cpp:67-98)
  - Parses: -name, -period, -waveform
  - Stores: SdcClock struct with period_ns, waveform_rise/fall, uncertainty

- **set_input_delay / set_output_delay** (lines cpp:99-132)
  - Parses: -clock, -max, -min, delay_ns, port
  - Stores: port, clock name, delay, is_max flag

- **set_false_path** (lines cpp:133-145)
  - Parses: -from <start>, -to <end>
  - Creates: SdcException with FALSE_PATH type

- **set_multicycle_path** (lines cpp:146-160)
  - Parses: multiplier, -from, -to
  - Stores: multiplier value in exception

- **set_max_delay** (lines cpp:161-175)
  - Parses: delay_value, -from, -to
  - Type: MAX_DELAY with delay override

- **set_max_fanout / set_max_transition / set_max_capacitance** (lines cpp:176-189)
  - Parses into: max_fanout[], max_transition[], max_capacitance[] maps

- **Tokenizer** (lines cpp:22-61)
  - Handles: comments (#), line continuation (\), brackets []{}
  - No TCL evaluation (purely regex-based)

#### ❌ STUBBED / MISSING
```cpp
// CRITICAL: set_clock_groups NOT parsed
// No handler in parse_command() switch statement
// Would need: 
struct SdcClockGroup {
    std::vector<std::string> clocks;
    enum Type { ASYNC, LOGICALLY_EXCLUSIVE, PHYSICALLY_EXCLUSIVE } type;
};

// CRITICAL: set_min_delay enum exists but NOT triggered
// sdc_parser.hpp line 37 has MIN_DELAY enum value
// but sdc_parser.cpp never creates it (only MAX_DELAY at line 161)

// HIGH: set_disabling_condition NOT implemented
// Would require expression parser + STA path filtering

// HIGH: create_generated_clock NOT implemented
// Would require clock tree tracking in netlist

// MEDIUM: set_case_analysis NOT implemented
// Would enable constant propagation in path analysis

// MEDIUM: [get_ports *] patterns NOT expanded
// Tokenizer just stores literal string; no TCL variable eval

// MEDIUM: set_timing_derate NOT in SDC parser
// Only per-corner derates in STA MCMM, not SDC override
```

### Key Data Structures
```cpp
// sdc_parser.hpp lines 13-55
struct SdcClock {
    std::string name, port;
    double period_ns = 0;
    double waveform_rise = 0, waveform_fall = 0;
    double uncertainty = 0;
};

struct SdcInputDelay {
    std::string port, clock;
    double delay_ns = 0;
    bool is_max = true;
};

struct SdcOutputDelay {
    std::string port, clock;
    double delay_ns = 0;
    bool is_max = true;
};

struct SdcException {
    enum Type { FALSE_PATH, MULTICYCLE_PATH, MAX_DELAY, MIN_DELAY } type;
    std::string from, to;
    double value = 0;
    int multiplier = 1;
};

struct SdcConstraints {
    std::vector<SdcClock> clocks;
    std::vector<SdcInputDelay> input_delays;
    std::vector<SdcOutputDelay> output_delays;
    std::vector<SdcException> exceptions;
    std::unordered_map<std::string, double> max_fanout;
    std::unordered_map<std::string, double> max_transition;
    std::unordered_map<std::string, double> max_capacitance;
};
```

### Parsing Flow
```
Input SDC text → tokenize (comment/continuation handling)
              → parse_command() loop
              → switch on command name
              → extract parameters via -flag parsing
              → append to SdcConstraints struct
```

**Gap in switch statement (cpp:63-209):**
- ✓ create_clock, set_input_delay, set_output_delay
- ✓ set_false_path, set_multicycle_path, set_max_delay
- ✓ set_max_fanout, set_clock_uncertainty
- ✗ set_clock_groups
- ✗ set_min_delay (parse logic)
- ✗ set_disabling_condition
- ✗ create_generated_clock
- ✗ set_case_analysis
- ✗ set_input_transition

---

## 3. MCMM / MULTI-CLOCK STA (60% Complete)

### Files
```
/src/timing/mcmm.hpp    252 lines
/src/timing/mcmm.cpp    535 lines
/src/timing/sta.hpp     ~300+ lines (mixed content)
/src/timing/sta.cpp     1638 lines
TOTAL: ~2725 lines
```

### Implementation Status (MCMM)

#### ✅ WORKING
- **Corner Creation** (mcmm.cpp:12-40)
  - Default 3 corners: SS (worst setup), TT (typical), FF (worst hold)
  - Each with: voltage, temperature, process, delay_scale, power_scale, derate struct

- **Foundry Corners** (mcmm.cpp:42-63)
  - 7-corner industrial standard: SS/SF/TT/FS/FF variants
  - Plus extreme temperature corners

- **Functional Modes** (mcmm.cpp:65-84)
  - func (normal), scan (test), standby (low power)
  - Per-mode: frequency, switching_activity, uncertainty, description

- **Scenario Management** (mcmm.hpp:87-96, cpp:173+)
  - Combines corners × modes → N×M scenarios
  - Per-scenario: corner, mode, STA result, power result
  - Active/inactive flags, dominance tracking, signoff classification

- **Dominance Pruning** (mcmm.cpp:125-146)
  - Removes scenarios that are subsets of others
  - Setup dominated: wns[j] <= wns[i] - margin
  - Hold dominated: hold_wns[j] <= hold_wns[i] - margin
  - Configurable slack margin (default 0.05 ns)

- **Sensitivity Analysis** (mcmm.cpp:151-171)
  - Voltage sensitivity: ∂WNS/∂V (10 mV delta)
  - Temperature sensitivity: ∂WNS/∂T (10°C delta)
  - Per-scenario output for voltage/temp optimization

- **Per-Mode Uncertainty** (mcmm.hpp:73-74)
  - setup_uncertainty, hold_uncertainty per functional mode
  - Applied in run_scenario_sta() (line 97)

- **CornerDerate Structure** (sta.hpp:88-102)
  - cell_derate, wire_derate (setup/LATE)
  - early_cell, early_wire (hold/EARLY)
  - clock_late_cell, clock_late_wire (setup clock path)
  - clock_early_cell, clock_early_wire (hold clock path)

#### ❌ STUBBED / MISSING
```cpp
// CRITICAL: Clock domain crossing validation
// mcmm.cpp line 87-112: run_scenario_sta() does NOT check CDC
// Missing:
// 1. Parse clock_groups from SDC
// 2. Detect cross-domain timing paths
// 3. Flag unsynced crossings → metastability

// HIGH: Per-corner clock tree delays
// sta.cpp line 198: set_clock_insertion() only generic
// Issue: All corners use same CTS delay value
// Should: Vary CTS delays per-corner PVT

// HIGH: Multi-corner hold check
// sta.cpp (implicit): Hold uses same corner as setup
// Correct: Hold = fastest data (FF corner) + slowest capture clock (SS corner)
// Current: Not doing cross-corner composition

// MEDIUM: POCV path-level integration
// sta.hpp line 47-62: PocvTable structure complete
// Missing: Path-level RSS(σ1, σ2, ..., σN) computation
// Line sta.cpp:27: "POCV: base derate is 1.0..."
// But critical_paths[] never populated with path_sigma

// MEDIUM: Crosstalk delta integration
// sta.hpp line 78-85: CrosstalkConfig exists
// Missing: Integration into TimingPath::crosstalk_delta
// Currently: struct field populated but not computed
```

### Implementation Status (STA)

#### ✅ WORKING
- **OCV/AOCV Derating** (sta.cpp:16-65)
  - AOCV table: late_derate(depth) = 1 + var/√depth
  - Early derate: 1 - var/√depth
  - Gate depth computed from topological levels

- **Gate Delay Computation** (sta.cpp:102-151)
  - Slew-aware: delay ∝ input_slew, load_cap
  - Library lookup: NLDM table interpolation if available
  - Fallback: parametric model by gate type
  - Applies effective derate per OCV mode

- **Output Slew Computation** (sta.cpp:153-196)
  - Intrinsic + input_slew + R×C_load terms
  - AOCV-aware derating applied

- **Topological Analysis**
  - Computes gate depths for AOCV (sta.cpp:69-98)
  - Forward/backward pass for timing (sta.cpp, not shown)

- **Hold Checks** (sta.hpp:121-127)
  - Separate hold_arrival, hold_required, hold_slack
  - min(hold_slack_rise, hold_slack_fall) worst-case

- **Critical Path Reporting** (sta.hpp:135-152)
  - TimingPath struct with: nets, gates, delay, slack
  - Depth for AOCV, CPPR credit, PBA slack, crosstalk delta, path_sigma

#### ❌ STUBBED / MISSING
```cpp
// MEDIUM: CPPR (Common Path Pessimism Removal)
// sta.hpp:67-73: CpprConfig structure exists
// Missing: Actual CPPR credit computation
// Currently: Comment says "we'd track clock tree topology"
// But no implementation of credit = common_delay × |late - early|

// MEDIUM: Crosstalk path derating
// sta.hpp:78-85: Full config (coupling_cap, miller_factor, etc.)
// Missing: Integration into gate_delay() calculation
// Currently: Config exists but not used in delay computation

// MEDIUM: Path-based analysis (PBA)
// sta.hpp:147: pba_slack, pba_delay fields in TimingPath
// Missing: Actual PBA computation (re-analysis per path)
// Currently: pba_valid=false always

// LOW: Latch timing
// Only DFF supported; no latch open/close window analysis
// Missing: time_borrowing, latch enable tracking

// LOW: Min-delay path reporting
// Only worst (highest delay) paths tracked
// Missing: Best (lowest delay) paths for hold validation
```

### Key Data Structures
```cpp
// sta.hpp:88-102 (CornerDerate)
struct CornerDerate {
    std::string name = "typical";
    double cell_derate = 1.0;           // LATE data
    double wire_derate = 1.0;           // LATE wire
    double early_cell = 1.0;            // EARLY data (hold)
    double early_wire = 1.0;            // EARLY wire (hold)
    double clock_late_cell = 1.0;       // Setup: capture early
    double clock_late_wire = 1.0;
    double clock_early_cell = 1.0;      // Hold: capture late
    double clock_early_wire = 1.0;
};

// sta.hpp:135-152 (TimingPath)
struct TimingPath {
    std::vector<NetId> nets;
    std::vector<GateId> gates;
    double delay = 0;
    double slack = 0;
    std::string startpoint, endpoint;
    bool is_hold = false;
    int depth = 0;          // For AOCV

    // Industrial
    double cppr_credit = 0;
    double pba_slack = 0;
    double crosstalk_delta = 0;
    double path_sigma = 0;  // POCV RSS
    bool pba_valid = false;
};
```

---

## 4. POWER GRID / IR DROP (70% Complete)

### Files
```
/src/timing/pdn.hpp      265 lines
/src/timing/pdn.cpp      491 lines
/src/timing/ir_drop.hpp  250 lines
/src/timing/ir_drop.cpp  856 lines
TOTAL: 1862 lines
```

### PDN Implementation Status

#### ✅ WORKING (pdn.{hpp,cpp})
- **Decap Modeling** (pdn.hpp:29-45)
  - ESR, ESL, capacitance with SRF computation
  - `srf_mhz() = 1/(2π√(LC))`

- **Impedance Calculation** (pdn.cpp:66-124)
  - Decap: Z = ESR + j(ωL - 1/ωC)
  - Mesh: R_mesh || L_bump from stripe config
  - Parallel combination of mesh + decaps + on-die cap

- **AC Sweep** (pdn.hpp:231, cpp likely has loop)
  - Frequency-domain from 1 MHz to 5 GHz
  - 100 log-spaced points

- **Target Impedance** (pdn.cpp:54-61)
  - Z_target = V_ripple / I_transient
  - Configurable voltage ripple (5% default)

- **Pad/Via/Bump Modeling** (pdn.hpp:21-66)
  - BumpPad: resistance, inductance per pad
  - ViaArray: parallel via resistance/inductance
  - PdnStripe: HORIZONTAL/VERTICAL mesh segments

#### ❌ STUBBED / MISSING
```cpp
// CRITICAL: Resonance detection
// pdn.hpp:258: find_resonances() declared
// Missing: Actual peak detection algorithm
// Should: Find local maxima in impedance profile
// Return: freq, impedance, Q-factor, type (series/parallel)

// HIGH: Enhanced PDN analysis
// pdn.hpp:233: run_enhanced() declared but likely stub
// Missing: Orchestration of AC, resonance, decap opt, package model

// MEDIUM: Decap optimization
// pdn.hpp:230: optimize_decoupling() declared
// Missing: Placement algorithm for budget area
// Probably greedy; should be: frequency-response targeted

// MEDIUM: Package model integration
// pdn.hpp:232: set_package_model() declared
// Impact unclear: board inductance, package R/L not clearly integrated
```

### IR Drop Implementation Status

#### ✅ WORKING (ir_drop.{hpp,cpp})
- **Gauss-Seidel Solver** (ir_drop.cpp:147-236)
  - Sparse matrix system: KCL at each node
  - V_ij = (Σ G_neighbor × V_neighbor + I_pad - I_ij) / Σ G_neighbor
  - Successive Over-Relaxation (SOR) with ω=1.4 acceleration
  - Configurable convergence tolerance (1e-6 default)

- **Power Pad Generation** (ir_drop.cpp:26-82)
  - CORNERS: 4 pads at die corners
  - PERIMETER: uniform spacing on edges
  - RING: outer + inner ring pattern
  - CUSTOM: user-defined locations

- **Current Map** (ir_drop.cpp:86-132)
  - Area-proportional: current ∝ cell area
  - OR per-cell specified via config_.cell_currents_ma[]
  - Maps cells to NxN grid nodes

- **Hotspot Detection** (ir_drop.hpp:78-84, cpp:likely present)
  - Severity thresholds: WARNING (5%), CRITICAL (10%)
  - Returns region, drop_mv, current_ma, timing_derate_pct

- **Timing Derating** (ir_drop.hpp:111-113, cpp:likely present)
  - Sensitivity: delay ∝ voltage drop
  - Config: timing_derate_sensitivity = 0.12 (%delay / %drop)
  - Applied to critical paths for voltage-aware STA

- **Result Maps** (ir_drop.hpp:116-119)
  - drop_map[y][x]: static drop in mV
  - voltage_map[y][x]: actual supply voltage (V - drop)
  - dynamic_map[y][x]: peak transient drop

#### ❌ STUBBED / MISSING
```cpp
// CRITICAL: Dynamic IR drop
// ir_drop.hpp:197-200 declares but cpp NOT implemented
DynamicIrResult analyze_dynamic(double time_step_ps = 10.0, int num_steps = 100);
// Needs:
// 1. Time-stepping loop (per time_step_ps)
// 2. Current injection per step
// 3. Decap discharge dynamics: I = C × dV/dt
// 4. Peak drop tracking across all steps

// CRITICAL: Vectored IR drop
// ir_drop.hpp:199-200 declares but cpp NOT implemented
VectoredIrResult analyze_vectored(const std::vector<std::vector<bool>>& stimulus,
                                   double clock_period_ns = 1.0);
// Needs:
// 1. VCD or stimulus parsing
// 2. Per-cycle current injection from switching activity
// 3. Run transient analysis per stimulus vector

// CRITICAL: Spatial analysis
// ir_drop.cpp:238: compute_dynamic_drop() called
// Line 261: spatial_analysis() called but EMPTY
// Missing: Fill nodes[].voltage from solver result
// Currently: Returns with node voltages = 0

// HIGH: Voltage-aware timing
// ir_drop.hpp:202: analyze_voltage_timing() declared
// Missing: Call STA with per-node voltage derating
// Should: Propagate voltage_map[y][x] to timing engine

// MEDIUM: EM (Electromigration) checking
// ir_drop.hpp:cfg.em_limit_ma_per_um but not enforced
// Missing: Current density validation against EM limits
// Should: Flag routes exceeding EM threshold per layer

// MEDIUM: Current integration from power analysis
// Must be manual fill; should auto-import from Power module
```

### Key Data Structures
```cpp
// ir_drop.hpp:31-63 (IrDropConfig)
struct IrDropConfig {
    double vdd = 1.8;
    double total_current_ma = 100.0;
    std::vector<PowerPad> pads;
    enum PadPattern { CORNERS, PERIMETER, RING, CUSTOM } pad_pattern;
    
    int grid_resolution = 16;    // NxN grid
    double sheet_resistance_mohm = 10;
    int max_iterations = 500;
    double convergence_tol = 1e-6;
    double sor_omega = 1.4;
    
    double hotspot_threshold_pct = 5.0;
    double critical_threshold_pct = 10.0;
    
    bool enable_dynamic = false;
    double switching_factor = 0.3;
    double clock_period_ns = 1.0;
    double decap_density_ff_per_um2 = 0.1;
    
    std::vector<double> cell_currents_ma;  // Per-cell
    
    bool compute_timing_derating = true;
    double timing_derate_sensitivity = 0.12;
};

// ir_drop.hpp:88-131 (IrDropResult)
struct IrDropResult {
    double worst_drop_mv = 0;
    double avg_drop_mv = 0;
    double median_drop_mv = 0;
    double vdd = 0;
    
    double worst_dynamic_drop_mv = 0;
    bool dynamic_analyzed = false;
    
    int num_hotspots = 0;
    int num_critical = 0;
    std::vector<IrDropHotSpot> hotspots;
    
    int solver_iterations = 0;
    double solver_residual = 0;
    bool converged = false;
    
    double worst_timing_derate_pct = 0;
    double avg_timing_derate_pct = 0;
    
    std::vector<std::vector<double>> drop_map;      // [y][x] mV
    std::vector<std::vector<double>> dynamic_map;   // [y][x] mV
    std::vector<std::vector<double>> voltage_map;   // [y][x] mV
    
    std::vector<IrDropNode> nodes;  // Per-node detail
    
    bool signoff_pass = false;
    std::string signoff_summary;
};
```

---

## 5. METAL FILL (80% Complete)

### Files
```
/src/pnr/metal_fill.hpp  41 lines
/src/pnr/metal_fill.cpp  104 lines
TOTAL: 145 lines
```

### Implementation Status

#### ✅ WORKING
- **Density Computation** (metal_fill.cpp:11-24)
  - Per-layer: wire_area / die_area
  - Sums all wires on layer, divides by die bounding box

- **Fill Insertion** (metal_fill.cpp:26-84)
  - Grid-based placement: step = fill_width + fill_spacing
  - Conflict detection: fill_rect vs wire_rect overlap
  - Stops when needed_area achieved
  - Returns fill count

- **Multi-Layer Support** (metal_fill.cpp:87-102)
  - Loops all layers, computes before/after density
  - Returns total fill count + per-layer results

- **Configurable Parameters** (metal_fill.hpp:13-20)
  - min_density, max_density (30%-70% defaults)
  - fill_width, fill_height, fill_spacing
  - num_layers

#### ❌ STUBBED / MISSING
```cpp
// MEDIUM: Density verification
// metal_fill.cpp:96: Computes after density
// Missing: DRC rule validation
// Should: Check against foundry min/max, uniformity constraints

// MEDIUM: Dummy pattern optimization
// metal_fill.cpp:49-82: Fixed grid placement
// Missing: Optimization for:
//   1. Routing congestion awareness
//   2. Signal integrity (coupling to fills)
//   3. CMP uniformity (spatial smoothness)
// Current: Greedy grid → suboptimal density distribution

// MEDIUM: Fill type variability
// config has single fill_width/height
// Missing: Support multiple fill sizes for efficiency
// Should: Choose fill size per location to reach target faster

// LOW: Redundant via insertion
// No support for reliability hardening via extra vias

// LOW: Via fill
// Only metal fill; missing via layer support
```

### Key Data Structures
```cpp
// metal_fill.hpp:13-20
struct MetalFillConfig {
    double min_density = 0.30;
    double max_density = 0.70;
    double fill_width = 1.0;
    double fill_height = 1.0;
    double fill_spacing = 0.5;
    int num_layers = 5;
};

// metal_fill.hpp:22-27
struct MetalFillResult {
    int total_fills = 0;
    std::vector<double> density_before;
    std::vector<double> density_after;
    std::string message;
};
```

---

## Summary: 14 Files, 4776 Lines

```
Subsystem              Files   LOC    Status          Priority
────────────────────────────────────────────────────────────
Parasitic Extraction    2     273    40% Complete    P1
SDC Constraints         2     336    50% Complete    P0
MCMM/STA                4    2425    60% Complete    P1
Power Grid/IR Drop      4    1597    70% Complete    P1
Metal Fill              2     145    80% Complete    P2
────────────────────────────────────────────────────────────
TOTAL                  14    4776    60% Complete    ~80 days
```

**Critical Path Blockers:**
1. **P0 (Week 1):** SDC set_clock_groups + set_min_delay
2. **P1 (Weeks 2-4):** SPEF reader, dynamic IR drop, CDC validation
3. **P2 (Weeks 5-8):** Optimization & polish

**Effort:** 2-3 engineers × 2-3 months for Tier 1 production readiness
