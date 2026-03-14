# SiliconForge Tier 1 Gap Closure - Quick Reference

## Status Overview

```
Subsystem                   Completeness    Industrial Ready?    Critical Blocker
────────────────────────────────────────────────────────────────────────────────
Parasitic Extraction        40%  ▓░░░░░    ❌ Low              SPEF reader missing
SDC Timing Constraints      50%  ▓▓░░░    ❌ Low              set_clock_groups missing
MCMM / Multi-Clock STA      60%  ▓▓▓░░    ⚠️  Medium          CDC validation missing
Power Grid / IR Drop        70%  ▓▓▓▓░    ⚠️  Medium-High     Dynamic IR drop stubbed
Metal Fill                  80%  ▓▓▓▓░    ✅ High             Minor (optimization only)
```

---

## Critical Blockers (Must Fix)

### 1. SDC Constraints - `set_clock_groups` (CRITICAL)
**Why:** Blocks all clock domain crossing (CDC) validation  
**Location:** `/src/frontend/sdc_parser.{hpp,cpp}`  
**What's Missing:** 
- Clock group relationship types: `{async|logically_exclusive|physically_exclusive|safe_sync}`
- Cross-domain path detection in STA
- Synchronizer metastability checking

**Quick Fix:** 
```cpp
// Add to sdc_parser.hpp after line 48
struct SdcClockGroup {
    std::vector<std::string> clocks;
    enum Type { ASYNC, LOGICALLY_EXCLUSIVE, PHYSICALLY_EXCLUSIVE, SAFE_SYNC } type;
};

// In SdcConstraints struct add:
std::vector<SdcClockGroup> clock_groups;
```
**Effort:** 2-3 days (parsing + STA validation)

---

### 2. Parasitic Extraction - SPEF Reader (CRITICAL)
**Why:** Blocks static IR drop analysis and parasitic-aware timing  
**Location:** `/src/timing/parasitics.{hpp,cpp}`  
**What's Missing:**
- SPEF file parser (write-only currently)
- Integration of loaded parasitics into STA delay calculation
- RC delay propagation

**Impact:** Cannot import SPEF from P&R tools; design is timing-disconnected

**Effort:** 3-5 days

---

### 3. IR Drop - Dynamic Analysis Stubbed (CRITICAL)
**Why:** Cannot catch peak IR drop during switching events  
**Location:** `/src/timing/ir_drop.cpp` line 197  
```cpp
DynamicIrResult analyze_dynamic(double time_step_ps = 10.0, int num_steps = 100);
// ⚠️ DECLARED BUT NOT IMPLEMENTED
```

**Missing Implementation:**
- Time-stepping transient solver
- Per-step current injection from power switching
- Decap discharge/charge dynamics
- Peak drop reporting

**Effort:** 7-10 days

---

### 4. MCMM - Clock Domain Crossing Validation (CRITICAL)
**Why:** Metastability violations not detected  
**Location:** `/src/timing/sta.cpp` (missing CDC validation logic)  
**Required:**
- Parse CDC constraints from SDC clock groups
- Detect cross-domain paths in STA
- Flag synchronizer insertion checks

**Effort:** 5-7 days

---

## High-Priority Gaps (Should Fix)

### 5. MCMM - Per-Corner Clock Delays
**File:** `/src/timing/mcmm.cpp` line 87-112  
**Issue:** Clock insertion delays not per-corner; generic model used  
**Impact:** Setup/hold accuracy off by 10-20%  
**Effort:** 3-5 days

---

### 6. IR Drop - Spatial Analysis Stubbed
**File:** `/src/timing/ir_drop.cpp` line 261  
```cpp
void spatial_analysis(int grid_res, PdnResult& r);
// EMPTY - fills zeros into node voltage map
```
**Impact:** Cannot see node-level voltage distribution  
**Effort:** 2-3 days (fill grid from Gauss-Seidel solve)

---

### 7. Parasitic Extraction - Via Extraction
**File:** `/src/timing/parasitics.cpp` line 40  
```cpp
size_t num_vias = 0;  // ⚠️ Hardcoded! Should count from physical layout
```
**Impact:** Via parasitics 100% wrong (zeros instead of actual values)  
**Effort:** 2-3 days

---

## Medium-Priority Gaps

### 8. MCMM - POCV Path-Level Integration
- Declared: `PocvTable pocv_table` (sta.hpp line 52)
- Missing: Path-level sigma RSS computation
- Effort: 3-5 days

### 9. Metal Fill - Density Verification
- Works but doesn't validate DRC rules
- Effort: 1-2 days

### 10. SDC - `set_min_delay` Parsing
- Enum exists, parsing missing
- Easy win: 1 day

---

## Implementation Roadmap (65-82 Days Total)

### Week 1: P0 Blocker Removal
```
Day 1-2:   SDC set_clock_groups parsing (basic struct + tokenizer)
Day 3-4:   SDC set_min_delay parsing (1-day task, include timing) 
Day 5:     MCMM CDC framework (enum + detector structure)
Day 6-7:   Parasitics via extraction fix (hardcoded to layout count)
           IR Drop spatial analysis completion
```

### Weeks 2-3: Critical Path Analysis
```
Day 8-12:  SPEF parser implementation
Day 13-15: Dynamic IR drop transient solver (basic version)
Day 16-18: MCMM CDC validation logic
Day 19-20: Parasitic coupling upgrade (multi-layer, fringe)
```

### Weeks 4-6: Deep Integration
```
Day 21-25: Per-corner CTS delays in MCMM
Day 26-30: Voltage-aware timing (IR drop → STA derating)
Day 31-35: POCV path-level integration
Day 36-40: Decap optimization + resonance analysis
```

### Weeks 7-8: Polish & Testing
```
Day 41-45: Vectored IR drop (VCD-driven)
Day 46-50: Crosstalk integration in STA
Day 51-55: Metal fill optimization + DRC checking
Day 56-65: Testing, validation, regression
```

---

## File Checklist (What Exists vs. What's Stubbed)

### ✅ EXISTS (Fully or Mostly Implemented)
- ✅ `parasitics.{hpp,cpp}` - RC extraction (basic)
- ✅ `sdc_parser.{hpp,cpp}` - Clock/delay/false_path parsing
- ✅ `mcmm.{hpp,cpp}` - Scenario creation, analysis loop
- ✅ `sta.{hpp,cpp}` - Graph-based STA engine
- ✅ `pdn.{hpp,cpp}` - AC impedance analysis
- ✅ `ir_drop.{hpp,cpp}` - Gauss-Seidel solver
- ✅ `metal_fill.{hpp,cpp}` - Grid-based fill insertion

### ❌ MISSING (Stubs or Empty)
- ❌ `parasitics.cpp` line 40: Via extraction (num_vias = 0)
- ❌ `ir_drop.cpp` line 197: `analyze_dynamic()` body empty
- ❌ `ir_drop.cpp` line 199: `analyze_vectored()` body empty
- ❌ `ir_drop.cpp` line 202: `analyze_voltage_timing()` body empty
- ❌ `ir_drop.cpp` line 238: `spatial_analysis()` body empty
- ❌ `pdn.cpp` line 258: `find_resonances()` likely stubbed
- ❌ `sdc_parser.cpp`: No `set_clock_groups` handler
- ❌ `sdc_parser.cpp`: No `set_min_delay` parse logic
- ❌ `sta.cpp`: No CDC validation logic

---

## Key Code Locations for Fixes

| Component | Header | Implementation | Key Lines |
|-----------|--------|-----------------|-----------|
| Parasitic coupling | parasitics.hpp:25-30 | parasitics.cpp:156-191 | Coupling model (linear) |
| Via extraction | parasitics.hpp:18-22 | parasitics.cpp:40-45 | **Hardcoded num_vias=0** |
| SPEF output | parasitics.hpp:39 | parasitics.cpp:80-123 | Write-only |
| Clock groups | sdc_parser.hpp:- | sdc_parser.cpp:- | **MISSING** |
| Set min delay | sdc_parser.hpp:37 | sdc_parser.cpp:161 | Enum exists, parse missing |
| OCV/AOCV | sta.hpp:21-40 | sta.cpp:16-65 | Implemented |
| POCV | sta.hpp:47-62 | sta.cpp:27 | Stub: "POCV: base derate..." |
| CDC detection | mcmm.hpp:- | mcmm.cpp:- | **MISSING** |
| IR Drop dynamic | ir_drop.hpp:137-143 | ir_drop.cpp:197 | **Empty impl** |
| Spatial analysis | ir_drop.hpp:261 | ir_drop.cpp:238 | **Empty impl** |
| PDN resonance | pdn.hpp:113-120 | pdn.cpp:258 | Find resonances: **stub** |

---

## Validation Checklist

After Tier 1 closure, verify:

```
[ ] SDC parser: set_clock_groups recognized + stored
[ ] SDC parser: set_min_delay parsed (not just MAX_DELAY)
[ ] STA: CDC paths detected and reported
[ ] Parasitics: SPEF file can be read + loaded
[ ] Parasitics: Via count extracted from layout (not hardcoded)
[ ] MCMM: Multi-corner hold check (FF data + SS clock)
[ ] IR Drop: Dynamic analysis runs (not empty stub)
[ ] IR Drop: Spatial voltage map populated from solver
[ ] PDN: Resonance peaks identified in AC sweep
[ ] Metal Fill: Density verified against DRC spec
```

---

## Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| SPEF reader bugs block import flow | High | CRITICAL | Early parser unit tests + golden SPEF files |
| Dynamic IR drop oscillation/divergence | High | CRITICAL | Conservative time stepping, RLC stability analysis |
| CDC false negatives (missed violations) | Medium | CRITICAL | Comprehensive test suite with known metastability cases |
| POCV path sigma incorrect | Medium | HIGH | Validate RSS against statistical literature |
| Per-corner CTS delays conflict with STA | Medium | MEDIUM | Clear precedence rules in documentation |

---

## Success Metrics

- ✅ All 5 subsystems pass unit test coverage ≥ 80%
- ✅ Industrial SPICE/GDS testcases process without errors
- ✅ Timing closure within ±5% of Synopsys PrimeTime
- ✅ IR drop within ±10% of detailed 3D solver
- ✅ CDC validation detects 100% of synthetic metastability cases
- ✅ All P0/P1 gaps closed; P2 gaps documented for future

