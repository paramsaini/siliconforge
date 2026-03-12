# SiliconForge IR Drop / Power Grid (PDN) Subsystem - Complete Research

## 1. FILE INVENTORY

### IR Drop / Power Grid / PDN Related Files:

| File Path | Type | Lines | Purpose |
|-----------|------|-------|---------|
| src/timing/ir_drop.hpp | Header | 46 | IR Drop Analyzer class definition |
| src/timing/ir_drop.cpp | Source | 90 | IR Drop analysis implementation |
| src/timing/pdn.hpp | Header | 62 | Power Distribution Network analyzer |
| src/timing/pdn.cpp | Source | 127 | PDN analysis implementation |
| src/timing/power.hpp | Header | 77 | Power analysis engine |
| src/timing/power.cpp | Source | 419 | Power analysis (static/dynamic/glitch/clock) |
| src/verify/reliability.hpp | Header | 70 | Reliability & IR-Drop embedded analysis |
| src/verify/reliability.cpp | Source | 239 | Aging + embedded IR-drop mesh solver |

**Total: 821 lines of core IR drop/PDN/power code**

### Related Files That Reference IR Drop/PDN:
- src/flow/engine.cpp - Flow orchestration (calls run_power(), run_reliability())
- src/shell/sf_shell.cpp - CLI interface (reliability/ir_drop command)
- src/shell/tcl_interp.cpp - TCL interpreter (ir_drop command registration)
- src/timing/mcmm.cpp - Multi-corner multi-mode power analysis
- src/timing/electromigration.cpp - EM analysis (depends on IR drop analysis)
- src/timing/noise.cpp - Power supply noise analysis

---

## 2. COMPLETE SOURCE CODE

### A. IR DROP ANALYZER (ir_drop.hpp + ir_drop.cpp)

#### ir_drop.hpp - Header File (46 lines)
```cpp
#pragma once
// SiliconForge — IR Drop Analyzer
// Computes supply voltage drop across the power grid.
// Reference: Zhuo et al., "Static and Dynamic IR-Drop Analysis", ICCAD 2004

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct IrDropResult {
    double worst_drop_mv = 0;
    double avg_drop_mv = 0;
    double vdd = 0;
    int num_hotspots = 0;     // regions exceeding threshold
    double threshold_pct = 5; // % of VDD
    double time_ms = 0;

    struct HotSpot {
        Rect region;
        double drop_mv;
        double current_ma;
    };
    std::vector<HotSpot> hotspots;
    std::string message;

    // Grid map of IR drop (for visualization)
    std::vector<std::vector<double>> drop_map; // drop_map[y][x] in mV
    int grid_x = 0, grid_y = 0;
};

class IrDropAnalyzer {
public:
    IrDropAnalyzer(const PhysicalDesign& pd, double vdd = 1.8, double total_current_ma = 100)
        : pd_(pd), vdd_(vdd), total_current_ma_(total_current_ma) {}

    IrDropResult analyze(int grid_res = 10);

private:
    const PhysicalDesign& pd_;
    double vdd_;
    double total_current_ma_;
};

} // namespace sf
```

#### ir_drop.cpp - Implementation (90 lines)
```cpp
// SiliconForge — IR Drop Analyzer Implementation
#include "timing/ir_drop.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace sf {

IrDropResult IrDropAnalyzer::analyze(int grid_res) {
    auto t0 = std::chrono::high_resolution_clock::now();
    IrDropResult r;
    r.vdd = vdd_;
    r.grid_x = grid_res;
    r.grid_y = grid_res;
    r.threshold_pct = 5.0;

    double cell_w = pd_.die_area.width() / grid_res;
    double cell_h = pd_.die_area.height() / grid_res;

    // Build current density map: how much current each grid cell draws
    std::vector<std::vector<double>> current_map(grid_res, std::vector<double>(grid_res, 0));
    double total_cell_area = 0;
    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        total_cell_area += c.width * c.height;
        int gx = std::clamp((int)((c.position.x - pd_.die_area.x0) / cell_w), 0, grid_res-1);
        int gy = std::clamp((int)((c.position.y - pd_.die_area.y0) / cell_h), 0, grid_res-1);
        // Current proportional to cell area
        current_map[gy][gx] += c.width * c.height;
    }

    // Normalize current map to total current
    if (total_cell_area > 0) {
        for (auto& row : current_map)
            for (auto& val : row)
                val = (val / total_cell_area) * total_current_ma_;
    }

    // Compute IR drop using resistive grid model
    // Simple model: IR drop increases with distance from the closest power pad
    // Assume power pads at corners
    double grid_resistance = 0.1; // Ohm per grid unit

    r.drop_map.resize(grid_res, std::vector<double>(grid_res, 0));
    double worst_drop = 0, total_drop = 0;
    int total_cells = 0;

    for (int y = 0; y < grid_res; ++y) {
        for (int x = 0; x < grid_res; ++x) {
            // Distance from nearest corner (power pad)
            double d_corner = std::min({
                std::sqrt((double)(x*x + y*y)),
                std::sqrt((double)((grid_res-1-x)*(grid_res-1-x) + y*y)),
                std::sqrt((double)(x*x + (grid_res-1-y)*(grid_res-1-y))),
                std::sqrt((double)((grid_res-1-x)*(grid_res-1-x) + (grid_res-1-y)*(grid_res-1-y)))
            });

            // IR drop = I × R × distance
            double local_current = current_map[y][x];
            double drop_v = local_current * grid_resistance * d_corner * 0.001;
            double drop_mv = drop_v * 1000;

            r.drop_map[y][x] = drop_mv;
            worst_drop = std::max(worst_drop, drop_mv);
            total_drop += drop_mv;
            total_cells++;

            // Check if hotspot
            if (drop_mv > vdd_ * 1000 * r.threshold_pct / 100.0) {
                Rect region(pd_.die_area.x0 + x * cell_w, pd_.die_area.y0 + y * cell_h,
                           pd_.die_area.x0 + (x+1) * cell_w, pd_.die_area.y0 + (y+1) * cell_h);
                r.hotspots.push_back({region, drop_mv, local_current});
                r.num_hotspots++;
            }
        }
    }

    r.worst_drop_mv = worst_drop;
    r.avg_drop_mv = total_cells > 0 ? total_drop / total_cells : 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Worst IR drop: " + std::to_string((int)r.worst_drop_mv) + "mV (" +
                std::to_string(r.worst_drop_mv / (vdd_ * 10)) + "%), " +
                std::to_string(r.num_hotspots) + " hotspot(s)";
    return r;
}

} // namespace sf
```

**Key Implementation Details - IR Drop:**
- **Algorithm**: Grid-based IR drop computation
- **Grid Resistance Model**: 0.1 Ohm per grid unit, distance-based from power pads
- **Current Distribution**: Normalized based on cell area placement
- **Hotspot Detection**: Regions where drop > 5% of VDD
- **Output**: Worst/avg drops, hotspot locations, 2D drop map for visualization
- **Computational Model**: O(grid_res²) complexity, ~Zhuo et al. 2004 reference

---

### B. PDN ANALYZER (pdn.hpp + pdn.cpp)

#### pdn.hpp - Header File (62 lines)
```cpp
#pragma once
// SiliconForge — Power Distribution Network (PDN) Analyzer
// Models VDD/VSS grid, computes voltage drops, and validates EM limits.
// Reference: Nassif, "Power Grid Analysis Benchmarks", ASP-DAC 2008

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct PdnStripe {
    enum Dir { HORIZONTAL, VERTICAL } direction;
    double offset;      // position on perpendicular axis
    double width;       // stripe width
    int layer;
    double resistance_per_um;
};

struct PdnConfig {
    double vdd = 1.8;
    double total_current_ma = 100;
    std::vector<PdnStripe> stripes;
    int pad_count = 4;
    double pad_resistance = 0.01;   // Ohm per pad
    double em_limit_ma_per_um = 1.0; // electromigration current limit
};

struct PdnResult {
    double worst_drop_mv = 0;
    double worst_drop_pct = 0;
    double avg_drop_mv = 0;
    int em_violations = 0;
    double worst_current_density = 0;
    double time_ms = 0;

    struct PdnNode {
        double x, y;
        double voltage;
        double current_draw;
    };
    std::vector<PdnNode> nodes;
    std::string message;
};

class PdnAnalyzer {
public:
    PdnAnalyzer(const PhysicalDesign& pd) : pd_(pd) {}

    void set_config(const PdnConfig& cfg) { cfg_ = cfg; }
    void auto_config(double vdd = 1.8, double current_ma = 100);

    PdnResult analyze(int grid_res = 10);

private:
    const PhysicalDesign& pd_;
    PdnConfig cfg_;

    double nearest_stripe_resistance(double x, double y) const;
};

} // namespace sf
```

#### pdn.cpp - Implementation (127 lines)
```cpp
// SiliconForge — PDN Analyzer Implementation
#include "timing/pdn.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace sf {

void PdnAnalyzer::auto_config(double vdd, double current_ma) {
    cfg_.vdd = vdd;
    cfg_.total_current_ma = current_ma;
    cfg_.pad_count = 4;
    cfg_.stripes.clear();

    // Generate default PDN grid: horizontal + vertical stripes
    double die_w = pd_.die_area.width();
    double die_h = pd_.die_area.height();
    int h_stripes = std::max(2, (int)(die_h / 20.0));
    int v_stripes = std::max(2, (int)(die_w / 20.0));

    for (int i = 0; i < h_stripes; ++i) {
        double offset = pd_.die_area.y0 + (i + 0.5) * die_h / h_stripes;
        cfg_.stripes.push_back({PdnStripe::HORIZONTAL, offset, 1.0, 3, 0.05});
    }
    for (int i = 0; i < v_stripes; ++i) {
        double offset = pd_.die_area.x0 + (i + 0.5) * die_w / v_stripes;
        cfg_.stripes.push_back({PdnStripe::VERTICAL, offset, 1.0, 4, 0.05});
    }
}

double PdnAnalyzer::nearest_stripe_resistance(double x, double y) const {
    double min_dist = 1e18;
    double min_res = 1.0; // default high resistance

    for (auto& s : cfg_.stripes) {
        double dist;
        if (s.direction == PdnStripe::HORIZONTAL)
            dist = std::abs(y - s.offset);
        else
            dist = std::abs(x - s.offset);

        if (dist < min_dist) {
            min_dist = dist;
            min_res = dist * s.resistance_per_um + cfg_.pad_resistance;
        }
    }
    return min_res;
}

PdnResult PdnAnalyzer::analyze(int grid_res) {
    auto t0 = std::chrono::high_resolution_clock::now();
    PdnResult r;

    if (cfg_.stripes.empty()) auto_config();

    double cell_w = pd_.die_area.width() / grid_res;
    double cell_h = pd_.die_area.height() / grid_res;

    // Build current demand map
    std::vector<std::vector<double>> current_map(grid_res, std::vector<double>(grid_res, 0));
    double total_cell_area = 0;
    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        total_cell_area += c.width * c.height;
        int gx = std::clamp((int)((c.position.x - pd_.die_area.x0) / cell_w), 0, grid_res-1);
        int gy = std::clamp((int)((c.position.y - pd_.die_area.y0) / cell_h), 0, grid_res-1);
        current_map[gy][gx] += c.width * c.height;
    }

    // Normalize to total current
    if (total_cell_area > 0) {
        for (auto& row : current_map)
            for (auto& val : row)
                val = (val / total_cell_area) * cfg_.total_current_ma;
    }

    // Compute voltage drop at each grid point: ΔV = I × R_path
    double worst_drop = 0, total_drop = 0;
    int total_nodes = 0;

    for (int y = 0; y < grid_res; ++y) {
        for (int x = 0; x < grid_res; ++x) {
            double px = pd_.die_area.x0 + (x + 0.5) * cell_w;
            double py = pd_.die_area.y0 + (y + 0.5) * cell_h;
            double r_path = nearest_stripe_resistance(px, py);
            double i_local = current_map[y][x];
            double drop_v = i_local * r_path * 0.001; // mA × Ohm = mV
            double drop_mv = drop_v * 1000;
            double voltage = cfg_.vdd * 1000 - drop_mv;

            r.nodes.push_back({px, py, voltage / 1000.0, i_local});

            worst_drop = std::max(worst_drop, drop_mv);
            total_drop += drop_mv;
            total_nodes++;

            // Check EM limits
            for (auto& s : cfg_.stripes) {
                double dist;
                if (s.direction == PdnStripe::HORIZONTAL)
                    dist = std::abs(py - s.offset);
                else
                    dist = std::abs(px - s.offset);
                if (dist < cell_w) {
                    double current_density = i_local / s.width;
                    r.worst_current_density = std::max(r.worst_current_density, current_density);
                    if (current_density > cfg_.em_limit_ma_per_um)
                        r.em_violations++;
                }
            }
        }
    }

    r.worst_drop_mv = worst_drop;
    r.worst_drop_pct = (cfg_.vdd > 0) ? worst_drop / (cfg_.vdd * 10) : 0;
    r.avg_drop_mv = total_nodes > 0 ? total_drop / total_nodes : 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Worst drop: " + std::to_string((int)r.worst_drop_mv) + "mV (" +
                std::to_string((int)r.worst_drop_pct) + "%), " +
                std::to_string(r.em_violations) + " EM violations";
    return r;
}

} // namespace sf
```

**Key Implementation Details - PDN:**
- **Stripe-Based Model**: Horizontal and vertical stripe grid (H+V mesh topology)
- **Auto-Config**: Generates default PDN with 2+ stripes per dimension based on die size
- **Resistance Model**: Distance to nearest stripe × resistance_per_um + pad_resistance
- **EM Checking**: Per-stripe current density checking against limits
- **Output**: Worst/avg drops, EM violations, node voltage map, current density
- **Reference**: Nassif's Power Grid Analysis Benchmarks (ASP-DAC 2008)

---

### C. POWER ANALYZER (power.hpp + power.cpp)

#### power.hpp - Header File (77 lines)
```cpp
#pragma once
// SiliconForge — Power Analysis Engine
// Computes static (leakage), dynamic (switching), internal, glitch, and clock power.
// Reference: Rabaey, Chandrakasan, Nikolic, "Digital Integrated Circuits", 2003

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

struct PowerResult {
    double total_power_mw = 0;
    double dynamic_power_mw = 0;
    double static_power_mw = 0;    // leakage
    double switching_power_mw = 0;
    double internal_power_mw = 0;
    double clock_power_mw = 0;
    double glitch_power_mw = 0;    // spurious transition power
    double clock_freq_mhz = 0;
    double supply_voltage = 1.8;
    int num_cells = 0;
    double time_ms = 0;

    struct CellPower {
        std::string name;
        double dynamic = 0, leakage = 0, total = 0;
    };
    std::vector<CellPower> top_consumers;  // sorted by power
    std::string message;
};

class PowerAnalyzer {
public:
    PowerAnalyzer(const Netlist& nl, const LibertyLibrary* lib = nullptr)
        : nl_(nl), lib_(lib) {}

    // Run power analysis
    PowerResult analyze(double clock_freq_mhz, double supply_voltage = 1.8,
                        double default_activity = 0.1);

    // Set per-net switching activity
    void set_activity(NetId net, double activity) { activities_[net] = activity; }

    // Set supply voltage domain for a gate
    void set_voltage_domain(GateId gid, double vdd) { voltage_domains_[gid] = vdd; }

    // Load VCD toggle counts for activity factors
    bool load_vcd(const std::string& filename);
    bool parse_vcd_string(const std::string& content);

    // Load SAIF activity factors
    bool load_saif(const std::string& filename);

    // UPF power intent
    void set_power_domain(const std::string& domain, double voltage, 
                          const std::vector<GateId>& gates) {
        for (auto gid : gates) voltage_domains_[gid] = voltage;
    }

private:
    const Netlist& nl_;
    const LibertyLibrary* lib_;
    std::unordered_map<NetId, double> activities_;
    std::unordered_map<GateId, double> voltage_domains_;
    std::unordered_map<NetId, int> topo_levels_; // for glitch estimation

    double cell_leakage(GateId gid) const;
    double cell_dynamic(GateId gid, double freq, double vdd, double activity) const;
    double net_capacitance(NetId nid) const;
    double glitch_activity(GateId gid, double base_activity) const;
    double gate_vdd(GateId gid, double default_vdd) const;
};

} // namespace sf
```

#### power.cpp - Implementation (419 lines - KEY PORTIONS)
**Key Methods:**

1. **Activity Propagation (lines 144-208):**
   - Topological propagation of switching activity through logic gates
   - Per-gate type activity rules (AND, OR, XOR, MUX, etc.)
   - Reconvergent path handling

2. **Glitch Power Estimation (lines 92-113):**
   - Detects reconvergent gates with different input arrival times
   - Estimates glitch activity as 5-15% extra activity per level of skew
   - Caps at 50% maximum glitch activity

3. **Power Computation (lines 115-289):**
   - Dynamic: P = α × C × V² × f
   - Leakage: Temperature and voltage domain aware (V² scaling)
   - Internal power: 15% of dynamic
   - Clock tree power: Estimated for CTS buffers
   - Top-10 consumers sorted by power

4. **VCD Parser (lines 299-374):**
   - IEEE 1364-2001 VCD format support
   - Toggle count extraction per signal
   - Activity factor conversion

5. **SAIF Parser (lines 376-417):**
   - Switching Activity Interchange Format support
   - Duration-aware toggle counting
   - Per-signal TC/T0/T1 extraction

**Power Components:**
- **Dynamic Power**: Switching + Internal + Glitch
- **Static Power**: Leakage (gate-level lookup from Liberty)
- **Clock Power**: DFF activity + CTS buffer estimation
- **Total Power**: Dynamic + Static + Clock

---

### D. RELIABILITY ANALYZER (reliability.hpp + reliability.cpp)

#### reliability.hpp - Relevant Portion (70 lines)
```cpp
// ... (see above for full content)
// Contains:
// - AgingConfig structure (lifetime, temperature, voltage, duty cycle)
// - AgingResult with NBTI, HCI, TDDB, EM failure rates
// - IrDropResult with embedded IR-drop resistive mesh
// - ReliabilityAnalyzer class with analyze() and analyze_ir_drop()
```

#### reliability.cpp - IR-Drop Implementation (lines 119-236)

**FULL IR-DROP RESISTIVE MESH SOLVER:**
```cpp
ReliabilityAnalyzer::IrDropResult
ReliabilityAnalyzer::analyze_ir_drop(double total_power_mw) {
    IrDropResult ir;
    double vdd = cfg_.voltage;
    if (vdd <= 0) vdd = 1.0;

    // Grid dimensions (NxN mesh)
    const int N = 16;
    double die_w = 100, die_h = 100;
    if (pd_) {
        die_w = std::max(1.0, pd_->die_area.width());
        die_h = std::max(1.0, pd_->die_area.height());
    }

    double dx = die_w / N, dy = die_h / N;

    // Sheet resistance model: R = Rsheet * length / width
    // For top-level power mesh (M8+): ~10 mΩ/sq typical
    double r_sheet = 0.010; // Ohms per square
    double r_x = r_sheet * dx / dy; // resistance between horizontal neighbors
    double r_y = r_sheet * dy / dx; // resistance between vertical neighbors

    // Current density map: distribute total current across grid
    double total_current = (total_power_mw > 0) ? total_power_mw / (vdd * 1000) : 0.001;
    // Per-cell current
    int num_logic = 0;
    for (size_t i = 0; i < nl_.num_gates(); ++i) {
        auto t = nl_.gate(i).type;
        if (t != GateType::INPUT && t != GateType::OUTPUT &&
            t != GateType::CONST0 && t != GateType::CONST1) num_logic++;
    }
    double per_cell_current = (num_logic > 0) ? total_current / num_logic : 0;

    // Build current source map on grid
    std::vector<std::vector<double>> I_sink(N, std::vector<double>(N, 0));
    if (pd_) {
        for (size_t ci = 0; ci < pd_->cells.size(); ++ci) {
            auto& c = pd_->cells[ci];
            if (!c.placed) continue;
            int gx = std::clamp((int)(c.position.x / dx), 0, N - 1);
            int gy = std::clamp((int)(c.position.y / dy), 0, N - 1);
            I_sink[gx][gy] += per_cell_current;
        }
    } else {
        // Uniform distribution
        double i_per_node = total_current / (N * N);
        for (int x = 0; x < N; x++)
            for (int y = 0; y < N; y++)
                I_sink[x][y] = i_per_node;
    }

    // Voltage grid — initialize to VDD
    std::vector<std::vector<double>> V(N, std::vector<double>(N, vdd));

    // Boundary: VDD pads at edges (held at VDD)
    auto is_pad = [&](int x, int y) {
        return x == 0 || x == N - 1 || y == 0 || y == N - 1;
    };

    // Gauss-Seidel iterative relaxation
    // Node equation: sum_neighbors((V_n - V_ij)/R_n) = I_sink_ij
    // V_ij = (sum(V_n/R_n) - I_sink_ij) / sum(1/R_n)
    const int MAX_ITER = 200;
    const double TOL = 1e-8;

    for (int iter = 0; iter < MAX_ITER; iter++) {
        double max_delta = 0;
        for (int x = 0; x < N; x++) {
            for (int y = 0; y < N; y++) {
                if (is_pad(x, y)) continue; // pads held at VDD

                double sum_g = 0, sum_gv = 0;
                // Left neighbor
                if (x > 0) { double g = 1.0 / r_x; sum_g += g; sum_gv += g * V[x-1][y]; }
                // Right neighbor
                if (x < N-1) { double g = 1.0 / r_x; sum_g += g; sum_gv += g * V[x+1][y]; }
                // Down neighbor
                if (y > 0) { double g = 1.0 / r_y; sum_g += g; sum_gv += g * V[x][y-1]; }
                // Up neighbor
                if (y < N-1) { double g = 1.0 / r_y; sum_g += g; sum_gv += g * V[x][y+1]; }

                double v_new = (sum_gv - I_sink[x][y]) / sum_g;
                v_new = std::min(v_new, vdd); // can't exceed VDD
                double delta = std::abs(v_new - V[x][y]);
                if (delta > max_delta) max_delta = delta;
                V[x][y] = v_new;
            }
        }
        if (max_delta < TOL) break;
    }

    // Analyze results
    double max_drop = 0, total_drop = 0;
    int count = 0;
    for (int x = 0; x < N; x++) {
        for (int y = 0; y < N; y++) {
            if (is_pad(x, y)) continue;
            double drop = vdd - V[x][y];
            if (drop > max_drop) max_drop = drop;
            total_drop += drop;
            count++;
            if (drop / vdd > 0.05) {
                ir.hotspots++;
                ir.hotspot_locations.push_back({x * dx + dx/2, y * dy + dy/2});
            }
        }
    }

    ir.max_drop_mv = max_drop * 1000;
    ir.avg_drop_mv = (count > 0) ? (total_drop / count) * 1000 : 0;
    ir.drop_pct = (max_drop / vdd) * 100;
    return ir;
}
```

**Key Features:**
- **Resistive Mesh Model**: 16x16 grid (configurable)
- **Sheet Resistance**: 10 mΩ/sq (typical for M8+ power mesh)
- **Solver**: Gauss-Seidel iterative relaxation (up to 200 iterations, tolerance 1e-8)
- **Boundary Conditions**: VDD pads held at all edges
- **Current Sinks**: Distributed per placed cell or uniformly if no placement
- **Output**: Max/avg drops (mV and %), hotspot count and locations

---

## 3. INTEGRATION WITH OTHER SUBSYSTEMS

### A. Flow Orchestration (flow/engine.cpp)

**Power Analysis Integration (run_power()):**
```cpp
bool SiliconForge::run_power() {
    PowerAnalyzer pa(nl_, lib_.cells.empty() ? nullptr : &lib_);
    auto report = pa.analyze(1.0, 1000.0);  // 1GHz, fixed 1000mV for demo
    
    // Store results:
    power_result_.dynamic_mw = report.dynamic_power_mw;
    power_result_.leakage_mw = report.static_power_mw;
    power_result_.switching_mw = report.switching_power_mw;
    power_result_.total_mw = report.total_power_mw;
}
```

**Reliability & IR-Drop Integration (run_reliability()):**
```cpp
bool SiliconForge::run_reliability() {
    ReliabilityAnalyzer ra(nl_, has_floorplan_ ? &pd_ : nullptr);
    AgingConfig cfg;
    cfg.lifetime_years = 10;
    cfg.temperature_c = 85;
    cfg.voltage = 1.0;
    cfg.duty_cycle = 0.5;
    ra.set_config(cfg);
    
    auto result = ra.analyze();
    
    // IR-drop mesh simulation using power from run_power()
    double power_mw = is_power_done_ ? power_result_.total_mw : 0;
    auto ir = ra.analyze_ir_drop(power_mw);
    
    // Warnings for > 5% hotspots or > 10% worst case
}
```

### B. Shell Integration

**TCL Command Registration (tcl_interp.cpp, line 1166):**
```cpp
register_command("ir_drop", [&engine](auto&) -> std::string {
    return engine.run_reliability() ? "1" : "0";
});
```

**CLI Command Dispatch (sf_shell.cpp, line 112):**
```cpp
} else if (cmd == "reliability" || cmd == "ir_drop") {
    engine_.run_reliability();
}
```

### C. Multi-Corner Multi-Mode Analysis (mcmm.cpp)

**Power Analysis Per Scenario:**
```cpp
PowerAnalyzer pa(nl_);
double freq = mode.clock_freq_mhz > 0 ? mode.clock_freq_mhz : 1.0;
scenario.power = pa.analyze(freq, corner.voltage, mode.switching_activity);
```

### D. Electromigration Analysis

**Depends on:**
- IR drop results (voltage distribution)
- Current density information
- Power estimates for current scaling

---

## 4. TEST COVERAGE

**Test File: test_phase5.cpp (Power/Timing Tests)**

Tests for PowerAnalyzer:
1. **test_power_basic**: Validates total/dynamic/static power > 0
2. **test_power_voltage_scaling**: Confirms V² scaling (lower V = lower P)

```cpp
TEST(power_basic) {
    auto nl = build_timing_circuit();
    PowerAnalyzer pa(nl);
    auto result = pa.analyze(500.0, 1.8); // 500MHz, 1.8V

    CHECK(result.total_power_mw > 0, "total power > 0");
    CHECK(result.dynamic_power_mw > 0, "dynamic > 0");
    CHECK(result.static_power_mw > 0, "static > 0");
    CHECK(result.dynamic_power_mw > result.static_power_mw, "dynamic > static at high freq");
}
```

---

## 5. CMakeLists.txt BUILD CONFIGURATION

**Timing Module Compilation (src/CMakeLists.txt, lines 73-82):**
```cmake
# ── Timing & Power ────────────────────────────────────
timing/sta.cpp
timing/power.cpp
timing/parasitics.cpp
timing/ir_drop.cpp
timing/pdn.cpp
timing/noise.cpp
timing/signal_integrity.cpp
timing/electromigration.cpp
timing/mcmm.cpp
```

All files are compiled into the main static library: `siliconforge_lib`

---

## 6. IMPLEMENTATION STATUS SUMMARY

### FULLY IMPLEMENTED:

✅ **PowerAnalyzer**
- Dynamic power (switching + internal)
- Static power (leakage)
- Glitch power estimation
- Clock tree power estimation
- Activity propagation
- VCD file parsing
- SAIF file parsing
- Multi-voltage domain support
- Top-10 consumer ranking

✅ **IrDropAnalyzer** 
- Grid-based IR drop computation
- Current distribution mapping
- Hotspot detection
- Distance-based pad resistance model
- Visualization-ready 2D drop map

✅ **PdnAnalyzer**
- Stripe-based PDN topology (H+V mesh)
- Auto-configuration
- Nearest-stripe resistance calculation
- EM current density checking
- Per-node voltage mapping

✅ **ReliabilityAnalyzer::analyze_ir_drop()**
- Full resistive mesh solver (16x16 grid)
- Gauss-Seidel iterative solver
- Sheet resistance model (~10mΩ/sq)
- Cell-based current sink distribution
- Boundary condition enforcement (VDD pads at edges)

### PARTIALLY IMPLEMENTED (Stub/Simple):

⚠️ **IrDropAnalyzer**
- Uses simplified distance-to-corner model
- Not a full resistive mesh solver (see ReliabilityAnalyzer for real solver)
- No stripe-aware grid refinement

⚠️ **PdnAnalyzer**
- Basic stripe model (no cross-coupling, via placement)
- No decap modeling
- No parasitic inductance
- EM violations counted but no detailed analysis

⚠️ **PowerAnalyzer Glitch Estimation**
- Level-difference based heuristic (not full reconvergent gate analysis)
- Capped at 50% activity

### NOT IMPLEMENTED:

❌ EM (Electromigration) - exists but heavily stubbed
❌ Noise/PSIJ - exists but basic implementation
❌ Voltage regulation module (VREG) stability
❌ Decap optimization
❌ PDN transient simulation
❌ Substrate coupling

---

## 7. DATA STRUCTURES

### Key Result Types:

**IrDropResult:**
- worst_drop_mv, avg_drop_mv
- num_hotspots, threshold_pct
- hotspots[] with region, drop, current
- drop_map[][] for visualization
- message, time_ms

**PdnResult:**
- worst_drop_mv, worst_drop_pct, avg_drop_mv
- em_violations, worst_current_density
- nodes[] with (x,y,voltage,current_draw)
- message, time_ms

**PowerResult:**
- total_power_mw, dynamic/static/switching/internal/clock/glitch (mW)
- clock_freq_mhz, supply_voltage
- num_cells, time_ms
- top_consumers[] sorted by power
- message

**AgingResult** (includes IR-Drop):
- nbti_vth_shift_mv, hci_vth_shift_mv
- tddb_failure_rate, em_failure_rate
- timing_degradation_pct
- cells_at_risk, details[]
- IrDropResult with max/avg drops, hotspots

---

## 8. KEY ALGORITHMS SUMMARY

| Algorithm | Location | Complexity | Notes |
|-----------|----------|-----------|-------|
| IR Drop (Simple) | ir_drop.cpp | O(N²) | Distance to 4 corner pads |
| Activity Propagation | power.cpp:115+ | O(gates) | Topo sort, gate-specific rules |
| Glitch Estimation | power.cpp:93-113 | O(gates) | Topo level difference heuristic |
| Gauss-Seidel Mesh | reliability.cpp:189+ | O(N² × iter) | 16×16 mesh, 200 iters max, 1e-8 tol |
| PDN Stripe Lookup | pdn.cpp:32-49 | O(stripes) | Linear search for nearest stripe |
| VCD Parse | power.cpp:299+ | O(vcd_lines) | HashMap-based signal mapping |

---

## 9. REFERENCES CITED IN CODE

1. **IR Drop**: Zhuo et al., "Static and Dynamic IR-Drop Analysis", ICCAD 2004
2. **PDN**: Nassif, "Power Grid Analysis Benchmarks", ASP-DAC 2008
3. **Power**: Rabaey et al., "Digital Integrated Circuits", 2003
4. **Reliability**: Rauch, "Review and Reexamination of Reliability Effects", IEEE TED 2007
5. **EM**: J.R. Black (1969), JEDEC JEP122H

---

## 10. USAGE EXAMPLES

### Power Analysis:
```cpp
PowerAnalyzer pa(netlist, &liberty_lib);
auto result = pa.analyze(500.0,  // 500 MHz
                        1.8,     // 1.8V
                        0.1);    // 10% default activity
```

### IR Drop (Simple):
```cpp
IrDropAnalyzer ir(physical_design, 1.8, 100);
auto result = ir.analyze(10);  // 10×10 grid
```

### PDN Analysis:
```cpp
PdnAnalyzer pdn(physical_design);
pdn.auto_config(1.8, 100);
auto result = pdn.analyze(10);
```

### Reliability with IR-Drop Mesh:
```cpp
ReliabilityAnalyzer ra(netlist, &physical_design);
auto aging = ra.analyze();
auto ir = ra.analyze_ir_drop(100);  // 100mW total power
```

---

## CONCLUSION

The SiliconForge IR Drop / Power Grid subsystem consists of **821 lines** of production code across **8 core files** (4 header + 4 implementation), with integration points throughout the timing, reliability, and flow orchestration subsystems. The implementation spans from simple grid-based models (IrDropAnalyzer, PdnAnalyzer) to a full Gauss-Seidel resistive mesh solver (ReliabilityAnalyzer), comprehensive power analysis with glitch and clock components, and activity propagation through netlist topology.

**Key Strengths:**
- Complete power estimation pipeline (static + dynamic + glitch + clock)
- Working resistive mesh solver with proper boundary conditions
- Activity-aware propagation through gate types
- VCD and SAIF file support for real simulation data
- EM-aware PDN analysis

**Known Limitations:**
- IrDropAnalyzer uses simplified distance model (not mesh-based)
- PDN stripe model lacks decap and via placement
- Glitch estimation is heuristic (level-difference based)
- No transient PDN simulation
- EM analysis is mostly stubbed

