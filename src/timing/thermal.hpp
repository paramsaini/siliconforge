#pragma once
// SiliconForge — Thermal Analysis Engine
// Phase 41: 2D/3D steady-state & transient thermal analysis
//
// Physics: Solves ∇·(k∇T) = q (heat equation) via finite-difference method
// Models: Fourier's law, thermal resistance network, package convection
// References:
//   - Skadron et al., "Temperature-Aware Microarchitecture", ISCA 2003 (HotSpot)
//   - Huang et al., "A Compact Thermal Modeling Methodology", IEEE T-VLSI 2006

#include "pnr/physical.hpp"
#include "core/die_to_die.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <map>

namespace sf {

// ── Material Thermal Properties ─────────────────────────────────────────────
struct ThermalMaterial {
    std::string name;
    double conductivity;       // W/(m·K)
    double specific_heat;      // J/(kg·K)
    double density;            // kg/m³

    double diffusivity() const {
        return (density > 0 && specific_heat > 0)
            ? conductivity / (density * specific_heat) : 0;
    }
};

// Preset materials
inline ThermalMaterial silicon_material()  { return {"silicon",  148.0, 712.0, 2329.0}; }
inline ThermalMaterial copper_material()   { return {"copper",   401.0, 385.0, 8960.0}; }
inline ThermalMaterial sio2_material()     { return {"SiO2",       1.4, 730.0, 2200.0}; }
inline ThermalMaterial tim_material()      { return {"TIM",        5.0, 1000.0, 2500.0}; }
inline ThermalMaterial pcb_material()      { return {"PCB_FR4",    0.3, 1100.0, 1850.0}; }

// ── Thermal Grid Cell ───────────────────────────────────────────────────────
struct ThermalCell {
    double temperature = 25.0;  // °C (current)
    double power_density = 0;   // W/m² (heat source)
    double conductivity = 148.0;// W/(m·K) (local)
    int material_id = 0;        // 0=silicon, 1=copper, 2=SiO2, etc.
};

// ── Package Thermal Model (Rjc-Rca network) ────────────────────────────────
struct PackageThermalModel {
    double r_jc = 0.5;        // Junction-to-case thermal resistance (°C/W)
    double r_ca = 5.0;        // Case-to-ambient thermal resistance (°C/W)
    double r_jb = 2.0;        // Junction-to-board (°C/W)
    double t_ambient = 25.0;  // Ambient temperature (°C)
    double heatsink_r = 0.3;  // Heatsink thermal resistance (°C/W)
    bool has_heatsink = false;

    double total_resistance() const {
        double r_top = has_heatsink ? (r_jc + heatsink_r) : (r_jc + r_ca);
        // Parallel path: top (case→ambient) || bottom (junction→board)
        return (r_top * r_jb) / (r_top + r_jb);
    }

    double junction_temp(double power_w) const {
        return t_ambient + power_w * total_resistance();
    }
};

// ── DVFS Thermal Control ────────────────────────────────────────────────────
struct DvfsState {
    double frequency_ghz = 1.0;
    double voltage_v = 0.8;
    std::string mode = "nominal";  // nominal, turbo, throttle, emergency
};

struct DvfsConfig {
    double turbo_freq_ghz = 1.5;
    double turbo_voltage_v = 0.9;
    double nominal_freq_ghz = 1.0;
    double nominal_voltage_v = 0.8;
    double throttle_freq_ghz = 0.6;
    double throttle_voltage_v = 0.65;
    double emergency_freq_ghz = 0.3;
    double emergency_voltage_v = 0.55;

    double turbo_threshold_c = 70.0;
    double throttle_threshold_c = 85.0;
    double emergency_threshold_c = 100.0;
    double critical_threshold_c = 110.0;   // shutdown

    DvfsState compute_state(double temp_c) const;
    double power_scaling(const DvfsState& state) const;
};

// ── Thermal Sensor ──────────────────────────────────────────────────────────
struct ThermalSensor {
    int id = -1;
    float x = 0, y = 0;
    int grid_x = 0, grid_y = 0;
    double reading = 25.0;     // last reading (°C)
    double accuracy = 1.0;     // ±°C
};

// ── Thermal Analysis Configuration ──────────────────────────────────────────
struct ThermalConfig {
    int grid_nx = 64;              // grid resolution X
    int grid_ny = 64;              // grid resolution Y
    double die_width_um = 10000;
    double die_height_um = 10000;
    double die_thickness_um = 100;
    ThermalMaterial material = silicon_material();
    PackageThermalModel package;
    double convergence_tol = 0.01;  // °C
    int max_iterations = 5000;
    double sor_omega = 1.6;         // SOR relaxation factor

    // Transient parameters
    double time_step_us = 1.0;     // μs
    int transient_steps = 100;
};

// ── Hotspot Information ─────────────────────────────────────────────────────
struct Hotspot {
    int grid_x = 0, grid_y = 0;
    double x_um = 0, y_um = 0;
    double temperature = 0;
    double power_density = 0;
};

// ── Thermal Analysis Result ─────────────────────────────────────────────────
struct ThermalResult {
    double max_temperature = 0;
    double min_temperature = 0;
    double avg_temperature = 0;
    double thermal_gradient = 0;   // max - min
    int iterations = 0;
    bool converged = false;
    double total_power_w = 0;
    int num_hotspots = 0;
    std::vector<Hotspot> hotspots;

    // Per-grid temperature map (flattened: grid[x * ny + y])
    std::vector<double> temperature_map;
    int grid_nx = 0, grid_ny = 0;
};

// ── Thermal Analysis Engine ─────────────────────────────────────────────────
class ThermalAnalyzer {
public:
    explicit ThermalAnalyzer(const ThermalConfig& cfg = {});

    // Set power map from physical design (cell-based)
    void set_power_from_cells(const PhysicalDesign& pd,
                              const std::vector<double>& cell_powers_mw);

    // Set power density at specific grid point
    void set_power_density(int gx, int gy, double watts_per_m2);

    // Set uniform power across entire die
    void set_uniform_power(double total_power_w);

    // Set block-level power (region of die)
    void set_block_power(double x0_um, double y0_um, double x1_um, double y1_um,
                         double power_w);

    // ── Steady-state analysis ─────────────────────────────────────────
    ThermalResult solve_steady_state();

    // ── Transient analysis ────────────────────────────────────────────
    // Returns temperature map at each time step
    std::vector<ThermalResult> solve_transient(int steps = -1);

    // ── 3D thermal (multi-die) ────────────────────────────────────────
    ThermalResult solve_3d_stack(const std::vector<double>& die_powers_w,
                                 int num_dies);

    // ── Hotspot detection ─────────────────────────────────────────────
    std::vector<Hotspot> find_hotspots(double threshold_c) const;

    // ── DVFS thermal management ───────────────────────────────────────
    DvfsState compute_dvfs(double max_temp_c, const DvfsConfig& dvfs = {}) const;

    // ── Sensor network ────────────────────────────────────────────────
    void place_sensors(int num_sensors);
    std::vector<ThermalSensor> read_sensors() const;

    // ── Coupled electro-thermal ───────────────────────────────────────
    // Returns timing derating factor due to thermal effects
    double timing_derating(double temp_c) const;
    // Leakage scaling with temperature
    double leakage_scaling(double temp_c, double ref_temp = 25.0) const;

    // ── Accessors ─────────────────────────────────────────────────────
    const ThermalConfig& config() const { return cfg_; }
    const std::vector<ThermalCell>& grid() const { return grid_; }

private:
    ThermalConfig cfg_;
    std::vector<ThermalCell> grid_;   // nx × ny
    std::vector<ThermalSensor> sensors_;
    ThermalResult last_result_;

    // Finite-difference SOR solver
    void sor_iteration(std::vector<double>& T, const std::vector<double>& Q,
                       double dx, double dy, double k, double omega);
    double boundary_temp(int gx, int gy, double total_power) const;
};

} // namespace sf
