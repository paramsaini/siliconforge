// SiliconForge — Thermal Analysis Engine Implementation
// Phase 41: 2D/3D steady-state & transient thermal solver
//
// Core algorithm: Finite-difference discretization of ∇·(k∇T) = q
// Solved via SOR (Successive Over-Relaxation) iterative method
// Package model: Rjc/Rca thermal resistance network with optional heatsink

#include "timing/thermal.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <cassert>

namespace sf {

// ── DVFS State Machine ──────────────────────────────────────────────────────

DvfsState DvfsConfig::compute_state(double temp_c) const {
    DvfsState s;
    if (temp_c >= critical_threshold_c) {
        // Critical: would normally shutdown, but report emergency
        s.frequency_ghz = emergency_freq_ghz;
        s.voltage_v = emergency_voltage_v;
        s.mode = "critical";
    } else if (temp_c >= emergency_threshold_c) {
        s.frequency_ghz = emergency_freq_ghz;
        s.voltage_v = emergency_voltage_v;
        s.mode = "emergency";
    } else if (temp_c >= throttle_threshold_c) {
        s.frequency_ghz = throttle_freq_ghz;
        s.voltage_v = throttle_voltage_v;
        s.mode = "throttle";
    } else if (temp_c < turbo_threshold_c) {
        s.frequency_ghz = turbo_freq_ghz;
        s.voltage_v = turbo_voltage_v;
        s.mode = "turbo";
    } else {
        s.frequency_ghz = nominal_freq_ghz;
        s.voltage_v = nominal_voltage_v;
        s.mode = "nominal";
    }
    return s;
}

double DvfsConfig::power_scaling(const DvfsState& state) const {
    // P ∝ C·V²·f — normalized to nominal
    double v_ratio = state.voltage_v / nominal_voltage_v;
    double f_ratio = state.frequency_ghz / nominal_freq_ghz;
    return v_ratio * v_ratio * f_ratio;
}

// ── ThermalAnalyzer Constructor ─────────────────────────────────────────────

ThermalAnalyzer::ThermalAnalyzer(const ThermalConfig& cfg) : cfg_(cfg) {
    int n = cfg_.grid_nx * cfg_.grid_ny;
    grid_.resize(n);
    for (auto& cell : grid_) {
        cell.temperature = cfg_.package.t_ambient;
        cell.conductivity = cfg_.material.conductivity;
    }
}

// ── Power Setting Methods ───────────────────────────────────────────────────

void ThermalAnalyzer::set_power_from_cells(const PhysicalDesign& pd,
                                            const std::vector<double>& cell_powers_mw) {
    // Clear existing power
    for (auto& cell : grid_) cell.power_density = 0;

    double dx_um = cfg_.die_width_um / cfg_.grid_nx;
    double dy_um = cfg_.die_height_um / cfg_.grid_ny;
    double cell_area_m2 = (dx_um * 1e-6) * (dy_um * 1e-6);

    for (size_t i = 0; i < pd.cells.size() && i < cell_powers_mw.size(); i++) {
        if (!pd.cells[i].placed) continue;
        double cx = pd.cells[i].position.x;
        double cy = pd.cells[i].position.y;
        int gx = std::clamp((int)(cx / dx_um), 0, cfg_.grid_nx - 1);
        int gy = std::clamp((int)(cy / dy_um), 0, cfg_.grid_ny - 1);
        // Convert mW to W/m²
        grid_[gx * cfg_.grid_ny + gy].power_density += (cell_powers_mw[i] * 1e-3) / cell_area_m2;
    }
}

void ThermalAnalyzer::set_power_density(int gx, int gy, double watts_per_m2) {
    if (gx >= 0 && gx < cfg_.grid_nx && gy >= 0 && gy < cfg_.grid_ny) {
        grid_[gx * cfg_.grid_ny + gy].power_density = watts_per_m2;
    }
}

void ThermalAnalyzer::set_uniform_power(double total_power_w) {
    double die_area_m2 = (cfg_.die_width_um * 1e-6) * (cfg_.die_height_um * 1e-6);
    double pd = total_power_w / die_area_m2;
    for (auto& cell : grid_) cell.power_density = pd;
}

void ThermalAnalyzer::set_block_power(double x0_um, double y0_um,
                                       double x1_um, double y1_um, double power_w) {
    double dx_um = cfg_.die_width_um / cfg_.grid_nx;
    double dy_um = cfg_.die_height_um / cfg_.grid_ny;

    int gx0 = std::clamp((int)(x0_um / dx_um), 0, cfg_.grid_nx - 1);
    int gy0 = std::clamp((int)(y0_um / dy_um), 0, cfg_.grid_ny - 1);
    int gx1 = std::clamp((int)(x1_um / dx_um), 0, cfg_.grid_nx - 1);
    int gy1 = std::clamp((int)(y1_um / dy_um), 0, cfg_.grid_ny - 1);

    int block_cells = std::max(1, (gx1 - gx0 + 1) * (gy1 - gy0 + 1));
    double cell_area_m2 = (dx_um * 1e-6) * (dy_um * 1e-6);
    double pd_per_cell = (power_w / block_cells) / cell_area_m2;

    for (int x = gx0; x <= gx1; x++) {
        for (int y = gy0; y <= gy1; y++) {
            grid_[x * cfg_.grid_ny + y].power_density += pd_per_cell;
        }
    }
}

// ── SOR Iteration ───────────────────────────────────────────────────────────
// Finite-difference: T[i,j] = (1/(4k)) × (k×(T[i±1,j] + T[i,j±1]) + q×dx²/k)
// With SOR: T_new = (1-ω)×T_old + ω×T_computed

void ThermalAnalyzer::sor_iteration(std::vector<double>& T,
                                     const std::vector<double>& Q,
                                     double dx, double dy,
                                     double k, double omega) {
    int nx = cfg_.grid_nx, ny = cfg_.grid_ny;
    double dx2 = dx * dx;
    double dy2 = dy * dy;

    for (int i = 1; i < nx - 1; i++) {
        for (int j = 1; j < ny - 1; j++) {
            int idx = i * ny + j;
            // 5-point stencil: ∂²T/∂x² + ∂²T/∂y² = -q/k
            double t_new = (
                (T[(i-1)*ny + j] + T[(i+1)*ny + j]) / dx2 +
                (T[i*ny + (j-1)] + T[i*ny + (j+1)]) / dy2 +
                Q[idx] / k
            ) / (2.0/dx2 + 2.0/dy2);

            T[idx] = (1.0 - omega) * T[idx] + omega * t_new;
        }
    }
}

// ── Boundary Temperature (Robin/convective BC) ─────────────────────────────

double ThermalAnalyzer::boundary_temp(int gx, int gy, double total_power) const {
    // Package thermal model: T_boundary = T_ambient + P × R_total
    // With spatial variation based on distance from center
    double cx = cfg_.grid_nx / 2.0;
    double cy = cfg_.grid_ny / 2.0;
    double dist = std::sqrt((gx - cx) * (gx - cx) + (gy - cy) * (gy - cy));
    double max_dist = std::sqrt(cx * cx + cy * cy);
    // Edge cells cooler than center due to lateral heat spreading
    double edge_factor = 1.0 - 0.3 * (dist / max_dist);

    return cfg_.package.t_ambient +
           total_power * cfg_.package.total_resistance() * edge_factor;
}

// ── Steady-State Solver ─────────────────────────────────────────────────────

ThermalResult ThermalAnalyzer::solve_steady_state() {
    ThermalResult result;
    int nx = cfg_.grid_nx, ny = cfg_.grid_ny;
    int n = nx * ny;

    // Grid spacing (meters)
    double dx = (cfg_.die_width_um * 1e-6) / nx;
    double dy = (cfg_.die_height_um * 1e-6) / ny;
    double k = cfg_.material.conductivity;
    double thickness_m = cfg_.die_thickness_um * 1e-6;

    // Initialize temperature and power arrays
    std::vector<double> T(n);
    std::vector<double> Q(n);

    // Total power for boundary conditions
    double total_power = 0;
    double cell_area_m2 = dx * dy;
    for (int i = 0; i < n; i++) {
        Q[i] = grid_[i].power_density * thickness_m; // volumetric → per-area
        total_power += grid_[i].power_density * cell_area_m2;
    }
    result.total_power_w = total_power;

    // Initialize with boundary estimate
    for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
            T[i * ny + j] = boundary_temp(i, j, total_power);
        }
    }

    // Set boundary conditions (convective: fixed at package model estimate)
    auto set_boundaries = [&]() {
        for (int i = 0; i < nx; i++) {
            T[i * ny + 0]        = boundary_temp(i, 0, total_power);
            T[i * ny + (ny - 1)] = boundary_temp(i, ny - 1, total_power);
        }
        for (int j = 0; j < ny; j++) {
            T[0 * ny + j]        = boundary_temp(0, j, total_power);
            T[(nx-1) * ny + j]   = boundary_temp(nx - 1, j, total_power);
        }
    };
    set_boundaries();

    // SOR iterations
    double omega = cfg_.sor_omega;
    for (int iter = 0; iter < cfg_.max_iterations; iter++) {
        std::vector<double> T_old = T;

        sor_iteration(T, Q, dx, dy, k, omega);
        set_boundaries();

        // Check convergence: max|T_new - T_old| < tol
        double max_diff = 0;
        for (int i = 0; i < n; i++) {
            max_diff = std::max(max_diff, std::abs(T[i] - T_old[i]));
        }

        result.iterations = iter + 1;
        if (max_diff < cfg_.convergence_tol) {
            result.converged = true;
            break;
        }
    }

    // Extract results
    result.temperature_map = T;
    result.grid_nx = nx;
    result.grid_ny = ny;

    result.max_temperature = *std::max_element(T.begin(), T.end());
    result.min_temperature = *std::min_element(T.begin(), T.end());
    result.avg_temperature = std::accumulate(T.begin(), T.end(), 0.0) / n;
    result.thermal_gradient = result.max_temperature - result.min_temperature;

    // Update grid
    for (int i = 0; i < n; i++) grid_[i].temperature = T[i];

    // Find hotspots
    result.hotspots = find_hotspots(result.avg_temperature + 0.5 * result.thermal_gradient);
    result.num_hotspots = (int)result.hotspots.size();

    last_result_ = result;
    return result;
}

// ── Transient Solver ────────────────────────────────────────────────────────
// Forward Euler: T(t+dt) = T(t) + dt × α × (∇²T + q/(ρ·cp))
// where α = k/(ρ·cp) is thermal diffusivity

std::vector<ThermalResult> ThermalAnalyzer::solve_transient(int steps) {
    if (steps < 0) steps = cfg_.transient_steps;
    std::vector<ThermalResult> results;

    int nx = cfg_.grid_nx, ny = cfg_.grid_ny;
    int n = nx * ny;
    double dx = (cfg_.die_width_um * 1e-6) / nx;
    double dy = (cfg_.die_height_um * 1e-6) / ny;
    double dt = cfg_.time_step_us * 1e-6; // convert to seconds
    double alpha = cfg_.material.diffusivity();
    double rho_cp = cfg_.material.density * cfg_.material.specific_heat;
    double thickness_m = cfg_.die_thickness_um * 1e-6;

    // CFL stability check: dt < dx²/(4α)
    double dt_max = (dx * dx) / (4.0 * alpha);
    if (dt > dt_max) {
        dt = dt_max * 0.9; // ensure stability
    }

    std::vector<double> T(n);
    // Initialize at ambient
    for (int i = 0; i < n; i++) T[i] = cfg_.package.t_ambient;

    // Power source term
    std::vector<double> Q_src(n);
    double total_power = 0;
    double cell_area = dx * dy;
    for (int i = 0; i < n; i++) {
        Q_src[i] = grid_[i].power_density * thickness_m / rho_cp;
        total_power += grid_[i].power_density * cell_area;
    }

    for (int step = 0; step < steps; step++) {
        std::vector<double> T_new = T;

        // Forward Euler update
        for (int i = 1; i < nx - 1; i++) {
            for (int j = 1; j < ny - 1; j++) {
                int idx = i * ny + j;
                double laplacian =
                    (T[(i-1)*ny+j] - 2*T[idx] + T[(i+1)*ny+j]) / (dx*dx) +
                    (T[i*ny+(j-1)] - 2*T[idx] + T[i*ny+(j+1)]) / (dy*dy);

                T_new[idx] = T[idx] + dt * (alpha * laplacian + Q_src[idx]);
            }
        }

        // Boundary: convective
        for (int i = 0; i < nx; i++) {
            T_new[i * ny + 0]        = boundary_temp(i, 0, total_power);
            T_new[i * ny + (ny - 1)] = boundary_temp(i, ny - 1, total_power);
        }
        for (int j = 0; j < ny; j++) {
            T_new[0 * ny + j]        = boundary_temp(0, j, total_power);
            T_new[(nx-1) * ny + j]   = boundary_temp(nx - 1, j, total_power);
        }

        T = T_new;

        // Record result at intervals
        if (step % std::max(1, steps / 10) == 0 || step == steps - 1) {
            ThermalResult r;
            r.temperature_map = T;
            r.grid_nx = nx;
            r.grid_ny = ny;
            r.max_temperature = *std::max_element(T.begin(), T.end());
            r.min_temperature = *std::min_element(T.begin(), T.end());
            r.avg_temperature = std::accumulate(T.begin(), T.end(), 0.0) / n;
            r.thermal_gradient = r.max_temperature - r.min_temperature;
            r.total_power_w = total_power;
            r.iterations = step + 1;
            r.converged = true;
            results.push_back(r);
        }
    }

    // Update grid with final temps
    for (int i = 0; i < n; i++) grid_[i].temperature = T[i];

    return results;
}

// ── 3D Multi-Die Thermal ────────────────────────────────────────────────────
// Simplified layer-coupled model: each die is a 2D thermal problem,
// coupled via inter-die thermal resistance (TIM + TSV thermal conductance)

ThermalResult ThermalAnalyzer::solve_3d_stack(const std::vector<double>& die_powers_w,
                                               int num_dies) {
    if (num_dies <= 0 || die_powers_w.empty()) return {};

    int actual_dies = std::min(num_dies, (int)die_powers_w.size());
    double r_tim = 0.5; // TIM thermal resistance between dies (°C/W)
    double t_ambient = cfg_.package.t_ambient;

    // Solve from bottom die (closest to heatsink) upward
    // T_die[i] = T_die[i-1] + P_die[i] × R_tim + P_above × R_package
    std::vector<double> die_temps(actual_dies);

    // Bottom die: directly on package
    double total_above = 0;
    for (int i = 0; i < actual_dies; i++) total_above += die_powers_w[i];

    die_temps[0] = cfg_.package.junction_temp(total_above);

    // Upper dies: cumulative thermal resistance
    for (int i = 1; i < actual_dies; i++) {
        double power_above = 0;
        for (int j = i; j < actual_dies; j++) power_above += die_powers_w[j];
        die_temps[i] = die_temps[i - 1] + power_above * r_tim;
    }

    // Now solve 2D thermal for each die with adjusted boundary
    ThermalResult combined;
    combined.min_temperature = 1e9;
    combined.max_temperature = -1e9;
    combined.total_power_w = 0;
    combined.converged = true;
    combined.grid_nx = cfg_.grid_nx;
    combined.grid_ny = cfg_.grid_ny;

    for (int d = 0; d < actual_dies; d++) {
        ThermalConfig die_cfg = cfg_;
        die_cfg.package.t_ambient = die_temps[d] - die_powers_w[d] * cfg_.package.total_resistance();

        ThermalAnalyzer die_solver(die_cfg);
        die_solver.set_uniform_power(die_powers_w[d]);
        auto r = die_solver.solve_steady_state();

        combined.min_temperature = std::min(combined.min_temperature, r.min_temperature);
        combined.max_temperature = std::max(combined.max_temperature, r.max_temperature);
        combined.total_power_w += r.total_power_w;
        if (!r.converged) combined.converged = false;
        combined.iterations = std::max(combined.iterations, r.iterations);

        // Keep the hottest die's temperature map
        if (r.max_temperature >= combined.max_temperature) {
            combined.temperature_map = r.temperature_map;
        }
        for (const auto& hs : r.hotspots) combined.hotspots.push_back(hs);
    }

    combined.avg_temperature = (combined.max_temperature + combined.min_temperature) / 2.0;
    combined.thermal_gradient = combined.max_temperature - combined.min_temperature;
    combined.num_hotspots = (int)combined.hotspots.size();

    return combined;
}

// ── Hotspot Detection ───────────────────────────────────────────────────────

std::vector<Hotspot> ThermalAnalyzer::find_hotspots(double threshold_c) const {
    std::vector<Hotspot> hotspots;
    double dx_um = cfg_.die_width_um / cfg_.grid_nx;
    double dy_um = cfg_.die_height_um / cfg_.grid_ny;

    for (int i = 0; i < cfg_.grid_nx; i++) {
        for (int j = 0; j < cfg_.grid_ny; j++) {
            int idx = i * cfg_.grid_ny + j;
            if (grid_[idx].temperature > threshold_c) {
                Hotspot hs;
                hs.grid_x = i;
                hs.grid_y = j;
                hs.x_um = (i + 0.5) * dx_um;
                hs.y_um = (j + 0.5) * dy_um;
                hs.temperature = grid_[idx].temperature;
                hs.power_density = grid_[idx].power_density;
                hotspots.push_back(hs);
            }
        }
    }

    // Sort by temperature descending
    std::sort(hotspots.begin(), hotspots.end(),
              [](const Hotspot& a, const Hotspot& b) {
                  return a.temperature > b.temperature;
              });

    return hotspots;
}

// ── DVFS Thermal Management ─────────────────────────────────────────────────

DvfsState ThermalAnalyzer::compute_dvfs(double max_temp_c,
                                         const DvfsConfig& dvfs) const {
    return dvfs.compute_state(max_temp_c);
}

// ── Sensor Network ──────────────────────────────────────────────────────────

void ThermalAnalyzer::place_sensors(int num_sensors) {
    sensors_.clear();
    if (num_sensors <= 0) return;

    // Place sensors in a grid pattern across die
    int side = (int)std::ceil(std::sqrt(num_sensors));
    int placed = 0;
    double dx_um = cfg_.die_width_um / (side + 1);
    double dy_um = cfg_.die_height_um / (side + 1);

    for (int i = 1; i <= side && placed < num_sensors; i++) {
        for (int j = 1; j <= side && placed < num_sensors; j++) {
            ThermalSensor s;
            s.id = placed;
            s.x = (float)(i * dx_um);
            s.y = (float)(j * dy_um);
            s.grid_x = std::clamp((int)(s.x / (cfg_.die_width_um / cfg_.grid_nx)),
                                  0, cfg_.grid_nx - 1);
            s.grid_y = std::clamp((int)(s.y / (cfg_.die_height_um / cfg_.grid_ny)),
                                  0, cfg_.grid_ny - 1);
            s.reading = grid_[s.grid_x * cfg_.grid_ny + s.grid_y].temperature;
            sensors_.push_back(s);
            placed++;
        }
    }
}

std::vector<ThermalSensor> ThermalAnalyzer::read_sensors() const {
    std::vector<ThermalSensor> readings = sensors_;
    for (auto& s : readings) {
        if (s.grid_x >= 0 && s.grid_x < cfg_.grid_nx &&
            s.grid_y >= 0 && s.grid_y < cfg_.grid_ny) {
            s.reading = grid_[s.grid_x * cfg_.grid_ny + s.grid_y].temperature;
        }
    }
    return readings;
}

// ── Coupled Electro-Thermal ─────────────────────────────────────────────────

double ThermalAnalyzer::timing_derating(double temp_c) const {
    // Mobility degrades with temperature: μ ∝ T^(-1.5)
    // Delay scales as: delay_ratio = (T/T_ref)^1.5
    // At 25°C (ref), ratio = 1.0. At 125°C, ratio ≈ 1.65
    double t_ref = 298.15; // 25°C in K
    double t_k = temp_c + 273.15;
    return std::pow(t_k / t_ref, 1.5);
}

double ThermalAnalyzer::leakage_scaling(double temp_c, double ref_temp) const {
    // Subthreshold leakage doubles every ~10°C (empirical)
    // I_leak ∝ exp((T - T_ref) / T_scale)
    double t_scale = 10.0 / std::log(2.0); // ≈ 14.4°C for 2× per 10°C
    return std::exp((temp_c - ref_temp) / t_scale);
}

// ── Enhanced: Thermal-Driven Placement ──────────────────────────────

ThermalPlacementResult ThermalAnalyzer::thermal_driven_placement() {
    ThermalPlacementResult result{};

    auto baseline = solve_steady_state();
    result.peak_temp_before = baseline.max_temperature;

    if (!pd_ || pd_->cells.empty()) {
        result.peak_temp_after = result.peak_temp_before;
        return result;
    }

    int nx = cfg_.grid_nx, ny = cfg_.grid_ny;
    double dx_um = cfg_.die_width_um / nx;
    double dy_um = cfg_.die_height_um / ny;

    // Find hotspot and coolspot grid cells
    double avg_temp = baseline.avg_temperature;
    std::vector<int> hot_cells, cool_cells;
    for (int i = 0; i < nx * ny; ++i) {
        if (grid_[i].temperature > avg_temp + 0.5 * baseline.thermal_gradient)
            hot_cells.push_back(i);
        else if (grid_[i].temperature < avg_temp - 0.25 * baseline.thermal_gradient)
            cool_cells.push_back(i);
    }

    // Move power from hot to cool regions
    int swaps = std::min(hot_cells.size(), cool_cells.size());
    for (int s = 0; s < swaps; ++s) {
        double hot_power = grid_[hot_cells[s]].power_density;
        double cool_power = grid_[cool_cells[s]].power_density;
        double transfer = (hot_power - cool_power) * 0.3;
        grid_[hot_cells[s]].power_density -= transfer;
        grid_[cool_cells[s]].power_density += transfer;
        result.cells_moved++;
    }

    auto after = solve_steady_state();
    result.peak_temp_after = after.max_temperature;
    result.improvement_pct = result.peak_temp_before > 0 ?
        (result.peak_temp_before - result.peak_temp_after) / result.peak_temp_before * 100.0 : 0;
    return result;
}

// ── Enhanced: Set Package Thermal Model ──────────────────────────────

void ThermalAnalyzer::set_package_thermal(const EnhancedPackageThermalModel& pkg) {
    enhanced_pkg_ = pkg;
    cfg_.package.t_ambient = pkg.ambient_temp_c;
    cfg_.package.r_jc = pkg.theta_jc;
    cfg_.package.r_ca = pkg.theta_ca;
}

// ── Enhanced: Transient Analysis ─────────────────────────────────────

TransientThermalResult ThermalAnalyzer::analyze_transient(double total_time_s, int steps) {
    TransientThermalResult result;
    int nx = cfg_.grid_nx, ny = cfg_.grid_ny;
    int n = nx * ny;

    double r_th = cfg_.package.total_resistance();
    double die_area_m2 = (cfg_.die_width_um * 1e-6) * (cfg_.die_height_um * 1e-6);
    double total_power = 0;
    for (auto& cell : grid_)
        total_power += cell.power_density * die_area_m2 / n;

    // Thermal capacitance: C_th = ρ × cp × volume
    double volume = die_area_m2 * cfg_.die_thickness_um * 1e-6;
    double c_th = cfg_.material.density * cfg_.material.specific_heat * volume;

    // Time constant: τ = R_th × C_th
    result.thermal_time_constant_ms = r_th * c_th * 1000.0;

    double dt = total_time_s / steps;
    double temp = cfg_.package.t_ambient;
    double t_amb = cfg_.package.t_ambient;

    // Euler integration: C×dT/dt = P - (T-Tamb)/R_th
    for (int s = 0; s <= steps; ++s) {
        double t = s * dt;
        result.time_points.push_back(t);
        result.peak_temps.push_back(temp);

        if (s < steps) {
            double heat_in = total_power;
            double heat_out = (r_th > 0) ? (temp - t_amb) / r_th : 0;
            double dT = (c_th > 0) ? dt * (heat_in - heat_out) / c_th : 0;
            temp += dT;
        }
    }

    result.steady_state_temp = t_amb + total_power * r_th;
    return result;
}

// ── Enhanced: Thermal Via Insertion ───────────────────────────────────

ThermalViaResult ThermalAnalyzer::insert_thermal_vias(double hotspot_threshold_c) {
    ThermalViaResult result{};
    auto hotspots = find_hotspots(hotspot_threshold_c);

    double dx_um = cfg_.die_width_um / cfg_.grid_nx;
    double dy_um = cfg_.die_height_um / cfg_.grid_ny;
    double via_conductivity_boost = 0.15;

    for (auto& hs : hotspots) {
        int idx = hs.grid_x * cfg_.grid_ny + hs.grid_y;
        // Insert thermal via: boost local conductivity
        grid_[idx].conductivity += copper_material().conductivity * via_conductivity_boost;
        result.vias_inserted++;
        result.via_locations.push_back({hs.x_um, hs.y_um});
    }

    if (!hotspots.empty()) {
        auto after = solve_steady_state();
        double before_max = hotspots[0].temperature;
        result.temp_reduction_c = before_max - after.max_temperature;
    }
    return result;
}

// ── Enhanced: Power-Thermal Iteration ────────────────────────────────

PowerThermalResult ThermalAnalyzer::iterate_power_thermal(int max_iter) {
    PowerThermalResult result{};
    double tol = 0.1;  // °C convergence

    double die_area_m2 = (cfg_.die_width_um * 1e-6) * (cfg_.die_height_um * 1e-6);
    int n = cfg_.grid_nx * cfg_.grid_ny;
    double cell_area = die_area_m2 / n;

    double prev_temp = 0;
    for (int iter = 0; iter < max_iter; ++iter) {
        // 1. Compute thermal
        auto thermal = solve_steady_state();
        result.converged_temp = thermal.max_temperature;

        // 2. Compute total power
        double total_power = 0;
        for (auto& cell : grid_)
            total_power += cell.power_density * cell_area;
        result.converged_power = total_power;

        // 3. Check convergence
        result.iterations = iter + 1;
        if (iter > 0 && std::abs(result.converged_temp - prev_temp) < tol) {
            result.converged = true;
            break;
        }
        prev_temp = result.converged_temp;

        // 4. Adjust leakage power based on temperature (temp-dependent)
        for (auto& cell : grid_) {
            double scale = leakage_scaling(cell.temperature, 25.0);
            // Leakage ~ 30% of total power, scale that portion
            double leakage_fraction = 0.3;
            cell.power_density *= (1.0 - leakage_fraction + leakage_fraction * scale);
        }
    }
    return result;
}

// ── Enhanced: Full Enhanced Thermal Run ───────────────────────────────

ThermalResult ThermalAnalyzer::run_enhanced() {
    auto result = solve_steady_state();

    auto transient = analyze_transient(1.0, 50);
    auto vias = insert_thermal_vias(result.avg_temperature + result.thermal_gradient * 0.8);
    auto pt = iterate_power_thermal(5);

    // Re-solve after all enhancements
    auto final_result = solve_steady_state();
    final_result.num_hotspots = (int)find_hotspots(
        final_result.avg_temperature + 0.5 * final_result.thermal_gradient).size();

    return final_result;
}

} // namespace sf
