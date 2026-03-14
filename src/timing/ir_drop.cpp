// SiliconForge — IR Drop Analyzer Industrial Implementation
// Sparse-matrix Gauss-Seidel solver with SOR, configurable pads, dynamic IR drop.
//
// Algorithm: Build resistive mesh → inject current sources → solve V = R⁻¹ × I
// iteratively with Successive Over-Relaxation (Gauss-Seidel + ω damping).

#include "timing/ir_drop.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>

namespace sf {

// ── Per-cell current setter ──────────────────────────────────────────────

void IrDropAnalyzer::set_cell_current(int cell_idx, double current_ma) {
    if (cell_idx >= (int)cfg_.cell_currents_ma.size())
        cfg_.cell_currents_ma.resize(cell_idx + 1, 0);
    cfg_.cell_currents_ma[cell_idx] = current_ma;
}

// ── Power pad generation ─────────────────────────────────────────────────

void IrDropAnalyzer::generate_pads(int grid_res) {
    effective_pads_.clear();

    if (!cfg_.pads.empty() && cfg_.pad_pattern == IrDropConfig::CUSTOM) {
        effective_pads_ = cfg_.pads;
        return;
    }

    double x0 = pd_.die_area.x0, y0 = pd_.die_area.y0;
    double x1 = pd_.die_area.x1, y1 = pd_.die_area.y1;
    double pad_r = 0.01; // default pad resistance

    switch (cfg_.pad_pattern) {
    case IrDropConfig::CORNERS:
        effective_pads_.push_back({x0, y0, pad_r, PowerPad::VDD, "VDD_BL"});
        effective_pads_.push_back({x1, y0, pad_r, PowerPad::VDD, "VDD_BR"});
        effective_pads_.push_back({x0, y1, pad_r, PowerPad::VDD, "VDD_TL"});
        effective_pads_.push_back({x1, y1, pad_r, PowerPad::VDD, "VDD_TR"});
        break;

    case IrDropConfig::PERIMETER: {
        int n = std::max(1, cfg_.pads_per_side);
        for (int i = 0; i < n; ++i) {
            double t = (i + 0.5) / n;
            effective_pads_.push_back({x0 + t * (x1 - x0), y0, pad_r, PowerPad::VDD, ""});
            effective_pads_.push_back({x0 + t * (x1 - x0), y1, pad_r, PowerPad::VDD, ""});
            effective_pads_.push_back({x0, y0 + t * (y1 - y0), pad_r, PowerPad::VDD, ""});
            effective_pads_.push_back({x1, y0 + t * (y1 - y0), pad_r, PowerPad::VDD, ""});
        }
        break;
    }

    case IrDropConfig::RING: {
        int n = std::max(1, cfg_.pads_per_side);
        double margin_x = (x1 - x0) * 0.1;
        double margin_y = (y1 - y0) * 0.1;
        for (int i = 0; i < n; ++i) {
            double t = (i + 0.5) / n;
            // Outer ring
            effective_pads_.push_back({x0 + t * (x1 - x0), y0, pad_r, PowerPad::VDD, ""});
            effective_pads_.push_back({x0 + t * (x1 - x0), y1, pad_r, PowerPad::VDD, ""});
            effective_pads_.push_back({x0, y0 + t * (y1 - y0), pad_r, PowerPad::VDD, ""});
            effective_pads_.push_back({x1, y0 + t * (y1 - y0), pad_r, PowerPad::VDD, ""});
            // Inner ring (offset by margin)
            effective_pads_.push_back({x0 + margin_x + t * (x1 - x0 - 2*margin_x),
                                       y0 + margin_y, pad_r * 1.5, PowerPad::VDD, ""});
            effective_pads_.push_back({x0 + margin_x + t * (x1 - x0 - 2*margin_x),
                                       y1 - margin_y, pad_r * 1.5, PowerPad::VDD, ""});
        }
        break;
    }

    case IrDropConfig::CUSTOM:
        effective_pads_ = cfg_.pads;
        break;
    }
}

// ── Current map construction ─────────────────────────────────────────────

void IrDropAnalyzer::build_current_map(int N, double cell_w, double cell_h,
                                        std::vector<std::vector<double>>& current_map) {
    current_map.assign(N, std::vector<double>(N, 0));

    bool have_per_cell = !cfg_.cell_currents_ma.empty();
    double total_cell_area = 0;
    double total_specified_current = 0;

    if (!have_per_cell) {
        // Area-proportional model
        for (auto& c : pd_.cells) {
            if (!c.placed) continue;
            total_cell_area += c.width * c.height;
        }
    }

    for (int ci = 0; ci < (int)pd_.cells.size(); ++ci) {
        auto& c = pd_.cells[ci];
        if (!c.placed) continue;
        int gx = std::clamp((int)((c.position.x - pd_.die_area.x0) / cell_w), 0, N-1);
        int gy = std::clamp((int)((c.position.y - pd_.die_area.y0) / cell_h), 0, N-1);

        if (have_per_cell && ci < (int)cfg_.cell_currents_ma.size() && cfg_.cell_currents_ma[ci] > 0) {
            current_map[gy][gx] += cfg_.cell_currents_ma[ci];
            total_specified_current += cfg_.cell_currents_ma[ci];
        } else if (!have_per_cell && total_cell_area > 0) {
            current_map[gy][gx] += (c.width * c.height / total_cell_area) * cfg_.total_current_ma;
        }
    }

    // If per-cell currents don't sum to total, distribute remainder proportionally
    if (have_per_cell && total_specified_current < cfg_.total_current_ma * 0.99) {
        double remainder = cfg_.total_current_ma - total_specified_current;
        double total_area = 0;
        for (auto& c : pd_.cells) if (c.placed) total_area += c.width * c.height;
        if (total_area > 0) {
            for (int ci = 0; ci < (int)pd_.cells.size(); ++ci) {
                auto& c = pd_.cells[ci];
                if (!c.placed) continue;
                if (ci < (int)cfg_.cell_currents_ma.size() && cfg_.cell_currents_ma[ci] > 0) continue;
                int gx = std::clamp((int)((c.position.x - pd_.die_area.x0) / cell_w), 0, N-1);
                int gy = std::clamp((int)((c.position.y - pd_.die_area.y0) / cell_h), 0, N-1);
                current_map[gy][gx] += (c.width * c.height / total_area) * remainder;
            }
        }
    }
}

// ── Gauss-Seidel Sparse Solver with SOR ──────────────────────────────────
//
// Resistive mesh model:
//   Each grid node (i,j) is connected to its 4 neighbors by R_sheet.
//   Power pads connect to nearest grid node(s) with R_pad.
//   Current source at each node = local current draw (mA).
//
// KCL at node (i,j):
//   sum_neighbors[ (V_neighbor - V_ij) / R_sheet ] - I_ij + I_pad_ij = 0
//
// Rearranged for Gauss-Seidel:
//   V_ij = (sum_neighbors(V_neighbor/R_sheet) + I_pad_ij - I_ij) / sum_neighbors(1/R_sheet)

IrDropAnalyzer::SolverResult
IrDropAnalyzer::solve_gauss_seidel(int N, double cell_w, double cell_h,
                                    const std::vector<std::vector<double>>& current_map) {
    SolverResult sr;
    sr.voltage.assign(N, std::vector<double>(N, cfg_.vdd));
    sr.converged = false;

    // Conductance between adjacent grid nodes: G = 1/R
    // R_sheet = sheet_resistance * (cell_w / cell_h) for horizontal,
    // or (cell_h / cell_w) for vertical. For square cells, R = R_sheet.
    double r_h = cfg_.sheet_resistance_mohm * 1e-3 * cell_w / cell_h; // Ohm
    double r_v = cfg_.sheet_resistance_mohm * 1e-3 * cell_h / cell_w;
    double g_h = (r_h > 0) ? 1.0 / r_h : 1e6;
    double g_v = (r_v > 0) ? 1.0 / r_v : 1e6;

    // Map power pads to grid nodes
    struct PadNode { int gx, gy; double conductance; };
    std::vector<PadNode> pad_nodes;
    for (auto& pad : effective_pads_) {
        if (pad.type != PowerPad::VDD) continue;
        int gx = std::clamp((int)((pad.x - pd_.die_area.x0) / cell_w), 0, N-1);
        int gy = std::clamp((int)((pad.y - pd_.die_area.y0) / cell_h), 0, N-1);
        double g_pad = (pad.resistance_ohm > 0) ? 1.0 / pad.resistance_ohm : 1e6;
        pad_nodes.push_back({gx, gy, g_pad});
    }

    // Build per-node pad conductance (for pads, V_ideal = VDD through R_pad)
    std::vector<std::vector<double>> g_pad_map(N, std::vector<double>(N, 0));
    for (auto& pn : pad_nodes) {
        g_pad_map[pn.gy][pn.gx] += pn.conductance;
    }

    // Convert current map from mA to A
    std::vector<std::vector<double>> i_map(N, std::vector<double>(N, 0));
    for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x)
            i_map[y][x] = current_map[y][x] * 1e-3; // mA → A

    double omega = cfg_.sor_omega;

    for (int iter = 0; iter < cfg_.max_iterations; ++iter) {
        double max_change = 0;

        for (int y = 0; y < N; ++y) {
            for (int x = 0; x < N; ++x) {
                double sum_gv = 0; // sum of (G_neighbor * V_neighbor)
                double sum_g = 0;  // sum of G_neighbor

                // Left neighbor
                if (x > 0) { sum_gv += g_h * sr.voltage[y][x-1]; sum_g += g_h; }
                // Right neighbor
                if (x < N-1) { sum_gv += g_h * sr.voltage[y][x+1]; sum_g += g_h; }
                // Bottom neighbor
                if (y > 0) { sum_gv += g_v * sr.voltage[y-1][x]; sum_g += g_v; }
                // Top neighbor
                if (y < N-1) { sum_gv += g_v * sr.voltage[y+1][x]; sum_g += g_v; }

                // Pad connection: current source from VDD through pad resistance
                double g_pad = g_pad_map[y][x];
                if (g_pad > 0) {
                    sum_gv += g_pad * cfg_.vdd;
                    sum_g += g_pad;
                }

                if (sum_g == 0) continue;

                // KCL: V_new = (sum_gv - I_draw) / sum_g
                double v_new = (sum_gv - i_map[y][x]) / sum_g;
                v_new = std::max(0.0, std::min(v_new, cfg_.vdd)); // clamp

                // SOR update
                double v_sor = sr.voltage[y][x] + omega * (v_new - sr.voltage[y][x]);
                double change = std::abs(v_sor - sr.voltage[y][x]);
                max_change = std::max(max_change, change);
                sr.voltage[y][x] = v_sor;
            }
        }

        sr.iterations = iter + 1;
        sr.residual = max_change;

        if (max_change < cfg_.convergence_tol) {
            sr.converged = true;
            break;
        }
    }

    return sr;
}

// ── Dynamic IR Drop ──────────────────────────────────────────────────────
// Peak transient drop from simultaneous switching (worst-case cycle).
// Model: Vdyn_drop = I_peak × R_eff − Q_decap / C_decap
// where I_peak = switching_factor × I_total (all cells switching in one cycle)

void IrDropAnalyzer::compute_dynamic_drop(int N, double cell_w, double cell_h,
                                           const std::vector<std::vector<double>>& current_map,
                                           IrDropResult& result) {
    result.dynamic_map.assign(N, std::vector<double>(N, 0));
    result.dynamic_analyzed = true;

    double sw_factor = cfg_.switching_factor;
    double decap_density = cfg_.decap_density_ff_per_um2;
    double period_ns = cfg_.clock_period_ns;

    double worst_dyn = 0, total_dyn = 0;
    int count = 0;

    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            // Peak switching current = sw_factor × static_current × 2 (both edges)
            double i_peak_ma = current_map[y][x] * sw_factor * 2.0;

            // Local decap charge: Q = C × V, C = decap_density × cell_area
            double local_area = cell_w * cell_h;
            double c_decap_ff = decap_density * local_area;
            double q_decap = c_decap_ff * 1e-15 * cfg_.vdd; // Coulombs
            double i_decap_ma = (period_ns > 0) ? (q_decap / (period_ns * 1e-9)) * 1e3 : 0;

            // Net peak current after decap compensation
            double i_net = std::max(0.0, i_peak_ma - i_decap_ma);

            // Dynamic drop = static_drop_ratio × i_net/i_static
            double static_drop = result.drop_map[y][x];
            double i_static = current_map[y][x];
            double dyn_drop = (i_static > 0) ? static_drop * (i_net / i_static) : 0;

            result.dynamic_map[y][x] = dyn_drop;
            worst_dyn = std::max(worst_dyn, dyn_drop);
            total_dyn += dyn_drop;
            count++;
        }
    }

    result.worst_dynamic_drop_mv = worst_dyn;
    result.avg_dynamic_drop_mv = (count > 0) ? total_dyn / count : 0;
}

// ── Timing Derating ──────────────────────────────────────────────────────
// Cell delay increases when supply voltage drops.
// Approximate: Δdelay/delay ≈ sensitivity × (Vdrop / VDD)

void IrDropAnalyzer::compute_timing_derating(IrDropResult& result) {
    double sensitivity = cfg_.timing_derate_sensitivity;
    double vdd_mv = cfg_.vdd * 1000;
    double worst_derate = 0, total_derate = 0;
    int count = 0;

    for (auto& node : result.nodes) {
        double drop_mv = cfg_.vdd * 1000 - node.voltage_mv;
        double derate_pct = sensitivity * (drop_mv / vdd_mv) * 100.0;
        node.timing_derate_pct = derate_pct;
        worst_derate = std::max(worst_derate, derate_pct);
        total_derate += derate_pct;
        count++;
    }

    result.worst_timing_derate_pct = worst_derate;
    result.avg_timing_derate_pct = (count > 0) ? total_derate / count : 0;
}

// ── Signoff Evaluation ───────────────────────────────────────────────────

void IrDropAnalyzer::evaluate_signoff(IrDropResult& result) {
    double vdd_mv = cfg_.vdd * 1000;
    double worst_pct = (vdd_mv > 0) ? result.worst_drop_mv / vdd_mv * 100.0 : 0;
    double dyn_pct = (vdd_mv > 0) ? result.worst_dynamic_drop_mv / vdd_mv * 100.0 : 0;

    bool static_pass = worst_pct < cfg_.hotspot_threshold_pct;
    bool dynamic_pass = !result.dynamic_analyzed || dyn_pct < cfg_.critical_threshold_pct;
    bool timing_pass = result.worst_timing_derate_pct < 5.0; // 5% derate limit

    result.signoff_pass = static_pass && dynamic_pass && timing_pass;
    result.signoff_summary = "Static: " + std::to_string((int)worst_pct) + "% (" +
        (static_pass ? "PASS" : "FAIL") + "), Dynamic: " +
        std::to_string((int)dyn_pct) + "% (" +
        (dynamic_pass ? "PASS" : "FAIL") + "), Timing derate: " +
        std::to_string((int)result.worst_timing_derate_pct) + "% (" +
        (timing_pass ? "PASS" : "FAIL") + ")";
}

// ── Main Analysis (Industrial) ───────────────────────────────────────────

IrDropResult IrDropAnalyzer::analyze(int grid_res) {
    auto t0 = std::chrono::high_resolution_clock::now();
    IrDropResult r;
    r.vdd = cfg_.vdd;
    r.threshold_pct = cfg_.hotspot_threshold_pct;

    int N = (grid_res > 0) ? grid_res : cfg_.grid_resolution;
    r.grid_x = N;
    r.grid_y = N;

    double cell_w = pd_.die_area.width() / N;
    double cell_h = pd_.die_area.height() / N;

    // Generate power pads
    generate_pads(N);

    // Build current demand map
    std::vector<std::vector<double>> current_map;
    build_current_map(N, cell_w, cell_h, current_map);

    // Solve resistive mesh with Gauss-Seidel + SOR
    auto sr = solve_gauss_seidel(N, cell_w, cell_h, current_map);
    r.solver_iterations = sr.iterations;
    r.solver_residual = sr.residual;
    r.converged = sr.converged;

    // Extract results from solver
    r.drop_map.resize(N, std::vector<double>(N, 0));
    r.voltage_map.resize(N, std::vector<double>(N, 0));
    double worst_drop = 0, total_drop = 0;
    std::vector<double> all_drops;

    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            double voltage_v = sr.voltage[y][x];
            double drop_mv = (cfg_.vdd - voltage_v) * 1000.0;
            drop_mv = std::max(0.0, drop_mv);

            r.drop_map[y][x] = drop_mv;
            r.voltage_map[y][x] = voltage_v * 1000.0;
            worst_drop = std::max(worst_drop, drop_mv);
            total_drop += drop_mv;
            all_drops.push_back(drop_mv);

            // Build node data
            double px = pd_.die_area.x0 + (x + 0.5) * cell_w;
            double py = pd_.die_area.y0 + (y + 0.5) * cell_h;
            IrDropNode node;
            node.x = px;
            node.y = py;
            node.static_drop_mv = drop_mv;
            node.voltage_mv = voltage_v * 1000.0;
            node.current_ma = current_map[y][x];
            r.nodes.push_back(node);

            // Check hotspots
            double drop_pct = (cfg_.vdd > 0) ? drop_mv / (cfg_.vdd * 1000) * 100.0 : 0;
            if (drop_pct > cfg_.hotspot_threshold_pct) {
                Rect region(pd_.die_area.x0 + x * cell_w, pd_.die_area.y0 + y * cell_h,
                           pd_.die_area.x0 + (x+1) * cell_w, pd_.die_area.y0 + (y+1) * cell_h);
                IrDropHotSpot::Severity sev = (drop_pct > cfg_.critical_threshold_pct) ?
                    IrDropHotSpot::CRITICAL : IrDropHotSpot::WARNING;
                double derate = cfg_.timing_derate_sensitivity * (drop_pct / 100.0) * 100.0;
                r.hotspots.push_back({region, drop_mv, current_map[y][x], derate, sev});
                r.num_hotspots++;
                if (sev == IrDropHotSpot::CRITICAL) r.num_critical++;
            }
        }
    }

    r.worst_drop_mv = worst_drop;
    int total_nodes = N * N;
    r.avg_drop_mv = (total_nodes > 0) ? total_drop / total_nodes : 0;

    // Median
    if (!all_drops.empty()) {
        std::sort(all_drops.begin(), all_drops.end());
        r.median_drop_mv = all_drops[all_drops.size() / 2];
    }

    // Dynamic IR drop
    if (cfg_.enable_dynamic) {
        compute_dynamic_drop(N, cell_w, cell_h, current_map, r);
    }

    // Timing derating
    if (cfg_.compute_timing_derating) {
        compute_timing_derating(r);
    }

    // Signoff evaluation
    evaluate_signoff(r);

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "IR Drop: worst=" + std::to_string((int)r.worst_drop_mv) + "mV (" +
                std::to_string((int)(r.worst_drop_mv / (cfg_.vdd * 10))) + "%), " +
                std::to_string(r.num_hotspots) + " hotspot(s), " +
                std::to_string(sr.iterations) + " solver iters, " +
                (r.converged ? "converged" : "NOT converged") +
                (r.signoff_pass ? " [SIGNOFF PASS]" : " [SIGNOFF FAIL]");

    // Cache for enhanced methods
    last_result_ = r;
    has_result_ = true;
    last_grid_n_ = N;
    last_cell_w_ = cell_w;
    last_cell_h_ = cell_h;
    last_current_map_ = current_map;

    return r;
}

// ── Legacy API ───────────────────────────────────────────────────────────
// Backward-compatible with old analyze(grid_res) behavior

IrDropResult IrDropAnalyzer::analyze_legacy(int grid_res) {
    auto t0 = std::chrono::high_resolution_clock::now();
    IrDropResult r;
    r.vdd = cfg_.vdd;
    r.grid_x = grid_res;
    r.grid_y = grid_res;
    r.threshold_pct = 5.0;

    double cell_w = pd_.die_area.width() / grid_res;
    double cell_h = pd_.die_area.height() / grid_res;

    std::vector<std::vector<double>> current_map(grid_res, std::vector<double>(grid_res, 0));
    double total_cell_area = 0;
    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        total_cell_area += c.width * c.height;
        int gx = std::clamp((int)((c.position.x - pd_.die_area.x0) / cell_w), 0, grid_res-1);
        int gy = std::clamp((int)((c.position.y - pd_.die_area.y0) / cell_h), 0, grid_res-1);
        current_map[gy][gx] += c.width * c.height;
    }
    if (total_cell_area > 0) {
        for (auto& row : current_map)
            for (auto& val : row)
                val = (val / total_cell_area) * cfg_.total_current_ma;
    }

    r.drop_map.resize(grid_res, std::vector<double>(grid_res, 0));
    double worst_drop = 0, total_drop = 0;
    int total_cells = 0;

    for (int y = 0; y < grid_res; ++y) {
        for (int x = 0; x < grid_res; ++x) {
            double d_corner = std::min({
                std::sqrt((double)(x*x + y*y)),
                std::sqrt((double)((grid_res-1-x)*(grid_res-1-x) + y*y)),
                std::sqrt((double)(x*x + (grid_res-1-y)*(grid_res-1-y))),
                std::sqrt((double)((grid_res-1-x)*(grid_res-1-x) + (grid_res-1-y)*(grid_res-1-y)))
            });
            double local_current = current_map[y][x];
            double drop_v = local_current * 0.1 * d_corner * 0.001;
            double drop_mv = drop_v * 1000;

            r.drop_map[y][x] = drop_mv;
            worst_drop = std::max(worst_drop, drop_mv);
            total_drop += drop_mv;
            total_cells++;

            if (drop_mv > cfg_.vdd * 1000 * r.threshold_pct / 100.0) {
                Rect region(pd_.die_area.x0 + x * cell_w, pd_.die_area.y0 + y * cell_h,
                           pd_.die_area.x0 + (x+1) * cell_w, pd_.die_area.y0 + (y+1) * cell_h);
                r.hotspots.push_back({region, drop_mv, local_current, 0, IrDropHotSpot::WARNING});
                r.num_hotspots++;
            }
        }
    }

    r.worst_drop_mv = worst_drop;
    r.avg_drop_mv = total_cells > 0 ? total_drop / total_cells : 0;
    r.converged = true;
    r.signoff_pass = true;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Worst IR drop: " + std::to_string((int)r.worst_drop_mv) + "mV (" +
                std::to_string(r.worst_drop_mv / (cfg_.vdd * 10)) + "%), " +
                std::to_string(r.num_hotspots) + " hotspot(s)";
    return r;
}

// ══════════════════════════════════════════════════════════════════════
// Enhanced IR Drop Analysis
// ══════════════════════════════════════════════════════════════════════

// ── Dynamic IR drop (transient) ──────────────────────────────────────
// Time-step simulation: at each step, compute current draw from switching
// cells, solve resistive network for voltage drop at each node.

DynamicIrResult IrDropAnalyzer::analyze_dynamic(double time_step_ps, int num_steps) {
    // Ensure we have a base analysis
    if (!has_result_) analyze();

    DynamicIrResult result{};
    result.time_steps = num_steps;

    int N = last_grid_n_ > 0 ? last_grid_n_ : cfg_.grid_resolution;
    double cell_w = pd_.die_area.width() / N;
    double cell_h = pd_.die_area.height() / N;

    // Decoupling capacitance per grid cell
    double decap_per_cell_ff = cfg_.decap_density_ff_per_um2 * cell_w * cell_h;
    double decap_per_cell_f = decap_per_cell_ff * 1e-15;

    // ESR per decap cell (convert mΩ → Ω)
    double decap_esr_ohm = cfg_.decap_esr_mohm * 1e-3;

    // Per-cell inductance from PDN mesh layers
    // Total inductance per cell = per_layer_inductance × mesh_pitch / num_mesh_segments
    double mesh_segments = std::max(1.0, cell_w / cfg_.mesh_pitch_um);
    double cell_inductance_h = cfg_.per_layer_inductance_ph_per_um * cell_w / mesh_segments;

    // Package-level inductance and resistance (shared across pad nodes)
    double pkg_l_h = cfg_.pkg_inductance_nh * 1e-9;
    double pkg_r_ohm = cfg_.pkg_resistance_mohm * 1e-3;

    // Compute LC resonance frequency: f = 1/(2π√(LC))
    double total_decap_f = decap_per_cell_f * N * N;
    double effective_l_h = pkg_l_h + cell_inductance_h;
    if (total_decap_f > 0 && effective_l_h > 0) {
        double f_hz = 1.0 / (2.0 * M_PI * std::sqrt(effective_l_h * total_decap_f));
        result.resonance_freq_ghz = f_hz * 1e-9;
    }

    // Voltage state per node (starts at VDD)
    std::vector<std::vector<double>> node_voltage(N, std::vector<double>(N, cfg_.vdd));

    // Track previous-step current for L*di/dt computation
    std::vector<std::vector<double>> prev_current(N, std::vector<double>(N, 0));

    double peak_drop = 0.0;
    double total_drop_sum = 0.0;
    int total_measurements = 0;
    int worst_x = 0, worst_y = 0;
    double peak_ldi_dt_mv = 0.0;
    double peak_resistive_mv = 0.0;

    double time_step_s = time_step_ps * 1e-12;

    for (int step = 0; step < num_steps; ++step) {
        double t_ps = step * time_step_ps;

        // Switching activity varies sinusoidally (modeling clock edges)
        double phase = 2.0 * M_PI * step / std::max(num_steps, 1);
        double sw_factor = cfg_.switching_factor * (0.5 + 0.5 * std::sin(phase));

        // Per-node transient current
        double step_worst_drop = 0.0;
        double step_total_drop = 0.0;

        for (int y = 0; y < N; ++y) {
            for (int x = 0; x < N; ++x) {
                double i_static = has_result_ ? last_current_map_[y][x] : 0.0;
                double i_switching = i_static * sw_factor * 2.0; // both edges

                // Decap provides charge: ΔV = I × Δt / C
                double total_current_ma = i_static + i_switching;
                double total_current_a = total_current_ma * 1e-3;

                double delta_v = 0.0;
                if (decap_per_cell_f > 0) {
                    delta_v = total_current_a * time_step_s / decap_per_cell_f;
                }

                // ESR voltage drop across decap: V_esr = I × ESR
                double v_esr = total_current_a * decap_esr_ohm;

                // Static IR drop component from base analysis
                double static_drop = 0.0;
                if (has_result_ && y < (int)last_result_.drop_map.size() &&
                    x < (int)last_result_.drop_map[0].size()) {
                    static_drop = last_result_.drop_map[y][x];
                }

                // Resistive drop = static × current_ratio + ESR
                double i_ratio = (i_static > 1e-15) ? (i_static + i_switching) / i_static : 1.0;
                double resistive_drop_mv = static_drop * i_ratio + v_esr * 1000.0;

                // Package-level resistive contribution
                double pkg_resistive_mv = total_current_a * pkg_r_ohm * 1000.0;
                resistive_drop_mv += pkg_resistive_mv;

                // L*di/dt inductive noise
                double di_dt = 0.0;
                if (time_step_s > 0) {
                    di_dt = (total_current_a - prev_current[y][x] * 1e-3) / time_step_s;
                }
                double ldi_dt_v = effective_l_h * std::abs(di_dt);
                double ldi_dt_mv = ldi_dt_v * 1000.0;

                // Combined dynamic drop
                double dynamic_drop = resistive_drop_mv + delta_v * 1000.0 + ldi_dt_mv;
                dynamic_drop = std::min(dynamic_drop, cfg_.vdd * 1000.0);

                // Track peak contributions
                peak_ldi_dt_mv = std::max(peak_ldi_dt_mv, ldi_dt_mv);
                peak_resistive_mv = std::max(peak_resistive_mv, resistive_drop_mv);

                prev_current[y][x] = total_current_ma;

                if (dynamic_drop > step_worst_drop) {
                    step_worst_drop = dynamic_drop;
                    if (dynamic_drop > peak_drop) {
                        worst_x = x;
                        worst_y = y;
                    }
                }
                step_total_drop += dynamic_drop;
                total_measurements++;
            }
        }

        peak_drop = std::max(peak_drop, step_worst_drop);
        total_drop_sum += step_total_drop / (N * N);

        result.waveform.push_back({t_ps, step_worst_drop});
    }

    result.peak_drop_mv = peak_drop;
    result.avg_drop_mv = (num_steps > 0) ? total_drop_sum / num_steps : 0.0;
    result.worst_region = "grid(" + std::to_string(worst_x) + "," + std::to_string(worst_y) + ")";
    result.ldi_dt_peak_mv = peak_ldi_dt_mv;
    result.resistive_peak_mv = peak_resistive_mv;

    return result;
}

// ── VCD-driven vectored analysis ─────────────────────────────────────

VectoredIrResult IrDropAnalyzer::analyze_vectored(
    const std::vector<std::vector<bool>>& stimulus, double clock_period_ns) {
    if (!has_result_) analyze();

    VectoredIrResult result{};
    result.peak_drop_mv = 0;
    result.peak_cycle = 0;

    int N = last_grid_n_ > 0 ? last_grid_n_ : cfg_.grid_resolution;

    for (size_t cycle = 0; cycle < stimulus.size(); ++cycle) {
        // Count switching cells in this cycle
        int switching_count = 0;
        for (bool active : stimulus[cycle]) {
            if (active) switching_count++;
        }
        double sw_fraction = (stimulus[cycle].empty()) ? cfg_.switching_factor
            : static_cast<double>(switching_count) / stimulus[cycle].size();

        // Scale current map by switching fraction
        double cycle_worst = 0.0;
        for (int y = 0; y < N; ++y) {
            for (int x = 0; x < N; ++x) {
                double i_static = (has_result_ && y < (int)last_current_map_.size() &&
                                   x < (int)last_current_map_[0].size())
                                  ? last_current_map_[y][x] : 0.0;
                double i_dynamic = i_static * sw_fraction * 2.0;
                double static_drop = (has_result_ && y < (int)last_result_.drop_map.size() &&
                                      x < (int)last_result_.drop_map[0].size())
                                     ? last_result_.drop_map[y][x] : 0.0;
                double i_ratio = (i_static > 1e-15) ? (i_static + i_dynamic) / i_static : 1.0;
                double drop = static_drop * i_ratio;
                cycle_worst = std::max(cycle_worst, drop);
            }
        }

        result.per_cycle_drops.push_back(cycle_worst);
        if (cycle_worst > result.peak_drop_mv) {
            result.peak_drop_mv = cycle_worst;
            result.peak_cycle = static_cast<int>(cycle);
        }
    }

    result.message = "Vectored IR: peak=" + std::to_string((int)result.peak_drop_mv) +
                     "mV at cycle " + std::to_string(result.peak_cycle) +
                     " (" + std::to_string(stimulus.size()) + " cycles)";
    return result;
}

// ── Voltage-drop-aware STA ───────────────────────────────────────────
// Use IR drop map to compute per-gate voltage. Lower voltage → slower gate
// (delay ∝ V/(V−Vth)²). Report timing degradation.

VoltageAwareTimingResult IrDropAnalyzer::analyze_voltage_timing() {
    if (!has_result_) analyze();

    VoltageAwareTimingResult result{};
    double vdd = cfg_.vdd;
    double vth = vdd * 0.25;  // approximate threshold voltage

    // Nominal WNS: use worst timing derate from base analysis
    result.nominal_wns = 0.0;

    double worst_extra_delay = 0.0;
    double total_extra_delay = 0.0;
    int gate_count = 0;

    for (int ci = 0; ci < (int)pd_.cells.size(); ++ci) {
        auto& c = pd_.cells[ci];
        if (!c.placed) continue;

        int N = last_grid_n_ > 0 ? last_grid_n_ : cfg_.grid_resolution;
        double cell_w = pd_.die_area.width() / N;
        double cell_h = pd_.die_area.height() / N;
        int gx = std::clamp((int)((c.position.x - pd_.die_area.x0) / cell_w), 0, N - 1);
        int gy = std::clamp((int)((c.position.y - pd_.die_area.y0) / cell_h), 0, N - 1);

        double drop_mv = 0.0;
        if (has_result_ && gy < (int)last_result_.drop_map.size() &&
            gx < (int)last_result_.drop_map[0].size()) {
            drop_mv = last_result_.drop_map[gy][gx];
        }

        double v_actual = vdd - drop_mv / 1000.0;
        v_actual = std::max(v_actual, vth + 0.01);  // prevent divide by zero

        // Delay ratio: delay_actual/delay_nominal = (V_nom/(V_nom−Vth))² / (V_act/(V_act−Vth))²
        // Simplified: extra_delay_fraction = 2 × (drop/V) × V/(V−Vth)
        double drop_fraction = drop_mv / (vdd * 1000.0);
        double v_headroom = (v_actual - vth);
        double delay_increase_pct = cfg_.timing_derate_sensitivity * drop_fraction * 100.0;

        // More accurate alpha-power model: delay ∝ V/(V-Vth)^alpha, alpha≈1.3
        double nom_factor = vdd / ((vdd - vth) * (vdd - vth));
        double act_factor = v_actual / (v_headroom * v_headroom);
        double accurate_ratio = act_factor / nom_factor;
        double extra_delay_ns = (accurate_ratio - 1.0) * 0.1; // assuming 100ps nominal
        extra_delay_ns = std::max(extra_delay_ns, 0.0);

        if (extra_delay_ns > 0.001) {
            result.gate_delay_increases.push_back({ci, extra_delay_ns});
        }

        worst_extra_delay = std::max(worst_extra_delay, extra_delay_ns);
        total_extra_delay += extra_delay_ns;
        gate_count++;
    }

    result.voltage_aware_wns = result.nominal_wns - worst_extra_delay;
    result.timing_degradation_pct = (gate_count > 0) ?
        (total_extra_delay / gate_count) / 0.1 * 100.0 : 0.0; // relative to 100ps nominal

    // Sort by delay increase descending
    std::sort(result.gate_delay_increases.begin(), result.gate_delay_increases.end(),
              [](auto& a, auto& b) { return a.second > b.second; });

    return result;
}

// ── Hotspot clustering ───────────────────────────────────────────────
// Cluster high-drop regions using spatial grouping.
// Hotspot = region where avg drop > threshold.

std::vector<IrHotspot> IrDropAnalyzer::find_hotspots(double threshold_mv) {
    if (!has_result_) analyze();

    std::vector<IrHotspot> hotspots;

    int N = last_grid_n_ > 0 ? last_grid_n_ : cfg_.grid_resolution;
    if (N <= 0 || !has_result_ || last_result_.drop_map.empty()) return hotspots;

    double cell_w = pd_.die_area.width() / N;
    double cell_h = pd_.die_area.height() / N;

    // Mark cells above threshold
    std::vector<std::vector<bool>> hot(N, std::vector<bool>(N, false));
    for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x)
            if (y < (int)last_result_.drop_map.size() && x < (int)last_result_.drop_map[0].size())
                hot[y][x] = (last_result_.drop_map[y][x] > threshold_mv);

    // Flood-fill clustering
    std::vector<std::vector<bool>> visited(N, std::vector<bool>(N, false));
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            if (!hot[y][x] || visited[y][x]) continue;

            // BFS to find connected hot region
            std::vector<std::pair<int,int>> cluster;
            std::vector<std::pair<int,int>> queue = {{x, y}};
            visited[y][x] = true;

            while (!queue.empty()) {
                auto [cx, cy] = queue.back();
                queue.pop_back();
                cluster.push_back({cx, cy});

                // 4-connected neighbors
                int dx[] = {-1, 1, 0, 0};
                int dy[] = {0, 0, -1, 1};
                for (int d = 0; d < 4; ++d) {
                    int nx = cx + dx[d], ny = cy + dy[d];
                    if (nx >= 0 && nx < N && ny >= 0 && ny < N &&
                        !visited[ny][nx] && hot[ny][nx]) {
                        visited[ny][nx] = true;
                        queue.push_back({nx, ny});
                    }
                }
            }

            // Compute hotspot centroid and stats
            double sum_x = 0, sum_y = 0, sum_drop = 0;
            double max_dist = 0;
            for (auto& [cx, cy] : cluster) {
                double px = pd_.die_area.x0 + (cx + 0.5) * cell_w;
                double py = pd_.die_area.y0 + (cy + 0.5) * cell_h;
                sum_x += px;
                sum_y += py;
                sum_drop += last_result_.drop_map[cy][cx];
            }
            double cx_avg = sum_x / cluster.size();
            double cy_avg = sum_y / cluster.size();

            for (auto& [cx, cy] : cluster) {
                double px = pd_.die_area.x0 + (cx + 0.5) * cell_w;
                double py = pd_.die_area.y0 + (cy + 0.5) * cell_h;
                double dist = std::sqrt((px - cx_avg) * (px - cx_avg) +
                                        (py - cy_avg) * (py - cy_avg));
                max_dist = std::max(max_dist, dist);
            }

            IrHotspot hs;
            hs.x = cx_avg;
            hs.y = cy_avg;
            hs.radius = std::max(max_dist, std::max(cell_w, cell_h));
            hs.avg_drop_mv = sum_drop / cluster.size();
            hs.cells_affected = static_cast<int>(cluster.size());
            hotspots.push_back(hs);
        }
    }

    // Sort by avg drop descending
    std::sort(hotspots.begin(), hotspots.end(),
              [](auto& a, auto& b) { return a.avg_drop_mv > b.avg_drop_mv; });

    return hotspots;
}

// ── EM-aware current density check ───────────────────────────────────
// Checks each grid node's current density against EM limits.
// Current density = current / wire_width, where wire_width ≈ mesh_pitch.

std::vector<IrEmHotspot> IrDropAnalyzer::check_em_limits(double max_current_density_ma_per_um) {
    if (!has_result_) analyze();

    std::vector<IrEmHotspot> violations;

    int N = last_grid_n_ > 0 ? last_grid_n_ : cfg_.grid_resolution;
    if (N <= 0 || !has_result_ || last_current_map_.empty()) return violations;

    double wire_width_um = cfg_.mesh_pitch_um;
    if (wire_width_um <= 0) wire_width_um = 50.0;

    double worst_density = 0.0;

    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            double current_ma = last_current_map_[y][x];
            double density = current_ma / wire_width_um;

            worst_density = std::max(worst_density, density);

            if (density > max_current_density_ma_per_um) {
                violations.push_back({x, y, density, max_current_density_ma_per_um});
            }
        }
    }

    // Update cached result with EM info
    last_result_.em_violations = static_cast<int>(violations.size());
    last_result_.worst_current_density = worst_density;

    return violations;
}

// ── Enhanced IR drop run ─────────────────────────────────────────────

IrDropResult IrDropAnalyzer::run_enhanced() {
    auto t0 = std::chrono::high_resolution_clock::now();

    // Run standard analysis (also caches data)
    auto result = analyze();

    // Enable dynamic analysis
    bool saved_dynamic = cfg_.enable_dynamic;
    cfg_.enable_dynamic = true;

    // Perform dynamic analysis
    auto dyn_result = analyze_dynamic(10.0, 50);
    result.worst_dynamic_drop_mv = dyn_result.peak_drop_mv;
    result.avg_dynamic_drop_mv = dyn_result.avg_drop_mv;
    result.dynamic_analyzed = true;

    cfg_.enable_dynamic = saved_dynamic;

    // Re-evaluate signoff with dynamic data
    evaluate_signoff(result);

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    result.message = "Enhanced IR Drop: static_worst=" + std::to_string((int)result.worst_drop_mv) +
                     "mV, dynamic_peak=" + std::to_string((int)result.worst_dynamic_drop_mv) +
                     "mV, " + std::to_string(result.num_hotspots) + " hotspot(s), " +
                     (result.signoff_pass ? "[SIGNOFF PASS]" : "[SIGNOFF FAIL]");

    last_result_ = result;
    has_result_ = true;
    return result;
}

} // namespace sf
