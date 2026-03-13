// SiliconForge — Statistical Static Timing Analysis Implementation
// Monte Carlo sampling with process variation, spatial correlation, yield.
#include "timing/ssta.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <sstream>

namespace sf {

// ── Nominal gate delay (no library → heuristic model) ────────────────

double SstaEngine::nominal_gate_delay(GateId gid) const {
    const auto& g = nl_.gate(gid);
    if (lib_) {
        // Look up cell name in Liberty library
        std::string cell_name;
        switch (g.type) {
            case GateType::BUF:   cell_name = "BUF_X1";  break;
            case GateType::NOT:   cell_name = "INV_X1";   break;
            case GateType::AND:   cell_name = "AND2_X1";  break;
            case GateType::OR:    cell_name = "OR2_X1";   break;
            case GateType::NAND:  cell_name = "NAND2_X1"; break;
            case GateType::NOR:   cell_name = "NOR2_X1";  break;
            case GateType::XOR:   cell_name = "XOR2_X1";  break;
            case GateType::XNOR:  cell_name = "XNOR2_X1"; break;
            default: break;
        }
        if (!cell_name.empty()) {
            auto* cell = lib_->find_cell(cell_name);
            if (cell && !cell->timings.empty()) {
                double delay = (cell->timings[0].cell_rise + cell->timings[0].cell_fall) / 2.0;
                if (delay > 0) return delay * 1000.0; // ns → ps
            }
        }
    }
    // Heuristic delay model (ps) based on gate type and fan-in
    switch (g.type) {
        case GateType::BUF:    return 20.0;
        case GateType::NOT:    return 15.0;
        case GateType::AND:    return 30.0 + 5.0 * (g.inputs.size() > 2 ? g.inputs.size() - 2 : 0);
        case GateType::OR:     return 30.0 + 5.0 * (g.inputs.size() > 2 ? g.inputs.size() - 2 : 0);
        case GateType::NAND:   return 25.0 + 5.0 * (g.inputs.size() > 2 ? g.inputs.size() - 2 : 0);
        case GateType::NOR:    return 25.0 + 5.0 * (g.inputs.size() > 2 ? g.inputs.size() - 2 : 0);
        case GateType::XOR:    return 40.0;
        case GateType::XNOR:   return 40.0;
        case GateType::MUX:    return 35.0;
        case GateType::DFF:    return 50.0;
        case GateType::DLATCH: return 45.0;
        default:               return 10.0;
    }
}

// ── Sample gate delay with variation ─────────────────────────────────

double SstaEngine::sample_gate_delay(GateId gid, double global_var,
                                     double local_var) const {
    double nom = nominal_gate_delay(gid);
    // delay_sample = nominal × (1 + global_var + local_var)
    double varied = nom * (1.0 + global_var + local_var);
    return std::max(varied, 0.1);  // clamp to positive
}

// ── Forward propagation for one MC sample ────────────────────────────

double SstaEngine::forward_propagate_sample(
    const std::vector<GateId>& topo,
    const std::unordered_map<GateId, double>& gate_delays) const
{
    // arrival[net] = latest arrival time at that net
    std::unordered_map<NetId, double> arrival;

    // Primary inputs arrive at time 0
    for (auto pi : nl_.primary_inputs())
        arrival[pi] = 0.0;
    // DFF outputs arrive at time 0 (launch edge)
    for (auto fid : nl_.flip_flops()) {
        auto& ff = nl_.gate(fid);
        if (ff.output >= 0) arrival[ff.output] = 0.0;
    }

    // Propagate through topological order
    for (GateId gid : topo) {
        const auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::DFF) continue;

        double max_input_arr = 0.0;
        for (NetId in_net : g.inputs) {
            auto it = arrival.find(in_net);
            if (it != arrival.end())
                max_input_arr = std::max(max_input_arr, it->second);
        }

        auto dit = gate_delays.find(gid);
        double delay = (dit != gate_delays.end()) ? dit->second : nominal_gate_delay(gid);

        if (g.output >= 0)
            arrival[g.output] = max_input_arr + delay;
    }

    // Find worst arrival at primary outputs and DFF inputs
    double worst_arrival = 0.0;
    for (auto po : nl_.primary_outputs()) {
        auto it = arrival.find(po);
        if (it != arrival.end())
            worst_arrival = std::max(worst_arrival, it->second);
    }
    for (auto fid : nl_.flip_flops()) {
        auto& ff = nl_.gate(fid);
        for (NetId in_net : ff.inputs) {
            auto it = arrival.find(in_net);
            if (it != arrival.end())
                worst_arrival = std::max(worst_arrival, it->second);
        }
    }

    return worst_arrival;
}

// ── Assign pseudo-positions for spatial correlation ──────────────────

void SstaEngine::assign_gate_positions() {
    gate_positions_.clear();
    auto topo = nl_.topo_order();
    // Simple grid placement based on topological index
    int cols = std::max(1, static_cast<int>(std::sqrt(topo.size())));
    double spacing = 10.0;  // um between gates
    for (size_t i = 0; i < topo.size(); ++i) {
        int row = static_cast<int>(i) / cols;
        int col = static_cast<int>(i) % cols;
        gate_positions_[topo[i]] = {col * spacing, row * spacing};
    }
}

// ── Spatial correlation ──────────────────────────────────────────────

double SstaEngine::spatial_correlation(GateId a, GateId b) const {
    auto ia = gate_positions_.find(a);
    auto ib = gate_positions_.find(b);
    if (ia == gate_positions_.end() || ib == gate_positions_.end())
        return 0.0;
    double dx = ia->second.x - ib->second.x;
    double dy = ia->second.y - ib->second.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    return std::exp(-dist / config_.spatial_correlation_length_um);
}

// ── Generate correlated local variations ─────────────────────────────

std::vector<double> SstaEngine::generate_correlated_locals(
    std::mt19937& rng, const std::vector<GateId>& gates) const
{
    size_t n = gates.size();
    std::normal_distribution<double> norm(0.0, 1.0);
    std::vector<double> independent(n);
    for (size_t i = 0; i < n; ++i)
        independent[i] = norm(rng);

    if (!config_.enable_spatial_correlation || n <= 1) {
        // Scale by local sigma
        double local_sigma = config_.local_variation_pct / 100.0;
        for (size_t i = 0; i < n; ++i)
            independent[i] *= local_sigma;
        return independent;
    }

    // Approximate spatial correlation via mixing with nearest-neighbor
    // For efficiency, use a simplified approach: for each gate, blend its
    // independent sample with the average of spatially-close gates.
    double local_sigma = config_.local_variation_pct / 100.0;
    std::vector<double> correlated(n);
    for (size_t i = 0; i < n; ++i) {
        double weighted_sum = independent[i];
        double weight_total = 1.0;
        for (size_t j = 0; j < n; ++j) {
            if (i == j) continue;
            double cor = spatial_correlation(gates[i], gates[j]);
            if (cor > 0.01) {  // threshold for efficiency
                weighted_sum += cor * independent[j];
                weight_total += cor;
            }
        }
        correlated[i] = (weighted_sum / weight_total) * local_sigma;
    }
    return correlated;
}

// ── Main Monte Carlo analysis ────────────────────────────────────────

SstaResult SstaEngine::run_monte_carlo() {
    auto t0 = std::chrono::high_resolution_clock::now();
    SstaResult result;

    // Get topological order
    auto topo = nl_.topo_order();
    if (topo.empty()) {
        result.message = "No combinational gates for SSTA";
        result.num_samples_run = 0;
        has_result_ = true;
        last_result_ = result;
        return result;
    }

    // Collect all combinational gates for variation application
    std::vector<GateId> comb_gates;
    for (GateId gid : topo) {
        auto& g = nl_.gate(gid);
        if (g.type != GateType::INPUT && g.type != GateType::OUTPUT)
            comb_gates.push_back(gid);
    }

    // Assign positions for spatial correlation
    if (config_.enable_spatial_correlation)
        assign_gate_positions();

    std::mt19937 rng(config_.seed);
    std::normal_distribution<double> global_dist(0.0, config_.global_variation_pct / 100.0);

    std::vector<double> wns_samples(config_.num_samples);
    std::vector<double> tns_samples(config_.num_samples);

    // Per-path tracking: identify paths by their endpoint
    // We track worst arrival at each endpoint per sample
    std::unordered_map<NetId, std::vector<double>> endpoint_arrivals;
    for (auto po : nl_.primary_outputs())
        endpoint_arrivals[po].resize(config_.num_samples, 0.0);
    for (auto fid : nl_.flip_flops()) {
        auto& ff = nl_.gate(fid);
        for (NetId in_net : ff.inputs)
            endpoint_arrivals[in_net].resize(config_.num_samples, 0.0);
    }

    for (int s = 0; s < config_.num_samples; ++s) {
        // Global variation (same for all gates in this sample)
        double global_var = global_dist(rng);

        // Local variations (per-gate, possibly spatially correlated)
        auto local_vars = generate_correlated_locals(rng, comb_gates);

        // Build gate delay map for this sample
        std::unordered_map<GateId, double> gate_delays;
        for (size_t i = 0; i < comb_gates.size(); ++i) {
            gate_delays[comb_gates[i]] = sample_gate_delay(
                comb_gates[i], global_var, local_vars[i]);
        }

        // Forward propagation
        std::unordered_map<NetId, double> arrival;
        for (auto pi : nl_.primary_inputs())
            arrival[pi] = 0.0;
        for (auto fid : nl_.flip_flops()) {
            auto& ff = nl_.gate(fid);
            if (ff.output >= 0) arrival[ff.output] = 0.0;
        }

        for (GateId gid : topo) {
            const auto& g = nl_.gate(gid);
            if (g.type == GateType::INPUT || g.type == GateType::DFF) continue;

            double max_arr = 0.0;
            for (NetId in_net : g.inputs) {
                auto it = arrival.find(in_net);
                if (it != arrival.end())
                    max_arr = std::max(max_arr, it->second);
            }

            auto dit = gate_delays.find(gid);
            double delay = (dit != gate_delays.end()) ? dit->second
                                                      : nominal_gate_delay(gid);
            if (g.output >= 0)
                arrival[g.output] = max_arr + delay;
        }

        // Compute WNS and TNS for this sample
        double worst_arr = 0.0;
        double total_neg_slack = 0.0;

        for (auto po : nl_.primary_outputs()) {
            auto it = arrival.find(po);
            if (it != arrival.end()) {
                worst_arr = std::max(worst_arr, it->second);
                double slack = clock_period_ - it->second;
                if (slack < 0) total_neg_slack += slack;
                endpoint_arrivals[po][s] = it->second;
            }
        }
        for (auto fid : nl_.flip_flops()) {
            auto& ff = nl_.gate(fid);
            for (NetId in_net : ff.inputs) {
                auto it = arrival.find(in_net);
                if (it != arrival.end()) {
                    worst_arr = std::max(worst_arr, it->second);
                    double slack = clock_period_ - it->second;
                    if (slack < 0) total_neg_slack += slack;
                    endpoint_arrivals[in_net][s] = it->second;
                }
            }
        }

        wns_samples[s] = clock_period_ - worst_arr;  // WNS (negative = violation)
        tns_samples[s] = total_neg_slack;
    }

    // Build statistical WNS
    result.statistical_wns.samples = wns_samples;
    result.statistical_wns.compute_from_samples();

    // Build statistical TNS
    result.statistical_tns.samples = tns_samples;
    result.statistical_tns.compute_from_samples();

    // Yield = fraction of samples with WNS >= 0
    int passing = 0;
    for (double wns : wns_samples)
        if (wns >= 0.0) ++passing;
    result.yield_estimate = static_cast<double>(passing) / config_.num_samples;

    // Sigma-to-yield table
    for (double sig = 1.0; sig <= 6.0; sig += 0.5) {
        double threshold = result.statistical_wns.mean_ps - sig * result.statistical_wns.sigma_ps;
        int count = 0;
        for (double wns : wns_samples)
            if (wns >= threshold) ++count;
        double yield_pct = 100.0 * count / config_.num_samples;
        result.sigma_to_yield[sig] = yield_pct;
        result.yield_at_sigma[sig] = yield_pct;
    }

    // Build critical path statistics (top endpoints by worst mean delay)
    struct EndpointInfo {
        NetId net;
        double mean_delay;
    };
    std::vector<EndpointInfo> endpoints;
    for (auto& [net, arrivals] : endpoint_arrivals) {
        if (arrivals.empty()) continue;
        double sum = std::accumulate(arrivals.begin(), arrivals.end(), 0.0);
        double mean = sum / arrivals.size();
        if (mean > 0.0) endpoints.push_back({net, mean});
    }
    std::sort(endpoints.begin(), endpoints.end(),
              [](auto& a, auto& b) { return a.mean_delay > b.mean_delay; });

    int max_paths = std::min(static_cast<int>(endpoints.size()), 5);
    for (int i = 0; i < max_paths; ++i) {
        StatisticalPath sp;
        sp.path_name = "path_to_net_" + std::to_string(endpoints[i].net);
        auto& arrivals = endpoint_arrivals[endpoints[i].net];
        sp.delay_samples = arrivals;
        double sum = std::accumulate(arrivals.begin(), arrivals.end(), 0.0);
        sp.mean_delay_ps = sum / arrivals.size();
        double sq_sum = 0.0;
        for (double v : arrivals) sq_sum += (v - sp.mean_delay_ps) * (v - sp.mean_delay_ps);
        sp.sigma_ps = std::sqrt(sq_sum / arrivals.size());
        auto mm = std::minmax_element(arrivals.begin(), arrivals.end());
        sp.best_sample_delay_ps = *mm.first;
        sp.worst_sample_delay_ps = *mm.second;

        // Yield for this path
        int path_passing = 0;
        for (double d : arrivals)
            if (d <= clock_period_) ++path_passing;
        sp.yield_probability = static_cast<double>(path_passing) / arrivals.size();

        // Percentiles
        auto sorted = arrivals;
        std::sort(sorted.begin(), sorted.end());
        for (double pct : {1.0, 5.0, 25.0, 50.0, 75.0, 95.0, 99.0}) {
            size_t idx = static_cast<size_t>(pct / 100.0 * (sorted.size() - 1));
            sp.percentile_delays[pct] = sorted[idx];
        }

        result.critical_paths.push_back(sp);
    }

    // Path correlations (between top 2 paths if available)
    if (result.critical_paths.size() >= 2) {
        auto& p0 = result.critical_paths[0].delay_samples;
        auto& p1 = result.critical_paths[1].delay_samples;
        if (p0.size() == p1.size() && !p0.empty()) {
            double mean0 = result.critical_paths[0].mean_delay_ps;
            double mean1 = result.critical_paths[1].mean_delay_ps;
            double cov = 0.0, var0 = 0.0, var1 = 0.0;
            for (size_t k = 0; k < p0.size(); ++k) {
                double d0 = p0[k] - mean0, d1 = p1[k] - mean1;
                cov += d0 * d1; var0 += d0 * d0; var1 += d1 * d1;
            }
            double denom = std::sqrt(var0 * var1);
            double corr = (denom > 0) ? cov / denom : 0.0;
            result.path_correlations["path0_path1"] = corr;
        }
    }

    result.num_samples_run = config_.num_samples;
    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Summary
    std::ostringstream oss;
    oss << "SSTA: " << config_.num_samples << " MC samples, "
        << "WNS mean=" << result.statistical_wns.mean_ps << "ps "
        << "sigma=" << result.statistical_wns.sigma_ps << "ps, "
        << "yield=" << (result.yield_estimate * 100.0) << "%, "
        << "time=" << result.time_ms << "ms";
    result.summary = oss.str();
    result.message = "SSTA completed successfully";

    has_result_ = true;
    last_result_ = result;
    return result;
}

// ── Compute yield at specific target period ──────────────────────────

double SstaEngine::compute_yield(double target_period_ps) {
    double saved_period = clock_period_;
    clock_period_ = target_period_ps;
    auto result = run_monte_carlo();
    clock_period_ = saved_period;
    return result.yield_estimate;
}

// ── Get statistical WNS ──────────────────────────────────────────────

StatisticalDelay SstaEngine::get_statistical_wns() {
    if (!has_result_)
        run_monte_carlo();
    return last_result_.statistical_wns;
}

// ── Sensitivity analysis ─────────────────────────────────────────────

SensitivityResult SstaEngine::sensitivity_analysis() {
    SensitivityResult sr;
    auto topo = nl_.topo_order();
    if (topo.empty()) return sr;

    // Baseline: run with current config
    auto baseline = run_monte_carlo();
    double baseline_mean = baseline.statistical_wns.mean_ps;

    // Test each variation parameter by increasing it by 50%
    struct ParamTest {
        std::string name;
        double* global_ptr;
        double* local_ptr;
    };

    double saved_global = config_.global_variation_pct;
    double saved_local  = config_.local_variation_pct;

    // Test global variation sensitivity
    {
        config_.global_variation_pct = saved_global * 1.5;
        auto result = run_monte_carlo();
        double impact = std::abs(result.statistical_wns.mean_ps - baseline_mean);
        sr.parameter_impact["global_variation"] = impact;
        config_.global_variation_pct = saved_global;
    }

    // Test local variation sensitivity
    {
        config_.local_variation_pct = saved_local * 1.5;
        auto result = run_monte_carlo();
        double impact = std::abs(result.statistical_wns.mean_ps - baseline_mean);
        sr.parameter_impact["local_variation"] = impact;
        config_.local_variation_pct = saved_local;
    }

    // Test gate length sensitivity
    {
        double saved_gl = pv_.gate_length_sigma_pct;
        double saved_gv = config_.global_variation_pct;
        config_.global_variation_pct = saved_global + pv_.gate_length_sigma_pct * 0.5;
        auto result = run_monte_carlo();
        double impact = std::abs(result.statistical_wns.mean_ps - baseline_mean);
        sr.parameter_impact["gate_length"] = impact;
        pv_.gate_length_sigma_pct = saved_gl;
        config_.global_variation_pct = saved_gv;
    }

    // Test Vth sensitivity
    {
        double saved_vth = pv_.vth_sigma_pct;
        double saved_gv = config_.global_variation_pct;
        config_.global_variation_pct = saved_global + pv_.vth_sigma_pct * 0.5;
        auto result = run_monte_carlo();
        double impact = std::abs(result.statistical_wns.mean_ps - baseline_mean);
        sr.parameter_impact["vth"] = impact;
        pv_.vth_sigma_pct = saved_vth;
        config_.global_variation_pct = saved_gv;
    }

    // Test wire width sensitivity
    {
        double saved_ww = pv_.wire_width_sigma_pct;
        double saved_lv = config_.local_variation_pct;
        config_.local_variation_pct = saved_local + pv_.wire_width_sigma_pct * 0.5;
        auto result = run_monte_carlo();
        double impact = std::abs(result.statistical_wns.mean_ps - baseline_mean);
        sr.parameter_impact["wire_width"] = impact;
        pv_.wire_width_sigma_pct = saved_ww;
        config_.local_variation_pct = saved_lv;
    }

    // Restore config
    config_.global_variation_pct = saved_global;
    config_.local_variation_pct  = saved_local;
    has_result_ = false;  // invalidate cache since we ran multiple configs

    // Find dominant parameter
    double max_impact = 0.0;
    for (auto& [name, impact] : sr.parameter_impact) {
        if (impact > max_impact) {
            max_impact = impact;
            sr.dominant_parameter = name;
            sr.dominant_impact_ps = impact;
        }
    }

    return sr;
}

// ══════════════════════════════════════════════════════════════════════
// Enhanced SSTA — canonical-delay propagation, PCA, yield, stat slacks
// ══════════════════════════════════════════════════════════════════════

// ── Standard normal CDF Φ(x) — Abramowitz & Stegun approximation ────

double SstaEngine::phi_cdf(double x) {
    // Handle extreme values
    if (x < -8.0) return 0.0;
    if (x > 8.0) return 1.0;
    // Use the error function: Φ(x) = 0.5 × (1 + erf(x / √2))
    return 0.5 * (1.0 + std::erf(x / std::sqrt(2.0)));
}

// ── Standard normal PDF φ(x) ─────────────────────────────────────────

double SstaEngine::phi_pdf(double x) {
    static const double inv_sqrt_2pi = 1.0 / std::sqrt(2.0 * M_PI);
    return inv_sqrt_2pi * std::exp(-0.5 * x * x);
}

// ── Clark's approximation: MAX of two canonical delays ───────────────
// max(a,b) ≈ a×Φ(θ) + b×Φ(−θ) + σ_ab×φ(θ)
// where θ = (μa − μb) / σ_ab,  σ_ab = √(σa² + σb² − 2ρσaσb)

CanonicalDelay SstaEngine::canonical_max(const CanonicalDelay& a, const CanonicalDelay& b) {
    // Compute total variance for each
    double var_a = a.random_sigma * a.random_sigma;
    for (double s : a.sensitivities) var_a += s * s;

    double var_b = b.random_sigma * b.random_sigma;
    for (double s : b.sensitivities) var_b += s * s;

    double sigma_a = std::sqrt(std::max(var_a, 1e-30));
    double sigma_b = std::sqrt(std::max(var_b, 1e-30));

    // Covariance from shared systematic sources
    size_t n = std::min(a.sensitivities.size(), b.sensitivities.size());
    double cov = 0.0;
    for (size_t i = 0; i < n; ++i)
        cov += a.sensitivities[i] * b.sensitivities[i];

    double var_diff = var_a + var_b - 2.0 * cov;
    double sigma_ab = std::sqrt(std::max(var_diff, 1e-30));

    double mu_a = a.nominal;
    double mu_b = b.nominal;
    double theta = (sigma_ab > 1e-15) ? (mu_a - mu_b) / sigma_ab : 0.0;

    double phi_t = phi_cdf(theta);
    double pdf_t = phi_pdf(theta);

    CanonicalDelay result;
    result.nominal = mu_a * phi_t + mu_b * (1.0 - phi_t) + sigma_ab * pdf_t;

    // Propagate sensitivities: s_max_i = s_a_i × Φ(θ) + s_b_i × Φ(−θ)
    size_t max_sz = std::max(a.sensitivities.size(), b.sensitivities.size());
    result.sensitivities.resize(max_sz, 0.0);
    for (size_t i = 0; i < max_sz; ++i) {
        double sa = (i < a.sensitivities.size()) ? a.sensitivities[i] : 0.0;
        double sb = (i < b.sensitivities.size()) ? b.sensitivities[i] : 0.0;
        result.sensitivities[i] = sa * phi_t + sb * (1.0 - phi_t);
    }

    // Random sigma propagation
    double rand_var = a.random_sigma * a.random_sigma * phi_t +
                      b.random_sigma * b.random_sigma * (1.0 - phi_t) +
                      (sigma_a * sigma_b - cov) * pdf_t / (sigma_ab > 1e-15 ? sigma_ab : 1.0) *
                      (a.random_sigma * b.random_sigma > 0 ? 1.0 : 0.0);
    rand_var = std::max(rand_var, 0.0);
    // Correction term to account for variance loss in max approximation
    double mu_max = result.nominal;
    double var_max = var_a * phi_t + var_b * (1.0 - phi_t) +
                     (mu_a * mu_a + var_a) * phi_t + (mu_b * mu_b + var_b) * (1.0 - phi_t) +
                     (mu_a + mu_b) * sigma_ab * pdf_t - mu_max * mu_max;
    var_max = std::max(var_max, 0.0);

    // Systematic variance of result
    double sys_var = 0.0;
    for (double s : result.sensitivities) sys_var += s * s;
    result.random_sigma = std::sqrt(std::max(var_max - sys_var, 0.0));

    return result;
}

// ── Set user-supplied spatial correlation ─────────────────────────────

void SstaEngine::set_spatial_correlation(const SpatialCorrelation& sc) {
    spatial_corr_ = sc;
    has_spatial_corr_ = true;

    // Also populate gate_positions_ from user data
    for (size_t i = 0; i < sc.gate_positions.size(); ++i) {
        gate_positions_[static_cast<GateId>(i)] = {
            sc.gate_positions[i].first, sc.gate_positions[i].second};
    }
    config_.spatial_correlation_length_um = sc.correlation_length;
}

// ── Build correlation matrix ─────────────────────────────────────────

void SstaEngine::build_correlation_matrix(const std::vector<GateId>& gates,
                                           std::vector<std::vector<double>>& corr) const {
    size_t n = gates.size();
    corr.assign(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        corr[i][i] = 1.0;
        for (size_t j = i + 1; j < n; ++j) {
            double c = spatial_correlation(gates[i], gates[j]);
            corr[i][j] = c;
            corr[j][i] = c;
        }
    }
}

// ── PCA-based variation reduction ────────────────────────────────────
// Eigendecompose the spatial correlation matrix using Jacobi iteration,
// keep the top-k principal components capturing >95% variance.

PcaResult SstaEngine::reduce_with_pca(int max_components) {
    auto topo = nl_.topo_order();
    std::vector<GateId> comb_gates;
    for (GateId gid : topo) {
        auto& g = nl_.gate(gid);
        if (g.type != GateType::INPUT && g.type != GateType::OUTPUT)
            comb_gates.push_back(gid);
    }

    if (config_.enable_spatial_correlation)
        assign_gate_positions();

    size_t n = comb_gates.size();
    PcaResult result;
    result.original_sources = static_cast<int>(n);

    if (n == 0) {
        result.principal_components = 0;
        result.variance_captured = 0.0;
        has_pca_ = true;
        pca_result_ = result;
        return result;
    }

    // Cap matrix size for performance (Jacobi is O(n³))
    size_t cap = std::min(n, size_t(64));
    std::vector<GateId> subset(comb_gates.begin(), comb_gates.begin() + cap);

    std::vector<std::vector<double>> corr;
    build_correlation_matrix(subset, corr);

    size_t m = cap;
    // Jacobi eigenvalue algorithm on the symmetric correlation matrix
    std::vector<std::vector<double>> V(m, std::vector<double>(m, 0.0));
    for (size_t i = 0; i < m; ++i) V[i][i] = 1.0;

    auto A = corr; // working copy
    for (int sweep = 0; sweep < 100; ++sweep) {
        double off_diag = 0.0;
        for (size_t i = 0; i < m; ++i)
            for (size_t j = i + 1; j < m; ++j)
                off_diag += A[i][j] * A[i][j];
        if (off_diag < 1e-12) break;

        for (size_t p = 0; p < m; ++p) {
            for (size_t q = p + 1; q < m; ++q) {
                if (std::abs(A[p][q]) < 1e-15) continue;
                double tau = (A[q][q] - A[p][p]) / (2.0 * A[p][q]);
                double t = (tau >= 0 ? 1.0 : -1.0) / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
                double c = 1.0 / std::sqrt(1.0 + t * t);
                double s = t * c;

                // Rotate A
                double app = A[p][p], aqq = A[q][q], apq = A[p][q];
                A[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
                A[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
                A[p][q] = 0.0;
                A[q][p] = 0.0;
                for (size_t r = 0; r < m; ++r) {
                    if (r == p || r == q) continue;
                    double arp = A[r][p], arq = A[r][q];
                    A[r][p] = c * arp - s * arq;
                    A[p][r] = A[r][p];
                    A[r][q] = s * arp + c * arq;
                    A[q][r] = A[r][q];
                }
                for (size_t r = 0; r < m; ++r) {
                    double vrp = V[r][p], vrq = V[r][q];
                    V[r][p] = c * vrp - s * vrq;
                    V[r][q] = s * vrp + c * vrq;
                }
            }
        }
    }

    // Collect eigenvalues and sort descending
    std::vector<std::pair<double, int>> eigen_pairs;
    double total_variance = 0.0;
    for (size_t i = 0; i < m; ++i) {
        double ev = std::max(A[i][i], 0.0);
        eigen_pairs.push_back({ev, static_cast<int>(i)});
        total_variance += ev;
    }
    std::sort(eigen_pairs.begin(), eigen_pairs.end(),
              [](auto& a, auto& b) { return a.first > b.first; });

    // Keep top-k capturing >95% variance
    int k = 0;
    double captured = 0.0;
    double threshold = 0.95 * total_variance;
    for (auto& [ev, idx] : eigen_pairs) {
        if (k >= max_components) break;
        captured += ev;
        k++;
        if (captured >= threshold) break;
    }
    k = std::max(k, 1);

    result.principal_components = k;
    result.variance_captured = (total_variance > 0) ? captured / total_variance : 0.0;

    // Build transform matrix: k × m (each row is a principal component)
    result.transform.resize(k, std::vector<double>(m, 0.0));
    for (int i = 0; i < k; ++i) {
        int col = eigen_pairs[i].second;
        double scale = std::sqrt(std::max(eigen_pairs[i].first, 0.0));
        for (size_t r = 0; r < m; ++r)
            result.transform[i][r] = V[r][col] * scale;
    }

    has_pca_ = true;
    pca_result_ = result;
    return result;
}

// ── Compute statistical slacks ───────────────────────────────────────
// Propagate canonical delays through the circuit, compute slack
// distribution at each endpoint.

std::vector<StatSlack> SstaEngine::compute_statistical_slacks() {
    auto topo = nl_.topo_order();
    if (topo.empty()) return {};

    // Collect combinational gates
    std::vector<GateId> comb_gates;
    for (GateId gid : topo) {
        auto& g = nl_.gate(gid);
        if (g.type != GateType::INPUT && g.type != GateType::OUTPUT)
            comb_gates.push_back(gid);
    }

    if (config_.enable_spatial_correlation)
        assign_gate_positions();

    // Number of systematic variation sources = number of comb gates (simplified)
    size_t num_sources = comb_gates.size();

    // Build canonical delay for each gate
    std::unordered_map<GateId, int> gate_source_idx;
    for (size_t i = 0; i < comb_gates.size(); ++i)
        gate_source_idx[comb_gates[i]] = static_cast<int>(i);

    // Propagate canonical arrival times
    std::unordered_map<NetId, CanonicalDelay> arrival;

    // Initialize: primary inputs and DFF outputs arrive at time 0
    CanonicalDelay zero_delay;
    zero_delay.nominal = 0.0;
    zero_delay.sensitivities.resize(num_sources, 0.0);
    zero_delay.random_sigma = 0.0;

    for (auto pi : nl_.primary_inputs())
        arrival[pi] = zero_delay;
    for (auto fid : nl_.flip_flops()) {
        auto& ff = nl_.gate(fid);
        if (ff.output >= 0) arrival[ff.output] = zero_delay;
    }

    // Forward propagation
    for (GateId gid : topo) {
        const auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::DFF) continue;
        if (g.output < 0) continue;

        // Build canonical delay for this gate
        double nom = nominal_gate_delay(gid);
        CanonicalDelay gate_cd;
        gate_cd.nominal = nom;
        gate_cd.sensitivities.resize(num_sources, 0.0);
        auto it_src = gate_source_idx.find(gid);
        if (it_src != gate_source_idx.end()) {
            // Sensitivity to its own variation source
            double local_sigma = config_.local_variation_pct / 100.0;
            gate_cd.sensitivities[it_src->second] = nom * local_sigma;
        }
        gate_cd.random_sigma = nom * (config_.global_variation_pct / 100.0) * 0.3;

        // Find max arrival among inputs using Clark's approximation
        CanonicalDelay max_arr = zero_delay;
        bool first = true;
        for (NetId in_net : g.inputs) {
            auto ait = arrival.find(in_net);
            if (ait == arrival.end()) continue;
            if (first) {
                max_arr = ait->second;
                first = false;
            } else {
                max_arr = canonical_max(max_arr, ait->second);
            }
        }

        // arrival at output = max_input_arrival + gate_delay
        CanonicalDelay out_arr;
        out_arr.nominal = max_arr.nominal + gate_cd.nominal;
        size_t sz = std::max(max_arr.sensitivities.size(), gate_cd.sensitivities.size());
        out_arr.sensitivities.resize(sz, 0.0);
        for (size_t i = 0; i < sz; ++i) {
            double sa = (i < max_arr.sensitivities.size()) ? max_arr.sensitivities[i] : 0.0;
            double sg = (i < gate_cd.sensitivities.size()) ? gate_cd.sensitivities[i] : 0.0;
            out_arr.sensitivities[i] = sa + sg;
        }
        out_arr.random_sigma = std::sqrt(max_arr.random_sigma * max_arr.random_sigma +
                                          gate_cd.random_sigma * gate_cd.random_sigma);

        arrival[g.output] = out_arr;
    }

    // Collect endpoints
    std::vector<NetId> endpoints;
    for (auto po : nl_.primary_outputs())
        endpoints.push_back(po);
    for (auto fid : nl_.flip_flops()) {
        auto& ff = nl_.gate(fid);
        for (NetId in_net : ff.inputs)
            endpoints.push_back(in_net);
    }

    // Compute statistical slack at each endpoint
    std::vector<StatSlack> slacks;
    for (NetId ep : endpoints) {
        auto ait = arrival.find(ep);
        if (ait == arrival.end()) continue;

        const auto& arr = ait->second;
        double mean_arr = arr.nominal;
        double var_arr = arr.random_sigma * arr.random_sigma;
        for (double s : arr.sensitivities) var_arr += s * s;
        double sigma_arr = std::sqrt(std::max(var_arr, 0.0));

        StatSlack ss;
        ss.endpoint = ep;
        ss.mean_slack = clock_period_ - mean_arr;
        ss.sigma_slack = sigma_arr;  // slack sigma = arrival sigma (clock is deterministic)
        // P(slack < 0) = P(arrival > clock_period) = 1 - Φ((T - μ)/σ)
        if (sigma_arr > 1e-15) {
            double z = (clock_period_ - mean_arr) / sigma_arr;
            ss.prob_violation = 1.0 - phi_cdf(z);
        } else {
            ss.prob_violation = (mean_arr > clock_period_) ? 1.0 : 0.0;
        }
        slacks.push_back(ss);
    }

    return slacks;
}

// ── Parametric yield estimation ──────────────────────────────────────
// From statistical slacks, yield = product of P(slack > 0) for all endpoints

YieldResult SstaEngine::estimate_yield(double clock_period) {
    double saved = clock_period_;
    clock_period_ = clock_period;

    auto slacks = compute_statistical_slacks();

    YieldResult yr;

    // Yield = product of P(slack >= 0) for all endpoints
    double yield_prob = 1.0;
    double worst_mean_arr = 0.0;
    double worst_var = 0.0;

    for (auto& ss : slacks) {
        double p_pass = 1.0 - ss.prob_violation;
        yield_prob *= p_pass;
        double mean_arr = clock_period_ - ss.mean_slack;
        if (mean_arr > worst_mean_arr) {
            worst_mean_arr = mean_arr;
            worst_var = ss.sigma_slack * ss.sigma_slack;
        }
    }

    yr.yield_pct = yield_prob * 100.0;
    yr.mean_delay = worst_mean_arr;
    yr.sigma_delay = std::sqrt(worst_var);
    yr.delay_3sigma = yr.mean_delay + 3.0 * yr.sigma_delay;

    // Generate histogram (20 bins from mean−4σ to mean+4σ)
    int nbins = 20;
    yr.delay_histogram.resize(nbins, 0.0);
    double lo = yr.mean_delay - 4.0 * yr.sigma_delay;
    double hi = yr.mean_delay + 4.0 * yr.sigma_delay;
    double bin_w = (hi - lo) / nbins;
    if (yr.sigma_delay > 1e-15 && bin_w > 0) {
        for (int i = 0; i < nbins; ++i) {
            double x = lo + (i + 0.5) * bin_w;
            yr.delay_histogram[i] = phi_pdf((x - yr.mean_delay) / yr.sigma_delay) * bin_w / yr.sigma_delay;
        }
    }

    clock_period_ = saved;
    return yr;
}

// ── Enhanced SSTA run ────────────────────────────────────────────────
// Combines canonical-delay propagation with Monte Carlo validation

SstaResult SstaEngine::run_enhanced() {
    auto t0 = std::chrono::high_resolution_clock::now();

    // First, run standard Monte Carlo
    auto result = run_monte_carlo();

    // Compute statistical slacks via canonical propagation
    auto slacks = compute_statistical_slacks();

    // Enhance yield estimate using analytical method
    double analytical_yield = 1.0;
    for (auto& ss : slacks) {
        double p_pass = 1.0 - ss.prob_violation;
        analytical_yield *= p_pass;
    }

    // Use analytical yield if MC yield is less precise (small sample count)
    if (config_.num_samples < 500) {
        result.yield_estimate = analytical_yield;
    } else {
        // Blend: 70% MC, 30% analytical for robustness
        result.yield_estimate = 0.7 * result.yield_estimate + 0.3 * analytical_yield;
    }

    // Try PCA reduction if spatial correlation enabled
    if (config_.enable_spatial_correlation && !has_pca_) {
        reduce_with_pca();
    }

    // Update summary
    auto t1 = std::chrono::high_resolution_clock::now();
    double enhanced_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.time_ms = enhanced_ms;

    std::ostringstream oss;
    oss << "Enhanced SSTA: " << config_.num_samples << " MC + canonical, "
        << "WNS mean=" << result.statistical_wns.mean_ps << "ps "
        << "sigma=" << result.statistical_wns.sigma_ps << "ps, "
        << "yield=" << (result.yield_estimate * 100.0) << "%, "
        << slacks.size() << " endpoints, ";
    if (has_pca_) {
        oss << "PCA " << pca_result_.original_sources << "→"
            << pca_result_.principal_components << " (" 
            << (pca_result_.variance_captured * 100.0) << "% var), ";
    }
    oss << "time=" << enhanced_ms << "ms";
    result.summary = oss.str();
    result.message = "Enhanced SSTA completed successfully";

    has_result_ = true;
    last_result_ = result;
    return result;
}

} // namespace sf
