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

} // namespace sf
