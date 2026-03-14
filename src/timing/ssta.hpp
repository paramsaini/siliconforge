#pragma once
// SiliconForge — Statistical Static Timing Analysis (SSTA)
// Monte Carlo-based statistical timing with process variation modeling,
// spatial correlation, and yield estimation.
// Reference: Blaauw et al., "Statistical Timing Analysis: From Basic Principles
//            to State of the Art", IEEE TCAD 2008

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>
#include <chrono>

namespace sf {

// ── Configuration ────────────────────────────────────────────────────

struct SstaConfig {
    int    num_samples                   = 1000;
    double n_sigma                       = 3.0;
    double global_variation_pct          = 5.0;    // inter-die (%)
    double local_variation_pct           = 3.0;    // intra-die (%)
    double spatial_correlation_length_um = 100.0;
    bool   enable_spatial_correlation    = true;
    unsigned int seed                    = 42;
    double confidence_level              = 0.997;  // 3-sigma

    // Importance sampling: biases MC toward tail distribution for better
    // estimation of rare timing failures. Uses mean-shift IS with
    // likelihood ratio correction.
    // Reference: Singhee & Rutenbar, "Statistical Blockade", DAC 2007
    bool   enable_importance_sampling    = false;
    double tail_threshold                = -0.1;   // focus on paths with slack < threshold (ns)

    // Quasi-random sampling mode (Tier 2)
    // Sobol/Halton low-discrepancy sequences converge faster than pseudo-random.
    // For 1000 samples, Sobol typically gives 3-5× lower estimation error.
    enum class SamplingMode { PSEUDO_RANDOM, HALTON, SOBOL };
    SamplingMode sampling_mode = SamplingMode::PSEUDO_RANDOM;
};

// ── Process Variation Model ──────────────────────────────────────────

struct ProcessVariation {
    // Gate parameters (% sigma)
    double gate_length_sigma_pct     = 3.0;
    double vth_sigma_pct             = 4.0;
    double tox_sigma_pct             = 2.0;
    // Wire parameters (% sigma)
    double wire_width_sigma_pct      = 3.0;
    double wire_thickness_sigma_pct  = 2.0;

    // Global (inter-die) and local (intra-die) components
    double global_fraction = 0.6;  // fraction of total variation that is global
    double local_fraction  = 0.4;  // fraction that is local/random
};

// ── Statistical Delay ────────────────────────────────────────────────

struct StatisticalDelay {
    double mean_ps  = 0.0;
    double sigma_ps = 0.0;
    double min_ps   = 0.0;
    double max_ps   = 0.0;
    std::map<double, double> percentile_values;  // percentile → delay
    std::vector<double> samples;                 // raw MC samples

    void compute_from_samples() {
        if (samples.empty()) return;
        auto sorted = samples;
        std::sort(sorted.begin(), sorted.end());
        min_ps = sorted.front();
        max_ps = sorted.back();
        double sum = std::accumulate(sorted.begin(), sorted.end(), 0.0);
        mean_ps = sum / static_cast<double>(sorted.size());
        double sq_sum = 0.0;
        for (double v : sorted) sq_sum += (v - mean_ps) * (v - mean_ps);
        sigma_ps = std::sqrt(sq_sum / static_cast<double>(sorted.size()));

        // Standard percentiles
        for (double pct : {1.0, 5.0, 10.0, 25.0, 50.0, 75.0, 90.0, 95.0, 99.0}) {
            size_t idx = static_cast<size_t>(pct / 100.0 * (sorted.size() - 1));
            percentile_values[pct] = sorted[idx];
        }
    }
};

// ── Statistical Path ─────────────────────────────────────────────────

struct StatisticalPath {
    std::string path_name;
    double mean_delay_ps         = 0.0;
    double sigma_ps              = 0.0;
    double yield_probability     = 1.0;   // fraction meeting timing
    double worst_sample_delay_ps = 0.0;
    double best_sample_delay_ps  = 0.0;
    std::map<double, double> percentile_delays;  // percentile → delay
    std::vector<double> delay_samples;           // per-sample delays
};

// ── SSTA Result ──────────────────────────────────────────────────────

struct SstaResult {
    StatisticalDelay statistical_wns;
    StatisticalDelay statistical_tns;
    double yield_estimate = 0.0;              // fraction passing all timing
    std::map<double, double> yield_at_sigma;  // sigma → yield %
    int    num_samples_run = 0;
    double time_ms         = 0.0;
    std::vector<StatisticalPath> critical_paths;
    std::map<std::string, double> path_correlations;
    std::map<double, double> sigma_to_yield;  // sigma → yield %
    std::string message;
    std::string summary;
};

// ── Sensitivity Result ───────────────────────────────────────────────

struct SensitivityResult {
    std::map<std::string, double> parameter_impact;  // param name → impact (ps)
    std::string dominant_parameter;
    double dominant_impact_ps = 0.0;
};

// ── First-order canonical form (SOCV style) ─────────────────────────

struct CanonicalDelay {
    double nominal;
    std::vector<double> sensitivities;  // sensitivity to each variation source
    double random_sigma;                // independent random component
};

// ── Spatial correlation model ────────────────────────────────────────

struct SpatialCorrelation {
    double correlation_length;   // um, beyond this distance correlation drops
    std::vector<std::pair<double,double>> gate_positions; // x,y per gate
};

// ── PCA-based variation reduction ────────────────────────────────────

struct PcaResult {
    int original_sources;
    int principal_components;    // number of significant PCs
    double variance_captured;    // fraction of total variance captured
    std::vector<std::vector<double>> transform;  // PC transform matrix
};

// ── Parametric yield estimation ──────────────────────────────────────

struct YieldResult {
    double yield_pct;           // estimated yield
    double mean_delay;
    double sigma_delay;
    double delay_3sigma;        // mean + 3σ delay
    std::vector<double> delay_histogram;  // binned distribution
};

// ── Statistical slack ────────────────────────────────────────────────

struct StatSlack {
    int endpoint;
    double mean_slack;
    double sigma_slack;
    double prob_violation;   // P(slack < 0)
};

// ── SSTA Engine ──────────────────────────────────────────────────────

class SstaEngine {
public:
    SstaEngine(const Netlist& nl, const LibertyLibrary* lib = nullptr)
        : nl_(nl), lib_(lib) {}

    void set_config(const SstaConfig& cfg) { config_ = cfg; }
    const SstaConfig& config() const { return config_; }

    void set_clock_period(double period_ps) { clock_period_ = period_ps; }
    double clock_period() const { return clock_period_; }

    void set_process_variation(const ProcessVariation& pv) { pv_ = pv; }
    const ProcessVariation& process_variation() const { return pv_; }

    // Main analysis
    SstaResult run_monte_carlo();

    // Yield at a specific target period
    double compute_yield(double target_period_ps);

    // Statistical WNS distribution
    StatisticalDelay get_statistical_wns();

    // Sensitivity analysis: which variation parameter dominates
    SensitivityResult sensitivity_analysis();

    // ── Enhanced SSTA features ───────────────────────────────────────

    void set_spatial_correlation(const SpatialCorrelation& sc);

    PcaResult reduce_with_pca(int max_components = 10);

    YieldResult estimate_yield(double clock_period);

    std::vector<StatSlack> compute_statistical_slacks();

    SstaResult run_enhanced();

private:
    const Netlist& nl_;
    const LibertyLibrary* lib_;
    SstaConfig config_;
    ProcessVariation pv_;
    double clock_period_ = 1000.0;  // default 1 ns

    // Cached result from last run
    SstaResult last_result_;
    bool has_result_ = false;

    // Gate position cache for spatial correlation
    struct GatePos { double x = 0, y = 0; };
    std::unordered_map<GateId, GatePos> gate_positions_;

    // Spatial correlation model (user-supplied or auto-generated)
    SpatialCorrelation spatial_corr_;
    bool has_spatial_corr_ = false;

    // PCA state
    PcaResult pca_result_;
    bool has_pca_ = false;

    // Core MC helpers
    double nominal_gate_delay(GateId gid) const;
    double sample_gate_delay(GateId gid, double global_var, double local_var) const;
    double forward_propagate_sample(
        const std::vector<GateId>& topo,
        const std::unordered_map<GateId, double>& gate_delays) const;
    void assign_gate_positions();
    double spatial_correlation(GateId a, GateId b) const;
    std::vector<double> generate_correlated_locals(
        std::mt19937& rng, const std::vector<GateId>& gates) const;

    // Quasi-random sequence generators (Tier 2)
    // Halton sequence — base-prime low-discrepancy sequence
    static double halton_seq(int index, int base);
    // Inverse normal CDF (rational approximation, Abramowitz & Stegun)
    static double inv_normal_cdf(double u);
    // Generate quasi-random normal sample for dimension d, sample s
    double quasi_random_normal(int sample_idx, int dimension) const;

    // Enhanced helpers
    static double phi_cdf(double x);   // standard normal CDF Φ(x)
    static double phi_pdf(double x);   // standard normal PDF φ(x)
    static CanonicalDelay canonical_max(const CanonicalDelay& a, const CanonicalDelay& b);
    void build_correlation_matrix(const std::vector<GateId>& gates,
                                  std::vector<std::vector<double>>& corr) const;
};

} // namespace sf
