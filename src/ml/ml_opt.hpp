#pragma once
// SiliconForge — ML Optimization Engine
// Phase 42: Real ML-based EDA optimization
//
// Implements from scratch (no external deps):
//   1. Multi-Layer Perceptron (MLP) — forward/backprop, mini-batch SGD
//   2. Bayesian Optimization — Gaussian Process surrogate + EI acquisition
//   3. Q-Learning RL — tabular RL for placement optimization
//   4. Ensemble predictor — weighted combination of models
//   5. Feature extraction — from physical design to ML features
//
// References:
//   - Mirhoseini et al., "A Graph Placement Methodology for Fast Chip Design", Nature 2021
//   - Xie et al., "RouteNet: Routability Prediction for Mixed-Size Designs", DAC 2018

#include "pnr/physical.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <random>
#include <map>
#include <functional>
#include <algorithm>
#include <numeric>

namespace sf {

// ── Activation Functions ────────────────────────────────────────────────────
enum class Activation { RELU, SIGMOID, TANH, LINEAR };

inline double activate(double x, Activation act) {
    switch (act) {
        case Activation::RELU:    return std::max(0.0, x);
        case Activation::SIGMOID: return 1.0 / (1.0 + std::exp(-std::clamp(x, -500.0, 500.0)));
        case Activation::TANH:    return std::tanh(x);
        case Activation::LINEAR:  return x;
    }
    return x;
}

inline double activate_deriv(double y, Activation act) {
    switch (act) {
        case Activation::RELU:    return y > 0 ? 1.0 : 0.0;
        case Activation::SIGMOID: return y * (1.0 - y);
        case Activation::TANH:    return 1.0 - y * y;
        case Activation::LINEAR:  return 1.0;
    }
    return 1.0;
}

// ── Dense Layer ─────────────────────────────────────────────────────────────
struct DenseLayer {
    int input_size = 0;
    int output_size = 0;
    Activation activation = Activation::RELU;
    std::vector<std::vector<double>> weights;  // [output][input]
    std::vector<double> biases;                // [output]
    // Cached for backprop
    std::vector<double> pre_activation;        // z values
    std::vector<double> post_activation;       // a values (output)
    std::vector<double> input_cache;

    void init(int in, int out, Activation act, std::mt19937& rng);
    std::vector<double> forward(const std::vector<double>& input);
    std::vector<double> backward(const std::vector<double>& grad_output, double lr);
};

// ── Multi-Layer Perceptron ──────────────────────────────────────────────────
struct MlpConfig {
    std::vector<int> layer_sizes = {8, 32, 16, 1};  // input → hidden → output
    std::vector<Activation> activations = {Activation::RELU, Activation::RELU, Activation::LINEAR};
    double learning_rate = 0.001;
    int epochs = 100;
    int batch_size = 32;
    double l2_reg = 0.0001;    // L2 regularization
    unsigned seed = 42;
};

struct TrainResult {
    std::vector<double> loss_history;
    double final_loss = 0;
    int epochs_trained = 0;
    double time_ms = 0;
};

class MLP {
public:
    explicit MLP(const MlpConfig& cfg = {});

    // Train on dataset
    TrainResult train(const std::vector<std::vector<double>>& X,
                      const std::vector<double>& y);

    // Predict single sample
    double predict(const std::vector<double>& x) const;

    // Predict batch
    std::vector<double> predict_batch(const std::vector<std::vector<double>>& X) const;

    // Get/set weights for transfer learning
    std::vector<double> get_flat_weights() const;
    void set_flat_weights(const std::vector<double>& w);

    int num_parameters() const;
    const MlpConfig& config() const { return cfg_; }

private:
    MlpConfig cfg_;
    std::vector<DenseLayer> layers_;
    mutable std::mt19937 rng_;
};

// ── Gaussian Process (for Bayesian Optimization) ────────────────────────────
struct GpConfig {
    double length_scale = 1.0;
    double signal_variance = 1.0;
    double noise_variance = 0.01;
};

class GaussianProcess {
public:
    explicit GaussianProcess(const GpConfig& cfg = {});

    void add_observation(const std::vector<double>& x, double y);
    void clear();

    // Predict mean and variance at new point
    struct Prediction { double mean = 0; double variance = 1.0; };
    Prediction predict(const std::vector<double>& x) const;

    // Expected Improvement acquisition function
    double expected_improvement(const std::vector<double>& x, double best_y) const;

    int num_observations() const { return (int)X_.size(); }

private:
    GpConfig cfg_;
    std::vector<std::vector<double>> X_;
    std::vector<double> y_;

    double kernel(const std::vector<double>& a, const std::vector<double>& b) const;
    std::vector<std::vector<double>> compute_K() const;
    std::vector<double> solve_linear(const std::vector<std::vector<double>>& K,
                                     const std::vector<double>& b) const;
};

// ── Bayesian Optimizer ──────────────────────────────────────────────────────
struct BayesOptConfig {
    int max_iterations = 50;
    int initial_random = 5;    // random samples before GP kicks in
    int candidate_samples = 100; // candidates per iteration
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    unsigned seed = 42;
    GpConfig gp_config;
};

struct BayesOptResult {
    std::vector<double> best_params;
    double best_value = -1e18;
    int iterations = 0;
    std::vector<double> value_history;
    double time_ms = 0;
};

class BayesianOptimizer {
public:
    explicit BayesianOptimizer(const BayesOptConfig& cfg);

    // Optimize a black-box objective function
    BayesOptResult optimize(std::function<double(const std::vector<double>&)> objective);

    const GaussianProcess& gp() const { return gp_; }

private:
    BayesOptConfig cfg_;
    GaussianProcess gp_;
    std::mt19937 rng_;

    std::vector<double> random_sample();
    std::vector<double> best_ei_candidate(double best_y);
};

// ── Q-Learning RL Agent ─────────────────────────────────────────────────────
struct QConfig {
    double alpha = 0.1;       // learning rate
    double gamma = 0.95;      // discount factor
    double epsilon = 0.3;     // exploration rate
    double epsilon_decay = 0.995;
    double min_epsilon = 0.01;
    int num_actions = 8;      // action space size
    int num_states = 64;      // state space size (discretized)
    unsigned seed = 42;
};

struct QResult {
    std::vector<double> reward_history;
    double total_reward = 0;
    int episodes = 0;
    int best_action = 0;
    double best_q_value = -1e18;
};

class QLearner {
public:
    explicit QLearner(const QConfig& cfg = {});

    // Select action (epsilon-greedy)
    int select_action(int state);

    // Update Q-value after taking action
    void update(int state, int action, double reward, int next_state);

    // Get Q-value
    double q_value(int state, int action) const;

    // Get best action for state
    int best_action(int state) const;

    // Decay exploration rate
    void decay_epsilon();

    // Run episode (with external environment step function)
    QResult run_episode(int initial_state, int max_steps,
                        std::function<std::pair<int, double>(int, int)> step_fn);

    const QConfig& config() const { return cfg_; }

private:
    QConfig cfg_;
    std::vector<std::vector<double>> Q_; // Q-table [states][actions]
    std::mt19937 rng_;
};

// ── Feature Extractor (Physical Design → ML features) ───────────────────────
struct DesignFeatures {
    int num_cells = 0;
    int num_nets = 0;
    double utilization = 0;
    double total_hpwl = 0;
    double avg_net_degree = 0;
    double max_net_degree = 0;
    double cell_density_variance = 0;
    double aspect_ratio = 0;

    // Convert to feature vector
    std::vector<double> to_vector() const;
};

class FeatureExtractor {
public:
    static DesignFeatures extract(const PhysicalDesign& pd);
    static std::vector<double> extract_net_features(const PhysicalDesign& pd, int net_id);
    static std::vector<double> extract_cell_features(const PhysicalDesign& pd, int cell_id);
};

// ── Ensemble Predictor ──────────────────────────────────────────────────────
class EnsemblePredictor {
public:
    // Add a model prediction
    void add_prediction(const std::string& model_name, double value, double confidence = 1.0);
    void clear();

    // Weighted ensemble average
    double predict() const;
    // Prediction with uncertainty
    double uncertainty() const;

    int num_models() const { return (int)predictions_.size(); }

private:
    struct ModelPred { std::string name; double value; double confidence; };
    std::vector<ModelPred> predictions_;
};

// ── ML Optimization Result ──────────────────────────────────────────────────
struct MlOptResult {
    // Best found parameters
    double optimal_density_weight = 0;
    double optimal_wirelength_weight = 0;
    double optimal_timing_weight = 0;

    // Quality metrics
    double predicted_wns = 0;
    double predicted_congestion = 0;
    double predicted_wirelength = 0;

    // Training stats
    int bayesopt_iterations = 0;
    int rl_episodes = 0;
    int mlp_epochs = 0;
    double total_time_ms = 0;

    // Model quality
    double mlp_loss = 0;
    double bayesopt_best = 0;
    double rl_best_reward = 0;
};

// ── Top-Level ML Optimization Engine ────────────────────────────────────────
class MlOptEngine {
public:
    MlOptEngine(PhysicalDesign& pd);

    // Full ML optimization pipeline
    MlOptResult optimize(int bayesopt_iters = 30, int rl_episodes = 50,
                         int mlp_epochs = 50);

    // Individual components
    BayesOptResult run_bayesian_opt(int iterations = 30);
    QResult run_rl_optimization(int episodes = 50);
    TrainResult train_surrogate(const std::vector<std::vector<double>>& X,
                                const std::vector<double>& y);

    // Feature extraction
    DesignFeatures extract_features() const;

private:
    PhysicalDesign& pd_;
    MLP surrogate_;
    GaussianProcess gp_;
    QLearner rl_;
};

} // namespace sf
