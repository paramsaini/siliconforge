// SiliconForge — Phase 42: ML-Based Optimization Tests
// 25 tests: MLP, Gaussian Process, BayesOpt, Q-Learning, Feature Extraction,
// Ensemble, Activation functions, Transfer learning, E2E pipeline

#include "ml/ml_opt.hpp"
#include "pnr/physical.hpp"
#include <iostream>
#include <cmath>
#include <string>

static int tests_run = 0, tests_passed = 0;

#define RUN(name) do { \
    tests_run++; \
    std::cout << "  [" << tests_run << "] " << name << "... "; \
    std::cout.flush(); \
} while(0)

#define PASS() do { \
    tests_passed++; \
    std::cout << "PASS\n"; \
} while(0)

#define CHECK(cond) do { \
    if (!(cond)) { \
        std::cout << "FAIL (" #cond ") at line " << __LINE__ << "\n"; \
        return; \
    } \
} while(0)

#define CHECKF(val, lo, hi) do { \
    double v_ = (val); \
    if (v_ < (lo) || v_ > (hi)) { \
        std::cout << "FAIL (" #val "=" << v_ << " not in [" << (lo) << "," << (hi) << "]) line " << __LINE__ << "\n"; \
        return; \
    } \
} while(0)

// ── Test 1: Activation Functions ────────────────────────────────────────────
void test_activations() {
    RUN("Activation functions (ReLU/Sigmoid/Tanh)");
    CHECK(sf::activate(-1.0, sf::Activation::RELU) == 0);
    CHECK(sf::activate(2.0, sf::Activation::RELU) == 2.0);
    CHECKF(sf::activate(0.0, sf::Activation::SIGMOID), 0.49, 0.51);
    CHECKF(sf::activate(100.0, sf::Activation::SIGMOID), 0.99, 1.01);
    CHECKF(sf::activate(0.0, sf::Activation::TANH), -0.01, 0.01);
    CHECK(sf::activate(5.0, sf::Activation::LINEAR) == 5.0);

    // Derivatives
    CHECK(sf::activate_deriv(0.5, sf::Activation::SIGMOID) > 0);
    CHECK(sf::activate_deriv(1.0, sf::Activation::RELU) == 1.0);
    CHECK(sf::activate_deriv(0.0, sf::Activation::RELU) == 0.0);
    PASS();
}

// ── Test 2: Dense Layer Forward ─────────────────────────────────────────────
void test_dense_forward() {
    RUN("Dense layer forward pass");
    std::mt19937 rng(42);
    sf::DenseLayer layer;
    layer.init(3, 2, sf::Activation::RELU, rng);

    auto output = layer.forward({1.0, 2.0, 3.0});
    CHECK(output.size() == 2);
    // Output should be non-negative (ReLU)
    CHECK(output[0] >= 0);
    CHECK(output[1] >= 0);
    PASS();
}

// ── Test 3: Dense Layer Backward ────────────────────────────────────────────
void test_dense_backward() {
    RUN("Dense layer backward pass");
    std::mt19937 rng(42);
    sf::DenseLayer layer;
    layer.init(3, 2, sf::Activation::LINEAR, rng);

    auto out = layer.forward({1.0, 0.5, -0.5});
    auto grad = layer.backward({1.0, 1.0}, 0.01);
    CHECK(grad.size() == 3);
    // Gradients should propagate back
    CHECK(std::abs(grad[0]) > 0 || std::abs(grad[1]) > 0 || std::abs(grad[2]) > 0);
    PASS();
}

// ── Test 4: MLP Construction ────────────────────────────────────────────────
void test_mlp_construction() {
    RUN("MLP construction and parameter count");
    sf::MlpConfig cfg;
    cfg.layer_sizes = {4, 8, 4, 1};
    cfg.activations = {sf::Activation::RELU, sf::Activation::RELU, sf::Activation::LINEAR};

    sf::MLP mlp(cfg);
    // Params: 4×8+8 + 8×4+4 + 4×1+1 = 40+36+5 = 81
    CHECK(mlp.num_parameters() == 81);
    PASS();
}

// ── Test 5: MLP Predict ─────────────────────────────────────────────────────
void test_mlp_predict() {
    RUN("MLP forward prediction");
    sf::MlpConfig cfg;
    cfg.layer_sizes = {3, 8, 1};
    cfg.activations = {sf::Activation::RELU, sf::Activation::LINEAR};
    sf::MLP mlp(cfg);

    double pred = mlp.predict({1.0, 2.0, 3.0});
    CHECK(std::isfinite(pred));
    PASS();
}

// ── Test 6: MLP Training ────────────────────────────────────────────────────
void test_mlp_training() {
    RUN("MLP training (loss decreases)");
    sf::MlpConfig cfg;
    cfg.layer_sizes = {2, 16, 1};
    cfg.activations = {sf::Activation::RELU, sf::Activation::LINEAR};
    cfg.learning_rate = 0.01;
    cfg.epochs = 100;
    cfg.batch_size = 4;

    sf::MLP mlp(cfg);

    // Simple function: y = x1 + x2
    std::vector<std::vector<double>> X = {
        {0,0}, {1,0}, {0,1}, {1,1}, {2,0}, {0,2}, {2,1}, {1,2}
    };
    std::vector<double> y = {0, 1, 1, 2, 2, 2, 3, 3};

    auto result = mlp.train(X, y);

    CHECK(result.epochs_trained == 100);
    CHECK(result.loss_history.size() == 100);
    // Loss should decrease
    CHECK(result.loss_history.back() < result.loss_history.front());
    PASS();
}

// ── Test 7: MLP Batch Predict ───────────────────────────────────────────────
void test_mlp_batch_predict() {
    RUN("MLP batch prediction");
    sf::MlpConfig cfg;
    cfg.layer_sizes = {2, 8, 1};
    cfg.activations = {sf::Activation::RELU, sf::Activation::LINEAR};
    sf::MLP mlp(cfg);

    std::vector<std::vector<double>> X = {{1,2}, {3,4}, {5,6}};
    auto preds = mlp.predict_batch(X);
    CHECK(preds.size() == 3);
    for (double p : preds) CHECK(std::isfinite(p));
    PASS();
}

// ── Test 8: MLP Weight Transfer ─────────────────────────────────────────────
void test_mlp_weight_transfer() {
    RUN("MLP weight get/set (transfer learning)");
    sf::MlpConfig cfg;
    cfg.layer_sizes = {2, 4, 1};
    sf::MLP mlp1(cfg), mlp2(cfg);

    // Get weights from mlp1, set to mlp2
    auto w = mlp1.get_flat_weights();
    CHECK(w.size() == mlp1.num_parameters());

    mlp2.set_flat_weights(w);
    // Both should give same prediction now
    double p1 = mlp1.predict({1.0, 2.0});
    double p2 = mlp2.predict({1.0, 2.0});
    CHECKF(p2, p1 - 0.001, p1 + 0.001);
    PASS();
}

// ── Test 9: Gaussian Process Predict ────────────────────────────────────────
void test_gp_predict() {
    RUN("Gaussian Process prediction");
    sf::GaussianProcess gp;

    gp.add_observation({0.0}, 0.0);
    gp.add_observation({1.0}, 1.0);
    gp.add_observation({2.0}, 4.0);

    // Predict at known point — should be close
    auto p1 = gp.predict({1.0});
    CHECKF(p1.mean, 0.5, 1.5);
    CHECK(p1.variance >= 0);

    // Predict at interpolation point
    auto p_mid = gp.predict({0.5});
    CHECK(std::isfinite(p_mid.mean));
    CHECK(p_mid.variance >= 0);
    PASS();
}

// ── Test 10: GP Uncertainty ─────────────────────────────────────────────────
void test_gp_uncertainty() {
    RUN("GP uncertainty (far points = high variance)");
    sf::GaussianProcess gp;
    gp.add_observation({0.0}, 0.0);
    gp.add_observation({1.0}, 1.0);

    auto p_near = gp.predict({0.5});  // between observations
    auto p_far = gp.predict({10.0});  // far from observations

    // Far point should have higher uncertainty
    CHECK(p_far.variance > p_near.variance);
    PASS();
}

// ── Test 11: Expected Improvement ───────────────────────────────────────────
void test_expected_improvement() {
    RUN("Expected Improvement acquisition function");
    sf::GaussianProcess gp;
    gp.add_observation({0.0}, 0.0);
    gp.add_observation({1.0}, 1.0);

    double best_y = 1.0;
    // EI should be non-negative
    double ei_known = gp.expected_improvement({1.0}, best_y);
    CHECK(ei_known >= 0);

    // EI at unexplored region should be positive
    double ei_far = gp.expected_improvement({5.0}, best_y);
    CHECK(ei_far >= 0);
    PASS();
}

// ── Test 12: Bayesian Optimization ──────────────────────────────────────────
void test_bayesian_optimization() {
    RUN("Bayesian optimization (simple 1D)");
    sf::BayesOptConfig cfg;
    cfg.max_iterations = 20;
    cfg.initial_random = 5;
    cfg.lower_bounds = {-5.0};
    cfg.upper_bounds = {5.0};
    cfg.candidate_samples = 50;

    sf::BayesianOptimizer opt(cfg);

    // Maximize f(x) = -(x-2)² + 10  → peak at x=2, value=10
    auto result = opt.optimize([](const std::vector<double>& x) {
        return -(x[0] - 2.0) * (x[0] - 2.0) + 10.0;
    });

    CHECK(result.iterations == 20);
    CHECK(result.best_value > 5.0);  // should find near peak
    CHECK(result.best_params.size() == 1);
    CHECKF(result.best_params[0], -1.0, 5.0); // near x=2
    PASS();
}

// ── Test 13: BayesOpt Multi-Dimensional ─────────────────────────────────────
void test_bayesopt_multidim() {
    RUN("BayesOpt multi-dimensional optimization");
    sf::BayesOptConfig cfg;
    cfg.max_iterations = 30;
    cfg.initial_random = 8;
    cfg.lower_bounds = {0.0, 0.0};
    cfg.upper_bounds = {4.0, 4.0};
    cfg.candidate_samples = 100;

    sf::BayesianOptimizer opt(cfg);

    // Maximize f(x,y) = -(x-1)² -(y-3)² + 20
    auto result = opt.optimize([](const std::vector<double>& x) {
        return -(x[0] - 1.0) * (x[0] - 1.0) - (x[1] - 3.0) * (x[1] - 3.0) + 20.0;
    });

    CHECK(result.best_value > 10.0);
    CHECK(result.value_history.size() == 30);
    PASS();
}

// ── Test 14: Q-Learning Basic ───────────────────────────────────────────────
void test_q_learning_basic() {
    RUN("Q-Learning basic update");
    sf::QConfig cfg;
    cfg.num_states = 4;
    cfg.num_actions = 3;
    cfg.alpha = 0.5;
    cfg.gamma = 0.9;
    cfg.epsilon = 0.0; // pure exploit for testing

    sf::QLearner ql(cfg);

    // Initial Q-values should be 0
    CHECK(ql.q_value(0, 0) == 0);

    // Update: state=0, action=1, reward=10, next_state=1
    ql.update(0, 1, 10.0, 1);
    CHECK(ql.q_value(0, 1) > 0);

    // Best action should now be 1
    CHECK(ql.best_action(0) == 1);
    PASS();
}

// ── Test 15: Q-Learning Episode ─────────────────────────────────────────────
void test_q_learning_episode() {
    RUN("Q-Learning full episode");
    sf::QConfig cfg;
    cfg.num_states = 8;
    cfg.num_actions = 4;
    cfg.epsilon = 0.5;

    sf::QLearner ql(cfg);

    // Simple environment: reward = 1 if action == state%4
    auto step = [](int state, int action) -> std::pair<int, double> {
        double reward = (action == state % 4) ? 1.0 : -0.1;
        int next = (state + 1) % 8;
        return {next, reward};
    };

    auto result = ql.run_episode(0, 20, step);
    CHECK(result.episodes == 1);
    CHECK(result.reward_history.size() == 20);
    CHECK(std::isfinite(result.total_reward));
    PASS();
}

// ── Test 16: Q-Learning Epsilon Decay ───────────────────────────────────────
void test_q_epsilon_decay() {
    RUN("Q-Learning epsilon decay");
    sf::QConfig cfg;
    cfg.epsilon = 0.5;
    cfg.epsilon_decay = 0.9;
    cfg.min_epsilon = 0.01;

    sf::QLearner ql(cfg);
    double eps_before = cfg.epsilon;
    ql.decay_epsilon();
    // Epsilon should decrease
    CHECK(ql.config().epsilon < eps_before);
    CHECK(ql.config().epsilon >= cfg.min_epsilon);
    PASS();
}

// ── Test 17: Feature Extraction ─────────────────────────────────────────────
void test_feature_extraction() {
    RUN("Feature extraction from physical design");
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 1000, 1000);
    for (int i = 0; i < 10; i++) {
        int c = pd.add_cell("c" + std::to_string(i), "NAND", 3, 3);
        pd.cells[c].position = sf::Point(i * 100, 500);
        pd.cells[c].placed = true;
    }
    pd.add_net("n0", {0, 1, 2});
    pd.add_net("n1", {3, 4});

    auto features = sf::FeatureExtractor::extract(pd);
    CHECK(features.num_cells == 10);
    CHECK(features.num_nets == 2);
    CHECK(features.aspect_ratio > 0);

    auto fv = features.to_vector();
    CHECK(fv.size() == 8);
    PASS();
}

// ── Test 18: Net Features ───────────────────────────────────────────────────
void test_net_features() {
    RUN("Per-net feature extraction");
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 1000, 1000);
    int c0 = pd.add_cell("a", "INV", 2, 2);
    int c1 = pd.add_cell("b", "BUF", 2, 2);
    pd.cells[c0].position = sf::Point(0, 0);
    pd.cells[c1].position = sf::Point(100, 200);
    pd.add_net("n", {c0, c1});

    auto nf = sf::FeatureExtractor::extract_net_features(pd, 0);
    CHECK(nf.size() == 4);
    CHECK(nf[0] == 2); // degree = 2
    CHECK(nf[1] > 0);  // HPWL > 0
    PASS();
}

// ── Test 19: Cell Features ──────────────────────────────────────────────────
void test_cell_features() {
    RUN("Per-cell feature extraction");
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 1000, 1000);
    int c = pd.add_cell("cell", "NAND", 5, 3);
    pd.cells[c].position = sf::Point(100, 200);
    pd.cells[c].placed = true;

    auto cf = sf::FeatureExtractor::extract_cell_features(pd, c);
    CHECK(cf.size() == 5);
    CHECK(cf[0] == 100); // x
    CHECK(cf[1] == 200); // y
    CHECK(cf[2] == 5);   // width
    CHECK(cf[3] == 3);   // height
    CHECK(cf[4] == 1.0); // placed
    PASS();
}

// ── Test 20: Ensemble Predictor ─────────────────────────────────────────────
void test_ensemble() {
    RUN("Ensemble weighted prediction");
    sf::EnsemblePredictor ens;
    ens.add_prediction("mlp", 10.0, 1.0);
    ens.add_prediction("gp", 12.0, 2.0);
    ens.add_prediction("rudy", 8.0, 0.5);

    CHECK(ens.num_models() == 3);
    // Weighted avg: (10×1 + 12×2 + 8×0.5)/(1+2+0.5) = 38/3.5 ≈ 10.86
    CHECKF(ens.predict(), 10.5, 11.2);
    CHECK(ens.uncertainty() > 0);
    PASS();
}

// ── Test 21: Ensemble Edge Cases ────────────────────────────────────────────
void test_ensemble_edge() {
    RUN("Ensemble edge cases");
    sf::EnsemblePredictor ens;

    // Empty ensemble
    CHECK(ens.predict() == 0);
    CHECK(ens.uncertainty() == 0);

    // Single model
    ens.add_prediction("only", 5.0, 1.0);
    CHECKF(ens.predict(), 4.99, 5.01);

    // Clear and re-add
    ens.clear();
    CHECK(ens.num_models() == 0);
    PASS();
}

// ── Test 22: GP Clear and Reuse ─────────────────────────────────────────────
void test_gp_clear_reuse() {
    RUN("GP clear and reuse");
    sf::GaussianProcess gp;
    gp.add_observation({1.0}, 5.0);
    CHECK(gp.num_observations() == 1);
    gp.clear();
    CHECK(gp.num_observations() == 0);

    // Reuse
    gp.add_observation({0.0}, 0.0);
    gp.add_observation({2.0}, 8.0);
    auto p = gp.predict({1.0});
    CHECK(std::isfinite(p.mean));
    CHECK(p.variance > 0);
    PASS();
}

// ── Test 23: ML Opt Engine Construction ─────────────────────────────────────
void test_ml_engine_construction() {
    RUN("ML optimization engine construction");
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 1000, 1000);
    for (int i = 0; i < 5; i++) {
        int c = pd.add_cell("c" + std::to_string(i), "INV", 2, 2);
        pd.cells[c].position = sf::Point(i * 100, 500);
    }

    sf::MlOptEngine engine(pd);
    auto features = engine.extract_features();
    CHECK(features.num_cells == 5);
    PASS();
}

// ── Test 24: Edge Cases ─────────────────────────────────────────────────────
void test_edge_cases() {
    RUN("Edge cases (empty MLP, out-of-bounds Q)");

    // MLP with single layer (no hidden)
    sf::MlpConfig cfg;
    cfg.layer_sizes = {2, 1};
    cfg.activations = {sf::Activation::LINEAR};
    sf::MLP tiny(cfg);
    double p = tiny.predict({1.0, 2.0});
    CHECK(std::isfinite(p));

    // Q-Learning out of bounds
    sf::QLearner ql;
    CHECK(ql.q_value(-1, 0) == 0);
    CHECK(ql.q_value(999, 0) == 0);
    CHECK(ql.best_action(-1) == 0);

    // Empty feature extraction
    sf::PhysicalDesign empty_pd;
    empty_pd.die_area = sf::Rect(0, 0, 100, 100);
    auto ef = sf::FeatureExtractor::extract(empty_pd);
    CHECK(ef.num_cells == 0);
    CHECK(ef.num_nets == 0);

    // Empty net/cell features
    auto nf = sf::FeatureExtractor::extract_net_features(empty_pd, 0);
    CHECK(nf.empty());
    auto cf = sf::FeatureExtractor::extract_cell_features(empty_pd, 0);
    CHECK(cf.empty());
    PASS();
}

// ── Test 25: E2E ML Optimization Pipeline ───────────────────────────────────
void test_e2e_ml_pipeline() {
    RUN("E2E: Full ML optimization pipeline");

    // Create a realistic physical design
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 5000, 5000);
    for (int i = 0; i < 20; i++) {
        int c = pd.add_cell("cell_" + std::to_string(i), "NAND", 5, 5);
        pd.cells[c].position = sf::Point(250 + (i % 5) * 1000, 250 + (i / 5) * 1000);
        pd.cells[c].placed = true;
    }
    for (int i = 0; i < 10; i++) {
        pd.add_net("net_" + std::to_string(i), {i, (i + 1) % 20});
    }

    // Run full ML optimization
    sf::MlOptEngine engine(pd);
    auto result = engine.optimize(15, 20, 30); // reduced for test speed

    // Verify all components ran
    CHECK(result.bayesopt_iterations == 15);
    CHECK(result.rl_episodes == 20);
    CHECK(result.mlp_epochs == 30);
    CHECK(result.total_time_ms > 0);

    // Parameters should be reasonable
    CHECK(result.optimal_density_weight >= 0);
    CHECK(result.optimal_wirelength_weight >= 0);
    CHECK(std::isfinite(result.bayesopt_best));
    CHECK(std::isfinite(result.rl_best_reward));
    CHECK(std::isfinite(result.mlp_loss));

    std::cout << "PASS [BayesOpt=" << result.bayesopt_best
              << ", RL_reward=" << result.rl_best_reward
              << ", MLP_loss=" << result.mlp_loss
              << ", time=" << result.total_time_ms << "ms]\n";
    tests_passed++;
}

// ── Main ────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << "  Phase 42: ML-Based Optimization Tests\n";
    std::cout << "═══════════════════════════════════════════════════════════\n\n";

    test_activations();
    test_dense_forward();
    test_dense_backward();
    test_mlp_construction();
    test_mlp_predict();
    test_mlp_training();
    test_mlp_batch_predict();
    test_mlp_weight_transfer();
    test_gp_predict();
    test_gp_uncertainty();
    test_expected_improvement();
    test_bayesian_optimization();
    test_bayesopt_multidim();
    test_q_learning_basic();
    test_q_learning_episode();
    test_q_epsilon_decay();
    test_feature_extraction();
    test_net_features();
    test_cell_features();
    test_ensemble();
    test_ensemble_edge();
    test_gp_clear_reuse();
    test_ml_engine_construction();
    test_edge_cases();
    test_e2e_ml_pipeline();

    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << "  Results: " << tests_passed << "/" << tests_run << " PASSED\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
