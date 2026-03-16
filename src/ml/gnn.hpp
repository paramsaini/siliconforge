#pragma once
// SiliconForge — Graph Neural Network for EDA
// Message-passing neural network operating on netlists represented as
// directed hypergraphs.  Each gate is a node, each net is a hyperedge.
//
// Applications:
//   - Congestion prediction from placement (pre-routing)
//   - Timing path delay estimation (pre-STA)
//   - Routability prediction
//   - Net criticality classification
//
// Architecture: GraphSAGE-style aggregation with gate-type embeddings,
// configurable message-passing rounds, and readout head for graph/node
// prediction.
//
// References:
//   Hamilton et al., "Inductive Representation Learning on Large Graphs",
//                     NeurIPS 2017
//   Mirhoseini et al., "A Graph Placement Methodology for Fast Chip Design",
//                       Nature 2021
//   Ghose et al., "Generalizable Cross-Graph Embedding for GNN-based
//                  Congestion Prediction", ICCAD 2023

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "ml/ml_opt.hpp"
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <random>
#include <functional>

namespace sf {

// ── Graph representation for GNN ────────────────────────────────────────

struct GnnGraph {
    int num_nodes = 0;
    int num_edges = 0;

    // Node features: [num_nodes][node_feat_dim]
    std::vector<std::vector<double>> node_features;

    // Edge list: pairs of (src, dst) indices
    std::vector<std::pair<int, int>> edges;

    // Edge features: [num_edges][edge_feat_dim]
    std::vector<std::vector<double>> edge_features;

    // Node labels (for supervised training)
    std::vector<double> node_labels;

    // Graph-level label (for graph classification/regression)
    double graph_label = 0;
};

// ── GNN Layer ───────────────────────────────────────────────────────────

struct GnnLayerConfig {
    int input_dim = 0;
    int output_dim = 0;
    int edge_feat_dim = 0;
    double dropout = 0.0;
    bool use_batch_norm = false;
    std::string aggregation = "mean";  // "mean", "max", "sum"
    std::string activation = "relu";   // "relu", "leaky_relu", "gelu"
};

class GnnLayer {
public:
    GnnLayer(const GnnLayerConfig& cfg);

    // Forward pass: message passing + aggregation + update
    // Input: node embeddings [N][in_dim]
    // Output: updated node embeddings [N][out_dim]
    std::vector<std::vector<double>> forward(
        const std::vector<std::vector<double>>& node_emb,
        const std::vector<std::pair<int, int>>& edges,
        const std::vector<std::vector<double>>& edge_feat = {},
        bool training = false) const;

    // Backward pass: compute gradients
    struct GnnGrad {
        std::vector<std::vector<double>> d_w_msg;    // message weight grad
        std::vector<std::vector<double>> d_w_upd;    // update weight grad
        std::vector<double> d_b_upd;                  // update bias grad
        std::vector<std::vector<double>> d_input;     // gradient w.r.t. input
    };
    GnnGrad backward(const std::vector<std::vector<double>>& node_emb,
                     const std::vector<std::vector<double>>& output,
                     const std::vector<std::vector<double>>& grad_output,
                     const std::vector<std::pair<int, int>>& edges) const;

    // Weight access
    int num_parameters() const;
    void init_weights(std::mt19937& rng);

    GnnLayerConfig config() const { return cfg_; }

private:
    GnnLayerConfig cfg_;

    // Message network: W_msg [in_dim][out_dim]
    std::vector<std::vector<double>> w_msg_;

    // Update network: W_upd [in_dim + out_dim][out_dim], b_upd [out_dim]
    std::vector<std::vector<double>> w_upd_;
    std::vector<double> b_upd_;

    // Batch normalization parameters
    std::vector<double> bn_gamma_, bn_beta_;
    mutable std::vector<double> bn_mean_, bn_var_;

    // Activation function
    double activate(double x) const;
    double activate_deriv(double x) const;

    // Aggregation
    std::vector<double> aggregate(
        const std::vector<std::vector<double>>& messages,
        const std::vector<int>& neighbor_indices) const;
};

// ── GNN Model ───────────────────────────────────────────────────────────

struct GnnConfig {
    int node_feat_dim = 16;        // input node feature dimension
    int edge_feat_dim = 4;         // input edge feature dimension
    std::vector<int> hidden_dims = {64, 64, 32};  // hidden layer dimensions
    int num_message_passes = 3;    // number of GNN layers
    int readout_dim = 16;          // readout MLP hidden dimension
    int output_dim = 1;            // final output dimension
    double learning_rate = 0.001;
    double weight_decay = 1e-5;
    int batch_size = 32;
    int epochs = 100;
    double dropout = 0.1;
    std::string task = "node_regression";  // "node_regression", "node_class", "graph_regression"
};

class GnnModel {
public:
    explicit GnnModel(const GnnConfig& cfg);

    // Build the model architecture
    void build();

    // Forward pass: graph -> predictions
    std::vector<double> predict(const GnnGraph& graph) const;

    // Training
    struct TrainResult {
        std::vector<double> loss_per_epoch;
        double final_loss = 0;
        double best_val_loss = 1e18;
        int best_epoch = 0;
        double time_ms = 0;
    };
    TrainResult train(const std::vector<GnnGraph>& train_graphs,
                      const std::vector<GnnGraph>& val_graphs = {});

    // Evaluate: returns MSE or accuracy depending on task
    double evaluate(const std::vector<GnnGraph>& graphs) const;

    // Save/load model weights
    void save_weights(const std::string& path) const;
    bool load_weights(const std::string& path);

    // Statistics
    int total_parameters() const;
    const GnnConfig& config() const { return cfg_; }

private:
    GnnConfig cfg_;
    std::vector<GnnLayer> layers_;
    bool built_ = false;

    // Readout MLP (graph-level prediction)
    MLP readout_mlp_;

    // Node embeddings (cached from last forward pass)
    mutable std::vector<std::vector<double>> last_embeddings_;

    // Loss computation
    double compute_loss(const std::vector<double>& predictions,
                        const std::vector<double>& targets) const;
};

// ── Netlist-to-Graph conversion ─────────────────────────────────────────

class NetlistGraphBuilder {
public:
    // Convert a placed netlist into a GNN graph
    // Node features: gate type embedding, area, fanin/fanout count,
    //                x/y position, timing criticality
    // Edge features: net HPWL, pin offset, criticality
    GnnGraph build_graph(const Netlist& nl, const PhysicalDesign& pd) const;

    // Feature engineering configuration
    struct FeatureConfig {
        bool include_position = true;
        bool include_timing = true;
        bool include_power = false;
        int gate_type_embedding_dim = 8;
        int max_gate_types = 32;
    };
    void set_feature_config(const FeatureConfig& cfg) { feat_cfg_ = cfg; }

    // Gate type vocabulary (for embedding)
    void add_gate_type(const std::string& type, int id) { gate_type_map_[type] = id; }

private:
    FeatureConfig feat_cfg_;
    std::unordered_map<std::string, int> gate_type_map_;

    std::vector<double> encode_gate_features(const Netlist& nl,
                                              const PhysicalDesign& pd,
                                              int gate_id) const;
    std::vector<double> encode_edge_features(const Netlist& nl,
                                              const PhysicalDesign& pd,
                                              int net_id) const;
};

// ── Pre-built GNN applications ──────────────────────────────────────────

// Congestion prediction: given placement, predict per-bin routing congestion
class GnnCongestionPredictor {
public:
    GnnCongestionPredictor();

    // Train on historical placement -> congestion data
    void train(const std::vector<std::pair<GnnGraph, std::vector<double>>>& data);

    // Predict congestion map from current placement
    std::vector<double> predict(const Netlist& nl, const PhysicalDesign& pd);

    GnnModel& model() { return model_; }

private:
    GnnModel model_;
    NetlistGraphBuilder builder_;
};

// Timing prediction: predict path delays from netlist structure
class GnnTimingPredictor {
public:
    GnnTimingPredictor();

    // Predict per-node arrival times
    std::vector<double> predict_arrival(const Netlist& nl, const PhysicalDesign& pd);

    GnnModel& model() { return model_; }

private:
    GnnModel model_;
    NetlistGraphBuilder builder_;
};

} // namespace sf
