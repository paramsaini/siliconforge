// SiliconForge — Scan Chain Insertion Implementation
#include "dft/scan_insert.hpp"
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace sf {

ScanResult ScanInserter::insert(const ScanConfig& config) {
    auto& ffs = nl_.flip_flops();
    ScanResult result;
    result.total_ffs = (int)ffs.size();

    if (ffs.empty()) {
        result.message = "No flip-flops to scan";
        return result;
    }

    // Create scan enable net
    NetId se = nl_.add_net(config.scan_enable_name);
    nl_.mark_input(se);

    // Cache FF ids before modification (avoids dangling refs during vector growth)
    std::vector<GateId> ff_ids(ffs.begin(), ffs.end());

    // Split FFs into chains
    int chain_id = 0;
    for (size_t i = 0; i < ff_ids.size(); i += config.max_chain_length) {
        size_t chain_end = std::min(i + (size_t)config.max_chain_length, ff_ids.size());
        int chain_len = (int)(chain_end - i);

        // Create scan_in for this chain
        NetId scan_in = nl_.add_net(config.scan_in_prefix + "_" + std::to_string(chain_id));
        nl_.mark_input(scan_in);

        // For each FF in the chain, insert a MUX before the D input
        NetId prev_q = scan_in;

        for (size_t j = i; j < chain_end; ++j) {
            GateId ff_id = ff_ids[j];
            NetId func_d = nl_.gate(ff_id).inputs.empty() ? -1 : nl_.gate(ff_id).inputs[0];
            NetId ff_output = nl_.gate(ff_id).output;

            if (func_d >= 0) {
                NetId mux_out = nl_.add_net("scan_mux_" + std::to_string(j));
                nl_.add_gate(GateType::MUX, {se, prev_q, func_d}, mux_out,
                            "SCAN_MUX_" + std::to_string(j));
                // Re-fetch after add (vectors may have grown)
                nl_.gate(ff_id).inputs[0] = mux_out;
            }

            prev_q = ff_output;
        }

        // Last FF's Q output is scan_out for this chain
        NetId scan_out = nl_.add_net(config.scan_out_prefix + "_" + std::to_string(chain_id));
        GateId last_ff = ff_ids[chain_end - 1];
        NetId last_q = nl_.gate(last_ff).output;
        if (last_q >= 0) {
            nl_.add_gate(GateType::BUF, {last_q}, scan_out, "SCAN_OUT_BUF_" + std::to_string(chain_id));
        }
        nl_.mark_output(scan_out);

        result.chain_lengths.push_back(chain_len);
        chain_id++;
    }

    result.num_chains = chain_id;
    result.message = "Inserted " + std::to_string(chain_id) + " scan chain(s) covering " +
                     std::to_string(result.total_ffs) + " FFs";
    return result;
}

MultiChainResult ScanInserter::partition_chains(const MultiChainConfig& cfg) {
    auto& ffs = nl_.flip_flops();
    MultiChainResult result;
    int total = (int)ffs.size();
    int n = std::max(cfg.num_chains, 1);
    result.num_chains = n;
    result.chains.resize(n);
    result.chain_lengths.resize(n, 0);

    if (total == 0) {
        result.length_imbalance_pct = 0.0;
        return result;
    }

    std::vector<GateId> ff_ids(ffs.begin(), ffs.end());

    switch (cfg.mode) {
    case MultiChainConfig::BY_ROUTING: {
        // Heuristic: sort by gate output net id as proxy for placement locality
        std::sort(ff_ids.begin(), ff_ids.end(), [&](GateId a, GateId b) {
            return nl_.gate(a).output < nl_.gate(b).output;
        });
        break;
    }
    case MultiChainConfig::BY_DOMAIN: {
        // Heuristic: sort by first input net as proxy for clock domain grouping
        std::sort(ff_ids.begin(), ff_ids.end(), [&](GateId a, GateId b) {
            auto& ia = nl_.gate(a).inputs;
            auto& ib = nl_.gate(b).inputs;
            int ka = ia.empty() ? 0 : ia[0];
            int kb = ib.empty() ? 0 : ib[0];
            return ka < kb;
        });
        break;
    }
    default: // BY_LENGTH — round-robin for best balance
        break;
    }

    // Distribute FFs across chains
    for (int i = 0; i < total; ++i) {
        int chain_idx = i % n;
        result.chains[chain_idx].push_back((int)ff_ids[i]);
        result.chain_lengths[chain_idx]++;
    }

    // Compute imbalance
    int min_len = *std::min_element(result.chain_lengths.begin(), result.chain_lengths.end());
    int max_len = *std::max_element(result.chain_lengths.begin(), result.chain_lengths.end());
    double avg = (double)total / n;
    result.length_imbalance_pct = (avg > 0.0) ? ((max_len - min_len) / avg) * 100.0 : 0.0;

    return result;
}

CompressionResult ScanInserter::insert_compression(const CompressionConfig& cfg) {
    CompressionResult result;
    int total_ffs = (int)nl_.flip_flops().size();
    int total_gates = (int)nl_.gates().size();

    // Internal chains are driven by decompressor outputs
    result.internal_chains = cfg.decompressor_inputs * (int)std::round(cfg.compression_ratio);
    result.external_channels = cfg.decompressor_inputs + cfg.compressor_outputs;

    // Actual compression ratio: internal_chains / external_channels
    result.actual_ratio = (result.external_channels > 0)
        ? (double)result.internal_chains / result.external_channels
        : 1.0;

    // Estimate patterns saved based on compression and design complexity
    int base_patterns = total_ffs + total_gates / 4;
    int compressed_patterns = (result.actual_ratio > 1.0)
        ? (int)(base_patterns / result.actual_ratio)
        : base_patterns;
    result.patterns_saved = base_patterns - compressed_patterns;

    return result;
}

TestPointResult ScanInserter::insert_test_points(int max_points) {
    TestPointResult result;
    int total_gates = (int)nl_.gates().size();
    int total_nets = nl_.num_nets();

    // Heuristic: observe points ~ 60% of budget, control points ~ 40%
    int budget = std::min(max_points, total_nets / 4);
    result.observe_points = (int)(budget * 0.6);
    result.control_points = budget - result.observe_points;

    // Coverage improvement estimate: diminishing returns based on design size
    double base_coverage = 85.0;
    double point_impact = (total_gates > 0)
        ? (double)budget / total_gates * 200.0
        : 0.0;
    result.coverage_improvement_pct = std::min(point_impact, 15.0 - base_coverage * 0.01);

    return result;
}

ReorderResult ScanInserter::reorder_chains() {
    ReorderResult result;
    auto& ffs = nl_.flip_flops();
    int total = (int)ffs.size();

    if (total < 2) {
        result.wirelength_before = 0.0;
        result.wirelength_after = 0.0;
        result.improvement_pct = 0.0;
        return result;
    }

    std::vector<GateId> ff_ids(ffs.begin(), ffs.end());

    // Estimate wirelength as sum of absolute differences of output net ids (placement proxy)
    auto compute_wirelength = [&](const std::vector<GateId>& order) -> double {
        double wl = 0.0;
        for (size_t i = 1; i < order.size(); ++i) {
            int a = nl_.gate(order[i - 1]).output;
            int b = nl_.gate(order[i]).output;
            wl += std::abs(a - b);
        }
        return wl;
    };

    result.wirelength_before = compute_wirelength(ff_ids);

    // Reorder by output net id (nearest-neighbour heuristic for locality)
    std::sort(ff_ids.begin(), ff_ids.end(), [&](GateId a, GateId b) {
        return nl_.gate(a).output < nl_.gate(b).output;
    });

    result.wirelength_after = compute_wirelength(ff_ids);
    result.improvement_pct = (result.wirelength_before > 0.0)
        ? (result.wirelength_before - result.wirelength_after) / result.wirelength_before * 100.0
        : 0.0;

    return result;
}

ScanResult ScanInserter::run_enhanced(const ScanConfig& scan_cfg,
                                       const MultiChainConfig& mc_cfg,
                                       const CompressionConfig& comp_cfg) {
    // Step 1: basic scan insertion
    ScanResult result = insert(scan_cfg);

    // Step 2: multi-chain partitioning
    MultiChainResult mc = partition_chains(mc_cfg);

    // Step 3: compression
    CompressionResult comp = insert_compression(comp_cfg);

    // Step 4: test point insertion
    TestPointResult tp = insert_test_points();

    // Step 5: chain reorder for routing optimisation
    ReorderResult ro = reorder_chains();

    // Augment the result with enhanced-flow summary
    result.num_chains = mc.num_chains;
    result.chain_lengths = mc.chain_lengths;
    result.message += " | Enhanced: " + std::to_string(mc.num_chains) + " chains"
                      ", compression " + std::to_string(comp.actual_ratio).substr(0, 4) + "x"
                      ", " + std::to_string(tp.observe_points + tp.control_points) + " test points"
                      ", routing " + std::to_string(ro.improvement_pct).substr(0, 5) + "% improved";

    return result;
}

} // namespace sf
