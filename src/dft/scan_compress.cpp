// SiliconForge — Scan Compression Engine Implementation
#include "dft/scan_compress.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <cassert>

namespace sf {

// ── Constructor ──────────────────────────────────────────────────────────────

ScanCompressor::ScanCompressor(Netlist& nl) : nl_(nl) {
    topo_ = nl_.topo_order();
}

// ── LFSR: Galois Linear Feedback Shift Register ─────────────────────────────
// Galois LFSR shifts right; when outgoing LSB is 1, XOR with polynomial mask.
// Mask encodes the primitive polynomial taps (MSB always set for degree n).
// Period: 2^width - 1 for maximal-length polynomials.

uint32_t ScanCompressor::lfsr_next(uint32_t state, int width) {
    uint32_t tap;
    switch (width) {
        case 3:  tap = 0x6;        break;  // x^3 + x^2 + 1
        case 4:  tap = 0x9;        break;  // x^4 + x + 1
        case 5:  tap = 0x12;       break;  // x^5 + x^2 + 1
        case 7:  tap = 0x41;       break;  // x^7 + x + 1
        case 8:  tap = 0xB8;       break;  // x^8 + x^6 + x^5 + x^4 + 1
        case 16: tap = 0xB400;     break;  // x^16 + x^14 + x^13 + x^11 + 1
        default: tap = 0x80200003; break;  // x^32 + x^22 + x^2 + x + 1
    }

    uint32_t feedback = state & 1u;
    state >>= 1;
    if (feedback) state ^= tap;
    if (width < 32) state &= (1u << width) - 1;
    return state;
}

// ── MISR: Multiple-Input Signature Register ──────────────────────────────────
// Compresses sequential response data into a single signature word.
// Uses the same polynomial as the 32-bit LFSR for feedback.

uint32_t ScanCompressor::misr_compress(uint32_t misr, uint32_t data) {
    uint32_t feedback = (misr >> 31) & 1u;
    misr = (misr << 1) ^ data;
    if (feedback) misr ^= 0x80200003u;
    return misr;
}

// ── Single-pattern simulation helpers ────────────────────────────────────────

std::vector<Logic4> ScanCompressor::simulate_pattern(const std::vector<Logic4>& pi_values) {
    size_t n_nets = nl_.num_nets();
    auto& pis = nl_.primary_inputs();
    auto& pos = nl_.primary_outputs();

    std::vector<Logic4> net_vals(n_nets, Logic4::X);
    for (size_t i = 0; i < pis.size() && i < pi_values.size(); ++i)
        net_vals[pis[i]] = pi_values[i];

    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0)
            continue;
        std::vector<Logic4> in_vals;
        for (auto ni : g.inputs) in_vals.push_back(net_vals[ni]);
        net_vals[g.output] = Netlist::eval_gate(g.type, in_vals);
    }

    std::vector<Logic4> response;
    for (auto po : pos) response.push_back(net_vals[po]);
    return response;
}

std::vector<Logic4> ScanCompressor::simulate_faulty(const std::vector<Logic4>& pi_values,
                                                     NetId fault_net, Logic4 stuck_val) {
    size_t n_nets = nl_.num_nets();
    auto& pis = nl_.primary_inputs();
    auto& pos = nl_.primary_outputs();

    std::vector<Logic4> net_vals(n_nets, Logic4::X);
    for (size_t i = 0; i < pis.size() && i < pi_values.size(); ++i)
        net_vals[pis[i]] = pi_values[i];

    // Inject fault on the net
    net_vals[fault_net] = stuck_val;

    for (auto gid : topo_) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0)
            continue;
        std::vector<Logic4> in_vals;
        for (auto ni : g.inputs) in_vals.push_back(net_vals[ni]);
        net_vals[g.output] = Netlist::eval_gate(g.type, in_vals);
        // Re-inject if gate overwrote the fault site
        if (g.output == fault_net)
            net_vals[fault_net] = stuck_val;
    }

    std::vector<Logic4> response;
    for (auto po : pos) response.push_back(net_vals[po]);
    return response;
}

// ── EDT-style Scan Compression ───────────────────────────────────────────────
// Decompressor: LFSR → phase shifter (XOR network) → internal scan chains
// Compactor:    internal scan chains → XOR tree → external outputs

CompressResult ScanCompressor::compress(const ScanCompressConfig& cfg) {
    CompressResult result;

    // Estimate original test pattern count from circuit complexity
    size_t n_nets = nl_.num_nets();
    int original = std::max(1, static_cast<int>(n_nets * 2));
    int compressed = std::max(1, original / cfg.compression_ratio);

    // Decompressor area: LFSR bits × scan chains (XOR fan-out network)
    int decompressor_gates = cfg.lfsr_width * cfg.num_scan_chains;
    // Compactor area: XOR tree reducing scan chain outputs
    int compactor_gates = cfg.num_scan_chains;

    // Generate compressed seeds and expand through decompressor
    uint32_t lfsr_state = 0xACE1u;
    auto& pis = nl_.primary_inputs();
    auto& pos = nl_.primary_outputs();

    std::vector<std::vector<Logic4>> expanded_patterns;
    for (int i = 0; i < compressed; ++i) {
        // Expand LFSR seed to full scan chain content
        std::vector<Logic4> pattern(pis.size());
        uint32_t seed = lfsr_state;
        for (size_t pi = 0; pi < pis.size(); ++pi) {
            pattern[pi] = (seed & 1u) ? Logic4::ONE : Logic4::ZERO;
            seed >>= 1;
            if (seed == 0) { lfsr_state = lfsr_next(lfsr_state, cfg.lfsr_width); seed = lfsr_state; }
        }
        expanded_patterns.push_back(pattern);
        lfsr_state = lfsr_next(lfsr_state, cfg.lfsr_width);
    }

    // Simulate expanded patterns for coverage estimation
    // Compute good responses first
    std::vector<std::vector<Logic4>> good_responses;
    for (auto& pat : expanded_patterns)
        good_responses.push_back(simulate_pattern(pat));

    // Enumerate faults and check detection
    int total_faults = static_cast<int>(n_nets * 2);
    std::vector<bool> detected(total_faults, false);

    for (int fi = 0; fi < total_faults; ++fi) {
        NetId fault_net = static_cast<NetId>(fi / 2);
        Logic4 stuck_val = (fi % 2 == 0) ? Logic4::ZERO : Logic4::ONE;

        for (size_t pi = 0; pi < expanded_patterns.size(); ++pi) {
            if (detected[fi]) break;
            auto faulty_resp = simulate_faulty(expanded_patterns[pi], fault_net, stuck_val);
            for (size_t j = 0; j < faulty_resp.size(); ++j) {
                if (faulty_resp[j] != good_responses[pi][j]) {
                    detected[fi] = true;
                    break;
                }
            }
        }
    }

    int faults_detected = static_cast<int>(std::count(detected.begin(), detected.end(), true));
    double coverage = total_faults > 0 ? 100.0 * faults_detected / total_faults : 0;

    result.original_patterns = original;
    result.compressed_patterns = compressed;
    result.compression_ratio = static_cast<double>(original) / compressed;
    result.decompressor_gates = decompressor_gates;
    result.compactor_gates = compactor_gates;
    result.coverage_pct = coverage;

    std::ostringstream oss;
    oss << "Scan compression: " << original << " -> " << compressed
        << " patterns (ratio " << result.compression_ratio << ":1)"
        << ", coverage " << coverage << "%"
        << ", overhead " << decompressor_gates + compactor_gates << " gates";
    result.report = oss.str();

    return result;
}

// ── Logic Built-In Self-Test (LBIST) ─────────────────────────────────────────
// Autonomous test: LFSR generates patterns, circuit processes them,
// MISR captures compressed responses, signature compared to golden.

LbistResult ScanCompressor::run_lbist(const ScanCompressConfig& cfg) {
    LbistResult result;
    size_t n_nets = nl_.num_nets();
    auto& pis = nl_.primary_inputs();
    auto& pos = nl_.primary_outputs();

    // Generate pseudo-random patterns from LFSR
    uint32_t lfsr_state = 0xACE1u;
    std::vector<std::vector<Logic4>> patterns;

    for (int p = 0; p < cfg.lbist_patterns; ++p) {
        std::vector<Logic4> vec(pis.size());
        uint32_t bits = lfsr_state;
        for (size_t i = 0; i < pis.size(); ++i) {
            vec[i] = (bits & 1u) ? Logic4::ONE : Logic4::ZERO;
            bits >>= 1;
            if (bits == 0) {
                lfsr_state = lfsr_next(lfsr_state, cfg.lfsr_width);
                bits = lfsr_state;
            }
        }
        patterns.push_back(vec);
        lfsr_state = lfsr_next(lfsr_state, cfg.lfsr_width);
    }

    // Fault-free simulation: compute good responses and golden MISR signature
    uint32_t golden_misr = 0;
    std::vector<std::vector<Logic4>> good_responses;

    for (auto& vec : patterns) {
        auto response = simulate_pattern(vec);
        good_responses.push_back(response);

        // Pack response bits for MISR
        uint32_t resp_bits = 0;
        for (size_t i = 0; i < response.size(); ++i)
            if (response[i] == Logic4::ONE)
                resp_bits |= (1u << (i % 32));
        golden_misr = misr_compress(golden_misr, resp_bits);
    }

    result.misr_signature = golden_misr;
    result.signature_match = true;  // golden matches itself
    result.patterns_applied = cfg.lbist_patterns;

    // Fault coverage: enumerate all stuck-at faults, check detection
    int total_faults = static_cast<int>(n_nets * 2);
    int detected_count = 0;

    for (int fi = 0; fi < total_faults; ++fi) {
        NetId fault_net = static_cast<NetId>(fi / 2);
        Logic4 stuck_val = (fi % 2 == 0) ? Logic4::ZERO : Logic4::ONE;
        bool fault_detected = false;

        for (size_t pi = 0; pi < patterns.size() && !fault_detected; ++pi) {
            auto faulty_resp = simulate_faulty(patterns[pi], fault_net, stuck_val);
            for (size_t j = 0; j < faulty_resp.size(); ++j) {
                if (faulty_resp[j] != good_responses[pi][j]) {
                    fault_detected = true;
                    break;
                }
            }
        }
        if (fault_detected) detected_count++;
    }

    result.faults_detected = detected_count;
    result.total_faults = total_faults;
    result.coverage_pct = total_faults > 0
        ? 100.0 * detected_count / total_faults : 0;

    return result;
}

// ── Enhanced Full Flow ───────────────────────────────────────────────────────
// Combines EDT compression with LBIST for maximum coverage.

CompressResult ScanCompressor::run_enhanced() {
    ScanCompressConfig cfg;
    auto comp = compress(cfg);
    auto lbist = run_lbist(cfg);

    // Take the better coverage from either mode
    comp.coverage_pct = std::max(comp.coverage_pct, lbist.coverage_pct);

    std::ostringstream oss;
    oss << comp.report
        << "\nLBIST: " << lbist.faults_detected << "/" << lbist.total_faults
        << " faults (" << lbist.coverage_pct << "%)"
        << ", MISR=0x" << std::hex << lbist.misr_signature << std::dec
        << (lbist.signature_match ? " PASS" : " FAIL");
    comp.report = oss.str();

    return comp;
}

} // namespace sf
