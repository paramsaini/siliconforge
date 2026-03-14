// SiliconForge — Parallel-Pattern Fault Simulator Implementation
#include "dft/fault_sim.hpp"
#include <algorithm>
#include <iostream>
#include <cassert>

namespace sf {

FaultSimulator::FaultSimulator(Netlist& nl) : nl_(nl) {
    topo_ = nl_.topo_order();
}

uint64_t FaultSimulator::eval_gate_64(GateType type, const std::vector<uint64_t>& in) {
    switch (type) {
        case GateType::BUF:   return in[0];
        case GateType::NOT:   return ~in[0];
        case GateType::AND:   { uint64_t r = ~0ULL; for (auto v : in) r &= v; return r; }
        case GateType::OR:    { uint64_t r = 0ULL; for (auto v : in) r |= v; return r; }
        case GateType::NAND:  { uint64_t r = ~0ULL; for (auto v : in) r &= v; return ~r; }
        case GateType::NOR:   { uint64_t r = 0ULL; for (auto v : in) r |= v; return ~r; }
        case GateType::XOR:   return in[0] ^ in[1];
        case GateType::XNOR:  return ~(in[0] ^ in[1]);
        case GateType::CONST0: return 0ULL;
        case GateType::CONST1: return ~0ULL;
        case GateType::BUFIF1: return (in.size() >= 2) ? (in[0] & in[1]) : 0ULL;
        case GateType::BUFIF0: return (in.size() >= 2) ? (in[0] & ~in[1]) : 0ULL;
        case GateType::NOTIF1: return (in.size() >= 2) ? (~in[0] & in[1]) : 0ULL;
        case GateType::NOTIF0: return (in.size() >= 2) ? (~in[0] & ~in[1]) : 0ULL;
        default: return 0ULL;
    }
}

FaultSimResult FaultSimulator::simulate(const std::vector<std::vector<Logic4>>& test_vectors) {
    if (test_vectors.empty()) return {0, 0};

    // Enumerate faults
    std::vector<Fault> all_faults;
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        all_faults.push_back({(NetId)i, Logic4::ZERO});
        all_faults.push_back({(NetId)i, Logic4::ONE});
    }

    FaultSimResult result;
    result.total_faults = all_faults.size();
    std::vector<bool> detected(all_faults.size(), false);

    size_t num_patterns = test_vectors.size();
    size_t n_nets = nl_.num_nets();

    // Process patterns in batches of 64
    for (size_t batch_start = 0; batch_start < num_patterns; batch_start += 64) {
        size_t batch_size = std::min<size_t>(64, num_patterns - batch_start);

        // Pack input patterns into 64-bit words
        std::vector<uint64_t> good_vals(n_nets, 0ULL);

        // Set PI values
        for (size_t pi_idx = 0; pi_idx < nl_.primary_inputs().size(); ++pi_idx) {
            NetId pi = nl_.primary_inputs()[pi_idx];
            uint64_t word = 0;
            for (size_t p = 0; p < batch_size; ++p) {
                if (pi_idx < test_vectors[batch_start + p].size() &&
                    test_vectors[batch_start + p][pi_idx] == Logic4::ONE)
                    word |= (1ULL << p);
            }
            good_vals[pi] = word;
        }

        // Evaluate good circuit
        for (auto gid : topo_) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0) continue;
            std::vector<uint64_t> in_vals;
            for (auto ni : g.inputs) in_vals.push_back(good_vals[ni]);
            good_vals[g.output] = eval_gate_64(g.type, in_vals);
        }

        // For each fault, inject and compare
        for (size_t fi = 0; fi < all_faults.size(); ++fi) {
            if (detected[fi]) continue;
            auto& fault = all_faults[fi];

            // Copy good values
            std::vector<uint64_t> faulty_vals = good_vals;

            // Inject fault
            faulty_vals[fault.net] = (fault.stuck_at == Logic4::ZERO) ? 0ULL : ~0ULL;

            // Re-evaluate from fault site forward
            for (auto gid : topo_) {
                auto& g = nl_.gate(gid);
                if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0) continue;

                // Only re-evaluate if in transitive fanout of fault
                bool affected = false;
                for (auto ni : g.inputs) {
                    if (faulty_vals[ni] != good_vals[ni]) { affected = true; break; }
                }
                if (!affected) continue;

                std::vector<uint64_t> in_vals;
                for (auto ni : g.inputs) in_vals.push_back(faulty_vals[ni]);
                faulty_vals[g.output] = eval_gate_64(g.type, in_vals);
            }

            // Check POs for differences
            for (auto po : nl_.primary_outputs()) {
                if (faulty_vals[po] != good_vals[po]) {
                    detected[fi] = true;
                    break;
                }
            }
        }
    }

    result.detected = std::count(detected.begin(), detected.end(), true);
    return result;
}

// ── Fault Dictionary ─────────────────────────────────────────────────────────
// For each stuck-at fault on each net, simulate all test patterns and record
// which patterns detect which faults. Uses bit-parallel simulation.

FaultDictionary FaultSimulator::build_fault_dictionary(
        const std::vector<std::vector<Logic4>>& test_vectors) {
    FaultDictionary dict;
    size_t n_nets = nl_.num_nets();

    // Enumerate all single stuck-at faults
    std::vector<Fault> all_faults;
    for (size_t i = 0; i < n_nets; ++i) {
        all_faults.push_back({static_cast<NetId>(i), Logic4::ZERO});
        all_faults.push_back({static_cast<NetId>(i), Logic4::ONE});
    }
    dict.total_faults = all_faults.size();
    dict.entries.resize(all_faults.size());
    for (size_t fi = 0; fi < all_faults.size(); ++fi)
        dict.entries[fi].fault = all_faults[fi];

    size_t num_patterns = test_vectors.size();

    // Process in 64-pattern batches (bit-parallel)
    for (size_t batch_start = 0; batch_start < num_patterns; batch_start += 64) {
        size_t batch_size = std::min<size_t>(64, num_patterns - batch_start);

        // Pack PI values into 64-bit words
        std::vector<uint64_t> good_vals(n_nets, 0ULL);
        for (size_t pi_idx = 0; pi_idx < nl_.primary_inputs().size(); ++pi_idx) {
            NetId pi = nl_.primary_inputs()[pi_idx];
            uint64_t word = 0;
            for (size_t p = 0; p < batch_size; ++p) {
                if (pi_idx < test_vectors[batch_start + p].size() &&
                    test_vectors[batch_start + p][pi_idx] == Logic4::ONE)
                    word |= (1ULL << p);
            }
            good_vals[pi] = word;
        }

        // Evaluate good circuit
        for (auto gid : topo_) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0) continue;
            std::vector<uint64_t> in_vals;
            for (auto ni : g.inputs) in_vals.push_back(good_vals[ni]);
            good_vals[g.output] = eval_gate_64(g.type, in_vals);
        }

        // For each fault, inject and record detecting patterns
        for (size_t fi = 0; fi < all_faults.size(); ++fi) {
            auto& fault = all_faults[fi];
            std::vector<uint64_t> faulty_vals = good_vals;
            faulty_vals[fault.net] = (fault.stuck_at == Logic4::ZERO) ? 0ULL : ~0ULL;

            // Re-evaluate gates in transitive fanout of fault
            for (auto gid : topo_) {
                auto& g = nl_.gate(gid);
                if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0) continue;
                bool affected = false;
                for (auto ni : g.inputs)
                    if (faulty_vals[ni] != good_vals[ni]) { affected = true; break; }
                if (!affected) continue;

                std::vector<uint64_t> in_vals;
                for (auto ni : g.inputs) in_vals.push_back(faulty_vals[ni]);
                faulty_vals[g.output] = eval_gate_64(g.type, in_vals);
            }

            // Check POs for differences — each set bit = a detecting pattern
            uint64_t detected_mask = 0;
            for (auto po : nl_.primary_outputs())
                detected_mask |= (faulty_vals[po] ^ good_vals[po]);

            for (size_t p = 0; p < batch_size; ++p)
                if (detected_mask & (1ULL << p))
                    dict.entries[fi].detecting_patterns.push_back(
                        static_cast<int>(batch_start + p));
        }
    }

    // Compute coverage
    size_t total_detected = 0;
    for (auto& entry : dict.entries)
        if (!entry.detecting_patterns.empty()) total_detected++;
    dict.coverage_pct = dict.total_faults > 0
        ? 100.0 * total_detected / dict.total_faults : 0;

    return dict;
}

// ── Fault Diagnosis ──────────────────────────────────────────────────────────
// Given observed (faulty) responses from a manufactured chip, compare against
// good-circuit responses and use the fault dictionary to identify candidate
// defects. Candidates sorted by number of matching failing patterns.

DiagnosisResult FaultSimulator::diagnose(
        const std::vector<std::vector<Logic4>>& test_vectors,
        const std::vector<std::vector<Logic4>>& observed_responses) {
    DiagnosisResult result;
    result.patterns_used = static_cast<int>(test_vectors.size());

    auto dict = build_fault_dictionary(test_vectors);

    // Simulate good circuit to get expected responses
    auto& pis = nl_.primary_inputs();
    auto& pos = nl_.primary_outputs();
    size_t n_nets = nl_.num_nets();
    auto topo = nl_.topo_order();

    std::vector<std::vector<Logic4>> good_responses;
    for (auto& vec : test_vectors) {
        std::vector<Logic4> net_vals(n_nets, Logic4::X);
        for (size_t i = 0; i < pis.size() && i < vec.size(); ++i)
            net_vals[pis[i]] = vec[i];
        for (auto gid : topo) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::DFF || g.type == GateType::INPUT || g.output < 0) continue;
            std::vector<Logic4> in_vals;
            for (auto ni : g.inputs) in_vals.push_back(net_vals[ni]);
            net_vals[g.output] = Netlist::eval_gate(g.type, in_vals);
        }
        std::vector<Logic4> response;
        for (auto po : pos) response.push_back(net_vals[po]);
        good_responses.push_back(response);
    }

    // Find failing patterns (observed differs from expected)
    std::vector<int> failing_patterns;
    for (size_t i = 0; i < test_vectors.size() && i < observed_responses.size(); ++i) {
        bool differs = false;
        for (size_t j = 0; j < pos.size() && j < observed_responses[i].size()
                && j < good_responses[i].size(); ++j) {
            if (observed_responses[i][j] != good_responses[i][j]) {
                differs = true;
                break;
            }
        }
        if (differs) failing_patterns.push_back(static_cast<int>(i));
    }

    // Score each fault by how many failing patterns it explains
    struct CandidateScore {
        Fault fault;
        int matches;
    };
    std::vector<CandidateScore> candidates;

    for (auto& entry : dict.entries) {
        if (entry.detecting_patterns.empty()) continue;
        int matches = 0;
        for (int fp : failing_patterns) {
            for (int dp : entry.detecting_patterns) {
                if (dp == fp) { matches++; break; }
            }
        }
        if (matches > 0)
            candidates.push_back({entry.fault, matches});
    }

    // Sort by match count descending (best candidates first)
    std::sort(candidates.begin(), candidates.end(),
              [](const CandidateScore& a, const CandidateScore& b) {
                  return a.matches > b.matches;
              });

    for (auto& c : candidates)
        result.candidate_faults.push_back(c.fault);

    if (!candidates.empty() && !failing_patterns.empty())
        result.confidence = 100.0 * candidates[0].matches
                            / static_cast<double>(failing_patterns.size());

    return result;
}

// ── Enhanced Fault Simulation ────────────────────────────────────────────────
// Augments user-provided vectors with supplementary patterns (all-0, all-1,
// alternating) to improve fault coverage beyond basic simulation.

FaultSimResult FaultSimulator::run_enhanced(
        const std::vector<std::vector<Logic4>>& test_vectors) {
    auto vectors = test_vectors;
    size_t n_pi = nl_.primary_inputs().size();

    // Supplementary patterns for better stuck-at coverage
    vectors.push_back(std::vector<Logic4>(n_pi, Logic4::ZERO));
    vectors.push_back(std::vector<Logic4>(n_pi, Logic4::ONE));

    std::vector<Logic4> alt1(n_pi), alt2(n_pi);
    for (size_t i = 0; i < n_pi; ++i) {
        alt1[i] = (i % 2 == 0) ? Logic4::ONE : Logic4::ZERO;
        alt2[i] = (i % 2 == 0) ? Logic4::ZERO : Logic4::ONE;
    }
    vectors.push_back(alt1);
    vectors.push_back(alt2);

    auto result = simulate(vectors);

    // Cross-check with fault dictionary
    auto dict = build_fault_dictionary(vectors);
    size_t dict_detected = 0;
    for (auto& entry : dict.entries)
        if (!entry.detecting_patterns.empty()) dict_detected++;
    result.detected = std::max(result.detected, dict_detected);

    return result;
}

} // namespace sf
