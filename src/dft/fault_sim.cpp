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

} // namespace sf
