#pragma once
// SiliconForge — Logic Equivalence Checking (LEC)
// Verifies structural/functional equivalence between two netlists
// (e.g., pre-synthesis vs post-synthesis, or pre-ECO vs post-ECO).
// Uses AIG-based miter construction + SAT solving.
// Reference: Brand, "Verification of Large Synthesized Designs", ICCAD 1993

#include "core/netlist.hpp"
#include "core/aig.hpp"
#include <string>
#include <vector>

namespace sf {

struct LecResult {
    bool equivalent = false;
    int key_points_compared = 0;
    int matched = 0;
    int mismatched = 0;
    int aborted = 0;
    double time_ms = 0;

    struct Mismatch {
        std::string point_name;
        std::string detail;
    };
    std::vector<Mismatch> mismatches;
    std::string message;
};

class LecEngine {
public:
    LecEngine(const Netlist& golden, const Netlist& revised)
        : golden_(golden), revised_(revised) {}

    LecResult check();

private:
    const Netlist& golden_;
    const Netlist& revised_;

    // Build AIG from netlist for a single output
    AigLit build_cone(AigGraph& aig, const Netlist& nl, NetId output,
                      std::unordered_map<NetId, AigLit>& net_map);

    // Compare single output cones using SAT
    bool compare_output(const std::string& name, NetId g_out, NetId r_out,
                        LecResult& result);

    // Simulation-based equivalence check (robust to structural changes)
    bool sim_compare(LecResult& result);
};

} // namespace sf
