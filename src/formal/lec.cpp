// SiliconForge — LEC Implementation
#include "formal/lec.hpp"
#include "formal/tseitin.hpp"
#include "sat/cdcl_solver.hpp"
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <functional>

namespace sf {

AigLit LecEngine::build_cone(AigGraph& aig, const Netlist& nl, NetId output,
                              std::unordered_map<NetId, AigLit>& net_map) {
    // Topological traversal from output back to inputs
    std::vector<NetId> topo;
    std::unordered_set<NetId> visited;

    std::function<void(NetId)> visit = [&](NetId nid) {
        if (visited.count(nid)) return;
        visited.insert(nid);
        auto& net = nl.net(nid);
        if (net.driver >= 0) {
            auto& g = nl.gate(net.driver);
            for (auto inp : g.inputs) visit(inp);
        }
        topo.push_back(nid);
    };
    visit(output);

    // Create AIG nodes
    for (auto nid : topo) {
        if (net_map.count(nid)) continue;
        auto& net = nl.net(nid);
        if (net.driver < 0) {
            net_map[nid] = aig.create_input(net.name);
            continue;
        }
        auto& g = nl.gate(net.driver);
        if (g.inputs.empty()) {
            net_map[nid] = aig.create_input(net.name);
            continue;
        }

        AigLit result;
        auto get = [&](NetId n) -> AigLit {
            if (net_map.count(n)) return net_map[n];
            AigLit lit = aig.create_input(nl.net(n).name);
            net_map[n] = lit;
            return lit;
        };

        switch (g.type) {
            case GateType::AND:
                result = get(g.inputs[0]);
                for (size_t i = 1; i < g.inputs.size(); ++i)
                    result = aig.create_and(result, get(g.inputs[i]));
                break;
            case GateType::OR:
                result = get(g.inputs[0]);
                for (size_t i = 1; i < g.inputs.size(); ++i)
                    result = aig.create_or(result, get(g.inputs[i]));
                break;
            case GateType::NOT:
                result = aig_not(get(g.inputs[0]));
                break;
            case GateType::NAND:
                result = get(g.inputs[0]);
                for (size_t i = 1; i < g.inputs.size(); ++i)
                    result = aig.create_and(result, get(g.inputs[i]));
                result = aig_not(result);
                break;
            case GateType::NOR:
                result = get(g.inputs[0]);
                for (size_t i = 1; i < g.inputs.size(); ++i)
                    result = aig.create_or(result, get(g.inputs[i]));
                result = aig_not(result);
                break;
            case GateType::XOR:
                result = aig.create_xor(get(g.inputs[0]),
                         g.inputs.size() > 1 ? get(g.inputs[1]) : AIG_FALSE);
                break;
            case GateType::XNOR:
                result = aig_not(aig.create_xor(get(g.inputs[0]),
                         g.inputs.size() > 1 ? get(g.inputs[1]) : AIG_FALSE));
                break;
            case GateType::BUF:
                result = get(g.inputs[0]);
                break;
            default:
                result = aig.create_input("unknown_" + g.name);
                break;
        }
        net_map[nid] = result;
    }
    return net_map.count(output) ? net_map[output] : AIG_FALSE;
}

bool LecEngine::compare_output(const std::string& name, NetId g_out, NetId r_out,
                                 LecResult& result) {
    AigGraph aig;
    std::unordered_map<NetId, AigLit> g_map, r_map;

    // Build golden cone first
    AigLit g_lit = build_cone(aig, golden_, g_out, g_map);

    // Build a name→AigLit map from golden primary inputs so revised can reuse them
    std::unordered_map<std::string, AigLit> shared_inputs;
    for (auto& [nid, lit] : g_map) {
        auto& net = golden_.net(nid);
        if (net.driver < 0) // primary input
            shared_inputs[net.name] = lit;
    }

    // Pre-populate revised map with shared inputs
    for (size_t i = 0; i < revised_.num_nets(); ++i) {
        auto& net = revised_.net(i);
        if (net.driver < 0) { // primary input
            auto it = shared_inputs.find(net.name);
            if (it != shared_inputs.end()) r_map[i] = it->second;
        }
    }

    AigLit r_lit = build_cone(aig, revised_, r_out, r_map);

    // Miter: XOR of the two outputs — SAT means they differ
    AigLit miter = aig.create_xor(g_lit, r_lit);
    aig.add_output(miter, "miter");

    // Encode to CNF and solve
    TseitinEncoder enc;
    CnfFormula cnf = enc.encode(aig);

    // Add unit clause asserting miter output is true
    CnfLit miter_cnf = enc.aig_lit_to_cnf(miter);
    cnf.add_unit(miter_cnf);

    CdclSolver solver;
    for (auto& cl : cnf.clauses())
        solver.add_clause(cl);

    auto sat_result = solver.solve();
    if (sat_result == SatResult::UNSAT) {
        result.matched++;
        return true; // equivalent
    } else {
        result.mismatched++;
        result.mismatches.push_back({name, "Output differs between golden and revised"});
        return false;
    }
}

LecResult LecEngine::check() {
    auto t0 = std::chrono::high_resolution_clock::now();
    LecResult r;

    auto g_outs = golden_.primary_outputs();
    auto r_outs = revised_.primary_outputs();

    for (auto go : g_outs) {
        std::string gname = golden_.net(go).name;
        bool found = false;
        for (auto ro : r_outs) {
            if (revised_.net(ro).name == gname) {
                r.key_points_compared++;
                compare_output(gname, go, ro, r);
                found = true;
                break;
            }
        }
        if (!found) {
            r.key_points_compared++;
            r.mismatched++;
            r.mismatches.push_back({gname, "Output missing in revised netlist"});
        }
    }

    r.equivalent = (r.mismatched == 0 && r.key_points_compared > 0);

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = r.equivalent ? "LEC PASS — " + std::to_string(r.matched) + " key points matched"
                             : "LEC FAIL — " + std::to_string(r.mismatched) + " mismatches";
    return r;
}

} // namespace sf
