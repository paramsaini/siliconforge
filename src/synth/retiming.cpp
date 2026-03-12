// SiliconForge — Register Retiming Implementation (Leiserson-Saxe)
//
// Algorithm overview:
//   1. Build retiming graph: combinational gates = nodes, connections = edges
//      with edge weight = number of DFFs on that edge.
//   2. Find current critical path delay T_cur (longest zero-weight path).
//   3. Binary search for minimum feasible clock period T_opt:
//      a. Build constraint graph for target T:
//         - For each edge (u,v): r(v) - r(u) ≤ w(e)
//         - For zero-weight paths with delay > T: r(u) - r(v) ≤ W(u,v) - 1
//      b. Run Bellman-Ford to find valid r-values (or detect infeasibility).
//   4. Apply retiming: for each edge, new_w = w + r(v) - r(u).
//      Insert DFFs where new_w > w, remove where new_w < w.
//
// Reference: Leiserson & Saxe, Algorithmica 1991

#include "synth/retiming.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

namespace sf {

// ============================================================================
// Gate Delay Model
// ============================================================================

double RetimingEngine::get_gate_delay(GateType type) const {
    if (config_.use_liberty_delays && lib_) {
        double d = get_liberty_delay(type);
        if (d > 0) return d;
    }
    switch (type) {
        case GateType::NOT:  return 1.0;
        case GateType::BUF:  return 0.5;
        case GateType::AND:
        case GateType::OR:
        case GateType::NAND:
        case GateType::NOR:  return 1.5;
        case GateType::XOR:
        case GateType::XNOR: return 2.5;
        case GateType::MUX:  return 2.0;
        default: return 0.0;
    }
}

double RetimingEngine::get_liberty_delay(GateType type) const {
    if (!lib_) return 0;
    std::string name = gate_type_str(type);
    auto* cell = lib_->find_cell(name);
    if (!cell || cell->timings.empty()) return 0;
    // Use average of rise and fall for retiming purposes
    double avg = 0;
    int count = 0;
    for (auto& t : cell->timings) {
        if (t.cell_rise > 0) { avg += t.cell_rise; count++; }
        if (t.cell_fall > 0) { avg += t.cell_fall; count++; }
    }
    return count > 0 ? avg / count : 0;
}

// ============================================================================
// Industrial: Clock Gating Detection
// ============================================================================

void RetimingEngine::detect_clock_gating(const Netlist& nl) {
    if (!config_.preserve_clock_gating) return;
    // Auto-detect ICG cells: gates that drive the clock input of DFFs
    // Pattern: gate output → DFF.clk means the gate is clock gating logic
    for (auto ff_id : nl.flip_flops()) {
        auto& ff = nl.gate(ff_id);
        if (ff.clk < 0) continue;
        auto& clk_net = nl.net(ff.clk);
        if (clk_net.driver >= 0) {
            auto& driver = nl.gate(clk_net.driver);
            if (driver.type != GateType::INPUT && driver.type != GateType::DFF) {
                clock_gating_cells_.insert(clk_net.driver);
            }
        }
    }
}

void RetimingEngine::detect_reset_paths(const Netlist& nl) {
    if (!config_.preserve_reset_paths) return;
    // Detect gates connected to async reset paths
    // Pattern: gate output → DFF.reset means gate is on reset path
    for (auto ff_id : nl.flip_flops()) {
        auto& ff = nl.gate(ff_id);
        if (ff.reset < 0) continue;
        auto& rst_net = nl.net(ff.reset);
        if (rst_net.driver >= 0) {
            auto& driver = nl.gate(rst_net.driver);
            if (driver.type != GateType::INPUT) {
                dont_retime_.insert(rst_net.driver);
            }
        }
    }
}

void RetimingEngine::apply_multicycle_relaxation() {
    if (!config_.honor_multicycle_paths || multicycle_paths_.empty()) return;
    for (auto& mcp : multicycle_paths_) {
        for (auto& e : edges_) {
            bool from_match = (mcp.from_gate < 0 || nodes_[e.u].gid == mcp.from_gate);
            bool to_match = (mcp.to_gate < 0 || nodes_[e.v].gid == mcp.to_gate);
            if (from_match && to_match) {
                e.multicycle_factor = mcp.cycles;
            }
        }
    }
}

void RetimingEngine::freeze_dont_retime_cells() {
    // Mark nodes as frozen if they're in dont_retime or clock_gating sets
    for (auto& node : nodes_) {
        if (dont_retime_.count(node.gid) || clock_gating_cells_.count(node.gid)) {
            node.frozen = true;
        }
    }
}

bool RetimingEngine::validate_resource_constraints() const {
    if (config_.max_register_increase < 0) return true; // unlimited
    int net_increase = 0;
    for (auto& e : edges_) {
        int new_w = e.weight + nodes_[e.v].r_val - nodes_[e.u].r_val;
        int delta = new_w - e.weight;
        if (delta > 0) net_increase += delta;
    }
    return net_increase <= config_.max_register_increase;
}

// ============================================================================
// Graph Construction
// ============================================================================

void RetimingEngine::build_graph(const Netlist& nl) {
    nodes_.clear();
    edges_.clear();
    gid_to_idx_.clear();

    // Create nodes for combinational gates only
    for (size_t i = 0; i < nl.num_gates(); ++i) {
        const Gate& g = nl.gate(static_cast<GateId>(i));
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT)
            continue;

        int idx = static_cast<int>(nodes_.size());
        gid_to_idx_[g.id] = idx;
        nodes_.push_back({g.id, get_gate_delay(g.type), 0, false});
    }

    // Create edges: for each node, follow its output net to find fanout gates.
    // If a DFF sits between source and sink, edge weight = 1, else 0.
    for (auto& node : nodes_) {
        const Gate& src = nl.gate(node.gid);
        if (src.output < 0) continue;
        const Net& out = nl.net(src.output);

        for (GateId fo_gid : out.fanout) {
            const Gate& fo = nl.gate(fo_gid);
            int weight = 0;
            GateId actual_sink = fo_gid;

            if (fo.type == GateType::DFF) {
                weight = 1;
                // Follow through DFF to its output's fanout
                if (fo.output >= 0) {
                    const Net& dff_out = nl.net(fo.output);
                    if (!dff_out.fanout.empty())
                        actual_sink = dff_out.fanout.front();
                    else
                        continue; // DFF output is unconnected
                } else {
                    continue;
                }
            }

            auto it = gid_to_idx_.find(actual_sink);
            if (it != gid_to_idx_.end()) {
                int u = gid_to_idx_[node.gid];
                int v = it->second;
                edges_.push_back({u, v, weight, node.delay, 1});
            }
        }
    }

    std::cout << "  Graph: " << nodes_.size() << " nodes, "
              << edges_.size() << " edges.\n";
}

// ============================================================================
// Critical Path Analysis
// ============================================================================

double RetimingEngine::compute_critical_path() const {
    // Longest combinational path = longest path using only zero-weight edges.
    // Use Bellman-Ford with negated delays to find longest path.
    int n = static_cast<int>(nodes_.size());
    if (n == 0) return 0;

    std::vector<double> dist(n, 0);

    for (int iter = 0; iter < n; ++iter) {
        bool changed = false;
        for (auto& e : edges_) {
            if (e.weight == 0) { // combinational edge
                double nd = dist[e.u] + nodes_[e.v].delay;
                if (nd > dist[e.v]) {
                    dist[e.v] = nd;
                    changed = true;
                }
            }
        }
        if (!changed) break;
    }

    double max_d = 0;
    for (int i = 0; i < n; ++i)
        max_d = std::max(max_d, dist[i] + nodes_[i].delay);
    return max_d;
}

// ============================================================================
// Leiserson-Saxe Retiming via Constraint Graph + Bellman-Ford
// ============================================================================

bool RetimingEngine::compute_retiming_values(double target_period) {
    // Adjust target for setup margin
    double effective_target = target_period - config_.setup_margin;
    if (effective_target <= 0) return false;

    int n = static_cast<int>(nodes_.size());
    if (n == 0) return false;

    struct ConstraintEdge { int from, to; int wt; };
    std::vector<ConstraintEdge> constraints;

    // Constraint 1: from original edges (with multicycle relaxation)
    for (auto& e : edges_)
        constraints.push_back({e.u, e.v, e.weight * e.multicycle_factor});

    // Build adjacency list of zero-weight edges for path enumeration
    std::vector<std::vector<std::pair<int, double>>> zero_adj(n);
    for (auto& e : edges_) {
        if (e.weight == 0)
            zero_adj[e.u].push_back({e.v, nodes_[e.v].delay});
    }

    // Constraint 2: find zero-weight paths with delay > effective_target
    for (int src = 0; src < n; ++src) {
        std::vector<double> best_delay(n, -1);
        std::queue<std::pair<int, double>> q;
        double src_delay = nodes_[src].delay;
        q.push({src, src_delay});
        best_delay[src] = src_delay;

        while (!q.empty()) {
            auto [u, d] = q.front(); q.pop();
            for (auto& [v, vd] : zero_adj[u]) {
                double nd = d + vd;
                if (nd > best_delay[v]) {
                    best_delay[v] = nd;
                    if (nd > effective_target) {
                        constraints.push_back({v, src, -1});
                    }
                    q.push({v, nd});
                }
            }
        }
    }

    // Hold-aware constraint: if hold_aware, add constraints to prevent
    // paths shorter than hold_margin from losing all registers
    if (config_.hold_aware) {
        for (auto& e : edges_) {
            if (e.weight > 0 && e.delay < config_.hold_margin) {
                // Ensure at least 1 register remains on short paths
                constraints.push_back({e.v, e.u, -(e.weight)});
            }
        }
    }

    // Frozen node constraints: r(v) = 0 for frozen nodes
    // Implement as: r(v) ≤ 0 AND r(v) ≥ 0 → edges s→v weight 0 AND v→s weight 0
    // (effectively, we force dist[v] = 0)

    // Add virtual source node (index n) with edges to all nodes, weight 0
    int s = n;
    for (int i = 0; i < n; ++i)
        constraints.push_back({s, i, 0});

    // Bellman-Ford from s
    int total = n + 1;
    const double INF = 1e18;
    std::vector<double> dist(total, INF);
    dist[s] = 0;

    for (int iter = 0; iter < total; ++iter) {
        bool changed = false;
        for (auto& c : constraints) {
            if (dist[c.from] < INF && dist[c.from] + c.wt < dist[c.to]) {
                dist[c.to] = dist[c.from] + c.wt;
                changed = true;
            }
        }
        if (!changed) break;

        // Negative cycle detection on last iteration
        if (iter == total - 1 && changed)
            return false; // infeasible for this target period
    }

    // Extract r-values (rounded to integer)
    bool any_nonzero = false;
    for (int i = 0; i < n; ++i) {
        if (nodes_[i].frozen) {
            nodes_[i].r_val = 0; // frozen nodes can't be retimed
        } else {
            nodes_[i].r_val = static_cast<int>(std::round(dist[i]));
        }
        if (nodes_[i].r_val != 0) any_nonzero = true;
    }

    // Validate resource constraints
    if (any_nonzero && !validate_resource_constraints()) {
        // Reset r-values if we'd exceed register budget
        for (auto& node : nodes_) node.r_val = 0;
        return false;
    }

    return any_nonzero;
}

// ============================================================================
// Structural DFF Movement
// ============================================================================

int RetimingEngine::apply_retiming(Netlist& nl) {
    // For each edge (u,v), the retimed register count is:
    //   new_w = w(e) + r(v) - r(u)
    //
    // If new_w > w(e): INSERT (new_w - w(e)) DFFs on this edge.
    // If new_w < w(e): REMOVE (w(e) - new_w) DFFs from this edge.
    //
    // Implementation:
    //   INSERT DFF: create new net, new DFF gate, splice into connection.
    //   REMOVE DFF: bypass the DFF, connect source directly to sink.

    int total_changes = 0;
    int inserted = 0, removed = 0;

    // Collect removals first (DFF gate IDs to mark for bypass)
    // and insertions (edges needing new DFFs)
    struct Insertion {
        GateId src_gid;  // source gate
        GateId sink_gid; // sink gate (the actual combinational gate)
        GateId dff_gid;  // existing DFF (-1 if none)
    };

    for (auto& e : edges_) {
        int new_w = e.weight + nodes_[e.v].r_val - nodes_[e.u].r_val;
        int delta = new_w - e.weight;

        if (delta > 0) {
            // INSERT delta DFFs on this edge
            GateId src_gid = nodes_[e.u].gid;
            GateId sink_gid = nodes_[e.v].gid;
            Gate& src_gate = nl.gate(src_gid);

            if (src_gate.output < 0) continue;

            for (int d = 0; d < delta; ++d) {
                // Create intermediate net and DFF
                NetId mid_net = nl.add_net("_retime_n" + std::to_string(total_changes));
                NetId dff_out = nl.add_net("_retime_q" + std::to_string(total_changes));

                // Find the clock net from an existing DFF (if any)
                NetId clk_net = -1;
                for (auto ff : nl.flip_flops()) {
                    clk_net = nl.gate(ff).clk;
                    if (clk_net >= 0) break;
                }

                // Splice: src_gate.output → mid_net → DFF → dff_out → sink_gate input
                // Reconnect: src drives mid_net, DFF latches mid_net → dff_out,
                //            sink reads from dff_out instead of src_gate.output

                // Add the DFF
                nl.add_dff(mid_net, clk_net, dff_out, -1,
                           "_retime_ff" + std::to_string(total_changes));

                // Reconnect sink gate: replace the input that was driven by src_gate
                Gate& sink_gate = nl.gate(sink_gid);
                NetId src_out = src_gate.output;
                for (auto& inp : sink_gate.inputs) {
                    if (inp == src_out) {
                        inp = dff_out;
                        break;
                    }
                }

                // Update fanout lists
                Net& orig_net = nl.net(src_out);
                auto& fo = orig_net.fanout;
                fo.erase(std::remove(fo.begin(), fo.end(), sink_gid), fo.end());
                // src now also drives mid_net
                nl.net(mid_net).driver = src_gid;
                nl.net(dff_out).fanout.push_back(sink_gid);

                inserted++;
                total_changes++;
            }
        } else if (delta < 0 && e.weight > 0) {
            // REMOVE |delta| DFFs from this edge
            // Find the DFF between src and sink
            GateId src_gid = nodes_[e.u].gid;
            Gate& src_gate = nl.gate(src_gid);
            if (src_gate.output < 0) continue;

            Net& src_out_net = nl.net(src_gate.output);
            for (int d = 0; d < std::abs(delta); ++d) {
                // Find a DFF in the fanout of src's output
                GateId dff_to_remove = -1;
                for (auto fo : src_out_net.fanout) {
                    if (nl.gate(fo).type == GateType::DFF) {
                        dff_to_remove = fo;
                        break;
                    }
                }

                if (dff_to_remove >= 0) {
                    Gate& dff = nl.gate(dff_to_remove);
                    if (dff.output >= 0) {
                        Net& dff_out = nl.net(dff.output);
                        // Bypass: connect DFF's fanout directly to src's output
                        for (auto fo : dff_out.fanout) {
                            Gate& sink = nl.gate(fo);
                            for (auto& inp : sink.inputs) {
                                if (inp == dff.output) {
                                    inp = src_gate.output;
                                    break;
                                }
                            }
                            src_out_net.fanout.push_back(fo);
                        }
                        dff_out.fanout.clear();
                    }
                    // Mark DFF as removed (set type to BUF as tombstone)
                    dff.type = GateType::BUF;
                    dff.inputs.clear();
                    removed++;
                    total_changes++;
                }
            }
        }
    }

    if (total_changes > 0) {
        std::cout << "[Retiming] " << inserted << " DFF(s) inserted, "
                  << removed << " DFF(s) removed. "
                  << "Net register count change: " << (inserted - removed) << "\n";
    }

    return total_changes;
}

// ============================================================================
// Top-Level Optimize
// ============================================================================

bool RetimingEngine::optimize(Netlist& nl) {
    auto res = optimize_with_result(nl);
    return res.improved;
}

RetimingResult RetimingEngine::optimize_with_result(Netlist& nl) {
    RetimingResult result;

    if (nl.flip_flops().empty()) {
        result.message = "No flip-flops — retiming not applicable.";
        std::cout << "[Retiming] " << result.message << "\n";
        return result;
    }

    result.register_count_before = static_cast<int>(nl.flip_flops().size());

    std::cout << "[Retiming] Building Leiserson-Saxe graph...\n";
    build_graph(nl);

    if (nodes_.empty() || edges_.empty()) {
        result.message = "Trivial graph — nothing to retime.";
        std::cout << "[Retiming] " << result.message << "\n";
        return result;
    }

    // Industrial: detect and freeze special cells
    detect_clock_gating(nl);
    detect_reset_paths(nl);
    apply_multicycle_relaxation();
    freeze_dont_retime_cells();

    // Count frozen cells for reporting
    for (auto& node : nodes_) {
        if (node.frozen) {
            if (clock_gating_cells_.count(node.gid))
                result.clock_gating_preserved++;
            else
                result.reset_paths_preserved++;
        }
    }
    result.multicycle_paths_honored = static_cast<int>(multicycle_paths_.size());

    // Find current critical path
    result.critical_path_before = compute_critical_path();
    std::cout << "  Critical path delay (before): " << result.critical_path_before
              << " units\n";

    if (result.critical_path_before <= 0) {
        result.message = "Zero critical path — nothing to optimize.";
        return result;
    }

    result.fmax_before = result.critical_path_before > 0 ? 1.0 / result.critical_path_before : 0;

    // Binary search for minimum feasible clock period
    double lo = 0;
    for (auto& n : nodes_) lo = std::max(lo, n.delay); // minimum = slowest gate
    double hi = result.critical_path_before;
    double best_T = hi;
    bool found_improvement = false;

    // Binary search: configurable iterations
    int steps = config_.binary_search_steps;
    for (int step = 0; step < steps; ++step) {
        double mid = (lo + hi) / 2.0;

        // Reset r-values
        for (auto& n : nodes_) n.r_val = 0;

        if (compute_retiming_values(mid)) {
            best_T = mid;
            hi = mid;
            found_improvement = true;
        } else {
            lo = mid;
        }
        result.iterations = step + 1;
    }

    double improvement_pct = (result.critical_path_before - best_T) / result.critical_path_before;
    if (!found_improvement || improvement_pct < config_.improvement_threshold) {
        result.message = "Already optimally pipelined.";
        result.critical_path_after = result.critical_path_before;
        result.fmax_after = result.fmax_before;
        result.register_count_after = result.register_count_before;
        std::cout << "[Retiming] " << result.message << "\n";
        return result;
    }

    // Recompute r-values for the best feasible period
    for (auto& n : nodes_) n.r_val = 0;
    if (!compute_retiming_values(best_T)) {
        result.message = "Failed to recompute retiming for best period.";
        result.critical_path_after = result.critical_path_before;
        result.fmax_after = result.fmax_before;
        result.register_count_after = result.register_count_before;
        return result;
    }

    // Apply structural changes
    std::cout << "[Retiming] Applying retiming for target period " << best_T << "...\n";
    int changes = apply_retiming(nl);
    result.registers_moved = changes;
    result.critical_path_after = best_T;
    result.improved = (changes > 0);

    // Count insertions/removals from r-values
    for (auto& e : edges_) {
        int delta = nodes_[e.v].r_val - nodes_[e.u].r_val;
        if (e.weight + delta > e.weight)
            result.dffs_inserted += (e.weight + delta - e.weight);
        else if (e.weight + delta < e.weight)
            result.dffs_removed += (e.weight - (e.weight + delta));
    }

    // Industrial metrics
    result.register_count_after = result.register_count_before + result.dffs_inserted - result.dffs_removed;
    result.fmax_after = best_T > 0 ? 1.0 / best_T : 0;
    result.fmax_improvement_pct = result.fmax_before > 0 ?
        ((result.fmax_after - result.fmax_before) / result.fmax_before) * 100.0 : 0;

    result.message = "Fmax improved: period " +
                     std::to_string(result.critical_path_before) + " -> " +
                     std::to_string(result.critical_path_after) + " units (" +
                     std::to_string(changes) + " register moves)";
    if (result.clock_gating_preserved > 0)
        result.message += ", " + std::to_string(result.clock_gating_preserved) + " ICG preserved";
    if (result.reset_paths_preserved > 0)
        result.message += ", " + std::to_string(result.reset_paths_preserved) + " reset preserved";

    std::cout << "[Retiming] " << result.message << "\n";

    return result;
}

} // namespace sf
