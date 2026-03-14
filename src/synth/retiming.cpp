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
#include <chrono>

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

// ============================================================================
// Bounded Retiming
// ============================================================================

RetimingEngine::BoundedResult RetimingEngine::retime_bounded(Netlist& nl, const BoundedConfig& cfg) {
    BoundedResult res{0, 0, 0.0, true};

    if (nl.flip_flops().empty()) return res;

    build_graph(nl);
    if (nodes_.empty() || edges_.empty()) return res;

    detect_clock_gating(nl);
    detect_reset_paths(nl);
    apply_multicycle_relaxation();
    freeze_dont_retime_cells();

    double cp_before = compute_critical_path();
    if (cp_before <= 0) return res;

    // Record IO-boundary node indices for latency preservation
    std::unordered_set<int> io_nodes;
    if (cfg.preserve_io_latency) {
        for (auto& node : nodes_) {
            const Gate& g = nl.gate(node.gid);
            if (g.inputs.empty() || g.type == GateType::DFF) {
                io_nodes.insert(gid_to_idx_[node.gid]);
            }
        }
    }

    // Binary search for best feasible period
    double lo = 0;
    for (auto& n : nodes_) lo = std::max(lo, n.delay);
    double hi = cp_before;
    double best_T = hi;

    for (int step = 0; step < config_.binary_search_steps; ++step) {
        double mid = (lo + hi) / 2.0;
        for (auto& n : nodes_) n.r_val = 0;
        if (compute_retiming_values(mid)) {
            best_T = mid;
            hi = mid;
        } else {
            lo = mid;
        }
    }

    // Recompute r-values at best period
    for (auto& n : nodes_) n.r_val = 0;
    if (!compute_retiming_values(best_T)) {
        res.delay_improvement_ns = 0;
        return res;
    }

    // Clamp r-values to bounded forward/backward move limits
    for (auto& n : nodes_) {
        if (n.frozen) continue;
        if (n.r_val > 0) {
            n.r_val = std::min(n.r_val, cfg.max_forward_moves);
            res.forward_moves += n.r_val;
        } else if (n.r_val < 0) {
            n.r_val = std::max(n.r_val, -cfg.max_backward_moves);
            res.backward_moves += (-n.r_val);
        }
    }

    // Preserve IO latency: zero out r-values on IO-boundary nodes
    if (cfg.preserve_io_latency) {
        for (int idx : io_nodes) {
            if (nodes_[idx].r_val != 0) {
                nodes_[idx].r_val = 0;
            }
        }
        res.io_latency_preserved = true;
    }

    apply_retiming(nl);

    double cp_after = compute_critical_path();
    res.delay_improvement_ns = cp_before - cp_after;
    if (res.delay_improvement_ns < 0) res.delay_improvement_ns = 0;

    std::cout << "[Retiming] Bounded: fwd=" << res.forward_moves
              << " bwd=" << res.backward_moves
              << " improvement=" << res.delay_improvement_ns << " ns\n";
    return res;
}

// ============================================================================
// Power-Aware Retiming
// ============================================================================

RetimingEngine::PowerRetimeResult RetimingEngine::retime_power_aware(Netlist& nl) {
    PowerRetimeResult res{0.0, 0.0, 0.0, 0};

    if (nl.flip_flops().empty()) return res;

    build_graph(nl);
    if (nodes_.empty() || edges_.empty()) return res;

    detect_clock_gating(nl);
    detect_reset_paths(nl);
    apply_multicycle_relaxation();
    freeze_dont_retime_cells();

    // Estimate switching power: sum of (fanout_count * gate_delay) as proxy
    // for dynamic switching activity on each edge
    auto estimate_switching_power = [&]() -> double {
        double power = 0.0;
        for (auto& e : edges_) {
            double activity = nodes_[e.u].delay;
            int weight = e.weight;
            // Registers on an edge reduce downstream switching activity
            double attenuation = (weight > 0) ? 0.5 : 1.0;
            power += activity * attenuation;
        }
        return power;
    };

    res.switching_power_before = estimate_switching_power();

    // Find critical path and target period (relax slightly to trade timing for power)
    double cp = compute_critical_path();
    if (cp <= 0) return res;

    // Target period: allow 5% timing slack to favour power-optimal register placement
    double target = cp * 1.05;

    for (auto& n : nodes_) n.r_val = 0;
    if (!compute_retiming_values(target)) {
        // Fall back to current period
        for (auto& n : nodes_) n.r_val = 0;
        if (!compute_retiming_values(cp)) return res;
    }

    // Bias r-values toward placing registers on high-activity edges
    // (move registers forward past high-switching nodes to reduce glitch propagation)
    for (size_t i = 0; i < nodes_.size(); ++i) {
        auto& n = nodes_[i];
        if (n.frozen) continue;
        // Nodes with high delay (proxy for high switching) benefit from forward retiming
        if (n.delay > cp * 0.3 && n.r_val == 0) {
            // Check if a small forward move is feasible (doesn't violate edge weights)
            bool can_move = true;
            for (auto& e : edges_) {
                if (e.u == static_cast<int>(i) && e.weight + 1 - n.r_val < 0) {
                    can_move = false;
                    break;
                }
            }
            if (can_move) n.r_val = 1;
        }
    }

    int changes = apply_retiming(nl);
    res.registers_moved = changes;

    // Rebuild graph to re-estimate power after retiming
    build_graph(nl);
    res.switching_power_after = estimate_switching_power();

    if (res.switching_power_before > 0) {
        res.power_reduction_pct =
            ((res.switching_power_before - res.switching_power_after) /
             res.switching_power_before) * 100.0;
    }

    std::cout << "[Retiming] Power-aware: power " << res.switching_power_before
              << " -> " << res.switching_power_after
              << " (" << res.power_reduction_pct << "% reduction, "
              << res.registers_moved << " regs moved)\n";
    return res;
}

// ============================================================================
// Incremental Retiming
// ============================================================================

RetimingEngine::IncrementalRetimeResult RetimingEngine::retime_incremental(
        Netlist& nl, const std::vector<int>& changed_gates) {
    IncrementalRetimeResult res{0, 0.0, 0.0};

    auto t_start = std::chrono::steady_clock::now();

    if (nl.flip_flops().empty() || changed_gates.empty()) {
        auto t_end = std::chrono::steady_clock::now();
        res.runtime_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        return res;
    }

    build_graph(nl);
    if (nodes_.empty() || edges_.empty()) {
        auto t_end = std::chrono::steady_clock::now();
        res.runtime_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        return res;
    }

    detect_clock_gating(nl);
    detect_reset_paths(nl);
    apply_multicycle_relaxation();
    freeze_dont_retime_cells();

    double cp_before = compute_critical_path();

    // Build neighbourhood: only allow retiming on nodes within 1 hop of changed gates
    std::unordered_set<int> active_set;
    for (int gid : changed_gates) {
        auto it = gid_to_idx_.find(gid);
        if (it == gid_to_idx_.end()) continue;
        active_set.insert(it->second);
    }
    // Expand to direct fanin/fanout neighbours
    std::unordered_set<int> expanded = active_set;
    for (auto& e : edges_) {
        if (active_set.count(e.u)) expanded.insert(e.v);
        if (active_set.count(e.v)) expanded.insert(e.u);
    }

    // Freeze everything outside the active neighbourhood
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (!expanded.count(static_cast<int>(i))) {
            nodes_[i].frozen = true;
        }
    }

    // Compute retiming for a slightly improved target
    double lo = 0;
    for (auto& n : nodes_) lo = std::max(lo, n.delay);
    double hi = cp_before;
    double best_T = hi;

    for (int step = 0; step < config_.binary_search_steps; ++step) {
        double mid = (lo + hi) / 2.0;
        for (auto& n : nodes_) n.r_val = 0;
        if (compute_retiming_values(mid)) {
            best_T = mid;
            hi = mid;
        } else {
            lo = mid;
        }
    }

    for (auto& n : nodes_) n.r_val = 0;
    if (compute_retiming_values(best_T)) {
        res.registers_adjusted = apply_retiming(nl);
    }

    double cp_after = compute_critical_path();
    res.timing_improvement = cp_before - cp_after;
    if (res.timing_improvement < 0) res.timing_improvement = 0;

    auto t_end = std::chrono::steady_clock::now();
    res.runtime_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    std::cout << "[Retiming] Incremental: " << res.registers_adjusted
              << " regs adjusted, improvement=" << res.timing_improvement
              << " ns, runtime=" << res.runtime_ms << " ms\n";
    return res;
}

// ============================================================================
// Sequential Optimization (iterative retiming)
// ============================================================================

RetimingEngine::SeqOptResult RetimingEngine::sequential_optimize(Netlist& nl, int max_iter) {
    SeqOptResult res{0.0, 0.0, 0, 0, 0};

    if (nl.flip_flops().empty()) return res;

    res.registers_before = static_cast<int>(nl.flip_flops().size());

    // Initial graph build to get baseline delay
    build_graph(nl);
    if (nodes_.empty() || edges_.empty()) return res;

    detect_clock_gating(nl);
    detect_reset_paths(nl);
    apply_multicycle_relaxation();
    freeze_dont_retime_cells();

    res.delay_before = compute_critical_path();
    double current_delay = res.delay_before;

    std::cout << "[Retiming] SeqOpt: initial delay=" << current_delay
              << ", max_iter=" << max_iter << "\n";

    for (int iter = 0; iter < max_iter; ++iter) {
        // Rebuild graph each iteration (netlist may have changed)
        build_graph(nl);
        if (nodes_.empty() || edges_.empty()) break;

        detect_clock_gating(nl);
        detect_reset_paths(nl);
        apply_multicycle_relaxation();
        freeze_dont_retime_cells();

        double cp = compute_critical_path();
        if (cp <= 0) break;

        // Binary search for optimal period this iteration
        double lo = 0;
        for (auto& n : nodes_) lo = std::max(lo, n.delay);
        double hi = cp;
        double best_T = hi;
        bool found = false;

        for (int step = 0; step < config_.binary_search_steps; ++step) {
            double mid = (lo + hi) / 2.0;
            for (auto& n : nodes_) n.r_val = 0;
            if (compute_retiming_values(mid)) {
                best_T = mid;
                hi = mid;
                found = true;
            } else {
                lo = mid;
            }
        }

        if (!found) break;

        double improvement_pct = (cp - best_T) / cp;
        if (improvement_pct < config_.improvement_threshold) break;

        // Apply retiming for this iteration
        for (auto& n : nodes_) n.r_val = 0;
        if (!compute_retiming_values(best_T)) break;

        int changes = apply_retiming(nl);
        if (changes == 0) break;

        current_delay = best_T;
        res.iterations = iter + 1;

        std::cout << "[Retiming] SeqOpt iter " << (iter + 1)
                  << ": delay=" << current_delay
                  << " (" << changes << " reg moves)\n";
    }

    res.delay_after = current_delay;
    res.registers_after = static_cast<int>(nl.flip_flops().size());

    std::cout << "[Retiming] SeqOpt complete: delay " << res.delay_before
              << " -> " << res.delay_after
              << ", regs " << res.registers_before << " -> " << res.registers_after
              << " in " << res.iterations << " iterations\n";
    return res;
}

// ============================================================================
// Register Merging — remove redundant DFFs with identical D/CLK inputs
// ============================================================================

RetimingEngine::RegMergeResult RetimingEngine::merge_redundant_registers(Netlist& nl) {
    RegMergeResult res;
    const auto& dffs = nl.flip_flops();
    res.registers_before = static_cast<int>(dffs.size());

    if (dffs.size() < 2) {
        res.registers_after = res.registers_before;
        res.report = "RegMerge: fewer than 2 DFFs, nothing to merge";
        return res;
    }

    // Build signature map: (D_net, CLK_net) → list of DFF gate IDs
    // DFFs with identical D and CLK inputs are merge candidates
    struct DffSig {
        NetId d_net;
        NetId clk_net;
        bool operator==(const DffSig& o) const { return d_net == o.d_net && clk_net == o.clk_net; }
    };
    struct SigHash {
        size_t operator()(const DffSig& s) const {
            return std::hash<int64_t>()(
                (static_cast<int64_t>(s.d_net) << 32) | static_cast<int64_t>(s.clk_net));
        }
    };

    std::unordered_map<DffSig, std::vector<GateId>, SigHash> sig_map;

    for (auto gid : dffs) {
        auto& g = nl.gate(gid);
        if (g.type != GateType::DFF) continue;
        if (g.inputs.empty()) continue;

        // DFF has D as first input, CLK stored in .clk
        NetId d_net = g.inputs[0];
        NetId clk_net = g.clk;

        // Skip DFFs in the don't-retime set
        if (dont_retime_.count(gid)) continue;

        sig_map[{d_net, clk_net}].push_back(gid);
    }

    // For each group of DFFs with identical signatures, keep one and redirect others
    const double dff_area = 6.0; // estimated DFF area in um²
    int total_merged = 0;

    for (auto& [sig, group] : sig_map) {
        if (group.size() < 2) continue;

        // Keep the first DFF as the "survivor"
        GateId survivor = group[0];
        NetId survivor_q = nl.gate(survivor).output;

        for (size_t i = 1; i < group.size(); ++i) {
            GateId dup = group[i];
            auto& dup_gate = nl.gate(dup);
            NetId dup_q = dup_gate.output;

            // Redirect all fanout of the duplicate Q net to the survivor Q net
            if (dup_q >= 0 && survivor_q >= 0) {
                auto& dup_net = nl.net(dup_q);
                std::vector<GateId> fanout_copy = dup_net.fanout;
                for (auto fanout_gid : fanout_copy) {
                    auto& fg = nl.gate(fanout_gid);
                    for (auto& inp : fg.inputs) {
                        if (inp == dup_q) inp = survivor_q;
                    }
                }
            }

            // Deactivate the duplicate DFF (mark as BUF with no inputs)
            dup_gate.type = GateType::BUF;
            dup_gate.inputs.clear();
            dup_gate.clk = -1;
            total_merged++;
        }
    }

    res.merges_performed = total_merged;
    res.registers_after = res.registers_before - total_merged;
    res.area_saved = total_merged * dff_area;

    std::cout << "[Retiming] RegMerge: " << res.registers_before << " DFFs -> "
              << res.registers_after << " DFFs (" << total_merged
              << " merged, " << res.area_saved << " um² saved)\n";

    res.report = "RegMerge: " + std::to_string(total_merged) + " redundant DFFs merged, "
               + std::to_string(res.registers_before) + " -> "
               + std::to_string(res.registers_after) + " registers, "
               + std::to_string(res.area_saved) + " um² area saved";
    return res;
}

// ============================================================================
// Pipeline Balancing — equalize combinational depth between register stages
// ============================================================================

RetimingEngine::RegBalanceResult RetimingEngine::balance_pipeline(Netlist& nl, int target_stages) {
    RegBalanceResult res;

    const auto& dffs = nl.flip_flops();
    if (dffs.empty()) {
        res.report = "PipeBalance: no DFFs in design";
        return res;
    }

    // Build a combinational depth map from PIs and DFF outputs (stage boundaries)
    // to the next DFF inputs (next stage boundary)
    std::unordered_map<GateId, int> gate_depth; // gate -> comb depth from nearest upstream reg
    std::unordered_set<NetId> stage_starts;      // nets that start a comb stage (PI or DFF Q)

    // Mark PI nets and DFF Q outputs as stage start boundaries
    for (auto pi : nl.primary_inputs()) stage_starts.insert(pi);
    for (auto gid : dffs) {
        auto& g = nl.gate(gid);
        if (g.output >= 0) stage_starts.insert(g.output);
    }

    // Topological traversal to compute depth of each comb gate
    auto topo = nl.topo_order();
    for (auto gid : topo) {
        auto& g = nl.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT) {
            gate_depth[gid] = 0;
            continue;
        }
        int max_input_depth = 0;
        for (auto inp_net : g.inputs) {
            if (stage_starts.count(inp_net)) {
                // Directly driven by stage boundary — depth = 1
                max_input_depth = std::max(max_input_depth, 1);
            } else {
                // Check driver gate depth
                auto& n = nl.net(inp_net);
                if (n.driver >= 0 && gate_depth.count(n.driver)) {
                    max_input_depth = std::max(max_input_depth, gate_depth[n.driver] + 1);
                } else {
                    max_input_depth = std::max(max_input_depth, 1);
                }
            }
        }
        gate_depth[gid] = max_input_depth;
    }

    // Compute per-stage depths: for each DFF, its D-input depth is the stage depth
    std::vector<int> stage_depths;
    for (auto gid : dffs) {
        auto& g = nl.gate(gid);
        if (g.inputs.empty()) continue;
        NetId d_net = g.inputs[0];
        auto& dn = nl.net(d_net);
        int depth = 0;
        if (dn.driver >= 0 && gate_depth.count(dn.driver)) {
            depth = gate_depth[dn.driver];
        }
        stage_depths.push_back(std::max(depth, 1));
    }

    if (stage_depths.empty()) {
        res.report = "PipeBalance: no measurable pipeline stages";
        return res;
    }

    res.stages_before = static_cast<int>(stage_depths.size());

    // Compute imbalance: max stage depth vs. average
    int max_depth = *std::max_element(stage_depths.begin(), stage_depths.end());
    int min_depth = *std::min_element(stage_depths.begin(), stage_depths.end());
    double avg_depth = 0;
    for (int d : stage_depths) avg_depth += d;
    avg_depth /= static_cast<double>(stage_depths.size());

    double imbalance = (max_depth > 0) ? static_cast<double>(max_depth - min_depth) / max_depth : 0;

    std::cout << "[Retiming] PipeBalance: " << stage_depths.size() << " stages, depth range ["
              << min_depth << ", " << max_depth << "], avg=" << avg_depth
              << ", imbalance=" << (imbalance * 100) << "%\n";

    // If imbalance ≤ 20%, no redistribution needed
    if (imbalance <= 0.20) {
        res.stages_balanced = res.stages_before;
        res.throughput_improvement_pct = 0;
        res.report = "PipeBalance: stages balanced (imbalance " +
                     std::to_string(static_cast<int>(imbalance * 100)) + "% <= 20%)";
        return res;
    }

    // Determine target depth per stage
    int tgt = (target_stages > 0) ? target_stages : static_cast<int>(std::ceil(avg_depth));
    if (tgt < 1) tgt = 1;

    // Use retiming to redistribute: for each overly deep stage, attempt
    // forward retiming to push registers deeper; for shallow stages,
    // attempt backward retiming to pull registers earlier
    build_graph(nl);
    if (nodes_.empty() || edges_.empty()) {
        res.report = "PipeBalance: could not build retiming graph";
        return res;
    }

    detect_clock_gating(nl);
    detect_reset_paths(nl);
    freeze_dont_retime_cells();

    double original_cp = compute_critical_path();
    if (original_cp <= 0) {
        res.report = "PipeBalance: zero critical path";
        return res;
    }

    // Target period: scale to equalize stages (reduce max depth impact)
    double target_period = original_cp * (avg_depth / max_depth);
    target_period = std::max(target_period, avg_depth); // floor to average

    // Reset r-values and attempt retiming to the balanced target
    for (auto& n : nodes_) n.r_val = 0;
    bool feasible = compute_retiming_values(target_period);
    int balanced_count = 0;

    if (feasible) {
        int changes = apply_retiming(nl);
        if (changes > 0) {
            // Recompute stage depths after retiming
            gate_depth.clear();
            auto topo2 = nl.topo_order();
            stage_starts.clear();
            for (auto pi : nl.primary_inputs()) stage_starts.insert(pi);
            for (auto gid2 : nl.flip_flops()) {
                auto& g2 = nl.gate(gid2);
                if (g2.output >= 0) stage_starts.insert(g2.output);
            }
            for (auto gid2 : topo2) {
                auto& g2 = nl.gate(gid2);
                if (g2.type == GateType::DFF || g2.type == GateType::INPUT) {
                    gate_depth[gid2] = 0;
                    continue;
                }
                int mxd = 0;
                for (auto inp_net : g2.inputs) {
                    if (stage_starts.count(inp_net)) {
                        mxd = std::max(mxd, 1);
                    } else {
                        auto& nn = nl.net(inp_net);
                        if (nn.driver >= 0 && gate_depth.count(nn.driver))
                            mxd = std::max(mxd, gate_depth[nn.driver] + 1);
                        else
                            mxd = std::max(mxd, 1);
                    }
                }
                gate_depth[gid2] = mxd;
            }

            // Recompute post-balance max depth
            int new_max = 0;
            for (auto gid2 : nl.flip_flops()) {
                auto& g2 = nl.gate(gid2);
                if (g2.inputs.empty()) continue;
                auto& dn2 = nl.net(g2.inputs[0]);
                int d = (dn2.driver >= 0 && gate_depth.count(dn2.driver))
                        ? gate_depth[dn2.driver] : 1;
                new_max = std::max(new_max, d);
            }

            balanced_count = static_cast<int>(nl.flip_flops().size());
            double new_throughput = (new_max > 0) ? 1.0 / new_max : 0;
            double old_throughput = (max_depth > 0) ? 1.0 / max_depth : 0;
            if (old_throughput > 0)
                res.throughput_improvement_pct =
                    (new_throughput - old_throughput) / old_throughput * 100.0;
        }
    }

    res.stages_balanced = (balanced_count > 0) ? balanced_count : res.stages_before;

    std::cout << "[Retiming] PipeBalance: redistribution "
              << (feasible ? "succeeded" : "infeasible")
              << ", throughput improvement: " << res.throughput_improvement_pct << "%\n";

    res.report = "PipeBalance: " + std::to_string(res.stages_before) + " stages, imbalance "
               + std::to_string(static_cast<int>(imbalance * 100)) + "% -> balanced"
               + ", throughput +" + std::to_string(static_cast<int>(res.throughput_improvement_pct)) + "%";
    return res;
}

} // namespace sf
