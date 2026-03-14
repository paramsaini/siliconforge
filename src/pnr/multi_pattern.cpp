// SiliconForge — Multi-Patterning (SADP/SAQP) Engine Implementation

#include "pnr/multi_pattern.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_set>

namespace sf {

// ────────────────────────────────────────────────────────────────────
// Construction
// ────────────────────────────────────────────────────────────────────

MultiPatternEngine::MultiPatternEngine(PhysicalDesign& pd) : pd_(pd) {}

// ────────────────────────────────────────────────────────────────────
// Geometry helpers
// ────────────────────────────────────────────────────────────────────

double MultiPatternEngine::parallel_run_length(const WireSegment& a,
                                               const WireSegment& b) const {
    bool a_horiz = is_horizontal(a);
    bool b_horiz = is_horizontal(b);
    if (a_horiz != b_horiz) return 0.0;

    if (a_horiz) {
        double a_lo = std::min(a.start.x, a.end.x);
        double a_hi = std::max(a.start.x, a.end.x);
        double b_lo = std::min(b.start.x, b.end.x);
        double b_hi = std::max(b.start.x, b.end.x);
        double overlap = std::min(a_hi, b_hi) - std::max(a_lo, b_lo);
        return std::max(0.0, overlap);
    }
    // both vertical
    double a_lo = std::min(a.start.y, a.end.y);
    double a_hi = std::max(a.start.y, a.end.y);
    double b_lo = std::min(b.start.y, b.end.y);
    double b_hi = std::max(b.start.y, b.end.y);
    double overlap = std::min(a_hi, b_hi) - std::max(a_lo, b_lo);
    return std::max(0.0, overlap);
}

double MultiPatternEngine::segment_spacing(const WireSegment& a,
                                           const WireSegment& b) const {
    bool a_horiz = is_horizontal(a);
    bool b_horiz = is_horizontal(b);
    if (a_horiz != b_horiz) return std::numeric_limits<double>::max();

    if (a_horiz) {
        double gap = std::abs(a.start.y - b.start.y);
        double edge = gap - (a.width + b.width) / 2.0;
        return std::max(0.0, edge);
    }
    double gap = std::abs(a.start.x - b.start.x);
    double edge = gap - (a.width + b.width) / 2.0;
    return std::max(0.0, edge);
}

// ────────────────────────────────────────────────────────────────────
// Conflict graph construction
// ────────────────────────────────────────────────────────────────────

MultiPatternEngine::ConflictGraph
MultiPatternEngine::build_conflict_graph(int layer, double min_space) {
    ConflictGraph g;

    // Collect wire indices on this layer
    for (int i = 0; i < static_cast<int>(pd_.wires.size()); ++i) {
        if (pd_.wires[i].layer == layer) {
            g.wire_indices.push_back(i);
        }
    }

    g.num_nodes = static_cast<int>(g.wire_indices.size());
    g.adj.resize(g.num_nodes);

    for (int i = 0; i < g.num_nodes; ++i) {
        for (int j = i + 1; j < g.num_nodes; ++j) {
            const auto& wa = pd_.wires[g.wire_indices[i]];
            const auto& wb = pd_.wires[g.wire_indices[j]];

            double prl = parallel_run_length(wa, wb);
            if (prl <= 0.0) continue;

            double sp = segment_spacing(wa, wb);
            if (sp < min_space) {
                g.adj[i].push_back(j);
                g.adj[j].push_back(i);
            }
        }
    }
    return g;
}

// ────────────────────────────────────────────────────────────────────
// 2-coloring (BFS bipartite check)
// ────────────────────────────────────────────────────────────────────

bool MultiPatternEngine::try_2_color(const ConflictGraph& g,
                                     std::vector<Color>& colors) {
    colors.assign(g.num_nodes, Color::UNCOLORED);
    if (g.num_nodes == 0) return true;

    for (int start = 0; start < g.num_nodes; ++start) {
        if (colors[start] != Color::UNCOLORED) continue;

        colors[start] = Color::COLOR_A;
        std::queue<int> q;
        q.push(start);

        while (!q.empty()) {
            int u = q.front();
            q.pop();
            Color next = (colors[u] == Color::COLOR_A) ? Color::COLOR_B
                                                       : Color::COLOR_A;
            for (int v : g.adj[u]) {
                if (colors[v] == Color::UNCOLORED) {
                    colors[v] = next;
                    q.push(v);
                } else if (colors[v] == colors[u]) {
                    return false; // odd cycle
                }
            }
        }
    }
    return true;
}

// ────────────────────────────────────────────────────────────────────
// 4-coloring (greedy with smallest available color)
// ────────────────────────────────────────────────────────────────────

bool MultiPatternEngine::try_4_color(const ConflictGraph& g,
                                     std::vector<Color>& colors) {
    colors.assign(g.num_nodes, Color::UNCOLORED);
    if (g.num_nodes == 0) return true;

    static const Color palette[] = {Color::COLOR_A, Color::COLOR_B,
                                    Color::COLOR_C, Color::COLOR_D};

    // Order nodes by descending degree for better coloring
    std::vector<int> order(g.num_nodes);
    for (int i = 0; i < g.num_nodes; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return g.adj[a].size() > g.adj[b].size();
    });

    for (int node : order) {
        std::unordered_set<int> used;
        for (int nb : g.adj[node]) {
            if (colors[nb] != Color::UNCOLORED)
                used.insert(static_cast<int>(colors[nb]));
        }

        bool assigned = false;
        for (auto c : palette) {
            if (used.find(static_cast<int>(c)) == used.end()) {
                colors[node] = c;
                assigned = true;
                break;
            }
        }
        if (!assigned) return false;
    }
    return true;
}

// ────────────────────────────────────────────────────────────────────
// color_graph — unified entry point
// ────────────────────────────────────────────────────────────────────

std::vector<ColorAssignment>
MultiPatternEngine::color_graph(int layer) {
    ConflictGraph g = build_conflict_graph(layer, cfg_.min_space_nm);
    std::vector<Color> colors;
    std::vector<ColorAssignment> out;

    bool ok = false;
    if (cfg_.max_colors <= 2) {
        ok = try_2_color(g, colors);
    }
    if (!ok) {
        ok = try_4_color(g, colors);
    }

    out.reserve(g.num_nodes);
    for (int i = 0; i < g.num_nodes; ++i) {
        ColorAssignment ca;
        ca.wire_idx = g.wire_indices[i];
        ca.layer = layer;
        ca.color = colors[i];
        out.push_back(ca);
    }
    return out;
}

// ────────────────────────────────────────────────────────────────────
// Conflict detection
// ────────────────────────────────────────────────────────────────────

std::vector<ColorConflict>
MultiPatternEngine::detect_conflicts(
    const std::vector<ColorAssignment>& assignments) {
    std::vector<ColorConflict> conflicts;

    for (size_t i = 0; i < assignments.size(); ++i) {
        for (size_t j = i + 1; j < assignments.size(); ++j) {
            const auto& ai = assignments[i];
            const auto& aj = assignments[j];
            if (ai.layer != aj.layer) continue;
            if (ai.color != aj.color) continue;
            if (ai.color == Color::UNCOLORED) continue;

            const auto& wa = pd_.wires[ai.wire_idx];
            const auto& wb = pd_.wires[aj.wire_idx];

            double prl = parallel_run_length(wa, wb);
            if (prl <= 0.0) continue;

            double sp = segment_spacing(wa, wb);
            if (sp < cfg_.min_space_nm) {
                ColorConflict cc;
                cc.wire_a = ai.wire_idx;
                cc.wire_b = aj.wire_idx;
                cc.layer = ai.layer;
                cc.spacing_nm = sp;
                cc.min_required_nm = cfg_.min_space_nm;
                cc.resolution = "none";
                conflicts.push_back(cc);
            }
        }
    }
    return conflicts;
}

// ────────────────────────────────────────────────────────────────────
// Conflict resolution by recoloring
// ────────────────────────────────────────────────────────────────────

int MultiPatternEngine::resolve_conflicts(
    std::vector<ColorAssignment>& assignments,
    const MultiPatternConfig& cfg) {
    int resolved = 0;
    int max_iters = static_cast<int>(assignments.size()) * 2 + 10;

    for (int iter = 0; iter < max_iters; ++iter) {
        auto conflicts = detect_conflicts(assignments);
        if (conflicts.empty()) break;

        bool progress = false;
        for (auto& cc : conflicts) {
            // Find the assignment for wire_b and try to recolor it
            ColorAssignment* target = nullptr;
            for (auto& a : assignments) {
                if (a.wire_idx == cc.wire_b && a.layer == cc.layer) {
                    target = &a;
                    break;
                }
            }
            if (!target) continue;

            // Collect colors used by conflicting neighbors
            std::unordered_set<int> forbidden;
            for (auto& other : assignments) {
                if (other.wire_idx == target->wire_idx) continue;
                if (other.layer != target->layer) continue;

                const auto& wa = pd_.wires[target->wire_idx];
                const auto& wb = pd_.wires[other.wire_idx];
                double prl = parallel_run_length(wa, wb);
                if (prl <= 0.0) continue;
                double sp = segment_spacing(wa, wb);
                if (sp < cfg.min_space_nm) {
                    forbidden.insert(static_cast<int>(other.color));
                }
            }

            static const Color palette[] = {Color::COLOR_A, Color::COLOR_B,
                                             Color::COLOR_C, Color::COLOR_D};
            int n_colors = cfg.max_colors;

            bool swapped = false;
            for (int ci = 0; ci < n_colors; ++ci) {
                if (forbidden.find(ci) == forbidden.end()) {
                    target->color = palette[ci];
                    swapped = true;
                    break;
                }
            }
            if (swapped) {
                cc.resolution = "recolor";
                ++resolved;
                progress = true;
            }
        }
        if (!progress) break;
    }
    return resolved;
}

// ────────────────────────────────────────────────────────────────────
// SADP decomposition (2-color: mandrel / spacer)
// ────────────────────────────────────────────────────────────────────

MultiPatternResult MultiPatternEngine::sadp_decompose(int layer) {
    MultiPatternResult res;
    auto t0 = std::chrono::steady_clock::now();

    MultiPatternConfig local = cfg_;
    local.max_colors = 2;
    cfg_ = local;

    auto assignments = color_graph(layer);
    res.wires_colored = static_cast<int>(assignments.size());

    auto conflicts = detect_conflicts(assignments);
    res.conflicts_found = static_cast<int>(conflicts.size());

    int resolved = resolve_conflicts(assignments, local);
    res.conflicts_resolved = resolved;
    res.color_changes = resolved;

    auto remaining = detect_conflicts(assignments);
    res.unresolved = remaining;
    res.assignments = assignments;
    res.decomposition_legal = remaining.empty();

    double min_sp = std::numeric_limits<double>::max();
    for (size_t i = 0; i < assignments.size(); ++i) {
        for (size_t j = i + 1; j < assignments.size(); ++j) {
            if (assignments[i].layer != assignments[j].layer) continue;
            if (assignments[i].color != assignments[j].color) continue;
            if (assignments[i].color == Color::UNCOLORED) continue;
            const auto& wa = pd_.wires[assignments[i].wire_idx];
            const auto& wb = pd_.wires[assignments[j].wire_idx];
            double prl = parallel_run_length(wa, wb);
            if (prl <= 0.0) continue;
            double sp = segment_spacing(wa, wb);
            if (sp < min_sp) min_sp = sp;
        }
    }
    res.min_spacing_achieved_nm =
        (min_sp == std::numeric_limits<double>::max()) ? 0.0 : min_sp;

    auto t1 = std::chrono::steady_clock::now();
    res.time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::ostringstream oss;
    oss << "SADP layer " << layer << ": " << res.wires_colored << " wires, "
        << res.conflicts_found << " conflicts, " << res.conflicts_resolved
        << " resolved, " << remaining.size() << " unresolved";
    res.report = oss.str();
    return res;
}

// ────────────────────────────────────────────────────────────────────
// SAQP decomposition (4-color)
// ────────────────────────────────────────────────────────────────────

MultiPatternResult MultiPatternEngine::saqp_decompose(int layer) {
    MultiPatternResult res;
    auto t0 = std::chrono::steady_clock::now();

    MultiPatternConfig local = cfg_;
    local.max_colors = 4;
    cfg_ = local;

    auto assignments = color_graph(layer);
    res.wires_colored = static_cast<int>(assignments.size());

    auto conflicts = detect_conflicts(assignments);
    res.conflicts_found = static_cast<int>(conflicts.size());

    int resolved = resolve_conflicts(assignments, local);
    res.conflicts_resolved = resolved;
    res.color_changes = resolved;

    auto remaining = detect_conflicts(assignments);
    res.unresolved = remaining;
    res.assignments = assignments;
    res.decomposition_legal = remaining.empty();

    double min_sp = std::numeric_limits<double>::max();
    for (size_t i = 0; i < assignments.size(); ++i) {
        for (size_t j = i + 1; j < assignments.size(); ++j) {
            if (assignments[i].layer != assignments[j].layer) continue;
            if (assignments[i].color != assignments[j].color) continue;
            if (assignments[i].color == Color::UNCOLORED) continue;
            const auto& wa = pd_.wires[assignments[i].wire_idx];
            const auto& wb = pd_.wires[assignments[j].wire_idx];
            double prl = parallel_run_length(wa, wb);
            if (prl <= 0.0) continue;
            double sp = segment_spacing(wa, wb);
            if (sp < min_sp) min_sp = sp;
        }
    }
    res.min_spacing_achieved_nm =
        (min_sp == std::numeric_limits<double>::max()) ? 0.0 : min_sp;

    auto t1 = std::chrono::steady_clock::now();
    res.time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::ostringstream oss;
    oss << "SAQP layer " << layer << ": " << res.wires_colored << " wires, "
        << res.conflicts_found << " conflicts, " << res.conflicts_resolved
        << " resolved, " << remaining.size() << " unresolved";
    res.report = oss.str();
    return res;
}

// ────────────────────────────────────────────────────────────────────
// decompose — top-level dispatcher
// ────────────────────────────────────────────────────────────────────

MultiPatternResult MultiPatternEngine::decompose(const MultiPatternConfig& cfg) {
    cfg_ = cfg;
    MultiPatternResult combined;
    auto t0 = std::chrono::steady_clock::now();

    std::vector<int> layers = cfg.mp_layers;
    if (layers.empty()) {
        // Auto-detect layers that have wires
        std::unordered_set<int> seen;
        for (const auto& w : pd_.wires) {
            if (seen.insert(w.layer).second) layers.push_back(w.layer);
        }
        std::sort(layers.begin(), layers.end());
    }

    for (int ly : layers) {
        MultiPatternResult lr;
        if (cfg.type == PatternType::SAQP) {
            lr = saqp_decompose(ly);
        } else {
            lr = sadp_decompose(ly);
        }

        combined.wires_colored += lr.wires_colored;
        combined.conflicts_found += lr.conflicts_found;
        combined.conflicts_resolved += lr.conflicts_resolved;
        combined.color_changes += lr.color_changes;
        combined.assignments.insert(combined.assignments.end(),
                                    lr.assignments.begin(),
                                    lr.assignments.end());
        combined.unresolved.insert(combined.unresolved.end(),
                                   lr.unresolved.begin(),
                                   lr.unresolved.end());

        if (lr.min_spacing_achieved_nm > 0.0) {
            if (combined.min_spacing_achieved_nm <= 0.0 ||
                lr.min_spacing_achieved_nm < combined.min_spacing_achieved_nm) {
                combined.min_spacing_achieved_nm = lr.min_spacing_achieved_nm;
            }
        }
    }

    combined.decomposition_legal = combined.unresolved.empty();

    auto t1 = std::chrono::steady_clock::now();
    combined.time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::ostringstream oss;
    oss << "Multi-patterning decomposition: "
        << combined.wires_colored << " wires colored across "
        << layers.size() << " layers, "
        << combined.conflicts_found << " conflicts found, "
        << combined.conflicts_resolved << " resolved, "
        << combined.unresolved.size() << " unresolved | "
        << (combined.decomposition_legal ? "LEGAL" : "ILLEGAL");
    combined.report = oss.str();
    return combined;
}

// ────────────────────────────────────────────────────────────────────
// run_enhanced — full flow with report
// ────────────────────────────────────────────────────────────────────

MultiPatternResult MultiPatternEngine::run_enhanced() {
    MultiPatternResult res;
    auto t0 = std::chrono::steady_clock::now();

    // Phase 1: SADP pass on all layers
    MultiPatternConfig sadp_cfg = cfg_;
    sadp_cfg.type = PatternType::SADP;
    sadp_cfg.max_colors = 2;
    auto first = decompose(sadp_cfg);

    if (first.decomposition_legal) {
        first.report += " [SADP sufficient]";
        return first;
    }

    // Phase 2: upgrade unresolved layers to SAQP
    std::unordered_set<int> bad_layers;
    for (const auto& cc : first.unresolved) {
        bad_layers.insert(cc.layer);
    }

    MultiPatternConfig saqp_cfg = cfg_;
    saqp_cfg.type = PatternType::SAQP;
    saqp_cfg.max_colors = 4;
    saqp_cfg.mp_layers.assign(bad_layers.begin(), bad_layers.end());
    std::sort(saqp_cfg.mp_layers.begin(), saqp_cfg.mp_layers.end());

    auto second = decompose(saqp_cfg);

    // Merge: keep SADP results for clean layers, SAQP for upgraded ones
    res.wires_colored = 0;
    res.conflicts_found = first.conflicts_found + second.conflicts_found;
    res.conflicts_resolved = first.conflicts_resolved + second.conflicts_resolved;
    res.color_changes = first.color_changes + second.color_changes;

    // Take assignments from first pass for layers NOT in bad_layers
    for (const auto& a : first.assignments) {
        if (bad_layers.find(a.layer) == bad_layers.end()) {
            res.assignments.push_back(a);
            ++res.wires_colored;
        }
    }
    // Add SAQP assignments for upgraded layers
    for (const auto& a : second.assignments) {
        res.assignments.push_back(a);
        ++res.wires_colored;
    }

    res.unresolved = second.unresolved;
    res.decomposition_legal = res.unresolved.empty();

    if (first.min_spacing_achieved_nm > 0.0)
        res.min_spacing_achieved_nm = first.min_spacing_achieved_nm;
    if (second.min_spacing_achieved_nm > 0.0 &&
        (res.min_spacing_achieved_nm <= 0.0 ||
         second.min_spacing_achieved_nm < res.min_spacing_achieved_nm)) {
        res.min_spacing_achieved_nm = second.min_spacing_achieved_nm;
    }

    auto t1 = std::chrono::steady_clock::now();
    res.time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::ostringstream oss;
    oss << "Enhanced MP: " << res.wires_colored << " wires, "
        << bad_layers.size() << " layers upgraded to SAQP, "
        << res.unresolved.size() << " unresolved | "
        << (res.decomposition_legal ? "LEGAL" : "ILLEGAL");
    res.report = oss.str();
    return res;
}

} // namespace sf
