// SiliconForge — CTS Implementation (DME Algorithm)
#include "pnr/cts.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <numeric>
#include <set>
#include <limits>
#include <cassert>
#include <utility>

namespace sf {

int CtsEngine::dme_merge(const std::vector<int>& sinks) {
    if (sinks.size() == 1) return sinks[0];
    if (sinks.size() == 2) {
        // Merge two sinks into a Steiner point (midpoint for zero skew)
        auto& s0 = tree_[sinks[0]];
        auto& s1 = tree_[sinks[1]];
        TreeNode steiner;
        steiner.position = Point(
            (s0.position.x + s1.position.x) / 2,
            (s0.position.y + s1.position.y) / 2
        );
        steiner.left = sinks[0];
        steiner.right = sinks[1];
        steiner.delay = std::max(s0.delay, s1.delay) +
                       s0.position.dist(steiner.position) * 0.001; // wire delay model
        int id = (int)tree_.size();
        tree_.push_back(steiner);
        return id;
    }

    // Split into two balanced halves — sort by x, divide
    auto sorted = sinks;
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        return tree_[a].position.x < tree_[b].position.x;
    });

    size_t mid = sorted.size() / 2;
    std::vector<int> left(sorted.begin(), sorted.begin() + mid);
    std::vector<int> right(sorted.begin() + mid, sorted.end());

    int left_root = dme_merge(left);
    int right_root = dme_merge(right);
    return dme_merge({left_root, right_root});
}

CtsResult CtsEngine::build_clock_tree(const Point& source, const std::vector<int>& sink_cells) {
    auto t0 = std::chrono::high_resolution_clock::now();
    CtsResult result;

    if (sink_cells.empty()) {
        result.message = "No sinks for clock tree";
        return result;
    }

    tree_.clear();

    // Create leaf nodes for each sink
    std::vector<int> sink_nodes;
    for (int cid : sink_cells) {
        TreeNode node;
        node.position = Point(
            pd_.cells[cid].position.x + pd_.cells[cid].width / 2,
            pd_.cells[cid].position.y + pd_.cells[cid].height / 2
        );
        node.cell_id = cid;
        node.delay = 0;
        int id = (int)tree_.size();
        tree_.push_back(node);
        sink_nodes.push_back(id);
    }

    // Build DME tree
    int root = dme_merge(sink_nodes);

    // Connect root to source
    tree_[root].position = source;

    // Calculate metrics
    double total_wl = 0;
    double max_delay = 0, min_delay = 1e18;
    for (auto sid : sink_nodes) {
        // Trace path from sink to root
        // Simple: use direct Manhattan distance as approximation
        double d = tree_[sid].position.dist(source);
        total_wl += d;
        max_delay = std::max(max_delay, d * 0.001);
        min_delay = std::min(min_delay, d * 0.001);
    }

    // Create wire segments for the clock tree
    std::function<void(int)> create_wires = [&](int node) {
        auto& n = tree_[node];
        if (n.left >= 0) {
            pd_.wires.push_back({0, n.position, tree_[n.left].position, 0.14});
            create_wires(n.left);
        }
        if (n.right >= 0) {
            pd_.wires.push_back({0, n.position, tree_[n.right].position, 0.14});
            create_wires(n.right);
        }
    };
    create_wires(root);

    // Initialize buffer library for drive-strength-aware sizing
    init_buffer_library();
    
    // Insert buffers at intermediate nodes with proper sizing
    int buffers = 0;
    for (size_t i = sink_nodes.size(); i < tree_.size(); ++i) {
        if ((int)i != root && tree_[i].left >= 0) {
            // Compute load capacitance of subtree
            double subtree_cap = compute_subtree_cap((int)i);
            double wl = tree_[i].position.dist(tree_[root].position);
            
            // Select appropriately-sized buffer
            auto buf = select_buffer(subtree_cap, wl);
            
            pd_.add_cell("clkbuf_" + std::to_string(i), buf.name, buf.area, pd_.row_height);
            pd_.cells.back().position = tree_[i].position;
            pd_.cells.back().placed = true;
            buffers++;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.skew = max_delay - min_delay;
    result.wirelength = total_wl;
    result.buffers_inserted = buffers;
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = "Clock tree built — skew: " + std::to_string(result.skew) +
                     "ps, " + std::to_string(buffers) + " buffers";
    return result;
}


void CtsEngine::init_buffer_library() {
    // Clock buffer library: different drive strengths
    buf_lib_ = {
        {"CLKBUF_X1",  200.0, 2.0,  15.0, 1.0},  // weak, small
        {"CLKBUF_X2",  100.0, 4.0,  12.0, 2.0},  // medium
        {"CLKBUF_X4",   50.0, 8.0,  10.0, 4.0},  // strong
        {"CLKBUF_X8",   25.0, 16.0,  8.0, 8.0},  // very strong
        {"CLKBUF_X16",  12.5, 32.0,  7.0, 16.0}, // maximum drive
    };
}

ClkBufType CtsEngine::select_buffer(double load_cap, double wire_length) {
    // Select buffer based on load: use smallest buffer that meets timing
    // RC delay = R_buf * (C_wire + C_load) + R_wire * C_load
    double wire_cap = wire_length * 0.05; // fF/um
    double wire_res = wire_length * 0.1;  // Ohm/um
    double total_cap = wire_cap + load_cap;
    
    for (auto& buf : buf_lib_) {
        double delay = buf.drive_strength * total_cap * 1e-3 + wire_res * load_cap * 1e-3;
        if (delay < 50.0) return buf; // 50ps target
    }
    return buf_lib_.back(); // strongest if nothing meets target
}

double CtsEngine::compute_subtree_cap(int node) {
    auto& n = tree_[node];
    double cap = 0;
    if (n.cell_id >= 0) {
        // Leaf: input capacitance of the flip-flop
        cap = 5.0; // fF typical FF clock pin cap
    } else {
        if (n.left >= 0) {
            double wl = n.position.dist(tree_[n.left].position);
            cap += compute_subtree_cap(n.left) + wl * 0.05;
        }
        if (n.right >= 0) {
            double wl = n.position.dist(tree_[n.right].position);
            cap += compute_subtree_cap(n.right) + wl * 0.05;
        }
    }
    return cap;
}

// ---------------------------------------------------------------------------
// Multi-clock CTS
// ---------------------------------------------------------------------------

MultiCtsResult CtsEngine::build_multi_clock(const CtsConfig& cfg) {
    auto t0 = std::chrono::high_resolution_clock::now();
    MultiCtsResult result;

    if (cfg.domains.empty()) {
        result.report = "No clock domains specified";
        return result;
    }

    partition_sinks(cfg);

    // Build a tree for each domain using the appropriate strategy
    for (size_t di = 0; di < cfg.domains.size(); ++di) {
        const auto& dom = cfg.domains[di];
        CtsResult dr;

        if (dom.sink_cells.empty()) {
            dr.message = "Domain " + dom.name + ": no sinks";
            result.domain_results.push_back(dr);
            continue;
        }

        if (cfg.power_aware) {
            dr = build_power_aware_tree(dom);
        } else if (cfg.enable_useful_skew && dom.target_skew_ps > 0) {
            // Generate dummy slacks proportional to sink index spread
            std::vector<double> slacks(dom.sink_cells.size());
            for (size_t i = 0; i < slacks.size(); ++i)
                slacks[i] = dom.target_skew_ps *
                             (static_cast<double>(i) / std::max(slacks.size() - 1, size_t(1)) - 0.5);
            dr = build_useful_skew_tree(dom, slacks);
        } else {
            dr = build_clock_tree(dom.source, dom.sink_cells);
        }

        dr.message = "Domain " + dom.name + ": " + dr.message;
        result.domain_results.push_back(dr);
    }

    // Aggregate totals
    for (const auto& dr : result.domain_results) {
        result.total_skew = std::max(result.total_skew, dr.skew);
        result.total_wirelength += dr.wirelength;
        result.total_buffers += dr.buffers_inserted;
    }

    // Inter-domain skew for synchronous groups
    result.inter_domain_skew =
        compute_inter_domain_skew(result.domain_results, cfg.groups, cfg);

    // Power estimation across all domains (use last-built tree as proxy per domain)
    double total_power = 0;
    for (const auto& dom : cfg.domains) {
        if (dom.sink_cells.empty()) continue;
        double freq_ghz = 1.0 / (dom.period_ps * 1e-3); // period_ps → GHz
        double vdd = 0.8;
        // Rebuild tree briefly for power estimate; reuse tree_ from last build
        total_power += estimate_clock_power(
            tree_.empty() ? 0 : static_cast<int>(tree_.size()) - 1, freq_ghz, vdd);
    }
    result.total_clock_power_uw = total_power;

    generate_multi_cts_report(result, cfg);

    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.report += "\nTotal CTS time: " + std::to_string(elapsed) + " ms\n";

    return result;
}

// ---------------------------------------------------------------------------
// Useful-skew CTS
// ---------------------------------------------------------------------------

CtsResult CtsEngine::build_useful_skew_tree(const ClockDomain& domain,
                                             const std::vector<double>& sink_slack_ps) {
    auto t0 = std::chrono::high_resolution_clock::now();

    // Step 1 — build a normal zero-skew DME tree
    CtsResult result = build_clock_tree(domain.source, domain.sink_cells);

    if (domain.sink_cells.empty() || sink_slack_ps.size() != domain.sink_cells.size())
        return result;

    init_buffer_library();

    // Step 2 — compute desired per-sink delay adjustments from slacks
    // Negative slack → needs more insertion delay (later arrival helps setup)
    // Positive slack → can tolerate less insertion delay
    double mean_slack = 0;
    for (double s : sink_slack_ps) mean_slack += s;
    mean_slack /= static_cast<double>(sink_slack_ps.size());

    std::vector<double> target_delays(sink_slack_ps.size());
    for (size_t i = 0; i < sink_slack_ps.size(); ++i)
        target_delays[i] = -(sink_slack_ps[i] - mean_slack); // invert: negative slack → positive delay

    // Build sink-node id list (first N entries in tree_ are the sinks)
    std::vector<int> sink_node_ids;
    for (size_t i = 0; i < domain.sink_cells.size() && i < tree_.size(); ++i)
        sink_node_ids.push_back(static_cast<int>(i));

    // Step 3 — apply useful-skew delay adjustments
    int root = tree_.empty() ? -1 : static_cast<int>(tree_.size()) - 1;
    if (root >= 0)
        apply_useful_skew(root, target_delays, sink_node_ids);

    // Step 4 — insert delay buffers at sinks that need extra delay
    int extra_bufs = 0;
    for (size_t i = 0; i < target_delays.size(); ++i) {
        if (target_delays[i] <= 0) continue;

        // Number of delay-buffer stages needed (each CLKBUF_X1 adds ~15 ps)
        const double buf_delay_ps = buf_lib_.front().intrinsic_delay;
        int stages = std::max(1, static_cast<int>(std::ceil(target_delays[i] / buf_delay_ps)));

        const auto& buf = buf_lib_.front(); // smallest buffer for delay padding
        for (int s = 0; s < stages; ++s) {
            std::string bname = "uskew_" + domain.name + "_s" +
                                std::to_string(i) + "_" + std::to_string(s);
            pd_.add_cell(bname, buf.name, buf.area, pd_.row_height);
            if (i < tree_.size()) {
                pd_.cells.back().position = tree_[i].position;
                pd_.cells.back().placed = true;
            }
            extra_bufs++;
        }
    }

    // Step 5 — recompute skew metrics
    double max_delay = 0, min_delay = 1e18;
    for (size_t i = 0; i < domain.sink_cells.size() && i < tree_.size(); ++i) {
        double base_d = tree_[i].position.dist(domain.source) * 0.001;
        double adj = (i < target_delays.size()) ? std::max(0.0, target_delays[i]) : 0.0;
        double d = base_d + adj;
        max_delay = std::max(max_delay, d);
        min_delay = std::min(min_delay, d);
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    result.skew = max_delay - min_delay;
    result.max_latency_ps = max_delay;
    result.min_latency_ps = min_delay;
    result.buffers_inserted += extra_bufs;
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = "Useful-skew tree (" + domain.name + ") — skew: " +
                     std::to_string(result.skew) + "ps, " +
                     std::to_string(result.buffers_inserted) + " buffers (+" +
                     std::to_string(extra_bufs) + " delay)";
    return result;
}

// ---------------------------------------------------------------------------
// Power-aware CTS
// ---------------------------------------------------------------------------

CtsResult CtsEngine::build_power_aware_tree(const ClockDomain& domain) {
    auto t0 = std::chrono::high_resolution_clock::now();

    // Step 1 — build DME topology
    CtsResult result = build_clock_tree(domain.source, domain.sink_cells);

    if (domain.sink_cells.empty()) return result;

    init_buffer_library();

    // Step 2 — replace buffers with minimum-drive-strength variants that still
    //          meet the transition-time constraint (~100 ps).
    int replaced = 0;
    for (size_t i = domain.sink_cells.size(); i < tree_.size(); ++i) {
        auto& n = tree_[i];
        if (n.left < 0) continue; // leaf

        double subtree_cap = compute_subtree_cap(static_cast<int>(i));
        // Pick weakest buffer whose transition ≤ max_transition
        // Transition ≈ 0.69 * R_buf * C_load  (first-order)
        const ClkBufType* chosen = &buf_lib_.back();
        for (auto& buf : buf_lib_) {
            double transition = 0.69 * buf.drive_strength * subtree_cap * 1e-3; // ps
            if (transition <= 100.0) { // meets transition constraint
                chosen = &buf;
                break; // buf_lib_ sorted weakest-first → first hit is minimum drive
            }
        }

        // Update cell in physical design to the smaller buffer
        std::string bname = "pabuf_" + domain.name + "_" + std::to_string(i);
        pd_.add_cell(bname, chosen->name, chosen->area, pd_.row_height);
        pd_.cells.back().position = n.position;
        pd_.cells.back().placed = true;
        replaced++;
    }

    // Step 3 — estimate power for this domain's tree
    double freq_ghz = 1.0 / (domain.period_ps * 1e-3);
    double vdd = 0.8;
    int root = tree_.empty() ? 0 : static_cast<int>(tree_.size()) - 1;
    double power_uw = estimate_clock_power(root, freq_ghz, vdd);

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = "Power-aware tree (" + domain.name + ") — skew: " +
                     std::to_string(result.skew) + "ps, " +
                     std::to_string(result.buffers_inserted) + " bufs (" +
                     std::to_string(replaced) + " downsized), power: " +
                     std::to_string(power_uw) + " uW";
    return result;
}

// ---------------------------------------------------------------------------
// partition_sinks — validate/log domain sink partitioning
// ---------------------------------------------------------------------------

void CtsEngine::partition_sinks(const CtsConfig& cfg) {
    // Verify that sink cell IDs are within range and log domain stats
    for (const auto& dom : cfg.domains) {
        for (int cid : dom.sink_cells) {
            if (cid < 0 || cid >= static_cast<int>(pd_.cells.size())) {
                std::cerr << "[CTS] WARNING: domain " << dom.name
                          << " references invalid cell ID " << cid << "\n";
            }
        }
    }
}

// ---------------------------------------------------------------------------
// apply_useful_skew — adjust tree node delays toward targets
// ---------------------------------------------------------------------------

void CtsEngine::apply_useful_skew(int root,
                                   const std::vector<double>& target_delays,
                                   const std::vector<int>& sinks) {
    if (root < 0 || root >= static_cast<int>(tree_.size())) return;

    // Walk from each sink toward the root, accumulating delay adjustments
    // into the TreeNode::delay field so downstream analysis reflects changes.
    for (size_t si = 0; si < sinks.size() && si < target_delays.size(); ++si) {
        int nid = sinks[si];
        if (nid < 0 || nid >= static_cast<int>(tree_.size())) continue;
        tree_[nid].delay += target_delays[si];
    }

    // Propagate adjusted delays upward through the merge tree
    std::function<double(int)> propagate = [&](int node) -> double {
        auto& n = tree_[node];
        if (n.left < 0 && n.right < 0) return n.delay; // leaf
        double ld = (n.left >= 0) ? propagate(n.left) : 0;
        double rd = (n.right >= 0) ? propagate(n.right) : 0;
        n.delay = std::max(ld, rd) +
                  ((n.left >= 0) ? n.position.dist(tree_[n.left].position) * 0.001 : 0);
        return n.delay;
    };
    propagate(root);
}

// ---------------------------------------------------------------------------
// estimate_clock_power
// ---------------------------------------------------------------------------

double CtsEngine::estimate_clock_power(int root, double freq_ghz, double vdd) {
    if (tree_.empty() || root < 0 || root >= static_cast<int>(tree_.size()))
        return 0;

    double total_cap_pf = 0; // picofarads

    std::function<void(int)> walk = [&](int node) {
        if (node < 0 || node >= static_cast<int>(tree_.size())) return;
        auto& n = tree_[node];
        if (n.left >= 0) {
            double wl = n.position.dist(tree_[n.left].position);
            total_cap_pf += wl * 0.00005; // 0.05 fF/um = 0.00005 pF/um
            walk(n.left);
        }
        if (n.right >= 0) {
            double wl = n.position.dist(tree_[n.right].position);
            total_cap_pf += wl * 0.00005;
            walk(n.right);
        }
        if (n.cell_id >= 0) {
            total_cap_pf += 0.005; // 5 fF FF clock-pin cap = 0.005 pF
        }
    };
    walk(root);

    // P = 0.5 * C * V^2 * f  (dynamic switching power)
    // C in pF, V in V, f in GHz → result in mW; multiply by 1e3 for uW
    return 0.5 * total_cap_pf * vdd * vdd * freq_ghz * 1e3; // uW
}

// ---------------------------------------------------------------------------
// compute_inter_domain_skew
// ---------------------------------------------------------------------------

double CtsEngine::compute_inter_domain_skew(
    const std::vector<CtsResult>& results,
    const std::vector<ClockGroup>& groups,
    const CtsConfig& cfg) {

    // Build name → index map
    std::unordered_map<std::string, size_t> name_idx;
    for (size_t i = 0; i < cfg.domains.size(); ++i)
        name_idx[cfg.domains[i].name] = i;

    double worst_skew = 0;

    for (const auto& grp : groups) {
        if (grp.type != ClockGroup::SYNCHRONOUS) continue;

        double grp_max = -1e18, grp_min = 1e18;
        for (const auto& cname : grp.clock_names) {
            auto it = name_idx.find(cname);
            if (it == name_idx.end()) continue;
            size_t idx = it->second;
            if (idx >= results.size()) continue;
            grp_max = std::max(grp_max, results[idx].max_latency_ps);
            grp_min = std::min(grp_min, results[idx].min_latency_ps);
        }
        if (grp_max > grp_min)
            worst_skew = std::max(worst_skew, grp_max - grp_min);
    }
    return worst_skew;
}

// ---------------------------------------------------------------------------
// generate_multi_cts_report
// ---------------------------------------------------------------------------

void CtsEngine::generate_multi_cts_report(MultiCtsResult& result,
                                           const CtsConfig& cfg) {
    std::ostringstream os;

    os << "\n";
    os << std::string(55, '=') << "\n";
    os << " SiliconForge Multi-Clock CTS Report\n";
    os << std::string(55, '=') << "\n";
    os << "Domains: " << cfg.domains.size() << "\n";

    for (size_t i = 0; i < cfg.domains.size(); ++i) {
        const auto& dom = cfg.domains[i];
        const auto& dr  = (i < result.domain_results.size())
                              ? result.domain_results[i]
                              : CtsResult{};
        os << "  " << std::setw(12) << std::left << dom.name
           << std::right
           << std::setw(5) << dom.sink_cells.size() << " sinks"
           << ", skew=" << std::fixed << std::setprecision(1) << dr.skew << "ps"
           << ", " << dr.buffers_inserted << " buffers"
           << ", latency=" << dr.max_latency_ps << "ps"
           << "\n";
    }

    // Clock groups
    if (!cfg.groups.empty()) {
        os << "\nClock Groups:\n";
        for (const auto& grp : cfg.groups) {
            const char* rel = (grp.type == ClockGroup::SYNCHRONOUS)  ? "SYNC"
                            : (grp.type == ClockGroup::ASYNCHRONOUS) ? "ASYNC"
                                                                     : "EXCL";
            os << "  " << rel << "(";
            for (size_t j = 0; j < grp.clock_names.size(); ++j) {
                if (j) os << ", ";
                os << grp.clock_names[j];
            }
            os << ")";
            if (grp.type == ClockGroup::SYNCHRONOUS)
                os << ": inter-domain skew = " << std::fixed << std::setprecision(1)
                   << result.inter_domain_skew << "ps";
            else
                os << ": N/A";
            os << "\n";
        }
    }

    int total_sinks = 0;
    for (const auto& d : cfg.domains) total_sinks += static_cast<int>(d.sink_cells.size());

    os << "\nTotal: " << total_sinks << " sinks, "
       << result.total_buffers << " buffers, wirelength="
       << std::fixed << std::setprecision(0) << result.total_wirelength << " um\n";
    os << "Clock Power: " << std::fixed << std::setprecision(1)
       << result.total_clock_power_uw << " uW";

    if (result.total_clock_power_uw >= 1000.0)
        os << " (" << std::setprecision(2) << result.total_clock_power_uw / 1000.0 << " mW)";
    os << "\n";

    result.report = os.str();
}

// ===========================================================================
//  Phase 43: H-tree Topology
// ===========================================================================

int CtsEngine::htree_branch(const Point& center, double span_x, double span_y,
                             int level, int max_levels, bool horizontal,
                             const std::vector<int>& nearby_sinks) {
    TreeNode node;
    node.position = center;
    int id = (int)tree_.size();
    tree_.push_back(node);

    if (level >= max_levels || nearby_sinks.size() <= 1) {
        // Leaf of H-tree — connect to nearest sink if any
        if (!nearby_sinks.empty()) {
            tree_[id].cell_id = nearby_sinks[0]; // link to sink
        }
        return id;
    }

    // Split sinks into two halves based on branching direction
    std::vector<int> left_sinks, right_sinks;
    for (int sid : nearby_sinks) {
        if (horizontal) {
            if (tree_[sid].position.x < center.x) left_sinks.push_back(sid);
            else right_sinks.push_back(sid);
        } else {
            if (tree_[sid].position.y < center.y) left_sinks.push_back(sid);
            else right_sinks.push_back(sid);
        }
    }
    // Ensure at least one sink per branch if we have sinks
    if (left_sinks.empty() && !right_sinks.empty()) {
        left_sinks.push_back(right_sinks.back());
        right_sinks.pop_back();
    } else if (right_sinks.empty() && !left_sinks.empty()) {
        right_sinks.push_back(left_sinks.back());
        left_sinks.pop_back();
    }

    double half_x = span_x / 2;
    double half_y = span_y / 2;
    Point left_center, right_center;

    if (horizontal) {
        left_center  = Point(center.x - half_x, center.y);
        right_center = Point(center.x + half_x, center.y);
    } else {
        left_center  = Point(center.x, center.y - half_y);
        right_center = Point(center.x, center.y + half_y);
    }

    int left_id  = htree_branch(left_center, half_x, half_y, level + 1,
                                 max_levels, !horizontal, left_sinks);
    int right_id = htree_branch(right_center, half_x, half_y, level + 1,
                                 max_levels, !horizontal, right_sinks);

    tree_[id].left = left_id;
    tree_[id].right = right_id;
    return id;
}

CtsResult CtsEngine::build_htree(const Point& source,
                                  const std::vector<int>& sink_cells,
                                  const HtreeConfig& cfg) {
    auto t0 = std::chrono::high_resolution_clock::now();
    CtsResult result;

    if (sink_cells.empty()) {
        result.message = "No sinks for H-tree";
        return result;
    }

    tree_.clear();
    clock_wire_count_ = 0;

    // Create leaf nodes for sinks
    std::vector<int> sink_nodes;
    for (int cid : sink_cells) {
        TreeNode node;
        if (cid >= 0 && cid < (int)pd_.cells.size()) {
            node.position = Point(
                pd_.cells[cid].position.x + pd_.cells[cid].width / 2,
                pd_.cells[cid].position.y + pd_.cells[cid].height / 2
            );
        }
        node.cell_id = cid;
        int id = (int)tree_.size();
        tree_.push_back(node);
        sink_nodes.push_back(id);
    }

    // Compute bounding box of sinks
    double xmin = 1e18, xmax = -1e18, ymin = 1e18, ymax = -1e18;
    for (int sid : sink_nodes) {
        xmin = std::min(xmin, tree_[sid].position.x);
        xmax = std::max(xmax, tree_[sid].position.x);
        ymin = std::min(ymin, tree_[sid].position.y);
        ymax = std::max(ymax, tree_[sid].position.y);
    }

    double span_x = (xmax - xmin) / 2;
    double span_y = (ymax - ymin) / 2;
    Point center((xmin + xmax) / 2, (ymin + ymax) / 2);

    // Build H-tree recursively with alternating X/Y branching
    int root = htree_branch(center, span_x, span_y, 0, cfg.levels,
                            cfg.alternate_xy, sink_nodes);

    // Place root at source
    tree_[root].position = source;

    // Create wire segments
    std::function<void(int)> create_wires = [&](int node) {
        auto& n = tree_[node];
        if (n.left >= 0) {
            pd_.wires.push_back({0, n.position, tree_[n.left].position, cfg.wire_width_um});
            clock_wire_count_++;
            create_wires(n.left);
        }
        if (n.right >= 0) {
            pd_.wires.push_back({0, n.position, tree_[n.right].position, cfg.wire_width_um});
            clock_wire_count_++;
            create_wires(n.right);
        }
    };
    create_wires(root);

    // Insert buffers at branch points
    init_buffer_library();
    int buffers = 0;
    for (size_t i = sink_nodes.size(); i < tree_.size(); ++i) {
        if ((int)i != root && tree_[i].left >= 0) {
            double subtree_cap = compute_subtree_cap((int)i);
            auto buf = select_buffer(subtree_cap, 0);
            pd_.add_cell("htree_buf_" + std::to_string(i), buf.name, buf.area, pd_.row_height);
            pd_.cells.back().position = tree_[i].position;
            pd_.cells.back().placed = true;
            buffers++;
        }
    }

    // Compute metrics
    double total_wl = 0, max_delay = 0, min_delay = 1e18;
    for (auto sid : sink_nodes) {
        double d = tree_[sid].position.dist(source);
        total_wl += d;
        double delay = d * 0.001;
        max_delay = std::max(max_delay, delay);
        min_delay = std::min(min_delay, delay);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.skew = max_delay - min_delay;
    result.wirelength = total_wl;
    result.buffers_inserted = buffers;
    result.max_latency_ps = max_delay;
    result.min_latency_ps = min_delay;
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = "H-tree (" + std::to_string(cfg.levels) + " levels) — skew: " +
                     std::to_string(result.skew) + "ps, " +
                     std::to_string(buffers) + " buffers";
    return result;
}

// ===========================================================================
//  Phase 43: Clock Shielding
// ===========================================================================

int CtsEngine::apply_clock_shielding(const ClkShieldConfig& cfg) {
    int shields_added = 0;

    // Shield all clock wires in the tree
    size_t wire_start = pd_.wires.size();
    // We shield the wires that were added during CTS (tracked by clock_wire_count_)
    // For safety, iterate all wires and shield the recent clock ones
    if (clock_wire_count_ <= 0) return 0;

    size_t first_clock_wire = pd_.wires.size() - std::min((size_t)clock_wire_count_, pd_.wires.size());

    // Collect shield wires separately to avoid invalidating references during iteration
    std::vector<WireSegment> shield_wires;
    for (size_t wi = first_clock_wire; wi < wire_start; ++wi) {
        auto& w = pd_.wires[wi];
        // Add VDD shield wire (parallel, offset by spacing+width)
        if (cfg.shield_vdd) {
            double offset = cfg.shield_spacing_um + cfg.shield_width_um / 2;
            Point s0(w.start.x, w.start.y + offset);
            Point s1(w.end.x, w.end.y + offset);
            shield_wires.push_back({0, s0, s1, cfg.shield_width_um});
            shields_added++;
        }
        // Add GND shield wire (opposite side)
        if (cfg.shield_gnd) {
            double offset = -(cfg.shield_spacing_um + cfg.shield_width_um / 2);
            Point s0(w.start.x, w.start.y + offset);
            Point s1(w.end.x, w.end.y + offset);
            shield_wires.push_back({0, s0, s1, cfg.shield_width_um});
            shields_added++;
        }
    }
    for (auto& sw : shield_wires) pd_.wires.push_back(std::move(sw));

    // Mark tree nodes as shielded
    for (auto& n : tree_) n.shielded = true;

    return shields_added;
}

double CtsEngine::coupling_reduction_factor(const ClkShieldConfig& cfg) const {
    // Shielding reduces coupling capacitance significantly
    if (cfg.coupling_cap_ff_per_um <= 0) return 1.0;
    return cfg.shielded_cap_ff_per_um / cfg.coupling_cap_ff_per_um;
}

// ===========================================================================
//  Phase 43: Slew-Driven Optimization
// ===========================================================================

double CtsEngine::propagate_slew(int node, const SlewConfig& cfg) {
    if (node < 0 || node >= (int)tree_.size()) return 0;
    auto& n = tree_[node];

    if (n.left < 0 && n.right < 0) {
        // Leaf: slew from buffer output (assume ideal)
        n.slew = 20.0; // 20ps ideal buffer output slew
        return n.slew;
    }

    double worst_child_slew = 0;
    if (n.left >= 0) {
        double child_slew = propagate_slew(n.left, cfg);
        double wl = n.position.dist(tree_[n.left].position);
        worst_child_slew = std::max(worst_child_slew,
                                     child_slew + wl * cfg.slew_per_wire_ps_per_um);
    }
    if (n.right >= 0) {
        double child_slew = propagate_slew(n.right, cfg);
        double wl = n.position.dist(tree_[n.right].position);
        worst_child_slew = std::max(worst_child_slew,
                                     child_slew + wl * cfg.slew_per_wire_ps_per_um);
    }

    // Add load-dependent slew degradation
    double subtree_cap = compute_subtree_cap(node);
    n.slew = worst_child_slew + subtree_cap * cfg.slew_per_load_ps_per_ff;
    return n.slew;
}

double CtsEngine::compute_slew_at_node(int node, const SlewConfig& cfg) const {
    if (node < 0 || node >= (int)tree_.size()) return 0;
    return tree_[node].slew;
}

CtsResult CtsEngine::build_slew_driven_tree(const Point& source,
                                              const std::vector<int>& sink_cells,
                                              const SlewConfig& cfg) {
    auto t0 = std::chrono::high_resolution_clock::now();

    // Step 1: Build base DME tree
    CtsResult result = build_clock_tree(source, sink_cells);
    if (sink_cells.empty()) return result;

    init_buffer_library();

    // Step 2: Bottom-up slew propagation
    int root = tree_.empty() ? -1 : (int)tree_.size() - 1;
    if (root >= 0) propagate_slew(root, cfg);

    // Step 3: Insert additional buffers where slew exceeds max
    int extra_bufs = 0;
    for (size_t i = 0; i < tree_.size(); ++i) {
        if (tree_[i].slew > cfg.max_slew_ps && tree_[i].left >= 0) {
            // Need stronger buffer or additional buffer stage
            double load = compute_subtree_cap((int)i);
            // Try strongest buffer first
            auto buf = buf_lib_.back();
            double new_transition = 0.69 * buf.drive_strength * load * 1e-3;
            if (new_transition < cfg.max_slew_ps) {
                pd_.add_cell("slew_buf_" + std::to_string(i), buf.name,
                             buf.area, pd_.row_height);
                pd_.cells.back().position = tree_[i].position;
                pd_.cells.back().placed = true;
                tree_[i].slew = new_transition;
                extra_bufs++;
            }
        }
    }

    // Step 4: Re-propagate slew after buffer insertion
    if (root >= 0) propagate_slew(root, cfg);

    // Compute worst slew
    double worst_slew = 0;
    for (const auto& n : tree_) worst_slew = std::max(worst_slew, n.slew);

    auto t1 = std::chrono::high_resolution_clock::now();
    result.buffers_inserted += extra_bufs;
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = "Slew-driven tree — skew: " + std::to_string(result.skew) +
                     "ps, worst_slew: " + std::to_string(worst_slew) +
                     "ps, " + std::to_string(result.buffers_inserted) + " buffers";
    return result;
}

// ===========================================================================
//  Phase 43: Incremental ECO CTS
// ===========================================================================

int CtsEngine::find_nearest_tree_node(const Point& p, bool internal_only) const {
    int best = -1;
    double best_dist = 1e18;
    for (int i = 0; i < (int)tree_.size(); i++) {
        if (internal_only && tree_[i].cell_id >= 0) continue; // skip leaves
        double d = tree_[i].position.dist(p);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
    }
    return best;
}

EcoCtsResult CtsEngine::apply_eco(const EcoCtsRequest& req, const CtsConfig& cts_cfg) {
    EcoCtsResult result;

    // Find the domain
    const ClockDomain* domain = nullptr;
    for (const auto& d : cts_cfg.domains) {
        if (d.name == req.domain_name) { domain = &d; break; }
    }
    if (!domain) {
        result.message = "ECO: domain '" + req.domain_name + "' not found";
        return result;
    }

    init_buffer_library();

    // Record skew before ECO
    double max_d = 0, min_d = 1e18;
    for (const auto& n : tree_) {
        if (n.cell_id >= 0) {
            double d = n.position.dist(domain->source) * 0.001;
            max_d = std::max(max_d, d);
            min_d = std::min(min_d, d);
        }
    }
    result.skew_before = (max_d > min_d) ? max_d - min_d : 0;

    // Remove sinks
    for (int cid : req.remove_sinks) {
        for (auto& n : tree_) {
            if (n.cell_id == cid) {
                n.cell_id = -2; // mark as removed
                result.sinks_removed++;
                break;
            }
        }
    }

    // Add sinks — attach each to nearest internal node
    for (int cid : req.add_sinks) {
        if (cid < 0 || cid >= (int)pd_.cells.size()) continue;

        TreeNode leaf;
        leaf.position = Point(
            pd_.cells[cid].position.x + pd_.cells[cid].width / 2,
            pd_.cells[cid].position.y + pd_.cells[cid].height / 2
        );
        leaf.cell_id = cid;
        int leaf_id = (int)tree_.size();
        tree_.push_back(leaf);

        // Find nearest internal/branch node
        int nearest = find_nearest_tree_node(leaf.position, true);
        if (nearest >= 0) {
            // Create a new Steiner point to splice in
            TreeNode steiner;
            steiner.position = Point(
                (tree_[nearest].position.x + leaf.position.x) / 2,
                (tree_[nearest].position.y + leaf.position.y) / 2
            );
            steiner.left = leaf_id;

            // Steal one child from nearest
            if (tree_[nearest].right >= 0) {
                steiner.right = tree_[nearest].right;
                tree_[nearest].right = (int)tree_.size();
            } else if (tree_[nearest].left >= 0) {
                steiner.right = tree_[nearest].left;
                tree_[nearest].left = (int)tree_.size();
            } else {
                tree_[nearest].left = leaf_id;
                result.sinks_added++;
                continue;
            }
            tree_.push_back(steiner);

            // Insert buffer at new splice point
            auto buf = select_buffer(5.0, leaf.position.dist(tree_[nearest].position));
            pd_.add_cell("eco_buf_" + std::to_string(leaf_id), buf.name,
                         buf.area, pd_.row_height);
            pd_.cells.back().position = steiner.position;
            pd_.cells.back().placed = true;
            result.buffers_added++;

            // Add wire
            pd_.wires.push_back({0, steiner.position, leaf.position, 0.14});
        }
        result.sinks_added++;
    }

    // Recompute skew after ECO
    max_d = 0; min_d = 1e18;
    for (const auto& n : tree_) {
        if (n.cell_id >= 0) { // skip removed (-2) and internal (-1)
            double d = n.position.dist(domain->source) * 0.001;
            max_d = std::max(max_d, d);
            min_d = std::min(min_d, d);
        }
    }
    result.skew_after = (max_d > min_d) ? max_d - min_d : 0;

    result.message = "ECO: +" + std::to_string(result.sinks_added) +
                     " -" + std::to_string(result.sinks_removed) +
                     " sinks, skew " + std::to_string(result.skew_before) +
                     " → " + std::to_string(result.skew_after) + "ps";
    return result;
}

// ===========================================================================
//  Phase 43: Derived/Generated Clocks
// ===========================================================================

DerivedClockResult CtsEngine::build_derived_clock(const ClockDomain& derived,
                                                    const ClockDomain& parent) {
    DerivedClockResult result;
    result.name = derived.name;

    // Compute effective period from division ratio
    result.effective_period_ps = parent.period_ps * derived.divide_by;

    // Cumulative jitter: parent uncertainty + derived uncertainty
    // Jitter adds in RSS (root-sum-square)
    result.cumulative_uncertainty_ps = std::sqrt(
        parent.uncertainty_ps * parent.uncertainty_ps +
        derived.uncertainty_ps * derived.uncertainty_ps
    );

    // Phase shift for divided clocks (half-cycle of parent per division)
    if (derived.divide_by > 1) {
        result.phase_shift_ps = parent.period_ps / 2.0;
    }

    // Insert clock divider cell at parent source location
    if (derived.divide_by > 1) {
        std::string div_name = "clkdiv_" + derived.name + "_div" +
                               std::to_string(derived.divide_by);
        pd_.add_cell(div_name, "CLKDIV", 4.0, pd_.row_height);
        pd_.cells.back().position = parent.source;
        pd_.cells.back().placed = true;
        result.divider_cells_inserted = 1;
    }

    // Build CTS for the derived domain's sinks
    result.tree_result = build_clock_tree(derived.source, derived.sink_cells);
    result.tree_result.message = "Derived clock '" + derived.name +
                                  "' (÷" + std::to_string(derived.divide_by) +
                                  " from " + parent.name + "): " +
                                  result.tree_result.message;

    return result;
}

// ---------------------------------------------------------------------------
// Phase C: Useful Skew Optimization
// ---------------------------------------------------------------------------

CtsEngine::UsefulSkewResult CtsEngine::apply_useful_skew_opt(
    const std::vector<int>& sink_cells,
    const std::vector<double>& sink_slack_ps,
    const CtsConfig& cfg)
{
    UsefulSkewResult result;

    if (sink_cells.empty() || sink_slack_ps.empty()) {
        result.message = "No sinks for useful skew";
        return result;
    }

    double max_useful_skew = cfg.useful_skew_max_ps;
    double total_improvement = 0;

    // Identify setup-critical paths (negative slack)
    for (size_t i = 0; i < sink_cells.size() && i < sink_slack_ps.size(); ++i) {
        double slack = sink_slack_ps[i];
        if (slack >= 0) continue; // not critical

        // For a critical capture FF, we can advance clock arrival
        // (reduce insertion delay) to improve setup slack.
        // Beneficial skew = min(|slack|, max_useful_skew)
        double beneficial_skew = std::min(std::abs(slack), max_useful_skew);

        // Find the tree node for this sink
        int sink_node = -1;
        for (size_t n = 0; n < tree_.size(); ++n) {
            if (tree_[n].cell_id == sink_cells[i]) {
                sink_node = static_cast<int>(n);
                break;
            }
        }

        if (sink_node < 0) continue;

        // Adjust the delay at this sink node (reduce = advance clock arrival)
        tree_[sink_node].delay -= beneficial_skew * 0.5;

        // Track the applied skew (buffer delay adjustment)
        // In practice: remove a buffer level or downsize buffers on this path
        result.paths_improved++;
        total_improvement += beneficial_skew * 0.5;
        result.max_applied_skew = std::max(result.max_applied_skew, beneficial_skew);
    }

    // Also look for opportunities to borrow slack from non-critical paths
    // by delaying their clock arrival (inserting extra buffer delay)
    for (size_t i = 0; i < sink_cells.size() && i < sink_slack_ps.size(); ++i) {
        double slack = sink_slack_ps[i];
        if (slack <= 0) continue; // skip critical paths

        // For non-critical launch FFs, delay clock to lend slack to capture
        double lendable = std::min(slack * 0.3, max_useful_skew);
        if (lendable < 1.0) continue;

        int sink_node = -1;
        for (size_t n = 0; n < tree_.size(); ++n) {
            if (tree_[n].cell_id == sink_cells[i]) {
                sink_node = static_cast<int>(n);
                break;
            }
        }
        if (sink_node < 0) continue;

        tree_[sink_node].delay += lendable * 0.5;
        total_improvement += lendable * 0.2; // partial benefit from slack lending
    }

    result.slack_improvement = total_improvement;
    result.message = std::to_string(result.paths_improved) + " paths improved, " +
                     std::to_string(static_cast<int>(result.slack_improvement)) +
                     "ps total slack improvement, max skew=" +
                     std::to_string(static_cast<int>(result.max_applied_skew)) + "ps";
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Advanced CTS Implementation — Buffer Library, RSMT, Multi-source, Clock Mesh,
// OCV-aware Balancing, Power-aware Optimization
// ═══════════════════════════════════════════════════════════════════════════════

// ── set_config ──────────────────────────────────────────────────────────────

void CtsEngine::set_config(const CtsConfig& cfg) {
    cts_config_ = cfg;
}

// ── Default buffer library (3 sizes × 3 Vt flavors) ────────────────────────

static std::vector<CtsBuffer> default_buffer_library() {
    std::vector<CtsBuffer> lib;
    // Small buffers
    lib.push_back({"CLKBUF_X1_SVT", 0.035, 0.005, 1.0, 2.0, 0.010, CtsVtType::SVT});
    lib.push_back({"CLKBUF_X1_LVT", 0.025, 0.006, 1.2, 2.2, 0.015, CtsVtType::LVT});
    lib.push_back({"CLKBUF_X1_HVT", 0.050, 0.004, 0.8, 1.8, 0.006, CtsVtType::HVT});
    // Medium buffers
    lib.push_back({"CLKBUF_X4_SVT", 0.030, 0.020, 4.0, 6.0, 0.035, CtsVtType::SVT});
    lib.push_back({"CLKBUF_X4_LVT", 0.020, 0.024, 4.8, 6.6, 0.050, CtsVtType::LVT});
    lib.push_back({"CLKBUF_X4_HVT", 0.045, 0.016, 3.2, 5.4, 0.020, CtsVtType::HVT});
    // Large buffers
    lib.push_back({"CLKBUF_X8_SVT", 0.025, 0.040, 8.0, 10.0, 0.065, CtsVtType::SVT});
    lib.push_back({"CLKBUF_X8_LVT", 0.018, 0.048, 9.6, 11.0, 0.090, CtsVtType::LVT});
    lib.push_back({"CLKBUF_X8_HVT", 0.040, 0.032, 6.4, 9.0, 0.038, CtsVtType::HVT});
    return lib;
}

// ── select_cts_buffer ───────────────────────────────────────────────────────

CtsBuffer CtsEngine::select_cts_buffer(double load_cap, double max_slew,
                                        bool critical_path) {
    auto& lib = cts_config_.buffer_library;
    if (lib.empty()) {
        lib = default_buffer_library();
    }

    // Filter buffers that can drive the load within slew constraint
    // Simplified model: slew ≈ delay × (1 + load_cap / drive)
    std::vector<std::pair<double, int>> candidates; // (cost, index)
    for (int i = 0; i < static_cast<int>(lib.size()); ++i) {
        const auto& buf = lib[i];
        double effective_slew = buf.delay * (1.0 + load_cap / std::max(buf.drive * 0.01, 1e-9));
        if (effective_slew > max_slew && max_slew > 0) continue;

        // Cost: for critical paths, minimize delay; otherwise minimize power
        double cost;
        if (critical_path) {
            cost = buf.delay * 1000.0; // heavily weight delay
            if (buf.vt == CtsVtType::LVT) cost *= 0.7;  // prefer LVT
            if (buf.vt == CtsVtType::HVT) cost *= 1.5;   // penalize HVT
        } else {
            cost = buf.power * 1000.0; // heavily weight power
            if (buf.vt == CtsVtType::HVT) cost *= 0.6;  // prefer HVT
            if (buf.vt == CtsVtType::LVT) cost *= 1.5;   // penalize LVT
        }
        candidates.emplace_back(cost, i);
    }

    if (candidates.empty()) {
        // Fallback: return the medium SVT buffer
        for (int i = 0; i < static_cast<int>(lib.size()); ++i) {
            if (lib[i].vt == CtsVtType::SVT && lib[i].drive >= 3.0) return lib[i];
        }
        return lib[0]; // absolute fallback
    }

    std::sort(candidates.begin(), candidates.end());
    return lib[candidates.front().second];
}

// ── build_hanan_grid ────────────────────────────────────────────────────────

std::vector<std::pair<double,double>> CtsEngine::build_hanan_grid(
        const std::vector<std::pair<double,double>>& points) {
    // Collect unique X and Y coordinates
    std::set<double> xs, ys;
    for (auto& p : points) {
        xs.insert(p.first);
        ys.insert(p.second);
    }

    // Hanan grid = all intersections of these horizontal/vertical lines
    std::set<std::pair<double,double>> point_set(points.begin(), points.end());
    std::vector<std::pair<double,double>> grid;
    for (double x : xs) {
        for (double y : ys) {
            auto candidate = std::make_pair(x, y);
            if (point_set.find(candidate) == point_set.end()) {
                grid.push_back(candidate);
            }
        }
    }
    return grid;
}

// ── steiner_from_hanan ──────────────────────────────────────────────────────
// Greedy RSMT: start with MST of sinks, iteratively add Steiner points that
// reduce total wirelength.

SteinerTree CtsEngine::steiner_from_hanan(
        const std::vector<std::pair<double,double>>& sinks,
        const std::vector<std::pair<double,double>>& hanan) {
    SteinerTree st;
    if (sinks.empty()) return st;

    int n = static_cast<int>(sinks.size());

    // Initialize nodes for sinks
    for (int i = 0; i < n; ++i) {
        SteinerTree::SteinerNode node;
        node.x = sinks[i].first;
        node.y = sinks[i].second;
        node.is_steiner = false;
        node.sink_idx = i;
        st.nodes.push_back(node);
    }

    if (n == 1) {
        st.root = 0;
        st.total_wirelength = 0;
        return st;
    }

    // Build MST using Prim's algorithm on Manhattan distance
    std::vector<bool> in_mst(n, false);
    std::vector<double> min_cost(n, std::numeric_limits<double>::max());
    std::vector<int> parent(n, -1);
    min_cost[0] = 0;

    for (int iter = 0; iter < n; ++iter) {
        int u = -1;
        double best = std::numeric_limits<double>::max();
        for (int i = 0; i < n; ++i) {
            if (!in_mst[i] && min_cost[i] < best) {
                best = min_cost[i];
                u = i;
            }
        }
        if (u < 0) break;
        in_mst[u] = true;

        for (int v = 0; v < n; ++v) {
            if (in_mst[v]) continue;
            double d = std::abs(sinks[u].first - sinks[v].first) +
                       std::abs(sinks[u].second - sinks[v].second);
            if (d < min_cost[v]) {
                min_cost[v] = d;
                parent[v] = u;
            }
        }
    }

    // Record MST edges
    struct Edge { int u, v; double w; };
    std::vector<Edge> mst_edges;
    double mst_wl = 0;
    for (int i = 1; i < n; ++i) {
        if (parent[i] >= 0) {
            double d = std::abs(sinks[i].first - sinks[parent[i]].first) +
                       std::abs(sinks[i].second - sinks[parent[i]].second);
            mst_edges.push_back({parent[i], i, d});
            mst_wl += d;
        }
    }

    // Build parent-child in tree
    for (auto& e : mst_edges) {
        st.nodes[e.u].children.push_back(e.v);
    }

    // Greedy Steiner point insertion: try each Hanan point, add if it reduces WL
    double current_wl = mst_wl;
    int max_steiner_iters = std::min(static_cast<int>(hanan.size()), 50);

    for (int hi = 0; hi < max_steiner_iters; ++hi) {
        double hx = hanan[hi].first;
        double hy = hanan[hi].second;

        // Find the two closest existing nodes
        double d1 = std::numeric_limits<double>::max();
        double d2 = std::numeric_limits<double>::max();
        int n1 = -1, n2 = -1;
        for (int i = 0; i < static_cast<int>(st.nodes.size()); ++i) {
            double d = std::abs(st.nodes[i].x - hx) + std::abs(st.nodes[i].y - hy);
            if (d < d1) { d2 = d1; n2 = n1; d1 = d; n1 = i; }
            else if (d < d2) { d2 = d; n2 = i; }
        }
        if (n1 < 0 || n2 < 0) continue;

        // Direct distance between n1 and n2
        double direct = std::abs(st.nodes[n1].x - st.nodes[n2].x) +
                        std::abs(st.nodes[n1].y - st.nodes[n2].y);
        double via_steiner = d1 + d2;

        if (via_steiner < direct * 0.95) { // at least 5% improvement
            SteinerTree::SteinerNode sp;
            sp.x = hx;
            sp.y = hy;
            sp.is_steiner = true;
            sp.sink_idx = -1;
            int sp_idx = static_cast<int>(st.nodes.size());
            sp.children.push_back(n1);
            sp.children.push_back(n2);
            st.nodes.push_back(sp);

            // Remove n2 from n1's children if present
            auto& c = st.nodes[n1].children;
            c.erase(std::remove(c.begin(), c.end(), n2), c.end());

            current_wl = current_wl - direct + via_steiner;
        }
    }

    // Root is the last added node (or node 0 if no Steiner points added)
    st.root = static_cast<int>(st.nodes.size()) - 1;
    if (st.root < 0) st.root = 0;
    st.total_wirelength = std::max(current_wl, 0.0);

    return st;
}

// ── build_rsmt ──────────────────────────────────────────────────────────────

SteinerTree CtsEngine::build_rsmt(const std::vector<std::pair<double,double>>& sinks) {
    if (sinks.empty()) return SteinerTree{};
    if (sinks.size() == 1) {
        SteinerTree st;
        SteinerTree::SteinerNode node;
        node.x = sinks[0].first;
        node.y = sinks[0].second;
        node.is_steiner = false;
        node.sink_idx = 0;
        st.nodes.push_back(node);
        st.root = 0;
        st.total_wirelength = 0;
        return st;
    }

    auto hanan = build_hanan_grid(sinks);
    return steiner_from_hanan(sinks, hanan);
}

// ── size_buffers ────────────────────────────────────────────────────────────
// Walk the Steiner tree and assign buffer types based on subtree load.

void CtsEngine::size_buffers(SteinerTree& tree, double target_slew) {
    if (tree.nodes.empty()) return;

    // Bottom-up pass: compute capacitive load at each node
    std::vector<double> load(tree.nodes.size(), 0.0);
    const double wire_cap_per_unit = 0.001; // pF per unit Manhattan distance
    const double sink_cap = 0.005;          // pF per sink

    // Topological order (reverse DFS from root)
    std::vector<int> order;
    std::vector<bool> visited(tree.nodes.size(), false);
    std::function<void(int)> dfs = [&](int u) {
        if (u < 0 || u >= static_cast<int>(tree.nodes.size()) || visited[u]) return;
        visited[u] = true;
        for (int c : tree.nodes[u].children) dfs(c);
        order.push_back(u);
    };
    dfs(tree.root);

    for (int u : order) {
        auto& node = tree.nodes[u];
        if (node.children.empty()) {
            load[u] = sink_cap; // leaf load
        } else {
            load[u] = 0;
            for (int c : node.children) {
                double wire_len = std::abs(node.x - tree.nodes[c].x) +
                                  std::abs(node.y - tree.nodes[c].y);
                load[u] += load[c] + wire_len * wire_cap_per_unit;
            }
        }

        // If load exceeds max, insert buffer (tracked via buffer_delays_)
        if (load[u] > cts_config_.max_cap_load_pf && node.is_steiner) {
            bool is_critical = (node.children.size() <= 2);
            CtsBuffer buf = select_cts_buffer(load[u], target_slew, is_critical);
            buffer_delays_.push_back(buf.delay);
            buffer_vt_assignments_.push_back(buf.vt);
            load[u] = buf.input_cap; // buffer isolates downstream load
        }
    }
}

// ── kmeans_partition_sinks ──────────────────────────────────────────────────
// Simple k-means clustering of sink positions into k groups.

std::vector<std::vector<int>> CtsEngine::kmeans_partition_sinks(
        const std::vector<std::pair<double,double>>& sinks, int k) {
    int n = static_cast<int>(sinks.size());
    if (n == 0 || k <= 0) return {};
    k = std::min(k, n);

    // Initialize centroids: evenly spaced from sink list
    std::vector<double> cx(k), cy(k);
    for (int i = 0; i < k; ++i) {
        int idx = (i * n) / k;
        cx[i] = sinks[idx].first;
        cy[i] = sinks[idx].second;
    }

    std::vector<int> assignment(n, 0);
    constexpr int max_iter = 50;

    for (int iter = 0; iter < max_iter; ++iter) {
        bool changed = false;

        // Assign each sink to nearest centroid (Manhattan distance)
        for (int i = 0; i < n; ++i) {
            double best_dist = std::numeric_limits<double>::max();
            int best_k = 0;
            for (int j = 0; j < k; ++j) {
                double d = std::abs(sinks[i].first - cx[j]) +
                           std::abs(sinks[i].second - cy[j]);
                if (d < best_dist) { best_dist = d; best_k = j; }
            }
            if (assignment[i] != best_k) { assignment[i] = best_k; changed = true; }
        }
        if (!changed) break;

        // Update centroids
        std::vector<double> sum_x(k, 0), sum_y(k, 0);
        std::vector<int> count(k, 0);
        for (int i = 0; i < n; ++i) {
            sum_x[assignment[i]] += sinks[i].first;
            sum_y[assignment[i]] += sinks[i].second;
            count[assignment[i]]++;
        }
        for (int j = 0; j < k; ++j) {
            if (count[j] > 0) {
                cx[j] = sum_x[j] / count[j];
                cy[j] = sum_y[j] / count[j];
            }
        }
    }

    std::vector<std::vector<int>> clusters(k);
    for (int i = 0; i < n; ++i) {
        clusters[assignment[i]].push_back(i);
    }

    // Remove empty clusters
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
                       [](const std::vector<int>& c) { return c.empty(); }),
        clusters.end());

    return clusters;
}

// ── compute_ocv_delay ───────────────────────────────────────────────────────

double CtsEngine::compute_ocv_delay(double nominal_delay, double margin, bool early) {
    if (early) {
        return nominal_delay * (1.0 - margin); // early = faster
    } else {
        return nominal_delay * (1.0 + margin); // late = slower
    }
}

// ── synthesize_multi_source ─────────────────────────────────────────────────

MultiSourceResult CtsEngine::synthesize_multi_source(int max_sources) {
    MultiSourceResult result;

    // Collect sink positions from tree
    std::vector<std::pair<double,double>> sink_positions;
    std::vector<int> sink_indices;
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        if (tree_[i].cell_id >= 0) {
            sink_positions.emplace_back(tree_[i].position.x, tree_[i].position.y);
            sink_indices.push_back(i);
        }
    }

    if (sink_positions.empty()) {
        result.message = "No sinks available for multi-source CTS";
        return result;
    }

    // Determine number of sources based on sink count
    int num_sources = std::min(max_sources,
        std::max(1, static_cast<int>(sink_positions.size()) / 200));
    if (num_sources <= 1) {
        result.num_sources = 1;
        result.source_assignments.assign(sink_positions.size(), 0);
        // Source at centroid
        double sx = 0, sy = 0;
        for (auto& p : sink_positions) { sx += p.first; sy += p.second; }
        sx /= static_cast<double>(sink_positions.size());
        sy /= static_cast<double>(sink_positions.size());
        result.source_x = {sx};
        result.source_y = {sy};
        result.max_skew = 0;
        result.message = "Single source CTS (insufficient sinks for multi-source)";
        return result;
    }

    // K-means partitioning
    auto clusters = kmeans_partition_sinks(sink_positions, num_sources);
    result.num_sources = static_cast<int>(clusters.size());
    result.source_assignments.resize(sink_positions.size(), 0);

    // Compute source location (centroid) for each cluster
    for (int c = 0; c < static_cast<int>(clusters.size()); ++c) {
        double sx = 0, sy = 0;
        for (int idx : clusters[c]) {
            result.source_assignments[idx] = c;
            sx += sink_positions[idx].first;
            sy += sink_positions[idx].second;
        }
        sx /= static_cast<double>(clusters[c].size());
        sy /= static_cast<double>(clusters[c].size());
        result.source_x.push_back(sx);
        result.source_y.push_back(sy);
    }

    // Build subtree for each cluster and compute per-cluster max delay
    std::vector<double> cluster_max_delay(clusters.size(), 0);
    std::vector<double> cluster_min_delay(clusters.size(), std::numeric_limits<double>::max());

    for (int c = 0; c < static_cast<int>(clusters.size()); ++c) {
        for (int idx : clusters[c]) {
            int tree_idx = sink_indices[idx];
            double dist = std::abs(tree_[tree_idx].position.x - result.source_x[c]) +
                          std::abs(tree_[tree_idx].position.y - result.source_y[c]);
            double delay = dist * 0.001; // simple wire delay model
            cluster_max_delay[c] = std::max(cluster_max_delay[c], delay);
            cluster_min_delay[c] = std::min(cluster_min_delay[c], delay);
        }
    }

    // Balance inter-source skew: adjust source delays so all clusters have same max latency
    double global_max = *std::max_element(cluster_max_delay.begin(), cluster_max_delay.end());
    double worst_skew = 0;
    for (int c = 0; c < static_cast<int>(clusters.size()); ++c) {
        double skew = cluster_max_delay[c] - cluster_min_delay[c];
        worst_skew = std::max(worst_skew, skew);
        // Insert delay padding at source to equalize
        double padding = global_max - cluster_max_delay[c];
        (void)padding; // padding applied during physical implementation
    }

    result.max_skew = worst_skew;
    std::ostringstream oss;
    oss << "Multi-source CTS: " << result.num_sources << " sources, "
        << sink_positions.size() << " sinks, worst skew="
        << std::fixed << std::setprecision(3) << result.max_skew << "ns";
    result.message = oss.str();
    return result;
}

// ── synthesize_mesh ─────────────────────────────────────────────────────────

ClockMeshResult CtsEngine::synthesize_mesh(double mesh_pitch) {
    ClockMeshResult result;
    result.mesh_pitch = mesh_pitch;

    if (mesh_pitch <= 0 || tree_.empty()) {
        result.message = "Invalid mesh pitch or empty tree";
        return result;
    }

    // Determine bounding box of all sinks
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    int sink_count = 0;

    for (auto& node : tree_) {
        if (node.cell_id >= 0) {
            min_x = std::min(min_x, node.position.x);
            max_x = std::max(max_x, node.position.x);
            min_y = std::min(min_y, node.position.y);
            max_y = std::max(max_y, node.position.y);
            sink_count++;
        }
    }

    if (sink_count == 0) {
        result.message = "No sinks in tree for mesh construction";
        return result;
    }

    // Create regular grid of mesh points
    result.mesh_cols = std::max(1, static_cast<int>((max_x - min_x) / mesh_pitch) + 1);
    result.mesh_rows = std::max(1, static_cast<int>((max_y - min_y) / mesh_pitch) + 1);

    for (int r = 0; r < result.mesh_rows; ++r) {
        for (int c = 0; c < result.mesh_cols; ++c) {
            double mx = min_x + c * mesh_pitch;
            double my = min_y + r * mesh_pitch;
            result.mesh_points.emplace_back(mx, my);
        }
    }

    // Connect each sink to the nearest mesh point via stub
    result.stubs_inserted = 0;
    for (auto& node : tree_) {
        if (node.cell_id < 0) continue;
        double best_dist = std::numeric_limits<double>::max();
        for (auto& mp : result.mesh_points) {
            double d = std::abs(node.position.x - mp.first) +
                       std::abs(node.position.y - mp.second);
            best_dist = std::min(best_dist, d);
        }
        if (best_dist < mesh_pitch * 2.0) {
            result.stubs_inserted++;
        }
    }

    std::ostringstream oss;
    oss << "Clock mesh: " << result.mesh_rows << "x" << result.mesh_cols
        << " grid (" << result.mesh_points.size() << " points), pitch="
        << std::fixed << std::setprecision(1) << mesh_pitch
        << "um, " << result.stubs_inserted << " stubs";
    result.message = oss.str();
    return result;
}

// ── balance_with_ocv ────────────────────────────────────────────────────────
// OCV-aware insertion delay balancing: adjust buffer delays so that worst-case
// skew (late_max - early_min) stays within target.

void CtsEngine::balance_with_ocv(double ocv_margin) {
    if (tree_.empty()) return;

    double target_skew = cts_config_.target_skew_ns > 0 ? cts_config_.target_skew_ns : 0.05;

    // Compute path delays from root to each sink
    // Use DFS traversal to accumulate delays
    struct PathInfo { int node; double delay; };
    std::vector<PathInfo> sink_paths;

    int root = static_cast<int>(tree_.size()) - 1;
    // BFS/DFS to find root (node with no parent)
    std::vector<int> parent_of(tree_.size(), -1);
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        if (tree_[i].left >= 0) parent_of[tree_[i].left] = i;
        if (tree_[i].right >= 0) parent_of[tree_[i].right] = i;
    }
    for (int i = static_cast<int>(tree_.size()) - 1; i >= 0; --i) {
        if (parent_of[i] < 0) { root = i; break; }
    }

    // Compute cumulative delay from root to each leaf
    std::function<void(int, double)> traverse = [&](int node, double cum_delay) {
        if (node < 0 || node >= static_cast<int>(tree_.size())) return;
        double new_delay = cum_delay + tree_[node].delay;
        if (tree_[node].cell_id >= 0) {
            sink_paths.push_back({node, new_delay});
        }
        if (tree_[node].left >= 0) traverse(tree_[node].left, new_delay);
        if (tree_[node].right >= 0) traverse(tree_[node].right, new_delay);
    };
    traverse(root, 0.0);

    if (sink_paths.empty()) return;

    // Compute early and late arrivals for each sink
    double worst_late = std::numeric_limits<double>::lowest();
    double best_early = std::numeric_limits<double>::max();

    for (auto& sp : sink_paths) {
        double late = compute_ocv_delay(sp.delay, ocv_margin, false);
        double early = compute_ocv_delay(sp.delay, ocv_margin, true);
        worst_late = std::max(worst_late, late);
        best_early = std::min(best_early, early);
    }

    double ocv_skew = worst_late - best_early;

    // If OCV skew exceeds target, reduce by adding delay padding to early paths
    if (ocv_skew > target_skew) {
        double target_arrival = (worst_late + best_early) / 2.0;
        for (auto& sp : sink_paths) {
            double early = compute_ocv_delay(sp.delay, ocv_margin, true);
            if (early < target_arrival - target_skew / 2.0) {
                double needed = target_arrival - target_skew / 2.0 - early;
                tree_[sp.node].delay += needed;
            }
            double late = compute_ocv_delay(sp.delay, ocv_margin, false);
            if (late > target_arrival + target_skew / 2.0) {
                double excess = late - (target_arrival + target_skew / 2.0);
                tree_[sp.node].delay = std::max(0.0, tree_[sp.node].delay - excess * 0.5);
            }
        }
    }
}

// ── optimize_power ──────────────────────────────────────────────────────────
// After tree synthesis: swap non-critical buffers to HVT for power reduction.

PowerAwareCtsResult CtsEngine::optimize_power() {
    PowerAwareCtsResult result;

    if (tree_.empty()) {
        result.message = "No tree built for power optimization";
        return result;
    }

    double target_skew = cts_config_.target_skew_ns > 0 ? cts_config_.target_skew_ns : 0.05;

    // Find root and compute path delays
    int root = static_cast<int>(tree_.size()) - 1;
    std::vector<int> parent_of(tree_.size(), -1);
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        if (tree_[i].left >= 0) parent_of[tree_[i].left] = i;
        if (tree_[i].right >= 0) parent_of[tree_[i].right] = i;
    }
    for (int i = static_cast<int>(tree_.size()) - 1; i >= 0; --i) {
        if (parent_of[i] < 0) { root = i; break; }
    }

    // Compute delay to each sink
    std::vector<double> sink_delays;
    std::function<void(int, double)> collect_delays = [&](int node, double cum) {
        if (node < 0 || node >= static_cast<int>(tree_.size())) return;
        double d = cum + tree_[node].delay;
        if (tree_[node].cell_id >= 0) sink_delays.push_back(d);
        if (tree_[node].left >= 0) collect_delays(tree_[node].left, d);
        if (tree_[node].right >= 0) collect_delays(tree_[node].right, d);
    };
    collect_delays(root, 0.0);

    double max_delay = sink_delays.empty() ? 0 :
        *std::max_element(sink_delays.begin(), sink_delays.end());
    double min_delay = sink_delays.empty() ? 0 :
        *std::min_element(sink_delays.begin(), sink_delays.end());

    // Count internal (buffer) nodes
    int total_buf = 0;
    int hvt_count = 0;
    int lvt_count = 0;
    double original_power = 0;
    double optimized_power = 0;

    // Power model: each internal node is a buffer
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        if (tree_[i].cell_id >= 0) continue; // skip sinks
        if (tree_[i].left < 0 && tree_[i].right < 0) continue; // isolated
        total_buf++;

        // Compute slack: how much extra delay this buffer path has
        double node_delay = tree_[i].delay;
        double slack = max_delay - min_delay - target_skew;
        original_power += 1.0; // normalized power unit

        if (slack > target_skew * 2.0 && node_delay < max_delay * 0.8) {
            // Non-critical buffer → swap to HVT
            hvt_count++;
            tree_[i].delay *= 1.15; // HVT is ~15% slower
            optimized_power += 0.6; // HVT saves ~40% dynamic power
        } else if (node_delay >= max_delay * 0.9) {
            // Critical path buffer → use LVT
            lvt_count++;
            optimized_power += 1.3; // LVT uses ~30% more power
        } else {
            // Keep SVT
            optimized_power += 1.0;
        }
    }

    result.total_buffers = total_buf;
    result.hvt_buffers = hvt_count;
    result.lvt_buffers = lvt_count;
    result.power_savings_pct = (original_power > 0) ?
        (1.0 - optimized_power / original_power) * 100.0 : 0.0;

    std::ostringstream oss;
    oss << "Power-aware CTS: " << total_buf << " buffers ("
        << hvt_count << " HVT, " << lvt_count << " LVT, "
        << (total_buf - hvt_count - lvt_count) << " SVT), "
        << std::fixed << std::setprecision(1) << result.power_savings_pct
        << "% power savings";
    result.message = oss.str();
    return result;
}

// ── synthesize_advanced ─────────────────────────────────────────────────────
// Full enhanced CTS flow combining all advanced techniques.

CtsResult CtsEngine::synthesize_advanced() {
    auto t0 = std::chrono::high_resolution_clock::now();
    CtsResult result;

    buffer_delays_.clear();
    buffer_vt_assignments_.clear();

    // Collect sink positions from physical design cells in tree
    std::vector<std::pair<double,double>> sink_positions;
    std::vector<int> sink_cell_ids;
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        if (tree_[i].cell_id >= 0) {
            sink_positions.emplace_back(tree_[i].position.x, tree_[i].position.y);
            sink_cell_ids.push_back(tree_[i].cell_id);
        }
    }

    // If tree is empty, try to build from physical design cells
    if (sink_positions.empty()) {
        auto& cells = pd_.cells;
        for (int i = 0; i < static_cast<int>(cells.size()); ++i) {
            if (cells[i].cell_type.find("FF") != std::string::npos ||
                cells[i].cell_type.find("ff") != std::string::npos ||
                cells[i].cell_type.find("DFF") != std::string::npos) {
                sink_positions.emplace_back(cells[i].position.x, cells[i].position.y);
                sink_cell_ids.push_back(i);
            }
        }
    }

    if (sink_positions.empty()) {
        result.message = "No clock sinks found for advanced CTS";
        return result;
    }

    // Step 1: Build RSMT topology
    SteinerTree rsmt = build_rsmt(sink_positions);
    result.wirelength = rsmt.total_wirelength;

    // Step 2: Multi-source partitioning (if enabled and large design)
    MultiSourceResult ms_result;
    if (cts_config_.enable_multi_source && sink_positions.size() > 500) {
        // Need a tree built first for multi-source to work on
        if (tree_.empty()) {
            // Build a basic tree from RSMT for multi-source analysis
            tree_.clear();
            for (auto& sn : rsmt.nodes) {
                TreeNode tn;
                tn.position = Point(sn.x, sn.y);
                tn.cell_id = sn.is_steiner ? -1 : (sn.sink_idx >= 0 &&
                    sn.sink_idx < static_cast<int>(sink_cell_ids.size()) ?
                    sink_cell_ids[sn.sink_idx] : -1);
                if (!sn.children.empty()) {
                    tn.left = sn.children[0];
                    if (sn.children.size() > 1) tn.right = sn.children[1];
                }
                tree_.push_back(tn);
            }
        }
        ms_result = synthesize_multi_source(4);
    }

    // Step 3: Buffer insertion with library selection
    double target_slew = cts_config_.max_transition_ns > 0 ?
        cts_config_.max_transition_ns : 0.5;
    size_buffers(rsmt, target_slew);
    result.buffers_inserted = static_cast<int>(buffer_delays_.size());

    // Step 4: Ensure tree_ is populated from RSMT for subsequent passes
    if (tree_.empty() || tree_.size() < rsmt.nodes.size()) {
        tree_.clear();
        for (auto& sn : rsmt.nodes) {
            TreeNode tn;
            tn.position = Point(sn.x, sn.y);
            tn.cell_id = sn.is_steiner ? -1 : (sn.sink_idx >= 0 &&
                sn.sink_idx < static_cast<int>(sink_cell_ids.size()) ?
                sink_cell_ids[sn.sink_idx] : -1);
            if (!sn.children.empty()) {
                tn.left = sn.children[0];
                if (sn.children.size() > 1) tn.right = sn.children[1];
            }
            // Assign wire delay based on distance to children
            double wire_delay = 0;
            for (int c : sn.children) {
                wire_delay += (std::abs(sn.x - rsmt.nodes[c].x) +
                               std::abs(sn.y - rsmt.nodes[c].y)) * 0.001;
            }
            tn.delay = wire_delay;
            tree_.push_back(tn);
        }
    }

    // Step 5: Clock mesh overlay (if enabled)
    ClockMeshResult mesh_result;
    if (cts_config_.enable_clock_mesh) {
        double pitch = 50.0; // default mesh pitch
        mesh_result = synthesize_mesh(pitch);
    }

    // Step 6: OCV-aware balancing
    balance_with_ocv(cts_config_.ocv_margin);

    // Step 7: Power optimization (if enabled)
    PowerAwareCtsResult power_result;
    if (cts_config_.enable_power_opt) {
        power_result = optimize_power();
    }

    // Compute final skew from tree
    double max_lat = std::numeric_limits<double>::lowest();
    double min_lat = std::numeric_limits<double>::max();

    int root = static_cast<int>(tree_.size()) - 1;
    std::vector<int> parent_of(tree_.size(), -1);
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        if (tree_[i].left >= 0) parent_of[tree_[i].left] = i;
        if (tree_[i].right >= 0) parent_of[tree_[i].right] = i;
    }
    for (int i = static_cast<int>(tree_.size()) - 1; i >= 0; --i) {
        if (parent_of[i] < 0) { root = i; break; }
    }

    std::function<void(int, double)> compute_latency = [&](int node, double cum) {
        if (node < 0 || node >= static_cast<int>(tree_.size())) return;
        double d = cum + tree_[node].delay;
        if (tree_[node].cell_id >= 0) {
            max_lat = std::max(max_lat, d);
            min_lat = std::min(min_lat, d);
        }
        if (tree_[node].left >= 0) compute_latency(tree_[node].left, d);
        if (tree_[node].right >= 0) compute_latency(tree_[node].right, d);
    };
    compute_latency(root, 0.0);

    if (max_lat > std::numeric_limits<double>::lowest() &&
        min_lat < std::numeric_limits<double>::max()) {
        result.skew = max_lat - min_lat;
        result.max_latency_ps = max_lat * 1000.0; // ns to ps
        result.min_latency_ps = min_lat * 1000.0;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::ostringstream oss;
    oss << "Advanced CTS: " << sink_positions.size() << " sinks, "
        << result.buffers_inserted << " buffers, WL="
        << std::fixed << std::setprecision(1) << result.wirelength
        << ", skew=" << std::setprecision(3) << result.skew << "ns";
    if (cts_config_.enable_multi_source && !ms_result.message.empty()) {
        oss << " | " << ms_result.message;
    }
    if (cts_config_.enable_clock_mesh && !mesh_result.message.empty()) {
        oss << " | " << mesh_result.message;
    }
    if (cts_config_.enable_power_opt && !power_result.message.empty()) {
        oss << " | " << power_result.message;
    }
    result.message = oss.str();
    return result;
}

} // namespace sf
