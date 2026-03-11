// SiliconForge — CTS Implementation (DME Algorithm)
#include "pnr/cts.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <numeric>

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
            pd_.wires.push_back({0, n.position, tree_[n.left].position, 0.1});
            create_wires(n.left);
        }
        if (n.right >= 0) {
            pd_.wires.push_back({0, n.position, tree_[n.right].position, 0.1});
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

} // namespace sf
