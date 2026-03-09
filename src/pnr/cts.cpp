// SiliconForge — CTS Implementation (DME Algorithm)
#include "pnr/cts.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <cmath>

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

    // Insert buffers at intermediate nodes (non-leaf, non-root)
    int buffers = 0;
    for (size_t i = sink_nodes.size(); i < tree_.size(); ++i) {
        if ((int)i != root && tree_[i].left >= 0) {
            // Add buffer cell
            pd_.add_cell("clkbuf_" + std::to_string(i), "CLKBUF_X1", 2.0, pd_.row_height);
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

} // namespace sf
