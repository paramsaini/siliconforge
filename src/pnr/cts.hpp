#pragma once
// SiliconForge — Clock Tree Synthesis (CTS)
// Zero-skew DME algorithm + buffer insertion.
// Reference: T.H. Chao et al., "Zero Skew Clock Routing with Minimum Wirelength", IEEE TCAD 1992

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct CtsResult {
    double skew = 0;
    double wirelength = 0;
    int buffers_inserted = 0;
    double time_ms = 0;
    std::string message;
};

class CtsEngine {
public:
    explicit CtsEngine(PhysicalDesign& pd) : pd_(pd) {}

    // Build clock tree for the given sink points
    CtsResult build_clock_tree(const Point& source, const std::vector<int>& sink_cells);

private:
    PhysicalDesign& pd_;

    struct TreeNode {
        Point position;
        int cell_id = -1;    // -1 if Steiner point
        int left = -1, right = -1;
        double delay = 0;
    };

    std::vector<TreeNode> tree_;

    // DME: build balanced merge tree
    int dme_merge(const std::vector<int>& sinks);
    void dme_bottom_up(int node);
    void dme_top_down(int node, const Point& parent_pos);
};

} // namespace sf
