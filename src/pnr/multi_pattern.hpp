#pragma once
// SiliconForge — Multi-Patterning (SADP/SAQP) Engine
// Color-aware layout decomposition for advanced FinFET nodes.
// Implements graph-coloring-based mask assignment with conflict detection
// and resolution for SADP (2-color) and SAQP (4-color) flows.
//
// References:
//   Kahng et al., "Layout Decomposition for Double Patterning Lithography", ICCAD 2008
//   Xu & Chu, "GREMA: Graph Reduction Based Efficient Mask Assignment", DAC 2009

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

enum class PatternType { LITHO_ETCH, SADP, SAQP, EUV };
enum class Color { UNCOLORED = -1, COLOR_A = 0, COLOR_B = 1, COLOR_C = 2, COLOR_D = 3 };

struct MultiPatternConfig {
    PatternType type = PatternType::SADP;
    double min_space_nm = 40.0;
    double min_width_nm = 20.0;
    int max_colors = 2;
    double coloring_conflict_penalty = 100.0;
    bool decompose_vias = true;
    std::vector<int> mp_layers;
};

struct ColorAssignment {
    int wire_idx = -1;
    int layer = 0;
    Color color = Color::UNCOLORED;
};

struct ColorConflict {
    int wire_a = -1;
    int wire_b = -1;
    int layer = 0;
    double spacing_nm = 0;
    double min_required_nm = 0;
    std::string resolution;
};

struct MultiPatternResult {
    int wires_colored = 0;
    int conflicts_found = 0;
    int conflicts_resolved = 0;
    int color_changes = 0;
    double min_spacing_achieved_nm = 0;
    bool decomposition_legal = false;
    std::vector<ColorAssignment> assignments;
    std::vector<ColorConflict> unresolved;
    std::string report;
    double time_ms = 0;
};

class MultiPatternEngine {
public:
    explicit MultiPatternEngine(PhysicalDesign& pd);

    MultiPatternResult decompose(const MultiPatternConfig& cfg = {});

    std::vector<ColorAssignment> color_graph(int layer);

    std::vector<ColorConflict> detect_conflicts(
        const std::vector<ColorAssignment>& assignments);

    int resolve_conflicts(std::vector<ColorAssignment>& assignments,
                          const MultiPatternConfig& cfg);

    MultiPatternResult sadp_decompose(int layer);
    MultiPatternResult saqp_decompose(int layer);

    MultiPatternResult run_enhanced();

    void set_config(const MultiPatternConfig& cfg) { cfg_ = cfg; }
    const MultiPatternConfig& config() const { return cfg_; }

private:
    PhysicalDesign& pd_;
    MultiPatternConfig cfg_;

    struct ConflictGraph {
        int num_nodes = 0;
        std::vector<std::vector<int>> adj;
        std::vector<int> wire_indices;
    };

    ConflictGraph build_conflict_graph(int layer, double min_space);
    bool try_2_color(const ConflictGraph& g, std::vector<Color>& colors);
    bool try_4_color(const ConflictGraph& g, std::vector<Color>& colors);

    double parallel_run_length(const WireSegment& a, const WireSegment& b) const;
    double segment_spacing(const WireSegment& a, const WireSegment& b) const;

    static bool is_horizontal(const WireSegment& w) {
        return std::abs(w.end.y - w.start.y) < 1e-9;
    }
    static bool is_vertical(const WireSegment& w) {
        return std::abs(w.end.x - w.start.x) < 1e-9;
    }
};

} // namespace sf
