#pragma once
// SiliconForge — Detailed Placement Engine
// Row-based legalization (Abacus/ISPD 2008), pairwise cell swapping,
// and window-based local reordering for HPWL minimization.
// References:
//   Spindler et al., "Abacus: Fast Legalization of Standard Cell Circuits
//                    with Minimal Movement", ISPD 2008
//   Kahng et al., "On Legalization of Row-Based Placements", GLSVLSI 2002

#include "pnr/physical.hpp"
#include "core/netlist.hpp"
#include <string>
#include <vector>

namespace sf {

struct DetailedPlaceConfig {
    bool   enable_legalization = true;
    bool   enable_cell_swap    = true;
    bool   enable_local_reorder = true;
    double max_displacement_um = 50.0;
    double row_height_um       = 2.8;
    double site_width_um       = 0.38;
    int    max_swap_distance   = 10;
    int    iterations          = 50;
};

struct DetailedPlaceResult {
    double total_displacement     = 0;
    double max_displacement       = 0;
    double hpwl_before            = 0;
    double hpwl_after             = 0;
    double hpwl_improvement_pct   = 0;
    int    cells_moved            = 0;
    int    swaps_performed        = 0;
    bool   legalization_pass      = false;
    double time_ms                = 0;
    std::string report;
};

struct CellSlot {
    int    row_idx  = -1;
    double x_pos    = 0;
    double width    = 0;
    int    cell_idx = -1; // -1 = empty
};

// Row computed from die area and row height parameters
struct PlacementRow {
    double y       = 0;
    double x_start = 0;
    double x_end   = 0;
    double height  = 0;
    double site_w  = 0;
    std::vector<CellSlot> slots;
};

class DetailedPlacer {
public:
    DetailedPlacer(PhysicalDesign& pd, const Netlist& nl);

    // High-level entry points
    DetailedPlaceResult legalize(const DetailedPlaceConfig& cfg = {});
    DetailedPlaceResult optimize(const DetailedPlaceConfig& cfg = {});
    DetailedPlaceResult run_enhanced();

    // Core algorithms
    int    abacus_legalize();
    int    cell_swap_optimize(double max_dist);
    int    local_reorder(int window_size = 3);
    double compute_hpwl() const;

private:
    PhysicalDesign& pd_;
    const Netlist&  nl_;
    DetailedPlaceConfig cfg_;

    std::vector<PlacementRow> rows_;

    void build_rows();
    int  find_nearest_row(double y) const;
    double snap_to_grid(double x) const;
    bool check_row_space(int row_idx, double x, double w) const;
    void insert_cell_in_row(int row_idx, int cell_idx, double x);
    void remove_cell_from_row(int cell_idx);

    // HPWL delta for a single net (fast incremental)
    double net_hpwl(int net_idx) const;
    double compute_swap_delta(int ci, int cj) const;

    // Row occupancy tracking — sorted intervals per row
    struct Interval {
        double x0, x1;
        int cell_idx;
    };
    std::vector<std::vector<Interval>> row_occupancy_;
    bool row_has_space(int row, double x, double w) const;
    void add_to_occupancy(int row, double x, double w, int cell_idx);
    void remove_from_occupancy(int row, int cell_idx);
    double find_nearest_free(int row, double x, double w) const;
};

} // namespace sf
