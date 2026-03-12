#pragma once
// SiliconForge — TSV insertion optimizer for 3D-IC
// Phase 40: Signal-integrity-aware TSV placement with cross-die routing

#include "core/die_to_die.hpp"
#include "pnr/physical.hpp"
#include <map>
#include <vector>
#include <cmath>

namespace sf {

// ── TSV Placement Strategy ──────────────────────────────────────────────────
enum class TsvPlaceStrategy {
    CENTER_OF_MASS,   // Original: place at CoM of net pins
    MIN_DELAY,        // Minimize total wire delay (weighted)
    GRID_ALIGNED,     // Snap to regular grid (manufacturing-friendly)
    CLUSTER_AWARE     // Avoid TSV clustering (reduce coupling noise)
};

struct TsvPlaceConfig {
    TsvPlaceStrategy strategy = TsvPlaceStrategy::CLUSTER_AWARE;
    TsvTechParams tech;
    double min_tsv_spacing_um = 10.0;   // minimum TSV-to-TSV distance
    int spare_ratio_percent = 5;         // 5% spare TSVs for redundancy
    bool insert_power_tsvs = true;       // auto-insert power/ground TSVs
    int power_grid_nx = 8;               // power TSV grid density
    int power_grid_ny = 8;
    double max_coupling_ff = 5.0;        // max acceptable coupling (fF)
};

struct TsvPlaceResult {
    int signal_tsvs_placed = 0;
    int power_tsvs_placed = 0;
    int spare_tsvs_placed = 0;
    double max_delay_ps = 0;
    double max_coupling_ff = 0;
    double total_wirelength_reduction = 0;
    bool drc_clean = true;              // no spacing violations
};

class TsvManager {
public:
    // Full 3D-IC TSV insertion with signal integrity awareness
    static TsvPlaceResult insert_tsvs_si(
        PackageDesign& pkg,
        PhysicalDesign& die1, PhysicalDesign& die2,
        int z1, int z2,
        const TsvPlaceConfig& cfg = {});

    // Legacy API (backward compatible)
    static void insert_tsvs(PackageDesign& pkg,
                            PhysicalDesign& die1, PhysicalDesign& die2,
                            int z1, int z2);

    // Cross-die routing: route interposer traces for 2.5D
    static int route_interposer(PackageDesign& pkg,
                                const InterposerConfig& icfg = {});

    // DRC: check TSV spacing violations
    static bool check_tsv_spacing(const PackageDesign& pkg, double min_spacing_um);

    // Optimize TSV positions to reduce coupling
    static int optimize_tsv_coupling(PackageDesign& pkg, const TsvTechParams& tech,
                                     double max_coupling_ff);

private:
    static void place_tsv_for_net(PackageDesign& pkg, int net_id,
                                  const PhysicalDesign& d1, const PhysicalDesign& d2,
                                  int d1_id, int d2_id,
                                  const TsvPlaceConfig& cfg);

    // Grid-aligned placement
    static void place_tsv_grid_aligned(PackageDesign& pkg, int net_id,
                                       float cx, float cy,
                                       int from_die, int to_die,
                                       const TsvPlaceConfig& cfg);

    // Check if position violates min spacing
    static bool violates_spacing(const PackageDesign& pkg, float x, float y,
                                 double min_spacing);
};

} // namespace sf
