// SiliconForge — TSV insertion optimizer
// Phase 40: Signal-integrity-aware placement with cross-die routing
#include "pnr/tsv_mgr.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace sf {

// ── Legacy API (backward compatible) ────────────────────────────────────────

void TsvManager::insert_tsvs(PackageDesign& pkg, PhysicalDesign& die1,
                              PhysicalDesign& die2, int z1, int z2) {
    TsvPlaceConfig cfg;
    cfg.strategy = TsvPlaceStrategy::CENTER_OF_MASS;
    cfg.insert_power_tsvs = false;
    cfg.spare_ratio_percent = 0; // legacy API: no spares
    insert_tsvs_si(pkg, die1, die2, z1, z2, cfg);
}

// ── Full SI-aware TSV Insertion ─────────────────────────────────────────────

TsvPlaceResult TsvManager::insert_tsvs_si(
    PackageDesign& pkg,
    PhysicalDesign& die1, PhysicalDesign& die2,
    int z1, int z2,
    const TsvPlaceConfig& cfg)
{
    TsvPlaceResult result;

    int d1_id = -1, d2_id = -1;
    for (const auto& d : pkg.dies()) {
        if (d.z_layer == z1) d1_id = d.id;
        if (d.z_layer == z2) d2_id = d.id;
    }

    if (d1_id == -1 || d2_id == -1) {
        std::cerr << "TsvManager: Could not find dies at layers " << z1 << " and " << z2 << "\n";
        return result;
    }

    // Find shared nets between the two dies
    std::map<int, bool> die1_nets;
    for (const auto& pnet : die1.nets) die1_nets[pnet.id] = true;

    std::vector<int> shared_nets;
    for (const auto& pnet2 : die2.nets) {
        if (die1_nets.count(pnet2.id)) shared_nets.push_back(pnet2.id);
    }

    // Place signal TSVs for shared nets
    for (int net_id : shared_nets) {
        place_tsv_for_net(pkg, net_id, die1, die2, d1_id, d2_id, cfg);
        result.signal_tsvs_placed++;
    }

    // Insert power/ground TSVs if configured
    if (cfg.insert_power_tsvs && !pkg.dies().empty()) {
        float w = 10000, h = 10000;
        for (const auto& d : pkg.dies()) {
            if (d.id == d1_id) { w = d.width_um; h = d.height_um; break; }
        }
        for (int ix = 0; ix < cfg.power_grid_nx; ix++) {
            for (int iy = 0; iy < cfg.power_grid_ny; iy++) {
                float px = (ix + 0.5f) * w / cfg.power_grid_nx;
                float py = (iy + 0.5f) * h / cfg.power_grid_ny;
                bool is_vdd = ((ix + iy) % 2 == 0);
                pkg.add_power_tsv(is_vdd, d1_id, d2_id, px, py, cfg.tech);
                result.power_tsvs_placed++;
            }
        }
    }

    // Insert spare TSVs for redundancy
    if (cfg.spare_ratio_percent > 0 && result.signal_tsvs_placed > 0) {
        int num_spares = std::max(1, result.signal_tsvs_placed * cfg.spare_ratio_percent / 100);
        float w = 10000, h = 10000;
        for (const auto& d : pkg.dies()) {
            if (d.id == d1_id) { w = d.width_um; h = d.height_um; break; }
        }
        // Distribute spares evenly along die edge
        for (int i = 0; i < num_spares; i++) {
            float sx = (i + 0.5f) * w / num_spares;
            float sy = h * 0.95f; // near top edge
            if (!violates_spacing(pkg, sx, sy, cfg.min_tsv_spacing_um)) {
                pkg.add_spare_tsv(d1_id, d2_id, sx, sy, cfg.tech);
                result.spare_tsvs_placed++;
            }
        }
    }

    // Compute result metrics
    result.max_delay_ps = pkg.worst_case_tsv_delay_ps();
    result.drc_clean = check_tsv_spacing(pkg, cfg.min_tsv_spacing_um);

    // Compute max coupling
    for (const auto& t : pkg.tsvs()) {
        auto si = pkg.compute_tsv_si(t.id, cfg.tech);
        double coup_ff = si.coupling_cap * 1e15;
        result.max_coupling_ff = std::max(result.max_coupling_ff, coup_ff);
    }

    return result;
}

// ── Net-level TSV Placement ─────────────────────────────────────────────────

void TsvManager::place_tsv_for_net(PackageDesign& pkg, int net_id,
                                    const PhysicalDesign& d1, const PhysicalDesign& d2,
                                    int d1_id, int d2_id,
                                    const TsvPlaceConfig& cfg)
{
    // Calculate Center of Mass for net across both dies
    float sum_x = 0, sum_y = 0;
    int count = 0;

    auto count_pins = [&](const PhysicalDesign& d) {
        for (const auto& n : d.nets) {
            if (n.id == net_id) {
                for (int cid : n.cell_ids) {
                    if (cid >= 0 && cid < (int)d.cells.size()) {
                        sum_x += (float)d.cells[cid].position.x;
                        sum_y += (float)d.cells[cid].position.y;
                        count++;
                    }
                }
            }
        }
    };

    count_pins(d1);
    count_pins(d2);

    if (count == 0) return;

    float cx = sum_x / count;
    float cy = sum_y / count;

    switch (cfg.strategy) {
        case TsvPlaceStrategy::GRID_ALIGNED:
            place_tsv_grid_aligned(pkg, net_id, cx, cy, d1_id, d2_id, cfg);
            break;
        case TsvPlaceStrategy::CLUSTER_AWARE: {
            // Shift position away from existing TSVs to reduce coupling
            for (int iter = 0; iter < 5; iter++) {
                if (!violates_spacing(pkg, cx, cy, cfg.min_tsv_spacing_um)) break;
                // Perturb outward from nearest TSV
                float shift = (float)cfg.min_tsv_spacing_um * 0.5f;
                cx += shift * ((iter % 2) ? 1.0f : -1.0f);
                cy += shift * ((iter % 3) ? 1.0f : -1.0f);
            }
            pkg.add_signal_tsv(net_id, d1_id, d2_id, cx, cy, cfg.tech);
            break;
        }
        case TsvPlaceStrategy::CENTER_OF_MASS:
        case TsvPlaceStrategy::MIN_DELAY:
        default:
            pkg.add_signal_tsv(net_id, d1_id, d2_id, cx, cy, cfg.tech);
            break;
    }
}

void TsvManager::place_tsv_grid_aligned(PackageDesign& pkg, int net_id,
                                         float cx, float cy,
                                         int from_die, int to_die,
                                         const TsvPlaceConfig& cfg)
{
    // Snap to nearest grid point
    double pitch = cfg.tech.pitch_um;
    if (pitch <= 0) pitch = 10.0;
    float gx = std::round(cx / pitch) * (float)pitch;
    float gy = std::round(cy / pitch) * (float)pitch;

    // If grid point occupied, search neighbors
    if (violates_spacing(pkg, gx, gy, cfg.min_tsv_spacing_um)) {
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                float nx = gx + dx * (float)pitch;
                float ny = gy + dy * (float)pitch;
                if (!violates_spacing(pkg, nx, ny, cfg.min_tsv_spacing_um)) {
                    pkg.add_signal_tsv(net_id, from_die, to_die, nx, ny, cfg.tech);
                    return;
                }
            }
        }
    }
    pkg.add_signal_tsv(net_id, from_die, to_die, gx, gy, cfg.tech);
}

// ── Cross-die Routing (2.5D Interposer) ────────────────────────────────────

int TsvManager::route_interposer(PackageDesign& pkg, const InterposerConfig& icfg) {
    int traces_routed = 0;
    // For each pair of dies connected by bumps, route interposer traces
    std::map<int, std::vector<const MicroBump*>> net_bumps;
    for (const auto& b : pkg.bumps()) {
        if (!b.is_power && b.net_id >= 0) {
            net_bumps[b.net_id].push_back(&b);
        }
    }

    for (const auto& [net_id, bumps] : net_bumps) {
        if (bumps.size() < 2) continue;
        // Connect bumps with L-shaped interposer routes
        for (size_t i = 0; i + 1 < bumps.size(); i++) {
            float x0 = bumps[i]->x, y0 = bumps[i]->y;
            float x1 = bumps[i+1]->x, y1 = bumps[i+1]->y;
            // Route on alternating RDL layers
            int layer = (int)(i % icfg.num_rdl_layers);
            // L-shape: horizontal then vertical
            float mid_x = x1, mid_y = y0;
            pkg.add_interposer_trace(net_id, layer, x0, y0, mid_x, mid_y, icfg);
            pkg.add_interposer_trace(net_id, layer, mid_x, mid_y, x1, y1, icfg);
            traces_routed += 2;
        }
    }
    return traces_routed;
}

// ── DRC: TSV Spacing Check ──────────────────────────────────────────────────

bool TsvManager::check_tsv_spacing(const PackageDesign& pkg, double min_spacing_um) {
    const auto& tsvs = pkg.tsvs();
    for (size_t i = 0; i < tsvs.size(); i++) {
        for (size_t j = i + 1; j < tsvs.size(); j++) {
            // Only check TSVs on same die pair
            if (tsvs[i].from_die_id != tsvs[j].from_die_id) continue;
            double dx = tsvs[i].x - tsvs[j].x;
            double dy = tsvs[i].y - tsvs[j].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_spacing_um) return false;
        }
    }
    return true;
}

// ── Coupling Optimization ───────────────────────────────────────────────────

int TsvManager::optimize_tsv_coupling(PackageDesign& pkg, const TsvTechParams& tech,
                                       double max_coupling_ff) {
    // Iterative spreading: push TSVs apart if coupling exceeds threshold
    int iterations = 0;
    // Note: we can't directly modify TSVs in PackageDesign (const access),
    // so this is a validation/reporting pass
    int violations = 0;
    for (const auto& t : pkg.tsvs()) {
        auto si = pkg.compute_tsv_si(t.id, tech);
        if (si.coupling_cap * 1e15 > max_coupling_ff) violations++;
    }
    return violations;
}

// ── Spacing Helper ──────────────────────────────────────────────────────────

bool TsvManager::violates_spacing(const PackageDesign& pkg, float x, float y,
                                   double min_spacing) {
    for (const auto& t : pkg.tsvs()) {
        double dx = t.x - x;
        double dy = t.y - y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_spacing && dist > 0.001) return true;
    }
    return false;
}

} // namespace sf
