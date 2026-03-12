// SiliconForge — 3D-IC & Multi-Die Packaging
// Phase 40: Industrial chiplet/multi-die integration
#include "core/die_to_die.hpp"
#include <iostream>
#include <cmath>
#include <cassert>

namespace sf {

// ── TSV Parasitics (physics-based) ──────────────────────────────────────────

TsvParasitics TsvTechParams::compute_parasitics() const {
    TsvParasitics p;
    double r_m = radius_um * 1e-6;
    double h_m = height_um * 1e-6;
    double r_ox_m = (radius_um + oxide_thick_um) * 1e-6;

    // R = ρ·h / (π·r²)
    p.resistance = cu_resistivity * h_m / (M_PI * r_m * r_m);

    // L = μ₀·h/(2π)·[ln(2h/r) - 1 + r/(2h)]
    if (r_m > 0 && h_m > 0) {
        p.inductance = (mu0 * h_m / (2.0 * M_PI)) *
                       (std::log(2.0 * h_m / r_m) - 1.0 + r_m / (2.0 * h_m));
    }

    // C = 2π·ε₀·εᵣ(oxide)·h / ln(r_oxide/r_tsv)
    if (r_ox_m > r_m) {
        p.capacitance = 2.0 * M_PI * eps0 * oxide_er * h_m / std::log(r_ox_m / r_m);
    }

    // Elmore delay: τ = R·C
    p.delay_ps = p.resistance * p.capacitance * 1e12;

    // Bandwidth ≈ 1/(2πRC)
    double rc = p.resistance * p.capacitance;
    if (rc > 0) {
        p.bandwidth_gbps = 1.0 / (2.0 * M_PI * rc) / 1e9;
    }

    return p;
}

TsvParasitics TsvTechParams::compute_coupling(double neighbor_dist_um) const {
    TsvParasitics p = compute_parasitics();
    double d_m = neighbor_dist_um * 1e-6;
    double r_m = radius_um * 1e-6;
    double h_m = height_um * 1e-6;

    // Coupling capacitance: C_coupling ≈ ε₀·εᵣ(Si)·h·r / d
    // Simplified fringe model for adjacent TSVs
    if (d_m > 0) {
        p.coupling_cap = eps0 * si_er * h_m * r_m / d_m;
    }
    return p;
}

// ── Yield Model ─────────────────────────────────────────────────────────────

double YieldModel::single_die_yield() const {
    // Poisson yield model: Y = e^(-D·A)
    return std::exp(-die_defect_density * die_area_cm2);
}

double YieldModel::tsv_array_yield(int num_tsvs, int spares) const {
    if (num_tsvs <= 0) return 1.0;
    // Binomial redundancy: probability of ≤ spares failures
    double p_good = 1.0 - tsv_defect_rate;
    if (spares == 0) {
        return std::pow(p_good, num_tsvs);
    }
    // Sum binomial: Σ C(n,k)·p^k·(1-p)^(n-k) for k=0..spares
    double yield = 0;
    double p_fail = tsv_defect_rate;
    for (int k = 0; k <= spares && k <= num_tsvs; k++) {
        // Use log to avoid overflow for large n
        double log_comb = 0;
        for (int j = 0; j < k; j++) {
            log_comb += std::log((double)(num_tsvs - j)) - std::log((double)(j + 1));
        }
        yield += std::exp(log_comb + k * std::log(p_fail) +
                         (num_tsvs - k) * std::log(p_good));
    }
    return yield;
}

double YieldModel::bump_array_yield(int num_bumps, int spares) const {
    if (num_bumps <= 0) return 1.0;
    double p_good = 1.0 - bump_defect_rate;
    if (spares == 0) return std::pow(p_good, num_bumps);
    double yield = 0;
    for (int k = 0; k <= spares && k <= num_bumps; k++) {
        double log_comb = 0;
        for (int j = 0; j < k; j++) {
            log_comb += std::log((double)(num_bumps - j)) - std::log((double)(j + 1));
        }
        yield += std::exp(log_comb + k * std::log(bump_defect_rate) +
                         (num_bumps - k) * std::log(p_good));
    }
    return yield;
}

double YieldModel::package_yield(int num_dies, int tsvs_per_die, int spare_tsvs) const {
    double y = 1.0;
    for (int i = 0; i < num_dies; i++) {
        y *= single_die_yield();
    }
    y *= tsv_array_yield(tsvs_per_die * (num_dies > 1 ? num_dies - 1 : 0), spare_tsvs);
    return y;
}

// ── PowerGrid3D ─────────────────────────────────────────────────────────────

double PowerGrid3D::max_ir_drop() const {
    double mx = 0;
    for (const auto& die_map : ir_drop_map) {
        for (double v : die_map) mx = std::max(mx, v);
    }
    return mx;
}

double PowerGrid3D::avg_ir_drop() const {
    double sum = 0;
    int count = 0;
    for (const auto& die_map : ir_drop_map) {
        for (double v : die_map) { sum += v; count++; }
    }
    return count > 0 ? sum / count : 0;
}

int PowerGrid3D::hotspot_count(double threshold_mv) const {
    int count = 0;
    for (const auto& die_map : ir_drop_map) {
        for (double v : die_map) if (v > threshold_mv) count++;
    }
    return count;
}

// ── PackageDesign Implementation ────────────────────────────────────────────

int PackageDesign::add_die(const std::string& name, int z_layer, float x, float y) {
    int id = die_counter_++;
    DieInstance d;
    d.id = id;
    d.name = name;
    d.z_layer = z_layer;
    d.x_offset = x;
    d.y_offset = y;
    dies_.push_back(d);
    return id;
}

void PackageDesign::set_die_size(int die_id, float w_um, float h_um) {
    for (auto& d : dies_) {
        if (d.id == die_id) { d.width_um = w_um; d.height_um = h_um; return; }
    }
}

void PackageDesign::set_die_power(int die_id, double power_w) {
    for (auto& d : dies_) {
        if (d.id == die_id) { d.power_w = power_w; return; }
    }
}

int PackageDesign::add_tsv(int net_id, int from_die, int to_die, float x, float y) {
    return add_signal_tsv(net_id, from_die, to_die, x, y);
}

int PackageDesign::add_signal_tsv(int net_id, int from_die, int to_die,
                                   float x, float y, const TsvTechParams& tech) {
    int id = tsv_counter_++;
    TSV t;
    t.id = id;
    t.net_id = net_id;
    t.from_die_id = from_die;
    t.to_die_id = to_die;
    t.x = x;
    t.y = y;
    t.is_signal = true;
    t.is_spare = false;
    t.parasitics = tech.compute_parasitics();
    tsvs_.push_back(t);
    return id;
}

int PackageDesign::add_power_tsv(bool is_vdd, int from_die, int to_die,
                                  float x, float y, const TsvTechParams& tech) {
    int id = tsv_counter_++;
    TSV t;
    t.id = id;
    t.net_id = is_vdd ? -2 : -3; // -2=VDD, -3=GND convention
    t.from_die_id = from_die;
    t.to_die_id = to_die;
    t.x = x;
    t.y = y;
    t.is_signal = false;
    t.is_spare = false;
    // Power TSVs: larger radius → lower R
    TsvTechParams ptech = tech;
    ptech.radius_um *= 1.5; // power TSVs are typically wider
    t.parasitics = ptech.compute_parasitics();
    tsvs_.push_back(t);
    return id;
}

int PackageDesign::add_spare_tsv(int from_die, int to_die, float x, float y,
                                  const TsvTechParams& tech) {
    int id = tsv_counter_++;
    TSV t;
    t.id = id;
    t.net_id = -1; // unassigned
    t.from_die_id = from_die;
    t.to_die_id = to_die;
    t.x = x;
    t.y = y;
    t.is_signal = true;
    t.is_spare = true;
    t.parasitics = tech.compute_parasitics();
    tsvs_.push_back(t);
    return id;
}

int PackageDesign::add_micro_bump(int net_id, int die_top, int die_bottom,
                                   float x, float y, const MicroBumpTech& tech) {
    int id = bump_counter_++;
    MicroBump b;
    b.id = id;
    b.net_id = net_id;
    b.die_top = die_top;
    b.die_bottom = die_bottom;
    b.x = x;
    b.y = y;
    b.contact_resistance = tech.contact_r_ohm;
    b.inductance_ph = tech.inductance_ph;
    b.capacitance_ff = tech.capacitance_ff;
    b.max_current_ma = tech.max_current_ma;
    b.is_power = false;
    b.is_spare = false;
    bumps_.push_back(b);
    return id;
}

int PackageDesign::add_power_bump(bool is_vdd, int die_top, int die_bottom,
                                   float x, float y, const MicroBumpTech& tech) {
    int id = bump_counter_++;
    MicroBump b;
    b.id = id;
    b.net_id = is_vdd ? -2 : -3;
    b.die_top = die_top;
    b.die_bottom = die_bottom;
    b.x = x;
    b.y = y;
    b.contact_resistance = tech.contact_r_ohm;
    b.inductance_ph = tech.inductance_ph;
    b.capacitance_ff = tech.capacitance_ff;
    b.max_current_ma = tech.max_current_ma;
    b.is_power = true;
    b.is_spare = false;
    bumps_.push_back(b);
    return id;
}

int PackageDesign::add_interposer_trace(int net_id, int layer, float x0, float y0,
                                         float x1, float y1, const InterposerConfig& cfg) {
    int id = trace_counter_++;
    InterposerTrace t;
    t.id = id;
    t.net_id = net_id;
    t.layer = layer;
    t.x0 = x0; t.y0 = y0;
    t.x1 = x1; t.y1 = y1;
    t.width_um = cfg.trace_width_um;
    t.r_per_um = cfg.trace_r_per_um;
    t.c_per_um = cfg.trace_c_per_um;
    traces_.push_back(t);
    return id;
}

int PackageDesign::add_d2d_link(int from_die, int to_die, int lanes, double rate_gbps) {
    int id = link_counter_++;
    D2DLink l;
    l.id = id;
    l.from_die = from_die;
    l.to_die = to_die;
    l.num_lanes = lanes;
    l.lane_rate_gbps = rate_gbps;
    l.latency_ns = 2.0;
    l.power_mw_per_lane = 0.5;
    d2d_links_.push_back(l);
    return id;
}

void PackageDesign::build_power_grid(int nx, int ny, const TsvTechParams& tech) {
    power_grid_.grid_nx = nx;
    power_grid_.grid_ny = ny;
    power_grid_.num_dies = (int)dies_.size();
    power_grid_.power_tsvs.clear();
    power_grid_.ir_drop_map.clear();

    if (dies_.size() < 2) return;

    // Place VDD/GND TSV pairs in checkerboard pattern across die pairs
    int ptsv_id = 0;
    for (size_t di = 0; di + 1 < dies_.size(); di++) {
        float w = dies_[di].width_um;
        float h = dies_[di].height_um;
        for (int ix = 0; ix < nx; ix++) {
            for (int iy = 0; iy < ny; iy++) {
                float px = (ix + 0.5f) * w / nx;
                float py = (iy + 0.5f) * h / ny;
                bool is_vdd = ((ix + iy) % 2 == 0);
                PowerTSV pt;
                pt.id = ptsv_id++;
                pt.x = px;
                pt.y = py;
                pt.is_vdd = is_vdd;
                // Power TSV resistance
                TsvTechParams ptech = tech;
                ptech.radius_um *= 1.5;
                pt.resistance = ptech.compute_parasitics().resistance;
                // Estimated current: die power / (Vdd × num_power_tsvs)
                double vdd = 0.8; // nominal
                int num_ptsvs = nx * ny / 2; // half are VDD
                if (num_ptsvs > 0)
                    pt.current_ma = (dies_[di].power_w / vdd * 1000.0) / num_ptsvs;
                power_grid_.power_tsvs.push_back(pt);
            }
        }
    }

    // Compute IR-drop map per die: V_drop = I × R_tsv (simplified)
    for (size_t di = 0; di < dies_.size(); di++) {
        std::vector<double> drop_map(nx * ny, 0.0);
        for (int ix = 0; ix < nx; ix++) {
            for (int iy = 0; iy < ny; iy++) {
                // Find nearest VDD power TSV and estimate drop
                double min_dist = 1e9;
                double tsv_r = 0.01;
                double tsv_i = 0;
                for (const auto& pt : power_grid_.power_tsvs) {
                    if (!pt.is_vdd) continue;
                    float gx = (ix + 0.5f) * dies_[di].width_um / nx;
                    float gy = (iy + 0.5f) * dies_[di].height_um / ny;
                    double dist = std::abs(pt.x - gx) + std::abs(pt.y - gy);
                    if (dist < min_dist) {
                        min_dist = dist;
                        tsv_r = pt.resistance;
                        tsv_i = pt.current_ma;
                    }
                }
                // IR-drop = I×R + sheet_R × distance_factor
                double sheet_r_per_um = 0.001; // mΩ/um sheet resistance
                double drop_mv = tsv_i * tsv_r * 1000.0 + min_dist * sheet_r_per_um;
                drop_map[ix * ny + iy] = drop_mv;
            }
        }
        power_grid_.ir_drop_map.push_back(drop_map);
    }
}

double PackageDesign::compute_ir_drop(int die_id) const {
    if (die_id < 0 || die_id >= (int)power_grid_.ir_drop_map.size()) return 0;
    double mx = 0;
    for (double v : power_grid_.ir_drop_map[die_id]) mx = std::max(mx, v);
    return mx;
}

double PackageDesign::estimate_yield(const YieldModel& model) const {
    int n_dies = (int)dies_.size();
    int n_tsvs = 0, n_spares = 0;
    for (const auto& t : tsvs_) {
        if (t.is_spare) n_spares++;
        else n_tsvs++;
    }
    return model.package_yield(n_dies, n_tsvs, n_spares);
}

TsvParasitics PackageDesign::compute_tsv_si(int tsv_id, const TsvTechParams& tech) const {
    // Find TSV and compute SI including coupling from neighbors
    for (const auto& t : tsvs_) {
        if (t.id == tsv_id) {
            TsvParasitics p = t.parasitics;
            // Find nearest neighbor TSV for coupling
            double min_dist = 1e9;
            for (const auto& t2 : tsvs_) {
                if (t2.id == tsv_id) continue;
                double dx = t.x - t2.x;
                double dy = t.y - t2.y;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < min_dist) min_dist = dist;
            }
            if (min_dist < 1e8) {
                p.coupling_cap = tech.compute_coupling(min_dist).coupling_cap;
            }
            return p;
        }
    }
    return {};
}

double PackageDesign::worst_case_tsv_delay_ps() const {
    double mx = 0;
    for (const auto& t : tsvs_) {
        mx = std::max(mx, t.parasitics.delay_ps);
    }
    return mx;
}

double PackageDesign::total_d2d_bandwidth_gbps() const {
    double bw = 0;
    for (const auto& l : d2d_links_) bw += l.bandwidth_gbps();
    return bw;
}

int PackageDesign::assign_spare_tsv(int failed_tsv_id) {
    // Find a spare TSV on the same die pair and reassign
    int from = -1, to = -1;
    for (auto& t : tsvs_) {
        if (t.id == failed_tsv_id) {
            from = t.from_die_id;
            to = t.to_die_id;
            t.net_id = -1; // mark failed
            break;
        }
    }
    if (from == -1) return -1;

    for (auto& t : tsvs_) {
        if (t.is_spare && t.from_die_id == from && t.to_die_id == to) {
            t.is_spare = false;
            // Inherit the failed TSV's net_id would be done by caller
            return t.id;
        }
    }
    return -1; // no spare available
}

int PackageDesign::count_spare_tsvs() const {
    int count = 0;
    for (const auto& t : tsvs_) if (t.is_spare) count++;
    return count;
}

ChipletReport PackageDesign::generate_report(const TsvTechParams& tsv_tech,
                                              const YieldModel& yield) const {
    ChipletReport rpt;
    rpt.total_dies = (int)dies_.size();
    rpt.total_tsvs = (int)tsvs_.size();
    rpt.total_bumps = (int)bumps_.size();
    rpt.total_d2d_links = (int)d2d_links_.size();

    for (const auto& t : tsvs_) {
        if (t.is_spare) rpt.spare_tsvs++;
        else if (t.is_signal) rpt.signal_tsvs++;
        else rpt.power_tsvs++;
        rpt.total_tsv_resistance += t.parasitics.resistance;
        rpt.total_tsv_capacitance_ff += t.parasitics.capacitance * 1e15;
        rpt.max_tsv_delay_ps = std::max(rpt.max_tsv_delay_ps, t.parasitics.delay_ps);
    }

    for (const auto& l : d2d_links_) {
        rpt.total_d2d_bandwidth_gbps += l.bandwidth_gbps();
        rpt.total_d2d_power_mw += l.total_power_mw();
    }

    rpt.max_ir_drop_mv = power_grid_.max_ir_drop();
    rpt.estimated_yield = estimate_yield(yield);

    return rpt;
}

} // namespace sf
