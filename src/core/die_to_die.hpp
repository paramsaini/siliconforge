#pragma once
// SiliconForge — 3D-IC & Multi-Die Packaging Definitions
// Phase 40: Industrial chiplet/multi-die integration with signal integrity

#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <algorithm>
#include <numeric>

namespace sf {

// ── TSV Electrical Model (physics-based RLC) ────────────────────────────────
// Based on: Katti et al., "Electrical Modeling and Characterization of TSVs"
// IEEE T-CPMT, 2010; Pak et al., "Modeling of TSV Parasitics", ECTC 2011
struct TsvParasitics {
    double resistance = 0;    // Ohms — ρ·h/(π·r²)
    double inductance = 0;    // Henries — μ₀·h/(2π)·[ln(2h/r)-1]
    double capacitance = 0;   // Farads — 2π·ε₀·εᵣ·h/ln(r_ox/r_tsv)
    double coupling_cap = 0;  // Farads — coupling to nearest neighbor TSV
    double delay_ps = 0;      // Elmore delay approximation: R·C (ps)
    double bandwidth_gbps = 0;// BW = 1/(2·π·R·C) converted to Gbps
};

struct TsvTechParams {
    double radius_um = 2.5;       // TSV copper radius (μm)
    double height_um = 50.0;      // TSV height / wafer thickness (μm)
    double oxide_thick_um = 0.5;  // SiO₂ liner thickness (μm)
    double pitch_um = 10.0;       // TSV-to-TSV pitch (μm)
    double cu_resistivity = 1.68e-8; // Ω·m (copper at 20°C)
    double oxide_er = 3.9;        // SiO₂ relative permittivity
    double si_er = 11.7;          // Silicon relative permittivity
    double mu0 = 1.2566e-6;       // Permeability of free space (H/m)
    double eps0 = 8.854e-12;      // Permittivity of free space (F/m)
    double max_current_ma = 20.0; // EM current limit per TSV (mA)

    TsvParasitics compute_parasitics() const;
    TsvParasitics compute_coupling(double neighbor_dist_um) const;
};

// ── Micro-bump Model ────────────────────────────────────────────────────────
struct MicroBump {
    int id = -1;
    int net_id = -1;
    int die_top = -1;      // die above
    int die_bottom = -1;   // die below
    float x = 0, y = 0;
    double contact_resistance = 0.015; // Ohms (typical Cu pillar)
    double inductance_ph = 5.0;        // pH
    double capacitance_ff = 20.0;      // fF
    double max_current_ma = 150.0;     // mA (EM limit)
    bool is_power = false;             // VDD/GND bump
    bool is_spare = false;             // redundancy spare
};

struct MicroBumpTech {
    double pitch_um = 40.0;         // bump pitch
    double diameter_um = 25.0;      // bump diameter
    double contact_r_ohm = 0.015;   // contact resistance
    double inductance_ph = 5.0;     // pH
    double capacitance_ff = 20.0;   // fF
    double max_current_ma = 150.0;  // EM limit
};

// ── Interposer / RDL Model ──────────────────────────────────────────────────
struct InterposerTrace {
    int id = -1;
    int net_id = -1;
    int layer = 0;          // RDL layer
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    double width_um = 2.0;
    double r_per_um = 0.025;  // Ω/μm
    double c_per_um = 0.15;   // fF/μm

    double length_um() const {
        return std::abs(x1 - x0) + std::abs(y1 - y0);
    }
    double resistance() const { return r_per_um * length_um(); }
    double capacitance_ff() const { return c_per_um * length_um(); }
};

struct InterposerConfig {
    double width_um = 20000;   // interposer width
    double height_um = 20000;  // interposer height
    int num_rdl_layers = 4;    // redistribution layers
    double trace_r_per_um = 0.025;
    double trace_c_per_um = 0.15;
    double trace_width_um = 2.0;
    double trace_pitch_um = 4.0;
};

// ── Die-to-Die Link (UCIe-inspired protocol) ───────────────────────────────
struct D2DLink {
    int id = -1;
    int from_die = -1;
    int to_die = -1;
    int num_lanes = 16;           // data lanes
    double lane_rate_gbps = 32.0; // per-lane data rate
    double latency_ns = 2.0;      // link latency
    double power_mw_per_lane = 0.5;

    double bandwidth_gbps() const { return num_lanes * lane_rate_gbps; }
    double total_power_mw() const { return num_lanes * power_mw_per_lane; }
};

// ── 3D Power Network ────────────────────────────────────────────────────────
struct PowerTSV {
    int id = -1;
    float x = 0, y = 0;
    bool is_vdd = true;         // true=VDD, false=GND
    double resistance = 0.005;  // Ohms (low-R for power)
    double current_ma = 0;      // operating current
};

struct PowerGrid3D {
    int grid_nx = 16, grid_ny = 16;
    int num_dies = 2;
    std::vector<PowerTSV> power_tsvs;
    std::vector<std::vector<double>> ir_drop_map; // per die, per grid point

    double max_ir_drop() const;
    double avg_ir_drop() const;
    int hotspot_count(double threshold_mv = 50.0) const;
};

// ── Yield & Redundancy ──────────────────────────────────────────────────────
struct YieldModel {
    double tsv_defect_rate = 1e-5;    // defects per TSV
    double bump_defect_rate = 5e-6;   // defects per bump
    double die_defect_density = 0.5;  // defects/cm² (Poisson)
    double die_area_cm2 = 1.0;

    double single_die_yield() const;  // Y = e^(-D·A)
    double tsv_array_yield(int num_tsvs, int spares = 0) const;
    double bump_array_yield(int num_bumps, int spares = 0) const;
    double package_yield(int num_dies, int tsvs_per_die, int spare_tsvs = 0) const;
};

// ── Represents a vertical die stack or a 2.5D interposer layout ─────────────
struct DieInstance {
    int id;
    std::string name;
    int z_layer = 0;       // 0 = bottom, 1 = layer above, etc.
    float x_offset = 0;    // For 2.5D interposers
    float y_offset = 0;
    float width_um = 10000;
    float height_um = 10000;
    double power_w = 1.0;  // die power consumption
    double temperature_c = 85.0;
};

// Represents a Through-Silicon Via connecting two adjacent Z-layers
struct TSV {
    int id;
    int net_id;
    int from_die_id;
    int to_die_id;
    float x;
    float y;
    bool is_signal = true;  // false = power/ground
    bool is_spare = false;  // redundancy spare
    TsvParasitics parasitics;
};

// ── Chiplet Design Report ────────────────────────────────────────────────────
struct ChipletReport {
    int total_dies = 0;
    int total_tsvs = 0;
    int signal_tsvs = 0;
    int power_tsvs = 0;
    int spare_tsvs = 0;
    int total_bumps = 0;
    int total_d2d_links = 0;
    double total_tsv_resistance = 0;
    double total_tsv_capacitance_ff = 0;
    double max_tsv_delay_ps = 0;
    double total_d2d_bandwidth_gbps = 0;
    double total_d2d_power_mw = 0;
    double max_ir_drop_mv = 0;
    double estimated_yield = 0;
};

class PackageDesign {
public:
    // Die management
    int add_die(const std::string& name, int z_layer, float x = 0, float y = 0);
    void set_die_size(int die_id, float w_um, float h_um);
    void set_die_power(int die_id, double power_w);

    // TSV management (signal + power)
    int add_tsv(int net_id, int from_die, int to_die, float x, float y);
    int add_signal_tsv(int net_id, int from_die, int to_die, float x, float y,
                       const TsvTechParams& tech = {});
    int add_power_tsv(bool is_vdd, int from_die, int to_die, float x, float y,
                      const TsvTechParams& tech = {});
    int add_spare_tsv(int from_die, int to_die, float x, float y,
                      const TsvTechParams& tech = {});

    // Micro-bump management
    int add_micro_bump(int net_id, int die_top, int die_bottom, float x, float y,
                       const MicroBumpTech& tech = {});
    int add_power_bump(bool is_vdd, int die_top, int die_bottom, float x, float y,
                       const MicroBumpTech& tech = {});

    // Interposer traces
    int add_interposer_trace(int net_id, int layer, float x0, float y0,
                             float x1, float y1, const InterposerConfig& cfg = {});

    // Die-to-die links
    int add_d2d_link(int from_die, int to_die, int lanes = 16, double rate_gbps = 32.0);

    // 3D Power network
    void build_power_grid(int nx, int ny, const TsvTechParams& tech = {});
    double compute_ir_drop(int die_id) const;

    // Yield analysis
    double estimate_yield(const YieldModel& model = {}) const;

    // Signal integrity analysis
    TsvParasitics compute_tsv_si(int tsv_id, const TsvTechParams& tech = {}) const;
    double worst_case_tsv_delay_ps() const;
    double total_d2d_bandwidth_gbps() const;

    // Redundancy
    int assign_spare_tsv(int failed_tsv_id);
    int count_spare_tsvs() const;

    // Full chiplet report
    ChipletReport generate_report(const TsvTechParams& tsv_tech = {},
                                  const YieldModel& yield = {}) const;

    // Accessors
    const std::vector<DieInstance>& dies() const { return dies_; }
    const std::vector<TSV>& tsvs() const { return tsvs_; }
    const std::vector<MicroBump>& bumps() const { return bumps_; }
    const std::vector<InterposerTrace>& traces() const { return traces_; }
    const std::vector<D2DLink>& d2d_links() const { return d2d_links_; }
    const PowerGrid3D& power_grid() const { return power_grid_; }

private:
    std::vector<DieInstance> dies_;
    std::vector<TSV> tsvs_;
    std::vector<MicroBump> bumps_;
    std::vector<InterposerTrace> traces_;
    std::vector<D2DLink> d2d_links_;
    PowerGrid3D power_grid_;
    int die_counter_ = 0;
    int tsv_counter_ = 0;
    int bump_counter_ = 0;
    int trace_counter_ = 0;
    int link_counter_ = 0;
};

} // namespace sf
