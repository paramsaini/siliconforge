#pragma once
// SiliconForge — SPEF Parasitic Extraction Engine
// Extracts RC parasitics from routed physical design and produces SpefData
// suitable for STA back-annotation.
//
// Models:
//   Resistance  — R = ρ·L / (W·t)
//   Ground cap  — C = ε·W·L/h  +  2·fringe (parallel-plate + fringing)
//   Coupling    — C_couple = ε·L_overlap·t / spacing
//
// References:
//   - Cong et al., "Interconnect Estimation and Planning", TCAD 2001
//   - Sapatnekar, "Timing", Springer 2004, Ch. 7 (Interconnect Parasitics)

#include "core/spef_parser.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

// ── Layer stack entry ────────────────────────────────────────────────────

struct LayerStackEntry {
    int    layer_id      = 0;
    double thickness_um  = 0.3;    // metal thickness
    double spacing_um    = 0.14;   // minimum spacing to neighbor
    double pitch_um      = 0.28;   // routing pitch
};

// ── Extraction mode ──────────────────────────────────────────────────────

enum class ExtractionMode {
    LUMPED,       // single R + C per net
    DISTRIBUTED,  // RC-π per wire segment
    COUPLED       // distributed + inter-net coupling caps
};

// ── Configuration ────────────────────────────────────────────────────────

struct ParasiticExtractConfig {
    // Process parameters
    double epsilon_r          = 3.9;       // relative permittivity (SiO2)
    double metal_resistivity  = 2.65e-8;   // Ω·m (copper)
    double substrate_height_um = 10.0;     // substrate to M1 height
    double fringe_cap_per_um  = 0.02;      // fF/μm fringe capacitance

    // Per-layer ILD thickness (μm); index = layer id
    std::vector<double> ild_thickness_um;  // if empty, use default 0.5 μm

    // Extraction mode
    ExtractionMode mode = ExtractionMode::DISTRIBUTED;

    // Coupling parameters
    double coupling_threshold = 0.001;     // minimum coupling cap to report (pF)

    // Layer stack description
    std::vector<LayerStackEntry> layer_stack;

    // Distributed-mode segment count per wire
    int    pi_segments = 5;

    // Via parasitics
    double via_resistance_ohm = 5.0;
    double via_cap_ff         = 0.5;

    // Helper: get ILD thickness for a given layer
    double ild_for_layer(int layer) const {
        if (layer >= 0 && layer < static_cast<int>(ild_thickness_um.size()))
            return ild_thickness_um[layer];
        return 0.5; // default ILD
    }

    // Helper: get layer stack entry (returns default if missing)
    LayerStackEntry stack_for_layer(int layer) const {
        for (auto& e : layer_stack)
            if (e.layer_id == layer) return e;
        return {};
    }
};

// ── SPEF Parasitic Extractor ─────────────────────────────────────────────

class SpefExtractor {
public:
    explicit SpefExtractor(const PhysicalDesign& pd);

    void set_config(const ParasiticExtractConfig& cfg) { cfg_ = cfg; }
    const ParasiticExtractConfig& config() const { return cfg_; }

    // Main extraction — produces a complete SpefData structure
    SpefData extract();

    // Per-wire computations (public for unit testing)
    double compute_wire_resistance(const WireSegment& seg) const;
    double compute_wire_cap_to_ground(const WireSegment& seg) const;
    double compute_coupling_cap(const WireSegment& a, const WireSegment& b) const;

private:
    const PhysicalDesign& pd_;
    ParasiticExtractConfig cfg_;

    // Vacuum permittivity (F/m)
    static constexpr double EPSILON_0 = 8.854187817e-12;

    // Build net-to-wire index
    using WireIndex = std::vector<std::vector<int>>; // net_id → [wire indices]
    WireIndex build_wire_index() const;

    // Extract a single net in LUMPED mode
    SpefNet extract_lumped(int net_id, const std::vector<int>& wire_ids) const;

    // Extract a single net in DISTRIBUTED mode (RC-π segments)
    SpefNet extract_distributed(int net_id, const std::vector<int>& wire_ids) const;

    // Append coupling capacitors between nets
    void extract_coupling_caps(const WireIndex& idx, std::vector<SpefNet>& nets) const;

    // Wire geometry helpers
    static double wire_length_um(const WireSegment& seg);
    double metal_thickness(int layer) const;
    double ild_height(int layer) const;
    double layer_spacing(int layer) const;

    // Compute overlap length between two wires on the same layer
    static double parallel_overlap_um(const WireSegment& a, const WireSegment& b);
    static double perpendicular_distance(const WireSegment& a, const WireSegment& b);
};

} // namespace sf
