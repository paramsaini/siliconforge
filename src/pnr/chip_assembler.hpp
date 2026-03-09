#pragma once
// SiliconForge — Chip Assembler
// Top-level integration: I/O pads, bump map, seal ring, corner cells.
// Generates the final chip-level layout from core + I/O.

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct IoPad {
    std::string name;
    std::string signal;
    enum Type { SIGNAL, POWER_VDD, POWER_VSS, ANALOG, NC } type;
    enum Side { NORTH, SOUTH, EAST, WEST } side;
    double offset;       // position along the side
    double width = 50;   // um
    double height = 50;
    Point bump_position;
};

struct ChipConfig {
    double core_margin = 100;       // um from core to pad ring
    double pad_pitch = 80;          // um between pads
    double seal_ring_width = 5;     // um
    double bump_pitch = 150;        // um
    std::string package_type = "FCBGA"; // flip-chip BGA
};

struct ChipResult {
    double chip_width = 0, chip_height = 0;
    double chip_area_mm2 = 0;
    int total_pads = 0;
    int signal_pads = 0;
    int power_pads = 0;
    int total_bumps = 0;
    double time_ms = 0;
    std::vector<IoPad> pads;
    std::string message;
};

class ChipAssembler {
public:
    ChipAssembler(const PhysicalDesign& core) : core_(core) {}

    void set_config(const ChipConfig& cfg) { cfg_ = cfg; }

    // Add I/O pad
    void add_pad(const std::string& name, const std::string& signal,
                 IoPad::Type type, IoPad::Side side);

    // Auto-generate power/ground pads
    void auto_power_pads(int vdd_count = 4, int vss_count = 4);

    // Assemble chip
    ChipResult assemble();

    // Generate final PhysicalDesign with everything
    PhysicalDesign to_physical_design() const;

private:
    const PhysicalDesign& core_;
    ChipConfig cfg_;
    std::vector<IoPad> pads_;
    double chip_w_ = 0, chip_h_ = 0;

    void place_pads();
    void generate_seal_ring(PhysicalDesign& pd) const;
    void generate_bump_map();
};

} // namespace sf
