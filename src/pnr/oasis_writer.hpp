#pragma once
// SiliconForge — OASIS Writer
// Generates simplified OASIS (SEMI P39) format from the physical design.
// Uses ASCII representation capturing cell placements, geometry, and routing.

#include "pnr/physical.hpp"
#include <string>

namespace sf {

class OasisWriter {
public:
    explicit OasisWriter(const PhysicalDesign& pd) : pd_(pd) {}

    bool write(const std::string& filename);
    std::string to_string(); // for testing

private:
    const PhysicalDesign& pd_;

    void write_header(std::ostream& os);
    void write_cells(std::ostream& os);
    void write_geometry(std::ostream& os);
};

} // namespace sf
