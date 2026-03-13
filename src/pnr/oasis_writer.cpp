// SiliconForge — OASIS Writer Implementation
// Simplified ASCII OASIS-like format (SEMI P39 compatible structure)
#include "pnr/oasis_writer.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>

namespace sf {

void OasisWriter::write_header(std::ostream& os) {
    os << "% SEMI-OASIS\n";
    os << "VERSION 1.0\n";
    os << "UNIT MICRON\n";
    os << "RESOLUTION 1000\n";
    os << "DIE_AREA " << std::fixed << std::setprecision(3)
       << pd_.die_area.x0 << " " << pd_.die_area.y0 << " "
       << pd_.die_area.x1 << " " << pd_.die_area.y1 << "\n";
    os << "\n";
}

void OasisWriter::write_cells(std::ostream& os) {
    os << "# Cell Placements\n";
    for (auto& cell : pd_.cells) {
        os << "CELL " << cell.name << "\n";
        os << "  TYPE " << cell.cell_type << "\n";
        os << "  PLACEMENT " << std::fixed << std::setprecision(3)
           << cell.position.x << " " << cell.position.y << "\n";
        os << "  SIZE " << cell.width << " " << cell.height << "\n";
        os << "  ORIENT " << cell.orientation << "\n";
        // Cell bounding box as RECTANGLE
        os << "  RECTANGLE 0 "
           << cell.position.x << " " << cell.position.y << " "
           << cell.position.x + cell.width << " " << cell.position.y + cell.height << "\n";
        os << "END_CELL\n";
    }
    os << "\n";
}

void OasisWriter::write_geometry(std::ostream& os) {
    // Write routing as PATH records
    os << "# Routing Geometry\n";
    for (auto& wire : pd_.wires) {
        os << "PATH " << wire.layer << " " << std::fixed << std::setprecision(3)
           << wire.width << " "
           << wire.start.x << " " << wire.start.y << " "
           << wire.end.x << " " << wire.end.y;
        if (wire.net_id >= 0) os << " NET " << wire.net_id;
        os << "\n";
    }

    // Write vias
    os << "# Vias\n";
    for (auto& via : pd_.vias) {
        os << "VIA " << std::fixed << std::setprecision(3)
           << via.position.x << " " << via.position.y
           << " " << via.lower_layer << " " << via.upper_layer << "\n";
    }
}

std::string OasisWriter::to_string() {
    std::ostringstream oss;
    write_header(oss);
    write_cells(oss);
    write_geometry(oss);
    oss << "END_OASIS\n";
    return oss.str();
}

bool OasisWriter::write(const std::string& filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) return false;

    write_header(ofs);
    write_cells(ofs);
    write_geometry(ofs);
    ofs << "END_OASIS\n";

    return ofs.good();
}

} // namespace sf
