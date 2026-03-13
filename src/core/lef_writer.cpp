// SiliconForge — LEF 5.8 Writer Implementation
#include "core/lef_parser.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>

namespace sf {

// ── LefWriter Implementation ────────────────────────────────────────────────

void LefWriter::write_header(std::ostream& os) {
    os << "VERSION " << lib_.version << " ;\n";
    os << "BUSBITCHARS \"" << lib_.bus_bit_chars << "\" ;\n";
    os << "DIVIDERCHAR \"" << lib_.divider_char << "\" ;\n";
    os << "\n";

    // UNITS section
    os << "UNITS\n";
    os << "  DATABASE MICRONS " << static_cast<int>(lib_.units.database_microns) << " ;\n";
    if (lib_.units.capacitance_pf != 1.0)
        os << "  CAPACITANCE PICOFARADS " << lib_.units.capacitance_pf << " ;\n";
    if (lib_.units.resistance_kohm != 1.0)
        os << "  RESISTANCE KOHMS " << lib_.units.resistance_kohm << " ;\n";
    if (lib_.units.time_ns != 1.0)
        os << "  TIME NANOSECONDS " << lib_.units.time_ns << " ;\n";
    if (lib_.units.power_mw != 1.0)
        os << "  POWER MILLIWATTS " << lib_.units.power_mw << " ;\n";
    os << "END UNITS\n\n";
}

void LefWriter::write_layers(std::ostream& os) {
    for (auto& layer : lib_.layers) {
        os << "LAYER " << layer.name << "\n";

        switch (layer.type) {
            case LefLayer::ROUTING:     os << "  TYPE ROUTING ;\n"; break;
            case LefLayer::CUT:         os << "  TYPE CUT ;\n"; break;
            case LefLayer::MASTERSLICE: os << "  TYPE MASTERSLICE ;\n"; break;
            case LefLayer::OVERLAP:     os << "  TYPE OVERLAP ;\n"; break;
            case LefLayer::IMPLANT:     os << "  TYPE IMPLANT ;\n"; break;
        }

        if (layer.direction != LefLayer::NONE) {
            os << "  DIRECTION "
               << (layer.direction == LefLayer::HORIZONTAL ? "HORIZONTAL" : "VERTICAL")
               << " ;\n";
        }

        if (layer.pitch > 0)
            os << "  PITCH " << std::fixed << std::setprecision(4) << layer.pitch << " ;\n";
        if (layer.width > 0)
            os << "  WIDTH " << layer.width << " ;\n";
        if (layer.spacing > 0)
            os << "  SPACING " << layer.spacing << " ;\n";
        if (layer.offset > 0)
            os << "  OFFSET " << layer.offset << " ;\n";
        if (layer.resistance_per_sq > 0)
            os << "  RESISTANCE RPERSQ " << layer.resistance_per_sq << " ;\n";
        if (layer.capacitance_per_um > 0)
            os << "  CAPACITANCE CPERSQDIST " << layer.capacitance_per_um << " ;\n";
        if (layer.edge_capacitance > 0)
            os << "  EDGECAPACITANCE " << layer.edge_capacitance << " ;\n";
        if (layer.thickness > 0)
            os << "  THICKNESS " << layer.thickness << " ;\n";
        if (layer.area > 0)
            os << "  AREA " << layer.area << " ;\n";

        // Wide-wire spacing table
        for (auto& [w, s] : layer.spacing_table) {
            os << "  SPACINGTABLE " << w << " " << s << " ;\n";
        }

        os << "END " << layer.name << "\n\n";
    }

    // Vias
    for (auto& via : lib_.vias) {
        os << "VIA " << via.name << "\n";
        if (via.resistance > 0)
            os << "  RESISTANCE " << via.resistance << " ;\n";
        for (auto& vl : via.layers) {
            os << "  LAYER " << vl.layer_name << " ;\n";
            for (auto& r : vl.rects) {
                os << "    RECT " << std::fixed << std::setprecision(4)
                   << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << " ;\n";
            }
        }
        os << "END " << via.name << "\n\n";
    }

    // Via rules
    for (auto& vr : lib_.via_rules) {
        os << "VIARULE " << vr.name;
        if (vr.is_generate) os << " GENERATE";
        os << "\n";
        for (auto& rl : vr.layers) {
            os << "  LAYER " << rl.layer_name << " ;\n";
            if (rl.width_lo > 0 || rl.width_hi > 0)
                os << "    WIDTH " << rl.width_lo << " TO " << rl.width_hi << " ;\n";
            if (rl.overhang1 > 0)
                os << "    OVERHANG " << rl.overhang1 << " ;\n";
            if (rl.enc_x > 0 || rl.enc_y > 0)
                os << "    ENCLOSURE " << rl.enc_x << " " << rl.enc_y << " ;\n";
            if (rl.spacing_x > 0 || rl.spacing_y > 0)
                os << "    SPACING " << rl.spacing_x << " BY " << rl.spacing_y << " ;\n";
        }
        os << "END " << vr.name << "\n\n";
    }

    // Sites
    for (auto& site : lib_.sites) {
        os << "SITE " << site.name << "\n";
        os << "  CLASS ";
        switch (site.site_class) {
            case LefSite::CORE: os << "CORE"; break;
            case LefSite::PAD:  os << "PAD"; break;
            case LefSite::IO:   os << "IO"; break;
        }
        os << " ;\n";
        if (site.width > 0 && site.height > 0)
            os << "  SIZE " << site.width << " BY " << site.height << " ;\n";
        os << "END " << site.name << "\n\n";
    }

    // Inter-layer spacings
    for (auto& sp : lib_.spacings) {
        os << "SPACING\n";
        os << "  SAMENET " << sp.layer1 << " " << sp.layer2 << " " << sp.min_spacing;
        if (sp.stack) os << " STACK";
        os << " ;\n";
        os << "END SPACING\n\n";
    }
}

void LefWriter::write_macros(std::ostream& os) {
    auto class_str = [](LefMacro::Class c) -> const char* {
        switch (c) {
            case LefMacro::CORE:              return "CORE";
            case LefMacro::CORE_TIEHIGH:      return "CORE TIEHIGH";
            case LefMacro::CORE_TIELOW:       return "CORE TIELOW";
            case LefMacro::CORE_ANTENNACELL:  return "CORE ANTENNACELL";
            case LefMacro::CORE_WELLTAP:      return "CORE WELLTAP";
            case LefMacro::PAD:               return "PAD";
            case LefMacro::PAD_INPUT:         return "PAD INPUT";
            case LefMacro::PAD_OUTPUT:        return "PAD OUTPUT";
            case LefMacro::PAD_INOUT:         return "PAD INOUT";
            case LefMacro::PAD_POWER:         return "PAD POWER";
            case LefMacro::PAD_SPACER:        return "PAD SPACER";
            case LefMacro::PAD_AREAIO:        return "PAD AREAIO";
            case LefMacro::BLOCK:             return "BLOCK";
            case LefMacro::ENDCAP_PRE:        return "ENDCAP PRE";
            case LefMacro::ENDCAP_POST:       return "ENDCAP POST";
            case LefMacro::RING:              return "RING";
            case LefMacro::COVER:             return "COVER";
        }
        return "CORE";
    };

    auto pin_dir_str = [](LefPin::Direction d) -> const char* {
        switch (d) {
            case LefPin::INPUT:   return "INPUT";
            case LefPin::OUTPUT:  return "OUTPUT";
            case LefPin::INOUT:   return "INOUT";
            case LefPin::FEEDTHRU: return "FEEDTHRU";
        }
        return "INOUT";
    };

    auto pin_use_str = [](LefPin::Use u) -> const char* {
        switch (u) {
            case LefPin::SIGNAL: return "SIGNAL";
            case LefPin::POWER:  return "POWER";
            case LefPin::GROUND: return "GROUND";
            case LefPin::CLOCK:  return "CLOCK";
            case LefPin::ANALOG: return "ANALOG";
        }
        return "SIGNAL";
    };

    for (auto& macro : lib_.macros) {
        os << "MACRO " << macro.name << "\n";
        os << "  CLASS " << class_str(macro.macro_class) << " ;\n";

        if (macro.origin_x != 0 || macro.origin_y != 0)
            os << "  ORIGIN " << macro.origin_x << " " << macro.origin_y << " ;\n";

        if (macro.width > 0 && macro.height > 0)
            os << "  SIZE " << std::fixed << std::setprecision(4)
               << macro.width << " BY " << macro.height << " ;\n";

        if (!macro.symmetry.empty())
            os << "  SYMMETRY " << macro.symmetry << " ;\n";

        if (!macro.source.empty())
            os << "  SOURCE " << macro.source << " ;\n";

        // Pins
        for (auto& pin : macro.pins) {
            os << "  PIN " << pin.name << "\n";
            os << "    DIRECTION " << pin_dir_str(pin.direction) << " ;\n";
            if (pin.use != LefPin::SIGNAL)
                os << "    USE " << pin_use_str(pin.use) << " ;\n";
            if (!pin.shape.empty())
                os << "    SHAPE " << pin.shape << " ;\n";

            for (auto& port : pin.ports) {
                os << "    PORT\n";
                os << "      LAYER " << port.layer << " ;\n";
                for (auto& r : port.rects) {
                    os << "      RECT " << std::fixed << std::setprecision(4)
                       << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << " ;\n";
                }
                os << "    END\n";
            }
            os << "  END " << pin.name << "\n";
        }

        // Obstructions
        if (!macro.obs.layers.empty()) {
            os << "  OBS\n";
            for (auto& ol : macro.obs.layers) {
                os << "    LAYER " << ol.layer << " ;\n";
                for (auto& r : ol.rects) {
                    os << "      RECT " << std::fixed << std::setprecision(4)
                       << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << " ;\n";
                }
            }
            os << "  END\n";
        }

        os << "END " << macro.name << "\n\n";
    }
}

std::string LefWriter::to_string() {
    std::ostringstream oss;
    write_header(oss);
    write_layers(oss);
    write_macros(oss);
    oss << "END LIBRARY\n";
    return oss.str();
}

bool LefWriter::write(const std::string& filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) return false;

    write_header(ofs);
    write_layers(ofs);
    write_macros(ofs);
    ofs << "END LIBRARY\n";

    return ofs.good();
}

} // namespace sf
