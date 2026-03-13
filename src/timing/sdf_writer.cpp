// SiliconForge — SDF Writer Implementation
// IEEE 1497 Standard Delay Format output
#include "timing/sdf_writer.hpp"
#include <fstream>
#include <sstream>
#include <ctime>
#include <iomanip>

namespace sf {

// Gate type to delay estimate (ns)
static double gate_type_delay(GateType t, int num_inputs) {
    switch (t) {
        case GateType::BUF:    return 0.02;
        case GateType::NOT:    return 0.02;
        case GateType::AND:    return 0.10 * std::max(1, num_inputs);
        case GateType::OR:     return 0.12 * std::max(1, num_inputs);
        case GateType::NAND:   return 0.08 * std::max(1, num_inputs);
        case GateType::NOR:    return 0.09 * std::max(1, num_inputs);
        case GateType::XOR:    return 0.15;
        case GateType::XNOR:   return 0.15;
        case GateType::MUX:    return 0.12;
        case GateType::DFF:    return 0.10;
        case GateType::DLATCH: return 0.08;
        default:               return 0.05;
    }
}

std::string SdfWriter::cell_delays(const SdfConfig& cfg) {
    std::ostringstream ss;
    double ts = cfg.timescale;

    for (size_t i = 0; i < nl_.num_gates(); ++i) {
        auto& g = nl_.gate(static_cast<GateId>(i));
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT ||
            g.type == GateType::CONST0 || g.type == GateType::CONST1)
            continue;

        std::string cell_type = gate_type_str(g.type);
        std::string inst_name = g.name.empty() ?
            (cell_type + "_" + std::to_string(i)) : g.name;

        double delay = gate_type_delay(g.type, (int)g.inputs.size()) / ts;
        // Rise and fall with ±5% variation
        double rise = delay * 1.0;
        double fall = delay * 1.05;

        ss << "    (CELL\n";
        ss << "      (CELLTYPE \"" << cell_type << "\")\n";
        ss << "      (INSTANCE " << inst_name << ")\n";
        ss << "      (DELAY\n";
        ss << "        (ABSOLUTE\n";

        // Generate IOPATH for each input to output
        for (size_t pi = 0; pi < g.inputs.size(); ++pi) {
            std::string in_pin = "IN" + std::to_string(pi);
            std::string out_pin = "OUT";

            if (g.type == GateType::DFF) {
                if (pi == 0) in_pin = "D";
                out_pin = "Q";
            }

            ss << "          (IOPATH " << in_pin << " " << out_pin
               << " (" << std::fixed << std::setprecision(4)
               << rise << ") (" << fall << "))\n";
        }

        ss << "        )\n";  // ABSOLUTE
        ss << "      )\n";    // DELAY
        ss << "    )\n";      // CELL
    }

    return ss.str();
}

std::string SdfWriter::interconnect_delays(const SdfConfig& cfg) {
    if (!cfg.include_interconnect) return "";

    std::ostringstream ss;
    double ts = cfg.timescale;

    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        auto& net = nl_.net(static_cast<NetId>(i));
        if (net.driver < 0 || net.fanout.empty()) continue;

        auto& drv = nl_.gate(net.driver);
        if (drv.type == GateType::INPUT || drv.type == GateType::CONST0 ||
            drv.type == GateType::CONST1) continue;

        std::string drv_name = drv.name.empty() ?
            (std::string(gate_type_str(drv.type)) + "_" + std::to_string(drv.id)) : drv.name;
        std::string drv_pin = (drv.type == GateType::DFF) ? "Q" : "OUT";

        // Estimate wire delay based on fanout
        double wire_delay = 0.001 * net.fanout.size() / ts;

        for (auto gid : net.fanout) {
            auto& sink = nl_.gate(gid);
            std::string sink_name = sink.name.empty() ?
                (std::string(gate_type_str(sink.type)) + "_" + std::to_string(sink.id)) : sink.name;

            // Find which input pin
            std::string sink_pin = "IN0";
            for (size_t pi = 0; pi < sink.inputs.size(); ++pi) {
                if (sink.inputs[pi] == static_cast<NetId>(i)) {
                    if (sink.type == GateType::DFF && pi == 0)
                        sink_pin = "D";
                    else
                        sink_pin = "IN" + std::to_string(pi);
                    break;
                }
            }

            ss << "    (INTERCONNECT " << drv_name << "/" << drv_pin
               << " " << sink_name << "/" << sink_pin
               << " (" << std::fixed << std::setprecision(4) << wire_delay
               << ") (" << wire_delay << "))\n";
        }
    }

    return ss.str();
}

std::string SdfWriter::generate(const SdfConfig& cfg) {
    std::ostringstream ss;

    // Timestamp
    std::time_t now = std::time(nullptr);
    char time_buf[64];
    std::strftime(time_buf, sizeof(time_buf), "%a %b %d %H:%M:%S %Y",
                  std::localtime(&now));

    // Header
    ss << "(DELAYFILE\n";
    ss << "  (SDFVERSION \"4.0\")\n";
    ss << "  (DESIGN \"" << cfg.design_name << "\")\n";
    ss << "  (DATE \"" << time_buf << "\")\n";
    ss << "  (VENDOR \"SiliconForge\")\n";
    ss << "  (PROGRAM \"SiliconForge SDF Writer\")\n";
    ss << "  (VERSION \"1.0\")\n";
    ss << "  (DIVIDER /)\n";
    ss << "  (TIMESCALE " << std::fixed << std::setprecision(1)
       << cfg.timescale << "ns)\n";

    // Cell delays
    ss << cell_delays(cfg);

    // Interconnect delays
    if (cfg.include_interconnect) {
        ss << interconnect_delays(cfg);
    }

    ss << ")\n";  // DELAYFILE
    return ss.str();
}

bool SdfWriter::write(const std::string& filename, const SdfConfig& cfg) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) return false;
    ofs << generate(cfg);
    return ofs.good();
}

} // namespace sf
