// SiliconForge — SDF Writer Implementation
// IEEE 1497 Standard Delay Format with min:typ:max triplet support
#include "timing/sdf_writer.hpp"
#include <fstream>
#include <sstream>
#include <ctime>
#include <iomanip>

namespace sf {

// Gate type to delay estimate (ns) — used when no Liberty library available
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

std::string SdfWriter::fmt_delay(double typ, const SdfConfig& cfg) const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);
    if (cfg.emit_triplets) {
        double mn = typ * cfg.min_factor;
        double mx = typ * cfg.max_factor;
        ss << "(" << mn << ":" << typ << ":" << mx << ")";
    } else {
        ss << "(" << typ << ")";
    }
    return ss.str();
}

double SdfWriter::nldm_delay(const std::string& cell_name, double slew, double load) const {
    // This would do real NLDM lookup — returns 0 if not found
    return 0.0;
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

        // Try NLDM lookup if library provided
        double delay = 0;
        if (cfg.lib) {
            delay = nldm_delay(cell_type, cfg.input_slew, cfg.load_cap);
        }
        if (delay <= 0) {
            delay = gate_type_delay(g.type, (int)g.inputs.size());
        }
        delay /= ts;

        // Rise and fall with ±5% variation
        double rise = delay * 1.0;
        double fall = delay * 1.05;

        ss << "    (CELL\n";
        ss << "      (CELLTYPE \"" << cell_type << "\")\n";
        ss << "      (INSTANCE " << inst_name << ")\n";
        ss << "      (DELAY\n";
        ss << "        (ABSOLUTE\n";

        for (size_t pi = 0; pi < g.inputs.size(); ++pi) {
            std::string in_pin = "IN" + std::to_string(pi);
            std::string out_pin = "OUT";

            if (g.type == GateType::DFF) {
                if (pi == 0) in_pin = "D";
                out_pin = "Q";
            }

            ss << "          (IOPATH " << in_pin << " " << out_pin
               << " " << fmt_delay(rise, cfg)
               << " " << fmt_delay(fall, cfg) << ")\n";
        }

        ss << "        )\n";  // ABSOLUTE
        ss << "      )\n";    // DELAY

        // Conditional delays
        if (!cfg.cond_delays.empty()) {
            for (auto& cd : cfg.cond_delays) {
                ss << "      (DELAY\n";
                ss << "        (ABSOLUTE\n";
                for (size_t pi = 0; pi < g.inputs.size(); ++pi) {
                    std::string in_pin = "IN" + std::to_string(pi);
                    std::string out_pin = "OUT";
                    if (g.type == GateType::DFF) {
                        if (pi == 0) in_pin = "D";
                        out_pin = "Q";
                    }
                    ss << "          (COND " << cd.condition
                       << " (IOPATH " << in_pin << " " << out_pin
                       << " " << fmt_delay(cd.delay_rise / ts, cfg)
                       << " " << fmt_delay(cd.delay_fall / ts, cfg) << "))\n";
                }
                ss << "        )\n";  // ABSOLUTE
                ss << "      )\n";    // DELAY
            }
        }

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

        double wire_delay = 0.001 * net.fanout.size() / ts;

        for (auto gid : net.fanout) {
            auto& sink = nl_.gate(gid);
            std::string sink_name = sink.name.empty() ?
                (std::string(gate_type_str(sink.type)) + "_" + std::to_string(sink.id)) : sink.name;

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
               << " " << fmt_delay(wire_delay, cfg)
               << " " << fmt_delay(wire_delay, cfg) << ")\n";
        }
    }

    return ss.str();
}

std::string SdfWriter::generate(const SdfConfig& cfg) {
    std::ostringstream ss;

    std::time_t now = std::time(nullptr);
    char time_buf[64];
    std::strftime(time_buf, sizeof(time_buf), "%a %b %d %H:%M:%S %Y",
                  std::localtime(&now));

    ss << "(DELAYFILE\n";
    ss << "  (SDFVERSION \"4.0\")\n";
    ss << "  (DESIGN \"" << cfg.design_name << "\")\n";
    ss << "  (DATE \"" << time_buf << "\")\n";
    ss << "  (VENDOR \"SiliconForge\")\n";
    ss << "  (PROGRAM \"SiliconForge SDF Writer\")\n";
    ss << "  (VERSION \"2.0\")\n";
    ss << "  (DIVIDER /)\n";
    ss << "  (TIMESCALE " << std::fixed << std::setprecision(1)
       << cfg.timescale << "ns)\n";

    ss << cell_delays(cfg);

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

// ---------------------------------------------------------------------------
// SdfReader — conditional SDF back-annotation reader
// ---------------------------------------------------------------------------

static std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

static double parse_delay_value(const std::string& tok) {
    std::string s = trim(tok);
    if (s.empty()) return 0.0;
    // Strip outer parentheses if present
    if (s.front() == '(' && s.back() == ')') s = s.substr(1, s.size() - 2);
    s = trim(s);
    // min:typ:max — take typ (middle) value
    size_t c1 = s.find(':');
    if (c1 != std::string::npos) {
        size_t c2 = s.find(':', c1 + 1);
        if (c2 != std::string::npos) {
            std::string typ = trim(s.substr(c1 + 1, c2 - c1 - 1));
            if (!typ.empty()) return std::stod(typ);
        }
    }
    if (!s.empty()) return std::stod(s);
    return 0.0;
}

static size_t find_matching_paren(const std::string& s, size_t open_pos) {
    int depth = 0;
    for (size_t i = open_pos; i < s.size(); ++i) {
        if (s[i] == '(') ++depth;
        else if (s[i] == ')') { if (--depth == 0) return i; }
    }
    return std::string::npos;
}

static std::string extract_quoted(const std::string& s, size_t start) {
    size_t q1 = s.find('"', start);
    if (q1 == std::string::npos) return "";
    size_t q2 = s.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return s.substr(q1 + 1, q2 - q1 - 1);
}

// Extract two delay tokens after IOPATH <in> <out> <rise> <fall>
static bool extract_iopath_delays(const std::string& block, size_t iopath_pos,
                                  double& rise, double& fall) {
    // Advance past "IOPATH"
    size_t p = iopath_pos + 6;
    // Skip whitespace then input pin name
    while (p < block.size() && std::isspace(static_cast<unsigned char>(block[p]))) ++p;
    while (p < block.size() && !std::isspace(static_cast<unsigned char>(block[p])) && block[p] != '(') ++p;
    // Skip whitespace then output pin name
    while (p < block.size() && std::isspace(static_cast<unsigned char>(block[p]))) ++p;
    while (p < block.size() && !std::isspace(static_cast<unsigned char>(block[p])) && block[p] != '(') ++p;

    // Now extract rise delay token
    while (p < block.size() && std::isspace(static_cast<unsigned char>(block[p]))) ++p;
    if (p >= block.size()) return false;

    auto extract_next_delay = [&](double& val) -> bool {
        while (p < block.size() && std::isspace(static_cast<unsigned char>(block[p]))) ++p;
        if (p >= block.size()) return false;
        if (block[p] == '(') {
            size_t close = find_matching_paren(block, p);
            if (close == std::string::npos) return false;
            val = parse_delay_value(block.substr(p, close - p + 1));
            p = close + 1;
        } else {
            size_t end = p;
            while (end < block.size() && !std::isspace(static_cast<unsigned char>(block[end]))
                   && block[end] != ')') ++end;
            val = parse_delay_value(block.substr(p, end - p));
            p = end;
        }
        return true;
    };

    return extract_next_delay(rise) && extract_next_delay(fall);
}

bool SdfReader::parse(const std::string& sdf_content) {
    delays_.clear();
    size_t pos = 0;

    while (pos < sdf_content.size()) {
        size_t cell_kw = sdf_content.find("(CELL", pos);
        if (cell_kw == std::string::npos) break;

        size_t cell_end = find_matching_paren(sdf_content, cell_kw);
        if (cell_end == std::string::npos) break;

        std::string cell_block = sdf_content.substr(cell_kw, cell_end - cell_kw + 1);

        SdfCellDelay cd;

        // Extract CELLTYPE
        size_t ct = cell_block.find("CELLTYPE");
        if (ct != std::string::npos) cd.cell_type = extract_quoted(cell_block, ct);

        // Extract INSTANCE
        size_t inst = cell_block.find("INSTANCE");
        if (inst != std::string::npos) {
            size_t p = inst + 8;
            while (p < cell_block.size() && std::isspace(static_cast<unsigned char>(cell_block[p]))) ++p;
            size_t end = p;
            while (end < cell_block.size() && !std::isspace(static_cast<unsigned char>(cell_block[end]))
                   && cell_block[end] != ')') ++end;
            cd.instance = cell_block.substr(p, end - p);
        }

        // Scan all DELAY/ABSOLUTE blocks within this CELL
        size_t scan = 0;
        while (scan < cell_block.size()) {
            size_t abs_kw = cell_block.find("(ABSOLUTE", scan);
            if (abs_kw == std::string::npos) break;

            size_t abs_end = find_matching_paren(cell_block, abs_kw);
            if (abs_end == std::string::npos) break;

            std::string abs_block = cell_block.substr(abs_kw, abs_end - abs_kw + 1);

            // Look for COND entries
            size_t cscan = 0;
            while (cscan < abs_block.size()) {
                size_t cond_kw = abs_block.find("(COND ", cscan);
                if (cond_kw == std::string::npos) break;

                size_t cond_end = find_matching_paren(abs_block, cond_kw);
                if (cond_end == std::string::npos) break;

                std::string cond_block = abs_block.substr(cond_kw, cond_end - cond_kw + 1);

                // Condition string sits between "(COND " and "(IOPATH"
                size_t io_in_cond = cond_block.find("(IOPATH");
                if (io_in_cond != std::string::npos) {
                    std::string cond_str = trim(cond_block.substr(6, io_in_cond - 6));
                    double rise = 0.0, fall = 0.0;
                    extract_iopath_delays(cond_block, io_in_cond + 1, rise, fall);
                    cd.conditional.push_back({cond_str, rise, fall});
                }
                cscan = cond_end + 1;
            }

            // Look for bare IOPATH (unconditional — not inside COND)
            size_t uscan = 0;
            while (uscan < abs_block.size()) {
                size_t io_kw = abs_block.find("(IOPATH", uscan);
                if (io_kw == std::string::npos) break;

                // Check this IOPATH is not nested inside a COND
                bool inside_cond = false;
                size_t ccheck = abs_block.rfind("(COND", io_kw);
                if (ccheck != std::string::npos) {
                    size_t ccheck_end = find_matching_paren(abs_block, ccheck);
                    if (ccheck_end != std::string::npos && ccheck_end >= io_kw)
                        inside_cond = true;
                }

                if (!inside_cond) {
                    size_t io_end = find_matching_paren(abs_block, io_kw);
                    if (io_end != std::string::npos) {
                        std::string io_block = abs_block.substr(io_kw, io_end - io_kw + 1);
                        double rise = 0.0, fall = 0.0;
                        extract_iopath_delays(io_block, 1, rise, fall);
                        cd.unconditional.push_back({"", rise, fall});
                    }
                }

                size_t io_end = find_matching_paren(abs_block, io_kw);
                uscan = (io_end != std::string::npos) ? io_end + 1 : io_kw + 7;
            }

            scan = abs_end + 1;
        }

        delays_.push_back(std::move(cd));
        pos = cell_end + 1;
    }

    return !delays_.empty();
}

bool SdfReader::load(const std::string& filename) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) return false;
    std::ostringstream ss;
    ss << ifs.rdbuf();
    return parse(ss.str());
}

double SdfReader::lookup_delay(const std::string& instance, bool is_rise,
                               const std::string& condition) const {
    for (auto& cd : delays_) {
        if (cd.instance != instance) continue;

        if (!condition.empty()) {
            for (auto& d : cd.conditional) {
                if (d.condition == condition)
                    return is_rise ? d.delay_rise : d.delay_fall;
            }
        }

        if (!cd.unconditional.empty()) {
            auto& d = cd.unconditional.front();
            return is_rise ? d.delay_rise : d.delay_fall;
        }
    }
    return 0.0;
}

} // namespace sf
