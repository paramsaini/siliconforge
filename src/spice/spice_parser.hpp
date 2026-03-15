// SPICE Netlist Parser — parses SPICE subcircuits, device instances,
// .model directives, and .param statements into structured representations
// suitable for circuit simulation via MNA-based engine.
#pragma once

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace sf {

// ────────────────────────────────────────────────────────────────────────
// Device type enumeration
// ────────────────────────────────────────────────────────────────────────
enum class SpiceDeviceType {
    MOSFET,     // Mxxx
    RESISTOR,   // Rxxx
    CAPACITOR,  // Cxxx
    VSOURCE,    // Vxxx
    ISOURCE,    // Ixxx
    INDUCTOR,   // Lxxx
    DIODE,      // Dxxx
    UNKNOWN
};

// ────────────────────────────────────────────────────────────────────────
// SpiceDevice — single element instance
// ────────────────────────────────────────────────────────────────────────
struct SpiceDevice {
    std::string                        name;
    SpiceDeviceType                    type = SpiceDeviceType::UNKNOWN;
    std::vector<std::string>           terminals;   // ordered terminal net names
    std::string                        model_name;  // for MOSFET / diode
    std::map<std::string, double>      params;      // W=, L=, value, etc.
};

// ────────────────────────────────────────────────────────────────────────
// SpiceModel — .model card
// ────────────────────────────────────────────────────────────────────────
struct SpiceModel {
    std::string                        name;
    std::string                        type;   // nmos, pmos, npn, d, ...
    int                                level = 1;
    std::map<std::string, double>      params;
};

// ────────────────────────────────────────────────────────────────────────
// SpiceSubckt — .subckt block
// ────────────────────────────────────────────────────────────────────────
struct SpiceSubckt {
    std::string                        name;
    std::vector<std::string>           ports;
    std::vector<SpiceDevice>           devices;
    std::vector<std::string>           internal_nets;
};

// ────────────────────────────────────────────────────────────────────────
// SpiceCircuit — top-level parsed representation
// ────────────────────────────────────────────────────────────────────────
struct SpiceCircuit {
    std::string                                title;
    std::vector<SpiceSubckt>                   subcircuits;
    std::vector<SpiceDevice>                   instances;   // top-level devices
    std::map<std::string, SpiceModel>          models;
    std::map<std::string, double>              params;
};

// ────────────────────────────────────────────────────────────────────────
// SpiceParser
// ────────────────────────────────────────────────────────────────────────
class SpiceParser {
public:
    SpiceCircuit parse(const std::string& netlist);

private:
    // Tokenize a line into whitespace-separated tokens
    static std::vector<std::string> tokenize(const std::string& line);

    // Parse a numeric value, handling engineering suffixes (k, m, u, n, p, f, etc.)
    static double parse_value(const std::string& s);

    // Parse key=value pairs from token list starting at idx
    static std::map<std::string, double> parse_kv_params(
        const std::vector<std::string>& tokens, size_t start_idx);

    // Determine device type from first character of instance name
    static SpiceDeviceType device_type_from_name(const std::string& name);

    // Parse a device instance line
    SpiceDevice parse_device(const std::vector<std::string>& tokens);

    // Parse .model line(s)
    SpiceModel parse_model(const std::string& combined_line);

    // Collect internal nets for a subcircuit (nets not in port list)
    static void resolve_internal_nets(SpiceSubckt& sub);
};

} // namespace sf
