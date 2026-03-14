#pragma once
// SiliconForge — SPEF/DSPF Parasitic File Parser
// Reads Standard Parasitic Exchange Format (IEEE 1481) files.
// Supports both SPEF (lumped) and DSPF (distributed) formats.
// Reference: IEEE Std 1481-2009

#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

struct SpefUnit {
    double time_scale       = 1e-9;   // default: 1 NS
    double cap_scale        = 1e-12;  // default: 1 PF
    double res_scale        = 1.0;    // default: 1 OHM
    double inductance_scale = 1e-9;   // default: 1 NH
};

struct SpefPinParasitic {
    std::string pin_name;
    double cap = 0.0;
    double x   = 0.0;
    double y   = 0.0;
};

struct SpefResistor {
    int id = 0;
    std::string node1;
    std::string node2;
    double value = 0.0;
};

struct SpefCapacitor {
    int id = 0;
    std::string node1;
    std::string node2;   // empty for ground cap
    double value = 0.0;
};

struct SpefInductor {
    int id = 0;
    std::string node1;
    std::string node2;
    double value = 0.0;  // Henries
};

struct MutualInductance {
    std::string net1;
    std::string net2;
    double coupling_l = 0.0;  // Henries
};

struct ExtractionCorner {
    std::string name;           // "typical", "cworst", "cbest", "rcworst", "rcbest"
    double r_factor = 1.0;     // resistance multiplier
    double c_factor = 1.0;     // capacitance multiplier
    double l_factor = 1.0;     // inductance multiplier
};

struct SpefNet {
    std::string name;
    double total_cap = 0.0;
    double total_ind = 0.0;    // total inductance (Henries)
    std::vector<SpefPinParasitic> pins;
    std::vector<SpefResistor>     resistors;
    std::vector<SpefCapacitor>    caps;
    std::vector<SpefInductor>     inductors;
};

struct SpefData {
    std::string design_name;
    SpefUnit    units;
    std::vector<SpefNet> nets;
    std::unordered_map<std::string, int> net_map; // name → index in nets
    std::vector<MutualInductance> mutual_inductances;

    // Scale all parasitics by temperature and voltage factors.
    // R_scaled = R * (1 + tcr * (temp_factor - 1))    [tcr = temp coeff of resistance]
    // C_scaled = C * (1 + vcc * (voltage_factor - 1))  [vcc = voltage coeff of cap]
    // L_scaled = L * temp_factor (mild temperature dependence)
    void scale_parasitics(double temp_factor, double voltage_factor,
                          double tcr = 0.003, double vcc = 0.001);

    // Apply process-corner derating factors
    void apply_corner(const ExtractionCorner& corner);
};

class SpefParser {
public:
    bool parse(const std::string& filename);
    bool parse_string(const std::string& content);

    const SpefData& data() const { return data_; }
    void print_stats() const;

private:
    SpefData data_;
    std::unordered_map<std::string, std::string> name_map_; // *N → actual name

    // Parsing helpers
    void parse_header(const std::vector<std::string>& lines, size_t& idx);
    void parse_name_map(const std::vector<std::string>& lines, size_t& idx);
    void parse_net(const std::vector<std::string>& lines, size_t& idx);
    void parse_mutual_inductance(const std::vector<std::string>& lines, size_t& idx);

    // Tokenization / utility
    static std::string strip(const std::string& s);
    static std::vector<std::string> split_tokens(const std::string& line);
    static std::vector<std::string> read_lines(const std::string& content);
    std::string resolve_name(const std::string& token) const;
    static double parse_unit_scale(const std::string& amount, const std::string& unit,
                                   double base_ns, double base_pf,
                                   double base_ohm, double base_nh);
};

} // namespace sf
