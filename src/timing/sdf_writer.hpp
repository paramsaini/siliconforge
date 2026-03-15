#pragma once
// SiliconForge — SDF (Standard Delay Format) Writer
// Generates IEEE 1497 SDF output for timing back-annotation.
// Supports min:typ:max delay triplets from Liberty NLDM lookup.
// Reference: IEEE Std 1497-2001

#include "core/netlist.hpp"
#include "timing/sta.hpp"
#include "core/liberty_parser.hpp"
#include <string>
#include <vector>

namespace sf {

struct SdfCondDelay {
    std::string condition;   // boolean expression
    double delay_rise = 0.0;
    double delay_fall = 0.0;
};

struct SdfConfig {
    std::string design_name = "top";
    double timescale = 1.0; // ns
    bool include_interconnect = true;
    bool emit_triplets = false;         // min:typ:max triplets
    double min_factor = 0.85;           // min = typ * min_factor (if no library)
    double max_factor = 1.15;           // max = typ * max_factor (if no library)
    const LibertyLibrary* lib = nullptr; // optional: use for NLDM-based delays
    const PhysicalDesign* pd = nullptr;  // optional: for wire delays
    double input_slew = 0.05;           // default input slew for NLDM lookup (ns)
    double load_cap = 0.01;             // default load cap for NLDM lookup (pF)
    std::vector<SdfCondDelay> cond_delays; // conditional delays
};

class SdfWriter {
public:
    SdfWriter(const Netlist& nl) : nl_(nl) {}

    std::string generate(const SdfConfig& cfg = {});
    bool write(const std::string& filename, const SdfConfig& cfg = {});

private:
    const Netlist& nl_;
    std::string cell_delays(const SdfConfig& cfg);
    std::string interconnect_delays(const SdfConfig& cfg);

    // Format a delay value as scalar or (min:typ:max) triplet
    std::string fmt_delay(double typ, const SdfConfig& cfg) const;
    // Lookup delay from NLDM if library available
    double nldm_delay(const std::string& cell_name, double slew, double load) const;
};

struct SdfCellDelay {
    std::string instance;
    std::string cell_type;
    std::vector<SdfCondDelay> unconditional;
    std::vector<SdfCondDelay> conditional;
};

class SdfReader {
public:
    explicit SdfReader(Netlist& nl) : nl_(nl) {}

    bool parse(const std::string& sdf_content);
    bool load(const std::string& filename);
    const std::vector<SdfCellDelay>& cell_delays() const { return delays_; }
    double lookup_delay(const std::string& instance, bool is_rise,
                        const std::string& condition = "") const;

private:
    Netlist& nl_;
    std::vector<SdfCellDelay> delays_;
};

} // namespace sf
