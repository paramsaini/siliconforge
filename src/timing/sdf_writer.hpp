#pragma once
// SiliconForge — SDF (Standard Delay Format) Writer
// Generates IEEE 1497 SDF output for timing back-annotation.
// Reference: IEEE Std 1497-2001

#include "core/netlist.hpp"
#include "timing/sta.hpp"
#include <string>

namespace sf {

struct SdfConfig {
    std::string design_name = "top";
    double timescale = 1.0; // ns
    bool include_interconnect = true;
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
};

} // namespace sf
