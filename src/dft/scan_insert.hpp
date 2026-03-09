#pragma once
// SiliconForge — Scan Chain Insertion for DFT
// Converts regular DFFs to muxed-scan flip-flops and stitches scan chains.

#include "core/netlist.hpp"
#include <string>
#include <vector>

namespace sf {

struct ScanConfig {
    int max_chain_length = 100;  // Max FFs per scan chain
    std::string scan_in_prefix = "scan_in";
    std::string scan_out_prefix = "scan_out";
    std::string scan_enable_name = "scan_enable";
};

struct ScanResult {
    int num_chains = 0;
    int total_ffs = 0;
    std::vector<int> chain_lengths;
    std::string message;
};

class ScanInserter {
public:
    explicit ScanInserter(Netlist& nl) : nl_(nl) {}

    // Insert scan chains — modifies the netlist in place
    ScanResult insert(const ScanConfig& config = {});

private:
    Netlist& nl_;
};

} // namespace sf
