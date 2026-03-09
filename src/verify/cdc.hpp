#pragma once
// SiliconForge — Clock Domain Crossing (CDC) Analyzer
// Detects signals crossing between clock domains and checks for
// proper synchronization (2-FF, pulse sync, handshake).
// Reference: Cummings, "Clock Domain Crossing (CDC) Design & Verification", SNUG 2008

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace sf {

struct CdcCrossing {
    NetId signal;
    std::string signal_name;
    std::string src_domain;
    std::string dst_domain;
    enum SyncType { NONE, TWO_FF, PULSE_SYNC, HANDSHAKE, MUX_SYNC } sync = NONE;
    bool is_safe = false;
    std::string detail;
};

struct CdcResult {
    int total_crossings = 0;
    int safe_crossings = 0;
    int unsafe_crossings = 0;
    int domains_found = 0;
    double time_ms = 0;
    std::vector<CdcCrossing> crossings;
    std::vector<std::string> domains;
    std::string message;
};

class CdcAnalyzer {
public:
    explicit CdcAnalyzer(const Netlist& nl) : nl_(nl) {}

    // Assign clock domain to a net
    void set_clock_domain(NetId net, const std::string& domain);
    void set_clock_domain(const std::string& net_name, const std::string& domain);

    // Auto-detect domains from DFF clock inputs
    void auto_detect_domains();

    CdcResult analyze();

private:
    const Netlist& nl_;
    std::unordered_map<NetId, std::string> domain_map_;

    std::string get_domain(NetId nid) const;
    bool check_synchronizer(NetId crossing_net, const std::string& dst_domain) const;
};

} // namespace sf
