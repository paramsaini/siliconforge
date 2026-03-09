// SiliconForge — CDC Analyzer Implementation
#include "verify/cdc.hpp"
#include <chrono>
#include <algorithm>

namespace sf {

void CdcAnalyzer::set_clock_domain(NetId net, const std::string& domain) {
    domain_map_[net] = domain;
}

void CdcAnalyzer::set_clock_domain(const std::string& net_name, const std::string& domain) {
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        if (nl_.net(i).name == net_name) {
            domain_map_[i] = domain;
            return;
        }
    }
}

std::string CdcAnalyzer::get_domain(NetId nid) const {
    auto it = domain_map_.find(nid);
    if (it != domain_map_.end()) return it->second;
    return "";
}

void CdcAnalyzer::auto_detect_domains() {
    // Find all DFFs and assign domains based on their clock net
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type != GateType::DFF) continue;
        // DFF has inputs: [D, CLK]
        std::string clk_name;
        if (g.inputs.size() >= 2) {
            clk_name = nl_.net(g.inputs[1]).name;
        }
        if (clk_name.empty()) clk_name = "clk_unknown";

        // Assign domain to DFF output and all nets in fanout
        if (g.output >= 0) {
            domain_map_[g.output] = clk_name;
        }
        // Also assign domain to DFF input D
        if (!g.inputs.empty()) {
            domain_map_[g.inputs[0]] = clk_name;
        }
    }

    // Propagate domains forward through combinational logic
    bool changed = true;
    int iters = 0;
    while (changed && iters++ < 100) {
        changed = false;
        for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::DFF || g.type == GateType::INPUT ||
                g.type == GateType::OUTPUT) continue;
            if (g.output < 0) continue;
            if (!get_domain(g.output).empty()) continue;

            // Inherit domain from inputs
            for (auto inp : g.inputs) {
                std::string d = get_domain(inp);
                if (!d.empty()) {
                    domain_map_[g.output] = d;
                    changed = true;
                    break;
                }
            }
        }
    }
}

bool CdcAnalyzer::check_synchronizer(NetId crossing_net, const std::string& dst_domain) const {
    // Check if the crossing net feeds into a chain of ≥2 DFFs in the destination domain
    // (standard 2-FF synchronizer pattern)
    auto& net = nl_.net(crossing_net);
    int ff_chain = 0;

    for (auto gid : net.fanout) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF) {
            ff_chain++;
            // Check if the DFF output feeds another DFF
            if (g.output >= 0) {
                auto& out_net = nl_.net(g.output);
                for (auto gid2 : out_net.fanout) {
                    if (nl_.gate(gid2).type == GateType::DFF) ff_chain++;
                }
            }
        }
    }
    return ff_chain >= 2;
}

CdcResult CdcAnalyzer::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    CdcResult r;

    // Collect all domains
    std::unordered_set<std::string> domains;
    for (auto& [nid, dom] : domain_map_) {
        if (!dom.empty()) domains.insert(dom);
    }
    r.domains.assign(domains.begin(), domains.end());
    std::sort(r.domains.begin(), r.domains.end());
    r.domains_found = (int)r.domains.size();

    // Find crossings: a gate whose inputs come from different domains than its output
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.output < 0) continue;
        std::string out_domain = get_domain(g.output);
        if (out_domain.empty()) continue;

        for (auto inp : g.inputs) {
            std::string in_domain = get_domain(inp);
            if (in_domain.empty() || in_domain == out_domain) continue;

            // This is a clock domain crossing
            CdcCrossing crossing;
            crossing.signal = inp;
            crossing.signal_name = nl_.net(inp).name;
            crossing.src_domain = in_domain;
            crossing.dst_domain = out_domain;

            // Check for synchronizer
            if (check_synchronizer(inp, out_domain)) {
                crossing.sync = CdcCrossing::TWO_FF;
                crossing.is_safe = true;
                crossing.detail = "2-FF synchronizer detected";
                r.safe_crossings++;
            } else {
                crossing.sync = CdcCrossing::NONE;
                crossing.is_safe = false;
                crossing.detail = "No synchronizer — metastability risk";
                r.unsafe_crossings++;
            }

            r.crossings.push_back(crossing);
            r.total_crossings++;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.domains_found) + " domains, " +
                std::to_string(r.total_crossings) + " crossings (" +
                std::to_string(r.safe_crossings) + " safe, " +
                std::to_string(r.unsafe_crossings) + " unsafe)";
    return r;
}

} // namespace sf
