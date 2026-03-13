// SiliconForge — Reset Domain Crossing Analyzer Implementation
#include "verify/rdc.hpp"
#include <unordered_set>
#include <algorithm>
#include <queue>

namespace sf {

// Trace reset nets from DFF reset pins back to primary inputs
std::unordered_map<int, std::string> RdcAnalyzer::detect_reset_domains() {
    // Map: DFF gate_id → reset domain name
    std::unordered_map<int, std::string> dff_domain;

    // For each DFF, trace its reset pin back to a primary input
    for (auto gid : nl_.flip_flops()) {
        auto& ff = nl_.gate(gid);
        if (ff.reset < 0) {
            dff_domain[gid] = "__no_reset__";
            continue;
        }

        // BFS back from reset net to find the source PI
        std::unordered_set<int> visited;
        std::queue<int> q;
        q.push(ff.reset);
        visited.insert(ff.reset);
        std::string domain = "__unknown__";

        while (!q.empty()) {
            int nid = q.front();
            q.pop();

            auto& net = nl_.net(nid);

            // Check if this net is a primary input
            for (auto pi : nl_.primary_inputs()) {
                if (pi == nid) {
                    domain = net.name.empty() ? "reset_" + std::to_string(nid) : net.name;
                    goto found;
                }
            }

            // Trace back through driver gate
            if (net.driver >= 0) {
                auto& drv = nl_.gate(net.driver);
                if (drv.type == GateType::DFF) {
                    // Reset comes from a DFF output — asynchronous reset
                    domain = "dff_rst_" + std::to_string(net.driver);
                    goto found;
                }
                for (auto inp : drv.inputs) {
                    if (!visited.count(inp)) {
                        visited.insert(inp);
                        q.push(inp);
                    }
                }
            }
        }
        found:
        dff_domain[gid] = domain;
    }

    return dff_domain;
}

RdcResult RdcAnalyzer::analyze() {
    RdcResult res;

    auto dff_domains = detect_reset_domains();

    // Count unique reset domains
    std::unordered_set<std::string> domains;
    for (auto& [gid, dom] : dff_domains) {
        if (dom != "__no_reset__" && dom != "__unknown__")
            domains.insert(dom);
    }
    res.reset_domains = (int)domains.size();

    if (res.reset_domains <= 1) {
        res.clean = true;
        res.message = "RDC: " + std::to_string(res.reset_domains) +
                      " reset domain — no crossings";
        return res;
    }

    // Build map: net_id → set of reset domains that fanout from it
    // For each DFF, mark its output net with its reset domain
    std::unordered_map<int, std::string> net_domain;
    for (auto& [gid, dom] : dff_domains) {
        auto& ff = nl_.gate(gid);
        if (ff.output >= 0)
            net_domain[ff.output] = dom;
    }

    // Propagate domains through combinational logic
    auto topo = nl_.topo_order();
    for (auto gid : topo) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF || g.type == GateType::INPUT ||
            g.type == GateType::OUTPUT || g.output < 0) continue;

        // Collect domains of inputs
        std::unordered_set<std::string> input_domains;
        for (auto ni : g.inputs) {
            auto it = net_domain.find(ni);
            if (it != net_domain.end())
                input_domains.insert(it->second);
        }

        if (input_domains.size() == 1)
            net_domain[g.output] = *input_domains.begin();
        else if (input_domains.size() > 1)
            net_domain[g.output] = "__mixed__";
    }

    // Detect crossings: DFF whose D input comes from a different reset domain
    for (auto& [gid, dest_dom] : dff_domains) {
        auto& ff = nl_.gate(gid);
        if (ff.inputs.empty() || dest_dom == "__no_reset__") continue;

        NetId d_net = ff.inputs[0];
        auto it = net_domain.find(d_net);
        if (it == net_domain.end()) continue;

        std::string src_dom = it->second;
        if (src_dom == dest_dom || src_dom == "__no_reset__" ||
            src_dom == "__unknown__" || src_dom == "__mixed__") continue;

        res.crossings++;

        // Check for synchronizer: two back-to-back DFFs in same reset domain
        bool has_sync = false;
        if (nl_.net(d_net).driver >= 0) {
            auto& drv = nl_.gate(nl_.net(d_net).driver);
            if (drv.type == GateType::DFF) {
                // Check if the driving DFF is in the destination domain
                auto dit = dff_domains.find(nl_.net(d_net).driver);
                if (dit != dff_domains.end() && dit->second == dest_dom)
                    has_sync = true;
            }
        }

        if (!has_sync) {
            res.violations++;
            res.details.push_back({
                RdcViolation::MISSING_SYNCHRONIZER,
                src_dom,
                dest_dom,
                ff.name,
                "Reset domain crossing: " + src_dom + " → " + dest_dom +
                " at DFF '" + ff.name + "' — no synchronizer detected"
            });
        } else {
            res.warnings++;
            res.details.push_back({
                RdcViolation::ASYNC_RESET_CROSSING,
                src_dom,
                dest_dom,
                ff.name,
                "Reset domain crossing: " + src_dom + " → " + dest_dom +
                " at DFF '" + ff.name + "' — synchronizer present"
            });
        }
    }

    res.clean = (res.violations == 0);
    res.message = "RDC: " + std::to_string(res.reset_domains) + " domains, " +
                  std::to_string(res.crossings) + " crossings, " +
                  std::to_string(res.violations) + " violations";
    return res;
}

} // namespace sf
