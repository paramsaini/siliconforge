// SiliconForge — ECO Engine Implementation
#include "synth/eco.hpp"
#include <iostream>
#include <algorithm>

namespace sf {

GateId EcoEngine::insert_buffer(NetId net, const std::string& name) {
    // Create a new net for the buffer output
    std::string buf_name = name.empty() ? ("eco_buf_" + std::to_string(changes_.size())) : name;
    NetId buf_out = nl_.add_net(buf_name + "_o");

    // Insert buffer: old_net → BUF → new_net
    GateId buf = nl_.add_gate(GateType::BUF, {net}, buf_out, buf_name);

    // Reconnect all fanout of original net (except the buffer) to buffer output
    auto& orig_net = nl_.net(net);
    std::vector<GateId> old_fanout = orig_net.fanout;
    for (auto gid : old_fanout) {
        if (gid == buf) continue; // don't reconnect the buffer itself
        auto& g = nl_.gate(gid);
        for (auto& inp : g.inputs) {
            if (inp == net) inp = buf_out;
        }
    }

    changes_.push_back({EcoChange::ADD_BUFFER, "Buffer " + buf_name + " on net " + std::to_string(net),
                       buf, net, GateType::BUF, buf_name});
    return buf;
}

bool EcoEngine::remove_gate(GateId gid) {
    auto& g = nl_.gate(gid);
    if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) return false;
    std::string gname = g.name;
    g.type = GateType::BUF; // Mark as dead (simplified — real impl would remove)
    g.inputs.clear();
    changes_.push_back({EcoChange::REMOVE_GATE, "Removed gate " + gname, gid, -1, GateType::BUF, gname});
    return true;
}

GateId EcoEngine::replace_gate(GateId gid, GateType new_type) {
    auto& g = nl_.gate(gid);
    GateType old = g.type;
    g.type = new_type;
    changes_.push_back({EcoChange::REPLACE_GATE,
        "Replaced " + std::string(gate_type_str(old)) + " → " + gate_type_str(new_type),
        gid, -1, new_type, g.name});
    return gid;
}

bool EcoEngine::reconnect(GateId gid, int input_idx, NetId new_net) {
    auto& g = nl_.gate(gid);
    if (input_idx < 0 || input_idx >= (int)g.inputs.size()) return false;
    NetId old_net = g.inputs[input_idx];
    g.inputs[input_idx] = new_net;
    changes_.push_back({EcoChange::RECONNECT,
        "Reconnected " + g.name + " input " + std::to_string(input_idx),
        gid, new_net, g.type, g.name});
    return true;
}

NetId EcoEngine::split_net(NetId nid, const std::string& name) {
    std::string net_name = name.empty() ? ("eco_split_" + std::to_string(nid)) : name;
    NetId new_net = nl_.add_net(net_name);
    changes_.push_back({EcoChange::ADD_NET, "Split net " + std::to_string(nid),
                       -1, new_net, GateType::BUF, net_name});
    return new_net;
}

EcoResult EcoEngine::fix_timing(const std::vector<NetId>& critical_nets) {
    EcoResult r;
    for (auto nid : critical_nets) {
        insert_buffer(nid);
        r.buffers_inserted++;
        r.changes_applied++;
    }
    r.success = true;
    r.message = "Inserted " + std::to_string(r.buffers_inserted) +
                " buffers on critical nets";
    r.changelog = changes_;
    return r;
}

EcoResult EcoEngine::fix_fanout(int max_fanout) {
    EcoResult r;
    for (size_t nid = 0; nid < nl_.num_nets(); ++nid) {
        auto& net = nl_.net(nid);
        if ((int)net.fanout.size() > max_fanout) {
            insert_buffer(nid);
            r.buffers_inserted++;
            r.nets_modified++;
            r.changes_applied++;
        }
    }
    r.success = true;
    r.message = "Fixed " + std::to_string(r.nets_modified) +
                " fanout violations (max=" + std::to_string(max_fanout) + ")";
    r.changelog = changes_;
    return r;
}

EcoResult EcoEngine::apply() {
    EcoResult r;
    r.changes_applied = (int)changes_.size();
    for (auto& c : changes_) {
        if (c.type == EcoChange::ADD_GATE || c.type == EcoChange::ADD_BUFFER) r.gates_added++;
        if (c.type == EcoChange::REMOVE_GATE) r.gates_removed++;
    }
    r.success = true;
    r.changelog = changes_;
    r.message = std::to_string(r.changes_applied) + " ECO changes applied";
    return r;
}

} // namespace sf
