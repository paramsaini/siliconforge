#pragma once
// SiliconForge — ECO (Engineering Change Order) Engine
// Performs incremental netlist modifications: insert/remove/replace gates,
// remap connections, and track changes for sign-off.

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <functional>

namespace sf {

struct EcoChange {
    enum Type { ADD_GATE, REMOVE_GATE, REPLACE_GATE,
                ADD_NET, REMOVE_NET, RECONNECT, ADD_BUFFER } type;
    std::string description;
    GateId gate_id = -1;
    NetId net_id = -1;
    GateType new_type = GateType::BUF;
    std::string name;
};

struct EcoResult {
    int changes_applied = 0;
    int gates_added = 0;
    int gates_removed = 0;
    int buffers_inserted = 0;
    int nets_modified = 0;
    bool success = true;
    std::string message;
    std::vector<EcoChange> changelog;
};

class EcoEngine {
public:
    explicit EcoEngine(Netlist& nl) : nl_(nl) {}

    // Atomic ECO operations
    GateId insert_buffer(NetId net, const std::string& name = "");
    bool remove_gate(GateId gid);
    GateId replace_gate(GateId gid, GateType new_type);
    NetId split_net(NetId nid, const std::string& name = "");
    bool reconnect(GateId gid, int input_idx, NetId new_net);

    // High-level ECO: fix timing by inserting buffers on critical nets
    EcoResult fix_timing(const std::vector<NetId>& critical_nets);

    // High-level ECO: fix fanout violations
    EcoResult fix_fanout(int max_fanout);

    // Get change log
    const std::vector<EcoChange>& changes() const { return changes_; }

    // Apply all queued ECO changes
    EcoResult apply();

private:
    Netlist& nl_;
    std::vector<EcoChange> changes_;
};

} // namespace sf
