#pragma once
// SiliconForge — ECO (Engineering Change Order) Engine
// Performs incremental netlist modifications: insert/remove/replace gates,
// remap connections, and track changes for sign-off.

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
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

// ── Phase C: Full ECO Flow ──────────────────────────────────────────────────

struct EcoConfig {
    enum Mode { FUNCTIONAL, METAL_ONLY, SPARE_CELL };
    Mode mode = FUNCTIONAL;
    int max_spare_cells = 50;
    double max_timing_impact_ns = 0.5;
};

struct FullEcoResult {
    int changes = 0;
    int gates_added = 0;
    int gates_removed = 0;
    int gates_resized = 0;
    int spare_cells_used = 0;
    int nets_rerouted = 0;
    double timing_impact_ns = 0;
    std::string message;
};

class FullEcoEngine {
public:
    explicit FullEcoEngine(Netlist& nl) : nl_(nl) {}

    FullEcoResult run_eco(const EcoConfig& cfg = {});

    // Metal-only ECO with spare cells
    struct SpareCellConfig {
        std::vector<std::string> spare_types;  // e.g., {"NAND2", "NOR2", "INV", "BUF"}
        int spare_count_per_type;
    };
    struct MetalOnlyResult {
        int spare_cells_used;
        int spare_cells_remaining;
        bool success;
        double timing_impact_ns;
        std::string message;
    };
    MetalOnlyResult eco_metal_only(const std::vector<std::pair<int,int>>& changes,
                                    const SpareCellConfig& spares);

    // Functional ECO with patch synthesis
    struct PatchResult {
        int gates_added;
        int gates_removed;
        int gates_modified;
        bool functionally_equivalent;
        std::string patch_netlist;
    };
    PatchResult eco_functional_patch(const Netlist& golden, const Netlist& revised);

    // Incremental LEC after ECO
    struct EcoLecResult {
        bool equivalent;
        int mismatches;
        std::vector<std::string> mismatch_outputs;
    };
    EcoLecResult verify_eco();

    // Timing-aware ECO placement
    struct EcoPlaceResult {
        int cells_placed;
        double max_displacement;
        double timing_slack_after;
    };
    EcoPlaceResult eco_place(double max_displacement_um = 10.0);

    // Enhanced ECO: run functional patch + metal-only + verify + place
    FullEcoResult run_enhanced();

    // ── Tier 3: Enhanced Metal-Only ECO ─────────────────────────────────
    struct SpareCellLibrary {
        struct SpareDef {
            std::string type;              // "NAND2", "NOR2", "INV", "BUF", "MUX2", "XOR2"
            int num_inputs = 2;
            double area = 1.0;
            double max_drive_strength = 1.0;
            std::vector<std::string> can_implement;  // gate types this spare can replace
        };
        std::vector<SpareDef> definitions;
        void add_spare_type(const std::string& type, int inputs, double area,
                           const std::vector<std::string>& implements);
    };

    struct SpareCellInventory {
        struct SpareInstance {
            std::string type;
            int cell_id = -1;
            Point location;
            bool used = false;
            int eco_revision = -1;         // which ECO revision consumed it
        };
        std::vector<SpareInstance> instances;
        int available(const std::string& type) const;
        int total_available() const;
        SpareInstance* find_nearest(const std::string& type, const Point& target);
    };

    struct EnhancedEcoResult {
        int spare_cells_used = 0;
        int spare_cells_remaining = 0;
        bool success = false;
        bool drc_clean = false;
        double timing_impact_ns = 0;
        double max_wire_length = 0;
        int eco_revision = 0;
        std::string message;
    };

    void set_spare_library(const SpareCellLibrary& lib) { spare_lib_ = lib; }
    void set_spare_inventory(const SpareCellInventory& inv) { spare_inv_ = inv; }
    EnhancedEcoResult eco_metal_only_enhanced(
        const std::vector<std::pair<int,int>>& changes,
        double max_wire_length = 100.0);
    bool eco_drc_check() const;

private:
    Netlist& nl_;
    SpareCellLibrary spare_lib_;
    SpareCellInventory spare_inv_;
    int eco_revision_ = 0;

    FullEcoResult run_functional_eco(const EcoConfig& cfg);
    FullEcoResult run_metal_only_eco(const EcoConfig& cfg);
    FullEcoResult run_spare_cell_eco(const EcoConfig& cfg);
};

} // namespace sf
