#pragma once
// SiliconForge — 3D-IC & Multi-Die Packaging Definitions

#include <vector>
#include <string>

namespace sf {

// Represents a vertical die stack or a 2.5D interposer layout.
struct DieInstance {
    int id;
    std::string name;
    int z_layer = 0;   // 0 = bottom, 1 = layer above, etc.
    float x_offset = 0; // For 2.5D interposers
    float y_offset = 0;
};

// Represents a Through-Silicon Via connecting two adjacent Z-layers
struct TSV {
    int id;
    int net_id;
    int from_die_id;
    int to_die_id;
    float x;
    float y;
};

class PackageDesign {
public:
    int add_die(const std::string& name, int z_layer, float x = 0, float y = 0);
    int add_tsv(int net_id, int from_die, int to_die, float x, float y);

    const std::vector<DieInstance>& dies() const { return dies_; }
    const std::vector<TSV>& tsvs() const { return tsvs_; }

private:
    std::vector<DieInstance> dies_;
    std::vector<TSV> tsvs_;
    int die_counter_ = 0;
    int tsv_counter_ = 0;
};

} // namespace sf
