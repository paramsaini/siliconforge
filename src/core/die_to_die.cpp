// SiliconForge — 3D-IC & Multi-Die Packaging
#include "core/die_to_die.hpp"

namespace sf {

int PackageDesign::add_die(const std::string& name, int z_layer, float x, float y) {
    int id = die_counter_++;
    dies_.push_back({id, name, z_layer, x, y});
    return id;
}

int PackageDesign::add_tsv(int net_id, int from_die, int to_die, float x, float y) {
    int id = tsv_counter_++;
    tsvs_.push_back({id, net_id, from_die, to_die, x, y});
    return id;
}

} // namespace sf
