#pragma once
// SiliconForge — TSV insertion optimizer for 3D-IC

#include "core/die_to_die.hpp"
#include "pnr/physical.hpp"
#include <map>

namespace sf {

class TsvManager {
public:
    // For a multi-die design, given the layout of two dies, insert TSVs to connect shared nets
    // places TSVs at optimal (x,y) to minimize wirelength on both dies
    static void insert_tsvs(PackageDesign& pkg, PhysicalDesign& die1, PhysicalDesign& die2, int z1, int z2);

private:
    // Simple heuristic: place TSV at the center-of-mass of the net's pins on both dies
    static void place_tsv_for_net(PackageDesign& pkg, int net_id,
                                  const PhysicalDesign& d1, const PhysicalDesign& d2,
                                  int d1_id, int d2_id);
};

} // namespace sf
