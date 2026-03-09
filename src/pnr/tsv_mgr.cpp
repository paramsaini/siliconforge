// SiliconForge — TSV insertion optimizer
#include "pnr/tsv_mgr.hpp"
#include <iostream>

namespace sf {

void TsvManager::insert_tsvs(PackageDesign& pkg, PhysicalDesign& die1, PhysicalDesign& die2, int z1, int z2) {
    int d1_id = -1, d2_id = -1;
    for (const auto& d : pkg.dies()) {
        if (d.z_layer == z1) d1_id = d.id;
        if (d.z_layer == z2) d2_id = d.id;
    }
    
    if (d1_id == -1 || d2_id == -1) {
        std::cerr << "TsvManager: Could not find dies at layers " << z1 << " and " << z2 << "\n";
        return;
    }

    std::map<int, bool> shared;
    for (const auto& pnet : die1.nets) shared[pnet.id] = true;
    for (const auto& w1 : die1.wires) shared[w1.layer] = true; // Use layer as a proxy for net_id in this simplification if needed, but nets is better
    
    for (auto net_id : shared) {
        bool found_in_2 = false;
        for (const auto& pnet2 : die2.nets) if (pnet2.id == net_id.first) found_in_2 = true;
        
        if (found_in_2) {
            place_tsv_for_net(pkg, net_id.first, die1, die2, d1_id, d2_id);
        }
    }
}

void TsvManager::place_tsv_for_net(PackageDesign& pkg, int net_id,
                                   const PhysicalDesign& d1, const PhysicalDesign& d2,
                                   int d1_id, int d2_id) {
    // Calculate Center of Mass for net
    float sum_x = 0, sum_y = 0;
    int count = 0;

    auto count_pins = [&](const PhysicalDesign& d) {
        for (const auto& n : d.nets) {
            if (n.id == net_id) {
                for (int cid : n.cell_ids) {
                    if (cid >= 0 && cid < (int)d.cells.size()) {
                        sum_x += d.cells[cid].position.x;
                        sum_y += d.cells[cid].position.y;
                        count++;
                    }
                }
            }
        }
    };

    count_pins(d1);
    count_pins(d2);

    if (count > 0) {
        float cx = sum_x / count;
        float cy = sum_y / count;
        pkg.add_tsv(net_id, d1_id, d2_id, cx, cy);
    }
}

} // namespace sf
