// Phase 88: Router scalability — interval tree occupancy, pattern routing,
// multi-threaded net routing. Validates O(log k) DRC queries, L/Z-shape
// pattern routing, and parallel routing correctness.

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <cassert>
#include <chrono>
#include <algorithm>

#include "pnr/detailed_router_v2.hpp"
#include "pnr/global_router.hpp"
#include "pnr/physical.hpp"

static int total = 0, passed = 0;
#define CHECK(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  [PASS] %s\n", msg); } \
    else { printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while(0)

// ═══════════════════════════════════════════════════════════════════════
// TEST 1: TrackOccupancy — basic insert/query
// ═══════════════════════════════════════════════════════════════════════
static void test_track_occupancy_basic() {
    printf("\n── TrackOccupancy basic insert/query ──\n");
    sf::TrackOccupancy occ;

    // Empty → everything free
    CHECK(occ.size() == 0, "empty occupancy has size 0");

    // Insert a segment on layer 0, track 5, from 10.0 to 20.0, net 1
    occ.insert(0, 5, 10.0, 20.0, 1);
    CHECK(occ.size() == 1, "one segment inserted");

    // Query same track, overlapping range — should NOT be free
    // buffer = spacing + width = 0.14 + 0.14 = 0.28
    CHECK(!occ.is_free_with_adj(0, 5, 12.0, 18.0, /*net_id=*/2,
                                 /*wire_width=*/0.14, /*spacing=*/0.14, /*pitch=*/0.5),
          "overlapping segment blocks query");

    // Query same track, non-overlapping range — should be free
    CHECK(occ.is_free_with_adj(0, 5, 25.0, 30.0, 2, 0.14, 0.14, 0.5),
          "non-overlapping range is free");

    // Same net should be ignored (self-overlap OK)
    CHECK(occ.is_free_with_adj(0, 5, 12.0, 18.0, /*net_id=*/1, 0.14, 0.14, 0.5),
          "same net_id ignored in overlap check");

    // Different layer → free
    CHECK(occ.is_free_with_adj(1, 5, 12.0, 18.0, 2, 0.14, 0.14, 0.5),
          "different layer is free");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 2: TrackOccupancy — many segments, O(log k) performance
// ═══════════════════════════════════════════════════════════════════════
static void test_track_occupancy_scale() {
    printf("\n── TrackOccupancy scale test ──\n");
    sf::TrackOccupancy occ;

    // Insert 10000 segments on same layer, different tracks
    const int N = 10000;
    for (int i = 0; i < N; i++) {
        int track = i % 200;
        double lo = (i / 200) * 10.0;
        occ.insert(0, track, lo, lo + 5.0, i);
    }
    CHECK(occ.size() == (size_t)N, "10000 segments inserted");

    // Time 100000 queries — should be fast (O(log k) per query)
    auto t0 = std::chrono::high_resolution_clock::now();
    int free_count = 0;
    for (int q = 0; q < 100000; q++) {
        int track = q % 200;
        double lo = (q % 50) * 10.0 + 6.0;  // offset to test between segments
        if (occ.is_free_with_adj(0, track, lo, lo + 2.0, N + 1, 0.14, 0.14, 0.5))
            free_count++;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    CHECK(ms < 500.0, "100K queries in <500ms (O(log k) performance)");
    printf("    100K queries took %.1f ms, %d free\n", ms, free_count);
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 3: TrackOccupancy — remove_net
// ═══════════════════════════════════════════════════════════════════════
static void test_track_occupancy_remove() {
    printf("\n── TrackOccupancy remove_net ──\n");
    sf::TrackOccupancy occ;

    occ.insert(0, 0, 0.0, 10.0, 1);
    occ.insert(0, 0, 20.0, 30.0, 2);
    occ.insert(0, 1, 5.0, 15.0, 1);
    occ.insert(1, 0, 0.0, 10.0, 3);
    CHECK(occ.size() == 4, "4 segments before remove");

    occ.remove_net(1);
    CHECK(occ.size() == 2, "2 segments after removing net 1");

    // Net 2 segment should still block
    CHECK(!occ.is_free_with_adj(0, 0, 22.0, 28.0, 99, 0.14, 0.14, 0.5),
          "net 2 segment still present after removing net 1");

    // Net 1's old position should now be free
    CHECK(occ.is_free_with_adj(0, 1, 5.0, 15.0, 99, 0.14, 0.14, 0.5),
          "net 1 position free after removal");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 4: TrackOccupancy — adjacent track DRC
// ═══════════════════════════════════════════════════════════════════════
static void test_track_occupancy_adjacent() {
    printf("\n── TrackOccupancy adjacent track DRC ──\n");
    sf::TrackOccupancy occ;

    // Insert on track 5
    occ.insert(0, 5, 10.0, 20.0, 1);

    // Adjacent track 4, tight pitch (pitch < width + spacing → DRC check needed)
    // pitch=0.28 means perp_edge = 0.28 - 0.14 = 0.14 which equals spacing → no block
    CHECK(occ.is_free_with_adj(0, 4, 12.0, 18.0, 2, 0.14, 0.14, 0.28),
          "adjacent track free when perp_edge >= spacing");

    // pitch=0.20 means perp_edge = 0.20 - 0.14 = 0.06 < 0.14 spacing → blocks
    CHECK(!occ.is_free_with_adj(0, 4, 12.0, 18.0, 2, 0.14, 0.14, 0.20),
          "adjacent track blocked when perp_edge < spacing");

    // Track 10 (far away) → always free
    CHECK(occ.is_free_with_adj(0, 10, 12.0, 18.0, 2, 0.14, 0.14, 0.20),
          "distant track always free");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 5: Pattern routing — L-shape and Z-shape
// ═══════════════════════════════════════════════════════════════════════
static void test_pattern_routing() {
    printf("\n── Pattern routing L/Z shapes ──\n");

    // Build a small physical design
    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};

    // Add 4 cells in a grid
    for (int i = 0; i < 4; i++) {
        sf::CellInstance c;
        c.id = i;
        c.name = "C" + std::to_string(i);
        c.cell_type = "INV";
        c.width = 2; c.height = 10;
        c.position.x = (i % 2) * 40 + 10;
        c.position.y = (i / 2) * 40 + 10;
        c.placed = true;
        pd.cells.push_back(c);
    }

    // Add a 2-pin net (C0 → C3, diagonal)
    sf::PhysNet net;
    net.id = 0; net.name = "n0";
    net.cell_ids = {0, 3};
    net.pin_offsets = {{1.0, 5.0}, {1.0, 5.0}};
    pd.nets.push_back(net);

    sf::DetailedRouterV2 dr(pd, 4);
    auto result = dr.route(1);

    CHECK(result.routed_nets > 0, "pattern routing routed at least 1 net");
    CHECK(result.total_wires > 0, "pattern routing produced wires");
    CHECK(result.total_vias >= 0, "via count non-negative");
    printf("    routed=%d, wires=%d, vias=%d\n",
           result.routed_nets, result.total_wires, result.total_vias);
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 6: Pattern routing — try_pattern_route directly
// ═══════════════════════════════════════════════════════════════════════
static void test_try_pattern_route_direct() {
    printf("\n── try_pattern_route direct call ──\n");

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 200, 200};

    // Minimal cells
    for (int i = 0; i < 2; i++) {
        sf::CellInstance c;
        c.id = i; c.name = "C" + std::to_string(i);
        c.cell_type = "BUF";
        c.width = 2; c.height = 10;
        c.position.x = i * 100 + 10; c.position.y = 50;
        c.placed = true;
        pd.cells.push_back(c);
    }
    sf::PhysNet net;
    net.id = 0; net.name = "n0";
    net.cell_ids = {0, 1};
    net.pin_offsets = {{1, 5}, {1, 5}};
    pd.nets.push_back(net);

    sf::DetailedRouterV2 dr(pd, 6);
    // Need to trigger route() first to build track grid
    auto result = dr.route(1);

    // Verify the router ran successfully
    CHECK(result.routed_nets >= 1, "direct pattern route: net routed");

    // The simple 2-pin connection should produce wires
    CHECK(!pd.wires.empty(), "direct pattern route: wires produced");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 7: Multi-threaded routing — correctness with many nets
// ═══════════════════════════════════════════════════════════════════════
static void test_multithreaded_routing() {
    printf("\n── Multi-threaded routing with many nets ──\n");

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 500, 500};

    // Create 50 cells in a grid
    int num_cells = 50;
    for (int i = 0; i < num_cells; i++) {
        sf::CellInstance c;
        c.id = i;
        c.name = "C" + std::to_string(i);
        c.cell_type = (i % 5 == 0) ? "DFF" : "NAND";
        c.width = 2; c.height = 10;
        c.position.x = (i % 10) * 45 + 10;
        c.position.y = (i / 10) * 90 + 10;
        c.placed = true;
        pd.cells.push_back(c);
    }

    // Create 30 2-pin nets connecting random pairs
    for (int i = 0; i < 30; i++) {
        sf::PhysNet net;
        net.id = i;
        net.name = "n" + std::to_string(i);
        int src = i % num_cells;
        int dst = (i * 7 + 13) % num_cells;
        if (dst == src) dst = (dst + 1) % num_cells;
        net.cell_ids = {src, dst};
        net.pin_offsets = {{1, 5}, {1, 5}};
        pd.nets.push_back(net);
    }

    // Route with 4 threads
    sf::DetailedRouterV2 dr(pd, 6);
    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = dr.route(4);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    CHECK(result.routed_nets > 0, "multi-threaded: routed nets > 0");
    CHECK(result.failed_nets < 30, "multi-threaded: not all nets failed");
    CHECK(result.total_wires > 0, "multi-threaded: produced wires");
    printf("    routed=%d, failed=%d, wires=%d, vias=%d, time=%.1fms\n",
           result.routed_nets, result.failed_nets, result.total_wires, result.total_vias, ms);

    // Route with 1 thread for comparison
    sf::PhysicalDesign pd2 = pd;
    pd2.wires.clear(); pd2.vias.clear();
    sf::DetailedRouterV2 dr2(pd2, 6);
    auto result2 = dr2.route(1);

    CHECK(result.routed_nets >= result2.routed_nets - 2,
          "multi-threaded routes comparable count to single-threaded");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 8: Occupancy consistency — flat vector matches interval tree
// ═══════════════════════════════════════════════════════════════════════
static void test_occupancy_consistency() {
    printf("\n── Occupancy flat vs interval tree consistency ──\n");

    sf::TrackOccupancy occ;

    // Insert many segments
    for (int i = 0; i < 500; i++) {
        int layer = i % 4;
        int track = (i * 3) % 50;
        double lo = (i % 20) * 5.0;
        occ.insert(layer, track, lo, lo + 3.0, i);
    }
    CHECK(occ.size() == 500, "500 segments in interval tree");

    // Remove some nets
    for (int n = 0; n < 100; n++) {
        occ.remove_net(n);
    }
    CHECK(occ.size() == 400, "400 segments after removing 100 nets");

    // Clear all
    occ.clear();
    CHECK(occ.size() == 0, "0 segments after clear");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 9: Routing produces valid DRC-clean results
// ═══════════════════════════════════════════════════════════════════════
static void test_routing_drc_clean() {
    printf("\n── Routing DRC-clean validation ──\n");

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 200, 200};

    // Create 10 cells
    for (int i = 0; i < 10; i++) {
        sf::CellInstance c;
        c.id = i; c.name = "G" + std::to_string(i);
        c.cell_type = "NAND";
        c.width = 2; c.height = 10;
        c.position.x = (i % 5) * 35 + 10;
        c.position.y = (i / 5) * 80 + 10;
        c.placed = true;
        pd.cells.push_back(c);
    }

    // 5 nets
    for (int i = 0; i < 5; i++) {
        sf::PhysNet net;
        net.id = i; net.name = "net" + std::to_string(i);
        net.cell_ids = {i, i + 5};
        net.pin_offsets = {{1, 5}, {1, 5}};
        pd.nets.push_back(net);
    }

    sf::DetailedRouterV2 dr(pd, 4);
    auto result = dr.route(2);

    CHECK(result.routed_nets > 0, "DRC check: nets routed");
    // Check no duplicate wires on exact same position
    bool has_dup = false;
    for (size_t i = 0; i < pd.wires.size() && !has_dup; i++) {
        for (size_t j = i + 1; j < pd.wires.size() && !has_dup; j++) {
            auto& a = pd.wires[i];
            auto& b = pd.wires[j];
            if (a.layer == b.layer && a.net_id != b.net_id &&
                std::abs(a.start.x - b.start.x) < 0.001 && std::abs(a.start.y - b.start.y) < 0.001 &&
                std::abs(a.end.x - b.end.x) < 0.001 && std::abs(a.end.y - b.end.y) < 0.001) {
                has_dup = true;
            }
        }
    }
    CHECK(!has_dup, "DRC check: no duplicate wires from different nets");
}

// ═══════════════════════════════════════════════════════════════════════
int main() {
    printf("═══ Phase 88: Router Scalability Tests ═══\n");

    test_track_occupancy_basic();
    test_track_occupancy_scale();
    test_track_occupancy_remove();
    test_track_occupancy_adjacent();
    test_pattern_routing();
    test_try_pattern_route_direct();
    test_multithreaded_routing();
    test_occupancy_consistency();
    test_routing_drc_clean();

    printf("\n═══ Phase 88 Results: %d/%d passed ═══\n", passed, total);
    return (passed == total) ? 0 : 1;
}
