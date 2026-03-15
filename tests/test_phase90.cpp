// Phase 90: Memory + Architecture scalability — NamePool string interning,
// compiled Liberty binary cache, hierarchical netlist flatten.
// Validates O(1) name lookup, binary cache round-trip, and hierarchy depth.

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <cassert>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <cstdlib>

#include "core/name_pool.hpp"
#include "core/liberty_cache.hpp"
#include "core/liberty_parser.hpp"
#include "core/netlist.hpp"

static int total = 0, passed = 0;
#define CHECK(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  [PASS] %s\n", msg); } \
    else { printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while(0)

// ═══════════════════════════════════════════════════════════════════════
// TEST 1: NamePool — basic intern/retrieve
// ═══════════════════════════════════════════════════════════════════════
static void test_name_pool_basic() {
    printf("\n── NamePool basic intern/retrieve ──\n");
    sf::NamePool pool;

    // Empty string → ID 0
    sf::NameId empty_id = pool.intern("");
    CHECK(empty_id == 0, "empty string returns ID 0");
    CHECK(std::string(pool.c_str(0)) == "", "ID 0 is empty string");

    // Intern some names
    sf::NameId a = pool.intern("net_clk");
    sf::NameId b = pool.intern("net_data");
    sf::NameId c = pool.intern("net_clk"); // duplicate

    CHECK(a > 0, "first name gets positive ID");
    CHECK(b > 0 && b != a, "second name gets different ID");
    CHECK(c == a, "duplicate returns same ID");

    CHECK(pool.str(a) == "net_clk", "retrieve by ID matches");
    CHECK(pool.str(b) == "net_data", "retrieve second name matches");
    CHECK(pool.view(a) == "net_clk", "string_view retrieval works");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 2: NamePool — find (lookup without inserting)
// ═══════════════════════════════════════════════════════════════════════
static void test_name_pool_find() {
    printf("\n── NamePool find (non-inserting lookup) ──\n");
    sf::NamePool pool;

    pool.intern("gate_G0");
    pool.intern("gate_G1");

    CHECK(pool.find("gate_G0") != sf::NAME_NONE, "find existing name succeeds");
    CHECK(pool.find("gate_G999") == sf::NAME_NONE, "find non-existing returns NAME_NONE");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 3: NamePool — scale test (100K names)
// ═══════════════════════════════════════════════════════════════════════
static void test_name_pool_scale() {
    printf("\n── NamePool scale test (100K names) ──\n");
    sf::NamePool pool;
    pool.reserve(100000);

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100000; i++) {
        pool.intern("core/alu/G" + std::to_string(i));
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double insert_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    CHECK(pool.size() == 100001, "100K names + empty = 100001 entries");

    // Dedup test: re-intern same names
    t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100000; i++) {
        sf::NameId id = pool.intern("core/alu/G" + std::to_string(i));
        (void)id;
    }
    t1 = std::chrono::high_resolution_clock::now();
    double dedup_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    CHECK(pool.size() == 100001, "dedup: no new entries after re-intern");
    CHECK(pool.arena_bytes() < 3000000, "arena < 3MB for 100K names");

    printf("    Insert: %.1fms, Dedup: %.1fms, Arena: %zu bytes\n",
           insert_ms, dedup_ms, pool.arena_bytes());
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 4: NamePool — clear
// ═══════════════════════════════════════════════════════════════════════
static void test_name_pool_clear() {
    printf("\n── NamePool clear ──\n");
    sf::NamePool pool;
    pool.intern("test1");
    pool.intern("test2");
    CHECK(pool.size() == 3, "3 entries before clear (including empty)");

    pool.clear();
    CHECK(pool.size() == 1, "1 entry after clear (empty string)");
    CHECK(pool.find("test1") == sf::NAME_NONE, "cleared names not found");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 5: Global name pool singleton
// ═══════════════════════════════════════════════════════════════════════
static void test_global_name_pool() {
    printf("\n── Global name pool singleton ──\n");
    auto& pool1 = sf::global_name_pool();
    auto& pool2 = sf::global_name_pool();
    CHECK(&pool1 == &pool2, "global_name_pool returns same instance");

    sf::NameId id = pool1.intern("global_test_net");
    CHECK(pool2.str(id) == "global_test_net", "shared access works");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 6: Liberty binary cache — round-trip
// ═══════════════════════════════════════════════════════════════════════
static void test_liberty_cache_roundtrip() {
    printf("\n── Liberty binary cache round-trip ──\n");

    // Build a test library in memory
    sf::LibertyLibrary lib;
    lib.name = "test_lib";
    lib.technology = "cmos";
    lib.nom_voltage = 1.1;
    lib.nom_temperature = 25.0;
    lib.time_unit = "1ns";
    lib.cap_unit = "1pF";

    // Add cells
    for (int i = 0; i < 10; i++) {
        sf::LibertyCell cell;
        cell.name = "INV_X" + std::to_string(i);
        cell.area = 1.0 + i * 0.5;
        cell.leakage_power = 0.001 * (i + 1);

        sf::LibertyPin pin_a;
        pin_a.name = "A";
        pin_a.direction = "input";
        pin_a.capacitance = 0.01 * (i + 1);
        cell.pins.push_back(pin_a);

        sf::LibertyPin pin_y;
        pin_y.name = "Y";
        pin_y.direction = "output";
        pin_y.function = "!A";
        cell.pins.push_back(pin_y);

        sf::LibertyTiming timing;
        timing.related_pin = "A";
        timing.timing_type = "combinational";
        timing.cell_rise = 0.05 + i * 0.01;
        timing.cell_fall = 0.04 + i * 0.01;
        timing.rise_transition = 0.03;
        timing.fall_transition = 0.025;

        // Add NLDM table data
        timing.nldm_rise.index_1 = {0.01, 0.05, 0.1};
        timing.nldm_rise.index_2 = {0.001, 0.01, 0.05};
        timing.nldm_rise.values = {{0.02, 0.03, 0.06},
                                    {0.03, 0.04, 0.07},
                                    {0.05, 0.06, 0.10}};
        cell.timings.push_back(timing);

        sf::LibertyPower pwr;
        pwr.related_pin = "A";
        pwr.rise_power = 0.01;
        pwr.fall_power = 0.008;
        cell.internal_powers.push_back(pwr);

        sf::LibertyLeakage lk;
        lk.when = "!A";
        lk.value = 0.0005;
        cell.leakage_powers.push_back(lk);

        lib.cells.push_back(cell);
    }

    // Save to temp file
    std::string cache_path = "/tmp/sf_test_cache.slib";
    bool saved = sf::LibertyCacheWriter::save_compiled(lib, cache_path, 12345);
    CHECK(saved, "cache saved successfully");

    // Load back
    sf::LibertyLibrary lib2;
    bool loaded = sf::LibertyCacheReader::load_compiled(cache_path, lib2);
    CHECK(loaded, "cache loaded successfully");

    // Verify round-trip fidelity
    CHECK(lib2.name == "test_lib", "library name matches");
    CHECK(lib2.nom_voltage == 1.1, "nom_voltage matches");
    CHECK(lib2.cells.size() == 10, "cell count matches");

    for (int i = 0; i < 10; i++) {
        auto& orig = lib.cells[i];
        auto& copy = lib2.cells[i];
        CHECK(orig.name == copy.name, ("cell " + std::to_string(i) + " name matches").c_str());
        CHECK(std::abs(orig.area - copy.area) < 1e-9,
              ("cell " + std::to_string(i) + " area matches").c_str());
        CHECK(orig.pins.size() == copy.pins.size(),
              ("cell " + std::to_string(i) + " pin count matches").c_str());
        CHECK(orig.timings.size() == copy.timings.size(),
              ("cell " + std::to_string(i) + " timing count matches").c_str());

        if (!orig.timings.empty() && !copy.timings.empty()) {
            CHECK(std::abs(orig.timings[0].cell_rise - copy.timings[0].cell_rise) < 1e-9,
                  ("cell " + std::to_string(i) + " cell_rise matches").c_str());

            // Verify NLDM table round-trip
            auto& ot = orig.timings[0].nldm_rise;
            auto& ct = copy.timings[0].nldm_rise;
            CHECK(ot.index_1.size() == ct.index_1.size(),
                  ("cell " + std::to_string(i) + " NLDM index_1 size matches").c_str());
            if (!ot.values.empty() && !ct.values.empty()) {
                CHECK(std::abs(ot.values[0][0] - ct.values[0][0]) < 1e-9,
                      ("cell " + std::to_string(i) + " NLDM value[0][0] matches").c_str());
            }
        }
    }

    // Clean up
    std::remove(cache_path.c_str());
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 7: Liberty cache — validity check
// ═══════════════════════════════════════════════════════════════════════
static void test_liberty_cache_validity() {
    printf("\n── Liberty cache validity check ──\n");

    sf::LibertyLibrary lib;
    lib.name = "validity_test";
    lib.nom_voltage = 0.9;
    lib.cells.push_back({"BUF_X1", 0.5, 0.001, {}, {}, {}, {}, {}, {}});

    std::string cache_path = "/tmp/sf_test_validity.slib";
    sf::LibertyCacheWriter::save_compiled(lib, cache_path, 99999);

    CHECK(sf::LibertyCacheReader::is_cache_valid(cache_path, 99999),
          "cache valid with matching mtime");
    CHECK(!sf::LibertyCacheReader::is_cache_valid(cache_path, 88888),
          "cache invalid with different mtime");
    CHECK(!sf::LibertyCacheReader::is_cache_valid("/tmp/nonexistent.slib", 99999),
          "cache invalid for missing file");

    std::remove(cache_path.c_str());
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 8: Liberty cache — performance
// ═══════════════════════════════════════════════════════════════════════
static void test_liberty_cache_performance() {
    printf("\n── Liberty cache performance ──\n");

    // Build a larger library (100 cells with NLDM tables)
    sf::LibertyLibrary lib;
    lib.name = "perf_test";
    for (int i = 0; i < 100; i++) {
        sf::LibertyCell cell;
        cell.name = "CELL_" + std::to_string(i);
        cell.area = 1.0 + i * 0.1;

        sf::LibertyPin pa; pa.name = "A"; pa.direction = "input"; pa.capacitance = 0.01;
        sf::LibertyPin py; py.name = "Y"; py.direction = "output"; py.function = "!A";
        cell.pins = {pa, py};

        sf::LibertyTiming t;
        t.related_pin = "A";
        t.cell_rise = 0.05;
        t.nldm_rise.index_1 = {0.01, 0.02, 0.05, 0.1, 0.2};
        t.nldm_rise.index_2 = {0.001, 0.005, 0.01, 0.02, 0.05};
        t.nldm_rise.values.resize(5);
        for (auto& row : t.nldm_rise.values) {
            row = {0.01, 0.02, 0.03, 0.05, 0.10};
        }
        cell.timings.push_back(t);
        lib.cells.push_back(cell);
    }

    std::string cache_path = "/tmp/sf_test_perf.slib";

    auto t0 = std::chrono::high_resolution_clock::now();
    sf::LibertyCacheWriter::save_compiled(lib, cache_path);
    auto t1 = std::chrono::high_resolution_clock::now();
    double write_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    sf::LibertyLibrary lib2;
    sf::LibertyCacheReader::load_compiled(cache_path, lib2);
    t1 = std::chrono::high_resolution_clock::now();
    double read_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    CHECK(lib2.cells.size() == 100, "100 cells loaded from cache");
    CHECK(write_ms < 50, "cache write <50ms for 100 cells");
    CHECK(read_ms < 50, "cache read <50ms for 100 cells");
    printf("    Write: %.2fms, Read: %.2fms, Cells: %zu\n",
           write_ms, read_ms, lib2.cells.size());

    std::remove(cache_path.c_str());
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 9: Hierarchical netlist — flatten
// ═══════════════════════════════════════════════════════════════════════
static void test_hierarchical_netlist() {
    printf("\n── Hierarchical netlist flatten ──\n");

    sf::HierarchicalNetlist hier;

    // Define a leaf module: INV
    auto& inv_mod = hier.add_module("INV");
    inv_mod.ports.push_back({"A", sf::ModulePort::INPUT, 1, -1});
    inv_mod.ports.push_back({"Y", sf::ModulePort::OUTPUT, 1, -1});
    auto a_net = inv_mod.internal_netlist.add_net("A");
    auto y_net = inv_mod.internal_netlist.add_net("Y");
    inv_mod.internal_netlist.mark_input(a_net);
    inv_mod.internal_netlist.mark_output(y_net);
    inv_mod.internal_netlist.add_gate(sf::GateType::NOT, {a_net}, y_net, "g0");
    inv_mod.ports[0].net_id = a_net;
    inv_mod.ports[1].net_id = y_net;

    // Define a top module with 3 INV instances chained
    auto& top = hier.add_module("top");
    top.ports.push_back({"in", sf::ModulePort::INPUT, 1, -1});
    top.ports.push_back({"out", sf::ModulePort::OUTPUT, 1, -1});

    auto in_net = top.internal_netlist.add_net("in");
    top.internal_netlist.add_net("w1");
    top.internal_netlist.add_net("w2");
    auto out_net = top.internal_netlist.add_net("out");
    top.internal_netlist.mark_input(in_net);
    top.internal_netlist.mark_output(out_net);
    top.ports[0].net_id = in_net;
    top.ports[1].net_id = out_net;

    hier.add_instance("top", "u0", "INV", {{"A", "in"}, {"Y", "w1"}});
    hier.add_instance("top", "u1", "INV", {{"A", "w1"}, {"Y", "w2"}});
    hier.add_instance("top", "u2", "INV", {{"A", "w2"}, {"Y", "out"}});

    hier.set_top("top");

    // Hierarchy queries
    CHECK(hier.depth() >= 1, "hierarchy depth >= 1");
    CHECK(hier.total_instances() >= 3, "total instances >= 3");

    // Flatten
    sf::Netlist flat = hier.flatten();
    CHECK(flat.num_gates() >= 3, "flattened has >= 3 gates");
    CHECK(flat.num_nets() >= 4, "flattened has >= 4 nets");

    printf("    Depth=%d, Instances=%d, Flat gates=%zu, Flat nets=%zu\n",
           hier.depth(), hier.total_instances(),
           flat.num_gates(), flat.num_nets());
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 10: Hierarchical netlist — block abstract
// ═══════════════════════════════════════════════════════════════════════
static void test_block_abstract() {
    printf("\n── Block abstract creation ──\n");

    sf::HierarchicalNetlist hier;
    auto& mod = hier.add_module("BUF2");
    mod.ports.push_back({"A", sf::ModulePort::INPUT, 1, -1});
    mod.ports.push_back({"Y", sf::ModulePort::OUTPUT, 1, -1});
    auto a = mod.internal_netlist.add_net("A");
    auto w = mod.internal_netlist.add_net("w");
    auto y = mod.internal_netlist.add_net("Y");
    mod.internal_netlist.mark_input(a);
    mod.internal_netlist.mark_output(y);
    mod.internal_netlist.add_gate(sf::GateType::NOT, {a}, w, "g0");
    mod.internal_netlist.add_gate(sf::GateType::NOT, {w}, y, "g1");
    mod.ports[0].net_id = a;
    mod.ports[1].net_id = y;

    hier.set_top("BUF2");

    auto abstract = hier.create_abstract("BUF2");
    CHECK(abstract.module_name == "BUF2", "abstract module name matches");
    CHECK(abstract.ports.size() == 2, "abstract has 2 ports");
    printf("    Abstract: %s, ports=%zu, arcs=%zu\n",
           abstract.module_name.c_str(), abstract.ports.size(), abstract.arcs.size());
}

// ═══════════════════════════════════════════════════════════════════════
int main() {
    printf("═══ Phase 90: Memory + Architecture Scalability Tests ═══\n");

    test_name_pool_basic();
    test_name_pool_find();
    test_name_pool_scale();
    test_name_pool_clear();
    test_global_name_pool();
    test_liberty_cache_roundtrip();
    test_liberty_cache_validity();
    test_liberty_cache_performance();
    test_hierarchical_netlist();
    test_block_abstract();

    printf("\n═══ Phase 90 Results: %d/%d passed ═══\n", passed, total);
    return (passed == total) ? 0 : 1;
}
