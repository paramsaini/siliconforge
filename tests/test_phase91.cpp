// Phase 91: Advanced Scalability — DCT spectral density smoothing,
// GPU-ready parallel density kernel, distributed routing partitioner.
// Validates spectral smoothing convergence, parallel density correctness,
// and net partitioning into local/boundary/global categories.

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <cassert>
#include <chrono>
#include <algorithm>
#include <numeric>

#include "pnr/spectral_density.hpp"
#include "pnr/gpu_density.hpp"
#include "pnr/distributed_route.hpp"
#include "pnr/placer.hpp"
#include "pnr/physical.hpp"

static int total = 0, passed = 0;
#define CHECK(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  [PASS] %s\n", msg); } \
    else { printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while(0)

// ═══════════════════════════════════════════════════════════════════════
// TEST 1: SpectralDensity — basic DCT round-trip
// ═══════════════════════════════════════════════════════════════════════
static void test_spectral_roundtrip() {
    printf("\n── SpectralDensity DCT round-trip ──\n");
    sf::SpectralDensity sd;
    sd.init(16, 16, 100.0, 100.0, 0.7, 8.0);

    // Create a uniform density map at target
    std::vector<std::vector<double>> density(16, std::vector<double>(16, 0.7));

    // Smoothing uniform density should produce near-zero potential
    auto smoothed = sd.smooth(density);
    double max_val = 0;
    for (auto& row : smoothed)
        for (auto v : row)
            max_val = std::max(max_val, std::abs(v));

    CHECK(max_val < 0.01, "uniform density at target → near-zero potential");
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 2: SpectralDensity — hotspot smoothing
// ═══════════════════════════════════════════════════════════════════════
static void test_spectral_hotspot() {
    printf("\n── SpectralDensity hotspot smoothing ──\n");
    sf::SpectralDensity sd;
    sd.init(16, 16, 100.0, 100.0, 0.7, 8.0);

    // Create density at target with one big hotspot
    std::vector<std::vector<double>> density(16, std::vector<double>(16, 0.7));
    density[8][8] = 2.5;  // big hotspot, well above target

    auto field = sd.compute_field(density);

    CHECK(field.overflow > 0, "hotspot causes overflow > 0");

    // The potential at hotspot should be the peak (maximum in the field)
    double max_pot = -1e18;
    int max_r = 0, max_c = 0;
    for (int r = 0; r < 16; r++)
        for (int c = 0; c < 16; c++)
            if (field.potential[r][c] > max_pot) {
                max_pot = field.potential[r][c]; max_r = r; max_c = c;
            }
    CHECK(max_r == 8 && max_c == 8, "peak potential at hotspot location");

    // Gradient magnitude should be non-zero near hotspot
    double grad_mag = std::abs(field.grad_x[8][7]) + std::abs(field.grad_y[7][8]);
    CHECK(grad_mag > 0.001, "non-zero gradient near hotspot");

    printf("    Overflow: %.4f, Peak potential: %.4f\n",
           field.overflow, field.potential[8][8]);
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 3: SpectralDensity — larger grid performance
// ═══════════════════════════════════════════════════════════════════════
static void test_spectral_performance() {
    printf("\n── SpectralDensity 32x32 performance ──\n");
    sf::SpectralDensity sd;
    sd.init(32, 32, 1000.0, 1000.0, 0.7, 8.0);

    // Random-ish density pattern
    std::vector<std::vector<double>> density(32, std::vector<double>(32, 0.5));
    for (int r = 0; r < 32; r++)
        for (int c = 0; c < 32; c++)
            density[r][c] = 0.3 + 0.8 * ((r * 7 + c * 13) % 17) / 17.0;

    auto t0 = std::chrono::high_resolution_clock::now();
    auto field = sd.compute_field(density);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    CHECK(ms < 200, "32x32 spectral density < 200ms");
    CHECK(field.potential.size() == 32, "output grid correct size");
    printf("    32x32 spectral: %.1fms, overflow: %.2f\n", ms, field.overflow);
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 4: ParallelDensityKernel — single vs multi-thread correctness
// ═══════════════════════════════════════════════════════════════════════
static void test_parallel_density_correctness() {
    printf("\n── ParallelDensityKernel correctness ──\n");
    sf::ParallelDensityKernel pk;
    pk.init(32, 32, 0, 0, 1000, 1000);

    // Create 5000 cells
    std::vector<sf::ParallelDensityKernel::CellRect> cells;
    for (int i = 0; i < 5000; i++) {
        double x = (i * 17 + 31) % 980 + 5.0;
        double y = (i * 23 + 47) % 980 + 5.0;
        cells.push_back({x, y, 2.0, 10.0});
    }

    // Sequential
    auto grid1 = pk.compute(cells, 1);
    // Parallel (4 threads)
    auto grid4 = pk.compute(cells, 4);

    double max_diff = 0;
    for (int r = 0; r < 32; r++)
        for (int c = 0; c < 32; c++)
            max_diff = std::max(max_diff, std::abs(grid1[r][c] - grid4[r][c]));

    CHECK(max_diff < 1e-10, "1-thread and 4-thread density maps match exactly");
    CHECK(grid1.size() == 32, "output grid correct rows");
    CHECK(grid1[0].size() == 32, "output grid correct cols");

    // Verify density is non-zero somewhere
    double total_density = 0;
    for (auto& row : grid1) for (auto v : row) total_density += v;
    CHECK(total_density > 0, "density map has non-zero content");
    printf("    Max diff: %.2e, Total density: %.1f\n", max_diff, total_density);
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 5: ParallelDensityKernel — performance scaling
// ═══════════════════════════════════════════════════════════════════════
static void test_parallel_density_performance() {
    printf("\n── ParallelDensityKernel performance ──\n");
    sf::ParallelDensityKernel pk;
    pk.init(64, 64, 0, 0, 2000, 2000);

    std::vector<sf::ParallelDensityKernel::CellRect> cells;
    for (int i = 0; i < 50000; i++) {
        double x = (i * 17 + 31) % 1980 + 5.0;
        double y = (i * 23 + 47) % 1980 + 5.0;
        cells.push_back({x, y, 2.0, 10.0});
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    auto g1 = pk.compute(cells, 1);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms1 = std::chrono::duration<double, std::milli>(t1 - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    auto g4 = pk.compute(cells, 4);
    t1 = std::chrono::high_resolution_clock::now();
    double ms4 = std::chrono::duration<double, std::milli>(t1 - t0).count();

    CHECK(ms1 < 100, "50K cells, 1 thread, 64x64 grid < 100ms");
    // Parallel should be at least somewhat faster (may not be 4x due to overhead)
    CHECK(ms4 <= ms1 * 1.5, "parallel not significantly slower than sequential");
    printf("    1-thread: %.1fms, 4-thread: %.1fms, speedup: %.2fx\n",
           ms1, ms4, ms1 / std::max(ms4, 0.001));
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 6: ParallelDensityKernel — WL gradient correctness
// ═══════════════════════════════════════════════════════════════════════
static void test_parallel_wl_gradient() {
    printf("\n── ParallelDensityKernel WL gradient ──\n");
    sf::ParallelDensityKernel pk;
    pk.init(16, 16, 0, 0, 100, 100);

    int n = 20;
    std::vector<double> cx(n), cy(n);
    for (int i = 0; i < n; i++) {
        cx[i] = 10 + (i % 5) * 20;
        cy[i] = 10 + (i / 5) * 20;
    }

    std::vector<sf::ParallelDensityKernel::NetPins> nets;
    for (int i = 0; i < 10; i++) {
        sf::ParallelDensityKernel::NetPins np;
        np.cell_ids = {i, (i + 5) % n};
        np.weight = 1.0;
        nets.push_back(np);
    }

    auto r1 = pk.compute_wl_gradient(cx, cy, nets, 5.0, 1);
    auto r4 = pk.compute_wl_gradient(cx, cy, nets, 5.0, 4);

    double wl_diff = std::abs(r1.total_wl - r4.total_wl);
    CHECK(wl_diff < 1e-6, "1-thread and 4-thread WL match");
    CHECK(r1.total_wl > 0, "total wirelength positive");

    double grad_diff = 0;
    for (int i = 0; i < n; i++) {
        grad_diff += std::abs(r1.grad_x[i] - r4.grad_x[i]);
        grad_diff += std::abs(r1.grad_y[i] - r4.grad_y[i]);
    }
    CHECK(grad_diff < 1e-6, "WL gradients match between 1 and 4 threads");
    printf("    WL: %.4f, grad_diff: %.2e\n", r1.total_wl, grad_diff);
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 7: DistributedRouter — net partitioning
// ═══════════════════════════════════════════════════════════════════════
static void test_distributed_partition() {
    printf("\n── DistributedRouter net partitioning ──\n");

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};

    // 100 cells in a 10x10 grid
    for (int i = 0; i < 100; i++) {
        sf::CellInstance c;
        c.id = i;
        c.name = "C" + std::to_string(i);
        c.cell_type = "NAND";
        c.width = 2; c.height = 10;
        c.position.x = (i % 10) * 95 + 20;
        c.position.y = (i / 10) * 95 + 20;
        c.placed = true;
        pd.cells.push_back(c);
    }

    // Local nets: both pins in same quadrant
    for (int i = 0; i < 20; i++) {
        sf::PhysNet net;
        net.id = i;
        net.name = "local_" + std::to_string(i);
        int base = (i / 5) * 20 + (i % 5);  // cells 0-4, 20-24, 40-44, 60-64
        net.cell_ids = {base, base + 1};
        net.pin_offsets = {{1, 5}, {1, 5}};
        pd.nets.push_back(net);
    }

    // Boundary nets: pins in adjacent quadrants
    for (int i = 0; i < 10; i++) {
        sf::PhysNet net;
        net.id = 20 + i;
        net.name = "boundary_" + std::to_string(i);
        net.cell_ids = {i, i + 50};  // top-left to bottom-left crossing
        net.pin_offsets = {{1, 5}, {1, 5}};
        pd.nets.push_back(net);
    }

    // Global nets: pins across 3+ tiles
    for (int i = 0; i < 5; i++) {
        sf::PhysNet net;
        net.id = 30 + i;
        net.name = "global_" + std::to_string(i);
        net.cell_ids = {i, i + 33, i + 66};  // across many tiles
        net.pin_offsets = {{1, 5}, {1, 5}, {1, 5}};
        pd.nets.push_back(net);
    }

    sf::DistributedRouter dr(pd, 4);
    dr.set_tile_grid(4, 4);
    dr.partition_nets();

    int local = dr.count_local();
    int boundary = dr.count_boundary();
    int global = dr.count_global();
    int total_nets = local + boundary + global;

    CHECK(total_nets == 35, "all 35 nets partitioned");
    CHECK(local > 0, "some nets are local");
    CHECK(dr.tiles().size() == 16, "4x4 = 16 tiles created");

    printf("    Local: %d, Boundary: %d, Global: %d, Tiles: %zu\n",
           local, boundary, global, dr.tiles().size());
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 8: DistributedRouter — route execution
// ═══════════════════════════════════════════════════════════════════════
static void test_distributed_route() {
    printf("\n── DistributedRouter route execution ──\n");

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 500, 500};

    for (int i = 0; i < 40; i++) {
        sf::CellInstance c;
        c.id = i;
        c.name = "C" + std::to_string(i);
        c.cell_type = "INV";
        c.width = 2; c.height = 10;
        c.position.x = (i % 8) * 55 + 20;
        c.position.y = (i / 8) * 90 + 20;
        c.placed = true;
        pd.cells.push_back(c);
    }

    for (int i = 0; i < 20; i++) {
        sf::PhysNet net;
        net.id = i;
        net.name = "n" + std::to_string(i);
        net.cell_ids = {i, (i + 7) % 40};
        net.pin_offsets = {{1, 5}, {1, 5}};
        pd.nets.push_back(net);
    }

    sf::DistributedRouter dr(pd, 4);
    dr.set_tile_grid(3, 3);
    dr.set_threads(4);

    auto result = dr.route();

    CHECK(result.total_routed > 0, "distributed route: nets routed > 0");
    CHECK(result.tiles_used == 9, "3x3 = 9 tiles");
    CHECK(result.threads_used == 4, "4 threads configured");
    CHECK(result.total_time_ms >= 0, "time is non-negative");

    printf("    Routed: %d, Failed: %d, Tiles: %d, Time: %.1fms\n",
           result.total_routed, result.total_failed, result.tiles_used, result.total_time_ms);
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 9: Placer spectral density integration
// ═══════════════════════════════════════════════════════════════════════
static void test_placer_spectral_integration() {
    printf("\n── Placer spectral density integration ──\n");

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 200, 200};

    for (int i = 0; i < 30; i++) {
        sf::CellInstance c;
        c.id = i;
        c.name = "C" + std::to_string(i);
        c.cell_type = "NAND";
        c.width = 2; c.height = 10;
        c.position.x = 100;  // all at center (maximum density)
        c.position.y = 100;
        c.placed = true;
        pd.cells.push_back(c);
    }

    for (int i = 0; i < 15; i++) {
        sf::PhysNet net;
        net.id = i;
        net.name = "n" + std::to_string(i);
        net.cell_ids = {i, i + 15};
        net.pin_offsets = {{1, 5}, {1, 5}};
        pd.nets.push_back(net);
    }

    // Test with spectral mode
    sf::AnalyticalPlacer ap(pd);
    sf::AnalyticalPlacer::DensityConfig dcfg;
    dcfg.bin_count_x = 16;
    dcfg.bin_count_y = 16;
    dcfg.target_density = 0.7;
    dcfg.use_spectral = true;
    dcfg.spectral_alpha = 8.0;
    ap.set_density_config(dcfg);

    auto result = ap.place_eplace();
    CHECK(result.hpwl >= 0, "spectral ePlace: HPWL non-negative");
    CHECK(result.iterations > 0, "spectral ePlace: iterations > 0");

    // Cells should have spread from center
    double max_x = 0, min_x = 1e18;
    for (auto& cell : pd.cells) {
        max_x = std::max(max_x, cell.position.x);
        min_x = std::min(min_x, cell.position.x);
    }
    double spread = max_x - min_x;
    CHECK(spread > 10.0, "spectral ePlace: cells spread from center");
    printf("    HPWL: %.1f, Spread: %.1f, Iterations: %d\n",
           result.hpwl, spread, result.iterations);
}

// ═══════════════════════════════════════════════════════════════════════
// TEST 10: DistributedRouter — auto tile sizing
// ═══════════════════════════════════════════════════════════════════════
static void test_distributed_auto_tiles() {
    printf("\n── DistributedRouter auto tile sizing ──\n");

    sf::PhysicalDesign pd;
    pd.die_area = {0, 0, 1000, 1000};

    // 500 cells
    for (int i = 0; i < 500; i++) {
        sf::CellInstance c;
        c.id = i;
        c.name = "C" + std::to_string(i);
        c.cell_type = "BUF";
        c.width = 2; c.height = 10;
        c.position.x = (i % 25) * 38 + 10;
        c.position.y = (i / 25) * 48 + 10;
        c.placed = true;
        pd.cells.push_back(c);
    }

    for (int i = 0; i < 100; i++) {
        sf::PhysNet net;
        net.id = i;
        net.name = "n" + std::to_string(i);
        net.cell_ids = {i, (i + 50) % 500};
        net.pin_offsets = {{1, 5}, {1, 5}};
        pd.nets.push_back(net);
    }

    sf::DistributedRouter dr(pd, 4);
    // Don't set tile grid — let it auto-size
    dr.partition_nets();

    int total_partitioned = dr.count_local() + dr.count_boundary() + dr.count_global();
    CHECK(total_partitioned == 100, "auto-tiling: all 100 nets partitioned");
    CHECK(dr.tiles().size() >= 4, "auto-tiling: at least 4 tiles");
    printf("    Auto tiles: %zu, Local: %d, Boundary: %d, Global: %d\n",
           dr.tiles().size(), dr.count_local(), dr.count_boundary(), dr.count_global());
}

// ═══════════════════════════════════════════════════════════════════════
int main() {
    printf("═══ Phase 91: Advanced Scalability Tests ═══\n");

    test_spectral_roundtrip();
    test_spectral_hotspot();
    test_spectral_performance();
    test_parallel_density_correctness();
    test_parallel_density_performance();
    test_parallel_wl_gradient();
    test_distributed_partition();
    test_distributed_route();
    test_placer_spectral_integration();
    test_distributed_auto_tiles();

    printf("\n═══ Phase 91 Results: %d/%d passed ═══\n", passed, total);
    return (passed == total) ? 0 : 1;
}
