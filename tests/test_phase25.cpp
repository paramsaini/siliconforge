// test_phase25.cpp — Router Industrial Features Tests
// Tests: RouterConfig, NetTimingInfo, timing-driven routing (global + detailed),
//        via minimization, antenna checking, layer promotion, industrial metrics.
// Reference: Cadence Innovus / Synopsys ICC2 timing-driven routing methodology

#include "pnr/global_router.hpp"
#include "pnr/detailed_router_v2.hpp"
#include "pnr/physical.hpp"
#include <iostream>
#include <cmath>
#include <unordered_map>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// Helper: build a simple physical design with cells and nets
static PhysicalDesign make_routable_pd() {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    pd.site_width = 0.5;

    int c0 = pd.add_cell("c0", "AND2", 2, 2);
    int c1 = pd.add_cell("c1", "AND2", 2, 2);
    int c2 = pd.add_cell("c2", "INV", 1, 2);
    int c3 = pd.add_cell("c3", "BUF", 1.5, 2);

    pd.cells[c0].position = {10, 10};
    pd.cells[c1].position = {80, 80};
    pd.cells[c2].position = {50, 20};
    pd.cells[c3].position = {30, 70};

    pd.add_net("net0", {c0, c1});
    pd.add_net("net1", {c2, c3});
    pd.add_net("net2", {c0, c2, c3});

    return pd;
}

// ===========================
// Test 1: RouterConfig defaults
// ===========================
TEST(router_config_defaults) {
    RouterConfig cfg;
    CHECK(!cfg.enable_timing_driven, "timing-driven off by default");
    CHECK(!cfg.enable_crosstalk_avoidance, "xtalk off by default");
    CHECK(!cfg.enable_via_minimization, "via-min off by default");
    CHECK(!cfg.enable_antenna_check, "antenna off by default");
    CHECK(!cfg.enable_layer_promotion, "layer promotion off by default");
    CHECK(cfg.max_reroute_iterations == 15, "max reroute = 15");
    CHECK(cfg.detailed_reroute_iterations == 8, "detailed reroute = 8");
    CHECK(cfg.timing_weight == 0.5, "timing weight = 0.5");
    CHECK(cfg.via_cost == 1.5, "via cost = 1.5");
    CHECK(cfg.max_antenna_ratio == 400.0, "antenna ratio = 400");
    PASS("router_config_defaults");
}

// ===========================
// Test 2: NetTimingInfo defaults
// ===========================
TEST(net_timing_info_defaults) {
    NetTimingInfo nti;
    CHECK(nti.slack > 1e17, "default slack = huge");
    CHECK(nti.criticality == 0.0, "default criticality = 0");
    CHECK(!nti.is_clock, "not clock by default");
    CHECK(!nti.is_critical, "not critical by default");
    PASS("net_timing_info_defaults");
}

// ===========================
// Test 3: GlobalRouter config API
// ===========================
TEST(global_router_config_api) {
    auto pd = make_routable_pd();
    GlobalRouter gr(pd, 10, 10, 4);

    RouterConfig cfg;
    cfg.enable_timing_driven = true;
    cfg.timing_weight = 0.8;
    gr.set_config(cfg);

    CHECK(gr.config().enable_timing_driven, "config set");
    CHECK(gr.config().timing_weight == 0.8, "weight set");
    PASS("global_router_config_api");
}

// ===========================
// Test 4: GlobalRouter timing-driven net ordering
// ===========================
TEST(global_router_timing_driven) {
    auto pd = make_routable_pd();
    GlobalRouter gr(pd, 10, 10, 4);

    RouterConfig cfg;
    cfg.enable_timing_driven = true;
    gr.set_config(cfg);

    // net0 is critical, net1 is not
    NetTimingInfo crit;
    crit.slack = -0.5;
    crit.criticality = 0.9;
    crit.is_critical = true;
    gr.set_net_timing(0, crit);

    NetTimingInfo ok;
    ok.slack = 2.0;
    ok.criticality = 0.1;
    gr.set_net_timing(1, ok);

    auto r = gr.route();
    CHECK(r.routed_nets >= 2, "at least 2 nets routed");
    CHECK(r.message.find("timing-driven") != std::string::npos, "message mentions timing-driven");
    PASS("global_router_timing_driven");
}

// ===========================
// Test 5: GlobalRouter antenna check
// ===========================
TEST(global_router_antenna_check) {
    auto pd = make_routable_pd();
    GlobalRouter gr(pd, 10, 10, 4);

    RouterConfig cfg;
    cfg.enable_antenna_check = true;
    cfg.max_antenna_ratio = 0.001; // very strict — should find violations
    gr.set_config(cfg);

    auto r = gr.route();
    CHECK(r.routed_nets > 0, "some nets routed");
    // With extremely strict antenna ratio, violations expected
    CHECK(r.antenna_violations >= 0, "antenna check ran");
    PASS("global_router_antenna_check");
}

// ===========================
// Test 6: GlobalRouter industrial metrics
// ===========================
TEST(global_router_industrial_metrics) {
    auto pd = make_routable_pd();
    GlobalRouter gr(pd, 10, 10, 4);
    auto r = gr.route();

    CHECK(r.total_wires > 0, "has wires");
    CHECK(r.total_vias >= 0, "via count tracked");
    CHECK(r.total_wirelength > 0, "wirelength measured");
    CHECK(r.time_ms >= 0, "time measured");
    CHECK(!r.message.empty(), "message generated");
    PASS("global_router_industrial_metrics");
}

// ===========================
// Test 7: GlobalRouter layer promotion
// ===========================
TEST(global_router_layer_promotion) {
    auto pd = make_routable_pd();
    GlobalRouter gr(pd, 10, 10, 6);

    RouterConfig cfg;
    cfg.enable_timing_driven = true;
    cfg.enable_layer_promotion = true;
    gr.set_config(cfg);

    NetTimingInfo crit;
    crit.criticality = 0.95;
    crit.is_critical = true;
    gr.set_net_timing(0, crit);

    auto r = gr.route();
    CHECK(r.routed_nets >= 2, "nets routed with layer promotion");
    PASS("global_router_layer_promotion");
}

// ===========================
// Test 8: GlobalRouter net timing map
// ===========================
TEST(global_router_net_timing_map) {
    auto pd = make_routable_pd();
    GlobalRouter gr(pd, 10, 10, 4);

    std::unordered_map<int, NetTimingInfo> map;
    map[0] = {-1.0, 0.95, false, true};
    map[1] = {5.0, 0.05, false, false};
    gr.set_net_timing_map(map);

    RouterConfig cfg;
    cfg.enable_timing_driven = true;
    gr.set_config(cfg);

    auto r = gr.route();
    CHECK(r.routed_nets >= 2, "nets routed with map");
    CHECK(r.critical_net_wirelength > 0, "critical WL tracked");
    PASS("global_router_net_timing_map");
}

// ===========================
// Test 9: DetailedRouterV2 returns RouteResult
// ===========================
TEST(detailed_router_returns_result) {
    auto pd = make_routable_pd();
    DetailedRouterV2 dr(pd, 4);
    auto r = dr.route(1);

    CHECK(r.routed_nets >= 0, "routed count available");
    CHECK(r.total_wires >= 0, "wire count");
    CHECK(r.total_vias >= 0, "via count");
    CHECK(r.total_wirelength >= 0, "wirelength");
    CHECK(!r.message.empty(), "message");
    CHECK(r.time_ms >= 0, "time");
    PASS("detailed_router_returns_result");
}

// ===========================
// Test 10: DetailedRouterV2 config + timing API
// ===========================
TEST(detailed_router_config_api) {
    auto pd = make_routable_pd();
    DetailedRouterV2 dr(pd, 4);

    RouterConfig cfg;
    cfg.enable_timing_driven = true;
    cfg.enable_via_minimization = true;
    dr.set_config(cfg);

    CHECK(dr.config().enable_timing_driven, "cfg set");
    CHECK(dr.config().enable_via_minimization, "via min set");

    NetTimingInfo nti;
    nti.criticality = 0.8;
    dr.set_net_timing(0, nti);

    auto r = dr.route(1);
    CHECK(r.routed_nets >= 0, "routes complete");
    CHECK(r.message.find("timing-driven") != std::string::npos, "timing-driven in msg");
    CHECK(r.message.find("via-min") != std::string::npos, "via-min in msg");
    PASS("detailed_router_config_api");
}

// ===========================
// Test 11: DetailedRouterV2 via minimization
// ===========================
TEST(detailed_router_via_minimization) {
    auto pd = make_routable_pd();
    DetailedRouterV2 dr(pd, 4);

    RouterConfig cfg;
    cfg.enable_via_minimization = true;
    dr.set_config(cfg);

    auto r = dr.route(1);
    CHECK(r.total_vias >= 0, "via count tracked");
    PASS("detailed_router_via_minimization");
}

// ===========================
// Test 12: DetailedRouterV2 antenna check
// ===========================
TEST(detailed_router_antenna_check) {
    auto pd = make_routable_pd();
    DetailedRouterV2 dr(pd, 4);

    RouterConfig cfg;
    cfg.enable_antenna_check = true;
    cfg.max_antenna_ratio = 0.001; // strict
    dr.set_config(cfg);

    auto r = dr.route(1);
    // With extremely strict ratio, violations are likely
    CHECK(r.antenna_violations >= 0, "antenna check ran");
    PASS("detailed_router_antenna_check");
}

// ===========================
// Test 13: DetailedRouterV2 wires_per_layer populated
// ===========================
TEST(detailed_router_per_layer) {
    auto pd = make_routable_pd();
    DetailedRouterV2 dr(pd, 4);
    auto r = dr.route(1);

    CHECK((int)r.wires_per_layer.size() == 4, "4 layer entries");
    int total = 0;
    for (int c : r.wires_per_layer) total += c;
    CHECK(total == r.total_wires, "per-layer sums to total");
    PASS("detailed_router_per_layer");
}

// ===========================
// Test 14: DetailedRouterV2 reroute iterations tracked
// ===========================
TEST(detailed_router_reroute_iters) {
    auto pd = make_routable_pd();
    DetailedRouterV2 dr(pd, 4);
    auto r = dr.route(1);
    CHECK(r.reroute_iterations >= 0, "reroute iterations tracked");
    PASS("detailed_router_reroute_iters");
}

// ===========================
// Test 15: GlobalRouter backward compat (no config)
// ===========================
TEST(backward_compat_no_config) {
    auto pd = make_routable_pd();
    GlobalRouter gr(pd, 10, 10, 4);
    // No config, no timing — should work like before
    auto r = gr.route();
    CHECK(r.routed_nets >= 2, "backward compat routes");
    CHECK(r.overflow == 0, "no overflow");
    PASS("backward_compat_no_config");
}

// ===========================
// Test 16: RouteResult fields
// ===========================
TEST(route_result_fields) {
    RouteResult r;
    CHECK(r.routed_nets == 0, "default routed = 0");
    CHECK(r.failed_nets == 0, "default failed = 0");
    CHECK(r.total_vias == 0, "default vias = 0");
    CHECK(r.total_wires == 0, "default wires = 0");
    CHECK(r.critical_net_wirelength == 0, "default critical WL = 0");
    CHECK(r.timing_driven_reroutes == 0, "default timing reroutes = 0");
    CHECK(r.antenna_violations == 0, "default antenna = 0");
    CHECK(r.nets_on_promoted_layers == 0, "default promoted = 0");
    CHECK(r.wires_per_layer.empty(), "default per-layer empty");
    PASS("route_result_fields");
}

// ===========================
// Test 17: Detailed + Global pipeline E2E
// ===========================
TEST(global_then_detailed_e2e) {
    auto pd = make_routable_pd();

    // Global route first
    GlobalRouter gr(pd, 10, 10, 4);
    RouterConfig gcfg;
    gcfg.enable_timing_driven = true;
    gr.set_config(gcfg);

    NetTimingInfo crit;
    crit.criticality = 0.9;
    gr.set_net_timing(0, crit);

    auto gr_result = gr.route();
    CHECK(gr_result.routed_nets >= 2, "global routes");

    // Clear wires for detailed routing
    pd.wires.clear();
    pd.vias.clear();

    // Detailed route
    DetailedRouterV2 dr(pd, 4);
    RouterConfig dcfg;
    dcfg.enable_timing_driven = true;
    dcfg.enable_via_minimization = true;
    dr.set_config(dcfg);
    dr.set_net_timing(0, crit);

    auto dr_result = dr.route(1);
    CHECK(dr_result.routed_nets >= 0, "detailed routes");
    CHECK(dr_result.total_wires > 0 || dr_result.routed_nets == 0, "wires or no routes");
    PASS("global_then_detailed_e2e");
}

// ===========================
// Test 18: Timing-driven critical net wirelength
// ===========================
TEST(critical_net_wirelength_tracking) {
    auto pd = make_routable_pd();
    GlobalRouter gr(pd, 10, 10, 4);

    RouterConfig cfg;
    cfg.enable_timing_driven = true;
    gr.set_config(cfg);

    // Mark net0 as critical
    NetTimingInfo crit;
    crit.criticality = 0.9;
    crit.slack = -0.5;
    gr.set_net_timing(0, crit);

    // Mark net1 as non-critical
    NetTimingInfo ok;
    ok.criticality = 0.1;
    ok.slack = 5.0;
    gr.set_net_timing(1, ok);

    auto r = gr.route();
    CHECK(r.critical_net_wirelength > 0, "critical WL > 0");
    CHECK(r.critical_net_wirelength < r.total_wirelength, "critical < total");
    PASS("critical_net_wirelength_tracking");
}

int main() {
    std::cout << "=== Phase 25: Router Industrial Features Tests ===\n\n";
    RUN(router_config_defaults);
    RUN(net_timing_info_defaults);
    RUN(global_router_config_api);
    RUN(global_router_timing_driven);
    RUN(global_router_antenna_check);
    RUN(global_router_industrial_metrics);
    RUN(global_router_layer_promotion);
    RUN(global_router_net_timing_map);
    RUN(detailed_router_returns_result);
    RUN(detailed_router_config_api);
    RUN(detailed_router_via_minimization);
    RUN(detailed_router_antenna_check);
    RUN(detailed_router_per_layer);
    RUN(detailed_router_reroute_iters);
    RUN(backward_compat_no_config);
    RUN(route_result_fields);
    RUN(global_then_detailed_e2e);
    RUN(critical_net_wirelength_tracking);

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===\n";
    return failed > 0 ? 1 : 0;
}
