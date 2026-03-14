// SiliconForge — Phase 65: Hierarchical Netlist Tests
// Validates module definition, instantiation, hierarchy flattening,
// block abstraction, and hierarchy queries.

#include "../src/core/netlist.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>

using namespace sf;

static int tests_run = 0, tests_passed = 0;
#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { std::cerr << "FAIL: " << msg << " [" << __LINE__ << "]\n"; } \
    else { tests_passed++; } \
} while(0)

// Helper: build a simple 2-level hierarchy
// top -> u_adder (adder module with 2 gates)
//     -> u_buf   (buffer module with 1 gate)
static HierarchicalNetlist build_test_hierarchy() {
    HierarchicalNetlist hier;

    // Define "adder" module: ports A, B, S
    auto& adder = hier.add_module("adder");
    adder.ports.push_back({"A", ModulePort::INPUT, 1, -1});
    adder.ports.push_back({"B", ModulePort::INPUT, 1, -1});
    adder.ports.push_back({"S", ModulePort::OUTPUT, 1, -1});
    // Internal netlist: 2 gates
    auto& anl = adder.internal_netlist;
    anl.add_net("a_in");   // 0
    anl.add_net("b_in");   // 1
    anl.add_net("w0");     // 2
    anl.add_net("s_out");  // 3
    anl.add_gate(GateType::BUF, {0, 1}, 2, "xor0"); // simplified
    anl.add_gate(GateType::BUF, {2}, 3, "and0");

    // Define "buffer" module: ports I, O
    auto& buf = hier.add_module("buffer");
    buf.ports.push_back({"I", ModulePort::INPUT, 1, -1});
    buf.ports.push_back({"O", ModulePort::OUTPUT, 1, -1});
    auto& bnl = buf.internal_netlist;
    bnl.add_net("bi");  // 0
    bnl.add_net("bo");  // 1
    bnl.add_gate(GateType::BUF, {0}, 1, "buf0");

    // Define "top" module
    auto& top = hier.add_module("top");
    top.ports.push_back({"clk", ModulePort::INPUT, 1, -1});
    top.ports.push_back({"in_a", ModulePort::INPUT, 1, -1});
    top.ports.push_back({"in_b", ModulePort::INPUT, 1, -1});
    top.ports.push_back({"out", ModulePort::OUTPUT, 1, -1});

    // top has internal nets
    auto& tnl = top.internal_netlist;
    tnl.add_net("n_clk");
    tnl.add_net("n_a");
    tnl.add_net("n_b");
    tnl.add_net("n_sum");
    tnl.add_net("n_out");

    hier.set_top("top");

    // Instantiate adder and buffer in top
    hier.add_instance("top", "u_adder", "adder",
        {{"A", "n_a"}, {"B", "n_b"}, {"S", "n_sum"}});
    hier.add_instance("top", "u_buf", "buffer",
        {{"I", "n_sum"}, {"O", "n_out"}});

    return hier;
}

// Test 1: Module creation and lookup
void test_module_creation() {
    HierarchicalNetlist hier;
    auto& m = hier.add_module("test_mod");
    m.ports.push_back({"A", ModulePort::INPUT, 1, -1});
    CHECK(hier.find_module("test_mod") != nullptr, "module found after creation");
    CHECK(hier.find_module("nonexistent") == nullptr, "null for missing module");
}

// Test 2: Top module setting
void test_top_module() {
    auto hier = build_test_hierarchy();
    CHECK(hier.top_module() == "top", "top module is 'top'");
}

// Test 3: Module names listing
void test_module_names() {
    auto hier = build_test_hierarchy();
    auto names = hier.module_names();
    CHECK(names.size() == 3, "3 modules defined");
    bool found_adder = false, found_buffer = false, found_top = false;
    for (auto& n : names) {
        if (n == "adder") found_adder = true;
        if (n == "buffer") found_buffer = true;
        if (n == "top") found_top = true;
    }
    CHECK(found_adder && found_buffer && found_top, "all module names found");
}

// Test 4: Hierarchy depth
void test_depth() {
    auto hier = build_test_hierarchy();
    int d = hier.depth();
    CHECK(d >= 1, "depth >= 1 (top has sub-modules)");
}

// Test 5: Total instances
void test_total_instances() {
    auto hier = build_test_hierarchy();
    int total = hier.total_instances();
    CHECK(total == 2, "2 instances in top (u_adder + u_buf)");
}

// Test 6: Instance addition and module lookup
void test_instances() {
    auto hier = build_test_hierarchy();
    auto* top = hier.find_module("top");
    CHECK(top != nullptr, "top module exists");
    CHECK(top->instances.size() == 2, "top has 2 instances");
    CHECK(top->instances[0].inst_name == "u_adder", "first instance is u_adder");
    CHECK(top->instances[0].module_name == "adder", "u_adder is type adder");
    CHECK(top->instances[1].inst_name == "u_buf", "second instance is u_buf");
}

// Test 7: Port lookup on ModuleDef
void test_port_lookup() {
    auto hier = build_test_hierarchy();
    auto* adder = hier.find_module("adder");
    CHECK(adder != nullptr, "adder found");
    auto* pA = adder->find_port("A");
    CHECK(pA != nullptr, "port A found");
    CHECK(pA->direction == ModulePort::INPUT, "port A is INPUT");
    auto* pS = adder->find_port("S");
    CHECK(pS != nullptr, "port S found");
    CHECK(pS->direction == ModulePort::OUTPUT, "port S is OUTPUT");
    CHECK(adder->find_port("X") == nullptr, "missing port returns null");
}

// Test 8: Flatten hierarchy
void test_flatten() {
    auto hier = build_test_hierarchy();
    Netlist flat = hier.flatten();
    // Flattened netlist should contain gates from both sub-modules
    int gate_count = 0;
    for (int i = 0; i < flat.num_gates(); i++) {
        auto& g = flat.gate(i);
        if (g.type != GateType::INPUT && g.type != GateType::OUTPUT)
            gate_count++;
    }
    // adder has 2 gates, buffer has 1 gate → total 3
    CHECK(gate_count >= 3, "flattened has >= 3 gates from sub-modules");
}

// Test 9: Elaborate validates module references
void test_elaborate() {
    auto hier = build_test_hierarchy();
    bool ok = hier.elaborate();
    CHECK(ok, "elaborate succeeds for valid hierarchy");
}

// Test 10: Block abstraction
void test_block_abstract() {
    auto hier = build_test_hierarchy();
    auto abstract = hier.create_abstract("adder");
    CHECK(abstract.module_name == "adder", "abstract for adder");
    CHECK(abstract.ports.size() == 3, "adder has 3 ports in abstract");
    // Should have delay arcs from inputs to outputs
    CHECK(abstract.arcs.size() > 0, "abstract has delay arcs");
}

// Test 11: Mutable module lookup
void test_mutable_lookup() {
    auto hier = build_test_hierarchy();
    auto* mod = hier.find_module_mut("buffer");
    CHECK(mod != nullptr, "mutable buffer found");
    // Can modify
    mod->ports.push_back({"EN", ModulePort::INPUT, 1, -1});
    CHECK(hier.find_module("buffer")->ports.size() == 3, "modification persists");
}

// Test 12: Print stats doesn't crash
void test_print_stats() {
    auto hier = build_test_hierarchy();
    std::ostringstream devnull;
    auto* old = std::cout.rdbuf(devnull.rdbuf());
    hier.print_stats();
    std::cout.rdbuf(old);
    CHECK(true, "print_stats doesn't crash");
}

int main() {
    std::cout << "=== Phase 65: Hierarchical Netlist Tests ===\n";
    test_module_creation();
    test_top_module();
    test_module_names();
    test_depth();
    test_total_instances();
    test_instances();
    test_port_lookup();
    test_flatten();
    test_elaborate();
    test_block_abstract();
    test_mutable_lookup();
    test_print_stats();
    std::cout << "Phase 65: " << tests_passed << "/" << tests_run << " passed\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
