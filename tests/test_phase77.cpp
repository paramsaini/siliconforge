// SiliconForge — Phase 77: SystemVerilog Elaboration Engine Tests
#include "../src/frontend/sv_elaborate.hpp"
#include "../src/core/netlist.hpp"
#include <cassert>
#include <iostream>

using namespace sf;

static int tests_run = 0, tests_passed = 0;
#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { std::cerr << "FAIL: " << msg << " [" << __LINE__ << "]\n"; } \
    else { tests_passed++; } \
} while(0)

// ────────────────────────────────────────────────────────────────────
// 1. Elaborate module with no parameters
// ────────────────────────────────────────────────────────────────────
static void test_no_params() {
    SvElaborator elab;
    SvModuleTemplate tmpl;
    tmpl.name = "simple";
    tmpl.body = "assign out = in;\n";
    elab.add_module_template(tmpl);
    elab.set_top_module("simple");

    Netlist nl;
    auto res = elab.elaborate(nl);
    CHECK(res.success, "no-params elaboration succeeds");
    CHECK(res.modules_elaborated == 1, "one module elaborated");
    CHECK(nl.num_gates() == 1, "one BUF gate from assign");
}

// ────────────────────────────────────────────────────────────────────
// 2. Elaborate with parameter override (WIDTH=16)
// ────────────────────────────────────────────────────────────────────
static void test_param_override() {
    SvElaborator elab;
    SvModuleTemplate tmpl;
    tmpl.name = "parammod";
    SvParameter p; p.name = "WIDTH"; p.default_value = 8; p.type = "integer";
    tmpl.parameters.push_back(p);
    tmpl.body = "assign out = in;\n";
    elab.add_module_template(tmpl);
    elab.set_top_module("parammod");
    elab.set_parameter("parammod", "WIDTH", 16);

    Netlist nl;
    auto res = elab.elaborate(nl);
    CHECK(res.success, "param override elaboration succeeds");
    CHECK(res.parameters_resolved == 1, "one parameter resolved");
}

// ────────────────────────────────────────────────────────────────────
// 3. Parameter expression resolution (WIDTH-1 = 15)
// ────────────────────────────────────────────────────────────────────
static void test_param_expr() {
    SvElaborator elab;
    std::unordered_map<std::string, int> params = {{"WIDTH", 16}};
    int val = elab.resolve_parameter("WIDTH - 1", params);
    CHECK(val == 15, "WIDTH-1 = 15");

    val = elab.resolve_parameter("WIDTH * 2 + 3", params);
    CHECK(val == 35, "WIDTH*2+3 = 35");

    val = elab.resolve_parameter("(WIDTH + 4) / 2", params);
    CHECK(val == 10, "(WIDTH+4)/2 = 10");
}

// ────────────────────────────────────────────────────────────────────
// 4. For-generate expansion (generate 4 instances)
// ────────────────────────────────────────────────────────────────────
static void test_for_generate() {
    SvElaborator elab;
    SvGenerateBlock gen;
    gen.type = SvGenerateBlock::FOR_GENERATE;
    gen.label = "gen_buf";
    gen.iterator = "i";
    gen.start = 0; gen.end = 4; gen.step = 1;
    gen.body = "assign out = in;\n";

    Netlist nl;
    std::unordered_map<std::string, int> params;
    int count = elab.expand_for_generate(gen, nl, params);
    CHECK(count == 4, "for-generate creates 4 instances");
    CHECK(nl.num_gates() == 4, "4 BUF gates from for-generate");
}

// ────────────────────────────────────────────────────────────────────
// 5. If-generate (condition true → 1 instance)
// ────────────────────────────────────────────────────────────────────
static void test_if_generate_true() {
    SvElaborator elab;
    SvGenerateBlock gen;
    gen.type = SvGenerateBlock::IF_GENERATE;
    gen.label = "if_blk";
    gen.condition = "ENABLE";
    gen.body = "assign out = in;\n";

    Netlist nl;
    std::unordered_map<std::string, int> params = {{"ENABLE", 1}};
    int count = elab.expand_if_generate(gen, nl, params);
    CHECK(count == 1, "if-generate true → 1 instance");
    CHECK(nl.num_gates() == 1, "1 BUF gate from if-generate true");
}

// ────────────────────────────────────────────────────────────────────
// 6. If-generate (condition false → 0 instances)
// ────────────────────────────────────────────────────────────────────
static void test_if_generate_false() {
    SvElaborator elab;
    SvGenerateBlock gen;
    gen.type = SvGenerateBlock::IF_GENERATE;
    gen.label = "if_blk";
    gen.condition = "ENABLE";
    gen.body = "assign out = in;\n";

    Netlist nl;
    std::unordered_map<std::string, int> params = {{"ENABLE", 0}};
    int count = elab.expand_if_generate(gen, nl, params);
    CHECK(count == 0, "if-generate false → 0 instances");
    CHECK(nl.num_gates() == 0, "no gates from if-generate false");
}

// ────────────────────────────────────────────────────────────────────
// 7. Interface definition
// ────────────────────────────────────────────────────────────────────
static void test_interface_def() {
    SvElaborator elab;
    SvInterface iface;
    iface.name = "axi_if";
    iface.signals.push_back({"AWVALID", 1, SvInterface::Signal::OUTPUT});
    iface.signals.push_back({"AWREADY", 1, SvInterface::Signal::INPUT});
    iface.signals.push_back({"WDATA", 32, SvInterface::Signal::OUTPUT});
    elab.add_interface(iface);

    CHECK(elab.interfaces().size() == 1, "one interface stored");
    CHECK(elab.interfaces()[0].signals.size() == 3, "3 signals in interface");
}

// ────────────────────────────────────────────────────────────────────
// 8. Modport binding
// ────────────────────────────────────────────────────────────────────
static void test_modport_binding() {
    SvElaborator elab;

    SvInterface iface;
    iface.name = "bus_if";
    iface.signals.push_back({"req", 1, SvInterface::Signal::OUTPUT});
    iface.signals.push_back({"gnt", 1, SvInterface::Signal::INPUT});

    SvInterface::Modport mp;
    mp.name = "master";
    mp.port_dirs.push_back({"req", SvInterface::Signal::Dir::OUTPUT});
    mp.port_dirs.push_back({"gnt", SvInterface::Signal::Dir::INPUT});
    iface.modports.push_back(mp);
    elab.add_interface(iface);

    SvModuleTemplate tmpl;
    tmpl.name = "bus_master";
    tmpl.interface_ports.push_back("bus_if.master");
    tmpl.body = "assign out = in;\n";
    elab.add_module_template(tmpl);
    elab.set_top_module("bus_master");

    Netlist nl;
    auto res = elab.elaborate(nl);
    CHECK(res.success, "modport binding elaboration succeeds");
    CHECK(res.interfaces_bound == 1, "one interface bound");
    CHECK(nl.num_nets() >= 2, "interface signals created as nets");
}

// ────────────────────────────────────────────────────────────────────
// 9. Nested module elaboration
// ────────────────────────────────────────────────────────────────────
static void test_nested_modules() {
    SvElaborator elab;

    SvModuleTemplate child;
    child.name = "child_mod";
    child.body = "assign out = in;\n";
    elab.add_module_template(child);

    SvModuleTemplate parent;
    parent.name = "parent_mod";
    parent.body = "inst child_mod;\nassign top_out = top_in;\n";
    elab.add_module_template(parent);
    elab.set_top_module("parent_mod");

    Netlist nl;
    auto res = elab.elaborate(nl);
    CHECK(res.success, "nested elaboration succeeds");
    CHECK(res.modules_elaborated >= 2, "at least 2 modules elaborated (parent + child)");
}

// ────────────────────────────────────────────────────────────────────
// 10. Multiple parameter overrides
// ────────────────────────────────────────────────────────────────────
static void test_multi_param() {
    SvElaborator elab;
    SvModuleTemplate tmpl;
    tmpl.name = "multi_p";
    SvParameter p1; p1.name = "WIDTH"; p1.default_value = 8; p1.type = "integer";
    SvParameter p2; p2.name = "DEPTH"; p2.default_value = 4; p2.type = "integer";
    SvParameter p3; p3.name = "MODE"; p3.default_value = 0; p3.type = "integer";
    tmpl.parameters = {p1, p2, p3};
    tmpl.body = "assign out = in;\n";
    elab.add_module_template(tmpl);
    elab.set_top_module("multi_p");
    elab.set_parameter("multi_p", "WIDTH", 32);
    elab.set_parameter("multi_p", "DEPTH", 16);
    elab.set_parameter("multi_p", "MODE", 2);

    Netlist nl;
    auto res = elab.elaborate(nl);
    CHECK(res.success, "multi-param elaboration succeeds");
    CHECK(res.parameters_resolved == 3, "all 3 parameters resolved");
}

// ────────────────────────────────────────────────────────────────────
// 11. Case-generate
// ────────────────────────────────────────────────────────────────────
static void test_case_generate() {
    SvElaborator elab;
    SvGenerateBlock gen;
    gen.type = SvGenerateBlock::CASE_GENERATE;
    gen.label = "case_sel";
    gen.condition = "MODE";
    gen.case_items[0] = "assign out = in;\n";
    gen.case_items[1] = "always @(posedge clk) q <= d;\n";
    gen.default_body = "assign fallback = a;\n";

    Netlist nl;
    std::unordered_map<std::string, int> params = {{"MODE", 1}};
    int count = elab.expand_case_generate(gen, nl, params);
    CHECK(count == 1, "case-generate selects one branch");
    CHECK(nl.num_gates() == 1, "DFF created from case branch 1");

    // Test default branch
    Netlist nl2;
    std::unordered_map<std::string, int> params2 = {{"MODE", 99}};
    int count2 = elab.expand_case_generate(gen, nl2, params2);
    CHECK(count2 == 1, "case-generate falls to default");
    CHECK(nl2.num_gates() == 1, "BUF from default case body");
}

// ────────────────────────────────────────────────────────────────────
// 12. run_enhanced() full flow
// ────────────────────────────────────────────────────────────────────
static void test_run_enhanced() {
    SvElaborator elab;

    SvInterface iface;
    iface.name = "data_if";
    iface.signals.push_back({"valid", 1, SvInterface::Signal::OUTPUT});
    iface.signals.push_back({"data", 8, SvInterface::Signal::OUTPUT});
    elab.add_interface(iface);

    SvModuleTemplate tmpl;
    tmpl.name = "top_design";
    SvParameter p; p.name = "N"; p.default_value = 4; p.type = "integer";
    tmpl.parameters.push_back(p);
    tmpl.interface_ports.push_back("data_if");

    SvGenerateBlock gen;
    gen.type = SvGenerateBlock::FOR_GENERATE;
    gen.label = "pipe_stage";
    gen.iterator = "i";
    gen.start = 0; gen.end = 4; gen.step = 1;
    gen.body = "always @(posedge clk) q <= d;\n";
    tmpl.generates.push_back(gen);

    tmpl.body = "assign out = in;\n";
    elab.add_module_template(tmpl);
    elab.set_top_module("top_design");

    Netlist nl;
    auto res = elab.run_enhanced(nl);
    CHECK(res.success, "run_enhanced succeeds");
    CHECK(!res.report.empty(), "report is non-empty");
    CHECK(res.report.find("SUCCESS") != std::string::npos, "report contains SUCCESS");
    CHECK(res.instances_generated == 4, "4 pipeline stages generated");
    CHECK(res.interfaces_bound == 1, "data_if interface bound");
    CHECK(res.generate_blocks_expanded == 1, "1 generate block expanded");
}

// ────────────────────────────────────────────────────────────────────
int main() {
    test_no_params();
    test_param_override();
    test_param_expr();
    test_for_generate();
    test_if_generate_true();
    test_if_generate_false();
    test_interface_def();
    test_modport_binding();
    test_nested_modules();
    test_multi_param();
    test_case_generate();
    test_run_enhanced();

    std::cout << "Phase 77 — SV Elaboration: " << tests_passed << " / "
              << tests_run << " passed\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
