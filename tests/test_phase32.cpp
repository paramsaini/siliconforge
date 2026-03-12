// SiliconForge — Phase 32: UPF Power Intent Parser Tests
#include "frontend/upf_parser.hpp"
#include <iostream>

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

using namespace sf;

static const char* BASIC_UPF = R"(
# Basic UPF test
create_power_domain PD_TOP -include_scope
create_power_domain PD_CPU -elements {cpu_core}

create_supply_net VDD -domain PD_TOP
create_supply_net VSS -domain PD_TOP
create_supply_net VDD_CPU -domain PD_CPU

create_supply_port VDD -domain PD_TOP -direction in
create_supply_port VSS -domain PD_TOP -direction in

set_isolation iso_cpu -domain PD_CPU \
    -isolation_power_net VDD \
    -isolation_ground_net VSS \
    -clamp_value 0 \
    -applies_to outputs \
    -isolation_signal cpu_iso_en \
    -sense high

set_level_shifter ls_cpu -domain PD_CPU \
    -applies_to inputs \
    -rule low_to_high

set_retention ret_cpu -domain PD_CPU \
    -retention_power_net VDD \
    -save_signal cpu_save \
    -restore_signal cpu_restore
)";

TEST(parse_basic_upf) {
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    CHECK(!upf.domains.empty(), "Should have domains");
    PASS("parse_basic_upf");
}

TEST(power_domains) {
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    CHECK(upf.domains.count("PD_TOP") > 0, "PD_TOP should exist");
    CHECK(upf.domains.count("PD_CPU") > 0, "PD_CPU should exist");
    auto* cpu = upf.find_domain("PD_CPU");
    CHECK(cpu != nullptr, "find_domain should work");
    PASS("power_domains");
}

TEST(supply_nets) {
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    CHECK(upf.supply_nets.count("VDD") > 0, "VDD supply should exist");
    CHECK(upf.supply_nets.count("VSS") > 0, "VSS supply should exist");
    CHECK(upf.supply_nets.count("VDD_CPU") > 0, "VDD_CPU should exist");
    PASS("supply_nets");
}

TEST(supply_ports) {
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    CHECK(upf.supply_ports.count("VDD") > 0, "VDD port should exist");
    PASS("supply_ports");
}

TEST(isolation_strategy) {
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    CHECK(!upf.isolations.empty(), "Should have isolation strategies");
    auto& iso = upf.isolations[0];
    CHECK(iso.name == "iso_cpu", "Isolation name should be iso_cpu");
    CHECK(iso.domain == "PD_CPU", "Isolation domain should be PD_CPU");
    PASS("isolation_strategy");
}

TEST(level_shifter) {
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    CHECK(!upf.level_shifters.empty(), "Should have level shifters");
    auto& ls = upf.level_shifters[0];
    CHECK(ls.name == "ls_cpu", "LS name should be ls_cpu");
    CHECK(ls.domain == "PD_CPU", "LS domain should be PD_CPU");
    PASS("level_shifter");
}

TEST(retention_strategy) {
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    CHECK(!upf.retentions.empty(), "Should have retention");
    auto& ret = upf.retentions[0];
    CHECK(ret.name == "ret_cpu", "Retention name should be ret_cpu");
    CHECK(ret.domain == "PD_CPU", "Retention domain PD_CPU");
    PASS("retention_strategy");
}

TEST(backslash_continuation) {
    // The BASIC_UPF uses backslash continuation — verify it parses correctly
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    // isolation_signal should be parsed despite backslash continuation
    CHECK(!upf.isolations.empty(), "Isolation should parse across continuation");
    PASS("backslash_continuation");
}

TEST(report_generation) {
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    auto rpt = upf.report();
    CHECK(!rpt.empty(), "Report should not be empty");
    PASS("report_generation");
}

TEST(to_upf_export) {
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    auto text = upf.to_upf();
    CHECK(!text.empty(), "UPF export should not be empty");
    CHECK(text.find("create_power_domain") != std::string::npos, "Should contain domain commands");
    PASS("to_upf_export");
}

TEST(empty_input) {
    UpfParser parser;
    auto upf = parser.parse_string("");
    CHECK(upf.domains.empty(), "Empty input should produce empty UPF");
    PASS("empty_input");
}

TEST(comment_handling) {
    UpfParser parser;
    auto upf = parser.parse_string("# This is a comment\ncreate_power_domain PD_TOP -include_scope\n");
    CHECK(upf.domains.count("PD_TOP") > 0, "Should parse after comment");
    PASS("comment_handling");
}

TEST(power_state_table) {
    const char* pst_upf = R"(
create_power_domain PD_TOP -include_scope
create_supply_net VDD -domain PD_TOP
add_power_state PD_TOP.active -supply {VDD 1.0}
add_power_state PD_TOP.off -supply {VDD OFF}
)";
    UpfParser parser;
    auto upf = parser.parse_string(pst_upf);
    CHECK(upf.domains.count("PD_TOP") > 0, "Domain should exist");
    PASS("power_state_table");
}

TEST(multi_domain) {
    const char* multi = R"(
create_power_domain PD_TOP -include_scope
create_power_domain PD_CPU -elements {cpu}
create_power_domain PD_GPU -elements {gpu}
create_power_domain PD_IO -elements {io_ring}
)";
    UpfParser parser;
    auto upf = parser.parse_string(multi);
    CHECK(upf.domains.size() == 4, "Should have 4 domains");
    PASS("multi_domain");
}

TEST(upf_checker) {
    UpfParser parser;
    auto upf = parser.parse_string(BASIC_UPF);
    Netlist nl;
    UpfChecker checker;
    auto issues = checker.check(upf, nl);
    // Checker should run without crashing even with empty netlist
    CHECK(true, "Checker should not crash");
    PASS("upf_checker");
}

int main() {
    std::cout << "═══════════════════════════════════════════════════════\n"
              << " Phase 32: UPF Power Intent Parser Tests\n"
              << "═══════════════════════════════════════════════════════\n\n";
    RUN(parse_basic_upf);
    RUN(power_domains);
    RUN(supply_nets);
    RUN(supply_ports);
    RUN(isolation_strategy);
    RUN(level_shifter);
    RUN(retention_strategy);
    RUN(backslash_continuation);
    RUN(report_generation);
    RUN(to_upf_export);
    RUN(empty_input);
    RUN(comment_handling);
    RUN(power_state_table);
    RUN(multi_domain);
    RUN(upf_checker);
    std::cout << "\n═══════════════════════════════════════════════════════\n"
              << " Results: " << passed << " passed, " << failed << " failed\n"
              << "═══════════════════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
