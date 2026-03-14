// SiliconForge — Phase 73 Test Suite
// Tests: Scan Compression (LBIST, EDT, DROST), Fault Dictionary, Fault Diagnosis

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "dft/scan_compress.hpp"
#include "dft/fault_sim.hpp"
#include "dft/podem.hpp"
#include <iostream>
#include <set>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// ── Test circuit: 6 gates, 3 PIs, 2 POs ─────────────────────────────────────
//   y1 = (a AND b) OR (NOT c)
//   y2 = (a XOR c) AND (b OR c)
// Nets: a=0, b=1, c=2, n1=3, n2=4, n3=5, n4=6, y1=7, y2=8

static Netlist make_test_circuit() {
    Netlist nl;
    NetId a  = nl.add_net("a");  nl.mark_input(a);
    NetId b  = nl.add_net("b");  nl.mark_input(b);
    NetId c  = nl.add_net("c");  nl.mark_input(c);
    NetId n1 = nl.add_net("n1");
    NetId n2 = nl.add_net("n2");
    NetId n3 = nl.add_net("n3");
    NetId n4 = nl.add_net("n4");
    NetId y1 = nl.add_net("y1"); nl.mark_output(y1);
    NetId y2 = nl.add_net("y2"); nl.mark_output(y2);
    nl.add_gate(GateType::AND, {a, b},   n1, "G1");
    nl.add_gate(GateType::NOT, {c},      n2, "G2");
    nl.add_gate(GateType::OR,  {n1, n2}, y1, "G3");
    nl.add_gate(GateType::XOR, {a, c},   n3, "G4");
    nl.add_gate(GateType::OR,  {b, c},   n4, "G5");
    nl.add_gate(GateType::AND, {n3, n4}, y2, "G6");
    return nl;
}

// ── Scan Compression Tests ───────────────────────────────────────────────────

TEST(lfsr_nonzero) {
    Netlist nl;
    ScanCompressor sc(nl);
    uint32_t state = 0xACE1u;
    for (int i = 0; i < 100; ++i) {
        state = sc.lfsr_next(state, 32);
        CHECK(state != 0, "LFSR should never reach zero");
    }
    PASS("lfsr_nonzero");
}

TEST(lfsr_maximal) {
    Netlist nl;
    ScanCompressor sc(nl);
    int width = 4;
    uint32_t state = 1;
    std::set<uint32_t> seen;
    int count = 0;
    do {
        seen.insert(state);
        state = sc.lfsr_next(state, width);
        count++;
    } while (state != 1 && count < 100);
    CHECK(seen.size() == 15, "LFSR 4-bit should have 2^4-1=15 unique states");
    CHECK(state == 1, "LFSR should cycle back to initial state");
    PASS("lfsr_maximal");
}

TEST(misr_deterministic) {
    Netlist nl;
    ScanCompressor sc(nl);
    // Compress same data sequence twice — must produce identical signature
    uint32_t misr1 = 0, misr2 = 0;
    uint32_t data[] = {0xDEADBEEF, 0xCAFEBABE, 0x12345678, 0x9ABCDEF0};
    for (auto d : data) misr1 = sc.misr_compress(misr1, d);
    for (auto d : data) misr2 = sc.misr_compress(misr2, d);
    CHECK(misr1 == misr2, "MISR should be deterministic");
    CHECK(misr1 != 0, "MISR signature should be non-zero for non-zero input");
    PASS("misr_deterministic");
}

TEST(compress_reduces_patterns) {
    auto nl = make_test_circuit();
    ScanCompressor sc(nl);
    ScanCompressConfig cfg;
    cfg.compression_ratio = 5;
    auto result = sc.compress(cfg);
    CHECK(result.compressed_patterns < result.original_patterns,
          "Compressed patterns should be less than original");
    CHECK(result.decompressor_gates > 0, "Should have decompressor overhead");
    CHECK(result.compactor_gates > 0, "Should have compactor overhead");
    PASS("compress_reduces_patterns");
}

TEST(lbist_combinational) {
    auto nl = make_test_circuit();
    ScanCompressor sc(nl);
    ScanCompressConfig cfg;
    cfg.lbist_patterns = 200;
    auto result = sc.run_lbist(cfg);
    CHECK(result.patterns_applied == 200, "Should apply requested pattern count");
    CHECK(result.faults_detected > 0, "LBIST should detect some faults");
    CHECK(result.coverage_pct > 0, "Coverage should be positive");
    CHECK(result.total_faults > 0, "Should enumerate faults");
    PASS("lbist_combinational");
}

TEST(lbist_signature_match) {
    auto nl = make_test_circuit();
    ScanCompressor sc(nl);
    ScanCompressConfig cfg;
    cfg.lbist_patterns = 100;
    auto r1 = sc.run_lbist(cfg);
    auto r2 = sc.run_lbist(cfg);
    CHECK(r1.signature_match, "LBIST golden signature should match");
    CHECK(r1.misr_signature == r2.misr_signature,
          "Same config should produce same MISR signature");
    PASS("lbist_signature_match");
}

// ── Fault Dictionary & Diagnosis Tests ───────────────────────────────────────

TEST(fault_dictionary_build) {
    auto nl = make_test_circuit();
    FaultSimulator sim(nl);
    std::vector<std::vector<Logic4>> vectors = {
        {Logic4::ZERO, Logic4::ZERO, Logic4::ONE},
        {Logic4::ONE,  Logic4::ONE,  Logic4::ZERO},
        {Logic4::ZERO, Logic4::ONE,  Logic4::ONE},
        {Logic4::ONE,  Logic4::ZERO, Logic4::ONE}
    };
    auto dict = sim.build_fault_dictionary(vectors);
    CHECK(dict.total_faults > 0, "Dictionary should enumerate faults");
    CHECK(!dict.entries.empty(), "Dictionary should have entries");
    bool any_detected = false;
    for (auto& e : dict.entries)
        if (!e.detecting_patterns.empty()) { any_detected = true; break; }
    CHECK(any_detected, "At least one fault should be detected");
    PASS("fault_dictionary_build");
}

TEST(fault_dictionary_all_sa) {
    auto nl = make_test_circuit();
    FaultSimulator sim(nl);
    std::vector<std::vector<Logic4>> vectors = {
        {Logic4::ZERO, Logic4::ZERO, Logic4::ZERO},
        {Logic4::ONE,  Logic4::ONE,  Logic4::ONE},
        {Logic4::ONE,  Logic4::ZERO, Logic4::ONE},
        {Logic4::ZERO, Logic4::ONE,  Logic4::ZERO}
    };
    auto dict = sim.build_fault_dictionary(vectors);
    // 9 nets × 2 stuck-at values = 18 total faults
    CHECK(dict.total_faults == 18, "Should have 2 faults per net (SA0, SA1)");
    // Each entry should have a valid fault
    for (auto& e : dict.entries)
        CHECK(e.fault.stuck_at == Logic4::ZERO || e.fault.stuck_at == Logic4::ONE,
              "Faults should be SA0 or SA1");
    PASS("fault_dictionary_all_sa");
}

TEST(fault_diagnosis) {
    auto nl = make_test_circuit();
    FaultSimulator sim(nl);
    // Inject SA1 on net n1 (index 3): AND output stuck-at-1
    // Good responses (y1, y2) for these vectors:
    //   (0,0,1) → (0,1)    faulty: (1,1) — y1 differs
    //   (1,1,0) → (1,1)    faulty: (1,1) — same (n1 already 1)
    //   (0,1,1) → (0,1)    faulty: (1,1) — y1 differs
    //   (1,0,1) → (0,0)    faulty: (1,0) — y1 differs
    std::vector<std::vector<Logic4>> vectors = {
        {Logic4::ZERO, Logic4::ZERO, Logic4::ONE},
        {Logic4::ONE,  Logic4::ONE,  Logic4::ZERO},
        {Logic4::ZERO, Logic4::ONE,  Logic4::ONE},
        {Logic4::ONE,  Logic4::ZERO, Logic4::ONE}
    };
    std::vector<std::vector<Logic4>> observed = {
        {Logic4::ONE,  Logic4::ONE},   // pattern 0: y1 stuck
        {Logic4::ONE,  Logic4::ONE},   // pattern 1: same
        {Logic4::ONE,  Logic4::ONE},   // pattern 2: y1 stuck
        {Logic4::ONE,  Logic4::ZERO}   // pattern 3: y1 stuck
    };
    auto diag = sim.diagnose(vectors, observed);
    CHECK(!diag.candidate_faults.empty(), "Should identify candidate faults");
    CHECK(diag.confidence > 0, "Confidence should be positive");
    // SA1 on n1 (net 3) should be among top candidates
    bool found_sa1_n1 = false;
    for (auto& f : diag.candidate_faults)
        if (f.net == 3 && f.stuck_at == Logic4::ONE) { found_sa1_n1 = true; break; }
    CHECK(found_sa1_n1, "SA1-n1 should be among diagnosis candidates");
    PASS("fault_diagnosis");
}

TEST(enhanced_fault_sim) {
    auto nl = make_test_circuit();
    FaultSimulator sim(nl);
    // Provide minimal vectors — enhanced should add patterns for better coverage
    std::vector<std::vector<Logic4>> vectors = {
        {Logic4::ZERO, Logic4::ZERO, Logic4::ZERO}
    };
    auto basic = sim.simulate(vectors);
    auto enhanced = sim.run_enhanced(vectors);
    CHECK(enhanced.detected >= basic.detected,
          "Enhanced should detect >= basic faults");
    CHECK(enhanced.detected > basic.detected,
          "Enhanced should detect more faults with supplementary patterns");
    PASS("enhanced_fault_sim");
}

TEST(compression_ratio) {
    auto nl = make_test_circuit();
    ScanCompressor sc(nl);
    ScanCompressConfig cfg;
    cfg.compression_ratio = 5;
    auto result = sc.compress(cfg);
    CHECK(result.compression_ratio > 1.0,
          "Compression ratio should be > 1");
    CHECK(std::abs(result.compression_ratio -
          static_cast<double>(result.original_patterns) / result.compressed_patterns) < 0.01,
          "Compression ratio should equal original/compressed");
    PASS("compression_ratio");
}

TEST(run_enhanced_flow) {
    auto nl = make_test_circuit();
    ScanCompressor sc(nl);
    auto result = sc.run_enhanced();
    CHECK(result.compressed_patterns > 0, "Should produce compressed patterns");
    CHECK(result.compression_ratio > 1.0, "Should achieve compression");
    CHECK(result.coverage_pct > 0, "Should have positive coverage");
    CHECK(!result.report.empty(), "Should produce a report");
    CHECK(result.report.find("LBIST") != std::string::npos,
          "Report should include LBIST results");
    PASS("run_enhanced_flow");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 73 — Test Suite              ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Scan Compression (LFSR/MISR) ──\n";
    RUN(lfsr_nonzero);
    RUN(lfsr_maximal);
    RUN(misr_deterministic);

    std::cout << "\n── EDT Compression ──\n";
    RUN(compress_reduces_patterns);
    RUN(compression_ratio);

    std::cout << "\n── LBIST ──\n";
    RUN(lbist_combinational);
    RUN(lbist_signature_match);

    std::cout << "\n── Fault Dictionary & Diagnosis ──\n";
    RUN(fault_dictionary_build);
    RUN(fault_dictionary_all_sa);
    RUN(fault_diagnosis);

    std::cout << "\n── Enhanced Flows ──\n";
    RUN(enhanced_fault_sim);
    RUN(run_enhanced_flow);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
