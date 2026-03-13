#pragma once
// SiliconForge — Clock Domain Crossing (CDC) Analyzer
// Detects signals crossing between clock domains and checks for
// proper synchronization (2-FF, pulse sync, handshake).
// Reference: Cummings, "Clock Domain Crossing (CDC) Design & Verification", SNUG 2008

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <cmath>
#include <utility>

namespace sf {

struct CdcCrossing {
    NetId signal;
    std::string signal_name;
    std::string src_domain;
    std::string dst_domain;
    enum SyncType { NONE, TWO_FF, PULSE_SYNC, HANDSHAKE, MUX_SYNC } sync = NONE;
    bool is_safe = false;
    std::string detail;
};

struct CdcResult {
    int total_crossings = 0;
    int safe_crossings = 0;
    int unsafe_crossings = 0;
    int domains_found = 0;
    double time_ms = 0;
    std::vector<CdcCrossing> crossings;
    std::vector<std::string> domains;
    std::string message;
};

class CdcAnalyzer {
public:
    explicit CdcAnalyzer(const Netlist& nl) : nl_(nl) {}

    // Assign clock domain to a net
    void set_clock_domain(NetId net, const std::string& domain);
    void set_clock_domain(const std::string& net_name, const std::string& domain);

    // Auto-detect domains from DFF clock inputs
    void auto_detect_domains();

    CdcResult analyze();

    // ── Reconvergence detection ──────────────────────────────────────
    struct Reconvergence {
        int source_ff;
        std::string source_domain;
        std::vector<int> paths;         // multiple paths to destination
        int destination_ff;
        std::string dest_domain;
        bool is_glitch_prone;
    };
    std::vector<Reconvergence> detect_reconvergence();

    // ── FIFO synchronizer verification ───────────────────────────────
    struct FifoCheck {
        std::string fifo_name;
        bool has_gray_code = false;
        bool has_sync_stages = false;
        int sync_depth = 0;
        bool write_ptr_ok = false;
        bool read_ptr_ok = false;
        bool full_empty_ok = false;
        std::vector<std::string> issues;
    };
    std::vector<FifoCheck> verify_fifos();

    // ── Gray code verification ───────────────────────────────────────
    struct GrayCodeCheck {
        std::string signal_name;
        int width;
        bool is_gray_encoded = false;
        std::vector<std::pair<int,int>> violations;
    };
    std::vector<GrayCodeCheck> check_gray_coding();

    // ── MTBF calculation ─────────────────────────────────────────────
    struct MtbfResult {
        double mtbf_years;
        double clock_freq_hz;
        double metastability_window_ps;
        double sync_stages;
        double setup_margin_ps;
        std::string risk_level;        // "SAFE", "MARGINAL", "CRITICAL"
    };
    MtbfResult compute_mtbf(double src_freq_hz, double dst_freq_hz,
                            int sync_stages = 2, double t_met_ps = 50.0);

    // ── CDC protocol verification ────────────────────────────────────
    struct ProtocolCheck {
        enum Protocol { TWO_PHASE, FOUR_PHASE, REQ_ACK, PULSE, UNKNOWN } type;
        std::string signal_group;
        bool protocol_correct = false;
        std::vector<std::string> violations;
    };
    std::vector<ProtocolCheck> verify_protocols();

    // ── Synchronizer chain identification ────────────────────────────
    struct SyncChain {
        std::string name;
        int depth;
        int source_domain_idx;
        int dest_domain_idx;
        std::vector<int> ff_chain;
        bool is_proper;
    };
    std::vector<SyncChain> identify_synchronizers(int min_depth = 2);

    // ── Enhanced CDC run ─────────────────────────────────────────────
    CdcResult run_enhanced();

private:
    const Netlist& nl_;
    std::unordered_map<NetId, std::string> domain_map_;

    std::string get_domain(NetId nid) const;
    bool check_synchronizer(NetId crossing_net, const std::string& dst_domain) const;

    // Helpers
    std::vector<std::vector<int>> find_paths_between(int src_ff, int dst_ff, int max_depth = 20);
    bool check_single_bit_transition(uint32_t val1, uint32_t val2);
    int count_sync_stages(int crossing_ff);
};

} // namespace sf
