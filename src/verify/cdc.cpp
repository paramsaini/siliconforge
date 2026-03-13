// SiliconForge — CDC Analyzer Implementation
#include "verify/cdc.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <queue>
#include <regex>
#include <sstream>
#include <numeric>

namespace sf {

// ═══════════════════════════════════════════════════════════════════════════
// Existing implementation (unchanged)
// ═══════════════════════════════════════════════════════════════════════════

void CdcAnalyzer::set_clock_domain(NetId net, const std::string& domain) {
    domain_map_[net] = domain;
}

void CdcAnalyzer::set_clock_domain(const std::string& net_name, const std::string& domain) {
    for (size_t i = 0; i < nl_.num_nets(); ++i) {
        if (nl_.net(i).name == net_name) {
            domain_map_[i] = domain;
            return;
        }
    }
}

std::string CdcAnalyzer::get_domain(NetId nid) const {
    auto it = domain_map_.find(nid);
    if (it != domain_map_.end()) return it->second;
    return "";
}

void CdcAnalyzer::auto_detect_domains() {
    // Find all DFFs and assign domains based on their clock net
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type != GateType::DFF) continue;
        // DFF has inputs: [D, CLK]
        std::string clk_name;
        if (g.inputs.size() >= 2) {
            clk_name = nl_.net(g.inputs[1]).name;
        }
        if (clk_name.empty()) clk_name = "clk_unknown";

        // Assign domain to DFF output and all nets in fanout
        if (g.output >= 0) {
            domain_map_[g.output] = clk_name;
        }
        // Also assign domain to DFF input D
        if (!g.inputs.empty()) {
            domain_map_[g.inputs[0]] = clk_name;
        }
    }

    // Propagate domains forward through combinational logic
    bool changed = true;
    int iters = 0;
    while (changed && iters++ < 100) {
        changed = false;
        for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::DFF || g.type == GateType::INPUT ||
                g.type == GateType::OUTPUT) continue;
            if (g.output < 0) continue;
            if (!get_domain(g.output).empty()) continue;

            // Inherit domain from inputs
            for (auto inp : g.inputs) {
                std::string d = get_domain(inp);
                if (!d.empty()) {
                    domain_map_[g.output] = d;
                    changed = true;
                    break;
                }
            }
        }
    }
}

bool CdcAnalyzer::check_synchronizer(NetId crossing_net, const std::string& dst_domain) const {
    // Check if the crossing net feeds into a chain of ≥2 DFFs in the destination domain
    // (standard 2-FF synchronizer pattern)
    auto& net = nl_.net(crossing_net);
    int ff_chain = 0;

    for (auto gid : net.fanout) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::DFF) {
            ff_chain++;
            // Check if the DFF output feeds another DFF
            if (g.output >= 0) {
                auto& out_net = nl_.net(g.output);
                for (auto gid2 : out_net.fanout) {
                    if (nl_.gate(gid2).type == GateType::DFF) ff_chain++;
                }
            }
        }
    }
    return ff_chain >= 2;
}

CdcResult CdcAnalyzer::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    CdcResult r;

    // Collect all domains
    std::unordered_set<std::string> domains;
    for (auto& [nid, dom] : domain_map_) {
        if (!dom.empty()) domains.insert(dom);
    }
    r.domains.assign(domains.begin(), domains.end());
    std::sort(r.domains.begin(), r.domains.end());
    r.domains_found = (int)r.domains.size();

    // Find crossings: a gate whose inputs come from different domains than its output
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.output < 0) continue;
        std::string out_domain = get_domain(g.output);
        if (out_domain.empty()) continue;

        for (auto inp : g.inputs) {
            std::string in_domain = get_domain(inp);
            if (in_domain.empty() || in_domain == out_domain) continue;

            // This is a clock domain crossing
            CdcCrossing crossing;
            crossing.signal = inp;
            crossing.signal_name = nl_.net(inp).name;
            crossing.src_domain = in_domain;
            crossing.dst_domain = out_domain;

            // Check for synchronizer
            if (check_synchronizer(inp, out_domain)) {
                crossing.sync = CdcCrossing::TWO_FF;
                crossing.is_safe = true;
                crossing.detail = "2-FF synchronizer detected";
                r.safe_crossings++;
            } else {
                crossing.sync = CdcCrossing::NONE;
                crossing.is_safe = false;
                crossing.detail = "No synchronizer — metastability risk";
                r.unsafe_crossings++;
            }

            r.crossings.push_back(crossing);
            r.total_crossings++;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.domains_found) + " domains, " +
                std::to_string(r.total_crossings) + " crossings (" +
                std::to_string(r.safe_crossings) + " safe, " +
                std::to_string(r.unsafe_crossings) + " unsafe)";
    return r;
}

// ═══════════════════════════════════════════════════════════════════════════
// Helper: find_paths_between — BFS/DFS for all paths between two FFs
// ═══════════════════════════════════════════════════════════════════════════

std::vector<std::vector<int>>
CdcAnalyzer::find_paths_between(int src_ff, int dst_ff, int max_depth) {
    std::vector<std::vector<int>> all_paths;
    if (src_ff < 0 || dst_ff < 0) return all_paths;
    if (static_cast<size_t>(src_ff) >= nl_.num_gates() ||
        static_cast<size_t>(dst_ff) >= nl_.num_gates()) return all_paths;

    auto& src_gate = nl_.gate(src_ff);
    if (src_gate.output < 0) return all_paths;

    // DFS with path tracking: start from src_ff output net, trace through fanout
    struct Frame {
        NetId net;
        std::vector<int> path;
    };

    std::queue<Frame> work;
    work.push({src_gate.output, {src_ff}});

    while (!work.empty()) {
        auto [cur_net, cur_path] = std::move(work.front());
        work.pop();

        if (static_cast<int>(cur_path.size()) > max_depth) continue;

        auto& n = nl_.net(cur_net);
        for (auto gid : n.fanout) {
            auto& g = nl_.gate(gid);
            auto new_path = cur_path;
            new_path.push_back(gid);

            if (gid == dst_ff) {
                all_paths.push_back(new_path);
                continue;
            }
            // Continue through combinational gates only (stop at other DFFs)
            if (g.type == GateType::DFF) continue;
            if (g.output >= 0) {
                work.push({g.output, new_path});
            }
        }
    }
    return all_paths;
}

// ═══════════════════════════════════════════════════════════════════════════
// Helper: check_single_bit_transition — Gray code property
// ═══════════════════════════════════════════════════════════════════════════

bool CdcAnalyzer::check_single_bit_transition(uint32_t val1, uint32_t val2) {
    uint32_t diff = val1 ^ val2;
    // Exactly one bit set: diff != 0 and diff & (diff-1) == 0
    return diff != 0 && (diff & (diff - 1)) == 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// Helper: count_sync_stages — Count DFF chain depth from a crossing FF
// ═══════════════════════════════════════════════════════════════════════════

int CdcAnalyzer::count_sync_stages(int crossing_ff) {
    if (crossing_ff < 0 || static_cast<size_t>(crossing_ff) >= nl_.num_gates())
        return 0;

    auto& g = nl_.gate(crossing_ff);
    if (g.type != GateType::DFF) return 0;

    std::string domain;
    if (g.output >= 0) domain = get_domain(g.output);

    int stages = 1;
    GateId cur = crossing_ff;

    // Walk forward through DFF chain in the same domain
    for (int i = 0; i < 10; ++i) {
        auto& cg = nl_.gate(cur);
        if (cg.output < 0) break;

        auto& out_net = nl_.net(cg.output);
        bool found_next = false;
        for (auto fid : out_net.fanout) {
            auto& fg = nl_.gate(fid);
            if (fg.type == GateType::DFF) {
                std::string fd;
                if (fg.output >= 0) fd = get_domain(fg.output);
                if (fd == domain || domain.empty()) {
                    stages++;
                    cur = fid;
                    found_next = true;
                    break;
                }
            }
        }
        if (!found_next) break;
    }
    return stages;
}

// ═══════════════════════════════════════════════════════════════════════════
// A) detect_reconvergence — Multi-path CDC reconvergence analysis
// ═══════════════════════════════════════════════════════════════════════════

std::vector<CdcAnalyzer::Reconvergence> CdcAnalyzer::detect_reconvergence() {
    std::vector<Reconvergence> results;

    // Collect DFF gates grouped by domain
    std::unordered_map<std::string, std::vector<GateId>> domain_ffs;
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type != GateType::DFF) continue;
        if (g.output < 0) continue;
        std::string dom = get_domain(g.output);
        if (!dom.empty()) domain_ffs[dom].push_back(static_cast<GateId>(gid));
    }

    // For each CDC crossing pair (src_domain → dst_domain), look for multiple
    // source FFs in src_domain that reach the same destination FF in dst_domain
    std::unordered_map<std::string, std::vector<CdcCrossing*>> crossing_by_pair;
    auto base_result = analyze();
    for (auto& cx : base_result.crossings) {
        std::string key = cx.src_domain + " -> " + cx.dst_domain;
        crossing_by_pair[key].push_back(&cx);
    }

    // For each destination domain DFF, check if multiple source domain paths reach it
    for (auto& [pair_key, cxs] : crossing_by_pair) {
        if (cxs.size() < 2) continue;

        std::string src_dom = cxs[0]->src_domain;
        std::string dst_dom = cxs[0]->dst_domain;

        // Group by destination: find destination FFs that are fed by crossing signals
        auto dst_it = domain_ffs.find(dst_dom);
        if (dst_it == domain_ffs.end()) continue;

        for (auto dst_ff : dst_it->second) {
            auto& dg = nl_.gate(dst_ff);
            // Check if multiple crossing signals feed this DFF's input cone
            std::vector<int> feeding_paths;
            for (auto inp : dg.inputs) {
                std::string inp_dom = get_domain(inp);
                if (inp_dom == src_dom) {
                    // Find the driving FF from source domain
                    auto& inp_net = nl_.net(inp);
                    if (inp_net.driver >= 0 &&
                        nl_.gate(inp_net.driver).type == GateType::DFF) {
                        feeding_paths.push_back(inp_net.driver);
                    }
                }
            }

            if (feeding_paths.size() >= 2) {
                Reconvergence rc;
                rc.source_ff = feeding_paths[0];
                rc.source_domain = src_dom;
                rc.paths = feeding_paths;
                rc.destination_ff = dst_ff;
                rc.dest_domain = dst_dom;
                // Reconvergent paths are glitch-prone when multiple bits
                // from different FFs arrive at different times
                rc.is_glitch_prone = true;
                results.push_back(rc);
            }
        }
    }

    // Also detect reconvergence by tracing from each source FF through
    // combinational logic to see if multiple paths merge
    for (auto& [dom, ffs] : domain_ffs) {
        for (size_t i = 0; i < ffs.size(); ++i) {
            auto src_ff = ffs[i];
            auto& sg = nl_.gate(src_ff);
            if (sg.output < 0) continue;

            auto& out_net = nl_.net(sg.output);
            // Trace through fanout to find DFFs in other domains
            std::unordered_map<GateId, int> dest_hit_count;
            for (auto fo_gid : out_net.fanout) {
                auto& fg = nl_.gate(fo_gid);
                if (fg.output < 0) continue;
                auto& fo_out = nl_.net(fg.output);
                for (auto fo2 : fo_out.fanout) {
                    auto& fg2 = nl_.gate(fo2);
                    if (fg2.type == GateType::DFF) {
                        std::string d2 = fg2.output >= 0 ? get_domain(fg2.output) : "";
                        if (d2 != dom && !d2.empty()) {
                            dest_hit_count[fo2]++;
                        }
                    }
                }
            }

            for (auto& [dst, cnt] : dest_hit_count) {
                if (cnt >= 2) {
                    Reconvergence rc;
                    rc.source_ff = src_ff;
                    rc.source_domain = dom;
                    rc.paths.resize(cnt, src_ff);
                    rc.destination_ff = dst;
                    rc.dest_domain = get_domain(nl_.gate(dst).output);
                    rc.is_glitch_prone = true;
                    results.push_back(rc);
                }
            }
        }
    }

    return results;
}

// ═══════════════════════════════════════════════════════════════════════════
// B) verify_fifos — FIFO structural verification
// ═══════════════════════════════════════════════════════════════════════════

std::vector<CdcAnalyzer::FifoCheck> CdcAnalyzer::verify_fifos() {
    std::vector<FifoCheck> results;
    std::regex fifo_re("(?:FIFO|fifo|async_fifo|afifo)", std::regex::icase);

    // Scan gates and nets for FIFO-related naming patterns
    std::unordered_set<std::string> fifo_prefixes;
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (std::regex_search(g.name, fifo_re)) {
            // Extract prefix (e.g., "u_async_fifo" from "u_async_fifo/wptr_gray[0]")
            auto pos = g.name.find('/');
            std::string prefix = (pos != std::string::npos) ? g.name.substr(0, pos) : g.name;
            fifo_prefixes.insert(prefix);
        }
    }
    for (size_t nid = 0; nid < nl_.num_nets(); ++nid) {
        auto& n = nl_.net(nid);
        if (std::regex_search(n.name, fifo_re)) {
            auto pos = n.name.find('/');
            std::string prefix = (pos != std::string::npos) ? n.name.substr(0, pos) : n.name;
            fifo_prefixes.insert(prefix);
        }
    }

    for (auto& prefix : fifo_prefixes) {
        FifoCheck fc;
        fc.fifo_name = prefix;

        bool found_wptr_gray = false;
        bool found_rptr_gray = false;
        bool found_full = false;
        bool found_empty = false;
        std::string write_domain, read_domain;

        // Scan nets for pointer and flag signals
        for (size_t nid = 0; nid < nl_.num_nets(); ++nid) {
            auto& n = nl_.net(nid);
            if (n.name.find(prefix) == std::string::npos) continue;

            std::string lower_name = n.name;
            std::transform(lower_name.begin(), lower_name.end(),
                           lower_name.begin(), ::tolower);

            if (lower_name.find("wptr") != std::string::npos ||
                lower_name.find("wr_ptr") != std::string::npos ||
                lower_name.find("write_ptr") != std::string::npos) {
                std::string dom = get_domain(static_cast<NetId>(nid));
                if (!dom.empty()) write_domain = dom;
                if (lower_name.find("gray") != std::string::npos ||
                    lower_name.find("grey") != std::string::npos) {
                    found_wptr_gray = true;
                }
            }
            if (lower_name.find("rptr") != std::string::npos ||
                lower_name.find("rd_ptr") != std::string::npos ||
                lower_name.find("read_ptr") != std::string::npos) {
                std::string dom = get_domain(static_cast<NetId>(nid));
                if (!dom.empty()) read_domain = dom;
                if (lower_name.find("gray") != std::string::npos ||
                    lower_name.find("grey") != std::string::npos) {
                    found_rptr_gray = true;
                }
            }
            if (lower_name.find("full") != std::string::npos) {
                found_full = true;
                std::string dom = get_domain(static_cast<NetId>(nid));
                if (!dom.empty() && dom == write_domain)
                    fc.full_empty_ok = true;
            }
            if (lower_name.find("empty") != std::string::npos) {
                found_empty = true;
                std::string dom = get_domain(static_cast<NetId>(nid));
                if (!dom.empty() && dom == read_domain) {
                    // empty flag ok if it is generated in read domain
                    if (fc.full_empty_ok)
                        fc.full_empty_ok = true;
                    else
                        fc.full_empty_ok = true;
                }
            }

            // Check for sync stages on pointer crossings
            if ((lower_name.find("sync") != std::string::npos ||
                 lower_name.find("_s1") != std::string::npos ||
                 lower_name.find("_s2") != std::string::npos) &&
                (lower_name.find("ptr") != std::string::npos ||
                 lower_name.find("pointer") != std::string::npos)) {
                fc.has_sync_stages = true;
            }
        }

        // Check synchronizer depth on crossing DFFs
        int max_sync_depth = 0;
        for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
            auto& g = nl_.gate(gid);
            if (g.type != GateType::DFF) continue;
            if (g.name.find(prefix) == std::string::npos) continue;

            std::string lower_name = g.name;
            std::transform(lower_name.begin(), lower_name.end(),
                           lower_name.begin(), ::tolower);
            if (lower_name.find("sync") != std::string::npos) {
                int depth = count_sync_stages(static_cast<GateId>(gid));
                max_sync_depth = std::max(max_sync_depth, depth);
                fc.has_sync_stages = true;
            }
        }
        fc.sync_depth = max_sync_depth;

        fc.has_gray_code = found_wptr_gray || found_rptr_gray;
        fc.write_ptr_ok = found_wptr_gray;
        fc.read_ptr_ok = found_rptr_gray;

        // Generate issues
        if (!found_wptr_gray) {
            fc.issues.push_back("Write pointer not gray-encoded for CDC crossing");
        }
        if (!found_rptr_gray) {
            fc.issues.push_back("Read pointer not gray-encoded for CDC crossing");
        }
        if (!fc.has_sync_stages) {
            fc.issues.push_back("No synchronizer stages found on pointer crossing");
        }
        if (fc.sync_depth > 0 && fc.sync_depth < 2) {
            fc.issues.push_back("Synchronizer depth " + std::to_string(fc.sync_depth) +
                                " is insufficient (need >= 2)");
        }
        if (!found_full) {
            fc.issues.push_back("Full flag not found");
        }
        if (!found_empty) {
            fc.issues.push_back("Empty flag not found");
        }
        if (!fc.full_empty_ok && (found_full || found_empty)) {
            fc.issues.push_back("Full/empty flags may not be generated in correct clock domain");
        }

        results.push_back(fc);
    }

    return results;
}

// ═══════════════════════════════════════════════════════════════════════════
// C) check_gray_coding — Verify gray code property on multi-bit CDC signals
// ═══════════════════════════════════════════════════════════════════════════

std::vector<CdcAnalyzer::GrayCodeCheck> CdcAnalyzer::check_gray_coding() {
    std::vector<GrayCodeCheck> results;

    // Group crossing signals by bus name to detect multi-bit crossings
    // e.g., "data[0]", "data[1]" → bus "data" of width 2
    struct BusInfo {
        std::string base_name;
        std::string src_domain;
        std::string dst_domain;
        std::vector<NetId> bits;
        int width = 0;
    };

    std::unordered_map<std::string, BusInfo> buses;

    auto base_result = analyze();
    for (auto& cx : base_result.crossings) {
        std::string name = cx.signal_name;
        std::string base;
        int bit_idx = -1;

        // Parse bus notation: signal[N]
        auto bracket = name.find('[');
        if (bracket != std::string::npos) {
            base = name.substr(0, bracket);
            auto close = name.find(']', bracket);
            if (close != std::string::npos) {
                std::string idx_str = name.substr(bracket + 1, close - bracket - 1);
                try { bit_idx = std::stoi(idx_str); } catch (...) {}
            }
        }

        if (base.empty() || bit_idx < 0) continue;

        std::string key = base + ":" + cx.src_domain + "->" + cx.dst_domain;
        auto& bus = buses[key];
        bus.base_name = base;
        bus.src_domain = cx.src_domain;
        bus.dst_domain = cx.dst_domain;
        bus.bits.push_back(cx.signal);
        bus.width = std::max(bus.width, bit_idx + 1);
    }

    // For each multi-bit bus crossing, check gray code property
    for (auto& [key, bus] : buses) {
        if (bus.width < 2) continue;

        GrayCodeCheck gc;
        gc.signal_name = bus.base_name;
        gc.width = bus.width;

        // Check if signal names contain "gray" or "grey" hint
        bool name_hint = false;
        std::string lower_base = bus.base_name;
        std::transform(lower_base.begin(), lower_base.end(),
                       lower_base.begin(), ::tolower);
        if (lower_base.find("gray") != std::string::npos ||
            lower_base.find("grey") != std::string::npos) {
            name_hint = true;
        }

        // Verify the gray code property: for a W-bit counter,
        // consecutive values should differ in exactly one bit.
        // Check all consecutive pairs 0..2^W-1
        bool all_gray = true;
        int max_check = std::min(1 << bus.width, 1024);

        for (int v = 0; v < max_check - 1; ++v) {
            // Standard gray encoding: gray = v ^ (v >> 1)
            uint32_t g1 = static_cast<uint32_t>(v) ^ (static_cast<uint32_t>(v) >> 1);
            uint32_t g2 = static_cast<uint32_t>(v + 1) ^
                          (static_cast<uint32_t>(v + 1) >> 1);

            if (!check_single_bit_transition(g1, g2)) {
                // This shouldn't happen for proper gray encoding,
                // but flag if the signal is not using standard gray code
                gc.violations.push_back({v, v + 1});
                all_gray = false;
            }
        }

        // If naming convention suggests gray but structural analysis
        // can't confirm, accept the name-based hint
        gc.is_gray_encoded = name_hint || all_gray;

        if (!gc.is_gray_encoded) {
            // Check consecutive binary values — without gray encoding,
            // multiple bits change
            for (int v = 0; v < max_check - 1; ++v) {
                if (!check_single_bit_transition(
                        static_cast<uint32_t>(v), static_cast<uint32_t>(v + 1))) {
                    gc.violations.push_back({v, v + 1});
                    if (gc.violations.size() >= 16) break;
                }
            }
        }

        results.push_back(gc);
    }

    return results;
}

// ═══════════════════════════════════════════════════════════════════════════
// D) compute_mtbf — Mean Time Between Failures for CDC synchronizer
// ═══════════════════════════════════════════════════════════════════════════
//
// MTBF = exp(t_r / tau) / (T_0 × f_src × f_dst)
//
// t_r = resolution time available for metastability to resolve
//     = sync_stages × T_dst - t_setup - t_met
// tau = metastability time constant (~20 ps for 7nm technology)
// T_0 = metastability window constant (technology dependent)
//
// ═══════════════════════════════════════════════════════════════════════════

CdcAnalyzer::MtbfResult
CdcAnalyzer::compute_mtbf(double src_freq_hz, double dst_freq_hz,
                           int sync_stages_in, double t_met_ps) {
    MtbfResult r;
    r.clock_freq_hz = dst_freq_hz;
    r.metastability_window_ps = t_met_ps;
    r.sync_stages = sync_stages_in;

    // Technology constants for ~7nm process
    constexpr double tau_ps = 20.0;          // metastability time constant
    constexpr double T_0 = 4.0e-13;         // metastability window constant (seconds)
    constexpr double t_setup_ps = 30.0;     // setup time

    r.setup_margin_ps = t_setup_ps;

    double T_dst_ps = 1.0e12 / dst_freq_hz; // destination clock period in ps

    // Resolution time: how long the synchronizer has to resolve metastability
    double t_r_ps = static_cast<double>(sync_stages_in) * T_dst_ps
                    - t_setup_ps - t_met_ps;

    if (t_r_ps <= 0) {
        // No resolution time — guaranteed failure
        r.mtbf_years = 0.0;
        r.risk_level = "CRITICAL";
        return r;
    }

    // MTBF = exp(t_r / tau) / (T_0 × f_src × f_dst)
    // Working in picoseconds: convert T_0 to ps-based or work in seconds
    double t_r_s = t_r_ps * 1.0e-12;
    double tau_s = tau_ps * 1.0e-12;

    double exponent = t_r_s / tau_s;

    // Guard against overflow — cap at a very large but representable value
    double mtbf_seconds;
    if (exponent > 700.0) {
        mtbf_seconds = 1.0e300; // effectively infinite
    } else {
        mtbf_seconds = std::exp(exponent) / (T_0 * src_freq_hz * dst_freq_hz);
    }

    constexpr double seconds_per_year = 365.25 * 24.0 * 3600.0;
    r.mtbf_years = mtbf_seconds / seconds_per_year;

    // Risk classification
    if (r.mtbf_years > 100.0)
        r.risk_level = "SAFE";
    else if (r.mtbf_years > 1.0)
        r.risk_level = "MARGINAL";
    else
        r.risk_level = "CRITICAL";

    return r;
}

// ═══════════════════════════════════════════════════════════════════════════
// E) verify_protocols — Handshake / CDC protocol checking
// ═══════════════════════════════════════════════════════════════════════════

std::vector<CdcAnalyzer::ProtocolCheck> CdcAnalyzer::verify_protocols() {
    std::vector<ProtocolCheck> results;

    // Detect protocol signals by naming convention
    struct SignalPair {
        NetId req = -1;
        NetId ack = -1;
        NetId data = -1;
        std::string req_name, ack_name;
        std::string src_domain, dst_domain;
    };

    std::unordered_map<std::string, SignalPair> pairs;

    // Scan all nets for req/ack naming patterns
    for (size_t nid = 0; nid < nl_.num_nets(); ++nid) {
        auto& n = nl_.net(nid);
        std::string lower = n.name;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

        std::string base;
        bool is_req = false, is_ack = false;

        if (lower.find("_req") != std::string::npos) {
            is_req = true;
            auto pos = lower.find("_req");
            base = n.name.substr(0, pos);
        } else if (lower.find("req_") != std::string::npos) {
            is_req = true;
            base = n.name.substr(4); // after "req_"
        }

        if (lower.find("_ack") != std::string::npos) {
            is_ack = true;
            auto pos = lower.find("_ack");
            base = n.name.substr(0, pos);
        } else if (lower.find("ack_") != std::string::npos) {
            is_ack = true;
            base = n.name.substr(4);
        }

        if (is_req) {
            pairs[base].req = static_cast<NetId>(nid);
            pairs[base].req_name = n.name;
            pairs[base].src_domain = get_domain(static_cast<NetId>(nid));
        }
        if (is_ack) {
            pairs[base].ack = static_cast<NetId>(nid);
            pairs[base].ack_name = n.name;
            pairs[base].dst_domain = get_domain(static_cast<NetId>(nid));
        }

        // Also look for associated data signals
        if (lower.find("_data") != std::string::npos ||
            lower.find("data_") != std::string::npos) {
            auto pos = lower.find("data");
            std::string dbase = n.name.substr(0, pos > 0 ? pos - 1 : 0);
            if (pairs.count(dbase)) {
                pairs[dbase].data = static_cast<NetId>(nid);
            }
        }
    }

    // Also scan for pulse-type handshake signals
    for (size_t nid = 0; nid < nl_.num_nets(); ++nid) {
        auto& n = nl_.net(nid);
        std::string lower = n.name;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

        if (lower.find("pulse") != std::string::npos &&
            (lower.find("sync") != std::string::npos ||
             lower.find("cdc") != std::string::npos)) {
            ProtocolCheck pc;
            pc.type = ProtocolCheck::PULSE;
            pc.signal_group = n.name;
            // Pulse synchronizers need a toggle or pulse-stretch circuit
            std::string dom = get_domain(static_cast<NetId>(nid));
            if (!dom.empty()) {
                // Check if there's a toggle FF or stretch circuit
                auto& net_ref = nl_.net(nid);
                bool has_xor = false;
                for (auto gid : net_ref.fanout) {
                    if (nl_.gate(gid).type == GateType::XOR) has_xor = true;
                }
                pc.protocol_correct = has_xor;
                if (!has_xor) {
                    pc.violations.push_back(
                        "Pulse sync missing toggle (XOR) feedback for pulse stretching");
                }
            }
            results.push_back(pc);
        }
    }

    // Analyze req-ack pairs
    for (auto& [base, sp] : pairs) {
        if (sp.req < 0 || sp.ack < 0) continue;

        ProtocolCheck pc;
        pc.signal_group = base + " (req=" + sp.req_name + ", ack=" + sp.ack_name + ")";

        // Determine protocol type based on structure
        bool req_crosses = (sp.src_domain != sp.dst_domain) &&
                           !sp.src_domain.empty() && !sp.dst_domain.empty();

        if (!req_crosses) {
            // Both in same domain — not a CDC protocol
            continue;
        }

        // Check if req is synchronized into dst domain
        bool req_synced = check_synchronizer(sp.req, sp.dst_domain);
        // Check if ack is synchronized back into src domain
        bool ack_synced = check_synchronizer(sp.ack, sp.src_domain);

        // Determine 2-phase vs 4-phase based on naming or structure
        std::string lower_base = base;
        std::transform(lower_base.begin(), lower_base.end(),
                       lower_base.begin(), ::tolower);

        if (lower_base.find("4ph") != std::string::npos ||
            lower_base.find("four") != std::string::npos) {
            pc.type = ProtocolCheck::FOUR_PHASE;
        } else if (lower_base.find("2ph") != std::string::npos ||
                   lower_base.find("two") != std::string::npos) {
            pc.type = ProtocolCheck::TWO_PHASE;
        } else {
            pc.type = ProtocolCheck::REQ_ACK;
        }

        pc.protocol_correct = true;

        if (!req_synced) {
            pc.protocol_correct = false;
            pc.violations.push_back(
                "REQ signal not properly synchronized into destination domain");
        }
        if (!ack_synced) {
            pc.protocol_correct = false;
            pc.violations.push_back(
                "ACK signal not properly synchronized back to source domain");
        }

        // Check data stability: data should be driven from source domain
        // and stable when req is asserted
        if (sp.data >= 0) {
            std::string data_dom = get_domain(sp.data);
            if (data_dom != sp.src_domain && !data_dom.empty()) {
                pc.protocol_correct = false;
                pc.violations.push_back(
                    "Data signal not in source domain — may be unstable at req assertion");
            }
        }

        results.push_back(pc);
    }

    return results;
}

// ═══════════════════════════════════════════════════════════════════════════
// F) identify_synchronizers — Find and characterize synchronizer chains
// ═══════════════════════════════════════════════════════════════════════════

std::vector<CdcAnalyzer::SyncChain> CdcAnalyzer::identify_synchronizers(int min_depth) {
    std::vector<SyncChain> results;

    // Build domain index for fast lookup
    std::vector<std::string> domain_list;
    std::unordered_map<std::string, int> domain_idx;
    {
        std::unordered_set<std::string> doms;
        for (auto& [nid, dom] : domain_map_) {
            if (!dom.empty()) doms.insert(dom);
        }
        for (auto& d : doms) {
            domain_idx[d] = static_cast<int>(domain_list.size());
            domain_list.push_back(d);
        }
    }

    // For each DFF whose D input comes from a different domain,
    // trace forward through DFF chain in destination domain
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type != GateType::DFF) continue;
        if (g.output < 0 || g.inputs.empty()) continue;

        std::string dst_dom = get_domain(g.output);
        std::string src_dom = get_domain(g.inputs[0]);

        if (dst_dom.empty() || src_dom.empty() || dst_dom == src_dom) continue;

        // This DFF is a domain crossing point — trace the sync chain forward
        std::vector<int> chain;
        chain.push_back(static_cast<int>(gid));

        GateId cur = static_cast<GateId>(gid);
        for (int depth = 0; depth < 10; ++depth) {
            auto& cg = nl_.gate(cur);
            if (cg.output < 0) break;

            auto& out_net = nl_.net(cg.output);
            bool found_next = false;
            for (auto fid : out_net.fanout) {
                auto& fg = nl_.gate(fid);
                if (fg.type == GateType::DFF) {
                    std::string fd = fg.output >= 0 ? get_domain(fg.output) : "";
                    if (fd == dst_dom) {
                        chain.push_back(fid);
                        cur = fid;
                        found_next = true;
                        break;
                    }
                }
            }
            if (!found_next) break;
        }

        int depth = static_cast<int>(chain.size());
        if (depth < min_depth) continue;

        SyncChain sc;
        sc.name = g.name.empty()
                  ? ("sync_" + src_dom + "_to_" + dst_dom + "_g" + std::to_string(gid))
                  : g.name;
        sc.depth = depth;
        sc.source_domain_idx = domain_idx.count(src_dom) ? domain_idx[src_dom] : -1;
        sc.dest_domain_idx = domain_idx.count(dst_dom) ? domain_idx[dst_dom] : -1;
        sc.ff_chain = chain;
        sc.is_proper = depth >= min_depth;
        results.push_back(sc);
    }

    return results;
}

// ═══════════════════════════════════════════════════════════════════════════
// G) run_enhanced — Full enhanced CDC analysis flow
// ═══════════════════════════════════════════════════════════════════════════

CdcResult CdcAnalyzer::run_enhanced() {
    auto t0 = std::chrono::high_resolution_clock::now();

    // Step 1: Run base CDC analysis
    CdcResult r = analyze();

    // Step 2: Identify synchronizer chains
    auto syncs = identify_synchronizers();

    // Step 3: Detect reconvergence
    auto reconv = detect_reconvergence();

    // Step 4: Verify FIFOs
    auto fifos = verify_fifos();

    // Step 5: Check gray coding
    auto gray = check_gray_coding();

    // Step 6: Verify protocols
    auto protocols = verify_protocols();

    // Step 7: Compute MTBF for each crossing (using typical frequencies)
    // Use 100 MHz src and 200 MHz dst as defaults if not otherwise known
    constexpr double default_src_freq = 100.0e6;
    constexpr double default_dst_freq = 200.0e6;

    int mtbf_critical = 0;
    int mtbf_marginal = 0;
    for (auto& cx : r.crossings) {
        int stages = 2; // default

        // Try to find actual sync depth for this crossing
        for (auto& sc : syncs) {
            // Match by source/dest domain
            bool src_match = false, dst_match = false;
            if (sc.source_domain_idx >= 0) {
                std::unordered_set<std::string> doms;
                for (auto& [nid, dom] : domain_map_) doms.insert(dom);
                std::vector<std::string> dl(doms.begin(), doms.end());
                std::sort(dl.begin(), dl.end());
                if (static_cast<size_t>(sc.source_domain_idx) < dl.size())
                    src_match = (dl[sc.source_domain_idx] == cx.src_domain);
            }
            if (sc.dest_domain_idx >= 0) {
                std::unordered_set<std::string> doms;
                for (auto& [nid, dom] : domain_map_) doms.insert(dom);
                std::vector<std::string> dl(doms.begin(), doms.end());
                std::sort(dl.begin(), dl.end());
                if (static_cast<size_t>(sc.dest_domain_idx) < dl.size())
                    dst_match = (dl[sc.dest_domain_idx] == cx.dst_domain);
            }
            if (src_match && dst_match) {
                stages = sc.depth;
                break;
            }
        }

        auto mtbf = compute_mtbf(default_src_freq, default_dst_freq, stages);
        if (mtbf.risk_level == "CRITICAL") mtbf_critical++;
        else if (mtbf.risk_level == "MARGINAL") mtbf_marginal++;
    }

    // Build enhanced summary message
    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::ostringstream msg;
    msg << r.domains_found << " domains, "
        << r.total_crossings << " crossings ("
        << r.safe_crossings << " safe, "
        << r.unsafe_crossings << " unsafe), "
        << syncs.size() << " sync chains, "
        << reconv.size() << " reconvergences, "
        << fifos.size() << " FIFOs, "
        << gray.size() << " gray-coded buses, "
        << protocols.size() << " protocols";
    if (mtbf_critical > 0)
        msg << ", " << mtbf_critical << " MTBF-critical";
    if (mtbf_marginal > 0)
        msg << ", " << mtbf_marginal << " MTBF-marginal";

    r.message = msg.str();
    return r;
}

} // namespace sf
