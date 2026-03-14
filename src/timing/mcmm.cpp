// SiliconForge — MCMM Analyzer Implementation
// N corners × M modes with proper CornerDerate, OCV/AOCV/POCV, per-mode constraints.
// Industrial: scenario classification, pruning, CPPR/POCV per corner, sensitivity.
#include "timing/mcmm.hpp"
#include "core/thread_pool.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include <future>

namespace sf {

void McmmAnalyzer::load_default_corners() {
    corners_.clear();

    // SS: slow-slow (worst setup)
    PvtCorner ss;
    ss.name = "ss_0p81v_125c";
    ss.voltage = 0.81; ss.temperature_c = 125; ss.process = PvtCorner::SLOW;
    ss.delay_scale = 1.5; ss.power_scale = 0.8;
    ss.derate = {"ss_0p81v_125c", 1.30, 1.25, 1.20, 1.15,
                 1.30, 1.25, 0.85, 0.90};
    corners_.push_back(ss);

    // TT: typical
    PvtCorner tt;
    tt.name = "tt_0p90v_25c";
    tt.voltage = 0.90; tt.temperature_c = 25; tt.process = PvtCorner::TYPICAL;
    tt.delay_scale = 1.0; tt.power_scale = 1.0;
    tt.derate = {"tt_0p90v_25c", 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    corners_.push_back(tt);

    // FF: fast-fast (worst hold)
    PvtCorner ff;
    ff.name = "ff_0p99v_m40c";
    ff.voltage = 0.99; ff.temperature_c = -40; ff.process = PvtCorner::FAST;
    ff.delay_scale = 0.7; ff.power_scale = 1.3;
    ff.derate = {"ff_0p99v_m40c", 0.75, 0.80, 0.70, 0.75,
                 1.15, 1.10, 0.70, 0.75};
    corners_.push_back(ff);
}

void McmmAnalyzer::load_foundry_corners() {
    corners_.clear();

    // Standard 7-corner foundry set: SS, SF, TT, FS, FF + extreme temp variants
    auto make = [](const std::string& n, double v, double t,
                   PvtCorner::Process p, double ds, double ps,
                   double cd, double wd, double ec, double ew) {
        PvtCorner c;
        c.name = n; c.voltage = v; c.temperature_c = t;
        c.process = p; c.delay_scale = ds; c.power_scale = ps;
        c.derate = {n, cd, wd, ec, ew, cd, wd, ec, ew};
        return c;
    };

    corners_.push_back(make("ss_0p72v_125c", 0.72, 125, PvtCorner::SLOW,  1.60, 0.70, 1.40, 1.30, 1.25, 1.20));
    corners_.push_back(make("ss_0p81v_125c", 0.81, 125, PvtCorner::SLOW,  1.50, 0.80, 1.30, 1.25, 1.20, 1.15));
    corners_.push_back(make("sf_0p81v_m40c", 0.81, -40, PvtCorner::SLOW,  1.20, 0.90, 1.15, 1.10, 0.90, 0.92));
    corners_.push_back(make("tt_0p90v_25c",  0.90,  25, PvtCorner::TYPICAL, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
    corners_.push_back(make("fs_0p99v_125c", 0.99, 125, PvtCorner::FAST,  0.90, 1.10, 0.88, 0.90, 0.80, 0.85));
    corners_.push_back(make("ff_0p99v_m40c", 0.99, -40, PvtCorner::FAST,  0.70, 1.30, 0.75, 0.80, 0.70, 0.75));
    corners_.push_back(make("ff_1p10v_m40c", 1.10, -40, PvtCorner::FAST,  0.60, 1.50, 0.65, 0.70, 0.60, 0.65));
}

void McmmAnalyzer::load_default_modes(double base_freq_mhz) {
    modes_.clear();
    FunctionalMode func;
    func.name = "func"; func.clock_freq_mhz = base_freq_mhz;
    func.switching_activity = 0.15; func.description = "Functional mode";
    func.signoff = SignoffType::ALL;
    modes_.push_back(func);

    FunctionalMode scan;
    scan.name = "scan"; scan.clock_freq_mhz = base_freq_mhz * 0.1;
    scan.switching_activity = 0.50; scan.description = "Scan test mode";
    scan.signoff = SignoffType::SETUP;
    modes_.push_back(scan);

    FunctionalMode standby;
    standby.name = "standby"; standby.clock_freq_mhz = 0;
    standby.switching_activity = 0.001; standby.description = "Standby / retention";
    standby.signoff = SignoffType::LEAKAGE;
    modes_.push_back(standby);
}

// Run STA for a single corner×mode scenario with proper derating
StaResult McmmAnalyzer::run_scenario_sta(const PvtCorner& corner,
                                          const FunctionalMode& mode) {
    double period = mode.clock_period_ns;
    if (period <= 0 && mode.clock_freq_mhz > 0)
        period = 1000.0 / mode.clock_freq_mhz;
    if (period <= 0) period = 10.0; // default

    StaEngine sta(nl_, lib_, pd_);

    // Apply per-mode uncertainty
    sta.set_clock_uncertainty(mode.setup_uncertainty, mode.hold_uncertainty);

    // Apply OCV/AOCV/POCV if configured for this corner
    sta.set_ocv_mode(corner.ocv_mode);
    if (corner.ocv_mode == OcvMode::AOCV)
        sta.set_aocv_table(corner.aocv_table);
    if (corner.ocv_mode == OcvMode::POCV && config_.enable_pocv_per_corner)
        sta.set_pocv_table(corner.pocv_table);

    // Apply CPPR if enabled for this corner
    if (corner.cppr_enabled && config_.enable_cppr_per_corner)
        sta.enable_cppr();

    // Run with proper CornerDerate (not clock period scaling!)
    return sta.analyze_corner(period, config_.max_paths_per_scenario, corner.derate);
}

// ============================================================================
// Industrial: Scenario active check
// ============================================================================
bool McmmAnalyzer::is_scenario_active(const std::string& name) const {
    if (!use_active_set_) return true;
    return active_set_.count(name) > 0;
}

// ============================================================================
// Industrial: Dominated scenario pruning
// ============================================================================
void McmmAnalyzer::prune_dominated_scenarios(std::vector<McmmScenario>& scenarios) {
    if (!config_.enable_scenario_pruning) return;
    double margin = config_.pruning_slack_margin;

    // A scenario A is dominated by B if:
    //   B.wns <= A.wns - margin (B is worse in setup) AND
    //   B.hold_wns <= A.hold_wns - margin (B is worse in hold)
    // In other words, B catches all timing issues A would catch plus more.
    for (size_t i = 0; i < scenarios.size(); i++) {
        if (!scenarios[i].active) continue;
        for (size_t j = 0; j < scenarios.size(); j++) {
            if (i == j || !scenarios[j].active) continue;
            // Check if scenario j dominates scenario i
            bool setup_dominated = scenarios[j].sta.wns <= scenarios[i].sta.wns - margin;
            bool hold_dominated = scenarios[j].sta.hold_wns <= scenarios[i].sta.hold_wns - margin;
            if (setup_dominated && hold_dominated) {
                scenarios[i].dominated = true;
                break;
            }
        }
    }
}

// ============================================================================
// Industrial: Sensitivity analysis
// ============================================================================
McmmResult::Sensitivity McmmAnalyzer::compute_sensitivity(
    const PvtCorner& corner, const FunctionalMode& mode, double base_wns)
{
    McmmResult::Sensitivity sens;
    double dv = 0.01; // 10mV voltage delta
    double dt = 10.0; // 10°C temperature delta

    // Voltage sensitivity: ∂WNS/∂V
    PvtCorner cv = corner;
    cv.voltage += dv;
    auto sta_v = run_scenario_sta(cv, mode);
    sens.voltage_sens = (sta_v.wns - base_wns) / dv;

    // Temperature sensitivity: ∂WNS/∂T
    PvtCorner ct = corner;
    ct.temperature_c += dt;
    auto sta_t = run_scenario_sta(ct, mode);
    sens.temp_sens = (sta_t.wns - base_wns) / dt;

    return sens;
}

McmmResult McmmAnalyzer::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    McmmResult r;
    r.corners = (int)corners_.size();
    r.modes = (int)modes_.size();
    r.worst_setup_wns = 1e18;
    r.worst_hold_wns = 1e18;

    // Phase 1: Build scenario list and identify active vs inactive
    struct ScenarioWork {
        PvtCorner corner;
        FunctionalMode mode;
        std::string name;
        SignoffType signoff = SignoffType::ALL;
        bool active = true;
    };
    std::vector<ScenarioWork> active_work;
    std::vector<McmmScenario> inactive_scenarios;

    for (auto& corner : corners_) {
        for (auto& mode : modes_) {
            std::string sname = corner.name + "/" + mode.name;
            bool is_active = is_scenario_active(sname);
            if (!is_active) {
                McmmScenario s;
                s.corner = corner;
                s.mode = mode;
                s.scenario_name = sname;
                s.active = false;
                inactive_scenarios.push_back(std::move(s));
                continue;
            }
            ScenarioWork sw;
            sw.corner = corner;
            sw.mode = mode;
            sw.name = sname;
            if (corner.signoff != SignoffType::ALL)
                sw.signoff = corner.signoff;
            else if (mode.signoff != SignoffType::ALL)
                sw.signoff = mode.signoff;
            active_work.push_back(std::move(sw));
        }
    }

    // Phase 2: Run STA + Power for each active scenario in parallel
    std::vector<McmmScenario> results(active_work.size());
    {
        sf::ThreadPool pool;
        std::vector<std::future<void>> futures;
        futures.reserve(active_work.size());

        for (size_t i = 0; i < active_work.size(); ++i) {
            futures.push_back(pool.submit([this, i, &active_work, &results]() {
                auto& sw = active_work[i];
                McmmScenario scenario;
                scenario.corner = sw.corner;
                scenario.mode = sw.mode;
                scenario.scenario_name = sw.name;
                scenario.signoff = sw.signoff;
                scenario.active = true;

                // STA
                if (sw.mode.clock_freq_mhz > 0 || sw.mode.clock_period_ns > 0) {
                    scenario.sta = run_scenario_sta(sw.corner, sw.mode);
                }

                // Power
                PowerAnalyzer pa(nl_);
                double freq = sw.mode.clock_freq_mhz > 0 ? sw.mode.clock_freq_mhz : 1.0;
                scenario.power = pa.analyze(freq, sw.corner.voltage, sw.mode.switching_activity);
                scenario.power.total_power_mw *= sw.corner.power_scale;
                scenario.power.dynamic_power_mw *= sw.corner.power_scale;

                results[i] = std::move(scenario);
            }));
        }

        for (auto& f : futures) f.get();
    }

    // Phase 3: Sequential merge of results (thread-safe accumulation)
    for (auto& s : inactive_scenarios) {
        r.details.push_back(std::move(s));
        r.scenarios++;
        r.pruned_scenarios++;
    }

    for (size_t i = 0; i < results.size(); ++i) {
        auto& scenario = results[i];
        auto& sw = active_work[i];

        if (sw.mode.clock_freq_mhz > 0 || sw.mode.clock_period_ns > 0) {
            // Track CPPR/POCV usage
            if (sw.corner.cppr_enabled && config_.enable_cppr_per_corner)
                r.cppr_scenarios++;
            if (sw.corner.ocv_mode == OcvMode::POCV && config_.enable_pocv_per_corner)
                r.pocv_scenarios++;

            // Track worst setup
            if (scenario.sta.wns < r.worst_setup_wns) {
                r.worst_setup_wns = scenario.sta.wns;
                r.worst_setup_tns = scenario.sta.tns;
                r.worst_setup_scenario = scenario.scenario_name;
            }
            // Track worst hold
            if (scenario.sta.hold_wns < r.worst_hold_wns) {
                r.worst_hold_wns = scenario.sta.hold_wns;
                r.worst_hold_tns = scenario.sta.hold_tns;
                r.worst_hold_scenario = scenario.scenario_name;
            }
            r.total_setup_violations += scenario.sta.num_violations;
            r.total_hold_violations += scenario.sta.hold_violations;

            // Update signoff summaries
            if (scenario.signoff == SignoffType::SETUP || scenario.signoff == SignoffType::ALL) {
                r.setup_signoff.scenario_count++;
                if (scenario.sta.wns < r.setup_signoff.worst_wns || r.setup_signoff.worst_scenario.empty()) {
                    r.setup_signoff.worst_wns = scenario.sta.wns;
                    r.setup_signoff.worst_tns = scenario.sta.tns;
                    r.setup_signoff.worst_scenario = scenario.scenario_name;
                }
                r.setup_signoff.total_violations += scenario.sta.num_violations;
            }
            if (scenario.signoff == SignoffType::HOLD || scenario.signoff == SignoffType::ALL) {
                r.hold_signoff.scenario_count++;
                if (scenario.sta.hold_wns < r.hold_signoff.worst_wns || r.hold_signoff.worst_scenario.empty()) {
                    r.hold_signoff.worst_wns = scenario.sta.hold_wns;
                    r.hold_signoff.worst_tns = scenario.sta.hold_tns;
                    r.hold_signoff.worst_scenario = scenario.scenario_name;
                }
                r.hold_signoff.total_violations += scenario.sta.hold_violations;
            }

            // Sensitivity analysis (optional)
            if (config_.enable_sensitivity) {
                auto sens = compute_sensitivity(sw.corner, sw.mode, scenario.sta.wns);
                r.sensitivities.push_back({scenario.scenario_name, sens});
            }
        }

        if (scenario.power.total_power_mw > r.max_power_mw) {
            r.max_power_mw = scenario.power.total_power_mw;
            r.max_power_scenario = scenario.scenario_name;
        }

        // Signoff power summaries
        if (scenario.signoff == SignoffType::LEAKAGE || scenario.signoff == SignoffType::ALL) {
            r.leakage_signoff.scenario_count++;
            if (scenario.power.static_power_mw > r.leakage_signoff.worst_wns ||
                r.leakage_signoff.worst_scenario.empty()) {
                r.leakage_signoff.worst_wns = scenario.power.static_power_mw;
                r.leakage_signoff.worst_scenario = scenario.scenario_name;
            }
        }
        if (scenario.signoff == SignoffType::DYNAMIC_POWER || scenario.signoff == SignoffType::ALL) {
            r.power_signoff.scenario_count++;
            if (scenario.power.dynamic_power_mw > r.power_signoff.worst_wns ||
                r.power_signoff.worst_scenario.empty()) {
                r.power_signoff.worst_wns = scenario.power.dynamic_power_mw;
                r.power_signoff.worst_scenario = scenario.scenario_name;
            }
        }

        r.active_scenarios++;
        r.details.push_back(std::move(scenario));
        r.scenarios++;
    }

    // Industrial: prune dominated scenarios (post-analysis, for reporting)
    if (config_.enable_scenario_pruning)
        prune_dominated_scenarios(r.details);

    // Backward compatibility
    r.worst_wns = r.worst_setup_wns;
    r.worst_tns = r.worst_setup_tns;
    r.worst_corner = r.worst_setup_scenario;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    r.message = std::to_string(r.scenarios) + " scenarios (" +
                std::to_string(r.corners) + " corners × " +
                std::to_string(r.modes) + " modes). " +
                "Worst setup WNS: " + std::to_string(r.worst_setup_wns) + "ns at " +
                r.worst_setup_scenario + ". " +
                "Worst hold WNS: " + std::to_string(r.worst_hold_wns) + "ns at " +
                r.worst_hold_scenario + ". " +
                "Max power: " + std::to_string(r.max_power_mw) + "mW at " +
                r.max_power_scenario;
    if (r.pruned_scenarios > 0)
        r.message += ". " + std::to_string(r.pruned_scenarios) + " inactive";
    if (r.cppr_scenarios > 0)
        r.message += ". " + std::to_string(r.cppr_scenarios) + " CPPR";
    if (r.pocv_scenarios > 0)
        r.message += ". " + std::to_string(r.pocv_scenarios) + " POCV";

    return r;
}

// ============================================================================
// Corner Pruning (sensitivity-based)
// ============================================================================

McmmAnalyzer::CornerPruning McmmAnalyzer::prune_corners(double sensitivity_threshold) {
    CornerPruning result;
    result.original_corners = static_cast<int>(corners_.size());

    if (corners_.empty()) {
        result.pruned_corners = 0;
        result.reason = "No corners to prune";
        return result;
    }

    // Run a quick STA for each corner with the first active mode to get timing deltas
    FunctionalMode ref_mode;
    bool found_mode = false;
    for (auto& m : modes_) {
        if (m.clock_freq_mhz > 0 || m.clock_period_ns > 0) {
            ref_mode = m;
            found_mode = true;
            break;
        }
    }
    if (!found_mode) {
        // No suitable mode; keep all corners
        for (auto& c : corners_)
            result.active_corners.push_back(c.name);
        result.pruned_corners = 0;
        result.reason = "No active mode for timing comparison";
        return result;
    }

    // Find nominal corner (TT or first typical)
    int nominal_idx = 0;
    for (int i = 0; i < static_cast<int>(corners_.size()); ++i) {
        if (corners_[i].process == PvtCorner::TYPICAL) {
            nominal_idx = i;
            break;
        }
    }

    auto nominal_sta = run_scenario_sta(corners_[nominal_idx], ref_mode);
    double nominal_wns = nominal_sta.wns;

    std::vector<bool> keep(corners_.size(), false);
    keep[nominal_idx] = true;  // always keep nominal

    for (int i = 0; i < static_cast<int>(corners_.size()); ++i) {
        if (i == nominal_idx) continue;
        auto sta = run_scenario_sta(corners_[i], ref_mode);
        double delta = std::abs(sta.wns - nominal_wns);

        // Corner is significant if its WNS differs by more than threshold
        if (delta >= sensitivity_threshold) {
            keep[i] = true;
        }
    }

    for (int i = 0; i < static_cast<int>(corners_.size()); ++i) {
        if (keep[i])
            result.active_corners.push_back(corners_[i].name);
        else
            result.pruned_away.push_back(corners_[i].name);
    }

    result.pruned_corners = static_cast<int>(result.pruned_away.size());
    result.reason = "Pruned corners with WNS delta < " +
                    std::to_string(sensitivity_threshold) + "ns vs nominal";
    return result;
}

// ============================================================================
// PVT Interpolation (trilinear)
// ============================================================================

double McmmAnalyzer::interpolate_pvt(const std::vector<PvtPoint>& table,
                                      double p, double v, double t)
{
    if (table.empty()) return 1.0;
    if (table.size() == 1) return table[0].delay_factor;

    // Find nearest points and perform inverse-distance-weighted interpolation
    // in PVT space. For sparse tables this degrades gracefully to nearest-neighbor.
    double total_weight = 0;
    double weighted_sum = 0;

    for (auto& pt : table) {
        // Normalized distances in each dimension
        double dp = (pt.process - p);
        double dv = (pt.voltage - v) * 10.0;  // scale V to comparable range
        double dt = (pt.temperature - t) / 100.0;  // scale T

        double dist_sq = dp * dp + dv * dv + dt * dt;
        if (dist_sq < 1e-12) return pt.delay_factor;  // exact match

        double w = 1.0 / dist_sq;
        total_weight += w;
        weighted_sum += w * pt.delay_factor;
    }

    return (total_weight > 0) ? weighted_sum / total_weight : 1.0;
}

// ============================================================================
// Scenario Reduction (clustering-based)
// ============================================================================

McmmAnalyzer::ScenarioReduction McmmAnalyzer::reduce_scenarios(int target_count) {
    ScenarioReduction result;
    int total = static_cast<int>(corners_.size()) * static_cast<int>(modes_.size());
    result.original_scenarios = total;

    if (total <= target_count) {
        result.reduced_scenarios = total;
        for (int i = 0; i < total; ++i)
            result.active_scenario_ids.push_back(i);
        result.accuracy_loss = 0;
        return result;
    }

    // Build scenario feature vectors: (WNS, hold_WNS, power) for clustering
    struct ScenarioFeature {
        int id;
        double wns;
        double hold_wns;
        double power;
    };
    std::vector<ScenarioFeature> features;

    FunctionalMode ref_mode;
    bool found_mode = false;
    for (auto& m : modes_) {
        if (m.clock_freq_mhz > 0 || m.clock_period_ns > 0) {
            ref_mode = m; found_mode = true; break;
        }
    }

    int sid = 0;
    for (auto& corner : corners_) {
        for (auto& mode : modes_) {
            ScenarioFeature sf;
            sf.id = sid++;
            if (found_mode && (mode.clock_freq_mhz > 0 || mode.clock_period_ns > 0)) {
                auto sta = run_scenario_sta(corner, mode);
                sf.wns = sta.wns;
                sf.hold_wns = sta.hold_wns;
            } else {
                sf.wns = 0;
                sf.hold_wns = 0;
            }
            PowerAnalyzer pa(nl_);
            double freq = mode.clock_freq_mhz > 0 ? mode.clock_freq_mhz : 1.0;
            auto pwr = pa.analyze(freq, corner.voltage, mode.switching_activity);
            sf.power = pwr.total_power_mw * corner.power_scale;
            features.push_back(sf);
        }
    }

    // Greedy clustering: iteratively merge closest pair until target_count
    std::vector<std::vector<int>> clusters;
    for (auto& f : features)
        clusters.push_back({f.id});

    auto dist = [&](int a, int b) {
        auto& fa = features[a];
        auto& fb = features[b];
        double dw = (fa.wns - fb.wns) * 10.0;
        double dh = (fa.hold_wns - fb.hold_wns) * 10.0;
        double dp = (fa.power - fb.power) / std::max(1.0, fa.power);
        return dw * dw + dh * dh + dp * dp;
    };

    while (static_cast<int>(clusters.size()) > target_count) {
        double best_dist = 1e18;
        int mi = 0, mj = 1;
        for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
            for (int j = i + 1; j < static_cast<int>(clusters.size()); ++j) {
                double d = dist(clusters[i][0], clusters[j][0]);
                if (d < best_dist) { best_dist = d; mi = i; mj = j; }
            }
        }
        // Merge cluster mj into mi
        for (auto id : clusters[mj])
            clusters[mi].push_back(id);
        clusters.erase(clusters.begin() + mj);
    }

    // Pick representative from each cluster (worst WNS = most critical)
    double max_wns_loss = 0;
    for (auto& cluster : clusters) {
        int best_id = cluster[0];
        double worst_wns = features[cluster[0]].wns;
        for (auto id : cluster) {
            if (features[id].wns < worst_wns) {
                worst_wns = features[id].wns;
                best_id = id;
            }
        }
        result.active_scenario_ids.push_back(best_id);

        // Accuracy loss: max WNS difference within cluster
        for (auto id : cluster) {
            double loss = std::abs(features[id].wns - worst_wns);
            max_wns_loss = std::max(max_wns_loss, loss);
        }
    }

    result.reduced_scenarios = static_cast<int>(result.active_scenario_ids.size());
    result.accuracy_loss = max_wns_loss;
    return result;
}

} // namespace sf
