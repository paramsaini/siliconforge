// SiliconForge — Electromigration Analysis Engine Implementation
#include "timing/electromigration.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace sf {

// ═══════════════════════════════════════════════════════════════════════
//  Black's equation — mean time to failure
// ═══════════════════════════════════════════════════════════════════════
//  MTTF = A · J^(−n) · exp(Ea / (k·T))
//  We normalise so that MTTF = target_lifetime when J == J_limit at the
//  reference temperature (105 °C / 378.15 K).
// ───────────────────────────────────────────────────────────────────────
double EmAnalyzer::compute_mttf(double j_actual, double j_limit,
                                double temp_c, double ea_ev,
                                double n) const {
    if (j_actual <= 0.0) return 1e20;   // no current → infinite life
    if (j_limit  <= 0.0) return 0.0;    // undefined limit → instant fail

    constexpr double k_boltzmann   = 8.617e-5;  // eV / K
    constexpr double T_ref_kelvin  = 378.15;     // 105 °C reference
    constexpr double mttf_base_yrs = 10.0;       // life at the limit

    const double T_kelvin = temp_c + 273.15;
    const double ratio    = j_limit / j_actual;

    // Temperature acceleration relative to reference
    const double exp_actual = std::exp(ea_ev / (k_boltzmann * T_kelvin));
    const double exp_ref    = std::exp(ea_ev / (k_boltzmann * T_ref_kelvin));

    return mttf_base_yrs * std::pow(ratio, n) * (exp_actual / exp_ref);
}

// ═══════════════════════════════════════════════════════════════════════
//  Current density  (mA / μm²)
// ═══════════════════════════════════════════════════════════════════════
double EmAnalyzer::current_density(double current_ma,
                                   double width_um,
                                   double thickness_um) const {
    const double area = width_um * thickness_um;
    if (area <= 0.0) return 0.0;
    return current_ma / area;
}

// ═══════════════════════════════════════════════════════════════════════
//  Signal-net current estimation
//  I_avg = C_total · V_swing · f · activity
// ═══════════════════════════════════════════════════════════════════════
double EmAnalyzer::estimate_signal_current(const Netlist& nl, int net_id,
                                           double vdd, double freq_ghz,
                                           double activity) const {
    if (net_id < 0 || net_id >= static_cast<int>(nl.num_nets()))
        return 0.0;

    const auto& net = nl.net(net_id);

    // Wire capacitance: rough HPWL model (0.2 fF / μm typical)
    // Approximate HPWL from fanout count × average span
    const double avg_span_um   = 50.0;  // conservative average
    const double cap_per_um_fF = 0.20;
    const double c_wire_pf = static_cast<double>(net.fanout.size())
                             * avg_span_um * cap_per_um_fF * 1e-3;  // fF → pF

    // Gate-input capacitance (~2 fF per sink)
    const double c_gate_pf = static_cast<double>(net.fanout.size()) * 0.002;

    const double c_total_pf = c_wire_pf + c_gate_pf;

    // I = C · V · f · α  (pF · V · GHz → mA)
    return c_total_pf * vdd * freq_ghz * activity;
}

// ═══════════════════════════════════════════════════════════════════════
//  Power-net current estimation  (I = P / V, distributed)
// ═══════════════════════════════════════════════════════════════════════
double EmAnalyzer::estimate_power_current(const PhysicalDesign& pd,
                                          int net_id,
                                          double total_power_mw) const {
    if (pd.nets.empty()) return total_power_mw;  // single-segment fallback

    // Distribute across power nets equally (simplified)
    int power_net_count = 0;
    for (const auto& pn : pd.nets) {
        const auto& nm = pn.name;
        if (nm.find("vdd") != std::string::npos ||
            nm.find("VDD") != std::string::npos ||
            nm.find("vss") != std::string::npos ||
            nm.find("VSS") != std::string::npos ||
            nm.find("gnd") != std::string::npos ||
            nm.find("GND") != std::string::npos) {
            ++power_net_count;
        }
    }
    if (power_net_count <= 0) power_net_count = 1;
    return total_power_mw / static_cast<double>(power_net_count);
}

// ═══════════════════════════════════════════════════════════════════════
//  Per-wire-segment EM check
// ═══════════════════════════════════════════════════════════════════════
void EmAnalyzer::check_wire_em(const std::string& net_name,
                               const WireSegment& seg,
                               double current_ma,
                               const EmLayerRule& rule,
                               const EmConfig& cfg,
                               EmResult& result) const {
    const double w = (seg.width > 0) ? seg.width : rule.width_um;
    const double t = rule.thickness_um;

    const double j = current_density(current_ma, w, t);

    // Track global density stats
    result.avg_current_density += j;
    if (j > result.max_current_density) result.max_current_density = j;

    // Location description
    std::ostringstream loc;
    loc << std::fixed << std::setprecision(2)
        << "(" << seg.start.x << "," << seg.start.y << ")->("
        << seg.end.x << "," << seg.end.y << ") w=" << w << "um";
    const std::string seg_info = loc.str();

    // ── DC density check ─────────────────────────────────────────────
    const double jdc_eff = rule.jdc_limit_ma_per_um * cfg.jdc_margin;
    if (jdc_eff > 0.0) {
        const double j_dc = j;   // average ≈ DC for worst-case
        const double ratio = j_dc / jdc_eff;
        if (ratio > 0.7) {       // report warnings above 70 %
            const double mttf = compute_mttf(
                j_dc, rule.jdc_limit_ma_per_um,
                cfg.temperature_c, cfg.activation_energy_ev,
                cfg.current_density_exponent);

            EmViolation v;
            v.net_name   = net_name;
            v.layer_name = rule.layer_name;
            v.type       = EmViolation::DC_DENSITY;
            v.current_ma = current_ma;
            v.limit_ma   = jdc_eff * w * t;
            v.ratio      = ratio;
            v.wire_width_um       = w;
            v.estimated_mttf_years = mttf;
            v.segment_info        = seg_info;

            if (ratio > 1.0)
                v.severity = EmViolation::ERROR;
            else if (ratio > 0.85)
                v.severity = EmViolation::WARNING;
            else
                v.severity = EmViolation::INFO;

            result.violations.push_back(v);

            if (v.severity == EmViolation::ERROR)
                result.pass = false;
            if (mttf < result.worst_mttf_years) {
                result.worst_mttf_years = mttf;
                result.worst_net        = net_name;
            }
        }
    }

    // ── RMS density check ────────────────────────────────────────────
    const double jrms_eff = rule.jrms_limit_ma_per_um * cfg.jrms_margin;
    if (jrms_eff > 0.0) {
        // RMS current ≈ I_avg / sqrt(duty)  (duty ~ activity)
        const double duty  = std::max(cfg.activity_factor, 0.01);
        const double j_rms = j / std::sqrt(duty);
        const double ratio = j_rms / jrms_eff;
        if (ratio > 1.0) {
            const double mttf = compute_mttf(
                j_rms, rule.jrms_limit_ma_per_um,
                cfg.temperature_c, cfg.activation_energy_ev,
                cfg.current_density_exponent);

            EmViolation v;
            v.net_name   = net_name;
            v.layer_name = rule.layer_name;
            v.type       = EmViolation::RMS_DENSITY;
            v.current_ma = current_ma;
            v.limit_ma   = jrms_eff * w * t;
            v.ratio      = ratio;
            v.wire_width_um       = w;
            v.estimated_mttf_years = mttf;
            v.segment_info        = seg_info;
            v.severity = (ratio > 1.5) ? EmViolation::ERROR
                                       : EmViolation::WARNING;

            result.violations.push_back(v);
            if (v.severity == EmViolation::ERROR)
                result.pass = false;
            if (mttf < result.worst_mttf_years) {
                result.worst_mttf_years = mttf;
                result.worst_net        = net_name;
            }
        }
    }

    // ── Peak density check ───────────────────────────────────────────
    if (rule.jpeak_limit_ma_per_um > 0.0) {
        // Peak ≈ I_avg / activity  (burst during single transition)
        const double act    = std::max(cfg.activity_factor, 0.001);
        const double j_peak = j / act;
        const double ratio  = j_peak / rule.jpeak_limit_ma_per_um;
        if (ratio > 1.0) {
            EmViolation v;
            v.net_name   = net_name;
            v.layer_name = rule.layer_name;
            v.type       = EmViolation::PEAK_DENSITY;
            v.current_ma = current_ma;
            v.limit_ma   = rule.jpeak_limit_ma_per_um * w * t;
            v.ratio      = ratio;
            v.wire_width_um       = w;
            v.estimated_mttf_years = compute_mttf(
                j_peak, rule.jpeak_limit_ma_per_um,
                cfg.temperature_c, cfg.activation_energy_ev,
                cfg.current_density_exponent);
            v.segment_info = seg_info;
            v.severity     = EmViolation::WARNING;  // peak is transient
            result.violations.push_back(v);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Per-via EM check
// ═══════════════════════════════════════════════════════════════════════
void EmAnalyzer::check_via_em(const std::string& net_name,
                              const Via& via,
                              double current_ma,
                              const EmLayerRule& rule,
                              const EmConfig& cfg,
                              EmResult& result) const {
    if (rule.via_current_limit_ma <= 0.0) return;

    const double ratio = current_ma / rule.via_current_limit_ma;
    if (ratio <= 0.7) return;  // well within margin

    const double mttf = compute_mttf(
        current_ma, rule.via_current_limit_ma,
        cfg.temperature_c, cfg.activation_energy_ev,
        cfg.current_density_exponent);

    std::ostringstream loc;
    loc << std::fixed << std::setprecision(2)
        << "via @ (" << via.position.x << "," << via.position.y
        << ") layers " << via.lower_layer << "->" << via.upper_layer;

    EmViolation v;
    v.net_name   = net_name;
    v.layer_name = rule.layer_name;
    v.type       = EmViolation::VIA_CURRENT;
    v.current_ma = current_ma;
    v.limit_ma   = rule.via_current_limit_ma;
    v.ratio      = ratio;
    v.wire_width_um       = 0;
    v.estimated_mttf_years = mttf;
    v.segment_info        = loc.str();

    if (ratio > 1.0)
        v.severity = EmViolation::ERROR;
    else if (ratio > 0.85)
        v.severity = EmViolation::WARNING;
    else
        v.severity = EmViolation::INFO;

    result.violations.push_back(v);
    ++result.via_violations;

    if (v.severity == EmViolation::ERROR)
        result.pass = false;
    if (mttf < result.worst_mttf_years) {
        result.worst_mttf_years = mttf;
        result.worst_net        = net_name;
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Helper: classify a net as signal / power / clock
// ═══════════════════════════════════════════════════════════════════════
namespace {

enum class NetKind { SIGNAL, POWER, CLOCK };

NetKind classify_net(const std::string& name, const Netlist& nl, int net_id) {
    // Power / ground patterns
    auto has = [&](const char* pat) {
        return name.find(pat) != std::string::npos;
    };
    if (has("vdd") || has("VDD") || has("vss") || has("VSS") ||
        has("gnd") || has("GND") || has("pwr") || has("PWR"))
        return NetKind::POWER;

    // Clock patterns
    if (has("clk") || has("CLK") || has("clock") || has("CLOCK"))
        return NetKind::CLOCK;

    // Check if the net drives any DFF clock pin
    if (net_id >= 0 && net_id < static_cast<int>(nl.num_nets())) {
        const auto& net = nl.net(net_id);
        for (auto gid : net.fanout) {
            if (gid >= 0 && gid < static_cast<int>(nl.num_gates())) {
                const auto& g = nl.gate(gid);
                if (g.type == GateType::DFF && g.clk == net_id)
                    return NetKind::CLOCK;
            }
        }
    }
    return NetKind::SIGNAL;
}

// Look up layer rule by name (case-insensitive prefix match)
const EmLayerRule* find_layer_rule(const std::vector<EmLayerRule>& rules,
                                   const std::string& layer_name) {
    for (const auto& r : rules) {
        if (r.layer_name == layer_name)
            return &r;
    }
    // Prefix / case-insensitive fallback
    for (const auto& r : rules) {
        if (layer_name.size() >= r.layer_name.size()) {
            bool match = true;
            for (size_t i = 0; i < r.layer_name.size(); ++i) {
                if (std::tolower(static_cast<unsigned char>(layer_name[i])) !=
                    std::tolower(static_cast<unsigned char>(r.layer_name[i]))) {
                    match = false;
                    break;
                }
            }
            if (match) return &r;
        }
    }
    return nullptr;
}

// Build net-name → netlist-id map
std::unordered_map<std::string, int>
build_net_lookup(const Netlist& nl) {
    std::unordered_map<std::string, int> m;
    for (size_t i = 0; i < nl.num_nets(); ++i) {
        const auto& n = nl.net(static_cast<int>(i));
        if (!n.name.empty()) m[n.name] = n.id;
    }
    return m;
}

} // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════
//  Main analysis entry point
// ═══════════════════════════════════════════════════════════════════════
EmResult EmAnalyzer::analyze(const Netlist& nl,
                             const PhysicalDesign& pd,
                             const EmConfig& cfg) const {
    EmResult result;

    // Empty-design guard
    if (nl.num_nets() == 0 && pd.nets.empty()) {
        result.summary = "No nets to check.";
        result.report  = result.summary;
        return result;
    }

    // Ensure we have layer rules
    const auto& rules = cfg.layer_rules.empty()
                        ? default_rules_sky130()
                        : cfg.layer_rules;

    // Build name → layer-rule lookup
    std::unordered_map<int, const EmLayerRule*> layer_id_to_rule;
    for (const auto& rl : pd.layers) {
        const EmLayerRule* r = find_layer_rule(rules, rl.name);
        if (r) layer_id_to_rule[rl.id] = r;
    }

    // Build name → netlist-id map
    const auto net_lookup = build_net_lookup(nl);

    // Estimate total power for power-net current distribution
    // P ≈ C_total · V² · f · α  (very rough)
    const double est_total_power_mw =
        static_cast<double>(nl.num_gates()) * 0.005 *   // ~5 μW / gate
        cfg.supply_voltage * cfg.supply_voltage *
        cfg.clock_freq_ghz;

    int wire_count = 0;   // for averaging density

    // ── Iterate physical nets ────────────────────────────────────────
    for (const auto& pnet : pd.nets) {
        const std::string& name = pnet.name;

        // Map to netlist id (if available)
        int nl_net_id = -1;
        auto it = net_lookup.find(name);
        if (it != net_lookup.end()) nl_net_id = it->second;

        // Classify
        NetKind kind = classify_net(name, nl, nl_net_id);

        // Skip unwanted categories
        if (kind == NetKind::SIGNAL && !cfg.check_signal_em) continue;
        if (kind == NetKind::POWER  && !cfg.check_power_em)  continue;
        if (kind == NetKind::CLOCK  && !cfg.check_clock_em)  continue;

        ++result.total_nets_checked;

        // ── Estimate current ────────────────────────────────────────
        double current_ma = 0.0;
        switch (kind) {
            case NetKind::SIGNAL:
                current_ma = estimate_signal_current(
                    nl, nl_net_id,
                    cfg.supply_voltage, cfg.clock_freq_ghz,
                    cfg.activity_factor);
                break;
            case NetKind::POWER:
                current_ma = estimate_power_current(
                    pd, pnet.id, est_total_power_mw);
                break;
            case NetKind::CLOCK:
                // Clock toggles every cycle → activity = 1.0
                current_ma = estimate_signal_current(
                    nl, nl_net_id,
                    cfg.supply_voltage, cfg.clock_freq_ghz,
                    1.0);
                break;
        }

        // ── Check wire segments belonging to this net ───────────────
        for (const auto& seg : pd.wires) {
            if (seg.net_id != pnet.id) continue;

            // Resolve layer rule
            auto lr = layer_id_to_rule.find(seg.layer);
            if (lr == layer_id_to_rule.end()) {
                // Try matching by index into rules vector
                if (seg.layer >= 0 &&
                    seg.layer < static_cast<int>(rules.size()) &&
                    !rules[seg.layer].is_via) {
                    check_wire_em(name, seg, current_ma,
                                  rules[seg.layer], cfg, result);
                    ++wire_count;
                }
                continue;
            }
            if (lr->second->is_via) continue;  // skip via rules for wires

            check_wire_em(name, seg, current_ma, *lr->second, cfg, result);
            ++wire_count;
        }

        // ── Check vias belonging to this net ────────────────────────
        // Via ownership: a via is associated with the net whose wire
        // segments touch the via location on either adjacent layer.
        for (const auto& via : pd.vias) {
            // Check if any wire of this net touches this via
            bool belongs = false;
            for (const auto& seg : pd.wires) {
                if (seg.net_id != pnet.id) continue;
                if (seg.layer != via.lower_layer &&
                    seg.layer != via.upper_layer) continue;
                // Simple proximity test
                const double dx = std::min(
                    std::abs(via.position.x - seg.start.x),
                    std::abs(via.position.x - seg.end.x));
                const double dy = std::min(
                    std::abs(via.position.y - seg.start.y),
                    std::abs(via.position.y - seg.end.y));
                if (dx < 1.0 && dy < 1.0) { belongs = true; break; }
            }
            if (!belongs) continue;

            // Find via rule
            for (const auto& rule : rules) {
                if (!rule.is_via) continue;
                // Accept if the rule name looks like it connects these layers
                bool name_match =
                    rule.layer_name.find("via") != std::string::npos ||
                    rule.layer_name.find("Via") != std::string::npos ||
                    rule.layer_name.find("VIA") != std::string::npos ||
                    rule.layer_name.find("mcon") != std::string::npos;
                if (!name_match) continue;

                check_via_em(name, via, current_ma, rule, cfg, result);
                break;  // one via rule per via
            }
        }

        // Tally per-kind violations added in this iteration
        const int prev_violations = result.signal_violations +
                                    result.power_violations +
                                    result.clock_violations +
                                    result.via_violations;
        (void)prev_violations;  // counted below after sorting
    }

    // ── Also check netlist-only nets without physical representation ─
    for (size_t i = 0; i < nl.num_nets(); ++i) {
        const auto& net = nl.net(static_cast<int>(i));
        if (net.name.empty()) continue;

        // Skip if already covered via physical net
        bool covered = false;
        for (const auto& pn : pd.nets) {
            if (pn.name == net.name) { covered = true; break; }
        }
        if (covered) continue;

        NetKind kind = classify_net(net.name, nl, net.id);
        if (kind != NetKind::SIGNAL) continue;
        if (!cfg.check_signal_em)    continue;

        ++result.total_nets_checked;

        // Use first non-via rule as fallback
        const EmLayerRule* fallback = nullptr;
        for (const auto& r : rules) {
            if (!r.is_via) { fallback = &r; break; }
        }
        if (!fallback) continue;

        const double current_ma = estimate_signal_current(
            nl, net.id,
            cfg.supply_voltage, cfg.clock_freq_ghz,
            cfg.activity_factor);

        const double j = current_density(current_ma,
                                         fallback->width_um,
                                         fallback->thickness_um);
        const double jdc_eff = fallback->jdc_limit_ma_per_um * cfg.jdc_margin;
        if (jdc_eff > 0.0 && j / jdc_eff > 0.7) {
            const double mttf = compute_mttf(
                j, fallback->jdc_limit_ma_per_um,
                cfg.temperature_c, cfg.activation_energy_ev,
                cfg.current_density_exponent);

            EmViolation v;
            v.net_name   = net.name;
            v.layer_name = fallback->layer_name;
            v.type       = EmViolation::DC_DENSITY;
            v.current_ma = current_ma;
            v.limit_ma   = jdc_eff * fallback->width_um * fallback->thickness_um;
            v.ratio      = j / jdc_eff;
            v.wire_width_um       = fallback->width_um;
            v.estimated_mttf_years = mttf;
            v.segment_info        = "(estimated — no physical routing)";
            v.severity = (v.ratio > 1.0) ? EmViolation::ERROR
                                         : EmViolation::WARNING;

            result.violations.push_back(v);
            if (v.severity == EmViolation::ERROR)
                result.pass = false;
            if (mttf < result.worst_mttf_years) {
                result.worst_mttf_years = mttf;
                result.worst_net        = net.name;
            }
        }
        result.avg_current_density += j;
        ++wire_count;
    }

    // ── Finalise statistics ──────────────────────────────────────────
    if (wire_count > 0)
        result.avg_current_density /= static_cast<double>(wire_count);

    // Categorise violations
    for (const auto& v : result.violations) {
        if (v.severity == EmViolation::INFO) continue;
        NetKind kind = classify_net(v.net_name, nl, -1);
        switch (kind) {
            case NetKind::SIGNAL: ++result.signal_violations; break;
            case NetKind::POWER:  ++result.power_violations;  break;
            case NetKind::CLOCK:  ++result.clock_violations;  break;
        }
    }

    // MTTF-based lifetime check
    if (result.worst_mttf_years < cfg.target_lifetime_years)
        result.pass = false;

    // Sort violations: ERROR first, then WARNING, then INFO
    std::sort(result.violations.begin(), result.violations.end(),
              [](const EmViolation& a, const EmViolation& b) {
                  if (a.severity != b.severity)
                      return a.severity < b.severity;  // ERROR=0 first
                  return a.ratio > b.ratio;            // worst ratio first
              });

    generate_report(result, cfg);
    return result;
}

// ═══════════════════════════════════════════════════════════════════════
//  Wire-sizing / via-doubling recommendations
// ═══════════════════════════════════════════════════════════════════════
std::vector<EmWireSizing>
EmAnalyzer::suggest_fixes(const EmResult& result,
                          const EmConfig& cfg) const {
    std::vector<EmWireSizing> fixes;

    for (const auto& v : result.violations) {
        if (v.severity == EmViolation::INFO) continue;

        EmWireSizing fix;
        fix.net_name   = v.net_name;
        fix.layer_name = v.layer_name;

        if (v.type == EmViolation::VIA_CURRENT) {
            // Via doubling
            fix.current_vias     = 1;
            fix.recommended_vias = static_cast<int>(
                std::ceil(v.current_ma / (v.limit_ma * cfg.jdc_margin)));
            if (fix.recommended_vias < 2)
                fix.recommended_vias = 2;
            fix.area_overhead_pct =
                (static_cast<double>(fix.recommended_vias) - 1.0) * 100.0;
        } else {
            // Wire widening
            fix.current_width_um = v.wire_width_um;
            // Need J_new ≤ J_limit * margin
            // J_new = I / (W_new * T)   →   W_new = I / (J_limit * margin * T)
            // Simpler: W_new = W_old * ratio / margin
            const double margin = cfg.jdc_margin;
            fix.recommended_width_um =
                fix.current_width_um * v.ratio / margin;
            // Round up to manufacturing grid (typically 0.005 μm)
            fix.recommended_width_um =
                std::ceil(fix.recommended_width_um / 0.005) * 0.005;
            if (fix.recommended_width_um <= fix.current_width_um)
                fix.recommended_width_um = fix.current_width_um * 1.5;

            fix.area_overhead_pct =
                (fix.recommended_width_um / fix.current_width_um - 1.0) * 100.0;
        }

        // Deduplicate: skip if we already have a fix for same net + layer
        bool dup = false;
        for (const auto& f : fixes) {
            if (f.net_name == fix.net_name && f.layer_name == fix.layer_name) {
                dup = true;
                break;
            }
        }
        if (!dup) fixes.push_back(fix);
    }

    return fixes;
}

// ═══════════════════════════════════════════════════════════════════════
//  Report generation
// ═══════════════════════════════════════════════════════════════════════
void EmAnalyzer::generate_report(EmResult& result,
                                 const EmConfig& cfg) const {
    std::ostringstream rpt;

    // Header
    rpt << "=======================================\n"
        << " SiliconForge Electromigration Report\n"
        << "=======================================\n";

    // Config summary
    rpt << std::fixed << std::setprecision(1)
        << "Config: T=" << cfg.temperature_c << "C"
        << ", Ea=" << cfg.activation_energy_ev << "eV"
        << ", n=" << cfg.current_density_exponent
        << ", target=" << cfg.target_lifetime_years << "yr\n";

    rpt << "Checked: " << result.total_nets_checked << " nets\n\n";

    // Violation tally
    int errors = 0, warnings = 0, infos = 0;
    for (const auto& v : result.violations) {
        switch (v.severity) {
            case EmViolation::ERROR:   ++errors;   break;
            case EmViolation::WARNING: ++warnings; break;
            case EmViolation::INFO:    ++infos;    break;
        }
    }

    if (result.violations.empty()) {
        rpt << "No EM violations found.\n";
    } else {
        rpt << "VIOLATIONS: " << errors << " errors, "
            << warnings << " warnings, " << infos << " info\n";

        // Print each violation
        for (const auto& v : result.violations) {
            const char* sev_str = (v.severity == EmViolation::ERROR) ? "ERROR"
                                : (v.severity == EmViolation::WARNING) ? "WARN "
                                : "INFO ";
            rpt << "  [" << sev_str << "] net \"" << v.net_name
                << "\" on " << v.layer_name << ": ";

            switch (v.type) {
                case EmViolation::DC_DENSITY:
                    rpt << "Jdc";   break;
                case EmViolation::RMS_DENSITY:
                    rpt << "Jrms";  break;
                case EmViolation::PEAK_DENSITY:
                    rpt << "Jpeak"; break;
                case EmViolation::VIA_CURRENT:
                    rpt << "Ivia";  break;
                case EmViolation::LIFETIME:
                    rpt << "MTTF";  break;
            }

            rpt << std::fixed << std::setprecision(2)
                << "=" << v.current_ma << "mA"
                << " (limit=" << v.limit_ma << "mA)"
                << " ratio=" << v.ratio
                << " MTTF=" << v.estimated_mttf_years << "yr\n";

            if (!v.segment_info.empty())
                rpt << "         " << v.segment_info << "\n";
        }
    }

    rpt << "\n";

    // Worst MTTF
    if (result.worst_mttf_years < 1e9) {
        rpt << std::fixed << std::setprecision(1)
            << "Worst MTTF: " << result.worst_mttf_years
            << " years (net \"" << result.worst_net << "\")\n";
    }
    rpt << "Target: " << cfg.target_lifetime_years << " years"
        << " -- " << (result.pass ? "PASS" : "FAIL") << "\n";

    // Recommendations
    auto fixes = suggest_fixes(result, cfg);
    if (!fixes.empty()) {
        rpt << "\nRecommendations:\n";
        for (const auto& f : fixes) {
            if (f.recommended_vias > 0 && f.current_vias > 0) {
                rpt << "  - Add " << (f.recommended_vias - f.current_vias)
                    << " extra vias on \"" << f.net_name
                    << "\" " << f.layer_name << "\n";
            } else {
                rpt << std::fixed << std::setprecision(2)
                    << "  - Widen \"" << f.net_name
                    << "\" " << f.layer_name
                    << " from " << f.current_width_um
                    << "um to " << f.recommended_width_um
                    << "um (+";
                rpt << std::fixed << std::setprecision(0)
                    << f.area_overhead_pct << "% area)\n";
            }
        }
    }

    // Density statistics
    rpt << "\nDensity stats: avg=" << std::fixed << std::setprecision(3)
        << result.avg_current_density << " mA/um^2"
        << ", max=" << result.max_current_density << " mA/um^2\n";

    result.report = rpt.str();

    // One-line summary
    std::ostringstream sum;
    sum << "EM " << (result.pass ? "PASS" : "FAIL")
        << ": " << result.total_nets_checked << " nets, "
        << errors << " errors, " << warnings << " warnings";
    if (result.worst_mttf_years < 1e9)
        sum << std::fixed << std::setprecision(1)
            << ", worst MTTF=" << result.worst_mttf_years << "yr";
    result.summary = sum.str();
}

// ═══════════════════════════════════════════════════════════════════════
//  SKY130 default EM rules
// ═══════════════════════════════════════════════════════════════════════
std::vector<EmLayerRule> EmAnalyzer::default_rules_sky130() {
    return {
        // name    Jdc   Jrms  Jpeak  width thick  ρ      via?  Ivia
        {"li1",    1.5,  7.5,  15.0,  0.17, 0.10, 0.028, false, 0.0},
        {"met1",   1.0,  5.0,  10.0,  0.14, 0.13, 0.028, false, 0.0},
        {"met2",   1.2,  6.0,  12.0,  0.14, 0.13, 0.028, false, 0.0},
        {"met3",   1.5,  7.5,  15.0,  0.30, 0.25, 0.028, false, 0.0},
        {"met4",   2.0, 10.0,  20.0,  0.30, 0.80, 0.028, false, 0.0},
        {"met5",   3.0, 15.0,  30.0,  1.60, 1.26, 0.028, false, 0.0},
        {"mcon",   0.0,  0.0,   0.0,  0.0,  0.0,  0.0,   true,  0.29},
        {"via",    0.0,  0.0,   0.0,  0.0,  0.0,  0.0,   true,  0.37},
        {"via2",   0.0,  0.0,   0.0,  0.0,  0.0,  0.0,   true,  0.37},
        {"via3",   0.0,  0.0,   0.0,  0.0,  0.0,  0.0,   true,  0.50},
        {"via4",   0.0,  0.0,   0.0,  0.0,  0.0,  0.0,   true,  0.65},
    };
}

// ═══════════════════════════════════════════════════════════════════════
//  7 nm advanced-node EM rules (tighter limits)
// ═══════════════════════════════════════════════════════════════════════
std::vector<EmLayerRule> EmAnalyzer::default_rules_7nm() {
    return {
        // name  Jdc  Jrms  Jpeak  width  thick  ρ      via?  Ivia
        {"M1",   0.5, 2.5,  5.0,   0.028, 0.036, 0.021, false, 0.0},
        {"M2",   0.5, 2.5,  5.0,   0.028, 0.036, 0.021, false, 0.0},
        {"M3",   0.8, 4.0,  8.0,   0.040, 0.036, 0.021, false, 0.0},
        {"M4",   0.8, 4.0,  8.0,   0.040, 0.036, 0.021, false, 0.0},
        {"M5",   1.2, 6.0, 12.0,   0.080, 0.050, 0.021, false, 0.0},
        {"V0",   0.0, 0.0,  0.0,   0.0,   0.0,   0.0,   true,  0.02},
        {"V1",   0.0, 0.0,  0.0,   0.0,   0.0,   0.0,   true,  0.03},
        {"V2",   0.0, 0.0,  0.0,   0.0,   0.0,   0.0,   true,  0.04},
    };
}

// ═══════════════════════════════════════════════════════════════════════
//  Enhanced: Current Density Extraction
// ═══════════════════════════════════════════════════════════════════════

std::vector<CurrentDensity> EmAnalyzer::extract_current_density() const {
    std::vector<CurrentDensity> result;
    if (!pd_ || !nl_) return result;

    auto rules = stored_cfg_.layer_rules.empty() ?
                 default_rules_sky130() : stored_cfg_.layer_rules;

    for (int w = 0; w < (int)pd_->wires.size(); ++w) {
        auto& wire = pd_->wires[w];
        int layer = wire.layer;
        if (layer < 0 || layer >= (int)rules.size()) continue;
        if (rules[layer].is_via) continue;

        auto& rule = rules[std::min(layer, (int)rules.size() - 1)];
        double width = rule.width_um;
        double thickness = rule.thickness_um;

        // I = VDD * freq * activity * Cload  (capacitive switching model)
        double dx = wire.end.x - wire.start.x;
        double dy = wire.end.y - wire.start.y;
        double length = std::hypot(dx, dy);
        if (length <= 0) continue;

        // Cload: wire cap ~0.2 fF/μm + gate load estimate
        constexpr double cap_per_um_fF = 0.2;
        double c_load_pf = length * cap_per_um_fF * 1e-3;  // fF → pF
        int fanout = (wire.net_id >= 0 &&
                      wire.net_id < static_cast<int>(nl_->num_nets()))
                     ? static_cast<int>(nl_->net(wire.net_id).fanout.size()) : 1;
        c_load_pf += fanout * 0.002;  // ~2 fF per gate input

        // I = C · V · f · α   (pF · V · GHz → mA)
        double avg_current = c_load_pf * stored_cfg_.supply_voltage
                             * stored_cfg_.clock_freq_ghz
                             * stored_cfg_.activity_factor;
        double peak_current = avg_current * 3.0;
        double j = current_density(avg_current, width, thickness);
        double j_limit = rule.jdc_limit_ma_per_um * stored_cfg_.jdc_margin;

        CurrentDensity cd;
        cd.wire_idx = w;
        cd.layer = layer;
        cd.avg_current_ma = avg_current;
        cd.peak_current_ma = peak_current;
        cd.current_density_ma_per_um2 = j;
        cd.limit_ma_per_um2 = j_limit;
        cd.exceeds_limit = j > j_limit;
        result.push_back(cd);
    }
    return result;
}

// ═══════════════════════════════════════════════════════════════════════
//  Enhanced: Blech Effect Check
// ═══════════════════════════════════════════════════════════════════════

std::vector<BlechResult> EmAnalyzer::check_blech_effect() const {
    std::vector<BlechResult> result;
    if (!pd_) return result;

    auto rules = stored_cfg_.layer_rules.empty() ?
                 default_rules_sky130() : stored_cfg_.layer_rules;

    // Critical jL product for Cu: 3000 A/cm = 300 mA/μm in (mA/μm²)·μm units.
    // Blech length = critical_jL / J.  Wire is EM-immune when length < blech_length
    // because the back-stress gradient counters atom migration.
    constexpr double critical_jL = 300.0;  // mA/μm  (≡ 3000 A/cm)

    for (int w = 0; w < (int)pd_->wires.size(); ++w) {
        auto& wire = pd_->wires[w];
        int layer = wire.layer;
        if (layer < 0 || layer >= (int)rules.size()) continue;
        if (rules[layer].is_via) continue;

        auto& rule = rules[std::min(layer, (int)rules.size() - 1)];
        double width = rule.width_um;
        double thickness = rule.thickness_um;

        double dx = wire.end.x - wire.start.x;
        double dy = wire.end.y - wire.start.y;
        double length = std::hypot(dx, dy);
        if (length <= 0) continue;

        // Capacitive current model (consistent with extract_current_density)
        constexpr double cap_per_um_fF = 0.2;
        double c_load_pf = length * cap_per_um_fF * 1e-3;
        int fanout = (wire.net_id >= 0 &&
                      wire.net_id < static_cast<int>(nl_->num_nets()))
                     ? static_cast<int>(nl_->net(wire.net_id).fanout.size()) : 1;
        c_load_pf += fanout * 0.002;
        double avg_current = c_load_pf * stored_cfg_.supply_voltage
                             * stored_cfg_.clock_freq_ghz
                             * stored_cfg_.activity_factor;
        double j = current_density(avg_current, width, thickness);

        double blech_length = (j > 0) ? critical_jL / j : 1e6;

        BlechResult br;
        br.wire_idx = w;
        br.wire_length_um = length;
        br.blech_length_um = blech_length;
        br.is_immune = length < blech_length;
        br.current_density_ma_per_um2 = j;
        result.push_back(br);
    }
    return result;
}

// ═══════════════════════════════════════════════════════════════════════
//  Enhanced: EM Fix Suggestions
// ═══════════════════════════════════════════════════════════════════════

std::vector<EmFix> EmAnalyzer::suggest_em_fixes() const {
    std::vector<EmFix> result;
    auto densities = extract_current_density();

    auto rules = stored_cfg_.layer_rules.empty() ?
                 default_rules_sky130() : stored_cfg_.layer_rules;

    for (auto& cd : densities) {
        if (!cd.exceeds_limit) continue;

        auto& rule = rules[std::min(cd.layer, (int)rules.size() - 1)];
        double ratio = (cd.limit_ma_per_um2 > 0)
                       ? cd.current_density_ma_per_um2 / cd.limit_ma_per_um2
                       : 1e6;

        EmFix fix;
        fix.wire_idx = cd.wire_idx;
        fix.current_width = rule.width_um;

        if (ratio < 2.0) {
            // Moderate violation — widen wire so J drops to the limit
            fix.type = EmFix::WIDEN_WIRE;
            fix.suggested_width = rule.width_um * ratio;
            fix.improvement_pct = (1.0 - 1.0 / ratio) * 100.0;
        } else {
            // Severe violation — widening alone is impractical; split the net
            fix.type = EmFix::SPLIT_NET;
            fix.suggested_width = rule.width_um;
            fix.improvement_pct = (1.0 - 1.0 / ratio) * 100.0;
        }
        result.push_back(fix);

        // Additionally recommend redundant via for current distribution
        if (ratio > 1.0) {
            EmFix via_fix;
            via_fix.wire_idx = cd.wire_idx;
            via_fix.type = EmFix::ADD_VIA;
            via_fix.current_width = rule.width_um;
            via_fix.suggested_width = rule.width_um;
            via_fix.improvement_pct = 50.0;
            result.push_back(via_fix);
        }
    }
    return result;
}

// ═══════════════════════════════════════════════════════════════════════
//  Enhanced: Redundant Via Addition
// ═══════════════════════════════════════════════════════════════════════

ViaEmResult EmAnalyzer::add_redundant_vias() const {
    ViaEmResult result{};
    if (!pd_) return result;

    // Every via in the design is initially a single-cut via and therefore
    // a potential EM reliability concern.  Flag each as at-risk and plan
    // one redundant via per location.
    for (int v = 0; v < (int)pd_->vias.size(); ++v) {
        result.total_vias++;
        result.single_vias_at_risk++;
        result.redundant_vias_added++;   // add 1 redundant via per location
    }

    // Reliability improvement for N vias per location vs. 1:
    //   improvement = (1 − 1/N²) × 100 %
    // After doubling each via, N = 2 → 75 % improvement.
    int N = (result.redundant_vias_added > 0) ? 2 : 1;
    result.reliability_improvement_pct = (1.0 - 1.0 / ((double)N * N)) * 100.0;
    return result;
}

// ═══════════════════════════════════════════════════════════════════════
//  Enhanced: Full Enhanced EM Run
// ═══════════════════════════════════════════════════════════════════════

EmResult EmAnalyzer::run_enhanced() const {
    if (!nl_ || !pd_) return {};

    EmResult r = analyze(*nl_, *pd_, stored_cfg_);

    auto densities = extract_current_density();
    auto blech = check_blech_effect();
    auto fixes = suggest_em_fixes();
    auto vias = add_redundant_vias();

    int exceeding = 0;
    for (auto& cd : densities)
        if (cd.exceeds_limit) exceeding++;

    int immune = 0;
    for (auto& b : blech)
        if (b.is_immune) immune++;

    r.summary += " | Enhanced: " +
        std::to_string(exceeding) + " density violations, " +
        std::to_string(immune) + "/" + std::to_string(blech.size()) + " Blech-immune, " +
        std::to_string(fixes.size()) + " fixes suggested, " +
        std::to_string(vias.redundant_vias_added) + " redundant vias";
    return r;
}

} // namespace sf
