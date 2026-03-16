// SiliconForge — CCS Delay Engine implementation

#include "timing/ccs_delay.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace sf {

// ============================================================================
// CCS current waveform integration
// ============================================================================

double CcsDelayEngine::integrate_current(const CcsTable& ccs,
                                           double input_slew,
                                           double load_ff,
                                           double vdd,
                                           double threshold_pct) const {
    if (!ccs.valid()) return 0;

    // CcsTable layout:
    //   index_1 = input slew breakpoints
    //   index_2 = output load breakpoints
    //   index_3 = time sample breakpoints
    //   values[slew_idx][load_idx] = vector of current samples at index_3 times

    int ns = (int)ccs.index_1.size();
    int nl = (int)ccs.index_2.size();
    int nt = (int)ccs.index_3.size();
    if (ns == 0 || nl == 0 || nt == 0) return 0;

    // Find bracketing slew index
    int si = 0;
    for (int i = 0; i < ns - 1; ++i)
        if (ccs.index_1[i] <= input_slew) si = i;
    int si1 = std::min(si + 1, ns - 1);
    double sf = (si == si1) ? 0 :
        (input_slew - ccs.index_1[si]) /
        (ccs.index_1[si1] - ccs.index_1[si]);
    sf = std::clamp(sf, 0.0, 1.0);

    // Find bracketing load index
    int li = 0;
    for (int i = 0; i < nl - 1; ++i)
        if (ccs.index_2[i] <= load_ff) li = i;
    int li1 = std::min(li + 1, nl - 1);
    double lf = (li == li1) ? 0 :
        (load_ff - ccs.index_2[li]) /
        (ccs.index_2[li1] - ccs.index_2[li]);
    lf = std::clamp(lf, 0.0, 1.0);

    // Bilinear interpolation of current waveform
    std::vector<double> waveform(nt, 0.0);
    for (int t = 0; t < nt; ++t) {
        auto get_val = [&](int s, int l, int ti) -> double {
            if (s < (int)ccs.values.size() &&
                l < (int)ccs.values[s].size() &&
                ti < (int)ccs.values[s][l].size())
                return ccs.values[s][l][ti];
            return 0;
        };
        double c00 = get_val(si,  li,  t);
        double c01 = get_val(si,  li1, t);
        double c10 = get_val(si1, li,  t);
        double c11 = get_val(si1, li1, t);

        waveform[t] = (1.0 - sf) * ((1.0 - lf) * c00 + lf * c01)
                    +        sf  * ((1.0 - lf) * c10 + lf * c11);
    }

    // Integrate current: V(t) = (1/C) * integral(I(t) dt)
    double threshold_v = threshold_pct * vdd;
    double dt = (nt > 1)
        ? (ccs.index_3.back() - ccs.index_3[0]) / (nt - 1)
        : 0.01;

    double voltage = 0;
    double charge = 0;
    double cap_ff = std::max(load_ff, 0.001);

    for (int i = 0; i < nt; ++i) {
        charge += waveform[i] * dt;
        voltage = charge / cap_ff;

        if (voltage >= threshold_v) {
            double t_prev = (i > 0) ? ccs.index_3[i - 1] : 0;
            double t_curr = ccs.index_3[i];
            double v_prev = (charge - waveform[i] * dt) / cap_ff;
            double frac = (threshold_v - v_prev) / std::max(voltage - v_prev, 1e-15);
            return t_prev + frac * (t_curr - t_prev);
        }
    }

    // If threshold not reached, extrapolate
    if (voltage > 0)
        return ccs.index_3.back() * threshold_v / voltage;
    return ccs.index_3.back();
}

double CcsDelayEngine::compute_output_slew(const CcsTable& ccs,
                                             double input_slew,
                                             double load_ff,
                                             double vdd) const {
    double t20 = integrate_current(ccs, input_slew, load_ff, vdd, 0.2);
    double t80 = integrate_current(ccs, input_slew, load_ff, vdd, 0.8);
    return std::max(0.001, t80 - t20);
}

// ============================================================================
// Public CCS delay API
// ============================================================================

CcsDelayResult CcsDelayEngine::compute_delay(const LibertyTiming& timing,
                                               double input_slew_ns,
                                               double output_load_ff,
                                               double vdd,
                                               bool rise) const {
    CcsDelayResult result;
    result.input_slew_ns = input_slew_ns;
    result.output_load_ff = output_load_ff;
    result.vdd = vdd;

    const auto& ccs = rise ? timing.ccs_rise : timing.ccs_fall;

    if (ccs.valid()) {
        // CCS mode: integrate current waveform
        result.delay_ns = integrate_current(ccs, input_slew_ns, output_load_ff,
                                             vdd, 0.5);
        result.slew_ns = compute_output_slew(ccs, input_slew_ns, output_load_ff, vdd);
        result.c_eff_ff = output_load_ff;
    } else {
        // Fallback to NLDM
        return compute_nldm_delay(timing, input_slew_ns, output_load_ff, rise);
    }

    return result;
}

CcsDelayResult CcsDelayEngine::compute_delay_pi(const LibertyTiming& timing,
                                                   double input_slew_ns,
                                                   const PiModelCap& load,
                                                   double vdd,
                                                   bool rise) const {
    double c_eff = compute_effective_cap(timing, input_slew_ns, load, vdd, rise);

    CcsDelayResult result = compute_delay(timing, input_slew_ns, c_eff, vdd, rise);
    result.c_eff_ff = c_eff;
    result.output_load_ff = load.total_cap();
    result.miller_cap_ff = load.total_cap() - c_eff;
    return result;
}

CcsDelayResult CcsDelayEngine::compute_nldm_delay(const LibertyTiming& timing,
                                                     double input_slew_ns,
                                                     double output_load_ff,
                                                     bool rise) const {
    CcsDelayResult result;
    result.input_slew_ns = input_slew_ns;
    result.output_load_ff = output_load_ff;
    result.c_eff_ff = output_load_ff;

    const auto& table = rise ? timing.nldm_rise : timing.nldm_fall;
    const auto& slew_table = rise ? timing.nldm_rise_tr : timing.nldm_fall_tr;

    if (table.valid()) {
        result.delay_ns = table.interpolate(input_slew_ns, output_load_ff);
    } else {
        // Use scalar values
        const auto& scalar_table = rise ? timing.cell_rise : timing.cell_fall;
        // cell_rise/cell_fall may be NldmTable or scalar — handle both
        result.delay_ns = 0.1;  // safe default
    }

    if (slew_table.valid()) {
        result.slew_ns = slew_table.interpolate(input_slew_ns, output_load_ff);
    } else {
        result.slew_ns = 0.05;  // safe default
    }

    return result;
}

double CcsDelayEngine::compute_effective_cap(const LibertyTiming& timing,
                                               double input_slew_ns,
                                               const PiModelCap& load,
                                               double vdd,
                                               bool rise,
                                               int max_iter) const {
    double c_eff = load.total_cap();
    double prev_c_eff = c_eff;

    for (int iter = 0; iter < max_iter; ++iter) {
        auto result = compute_delay(timing, input_slew_ns, c_eff, vdd, rise);

        double new_c_eff = load.effective_cap(result.slew_ns);
        c_eff = 0.5 * (c_eff + new_c_eff);  // damped update

        if (std::abs(c_eff - prev_c_eff) < 0.001 * std::max(prev_c_eff, 0.001)) break;
        prev_c_eff = c_eff;
    }

    return c_eff;
}

CcsDelayResult CcsDelayEngine::compute_smis_delay(const LibertyTiming& timing,
                                                     double input_slew_ns,
                                                     double output_load_ff,
                                                     const SmisConfig& smis,
                                                     double vdd,
                                                     bool rise) const {
    double smis_factor = 1.0 + smis.alignment_factor * (smis.num_switching_inputs - 1) * 0.15;
    double effective_load = output_load_ff * smis_factor;

    return compute_delay(timing, input_slew_ns, effective_load, vdd, rise);
}

CcsDelayEngine::WireDelayResult CcsDelayEngine::compute_with_wire(
    const LibertyTiming& timing,
    double input_slew_ns,
    const ParasiticNet& wire_rc,
    double vdd,
    bool rise) const {
    WireDelayResult result;

    PiModelCap pi;
    if (!wire_rc.segments.empty()) {
        double half = wire_rc.segments.size() / 2.0;
        for (size_t i = 0; i < wire_rc.segments.size(); ++i) {
            if ((double)i < half) {
                pi.c_near += wire_rc.segments[i].capacitance;
            } else {
                pi.c_far += wire_rc.segments[i].capacitance;
            }
        }
        pi.r_wire = wire_rc.total_res_ohm;
        pi.c_far += wire_rc.total_coupling_ff;
    }

    auto ccs_result = compute_delay_pi(timing, input_slew_ns, pi, vdd, rise);
    result.driver_delay_ns = ccs_result.delay_ns;

    result.wire_delay_ns = wire_rc.elmore_delay_ps / 1000.0;

    result.total_delay_ns = result.driver_delay_ns + result.wire_delay_ns;
    result.total_slew_ns = ccs_result.slew_ns * (1.0 + 0.3 * pi.r_wire * pi.c_far);

    return result;
}

} // namespace sf
