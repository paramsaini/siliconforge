#pragma once
// SiliconForge — CCS (Composite Current Source) Delay Engine
// Signoff-quality delay computation using current-source driver model
// with receiver capacitance (pi-model) and AWE wire delay.
//
// CCS delay replaces the voltage-step NLDM model with a time-varying
// current waveform that more accurately captures Miller effect, stack
// interaction, and multi-input switching at sub-28nm nodes.
//
// References:
//   Synopsys CCS timing model, Liberty LRM 2007
//   Keller et al., "Robust Analytical Gate Delay Modeling", DAC 2004
//   O'Brien & Savarino, "Modeling the Driving-Point Characteristic of
//                        Resistive Interconnect for Accurate Delay Est.", DAC 1989

#include "core/liberty_parser.hpp"
#include "timing/parasitics.hpp"
#include <vector>
#include <string>
#include <cmath>

namespace sf {

// CCS delay computation result
struct CcsDelayResult {
    double delay_ns = 0;           // 50% crossing delay
    double slew_ns = 0;            // 20-80% output transition time
    double input_slew_ns = 0;      // input slew used
    double output_load_ff = 0;     // total output load (C_eff)
    double miller_cap_ff = 0;      // Miller capacitance contribution
    double c_eff_ff = 0;           // effective capacitance seen by driver

    // Current waveform
    std::vector<double> time_points;
    std::vector<double> current_ma;

    // Voltage waveform at output
    std::vector<double> v_out;
    double vdd = 1.0;
};

// Pi-model receiver capacitance (CCS receiver model)
struct PiModelCap {
    double c_near = 0;    // near-end capacitance (C1)
    double r_wire = 0;    // interconnect resistance
    double c_far = 0;     // far-end capacitance (C2)

    double total_cap() const { return c_near + c_far; }

    // Effective capacitance (Thevenin equivalent)
    // C_eff accounts for shielding of far-end cap by wire resistance
    double effective_cap(double slew_ns) const {
        if (r_wire < 1e-6 || slew_ns < 1e-12) return c_near + c_far;
        double tau = r_wire * c_far;
        double ratio = tau / std::max(slew_ns, 1e-12);
        // Ceff formula from O'Brien-Savarino
        double k = 1.0 - 0.6 * (1.0 - std::exp(-ratio));
        return c_near + k * c_far;
    }
};

class CcsDelayEngine {
public:
    // Compute CCS delay for a cell timing arc
    // Looks up CCS current waveform tables and integrates the current
    // into a load to find the 50% crossing time.
    CcsDelayResult compute_delay(const LibertyTiming& timing,
                                  double input_slew_ns,
                                  double output_load_ff,
                                  double vdd = 1.0,
                                  bool rise = true) const;

    // CCS delay with pi-model receiver (more accurate for routed nets)
    CcsDelayResult compute_delay_pi(const LibertyTiming& timing,
                                     double input_slew_ns,
                                     const PiModelCap& load,
                                     double vdd = 1.0,
                                     bool rise = true) const;

    // Fallback: NLDM delay when CCS tables aren't available
    CcsDelayResult compute_nldm_delay(const LibertyTiming& timing,
                                       double input_slew_ns,
                                       double output_load_ff,
                                       bool rise = true) const;

    // Effective capacitance iteration
    // Iteratively finds C_eff such that the CCS driver's average current
    // matches what a simple capacitor C_eff would draw.
    double compute_effective_cap(const LibertyTiming& timing,
                                  double input_slew_ns,
                                  const PiModelCap& load,
                                  double vdd = 1.0,
                                  bool rise = true,
                                  int max_iter = 5) const;

    // Simultaneous Multi-Input Switching (SMIS)
    // Computes worst-case delay when multiple inputs switch simultaneously.
    struct SmisConfig {
        int num_switching_inputs = 1;
        double alignment_factor = 0.5;  // 0 = best case, 1 = worst case
    };
    CcsDelayResult compute_smis_delay(const LibertyTiming& timing,
                                       double input_slew_ns,
                                       double output_load_ff,
                                       const SmisConfig& smis,
                                       double vdd = 1.0,
                                       bool rise = true) const;

    // AWE wire delay integration
    // Combines CCS driver delay with AWE interconnect delay
    struct WireDelayResult {
        double driver_delay_ns = 0;
        double wire_delay_ns = 0;
        double total_delay_ns = 0;
        double total_slew_ns = 0;
    };
    WireDelayResult compute_with_wire(const LibertyTiming& timing,
                                       double input_slew_ns,
                                       const ParasiticNet& wire_rc,
                                       double vdd = 1.0,
                                       bool rise = true) const;

private:
    // Integrate CCS current waveform through a load capacitance
    // Returns time at which output voltage crosses threshold
    double integrate_current(const CcsTable& ccs,
                              double input_slew,
                              double load_ff,
                              double vdd,
                              double threshold_pct) const;

    // Compute output slew from CCS waveform
    double compute_output_slew(const CcsTable& ccs,
                                double input_slew,
                                double load_ff,
                                double vdd) const;
};

} // namespace sf
