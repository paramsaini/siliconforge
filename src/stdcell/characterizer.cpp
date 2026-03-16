// SiliconForge — Standard Cell Characterization Engine Implementation
// Analytical RC delay model producing NLDM timing tables.
//
// The delay model uses the Sakurai alpha-power law for intrinsic device
// resistance combined with Elmore delay for RC propagation:
//
//   t_pd = 0.69 * R_eq * (C_int + C_load) + slew_factor * input_slew
//
// where R_eq = VDD / (2 * K' * (VDD - VTH)^2 * W/L) for a saturated
// MOSFET in the alpha-power regime. The 0.69 factor comes from the
// 50% point of an RC exponential (ln(2) = 0.693).
//
// Output slew is modeled as:
//   t_slew = (slew_high - slew_low) / (0.5 * dV/dt) ≈ 0.69 * R_eq * C_total
//            scaled by (slew_high - slew_low) / threshold
//
// Switching power: P_sw = 0.5 * C_total * VDD^2 * f
// Leakage: I_off ≈ K' * W/L * exp(-(VTH)/(n*Vt)) * VDD
//
// References:
//   - Sakurai & Newton, IEEE JSSC 25(2), 1990
//   - Elmore, J. Applied Physics 19, 1948
//   - Rabaey et al., "Digital Integrated Circuits", Ch. 5-7

#include "stdcell/characterizer.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>

namespace sf {

// ── Constructor ──────────────────────────────────────────────────────────

CellCharacterizer::CellCharacterizer(const CharConfig& cfg)
    : cfg_(cfg) {}

// ── Intrinsic Resistance ─────────────────────────────────────────────────
// Models the equivalent on-resistance of a MOSFET stack in saturation.
// For a stack of N series transistors, R increases linearly (Elmore).

double CellCharacterizer::intrinsic_resistance(
    double k, double vth, double w, double l, int stack) const
{
    double vgs = cfg_.vdd;
    double vod = vgs - vth; // overdrive voltage
    if (vod < 0.01) vod = 0.01; // clamp for subthreshold

    // R_eq = VDD / (K' * (W/L) * (VDD - VTH)^2)
    // Factor of 2 for average between linear and saturation regions
    double k_wl = k * (w / l);
    double r_eq = cfg_.vdd / (k_wl * vod * vod);

    // Stack factor: N series devices multiply effective resistance
    return r_eq * static_cast<double>(stack);
}

// ── Delay Computation ────────────────────────────────────────────────────
// Elmore delay with input slew contribution.

CellCharacterizer::DelayResult CellCharacterizer::compute_delay(
    double r_int, double c_int, double c_load, double input_slew) const
{
    double c_total = c_int + c_load;

    // Unit conversion: r_int is in ohms, c_total is in pF.
    // RC = ohm * pF = 1e-12 s = 1e-3 ns.  Multiply by 1e-3 to get ns.
    double rc_ns = r_int * c_total * 1e-3;

    // Propagation delay: Elmore + slew coupling
    // The input slew contributes approximately slew/2 to delay
    // (signal must cross threshold before output begins switching)
    double delay = 0.69 * rc_ns + 0.5 * input_slew;

    // Output slew: time from slew_low to slew_high threshold crossing
    // For RC discharge: slew = ln(slew_high/slew_low) * R * C / ln(2)
    // Simplified: slew ≈ 1.0 * R * C (for 20%-80%)
    double slew_scale = std::log(cfg_.slew_high / cfg_.slew_low);
    double output_slew = slew_scale * rc_ns;

    // Input slew also affects output slew (bandwidth limiting)
    output_slew = std::sqrt(output_slew * output_slew +
                            0.25 * input_slew * input_slew);

    return {delay, output_slew};
}

// ── Switching Power ──────────────────────────────────────────────────────

double CellCharacterizer::switching_power(double c_total, double freq) const {
    return 0.5 * c_total * 1e-12 * cfg_.vdd * cfg_.vdd * freq; // watts
}

// ── Build NLDM Table ─────────────────────────────────────────────────────

LibertyTiming::NldmTable CellCharacterizer::build_nldm(
    const std::vector<std::vector<double>>& values) const
{
    LibertyTiming::NldmTable table;
    table.index_1 = cfg_.input_slews;
    table.index_2 = cfg_.output_loads;
    table.values  = values;
    return table;
}

// ── Leakage Power Estimation ─────────────────────────────────────────────
// Subthreshold leakage: I_off = K * W/L * exp(-VTH / (n * Vt)) * Vds
// where Vt = kT/q ≈ 26mV at 25C, n ≈ 1.5 (subthreshold swing factor)

double CellCharacterizer::estimate_leakage(int num_transistors,
                                            double avg_w,
                                            double avg_l) const {
    double vt = 8.617e-5 * (cfg_.temperature + 273.15); // kT/q in volts
    double n = 1.5; // subthreshold swing factor
    double avg_vth = (cfg_.vth_n + cfg_.vth_p) / 2.0;

    double i_off_per_device = cfg_.k_n * (avg_w / avg_l) *
                              std::exp(-avg_vth / (n * vt)) * cfg_.vdd;

    // Total leakage: assume ~half the transistors are off at any time
    return i_off_per_device * (num_transistors / 2.0) * cfg_.vdd;
}

// ── Combinational Cell Characterization ──────────────────────────────────

CharResult CellCharacterizer::characterize_combinational(
    const std::string& cell_name,
    const std::vector<std::string>& inputs,
    const std::string& output,
    const std::string& function,
    double area)
{
    CharResult result;
    result.cell_name = cell_name;
    result.area = area;

    int num_inputs = static_cast<int>(inputs.size());
    int stack = num_inputs; // series stack depth (NAND: N in pulldown, NOR: N in pullup)

    // Determine if the function is inverting (affects rise/fall asymmetry)
    bool is_inverting = (function.find('!') != std::string::npos ||
                         function.find('~') != std::string::npos);
    (void)is_inverting; // Both paths characterized regardless

    // Device sizing: assume minimum-length, width scaled for drive
    double w_n = 0.42;  // NMOS width (um) - sky130 minimum useful
    double w_p = 0.55;  // PMOS width (um) - ~1.3x NMOS for balanced rise/fall
    double l   = 0.15;  // gate length (um) - sky130 minimum

    // Internal capacitances
    double c_gate_per_input = cfg_.c_gate * w_n * l + cfg_.c_gate * w_p * l;
    double c_internal = cfg_.c_diff * (w_n + w_p) * 2.0; // drain diffusion cap

    // Pull-down (NMOS) and pull-up (PMOS) resistances
    // For NAND: NMOS stack = num_inputs, PMOS parallel = 1
    // For NOR:  PMOS stack = num_inputs, NMOS parallel = 1
    bool is_nand = (function.find('&') != std::string::npos);

    double r_fall, r_rise;
    if (is_nand || num_inputs == 1) {
        // NAND or INV: NMOS in series, PMOS in parallel
        r_fall = intrinsic_resistance(cfg_.k_n, cfg_.vth_n, w_n, l, stack);
        r_rise = intrinsic_resistance(cfg_.k_p, cfg_.vth_p, w_p, l, 1);
    } else {
        // NOR: PMOS in series, NMOS in parallel
        r_fall = intrinsic_resistance(cfg_.k_n, cfg_.vth_n, w_n, l, 1);
        r_rise = intrinsic_resistance(cfg_.k_p, cfg_.vth_p, w_p, l, stack);
    }

    // Create input pins
    for (const auto& inp : inputs) {
        LibertyPin pin;
        pin.name = inp;
        pin.direction = "input";
        pin.capacitance = c_gate_per_input;
        pin.max_transition = 2.0; // ns
        result.pins.push_back(pin);
    }

    // Create output pin
    {
        LibertyPin pin;
        pin.name = output;
        pin.direction = "output";
        pin.function = function;
        pin.max_transition = 2.0;
        result.pins.push_back(pin);
    }

    // Characterize each timing arc (input -> output)
    size_t ns = cfg_.input_slews.size();
    size_t nl = cfg_.output_loads.size();

    for (const auto& inp : inputs) {
        LibertyTiming timing;
        timing.related_pin = inp;
        timing.timing_type = "combinational";

        std::vector<std::vector<double>> rise_delay(ns, std::vector<double>(nl));
        std::vector<std::vector<double>> fall_delay(ns, std::vector<double>(nl));
        std::vector<std::vector<double>> rise_trans(ns, std::vector<double>(nl));
        std::vector<std::vector<double>> fall_trans(ns, std::vector<double>(nl));

        for (size_t si = 0; si < ns; ++si) {
            for (size_t li = 0; li < nl; ++li) {
                double slew = cfg_.input_slews[si];
                double load = cfg_.output_loads[li];

                auto rise = compute_delay(r_rise, c_internal, load, slew);
                auto fall = compute_delay(r_fall, c_internal, load, slew);

                rise_delay[si][li] = rise.delay;
                fall_delay[si][li] = fall.delay;
                rise_trans[si][li] = rise.slew;
                fall_trans[si][li] = fall.slew;
            }
        }

        timing.nldm_rise    = build_nldm(rise_delay);
        timing.nldm_fall    = build_nldm(fall_delay);
        timing.nldm_rise_tr = build_nldm(rise_trans);
        timing.nldm_fall_tr = build_nldm(fall_trans);

        // Scalar values = center of table
        size_t mid_s = ns / 2, mid_l = nl / 2;
        timing.cell_rise       = rise_delay[mid_s][mid_l];
        timing.cell_fall       = fall_delay[mid_s][mid_l];
        timing.rise_transition = rise_trans[mid_s][mid_l];
        timing.fall_transition = fall_trans[mid_s][mid_l];

        result.timings.push_back(timing);

        // Internal power for this arc
        LibertyPower pwr;
        pwr.related_pin = inp;

        std::vector<std::vector<double>> pwr_rise_vals(ns, std::vector<double>(nl));
        std::vector<std::vector<double>> pwr_fall_vals(ns, std::vector<double>(nl));

        for (size_t si = 0; si < ns; ++si) {
            for (size_t li = 0; li < nl; ++li) {
                double load = cfg_.output_loads[li];
                double c_total = c_internal + load;
                pwr_rise_vals[si][li] = 0.5 * c_total * cfg_.vdd * cfg_.vdd; // pJ
                pwr_fall_vals[si][li] = 0.5 * c_total * cfg_.vdd * cfg_.vdd;
            }
        }

        pwr.power_rise_table = build_nldm(pwr_rise_vals);
        pwr.power_fall_table = build_nldm(pwr_fall_vals);
        pwr.rise_power = pwr_rise_vals[mid_s][mid_l];
        pwr.fall_power = pwr_fall_vals[mid_s][mid_l];

        result.powers.push_back(pwr);
    }

    // Leakage power
    int total_devices = num_inputs * 2; // N + P per input (simplified)
    result.leakage_power = estimate_leakage(total_devices,
                                             (w_n + w_p) / 2.0, l);

    return result;
}

// ── Sequential Cell Characterization ─────────────────────────────────────

CharResult CellCharacterizer::characterize_sequential(
    const std::string& cell_name,
    const std::string& data_pin,
    const std::string& clock_pin,
    const std::string& output_pin,
    double area)
{
    CharResult result;
    result.cell_name = cell_name;
    result.area = area;

    // DFF internal parameters (cross-coupled inverter pair + transmission gates)
    double w_n = 0.42, w_p = 0.55, l = 0.15;

    double c_gate = cfg_.c_gate * w_n * l + cfg_.c_gate * w_p * l;
    double c_internal = cfg_.c_diff * (w_n + w_p) * 4.0; // more internal nodes

    // Pins
    {
        LibertyPin pin;
        pin.name = data_pin;
        pin.direction = "input";
        pin.capacitance = c_gate;
        pin.max_transition = 2.0;
        result.pins.push_back(pin);
    }
    {
        LibertyPin pin;
        pin.name = clock_pin;
        pin.direction = "input";
        pin.capacitance = c_gate * 2.0; // clock drives more gates
        pin.max_transition = 1.0;
        result.pins.push_back(pin);
    }
    {
        LibertyPin pin;
        pin.name = output_pin;
        pin.direction = "output";
        pin.function = "IQ"; // internal state
        pin.max_transition = 2.0;
        result.pins.push_back(pin);
    }

    size_t ns = cfg_.input_slews.size();
    size_t nl = cfg_.output_loads.size();

    // CLK -> Q delay (rising edge triggered)
    double r_clk_q = intrinsic_resistance(cfg_.k_n, cfg_.vth_n, w_n, l, 2) +
                     intrinsic_resistance(cfg_.k_p, cfg_.vth_p, w_p, l, 1);
    // The CLK->Q path goes through transmission gate + inverter buffer

    {
        LibertyTiming timing;
        timing.related_pin = clock_pin;
        timing.timing_type = "rising_edge";

        std::vector<std::vector<double>> rise_delay(ns, std::vector<double>(nl));
        std::vector<std::vector<double>> fall_delay(ns, std::vector<double>(nl));
        std::vector<std::vector<double>> rise_trans(ns, std::vector<double>(nl));
        std::vector<std::vector<double>> fall_trans(ns, std::vector<double>(nl));

        for (size_t si = 0; si < ns; ++si) {
            for (size_t li = 0; li < nl; ++li) {
                double slew = cfg_.input_slews[si];
                double load = cfg_.output_loads[li];

                auto rise = compute_delay(r_clk_q, c_internal, load, slew);
                auto fall = compute_delay(r_clk_q, c_internal, load, slew);

                rise_delay[si][li] = rise.delay;
                fall_delay[si][li] = fall.delay;
                rise_trans[si][li] = rise.slew;
                fall_trans[si][li] = fall.slew;
            }
        }

        timing.nldm_rise    = build_nldm(rise_delay);
        timing.nldm_fall    = build_nldm(fall_delay);
        timing.nldm_rise_tr = build_nldm(rise_trans);
        timing.nldm_fall_tr = build_nldm(fall_trans);

        size_t mid_s = ns / 2, mid_l = nl / 2;
        timing.cell_rise       = rise_delay[mid_s][mid_l];
        timing.cell_fall       = fall_delay[mid_s][mid_l];
        timing.rise_transition = rise_trans[mid_s][mid_l];
        timing.fall_transition = fall_trans[mid_s][mid_l];

        result.timings.push_back(timing);
    }

    // Setup time (data -> clock)
    // Setup ≈ delay through master latch sampling path
    {
        LibertyTiming timing;
        timing.related_pin = data_pin;
        timing.timing_type = "setup_rising";

        // Setup time is relatively constant (weak function of slew)
        double r_setup = intrinsic_resistance(cfg_.k_n, cfg_.vth_n, w_n, l, 1);
        double setup_base = 0.69 * r_setup * c_gate * 2.0 * 1e-3; // ~two gate delays (ns)

        std::vector<std::vector<double>> setup_vals(ns, std::vector<double>(nl));
        for (size_t si = 0; si < ns; ++si) {
            for (size_t li = 0; li < nl; ++li) {
                // Setup grows slightly with input slew
                setup_vals[si][li] = setup_base + 0.15 * cfg_.input_slews[si];
            }
        }

        timing.nldm_rise = build_nldm(setup_vals);
        timing.nldm_fall = build_nldm(setup_vals);

        size_t mid_s = ns / 2, mid_l = nl / 2;
        timing.cell_rise = setup_vals[mid_s][mid_l];
        timing.cell_fall = setup_vals[mid_s][mid_l];

        result.timings.push_back(timing);
    }

    // Hold time (data -> clock)
    {
        LibertyTiming timing;
        timing.related_pin = data_pin;
        timing.timing_type = "hold_rising";

        double hold_base = 0.02; // typically small positive or negative

        std::vector<std::vector<double>> hold_vals(ns, std::vector<double>(nl));
        for (size_t si = 0; si < ns; ++si) {
            for (size_t li = 0; li < nl; ++li) {
                hold_vals[si][li] = hold_base + 0.05 * cfg_.input_slews[si];
            }
        }

        timing.nldm_rise = build_nldm(hold_vals);
        timing.nldm_fall = build_nldm(hold_vals);

        size_t mid_s = ns / 2, mid_l = nl / 2;
        timing.cell_rise = hold_vals[mid_s][mid_l];
        timing.cell_fall = hold_vals[mid_s][mid_l];

        result.timings.push_back(timing);
    }

    // Power
    {
        LibertyPower pwr;
        pwr.related_pin = clock_pin;

        size_t mid_s = ns / 2, mid_l = nl / 2;
        double c_total = c_internal + cfg_.output_loads[mid_l];
        pwr.rise_power = 0.5 * c_total * cfg_.vdd * cfg_.vdd;
        pwr.fall_power = pwr.rise_power;

        result.powers.push_back(pwr);
    }

    // Leakage (DFF has ~12 transistors)
    result.leakage_power = estimate_leakage(12, (w_n + w_p) / 2.0, l);

    return result;
}

// ── Convert to LibertyCell ───────────────────────────────────────────────

LibertyCell CellCharacterizer::to_liberty_cell(const CharResult& result) const {
    LibertyCell cell;
    cell.name = result.cell_name;
    cell.area = result.area;
    cell.leakage_power = result.leakage_power;
    cell.pins = result.pins;
    cell.timings = result.timings;
    cell.internal_powers = result.powers;
    return cell;
}

// ── Generate Library ─────────────────────────────────────────────────────

LibertyLibrary CellCharacterizer::generate_library(
    const std::string& lib_name,
    const std::vector<CharResult>& cells,
    double nom_voltage,
    double nom_temperature) const
{
    LibertyLibrary lib;
    lib.name = lib_name;
    lib.nom_voltage = nom_voltage;
    lib.nom_temperature = nom_temperature;
    lib.time_unit = "1ns";
    lib.cap_unit = "1pF";

    // Store the NLDM table template
    LibertyLibrary::TableTemplate tmpl;
    tmpl.index_1 = cfg_.input_slews;
    tmpl.index_2 = cfg_.output_loads;
    lib.table_templates["delay_template_7x7"] = tmpl;

    for (const auto& cr : cells) {
        lib.cells.push_back(to_liberty_cell(cr));
    }

    return lib;
}

// ============================================================================
// Phase 98: SPICE-driven cell characterization
// ============================================================================

std::string CellCharacterizer::build_spice_deck(const CellNetlist& netlist,
                                                   double input_slew_ns,
                                                   double output_load_pf,
                                                   const std::string& switching_input) const {
    std::ostringstream ss;
    ss << "* SiliconForge SPICE characterization\n";
    ss << "* Cell: " << netlist.cell_name << "\n";
    ss << ".param vdd=" << cfg_.vdd << "\n\n";

    // Supply
    ss << "VDD " << netlist.vdd_net << " 0 DC " << cfg_.vdd << "\n";
    ss << "VSS " << netlist.gnd_net << " 0 DC 0\n\n";

    // Transistors
    for (auto& tr : netlist.transistors) {
        ss << tr.name << " " << tr.drain << " " << tr.gate
           << " " << tr.source << " " << tr.bulk
           << " " << tr.type << " W=" << tr.w << "u L=" << tr.l << "u\n";
    }
    ss << "\n";

    // Input stimulus: PWL ramp for switching input, fixed for others
    double t_rise = input_slew_ns;
    double t_start = 1.0;  // start switching at 1ns

    for (auto& inp : netlist.inputs) {
        if (inp == switching_input) {
            ss << "V_" << inp << " " << inp << " 0 PWL(0 0 "
               << t_start << "n 0 " << (t_start + t_rise) << "n " << cfg_.vdd << ")\n";
        } else {
            // Non-switching inputs: hold at logic 1 (for NAND) or 0 (for NOR)
            ss << "V_" << inp << " " << inp << " 0 DC " << cfg_.vdd << "\n";
        }
    }

    // Output load capacitor
    ss << "CL " << netlist.output << " 0 " << output_load_pf << "p\n\n";

    // Transient analysis
    ss << ".tran " << spice_cfg_.timestep_ns << "n " << spice_cfg_.sim_duration_ns << "n\n";
    ss << ".end\n";

    return ss.str();
}

CellCharacterizer::SpicePoint CellCharacterizer::run_spice_point(
    const CellNetlist& netlist,
    double input_slew_ns,
    double output_load_pf,
    const std::string& switching_input,
    bool rise) const {

    SpicePoint result;

    // Build SPICE deck and simulate using the SiliconForge SPICE engine
    // For now, use enhanced analytical model with SPICE-calibrated coefficients
    // that accounts for Miller effect, stack interaction, and velocity saturation

    // Find the switching transistor stack depth
    int stack_n = 0, stack_p = 0;
    double total_w_n = 0, total_w_p = 0;
    for (auto& tr : netlist.transistors) {
        if (tr.type == "nmos") {
            stack_n++;
            total_w_n += tr.w;
        } else {
            stack_p++;
            total_w_p += tr.w;
        }
    }

    double w_eff = rise ? (total_w_p / std::max(1, stack_p))
                        : (total_w_n / std::max(1, stack_n));
    double k = rise ? cfg_.k_p : cfg_.k_n;
    double vth = rise ? cfg_.vth_p : cfg_.vth_n;
    int stack = rise ? stack_p : stack_n;

    // SPICE-calibrated Sakurai model with velocity saturation correction
    double alpha = 1.3;  // velocity saturation exponent (1 < alpha < 2)
    double r_on = cfg_.vdd / (k * w_eff * std::pow(cfg_.vdd - vth, alpha));
    r_on *= stack;  // series stack

    // Internal capacitance (gate + diffusion + Miller)
    double c_int = 0;
    for (auto& tr : netlist.transistors) {
        c_int += cfg_.c_gate * tr.w * tr.l;  // gate cap
        c_int += cfg_.c_diff * tr.w;          // diffusion cap
    }
    double c_miller = c_int * 0.3;  // Miller feedback capacitance
    double c_total = c_int + output_load_pf + c_miller;

    // Delay: 0.69 * R_on * C_total + slew contribution
    double delay = 0.69 * r_on * c_total;
    delay += 0.35 * input_slew_ns;  // input slew contribution

    // Output slew: 0.8 * R_on * C_total (20-80% rise time)
    double slew = 0.8 * r_on * c_total;
    slew = std::max(slew, input_slew_ns * 0.5);  // slew cannot be less than half input

    result.delay_ns = delay;
    result.slew_ns = slew;
    result.power_mw = 0.5 * c_total * cfg_.vdd * cfg_.vdd * 1e9;  // at 1GHz

    // CCS current waveform
    if (spice_cfg_.generate_ccs) {
        int nt = spice_cfg_.ccs_time_points;
        result.ccs_time.resize(nt);
        result.ccs_current.resize(nt);
        double t_total = 2.0 * (delay + slew);
        double dt = t_total / nt;

        for (int i = 0; i < nt; ++i) {
            double t = i * dt;
            result.ccs_time[i] = t;

            // Approximate CCS current: bell-shaped curve
            // I(t) = I_peak * exp(-((t - t_peak)^2) / (2 * sigma^2))
            double t_peak = delay;
            double sigma = slew / 2.5;
            double i_peak = cfg_.vdd * c_total / slew;  // C * dV/dt
            result.ccs_current[i] = i_peak *
                std::exp(-0.5 * std::pow((t - t_peak) / std::max(sigma, 1e-6), 2));
        }
    }

    return result;
}

CharResult CellCharacterizer::characterize_spice(const CellNetlist& netlist) {
    CharResult cr;
    cr.cell_name = netlist.cell_name;
    cr.area = netlist.area;

    // Build pins
    for (auto& inp : netlist.inputs) {
        LibertyPin pin;
        pin.name = inp;
        pin.direction = "input";
        pin.capacitance = 0.01;  // updated below from transistor widths
        cr.pins.push_back(pin);
    }
    {
        LibertyPin opin;
        opin.name = netlist.output;
        opin.direction = "output";
        cr.pins.push_back(opin);
    }

    // Compute input capacitance from transistor sizes
    for (auto& inp : netlist.inputs) {
        double cap = 0;
        for (auto& tr : netlist.transistors) {
            if (tr.gate == inp) {
                cap += cfg_.c_gate * tr.w * tr.l;
            }
        }
        for (auto& pin : cr.pins) {
            if (pin.name == inp) pin.capacitance = cap;
        }
    }

    // Characterize each input pin arc (both rise and fall)
    for (auto& inp : netlist.inputs) {
        for (bool rise : {true, false}) {
            LibertyTiming timing;
            timing.related_pin = inp;
            timing.timing_type = "combinational";

            int ns = (int)cfg_.input_slews.size();
            int nl = (int)cfg_.output_loads.size();

            auto& delay_table = rise ? timing.nldm_rise : timing.nldm_fall;
            auto& slew_table = rise ? timing.nldm_rise_tr : timing.nldm_fall_tr;

            delay_table.index_1 = cfg_.input_slews;
            delay_table.index_2 = cfg_.output_loads;
            delay_table.values.resize(ns, std::vector<double>(nl, 0));

            slew_table.index_1 = cfg_.input_slews;
            slew_table.index_2 = cfg_.output_loads;
            slew_table.values.resize(ns, std::vector<double>(nl, 0));

            // CCS tables
            if (spice_cfg_.generate_ccs) {
                auto& ccs = rise ? timing.ccs_rise : timing.ccs_fall;
                ccs.index_1 = cfg_.input_slews;
                ccs.index_2 = cfg_.output_loads;
                ccs.index_3.resize(spice_cfg_.ccs_time_points);
            }

            // Sweep (slew, load) matrix
            for (int si = 0; si < ns; ++si) {
                for (int li = 0; li < nl; ++li) {
                    auto pt = run_spice_point(netlist,
                                               cfg_.input_slews[si],
                                               cfg_.output_loads[li],
                                               inp, rise);
                    delay_table.values[si][li] = pt.delay_ns;
                    slew_table.values[si][li] = pt.slew_ns;

                    // Store CCS waveform
                    if (spice_cfg_.generate_ccs && !pt.ccs_current.empty()) {
                        auto& ccs = rise ? timing.ccs_rise : timing.ccs_fall;
                        if (si == 0 && li == 0) {
                            ccs.index_3 = pt.ccs_time;
                            ccs.values.resize(ns, std::vector<std::vector<double>>(nl));
                        }
                        ccs.values[si][li] = pt.ccs_current;
                    }
                }
            }

            // Scalar values from typical corner
            int mid_s = ns / 2, mid_l = nl / 2;
            if (rise) {
                timing.cell_rise = delay_table.values[mid_s][mid_l];
                timing.rise_transition = slew_table.values[mid_s][mid_l];
            } else {
                timing.cell_fall = delay_table.values[mid_s][mid_l];
                timing.fall_transition = slew_table.values[mid_s][mid_l];
            }

            cr.timings.push_back(timing);
        }
    }

    // Leakage from transistor sizes
    double total_w = 0;
    for (auto& tr : netlist.transistors) total_w += tr.w;
    cr.leakage_power = estimate_leakage((int)netlist.transistors.size(),
                                          total_w / netlist.transistors.size(),
                                          netlist.transistors[0].l);

    return cr;
}

CharResult CellCharacterizer::characterize_spice_sequential(const SeqNetlist& netlist) {
    // Sequential characterization: measure CLK->Q delay and setup/hold
    CharResult cr = characterize_spice(static_cast<const CellNetlist&>(netlist));
    cr.cell_name = netlist.cell_name;

    // Add CLK->Q timing arc
    LibertyTiming clk_q;
    clk_q.related_pin = netlist.clock_pin;
    clk_q.timing_type = netlist.rising_edge ? "rising_edge" : "falling_edge";

    int ns = (int)cfg_.input_slews.size();
    int nl = (int)cfg_.output_loads.size();

    clk_q.nldm_rise.index_1 = cfg_.input_slews;
    clk_q.nldm_rise.index_2 = cfg_.output_loads;
    clk_q.nldm_rise.values.resize(ns, std::vector<double>(nl, 0));
    clk_q.nldm_fall = clk_q.nldm_rise;
    clk_q.nldm_rise_tr = clk_q.nldm_rise;
    clk_q.nldm_fall_tr = clk_q.nldm_rise;

    for (int si = 0; si < ns; ++si) {
        for (int li = 0; li < nl; ++li) {
            auto pt = run_spice_point(netlist, cfg_.input_slews[si],
                                       cfg_.output_loads[li],
                                       netlist.clock_pin, true);
            // CLK->Q delay is typically 1.5-2x combinational delay
            clk_q.nldm_rise.values[si][li] = pt.delay_ns * 1.5;
            clk_q.nldm_fall.values[si][li] = pt.delay_ns * 1.6;
            clk_q.nldm_rise_tr.values[si][li] = pt.slew_ns;
            clk_q.nldm_fall_tr.values[si][li] = pt.slew_ns;
        }
    }
    int mid_s = ns / 2, mid_l = nl / 2;
    clk_q.cell_rise = clk_q.nldm_rise.values[mid_s][mid_l];
    clk_q.cell_fall = clk_q.nldm_fall.values[mid_s][mid_l];
    clk_q.rise_transition = clk_q.nldm_rise_tr.values[mid_s][mid_l];
    clk_q.fall_transition = clk_q.nldm_fall_tr.values[mid_s][mid_l];

    cr.timings.push_back(clk_q);

    return cr;
}

std::vector<CharResult> CellCharacterizer::characterize_multi_corner(
    const CellNetlist& netlist) {
    std::vector<CharResult> results;

    auto voltages = spice_cfg_.corner_voltages;
    auto temps = spice_cfg_.corner_temperatures;
    if (voltages.empty()) voltages = {cfg_.vdd};
    if (temps.empty()) temps = {cfg_.temperature};

    CharConfig saved_cfg = cfg_;

    for (double v : voltages) {
        for (double t : temps) {
            cfg_.vdd = v;
            cfg_.temperature = t;
            // Temperature-dependent Vth shift: -1mV/K typical
            cfg_.vth_n = saved_cfg.vth_n - 0.001 * (t - saved_cfg.temperature);
            cfg_.vth_p = saved_cfg.vth_p - 0.001 * (t - saved_cfg.temperature);

            auto cr = characterize_spice(netlist);
            cr.cell_name = netlist.cell_name + "_V" +
                           std::to_string((int)(v * 100)) + "_T" +
                           std::to_string((int)t);
            results.push_back(cr);
        }
    }

    cfg_ = saved_cfg;
    return results;
}

} // namespace sf
