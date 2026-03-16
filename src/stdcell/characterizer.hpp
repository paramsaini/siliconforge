#pragma once
// SiliconForge — Standard Cell Characterization Engine
// Generates NLDM timing/power tables for Liberty (.lib) output.
// Uses analytical RC delay models calibrated to foundry SPICE data;
// upgradeable to full transient SPICE simulation when the engine is ready.
//
// References:
//   - Synopsys Liberty User Guide (NLDM table format)
//   - Rabaey, Chandrakasan, Nikolic, "Digital Integrated Circuits", 2nd ed.
//     Ch. 5 (The Inverter), Ch. 6 (Combinational Logic), Ch. 7 (Sequential)
//   - Sakurai & Newton, "Alpha-Power Law MOSFET Model", IEEE JSSC 1990
//   - Elmore delay model: W.C. Elmore, "The Transient Response of Damped
//     Linear Networks", J. Applied Physics 19, 1948

#include "core/liberty_parser.hpp"
#include <string>
#include <vector>

namespace sf {

// ── Characterization Configuration ───────────────────────────────────────

struct CharConfig {
    // NLDM table index breakpoints
    std::vector<double> input_slews  = {0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0};  // ns
    std::vector<double> output_loads = {0.001, 0.005, 0.01, 0.02, 0.05, 0.1, 0.5}; // pF

    // Operating point
    double vdd         = 1.8;    // supply voltage (V)
    double temperature = 25.0;   // junction temperature (C)

    // Delay measurement thresholds (fraction of VDD)
    double threshold  = 0.5;     // 50% VDD crossing for delay
    double slew_low   = 0.2;     // 20% VDD for slew measurement
    double slew_high  = 0.8;     // 80% VDD for slew measurement

    // Process parameters (sky130 nmos/pmos typical)
    double vth_n      = 0.49;    // NMOS threshold voltage (V)
    double vth_p      = 0.42;    // PMOS threshold voltage (V)
    double k_n        = 170e-6;  // NMOS transconductance (A/V^2) per um/um
    double k_p        = 70e-6;   // PMOS transconductance (A/V^2) per um/um
    double c_gate     = 1.0e-3;  // gate capacitance per um^2 (pF/um^2)
    double c_diff     = 0.5e-3;  // diffusion capacitance per um (pF/um)
};

// ── Characterization Result ──────────────────────────────────────────────

struct CharResult {
    std::string cell_name;
    double area = 0.0;
    std::vector<LibertyPin>    pins;
    std::vector<LibertyTiming> timings;
    std::vector<LibertyPower>  powers;
    double leakage_power = 0.0;  // watts
};

// ── Cell Characterizer ───────────────────────────────────────────────────

class CellCharacterizer {
public:
    explicit CellCharacterizer(const CharConfig& cfg);

    /// Characterize a combinational cell (INV, NAND, NOR, etc.).
    /// @param cell_name  e.g., "INV_X1", "NAND2_X1"
    /// @param inputs     input pin names
    /// @param output     output pin name
    /// @param function   Boolean function string, e.g. "!(A & B)"
    /// @param area       cell area in um^2
    CharResult characterize_combinational(
        const std::string& cell_name,
        const std::vector<std::string>& inputs,
        const std::string& output,
        const std::string& function,
        double area);

    /// Characterize a sequential cell (DFF, DFFR, etc.).
    /// @param cell_name   e.g., "DFF_X1"
    /// @param data_pin    data input pin name
    /// @param clock_pin   clock pin name
    /// @param output_pin  Q output pin name
    /// @param area        cell area in um^2
    CharResult characterize_sequential(
        const std::string& cell_name,
        const std::string& data_pin,
        const std::string& clock_pin,
        const std::string& output_pin,
        double area);

    /// Convert a CharResult into a LibertyCell suitable for .lib writing.
    LibertyCell to_liberty_cell(const CharResult& result) const;

    /// Generate a complete LibertyLibrary from a set of characterized cells.
    LibertyLibrary generate_library(
        const std::string& lib_name,
        const std::vector<CharResult>& cells,
        double nom_voltage,
        double nom_temperature) const;

private:
    CharConfig cfg_;

    // Analytical delay/slew models (Sakurai alpha-power / Elmore)

    /// Intrinsic resistance of a pull-down/pull-up stack.
    /// @param k        transconductance parameter (A/V^2)
    /// @param vth      threshold voltage (V)
    /// @param w        device width (um)
    /// @param l        device length (um)
    /// @param stack    number of series devices (1 for INV, 2 for NAND2, etc.)
    double intrinsic_resistance(double k, double vth, double w, double l,
                                int stack) const;

    /// Compute propagation delay for one input slew / output load point.
    /// Returns {delay_ns, output_slew_ns}.
    struct DelayResult { double delay; double slew; };
    DelayResult compute_delay(double r_int, double c_int, double c_load,
                              double input_slew) const;

    /// Compute switching power for one transition.
    double switching_power(double c_total, double freq = 1e9) const;

    /// Build an NLDM table from a 2D sweep.
    LibertyTiming::NldmTable build_nldm(
        const std::vector<std::vector<double>>& values) const;

    /// Estimate leakage power from subthreshold current.
    double estimate_leakage(int num_transistors, double avg_w, double avg_l) const;

public:
    // ── Phase 98: SPICE-driven characterization ──────────────────────────
    // Replaces analytical Sakurai/Elmore model with actual transient SPICE
    // simulation for each (slew, load) table point.  Produces CCS current
    // waveforms in addition to NLDM delay/slew tables.
    //
    // Flow:  1) Build SPICE netlist from cell topology
    //        2) For each (input_slew, output_load) point:
    //           a) Run transient simulation with PWL input
    //           b) Measure delay (50% threshold crossing)
    //           c) Measure slew (20%-80%)
    //           d) Record output current waveform (CCS)
    //        3) Pack results into LibertyTiming with NLDM + CCS tables

    struct SpiceCharConfig {
        double sim_duration_ns = 10.0;    // transient sim length
        double timestep_ns = 0.001;        // simulation timestep
        double supply_ramp_ns = 0.01;      // power supply ramp time
        bool generate_ccs = true;          // produce CCS current tables
        int ccs_time_points = 32;          // time samples per CCS waveform
        std::string spice_model_file;      // path to .spice model file
        bool multi_corner = false;         // characterize multiple corners
        std::vector<double> corner_voltages;    // e.g., {1.62, 1.8, 1.98}
        std::vector<double> corner_temperatures; // e.g., {-40, 25, 125}
    };

    void set_spice_config(const SpiceCharConfig& cfg) { spice_cfg_ = cfg; }

    // SPICE-driven characterization of a combinational cell
    // Requires transistor-level netlist description
    struct TransistorInfo {
        std::string name;            // M0, M1, ...
        std::string type;            // "nmos" or "pmos"
        double w = 1.0;             // width in um
        double l = 0.18;            // length in um
        std::string gate, drain, source, bulk;
    };
    struct CellNetlist {
        std::string cell_name;
        std::vector<std::string> inputs;
        std::string output;
        std::string vdd_net = "VDD";
        std::string gnd_net = "VSS";
        std::vector<TransistorInfo> transistors;
        double area = 0;
    };

    CharResult characterize_spice(const CellNetlist& netlist);

    // SPICE-driven sequential cell characterization
    struct SeqNetlist : CellNetlist {
        std::string clock_pin;
        std::string q_pin;
        std::string qn_pin;         // optional inverted output
        bool rising_edge = true;
    };
    CharResult characterize_spice_sequential(const SeqNetlist& netlist);

    // Multi-corner characterization: runs characterize_spice at each
    // (voltage, temperature) corner and returns per-corner results
    std::vector<CharResult> characterize_multi_corner(const CellNetlist& netlist);

private:
    SpiceCharConfig spice_cfg_;

    // Build SPICE deck string from transistor-level netlist
    std::string build_spice_deck(const CellNetlist& netlist,
                                  double input_slew_ns,
                                  double output_load_pf,
                                  const std::string& switching_input) const;

    // Run single SPICE point and measure
    struct SpicePoint {
        double delay_ns = 0;
        double slew_ns = 0;
        double power_mw = 0;
        std::vector<double> ccs_time;     // CCS current waveform time points
        std::vector<double> ccs_current;  // CCS current samples
    };
    SpicePoint run_spice_point(const CellNetlist& netlist,
                                double input_slew_ns,
                                double output_load_pf,
                                const std::string& switching_input,
                                bool rise) const;
};

} // namespace sf
