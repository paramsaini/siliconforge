// SPICE Simulation Engine — MNA-based DC operating point (Newton-Raphson)
// and transient analysis (Backward Euler) with measurement utilities.
#pragma once

#include "spice/spice_parser.hpp"
#include "spice/device_models.hpp"

#include <string>
#include <vector>
#include <map>
#include <functional>

namespace sf {

// ────────────────────────────────────────────────────────────────────────
// PWL stimulus definition
// ────────────────────────────────────────────────────────────────────────
struct PWLStimulus {
    std::vector<double> times;
    std::vector<double> values;

    double eval(double t) const {
        if (times.empty()) return 0.0;
        if (t <= times.front()) return values.front();
        if (t >= times.back()) return values.back();
        // Linear interpolation
        for (size_t i = 1; i < times.size(); ++i) {
            if (t <= times[i]) {
                double frac = (t - times[i-1]) / (times[i] - times[i-1]);
                return values[i-1] + frac * (values[i] - values[i-1]);
            }
        }
        return values.back();
    }
};

// ────────────────────────────────────────────────────────────────────────
// Transient result
// ────────────────────────────────────────────────────────────────────────
struct TransientResult {
    std::vector<double>                          time_points;
    std::map<std::string, std::vector<double>>   node_voltages;  // node_name → V(t)
};

// ────────────────────────────────────────────────────────────────────────
// SpiceEngine
// ────────────────────────────────────────────────────────────────────────
class SpiceEngine {
public:
    // Load a parsed circuit
    void load(const SpiceCircuit& circuit);

    // Register a MOSFET model
    void add_model(const std::string& name, const MosfetModel& model);

    // Register PWL stimulus for a voltage source
    void add_pwl(const std::string& source_name, const PWLStimulus& stim);

    // DC operating point — returns map of node_name → voltage
    std::map<std::string, double> dc_operating_point();

    // Transient analysis
    TransientResult transient(double tstep, double tstop);

    // ── Measurement utilities ──────────────────────────────────────────
    static double measure_delay(const std::vector<double>& time,
                                const std::vector<double>& waveform,
                                double threshold,
                                bool rising = true);

    static double measure_slew(const std::vector<double>& time,
                               const std::vector<double>& waveform,
                               double low_pct, double high_pct,
                               bool rising = true);

    static double measure_power(const std::vector<double>& v_wave,
                                const std::vector<double>& i_wave,
                                const std::vector<double>& time,
                                double period);

    // Configuration
    int    max_iterations    = 100;
    double convergence_tol   = 1e-9;
    bool   source_stepping   = true;

private:
    // ── Internal data structures ───────────────────────────────────────
    struct NodeInfo {
        int    index = -1;   // row/col in MNA matrix
        std::string name;
    };

    // Flattened device with resolved model pointers
    struct ResolvedDevice {
        SpiceDevice        dev;
        MosfetModel        mos_model;    // valid if MOSFET
        bool               has_model = false;
    };

    // Circuit data
    SpiceCircuit                            circuit_;
    std::map<std::string, MosfetModel>      models_;
    std::map<std::string, PWLStimulus>       pwl_stimuli_;

    // MNA bookkeeping
    std::map<std::string, int>              node_map_;       // node_name → matrix index
    int                                     num_nodes_ = 0;
    int                                     num_vsrc_  = 0;  // extra MNA rows for V sources
    int                                     mna_size_  = 0;

    std::vector<ResolvedDevice>             resolved_devices_;
    std::vector<std::string>                vsrc_names_;     // voltage source ordering

    // Build node map from circuit
    void build_node_map();

    // Get node index (0 = GND)
    int node_idx(const std::string& name) const;

    // Stamp MNA matrix for a given operating point
    void stamp_mna(std::vector<std::vector<double>>& G,
                   std::vector<double>& I,
                   const std::vector<double>& V,
                   double source_factor = 1.0,
                   double time = -1.0,
                   double dt = -1.0,
                   const std::vector<double>* V_prev = nullptr);

    // Dense LU solve (for moderate-size circuits)
    static bool lu_solve(std::vector<std::vector<double>>& A,
                         std::vector<double>& b,
                         std::vector<double>& x);
};

} // namespace sf
