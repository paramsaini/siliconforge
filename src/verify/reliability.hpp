#pragma once
// SiliconForge — Reliability / Aging Analyzer
// Models NBTI, HCI, TDDB, and EM degradation over chip lifetime.
// Reference: Rauch, "Review and Reexamination of Reliability Effects", IEEE TED 2007

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct AgingConfig {
    double lifetime_years = 10;
    double temperature_c = 85;
    double voltage = 1.0;
    double duty_cycle = 0.5;
};

struct AgingResult {
    double nbti_vth_shift_mv = 0;    // NBTI threshold shift
    double hci_vth_shift_mv = 0;     // HCI threshold shift
    double tddb_failure_rate = 0;     // TDDB failure rate (FIT)
    double em_failure_rate = 0;       // EM failure rate
    double timing_degradation_pct = 0;
    int cells_at_risk = 0;
    double time_ms = 0;

    struct CellAging {
        std::string name;
        double delay_increase_pct;
        double vth_shift_mv;
        bool at_risk;
    };
    std::vector<CellAging> details;
    std::string message;
};

class ReliabilityAnalyzer {
public:
    ReliabilityAnalyzer(const Netlist& nl) : nl_(nl) {}

    void set_config(const AgingConfig& cfg) { cfg_ = cfg; }

    AgingResult analyze();

private:
    const Netlist& nl_;
    AgingConfig cfg_;

    double compute_nbti(double t_years, double temp_c, double vdd, double duty) const;
    double compute_hci(double t_years, double temp_c, double vdd, double freq_ghz) const;
    double compute_tddb(double t_years, double temp_c, double vdd, double area_um2) const;
    double vth_to_delay_pct(double vth_shift, double vdd) const;
};

} // namespace sf
