// SiliconForge — Reliability Analyzer Implementation
#include "verify/reliability.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

namespace sf {

double ReliabilityAnalyzer::compute_nbti(double t_years, double temp_c,
                                          double vdd, double duty) const {
    // NBTI model: ΔVth ∝ (Vgs - Vth0)^γ × exp(-Ea/kT) × t^n
    // Simplified: power-law with Arrhenius temperature dependence
    double Ea = 0.1; // eV activation energy
    double kT = 8.617e-5 * (temp_c + 273.15); // eV
    double n = 0.25; // time exponent
    double gamma = 2.0;

    double vth0 = vdd * 0.3; // threshold ≈ 30% of VDD
    double stress = std::pow(std::abs(vdd - vth0), gamma);
    double temp_factor = std::exp(-Ea / kT);
    double time_factor = std::pow(t_years * 365 * 24 * 3600, n);
    double duty_factor = std::pow(duty, 0.5);

    return stress * temp_factor * time_factor * duty_factor * 10; // mV
}

double ReliabilityAnalyzer::compute_hci(double t_years, double temp_c,
                                         double vdd, double freq_ghz) const {
    // HCI: hot carrier injection, driven by switching activity
    double Ea = 0.05; // lower activation energy than NBTI
    double kT = 8.617e-5 * (temp_c + 273.15);
    double n = 0.5; // time exponent

    double cycles = freq_ghz * 1e9 * t_years * 365 * 24 * 3600;
    double stress = std::pow(vdd, 3.0); // strong voltage dependence
    double temp_factor = std::exp(-Ea / kT);

    return stress * temp_factor * std::pow(cycles, n) * 1e-12; // mV
}

double ReliabilityAnalyzer::compute_tddb(double t_years, double temp_c,
                                          double vdd, double area_um2) const {
    // TDDB: time-dependent dielectric breakdown
    // Failure rate in FIT (failures in 10^9 hours)
    double Ea = 0.7; // eV
    double kT = 8.617e-5 * (temp_c + 273.15);
    double field = vdd / 0.002; // electric field (V/m with ~2nm oxide)

    double base_rate = area_um2 * 0.001; // base FIT per um²
    double accel = std::exp(-Ea / kT) * std::exp(field * 1e-8);

    return base_rate * accel;
}

double ReliabilityAnalyzer::vth_to_delay_pct(double vth_shift, double vdd) const {
    // Simple α-power law: delay ∝ 1/(Vdd - Vth)
    double vth0 = vdd * 0.3 * 1000; // mV
    double vdd_mv = vdd * 1000;
    double old_drive = vdd_mv - vth0;
    double new_drive = vdd_mv - (vth0 + vth_shift);
    if (new_drive <= 0) return 100; // complete failure
    return ((old_drive / new_drive) - 1.0) * 100;
}

AgingResult ReliabilityAnalyzer::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    AgingResult r;

    double total_nbti = compute_nbti(cfg_.lifetime_years, cfg_.temperature_c,
                                      cfg_.voltage, cfg_.duty_cycle);
    double total_hci = compute_hci(cfg_.lifetime_years, cfg_.temperature_c,
                                    cfg_.voltage, 1.0); // assume 1GHz
    r.nbti_vth_shift_mv = total_nbti;
    r.hci_vth_shift_mv = total_hci;

    double total_vth_shift = total_nbti + total_hci;
    r.timing_degradation_pct = vth_to_delay_pct(total_vth_shift, cfg_.voltage);

    // Per-cell analysis
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;

        double cell_nbti = total_nbti;
        double cell_hci = total_hci;

        // PMOS-heavy gates (inverters, NAND) more susceptible to NBTI
        if (g.type == GateType::NOT || g.type == GateType::NAND)
            cell_nbti *= 1.3;
        // High fanout gates switch more → more HCI
        auto& net = nl_.net(g.output);
        if (net.fanout.size() > 4)
            cell_hci *= 1.0 + net.fanout.size() * 0.05;

        double shift = cell_nbti + cell_hci;
        double delay_inc = vth_to_delay_pct(shift, cfg_.voltage);
        bool at_risk = delay_inc > 15; // >15% degradation = at risk

        r.details.push_back({g.name, delay_inc, shift, at_risk});
        if (at_risk) r.cells_at_risk++;
    }

    // TDDB and EM
    double total_area = nl_.num_gates() * 1.0; // rough area estimate
    r.tddb_failure_rate = compute_tddb(cfg_.lifetime_years, cfg_.temperature_c,
                                        cfg_.voltage, total_area);
    r.em_failure_rate = nl_.num_gates() * 0.001; // simplified EM

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Aging @" + std::to_string(cfg_.lifetime_years) + "yr: " +
                "NBTI=" + std::to_string((int)r.nbti_vth_shift_mv) + "mV, " +
                "HCI=" + std::to_string((int)r.hci_vth_shift_mv) + "mV, " +
                "delay+" + std::to_string((int)r.timing_degradation_pct) + "%, " +
                std::to_string(r.cells_at_risk) + " at-risk cells";
    return r;
}

} // namespace sf
