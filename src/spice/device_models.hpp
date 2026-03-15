// Compact MOSFET Device Models — Level 1 (Shichman-Hodges) and
// simplified Level 54 (BSIM4) Ids evaluation with capacitance models.
// Parameter sets calibrated to SKY130-class process.
#pragma once

#include <cmath>
#include <string>
#include <map>
#include <algorithm>

namespace sf {

// ────────────────────────────────────────────────────────────────────────
// Capacitance result struct
// ────────────────────────────────────────────────────────────────────────
struct MosfetCaps {
    double Cgs = 0.0;
    double Cgd = 0.0;
    double Csb = 0.0;
    double Cdb = 0.0;
};

// ────────────────────────────────────────────────────────────────────────
// MosfetModel — BSIM4-core parameter set with evaluation methods
// ────────────────────────────────────────────────────────────────────────
struct MosfetModel {
    // Identity
    std::string name;
    bool        is_pmos   = false;
    int         level     = 1;

    // Threshold voltage parameters
    double VTH0     = 0.45;     // [V] zero-bias threshold
    double K1       = 0.5;      // [V^0.5] body-effect coefficient
    double K2       = -0.05;    // body-effect coefficient 2
    double VFB      = -1.0;     // [V] flat-band voltage

    // Mobility / transport
    double U0       = 300.0;    // [cm^2/V-s] low-field mobility
    double UA       = 2.25e-9;  // [m/V] first-order mobility degradation
    double UB       = 5.87e-19; // [m^2/V^2] second-order mobility degradation
    double VSAT     = 1.5e5;    // [m/s] saturation velocity

    // Subthreshold
    double ETA0     = 0.08;     // DIBL coefficient
    double DSUB     = 0.56;     // DIBL exponent
    double NFACTOR  = 1.5;      // subthreshold swing factor

    // Output resistance
    double PCLM     = 1.3;      // channel length modulation
    double PDIBLC1  = 0.39;     // DIBL correction 1

    // Oxide
    double TOXE     = 4.1e-9;   // [m] electrical oxide thickness
    double EPSROX   = 3.9;      // relative permittivity SiO2

    // Geometry correction
    double WINT     = 5e-9;     // [m] channel width offset
    double LINT     = 0.0;      // [m] channel length offset

    // Junction capacitance
    double CJ       = 1.0e-3;   // [F/m^2] zero-bias bottom junction cap
    double CJSW     = 5.0e-10;  // [F/m] zero-bias sidewall junction cap
    double MJ       = 0.5;      // bottom grading coefficient
    double MJSW     = 0.33;     // sidewall grading coefficient
    double PB       = 0.8;      // [V] built-in potential

    // Overlap capacitance
    double CGSO     = 2.5e-10;  // [F/m] gate-source overlap cap
    double CGDO     = 2.5e-10;  // [F/m] gate-drain overlap cap
    double CGBO     = 1.0e-12;  // [F/m] gate-bulk overlap cap

    // Default geometry (can be overridden per instance)
    double W        = 1.0e-6;   // [m] channel width
    double L        = 150e-9;   // [m] channel length

    // Physical constants
    static constexpr double EPS0    = 8.854e-12;   // [F/m]
    static constexpr double EPSSI   = 11.7 * EPS0;
    static constexpr double Q_ELEC  = 1.602e-19;   // [C]
    static constexpr double K_BOLTZ = 1.381e-23;   // [J/K]
    static constexpr double T_NOM   = 300.15;       // [K]
    static constexpr double VT_NOM  = K_BOLTZ * T_NOM / Q_ELEC; // ~25.9mV

    // ── Effective geometry ──────────────────────────────────────────────
    double Weff() const { return std::max(W - 2.0 * WINT, 1e-9); }
    double Leff() const { return std::max(L - 2.0 * LINT, 1e-9); }
    double Cox()  const { return EPSROX * EPS0 / TOXE; }

    // ═══════════════════════════════════════════════════════════════════
    // Level 1: Shichman-Hodges Ids
    // ═══════════════════════════════════════════════════════════════════
    double evaluate_ids_level1(double Vgs, double Vds, double Vbs) const {
        double sign = is_pmos ? -1.0 : 1.0;
        double vgs = sign * Vgs;
        double vds = sign * Vds;
        double vbs = sign * Vbs;

        // Threshold voltage with body effect
        double phi_s = 2.0 * VT_NOM * std::log(1e16 / 1.45e10); // ~0.72V typical
        double sqrt_term = std::sqrt(std::max(phi_s - vbs, 0.01));
        double sqrt_phi  = std::sqrt(phi_s);
        double Vt = std::abs(VTH0) + K1 * (sqrt_term - sqrt_phi);

        double Vov = vgs - Vt;
        if (Vov <= 0.0) {
            // Subthreshold — exponential leakage
            double n_sub = 1.0 + NFACTOR * 0.1;
            double I_sub = (U0 * 1e-4) * Cox() * (Weff() / Leff()) *
                           (n_sub * VT_NOM * VT_NOM) *
                           std::exp((Vov) / (n_sub * VT_NOM));
            return sign * std::min(I_sub, 1e-6); // clamp leakage
        }

        // KP = mu * Cox
        double KP = (U0 * 1e-4) * Cox();   // U0 in cm^2/Vs → m^2/Vs
        double beta = KP * (Weff() / Leff());

        // Channel length modulation — Level 1 lambda is a per-volt parameter,
        // typically 0.01 – 0.1 V⁻¹.  Scale inversely with channel length
        // relative to a reference (1um), clamped to a sane range.
        double lambda = std::min(PCLM * 0.01 * (1e-6 / Leff()), 0.5);

        double Ids;
        if (vds < Vov) {
            // Linear region
            Ids = beta * (Vov * vds - 0.5 * vds * vds) * (1.0 + lambda * vds);
        } else {
            // Saturation region
            Ids = 0.5 * beta * Vov * Vov * (1.0 + lambda * vds);
        }

        return sign * Ids;
    }

    // ═══════════════════════════════════════════════════════════════════
    // Level 54: Simplified BSIM4 Ids
    // ═══════════════════════════════════════════════════════════════════
    double evaluate_ids_bsim4(double Vgs, double Vds, double Vbs) const {
        double sign = is_pmos ? -1.0 : 1.0;
        double vgs = sign * Vgs;
        double vds = sign * Vds;
        double vbs = sign * Vbs;

        // Threshold voltage with DIBL and body effect
        double phi_s = 2.0 * VT_NOM * std::log(1e16 / 1.45e10);
        double sqrt_term = std::sqrt(std::max(phi_s - vbs, 0.01));
        double sqrt_phi  = std::sqrt(phi_s);
        double Vt = std::abs(VTH0)
                    + K1 * (sqrt_term - sqrt_phi)
                    + K2 * vbs
                    - ETA0 * vds;     // DIBL

        double Vov = vgs - Vt;

        // Subthreshold
        if (Vov <= 0.0) {
            double n_sub = 1.0 + NFACTOR * 0.1;
            double I_sub = (U0 * 1e-4) * Cox() * (Weff() / Leff()) *
                           (n_sub * VT_NOM * VT_NOM) *
                           std::exp(Vov / (n_sub * VT_NOM));
            return sign * std::min(I_sub, 1e-6);
        }

        // Effective mobility with vertical-field degradation
        double Eeff = (vgs + Vt) / (2.0 * TOXE);
        double mu_eff = (U0 * 1e-4) / (1.0 + UA * Eeff + UB * Eeff * Eeff);

        double beta = mu_eff * Cox() * (Weff() / Leff());

        // Velocity saturation
        double Vdsat = (2.0 * Vov * VSAT * Leff()) /
                       (2.0 * VSAT * Leff() + mu_eff * Vov);
        Vdsat = std::max(Vdsat, 1e-6);

        double vds_eff = std::min(vds, Vdsat);

        // Drain current with CLM
        double Ids;
        if (vds < Vdsat) {
            // Linear
            Ids = beta * (Vov * vds_eff - 0.5 * vds_eff * vds_eff);
        } else {
            // Saturation with velocity saturation
            Ids = 0.5 * beta * Vdsat * Vdsat;
            // CLM — Early voltage proportional to PCLM * Leff / sqrt(excess Vds)
            // Clamp to keep ratio physically bounded (< ~2x)
            double excess = std::max(vds - Vdsat, 1e-6);
            double Va = PCLM * Leff() * 1e6;  // scale to reasonable voltage (~0.2V for 150nm)
            Va = std::max(Va, 0.5);            // floor 0.5V
            Ids *= (1.0 + excess / Va);
        }

        // DIBL output conductance
        double dibl_factor = 1.0 + PDIBLC1 * (vds - Vdsat) /
                             std::max(Vov, VT_NOM);
        if (vds > Vdsat) Ids *= std::max(dibl_factor, 1.0);

        return sign * std::max(Ids, 0.0);
    }

    // ═══════════════════════════════════════════════════════════════════
    // Evaluate Ids based on model level
    // ═══════════════════════════════════════════════════════════════════
    double evaluate_ids(double Vgs, double Vds, double Vbs) const {
        if (level >= 49) return evaluate_ids_bsim4(Vgs, Vds, Vbs);
        return evaluate_ids_level1(Vgs, Vds, Vbs);
    }

    // ═══════════════════════════════════════════════════════════════════
    // Capacitance model (Meyer-like partitioning)
    // ═══════════════════════════════════════════════════════════════════
    MosfetCaps compute_caps(double Vgs, double Vds, double Vbs) const {
        MosfetCaps caps;

        double cox_total = Cox() * Weff() * Leff();

        // Overlap capacitances
        double C_ov_gs = CGSO * Weff();
        double C_ov_gd = CGDO * Weff();

        double sign = is_pmos ? -1.0 : 1.0;
        double vgs = sign * Vgs;
        double vds = sign * Vds;
        double vbs = sign * Vbs;

        double phi_s = 2.0 * VT_NOM * std::log(1e16 / 1.45e10);
        double sqrt_term = std::sqrt(std::max(phi_s - vbs, 0.01));
        double sqrt_phi  = std::sqrt(phi_s);
        double Vt = std::abs(VTH0) + K1 * (sqrt_term - sqrt_phi);
        double Vov = vgs - Vt;

        if (Vov <= 0.0) {
            // Cutoff — all intrinsic cap to bulk
            caps.Cgs = C_ov_gs;
            caps.Cgd = C_ov_gd;
        } else if (vds < Vov) {
            // Linear — gate cap split between source and drain
            double ratio = vds / std::max(2.0 * Vov, 1e-12);
            caps.Cgs = C_ov_gs + cox_total * (0.5 - ratio / 3.0);
            caps.Cgd = C_ov_gd + cox_total * (0.5 - (1.0 - ratio) / 3.0);
        } else {
            // Saturation — 2/3 to source, none to drain
            caps.Cgs = C_ov_gs + (2.0 / 3.0) * cox_total;
            caps.Cgd = C_ov_gd;
        }

        // Junction capacitances (zero-bias approximation for simplicity)
        double Ad = Weff() * 0.5e-6;  // approximate diffusion area
        double Pd = Weff() + 1.0e-6;  // approximate diffusion perimeter
        caps.Csb = CJ * Ad + CJSW * Pd;
        caps.Cdb = CJ * Ad + CJSW * Pd;

        return caps;
    }

    // ═══════════════════════════════════════════════════════════════════
    // Transconductance (gm) for Jacobian — numerical derivative
    // ═══════════════════════════════════════════════════════════════════
    double gm(double Vgs, double Vds, double Vbs) const {
        const double dv = 1e-6;
        return (evaluate_ids(Vgs + dv, Vds, Vbs) -
                evaluate_ids(Vgs - dv, Vds, Vbs)) / (2.0 * dv);
    }

    // Output conductance (gds)
    double gds(double Vgs, double Vds, double Vbs) const {
        const double dv = 1e-6;
        return (evaluate_ids(Vgs, Vds + dv, Vbs) -
                evaluate_ids(Vgs, Vds - dv, Vbs)) / (2.0 * dv);
    }

    // Body transconductance (gmb)
    double gmb(double Vgs, double Vds, double Vbs) const {
        const double dv = 1e-6;
        return (evaluate_ids(Vgs, Vds, Vbs + dv) -
                evaluate_ids(Vgs, Vds, Vbs - dv)) / (2.0 * dv);
    }

    // ═══════════════════════════════════════════════════════════════════
    // Default parameter sets (SKY130-class)
    // ═══════════════════════════════════════════════════════════════════
    static MosfetModel default_nmos() {
        MosfetModel m;
        m.name    = "nmos_3p3";
        m.is_pmos = false;
        m.level   = 1;
        m.VTH0    = 0.45;
        m.K1      = 0.5;
        m.K2      = -0.05;
        m.U0      = 300.0;
        m.TOXE    = 4.1e-9;
        m.VSAT    = 1.5e5;
        m.CGSO    = 2.5e-10;
        m.CGDO    = 2.5e-10;
        m.CGBO    = 1.0e-12;
        m.CJ      = 1.0e-3;
        m.CJSW    = 5.0e-10;
        m.W       = 1.0e-6;
        m.L       = 150e-9;
        return m;
    }

    static MosfetModel default_pmos() {
        MosfetModel m;
        m.name    = "pmos_3p3";
        m.is_pmos = true;
        m.level   = 1;
        m.VTH0    = -0.45;
        m.K1      = 0.5;
        m.K2      = -0.05;
        m.U0      = 100.0;
        m.TOXE    = 4.1e-9;
        m.VSAT    = 1.5e5;
        m.CGSO    = 2.5e-10;
        m.CGDO    = 2.5e-10;
        m.CGBO    = 1.0e-12;
        m.CJ      = 1.0e-3;
        m.CJSW    = 5.0e-10;
        m.W       = 2.0e-6;   // PMOS typically wider
        m.L       = 150e-9;
        return m;
    }
};

} // namespace sf
