#pragma once
// SiliconForge -- Technology Design Rules for Standard Cell Layout
// Encapsulates critical dimensions, spacing constraints, and process
// parameters for transistor-level cell generation.
//
// Rule values are in micrometers (um) and derived from published PDK
// documentation:
//   - SkyWater SKY130: skywater-pdk.readthedocs.io (130nm, 1.8V)
//   - ASAP7: Clark et al., "ASAP7: A 7-nm FinFET Predictive PDK", 2016

#include <string>

namespace sf {

struct TechRules {
    std::string process_name;

    // Gate / poly parameters
    double gate_length         = 0.15;   // poly width over active (drawn Lg)
    double min_poly_spacing    = 0.21;   // poly-to-poly spacing
    double poly_endcap         = 0.13;   // poly extension past diffusion edge

    // Diffusion (active) parameters
    double diff_width          = 0.42;   // minimum diffusion width (W direction)
    double diff_spacing        = 0.27;   // diffusion-to-diffusion spacing
    double diff_contact_enclosure = 0.06; // diffusion enclosure of contact

    // Contact / LICON parameters
    double contact_size        = 0.17;   // contact square side length
    double contact_spacing     = 0.17;   // contact-to-contact spacing

    // Well parameters
    double nwell_enclosure     = 0.18;   // NWELL enclosure of PMOS active

    // Implant enclosure
    double nsdm_enclosure      = 0.125;  // NSDM (n+ S/D implant) enclosure of active
    double psdm_enclosure      = 0.125;  // PSDM (p+ S/D implant) enclosure of active

    // Metal 1 parameters
    double m1_width            = 0.14;   // minimum M1 width
    double m1_spacing          = 0.14;   // minimum M1 spacing
    double m1_pitch            = 0.34;   // M1 pitch (width + spacing + ...

    // Local interconnect (LI1) parameters
    double li_width            = 0.17;   // minimum LI1 width
    double li_spacing          = 0.17;   // minimum LI1 spacing

    // Cell architecture
    double row_height          = 2.72;   // standard cell height (um)
    double site_width          = 0.46;   // placement site width (um)
    double power_rail_width    = 0.48;   // VDD/VSS rail width on M1
    double tap_width           = 0.26;   // well-tap diffusion width

    // Electrical
    double vdd_voltage         = 1.8;    // nominal supply voltage (V)

    // ----------------------------------------------------------------
    // Factory: SKY130 130nm open-source PDK
    // Values from SkyWater SKY130 DRM rev 0.1
    // ----------------------------------------------------------------
    static TechRules sky130_rules() {
        TechRules r;
        r.process_name          = "SKY130";
        r.gate_length           = 0.15;
        r.min_poly_spacing      = 0.21;
        r.poly_endcap           = 0.13;
        r.diff_width            = 0.42;
        r.diff_spacing          = 0.27;
        r.diff_contact_enclosure = 0.06;
        r.contact_size          = 0.17;
        r.contact_spacing       = 0.17;
        r.nwell_enclosure       = 0.18;
        r.nsdm_enclosure        = 0.125;
        r.psdm_enclosure        = 0.125;
        r.m1_width              = 0.14;
        r.m1_spacing            = 0.14;
        r.m1_pitch              = 0.34;
        r.li_width              = 0.17;
        r.li_spacing            = 0.17;
        r.row_height            = 2.72;
        r.site_width            = 0.46;
        r.power_rail_width      = 0.48;
        r.tap_width             = 0.26;
        r.vdd_voltage           = 1.8;
        return r;
    }

    // ----------------------------------------------------------------
    // Factory: ASAP7 predictive 7nm FinFET PDK
    // Values from Clark et al., IEEE TCAD 2016
    // ----------------------------------------------------------------
    static TechRules asap7_rules() {
        TechRules r;
        r.process_name          = "ASAP7";
        r.gate_length           = 0.007;  // 7nm drawn gate
        r.min_poly_spacing      = 0.027;  // contacted poly pitch ~ 54nm
        r.poly_endcap           = 0.010;
        r.diff_width            = 0.027;  // fin pitch constrained
        r.diff_spacing          = 0.027;
        r.diff_contact_enclosure = 0.005;
        r.contact_size          = 0.018;
        r.contact_spacing       = 0.018;
        r.nwell_enclosure       = 0.030;
        r.nsdm_enclosure        = 0.020;
        r.psdm_enclosure        = 0.020;
        r.m1_width              = 0.018;
        r.m1_spacing            = 0.018;
        r.m1_pitch              = 0.036;
        r.li_width              = 0.018;
        r.li_spacing            = 0.018;
        r.row_height            = 0.270;  // 7.5T cell height
        r.site_width            = 0.054;  // contacted poly pitch
        r.power_rail_width      = 0.048;
        r.tap_width             = 0.027;
        r.vdd_voltage           = 0.7;    // 7nm nominal Vdd
        return r;
    }
};

} // namespace sf
