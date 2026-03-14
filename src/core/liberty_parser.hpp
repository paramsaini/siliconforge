#pragma once
// SiliconForge — Liberty (.lib) Parser
// Reads cell timing and power models from Liberty format files.
// Reference: Liberty User Guide, Synopsys

#include <string>
#include <vector>
#include <unordered_map>
#include <optional>

namespace sf {

struct LibertyPin {
    std::string name;
    std::string direction; // "input", "output", "inout"
    std::string function;  // Boolean function (e.g., "A & B", "!A")
    double capacitance = 0.0;
    double max_transition = 0.0;
};

// CCS (Composite Current Source) model — IEEE Liberty extension
// Models output current waveform as function of input slew and output load.
// More accurate than NLDM for sub-28nm nodes.
struct CcsTable {
    std::vector<double> index_1;  // input transition breakpoints
    std::vector<double> index_2;  // output capacitance breakpoints
    std::vector<double> index_3;  // time breakpoints for current waveform
    // 3D table: values[slew_idx][load_idx] = vector of current samples at index_3 times
    std::vector<std::vector<std::vector<double>>> values;
    bool valid() const { return !index_1.empty() && !index_2.empty() && !index_3.empty(); }
    // Interpolate current at given slew, load, time
    double interpolate(double slew, double load, double time) const;
    // Compute delay by integrating current waveform against load capacitance
    double compute_delay(double slew, double load, double threshold_pct = 0.5) const;
    // Compute output slew from current waveform (20%-80% or 10%-90%)
    double compute_slew(double slew, double load, double low_pct = 0.2, double high_pct = 0.8) const;
};

// ECSM (Effective Current Source Model)
// Models driver as voltage-dependent current source with Miller effect.
struct EcsmTable {
    std::vector<double> index_1;  // input transition breakpoints
    std::vector<double> index_2;  // output capacitance breakpoints
    std::vector<double> index_3;  // output voltage breakpoints (0 to VDD)
    // 3D table: values[slew_idx][load_idx] = vector of currents at each output voltage
    std::vector<std::vector<std::vector<double>>> values;
    bool valid() const { return !index_1.empty() && !index_2.empty() && !index_3.empty(); }
    double interpolate(double slew, double load, double voltage) const;
    double compute_delay(double slew, double load, double vdd = 1.0, double threshold_pct = 0.5) const;
};

struct LibertyTiming {
    std::string related_pin;
    std::string timing_type; // "combinational", "rising_edge", etc.
    double cell_rise = 0.0;
    double cell_fall = 0.0;
    double rise_transition = 0.0;
    double fall_transition = 0.0;

    // NLDM 2D tables (indexed by input_slew × output_load)
    struct NldmTable {
        std::vector<double> index_1; // input transition (slew) breakpoints
        std::vector<double> index_2; // output capacitance (load) breakpoints
        std::vector<std::vector<double>> values; // 2D table [slew][load]
        bool valid() const { return !index_1.empty() && !index_2.empty() && !values.empty(); }
        double interpolate(double slew, double load) const;
    };
    NldmTable nldm_rise;     // cell_rise table
    NldmTable nldm_fall;     // cell_fall table
    NldmTable nldm_rise_tr;  // rise_transition table
    NldmTable nldm_fall_tr;  // fall_transition table

    // CCS current source tables (sub-28nm signoff accuracy)
    CcsTable ccs_rise;
    CcsTable ccs_fall;

    // ECSM voltage-dependent current tables
    EcsmTable ecsm_rise;
    EcsmTable ecsm_fall;

    // Receiver capacitance (CCS)
    NldmTable receiver_cap1_rise;  // C1 (near-end)
    NldmTable receiver_cap1_fall;
    NldmTable receiver_cap2_rise;  // C2 (far-end, for π-model)
    NldmTable receiver_cap2_fall;
};

struct LibertyPower {
    std::string related_pin;
    std::string when; // conditional power (input vector)
    double rise_power = 0.0;
    double fall_power = 0.0;

    // NLDM-style 2D power tables
    LibertyTiming::NldmTable power_rise_table;
    LibertyTiming::NldmTable power_fall_table;
};

struct LibertyLeakage {
    std::string when; // input vector condition (e.g., "!A & B")
    double value = 0.0;
};

struct LibertyCell {
    std::string name;
    double area = 0.0;
    double leakage_power = 0.0;
    std::vector<LibertyPin> pins;
    std::vector<LibertyTiming> timings;
    std::vector<LibertyPower> internal_powers;
    std::vector<LibertyLeakage> leakage_powers;

    // Quick lookup
    const LibertyPin* find_pin(const std::string& name) const;
    std::string output_function() const; // Returns the Boolean function of the output pin
    int num_inputs() const;

    // Total internal power at given slew/load (average rise+fall)
    double internal_power_at(double input_slew, double output_load) const;
};

class LibertyLibrary {
public:
    std::string name;
    std::string technology;
    double nom_voltage = 1.8;
    double nom_temperature = 25.0;
    std::string time_unit = "1ns";
    std::string cap_unit = "1pF";

    std::vector<LibertyCell> cells;

    // lu_table_template storage: template_name → {index_1, index_2}
    struct TableTemplate {
        std::vector<double> index_1;
        std::vector<double> index_2;
    };
    std::unordered_map<std::string, TableTemplate> table_templates;

    // Parse a Liberty file
    bool parse(const std::string& filename);

    // Parse from string (for embedded/testing)
    bool parse_string(const std::string& content);

    // Lookup
    const LibertyCell* find_cell(const std::string& name) const;
    std::vector<const LibertyCell*> cells_by_function(const std::string& func) const;

    void print_stats() const;

private:
    // Simple recursive descent parser for Liberty syntax
    struct Token {
        enum Type { IDENT, STRING, NUMBER, LBRACE, RBRACE, LPAREN, RPAREN,
                    COLON, SEMI, COMMA, END };
        Type type;
        std::string value;
    };

    std::vector<Token> tokenize(const std::string& content);
    bool parse_tokens(const std::vector<Token>& tokens);
    size_t parse_group(const std::vector<Token>& tokens, size_t pos,
                       const std::string& parent);
    size_t parse_cell(const std::vector<Token>& tokens, size_t pos);
    size_t parse_pin(const std::vector<Token>& tokens, size_t pos, LibertyCell& cell);
    size_t parse_timing(const std::vector<Token>& tokens, size_t pos, LibertyCell& cell,
                        const std::string& pin_name);
    size_t parse_internal_power(const std::vector<Token>& tokens, size_t pos,
                                LibertyCell& cell);
    size_t parse_nldm_table(const std::vector<Token>& tokens, size_t pos,
                            LibertyTiming::NldmTable& table);
    size_t parse_ccs_table(const std::vector<Token>& tokens, size_t pos, CcsTable& table);
    size_t parse_ecsm_table(const std::vector<Token>& tokens, size_t pos, EcsmTable& table);
    size_t parse_lu_table_template(const std::vector<Token>& tokens, size_t pos);
    size_t skip_group(const std::vector<Token>& tokens, size_t pos);

    // Helper: parse comma-separated numbers from a string like "0.01, 0.02, 0.05"
    static std::vector<double> parse_number_list(const std::string& s);
};

} // namespace sf
