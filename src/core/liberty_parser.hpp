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
};

struct LibertyCell {
    std::string name;
    double area = 0.0;
    double leakage_power = 0.0;
    std::vector<LibertyPin> pins;
    std::vector<LibertyTiming> timings;

    // Quick lookup
    const LibertyPin* find_pin(const std::string& name) const;
    std::string output_function() const; // Returns the Boolean function of the output pin
    int num_inputs() const;
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
    size_t parse_timing(const std::vector<Token>& tokens, size_t pos, LibertyCell& cell);
    size_t skip_group(const std::vector<Token>& tokens, size_t pos);
};

} // namespace sf
