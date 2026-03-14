#pragma once
// SiliconForge — SDC (Synopsys Design Constraints) Parser
// Parses timing constraints: create_clock, set_input_delay, set_output_delay,
// set_max_delay, set_false_path, set_multicycle_path, set_clock_groups.
// Reference: IEEE 1801, Synopsys SDC specification

#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

struct SdcClock {
    std::string name;
    std::string port;
    double period_ns = 0;
    double waveform_rise = 0;   // rising edge time
    double waveform_fall = 0;   // falling edge time
    double uncertainty = 0;     // jitter
    bool is_generated = false;  // true for generated clocks
    std::string source;         // source clock for generated clocks
    int divide_by = 1;          // clock divider ratio
    int multiply_by = 1;       // clock multiplier ratio
    double duty_cycle = 50.0;  // duty cycle percentage
    bool invert = false;        // inverted clock
    double phase_shift = 0;    // phase offset in ns
    std::vector<int> edges;     // edge list for generated clocks
};

struct SdcInputDelay {
    std::string port;
    std::string clock;
    double delay_ns = 0;
    bool is_max = true;         // max or min
};

struct SdcOutputDelay {
    std::string port;
    std::string clock;
    double delay_ns = 0;
    bool is_max = true;
};

struct SdcException {
    enum Type { FALSE_PATH, MULTICYCLE_PATH, MAX_DELAY, MIN_DELAY } type;
    std::string from;           // start point
    std::string to;             // end point
    double value = 0;           // multiplier for multicycle, ns for max/min
    int multiplier = 1;         // for multicycle paths
    std::string through;        // optional through point
    bool is_setup = true;       // setup or hold for multicycle
};

struct SdcClockGroup {
    std::string name;
    enum Relation { ASYNC, EXCLUSIVE, PHYSICALLY_EXCLUSIVE } relation = ASYNC;
    std::vector<std::vector<std::string>> groups; // each inner vector is a group of clock names
};

struct SdcCaseAnalysis {
    std::string pin;
    enum Value { ZERO, ONE, RISING, FALLING } value = ZERO;
};

struct SdcDisableTiming {
    std::string cell_instance;
    std::string from_pin;
    std::string to_pin;
};

struct SdcDrivingCell {
    std::string port;
    std::string lib_cell;
    std::string pin;
    double input_transition_rise = 0.0;
    double input_transition_fall = 0.0;
};

struct SdcLoad {
    std::string port;
    double capacitance = 0.0;  // pF
};

struct SdcGroupPath {
    std::string name;
    std::string from;
    std::string to;
    double weight = 1.0;
};

struct SdcWireLoad {
    std::string model_name;
    std::string library;
};

struct SdcConstraints {
    std::vector<SdcClock> clocks;
    std::vector<SdcInputDelay> input_delays;
    std::vector<SdcOutputDelay> output_delays;
    std::vector<SdcException> exceptions;
    std::vector<SdcClockGroup> clock_groups;
    std::unordered_map<std::string, double> max_fanout;
    std::unordered_map<std::string, double> max_transition;
    std::unordered_map<std::string, double> max_capacitance;
    std::vector<SdcCaseAnalysis> case_analyses;
    std::vector<SdcDisableTiming> disable_timings;
    std::vector<SdcDrivingCell> driving_cells;
    std::vector<SdcLoad> loads;
    std::vector<SdcGroupPath> group_paths;
    std::vector<SdcWireLoad> wire_loads;
    std::vector<std::string> propagated_clocks;
    std::vector<std::string> ideal_networks;

    const SdcClock* find_clock(const std::string& name) const;
    double get_clock_period(const std::string& name) const;
};

struct SdcParseResult {
    bool success = false;
    int num_constraints = 0;
    std::string error;
};

class SdcParser {
public:
    SdcParseResult parse_string(const std::string& src, SdcConstraints& sdc);
    SdcParseResult parse_file(const std::string& filename, SdcConstraints& sdc);

    // Export constraints back to SDC text
    static std::string to_sdc(const SdcConstraints& sdc);

private:
    struct Token { std::string value; int line = 0; };
    std::vector<Token> tokenize(const std::string& src);
    size_t parse_command(const std::vector<Token>& t, size_t pos,
                         SdcConstraints& sdc, SdcParseResult& r);
};

} // namespace sf
