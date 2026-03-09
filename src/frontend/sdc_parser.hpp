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
};

struct SdcConstraints {
    std::vector<SdcClock> clocks;
    std::vector<SdcInputDelay> input_delays;
    std::vector<SdcOutputDelay> output_delays;
    std::vector<SdcException> exceptions;
    std::unordered_map<std::string, double> max_fanout;
    std::unordered_map<std::string, double> max_transition;
    std::unordered_map<std::string, double> max_capacitance;

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
