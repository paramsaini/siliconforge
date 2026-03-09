#pragma once
// SiliconForge — DEF/LEF Parser
// Parses Design Exchange Format files for physical design import.
// Reference: LEF/DEF Language Reference, Cadence

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

class DefParser {
public:
    // Parse DEF from string (for embedded/testing)
    bool parse_string(const std::string& content, PhysicalDesign& pd);

    // Parse DEF file
    bool parse_file(const std::string& filename, PhysicalDesign& pd);

    // Export to DEF string
    static std::string export_def(const PhysicalDesign& pd);

private:
    std::vector<std::string> tokenize(const std::string& content);
    bool parse_tokens(const std::vector<std::string>& tokens, PhysicalDesign& pd);
};

} // namespace sf
