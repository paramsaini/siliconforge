#pragma once
// SiliconForge — SystemVerilog Parser (IEEE 1800-2017)
// Parameterized module types, $readmemh/$readmemb execution,
// type inference for parameters and localparams.
// Reference: IEEE 1800-2017 ss6.20 (Parameters), ss21.4 ($readmemh/$readmemb)

#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace sf {

// ============================================================================
// SvParserParam — parameterized module support
// ============================================================================

struct SvParserParam {
    std::string name;
    std::string type;           // "integer", "real", "string", "bit", "logic", or width spec
    std::string default_value;  // expression or literal (unevaluated text)
    bool is_local = false;      // localparam vs parameter

    // Resolved value after elaboration
    int64_t resolved_int = 0;
    double resolved_real = 0.0;
    bool resolved = false;
};

// ============================================================================
// Parameterized module representation
// ============================================================================

struct SvParamPort {
    std::string name;
    std::string direction;  // "input", "output", "inout"
    int width = 1;
    std::string type_name;  // user-defined type or empty
};

struct SvParamModule {
    std::string name;
    std::vector<SvParserParam> parameters;
    std::vector<SvParamPort> ports;
    std::string body;  // raw body text for re-elaboration
    bool is_parameterized() const { return !parameters.empty(); }
};

// ============================================================================
// Memory initialization — $readmemh / $readmemb
// ============================================================================

struct SvMemoryArray {
    std::string name;
    int word_width = 8;       // bits per word
    int depth = 0;            // number of words
    std::vector<uint64_t> data;  // initialized contents

    bool initialized() const { return !data.empty(); }
};

// ============================================================================
// SvParser — SystemVerilog front-end with parameterized type support
// ============================================================================

class SvParser {
public:
    SvParser() = default;

    // Parse a SystemVerilog source string; populates modules and memories
    bool parse(const std::string& source);

    // Execute $readmemh — reads hex memory initialization file
    // file_format: one hex value per line, optional @address prefix
    // Returns number of words loaded
    int execute_readmemh(const std::string& filename, const std::string& mem_name,
                         int start_addr = 0, int end_addr = -1);

    // Execute $readmemb — reads binary memory initialization file
    int execute_readmemb(const std::string& filename, const std::string& mem_name,
                         int start_addr = 0, int end_addr = -1);

    // In-memory variants (for testing without filesystem)
    int execute_readmemh_data(const std::string& hex_data, const std::string& mem_name,
                              int start_addr = 0, int end_addr = -1);
    int execute_readmemb_data(const std::string& bin_data, const std::string& mem_name,
                              int start_addr = 0, int end_addr = -1);

    // Resolve parameter values for a module instantiation
    // param_overrides: parameter_name -> override_value_string
    bool resolve_parameters(const std::string& module_name,
                            const std::unordered_map<std::string, std::string>& param_overrides);

    // Evaluate a constant parameter expression (supports +, -, *, /, %, $clog2)
    int64_t eval_param_expr(const std::string& expr,
                            const std::unordered_map<std::string, int64_t>& ctx) const;

    // Accessors
    const std::vector<SvParamModule>& modules() const { return modules_; }
    const std::unordered_map<std::string, SvMemoryArray>& memories() const { return memories_; }

    SvParamModule* find_module(const std::string& name);
    SvMemoryArray* find_memory(const std::string& name);

private:
    std::vector<SvParamModule> modules_;
    std::unordered_map<std::string, SvMemoryArray> memories_;

    // Internal parse helpers
    void parse_module_decl(const std::string& source, size_t& pos);
    void parse_parameter_list(const std::string& source, size_t& pos,
                              std::vector<SvParserParam>& params);
    SvParserParam parse_single_parameter(const std::string& text, bool is_local);
    void parse_port_list(const std::string& source, size_t& pos,
                         std::vector<SvParamPort>& ports);
    void parse_memory_decl(const std::string& source, size_t& pos);
    void scan_readmem_calls(const std::string& source);

    // Utility
    static std::string trim(const std::string& s);
    static std::string extract_until(const std::string& src, size_t& pos, char delim);
    static bool is_ident_char(char c);

    // Internal readmem worker
    int readmem_worker(const std::string& content, const std::string& mem_name,
                       int start_addr, int end_addr, int base);
};

} // namespace sf
