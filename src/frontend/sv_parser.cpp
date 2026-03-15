// SiliconForge — SystemVerilog Parser Implementation
// Parameterized module parsing, $readmemh/$readmemb execution,
// parameter resolution with constant expression evaluation.
// Reference: IEEE 1800-2017 ss6.20, ss21.4

#include "frontend/sv_parser.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace sf {

// ============================================================================
// Utility helpers
// ============================================================================

std::string SvParser::trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

bool SvParser::is_ident_char(char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
}

std::string SvParser::extract_until(const std::string& src, size_t& pos, char delim) {
    std::string result;
    while (pos < src.size() && src[pos] != delim) {
        result += src[pos++];
    }
    if (pos < src.size()) pos++; // skip delimiter
    return result;
}

// ============================================================================
// Parameter expression evaluator
// Supports: integer literals, identifiers, +, -, *, /, %, parentheses, $clog2
// ============================================================================

namespace {

struct ParamExprParser {
    const std::string& src;
    const std::unordered_map<std::string, int64_t>& ctx;
    size_t pos = 0;

    ParamExprParser(const std::string& s,
                    const std::unordered_map<std::string, int64_t>& c)
        : src(s), ctx(c) {}

    void skip_ws() {
        while (pos < src.size() && std::isspace(static_cast<unsigned char>(src[pos])))
            ++pos;
    }

    int64_t parse() { return parse_additive(); }

    int64_t parse_additive() {
        int64_t left = parse_multiplicative();
        while (true) {
            skip_ws();
            if (pos >= src.size()) break;
            char op = src[pos];
            if (op != '+' && op != '-') break;
            ++pos;
            int64_t right = parse_multiplicative();
            left = (op == '+') ? left + right : left - right;
        }
        return left;
    }

    int64_t parse_multiplicative() {
        int64_t left = parse_unary();
        while (true) {
            skip_ws();
            if (pos >= src.size()) break;
            char op = src[pos];
            if (op != '*' && op != '/' && op != '%') break;
            ++pos;
            int64_t right = parse_unary();
            if (op == '*') left *= right;
            else if (op == '/' && right != 0) left /= right;
            else if (op == '%' && right != 0) left %= right;
        }
        return left;
    }

    int64_t parse_unary() {
        skip_ws();
        if (pos < src.size() && src[pos] == '-') {
            ++pos;
            return -parse_primary();
        }
        if (pos < src.size() && src[pos] == '~') {
            ++pos;
            return ~parse_primary();
        }
        return parse_primary();
    }

    int64_t parse_primary() {
        skip_ws();
        if (pos >= src.size()) return 0;

        // Parenthesized expression
        if (src[pos] == '(') {
            ++pos;
            int64_t val = parse_additive();
            skip_ws();
            if (pos < src.size() && src[pos] == ')') ++pos;
            return val;
        }

        // Verilog-style sized literals: N'bXXXX, N'hXXXX, N'dXXXX
        // Also handle unsized 'h, 'b, 'd
        if (pos < src.size() && src[pos] == '\'') {
            ++pos;
            skip_ws();
            int base = 10;
            if (pos < src.size()) {
                char bc = std::tolower(static_cast<unsigned char>(src[pos]));
                if (bc == 'h') { base = 16; ++pos; }
                else if (bc == 'b') { base = 2; ++pos; }
                else if (bc == 'd') { base = 10; ++pos; }
                else if (bc == 'o') { base = 8; ++pos; }
            }
            skip_ws();
            std::string digits;
            while (pos < src.size() &&
                   (std::isxdigit(static_cast<unsigned char>(src[pos])) || src[pos] == '_')) {
                if (src[pos] != '_') digits += src[pos];
                ++pos;
            }
            if (digits.empty()) return 0;
            return static_cast<int64_t>(std::stoull(digits, nullptr, base));
        }

        // Number (decimal, or sized literal like 8'hFF)
        if (std::isdigit(static_cast<unsigned char>(src[pos]))) {
            std::string num;
            while (pos < src.size() && std::isdigit(static_cast<unsigned char>(src[pos])))
                num += src[pos++];

            // Check for sized literal: digits followed by '
            skip_ws();
            if (pos < src.size() && src[pos] == '\'') {
                ++pos;
                int base = 10;
                if (pos < src.size()) {
                    char bc = std::tolower(static_cast<unsigned char>(src[pos]));
                    if (bc == 'h') { base = 16; ++pos; }
                    else if (bc == 'b') { base = 2; ++pos; }
                    else if (bc == 'd') { base = 10; ++pos; }
                    else if (bc == 'o') { base = 8; ++pos; }
                }
                skip_ws();
                std::string digits;
                while (pos < src.size() &&
                       (std::isxdigit(static_cast<unsigned char>(src[pos])) || src[pos] == '_')) {
                    if (src[pos] != '_') digits += src[pos];
                    ++pos;
                }
                if (digits.empty()) return 0;
                return static_cast<int64_t>(std::stoull(digits, nullptr, base));
            }

            return std::stoll(num);
        }

        // $clog2 system function
        if (pos + 5 < src.size() && src.substr(pos, 6) == "$clog2") {
            pos += 6;
            skip_ws();
            if (pos < src.size() && src[pos] == '(') {
                ++pos;
                int64_t val = parse_additive();
                skip_ws();
                if (pos < src.size() && src[pos] == ')') ++pos;
                if (val <= 1) return 1;
                return static_cast<int64_t>(std::ceil(std::log2(static_cast<double>(val))));
            }
            return 0;
        }

        // Identifier (parameter reference)
        if (std::isalpha(static_cast<unsigned char>(src[pos])) || src[pos] == '_') {
            std::string id;
            while (pos < src.size() &&
                   (std::isalnum(static_cast<unsigned char>(src[pos])) || src[pos] == '_'))
                id += src[pos++];
            auto it = ctx.find(id);
            return (it != ctx.end()) ? it->second : 0;
        }
        return 0;
    }
};

} // anonymous namespace

int64_t SvParser::eval_param_expr(const std::string& expr,
                                   const std::unordered_map<std::string, int64_t>& ctx) const {
    std::string trimmed = trim(expr);
    if (trimmed.empty()) return 0;
    ParamExprParser ep(trimmed, ctx);
    return ep.parse();
}

// ============================================================================
// Single parameter parsing
// Handles: parameter [type] NAME = DEFAULT
//          localparam [type] NAME = DEFAULT
// ============================================================================

SvParserParam SvParser::parse_single_parameter(const std::string& text, bool is_local) {
    SvParserParam p;
    p.is_local = is_local;

    std::string t = trim(text);
    // Remove leading "parameter" or "localparam" keyword if present
    if (t.substr(0, 10) == "localparam") {
        t = trim(t.substr(10));
        p.is_local = true;
    } else if (t.substr(0, 9) == "parameter") {
        t = trim(t.substr(9));
    }

    // Check for type keyword: integer, real, string, bit, logic, reg, signed
    // Also handle width specifiers like [7:0]
    if (t.substr(0, 7) == "integer") {
        p.type = "integer"; t = trim(t.substr(7));
    } else if (t.substr(0, 4) == "real") {
        p.type = "real"; t = trim(t.substr(4));
    } else if (t.substr(0, 6) == "string") {
        p.type = "string"; t = trim(t.substr(6));
    } else if (t.substr(0, 3) == "bit") {
        p.type = "bit"; t = trim(t.substr(3));
    } else if (t.substr(0, 5) == "logic") {
        p.type = "logic"; t = trim(t.substr(5));
    } else if (t.substr(0, 6) == "signed") {
        p.type = "signed"; t = trim(t.substr(6));
    } else if (!t.empty() && t[0] == '[') {
        // Width specifier: [MSB:LSB]
        size_t close = t.find(']');
        if (close != std::string::npos) {
            p.type = t.substr(0, close + 1);
            t = trim(t.substr(close + 1));
        }
    }

    // If no explicit type, infer "integer" (IEEE 1800-2017 default)
    if (p.type.empty()) p.type = "integer";

    // Parse NAME = DEFAULT
    auto eq = t.find('=');
    if (eq != std::string::npos) {
        p.name = trim(t.substr(0, eq));
        p.default_value = trim(t.substr(eq + 1));
    } else {
        p.name = trim(t);
        p.default_value = "0";
    }

    return p;
}

// ============================================================================
// Parse parameter list: #( parameter A = 1, parameter B = 2 )
// ============================================================================

void SvParser::parse_parameter_list(const std::string& source, size_t& pos,
                                     std::vector<SvParserParam>& params) {
    // Expect '#' '('
    while (pos < source.size() && source[pos] != '(') pos++;
    if (pos >= source.size()) return;
    pos++; // skip '('

    int depth = 1;
    std::string current;
    while (pos < source.size() && depth > 0) {
        char c = source[pos];
        if (c == '(') { depth++; current += c; }
        else if (c == ')') {
            depth--;
            if (depth == 0) {
                // Process last parameter
                std::string t = trim(current);
                if (!t.empty()) {
                    bool local = (t.find("localparam") != std::string::npos);
                    params.push_back(parse_single_parameter(t, local));
                }
                break;
            }
            current += c;
        } else if (c == ',' && depth == 1) {
            std::string t = trim(current);
            if (!t.empty()) {
                bool local = (t.find("localparam") != std::string::npos);
                params.push_back(parse_single_parameter(t, local));
            }
            current.clear();
        } else {
            current += c;
        }
        pos++;
    }
    if (pos < source.size()) pos++; // skip closing ')'
}

// ============================================================================
// Parse port list: ( input [7:0] data, output valid, ... )
// ============================================================================

void SvParser::parse_port_list(const std::string& source, size_t& pos,
                                std::vector<SvParamPort>& ports) {
    while (pos < source.size() && source[pos] != '(') pos++;
    if (pos >= source.size()) return;
    pos++; // skip '('

    int depth = 1;
    std::string current;
    while (pos < source.size() && depth > 0) {
        char c = source[pos];
        if (c == '(') { depth++; current += c; }
        else if (c == ')') {
            depth--;
            if (depth == 0) {
                std::string t = trim(current);
                if (!t.empty()) {
                    SvParamPort port;
                    // Parse direction
                    if (t.substr(0, 5) == "input") { port.direction = "input"; t = trim(t.substr(5)); }
                    else if (t.substr(0, 6) == "output") { port.direction = "output"; t = trim(t.substr(6)); }
                    else if (t.substr(0, 5) == "inout") { port.direction = "inout"; t = trim(t.substr(5)); }
                    else { port.direction = "input"; }

                    // Parse optional width [MSB:LSB]
                    if (!t.empty() && t[0] == '[') {
                        size_t close = t.find(']');
                        if (close != std::string::npos) {
                            std::string range = t.substr(1, close - 1);
                            auto colon = range.find(':');
                            if (colon != std::string::npos) {
                                try {
                                    int msb = std::stoi(trim(range.substr(0, colon)));
                                    int lsb = std::stoi(trim(range.substr(colon + 1)));
                                    port.width = std::abs(msb - lsb) + 1;
                                } catch (...) { /* parameterized width — defer to elaboration */ }
                            }
                            t = trim(t.substr(close + 1));
                        }
                    }
                    port.name = trim(t);
                    if (!port.name.empty()) ports.push_back(port);
                }
                break;
            }
            current += c;
        } else if (c == ',' && depth == 1) {
            std::string t = trim(current);
            if (!t.empty()) {
                SvParamPort port;
                if (t.substr(0, 5) == "input") { port.direction = "input"; t = trim(t.substr(5)); }
                else if (t.substr(0, 6) == "output") { port.direction = "output"; t = trim(t.substr(6)); }
                else if (t.substr(0, 5) == "inout") { port.direction = "inout"; t = trim(t.substr(5)); }
                else { port.direction = "input"; }

                if (!t.empty() && t[0] == '[') {
                    size_t close = t.find(']');
                    if (close != std::string::npos) {
                        std::string range = t.substr(1, close - 1);
                        auto colon = range.find(':');
                        if (colon != std::string::npos) {
                            try {
                                int msb = std::stoi(trim(range.substr(0, colon)));
                                int lsb = std::stoi(trim(range.substr(colon + 1)));
                                port.width = std::abs(msb - lsb) + 1;
                            } catch (...) { /* parameterized width */ }
                        }
                        t = trim(t.substr(close + 1));
                    }
                }
                port.name = trim(t);
                if (!port.name.empty()) ports.push_back(port);
            }
            current.clear();
        } else {
            current += c;
        }
        pos++;
    }
    if (pos < source.size()) pos++;
}

// ============================================================================
// Parse memory declarations: reg [WIDTH-1:0] mem_name [0:DEPTH-1];
// ============================================================================

void SvParser::parse_memory_decl(const std::string& source, size_t& pos) {
    // Already past "reg" keyword
    std::string t = trim(source.substr(pos));

    // Parse word width [MSB:LSB]
    int word_width = 8;
    if (!t.empty() && t[0] == '[') {
        size_t close = t.find(']');
        if (close != std::string::npos) {
            std::string range = t.substr(1, close - 1);
            auto colon = range.find(':');
            if (colon != std::string::npos) {
                try {
                    int msb = std::stoi(trim(range.substr(0, colon)));
                    int lsb = std::stoi(trim(range.substr(colon + 1)));
                    word_width = std::abs(msb - lsb) + 1;
                } catch (...) {}
            }
            t = trim(t.substr(close + 1));
        }
    }

    // Parse name
    std::string name;
    size_t ti = 0;
    while (ti < t.size() && is_ident_char(t[ti])) name += t[ti++];
    t = trim(t.substr(ti));

    // Parse depth [START:END]
    int depth = 0;
    if (!t.empty() && t[0] == '[') {
        size_t close = t.find(']');
        if (close != std::string::npos) {
            std::string range = t.substr(1, close - 1);
            auto colon = range.find(':');
            if (colon != std::string::npos) {
                try {
                    int lo = std::stoi(trim(range.substr(0, colon)));
                    int hi = std::stoi(trim(range.substr(colon + 1)));
                    depth = std::abs(hi - lo) + 1;
                } catch (...) {}
            }
        }
    }

    if (!name.empty() && depth > 0) {
        SvMemoryArray mem;
        mem.name = name;
        mem.word_width = word_width;
        mem.depth = depth;
        memories_[name] = std::move(mem);
    }
}

// ============================================================================
// Scan for $readmemh / $readmemb calls in source
// ============================================================================

void SvParser::scan_readmem_calls(const std::string& source) {
    // Scan for $readmemh("file", mem) and $readmemb("file", mem)
    for (size_t i = 0; i < source.size(); ++i) {
        bool is_hex = false;
        if (i + 9 < source.size() && source.substr(i, 9) == "$readmemh") {
            is_hex = true;
        } else if (i + 9 < source.size() && source.substr(i, 9) == "$readmemb") {
            is_hex = false;
        } else {
            continue;
        }

        size_t p = i + 9;
        while (p < source.size() && std::isspace(static_cast<unsigned char>(source[p]))) p++;
        if (p >= source.size() || source[p] != '(') continue;
        p++;

        // Extract filename (quoted string)
        while (p < source.size() && std::isspace(static_cast<unsigned char>(source[p]))) p++;
        if (p >= source.size() || source[p] != '"') continue;
        p++;
        std::string filename;
        while (p < source.size() && source[p] != '"') filename += source[p++];
        if (p < source.size()) p++; // skip closing quote

        // Skip comma
        while (p < source.size() && (source[p] == ',' || std::isspace(static_cast<unsigned char>(source[p])))) p++;

        // Extract memory name
        std::string mem_name;
        while (p < source.size() && is_ident_char(source[p])) mem_name += source[p++];

        // Optional start/end addresses
        int start_addr = 0, end_addr = -1;
        while (p < source.size() && std::isspace(static_cast<unsigned char>(source[p]))) p++;
        if (p < source.size() && source[p] == ',') {
            p++;
            while (p < source.size() && std::isspace(static_cast<unsigned char>(source[p]))) p++;
            std::string addr_str;
            while (p < source.size() && std::isdigit(static_cast<unsigned char>(source[p])))
                addr_str += source[p++];
            if (!addr_str.empty()) start_addr = std::stoi(addr_str);

            while (p < source.size() && std::isspace(static_cast<unsigned char>(source[p]))) p++;
            if (p < source.size() && source[p] == ',') {
                p++;
                while (p < source.size() && std::isspace(static_cast<unsigned char>(source[p]))) p++;
                addr_str.clear();
                while (p < source.size() && std::isdigit(static_cast<unsigned char>(source[p])))
                    addr_str += source[p++];
                if (!addr_str.empty()) end_addr = std::stoi(addr_str);
            }
        }

        // If the memory exists, attempt to load from file
        if (!mem_name.empty() && memories_.count(mem_name)) {
            if (is_hex)
                execute_readmemh(filename, mem_name, start_addr, end_addr);
            else
                execute_readmemb(filename, mem_name, start_addr, end_addr);
        }

        i = p; // advance past the call
    }
}

// ============================================================================
// Module declaration parser
// ============================================================================

void SvParser::parse_module_decl(const std::string& source, size_t& pos) {
    SvParamModule mod;

    // Skip whitespace after "module" keyword
    while (pos < source.size() && std::isspace(static_cast<unsigned char>(source[pos]))) pos++;

    // Read module name
    while (pos < source.size() && is_ident_char(source[pos]))
        mod.name += source[pos++];
    while (pos < source.size() && std::isspace(static_cast<unsigned char>(source[pos]))) pos++;

    // Check for parameter list: #(...)
    if (pos < source.size() && source[pos] == '#') {
        pos++; // skip '#'
        parse_parameter_list(source, pos, mod.parameters);
        while (pos < source.size() && std::isspace(static_cast<unsigned char>(source[pos]))) pos++;
    }

    // Parse port list: (...)
    if (pos < source.size() && source[pos] == '(') {
        parse_port_list(source, pos, mod.ports);
    }

    // Skip to semicolon
    while (pos < source.size() && source[pos] != ';') pos++;
    if (pos < source.size()) pos++;

    // Extract body until "endmodule"
    std::string body;
    while (pos < source.size()) {
        if (pos + 9 <= source.size() && source.substr(pos, 9) == "endmodule") {
            pos += 9;
            break;
        }
        body += source[pos++];
    }
    mod.body = trim(body);

    // Scan body for memory declarations (reg [...] name [...])
    {
        size_t bp = 0;
        while (bp < body.size()) {
            // Look for "reg" keyword followed by brackets
            if (bp + 3 < body.size() && body.substr(bp, 3) == "reg" &&
                !is_ident_char(body[bp + 3])) {
                size_t start = bp + 3;
                // Check if this is a memory declaration (has two bracket pairs)
                std::string rest = body.substr(start);
                size_t first_bracket = rest.find('[');
                if (first_bracket != std::string::npos) {
                    size_t after_first = rest.find(']', first_bracket);
                    if (after_first != std::string::npos) {
                        // Skip whitespace and identifier, look for second bracket
                        size_t p2 = after_first + 1;
                        while (p2 < rest.size() && std::isspace(static_cast<unsigned char>(rest[p2]))) p2++;
                        // Skip identifier
                        while (p2 < rest.size() && is_ident_char(rest[p2])) p2++;
                        while (p2 < rest.size() && std::isspace(static_cast<unsigned char>(rest[p2]))) p2++;
                        if (p2 < rest.size() && rest[p2] == '[') {
                            // This is a memory declaration
                            parse_memory_decl(body, start);
                        }
                    }
                }
            }

            // Also look for inline parameter / localparam declarations inside body
            if (bp + 10 < body.size() && body.substr(bp, 10) == "localparam" &&
                (bp == 0 || !is_ident_char(body[bp - 1]))) {
                size_t start = bp;
                // Find semicolon
                size_t semi = body.find(';', bp);
                if (semi != std::string::npos) {
                    std::string decl = body.substr(start, semi - start);
                    mod.parameters.push_back(parse_single_parameter(decl, true));
                    bp = semi + 1;
                    continue;
                }
            }

            bp++;
        }
    }

    modules_.push_back(std::move(mod));
}

// ============================================================================
// Top-level parse
// ============================================================================

bool SvParser::parse(const std::string& source) {
    modules_.clear();
    memories_.clear();

    size_t pos = 0;
    while (pos < source.size()) {
        // Skip whitespace and comments
        while (pos < source.size() && std::isspace(static_cast<unsigned char>(source[pos]))) pos++;
        if (pos >= source.size()) break;

        // Skip single-line comments
        if (pos + 1 < source.size() && source[pos] == '/' && source[pos + 1] == '/') {
            while (pos < source.size() && source[pos] != '\n') pos++;
            continue;
        }

        // Skip block comments
        if (pos + 1 < source.size() && source[pos] == '/' && source[pos + 1] == '*') {
            pos += 2;
            while (pos + 1 < source.size() && !(source[pos] == '*' && source[pos + 1] == '/')) pos++;
            if (pos + 1 < source.size()) pos += 2;
            continue;
        }

        // Look for "module" keyword
        if (pos + 6 < source.size() && source.substr(pos, 6) == "module" &&
            !is_ident_char(source[pos + 6])) {
            pos += 6;
            parse_module_decl(source, pos);
            continue;
        }

        pos++;
    }

    // Scan all module bodies for $readmem calls
    for (auto& mod : modules_) {
        scan_readmem_calls(mod.body);
    }

    return !modules_.empty() || !memories_.empty();
}

// ============================================================================
// $readmemh / $readmemb execution
// ============================================================================

int SvParser::readmem_worker(const std::string& content, const std::string& mem_name,
                              int start_addr, int end_addr, int base) {
    auto it = memories_.find(mem_name);
    if (it == memories_.end()) {
        // Auto-create memory if not declared
        SvMemoryArray mem;
        mem.name = mem_name;
        mem.word_width = 8;
        mem.depth = 256; // default depth
        memories_[mem_name] = mem;
        it = memories_.find(mem_name);
    }

    SvMemoryArray& mem = it->second;
    if (mem.data.empty()) {
        mem.data.resize(static_cast<size_t>(mem.depth), 0);
    }

    int addr = start_addr;
    int max_addr = (end_addr >= 0) ? end_addr : mem.depth - 1;
    int words_loaded = 0;

    std::istringstream iss(content);
    std::string line;
    while (std::getline(iss, line)) {
        std::string t = trim(line);
        if (t.empty()) continue;

        // Skip comments
        if (t[0] == '/' && t.size() > 1 && (t[1] == '/' || t[1] == '*')) continue;

        // Address specification: @HEXADDR
        if (t[0] == '@') {
            std::string addr_str = t.substr(1);
            addr = static_cast<int>(std::stoul(trim(addr_str), nullptr, 16));
            continue;
        }

        // Parse data values (may be space-separated on one line)
        std::istringstream vals(t);
        std::string val;
        while (vals >> val) {
            if (val.empty()) continue;
            // Skip inline comments
            if (val[0] == '/' && val.size() > 1 && (val[1] == '/' || val[1] == '*')) break;

            if (addr > max_addr) break;
            if (addr >= 0 && addr < static_cast<int>(mem.data.size())) {
                // Remove underscores
                std::string clean;
                for (char c : val) {
                    if (c != '_') clean += c;
                }
                try {
                    mem.data[static_cast<size_t>(addr)] =
                        static_cast<uint64_t>(std::stoull(clean, nullptr, base));
                } catch (...) {
                    // Handle 'x' or 'z' values — store as 0
                    mem.data[static_cast<size_t>(addr)] = 0;
                }
                words_loaded++;
            }
            addr++;
        }
    }
    return words_loaded;
}

int SvParser::execute_readmemh(const std::string& filename, const std::string& mem_name,
                                int start_addr, int end_addr) {
    std::ifstream f(filename);
    if (!f.is_open()) return 0;
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    return readmem_worker(content, mem_name, start_addr, end_addr, 16);
}

int SvParser::execute_readmemb(const std::string& filename, const std::string& mem_name,
                                int start_addr, int end_addr) {
    std::ifstream f(filename);
    if (!f.is_open()) return 0;
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    return readmem_worker(content, mem_name, start_addr, end_addr, 2);
}

int SvParser::execute_readmemh_data(const std::string& hex_data, const std::string& mem_name,
                                     int start_addr, int end_addr) {
    return readmem_worker(hex_data, mem_name, start_addr, end_addr, 16);
}

int SvParser::execute_readmemb_data(const std::string& bin_data, const std::string& mem_name,
                                     int start_addr, int end_addr) {
    return readmem_worker(bin_data, mem_name, start_addr, end_addr, 2);
}

// ============================================================================
// Parameter resolution
// ============================================================================

bool SvParser::resolve_parameters(const std::string& module_name,
                                   const std::unordered_map<std::string, std::string>& param_overrides) {
    SvParamModule* mod = find_module(module_name);
    if (!mod) return false;

    // Build context from already-resolved parameters
    std::unordered_map<std::string, int64_t> ctx;

    // First pass: resolve parameters with explicit overrides or literal defaults
    for (auto& p : mod->parameters) {
        auto ov = param_overrides.find(p.name);
        if (ov != param_overrides.end()) {
            p.resolved_int = eval_param_expr(ov->second, ctx);
        } else {
            p.resolved_int = eval_param_expr(p.default_value, ctx);
        }
        p.resolved = true;
        ctx[p.name] = p.resolved_int;
    }

    // Second pass: re-evaluate in case of forward references (localparams depending on params)
    for (auto& p : mod->parameters) {
        auto ov = param_overrides.find(p.name);
        if (ov != param_overrides.end()) {
            p.resolved_int = eval_param_expr(ov->second, ctx);
        } else {
            p.resolved_int = eval_param_expr(p.default_value, ctx);
        }
        ctx[p.name] = p.resolved_int;
    }

    return true;
}

// ============================================================================
// Accessors
// ============================================================================

SvParamModule* SvParser::find_module(const std::string& name) {
    for (auto& m : modules_) {
        if (m.name == name) return &m;
    }
    return nullptr;
}

SvMemoryArray* SvParser::find_memory(const std::string& name) {
    auto it = memories_.find(name);
    return (it != memories_.end()) ? &it->second : nullptr;
}

} // namespace sf
