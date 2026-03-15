#pragma once
// SiliconForge — High-Level Synthesis (HLS) Engine
// Industrial: if/else, for/while loops, loop unrolling, ASAP/ALAP/list scheduling,
//   resource allocation, binding, pipeline generation
// Reference: De Micheli "Synthesis and Optimization of Digital Circuits"
//   Paulin & Knight "Force-Directed Scheduling"

#include <string>
#include <vector>
#include <memory>
#include <map>

namespace sf {

enum class HlsOp {
    ADD, SUB, MUL, DIV, AND, OR, XOR, SHL, SHR,
    CMP_EQ, CMP_NE, CMP_LT, CMP_GT, CMP_LE, CMP_GE,
    ASSIGN, CONST, READ_VAR, PHI, NOP, SELECT
};

inline const char* hls_op_str(HlsOp op) {
    switch(op) {
        case HlsOp::ADD: return "ADD"; case HlsOp::SUB: return "SUB";
        case HlsOp::MUL: return "MUL"; case HlsOp::DIV: return "DIV";
        case HlsOp::AND: return "AND"; case HlsOp::OR: return "OR";
        case HlsOp::XOR: return "XOR"; case HlsOp::SHL: return "SHL";
        case HlsOp::SHR: return "SHR"; case HlsOp::ASSIGN: return "ASSIGN";
        case HlsOp::CONST: return "CONST"; case HlsOp::READ_VAR: return "READ_VAR";
        case HlsOp::PHI: return "PHI"; case HlsOp::NOP: return "NOP";
        case HlsOp::SELECT: return "SELECT";
        default: return "CMP";
    }
}

// Data Flow Graph Node
struct DfgNode {
    int id;
    HlsOp op;
    long value;
    std::string var_name;
    std::vector<int> inputs;
    int cycle = -1;         // Assigned during scheduling
    int alap_cycle = -1;    // ALAP schedule
    int resource_id = -1;   // Bound resource instance
    double mobility = 0;    // ALAP - ASAP (scheduling freedom)
};

// Control Flow Graph Block
struct CfgBlock {
    int id;
    std::string label;
    std::vector<DfgNode> datapath;
    int next_block_id = -1;
    int true_block_id = -1;   // for if-then-else
    int false_block_id = -1;
    int cond_node_id = -1;    // condition for branch
    bool is_loop_header = false;
    int loop_bound = -1;      // for loop unrolling
};

// Resource type for allocation/binding
struct HlsResource {
    int id;
    HlsOp type;          // functional unit type
    int latency = 1;      // cycles to complete
    int count = 1;         // number of instances
    double area = 1.0;     // relative area cost
};

// Pipeline stage info
struct PipelineStage {
    int stage_id;
    std::vector<int> node_ids;  // nodes executing in this stage
    int initiation_interval = 1;
};

// Array type for HLS
struct HlsArrayType {
    std::string element_type;
    int size = 0;
};

// Struct type for HLS
struct HlsStructType {
    std::string name;
    std::vector<std::pair<std::string, std::string>> fields; // name, type
};

// Floating-point type for HLS synthesis
// Reference: IEEE 754-2019 — binary16 (half), binary32 (single), binary64 (double)
struct HlsFloatType {
    int width = 32;           // total bit width
    int exponent_bits = 8;    // exponent field width
    int mantissa_bits = 23;   // mantissa (significand) field width (excludes implicit leading 1)
    bool is_signed = true;    // IEEE 754 always has sign bit

    // Predefined IEEE 754 formats
    static HlsFloatType half()   { HlsFloatType f; f.width=16; f.exponent_bits=5;  f.mantissa_bits=10; return f; }
    static HlsFloatType single() { HlsFloatType f; f.width=32; f.exponent_bits=8;  f.mantissa_bits=23; return f; }
    static HlsFloatType dbl()    { HlsFloatType f; f.width=64; f.exponent_bits=11; f.mantissa_bits=52; return f; }

    // Fixed-point conversion parameters
    int fixed_int_bits = 16;    // integer part width after float-to-fixed
    int fixed_frac_bits = 16;   // fractional part width after float-to-fixed
    int fixed_total() const { return fixed_int_bits + fixed_frac_bits; }
};

// Floating-point operator synthesis result
struct HlsFloatOpResult {
    enum OpType { FADD, FSUB, FMUL, FDIV, F2I, I2F, F2FIXED, FIXED2F };
    OpType op = FADD;
    int latency_cycles = 1;    // pipeline latency in clock cycles
    double area_estimate = 0;  // relative area (normalized to integer adder = 1.0)
    int dsp_blocks = 0;        // DSP block count (for FPGA targets)
    std::string rtl_module;    // name of generated RTL module
};

// HLS scheduling and synthesis configuration
struct HlsConfig {
    int target_clock_mhz = 100;
    int max_adders = 2;
    int max_multipliers = 1;
    int max_dividers = 1;
    bool enable_loop_unrolling = true;
    int max_unroll_factor = 8;
    bool enable_pipelining = false;
    int pipeline_ii = 1;  // initiation interval
    enum ScheduleAlgo { ASAP, ALAP, LIST, FORCE_DIRECTED } schedule_algo = ASAP;
};

// HLS synthesis result
struct HlsResult {
    bool success = false;
    int total_cycles = 0;
    int num_states = 0;
    int num_adders = 0;
    int num_multipliers = 0;
    int num_registers = 0;
    double estimated_area = 0;
    double estimated_freq_mhz = 0;
    std::vector<PipelineStage> pipeline;
    std::string schedule_report;
    std::string binding_report;
};

class CParser {
public:
    std::vector<CfgBlock> parse(const std::string& source);

    // Access parsed type information
    const std::vector<HlsArrayType>& array_types() const { return array_types_; }
    const std::vector<HlsStructType>& struct_types() const { return struct_types_; }
    const std::vector<HlsFloatType>& float_types() const { return float_types_; }

private:
    int id_counter_ = 0;
    int block_counter_ = 0;
    std::vector<HlsArrayType> array_types_;
    std::vector<HlsStructType> struct_types_;
    std::vector<HlsFloatType> float_types_;
    std::vector<std::string> tokenize(const std::string& src) const;
    int parse_block(const std::vector<std::string>& tokens, size_t& i,
                    std::vector<CfgBlock>& blocks, std::map<std::string,int>& var_map);
    int parse_expr(const std::vector<std::string>& tokens, size_t& i,
                   CfgBlock& block, std::map<std::string,int>& var_map);
    void parse_array_decl(const std::vector<std::string>& tokens, size_t& i);
    void parse_struct_decl(const std::vector<std::string>& tokens, size_t& i);
    void parse_float_decl(const std::vector<std::string>& tokens, size_t& i);
};

// ============================================================================
// Floating-Point Synthesis Engine
// Reference: IEEE 754-2019, Muller et al. "Handbook of Floating-Point Arithmetic"
// ============================================================================

class HlsFloatSynthesizer {
public:
    // Synthesize a floating-point add/sub unit
    // Stages: align exponents -> add mantissas -> normalize -> round
    static HlsFloatOpResult synthesize_fadd(const HlsFloatType& fmt);

    // Synthesize a floating-point multiplier
    // Stages: multiply mantissas -> add exponents -> normalize -> round
    static HlsFloatOpResult synthesize_fmul(const HlsFloatType& fmt);

    // Synthesize a floating-point divider (non-restoring or SRT)
    // Stages: iterative division -> normalize -> round
    static HlsFloatOpResult synthesize_fdiv(const HlsFloatType& fmt);

    // Float-to-fixed conversion (for fixed-point datapath optimization)
    // Returns the fixed-point configuration and conversion unit parameters
    static HlsFloatOpResult synthesize_f2fixed(const HlsFloatType& fmt);

    // Fixed-to-float conversion
    static HlsFloatOpResult synthesize_fixed2f(const HlsFloatType& fmt);

    // Float-to-integer conversion
    static HlsFloatOpResult synthesize_f2i(const HlsFloatType& fmt);

    // Integer-to-float conversion
    static HlsFloatOpResult synthesize_i2f(const HlsFloatType& fmt);

    // Determine IEEE 754 format from C type name
    static HlsFloatType type_from_name(const std::string& type_name);

    // Estimate total area and latency for a floating-point expression tree
    struct FloatExprCost {
        int total_latency = 0;
        double total_area = 0;
        int adders = 0;
        int multipliers = 0;
        int dividers = 0;
        int converters = 0;
    };
    static FloatExprCost estimate_cost(const std::vector<HlsFloatOpResult>& ops);
};

class HlsScheduler {
public:
    static void schedule_asap(std::vector<CfgBlock>& cdfg);
    static void schedule_alap(std::vector<CfgBlock>& cdfg, int deadline);
    static void schedule_list(std::vector<CfgBlock>& cdfg, const HlsConfig& cfg);
    static void compute_mobility(std::vector<CfgBlock>& cdfg);
    static int get_latency(HlsOp op);
};

class HlsResourceAllocator {
public:
    static std::vector<HlsResource> allocate(const std::vector<CfgBlock>& cdfg, const HlsConfig& cfg);
    static void bind(std::vector<CfgBlock>& cdfg, const std::vector<HlsResource>& resources);
};

class HlsPipeliner {
public:
    static std::vector<PipelineStage> pipeline(std::vector<CfgBlock>& cdfg, int ii);
};

class HlsSynthesizer {
public:
    static HlsResult synthesize(std::vector<CfgBlock>& cdfg, const HlsConfig& cfg);
    static void synthesize_to_netlist(const std::vector<CfgBlock>& cdfg, class Netlist& nl);
};

// Loop pipelining
struct HlsPipelineConfig {
    int target_ii;
    int max_stages;
};
struct HlsPipelineResult {
    int achieved_ii;
    int stages;
    double throughput_improvement;
};

// Scheduling algorithms (enhanced)
enum class HlsScheduleAlgo { ASAP, ALAP, FORCE_DIRECTED, LIST };
struct HlsScheduleResult {
    int total_cycles;
    int total_operations;
    std::vector<std::pair<std::string,int>> op_to_cycle;
    double resource_utilization;
};

// Resource binding
struct HlsBindingResult {
    int functional_units;
    int registers;
    int muxes;
    std::vector<std::pair<std::string,int>> op_to_fu;
};

// Array partitioning
struct ArrayPartResult {
    std::string array_name;
    int partitions;
    enum PartType { BLOCK, CYCLIC, COMPLETE } type;
    int bank_width;
};

// Enhanced HLS engine
class HlsEnhanced {
public:
    explicit HlsEnhanced(std::vector<CfgBlock>& cdfg) : cdfg_(cdfg) {}
    
    HlsPipelineResult pipeline_loop(const std::string& loop_name, const HlsPipelineConfig& cfg);
    HlsScheduleResult schedule(HlsScheduleAlgo algo = HlsScheduleAlgo::FORCE_DIRECTED);
    HlsBindingResult bind_resources();
    ArrayPartResult partition_array(const std::string& name, int factor);
    HlsResult run_enhanced(const HlsConfig& cfg = {});
    
private:
    std::vector<CfgBlock>& cdfg_;
};

} // namespace sf
