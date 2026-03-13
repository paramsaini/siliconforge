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

private:
    int id_counter_ = 0;
    int block_counter_ = 0;
    std::vector<std::string> tokenize(const std::string& src) const;
    int parse_block(const std::vector<std::string>& tokens, size_t& i,
                    std::vector<CfgBlock>& blocks, std::map<std::string,int>& var_map);
    int parse_expr(const std::vector<std::string>& tokens, size_t& i,
                   CfgBlock& block, std::map<std::string,int>& var_map);
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
