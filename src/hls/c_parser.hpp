#pragma once
// SiliconForge — Mini-C High-Level Synthesis (HLS) Parser
// Parses a tiny subset of C into a Control-Data Flow Graph (CDFG)

#include <string>
#include <vector>
#include <memory>
#include <map>

namespace sf {

enum class HlsOp {
    ADD, SUB, MUL, DIV, AND, OR, XOR,
    ASSIGN, CONST, READ_VAR
};

// Data Flow Graph Node
struct DfgNode {
    int id;
    HlsOp op;
    long value; // For CONST
    std::string var_name; // For READ_VAR / ASSIGN target
    std::vector<int> inputs; // IDs of input DfgNodes
    int cycle = -1; // Assigned during scheduling
};

// Control Flow Graph Block
struct CfgBlock {
    int id;
    std::vector<DfgNode> datapath;
    // Simplification: just a linear block sequence for now (no branches yet)
    int next_block_id = -1; 
};

class CParser {
public:
    // Parses a simplified C-like string:
    // e.g. "x = a + b; y = x * c;"
    std::vector<CfgBlock> parse(const std::string& source);

private:
    int id_counter_ = 0;
    std::vector<std::string> tokenize(const std::string& src) const;
};

// Scheduler takes the parsed CDFG and assigns clock cycles to operations
class HlsScheduler {
public:
    // Schedules Operations ASAP (As Soon As Possible)
    static void schedule_asap(std::vector<CfgBlock>& cdfg);
};

// RTL Generator takes scheduled CDFG and builds a Netlist
class HlsSynthesizer {
public:
    // Translates the scheduled CDFG into an actual cycle-accurate Netlist
    // (creating adders, multipliers, registers, and FSM)
    static void synthesize_to_netlist(const std::vector<CfgBlock>& cdfg, class Netlist& nl);
};

} // namespace sf
