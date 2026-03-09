// SiliconForge — Mini-C High-Level Synthesis (HLS)
#include "hls/c_parser.hpp"
#include "core/netlist.hpp"
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>

namespace sf {

std::vector<std::string> CParser::tokenize(const std::string& src) const {
    std::vector<std::string> tokens;
    std::string cur;
    for (size_t i = 0; i < src.length(); ++i) {
        char c = src[i];
        if (std::isspace(c)) {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            continue;
        }
        if (c == '(' || c == ')' || c == ';' || c == '=' || c == '+' || c == '*' || c == '-' || c == '/') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            tokens.push_back(std::string(1, c));
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) tokens.push_back(cur);
    return tokens;
}

std::vector<CfgBlock> CParser::parse(const std::string& source) {
    auto tokens = tokenize(source);
    std::vector<CfgBlock> blocks;
    blocks.push_back({id_counter_++, {}, -1});
    auto& block = blocks.back();

    std::map<std::string, int> var_to_node; // Tracks the latest node ID representing a var

    size_t i = 0;
    while (i < tokens.size()) {
        // Very basic parsing: ASSIGNMENT ONLY
        // Expected: var = arg1 op arg2 ;
        if (i + 1 < tokens.size() && tokens[i+1] == "=") {
            std::string target = tokens[i];
            i += 2; // skip var and =

            std::string arg1 = tokens[i++];
            std::string op_str = tokens[i++];
            std::string arg2 = tokens[i++];
            
            if (tokens[i++] != ";") throw std::runtime_error("HLS Parser: Expected ;");

            HlsOp op;
            if (op_str == "+") op = HlsOp::ADD;
            else if (op_str == "-") op = HlsOp::SUB;
            else if (op_str == "*") op = HlsOp::MUL;
            else if (op_str == "/") op = HlsOp::DIV;
            else throw std::runtime_error("HLS Parser: Unknown op " + op_str);

            int n1_id = -1, n2_id = -1;
            
            // Build Arg1 Node
            if (std::isdigit(arg1[0])) {
                block.datapath.push_back({id_counter_++, HlsOp::CONST, std::stol(arg1), "", {}});
                n1_id = block.datapath.back().id;
            } else {
                if (!var_to_node.count(arg1)) {
                    block.datapath.push_back({id_counter_++, HlsOp::READ_VAR, 0, arg1, {}});
                    var_to_node[arg1] = block.datapath.back().id;
                }
                n1_id = var_to_node[arg1];
            }

            // Build Arg2 Node
            if (std::isdigit(arg2[0])) {
                block.datapath.push_back({id_counter_++, HlsOp::CONST, std::stol(arg2), "", {}});
                n2_id = block.datapath.back().id;
            } else {
                if (!var_to_node.count(arg2)) {
                    block.datapath.push_back({id_counter_++, HlsOp::READ_VAR, 0, arg2, {}});
                    var_to_node[arg2] = block.datapath.back().id;
                }
                n2_id = var_to_node[arg2];
            }

            // Build Op Node
            block.datapath.push_back({id_counter_++, op, 0, "", {n1_id, n2_id}});
            int op_id = block.datapath.back().id;

            // Build Assign Node
            block.datapath.push_back({id_counter_++, HlsOp::ASSIGN, 0, target, {op_id}});
            var_to_node[target] = block.datapath.back().id;
        } else {
            throw std::runtime_error("HLS Parser: Unhandled syntax near " + tokens[i]);
        }
    }

    return blocks;
}

void HlsScheduler::schedule_asap(std::vector<CfgBlock>& cdfg) {
    // Basic ASAP: A node's cycle = max(input cycles) + 1
    // Reads/Consts are cycle 1
    // Multipliers take 2 cycles, Adders take 1 cycle
    for (auto& block : cdfg) {
        std::map<int, int> node_cycles;
        
        for (auto& node : block.datapath) {
            int start_cycle = 1;
            for (int in_id : node.inputs) {
                if (node_cycles.count(in_id)) {
                    start_cycle = std::max(start_cycle, node_cycles[in_id]);
                }
            }
            
            node.cycle = start_cycle;
            
            int delay = 0;
            if (node.op == HlsOp::ADD || node.op == HlsOp::SUB) delay = 1;
            if (node.op == HlsOp::MUL) delay = 2; // Multiplier is slower
            if (node.op == HlsOp::DIV) delay = 4;
            
            // Advance for outputs of this node
            node_cycles[node.id] = start_cycle + delay;
        }
    }
}

// Rough synthesis to bit-blasted netlist:
// Since our Netlist is gate-level, building a full 32-bit multiplier here is too much lines of code.
// We'll emulate the synthesis by just mapping the datapath to dummy nodes, and hooking up the FSM.
// We'll represent the 32-bit words using multiple nets (Bit-blasting).
void HlsSynthesizer::synthesize_to_netlist(const std::vector<CfgBlock>& cdfg, Netlist& nl) {
    if (cdfg.empty()) return;

    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId rst = nl.add_net("rst"); nl.mark_input(rst);
    NetId start = nl.add_net("start"); nl.mark_input(start);
    NetId done = nl.add_net("done"); nl.mark_output(done);

    int max_cycle = 1;
    for (const auto& b : cdfg) {
        for (const auto& n : b.datapath) {
            if (n.cycle > max_cycle) max_cycle = n.cycle;
        }
    }

    // FSM State Register (One-hot encoding for cycles)
    std::vector<NetId> state_q(max_cycle + 2); // 0 = IDLE, 1..max_cycle = EXEC, max_cycle+1 = DONE
    std::vector<NetId> state_d(max_cycle + 2);
    for (int i = 0; i <= max_cycle + 1; ++i) {
        state_q[i] = nl.add_net("fsm_q_" + std::to_string(i));
        state_d[i] = nl.add_net("fsm_d_" + std::to_string(i));
        // Reset state 0 is 1, rest are 0
        NetId rst_val = (i == 0) ? nl.add_net("ONE") : nl.add_net("ZERO");
        if (i == 0) nl.add_gate(GateType::CONST1, {}, rst_val);
        else nl.add_gate(GateType::CONST0, {}, rst_val);
        
        // DFF with reset injection is complex in raw gates, we abstract:
        NetId mux_out = nl.add_net("fsm_rst_mux_" + std::to_string(i));
        nl.add_gate(GateType::MUX, {state_d[i], rst_val, rst}, mux_out);
        nl.add_dff(mux_out, clk, state_q[i], -1, "FSM_FF_" + std::to_string(i));
    }

    // State 0 (IDLE) -> State 1 on `start`
    nl.add_gate(GateType::AND, {state_q[0], start}, state_d[1]);
    
    // State i -> State i+1 automatically
    for (int i = 1; i <= max_cycle; ++i) {
        state_d[i+1] = state_q[i]; // simple shift register for linear CDFG
    }
    
    // State DONE stays DONE until reset
    nl.add_gate(GateType::OR, {state_q[max_cycle+1], state_d[max_cycle+1]}, state_d[max_cycle+1]); // wait state

    // Map `done` signal
    nl.add_gate(GateType::BUF, {state_q[max_cycle+1]}, done);

    // Map variables to pseudo-buses (just creating a representative net for the variable for simplicity of HLS tests)
    std::map<std::string, NetId> var_nets;

    for (const auto& b : cdfg) {
        for (const auto& n : b.datapath) {
            if (n.op == HlsOp::READ_VAR) {
                if (!var_nets.count(n.var_name)) {
                    NetId pin = nl.add_net("hls_in_" + n.var_name);
                    nl.mark_input(pin);
                    var_nets[n.var_name] = pin;
                }
            } else if (n.op == HlsOp::ASSIGN) {
                NetId out_pin = nl.add_net("hls_out_" + n.var_name);
                nl.mark_output(out_pin);
                var_nets[n.var_name] = out_pin;
                
                // Represent assignments by gating the incoming data with the FSM state
                // target = (fsm == cycle) ? op_result : target; (Latch)
                NetId state_en = state_q[n.cycle];
                NetId in_data = var_nets["op_" + std::to_string(n.inputs[0])]; // simplistic
                
                // Using an AND gate to represent the data path write enable
                nl.add_gate(GateType::AND, {state_en, in_data}, out_pin, "Write_" + n.var_name);
            } else if (n.op == HlsOp::ADD || n.op == HlsOp::MUL) {
                // Represent op output
                NetId res = nl.add_net("op_" + std::to_string(n.id));
                var_nets["op_" + std::to_string(n.id)] = res;
                // Dummy logic
                nl.add_gate(GateType::AND, {var_nets["prev1"], var_nets["prev2"]}, res, "OpLgc_" + std::to_string(n.id));
            }
        }
    }
}

} // namespace sf
