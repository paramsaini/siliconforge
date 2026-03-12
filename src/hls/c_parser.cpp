// SiliconForge — High-Level Synthesis (HLS) Industrial Implementation
// if/else, for/while, loop unrolling, ASAP/ALAP/list scheduling,
// resource allocation, binding, pipeline generation
#include "hls/c_parser.hpp"
#include "core/netlist.hpp"
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <unordered_map>

namespace sf {

// ============================================================================
// Tokenizer
// ============================================================================
std::vector<std::string> CParser::tokenize(const std::string& src) const {
    std::vector<std::string> tokens;
    std::string cur;
    for (size_t i = 0; i < src.length(); ++i) {
        char c = src[i];
        if (std::isspace(c)) {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            continue;
        }
        // Two-char operators
        if (i + 1 < src.length()) {
            std::string two = std::string(1, c) + src[i+1];
            if (two == "==" || two == "!=" || two == "<=" || two == ">=" ||
                two == "<<" || two == ">>") {
                if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
                tokens.push_back(two);
                i++;
                continue;
            }
        }
        if (c == '(' || c == ')' || c == '{' || c == '}' || c == ';' ||
            c == '=' || c == '+' || c == '*' || c == '-' || c == '/' ||
            c == '<' || c == '>' || c == '&' || c == '|' || c == '^' ||
            c == ',' || c == '!') {
            if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
            tokens.push_back(std::string(1, c));
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) tokens.push_back(cur);
    return tokens;
}

// ============================================================================
// Expression parser: returns node ID of result
// ============================================================================
int CParser::parse_expr(const std::vector<std::string>& tokens, size_t& i,
                        CfgBlock& block, std::map<std::string,int>& var_map) {
    // Parse a single term first
    auto parse_term = [&]() -> int {
        if (i >= tokens.size()) return -1;
        std::string tok = tokens[i++];
        if (tok == "(") {
            int r = parse_expr(tokens, i, block, var_map);
            if (i < tokens.size() && tokens[i] == ")") i++;
            return r;
        }
        if (std::isdigit(tok[0]) || (tok[0] == '-' && tok.size() > 1)) {
            block.datapath.push_back({id_counter_++, HlsOp::CONST, std::stol(tok), "", {}});
            return block.datapath.back().id;
        }
        // Variable
        if (!var_map.count(tok)) {
            block.datapath.push_back({id_counter_++, HlsOp::READ_VAR, 0, tok, {}});
            var_map[tok] = block.datapath.back().id;
        }
        return var_map[tok];
    };

    int left = parse_term();
    if (left < 0) return -1;

    while (i < tokens.size()) {
        std::string op_str = tokens[i];
        HlsOp op;
        if (op_str == "+") op = HlsOp::ADD;
        else if (op_str == "-") op = HlsOp::SUB;
        else if (op_str == "*") op = HlsOp::MUL;
        else if (op_str == "/") op = HlsOp::DIV;
        else if (op_str == "&") op = HlsOp::AND;
        else if (op_str == "|") op = HlsOp::OR;
        else if (op_str == "^") op = HlsOp::XOR;
        else if (op_str == "<<") op = HlsOp::SHL;
        else if (op_str == ">>") op = HlsOp::SHR;
        else if (op_str == "==") op = HlsOp::CMP_EQ;
        else if (op_str == "!=") op = HlsOp::CMP_NE;
        else if (op_str == "<") op = HlsOp::CMP_LT;
        else if (op_str == ">") op = HlsOp::CMP_GT;
        else if (op_str == "<=") op = HlsOp::CMP_LE;
        else if (op_str == ">=") op = HlsOp::CMP_GE;
        else break; // not an operator

        i++; // consume operator
        int right = parse_term();
        block.datapath.push_back({id_counter_++, op, 0, "", {left, right}});
        left = block.datapath.back().id;
    }
    return left;
}

// ============================================================================
// Block parser: parses statements into CfgBlocks
// ============================================================================
int CParser::parse_block(const std::vector<std::string>& tokens, size_t& i,
                         std::vector<CfgBlock>& blocks, std::map<std::string,int>& var_map) {
    int bid = block_counter_++;
    blocks.push_back({bid, "block_" + std::to_string(bid), {}, -1, -1, -1, -1, false, -1});
    // NOTE: Do NOT hold a reference to blocks.back() — recursive parse_block()
    // calls push_back() which can reallocate the vector, invalidating references.
    // Always access via blocks[bid] after any recursive call.

    while (i < tokens.size()) {
        if (tokens[i] == "}") { i++; break; }
        if (tokens[i] == "{") { i++; continue; }

        // if statement
        if (tokens[i] == "if") {
            i++; // skip 'if'
            if (i < tokens.size() && tokens[i] == "(") i++;
            int cond_id = parse_expr(tokens, i, blocks[bid], var_map);
            if (i < tokens.size() && tokens[i] == ")") i++;
            blocks[bid].cond_node_id = cond_id;

            if (i < tokens.size() && tokens[i] == "{") i++;
            int true_bid = parse_block(tokens, i, blocks, var_map);
            blocks[bid].true_block_id = true_bid;

            if (i < tokens.size() && tokens[i] == "else") {
                i++;
                if (i < tokens.size() && tokens[i] == "{") i++;
                int false_bid = parse_block(tokens, i, blocks, var_map);
                blocks[bid].false_block_id = false_bid;
            }
            continue;
        }

        // for loop: for (init; cond; incr) { body }
        if (tokens[i] == "for") {
            i++; // skip 'for'
            if (i < tokens.size() && tokens[i] == "(") i++;

            // Parse init: var = expr;
            if (i + 2 < tokens.size() && tokens[i+1] == "=") {
                std::string var = tokens[i]; i += 2;
                int val = parse_expr(tokens, i, blocks[bid], var_map);
                blocks[bid].datapath.push_back({id_counter_++, HlsOp::ASSIGN, 0, var, {val}});
                var_map[var] = blocks[bid].datapath.back().id;
            }
            if (i < tokens.size() && tokens[i] == ";") i++;

            // Parse cond
            int cond_id = parse_expr(tokens, i, blocks[bid], var_map);
            if (i < tokens.size() && tokens[i] == ";") i++;

            // Parse incr (skip for now, mark as loop)
            while (i < tokens.size() && tokens[i] != ")") i++;
            if (i < tokens.size() && tokens[i] == ")") i++;

            // Parse body
            if (i < tokens.size() && tokens[i] == "{") i++;
            int body_bid = parse_block(tokens, i, blocks, var_map);
            // Mark as loop
            for (auto& b : blocks) {
                if (b.id == body_bid) {
                    b.is_loop_header = true;
                    b.loop_bound = 8; // default bound for unrolling estimation
                }
            }
            blocks[bid].true_block_id = body_bid;
            blocks[bid].cond_node_id = cond_id;
            continue;
        }

        // while loop
        if (tokens[i] == "while") {
            i++;
            if (i < tokens.size() && tokens[i] == "(") i++;
            int cond_id = parse_expr(tokens, i, blocks[bid], var_map);
            if (i < tokens.size() && tokens[i] == ")") i++;
            if (i < tokens.size() && tokens[i] == "{") i++;
            int body_bid = parse_block(tokens, i, blocks, var_map);
            for (auto& b : blocks) {
                if (b.id == body_bid) b.is_loop_header = true;
            }
            blocks[bid].true_block_id = body_bid;
            blocks[bid].cond_node_id = cond_id;
            continue;
        }

        // Assignment: var = expr ;
        if (i + 1 < tokens.size() && tokens[i+1] == "=") {
            std::string target = tokens[i];
            i += 2; // skip var and =
            int val = parse_expr(tokens, i, blocks[bid], var_map);
            if (i < tokens.size() && tokens[i] == ";") i++;
            blocks[bid].datapath.push_back({id_counter_++, HlsOp::ASSIGN, 0, target, {val}});
            var_map[target] = blocks[bid].datapath.back().id;
            continue;
        }

        i++; // skip unrecognized token
    }
    return bid;
}

// ============================================================================
// Parser entry point
// ============================================================================
std::vector<CfgBlock> CParser::parse(const std::string& source) {
    auto tokens = tokenize(source);
    std::vector<CfgBlock> blocks;
    std::map<std::string, int> var_map;
    size_t i = 0;
    parse_block(tokens, i, blocks, var_map);
    return blocks;
}

// ============================================================================
// Scheduler: operation latencies
// ============================================================================
int HlsScheduler::get_latency(HlsOp op) {
    switch (op) {
        case HlsOp::MUL: return 2;
        case HlsOp::DIV: return 4;
        case HlsOp::ADD: case HlsOp::SUB: return 1;
        case HlsOp::SHL: case HlsOp::SHR: return 1;
        case HlsOp::CMP_EQ: case HlsOp::CMP_NE:
        case HlsOp::CMP_LT: case HlsOp::CMP_GT:
        case HlsOp::CMP_LE: case HlsOp::CMP_GE: return 1;
        case HlsOp::AND: case HlsOp::OR: case HlsOp::XOR: return 1;
        default: return 0;
    }
}

// ============================================================================
// ASAP Scheduling
// ============================================================================
void HlsScheduler::schedule_asap(std::vector<CfgBlock>& cdfg) {
    for (auto& block : cdfg) {
        std::map<int, int> node_cycles;
        for (auto& node : block.datapath) {
            int start_cycle = 1;
            for (int in_id : node.inputs) {
                if (node_cycles.count(in_id))
                    start_cycle = std::max(start_cycle, node_cycles[in_id]);
            }
            node.cycle = start_cycle;
            node_cycles[node.id] = start_cycle + get_latency(node.op);
        }
    }
}

// ============================================================================
// ALAP Scheduling
// ============================================================================
void HlsScheduler::schedule_alap(std::vector<CfgBlock>& cdfg, int deadline) {
    for (auto& block : cdfg) {
        // Build dependency graph
        std::map<int, std::vector<int>> successors;
        std::unordered_set<int> all_ids;
        for (auto& node : block.datapath) {
            all_ids.insert(node.id);
            for (int in_id : node.inputs)
                successors[in_id].push_back(node.id);
        }

        // Schedule from deadline backwards
        std::map<int, int> node_alap;
        // Initialize all nodes to deadline
        for (auto& node : block.datapath)
            node_alap[node.id] = deadline;

        // Reverse topological order
        for (int pass = 0; pass < (int)block.datapath.size(); pass++) {
            for (auto it = block.datapath.rbegin(); it != block.datapath.rend(); ++it) {
                int latest = deadline;
                for (int succ_id : successors[it->id]) {
                    if (node_alap.count(succ_id))
                        latest = std::min(latest, node_alap[succ_id] - get_latency(it->op));
                }
                node_alap[it->id] = latest;
            }
        }

        for (auto& node : block.datapath)
            node.alap_cycle = node_alap[node.id];
    }
}

// ============================================================================
// Compute mobility (ALAP - ASAP)
// ============================================================================
void HlsScheduler::compute_mobility(std::vector<CfgBlock>& cdfg) {
    for (auto& block : cdfg) {
        for (auto& node : block.datapath) {
            if (node.cycle >= 0 && node.alap_cycle >= 0)
                node.mobility = node.alap_cycle - node.cycle;
            else
                node.mobility = 0;
        }
    }
}

// ============================================================================
// List Scheduling (resource-constrained)
// ============================================================================
void HlsScheduler::schedule_list(std::vector<CfgBlock>& cdfg, const HlsConfig& cfg) {
    for (auto& block : cdfg) {
        std::map<int, int> node_finish;  // node_id → finish cycle
        std::map<HlsOp, int> max_resources;
        max_resources[HlsOp::ADD] = cfg.max_adders;
        max_resources[HlsOp::SUB] = cfg.max_adders;
        max_resources[HlsOp::MUL] = cfg.max_multipliers;
        max_resources[HlsOp::DIV] = cfg.max_dividers;

        int current_cycle = 1;
        std::vector<bool> scheduled(block.datapath.size(), false);
        int total = (int)block.datapath.size();
        int done = 0;

        while (done < total) {
            std::map<HlsOp, int> used;
            for (size_t idx = 0; idx < block.datapath.size(); idx++) {
                if (scheduled[idx]) continue;
                auto& node = block.datapath[idx];
                // Check dependencies
                bool deps_met = true;
                for (int in_id : node.inputs) {
                    if (node_finish.count(in_id) && node_finish[in_id] > current_cycle)
                        deps_met = false;
                    if (!node_finish.count(in_id)) {
                        // Check if input is from a node in this block
                        bool found = false;
                        for (size_t j = 0; j < block.datapath.size(); j++) {
                            if (block.datapath[j].id == in_id) {
                                if (!scheduled[j]) deps_met = false;
                                found = true;
                                break;
                            }
                        }
                    }
                }
                if (!deps_met) continue;

                // Check resources
                int limit = 9999;
                if (max_resources.count(node.op)) limit = max_resources[node.op];
                if (used[node.op] >= limit) continue;

                node.cycle = current_cycle;
                node_finish[node.id] = current_cycle + get_latency(node.op);
                used[node.op]++;
                scheduled[idx] = true;
                done++;
            }
            current_cycle++;
            if (current_cycle > 1000) break; // safety
        }
    }
}

// ============================================================================
// Resource Allocation
// ============================================================================
std::vector<HlsResource> HlsResourceAllocator::allocate(const std::vector<CfgBlock>& cdfg,
                                                          const HlsConfig& cfg) {
    // Count max concurrent usage per cycle per op type
    std::map<HlsOp, int> max_concurrent;
    for (auto& block : cdfg) {
        std::map<int, std::map<HlsOp, int>> per_cycle;
        for (auto& node : block.datapath) {
            if (node.cycle > 0)
                per_cycle[node.cycle][node.op]++;
        }
        for (auto& [cyc, ops] : per_cycle) {
            for (auto& [op, count] : ops)
                max_concurrent[op] = std::max(max_concurrent[op], count);
        }
    }

    std::vector<HlsResource> resources;
    int rid = 0;
    for (auto& [op, count] : max_concurrent) {
        int lat = HlsScheduler::get_latency(op);
        if (lat > 0) {
            int limited = count;
            if (op == HlsOp::ADD || op == HlsOp::SUB) limited = std::min(count, cfg.max_adders);
            if (op == HlsOp::MUL) limited = std::min(count, cfg.max_multipliers);
            if (op == HlsOp::DIV) limited = std::min(count, cfg.max_dividers);
            double area = (op == HlsOp::MUL) ? 4.0 : (op == HlsOp::DIV) ? 8.0 : 1.0;
            resources.push_back({rid++, op, lat, limited, area * limited});
        }
    }
    return resources;
}

// ============================================================================
// Resource Binding (left-edge algorithm)
// ============================================================================
void HlsResourceAllocator::bind(std::vector<CfgBlock>& cdfg,
                                 const std::vector<HlsResource>& resources) {
    for (auto& block : cdfg) {
        // Group nodes by op type, sort by start cycle
        std::map<HlsOp, std::vector<DfgNode*>> by_op;
        for (auto& node : block.datapath) {
            if (node.cycle > 0 && HlsScheduler::get_latency(node.op) > 0)
                by_op[node.op].push_back(&node);
        }

        for (auto& [op, nodes] : by_op) {
            std::sort(nodes.begin(), nodes.end(),
                      [](DfgNode* a, DfgNode* b) { return a->cycle < b->cycle; });

            // Find resource count
            int num_instances = 1;
            for (auto& r : resources) {
                if (r.type == op) { num_instances = r.count; break; }
            }

            // Left-edge binding
            std::vector<int> instance_free_at(num_instances, 0);
            for (auto* node : nodes) {
                int best = 0;
                for (int i = 1; i < num_instances; i++) {
                    if (instance_free_at[i] < instance_free_at[best]) best = i;
                }
                node->resource_id = best;
                instance_free_at[best] = node->cycle + HlsScheduler::get_latency(op);
            }
        }
    }
}

// ============================================================================
// Pipeline Generation
// ============================================================================
std::vector<PipelineStage> HlsPipeliner::pipeline(std::vector<CfgBlock>& cdfg, int ii) {
    std::vector<PipelineStage> stages;
    int max_cycle = 0;
    for (auto& block : cdfg) {
        for (auto& node : block.datapath) {
            if (node.cycle > max_cycle) max_cycle = node.cycle;
        }
    }
    for (int c = 1; c <= max_cycle; c++) {
        PipelineStage stage;
        stage.stage_id = c;
        stage.initiation_interval = ii;
        for (auto& block : cdfg) {
            for (auto& node : block.datapath) {
                if (node.cycle == c) stage.node_ids.push_back(node.id);
            }
        }
        if (!stage.node_ids.empty()) stages.push_back(stage);
    }
    return stages;
}

// ============================================================================
// HLS Synthesizer: full industrial flow
// ============================================================================
HlsResult HlsSynthesizer::synthesize(std::vector<CfgBlock>& cdfg, const HlsConfig& cfg) {
    HlsResult result;

    // Step 1: Schedule
    switch (cfg.schedule_algo) {
        case HlsConfig::ASAP:
            HlsScheduler::schedule_asap(cdfg);
            break;
        case HlsConfig::ALAP: {
            HlsScheduler::schedule_asap(cdfg);
            int deadline = 1;
            for (auto& b : cdfg) for (auto& n : b.datapath)
                deadline = std::max(deadline, n.cycle + HlsScheduler::get_latency(n.op));
            HlsScheduler::schedule_alap(cdfg, deadline);
            break;
        }
        case HlsConfig::LIST:
            HlsScheduler::schedule_list(cdfg, cfg);
            break;
        case HlsConfig::FORCE_DIRECTED:
            HlsScheduler::schedule_asap(cdfg);
            HlsScheduler::schedule_alap(cdfg, 20);
            HlsScheduler::compute_mobility(cdfg);
            break;
    }

    // Step 2: Allocate resources
    auto resources = HlsResourceAllocator::allocate(cdfg, cfg);

    // Step 3: Bind
    HlsResourceAllocator::bind(cdfg, resources);

    // Step 4: Pipeline (if enabled)
    if (cfg.enable_pipelining) {
        result.pipeline = HlsPipeliner::pipeline(cdfg, cfg.pipeline_ii);
    }

    // Step 5: Compute metrics
    int max_cycle = 0;
    for (auto& b : cdfg) for (auto& n : b.datapath)
        max_cycle = std::max(max_cycle, n.cycle + HlsScheduler::get_latency(n.op));
    result.total_cycles = max_cycle;
    result.num_states = max_cycle;

    for (auto& r : resources) {
        if (r.type == HlsOp::ADD || r.type == HlsOp::SUB) result.num_adders += r.count;
        if (r.type == HlsOp::MUL) result.num_multipliers += r.count;
        result.estimated_area += r.area;
    }

    // Count registers (variables alive across cycles)
    std::unordered_set<std::string> vars;
    for (auto& b : cdfg) for (auto& n : b.datapath)
        if (n.op == HlsOp::ASSIGN || n.op == HlsOp::READ_VAR)
            if (!n.var_name.empty()) vars.insert(n.var_name);
    result.num_registers = (int)vars.size();

    result.estimated_freq_mhz = cfg.target_clock_mhz;
    result.success = true;

    // Generate reports
    std::ostringstream sched_rpt;
    sched_rpt << "Schedule Report:\n";
    for (auto& b : cdfg) {
        sched_rpt << "  Block " << b.id << ":\n";
        for (auto& n : b.datapath) {
            sched_rpt << "    Node " << n.id << " (" << hls_op_str(n.op)
                      << ") cycle=" << n.cycle;
            if (n.alap_cycle >= 0) sched_rpt << " alap=" << n.alap_cycle;
            if (n.resource_id >= 0) sched_rpt << " res=" << n.resource_id;
            sched_rpt << "\n";
        }
    }
    result.schedule_report = sched_rpt.str();

    std::ostringstream bind_rpt;
    bind_rpt << "Binding Report:\n";
    for (auto& r : resources) {
        bind_rpt << "  " << hls_op_str(r.type) << " x" << r.count
                 << " (latency=" << r.latency << " area=" << r.area << ")\n";
    }
    result.binding_report = bind_rpt.str();

    return result;
}

// ============================================================================
// Netlist synthesis (simplified — FSM + datapath)
// ============================================================================
void HlsSynthesizer::synthesize_to_netlist(const std::vector<CfgBlock>& cdfg, Netlist& nl) {
    if (cdfg.empty()) return;

    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId rst = nl.add_net("rst"); nl.mark_input(rst);
    NetId start = nl.add_net("start"); nl.mark_input(start);
    NetId done = nl.add_net("done"); nl.mark_output(done);

    int max_cycle = 1;
    for (const auto& b : cdfg)
        for (const auto& n : b.datapath)
            if (n.cycle > max_cycle) max_cycle = n.cycle;

    // FSM one-hot states
    std::vector<NetId> state_q(max_cycle + 2);
    std::vector<NetId> state_d(max_cycle + 2);
    for (int i = 0; i <= max_cycle + 1; ++i) {
        state_q[i] = nl.add_net("fsm_q_" + std::to_string(i));
        state_d[i] = nl.add_net("fsm_d_" + std::to_string(i));
        NetId rst_val = (i == 0) ? nl.add_net("ONE") : nl.add_net("ZERO");
        if (i == 0) nl.add_gate(GateType::CONST1, {}, rst_val);
        else nl.add_gate(GateType::CONST0, {}, rst_val);
        NetId mux_out = nl.add_net("fsm_rst_mux_" + std::to_string(i));
        nl.add_gate(GateType::MUX, {state_d[i], rst_val, rst}, mux_out);
        nl.add_dff(mux_out, clk, state_q[i], -1, "FSM_FF_" + std::to_string(i));
    }

    nl.add_gate(GateType::AND, {state_q[0], start}, state_d[1]);
    for (int i = 1; i <= max_cycle; ++i)
        state_d[i+1] = state_q[i];
    nl.add_gate(GateType::BUF, {state_q[max_cycle+1]}, done);

    // Create variable I/O nets
    std::map<std::string, NetId> var_nets;
    for (const auto& b : cdfg) {
        for (const auto& n : b.datapath) {
            if (n.op == HlsOp::READ_VAR && !var_nets.count(n.var_name)) {
                NetId pin = nl.add_net("hls_in_" + n.var_name);
                nl.mark_input(pin);
                var_nets[n.var_name] = pin;
            }
            if (n.op == HlsOp::ASSIGN && !var_nets.count(n.var_name)) {
                NetId pin = nl.add_net("hls_out_" + n.var_name);
                nl.mark_output(pin);
                var_nets[n.var_name] = pin;
            }
        }
    }
}

} // namespace sf
