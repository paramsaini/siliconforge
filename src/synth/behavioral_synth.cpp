// SiliconForge — Behavioral Verilog Synthesis Engine (Multi-Bit Bus Support)
#include "synth/behavioral_synth.hpp"
#include <iostream>
#include <set>
#include <algorithm>
#include <cmath>
#include <functional>

namespace sf {

// ============================================================
// Helper: extract base signal name and bit index from "name[N]"
// ============================================================
static std::string extract_base(const std::string& name) {
    auto p = name.find('[');
    return (p != std::string::npos) ? name.substr(0, p) : name;
}
static int extract_bit(const std::string& name) {
    auto p = name.find('[');
    if (p == std::string::npos) return -1;
    try { return std::stoi(name.substr(p + 1)); } catch (...) { return -1; }
}

// ============================================================
// Core entry point
// ============================================================
bool BehavioralSynthesizer::synthesize(const BehavioralAST& ast, Netlist& nl) {
    if (!ast.root || ast.root->type != AstNodeType::MODULE) return false;
    bus_ranges_ = ast.bus_ranges;
    net_map_ref_ = ast.net_map;
    memory_arrays_ = ast.memory_arrays;
    signed_signals_ = ast.signed_signals;
    uid_counter_ = 0;
    shared_resources_.clear();
    stats_ = SynthStats{};
    synth_node(ast.root, nl);
    return true;
}

std::string BehavioralSynthesizer::uid() { return std::to_string(uid_counter_++); }

// ============================================================
// Width / net helpers
// ============================================================
int BehavioralSynthesizer::get_bus_width(const std::string& name) const {
    auto it = bus_ranges_.find(name);
    if (it == bus_ranges_.end()) return 1;
    return std::abs(it->second.first - it->second.second) + 1;
}

int BehavioralSynthesizer::get_bus_lo(const std::string& name) const {
    auto it = bus_ranges_.find(name);
    if (it == bus_ranges_.end()) return 0;
    return std::min(it->second.first, it->second.second);
}

NetId BehavioralSynthesizer::get_net(Netlist& nl, const std::string& name) {
    for (size_t i = 0; i < nl.num_nets(); ++i) {
        if (nl.net(i).name == name) return i;
    }
    return nl.add_net(name);
}

std::vector<NetId> BehavioralSynthesizer::get_bus_nets(Netlist& nl, const std::string& name,
    const std::map<std::string, std::vector<NetId>>& next_state) {
    auto ns_it = next_state.find(name);
    if (ns_it != next_state.end()) return ns_it->second;
    int w = get_bus_width(name);
    if (w <= 1) return {get_net(nl, name)};
    int lo = get_bus_lo(name);
    std::vector<NetId> bits;
    for (int i = 0; i < w; i++)
        bits.push_back(get_net(nl, name + "[" + std::to_string(lo + i) + "]"));
    return bits;
}

NetId BehavioralSynthesizer::make_const(int bit, Netlist& nl) {
    GateType ct = bit ? GateType::CONST1 : GateType::CONST0;
    NetId out = nl.add_net("const" + std::to_string(bit) + "_" + uid());
    nl.add_gate(ct, {}, out, "beh_const");
    return out;
}

uint64_t BehavioralSynthesizer::parse_number(const std::string& s) const {
    uint64_t val = 0;
    auto tick = s.find('\'');
    if (tick != std::string::npos) {
        char base = (tick+1 < s.size()) ? s[tick+1] : 'd';
        std::string digits = s.substr(tick+2);
        digits.erase(std::remove(digits.begin(), digits.end(), '_'), digits.end());
        if (digits.empty()) return 0;
        // Replace x/X/z/Z/? with 0 for numeric conversion
        for (auto& c : digits) {
            if (c=='x'||c=='X'||c=='z'||c=='Z'||c=='?') c = '0';
        }
        try {
            if (base=='h'||base=='H') val = std::stoull(digits, nullptr, 16);
            else if (base=='b'||base=='B') val = std::stoull(digits, nullptr, 2);
            else if (base=='o'||base=='O') val = std::stoull(digits, nullptr, 8);
            else val = std::stoull(digits, nullptr, 10);
        } catch (...) {}
    } else if (!s.empty()) {
        try { val = std::stoull(s); } catch (...) {}
    }
    return val;
}

std::vector<NetId> BehavioralSynthesizer::const_to_bits(const std::string& val_str, int width, Netlist& nl) {
    uint64_t val = parse_number(val_str);
    std::vector<NetId> bits;
    for (int i = 0; i < width; i++)
        bits.push_back(make_const((val >> i) & 1, nl));
    return bits;
}

std::vector<NetId> BehavioralSynthesizer::pad_to_width(std::vector<NetId> bits, int width, Netlist& nl) {
    while ((int)bits.size() < width) bits.push_back(make_const(0, nl));
    if ((int)bits.size() > width) bits.resize(width);
    return bits;
}

int BehavioralSynthesizer::get_const_value(std::shared_ptr<AstNode> expr) const {
    if (!expr) return -1;
    if (expr->type == AstNodeType::NUMBER_LITERAL) return (int)parse_number(expr->value);
    return -1;
}

// ============================================================
// Expression width inference
// ============================================================
int BehavioralSynthesizer::infer_expr_width(std::shared_ptr<AstNode> expr) const {
    if (!expr) return 1;
    if (expr->type == AstNodeType::NUMBER_LITERAL) {
        auto tick = expr->value.find('\'');
        if (tick != std::string::npos && tick > 0) {
            try { return std::stoi(expr->value.substr(0, tick)); } catch (...) {}
        }
        return 1;
    }
    if (expr->type == AstNodeType::WIRE_DECL || (expr->children.empty() && expr->type != AstNodeType::NUMBER_LITERAL)) {
        std::string base = extract_base(expr->value);
        if (extract_bit(expr->value) >= 0) return 1;
        return get_bus_width(base);
    }
    if (expr->type == AstNodeType::CONCAT) {
        int w = 0;
        for (auto& ch : expr->children) w += infer_expr_width(ch);
        return w;
    }
    if (expr->type == AstNodeType::REPLICATE && !expr->children.empty()) {
        int n = expr->int_val > 0 ? expr->int_val : 1;
        return n * infer_expr_width(expr->children[0]);
    }
    if (expr->type == AstNodeType::TERNARY_OP && expr->children.size() >= 3)
        return std::max(infer_expr_width(expr->children[1]), infer_expr_width(expr->children[2]));
    if (expr->type == AstNodeType::MEM_READ) {
        auto mit = memory_arrays_.find(expr->value);
        if (mit != memory_arrays_.end()) return mit->second.first;
        return 1;
    }
    if (expr->type == AstNodeType::UNARY_OP) {
        if (expr->value == "~" || expr->value == "-" || expr->value == "signed" || expr->value == "unsigned")
            return expr->children.empty() ? 1 : infer_expr_width(expr->children[0]);
        return 1; // reduction
    }
    if (expr->type == AstNodeType::BIN_OP && expr->children.size() >= 2) {
        std::string op = expr->value;
        if (op=="=="||op=="!="||op=="<"||op==">"||op=="<="||op==">="||op=="&&"||op=="||") return 1;
        return std::max(infer_expr_width(expr->children[0]), infer_expr_width(expr->children[1]));
    }
    return 1;
}

// ============================================================
// Traversal
// ============================================================
void BehavioralSynthesizer::synth_node(std::shared_ptr<AstNode> node, Netlist& nl) {
    if (!node) return;
    if (node->type == AstNodeType::MODULE) {
        for (auto& child : node->children) synth_node(child, nl);
    } else if (node->type == AstNodeType::ALWAYS_POS_CLK || node->type == AstNodeType::ALWAYS_NEG_CLK) {
        synth_always(node, nl);
    } else if (node->type == AstNodeType::ALWAYS_COMB) {
        synth_always_comb(node, nl);
    }
}

// ============================================================
// synth_always — clocked: creates per-bit DFFs
// ============================================================
void BehavioralSynthesizer::synth_always(std::shared_ptr<AstNode> node, Netlist& nl) {
    BlockState state;
    // SV Phase 9: Parse "clk:rst:edge" format from value
    std::string clk_name, rst_name;
    bool rst_negedge = true;
    std::string val = node->value.empty() ? "clk" : node->value;
    auto colon1 = val.find(':');
    if (colon1 != std::string::npos) {
        clk_name = val.substr(0, colon1);
        auto colon2 = val.find(':', colon1 + 1);
        if (colon2 != std::string::npos) {
            rst_name = val.substr(colon1 + 1, colon2 - colon1 - 1);
            rst_negedge = (val.substr(colon2 + 1) == "neg");
        } else {
            rst_name = val.substr(colon1 + 1);
        }
    } else {
        clk_name = val;
    }
    state.clock_net = get_net(nl, clk_name);
    NetId rst_net = rst_name.empty() ? -1 : get_net(nl, rst_name);
    state.is_combinational = false;
    std::map<std::string, std::vector<NetId>> next_state;

    for (auto& child : node->children) {
        if (child->type == AstNodeType::BLOCK_BEGIN_END) {
            for (auto& stmt : child->children) synth_statement(stmt, nl, state, next_state);
        } else {
            synth_statement(child, nl, state, next_state);
        }
    }

    for (auto& pair : next_state) {
        const auto& name = pair.first;
        auto& d_bits = pair.second;
        int w = (int)d_bits.size();
        if (w == 1) {
            NetId q_net = get_net(nl, name);
            nl.add_dff(d_bits[0], state.clock_net, q_net, rst_net, "reg_" + name);
        } else {
            int lo = get_bus_lo(name);
            for (int i = 0; i < w; i++) {
                std::string bname = name + "[" + std::to_string(lo + i) + "]";
                NetId q_net = get_net(nl, bname);
                nl.add_dff(d_bits[i], state.clock_net, q_net, rst_net, "reg_" + bname);
            }
        }
    }
}

// ============================================================
// synth_always_comb — combinational: creates per-bit BUFs
// ============================================================
void BehavioralSynthesizer::synth_always_comb(std::shared_ptr<AstNode> node, Netlist& nl) {
    BlockState state;
    state.clock_net = -1;
    state.is_combinational = true;
    std::map<std::string, std::vector<NetId>> next_state;

    for (auto& child : node->children) {
        if (child->type == AstNodeType::BLOCK_BEGIN_END) {
            for (auto& stmt : child->children) synth_statement(stmt, nl, state, next_state);
        } else {
            synth_statement(child, nl, state, next_state);
        }
    }

    for (auto& pair : next_state) {
        const auto& name = pair.first;
        auto& bits = pair.second;
        int w = (int)bits.size();
        if (w == 1) {
            NetId target = get_net(nl, name);
            if (bits[0] != target) nl.add_gate(GateType::BUF, {bits[0]}, target, "comb_" + name);
        } else {
            int lo = get_bus_lo(name);
            for (int i = 0; i < w; i++) {
                std::string bname = name + "[" + std::to_string(lo + i) + "]";
                NetId target = get_net(nl, bname);
                if (bits[i] != target) nl.add_gate(GateType::BUF, {bits[i]}, target, "comb_" + bname);
            }
        }
    }
}

// ============================================================
// Helper: deep-clone AST subtree, replacing loop variable with constant
// ============================================================
static std::shared_ptr<AstNode> substitute_var(std::shared_ptr<AstNode> node,
    const std::string& var, int value) {
    if (!node) return nullptr;
    auto clone = std::make_shared<AstNode>();
    clone->type = node->type;
    clone->value = node->value;
    clone->int_val = node->int_val;

    // Replace identifier matching loop variable with NUMBER_LITERAL
    if ((node->type == AstNodeType::WIRE_DECL ||
         (node->children.empty() && node->type != AstNodeType::NUMBER_LITERAL)) &&
        node->value == var) {
        clone->type = AstNodeType::NUMBER_LITERAL;
        clone->value = std::to_string(value);
        clone->int_val = value;
        return clone;
    }

    for (auto& child : node->children) {
        clone->children.push_back(substitute_var(child, var, value));
    }
    return clone;
}

// ============================================================
// extract_loop_bounds — parse FOR_LOOP children for static bounds
// ============================================================
BehavioralSynthesizer::LoopBounds BehavioralSynthesizer::extract_loop_bounds(
    std::shared_ptr<AstNode> for_node) const {
    LoopBounds bounds;
    bounds.var_name = "";
    bounds.start = -1;
    bounds.end = -1;
    bounds.step = 1;

    if (!for_node || for_node->children.size() < 4) return bounds;

    // child[0] = init assignment (e.g., i = 0)
    auto init = for_node->children[0];
    if (init && (init->type == AstNodeType::BLOCK_ASSIGN ||
                 init->type == AstNodeType::NONBLOCK_ASSIGN) &&
        !init->children.empty()) {
        bounds.var_name = init->value;
        bounds.start = get_const_value(init->children[0]);
    }

    // child[1] = condition (e.g., i < 8)
    auto cond = for_node->children[1];
    if (cond && cond->type == AstNodeType::BIN_OP && cond->children.size() >= 2) {
        int limit = get_const_value(cond->children[1]);
        if (limit >= 0) {
            if (cond->value == "<") bounds.end = limit;
            else if (cond->value == "<=") bounds.end = limit + 1;
            else if (cond->value == ">") bounds.end = limit;
            else if (cond->value == ">=") bounds.end = limit;
        }
    }

    // child[2] = increment (e.g., i = i + 1)
    auto incr = for_node->children[2];
    if (incr && (incr->type == AstNodeType::BLOCK_ASSIGN ||
                 incr->type == AstNodeType::NONBLOCK_ASSIGN) &&
        !incr->children.empty()) {
        auto expr = incr->children[0];
        if (expr && expr->type == AstNodeType::BIN_OP && expr->children.size() >= 2) {
            if (expr->value == "+") {
                int step_val = get_const_value(expr->children[1]);
                if (step_val > 0) bounds.step = step_val;
            } else if (expr->value == "-") {
                int step_val = get_const_value(expr->children[1]);
                if (step_val > 0) bounds.step = -step_val;
            }
        }
    }

    return bounds;
}

// ============================================================
// synth_for_loop — unroll FOR_LOOP with static bounds
// ============================================================
void BehavioralSynthesizer::synth_for_loop(std::shared_ptr<AstNode> for_node, Netlist& nl,
    const BlockState& state, std::map<std::string, std::vector<NetId>>& next_state) {
    if (!for_node || for_node->children.size() < 4) return;

    auto bounds = extract_loop_bounds(for_node);
    auto body = for_node->children[3];

    if (bounds.start >= 0 && bounds.end >= 0 && bounds.step != 0) {
        // Static bounds: fully unroll
        const int max_unroll = 256;
        int iterations = 0;

        if (bounds.step > 0) {
            for (int i = bounds.start; i < bounds.end && iterations < max_unroll;
                 i += bounds.step, iterations++) {
                auto substituted = substitute_var(body, bounds.var_name, i);
                synth_statement(substituted, nl, state, next_state);
            }
        } else {
            for (int i = bounds.start; i > bounds.end && iterations < max_unroll;
                 i += bounds.step, iterations++) {
                auto substituted = substitute_var(body, bounds.var_name, i);
                synth_statement(substituted, nl, state, next_state);
            }
        }
        stats_.loops_unrolled++;
    } else {
        // Dynamic bounds: create MUXed iteration chain (up to configurable max)
        const int max_dynamic = 256;
        if (!bounds.var_name.empty() && body) {
            int effective_start = (bounds.start >= 0) ? bounds.start : 0;
            int effective_step = (bounds.step != 0) ? std::abs(bounds.step) : 1;
            for (int i = 0; i < max_dynamic; i++) {
                int iter_val = effective_start + i * effective_step;
                auto substituted = substitute_var(body, bounds.var_name, iter_val);

                // Create a condition: loop_var < bounds (if dynamic, compare against runtime)
                auto cond_state = next_state;
                synth_statement(substituted, nl, state, cond_state);

                // MUX: if iteration index < dynamic bound, use updated state
                auto idx_bits = const_to_bits(std::to_string(iter_val), 32, nl);
                auto bound_node = for_node->children[1];
                if (bound_node && bound_node->type == AstNodeType::BIN_OP &&
                    bound_node->children.size() >= 2) {
                    int bound_w = infer_expr_width(bound_node->children[1]);
                    if (bound_w < 1) bound_w = 32;
                    auto bound_bits = synth_expr_bus(bound_node->children[1], nl, next_state,
                                                     bound_w);
                    idx_bits.resize(bound_w);
                    for (int b = (int)idx_bits.size(); b < bound_w; b++)
                        idx_bits.push_back(make_const(0, nl));
                    NetId in_range = build_bus_lt(idx_bits, bound_bits, nl);

                    for (auto& p : cond_state) {
                        if (next_state.find(p.first) == next_state.end() ||
                            next_state[p.first] != p.second) {
                            auto fv = next_state.count(p.first) ?
                                      next_state[p.first] :
                                      get_bus_nets(nl, p.first, next_state);
                            next_state[p.first] = build_bus_mux(in_range, p.second, fv, nl);
                        }
                    }
                }
            }
            stats_.loops_unrolled++;
        }
    }
}

// ============================================================
// try_share_resource — reuse operators under mutually exclusive conditions
// GateType::AND represents MUL, GateType::XOR represents ADD
// ============================================================
std::vector<NetId> BehavioralSynthesizer::try_share_resource(GateType op, int width,
    const std::vector<NetId>& a, const std::vector<NetId>& b,
    NetId condition, Netlist& nl,
    std::map<std::string, std::vector<NetId>>& next_state) {

    // Only share MUL (AND) and ADD (XOR) composite operations
    if (op != GateType::AND && op != GateType::XOR) {
        if (op == GateType::AND)
            return pad_to_width(build_bus_multiplier(a, b, nl), width, nl);
        return pad_to_width(build_bus_adder(a, b, nl), width, nl);
    }

    // Check for existing resource of same type and width with mutually exclusive condition
    for (auto& res : shared_resources_) {
        if (res.op_type == op && res.width == width &&
            condition >= 0 && res.condition >= 0 && condition != res.condition) {
            // Found shareable resource — MUX inputs to reuse operator
            auto muxed_a = build_bus_mux(condition, a, res.inputs_a, nl);
            auto muxed_b = build_bus_mux(condition, b, res.inputs_b, nl);

            std::vector<NetId> result;
            if (op == GateType::AND)
                result = pad_to_width(build_bus_multiplier(muxed_a, muxed_b, nl), width, nl);
            else
                result = pad_to_width(build_bus_adder(muxed_a, muxed_b, nl), width, nl);

            res.usage_count++;
            res.inputs_a = muxed_a;
            res.inputs_b = muxed_b;
            res.output = result;
            stats_.resources_shared++;
            return result;
        }
    }

    // No existing resource found — create new and register
    std::vector<NetId> result;
    if (op == GateType::AND)
        result = pad_to_width(build_bus_multiplier(a, b, nl), width, nl);
    else
        result = pad_to_width(build_bus_adder(a, b, nl), width, nl);

    SharedResource res;
    res.op_type = op;
    res.width = width;
    res.inputs_a = a;
    res.inputs_b = b;
    res.output = result;
    res.condition = condition;
    res.usage_count = 1;
    shared_resources_.push_back(res);

    return result;
}

// ============================================================
// chain_operations — build balanced tree for associative operator chains
// e.g., a + b + c + d → (a+b) + (c+d) instead of ((a+b)+c)+d
// ============================================================
std::vector<NetId> BehavioralSynthesizer::chain_operations(std::shared_ptr<AstNode> expr,
    Netlist& nl, const std::map<std::string, std::vector<NetId>>& next_state, int target_w) {
    if (!expr || expr->type != AstNodeType::BIN_OP || expr->children.size() < 2) {
        return synth_expr_bus(expr, nl, next_state, target_w);
    }

    const auto& op = expr->value;
    // Only chain associative operators
    if (op != "+" && op != "*" && op != "&" && op != "|" && op != "^") {
        return synth_expr_bus(expr, nl, next_state, target_w);
    }

    // Flatten chain of same operator into leaf operands
    std::vector<std::shared_ptr<AstNode>> operands;
    std::function<void(std::shared_ptr<AstNode>)> flatten = [&](std::shared_ptr<AstNode> n) {
        if (n && n->type == AstNodeType::BIN_OP && n->value == op && n->children.size() >= 2) {
            flatten(n->children[0]);
            flatten(n->children[1]);
        } else {
            operands.push_back(n);
        }
    };
    flatten(expr);

    if (operands.size() <= 2) {
        // Not a chain worth balancing
        return synth_expr_bus(expr, nl, next_state, target_w);
    }

    // Synthesize all leaf operands
    std::vector<std::vector<NetId>> values;
    for (auto& operand : operands) {
        values.push_back(synth_expr_bus(operand, nl, next_state, target_w));
    }

    // Build balanced binary tree
    while (values.size() > 1) {
        std::vector<std::vector<NetId>> next_level;
        for (size_t i = 0; i + 1 < values.size(); i += 2) {
            std::vector<NetId> combined;
            if (op == "+")
                combined = pad_to_width(build_bus_adder(values[i], values[i+1], nl), target_w, nl);
            else if (op == "*")
                combined = pad_to_width(build_bus_multiplier(values[i], values[i+1], nl), target_w, nl);
            else if (op == "&")
                combined = bitwise_op(values[i], values[i+1], GateType::AND, nl);
            else if (op == "|")
                combined = bitwise_op(values[i], values[i+1], GateType::OR, nl);
            else
                combined = bitwise_op(values[i], values[i+1], GateType::XOR, nl);
            next_level.push_back(combined);
        }
        if (values.size() % 2 == 1) next_level.push_back(values.back());
        values = next_level;
    }

    stats_.operators_chained++;
    return pad_to_width(values[0], target_w, nl);
}

// ============================================================
// synth_statement — bus-aware statement synthesis
// ============================================================
void BehavioralSynthesizer::synth_statement(std::shared_ptr<AstNode> stmt, Netlist& nl,
    const BlockState& state, std::map<std::string, std::vector<NetId>>& next_state) {
    if (!stmt) return;

    if (stmt->type == AstNodeType::NONBLOCK_ASSIGN || stmt->type == AstNodeType::BLOCK_ASSIGN) {
        std::string target_name = stmt->value;
        if (stmt->children.empty()) return;
        std::string base = extract_base(target_name);
        int bit_idx = extract_bit(target_name);

        if (bit_idx >= 0) {
            // Single-bit assign to bus bit: q[3] <= expr
            NetId d = synth_expr_single(stmt->children[0], nl, next_state);
            int lo = get_bus_lo(base);
            if (next_state.find(base) == next_state.end())
                next_state[base] = get_bus_nets(nl, base, {});
            int idx = bit_idx - lo;
            if (idx >= 0 && idx < (int)next_state[base].size())
                next_state[base][idx] = d;
        } else {
            int w = get_bus_width(target_name);
            if (w > 1) {
                next_state[target_name] = synth_expr_bus(stmt->children[0], nl, next_state, w);
            } else {
                NetId d = synth_expr_single(stmt->children[0], nl, next_state);
                next_state[target_name] = {d};
            }
        }
    } else if (stmt->type == AstNodeType::BLOCK_BEGIN_END) {
        for (auto& child : stmt->children) synth_statement(child, nl, state, next_state);
    } else if (stmt->type == AstNodeType::MEM_WRITE) {
        // Memory write: mem[addr] <= data
        // For synthesis: expand into per-location write with decoder
        synth_mem_write(stmt->value, stmt->children[0], stmt->children[1], nl, next_state);
    } else if (stmt->type == AstNodeType::IF_ELSE) {
        if (stmt->children.empty()) return;
        NetId cond = synth_expr_single(stmt->children[0], nl, next_state);

        auto true_state = next_state;
        if (stmt->children.size() > 1) synth_statement(stmt->children[1], nl, state, true_state);
        auto false_state = next_state;
        if (stmt->children.size() > 2) synth_statement(stmt->children[2], nl, state, false_state);

        std::set<std::string> modified;
        for (auto& p : true_state)
            if (next_state.find(p.first) == next_state.end() || next_state[p.first] != p.second)
                modified.insert(p.first);
        for (auto& p : false_state)
            if (next_state.find(p.first) == next_state.end() || next_state[p.first] != p.second)
                modified.insert(p.first);

        for (const auto& var : modified) {
            auto tv = true_state.count(var) ? true_state[var] : get_bus_nets(nl, var, next_state);
            auto fv = false_state.count(var) ? false_state[var] : get_bus_nets(nl, var, next_state);
            next_state[var] = build_bus_mux(cond, tv, fv, nl);
        }
    } else if (stmt->type == AstNodeType::CASE_STMT) {
        if (stmt->children.empty()) return;
        auto sel_expr = stmt->children[0];
        int sel_w = infer_expr_width(sel_expr);
        bool is_casex = (stmt->value == "casex");
        bool is_casez = (stmt->value == "casez");

        std::vector<std::pair<std::string, std::shared_ptr<AstNode>>> items;
        std::shared_ptr<AstNode> default_body;
        for (size_t ci = 1; ci < stmt->children.size(); ci++) {
            auto& item = stmt->children[ci];
            if (item->type == AstNodeType::CASE_ITEM) {
                if (item->value == "default") {
                    if (!item->children.empty()) default_body = item->children[0];
                } else {
                    if (!item->children.empty()) items.push_back({item->value, item->children[0]});
                }
            }
        }

        auto result_state = next_state;
        if (default_body) synth_statement(default_body, nl, state, result_state);

        for (int ci = (int)items.size()-1; ci >= 0; ci--) {
            NetId eq_net;
            std::string item_str = items[ci].first;

            // Compute don't-care mask for casex/casez
            // casex: X, x, Z, z, ? are don't-care
            // casez: Z, z, ? are don't-care
            uint64_t dc_mask = 0;
            if (is_casex || is_casez) {
                std::string bits_only;
                // Extract bit pattern from Verilog literal (e.g., 8'b10x1_01?z)
                size_t apos = item_str.find('\'');
                if (apos != std::string::npos && apos + 1 < item_str.size()) {
                    char base = item_str[apos + 1];
                    std::string digits = item_str.substr(apos + 2);
                    // Remove underscores
                    digits.erase(std::remove(digits.begin(), digits.end(), '_'), digits.end());
                    if (base == 'b' || base == 'B') {
                        for (int bi = 0; bi < (int)digits.size(); bi++) {
                            char c = digits[digits.size() - 1 - bi];
                            bool is_dc = false;
                            if (is_casex && (c=='x'||c=='X'||c=='z'||c=='Z'||c=='?')) is_dc = true;
                            if (is_casez && (c=='z'||c=='Z'||c=='?')) is_dc = true;
                            if (is_dc) dc_mask |= (1ULL << bi);
                        }
                    }
                }
            }

            if (sel_w > 1) {
                auto sel_bits = synth_expr_bus(sel_expr, nl, next_state, sel_w);
                auto val_bits = const_to_bits(item_str, sel_w, nl);

                if (dc_mask) {
                    // casex/casez: only compare non-don't-care bits
                    // eq = AND of (sel[i] XNOR val[i]) for all non-dc bits
                    NetId running_eq = make_const(1, nl);
                    for (int bi = 0; bi < sel_w; bi++) {
                        if (dc_mask & (1ULL << bi)) continue; // skip don't-care bits
                        NetId xo = nl.add_net("casex_xor_" + uid());
                        nl.add_gate(GateType::XOR, {sel_bits[bi], val_bits[bi]}, xo, "casex_xor");
                        NetId xnor_out = nl.add_net("casex_xnor_" + uid());
                        nl.add_gate(GateType::NOT, {xo}, xnor_out, "casex_xnor");
                        NetId new_eq = nl.add_net("casex_and_" + uid());
                        nl.add_gate(GateType::AND, {running_eq, xnor_out}, new_eq, "casex_and");
                        running_eq = new_eq;
                    }
                    eq_net = running_eq;
                } else {
                    eq_net = build_bus_eq(sel_bits, val_bits, nl);
                }
            } else {
                NetId sel_net = synth_expr_single(sel_expr, nl, next_state);
                NetId val_net;
                try { val_net = make_const(parse_number(item_str) & 1, nl); }
                catch (...) { val_net = get_net(nl, item_str); }
                NetId xo = nl.add_net("case_xor_" + uid());
                nl.add_gate(GateType::XOR, {sel_net, val_net}, xo, "case_xor");
                eq_net = nl.add_net("case_eq_" + uid());
                nl.add_gate(GateType::NOT, {xo}, eq_net, "case_eq_inv");
            }

            auto item_state = next_state;
            synth_statement(items[ci].second, nl, state, item_state);

            std::set<std::string> mvars;
            for (auto& p : item_state)
                if (result_state.find(p.first)==result_state.end() || result_state[p.first]!=p.second)
                    mvars.insert(p.first);
            for (const auto& v : mvars) {
                auto tv = item_state.count(v) ? item_state[v] : get_bus_nets(nl, v, next_state);
                auto fv = result_state.count(v) ? result_state[v] : get_bus_nets(nl, v, next_state);
                result_state[v] = build_bus_mux(eq_net, tv, fv, nl);
            }
        }
        next_state = result_state;
    } else if (stmt->type == AstNodeType::FOR_LOOP) {
        synth_for_loop(stmt, nl, state, next_state);
    }
}

// ============================================================
// synth_expr_single — single-bit context (conditions, 1-bit signals)
// ============================================================
NetId BehavioralSynthesizer::synth_expr_single(std::shared_ptr<AstNode> expr, Netlist& nl,
    const std::map<std::string, std::vector<NetId>>& next_state) {
    if (!expr) return -1;

    // Identifier / wire
    if (expr->type == AstNodeType::WIRE_DECL ||
        (expr->children.empty() && expr->type != AstNodeType::NUMBER_LITERAL)) {
        std::string name = expr->value;
        std::string base = extract_base(name);
        int bit = extract_bit(name);
        auto ns_it = next_state.find(base);
        if (ns_it != next_state.end()) {
            if (bit >= 0) {
                int idx = bit - get_bus_lo(base);
                if (idx >= 0 && idx < (int)ns_it->second.size()) return ns_it->second[idx];
            } else if (ns_it->second.size() == 1) return ns_it->second[0];
            else return reduce(ns_it->second, "|", nl); // bus in 1-bit context: OR-reduce (Verilog boolean semantics)
        }
        return get_net(nl, name);
    }

    // Number
    if (expr->type == AstNodeType::NUMBER_LITERAL)
        return make_const(parse_number(expr->value) & 1, nl);

    // Unary
    if (expr->type == AstNodeType::UNARY_OP && !expr->children.empty()) {
        const auto& op = expr->value;
        // Reduction: &, |, ^
        if (op == "&" || op == "|" || op == "^" ||
            op == "red_&" || op == "red_|" || op == "red_^") {
            std::string red_op = op;
            if (red_op.substr(0, 4) == "red_") red_op = red_op.substr(4);
            int w = infer_expr_width(expr->children[0]);
            if (w > 1) {
                auto bits = synth_expr_bus(expr->children[0], nl, next_state, w);
                return reduce(bits, red_op, nl);
            }
            return synth_expr_single(expr->children[0], nl, next_state);
        }
        // Reduction inverted: ~&, ~|, ~^
        if (op == "red_n&" || op == "red_n|" || op == "red_n^") {
            std::string red_op(1, op.back()); // &, |, or ^
            int w = infer_expr_width(expr->children[0]);
            NetId reduced;
            if (w > 1) {
                auto bits = synth_expr_bus(expr->children[0], nl, next_state, w);
                reduced = reduce(bits, red_op, nl);
            } else {
                reduced = synth_expr_single(expr->children[0], nl, next_state);
            }
            NetId out = nl.add_net("redn_" + uid());
            nl.add_gate(GateType::NOT, {reduced}, out, "beh_red_inv");
            return out;
        }
        NetId operand = synth_expr_single(expr->children[0], nl, next_state);
        if (op == "~" || op == "!") {
            NetId out = nl.add_net("not_" + uid());
            nl.add_gate(GateType::NOT, {operand}, out, "beh_not"); return out;
        }
        if (op == "-") {
            NetId out = nl.add_net("neg_" + uid());
            nl.add_gate(GateType::NOT, {operand}, out, "beh_neg"); return out;
        }
        return operand;
    }

    // Ternary
    if (expr->type == AstNodeType::TERNARY_OP && expr->children.size() >= 3) {
        NetId sel = synth_expr_single(expr->children[0], nl, next_state);
        NetId tv = synth_expr_single(expr->children[1], nl, next_state);
        NetId fv = synth_expr_single(expr->children[2], nl, next_state);
        return build_mux2(sel, tv, fv, nl);
    }

    // Binary
    if (expr->type == AstNodeType::BIN_OP && expr->children.size() >= 2) {
        const auto& op = expr->value;
        int lw = infer_expr_width(expr->children[0]);
        int rw = infer_expr_width(expr->children[1]);
        int maxw = std::max(lw, rw);

        // Multi-bit comparisons
        if ((op=="=="||op=="!="||op=="<"||op==">"||op=="<="||op==">=") && maxw > 1) {
            auto lb = synth_expr_bus(expr->children[0], nl, next_state, maxw);
            auto rb = synth_expr_bus(expr->children[1], nl, next_state, maxw);

            // Check if operands are signed (wrapped in $signed or declared as signed reg/wire)
            bool is_signed = false;
            if (expr->children[0]->type == AstNodeType::UNARY_OP && expr->children[0]->value == "signed") is_signed = true;
            if (expr->children[1]->type == AstNodeType::UNARY_OP && expr->children[1]->value == "signed") is_signed = true;
            // Auto-detect from signed declarations
            if (!is_signed && expr->children[0]->type == AstNodeType::WIRE_DECL &&
                signed_signals_.count(extract_base(expr->children[0]->value))) is_signed = true;
            if (!is_signed && expr->children[1]->type == AstNodeType::WIRE_DECL &&
                signed_signals_.count(extract_base(expr->children[1]->value))) is_signed = true;

            if (op == "==") return build_bus_eq(lb, rb, nl);
            if (op == "!=") {
                NetId eq = build_bus_eq(lb, rb, nl);
                NetId out = nl.add_net("neq_" + uid());
                nl.add_gate(GateType::NOT, {eq}, out, "beh_neq"); return out;
            }
            if (op == "<")  return is_signed ? build_bus_lt_signed(lb, rb, nl) : build_bus_lt(lb, rb, nl);
            if (op == ">")  return is_signed ? build_bus_lt_signed(rb, lb, nl) : build_bus_lt(rb, lb, nl);
            if (op == "<=") {
                NetId lt = is_signed ? build_bus_lt_signed(lb, rb, nl) : build_bus_lt(lb, rb, nl);
                NetId eq = build_bus_eq(lb, rb, nl);
                NetId out = nl.add_net("lte_" + uid());
                nl.add_gate(GateType::OR, {lt, eq}, out, "beh_lte"); return out;
            }
            if (op == ">=") {
                NetId gt = is_signed ? build_bus_lt_signed(rb, lb, nl) : build_bus_lt(rb, lb, nl);
                NetId eq = build_bus_eq(lb, rb, nl);
                NetId out = nl.add_net("gte_" + uid());
                nl.add_gate(GateType::OR, {gt, eq}, out, "beh_gte"); return out;
            }
        }

        // Logical &&/|| on buses
        if ((op == "&&" || op == "||") && (lw > 1 || rw > 1)) {
            NetId l1 = (lw > 1) ? reduce(synth_expr_bus(expr->children[0], nl, next_state, lw), "|", nl)
                                : synth_expr_single(expr->children[0], nl, next_state);
            NetId r1 = (rw > 1) ? reduce(synth_expr_bus(expr->children[1], nl, next_state, rw), "|", nl)
                                : synth_expr_single(expr->children[1], nl, next_state);
            GateType gt = (op == "&&") ? GateType::AND : GateType::OR;
            NetId out = nl.add_net("log_" + uid());
            nl.add_gate(gt, {l1, r1}, out, "beh_log"); return out;
        }

        // Single-bit binary
        NetId lhs = synth_expr_single(expr->children[0], nl, next_state);
        NetId rhs = synth_expr_single(expr->children[1], nl, next_state);
        if (op == "+") return build_adder(lhs, rhs, nl);
        if (op == "-") return build_subtractor(lhs, rhs, nl);
        if (op == "*") { NetId out = nl.add_net("mul_" + uid()); nl.add_gate(GateType::AND, {lhs, rhs}, out, "beh_mul"); return out; }
        if (op == "^") { NetId out = nl.add_net("xor_" + uid()); nl.add_gate(GateType::XOR, {lhs, rhs}, out, "beh_xor"); return out; }
        if (op == "~^" || op == "^~") { NetId out = nl.add_net("xnor_" + uid()); nl.add_gate(GateType::XNOR, {lhs, rhs}, out, "beh_xnor"); return out; }
        if (op == "&" || op == "&&") { NetId out = nl.add_net("and_" + uid()); nl.add_gate(GateType::AND, {lhs, rhs}, out, "beh_and"); return out; }
        if (op == "|" || op == "||") { NetId out = nl.add_net("or_" + uid()); nl.add_gate(GateType::OR, {lhs, rhs}, out, "beh_or"); return out; }
        if (op == "==") {
            NetId x = nl.add_net("eq_x_" + uid()); nl.add_gate(GateType::XOR, {lhs, rhs}, x, "beh_eq_xor");
            NetId out = nl.add_net("eq_" + uid()); nl.add_gate(GateType::NOT, {x}, out, "beh_eq_inv"); return out;
        }
        if (op == "!=") { NetId out = nl.add_net("neq_" + uid()); nl.add_gate(GateType::XOR, {lhs, rhs}, out, "beh_neq"); return out; }
        if (op == "<") return build_comparator_lt(lhs, rhs, nl);
        if (op == ">") return build_comparator_gt(lhs, rhs, nl);
        if (op == "<=" || op == ">=") {
            NetId c = (op=="<=") ? build_comparator_lt(lhs,rhs,nl) : build_comparator_gt(lhs,rhs,nl);
            NetId x = nl.add_net("cmp_x_" + uid()); nl.add_gate(GateType::XOR, {lhs,rhs}, x, "cmp_xor");
            NetId e = nl.add_net("cmp_e_" + uid()); nl.add_gate(GateType::NOT, {x}, e, "cmp_eq");
            NetId out = nl.add_net("cmp_" + uid()); nl.add_gate(GateType::OR, {c,e}, out, "cmp_or"); return out;
        }
    }
    return -1;
}

// ============================================================
// synth_expr_bus — multi-bit expression synthesis
// ============================================================
std::vector<NetId> BehavioralSynthesizer::synth_expr_bus(std::shared_ptr<AstNode> expr, Netlist& nl,
    const std::map<std::string, std::vector<NetId>>& next_state, int target_w) {
    if (!expr) return pad_to_width({}, target_w, nl);

    // Identifier / wire
    if (expr->type == AstNodeType::WIRE_DECL ||
        (expr->children.empty() && expr->type != AstNodeType::NUMBER_LITERAL)) {
        std::string name = expr->value;
        std::string base = extract_base(name);
        int bit = extract_bit(name);
        if (bit >= 0) {
            auto ns_it = next_state.find(base);
            NetId b;
            if (ns_it != next_state.end()) {
                int idx = bit - get_bus_lo(base);
                b = (idx >= 0 && idx < (int)ns_it->second.size()) ? ns_it->second[idx] : get_net(nl, name);
            } else b = get_net(nl, name);
            return pad_to_width({b}, target_w, nl);
        }
        auto bits = get_bus_nets(nl, name, next_state);
        return pad_to_width(bits, target_w, nl);
    }

    // Number literal
    if (expr->type == AstNodeType::NUMBER_LITERAL)
        return const_to_bits(expr->value, target_w, nl);

    // Concatenation
    if (expr->type == AstNodeType::CONCAT) {
        std::vector<NetId> result;
        for (int i = (int)expr->children.size()-1; i >= 0; i--) {
            int cw = infer_expr_width(expr->children[i]);
            auto cbits = synth_expr_bus(expr->children[i], nl, next_state, cw);
            result.insert(result.end(), cbits.begin(), cbits.end());
        }
        return pad_to_width(result, target_w, nl);
    }

    // Replication
    if (expr->type == AstNodeType::REPLICATE && !expr->children.empty()) {
        int n = expr->int_val > 0 ? expr->int_val : 1;
        int cw = infer_expr_width(expr->children[0]);
        auto part = synth_expr_bus(expr->children[0], nl, next_state, cw);
        std::vector<NetId> result;
        for (int i = 0; i < n; i++) result.insert(result.end(), part.begin(), part.end());
        return pad_to_width(result, target_w, nl);
    }

    // Ternary
    if (expr->type == AstNodeType::TERNARY_OP && expr->children.size() >= 3) {
        // Detect tri-state pattern: en ? data : {W}'bz or en ? data : 1'bz
        auto& false_expr = expr->children[2];
        bool is_tristate = false;
        if (false_expr->type == AstNodeType::NUMBER_LITERAL) {
            const auto& v = false_expr->value;
            // Check for z/Z in the literal (e.g., 1'bz, 8'bzzzzzzzz, 1'bZ)
            for (char c : v) { if (c == 'z' || c == 'Z') { is_tristate = true; break; } }
        }
        if (is_tristate) {
            // Tri-state buffer: en selects data to output, else high-Z
            NetId en = synth_expr_single(expr->children[0], nl, next_state);
            auto data_bits = synth_expr_bus(expr->children[1], nl, next_state, target_w);
            std::vector<NetId> result;
            for (int i = 0; i < target_w; i++) {
                NetId d = (i < (int)data_bits.size()) ? data_bits[i] : make_const(0, nl);
                NetId out = nl.add_net("tri_" + uid());
                nl.add_gate(GateType::TRI, {d, en}, out, "tristate");
                result.push_back(out);
            }
            return result;
        }
        NetId sel = synth_expr_single(expr->children[0], nl, next_state);
        auto t_bits = synth_expr_bus(expr->children[1], nl, next_state, target_w);
        auto f_bits = synth_expr_bus(expr->children[2], nl, next_state, target_w);
        return build_bus_mux(sel, t_bits, f_bits, nl);
    }

    // Memory read: mem[addr] → MUX tree selecting word by address
    if (expr->type == AstNodeType::MEM_READ && !expr->children.empty()) {
        return synth_mem_read(expr->value, expr->children[0], nl, next_state, target_w);
    }

    // Unary
    if (expr->type == AstNodeType::UNARY_OP && !expr->children.empty()) {
        const auto& op = expr->value;
        if (op == "~") {
            auto operand = synth_expr_bus(expr->children[0], nl, next_state, target_w);
            std::vector<NetId> result;
            for (auto b : operand) {
                NetId out = nl.add_net("bnot_" + uid());
                nl.add_gate(GateType::NOT, {b}, out, "beh_bnot");
                result.push_back(out);
            }
            return result;
        }
        if (op == "-") {
            auto operand = synth_expr_bus(expr->children[0], nl, next_state, target_w);
            std::vector<NetId> inv;
            for (auto b : operand) {
                NetId out = nl.add_net("bneg_" + uid());
                nl.add_gate(GateType::NOT, {b}, out, "beh_bneg");
                inv.push_back(out);
            }
            auto one = const_to_bits("1", target_w, nl);
            return build_bus_adder(inv, one, nl);
        }
        if (op == "&" || op == "|" || op == "^" ||
            op == "red_&" || op == "red_|" || op == "red_^") {
            std::string red_op = op;
            if (red_op.substr(0, 4) == "red_") red_op = red_op.substr(4);
            int w = infer_expr_width(expr->children[0]);
            auto bits = synth_expr_bus(expr->children[0], nl, next_state, w);
            NetId r = reduce(bits, red_op, nl);
            return pad_to_width({r}, target_w, nl);
        }
        if (op == "red_n&" || op == "red_n|" || op == "red_n^") {
            std::string red_op(1, op.back());
            int w = infer_expr_width(expr->children[0]);
            auto bits = synth_expr_bus(expr->children[0], nl, next_state, w);
            NetId r = reduce(bits, red_op, nl);
            NetId inv = nl.add_net("redn_bus_" + uid());
            nl.add_gate(GateType::NOT, {r}, inv, "beh_red_inv");
            return pad_to_width({inv}, target_w, nl);
        }
        if (op == "!") {
            NetId s = synth_expr_single(expr->children[0], nl, next_state);
            NetId out = nl.add_net("lnot_" + uid());
            nl.add_gate(GateType::NOT, {s}, out, "beh_lnot");
            return pad_to_width({out}, target_w, nl);
        }
        // $signed/$unsigned: pass-through (signedness handled at comparison level)
        if (op == "signed" || op == "unsigned") {
            return synth_expr_bus(expr->children[0], nl, next_state, target_w);
        }
    }

    // Binary
    if (expr->type == AstNodeType::BIN_OP && expr->children.size() >= 2) {
        const auto& op = expr->value;
        int lw = infer_expr_width(expr->children[0]);
        int rw = infer_expr_width(expr->children[1]);
        int maxw = std::max({lw, rw, target_w});

        // Operator chaining: detect chains of same associative operator
        if ((op == "+" || op == "*" || op == "&" || op == "|" || op == "^") &&
            expr->children[0]->type == AstNodeType::BIN_OP &&
            expr->children[0]->value == op) {
            return chain_operations(expr, nl, next_state, target_w);
        }

        if (op == "+") {
            auto la = synth_expr_bus(expr->children[0], nl, next_state, maxw);
            auto ra = synth_expr_bus(expr->children[1], nl, next_state, maxw);
            // Resource sharing for ADD (GateType::XOR represents composite ADD)
            auto ns_copy = const_cast<std::map<std::string, std::vector<NetId>>&>(next_state);
            return try_share_resource(GateType::XOR, target_w, la, ra, -1, nl, ns_copy);
        }
        if (op == "-") {
            auto la = synth_expr_bus(expr->children[0], nl, next_state, maxw);
            auto ra = synth_expr_bus(expr->children[1], nl, next_state, maxw);
            return pad_to_width(build_bus_subtractor(la, ra, nl), target_w, nl);
        }
        if (op == "*") {
            auto la = synth_expr_bus(expr->children[0], nl, next_state, lw);
            auto ra = synth_expr_bus(expr->children[1], nl, next_state, rw);
            // Resource sharing for MUL (GateType::AND represents composite MUL)
            auto ns_copy = const_cast<std::map<std::string, std::vector<NetId>>&>(next_state);
            return try_share_resource(GateType::AND, target_w, la, ra, -1, nl, ns_copy);
        }
        if (op == "<<") {
            auto data = synth_expr_bus(expr->children[0], nl, next_state, target_w);
            int amt = get_const_value(expr->children[1]);
            if (amt >= 0) return build_bus_shift_left(data, amt, nl);
            auto shamt = synth_expr_bus(expr->children[1], nl, next_state, rw);
            return build_barrel_shift_left(data, shamt, nl);
        }
        if (op == ">>") {
            auto data = synth_expr_bus(expr->children[0], nl, next_state, target_w);
            int amt = get_const_value(expr->children[1]);
            if (amt >= 0) return build_bus_shift_right(data, amt, nl);
            auto shamt = synth_expr_bus(expr->children[1], nl, next_state, rw);
            return build_barrel_shift_right(data, shamt, nl);
        }
        if (op == ">>>") {
            // Arithmetic right shift: shift right + sign-extend
            auto data = synth_expr_bus(expr->children[0], nl, next_state, target_w);
            int amt = get_const_value(expr->children[1]);
            if (amt >= 0) {
                return build_arith_shift_right(data, amt, nl);
            }
            auto shamt = synth_expr_bus(expr->children[1], nl, next_state, rw);
            return build_barrel_arith_shift_right(data, shamt, nl);
        }
        if (op == "/") {
            int lw = infer_expr_width(expr->children[0]), rw2 = infer_expr_width(expr->children[1]);
            auto la = synth_expr_bus(expr->children[0], nl, next_state, lw);
            auto ra = synth_expr_bus(expr->children[1], nl, next_state, rw2);
            auto result = build_divider(la, ra, nl);
            return pad_to_width(result.first, target_w, nl); // quotient
        }
        if (op == "%") {
            int lw = infer_expr_width(expr->children[0]), rw2 = infer_expr_width(expr->children[1]);
            auto la = synth_expr_bus(expr->children[0], nl, next_state, lw);
            auto ra = synth_expr_bus(expr->children[1], nl, next_state, rw2);
            auto result = build_divider(la, ra, nl);
            return pad_to_width(result.second, target_w, nl); // remainder
        }
        if (op == "&") {
            auto la = synth_expr_bus(expr->children[0], nl, next_state, target_w);
            auto ra = synth_expr_bus(expr->children[1], nl, next_state, target_w);
            return bitwise_op(la, ra, GateType::AND, nl);
        }
        if (op == "|") {
            auto la = synth_expr_bus(expr->children[0], nl, next_state, target_w);
            auto ra = synth_expr_bus(expr->children[1], nl, next_state, target_w);
            return bitwise_op(la, ra, GateType::OR, nl);
        }
        if (op == "^") {
            auto la = synth_expr_bus(expr->children[0], nl, next_state, target_w);
            auto ra = synth_expr_bus(expr->children[1], nl, next_state, target_w);
            return bitwise_op(la, ra, GateType::XOR, nl);
        }
        if (op == "~^" || op == "^~") {
            auto la = synth_expr_bus(expr->children[0], nl, next_state, target_w);
            auto ra = synth_expr_bus(expr->children[1], nl, next_state, target_w);
            return bitwise_op(la, ra, GateType::XNOR, nl);
        }
        if (op == "**") {
            // Power operator: only constant exponent is synthesizable
            // Synthesize as repeated multiplication
            int exp_val = get_const_value(expr->children[1]);
            if (exp_val >= 0) {
                if (exp_val == 0) return const_to_bits("1", target_w, nl);
                auto base = synth_expr_bus(expr->children[0], nl, next_state, target_w);
                auto result = base;
                for (int i = 1; i < exp_val; i++) {
                    result = pad_to_width(build_bus_multiplier(result, base, nl), target_w, nl);
                }
                return result;
            }
            // Variable exponent: not synthesizable, return base
            return synth_expr_bus(expr->children[0], nl, next_state, target_w);
        }
        // Comparisons return 1 bit, zero-extended
        if (op=="=="||op=="!="||op=="<"||op==">"||op=="<="||op==">="||op=="&&"||op=="||") {
            NetId r = synth_expr_single(expr, nl, next_state);
            return pad_to_width({r}, target_w, nl);
        }
    }

    // Fallback: single-bit zero-extended
    NetId s = synth_expr_single(expr, nl, next_state);
    return pad_to_width({(s >= 0) ? s : make_const(0, nl)}, target_w, nl);
}

// ============================================================
// Reduction and bitwise
// ============================================================
NetId BehavioralSynthesizer::reduce(const std::vector<NetId>& bits, const std::string& op, Netlist& nl) {
    if (bits.empty()) return make_const(0, nl);
    if (bits.size() == 1) return bits[0];
    GateType gt = GateType::XOR;
    if (op == "&") gt = GateType::AND;
    else if (op == "|") gt = GateType::OR;
    NetId acc = bits[0];
    for (size_t i = 1; i < bits.size(); i++) {
        NetId out = nl.add_net("red_" + uid());
        nl.add_gate(gt, {acc, bits[i]}, out, "beh_reduce");
        acc = out;
    }
    return acc;
}

std::vector<NetId> BehavioralSynthesizer::bitwise_op(const std::vector<NetId>& a,
    const std::vector<NetId>& b, GateType gt, Netlist& nl) {
    int w = std::max(a.size(), b.size());
    std::vector<NetId> result;
    for (int i = 0; i < w; i++) {
        NetId ai = (i < (int)a.size()) ? a[i] : make_const(0, nl);
        NetId bi = (i < (int)b.size()) ? b[i] : make_const(0, nl);
        NetId out = nl.add_net("bw_" + uid());
        nl.add_gate(gt, {ai, bi}, out, "beh_bw");
        result.push_back(out);
    }
    return result;
}

// ============================================================
// Multi-bit arithmetic builders
// ============================================================

// Ripple-carry full adder
std::vector<NetId> BehavioralSynthesizer::build_bus_adder(const std::vector<NetId>& a,
    const std::vector<NetId>& b, Netlist& nl) {
    int w = std::max(a.size(), b.size());
    std::vector<NetId> sum;
    NetId carry = make_const(0, nl);
    for (int i = 0; i < w; i++) {
        NetId ai = (i < (int)a.size()) ? a[i] : make_const(0, nl);
        NetId bi = (i < (int)b.size()) ? b[i] : make_const(0, nl);
        NetId xab = nl.add_net("fa_xab_" + uid()); nl.add_gate(GateType::XOR, {ai, bi}, xab, "fa_xor1");
        NetId si = nl.add_net("fa_s_" + uid()); nl.add_gate(GateType::XOR, {xab, carry}, si, "fa_xor2");
        NetId ab = nl.add_net("fa_ab_" + uid()); nl.add_gate(GateType::AND, {ai, bi}, ab, "fa_and1");
        NetId cxab = nl.add_net("fa_cx_" + uid()); nl.add_gate(GateType::AND, {carry, xab}, cxab, "fa_and2");
        NetId co = nl.add_net("fa_co_" + uid()); nl.add_gate(GateType::OR, {ab, cxab}, co, "fa_or");
        sum.push_back(si);
        carry = co;
    }
    return sum;
}

// 2's complement subtractor: a - b = a + ~b + 1
std::vector<NetId> BehavioralSynthesizer::build_bus_subtractor(const std::vector<NetId>& a,
    const std::vector<NetId>& b, Netlist& nl) {
    int w = std::max(a.size(), b.size());
    std::vector<NetId> result;
    NetId carry = make_const(1, nl); // +1 for 2's complement
    for (int i = 0; i < w; i++) {
        NetId ai = (i < (int)a.size()) ? a[i] : make_const(0, nl);
        NetId bi = (i < (int)b.size()) ? b[i] : make_const(0, nl);
        NetId nbi = nl.add_net("sb_inv_" + uid()); nl.add_gate(GateType::NOT, {bi}, nbi, "sb_not");
        NetId xab = nl.add_net("sb_xab_" + uid()); nl.add_gate(GateType::XOR, {ai, nbi}, xab, "sb_xor1");
        NetId si = nl.add_net("sb_s_" + uid()); nl.add_gate(GateType::XOR, {xab, carry}, si, "sb_xor2");
        NetId ab = nl.add_net("sb_ab_" + uid()); nl.add_gate(GateType::AND, {ai, nbi}, ab, "sb_and1");
        NetId cxab = nl.add_net("sb_cx_" + uid()); nl.add_gate(GateType::AND, {carry, xab}, cxab, "sb_and2");
        NetId co = nl.add_net("sb_co_" + uid()); nl.add_gate(GateType::OR, {ab, cxab}, co, "sb_or");
        result.push_back(si);
        carry = co;
    }
    return result;
}

// Shift-add multiplier
std::vector<NetId> BehavioralSynthesizer::build_bus_multiplier(const std::vector<NetId>& a,
    const std::vector<NetId>& b, Netlist& nl) {
    int aw = a.size(), bw = b.size(), rw = aw + bw;
    std::vector<NetId> result;
    for (int i = 0; i < rw; i++) result.push_back(make_const(0, nl));
    for (int j = 0; j < bw; j++) {
        std::vector<NetId> partial;
        for (int i = 0; i < j; i++) partial.push_back(make_const(0, nl));
        for (int i = 0; i < aw; i++) {
            NetId p = nl.add_net("mul_p_" + uid());
            nl.add_gate(GateType::AND, {a[i], b[j]}, p, "mul_and");
            partial.push_back(p);
        }
        while ((int)partial.size() < rw) partial.push_back(make_const(0, nl));
        partial.resize(rw);
        result = build_bus_adder(result, partial, nl);
    }
    return result;
}

std::vector<NetId> BehavioralSynthesizer::build_bus_shift_left(const std::vector<NetId>& data, int amount, Netlist& nl) {
    int w = data.size();
    std::vector<NetId> result;
    for (int i = 0; i < w; i++)
        result.push_back(i < amount ? make_const(0, nl) : data[i - amount]);
    return result;
}

std::vector<NetId> BehavioralSynthesizer::build_bus_shift_right(const std::vector<NetId>& data, int amount, Netlist& nl) {
    int w = data.size();
    std::vector<NetId> result;
    for (int i = 0; i < w; i++)
        result.push_back((i + amount < w) ? data[i + amount] : make_const(0, nl));
    return result;
}

std::vector<NetId> BehavioralSynthesizer::build_barrel_shift_left(const std::vector<NetId>& data,
    const std::vector<NetId>& shamt, Netlist& nl) {
    auto current = data;
    for (int s = 0; s < (int)shamt.size(); s++) {
        auto shifted = build_bus_shift_left(current, (1 << s), nl);
        current = build_bus_mux(shamt[s], shifted, current, nl);
    }
    return current;
}

std::vector<NetId> BehavioralSynthesizer::build_barrel_shift_right(const std::vector<NetId>& data,
    const std::vector<NetId>& shamt, Netlist& nl) {
    auto current = data;
    for (int s = 0; s < (int)shamt.size(); s++) {
        auto shifted = build_bus_shift_right(current, (1 << s), nl);
        current = build_bus_mux(shamt[s], shifted, current, nl);
    }
    return current;
}

// Cascade comparator a < b (unsigned)
NetId BehavioralSynthesizer::build_bus_lt(const std::vector<NetId>& a,
    const std::vector<NetId>& b, Netlist& nl) {
    int w = std::max(a.size(), b.size());
    NetId lt = make_const(0, nl);
    for (int i = w-1; i >= 0; i--) {
        NetId ai = (i < (int)a.size()) ? a[i] : make_const(0, nl);
        NetId bi = (i < (int)b.size()) ? b[i] : make_const(0, nl);
        NetId nai = nl.add_net("lt_na_" + uid()); nl.add_gate(GateType::NOT, {ai}, nai, "lt_inv");
        NetId bl = nl.add_net("lt_bl_" + uid()); nl.add_gate(GateType::AND, {nai, bi}, bl, "lt_and");
        NetId xr = nl.add_net("lt_xr_" + uid()); nl.add_gate(GateType::XOR, {ai, bi}, xr, "lt_xor");
        NetId eq = nl.add_net("lt_eq_" + uid()); nl.add_gate(GateType::NOT, {xr}, eq, "lt_eq_inv");
        NetId eq_lt = nl.add_net("lt_eql_" + uid()); nl.add_gate(GateType::AND, {eq, lt}, eq_lt, "lt_eq_and");
        NetId new_lt = nl.add_net("lt_r_" + uid()); nl.add_gate(GateType::OR, {bl, eq_lt}, new_lt, "lt_or");
        lt = new_lt;
    }
    return lt;
}

// Signed comparator a < b (2's complement)
// If MSBs differ: a negative (MSB=1) => a < b
// If MSBs same: use unsigned comparison on lower bits
NetId BehavioralSynthesizer::build_bus_lt_signed(const std::vector<NetId>& a,
    const std::vector<NetId>& b, Netlist& nl) {
    int w = std::max(a.size(), b.size());
    if (w <= 1) return build_bus_lt(a, b, nl); // 1-bit: just unsigned

    NetId a_msb = (w-1 < (int)a.size()) ? a[w-1] : make_const(0, nl);
    NetId b_msb = (w-1 < (int)b.size()) ? b[w-1] : make_const(0, nl);

    // signs_differ = a_msb XOR b_msb
    NetId signs_differ = nl.add_net("slt_sd_" + uid());
    nl.add_gate(GateType::XOR, {a_msb, b_msb}, signs_differ, "slt_xor");

    // If signs differ: a < b iff a is negative (a_msb=1)
    // If signs same: use unsigned comparison on full word
    NetId unsigned_lt = build_bus_lt(a, b, nl);

    // result = signs_differ ? a_msb : unsigned_lt
    NetId not_sd = nl.add_net("slt_nsd_" + uid());
    nl.add_gate(GateType::NOT, {signs_differ}, not_sd, "slt_inv");
    NetId branch1 = nl.add_net("slt_b1_" + uid());
    nl.add_gate(GateType::AND, {signs_differ, a_msb}, branch1, "slt_and1");
    NetId branch2 = nl.add_net("slt_b2_" + uid());
    nl.add_gate(GateType::AND, {not_sd, unsigned_lt}, branch2, "slt_and2");
    NetId result = nl.add_net("slt_r_" + uid());
    nl.add_gate(GateType::OR, {branch1, branch2}, result, "slt_or");
    return result;
}

// Bus equality: AND of per-bit XNOR
NetId BehavioralSynthesizer::build_bus_eq(const std::vector<NetId>& a,
    const std::vector<NetId>& b, Netlist& nl) {
    int w = std::max(a.size(), b.size());
    NetId eq = make_const(1, nl);
    for (int i = 0; i < w; i++) {
        NetId ai = (i < (int)a.size()) ? a[i] : make_const(0, nl);
        NetId bi = (i < (int)b.size()) ? b[i] : make_const(0, nl);
        NetId xr = nl.add_net("eq_xr_" + uid()); nl.add_gate(GateType::XOR, {ai, bi}, xr, "eq_xor");
        NetId be = nl.add_net("eq_be_" + uid()); nl.add_gate(GateType::NOT, {xr}, be, "eq_inv");
        NetId ne = nl.add_net("eq_r_" + uid()); nl.add_gate(GateType::AND, {eq, be}, ne, "eq_and");
        eq = ne;
    }
    return eq;
}

// Per-bit MUX: sel ? t : f
std::vector<NetId> BehavioralSynthesizer::build_bus_mux(NetId sel,
    const std::vector<NetId>& t_val, const std::vector<NetId>& f_val, Netlist& nl) {
    int w = std::max(t_val.size(), f_val.size());
    std::vector<NetId> result;
    NetId ns = nl.add_net("bmux_ns_" + uid());
    nl.add_gate(GateType::NOT, {sel}, ns, "bmux_inv");
    for (int i = 0; i < w; i++) {
        NetId ti = (i < (int)t_val.size()) ? t_val[i] : make_const(0, nl);
        NetId fi = (i < (int)f_val.size()) ? f_val[i] : make_const(0, nl);
        NetId a1 = nl.add_net("bmux_a1_" + uid()); nl.add_gate(GateType::AND, {sel, ti}, a1, "bmux_and1");
        NetId a0 = nl.add_net("bmux_a0_" + uid()); nl.add_gate(GateType::AND, {ns, fi}, a0, "bmux_and0");
        NetId out = nl.add_net("bmux_o_" + uid()); nl.add_gate(GateType::OR, {a1, a0}, out, "bmux_or");
        result.push_back(out);
    }
    return result;
}

// ============================================================
// Single-bit legacy builders
// ============================================================
NetId BehavioralSynthesizer::build_adder(NetId a, NetId b, Netlist& nl) {
    NetId out = nl.add_net("sum_" + uid());
    nl.add_gate(GateType::XOR, {a, b}, out, "ha_sum"); return out;
}
NetId BehavioralSynthesizer::build_subtractor(NetId a, NetId b, Netlist& nl) {
    NetId out = nl.add_net("sub_" + uid());
    nl.add_gate(GateType::XOR, {a, b}, out, "sub_xor"); return out;
}
NetId BehavioralSynthesizer::build_comparator_lt(NetId a, NetId b, Netlist& nl) {
    NetId na = nl.add_net("lt_na_" + uid()); nl.add_gate(GateType::NOT, {a}, na, "lt_inv");
    NetId out = nl.add_net("lt_" + uid()); nl.add_gate(GateType::AND, {na, b}, out, "lt_and"); return out;
}
NetId BehavioralSynthesizer::build_comparator_gt(NetId a, NetId b, Netlist& nl) {
    NetId nb = nl.add_net("gt_nb_" + uid()); nl.add_gate(GateType::NOT, {b}, nb, "gt_inv");
    NetId out = nl.add_net("gt_" + uid()); nl.add_gate(GateType::AND, {a, nb}, out, "gt_and"); return out;
}
NetId BehavioralSynthesizer::build_mux2(NetId sel, NetId in1, NetId in0, Netlist& nl) {
    NetId ns = nl.add_net("mux_ns_" + uid()); nl.add_gate(GateType::NOT, {sel}, ns, "mux_inv");
    NetId a1 = nl.add_net("mux_a1_" + uid()); nl.add_gate(GateType::AND, {sel, in1}, a1, "mux_and1");
    NetId a0 = nl.add_net("mux_a0_" + uid()); nl.add_gate(GateType::AND, {ns, in0}, a0, "mux_and0");
    NetId out = nl.add_net("mux_o_" + uid()); nl.add_gate(GateType::OR, {a1, a0}, out, "mux_or"); return out;
}

// ============================================================
// Arithmetic right shift (constant amount): shift right, fill MSB with sign bit
// ============================================================
std::vector<NetId> BehavioralSynthesizer::build_arith_shift_right(const std::vector<NetId>& data, int amt, Netlist& nl) {
    int w = (int)data.size();
    if (amt >= w) {
        // All bits become the sign bit
        std::vector<NetId> result(w, data[w-1]);
        return result;
    }
    std::vector<NetId> result(w);
    for (int i = 0; i < w - amt; i++) result[i] = data[i + amt];
    for (int i = w - amt; i < w; i++) result[i] = data[w-1]; // sign extension
    return result;
}

// ============================================================
// Barrel arithmetic right shift (variable amount): like barrel right, but sign-extend
// ============================================================
std::vector<NetId> BehavioralSynthesizer::build_barrel_arith_shift_right(const std::vector<NetId>& data,
    const std::vector<NetId>& shamt, Netlist& nl) {
    int w = (int)data.size();
    int sw = (int)shamt.size();
    std::vector<NetId> current = data;
    NetId sign_bit = data[w-1];
    for (int s = 0; s < sw && (1 << s) < w; s++) {
        int shift = 1 << s;
        std::vector<NetId> next(w);
        for (int i = 0; i < w; i++) {
            if (i + shift < w) {
                next[i] = build_mux2(shamt[s], current[i + shift], current[i], nl);
            } else {
                next[i] = build_mux2(shamt[s], sign_bit, current[i], nl);
            }
        }
        current = next;
    }
    return current;
}

// ============================================================
// Restoring divider: quotient = a / b, remainder = a % b
// ============================================================
std::pair<std::vector<NetId>, std::vector<NetId>> BehavioralSynthesizer::build_divider(
    const std::vector<NetId>& a, const std::vector<NetId>& b, Netlist& nl) {
    int w = (int)a.size();
    int bw = (int)b.size();
    // Use max width
    int dw = std::max(w, bw);

    // Pad both to dw
    auto dividend = pad_to_width(a, dw, nl);
    auto divisor = pad_to_width(b, dw, nl);

    std::vector<NetId> quotient(dw);
    std::vector<NetId> remainder(dw);

    // Initialize remainder to 0
    for (int i = 0; i < dw; i++) {
        remainder[i] = nl.add_net("div_r0_" + std::to_string(i) + "_" + uid());
        nl.add_gate(GateType::BUF, {nl.add_net("__const0_" + uid())}, remainder[i], "div_zero");
    }

    // Restoring division algorithm
    for (int i = dw - 1; i >= 0; i--) {
        // Shift remainder left by 1 and bring in dividend bit
        std::vector<NetId> shifted_rem(dw);
        shifted_rem[0] = dividend[i];
        for (int j = 1; j < dw; j++) shifted_rem[j] = remainder[j-1];

        // Try subtract: shifted_rem - divisor
        auto sub_result = build_bus_subtractor(shifted_rem, divisor, nl);

        // If subtraction result MSB is 0 (non-negative), quotient bit = 1, remainder = sub_result
        // If MSB is 1 (negative), quotient bit = 0, remainder = shifted_rem
        NetId neg = sub_result[dw-1]; // sign bit
        NetId pos = nl.add_net("div_pos_" + uid());
        nl.add_gate(GateType::NOT, {neg}, pos, "div_inv");

        quotient[i] = pos;

        // Select remainder: pos ? sub_result : shifted_rem
        for (int j = 0; j < dw; j++) {
            remainder[j] = build_mux2(pos, sub_result[j], shifted_rem[j], nl);
        }
    }

    return {quotient, remainder};
}

// ============================================================
// Memory read: MUX tree selecting word by address
// ============================================================
std::vector<NetId> BehavioralSynthesizer::synth_mem_read(const std::string& mem_name,
    std::shared_ptr<AstNode> addr_expr, Netlist& nl,
    const std::map<std::string, std::vector<NetId>>& next_state, int target_w) {

    auto mit = memory_arrays_.find(mem_name);
    if (mit == memory_arrays_.end()) return pad_to_width({make_const(0, nl)}, target_w, nl);

    int word_w = mit->second.first;
    int depth = mit->second.second;
    if (depth <= 0) depth = 1;
    int addr_w = 1;
    while ((1 << addr_w) < depth) addr_w++;

    auto addr_bits = synth_expr_bus(addr_expr, nl, next_state, addr_w);

    // For each memory location, get the word bits
    // mem_name_i[j] for location i, bit j
    std::vector<std::vector<NetId>> words(depth);
    for (int i = 0; i < depth; i++) {
        for (int b = 0; b < word_w; b++) {
            std::string bname = mem_name + "_" + std::to_string(i) + "[" + std::to_string(b) + "]";
            words[i].push_back(get_net(nl, bname));
        }
    }

    // Build MUX tree: for each bit position, create depth-to-1 MUX
    std::vector<NetId> result(word_w);
    for (int b = 0; b < word_w; b++) {
        // Binary MUX tree using address bits
        std::vector<NetId> level;
        for (int i = 0; i < depth; i++) level.push_back(words[i][b]);
        // Pad to power of 2
        while ((int)level.size() < (1 << addr_w))
            level.push_back(make_const(0, nl));

        for (int s = 0; s < addr_w; s++) {
            std::vector<NetId> next_level;
            for (int j = 0; j < (int)level.size(); j += 2) {
                if (j + 1 < (int)level.size()) {
                    auto mux_out = build_bus_mux(addr_bits[s], {level[j+1]}, {level[j]}, nl);
                    next_level.push_back(mux_out[0]);
                } else {
                    next_level.push_back(level[j]);
                }
            }
            level = next_level;
        }
        result[b] = level[0];
    }
    return pad_to_width(result, target_w, nl);
}

// ============================================================
// Memory write: decoder + data routing
// ============================================================
void BehavioralSynthesizer::synth_mem_write(const std::string& mem_name,
    std::shared_ptr<AstNode> addr_expr, std::shared_ptr<AstNode> data_expr,
    Netlist& nl, std::map<std::string, std::vector<NetId>>& next_state) {

    auto mit = memory_arrays_.find(mem_name);
    if (mit == memory_arrays_.end()) return;

    int word_w = mit->second.first;
    int depth = mit->second.second;
    if (depth <= 0) depth = 1;
    int addr_w = 1;
    while ((1 << addr_w) < depth) addr_w++;

    auto addr_bits = synth_expr_bus(addr_expr, nl, next_state, addr_w);
    auto data_bits = synth_expr_bus(data_expr, nl, next_state, word_w);

    // Build address decoder: for each location, generate write-enable
    std::vector<NetId> we(depth);
    for (int i = 0; i < depth; i++) {
        auto addr_val = const_to_bits(std::to_string(i), addr_w, nl);
        we[i] = build_bus_eq(addr_bits, addr_val, nl);
    }

    // For each location: if write-enable, use new data; else keep old value
    for (int i = 0; i < depth; i++) {
        std::string loc_name = mem_name + "_" + std::to_string(i);
        std::vector<NetId> old_bits;
        for (int b = 0; b < word_w; b++) {
            std::string bname = loc_name + "[" + std::to_string(b) + "]";
            old_bits.push_back(get_net(nl, bname));
        }
        auto new_bits = build_bus_mux(we[i], data_bits, old_bits, nl);
        next_state[loc_name] = new_bits;
    }
}

} // namespace sf
