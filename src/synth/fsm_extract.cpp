// SiliconForge — Algorithmic FSM Extraction & Encoding Implementation
#include "synth/fsm_extract.hpp"
#include <iostream>
#include <algorithm>
#include <queue>
#include <sstream>
#include <numeric>
#include <iomanip>

namespace sf {

// ─────────────────────────────────────────────────────────────────────────────
// Existing methods (unchanged)
// ─────────────────────────────────────────────────────────────────────────────

std::vector<StateMachine> FsmExtractor::extract_fsms(std::shared_ptr<AstNode> root) {
    std::vector<StateMachine> fsms;
    find_state_machines(root, fsms);
    return fsms;
}

void FsmExtractor::find_state_machines(std::shared_ptr<AstNode> node, std::vector<StateMachine>& fsms) {
    if (!node) return;

    // A state machine is typically modeled inside an ALWAYS_POS_CLK block
    if (node->type == AstNodeType::ALWAYS_POS_CLK) {
        // Look for a switch/case on a state variable, or large if/else tree
        // For phase 16 MVP, we'll extract explicit IF_ELSE structures assigning to the same variable
        StateMachine fsm;
        extract_transitions(node, fsm);
        
        if (fsm.state_names.size() > 1) { // Found a valid state machine with at least 2 states
            std::cout << "[FSM] Extracted State Machine for variable '" << fsm.state_var_name << "' with " << fsm.num_states << " states.\n";
            fsms.push_back(fsm);
        }
    } else {
        for (auto& child : node->children) {
            find_state_machines(child, fsms);
        }
    }
}

// Simple heuristic: Walk the AST of the always block. If an IF_ELSE checks a variable against a literal,
// and assigns functionally to that same variable, infer it as a State Register.
// E.g., if (state == 0) state <= 1;
void FsmExtractor::extract_transitions(std::shared_ptr<AstNode> node, StateMachine& fsm) {
    if (!node) return;

    if (node->type == AstNodeType::IF_ELSE && node->children.size() > 0) {
        auto cond = node->children[0];
        if (cond->type == AstNodeType::BIN_OP && cond->value == "==") {
            auto lhs = cond->children[0];
            auto rhs = cond->children[1];
            
            // Assuming lhs is the variable and rhs is the literal state value
            if (lhs->children.empty() && rhs->children.empty()) { // Simple wire and literal
                if (fsm.state_var_name.empty()) {
                    fsm.state_var_name = lhs->value;
                }
                
                if (lhs->value == fsm.state_var_name) {
                    std::string from_state = rhs->value;
                    if (std::find(fsm.state_names.begin(), fsm.state_names.end(), from_state) == fsm.state_names.end()) {
                        fsm.state_names.push_back(from_state);
                        fsm.num_states++;
                    }
                }
            }
        }
    }
    
    // Only recurse into control flow blocks, not individual expressions to avoid infinite loops
    if (node->type == AstNodeType::ALWAYS_POS_CLK || 
        node->type == AstNodeType::BLOCK_BEGIN_END || 
        node->type == AstNodeType::IF_ELSE) {
        for (auto& child : node->children) {
            extract_transitions(child, fsm);
        }
    }
}

// Apply Mathematical State Encodings
void FsmExtractor::apply_encoding(StateMachine& fsm, FsmEncoding encoding) {
    fsm.encoded_values.clear();
    
    for (int i = 0; i < fsm.num_states; ++i) {
        int encoded_val = 0;
        
        if (encoding == FsmEncoding::BINARY) {
            encoded_val = i;
        } else if (encoding == FsmEncoding::ONE_HOT) {
            encoded_val = 1 << i;
        } else if (encoding == FsmEncoding::GRAY_CODE) {
            encoded_val = i ^ (i >> 1);
        }
        
        fsm.encoded_values[fsm.state_names[i]] = encoded_val;
    }
    
    std::cout << "[FSM] Re-encoded '" << fsm.state_var_name << "' elements.\n";
}

void FsmExtractor::optimize_fsms(std::shared_ptr<AstNode> root, const std::vector<StateMachine>& fsms, FsmEncoding encoding) {
    for (auto fsm : fsms) {
        apply_encoding(fsm, encoding);
        rewrite_ast(root, fsm);
        std::cout << "[FSM] Optimized AST for State Machine '" << fsm.state_var_name << "' using advanced encoding.\n";
    }
}

// Rewrites AST conditional == checks and <= assignments with the new mathematically optimal encoding
void FsmExtractor::rewrite_ast(std::shared_ptr<AstNode> node, const StateMachine& fsm) {
    if (!node) return;
    
    // Replace literals in comparisons (if state == 0)
    if (node->type == AstNodeType::BIN_OP && node->value == "==") {
        if (!node->children.empty() && node->children[0]->value == fsm.state_var_name) {
            std::string old_val = node->children[1]->value;
            if (fsm.encoded_values.count(old_val)) {
                node->children[1]->value = std::to_string(fsm.encoded_values.at(old_val));
            }
        }
    }
    
    // Replace literals in assignments (state <= 1)
    if (node->type == AstNodeType::NONBLOCK_ASSIGN && node->value == fsm.state_var_name) {
        if (!node->children.empty() && node->children[0]->children.empty()) { // Simple wire/literal assignment
            std::string old_val = node->children[0]->value;
            if (fsm.encoded_values.count(old_val)) {
                node->children[0]->value = std::to_string(fsm.encoded_values.at(old_val));
            }
        }
    }
    
    for (auto& child : node->children) {
        rewrite_ast(child, fsm);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

uint32_t FsmExtractor::gray_encode(uint32_t n) {
    return n ^ (n >> 1);
}

int FsmExtractor::count_transitions_from(const StateMachine& fsm, const std::string& state) {
    auto it = fsm.transitions.find(state);
    if (it == fsm.transitions.end()) return 0;
    return static_cast<int>(it->second.size());
}

std::set<std::string> FsmExtractor::find_reachable_states(const StateMachine& fsm, const std::string& start) {
    std::set<std::string> visited;
    std::queue<std::string> worklist;
    
    worklist.push(start);
    visited.insert(start);
    
    while (!worklist.empty()) {
        std::string current = worklist.front();
        worklist.pop();
        
        auto it = fsm.transitions.find(current);
        if (it != fsm.transitions.end()) {
            for (auto& [to_state, condition] : it->second) {
                if (visited.find(to_state) == visited.end()) {
                    visited.insert(to_state);
                    worklist.push(to_state);
                }
            }
        }
    }
    
    return visited;
}

bool FsmExtractor::states_equivalent(const StateMachine& fsm, const std::string& s1, const std::string& s2,
                                     const std::map<std::string, int>& partition) {
    // Two states are equivalent if they have the same transition targets (by partition)
    // for all inputs, and the same set of destination partitions
    
    auto it1 = fsm.transitions.find(s1);
    auto it2 = fsm.transitions.find(s2);
    
    bool has_t1 = (it1 != fsm.transitions.end());
    bool has_t2 = (it2 != fsm.transitions.end());
    
    // If one has transitions and the other doesn't, they differ
    if (has_t1 != has_t2) return false;
    
    // Both have no transitions — equivalent (terminal states)
    if (!has_t1 && !has_t2) return true;
    
    const auto& trans1 = it1->second;
    const auto& trans2 = it2->second;
    
    // Must have same number of outgoing transitions
    if (trans1.size() != trans2.size()) return false;
    
    // Collect the set of destination partitions for each state
    std::set<int> dest_partitions_1, dest_partitions_2;
    for (auto& [to, cond] : trans1) {
        auto pit = partition.find(to);
        if (pit != partition.end()) dest_partitions_1.insert(pit->second);
    }
    for (auto& [to, cond] : trans2) {
        auto pit = partition.find(to);
        if (pit != partition.end()) dest_partitions_2.insert(pit->second);
    }
    
    return dest_partitions_1 == dest_partitions_2;
}

// ─────────────────────────────────────────────────────────────────────────────
// A) State Minimization — Hopcroft's partition-refinement algorithm
// ─────────────────────────────────────────────────────────────────────────────

MinimizedFsm FsmExtractor::minimize_states(const StateMachine& fsm) {
    MinimizedFsm result;
    result.original = fsm;
    
    if (fsm.num_states <= 1) {
        result.minimized = fsm;
        result.states_removed = 0;
        return result;
    }
    
    // Step 1: Initial partition — group states by their transition target count
    // States with the same number of outgoing transitions start in the same partition
    std::map<std::string, int> partition_map;
    std::map<int, std::vector<std::string>> partition_groups;
    
    // Initial partition: group by transition fan-out count
    std::map<int, int> fanout_to_partition;
    int next_partition_id = 0;
    
    for (auto& state : fsm.state_names) {
        int fanout = count_transitions_from(fsm, state);
        if (fanout_to_partition.find(fanout) == fanout_to_partition.end()) {
            fanout_to_partition[fanout] = next_partition_id++;
        }
        int pid = fanout_to_partition[fanout];
        partition_map[state] = pid;
        partition_groups[pid].push_back(state);
    }
    
    // Step 2: Iterative refinement
    bool changed = true;
    while (changed) {
        changed = false;
        std::map<std::string, int> new_partition_map;
        std::map<int, std::vector<std::string>> new_partition_groups;
        int new_pid = 0;
        
        for (auto& [pid, states] : partition_groups) {
            if (states.empty()) continue;
            
            // First state in the group defines the reference
            std::vector<std::vector<std::string>> sub_groups;
            sub_groups.push_back({states[0]});
            
            for (size_t i = 1; i < states.size(); ++i) {
                bool placed = false;
                for (auto& sg : sub_groups) {
                    if (states_equivalent(fsm, states[i], sg[0], partition_map)) {
                        sg.push_back(states[i]);
                        placed = true;
                        break;
                    }
                }
                if (!placed) {
                    sub_groups.push_back({states[i]});
                }
            }
            
            if (sub_groups.size() > 1) changed = true;
            
            for (auto& sg : sub_groups) {
                for (auto& s : sg) {
                    new_partition_map[s] = new_pid;
                    new_partition_groups[new_pid].push_back(s);
                }
                new_pid++;
            }
        }
        
        partition_map = new_partition_map;
        partition_groups = new_partition_groups;
    }
    
    // Step 3: Build minimized FSM — pick representative from each partition
    StateMachine minimized;
    minimized.state_var_name = fsm.state_var_name;
    
    std::map<int, std::string> representative; // partition_id -> representative state
    for (auto& [pid, states] : partition_groups) {
        if (states.empty()) continue;
        std::string rep = states[0];
        representative[pid] = rep;
        minimized.state_names.push_back(rep);
        
        // Build merge map for non-representative states
        for (size_t i = 1; i < states.size(); ++i) {
            result.state_merge_map[states[i]] = rep;
        }
    }
    minimized.num_states = static_cast<int>(minimized.state_names.size());
    
    // Step 4: Rebuild transitions using representatives
    for (auto& state : minimized.state_names) {
        auto it = fsm.transitions.find(state);
        if (it == fsm.transitions.end()) continue;
        
        for (auto& [to_state, condition] : it->second) {
            // Map the destination to its representative
            int to_pid = partition_map[to_state];
            std::string to_rep = representative[to_pid];
            minimized.transitions[state][to_rep] = condition;
        }
    }
    
    result.minimized = minimized;
    result.states_removed = fsm.num_states - minimized.num_states;
    
    if (result.states_removed > 0) {
        std::cout << "[FSM] Minimized '" << fsm.state_var_name << "': "
                  << fsm.num_states << " → " << minimized.num_states
                  << " states (" << result.states_removed << " removed).\n";
    }
    
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// B) Unreachable State Pruning — BFS from initial state
// ─────────────────────────────────────────────────────────────────────────────

StateMachine FsmExtractor::prune_unreachable(const StateMachine& fsm, const std::string& initial_state) {
    if (fsm.state_names.empty()) return fsm;
    
    // Determine initial state
    std::string start = initial_state;
    if (start.empty()) {
        // Heuristic: look for common initial state names
        static const std::vector<std::string> candidates = {"IDLE", "INIT", "S0", "RESET", "idle", "init", "s0", "reset"};
        for (auto& c : candidates) {
            if (std::find(fsm.state_names.begin(), fsm.state_names.end(), c) != fsm.state_names.end()) {
                start = c;
                break;
            }
        }
        // Fall back to first state
        if (start.empty()) {
            start = fsm.state_names[0];
        }
    }
    
    // BFS to find reachable states
    std::set<std::string> reachable = find_reachable_states(fsm, start);
    
    // Build pruned FSM
    StateMachine pruned;
    pruned.state_var_name = fsm.state_var_name;
    
    for (auto& state : fsm.state_names) {
        if (reachable.count(state)) {
            pruned.state_names.push_back(state);
        }
    }
    pruned.num_states = static_cast<int>(pruned.state_names.size());
    
    // Copy only transitions involving reachable states
    for (auto& [from, targets] : fsm.transitions) {
        if (!reachable.count(from)) continue;
        for (auto& [to, cond] : targets) {
            if (reachable.count(to)) {
                pruned.transitions[from][to] = cond;
            }
        }
    }
    
    // Preserve existing encoding for reachable states
    for (auto& [state, val] : fsm.encoded_values) {
        if (reachable.count(state)) {
            pruned.encoded_values[state] = val;
        }
    }
    
    int removed = fsm.num_states - pruned.num_states;
    if (removed > 0) {
        std::cout << "[FSM] Pruned " << removed << " unreachable state(s) from '"
                  << fsm.state_var_name << "' (start='" << start << "').\n";
    }
    
    return pruned;
}

// ─────────────────────────────────────────────────────────────────────────────
// C) Encoding Analysis — area/performance cost estimation
// ─────────────────────────────────────────────────────────────────────────────

EncodingAnalysis FsmExtractor::analyze_encoding(const StateMachine& fsm) {
    EncodingAnalysis analysis{};
    int n = fsm.num_states;
    
    if (n <= 1) {
        analysis.recommended = FsmEncoding::BINARY;
        analysis.binary_bits = 1;
        analysis.onehot_bits = std::max(1, n);
        analysis.gray_bits = 1;
        analysis.binary_area_estimate = 1.0;
        analysis.onehot_area_estimate = 1.0;
        analysis.gray_area_estimate = 1.0;
        analysis.reason = "Trivial FSM (0-1 states)";
        return analysis;
    }
    
    // Bit width calculations
    analysis.binary_bits = static_cast<int>(std::ceil(std::log2(static_cast<double>(n))));
    if (analysis.binary_bits < 1) analysis.binary_bits = 1;
    analysis.onehot_bits = n;
    analysis.gray_bits = analysis.binary_bits;
    
    // Compute transition density: total transitions / num_states
    int total_transitions = 0;
    for (auto& [from, targets] : fsm.transitions) {
        total_transitions += static_cast<int>(targets.size());
    }
    double avg_transitions = (n > 0) ? static_cast<double>(total_transitions) / n : 0.0;
    double density = (n > 1) ? static_cast<double>(total_transitions) / (static_cast<double>(n) * (n - 1)) : 0.0;
    
    // Area estimates: model next-state logic complexity
    // Binary: comparator + decoder width proportional to bits × transitions
    analysis.binary_area_estimate = analysis.binary_bits * (avg_transitions + 1.0) * 2.5;
    
    // One-hot: one flip-flop per state, but simpler next-state logic (single-bit checks)
    analysis.onehot_area_estimate = n * 2.0 + avg_transitions * 1.5;
    
    // Gray: same bits as binary but reduced glitch power (~15% area savings on transition logic)
    analysis.gray_area_estimate = analysis.binary_area_estimate * 0.85;
    
    // Decision logic
    if (n <= 4) {
        // Very small FSMs: binary is efficient, minimal overhead
        analysis.recommended = FsmEncoding::BINARY;
        analysis.reason = "Small FSM (<=4 states): binary encoding is compact";
    } else if (n <= 16) {
        if (density > 0.5) {
            // Dense transitions: binary saves flip-flops
            analysis.recommended = FsmEncoding::BINARY;
            analysis.reason = "Dense transitions favor binary (density=" +
                              std::to_string(density).substr(0, 4) + ")";
        } else {
            // Sparse transitions: one-hot has simpler next-state logic
            analysis.recommended = FsmEncoding::ONE_HOT;
            analysis.reason = "Sparse FSM (<=16 states): one-hot gives faster decode";
        }
    } else {
        // Large FSMs: binary or gray to minimize register count
        if (density < 0.3) {
            analysis.recommended = FsmEncoding::GRAY_CODE;
            analysis.reason = "Large sparse FSM: Gray code reduces glitching";
        } else {
            analysis.recommended = FsmEncoding::BINARY;
            analysis.reason = "Large FSM (>16 states): binary minimizes registers";
        }
    }
    
    return analysis;
}

// ─────────────────────────────────────────────────────────────────────────────
// D) Auto-select encoding — wrapper around analyze_encoding
// ─────────────────────────────────────────────────────────────────────────────

FsmEncoding FsmExtractor::auto_select_encoding(const StateMachine& fsm) {
    EncodingAnalysis analysis = analyze_encoding(fsm);
    std::cout << "[FSM] Auto-selected encoding for '" << fsm.state_var_name
              << "': " << (analysis.recommended == FsmEncoding::BINARY ? "BINARY" :
                           analysis.recommended == FsmEncoding::ONE_HOT ? "ONE_HOT" : "GRAY_CODE")
              << " (" << analysis.reason << ").\n";
    return analysis.recommended;
}

// ─────────────────────────────────────────────────────────────────────────────
// E) FSM Re-synthesis — generate optimized behavioral AST from FSM
// ─────────────────────────────────────────────────────────────────────────────

std::shared_ptr<AstNode> FsmExtractor::resynthesize_fsm(const StateMachine& fsm, FsmEncoding encoding) {
    // Apply encoding to a copy
    StateMachine encoded_fsm = fsm;
    apply_encoding(encoded_fsm, encoding);
    
    // Create: always @(posedge clk)
    auto always_block = AstNode::make(AstNodeType::ALWAYS_POS_CLK);
    
    // Create: case(state_var)
    auto case_stmt = AstNode::make(AstNodeType::CASE_STMT, encoded_fsm.state_var_name);
    
    for (auto& state_name : encoded_fsm.state_names) {
        int enc_val = encoded_fsm.encoded_values[state_name];
        
        // Create case item: encoded_value: begin ... end
        auto case_item = AstNode::make(AstNodeType::CASE_ITEM, std::to_string(enc_val));
        auto block = AstNode::make(AstNodeType::BLOCK_BEGIN_END);
        
        // Check if this state has transitions
        auto trans_it = encoded_fsm.transitions.find(state_name);
        if (trans_it != encoded_fsm.transitions.end() && !trans_it->second.empty()) {
            // Build if-else chain for transitions from this state
            std::shared_ptr<AstNode> current_if = nullptr;
            std::shared_ptr<AstNode> first_if = nullptr;
            
            for (auto& [to_state, condition] : trans_it->second) {
                int to_enc = encoded_fsm.encoded_values[to_state];
                
                // Create: if (condition) state <= to_encoded_val
                auto if_node = AstNode::make(AstNodeType::IF_ELSE);
                
                // Add condition (use original if available, else synthesize a true literal)
                if (condition) {
                    if_node->add(condition);
                } else {
                    auto true_cond = AstNode::make(AstNodeType::NUMBER_LITERAL, "1");
                    true_cond->int_val = 1;
                    if_node->add(true_cond);
                }
                
                // Add assignment: state_var <= to_encoded_val
                auto assign = AstNode::make(AstNodeType::NONBLOCK_ASSIGN, encoded_fsm.state_var_name);
                auto val_node = AstNode::make(AstNodeType::NUMBER_LITERAL, std::to_string(to_enc));
                val_node->int_val = to_enc;
                assign->add(val_node);
                
                auto then_block = AstNode::make(AstNodeType::BLOCK_BEGIN_END);
                then_block->add(assign);
                if_node->add(then_block);
                
                if (!first_if) {
                    first_if = if_node;
                    current_if = if_node;
                } else {
                    // Chain as else branch of previous if
                    auto else_block = AstNode::make(AstNodeType::BLOCK_BEGIN_END);
                    else_block->add(if_node);
                    current_if->add(else_block);
                    current_if = if_node;
                }
            }
            
            if (first_if) {
                block->add(first_if);
            }
        } else {
            // No transitions: self-loop (state holds value)
            auto assign = AstNode::make(AstNodeType::NONBLOCK_ASSIGN, encoded_fsm.state_var_name);
            auto val_node = AstNode::make(AstNodeType::NUMBER_LITERAL, std::to_string(enc_val));
            val_node->int_val = enc_val;
            assign->add(val_node);
            block->add(assign);
        }
        
        case_item->add(block);
        case_stmt->add(case_item);
    }
    
    always_block->add(case_stmt);
    
    std::cout << "[FSM] Re-synthesized '" << encoded_fsm.state_var_name
              << "' with " << encoded_fsm.num_states << " states using "
              << (encoding == FsmEncoding::BINARY ? "BINARY" :
                  encoding == FsmEncoding::ONE_HOT ? "ONE_HOT" : "GRAY_CODE")
              << " encoding.\n";
    
    return always_block;
}

// ─────────────────────────────────────────────────────────────────────────────
// F) Full Optimization Pipeline
// ─────────────────────────────────────────────────────────────────────────────

FsmExtractor::FsmOptResult FsmExtractor::optimize_all(std::shared_ptr<AstNode> root) {
    auto t_start = std::chrono::steady_clock::now();
    FsmOptResult result;
    
    // Step 1: Extract all FSMs
    auto fsms = extract_fsms(root);
    result.fsms_found = static_cast<int>(fsms.size());
    
    if (fsms.empty()) {
        result.message = "No FSMs found in design.";
        auto t_end = std::chrono::steady_clock::now();
        result.time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        return result;
    }
    
    std::cout << "[FSM] Pipeline: found " << result.fsms_found << " FSM(s).\n";
    
    // Step 2: For each FSM: prune → minimize → auto-encode → rewrite
    for (auto& fsm : fsms) {
        // Prune unreachable states
        int pre_prune = fsm.num_states;
        StateMachine pruned = prune_unreachable(fsm);
        result.unreachable_pruned += (pre_prune - pruned.num_states);
        
        // Minimize equivalent states
        MinimizedFsm minimized = minimize_states(pruned);
        result.states_minimized += minimized.states_removed;
        
        // Auto-select encoding
        EncodingAnalysis enc = analyze_encoding(minimized.minimized);
        result.encoding_decisions.push_back(enc);
        
        // Apply encoding and rewrite AST
        StateMachine final_fsm = minimized.minimized;
        apply_encoding(final_fsm, enc.recommended);
        rewrite_ast(root, final_fsm);
        
        std::cout << "[FSM] Pipeline complete for '" << final_fsm.state_var_name
                  << "': " << pre_prune << " → " << final_fsm.num_states << " states.\n";
    }
    
    auto t_end = std::chrono::steady_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    std::ostringstream msg;
    msg << "Optimized " << result.fsms_found << " FSM(s): "
        << result.unreachable_pruned << " unreachable pruned, "
        << result.states_minimized << " states minimized in "
        << std::fixed << std::setprecision(2) << result.time_ms << "ms";
    result.message = msg.str();
    
    std::cout << "[FSM] " << result.message << "\n";
    
    return result;
}

} // namespace sf
