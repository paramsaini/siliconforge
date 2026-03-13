#pragma once
// SiliconForge — Algorithmic FSM Extraction & Encoding
// Extracts State Transition Graphs from Behavioral AST and applies optimal state encoding

#include "synth/behavioral_synth.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <set>
#include <cmath>
#include <chrono>

namespace sf {

// Represents a mathematical Finite State Machine extracted from the AST
struct StateMachine {
    std::string state_var_name;
    int num_states = 0;
    std::vector<std::string> state_names;
    
    // Transitions: from_state -> to_state -> condition (AstNode)
    std::map<std::string, std::map<std::string, std::shared_ptr<AstNode>>> transitions;
    
    // New Encoding
    std::map<std::string, int> encoded_values; // state_name -> integer value
};

enum class FsmEncoding {
    BINARY,
    ONE_HOT,
    GRAY_CODE
};

// State minimization result (Hopcroft's algorithm)
struct MinimizedFsm {
    StateMachine original;
    StateMachine minimized;
    int states_removed = 0;
    std::map<std::string, std::string> state_merge_map; // merged_from -> merged_to
};

// Encoding analysis result
struct EncodingAnalysis {
    FsmEncoding recommended;
    int binary_bits;      // ceil(log2(states))
    int onehot_bits;      // = num_states
    int gray_bits;        // = binary_bits
    double binary_area_estimate;
    double onehot_area_estimate;
    double gray_area_estimate;
    std::string reason;
};

class FsmExtractor {
public:
    // Pass 1: Identifies FSMs within the behavioral AST
    std::vector<StateMachine> extract_fsms(std::shared_ptr<AstNode> root);
    
    // Pass 2: Re-encodes the state variables in the AST to the target encoding
    void optimize_fsms(std::shared_ptr<AstNode> root, const std::vector<StateMachine>& fsms, FsmEncoding encoding = FsmEncoding::ONE_HOT);
    
    // State minimization (Hopcroft's partition refinement)
    MinimizedFsm minimize_states(const StateMachine& fsm);
    
    // Unreachable state detection and pruning
    StateMachine prune_unreachable(const StateMachine& fsm, const std::string& initial_state = "");
    
    // Automatic encoding selection
    EncodingAnalysis analyze_encoding(const StateMachine& fsm);
    FsmEncoding auto_select_encoding(const StateMachine& fsm);
    
    // FSM re-synthesis — convert FSM back to optimized behavioral AST
    std::shared_ptr<AstNode> resynthesize_fsm(const StateMachine& fsm, FsmEncoding encoding);
    
    // Full optimization pipeline
    struct FsmOptResult {
        int fsms_found = 0;
        int states_minimized = 0;
        int unreachable_pruned = 0;
        std::vector<EncodingAnalysis> encoding_decisions;
        double time_ms = 0;
        std::string message;
    };
    FsmOptResult optimize_all(std::shared_ptr<AstNode> root);

private:
    void find_state_machines(std::shared_ptr<AstNode> node, std::vector<StateMachine>& fsms);
    void extract_transitions(std::shared_ptr<AstNode> always_block, StateMachine& fsm);
    void apply_encoding(StateMachine& fsm, FsmEncoding encoding);
    void rewrite_ast(std::shared_ptr<AstNode> node, const StateMachine& fsm);
    
    // Private helpers for new functionality
    std::set<std::string> find_reachable_states(const StateMachine& fsm, const std::string& start);
    bool states_equivalent(const StateMachine& fsm, const std::string& s1, const std::string& s2,
                          const std::map<std::string, int>& partition);
    uint32_t gray_encode(uint32_t n);
    int count_transitions_from(const StateMachine& fsm, const std::string& state);
};

} // namespace sf
