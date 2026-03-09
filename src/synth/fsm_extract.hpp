#pragma once
// SiliconForge — Algorithmic FSM Extraction & Encoding
// Extracts State Transition Graphs from Behavioral AST and applies optimal state encoding

#include "synth/behavioral_synth.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <set>

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

class FsmExtractor {
public:
    // Pass 1: Identifies FSMs within the behavioral AST
    std::vector<StateMachine> extract_fsms(std::shared_ptr<AstNode> root);
    
    // Pass 2: Re-encodes the state variables in the AST to the target encoding
    void optimize_fsms(std::shared_ptr<AstNode> root, const std::vector<StateMachine>& fsms, FsmEncoding encoding = FsmEncoding::ONE_HOT);

private:
    void find_state_machines(std::shared_ptr<AstNode> node, std::vector<StateMachine>& fsms);
    void extract_transitions(std::shared_ptr<AstNode> always_block, StateMachine& fsm);
    void apply_encoding(StateMachine& fsm, FsmEncoding encoding);
    void rewrite_ast(std::shared_ptr<AstNode> node, const StateMachine& fsm);
};

} // namespace sf
