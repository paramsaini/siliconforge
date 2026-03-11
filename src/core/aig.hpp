#pragma once
// SiliconForge — And-Inverter Graph (AIG)
// The central data structure for formal verification and synthesis.
// Reference: A. Mishchenko, "DAG-Aware AIG Rewriting", DAC 2006
// Encoding: AIGER standard — literal = (variable << 1) | complement

#include <cstdint>
#include <vector>
#include <string>
#include <unordered_map>
#include <iostream>

namespace sf {

using AigLit = uint32_t;

constexpr AigLit AIG_FALSE = 0;
constexpr AigLit AIG_TRUE  = 1;

inline constexpr uint32_t aig_var(AigLit lit)  { return lit >> 1; }
inline constexpr bool     aig_sign(AigLit lit) { return lit & 1; }
inline constexpr AigLit   aig_not(AigLit lit)  { return lit ^ 1; }
inline constexpr AigLit   aig_regular(AigLit lit) { return lit & ~1u; }
inline constexpr AigLit   aig_make(uint32_t var, bool neg = false) {
    return (var << 1) | static_cast<uint32_t>(neg);
}

struct AigNode {
    AigLit fanin0;
    AigLit fanin1;
};

// Sequential latch: holds state between clock cycles
struct AigLatch {
    AigLit next;        // Next-state function (combinational)
    AigLit init;        // Initial value literal (AIG_FALSE or AIG_TRUE)
    std::string name;
};

class AigGraph {
public:
    AigGraph() = default;

    // --- Construction ---
    AigLit create_input(const std::string& name = "");
    AigLit create_and(AigLit a, AigLit b);   // Structural hashing + simplification
    AigLit create_or(AigLit a, AigLit b);
    AigLit create_xor(AigLit a, AigLit b);
    AigLit create_mux(AigLit sel, AigLit t, AigLit e); // sel ? t : e
    AigLit create_nand(AigLit a, AigLit b);
    AigLit create_nor(AigLit a, AigLit b);

    void add_output(AigLit lit, const std::string& name = "");
    void add_latch(AigLit next, AigLit init, const std::string& name = "");

    // --- Accessors ---
    size_t num_inputs()  const { return inputs_.size(); }
    size_t num_outputs() const { return outputs_.size(); }
    size_t num_ands()    const { return nodes_.size(); }
    size_t num_latches() const { return latches_.size(); }
    uint32_t max_var()   const { return next_var_ - 1; }

    const std::vector<uint32_t>&   inputs()      const { return inputs_; }
    const std::vector<AigLit>&     outputs()     const { return outputs_; }
    const std::vector<AigLatch>&   latches()     const { return latches_; }
    const std::vector<std::string>& input_names() const { return input_names_; }
    const std::vector<std::string>& output_names() const { return output_names_; }

    bool is_const(AigLit lit) const { return aig_var(lit) == 0; }
    bool is_input(uint32_t var) const;
    bool is_and(uint32_t var) const;
    bool is_latch(uint32_t var) const;
    const AigNode& and_node(uint32_t var) const;
    AigNode& and_node_mut(uint32_t var);

    // --- Simulation ---
    // Evaluate all nodes given input values. Returns value for each variable.
    std::vector<bool> evaluate(const std::vector<bool>& input_values) const;
    bool eval_lit(AigLit lit, const std::vector<bool>& node_vals) const;

    // --- Debug ---
    void print_stats() const;
    std::string to_dot() const;

private:
    std::vector<uint32_t>    inputs_;
    std::vector<std::string> input_names_;
    std::vector<AigNode>     nodes_;         // AND gate definitions
    std::vector<uint32_t>    and_vars_;      // Variable index for each AND gate
    std::vector<AigLit>      outputs_;
    std::vector<std::string> output_names_;
    std::vector<AigLatch>    latches_;
    std::vector<uint32_t>    latch_vars_;    // Variable index for each latch

    uint32_t next_var_ = 1;  // 0 = constant FALSE

    // Structural hashing: (min(f0,f1), max(f0,f1)) -> variable
    std::unordered_map<uint64_t, uint32_t> strash_;
    uint64_t strash_key(AigLit a, AigLit b) const {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(a) << 32) | b;
    }
};

// ============================================================================
// Multi-bit AIG vector — build n-bit circuits
// ============================================================================
class AigVec {
public:
    AigVec() = default;
    explicit AigVec(std::vector<AigLit> bits) : bits_(std::move(bits)) {}

    size_t width() const { return bits_.size(); }
    AigLit operator[](size_t i) const { return bits_[i]; }
    AigLit& operator[](size_t i) { return bits_[i]; }
    const std::vector<AigLit>& bits() const { return bits_; }

    static AigVec create_input(AigGraph& g, size_t w, const std::string& prefix = "");
    static AigVec bitwise_and(AigGraph& g, const AigVec& a, const AigVec& b);
    static AigVec bitwise_or(AigGraph& g, const AigVec& a, const AigVec& b);
    static AigVec bitwise_xor(AigGraph& g, const AigVec& a, const AigVec& b);
    static AigVec bitwise_not(AigGraph& g, const AigVec& a);
    static AigVec mux(AigGraph& g, AigLit sel, const AigVec& t, const AigVec& e);
    static AigLit eq(AigGraph& g, const AigVec& a, const AigVec& b);
    static AigVec add(AigGraph& g, const AigVec& a, const AigVec& b, AigLit cin = AIG_FALSE);

private:
    std::vector<AigLit> bits_;
};

} // namespace sf
