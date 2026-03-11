// SiliconForge — And-Inverter Graph Implementation
#include "core/aig.hpp"
#include <algorithm>
#include <sstream>
#include <cassert>

namespace sf {

AigLit AigGraph::create_input(const std::string& name) {
    uint32_t var = next_var_++;
    inputs_.push_back(var);
    input_names_.push_back(name.empty() ? ("i" + std::to_string(inputs_.size() - 1)) : name);
    return aig_make(var);
}

AigLit AigGraph::create_and(AigLit a, AigLit b) {
    // Trivial simplifications
    if (a == AIG_FALSE || b == AIG_FALSE) return AIG_FALSE;
    if (a == AIG_TRUE) return b;
    if (b == AIG_TRUE) return a;
    if (a == b) return a;
    if (a == aig_not(b)) return AIG_FALSE;
    // Canonical order
    if (a > b) std::swap(a, b);
    // Structural hashing
    uint64_t key = strash_key(a, b);
    auto it = strash_.find(key);
    if (it != strash_.end()) return aig_make(it->second);
    // New AND gate
    uint32_t var = next_var_++;
    nodes_.push_back({a, b});
    and_vars_.push_back(var);
    strash_[key] = var;
    return aig_make(var);
}

AigLit AigGraph::create_or(AigLit a, AigLit b) {
    return aig_not(create_and(aig_not(a), aig_not(b)));
}

AigLit AigGraph::create_xor(AigLit a, AigLit b) {
    AigLit t1 = create_and(a, aig_not(b));
    AigLit t2 = create_and(aig_not(a), b);
    return create_or(t1, t2);
}

AigLit AigGraph::create_mux(AigLit sel, AigLit t, AigLit e) {
    return create_or(create_and(sel, t), create_and(aig_not(sel), e));
}

AigLit AigGraph::create_nand(AigLit a, AigLit b) { return aig_not(create_and(a, b)); }
AigLit AigGraph::create_nor(AigLit a, AigLit b)  { return aig_not(create_or(a, b)); }

void AigGraph::add_output(AigLit lit, const std::string& name) {
    outputs_.push_back(lit);
    output_names_.push_back(name.empty() ? ("o" + std::to_string(outputs_.size() - 1)) : name);
}

void AigGraph::add_latch(AigLit next, AigLit init, const std::string& name) {
    uint32_t var = next_var_++;
    latch_vars_.push_back(var);
    latches_.push_back({next, init, name.empty() ? ("l" + std::to_string(latches_.size())) : name});
}

bool AigGraph::is_input(uint32_t var) const {
    for (auto v : inputs_) if (v == var) return true;
    return false;
}

bool AigGraph::is_and(uint32_t var) const {
    for (auto v : and_vars_) if (v == var) return true;
    return false;
}

bool AigGraph::is_latch(uint32_t var) const {
    for (auto v : latch_vars_) if (v == var) return true;
    return false;
}

const AigNode& AigGraph::and_node(uint32_t var) const {
    for (size_t i = 0; i < and_vars_.size(); ++i)
        if (and_vars_[i] == var) return nodes_[i];
    throw std::runtime_error("Variable " + std::to_string(var) + " is not an AND gate");
}

AigNode& AigGraph::and_node_mut(uint32_t var) {
    for (size_t i = 0; i < and_vars_.size(); ++i)
        if (and_vars_[i] == var) return nodes_[i];
    throw std::runtime_error("Variable " + std::to_string(var) + " is not an AND gate");
}

std::vector<bool> AigGraph::evaluate(const std::vector<bool>& input_values) const {
    std::vector<bool> vals(next_var_, false);
    for (size_t i = 0; i < inputs_.size(); ++i)
        vals[inputs_[i]] = input_values[i];
    for (size_t i = 0; i < latches_.size(); ++i)
        vals[latch_vars_[i]] = (latches_[i].init == AIG_TRUE);
    for (size_t i = 0; i < nodes_.size(); ++i) {
        bool v0 = vals[aig_var(nodes_[i].fanin0)] ^ aig_sign(nodes_[i].fanin0);
        bool v1 = vals[aig_var(nodes_[i].fanin1)] ^ aig_sign(nodes_[i].fanin1);
        vals[and_vars_[i]] = v0 && v1;
    }
    return vals;
}

bool AigGraph::eval_lit(AigLit lit, const std::vector<bool>& node_vals) const {
    return node_vals[aig_var(lit)] ^ aig_sign(lit);
}

void AigGraph::print_stats() const {
    std::cout << "AIG: " << inputs_.size() << " inputs, "
              << nodes_.size() << " ANDs, "
              << outputs_.size() << " outputs, "
              << latches_.size() << " latches\n";
}

std::string AigGraph::to_dot() const {
    std::ostringstream ss;
    ss << "digraph AIG {\n  rankdir=BT;\n";
    for (size_t i = 0; i < inputs_.size(); ++i)
        ss << "  n" << inputs_[i] << " [label=\"" << input_names_[i] << "\" shape=triangle];\n";
    for (size_t i = 0; i < nodes_.size(); ++i) {
        uint32_t v = and_vars_[i];
        ss << "  n" << v << " [label=\"&\" shape=ellipse];\n";
        ss << "  n" << aig_var(nodes_[i].fanin0) << " -> n" << v
           << (aig_sign(nodes_[i].fanin0) ? " [style=dashed]" : "") << ";\n";
        ss << "  n" << aig_var(nodes_[i].fanin1) << " -> n" << v
           << (aig_sign(nodes_[i].fanin1) ? " [style=dashed]" : "") << ";\n";
    }
    for (size_t i = 0; i < outputs_.size(); ++i) {
        ss << "  out" << i << " [label=\"" << output_names_[i] << "\" shape=box color=red];\n";
        ss << "  n" << aig_var(outputs_[i]) << " -> out" << i
           << (aig_sign(outputs_[i]) ? " [style=dashed]" : "") << ";\n";
    }
    ss << "}\n";
    return ss.str();
}

// --- AigVec multi-bit operations ---

AigVec AigVec::create_input(AigGraph& g, size_t w, const std::string& prefix) {
    std::vector<AigLit> bits(w);
    for (size_t i = 0; i < w; ++i)
        bits[i] = g.create_input(prefix + "[" + std::to_string(i) + "]");
    return AigVec(std::move(bits));
}

AigVec AigVec::bitwise_and(AigGraph& g, const AigVec& a, const AigVec& b) {
    assert(a.width() == b.width());
    std::vector<AigLit> bits(a.width());
    for (size_t i = 0; i < a.width(); ++i) bits[i] = g.create_and(a[i], b[i]);
    return AigVec(std::move(bits));
}

AigVec AigVec::bitwise_or(AigGraph& g, const AigVec& a, const AigVec& b) {
    assert(a.width() == b.width());
    std::vector<AigLit> bits(a.width());
    for (size_t i = 0; i < a.width(); ++i) bits[i] = g.create_or(a[i], b[i]);
    return AigVec(std::move(bits));
}

AigVec AigVec::bitwise_xor(AigGraph& g, const AigVec& a, const AigVec& b) {
    assert(a.width() == b.width());
    std::vector<AigLit> bits(a.width());
    for (size_t i = 0; i < a.width(); ++i) bits[i] = g.create_xor(a[i], b[i]);
    return AigVec(std::move(bits));
}

AigVec AigVec::bitwise_not(AigGraph& g, const AigVec& a) {
    (void)g;
    std::vector<AigLit> bits(a.width());
    for (size_t i = 0; i < a.width(); ++i) bits[i] = aig_not(a[i]);
    return AigVec(std::move(bits));
}

AigVec AigVec::mux(AigGraph& g, AigLit sel, const AigVec& t, const AigVec& e) {
    assert(t.width() == e.width());
    std::vector<AigLit> bits(t.width());
    for (size_t i = 0; i < t.width(); ++i) bits[i] = g.create_mux(sel, t[i], e[i]);
    return AigVec(std::move(bits));
}

AigLit AigVec::eq(AigGraph& g, const AigVec& a, const AigVec& b) {
    assert(a.width() == b.width());
    AigLit result = AIG_TRUE;
    for (size_t i = 0; i < a.width(); ++i)
        result = g.create_and(result, aig_not(g.create_xor(a[i], b[i])));
    return result;
}

AigVec AigVec::add(AigGraph& g, const AigVec& a, const AigVec& b, AigLit cin) {
    assert(a.width() == b.width());
    std::vector<AigLit> sum(a.width());
    AigLit carry = cin;
    for (size_t i = 0; i < a.width(); ++i) {
        AigLit ab_xor = g.create_xor(a[i], b[i]);
        sum[i] = g.create_xor(ab_xor, carry);
        AigLit ab_and = g.create_and(a[i], b[i]);
        AigLit ac_and = g.create_and(a[i], carry);
        AigLit bc_and = g.create_and(b[i], carry);
        carry = g.create_or(ab_and, g.create_or(ac_and, bc_and));
    }
    return AigVec(std::move(sum));
}

} // namespace sf
