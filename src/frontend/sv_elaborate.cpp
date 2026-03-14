// SiliconForge — SystemVerilog Elaboration Engine (Implementation)
// Handles parameterized module instantiation, generate-block expansion,
// interface/modport binding, and recursive hierarchical elaboration.

#include "frontend/sv_elaborate.hpp"
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace sf {

// ============================================================================
// State management
// ============================================================================

void SvElaborator::add_module_template(const SvModuleTemplate& tmpl) {
    templates_.push_back(tmpl);
}

void SvElaborator::add_interface(const SvInterface& iface) {
    interfaces_.push_back(iface);
}

void SvElaborator::set_top_module(const std::string& name) {
    top_module_ = name;
}

void SvElaborator::set_parameter(const std::string& module,
                                  const std::string& param, int value) {
    param_overrides_[module][param] = value;
}

const SvModuleTemplate* SvElaborator::find_template(const std::string& name) const {
    for (auto& t : templates_) {
        if (t.name == name) return &t;
    }
    return nullptr;
}

// ============================================================================
// Parameter resolution — simple expression evaluator
// Supports: integer constants, parameter references, +, -, *, /, %, parentheses
// ============================================================================

static std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

namespace detail {

struct ExprParser {
    const std::string& src;
    const std::unordered_map<std::string, int>& params;
    size_t pos = 0;

    ExprParser(const std::string& s, const std::unordered_map<std::string, int>& p)
        : src(s), params(p) {}

    void skip_ws() {
        while (pos < src.size() && std::isspace(static_cast<unsigned char>(src[pos]))) ++pos;
    }

    int parse_primary() {
        skip_ws();
        if (pos >= src.size()) return 0;

        // Parenthesized sub-expression
        if (src[pos] == '(') {
            ++pos;
            int val = parse_additive();
            skip_ws();
            if (pos < src.size() && src[pos] == ')') ++pos;
            return val;
        }

        // Unary minus
        if (src[pos] == '-') {
            ++pos;
            return -parse_primary();
        }

        // Number
        if (std::isdigit(static_cast<unsigned char>(src[pos]))) {
            int val = 0;
            while (pos < src.size() && std::isdigit(static_cast<unsigned char>(src[pos]))) {
                val = val * 10 + (src[pos] - '0');
                ++pos;
            }
            return val;
        }

        // Identifier (parameter reference)
        if (std::isalpha(static_cast<unsigned char>(src[pos])) || src[pos] == '_') {
            std::string id;
            while (pos < src.size() &&
                   (std::isalnum(static_cast<unsigned char>(src[pos])) || src[pos] == '_')) {
                id += src[pos++];
            }
            auto it = params.find(id);
            return (it != params.end()) ? it->second : 0;
        }
        return 0;
    }

    int parse_multiplicative() {
        int left = parse_primary();
        while (true) {
            skip_ws();
            if (pos >= src.size()) break;
            char op = src[pos];
            if (op != '*' && op != '/' && op != '%') break;
            ++pos;
            int right = parse_primary();
            if (op == '*') left *= right;
            else if (op == '/' && right != 0) left /= right;
            else if (op == '%' && right != 0) left %= right;
        }
        return left;
    }

    int parse_additive() {
        int left = parse_multiplicative();
        while (true) {
            skip_ws();
            if (pos >= src.size()) break;
            char op = src[pos];
            if (op != '+' && op != '-') break;
            ++pos;
            int right = parse_multiplicative();
            if (op == '+') left += right;
            else left -= right;
        }
        return left;
    }

    int parse() { return parse_additive(); }
};

} // namespace detail

int SvElaborator::resolve_parameter(const std::string& expr,
                                     const std::unordered_map<std::string, int>& params) {
    std::string trimmed = trim(expr);
    if (trimmed.empty()) return 0;
    detail::ExprParser ep(trimmed, params);
    return ep.parse();
}

// ============================================================================
// Body instantiation — simplified gate-level netlist creation
// Each 'assign' → BUF gate, each 'always @(posedge clk)' → DFF
// ============================================================================

void SvElaborator::instantiate_body(const std::string& body, const std::string& prefix,
                                     Netlist& nl,
                                     const std::unordered_map<std::string, int>& params) {
    std::istringstream iss(body);
    std::string line;
    int assign_cnt = 0, dff_cnt = 0;

    while (std::getline(iss, line)) {
        std::string tl = trim(line);
        if (tl.empty()) continue;

        // Substitute parameter references in the line
        for (auto& [pname, pval] : params) {
            size_t fpos = 0;
            while ((fpos = tl.find(pname, fpos)) != std::string::npos) {
                bool left_ok = (fpos == 0) ||
                    !std::isalnum(static_cast<unsigned char>(tl[fpos - 1]));
                bool right_ok = (fpos + pname.size() >= tl.size()) ||
                    !std::isalnum(static_cast<unsigned char>(tl[fpos + pname.size()]));
                if (left_ok && right_ok) {
                    tl.replace(fpos, pname.size(), std::to_string(pval));
                }
                fpos += 1;
            }
        }

        if (tl.find("assign") == 0) {
            // Create a BUF gate for assign statements
            std::string net_name = prefix + "_assign_" + std::to_string(assign_cnt);
            NetId in_net = nl.add_net(net_name + "_in");
            NetId out_net = nl.add_net(net_name + "_out");
            nl.add_gate(GateType::BUF, {in_net}, out_net, net_name);
            assign_cnt++;
        } else if (tl.find("always") != std::string::npos &&
                   tl.find("posedge") != std::string::npos) {
            // Create a DFF for always @(posedge clk) blocks
            std::string dff_name = prefix + "_dff_" + std::to_string(dff_cnt);
            NetId d_net = nl.add_net(dff_name + "_d");
            NetId clk_net = nl.add_net(dff_name + "_clk");
            NetId q_net = nl.add_net(dff_name + "_q");
            nl.add_dff(d_net, clk_net, q_net, -1, dff_name);
            dff_cnt++;
        }
    }
}

// ============================================================================
// Generate-block expansion
// ============================================================================

int SvElaborator::expand_for_generate(const SvGenerateBlock& gen, Netlist& nl,
                                       const std::unordered_map<std::string, int>& params) {
    int count = 0;
    if (gen.step == 0) return 0;

    for (int i = gen.start;
         (gen.step > 0) ? (i < gen.end) : (i > gen.end);
         i += gen.step) {
        std::string prefix = gen.label + "[" + std::to_string(i) + "]";
        auto local_params = params;
        if (!gen.iterator.empty()) {
            local_params[gen.iterator] = i;
        }
        instantiate_body(gen.body, prefix, nl, local_params);
        count++;
    }
    return count;
}

int SvElaborator::expand_if_generate(const SvGenerateBlock& gen, Netlist& nl,
                                      const std::unordered_map<std::string, int>& params) {
    int cond_val = resolve_parameter(gen.condition, params);
    if (cond_val != 0) {
        std::string prefix = gen.label.empty() ? "if_gen" : gen.label;
        instantiate_body(gen.body, prefix, nl, params);
        return 1;
    }
    return 0;
}

int SvElaborator::expand_case_generate(const SvGenerateBlock& gen, Netlist& nl,
                                        const std::unordered_map<std::string, int>& params) {
    int sel = resolve_parameter(gen.condition, params);
    auto it = gen.case_items.find(sel);
    if (it != gen.case_items.end()) {
        std::string prefix = gen.label.empty() ? "case_gen" : gen.label;
        instantiate_body(it->second, prefix, nl, params);
        return 1;
    }
    if (!gen.default_body.empty()) {
        std::string prefix = gen.label.empty() ? "case_gen_default" : gen.label + "_default";
        instantiate_body(gen.default_body, prefix, nl, params);
        return 1;
    }
    return 0;
}

// ============================================================================
// Interface / modport binding
// ============================================================================

void SvElaborator::bind_interfaces(const SvModuleTemplate& tmpl, Netlist& nl,
                                    ElaborateResult& res) {
    for (auto& port_name : tmpl.interface_ports) {
        // Parse "interface_name.modport_name" format
        auto dot = port_name.find('.');
        std::string iface_name = (dot != std::string::npos) ? port_name.substr(0, dot) : port_name;
        std::string mp_name = (dot != std::string::npos) ? port_name.substr(dot + 1) : "";

        for (auto& iface : interfaces_) {
            if (iface.name != iface_name) continue;

            // Find matching modport (if specified)
            const SvInterface::Modport* mp = nullptr;
            if (!mp_name.empty()) {
                for (auto& m : iface.modports) {
                    if (m.name == mp_name) { mp = &m; break; }
                }
            }

            // Create nets for interface signals
            for (auto& sig : iface.signals) {
                std::string net_name = iface_name + "_" + sig.name;
                for (int b = 0; b < sig.width; ++b) {
                    std::string bit_name = (sig.width > 1)
                        ? net_name + "[" + std::to_string(b) + "]"
                        : net_name;
                    NetId nid = nl.add_net(bit_name);

                    // Apply modport directions
                    SvInterface::Signal::Dir dir = sig.dir;
                    if (mp) {
                        for (auto& [pn, pd] : mp->port_dirs) {
                            if (pn == sig.name) { dir = pd; break; }
                        }
                    }
                    if (dir == SvInterface::Signal::Dir::INPUT) nl.mark_input(nid);
                    else if (dir == SvInterface::Signal::Dir::OUTPUT) nl.mark_output(nid);
                }
            }
            res.interfaces_bound++;
            break;
        }
    }
}

// ============================================================================
// Recursive elaboration
// ============================================================================

void SvElaborator::elaborate_recursive(const std::string& module_name,
                                        const std::string& prefix,
                                        Netlist& nl, ElaborateResult& res,
                                        const std::unordered_map<std::string, int>& param_ctx,
                                        int depth) {
    if (depth > 32) {
        res.error = "Maximum elaboration depth exceeded";
        return;
    }

    const SvModuleTemplate* tmpl = find_template(module_name);
    if (!tmpl) {
        res.error = "Module template '" + module_name + "' not found";
        return;
    }

    // Build resolved parameter map: defaults → overrides → context
    std::unordered_map<std::string, int> resolved = param_ctx;
    for (auto& p : tmpl->parameters) {
        if (resolved.find(p.name) == resolved.end()) {
            resolved[p.name] = p.default_value;
        }
    }
    auto ov_it = param_overrides_.find(module_name);
    if (ov_it != param_overrides_.end()) {
        for (auto& [k, v] : ov_it->second) {
            resolved[k] = v;
        }
    }

    // Evaluate parameter expressions (defaults may reference other params)
    for (auto& p : tmpl->parameters) {
        if (!p.is_local && resolved.find(p.name) != resolved.end()) {
            res.parameters_resolved++;
        }
    }

    // Expand generate blocks
    for (auto& gen : tmpl->generates) {
        int inst = 0;
        switch (gen.type) {
            case SvGenerateBlock::FOR_GENERATE:
                inst = expand_for_generate(gen, nl, resolved);
                break;
            case SvGenerateBlock::IF_GENERATE:
                inst = expand_if_generate(gen, nl, resolved);
                break;
            case SvGenerateBlock::CASE_GENERATE:
                inst = expand_case_generate(gen, nl, resolved);
                break;
        }
        res.instances_generated += inst;
        if (inst > 0) res.generate_blocks_expanded++;
    }

    // Instantiate module body
    instantiate_body(tmpl->body, prefix.empty() ? module_name : prefix, nl, resolved);

    // Bind interfaces
    bind_interfaces(*tmpl, nl, res);

    // Look for sub-module instantiations in body (pattern: "inst <module_name>")
    std::istringstream iss(tmpl->body);
    std::string line;
    while (std::getline(iss, line)) {
        std::string tl = trim(line);
        if (tl.find("inst ") == 0) {
            std::string sub_mod = tl.substr(5);
            sub_mod = trim(sub_mod);
            // Remove trailing semicolon
            if (!sub_mod.empty() && sub_mod.back() == ';') sub_mod.pop_back();
            sub_mod = trim(sub_mod);
            if (find_template(sub_mod)) {
                std::string sub_prefix = (prefix.empty() ? module_name : prefix) + "/" + sub_mod;
                elaborate_recursive(sub_mod, sub_prefix, nl, res, {}, depth + 1);
                res.modules_elaborated++;
            }
        }
    }

    res.modules_elaborated++;
}

// ============================================================================
// Top-level elaborate
// ============================================================================

ElaborateResult SvElaborator::elaborate(Netlist& nl) {
    ElaborateResult res;

    if (top_module_.empty() && !templates_.empty()) {
        top_module_ = templates_.front().name;
    }

    if (top_module_.empty()) {
        res.error = "No top module specified";
        return res;
    }

    elaborate_recursive(top_module_, "", nl, res, {}, 0);

    if (res.error.empty()) {
        res.success = true;
    }
    return res;
}

// ============================================================================
// Enhanced flow with report generation
// ============================================================================

ElaborateResult SvElaborator::run_enhanced(Netlist& nl) {
    ElaborateResult res = elaborate(nl);

    std::ostringstream rpt;
    rpt << "=== SystemVerilog Elaboration Report ===\n";
    rpt << "Top module:            " << top_module_ << "\n";
    rpt << "Module templates:      " << templates_.size() << "\n";
    rpt << "Interfaces defined:    " << interfaces_.size() << "\n";
    rpt << "Modules elaborated:    " << res.modules_elaborated << "\n";
    rpt << "Instances generated:   " << res.instances_generated << "\n";
    rpt << "Parameters resolved:   " << res.parameters_resolved << "\n";
    rpt << "Generate blocks expanded: " << res.generate_blocks_expanded << "\n";
    rpt << "Interfaces bound:      " << res.interfaces_bound << "\n";
    rpt << "Netlist nets:          " << nl.num_nets() << "\n";
    rpt << "Netlist gates:         " << nl.num_gates() << "\n";
    rpt << "Status:                " << (res.success ? "SUCCESS" : "FAILED") << "\n";
    if (!res.error.empty()) {
        rpt << "Error:                 " << res.error << "\n";
    }
    rpt << "========================================\n";

    res.report = rpt.str();
    return res;
}

} // namespace sf
