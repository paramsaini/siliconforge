// SiliconForge — Technology Mapper Implementation
#include "synth/tech_mapper.hpp"
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <algorithm>

namespace sf {

TechMapper::CellMatch TechMapper::match_node(uint32_t var) {
    if (!aig_.is_and(var)) return {nullptr, {}, AIG_FALSE};

    const auto& nd = aig_.and_node(var);
    CellMatch best{nullptr, {}, aig_make(var)};

    // Try to match 2-input cells
    // AND gate: output = fanin0 AND fanin1
    // Both non-inverted → AND2
    if (!aig_sign(nd.fanin0) && !aig_sign(nd.fanin1)) {
        auto* cell = lib_.find_cell("AND2X1");
        if (!cell) cell = lib_.find_cell("AND2_X1");
        if (!cell) {
            for (auto& c : lib_.cells) {
                if (c.output_function() == "A & B" || c.output_function() == "A&B") {
                    cell = &c; break;
                }
            }
        }
        if (cell) {
            best.cell = cell;
            best.inputs = {nd.fanin0, nd.fanin1};
            return best;
        }
    }

    // NAND: output = NOT(fanin0 AND fanin1)
    // This is represented as an AND node whose output is used inverted
    // We'll handle this at the output level

    // If one input is inverted: check for specific patterns
    // AND(NOT a, b) can map to... we'll just use AND + INV

    // Default: use NAND + INV or just AND
    // Find any 2-input cell
    for (auto& c : lib_.cells) {
        if (c.num_inputs() == 2 && !c.output_function().empty()) {
            std::string func = c.output_function();
            // Simple match for basic gates
            if (func == "A & B" || func == "A&B") {
                if (!aig_sign(nd.fanin0) && !aig_sign(nd.fanin1)) {
                    best.cell = &c;
                    best.inputs = {nd.fanin0, nd.fanin1};
                    return best;
                }
            }
            if (func == "!(A & B)" || func == "!(A&B)") {
                best.cell = &c;
                best.inputs = {nd.fanin0, nd.fanin1};
                return best;
            }
        }
    }

    // Fallback: generic 2-input gate
    if (!best.cell && !lib_.cells.empty()) {
        for (auto& c : lib_.cells) {
            if (c.num_inputs() == 2) {
                best.cell = &c;
                best.inputs = {nd.fanin0, nd.fanin1};
                return best;
            }
        }
    }

    return best;
}

Netlist TechMapper::build_netlist(const std::vector<CellMatch>& matches) {
    Netlist nl;
    std::unordered_map<uint32_t, NetId> var_to_net;

    // Create nets for inputs
    for (size_t i = 0; i < aig_.num_inputs(); ++i) {
        uint32_t v = aig_.inputs()[i];
        std::string iname = i < aig_.input_names().size() ? aig_.input_names()[i] : ("i" + std::to_string(i));
        NetId n = nl.add_net(iname);
        nl.mark_input(n);
        var_to_net[v] = n;
    }

    // Create constant net
    var_to_net[0] = nl.add_net("GND");

    // Create gates from matches
    for (auto& m : matches) {
        if (!m.cell) continue;

        // Ensure output net exists
        uint32_t out_var = aig_var(m.output);
        if (!var_to_net.count(out_var))
            var_to_net[out_var] = nl.add_net("n" + std::to_string(out_var));

        // Collect input nets
        std::vector<NetId> in_nets;
        for (auto lit : m.inputs) {
            uint32_t v = aig_var(lit);
            if (!var_to_net.count(v))
                var_to_net[v] = nl.add_net("n" + std::to_string(v));

            NetId net = var_to_net[v];
            if (aig_sign(lit)) {
                // Need an inverter
                NetId inv_out = nl.add_net("inv_n" + std::to_string(v));
                nl.add_gate(GateType::NOT, {net}, inv_out, "INV_" + std::to_string(v));
                in_nets.push_back(inv_out);
            } else {
                in_nets.push_back(net);
            }
        }

        // Determine gate type from cell function
        GateType gtype = GateType::AND;
        std::string func = m.cell->output_function();
        if (func.find('|') != std::string::npos) gtype = GateType::OR;
        if (func.find('^') != std::string::npos) gtype = GateType::XOR;
        if (func[0] == '!') {
            if (func.find('&') != std::string::npos) gtype = GateType::NAND;
            else if (func.find('|') != std::string::npos) gtype = GateType::NOR;
        }

        nl.add_gate(gtype, in_nets, var_to_net[out_var], m.cell->name + "_i" + std::to_string(out_var));
        stats_.total_area += m.cell->area;
        stats_.num_cells++;
    }

    // Mark outputs
    for (size_t i = 0; i < aig_.num_outputs(); ++i) {
        AigLit olit = aig_.outputs()[i];
        uint32_t v = aig_var(olit);
        if (!var_to_net.count(v))
            var_to_net[v] = nl.add_net("o" + std::to_string(i));

        NetId out_net = var_to_net[v];
        if (aig_sign(olit)) {
            NetId inv_out = nl.add_net("out_" + std::to_string(i));
            nl.add_gate(GateType::NOT, {out_net}, inv_out, "OUT_INV_" + std::to_string(i));
            nl.mark_output(inv_out);
        } else {
            nl.mark_output(out_net);
        }
    }

    return nl;
}

Netlist TechMapper::map(bool optimize_area) {
    auto t0 = std::chrono::high_resolution_clock::now();
    stats_ = {};

    // Match each AND gate to a cell
    std::vector<CellMatch> matches;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v)) continue;
        auto m = match_node(v);
        if (m.cell) matches.push_back(m);
    }

    auto nl = build_netlist(matches);
    auto t1 = std::chrono::high_resolution_clock::now();
    stats_.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return nl;
}

} // namespace sf
