// SiliconForge — Scan Chain Insertion Implementation
#include "dft/scan_insert.hpp"
#include <iostream>
#include <algorithm>

namespace sf {

ScanResult ScanInserter::insert(const ScanConfig& config) {
    auto& ffs = nl_.flip_flops();
    ScanResult result;
    result.total_ffs = (int)ffs.size();

    if (ffs.empty()) {
        result.message = "No flip-flops to scan";
        return result;
    }

    // Create scan enable net
    NetId se = nl_.add_net(config.scan_enable_name);
    nl_.mark_input(se);

    // Cache FF ids before modification (avoids dangling refs during vector growth)
    std::vector<GateId> ff_ids(ffs.begin(), ffs.end());

    // Split FFs into chains
    int chain_id = 0;
    for (size_t i = 0; i < ff_ids.size(); i += config.max_chain_length) {
        size_t chain_end = std::min(i + (size_t)config.max_chain_length, ff_ids.size());
        int chain_len = (int)(chain_end - i);

        // Create scan_in for this chain
        NetId scan_in = nl_.add_net(config.scan_in_prefix + "_" + std::to_string(chain_id));
        nl_.mark_input(scan_in);

        // For each FF in the chain, insert a MUX before the D input
        NetId prev_q = scan_in;

        for (size_t j = i; j < chain_end; ++j) {
            GateId ff_id = ff_ids[j];
            NetId func_d = nl_.gate(ff_id).inputs.empty() ? -1 : nl_.gate(ff_id).inputs[0];
            NetId ff_output = nl_.gate(ff_id).output;

            if (func_d >= 0) {
                NetId mux_out = nl_.add_net("scan_mux_" + std::to_string(j));
                nl_.add_gate(GateType::MUX, {se, prev_q, func_d}, mux_out,
                            "SCAN_MUX_" + std::to_string(j));
                // Re-fetch after add (vectors may have grown)
                nl_.gate(ff_id).inputs[0] = mux_out;
            }

            prev_q = ff_output;
        }

        // Last FF's Q output is scan_out for this chain
        NetId scan_out = nl_.add_net(config.scan_out_prefix + "_" + std::to_string(chain_id));
        GateId last_ff = ff_ids[chain_end - 1];
        NetId last_q = nl_.gate(last_ff).output;
        if (last_q >= 0) {
            nl_.add_gate(GateType::BUF, {last_q}, scan_out, "SCAN_OUT_BUF_" + std::to_string(chain_id));
        }
        nl_.mark_output(scan_out);

        result.chain_lengths.push_back(chain_len);
        chain_id++;
    }

    result.num_chains = chain_id;
    result.message = "Inserted " + std::to_string(chain_id) + " scan chain(s) covering " +
                     std::to_string(result.total_ffs) + " FFs";
    return result;
}

} // namespace sf
