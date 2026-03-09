#pragma once
// SiliconForge — Logic Visualizer Dashboard
// Consolidates Formal Verification (BMC), Logic Synthesis (AIG reductions), 
// and DFT (Fault Coverage) statistics into interactive HTML charts.

#include <string>
#include <vector>

namespace sf {

// Simple metrics structures to decouple Viz from heavy dependencies
struct FormalMetrics {
    int max_depth_unrolled = 0;
    int properties_proved = 0;
    int properties_failed = 0;
    // Trace length if failed
    int cex_cycle = -1;
};

struct SynthesisMetrics {
    std::vector<int> node_count_history; 
    std::vector<std::string> pass_names;
    int initial_nodes = 0;
    int final_nodes = 0;
};

struct DftMetrics {
    int total_faults = 0;
    int detected_faults = 0;
    double coverage_pct = 0;
    int scan_chain_length = 0;
    std::vector<std::vector<double>> fake_heat_map; // Mock of module density for demo
};

class LogicVisualizer {
public:
    LogicVisualizer() = default;

    void set_formal(const FormalMetrics& m) { formal_ = m; }
    void set_synth(const SynthesisMetrics& m) { synth_ = m; }
    void set_dft(const DftMetrics& m) { dft_ = m; }

    std::string generate_html() const;
    bool export_to_file(const std::string& filename) const;

private:
    FormalMetrics formal_;
    SynthesisMetrics synth_;
    DftMetrics dft_;

    std::string generate_css() const;
    std::string generate_js_logic() const;
    std::string generate_json_data() const;
};

} // namespace sf
