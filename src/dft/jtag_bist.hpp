#pragma once
// SiliconForge — JTAG TAP Controller, Boundary Scan & Built-In Self-Test
// Implements IEEE 1149.1 TAP state machine, boundary scan register,
// LFSR/MISR-based Logic BIST, and March-algorithm Memory BIST.

#include "core/netlist.hpp"
#include "dft/scan_insert.hpp"
#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include <map>
#include <sstream>

namespace sf {

// ── IEEE 1149.1 TAP State Machine ───────────────────────────────────────────

enum class TapState {
    TEST_LOGIC_RESET,
    RUN_TEST_IDLE,
    SELECT_DR_SCAN,
    CAPTURE_DR,
    SHIFT_DR,
    EXIT1_DR,
    PAUSE_DR,
    EXIT2_DR,
    UPDATE_DR,
    SELECT_IR_SCAN,
    CAPTURE_IR,
    SHIFT_IR,
    EXIT1_IR,
    PAUSE_IR,
    EXIT2_IR,
    UPDATE_IR
};

class TapController {
public:
    TapController() : state_(TapState::TEST_LOGIC_RESET) {}

    void clock_tms(bool tms);
    TapState state() const { return state_; }
    void reset();

    // transition_table[state][tms] → next state
    static const TapState transition_table[16][2];

private:
    TapState state_;
};

// ── JTAG Instructions & Configuration ───────────────────────────────────────

enum class JtagInstruction {
    BYPASS,
    IDCODE,
    EXTEST,
    INTEST,
    SAMPLE_PRELOAD,
    USERCODE,
    CLAMP,
    HIGHZ,
    USER1,
    USER2
};

struct JtagConfig {
    int ir_length = 4;
    uint32_t idcode = 0x149511C3;
    int num_boundary_cells = 0;
    std::string device_name = "siliconforge";
    bool has_trst = true;
};

// ── Boundary Scan Register ──────────────────────────────────────────────────

enum class BoundaryCellType { BC_1, BC_2, BC_4, BC_7 };

struct BoundaryCell {
    BoundaryCellType type;
    std::string pin_name;
    std::string cell_function; // "input", "output", "control", "bidir"
};

class BoundaryScanRegister {
public:
    void add_cell(const BoundaryCell& cell);

    void capture();
    bool shift(bool tdi);
    void update();

    std::vector<bool> get_parallel_outputs() const;
    void set_parallel_inputs(const std::vector<bool>& inputs);

    size_t size() const { return cells_.size(); }

private:
    std::vector<BoundaryCell> cells_;
    std::vector<bool> shift_reg_;
    std::vector<bool> parallel_in_;
    std::vector<bool> parallel_out_;
};

// ── JTAG Controller ─────────────────────────────────────────────────────────

class JtagController {
public:
    explicit JtagController(const JtagConfig& cfg = {});

    void shift_ir(const std::vector<bool>& ir_bits);
    std::vector<bool> shift_dr(const std::vector<bool>& tdi_bits);
    uint32_t read_idcode();
    bool bypass_test();
    JtagInstruction current_instruction() const { return current_instr_; }

    TapController& tap() { return tap_; }
    BoundaryScanRegister& bsr() { return bsr_; }

private:
    JtagInstruction decode_ir(uint32_t ir_val) const;
    void navigate_to(TapState target);

    JtagConfig cfg_;
    TapController tap_;
    BoundaryScanRegister bsr_;
    JtagInstruction current_instr_ = JtagInstruction::IDCODE;
    uint32_t ir_value_ = 0;
    bool bypass_reg_ = false;
    uint32_t idcode_reg_ = 0;
};

// ── LFSR (Linear Feedback Shift Register) ───────────────────────────────────

struct LfsrConfig {
    int width = 16;
    uint32_t polynomial = 0;  // 0 = use standard_polynomial(width)
    uint32_t seed = 1;
};

class Lfsr {
public:
    explicit Lfsr(const LfsrConfig& cfg = {});

    void clock();
    uint32_t value() const { return state_; }
    void reset();
    std::vector<bool> get_pattern(int num_bits) const;

    static uint32_t standard_polynomial(int width);

private:
    int width_;
    uint32_t polynomial_;
    uint32_t seed_;
    uint32_t state_;
};

// ── MISR (Multiple Input Signature Register) ────────────────────────────────

class Misr {
public:
    explicit Misr(int width = 32, uint32_t polynomial = 0);

    void compress(const std::vector<bool>& response);
    uint32_t signature() const { return state_; }
    void reset();

private:
    int width_;
    uint32_t polynomial_;
    uint32_t state_ = 0;
};

// ── Logic BIST ──────────────────────────────────────────────────────────────

struct LbistConfig {
    int num_patterns = 1000;
    int scan_chain_length = 100;
    int num_scan_chains = 1;
};

struct LbistResult {
    uint32_t signature = 0;
    int patterns_applied = 0;
    bool pass = false;
    std::string report;
};

class LbistController {
public:
    LbistResult run(Netlist& nl, const LbistConfig& cfg);

private:
    std::vector<bool> generate_pattern(Lfsr& lfsr, int length);
    std::vector<bool> simulate_pattern(Netlist& nl, const std::vector<bool>& pattern);
};

// ── Memory BIST ─────────────────────────────────────────────────────────────

enum class MarchAlgorithm {
    MARCH_C_MINUS,
    MARCH_X,
    MARCH_Y,
    MARCH_LR,
    CHECKERBOARD
};

struct MbistConfig {
    int words = 256;
    int bits = 8;
    MarchAlgorithm algo = MarchAlgorithm::MARCH_C_MINUS;
    std::string mem_name = "mem0";
};

struct MbistResult {
    bool pass = true;
    int faults_detected = 0;
    std::vector<std::pair<int, int>> fault_locations; // (address, bit_position)
    std::string report;
};

class MbistController {
public:
    MbistResult run(const MbistConfig& cfg);

private:
    using MemArray = std::vector<std::vector<bool>>;

    void write_cell(MemArray& mem, int addr, int bit, bool val);
    bool read_cell(const MemArray& mem, int addr, int bit) const;

    MbistResult run_march_c_minus(const MbistConfig& cfg, MemArray& mem);
    MbistResult run_march_x(const MbistConfig& cfg, MemArray& mem);
    MbistResult run_march_y(const MbistConfig& cfg, MemArray& mem);
    MbistResult run_checkerboard(const MbistConfig& cfg, MemArray& mem);

    void inject_faults(MemArray& mem, int num_faults);
    bool check_and_record(const MemArray& mem, int addr, int bit,
                          bool expected, MbistResult& result);
};

// ── BSDL Generator ─────────────────────────────────────────────────────────

class BsdlGenerator {
public:
    std::string generate(const JtagConfig& cfg,
                         const std::vector<BoundaryCell>& cells);
};

// ── DFT Integration ─────────────────────────────────────────────────────────

struct DftConfig {
    ScanConfig scan;
    JtagConfig jtag;
    LbistConfig lbist;
    std::vector<MbistConfig> mbist;
};

struct DftResult {
    ScanResult scan;
    JtagConfig jtag;
    LbistResult lbist;
    std::vector<MbistResult> mbist;
    std::string bsdl;
    double coverage_estimate = 0.0;
    std::string report;
};

class DftIntegrator {
public:
    DftResult run_full_dft(Netlist& nl, const DftConfig& cfg);
};

} // namespace sf
