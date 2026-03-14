#pragma once
// SiliconForge — Verilog Parser (Structural + Behavioral)
// Supports: module, input, output, wire, assign, gate primitives, always blocks,
//           full expression parsing with operator precedence, for loops, parameters.

#include "core/netlist.hpp"
#include "synth/behavioral_synth.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>

namespace sf {

struct SpecifyPath {
    std::string from_port;
    std::string to_port;
    bool is_full_path = true;    // (a => b) vs (a *> b)
    bool is_posedge = false;
    bool is_negedge = false;
    double rise_delay = 0.0;     // ns
    double fall_delay = 0.0;     // ns
};

struct SpecifyTimingCheck {
    enum Type { SETUP, HOLD, RECOVERY, REMOVAL, WIDTH, PERIOD };
    Type type = SETUP;
    std::string data_port;
    std::string ref_port;        // clock or reference signal
    bool ref_edge_pos = true;    // posedge/negedge of reference
    double limit_ns = 0.0;       // timing limit value
};

struct SpecifyBlock {
    std::vector<SpecifyPath> paths;
    std::vector<SpecifyTimingCheck> timing_checks;
    std::unordered_map<std::string, double> specparams; // specparam name → value
};

struct VerilogParseResult {
    bool success = false;
    std::string module_name;
    int num_inputs = 0;
    int num_outputs = 0;
    int num_wires = 0;
    int num_gates = 0;
    int num_ffs = 0;
    std::string error;
    SpecifyBlock specify_block;
};

class VerilogParser {
public:
    bool has_behavioral_blocks = false;
    BehavioralAST ast;

    VerilogParseResult parse_string(const std::string& src, Netlist& nl);
    VerilogParseResult parse_file(const std::string& filename, Netlist& nl);
    static std::string to_verilog(const Netlist& nl, const std::string& module_name = "top");

    // Tier 3: Enhanced post-synthesis Verilog writer
    struct VerilogWriterConfig {
        bool emit_hierarchy = false;
        bool emit_attributes = true;       // (* dont_touch *), (* keep *)
        bool emit_drive_strength = true;    // cell drive info in comments
        bool emit_bus_notation = true;      // wire [7:0] bus vs scalar wires
        bool emit_power_pins = false;       // VDD/VSS pin connections
        std::string default_lib = "std_cell_lib";
        std::unordered_set<std::string> dont_touch_nets;
        std::unordered_set<std::string> dont_touch_cells;
    };
    static std::string to_verilog_enhanced(const Netlist& nl,
                                           const std::string& module_name,
                                           const VerilogWriterConfig& cfg);

private:
    struct Token {
        enum Type {
            IDENT, NUMBER, LPAREN, RPAREN, LBRACKET, RBRACKET,
            COMMA, SEMI, DOT, ASSIGN, TILDE, AMP, PIPE, CARET,
            LBRACE, RBRACE, AT, HASH, COLON, QUESTION,
            // Behavioral keywords
            ALWAYS, IF, ELSE, BEGIN_KW, END_KW, POSEDGE, NEGEDGE,
            LEQ, PLUS, MINUS, STAR, EQEQ, NEQ,
            CASE_KW, ENDCASE_KW, DEFAULT_KW,
            // New operators
            LSHIFT, RSHIFT,      // << >>
            LT, GT, GTE,         // < > >=
            LAND, LOR,           // && ||
            BANG,                 // !
            SLASH, PERCENT,      // / %
            // New keywords
            FOR_KW,              // for
            PARAMETER_KW,        // parameter, localparam
            SIGNED_KW,           // signed
            INTEGER_KW,          // integer
            // Phase 16 — new keywords
            GENERATE_KW, ENDGENERATE_KW, GENVAR_KW,
            FUNCTION_KW, ENDFUNCTION_KW, TASK_KW, ENDTASK_KW,
            WHILE_KW, REPEAT_KW, FOREVER_KW,
            INITIAL_KW,
            ARSHIFT,             // >>>
            // Phase 17 — remaining Verilog-2001
            SPECIFY_KW, ENDSPECIFY_KW,
            DISABLE_KW,
            DEFPARAM_KW,
            // Phase 18 — final Verilog-2001 coverage
            XNOR_OP,            // ~^ or ^~
            POWER_OP,            // **
            CASE_EQ, CASE_NEQ,   // === !==
            PLUS_COLON, MINUS_COLON, // +: -:
            AUTOMATIC_KW,        // automatic
            // SystemVerilog IEEE 1800 — Phase 1 keywords
            ALWAYS_FF_KW,        // always_ff
            ALWAYS_COMB_KW,      // always_comb
            ALWAYS_LATCH_KW,     // always_latch
            UNIQUE_KW,           // unique (case/if synthesis directive)
            PRIORITY_KW,         // priority (case/if synthesis directive)
            // SystemVerilog IEEE 1800 — Phase 2 keywords
            TYPEDEF_KW,          // typedef
            ENUM_KW,             // enum
            STRUCT_KW,           // struct
            PACKED_KW,           // packed
            PACKAGE_KW,          // package
            ENDPACKAGE_KW,       // endpackage
            IMPORT_KW,           // import
            // SystemVerilog IEEE 1800 — Phase 3 keywords
            INTERFACE_KW,        // interface
            ENDINTERFACE_KW,     // endinterface
            MODPORT_KW,          // modport
            // SystemVerilog IEEE 1800 — Phase 4 operators
            PLUS_ASSIGN,         // +=
            MINUS_ASSIGN,        // -=
            STAR_ASSIGN,         // *=
            AMP_ASSIGN,          // &=
            PIPE_ASSIGN,         // |=
            CARET_ASSIGN,        // ^=
            INC_OP,              // ++
            DEC_OP,              // --
            INSIDE_KW,           // inside
            // SystemVerilog IEEE 1800 — Phase 5 tokens
            WILDCARD_EQ,         // ==?
            WILDCARD_NEQ,        // !=?
            DO_KW,               // do
            BREAK_KW,            // break
            CONTINUE_KW,         // continue
            FOREACH_KW,          // foreach
            RETURN_KW,           // return
            VOID_KW,             // void
            // SystemVerilog IEEE 1800 — Phase 6 tokens
            UNION_KW,            // union
            ASSERT_KW,           // assert
            ASSUME_KW,           // assume
            COVER_KW,            // cover
            PROPERTY_KW,         // property
            SEQUENCE_KW,         // sequence
            CLOCKING_KW,         // clocking
            ENDCLOCKING_KW,      // endclocking
            FINAL_KW,            // final
            ENDPROPERTY_KW,      // endproperty
            ENDSEQUENCE_KW,      // endsequence
            // SystemVerilog IEEE 1800 — Phase 7 tokens
            CONST_KW,            // const
            STATIC_KW,           // static
            EXTERN_KW,           // extern
            PROGRAM_KW,          // program
            ENDPROGRAM_KW,       // endprogram
            CLASS_KW,            // class
            ENDCLASS_KW,         // endclass
            BIND_KW,             // bind
            VIRTUAL_KW,          // virtual
            PURE_KW,             // pure
            PROTECTED_KW,        // protected
            LOCAL_KW,            // local (in class context)
            TIMEUNIT_KW,         // timeunit
            TIMEPRECISION_KW,    // timeprecision
            COVERGROUP_KW,       // covergroup
            ENDGROUP_KW,         // endgroup
            CONSTRAINT_KW,       // constraint
            RAND_KW,             // rand/randc
            // SystemVerilog IEEE 1800 — Phase 8 tokens
            FORK_KW,             // fork
            JOIN_KW,             // join
            JOIN_ANY_KW,         // join_any
            JOIN_NONE_KW,        // join_none
            CHECKER_KW,          // checker
            ENDCHECKER_KW,       // endchecker
            CONFIG_KW,           // config
            ENDCONFIG_KW,        // endconfig
            LET_KW,              // let
            END
        };
        Type type;
        std::string value;
        int line = 0;
    };

    std::vector<Token> tokenize(const std::string& src);
    VerilogParseResult parse_tokens(const std::vector<Token>& tokens, Netlist& nl);

    // Parse helpers
    size_t parse_module(const std::vector<Token>& t, size_t pos, Netlist& nl, VerilogParseResult& r);
    size_t parse_port_decl(const std::vector<Token>& t, size_t pos, Netlist& nl,
                           VerilogParseResult& r, const std::string& dir);
    size_t parse_gate_inst(const std::vector<Token>& t, size_t pos, Netlist& nl,
                           VerilogParseResult& r, GateType gtype);
    size_t parse_assign(const std::vector<Token>& t, size_t pos, Netlist& nl, VerilogParseResult& r);
    size_t parse_module_inst(const std::vector<Token>& t, size_t pos, Netlist& nl,
                             VerilogParseResult& r, const std::string& mod_type);
    size_t parse_parameter(const std::vector<Token>& t, size_t pos);

    // Behavioral block parsers
    size_t parse_always(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent);
    size_t parse_always_sv(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent);
    size_t parse_statement_block(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> block);
    size_t parse_statement(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent);
    size_t parse_case(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent);
    size_t parse_for_loop(const std::vector<Token>& t, size_t pos, std::shared_ptr<AstNode> parent);

    // Phase 16 — generate, function, task
    size_t parse_generate_for(const std::vector<Token>& t, size_t pos, Netlist& nl, VerilogParseResult& r);
    size_t parse_generate_if(const std::vector<Token>& t, size_t pos, Netlist& nl, VerilogParseResult& r);
    size_t parse_function_def(const std::vector<Token>& t, size_t pos);
    size_t parse_task_def(const std::vector<Token>& t, size_t pos);
    size_t parse_generate_case(const std::vector<Token>& t, size_t pos, Netlist& nl, VerilogParseResult& r);
    size_t parse_specify_block(const std::vector<Token>& t, size_t pos, VerilogParseResult& r);
    std::string preprocess(const std::string& src);
    int eval_const_expr(const std::vector<Token>& t, size_t& pos);

    // Precedence-based expression parser (recursive descent)
    std::shared_ptr<AstNode> parse_expression(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_ternary(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_logor(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_logand(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_bitor(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_bitxor(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_bitand(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_equality(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_relational(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_shift(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_additive(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_multiplicative(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_unary(const std::vector<Token>& t, size_t& pos);
    std::shared_ptr<AstNode> parse_primary(const std::vector<Token>& t, size_t& pos);

    // Bus helpers
    std::pair<int,int> parse_bus_range(const std::vector<Token>& t, size_t& pos);
    std::vector<NetId> get_or_create_bus(Netlist& nl, const std::string& name, int msb, int lsb);
    NetId get_or_create_net(Netlist& nl, const std::string& name);
    int bus_width(const std::string& name) const;

    std::unordered_map<std::string, NetId> net_map_;
    std::unordered_map<std::string, std::pair<int,int>> bus_ranges_;
    std::unordered_map<std::string, int> params_;  // parameter name → integer value
    std::vector<std::string> known_modules_;
    StructuralSynthesizer struct_synth_;

    // Module library for hierarchical elaboration
    struct ModuleDef {
        std::string name;
        std::vector<Token> tokens;  // tokens from 'module' to 'endmodule'
        std::vector<std::string> port_names;
        std::unordered_map<std::string, std::string> port_dirs; // port → "input"/"output"/"inout"
        std::unordered_map<std::string, std::pair<int,int>> port_ranges; // port → {msb,lsb}
    };
    std::unordered_map<std::string, ModuleDef> module_defs_;
    int hierarchy_depth_ = 0;
    static constexpr int MAX_HIERARCHY_DEPTH = 16;

    // Hierarchical elaboration: expand instance into parent netlist
    void elaborate_instance(const std::string& mod_type, const std::string& inst_name,
                            const std::vector<std::pair<std::string,std::string>>& connections,
                            const std::unordered_map<std::string,int>& param_overrides,
                            Netlist& nl, VerilogParseResult& r);

    // Phase 16 — preprocessor defines, function storage, signed tracking
    std::unordered_map<std::string, std::string> defines_;
    struct FuncDef {
        std::pair<int,int> return_range = {-1,-1};
        std::vector<std::string> param_names;
        std::vector<Token> body_tokens;
    };
    std::unordered_map<std::string, FuncDef> functions_;
    std::set<std::string> signed_signals_;
    // Memory arrays: name → {word_width, depth}
    std::unordered_map<std::string, std::pair<int,int>> memory_arrays_;
    // Task storage: name → {param_names, body_tokens} (reuse FuncDef, no return range)
    std::unordered_map<std::string, FuncDef> tasks_;
    // Multi-dimensional arrays: name → {word_width, dim1_depth, dim2_depth}
    std::unordered_map<std::string, std::tuple<int,int,int>> multidim_arrays_;

    // SystemVerilog Phase 2 — type system
    struct SvTypeDef {
        int width = 0;                      // total bit width
        bool is_enum = false;
        bool is_struct = false;
        bool is_union = false;
        std::vector<std::string> enum_names; // enum constant names (in order)
        std::vector<std::pair<std::string,int>> struct_fields; // {name, width}
    };
    std::unordered_map<std::string, SvTypeDef> sv_typedefs_; // type_name → definition

    struct SvPackage {
        std::string name;
        std::unordered_map<std::string, SvTypeDef> typedefs;
        std::unordered_map<std::string, int> params; // constants
    };
    std::unordered_map<std::string, SvPackage> sv_packages_;

    size_t parse_typedef(const std::vector<Token>& t, size_t pos);
    size_t parse_package(const std::vector<Token>& t, size_t pos);
    size_t parse_import(const std::vector<Token>& t, size_t pos);
    bool resolve_sv_type(const std::string& name, int& width) const;

    // SystemVerilog Phase 3 — interfaces
    struct SvInterfaceSignal {
        std::string name;
        int width = 1;
    };
    struct SvModport {
        std::string name;
        std::vector<std::pair<std::string, std::string>> signals; // {dir, signal_name}
    };
    struct SvInterface {
        std::string name;
        std::vector<SvInterfaceSignal> signals;
        std::unordered_map<std::string, SvModport> modports;
    };
    std::unordered_map<std::string, SvInterface> sv_interfaces_;

    size_t parse_interface(const std::vector<Token>& t, size_t pos);
    void expand_interface_port(const std::string& iface_name, const std::string& modport_name,
                               const std::string& port_name, Netlist& nl, VerilogParseResult& r);
};

} // namespace sf
