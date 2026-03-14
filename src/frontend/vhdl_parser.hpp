#pragma once
// SiliconForge — VHDL 2008 Parser (Entity/Architecture/Process)
// Supports: entity, architecture, signal assignments, process blocks,
//           concurrent statements, component instantiation, VHDL writer.

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

struct VhdlParseResult {
    bool success = false;
    std::string entity_name;
    int num_inputs = 0;
    int num_outputs = 0;
    int num_signals = 0;
    int num_gates = 0;
    int num_processes = 0;
    int num_generics = 0;
    std::string error;
};

struct VhdlPort {
    std::string name;
    enum Direction { IN, OUT, INOUT, BUFFER } direction;
    int width = 1;
    std::string type_name;
};

struct VhdlSignal {
    std::string name;
    int width = 1;
    std::string type_name;
};

struct VhdlGeneric {
    std::string name;
    std::string type;          // "integer", "natural", "std_logic_vector", etc.
    std::string default_value;
};

class VhdlParser {
public:
    VhdlParseResult parse_string(const std::string& src, Netlist& nl);
    VhdlParseResult parse_file(const std::string& filename, Netlist& nl);
    static std::string to_vhdl(const Netlist& nl,
                               const std::string& entity_name = "top");

    const std::vector<VhdlPort>& ports() const { return ports_; }
    const std::vector<VhdlSignal>& signals() const { return signals_; }
    const std::vector<VhdlGeneric>& generics() const { return generics_; }

private:
    struct Token {
        enum Type {
            IDENT, NUMBER, LPAREN, RPAREN, SEMI, COLON, COMMA,
            ASSIGN_SIG, ASSIGN_VAR, LT, GT, AMP, BAR, DOT, TICK,
            LBRACKET, RBRACKET, STRING_LIT, ARROW, EQ, HASH,
            EOF_TOK
        };
        Type type;
        std::string text;
        int line;
    };

    std::vector<Token> tokenize(const std::string& src);

    // Top-level parsing
    void parse_entity(const std::vector<Token>& t, size_t& pos,
                      Netlist& nl, VhdlParseResult& r);
    void parse_architecture(const std::vector<Token>& t, size_t& pos,
                            Netlist& nl, VhdlParseResult& r);

    // Statement parsing
    void parse_signal_decl(const std::vector<Token>& t, size_t& pos,
                           Netlist& nl, VhdlParseResult& r);
    void parse_signal_assignment(const std::vector<Token>& t, size_t& pos,
                                 Netlist& nl, VhdlParseResult& r);
    void parse_process(const std::vector<Token>& t, size_t& pos,
                       Netlist& nl, VhdlParseResult& r);
    void parse_component_inst(const std::vector<Token>& t, size_t& pos,
                              Netlist& nl, VhdlParseResult& r,
                              const std::string& label);

    // Helpers
    bool is_keyword(const Token& tok, const std::string& kw) const;
    void skip_until_semi(const std::vector<Token>& t, size_t& pos);

    // State
    std::vector<VhdlPort> ports_;
    std::vector<VhdlSignal> signals_;
    std::vector<VhdlGeneric> generics_;
    Netlist* nl_ = nullptr;
    std::unordered_map<std::string, NetId> name_map_;
    int gate_counter_ = 0;
    int dff_counter_ = 0;

    NetId lookup_or_create(const std::string& name, Netlist& nl);
    std::string to_lower(const std::string& s) const;
};

} // namespace sf
