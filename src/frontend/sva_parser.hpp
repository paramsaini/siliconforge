#pragma once
// SiliconForge — SystemVerilog Assertions (SVA) Parser
// Parses linear temporal logic (LTL) properties.
// Supported subset: assert property (@(posedge clk) a |-> b); and |=>

#include <string>
#include <vector>
#include <memory>
#include <stdexcept>

namespace sf {

enum class SvaOp {
    PROP_OVERLAPPING,    // |->
    PROP_NON_OVERLAPPING,// |=>
    LITERAL,
    // Tier 2: Temporal operators
    NEXT,                // ##1  / nexttime
    DELAY,               // ##N  (cycle delay)
    GLOBALLY,            // always / G
    EVENTUALLY,          // eventually / F
    UNTIL,               // until / U
    AND,                 // &&
    OR,                  // ||
    NOT,                 // !
    REPEAT,              // [*N] repetition
    SEQUENCE_CONCAT      // sequence concatenation
};

struct SvaNode {
    SvaOp op;
    std::string literal; // For LITERAL
    int delay_cycles = 1; // For DELAY/NEXT/REPEAT
    std::shared_ptr<SvaNode> left;
    std::shared_ptr<SvaNode> right;
};

struct SvaSequence {
    std::string name;
    std::string expression;     // the sequence body expression
    std::vector<std::string> arguments; // formal arguments
};

struct SvaProperty {
    std::string name;
    std::string clock_domain;
    std::shared_ptr<SvaNode> expr;
    bool is_assert = true;  // false for cover
    bool is_assume = false; // assume property (environmental constraint)
    bool is_cover  = false; // cover property (liveness check)
};

class SvaParser {
public:
    SvaParser() = default;

    // Parses a block of SVA text, extracting properties and sequences
    std::vector<SvaProperty> parse(const std::string& source);

    // Access parsed sequences
    const std::vector<SvaSequence>& sequences() const { return sequences_; }

private:
    std::vector<std::string> tokenize(const std::string& src) const;
    std::shared_ptr<SvaNode> parse_expr(const std::vector<std::string>& tokens, size_t& pos);
    std::shared_ptr<SvaNode> parse_binary(const std::vector<std::string>& tokens, size_t& pos,
                                           std::shared_ptr<SvaNode> left);
    void parse_sequence_decl(const std::vector<std::string>& tokens, size_t& pos);

    std::vector<SvaSequence> sequences_;
};

} // namespace sf
