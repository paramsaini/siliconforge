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
    LITERAL
};

struct SvaNode {
    SvaOp op;
    std::string literal; // For LITERAL
    std::shared_ptr<SvaNode> left;
    std::shared_ptr<SvaNode> right;
};

struct SvaProperty {
    std::string name;
    std::string clock_domain;
    std::shared_ptr<SvaNode> expr;
    bool is_assert = true; // false for cover
};

class SvaParser {
public:
    SvaParser() = default;

    // Parses a block of SVA text, extracting properties
    std::vector<SvaProperty> parse(const std::string& source);

private:
    std::vector<std::string> tokenize(const std::string& src) const;
    std::shared_ptr<SvaNode> parse_expr(const std::vector<std::string>& tokens, size_t& pos);
};

} // namespace sf
