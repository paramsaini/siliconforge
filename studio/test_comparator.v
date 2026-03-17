// test_comparator.v — Signed/unsigned magnitude comparator + equality checker
// Expected: ~120 gates (32-bit comparator chain)
// Tests: signed arithmetic, comparison operators, conditional assignments

module comparator_32bit (
    input  signed [31:0] a,
    input  signed [31:0] b,
    input                is_signed,
    output               eq,
    output               lt,
    output               gt,
    output               le,
    output               ge,
    output               ne,
    output        [31:0] min_val,
    output        [31:0] max_val,
    output        [31:0] diff,
    output        [31:0] abs_diff
);
    wire signed_lt = (a < b);
    wire unsigned_lt = ($unsigned(a) < $unsigned(b));
    wire cmp_lt = is_signed ? signed_lt : unsigned_lt;

    assign eq  = (a == b);
    assign ne  = (a != b);
    assign lt  = cmp_lt;
    assign gt  = ~cmp_lt & ~eq;
    assign le  = cmp_lt | eq;
    assign ge  = ~cmp_lt;

    assign min_val  = cmp_lt ? a : b;
    assign max_val  = cmp_lt ? b : a;
    assign diff     = a - b;
    assign abs_diff = (a >= b) ? (a - b) : (b - a);
endmodule
