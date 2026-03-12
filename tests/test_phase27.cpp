// Phase 27: TCL Shell Industrial Tests
// Tests arrays, namespaces, regex, list ops, catch/error, break/continue, etc.
#include "shell/tcl_interp.hpp"
#include <iostream>
#include <string>
#include <cassert>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// ════════════════════════════════════════════════
// ARRAY TESTS
// ════════════════════════════════════════════════
TEST(array_set_get) {
    TclInterp tcl;
    tcl.eval("set colors(red) FF0000");
    tcl.eval("set colors(green) 00FF00");
    tcl.eval("set colors(blue) 0000FF");
    CHECK(tcl.get_array("colors", "red") == "FF0000", "array get red");
    CHECK(tcl.get_array("colors", "blue") == "0000FF", "array get blue");
    CHECK(tcl.array_size("colors") == 3, "array size 3");
    PASS("array_set_get");
}

TEST(array_command) {
    TclInterp tcl;
    tcl.eval("array set cfg {width 100 height 200 depth 5}");
    std::string size = tcl.eval("array size cfg");
    CHECK(size == "3", "array size = 3, got: " + size);
    std::string exists = tcl.eval("array exists cfg");
    CHECK(exists == "1", "array exists = 1");
    std::string not_exists = tcl.eval("array exists nonexist");
    CHECK(not_exists == "0", "array not exists = 0");
    PASS("array_command");
}

TEST(array_substitution) {
    TclInterp tcl;
    tcl.eval("set data(x) 42");
    std::string r = tcl.eval("set result $data(x)");
    CHECK(r == "42", "array substitution: $data(x) = 42, got: " + r);
    PASS("array_substitution");
}

// ════════════════════════════════════════════════
// LIST OPERATION TESTS
// ════════════════════════════════════════════════
TEST(lappend_basic) {
    TclInterp tcl;
    tcl.eval("set mylist {}");
    tcl.eval("lappend mylist apple");
    tcl.eval("lappend mylist banana cherry");
    std::string r = tcl.eval("llength $mylist");
    CHECK(r == "3", "lappend 3 items, got length: " + r);
    PASS("lappend_basic");
}

TEST(lsort_basic) {
    TclInterp tcl;
    std::string r = tcl.eval("lsort {cherry apple banana}");
    CHECK(r == "apple banana cherry", "lsort alphabetical, got: " + r);
    PASS("lsort_basic");
}

TEST(lsort_integer) {
    TclInterp tcl;
    std::string r = tcl.eval("lsort -integer {3 1 4 1 5 9 2 6}");
    CHECK(r == "1 1 2 3 4 5 6 9", "lsort -integer, got: " + r);
    PASS("lsort_integer");
}

TEST(lsort_decreasing) {
    TclInterp tcl;
    std::string r = tcl.eval("lsort -decreasing {apple banana cherry}");
    CHECK(r == "cherry banana apple", "lsort -decreasing, got: " + r);
    PASS("lsort_decreasing");
}

TEST(lsearch_basic) {
    TclInterp tcl;
    std::string r = tcl.eval("lsearch {alpha beta gamma} beta");
    CHECK(r == "1", "lsearch found at 1, got: " + r);
    std::string r2 = tcl.eval("lsearch {alpha beta gamma} delta");
    CHECK(r2 == "-1", "lsearch not found = -1, got: " + r2);
    PASS("lsearch_basic");
}

TEST(lrange_basic) {
    TclInterp tcl;
    std::string r = tcl.eval("lrange {a b c d e} 1 3");
    CHECK(r == "b c d", "lrange 1 3, got: " + r);
    PASS("lrange_basic");
}

TEST(lrange_end) {
    TclInterp tcl;
    std::string r = tcl.eval("lrange {a b c d e} 2 end");
    CHECK(r == "c d e", "lrange 2 end, got: " + r);
    PASS("lrange_end");
}

// ════════════════════════════════════════════════
// STRING TESTS
// ════════════════════════════════════════════════
TEST(string_index) {
    TclInterp tcl;
    std::string r = tcl.eval("string index hello 1");
    CHECK(r == "e", "string index hello 1 = e, got: " + r);
    PASS("string_index");
}

TEST(string_range) {
    TclInterp tcl;
    std::string r = tcl.eval("string range hello 1 3");
    CHECK(r == "ell", "string range hello 1 3 = ell, got: " + r);
    PASS("string_range");
}

TEST(string_first) {
    TclInterp tcl;
    std::string r = tcl.eval("string first ll hello");
    CHECK(r == "2", "string first ll hello = 2, got: " + r);
    PASS("string_first");
}

TEST(string_repeat) {
    TclInterp tcl;
    std::string r = tcl.eval("string repeat ab 3");
    CHECK(r == "ababab", "string repeat ab 3 = ababab, got: " + r);
    PASS("string_repeat");
}

TEST(string_is_integer) {
    TclInterp tcl;
    CHECK(tcl.eval("string is integer 42") == "1", "42 is integer");
    CHECK(tcl.eval("string is integer abc") == "0", "abc is not integer");
    PASS("string_is_integer");
}

// ════════════════════════════════════════════════
// REGEX TESTS
// ════════════════════════════════════════════════
TEST(regexp_basic) {
    TclInterp tcl;
    std::string r = tcl.eval("regexp {[0-9]+} abc123def");
    CHECK(r == "1", "regexp finds digits");
    PASS("regexp_basic");
}

TEST(regexp_capture) {
    TclInterp tcl;
    tcl.eval("regexp {([a-z]+)([0-9]+)} abc123 full word num");
    CHECK(tcl.get_var("full") == "abc123", "full match");
    CHECK(tcl.get_var("word") == "abc", "word capture");
    CHECK(tcl.get_var("num") == "123", "num capture");
    PASS("regexp_capture");
}

TEST(regsub_basic) {
    TclInterp tcl;
    std::string r = tcl.eval("regsub {world} {hello world} universe");
    CHECK(r == "hello universe", "regsub replace, got: " + r);
    PASS("regsub_basic");
}

// ════════════════════════════════════════════════
// CONTROL FLOW TESTS
// ════════════════════════════════════════════════
TEST(catch_success) {
    TclInterp tcl;
    std::string r = tcl.eval("catch {expr 2 + 3} result");
    CHECK(r == "0", "catch returns 0 on success");
    CHECK(tcl.get_var("result") == "5", "result captured");
    PASS("catch_success");
}

TEST(catch_error) {
    TclInterp tcl;
    std::string r = tcl.eval("catch {error {bad input}} errmsg");
    CHECK(r == "1", "catch returns 1 on error");
    CHECK(tcl.get_var("errmsg") == "bad input", "error message captured");
    PASS("catch_error");
}

TEST(switch_basic) {
    TclInterp tcl;
    tcl.eval("set x 2");
    std::string r = tcl.eval("switch $x 1 {set r one} 2 {set r two} 3 {set r three}");
    CHECK(tcl.get_var("r") == "two", "switch matched 2, got: " + tcl.get_var("r"));
    PASS("switch_basic");
}

TEST(break_in_for) {
    TclInterp tcl;
    tcl.eval("set sum 0");
    tcl.eval("for {set i 0} {$i < 10} {incr i} { if {$i == 5} { break }; incr sum }");
    std::string r = tcl.get_var("sum");
    CHECK(r == "5", "break at i=5, sum=5, got: " + r);
    PASS("break_in_for");
}

TEST(continue_in_foreach) {
    TclInterp tcl;
    tcl.eval("set result {}");
    tcl.eval("foreach x {1 2 3 4 5} { if {$x == 3} { continue }; lappend result $x }");
    std::string r = tcl.get_var("result");
    CHECK(r == "1 2 4 5", "continue skips 3, got: " + r);
    PASS("continue_in_foreach");
}

// ════════════════════════════════════════════════
// NAMESPACE TESTS
// ════════════════════════════════════════════════
TEST(namespace_eval) {
    TclInterp tcl;
    tcl.eval("namespace eval ::myns { set x 42 }");
    CHECK(tcl.current_namespace() == "::", "back to global ns");
    PASS("namespace_eval");
}

// ════════════════════════════════════════════════
// JOIN / SPLIT / FORMAT / CONCAT
// ════════════════════════════════════════════════
TEST(join_basic) {
    TclInterp tcl;
    std::string r = tcl.eval("join {a b c} ,");
    CHECK(r == "a,b,c", "join with comma, got: " + r);
    PASS("join_basic");
}

TEST(split_basic) {
    TclInterp tcl;
    std::string r = tcl.eval("split {a,b,c} ,");
    CHECK(r == "a b c", "split on comma, got: " + r);
    PASS("split_basic");
}

TEST(format_basic) {
    TclInterp tcl;
    std::string r = tcl.eval("format {name=%s val=%d} hello 42");
    CHECK(r == "name=hello val=42", "format, got: " + r);
    PASS("format_basic");
}

TEST(concat_basic) {
    TclInterp tcl;
    std::string r = tcl.eval("concat {a b} {c d}");
    CHECK(r == "a b c d", "concat, got: " + r);
    PASS("concat_basic");
}

TEST(append_basic) {
    TclInterp tcl;
    tcl.eval("set s hello");
    tcl.eval("append s { world}");
    std::string r = tcl.get_var("s");
    CHECK(r == "hello world", "append, got: " + r);
    PASS("append_basic");
}

TEST(lmap_basic) {
    TclInterp tcl;
    std::string r = tcl.eval("lmap x {1 2 3} {expr $x * 2}");
    CHECK(r == "2 4 6", "lmap doubling, got: " + r);
    PASS("lmap_basic");
}

TEST(unset_basic) {
    TclInterp tcl;
    tcl.eval("set x 42");
    CHECK(tcl.eval("info exists x") == "1", "x exists");
    tcl.eval("unset x");
    CHECK(tcl.eval("info exists x") == "0", "x unset");
    PASS("unset_basic");
}

// ════════════════════════════════════════════════
// BACKWARD COMPATIBILITY
// ════════════════════════════════════════════════
TEST(backward_compat_basics) {
    TclInterp tcl;
    CHECK(tcl.eval("expr 3 + 4") == "7", "expr works");
    CHECK(tcl.eval("string length hello") == "5", "string length");
    CHECK(tcl.eval("llength {a b c}") == "3", "llength");
    CHECK(tcl.eval("lindex {a b c} 1") == "b", "lindex");
    PASS("backward_compat_basics");
}

int main() {
    std::cout << "═══════════════════════════════════════\n";
    std::cout << " Phase 27: TCL Shell Industrial Tests\n";
    std::cout << "═══════════════════════════════════════\n\n";

    std::cout << "── Arrays ──\n";
    RUN(array_set_get);
    RUN(array_command);
    RUN(array_substitution);

    std::cout << "\n── List Operations ──\n";
    RUN(lappend_basic);
    RUN(lsort_basic);
    RUN(lsort_integer);
    RUN(lsort_decreasing);
    RUN(lsearch_basic);
    RUN(lrange_basic);
    RUN(lrange_end);

    std::cout << "\n── String Operations ──\n";
    RUN(string_index);
    RUN(string_range);
    RUN(string_first);
    RUN(string_repeat);
    RUN(string_is_integer);

    std::cout << "\n── Regex ──\n";
    RUN(regexp_basic);
    RUN(regexp_capture);
    RUN(regsub_basic);

    std::cout << "\n── Control Flow ──\n";
    RUN(catch_success);
    RUN(catch_error);
    RUN(switch_basic);
    RUN(break_in_for);
    RUN(continue_in_foreach);

    std::cout << "\n── Namespace ──\n";
    RUN(namespace_eval);

    std::cout << "\n── Join/Split/Format/Concat ──\n";
    RUN(join_basic);
    RUN(split_basic);
    RUN(format_basic);
    RUN(concat_basic);
    RUN(append_basic);
    RUN(lmap_basic);
    RUN(unset_basic);

    std::cout << "\n── Backward Compatibility ──\n";
    RUN(backward_compat_basics);

    std::cout << "\n════════════════════════════════\n";
    std::cout << "Results: " << passed << " passed, " << failed << " failed\n";
    return failed > 0 ? 1 : 0;
}
