#pragma once
// SiliconForge — Core Types
// IEEE 1364/1800 compliant 4-state logic system

#include <cstdint>
#include <string>
#include <vector>
#include <cassert>

namespace sf {

// ============================================================================
// 4-State Logic (IEEE 1364-2005, IEEE 1800-2023)
// Values: 0 (logic low), 1 (logic high), X (unknown), Z (high-impedance)
// ============================================================================
enum class Logic4 : uint8_t {
    ZERO = 0,
    ONE  = 1,
    X    = 2,
    Z    = 3
};

// --- Truth Tables (IEEE 1364-2005 Table 5-1 through 5-5) ---

inline Logic4 logic_not(Logic4 a) {
    constexpr Logic4 table[] = {Logic4::ONE, Logic4::ZERO, Logic4::X, Logic4::X};
    return table[static_cast<uint8_t>(a)];
}

inline Logic4 logic_and(Logic4 a, Logic4 b) {
    // 0 dominates, then X, then Z; 1&1=1
    if (a == Logic4::ZERO || b == Logic4::ZERO) return Logic4::ZERO;
    if (a == Logic4::ONE  && b == Logic4::ONE)  return Logic4::ONE;
    return Logic4::X;
}

inline Logic4 logic_or(Logic4 a, Logic4 b) {
    // 1 dominates, then X, then Z; 0|0=0
    if (a == Logic4::ONE  || b == Logic4::ONE)  return Logic4::ONE;
    if (a == Logic4::ZERO && b == Logic4::ZERO) return Logic4::ZERO;
    return Logic4::X;
}

inline Logic4 logic_xor(Logic4 a, Logic4 b) {
    if (a == Logic4::X || a == Logic4::Z || b == Logic4::X || b == Logic4::Z)
        return Logic4::X;
    return (a == b) ? Logic4::ZERO : Logic4::ONE;
}

inline char logic_to_char(Logic4 v) {
    constexpr char table[] = {'0', '1', 'x', 'z'};
    return table[static_cast<uint8_t>(v)];
}

inline Logic4 char_to_logic(char c) {
    switch (c) {
        case '0': return Logic4::ZERO;
        case '1': return Logic4::ONE;
        case 'x': case 'X': return Logic4::X;
        case 'z': case 'Z': return Logic4::Z;
        default: return Logic4::X;
    }
}

// ============================================================================
// BitVector — n-bit signal with 4-state values
// LSB-first storage: bits_[0] is the least significant bit
// ============================================================================
class BitVector {
public:
    BitVector() = default;
    explicit BitVector(size_t width, Logic4 init = Logic4::X)
        : bits_(width, init) {}

    static BitVector from_uint(size_t width, uint64_t value) {
        BitVector bv(width, Logic4::ZERO);
        for (size_t i = 0; i < width && i < 64; ++i)
            bv.bits_[i] = (value & (1ULL << i)) ? Logic4::ONE : Logic4::ZERO;
        return bv;
    }

    size_t width() const { return bits_.size(); }

    Logic4 operator[](size_t i) const { assert(i < bits_.size()); return bits_[i]; }
    Logic4& operator[](size_t i) { assert(i < bits_.size()); return bits_[i]; }

    bool is_known() const {
        for (auto b : bits_)
            if (b != Logic4::ZERO && b != Logic4::ONE) return false;
        return true;
    }

    uint64_t to_uint() const {
        assert(is_known());
        uint64_t val = 0;
        for (size_t i = 0; i < bits_.size() && i < 64; ++i)
            if (bits_[i] == Logic4::ONE) val |= (1ULL << i);
        return val;
    }

    std::string to_string() const {
        std::string s;
        for (int i = static_cast<int>(bits_.size()) - 1; i >= 0; --i)
            s += logic_to_char(bits_[i]);
        return s;
    }

private:
    std::vector<Logic4> bits_;
};

} // namespace sf
