#pragma once
// SiliconForge — Name Pool (String Interning)
// Stores all design names (gate, net, cell, pin) in a contiguous arena.
// Each name gets a 32-bit NameId handle. Lookup by ID is O(1), lookup by
// string is O(1) amortized (hash map). Duplicate strings share storage.
//
// Memory savings at 1M gates:
//   Before: std::string per gate + per net = ~64 MB strings
//   After:  ~8 MB arena + 4 bytes/handle = ~12 MB total (5x reduction)
//
// Reference: LLVM StringPool, ABC string manager, OpenTimer name_pool

#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <cassert>

namespace sf {

using NameId = int32_t;
constexpr NameId NAME_NONE = -1;

class NamePool {
public:
    NamePool() {
        // Reserve slot 0 for empty string
        offsets_.push_back(0);
        arena_.push_back('\0');
    }

    // Intern a string — returns existing ID if already stored, else allocates.
    // Thread-safe: NO (single-threaded design phase, then read-only)
    NameId intern(const std::string& name) {
        if (name.empty()) return 0;

        auto it = lookup_.find(name);
        if (it != lookup_.end()) return it->second;

        NameId id = static_cast<NameId>(offsets_.size());
        size_t offset = arena_.size();
        offsets_.push_back(static_cast<uint32_t>(offset));

        // Append name + null terminator to arena
        arena_.insert(arena_.end(), name.begin(), name.end());
        arena_.push_back('\0');

        lookup_[name] = id;
        return id;
    }

    // Intern from C-string (resolves overload ambiguity for string literals)
    NameId intern(const char* name) {
        return intern(std::string(name ? name : ""));
    }

    // Retrieve name by ID — O(1)
    const char* c_str(NameId id) const {
        if (id < 0 || id >= static_cast<NameId>(offsets_.size())) return "";
        return arena_.data() + offsets_[id];
    }

    // Retrieve as string_view — O(1), no allocation
    std::string_view view(NameId id) const {
        return c_str(id);
    }

    // Retrieve as std::string — allocates, use sparingly
    std::string str(NameId id) const {
        return std::string(c_str(id));
    }

    // Lookup by string — returns NAME_NONE if not interned
    NameId find(const std::string& name) const {
        auto it = lookup_.find(name);
        return (it != lookup_.end()) ? it->second : NAME_NONE;
    }

    // Number of unique strings stored
    size_t size() const { return offsets_.size(); }

    // Total arena memory (bytes)
    size_t arena_bytes() const { return arena_.size(); }

    // Clear all stored names (invalidates all NameIds)
    void clear() {
        arena_.clear();
        offsets_.clear();
        lookup_.clear();
        // Re-add empty string at slot 0
        offsets_.push_back(0);
        arena_.push_back('\0');
    }

    // Reserve capacity for expected number of names
    void reserve(size_t expected_names, size_t avg_name_len = 16) {
        offsets_.reserve(expected_names);
        arena_.reserve(expected_names * (avg_name_len + 1));
        lookup_.reserve(expected_names);
    }

private:
    std::vector<char> arena_;           // contiguous name storage
    std::vector<uint32_t> offsets_;     // offset[id] → position in arena_
    std::unordered_map<std::string, NameId> lookup_;  // dedup map
};

// Global name pool singleton (optional — designs can use their own)
NamePool& global_name_pool();

} // namespace sf
