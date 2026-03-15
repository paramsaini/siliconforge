#pragma once
// SiliconForge — Compiled Liberty Binary Cache
// Serializes a parsed LibertyLibrary to a compact binary format and
// deserializes it ~10-50x faster than re-parsing the ASCII .lib file.
//
// Format: header + cell records + NLDM table data + string pool
// Uses fixed-width records where possible for direct memory mapping.
//
// Workflow:
//   1. First run: parse .lib → save_compiled("sky130.slib")
//   2. Subsequent runs: load_compiled("sky130.slib") → skip parsing
//   3. Cache invalidated when .lib mtime changes or format version bumps
//
// Reference: OpenSTA compiled Liberty, Synopsys .db format concept

#include "core/liberty_parser.hpp"
#include <string>
#include <cstdint>

namespace sf {

// Magic number and version for binary cache format
constexpr uint32_t SLIB_MAGIC   = 0x534C4942; // "SLIB"
constexpr uint32_t SLIB_VERSION = 1;

struct SlibHeader {
    uint32_t magic = SLIB_MAGIC;
    uint32_t version = SLIB_VERSION;
    uint64_t source_mtime = 0;       // modification time of source .lib
    uint32_t num_cells = 0;
    uint32_t num_strings = 0;        // string pool entries
    uint64_t string_pool_offset = 0; // byte offset to string pool
    double nom_voltage = 0;
    double nom_temperature = 0;
    char lib_name[64] = {};
    char technology[32] = {};
    char time_unit[16] = {};
    char cap_unit[16] = {};
    uint32_t reserved[8] = {};
};

class LibertyCacheWriter {
public:
    // Serialize a parsed library to binary file
    // Returns true on success
    static bool save_compiled(const LibertyLibrary& lib,
                              const std::string& output_path,
                              uint64_t source_mtime = 0);
};

class LibertyCacheReader {
public:
    // Deserialize a compiled library from binary file
    // Returns true on success, populates lib
    static bool load_compiled(const std::string& input_path,
                              LibertyLibrary& lib);

    // Check if cache is valid (magic, version, mtime match)
    static bool is_cache_valid(const std::string& cache_path,
                               uint64_t source_mtime);

    // High-level: load from cache if valid, else parse .lib and create cache
    static bool load_or_parse(const std::string& lib_path,
                              const std::string& cache_path,
                              LibertyLibrary& lib);
};

} // namespace sf
