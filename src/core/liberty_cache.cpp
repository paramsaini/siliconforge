// SiliconForge — Compiled Liberty Binary Cache Implementation
// Serializes/deserializes Liberty libraries to compact binary format.
// ~10-50x faster than re-parsing ASCII .lib files.

#include "core/liberty_cache.hpp"
#include <fstream>
#include <cstring>
#include <sys/stat.h>

namespace sf {

// ── Helper: write/read POD to stream ─────────────────────────────────
template<typename T>
static void write_pod(std::ofstream& out, const T& val) {
    out.write(reinterpret_cast<const char*>(&val), sizeof(T));
}

template<typename T>
static bool read_pod(std::ifstream& in, T& val) {
    in.read(reinterpret_cast<char*>(&val), sizeof(T));
    return in.good();
}

static void write_string(std::ofstream& out, const std::string& s) {
    uint32_t len = static_cast<uint32_t>(s.size());
    write_pod(out, len);
    if (len > 0) out.write(s.data(), len);
}

static bool read_string(std::ifstream& in, std::string& s) {
    uint32_t len = 0;
    if (!read_pod(in, len)) return false;
    s.resize(len);
    if (len > 0) in.read(&s[0], len);
    return in.good();
}

static void write_doubles(std::ofstream& out, const std::vector<double>& v) {
    uint32_t n = static_cast<uint32_t>(v.size());
    write_pod(out, n);
    if (n > 0) out.write(reinterpret_cast<const char*>(v.data()), n * sizeof(double));
}

static bool read_doubles(std::ifstream& in, std::vector<double>& v) {
    uint32_t n = 0;
    if (!read_pod(in, n)) return false;
    v.resize(n);
    if (n > 0) in.read(reinterpret_cast<char*>(v.data()), n * sizeof(double));
    return in.good();
}

static void write_nldm(std::ofstream& out, const LibertyTiming::NldmTable& t) {
    write_doubles(out, t.index_1);
    write_doubles(out, t.index_2);
    uint32_t rows = static_cast<uint32_t>(t.values.size());
    write_pod(out, rows);
    for (auto& row : t.values) write_doubles(out, row);
}

static bool read_nldm(std::ifstream& in, LibertyTiming::NldmTable& t) {
    if (!read_doubles(in, t.index_1)) return false;
    if (!read_doubles(in, t.index_2)) return false;
    uint32_t rows = 0;
    if (!read_pod(in, rows)) return false;
    t.values.resize(rows);
    for (uint32_t i = 0; i < rows; i++)
        if (!read_doubles(in, t.values[i])) return false;
    return true;
}

// ── Writer ───────────────────────────────────────────────────────────
bool LibertyCacheWriter::save_compiled(const LibertyLibrary& lib,
                                        const std::string& output_path,
                                        uint64_t source_mtime) {
    std::ofstream out(output_path, std::ios::binary);
    if (!out.is_open()) return false;

    // Write header
    SlibHeader hdr;
    hdr.source_mtime = source_mtime;
    hdr.num_cells = static_cast<uint32_t>(lib.cells.size());
    hdr.nom_voltage = lib.nom_voltage;
    hdr.nom_temperature = lib.nom_temperature;
    std::strncpy(hdr.lib_name, lib.name.c_str(), sizeof(hdr.lib_name) - 1);
    std::strncpy(hdr.technology, lib.technology.c_str(), sizeof(hdr.technology) - 1);
    std::strncpy(hdr.time_unit, lib.time_unit.c_str(), sizeof(hdr.time_unit) - 1);
    std::strncpy(hdr.cap_unit, lib.cap_unit.c_str(), sizeof(hdr.cap_unit) - 1);
    write_pod(out, hdr);

    // Write cells
    for (auto& cell : lib.cells) {
        write_string(out, cell.name);
        write_pod(out, cell.area);
        write_pod(out, cell.leakage_power);

        // Pins
        uint32_t npins = static_cast<uint32_t>(cell.pins.size());
        write_pod(out, npins);
        for (auto& pin : cell.pins) {
            write_string(out, pin.name);
            write_string(out, pin.direction);
            write_string(out, pin.function);
            write_pod(out, pin.capacitance);
            write_pod(out, pin.max_transition);
        }

        // Timings
        uint32_t ntimings = static_cast<uint32_t>(cell.timings.size());
        write_pod(out, ntimings);
        for (auto& timing : cell.timings) {
            write_string(out, timing.related_pin);
            write_string(out, timing.timing_type);
            write_pod(out, timing.cell_rise);
            write_pod(out, timing.cell_fall);
            write_pod(out, timing.rise_transition);
            write_pod(out, timing.fall_transition);

            // NLDM tables
            write_nldm(out, timing.nldm_rise);
            write_nldm(out, timing.nldm_fall);
            write_nldm(out, timing.nldm_rise_tr);
            write_nldm(out, timing.nldm_fall_tr);
        }

        // Internal powers
        uint32_t npowers = static_cast<uint32_t>(cell.internal_powers.size());
        write_pod(out, npowers);
        for (auto& pwr : cell.internal_powers) {
            write_string(out, pwr.related_pin);
            write_string(out, pwr.when);
            write_pod(out, pwr.rise_power);
            write_pod(out, pwr.fall_power);
        }

        // Leakage powers
        uint32_t nleakage = static_cast<uint32_t>(cell.leakage_powers.size());
        write_pod(out, nleakage);
        for (auto& lk : cell.leakage_powers) {
            write_string(out, lk.when);
            write_pod(out, lk.value);
        }
    }

    return out.good();
}

// ── Reader ───────────────────────────────────────────────────────────
bool LibertyCacheReader::load_compiled(const std::string& input_path,
                                        LibertyLibrary& lib) {
    std::ifstream in(input_path, std::ios::binary);
    if (!in.is_open()) return false;

    SlibHeader hdr;
    if (!read_pod(in, hdr)) return false;
    if (hdr.magic != SLIB_MAGIC || hdr.version != SLIB_VERSION) return false;

    lib.name = hdr.lib_name;
    lib.technology = hdr.technology;
    lib.nom_voltage = hdr.nom_voltage;
    lib.nom_temperature = hdr.nom_temperature;
    lib.time_unit = hdr.time_unit;
    lib.cap_unit = hdr.cap_unit;

    lib.cells.resize(hdr.num_cells);
    for (uint32_t c = 0; c < hdr.num_cells; c++) {
        auto& cell = lib.cells[c];
        if (!read_string(in, cell.name)) return false;
        if (!read_pod(in, cell.area)) return false;
        if (!read_pod(in, cell.leakage_power)) return false;

        // Pins
        uint32_t npins = 0;
        if (!read_pod(in, npins)) return false;
        cell.pins.resize(npins);
        for (uint32_t p = 0; p < npins; p++) {
            auto& pin = cell.pins[p];
            if (!read_string(in, pin.name)) return false;
            if (!read_string(in, pin.direction)) return false;
            if (!read_string(in, pin.function)) return false;
            if (!read_pod(in, pin.capacitance)) return false;
            if (!read_pod(in, pin.max_transition)) return false;
        }

        // Timings
        uint32_t ntimings = 0;
        if (!read_pod(in, ntimings)) return false;
        cell.timings.resize(ntimings);
        for (uint32_t t = 0; t < ntimings; t++) {
            auto& timing = cell.timings[t];
            if (!read_string(in, timing.related_pin)) return false;
            if (!read_string(in, timing.timing_type)) return false;
            if (!read_pod(in, timing.cell_rise)) return false;
            if (!read_pod(in, timing.cell_fall)) return false;
            if (!read_pod(in, timing.rise_transition)) return false;
            if (!read_pod(in, timing.fall_transition)) return false;

            if (!read_nldm(in, timing.nldm_rise)) return false;
            if (!read_nldm(in, timing.nldm_fall)) return false;
            if (!read_nldm(in, timing.nldm_rise_tr)) return false;
            if (!read_nldm(in, timing.nldm_fall_tr)) return false;
        }

        // Internal powers
        uint32_t npowers = 0;
        if (!read_pod(in, npowers)) return false;
        cell.internal_powers.resize(npowers);
        for (uint32_t p = 0; p < npowers; p++) {
            auto& pwr = cell.internal_powers[p];
            if (!read_string(in, pwr.related_pin)) return false;
            if (!read_string(in, pwr.when)) return false;
            if (!read_pod(in, pwr.rise_power)) return false;
            if (!read_pod(in, pwr.fall_power)) return false;
        }

        // Leakage powers
        uint32_t nleakage = 0;
        if (!read_pod(in, nleakage)) return false;
        cell.leakage_powers.resize(nleakage);
        for (uint32_t l = 0; l < nleakage; l++) {
            auto& lk = cell.leakage_powers[l];
            if (!read_string(in, lk.when)) return false;
            if (!read_pod(in, lk.value)) return false;
        }
    }

    return in.good();
}

bool LibertyCacheReader::is_cache_valid(const std::string& cache_path,
                                         uint64_t source_mtime) {
    std::ifstream in(cache_path, std::ios::binary);
    if (!in.is_open()) return false;

    SlibHeader hdr;
    if (!read_pod(in, hdr)) return false;

    return hdr.magic == SLIB_MAGIC &&
           hdr.version == SLIB_VERSION &&
           hdr.source_mtime == source_mtime;
}

bool LibertyCacheReader::load_or_parse(const std::string& lib_path,
                                        const std::string& cache_path,
                                        LibertyLibrary& lib) {
    // Get source file mtime
    struct stat st;
    uint64_t mtime = 0;
    if (stat(lib_path.c_str(), &st) == 0) {
        mtime = static_cast<uint64_t>(st.st_mtime);
    }

    // Try cache first
    if (is_cache_valid(cache_path, mtime)) {
        if (load_compiled(cache_path, lib)) {
            return true;
        }
    }

    // Cache miss or invalid — parse from source
    if (!lib.parse(lib_path)) return false;

    // Save cache for next time (best-effort, don't fail if can't write)
    LibertyCacheWriter::save_compiled(lib, cache_path, mtime);
    return true;
}

} // namespace sf
