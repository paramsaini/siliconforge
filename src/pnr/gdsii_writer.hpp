#pragma once
// SiliconForge — GDSII Writer
// Generates a GDSII binary file from the physical design.
// Reference: GDSII Stream Format Manual, Calma Company, 1987

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <cstdint>

namespace sf {

class GdsiiWriter {
public:
    explicit GdsiiWriter(const PhysicalDesign& pd) : pd_(pd) {}

    // Write GDSII to binary file
    bool write(const std::string& filename);

    // Write GDSII to a DEF-like text format (for debugging/inspection)
    std::string to_def() const;

private:
    const PhysicalDesign& pd_;

    // GDSII binary helpers
    void write_header(std::vector<uint8_t>& buf);
    void write_bgnlib(std::vector<uint8_t>& buf, const std::string& libname);
    void write_bgnstr(std::vector<uint8_t>& buf, const std::string& strname);
    void write_boundary(std::vector<uint8_t>& buf, int layer, const Rect& r);
    void write_path(std::vector<uint8_t>& buf, int layer, const Point& p0, const Point& p1, int width);
    void write_endstr(std::vector<uint8_t>& buf);
    void write_endlib(std::vector<uint8_t>& buf);

    void write_record(std::vector<uint8_t>& buf, uint8_t rtype, uint8_t dtype,
                      const std::vector<uint8_t>& data = {});
    void write_int16(std::vector<uint8_t>& buf, int16_t val);
    void write_int32(std::vector<uint8_t>& buf, int32_t val);
};

} // namespace sf
