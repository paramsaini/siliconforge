#pragma once
// SiliconForge — GDSII Reader
// Parses GDSII Stream Format binary files into PhysicalDesign structures.
// Supports: BOUNDARY, PATH, SREF, AREF, TEXT records.
// Reference: GDSII Stream Format Manual, Calma Company, 1987
//
// Record format: 2-byte length (including header) + 1-byte record type
//                + 1-byte data type + variable data payload.

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <cstdint>
#include <unordered_map>
#include <fstream>

namespace sf {

// GDSII record types (from spec)
namespace gdsii {
    constexpr uint8_t HEADER     = 0x00;
    constexpr uint8_t BGNLIB     = 0x01;
    constexpr uint8_t LIBNAME    = 0x02;
    constexpr uint8_t UNITS      = 0x03;
    constexpr uint8_t ENDLIB     = 0x04;
    constexpr uint8_t BGNSTR     = 0x05;
    constexpr uint8_t STRNAME    = 0x06;
    constexpr uint8_t ENDSTR     = 0x07;
    constexpr uint8_t BOUNDARY   = 0x08;
    constexpr uint8_t PATH       = 0x09;
    constexpr uint8_t SREF       = 0x0A;
    constexpr uint8_t AREF       = 0x0B;
    constexpr uint8_t TEXT       = 0x0C;
    constexpr uint8_t LAYER      = 0x0D;
    constexpr uint8_t DATATYPE   = 0x0E;
    constexpr uint8_t WIDTH      = 0x0F;
    constexpr uint8_t XY         = 0x10;
    constexpr uint8_t ENDEL      = 0x11;
    constexpr uint8_t SNAME      = 0x12;
    constexpr uint8_t COLROW     = 0x13;
    constexpr uint8_t STRANS     = 0x1A;
    constexpr uint8_t MAG        = 0x1B;
    constexpr uint8_t ANGLE      = 0x1C;
    constexpr uint8_t PATHTYPE   = 0x21;
    constexpr uint8_t STRING     = 0x19;
    constexpr uint8_t PROPATTR   = 0x2B;
    constexpr uint8_t PROPVALUE  = 0x2C;
    constexpr uint8_t BOX        = 0x2D;
    constexpr uint8_t BOXTYPE    = 0x2E;

    // Data types
    constexpr uint8_t DT_NONE    = 0x00;
    constexpr uint8_t DT_INT2    = 0x01;
    constexpr uint8_t DT_INT4    = 0x02;
    constexpr uint8_t DT_REAL8   = 0x05;
    constexpr uint8_t DT_ASCII   = 0x06;
}

// Parsed GDSII structures

struct GdsPolygon {
    int layer = 0;
    int datatype = 0;
    std::vector<std::pair<double, double>> points;  // (x, y) in um
};

struct GdsPath {
    int layer = 0;
    int datatype = 0;
    int pathtype = 0;           // 0=flush, 1=half-round, 2=half-width extend
    double width = 0;           // in um
    std::vector<std::pair<double, double>> points;
};

struct GdsText {
    int layer = 0;
    std::string text;
    double x = 0, y = 0;
};

struct GdsCellRef {
    std::string cell_name;
    double x = 0, y = 0;
    double angle = 0;
    double mag = 1.0;
    bool mirror_x = false;
};

struct GdsArrayRef {
    std::string cell_name;
    double x = 0, y = 0;
    int cols = 1, rows = 1;
    double col_pitch = 0, row_pitch = 0;
    double angle = 0;
    double mag = 1.0;
    bool mirror_x = false;
};

struct GdsCell {
    std::string name;
    std::vector<GdsPolygon> polygons;
    std::vector<GdsPath> paths;
    std::vector<GdsText> texts;
    std::vector<GdsCellRef> refs;
    std::vector<GdsArrayRef> arefs;
    Rect bbox;  // computed bounding box
};

struct GdsLibrary {
    std::string name;
    double user_units = 0.001;    // user units per database unit
    double db_units = 1e-9;       // database units in meters
    std::vector<GdsCell> cells;
    std::unordered_map<std::string, int> cell_index;  // name -> index

    const GdsCell* find_cell(const std::string& name) const {
        auto it = cell_index.find(name);
        return (it != cell_index.end()) ? &cells[it->second] : nullptr;
    }
};

struct GdsReadResult {
    bool success = false;
    std::string error;
    int cells_read = 0;
    int polygons_read = 0;
    int paths_read = 0;
    int refs_read = 0;
    double time_ms = 0;
    GdsLibrary library;
};

class GdsiiReader {
public:
    GdsiiReader() = default;

    // Parse a GDSII binary file
    GdsReadResult read(const std::string& filename);

    // Parse from memory buffer
    GdsReadResult read_buffer(const uint8_t* data, size_t size);

    // Flatten a hierarchical GDSII into a PhysicalDesign
    // Resolves all SREF/AREF into flat geometry
    PhysicalDesign flatten(const GdsLibrary& lib,
                           const std::string& top_cell = "") const;

    // Layer mapping: GDSII layer -> PhysicalDesign layer index
    void set_layer_map(const std::unordered_map<int, int>& map) {
        layer_map_ = map;
    }

    // Scale factor: GDSII units -> um
    void set_scale(double scale_um) { scale_ = scale_um; }

private:
    std::unordered_map<int, int> layer_map_;
    double scale_ = 0.001;  // default: nm -> um

    // Binary parsing helpers
    struct Record {
        uint8_t type;
        uint8_t datatype;
        std::vector<uint8_t> data;
    };

    bool read_record(std::istream& in, Record& rec);
    int16_t read_int16(const uint8_t* p) const;
    int32_t read_int32(const uint8_t* p) const;
    double read_real8(const uint8_t* p) const;
    std::string read_string(const Record& rec) const;
    std::vector<std::pair<double, double>> read_xy(const Record& rec) const;

    // Cell parsing state machine
    void parse_boundary(std::istream& in, GdsCell& cell);
    void parse_path(std::istream& in, GdsCell& cell);
    void parse_sref(std::istream& in, GdsCell& cell);
    void parse_aref(std::istream& in, GdsCell& cell);
    void parse_text(std::istream& in, GdsCell& cell);

    // Flatten helper: recursively instantiate cell refs
    void flatten_cell(const GdsLibrary& lib, const GdsCell& cell,
                      double ox, double oy, double angle, double mag,
                      bool mirror, PhysicalDesign& pd,
                      int depth = 0) const;
};

} // namespace sf
