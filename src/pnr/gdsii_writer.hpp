#pragma once
// SiliconForge — GDSII Writer
// Generates a GDSII binary file from the physical design.
// Reference: GDSII Stream Format Manual, Calma Company, 1987

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <cstdint>
#include <fstream>
#include <utility>

namespace sf {

class GdsiiWriter {
public:
    explicit GdsiiWriter(const PhysicalDesign& pd) : pd_(pd) {}

    // Write GDSII to binary file (original flat output)
    bool write(const std::string& filename);

    // Write GDSII to a DEF-like text format (for debugging/inspection)
    std::string to_def() const;

    // --- Hierarchical cell references ---
    struct CellRef {
        std::string cell_name;
        double origin_x, origin_y;
        double angle = 0.0;
        bool mirror_x = false;
        double magnification = 1.0;
    };

    struct ArrayRef {
        std::string cell_name;
        double origin_x, origin_y;
        int cols, rows;
        double col_pitch, row_pitch;
        double angle = 0.0;
    };

    // --- Text labels ---
    struct TextLabel {
        std::string text;
        int layer;
        int datatype = 0;
        double x, y;
        double height = 0.1;
        int anchor = 0;   // 0=NW, 1=N, 2=NE, etc.
    };

    // --- Path elements (wires) ---
    struct PathElement {
        int layer;
        int datatype = 0;
        double width;
        int path_type = 0;  // 0=flush, 1=half-round, 2=half-width extension
        std::vector<std::pair<double,double>> points;
    };

    // --- Layer/Purpose pairs ---
    struct LayerPurpose {
        int layer;
        int datatype;
        std::string name;  // e.g., "M1:drawing", "M1:pin"
    };
    void set_layer_map(const std::vector<LayerPurpose>& map) { layer_map_ = map; }

    // --- Box element ---
    struct BoxElement {
        int layer;
        int datatype;
        double x_lo, y_lo, x_hi, y_hi;
    };

    // Cell library for hierarchy
    void add_cell(const std::string& name,
                  const std::vector<std::pair<int,std::vector<std::pair<double,double>>>>& boundaries,
                  const std::vector<PathElement>& paths = {},
                  const std::vector<TextLabel>& labels = {},
                  const std::vector<CellRef>& srefs = {},
                  const std::vector<ArrayRef>& arefs = {});

    // GDS merge: import cells from another GDS file
    void merge_gds(const std::string& filename);
    void merge_gds_cells(const std::vector<std::string>& cell_names, const std::string& gds_data);

    // Enhanced write with full hierarchy
    void write_hierarchical(const std::string& filename);

private:
    const PhysicalDesign& pd_;
    std::vector<LayerPurpose> layer_map_;

    // Cell definitions for hierarchical output
    struct CellDef {
        std::string name;
        std::vector<std::pair<int,std::vector<std::pair<double,double>>>> boundaries;
        std::vector<PathElement> paths;
        std::vector<TextLabel> labels;
        std::vector<CellRef> srefs;
        std::vector<ArrayRef> arefs;
        std::vector<BoxElement> boxes;
    };
    std::vector<CellDef> cell_defs_;

    // --- Legacy buffer-based helpers (used by write()) ---
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

    // --- Stream-based helpers (used by write_hierarchical()) ---
    void write_record(std::ofstream& f, uint16_t rec_type, const std::vector<uint8_t>& data = {});
    void write_int16_to(std::vector<uint8_t>& buf, int16_t val);
    void write_int32_to(std::vector<uint8_t>& buf, int32_t val);
    void write_real8(std::vector<uint8_t>& buf, double val);
    void write_string_record(std::ofstream& f, uint16_t rec_type, const std::string& str);

    // GDS record type constants (16-bit: high byte = record type, low byte = data type)
    static constexpr uint16_t REC_HEADER   = 0x0002;
    static constexpr uint16_t REC_BGNLIB   = 0x0102;
    static constexpr uint16_t REC_LIBNAME  = 0x0206;
    static constexpr uint16_t REC_UNITS    = 0x0305;
    static constexpr uint16_t REC_ENDLIB   = 0x0400;
    static constexpr uint16_t REC_BGNSTR   = 0x0502;
    static constexpr uint16_t REC_STRNAME  = 0x0606;
    static constexpr uint16_t REC_ENDSTR   = 0x0700;
    static constexpr uint16_t REC_BOUNDARY = 0x0800;
    static constexpr uint16_t REC_PATH     = 0x0900;
    static constexpr uint16_t REC_SREF     = 0x0A00;
    static constexpr uint16_t REC_AREF     = 0x0B00;
    static constexpr uint16_t REC_TEXT     = 0x0C00;
    static constexpr uint16_t REC_LAYER    = 0x0D02;
    static constexpr uint16_t REC_DATATYPE = 0x0E02;
    static constexpr uint16_t REC_WIDTH    = 0x0F03;
    static constexpr uint16_t REC_XY       = 0x1003;
    static constexpr uint16_t REC_ENDEL    = 0x1100;
    static constexpr uint16_t REC_SNAME    = 0x1206;
    static constexpr uint16_t REC_STRANS   = 0x1A01;
    static constexpr uint16_t REC_MAG      = 0x1B05;
    static constexpr uint16_t REC_ANGLE    = 0x1C05;
    static constexpr uint16_t REC_PATHTYPE = 0x2102;
    static constexpr uint16_t REC_TEXTTYPE = 0x1602;
    static constexpr uint16_t REC_STRING   = 0x1906;
    static constexpr uint16_t REC_COLROW   = 0x1302;
    static constexpr uint16_t REC_BOX      = 0x2D00;
    static constexpr uint16_t REC_BOXTYPE  = 0x2E02;
};

} // namespace sf
