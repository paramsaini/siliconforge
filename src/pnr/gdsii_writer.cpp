// SiliconForge — GDSII Writer Implementation
#include "pnr/gdsii_writer.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime>
#include <cstring>

namespace sf {

// GDSII record types
static constexpr uint8_t GDS_HEADER     = 0x00;
static constexpr uint8_t GDS_BGNLIB     = 0x01;
static constexpr uint8_t GDS_LIBNAME    = 0x02;
static constexpr uint8_t GDS_UNITS      = 0x03;
static constexpr uint8_t GDS_ENDLIB     = 0x04;
static constexpr uint8_t GDS_BGNSTR     = 0x05;
static constexpr uint8_t GDS_STRNAME    = 0x06;
static constexpr uint8_t GDS_ENDSTR     = 0x07;
static constexpr uint8_t GDS_BOUNDARY   = 0x08;
static constexpr uint8_t GDS_PATH       = 0x09;
static constexpr uint8_t GDS_ENDEL      = 0x11;
static constexpr uint8_t GDS_LAYER      = 0x0D;
static constexpr uint8_t GDS_DATATYPE   = 0x0E;
static constexpr uint8_t GDS_XY         = 0x10;
static constexpr uint8_t GDS_WIDTH      = 0x0F;
static constexpr uint8_t GDS_PATHTYPE   = 0x21;

// Data types
static constexpr uint8_t DT_NONE    = 0x00;
static constexpr uint8_t DT_INT16   = 0x01;
static constexpr uint8_t DT_INT32   = 0x03;
static constexpr uint8_t DT_REAL8   = 0x05;
static constexpr uint8_t DT_ASCII   = 0x06;

void GdsiiWriter::write_int16(std::vector<uint8_t>& buf, int16_t val) {
    buf.push_back((val >> 8) & 0xFF);
    buf.push_back(val & 0xFF);
}

void GdsiiWriter::write_int32(std::vector<uint8_t>& buf, int32_t val) {
    buf.push_back((val >> 24) & 0xFF);
    buf.push_back((val >> 16) & 0xFF);
    buf.push_back((val >> 8) & 0xFF);
    buf.push_back(val & 0xFF);
}

void GdsiiWriter::write_record(std::vector<uint8_t>& buf, uint8_t rtype, uint8_t dtype,
                                const std::vector<uint8_t>& data) {
    uint16_t len = 4 + (uint16_t)data.size();
    // Pad to even length
    if (len & 1) len++;
    buf.push_back((len >> 8) & 0xFF);
    buf.push_back(len & 0xFF);
    buf.push_back(rtype);
    buf.push_back(dtype);
    buf.insert(buf.end(), data.begin(), data.end());
    if (data.size() & 1) buf.push_back(0); // padding
}

void GdsiiWriter::write_header(std::vector<uint8_t>& buf) {
    std::vector<uint8_t> data;
    write_int16(data, 600); // GDSII version 6.0.0
    write_record(buf, GDS_HEADER, DT_INT16, data);
}

void GdsiiWriter::write_bgnlib(std::vector<uint8_t>& buf, const std::string& libname) {
    // BGNLIB — 12 int16 values for modification/access timestamps
    std::vector<uint8_t> tstamp;
    for (int i = 0; i < 12; ++i) write_int16(tstamp, 1);
    write_record(buf, GDS_BGNLIB, DT_INT16, tstamp);

    // LIBNAME
    std::vector<uint8_t> name(libname.begin(), libname.end());
    if (name.size() & 1) name.push_back(0);
    write_record(buf, GDS_LIBNAME, DT_ASCII, name);

    // UNITS: database units / user units = 0.001, database units / meters = 1e-9
    std::vector<uint8_t> units(16, 0);
    // Real8 encoding for 0.001 and 1e-9 (simplified — using IEEE double trick)
    // For simplicity, we'll write a valid GDSII real8 for 0.001 and 1e-9
    auto write_gds_real8 = [&](double val) {
        if (val == 0) { for(int i=0;i<8;i++) units.push_back(0); return; }
        bool neg = val < 0; if (neg) val = -val;
        int exp = 0;
        while (val < 1.0/16.0) { val *= 16.0; exp--; }
        while (val >= 1.0)     { val /= 16.0; exp++; }
        exp += 64;
        uint64_t mantissa = (uint64_t)(val * (1ULL << 56));
        uint8_t byte0 = (neg ? 0x80 : 0) | (exp & 0x7F);
        units.clear();
        units.push_back(byte0);
        for (int i = 6; i >= 0; --i)
            units.push_back((mantissa >> (i * 8)) & 0xFF);
    };

    // Write 0.001 then 1e-9
    std::vector<uint8_t> unit_data;
    auto append_real8 = [&](double val) {
        bool neg = val < 0; if (neg) val = -val;
        int exp = 0;
        while (val > 0 && val < 1.0/16.0) { val *= 16.0; exp--; }
        while (val >= 1.0) { val /= 16.0; exp++; }
        exp += 64;
        uint64_t mantissa = (uint64_t)(val * (double)(1ULL << 56));
        unit_data.push_back((neg ? 0x80 : 0) | (exp & 0x7F));
        for (int i = 6; i >= 0; --i)
            unit_data.push_back((mantissa >> (i * 8)) & 0xFF);
    };
    append_real8(0.001);
    append_real8(1e-9);
    write_record(buf, GDS_UNITS, DT_REAL8, unit_data);
}

void GdsiiWriter::write_bgnstr(std::vector<uint8_t>& buf, const std::string& strname) {
    std::vector<uint8_t> tstamp;
    for (int i = 0; i < 12; ++i) write_int16(tstamp, 1);
    write_record(buf, GDS_BGNSTR, DT_INT16, tstamp);

    std::vector<uint8_t> name(strname.begin(), strname.end());
    if (name.size() & 1) name.push_back(0);
    write_record(buf, GDS_STRNAME, DT_ASCII, name);
}

void GdsiiWriter::write_boundary(std::vector<uint8_t>& buf, int layer, const Rect& r) {
    write_record(buf, GDS_BOUNDARY, DT_NONE);

    // LAYER
    std::vector<uint8_t> ldata;
    write_int16(ldata, (int16_t)layer);
    write_record(buf, GDS_LAYER, DT_INT16, ldata);

    // DATATYPE
    std::vector<uint8_t> dt;
    write_int16(dt, 0);
    write_record(buf, GDS_DATATYPE, DT_INT16, dt);

    // XY — 5 points (closed polygon), coordinates in database units (nm)
    int scale = 1000; // convert from um to nm
    std::vector<uint8_t> xy;
    auto add_pt = [&](double x, double y) {
        write_int32(xy, (int32_t)(x * scale));
        write_int32(xy, (int32_t)(y * scale));
    };
    add_pt(r.x0, r.y0);
    add_pt(r.x1, r.y0);
    add_pt(r.x1, r.y1);
    add_pt(r.x0, r.y1);
    add_pt(r.x0, r.y0); // close polygon
    write_record(buf, GDS_XY, DT_INT32, xy);

    write_record(buf, GDS_ENDEL, DT_NONE);
}

void GdsiiWriter::write_path(std::vector<uint8_t>& buf, int layer,
                              const Point& p0, const Point& p1, int width) {
    write_record(buf, GDS_PATH, DT_NONE);

    std::vector<uint8_t> ldata;
    write_int16(ldata, (int16_t)layer);
    write_record(buf, GDS_LAYER, DT_INT16, ldata);

    std::vector<uint8_t> dt;
    write_int16(dt, 0);
    write_record(buf, GDS_DATATYPE, DT_INT16, dt);

    std::vector<uint8_t> w;
    write_int32(w, width);
    write_record(buf, GDS_WIDTH, DT_INT32, w);

    int scale = 1000;
    std::vector<uint8_t> xy;
    write_int32(xy, (int32_t)(p0.x * scale));
    write_int32(xy, (int32_t)(p0.y * scale));
    write_int32(xy, (int32_t)(p1.x * scale));
    write_int32(xy, (int32_t)(p1.y * scale));
    write_record(buf, GDS_XY, DT_INT32, xy);

    write_record(buf, GDS_ENDEL, DT_NONE);
}

void GdsiiWriter::write_endstr(std::vector<uint8_t>& buf) {
    write_record(buf, GDS_ENDSTR, DT_NONE);
}

void GdsiiWriter::write_endlib(std::vector<uint8_t>& buf) {
    write_record(buf, GDS_ENDLIB, DT_NONE);
}

bool GdsiiWriter::write(const std::string& filename) {
    std::vector<uint8_t> buf;

    write_header(buf);
    write_bgnlib(buf, "siliconforge");
    write_bgnstr(buf, "TOP");

    // Write cell boundaries
    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        Rect r(c.position.x, c.position.y,
               c.position.x + c.width, c.position.y + c.height);
        int layer = 1; // cell layer
        write_boundary(buf, layer, r);
    }

    // Write wires
    for (auto& w : pd_.wires) {
        write_path(buf, w.layer + 10, w.start, w.end, (int)(w.width * 1000));
    }

    write_endstr(buf);
    write_endlib(buf);

    std::ofstream f(filename, std::ios::binary);
    if (!f.is_open()) return false;
    f.write(reinterpret_cast<const char*>(buf.data()), buf.size());
    return true;
}

std::string GdsiiWriter::to_def() const {
    std::ostringstream def;
    def << "# SiliconForge DEF Output\n"
        << "DESIGN TOP ;\n"
        << "DIEAREA ( " << (int)(pd_.die_area.x0*1000) << " "
        << (int)(pd_.die_area.y0*1000) << " ) ( "
        << (int)(pd_.die_area.x1*1000) << " "
        << (int)(pd_.die_area.y1*1000) << " ) ;\n\n";

    def << "COMPONENTS " << pd_.cells.size() << " ;\n";
    for (auto& c : pd_.cells) {
        def << "  - " << c.name << " " << c.cell_type
            << " + PLACED ( " << (int)(c.position.x*1000) << " "
            << (int)(c.position.y*1000) << " ) N ;\n";
    }
    def << "END COMPONENTS\n\n";

    def << "NETS " << pd_.nets.size() << " ;\n";
    for (auto& n : pd_.nets) {
        def << "  - " << n.name;
        for (auto cid : n.cell_ids)
            def << " ( " << pd_.cells[cid].name << " Z )";
        def << " ;\n";
    }
    def << "END NETS\n";

    def << "END DESIGN\n";
    return def.str();
}

} // namespace sf
