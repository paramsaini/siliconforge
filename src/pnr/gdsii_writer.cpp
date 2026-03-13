// SiliconForge — GDSII Writer Implementation
#include "pnr/gdsii_writer.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace sf {

// ============================================================================
// Legacy record-type constants (uint8_t, used by write() flat path)
// ============================================================================
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

// Data-type tags (used in legacy record encoding)
static constexpr uint8_t DT_NONE    = 0x00;
static constexpr uint8_t DT_INT16   = 0x01;
static constexpr uint8_t DT_INT32   = 0x03;
static constexpr uint8_t DT_REAL8   = 0x05;
static constexpr uint8_t DT_ASCII   = 0x06;

// ============================================================================
// Legacy buffer-based helpers — used by the original write()
// ============================================================================

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
    if (len & 1) len++;
    buf.push_back((len >> 8) & 0xFF);
    buf.push_back(len & 0xFF);
    buf.push_back(rtype);
    buf.push_back(dtype);
    buf.insert(buf.end(), data.begin(), data.end());
    if (data.size() & 1) buf.push_back(0);
}

void GdsiiWriter::write_header(std::vector<uint8_t>& buf) {
    std::vector<uint8_t> data;
    write_int16(data, 600);
    write_record(buf, GDS_HEADER, DT_INT16, data);
}

void GdsiiWriter::write_bgnlib(std::vector<uint8_t>& buf, const std::string& libname) {
    std::vector<uint8_t> tstamp;
    for (int i = 0; i < 12; ++i) write_int16(tstamp, 1);
    write_record(buf, GDS_BGNLIB, DT_INT16, tstamp);

    std::vector<uint8_t> name(libname.begin(), libname.end());
    if (name.size() & 1) name.push_back(0);
    write_record(buf, GDS_LIBNAME, DT_ASCII, name);

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

    std::vector<uint8_t> ldata;
    write_int16(ldata, (int16_t)layer);
    write_record(buf, GDS_LAYER, DT_INT16, ldata);

    std::vector<uint8_t> dt;
    write_int16(dt, 0);
    write_record(buf, GDS_DATATYPE, DT_INT16, dt);

    int scale = 1000;
    std::vector<uint8_t> xy;
    auto add_pt = [&](double x, double y) {
        write_int32(xy, (int32_t)(x * scale));
        write_int32(xy, (int32_t)(y * scale));
    };
    add_pt(r.x0, r.y0);
    add_pt(r.x1, r.y0);
    add_pt(r.x1, r.y1);
    add_pt(r.x0, r.y1);
    add_pt(r.x0, r.y0);
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

// ============================================================================
// Original flat write — unchanged interface
// ============================================================================

bool GdsiiWriter::write(const std::string& filename) {
    std::vector<uint8_t> buf;

    write_header(buf);
    write_bgnlib(buf, "siliconforge");
    write_bgnstr(buf, "TOP");

    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        Rect r(c.position.x, c.position.y,
               c.position.x + c.width, c.position.y + c.height);
        int layer = 1;
        write_boundary(buf, layer, r);
    }

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

// ============================================================================
// Stream-based helpers — used by write_hierarchical()
// ============================================================================

void GdsiiWriter::write_int16_to(std::vector<uint8_t>& buf, int16_t val) {
    buf.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>(val & 0xFF));
}

void GdsiiWriter::write_int32_to(std::vector<uint8_t>& buf, int32_t val) {
    buf.push_back(static_cast<uint8_t>((val >> 24) & 0xFF));
    buf.push_back(static_cast<uint8_t>((val >> 16) & 0xFF));
    buf.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>(val & 0xFF));
}

// GDS excess-64 (IBM hex float) conversion.
// Format: 1-bit sign | 7-bit exponent (base-16, excess-64) | 56-bit mantissa
// value = (-1)^sign × (mantissa / 2^56) × 16^(exponent - 64)
void GdsiiWriter::write_real8(std::vector<uint8_t>& buf, double val) {
    if (val == 0.0) {
        for (int i = 0; i < 8; ++i) buf.push_back(0);
        return;
    }

    uint8_t sign = 0;
    if (val < 0.0) { sign = 0x80; val = -val; }

    // Normalise so that 1/16 <= val < 1 with base-16 exponent
    int exponent = 64;
    while (val < 1.0 / 16.0 && exponent > 0) { val *= 16.0; --exponent; }
    while (val >= 1.0 && exponent < 127)      { val /= 16.0; ++exponent; }

    // Mantissa occupies 56 bits — multiply fractional part by 2^56
    uint64_t mantissa = static_cast<uint64_t>(val * 72057594037927936.0);  // 2^56

    buf.push_back(sign | static_cast<uint8_t>(exponent & 0x7F));
    for (int i = 6; i >= 0; --i)
        buf.push_back(static_cast<uint8_t>((mantissa >> (i * 8)) & 0xFF));
}

// Write a complete GDS record to an ofstream.
// rec_type is the full 16-bit value (record_type << 8 | data_type).
void GdsiiWriter::write_record(std::ofstream& f, uint16_t rec_type,
                                const std::vector<uint8_t>& data) {
    uint16_t data_len = static_cast<uint16_t>(data.size());
    uint16_t len = 4 + data_len;
    if (len & 1) ++len;  // pad to even

    uint8_t hdr[4];
    hdr[0] = static_cast<uint8_t>((len >> 8) & 0xFF);
    hdr[1] = static_cast<uint8_t>(len & 0xFF);
    hdr[2] = static_cast<uint8_t>((rec_type >> 8) & 0xFF);
    hdr[3] = static_cast<uint8_t>(rec_type & 0xFF);
    f.write(reinterpret_cast<const char*>(hdr), 4);

    if (!data.empty())
        f.write(reinterpret_cast<const char*>(data.data()), data.size());
    if (data_len & 1) {
        uint8_t pad = 0;
        f.write(reinterpret_cast<const char*>(&pad), 1);
    }
}

void GdsiiWriter::write_string_record(std::ofstream& f, uint16_t rec_type,
                                       const std::string& str) {
    std::vector<uint8_t> data(str.begin(), str.end());
    if (data.size() & 1) data.push_back(0);  // pad to even
    write_record(f, rec_type, data);
}

// ============================================================================
// Cell management
// ============================================================================

void GdsiiWriter::add_cell(
        const std::string& name,
        const std::vector<std::pair<int,std::vector<std::pair<double,double>>>>& boundaries,
        const std::vector<PathElement>& paths,
        const std::vector<TextLabel>& labels,
        const std::vector<CellRef>& srefs,
        const std::vector<ArrayRef>& arefs)
{
    CellDef cd;
    cd.name       = name;
    cd.boundaries = boundaries;
    cd.paths      = paths;
    cd.labels     = labels;
    cd.srefs      = srefs;
    cd.arefs      = arefs;
    cell_defs_.push_back(std::move(cd));
}

// ============================================================================
// GDS merge — import cell definitions from an external GDS file
// ============================================================================

void GdsiiWriter::merge_gds(const std::string& filename) {
    std::ifstream in(filename, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "GdsiiWriter::merge_gds: cannot open " << filename << "\n";
        return;
    }

    // Read entire file into a buffer
    std::vector<uint8_t> buf((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
    in.close();

    // Parse record by record, extracting structure definitions
    size_t pos = 0;
    bool in_struct = false;
    CellDef current;
    int boundary_layer = 0;
    std::vector<std::pair<double,double>> boundary_pts;
    bool in_boundary = false;

    auto read_u16 = [&](size_t offset) -> uint16_t {
        return (static_cast<uint16_t>(buf[offset]) << 8) | buf[offset + 1];
    };

    auto read_i32 = [&](size_t offset) -> int32_t {
        return (static_cast<int32_t>(buf[offset]) << 24) |
               (static_cast<int32_t>(buf[offset+1]) << 16) |
               (static_cast<int32_t>(buf[offset+2]) << 8) |
               static_cast<int32_t>(buf[offset+3]);
    };

    while (pos + 4 <= buf.size()) {
        uint16_t rec_len  = read_u16(pos);
        uint16_t rec_type = read_u16(pos + 2);

        if (rec_len < 4 || pos + rec_len > buf.size()) break;

        uint8_t rtype = static_cast<uint8_t>((rec_type >> 8) & 0xFF);
        size_t data_start = pos + 4;
        size_t data_len   = rec_len - 4;

        if (rtype == 0x05) {  // BGNSTR
            in_struct = true;
            current = CellDef{};
        } else if (rtype == 0x06 && in_struct) {  // STRNAME
            std::string sname(buf.begin() + data_start,
                              buf.begin() + data_start + data_len);
            while (!sname.empty() && sname.back() == '\0') sname.pop_back();
            current.name = sname;
        } else if (rtype == 0x08) {  // BOUNDARY
            in_boundary = true;
            boundary_layer = 0;
            boundary_pts.clear();
        } else if (rtype == 0x0D && in_boundary) {  // LAYER
            if (data_len >= 2)
                boundary_layer = static_cast<int16_t>(read_u16(data_start));
        } else if (rtype == 0x10 && in_boundary) {  // XY
            size_t n_coords = data_len / 4;
            for (size_t i = 0; i + 1 < n_coords; i += 2) {
                double x = read_i32(data_start + i * 4) / 1000.0;
                double y = read_i32(data_start + (i + 1) * 4) / 1000.0;
                boundary_pts.emplace_back(x, y);
            }
        } else if (rtype == 0x11 && in_boundary) {  // ENDEL
            if (!boundary_pts.empty() && in_struct) {
                current.boundaries.push_back({boundary_layer, boundary_pts});
            }
            in_boundary = false;
        } else if (rtype == 0x07 && in_struct) {  // ENDSTR
            if (!current.name.empty())
                cell_defs_.push_back(std::move(current));
            in_struct = false;
        }

        pos += rec_len;
    }
}

void GdsiiWriter::merge_gds_cells(const std::vector<std::string>& cell_names,
                                    const std::string& gds_data) {
    // Write gds_data to a temp file, then use merge_gds
    std::string tmpfile = "/tmp/sf_merge_tmp.gds";
    {
        std::ofstream tmp(tmpfile, std::ios::binary);
        tmp.write(gds_data.data(), static_cast<std::streamsize>(gds_data.size()));
    }
    merge_gds(tmpfile);

    if (!cell_names.empty()) {
        // Filter cell_defs_ to keep only requested cells plus any already present
        std::vector<CellDef> filtered;
        for (auto& cd : cell_defs_) {
            bool keep = false;
            for (auto& cn : cell_names) {
                if (cd.name == cn) { keep = true; break; }
            }
            if (keep) filtered.push_back(std::move(cd));
        }
        cell_defs_ = std::move(filtered);
    }
    std::remove(tmpfile.c_str());
}

// ============================================================================
// Hierarchical GDS output
// ============================================================================

void GdsiiWriter::write_hierarchical(const std::string& filename) {
    std::ofstream f(filename, std::ios::binary);
    if (!f.is_open()) {
        std::cerr << "GdsiiWriter::write_hierarchical: cannot open " << filename << "\n";
        return;
    }

    constexpr int SCALE = 1000;  // um → database units (nm)

    // ----- HEADER -----
    {
        std::vector<uint8_t> d;
        write_int16_to(d, 600);
        write_record(f, REC_HEADER, d);
    }

    // ----- BGNLIB -----
    {
        std::vector<uint8_t> d;
        for (int i = 0; i < 12; ++i) write_int16_to(d, 1);
        write_record(f, REC_BGNLIB, d);
    }

    // ----- LIBNAME -----
    write_string_record(f, REC_LIBNAME, "siliconforge");

    // ----- UNITS: user unit = 0.001 um, db unit = 1e-9 m -----
    {
        std::vector<uint8_t> d;
        write_real8(d, 0.001);
        write_real8(d, 1e-9);
        write_record(f, REC_UNITS, d);
    }

    // ----- Sub-cells -----
    for (auto& cell : cell_defs_) {
        // BGNSTR
        {
            std::vector<uint8_t> d;
            for (int i = 0; i < 12; ++i) write_int16_to(d, 1);
            write_record(f, REC_BGNSTR, d);
        }
        write_string_record(f, REC_STRNAME, cell.name);

        // Boundary elements
        for (auto& [layer, pts] : cell.boundaries) {
            write_record(f, REC_BOUNDARY);
            {
                std::vector<uint8_t> d; write_int16_to(d, static_cast<int16_t>(layer));
                write_record(f, REC_LAYER, d);
            }
            {
                std::vector<uint8_t> d; write_int16_to(d, 0);
                write_record(f, REC_DATATYPE, d);
            }
            {
                std::vector<uint8_t> d;
                for (auto& [x, y] : pts) {
                    write_int32_to(d, static_cast<int32_t>(x * SCALE));
                    write_int32_to(d, static_cast<int32_t>(y * SCALE));
                }
                write_record(f, REC_XY, d);
            }
            write_record(f, REC_ENDEL);
        }

        // Path elements
        for (auto& pe : cell.paths) {
            write_record(f, REC_PATH);
            {
                std::vector<uint8_t> d; write_int16_to(d, static_cast<int16_t>(pe.layer));
                write_record(f, REC_LAYER, d);
            }
            {
                std::vector<uint8_t> d; write_int16_to(d, static_cast<int16_t>(pe.datatype));
                write_record(f, REC_DATATYPE, d);
            }
            {
                std::vector<uint8_t> d; write_int16_to(d, static_cast<int16_t>(pe.path_type));
                write_record(f, REC_PATHTYPE, d);
            }
            {
                std::vector<uint8_t> d;
                write_int32_to(d, static_cast<int32_t>(pe.width * SCALE));
                write_record(f, REC_WIDTH, d);
            }
            {
                std::vector<uint8_t> d;
                for (auto& [x, y] : pe.points) {
                    write_int32_to(d, static_cast<int32_t>(x * SCALE));
                    write_int32_to(d, static_cast<int32_t>(y * SCALE));
                }
                write_record(f, REC_XY, d);
            }
            write_record(f, REC_ENDEL);
        }

        // Text labels
        for (auto& tl : cell.labels) {
            write_record(f, REC_TEXT);
            {
                std::vector<uint8_t> d; write_int16_to(d, static_cast<int16_t>(tl.layer));
                write_record(f, REC_LAYER, d);
            }
            {
                std::vector<uint8_t> d; write_int16_to(d, static_cast<int16_t>(tl.datatype));
                write_record(f, REC_TEXTTYPE, d);
            }
            {
                std::vector<uint8_t> d;
                write_int32_to(d, static_cast<int32_t>(tl.x * SCALE));
                write_int32_to(d, static_cast<int32_t>(tl.y * SCALE));
                write_record(f, REC_XY, d);
            }
            write_string_record(f, REC_STRING, tl.text);
            write_record(f, REC_ENDEL);
        }

        // SREF — single cell references
        for (auto& sr : cell.srefs) {
            write_record(f, REC_SREF);
            write_string_record(f, REC_SNAME, sr.cell_name);

            bool need_strans = sr.mirror_x || sr.angle != 0.0 || sr.magnification != 1.0;
            if (need_strans) {
                std::vector<uint8_t> d;
                uint16_t flags = sr.mirror_x ? 0x8000 : 0x0000;
                write_int16_to(d, static_cast<int16_t>(flags));
                write_record(f, REC_STRANS, d);

                if (sr.magnification != 1.0) {
                    std::vector<uint8_t> md;
                    write_real8(md, sr.magnification);
                    write_record(f, REC_MAG, md);
                }
                if (sr.angle != 0.0) {
                    std::vector<uint8_t> ad;
                    write_real8(ad, sr.angle);
                    write_record(f, REC_ANGLE, ad);
                }
            }
            {
                std::vector<uint8_t> d;
                write_int32_to(d, static_cast<int32_t>(sr.origin_x * SCALE));
                write_int32_to(d, static_cast<int32_t>(sr.origin_y * SCALE));
                write_record(f, REC_XY, d);
            }
            write_record(f, REC_ENDEL);
        }

        // AREF — array cell references
        for (auto& ar : cell.arefs) {
            write_record(f, REC_AREF);
            write_string_record(f, REC_SNAME, ar.cell_name);

            if (ar.angle != 0.0) {
                std::vector<uint8_t> d;
                write_int16_to(d, 0);
                write_record(f, REC_STRANS, d);
                std::vector<uint8_t> ad;
                write_real8(ad, ar.angle);
                write_record(f, REC_ANGLE, ad);
            }

            {
                std::vector<uint8_t> d;
                write_int16_to(d, static_cast<int16_t>(ar.cols));
                write_int16_to(d, static_cast<int16_t>(ar.rows));
                write_record(f, REC_COLROW, d);
            }
            {
                // XY for AREF: 3 points — origin, col-end, row-end
                std::vector<uint8_t> d;
                write_int32_to(d, static_cast<int32_t>(ar.origin_x * SCALE));
                write_int32_to(d, static_cast<int32_t>(ar.origin_y * SCALE));
                write_int32_to(d, static_cast<int32_t>((ar.origin_x + ar.cols * ar.col_pitch) * SCALE));
                write_int32_to(d, static_cast<int32_t>(ar.origin_y * SCALE));
                write_int32_to(d, static_cast<int32_t>(ar.origin_x * SCALE));
                write_int32_to(d, static_cast<int32_t>((ar.origin_y + ar.rows * ar.row_pitch) * SCALE));
                write_record(f, REC_XY, d);
            }
            write_record(f, REC_ENDEL);
        }

        // Box elements
        for (auto& bx : cell.boxes) {
            write_record(f, REC_BOX);
            {
                std::vector<uint8_t> d; write_int16_to(d, static_cast<int16_t>(bx.layer));
                write_record(f, REC_LAYER, d);
            }
            {
                std::vector<uint8_t> d; write_int16_to(d, static_cast<int16_t>(bx.datatype));
                write_record(f, REC_BOXTYPE, d);
            }
            {
                std::vector<uint8_t> d;
                write_int32_to(d, static_cast<int32_t>(bx.x_lo * SCALE));
                write_int32_to(d, static_cast<int32_t>(bx.y_lo * SCALE));
                write_int32_to(d, static_cast<int32_t>(bx.x_hi * SCALE));
                write_int32_to(d, static_cast<int32_t>(bx.y_lo * SCALE));
                write_int32_to(d, static_cast<int32_t>(bx.x_hi * SCALE));
                write_int32_to(d, static_cast<int32_t>(bx.y_hi * SCALE));
                write_int32_to(d, static_cast<int32_t>(bx.x_lo * SCALE));
                write_int32_to(d, static_cast<int32_t>(bx.y_hi * SCALE));
                write_int32_to(d, static_cast<int32_t>(bx.x_lo * SCALE));
                write_int32_to(d, static_cast<int32_t>(bx.y_lo * SCALE));
                write_record(f, REC_XY, d);
            }
            write_record(f, REC_ENDEL);
        }

        write_record(f, REC_ENDSTR);
    }

    // ----- Top-level structure with placed cells as SREFs -----
    {
        std::vector<uint8_t> d;
        for (int i = 0; i < 12; ++i) write_int16_to(d, 1);
        write_record(f, REC_BGNSTR, d);
    }
    write_string_record(f, REC_STRNAME, "TOP");

    // Boundary for die area
    {
        write_record(f, REC_BOUNDARY);
        {
            std::vector<uint8_t> d; write_int16_to(d, 0);
            write_record(f, REC_LAYER, d);
        }
        {
            std::vector<uint8_t> d; write_int16_to(d, 0);
            write_record(f, REC_DATATYPE, d);
        }
        {
            std::vector<uint8_t> d;
            write_int32_to(d, static_cast<int32_t>(pd_.die_area.x0 * SCALE));
            write_int32_to(d, static_cast<int32_t>(pd_.die_area.y0 * SCALE));
            write_int32_to(d, static_cast<int32_t>(pd_.die_area.x1 * SCALE));
            write_int32_to(d, static_cast<int32_t>(pd_.die_area.y0 * SCALE));
            write_int32_to(d, static_cast<int32_t>(pd_.die_area.x1 * SCALE));
            write_int32_to(d, static_cast<int32_t>(pd_.die_area.y1 * SCALE));
            write_int32_to(d, static_cast<int32_t>(pd_.die_area.x0 * SCALE));
            write_int32_to(d, static_cast<int32_t>(pd_.die_area.y1 * SCALE));
            write_int32_to(d, static_cast<int32_t>(pd_.die_area.x0 * SCALE));
            write_int32_to(d, static_cast<int32_t>(pd_.die_area.y0 * SCALE));
            write_record(f, REC_XY, d);
        }
        write_record(f, REC_ENDEL);
    }

    // Place each PhysicalDesign cell as an SREF if a matching CellDef exists
    for (auto& c : pd_.cells) {
        if (!c.placed) continue;

        // Check if there is a cell_def for this cell_type
        bool has_def = false;
        for (auto& cd : cell_defs_) {
            if (cd.name == c.cell_type) { has_def = true; break; }
        }

        if (has_def) {
            // Write as SREF
            write_record(f, REC_SREF);
            write_string_record(f, REC_SNAME, c.cell_type);
            {
                std::vector<uint8_t> d;
                write_int32_to(d, static_cast<int32_t>(c.position.x * SCALE));
                write_int32_to(d, static_cast<int32_t>(c.position.y * SCALE));
                write_record(f, REC_XY, d);
            }
            write_record(f, REC_ENDEL);
        } else {
            // Fallback: write as boundary
            write_record(f, REC_BOUNDARY);
            {
                std::vector<uint8_t> d; write_int16_to(d, 1);
                write_record(f, REC_LAYER, d);
            }
            {
                std::vector<uint8_t> d; write_int16_to(d, 0);
                write_record(f, REC_DATATYPE, d);
            }
            {
                std::vector<uint8_t> d;
                double x0 = c.position.x, y0 = c.position.y;
                double x1 = x0 + c.width,  y1 = y0 + c.height;
                write_int32_to(d, static_cast<int32_t>(x0 * SCALE));
                write_int32_to(d, static_cast<int32_t>(y0 * SCALE));
                write_int32_to(d, static_cast<int32_t>(x1 * SCALE));
                write_int32_to(d, static_cast<int32_t>(y0 * SCALE));
                write_int32_to(d, static_cast<int32_t>(x1 * SCALE));
                write_int32_to(d, static_cast<int32_t>(y1 * SCALE));
                write_int32_to(d, static_cast<int32_t>(x0 * SCALE));
                write_int32_to(d, static_cast<int32_t>(y1 * SCALE));
                write_int32_to(d, static_cast<int32_t>(x0 * SCALE));
                write_int32_to(d, static_cast<int32_t>(y0 * SCALE));
                write_record(f, REC_XY, d);
            }
            write_record(f, REC_ENDEL);
        }
    }

    // Wires as PATH elements in the top cell
    for (auto& w : pd_.wires) {
        write_record(f, REC_PATH);
        {
            std::vector<uint8_t> d; write_int16_to(d, static_cast<int16_t>(w.layer + 10));
            write_record(f, REC_LAYER, d);
        }
        {
            std::vector<uint8_t> d; write_int16_to(d, 0);
            write_record(f, REC_DATATYPE, d);
        }
        {
            std::vector<uint8_t> d;
            write_int32_to(d, static_cast<int32_t>(w.width * SCALE));
            write_record(f, REC_WIDTH, d);
        }
        {
            std::vector<uint8_t> d;
            write_int32_to(d, static_cast<int32_t>(w.start.x * SCALE));
            write_int32_to(d, static_cast<int32_t>(w.start.y * SCALE));
            write_int32_to(d, static_cast<int32_t>(w.end.x * SCALE));
            write_int32_to(d, static_cast<int32_t>(w.end.y * SCALE));
            write_record(f, REC_XY, d);
        }
        write_record(f, REC_ENDEL);
    }

    write_record(f, REC_ENDSTR);

    // ----- ENDLIB -----
    write_record(f, REC_ENDLIB);
}

} // namespace sf
