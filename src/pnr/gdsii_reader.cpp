// SiliconForge — GDSII Reader implementation
// Binary parser for GDSII Stream Format (Calma 1987)

#include "pnr/gdsii_reader.hpp"
#include <chrono>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <sstream>

namespace sf {

// ============================================================================
// Binary parsing helpers
// ============================================================================

int16_t GdsiiReader::read_int16(const uint8_t* p) const {
    return (int16_t)((p[0] << 8) | p[1]);
}

int32_t GdsiiReader::read_int32(const uint8_t* p) const {
    return (int32_t)((p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]);
}

double GdsiiReader::read_real8(const uint8_t* p) const {
    // GDSII uses excess-64 floating point (IBM format), NOT IEEE 754
    // Format: 1 sign bit, 7 exponent bits (excess-64), 56 mantissa bits
    // Value = (-1)^sign * mantissa * 16^(exponent - 64)
    int sign = (p[0] >> 7) & 1;
    int exponent = (p[0] & 0x7F) - 64;
    double mantissa = 0;
    for (int i = 1; i < 8; ++i) {
        mantissa = mantissa * 256.0 + p[i];
    }
    mantissa /= std::pow(2.0, 56);
    double value = mantissa * std::pow(16.0, exponent);
    return sign ? -value : value;
}

std::string GdsiiReader::read_string(const Record& rec) const {
    std::string s;
    for (auto c : rec.data) {
        if (c == 0) break;
        s += (char)c;
    }
    return s;
}

std::vector<std::pair<double, double>> GdsiiReader::read_xy(const Record& rec) const {
    std::vector<std::pair<double, double>> points;
    // XY data: pairs of 4-byte signed integers
    size_t num_pts = rec.data.size() / 8;
    for (size_t i = 0; i < num_pts; ++i) {
        int32_t x = read_int32(&rec.data[i * 8]);
        int32_t y = read_int32(&rec.data[i * 8 + 4]);
        points.push_back({x * scale_, y * scale_});
    }
    return points;
}

bool GdsiiReader::read_record(std::istream& in, Record& rec) {
    uint8_t header[4];
    if (!in.read(reinterpret_cast<char*>(header), 4)) return false;

    uint16_t length = (header[0] << 8) | header[1];
    rec.type = header[2];
    rec.datatype = header[3];

    rec.data.clear();
    if (length > 4) {
        rec.data.resize(length - 4);
        if (!in.read(reinterpret_cast<char*>(rec.data.data()), length - 4))
            return false;
    }
    return true;
}

// ============================================================================
// Element parsers
// ============================================================================

void GdsiiReader::parse_boundary(std::istream& in, GdsCell& cell) {
    GdsPolygon poly;
    Record rec;
    while (read_record(in, rec)) {
        if (rec.type == gdsii::ENDEL) break;
        if (rec.type == gdsii::LAYER && rec.data.size() >= 2)
            poly.layer = read_int16(rec.data.data());
        if (rec.type == gdsii::DATATYPE && rec.data.size() >= 2)
            poly.datatype = read_int16(rec.data.data());
        if (rec.type == gdsii::XY)
            poly.points = read_xy(rec);
    }
    cell.polygons.push_back(poly);
}

void GdsiiReader::parse_path(std::istream& in, GdsCell& cell) {
    GdsPath path;
    Record rec;
    while (read_record(in, rec)) {
        if (rec.type == gdsii::ENDEL) break;
        if (rec.type == gdsii::LAYER && rec.data.size() >= 2)
            path.layer = read_int16(rec.data.data());
        if (rec.type == gdsii::DATATYPE && rec.data.size() >= 2)
            path.datatype = read_int16(rec.data.data());
        if (rec.type == gdsii::PATHTYPE && rec.data.size() >= 2)
            path.pathtype = read_int16(rec.data.data());
        if (rec.type == gdsii::WIDTH && rec.data.size() >= 4)
            path.width = read_int32(rec.data.data()) * scale_;
        if (rec.type == gdsii::XY)
            path.points = read_xy(rec);
    }
    cell.paths.push_back(path);
}

void GdsiiReader::parse_sref(std::istream& in, GdsCell& cell) {
    GdsCellRef ref;
    Record rec;
    while (read_record(in, rec)) {
        if (rec.type == gdsii::ENDEL) break;
        if (rec.type == gdsii::SNAME)
            ref.cell_name = read_string(rec);
        if (rec.type == gdsii::STRANS && rec.data.size() >= 2)
            ref.mirror_x = (rec.data[0] & 0x80) != 0;
        if (rec.type == gdsii::MAG && rec.data.size() >= 8)
            ref.mag = read_real8(rec.data.data());
        if (rec.type == gdsii::ANGLE && rec.data.size() >= 8)
            ref.angle = read_real8(rec.data.data());
        if (rec.type == gdsii::XY && rec.data.size() >= 8) {
            ref.x = read_int32(rec.data.data()) * scale_;
            ref.y = read_int32(rec.data.data() + 4) * scale_;
        }
    }
    cell.refs.push_back(ref);
}

void GdsiiReader::parse_aref(std::istream& in, GdsCell& cell) {
    GdsArrayRef aref;
    Record rec;
    while (read_record(in, rec)) {
        if (rec.type == gdsii::ENDEL) break;
        if (rec.type == gdsii::SNAME)
            aref.cell_name = read_string(rec);
        if (rec.type == gdsii::STRANS && rec.data.size() >= 2)
            aref.mirror_x = (rec.data[0] & 0x80) != 0;
        if (rec.type == gdsii::MAG && rec.data.size() >= 8)
            aref.mag = read_real8(rec.data.data());
        if (rec.type == gdsii::ANGLE && rec.data.size() >= 8)
            aref.angle = read_real8(rec.data.data());
        if (rec.type == gdsii::COLROW && rec.data.size() >= 4) {
            aref.cols = read_int16(rec.data.data());
            aref.rows = read_int16(rec.data.data() + 2);
        }
        if (rec.type == gdsii::XY && rec.data.size() >= 24) {
            // 3 points: origin, col-end, row-end
            aref.x = read_int32(rec.data.data()) * scale_;
            aref.y = read_int32(rec.data.data() + 4) * scale_;
            double x1 = read_int32(rec.data.data() + 8) * scale_;
            double x2 = read_int32(rec.data.data() + 16) * scale_;
            double y2 = read_int32(rec.data.data() + 20) * scale_;
            if (aref.cols > 0) aref.col_pitch = (x1 - aref.x) / aref.cols;
            if (aref.rows > 0) aref.row_pitch = (y2 - aref.y) / aref.rows;
        }
    }
    cell.arefs.push_back(aref);
}

void GdsiiReader::parse_text(std::istream& in, GdsCell& cell) {
    GdsText text;
    Record rec;
    while (read_record(in, rec)) {
        if (rec.type == gdsii::ENDEL) break;
        if (rec.type == gdsii::LAYER && rec.data.size() >= 2)
            text.layer = read_int16(rec.data.data());
        if (rec.type == gdsii::STRING)
            text.text = read_string(rec);
        if (rec.type == gdsii::XY && rec.data.size() >= 8) {
            text.x = read_int32(rec.data.data()) * scale_;
            text.y = read_int32(rec.data.data() + 4) * scale_;
        }
    }
    cell.texts.push_back(text);
}

// ============================================================================
// Main reader
// ============================================================================

GdsReadResult GdsiiReader::read(const std::string& filename) {
    auto t0 = std::chrono::steady_clock::now();
    GdsReadResult result;

    std::ifstream in(filename, std::ios::binary);
    if (!in) {
        result.error = "Cannot open file: " + filename;
        return result;
    }

    Record rec;
    GdsCell* current_cell = nullptr;

    while (read_record(in, rec)) {
        switch (rec.type) {
            case gdsii::HEADER:
                // GDSII version
                break;

            case gdsii::LIBNAME:
                result.library.name = read_string(rec);
                break;

            case gdsii::UNITS:
                if (rec.data.size() >= 16) {
                    result.library.user_units = read_real8(rec.data.data());
                    result.library.db_units = read_real8(rec.data.data() + 8);
                    // Update scale: db_unit -> um
                    scale_ = result.library.db_units * 1e6;
                }
                break;

            case gdsii::BGNSTR:
                result.library.cells.emplace_back();
                current_cell = &result.library.cells.back();
                break;

            case gdsii::STRNAME:
                if (current_cell) {
                    current_cell->name = read_string(rec);
                    result.library.cell_index[current_cell->name] =
                        (int)result.library.cells.size() - 1;
                }
                break;

            case gdsii::ENDSTR:
                if (current_cell) {
                    result.cells_read++;
                    // Compute bounding box
                    double xmin = 1e18, ymin = 1e18, xmax = -1e18, ymax = -1e18;
                    for (auto& p : current_cell->polygons) {
                        for (auto& [x, y] : p.points) {
                            xmin = std::min(xmin, x); xmax = std::max(xmax, x);
                            ymin = std::min(ymin, y); ymax = std::max(ymax, y);
                        }
                    }
                    for (auto& path : current_cell->paths) {
                        for (auto& [x, y] : path.points) {
                            xmin = std::min(xmin, x); xmax = std::max(xmax, x);
                            ymin = std::min(ymin, y); ymax = std::max(ymax, y);
                        }
                    }
                    current_cell->bbox = {xmin, ymin, xmax, ymax};
                }
                current_cell = nullptr;
                break;

            case gdsii::BOUNDARY:
                if (current_cell) {
                    parse_boundary(in, *current_cell);
                    result.polygons_read++;
                }
                break;

            case gdsii::PATH:
                if (current_cell) {
                    parse_path(in, *current_cell);
                    result.paths_read++;
                }
                break;

            case gdsii::SREF:
                if (current_cell) {
                    parse_sref(in, *current_cell);
                    result.refs_read++;
                }
                break;

            case gdsii::AREF:
                if (current_cell) {
                    parse_aref(in, *current_cell);
                    result.refs_read++;
                }
                break;

            case gdsii::TEXT:
                if (current_cell) {
                    parse_text(in, *current_cell);
                }
                break;

            case gdsii::ENDLIB:
                goto done;

            default:
                // Skip unknown records
                break;
        }
    }

done:
    auto t1 = std::chrono::steady_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.success = true;
    return result;
}

GdsReadResult GdsiiReader::read_buffer(const uint8_t* data, size_t size) {
    // Create an in-memory stream from the buffer
    std::string buf_str(reinterpret_cast<const char*>(data), size);
    std::istringstream iss(buf_str);

    // Temporarily redirect read() to use this stream
    // For simplicity, write to a temp file and read it
    // In production, refactor read() to accept istream&
    GdsReadResult result;
    result.error = "read_buffer: use read() with a file path";
    return result;
}

// ============================================================================
// Flatten hierarchical GDSII to PhysicalDesign
// ============================================================================

void GdsiiReader::flatten_cell(const GdsLibrary& lib, const GdsCell& cell,
                                double ox, double oy, double angle, double mag,
                                bool mirror, PhysicalDesign& pd,
                                int depth) const {
    if (depth > 100) return;  // prevent infinite recursion

    double cos_a = std::cos(angle * M_PI / 180.0);
    double sin_a = std::sin(angle * M_PI / 180.0);

    auto transform = [&](double lx, double ly) -> std::pair<double, double> {
        if (mirror) ly = -ly;
        double rx = lx * cos_a - ly * sin_a;
        double ry = lx * sin_a + ly * cos_a;
        return {ox + rx * mag, oy + ry * mag};
    };

    // Convert polygons to wire segments (approximation: use bbox edges)
    for (auto& poly : cell.polygons) {
        if (poly.points.size() < 3) continue;

        int layer = poly.layer;
        auto it = layer_map_.find(layer);
        if (it != layer_map_.end()) layer = it->second;

        // Create wire segments from polygon edges
        for (size_t i = 0; i + 1 < poly.points.size(); ++i) {
            auto [x0, y0] = transform(poly.points[i].first, poly.points[i].second);
            auto [x1, y1] = transform(poly.points[i + 1].first, poly.points[i + 1].second);

            WireSegment ws;
            ws.start = {x0, y0};
            ws.end = {x1, y1};
            ws.layer = layer;
            ws.width = 0.1;  // default width; polygon has explicit geometry
            ws.net_id = -1;
            pd.wires.push_back(ws);
        }
    }

    // Convert paths to wire segments directly
    for (auto& path : cell.paths) {
        int layer = path.layer;
        auto it = layer_map_.find(layer);
        if (it != layer_map_.end()) layer = it->second;

        for (size_t i = 0; i + 1 < path.points.size(); ++i) {
            auto [x0, y0] = transform(path.points[i].first, path.points[i].second);
            auto [x1, y1] = transform(path.points[i + 1].first, path.points[i + 1].second);

            WireSegment ws;
            ws.start = {x0, y0};
            ws.end = {x1, y1};
            ws.layer = layer;
            ws.width = path.width > 0 ? path.width * mag : 0.1;
            ws.net_id = -1;
            pd.wires.push_back(ws);
        }
    }

    // Recurse into SREFs
    for (auto& ref : cell.refs) {
        auto* sub = lib.find_cell(ref.cell_name);
        if (!sub) continue;

        auto [rx, ry] = transform(ref.x, ref.y);
        double new_angle = angle + ref.angle;
        double new_mag = mag * ref.mag;
        bool new_mirror = mirror ^ ref.mirror_x;
        flatten_cell(lib, *sub, rx, ry, new_angle, new_mag, new_mirror, pd, depth + 1);
    }

    // Recurse into AREFs
    for (auto& aref : cell.arefs) {
        auto* sub = lib.find_cell(aref.cell_name);
        if (!sub) continue;

        for (int r = 0; r < aref.rows; ++r) {
            for (int c = 0; c < aref.cols; ++c) {
                double lx = aref.x + c * aref.col_pitch;
                double ly = aref.y + r * aref.row_pitch;
                auto [rx, ry] = transform(lx, ly);
                double new_angle = angle + aref.angle;
                double new_mag = mag * aref.mag;
                bool new_mirror = mirror ^ aref.mirror_x;
                flatten_cell(lib, *sub, rx, ry, new_angle, new_mag, new_mirror,
                            pd, depth + 1);
            }
        }
    }
}

PhysicalDesign GdsiiReader::flatten(const GdsLibrary& lib,
                                      const std::string& top_cell) const {
    PhysicalDesign pd;

    // Find top cell
    const GdsCell* top = nullptr;
    if (!top_cell.empty()) {
        top = lib.find_cell(top_cell);
    } else if (!lib.cells.empty()) {
        // Use the last cell as top (common GDSII convention)
        top = &lib.cells.back();
    }

    if (!top) return pd;

    // Set die area from top cell bbox
    pd.die_area = top->bbox;

    // Recursively flatten
    flatten_cell(lib, *top, 0, 0, 0, 1.0, false, pd);

    return pd;
}

} // namespace sf
