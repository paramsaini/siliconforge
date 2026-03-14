// SiliconForge — DEF Parser Implementation
#include "core/def_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

namespace sf {

std::vector<std::string> DefParser::tokenize(const std::string& content) {
    std::vector<std::string> tokens;
    std::istringstream ss(content);
    std::string tok;
    while (ss >> tok) {
        // Skip comments
        if (tok.size() >= 1 && tok[0] == '#') {
            std::string line;
            std::getline(ss, line);
            continue;
        }
        // Split around semicolons and parens
        std::string cur;
        for (char c : tok) {
            if (c == ';' || c == '(' || c == ')') {
                if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
                tokens.push_back(std::string(1, c));
            } else {
                cur += c;
            }
        }
        if (!cur.empty()) tokens.push_back(cur);
    }
    return tokens;
}

bool DefParser::parse_tokens(const std::vector<std::string>& t, PhysicalDesign& pd) {
    size_t pos = 0;
    double dbu = pd.dbu_per_micron;

    // Helper: parse coordinate with DBU conversion
    auto to_um = [&](const std::string& s) -> double {
        try { return std::stod(s) / dbu; } catch (...) { return 0; }
    };

    while (pos < t.size()) {
        if (t[pos] == "UNITS" && pos + 3 < t.size() && t[pos+1] == "DISTANCE" && t[pos+2] == "MICRONS") {
            pos += 3;
            if (pos < t.size()) {
                try { dbu = std::stod(t[pos]); pd.dbu_per_micron = dbu; } catch (...) {}
                pos++;
            }
            if (pos < t.size() && t[pos] == ";") pos++;
        }
        else if (t[pos] == "DIEAREA") {
            pos++;
            // Collect all points: ( x y ) ( x y ) ... ;
            std::vector<Point> pts;
            while (pos < t.size() && t[pos] != ";") {
                if (t[pos] == "(") {
                    pos++;
                    double px_val = 0, py_val = 0;
                    if (pos < t.size()) { px_val = to_um(t[pos]); pos++; }
                    if (pos < t.size()) { py_val = to_um(t[pos]); pos++; }
                    if (pos < t.size() && t[pos] == ")") pos++;
                    pts.push_back(Point(px_val, py_val));
                } else {
                    pos++;
                }
            }
            if (pos < t.size() && t[pos] == ";") pos++;
            if (pts.size() == 2) {
                // Rectangle: two corners
                pd.die_area = Rect(pts[0].x, pts[0].y, pts[1].x, pts[1].y);
            } else if (pts.size() > 2) {
                // Polygon die area
                pd.die_area_polygon = pts;
                // Also compute bounding box for die_area
                double minx = pts[0].x, miny = pts[0].y;
                double maxx = pts[0].x, maxy = pts[0].y;
                for (auto& p : pts) {
                    if (p.x < minx) minx = p.x;
                    if (p.y < miny) miny = p.y;
                    if (p.x > maxx) maxx = p.x;
                    if (p.y > maxy) maxy = p.y;
                }
                pd.die_area = Rect(minx, miny, maxx, maxy);
            }
        }
        else if (t[pos] == "TRACKS") {
            pos++;
            // TRACKS X|Y start DO count STEP step LAYER name ;
            TrackDef td;
            if (pos < t.size()) { td.direction = t[pos]; pos++; }
            if (pos < t.size()) { td.start = to_um(t[pos]); pos++; }
            if (pos < t.size() && t[pos] == "DO") pos++;
            if (pos < t.size()) { try { td.count = std::stoi(t[pos]); } catch(...) {} pos++; }
            if (pos < t.size() && t[pos] == "STEP") pos++;
            if (pos < t.size()) { td.step = to_um(t[pos]); pos++; }
            while (pos < t.size() && t[pos] != ";") {
                if (t[pos] == "LAYER") { pos++; if (pos < t.size()) { td.layer_names.push_back(t[pos]); pos++; } }
                else pos++;
            }
            if (pos < t.size() && t[pos] == ";") pos++;
            pd.tracks.push_back(td);
        }
        else if (t[pos] == "ROW") {
            pos++;
            // ROW rowName siteName origX origY siteOrient
            //   DO numX BY numY STEP stepX stepY ;
            PlacementRow row;
            if (pos < t.size()) { row.name = t[pos]; pos++; }
            if (pos < t.size()) { row.site_name = t[pos]; pos++; }
            if (pos < t.size()) { row.origin_x = to_um(t[pos]); pos++; }
            if (pos < t.size()) { row.origin_y = to_um(t[pos]); pos++; }
            if (pos < t.size()) { row.orient = t[pos]; pos++; }
            if (pos < t.size() && t[pos] == "DO") {
                pos++;
                if (pos < t.size()) { try { row.num_x = std::stoi(t[pos]); } catch (...) {} pos++; }
                if (pos < t.size() && t[pos] == "BY") pos++;
                if (pos < t.size()) { try { row.num_y = std::stoi(t[pos]); } catch (...) {} pos++; }
                if (pos < t.size() && t[pos] == "STEP") pos++;
                if (pos < t.size()) { row.step_x = to_um(t[pos]); pos++; }
                if (pos < t.size()) { row.step_y = to_um(t[pos]); pos++; }
            }
            if (pos < t.size() && t[pos] == ";") pos++;
            pd.placement_rows.push_back(row);
        }
        else if (t[pos] == "VIAS") {
            pos++;
            if (pos < t.size()) pos++; // count
            if (pos < t.size() && t[pos] == ";") pos++;
            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    ViaDef vd;
                    if (pos < t.size()) { vd.name = t[pos]; pos++; }
                    while (pos < t.size() && t[pos] != ";") {
                        if (t[pos] == "+" && pos + 1 < t.size() && t[pos+1] == "RECT") {
                            pos += 2;
                            ViaDef::ViaLayer vl;
                            if (pos < t.size()) { vl.layer_name = t[pos]; pos++; }
                            if (pos < t.size() && t[pos] == "(") pos++;
                            double rx0=0, ry0=0, rx1=0, ry1=0;
                            if (pos < t.size()) { rx0 = to_um(t[pos]); pos++; }
                            if (pos < t.size()) { ry0 = to_um(t[pos]); pos++; }
                            if (pos < t.size() && t[pos] == ")") pos++;
                            if (pos < t.size() && t[pos] == "(") pos++;
                            if (pos < t.size()) { rx1 = to_um(t[pos]); pos++; }
                            if (pos < t.size()) { ry1 = to_um(t[pos]); pos++; }
                            if (pos < t.size() && t[pos] == ")") pos++;
                            vl.rect = Rect(rx0, ry0, rx1, ry1);
                            vd.layers.push_back(vl);
                        } else pos++;
                    }
                    if (pos < t.size() && t[pos] == ";") pos++;
                    pd.via_defs.push_back(vd);
                } else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; if (pos < t.size()) pos++; }
        }
        else if (t[pos] == "PINS") {
            pos++;
            if (pos < t.size()) pos++; // count
            if (pos < t.size() && t[pos] == ";") pos++;
            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    IoPin pin;
                    if (pos < t.size()) { pin.name = t[pos]; pos++; }
                    while (pos < t.size() && t[pos] != ";") {
                        if (t[pos] == "+" && pos + 1 < t.size()) {
                            if (t[pos+1] == "NET") { pos += 2; if (pos < t.size()) { pin.net_name = t[pos]; pos++; } }
                            else if (t[pos+1] == "DIRECTION") { pos += 2; if (pos < t.size()) { pin.direction = t[pos]; pos++; } }
                            else if (t[pos+1] == "LAYER") {
                                pos += 2;
                                if (pos < t.size()) pos++; // layer name
                                // skip layer rect
                                while (pos < t.size() && t[pos] != "+" && t[pos] != ";") pos++;
                            }
                            else if (t[pos+1] == "PLACED" || t[pos+1] == "FIXED" || t[pos+1] == "COVER") {
                                pos += 2;
                                if (pos < t.size() && t[pos] == "(") pos++;
                                if (pos < t.size()) { pin.position.x = to_um(t[pos]); pos++; }
                                if (pos < t.size()) { pin.position.y = to_um(t[pos]); pos++; }
                                if (pos < t.size() && t[pos] == ")") pos++;
                                if (pos < t.size()) pos++; // orientation
                                pin.placed = true;
                            }
                            else { pos += 2; }
                        } else pos++;
                    }
                    if (pos < t.size() && t[pos] == ";") pos++;
                    pd.io_pins.push_back(pin);
                } else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; if (pos < t.size()) pos++; }
        }
        else if (t[pos] == "BLOCKAGES") {
            pos++;
            if (pos < t.size()) pos++; // count
            if (pos < t.size() && t[pos] == ";") pos++;
            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    RoutingBlockage blk;
                    while (pos < t.size() && t[pos] != ";") {
                        if (t[pos] == "LAYER") { pos++; if (pos < t.size()) { blk.type = "ROUTING"; pos++; } }
                        else if (t[pos] == "PLACEMENT") { blk.type = "PLACEMENT"; pos++; }
                        else if (t[pos] == "RECT") {
                            pos++;
                            if (pos < t.size() && t[pos] == "(") pos++;
                            double bx0=0, by0=0, bx1=0, by1=0;
                            if (pos < t.size()) { bx0 = to_um(t[pos]); pos++; }
                            if (pos < t.size()) { by0 = to_um(t[pos]); pos++; }
                            if (pos < t.size() && t[pos] == ")") pos++;
                            if (pos < t.size() && t[pos] == "(") pos++;
                            if (pos < t.size()) { bx1 = to_um(t[pos]); pos++; }
                            if (pos < t.size()) { by1 = to_um(t[pos]); pos++; }
                            if (pos < t.size() && t[pos] == ")") pos++;
                            blk.rect = Rect(bx0, by0, bx1, by1);
                        }
                        else pos++;
                    }
                    if (pos < t.size() && t[pos] == ";") pos++;
                    pd.blockages.push_back(blk);
                } else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; if (pos < t.size()) pos++; }
        }
        else if (t[pos] == "SPECIALNETS") {
            pos++;
            if (pos < t.size()) pos++; // count
            if (pos < t.size() && t[pos] == ";") pos++;
            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    SpecialNet sn;
                    if (pos < t.size()) { sn.name = t[pos]; pos++; }
                    while (pos < t.size() && t[pos] != ";") {
                        if (t[pos] == "+" && pos + 1 < t.size() && t[pos+1] == "USE") {
                            pos += 2;
                            if (pos < t.size()) { sn.use = t[pos]; pos++; }
                        } else if (t[pos] == "+" && pos + 1 < t.size() && t[pos+1] == "ROUTED") {
                            pos += 2;
                            // ROUTED layer width ( x y ) ( x y ) ...
                            while (pos < t.size() && t[pos] != ";" && t[pos] != "+") {
                                SpecialNet::SpecialWire sw;
                                if (pos < t.size() && t[pos] != "(" && t[pos] != "NEW") {
                                    sw.layer_name = t[pos]; pos++;
                                }
                                if (pos < t.size() && t[pos] != "(" && t[pos] != ";") {
                                    try { sw.width = to_um(t[pos]); } catch (...) {}
                                    pos++;
                                }
                                if (pos < t.size() && t[pos] == "(") {
                                    pos++;
                                    if (pos < t.size()) { sw.start.x = to_um(t[pos]); pos++; }
                                    if (pos < t.size()) { sw.start.y = to_um(t[pos]); pos++; }
                                    if (pos < t.size() && t[pos] == ")") pos++;
                                }
                                if (pos < t.size() && t[pos] == "(") {
                                    pos++;
                                    if (pos < t.size()) { sw.end.x = to_um(t[pos]); pos++; }
                                    if (pos < t.size()) { sw.end.y = to_um(t[pos]); pos++; }
                                    if (pos < t.size() && t[pos] == ")") pos++;
                                }
                                sn.wires.push_back(sw);
                                if (pos < t.size() && t[pos] == "NEW") pos++;
                            }
                        } else pos++;
                    }
                    if (pos < t.size() && t[pos] == ";") pos++;
                    pd.special_nets.push_back(sn);
                } else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; if (pos < t.size()) pos++; }
        }
        else if (t[pos] == "REGIONS") {
            pos++;
            if (pos < t.size()) pos++; // count
            if (pos < t.size() && t[pos] == ";") pos++;
            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    Region rg;
                    if (pos < t.size()) { rg.name = t[pos]; pos++; }
                    while (pos < t.size() && t[pos] != ";") {
                        if (t[pos] == "(") {
                            pos++;
                            Rect r;
                            if (pos < t.size()) { r.x0 = to_um(t[pos]); pos++; }
                            if (pos < t.size()) { r.y0 = to_um(t[pos]); pos++; }
                            if (pos < t.size() && t[pos] == ")") pos++;
                            if (pos < t.size() && t[pos] == "(") pos++;
                            if (pos < t.size()) { r.x1 = to_um(t[pos]); pos++; }
                            if (pos < t.size()) { r.y1 = to_um(t[pos]); pos++; }
                            if (pos < t.size() && t[pos] == ")") pos++;
                            rg.rects.push_back(r);
                        }
                        else if (t[pos] == "+" && pos + 1 < t.size() && t[pos+1] == "TYPE") {
                            pos += 2;
                            if (pos < t.size()) { rg.type = t[pos]; pos++; }
                        }
                        else pos++;
                    }
                    if (pos < t.size() && t[pos] == ";") pos++;
                    pd.regions.push_back(rg);
                } else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; if (pos < t.size()) pos++; }
        }
        else if (t[pos] == "GROUPS") {
            pos++;
            if (pos < t.size()) pos++; // count
            if (pos < t.size() && t[pos] == ";") pos++;
            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    Group grp;
                    if (pos < t.size()) { grp.name = t[pos]; pos++; }
                    while (pos < t.size() && t[pos] != ";" && t[pos] != "+") {
                        grp.members.push_back(t[pos]); pos++;
                    }
                    while (pos < t.size() && t[pos] != ";") {
                        if (t[pos] == "+" && pos + 1 < t.size() && t[pos+1] == "REGION") {
                            pos += 2;
                            if (pos < t.size()) { grp.region_name = t[pos]; pos++; }
                        } else pos++;
                    }
                    if (pos < t.size() && t[pos] == ";") pos++;
                    pd.groups.push_back(grp);
                } else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; if (pos < t.size()) pos++; }
        }
        else if (t[pos] == "COMPONENTS") {
            pos++; // skip count
            if (pos < t.size()) pos++;
            if (pos < t.size() && t[pos] == ";") pos++;

            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    std::string name, type;
                    if (pos < t.size()) { name = t[pos]; pos++; }
                    if (pos < t.size()) { type = t[pos]; pos++; }

                    double px = 0, py = 0;
                    std::string orient = "N";
                    bool is_fixed = false;
                    while (pos < t.size() && t[pos] != ";") {
                        if (t[pos] == "PLACED" || t[pos] == "FIXED") {
                            is_fixed = (t[pos] == "FIXED");
                            pos++;
                            if (pos < t.size() && t[pos] == "(") pos++;
                            if (pos < t.size()) { px = to_um(t[pos]); pos++; }
                            if (pos < t.size()) { py = to_um(t[pos]); pos++; }
                            if (pos < t.size() && t[pos] == ")") pos++;
                            if (pos < t.size()) { orient = t[pos]; pos++; }
                        }
                        else pos++;
                    }
                    if (pos < t.size() && t[pos] == ";") pos++;

                    int cid = pd.add_cell(name, type, 3.0, pd.row_height);
                    pd.cells[cid].position = {px, py};
                    pd.cells[cid].placed = true;
                    // Map DEF orientation string to int
                    if (orient == "N") pd.cells[cid].orientation = 0;
                    else if (orient == "S") pd.cells[cid].orientation = 1;
                    else if (orient == "W") pd.cells[cid].orientation = 2;
                    else if (orient == "E") pd.cells[cid].orientation = 3;
                    else if (orient == "FN") pd.cells[cid].orientation = 4;
                    else if (orient == "FS") pd.cells[cid].orientation = 5;
                    else if (orient == "FW") pd.cells[cid].orientation = 6;
                    else if (orient == "FE") pd.cells[cid].orientation = 7;
                    (void)is_fixed;
                }
                else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; pos++; } // END COMPONENTS
        }
        else if (t[pos] == "NETS") {
            pos++;
            if (pos < t.size()) pos++;
            if (pos < t.size() && t[pos] == ";") pos++;

            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    std::string net_name;
                    if (pos < t.size()) { net_name = t[pos]; pos++; }

                    std::vector<int> cell_ids;
                    while (pos < t.size() && t[pos] != ";" && t[pos] != "+") {
                        if (t[pos] == "(") {
                            pos++;
                            if (pos < t.size()) {
                                std::string cname = t[pos]; pos++;
                                // Use cell_name_map for O(1) lookup
                                auto it = pd.cell_name_map.find(cname);
                                if (it != pd.cell_name_map.end()) {
                                    cell_ids.push_back(it->second);
                                } else if (cname == "PIN") {
                                    // IO pin reference — skip pin name
                                    if (pos < t.size()) pos++;
                                }
                            }
                            while (pos < t.size() && t[pos] != ")") pos++;
                            if (pos < t.size()) pos++; // skip )
                        }
                        else pos++;
                    }
                    // Skip net properties
                    while (pos < t.size() && t[pos] != ";") pos++;
                    if (pos < t.size() && t[pos] == ";") pos++;
                    if (cell_ids.size() >= 2)
                        pd.add_net(net_name, cell_ids);
                }
                else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; pos++; }
        }
        else if (t[pos] == "FILLS") {
            pos++;
            if (pos < t.size()) pos++; // count
            if (pos < t.size() && t[pos] == ";") pos++;
            while (pos < t.size() && t[pos] != "END") {
                if (t[pos] == "-") {
                    pos++;
                    FillShape fs;
                    // LAYER layerName
                    if (pos < t.size() && t[pos] == "LAYER") {
                        pos++;
                        if (pos < t.size()) { fs.layer = t[pos]; pos++; }
                    }
                    // Parse rectangles until ';'
                    while (pos < t.size() && t[pos] != ";") {
                        if (t[pos] == "RECT") {
                            pos++;
                            if (pos < t.size() && t[pos] == "(") pos++;
                            if (pos < t.size()) { fs.x0 = to_um(t[pos]); pos++; }
                            if (pos < t.size()) { fs.y0 = to_um(t[pos]); pos++; }
                            if (pos < t.size() && t[pos] == ")") pos++;
                            if (pos < t.size() && t[pos] == "(") pos++;
                            if (pos < t.size()) { fs.x1 = to_um(t[pos]); pos++; }
                            if (pos < t.size()) { fs.y1 = to_um(t[pos]); pos++; }
                            if (pos < t.size() && t[pos] == ")") pos++;
                            pd.fills.push_back(fs);
                        }
                        else if (t[pos] == "+" && pos + 1 < t.size() && t[pos+1] == "LAYER") {
                            pos += 2;
                            if (pos < t.size()) { fs.layer = t[pos]; pos++; }
                        }
                        else pos++;
                    }
                    if (pos < t.size() && t[pos] == ";") pos++;
                } else pos++;
            }
            if (pos < t.size() && t[pos] == "END") { pos++; if (pos < t.size()) pos++; }
        }
        else pos++;
    }
    return true;
}

bool DefParser::parse_string(const std::string& content, PhysicalDesign& pd) {
    auto tokens = tokenize(content);
    return parse_tokens(tokens, pd);
}

bool DefParser::parse_file(const std::string& filename, PhysicalDesign& pd) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return parse_string(content, pd);
}

std::string DefParser::export_def(const PhysicalDesign& pd) {
    std::ostringstream def;
    def << "VERSION 5.8 ;\n"
        << "DESIGN SiliconForge ;\n"
        << "UNITS DISTANCE MICRONS 1000 ;\n"
        << "DIEAREA ( " << (int)(pd.die_area.x0*1000) << " "
        << (int)(pd.die_area.y0*1000) << " ) ( "
        << (int)(pd.die_area.x1*1000) << " "
        << (int)(pd.die_area.y1*1000) << " ) ;\n\n";

    if (!pd.placement_rows.empty()) {
        for (auto& r : pd.placement_rows) {
            def << "ROW " << r.name << " " << r.site_name
                << " " << (int)(r.origin_x*1000) << " " << (int)(r.origin_y*1000)
                << " " << r.orient
                << " DO " << r.num_x << " BY " << r.num_y
                << " STEP " << (int)(r.step_x*1000) << " " << (int)(r.step_y*1000) << " ;\n";
        }
        def << "\n";
    }

    def << "COMPONENTS " << pd.cells.size() << " ;\n";
    for (auto& c : pd.cells) {
        def << "  - " << c.name << " " << c.cell_type
            << " + PLACED ( " << (int)(c.position.x*1000) << " "
            << (int)(c.position.y*1000) << " ) N ;\n";
    }
    def << "END COMPONENTS\n\n";

    def << "NETS " << pd.nets.size() << " ;\n";
    for (auto& n : pd.nets) {
        def << "  - " << n.name;
        for (auto cid : n.cell_ids)
            def << " ( " << pd.cells[cid].name << " Z )";
        def << " ;\n";
    }
    def << "END NETS\n\n";
    def << "END DESIGN\n";
    return def.str();
}

} // namespace sf
