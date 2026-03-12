// SiliconForge — LEF 5.8 Parser Implementation
#include "core/lef_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace sf {

// ── LefLibrary helpers ───────────────────────────────────────────────────────

const LefMacro* LefLibrary::find_macro(const std::string& name) const {
    for (auto& m : macros) if (m.name == name) return &m;
    return nullptr;
}

const LefLayer* LefLibrary::find_layer(const std::string& name) const {
    for (auto& l : layers) if (l.name == name) return &l;
    return nullptr;
}

const LefVia* LefLibrary::find_via(const std::string& name) const {
    for (auto& v : vias) if (v.name == name) return &v;
    return nullptr;
}

const LefSite* LefLibrary::find_site(const std::string& name) const {
    for (auto& s : sites) if (s.name == name) return &s;
    return nullptr;
}

int LefLibrary::num_routing_layers() const {
    int n = 0;
    for (auto& l : layers) if (l.type == LefLayer::ROUTING) ++n;
    return n;
}

int LefLibrary::num_cut_layers() const {
    int n = 0;
    for (auto& l : layers) if (l.type == LefLayer::CUT) ++n;
    return n;
}

// ── Tokenizer ────────────────────────────────────────────────────────────────

std::vector<LefParser::Token> LefParser::tokenize(const std::string& content) {
    std::vector<Token> toks;
    std::istringstream stream(content);
    std::string line;
    int line_num = 0;

    while (std::getline(stream, line)) {
        ++line_num;

        // Strip comment (# to end of line)
        auto hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);

        size_t i = 0;
        while (i < line.size()) {
            // Skip whitespace
            if (std::isspace(static_cast<unsigned char>(line[i]))) { ++i; continue; }

            // Quoted string — keep as single token (without quotes)
            if (line[i] == '"') {
                ++i;
                std::string qs;
                while (i < line.size() && line[i] != '"') qs += line[i++];
                if (i < line.size()) ++i; // skip closing quote
                toks.push_back({qs, line_num});
                continue;
            }

            // Semicolons are standalone tokens
            if (line[i] == ';') {
                toks.push_back({";", line_num});
                ++i;
                continue;
            }

            // Regular token
            std::string tok;
            while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i]))
                   && line[i] != ';' && line[i] != '"') {
                tok += line[i++];
            }
            if (!tok.empty()) toks.push_back({tok, line_num});
        }
    }

    return toks;
}

// ── Token helpers ────────────────────────────────────────────────────────────

static const std::string k_eof = "<EOF>";

const std::string& LefParser::peek() const {
    if (pos_ >= tokens_.size()) return k_eof;
    return tokens_[pos_].value;
}

const std::string& LefParser::advance() {
    if (pos_ >= tokens_.size()) error("unexpected end of input");
    return tokens_[pos_++].value;
}

bool LefParser::match(const std::string& expected) {
    if (peek() == expected) { ++pos_; return true; }
    return false;
}

void LefParser::expect(const std::string& expected) {
    if (!match(expected))
        error("expected '" + expected + "', got '" + peek() + "'");
}

double LefParser::next_double() {
    const auto& v = advance();
    try { return std::stod(v); }
    catch (...) { error("expected number, got '" + v + "'"); }
}

int LefParser::next_int() {
    const auto& v = advance();
    try { return std::stoi(v); }
    catch (...) { error("expected integer, got '" + v + "'"); }
}

void LefParser::skip_to_semicolon() {
    while (pos_ < tokens_.size() && tokens_[pos_].value != ";") ++pos_;
    if (pos_ < tokens_.size()) ++pos_;
}

void LefParser::skip_past_end(const std::string& block_name) {
    // Skip until "END <block_name>" or just "END"
    while (pos_ < tokens_.size()) {
        if (tokens_[pos_].value == "END") {
            ++pos_;
            if (pos_ < tokens_.size() && tokens_[pos_].value == block_name) {
                ++pos_;
                return;
            }
            // It was just END (no matching name); accept it
            return;
        }
        ++pos_;
    }
}

[[noreturn]] void LefParser::error(const std::string& msg) const {
    int ln = (pos_ < tokens_.size()) ? tokens_[pos_].line : -1;
    throw std::runtime_error("LEF parse error (line " + std::to_string(ln) + "): " + msg);
}

// ── Public API ───────────────────────────────────────────────────────────────

LefLibrary LefParser::parse_file(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open())
        throw std::runtime_error("LEF: cannot open file '" + filename + "'");
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    return parse_string(content);
}

LefLibrary LefParser::parse_string(const std::string& content) {
    tokens_ = tokenize(content);
    pos_    = 0;

    LefLibrary lib;

    while (pos_ < tokens_.size()) {
        const auto& tok = peek();

        if (tok == "VERSION") {
            advance();
            lib.version = advance();
            match(";");
        }
        else if (tok == "BUSBITCHARS") {
            advance();
            lib.bus_bit_chars = advance();
            match(";");
        }
        else if (tok == "DIVIDERCHAR") {
            advance();
            lib.divider_char = advance();
            match(";");
        }
        else if (tok == "UNITS") {
            advance();
            parse_units(lib);
        }
        else if (tok == "LAYER") {
            advance();
            parse_layer(lib);
        }
        else if (tok == "VIA") {
            advance();
            parse_via(lib);
        }
        else if (tok == "VIARULE") {
            advance();
            parse_via_rule(lib);
        }
        else if (tok == "SITE") {
            advance();
            parse_site(lib);
        }
        else if (tok == "MACRO") {
            advance();
            parse_macro(lib);
        }
        else if (tok == "SPACING") {
            advance();
            parse_spacing(lib);
        }
        else if (tok == "PROPERTYDEFINITIONS") {
            advance();
            parse_property(lib);
        }
        else if (tok == "END") {
            // END LIBRARY or stray END — consume and continue
            advance();
            if (pos_ < tokens_.size() && peek() == "LIBRARY") advance();
        }
        else if (tok == "NOWIREEXTENSIONATPIN" || tok == "MANUFACTURINGGRID"
              || tok == "USEMINSPACING" || tok == "CLEARANCEMEASURE"
              || tok == "NAMESCASESENSITIVE" || tok == "FIXEDMASK") {
            // Known top-level statements — skip to semicolon
            advance();
            skip_to_semicolon();
        }
        else {
            // Unknown token — skip
            advance();
        }
    }

    return lib;
}

// ── UNITS ────────────────────────────────────────────────────────────────────

void LefParser::parse_units(LefLibrary& lib) {
    while (pos_ < tokens_.size()) {
        const auto& tok = peek();
        if (tok == "END") {
            advance(); // END
            if (pos_ < tokens_.size() && peek() == "UNITS") advance();
            return;
        }

        if (tok == "DATABASE") {
            advance();
            expect("MICRONS");
            lib.units.database_microns = next_double();
            match(";");
        }
        else if (tok == "CAPACITANCE") {
            advance();
            expect("PICOFARADS");
            lib.units.capacitance_pf = next_double();
            match(";");
        }
        else if (tok == "RESISTANCE") {
            advance();
            expect("KOHMS");
            lib.units.resistance_kohm = next_double();
            match(";");
        }
        else if (tok == "TIME") {
            advance();
            expect("NANOSECONDS");
            lib.units.time_ns = next_double();
            match(";");
        }
        else if (tok == "POWER") {
            advance();
            expect("MILLIWATTS");
            lib.units.power_mw = next_double();
            match(";");
        }
        else {
            advance();
            skip_to_semicolon();
        }
    }
}

// ── LAYER ────────────────────────────────────────────────────────────────────

void LefParser::parse_layer(LefLibrary& lib) {
    LefLayer layer;
    layer.name = advance();

    while (pos_ < tokens_.size()) {
        const auto& tok = peek();

        if (tok == "END") {
            advance(); // END
            if (pos_ < tokens_.size() && peek() == layer.name) advance();
            break;
        }

        if (tok == "TYPE") {
            advance();
            const auto& t = advance();
            if      (t == "ROUTING")     layer.type = LefLayer::ROUTING;
            else if (t == "CUT")         layer.type = LefLayer::CUT;
            else if (t == "MASTERSLICE") layer.type = LefLayer::MASTERSLICE;
            else if (t == "OVERLAP")     layer.type = LefLayer::OVERLAP;
            else if (t == "IMPLANT")     layer.type = LefLayer::IMPLANT;
            match(";");
        }
        else if (tok == "DIRECTION") {
            advance();
            const auto& d = advance();
            if      (d == "HORIZONTAL") layer.direction = LefLayer::HORIZONTAL;
            else if (d == "VERTICAL")   layer.direction = LefLayer::VERTICAL;
            match(";");
        }
        else if (tok == "PITCH") {
            advance();
            layer.pitch = next_double();
            match(";");
        }
        else if (tok == "OFFSET") {
            advance();
            layer.offset = next_double();
            match(";");
        }
        else if (tok == "WIDTH") {
            advance();
            layer.width = next_double();
            match(";");
        }
        else if (tok == "MINWIDTH") {
            advance();
            layer.min_width = next_double();
            match(";");
        }
        else if (tok == "MAXWIDTH") {
            advance();
            layer.max_width = next_double();
            match(";");
        }
        else if (tok == "SPACING") {
            advance();
            double sp = next_double();
            layer.spacing = sp;
            // Possible RANGE or other qualifiers before semicolon
            skip_to_semicolon();
        }
        else if (tok == "RESISTANCE") {
            advance();
            if (peek() == "RPERSQ") {
                advance();
                layer.resistance_per_sq = next_double();
            } else {
                next_double(); // plain value
            }
            match(";");
        }
        else if (tok == "CAPACITANCE") {
            advance();
            if (peek() == "CPERSQDIST") {
                advance();
                layer.capacitance_per_um = next_double();
            } else {
                layer.capacitance_per_um = next_double();
            }
            match(";");
        }
        else if (tok == "EDGECAPACITANCE") {
            advance();
            layer.edge_capacitance = next_double();
            match(";");
        }
        else if (tok == "THICKNESS") {
            advance();
            layer.thickness = next_double();
            match(";");
        }
        else if (tok == "HEIGHT") {
            advance();
            layer.height = next_double();
            match(";");
        }
        else if (tok == "AREA") {
            advance();
            layer.area = next_double();
            match(";");
        }
        else if (tok == "SPACINGTABLE") {
            advance();
            // SPACINGTABLE PARALLELRUNLENGTH ... or INFLUENCE ...
            // Skip to semicolon but try to grab width/spacing pairs
            skip_to_semicolon();
        }
        else {
            // MINIMUMDENSITY, MAXIMUMDENSITY, ANTENNAMODEL, etc. — skip
            advance();
            skip_to_semicolon();
        }
    }

    lib.layers.push_back(std::move(layer));
}

// ── VIA ──────────────────────────────────────────────────────────────────────

void LefParser::parse_via(LefLibrary& lib) {
    LefVia via;
    via.name = advance();

    // Optional DEFAULT keyword
    if (peek() != ";" && peek() != "LAYER" && peek() != "RESISTANCE"
        && peek() != "END") {
        advance(); // e.g. DEFAULT
    }

    while (pos_ < tokens_.size()) {
        const auto& tok = peek();

        if (tok == "END") {
            advance();
            if (pos_ < tokens_.size() && peek() == via.name) advance();
            break;
        }

        if (tok == "RESISTANCE") {
            advance();
            via.resistance = next_double();
            match(";");
        }
        else if (tok == "LAYER") {
            advance();
            LefVia::ViaLayer vl;
            vl.layer_name = advance();
            match(";");

            while (pos_ < tokens_.size() && peek() == "RECT") {
                advance();
                double x0 = next_double();
                double y0 = next_double();
                double x1 = next_double();
                double y1 = next_double();
                vl.rects.push_back({x0, y0, x1, y1});
                match(";");
            }

            via.layers.push_back(std::move(vl));
        }
        else {
            advance();
            skip_to_semicolon();
        }
    }

    lib.vias.push_back(std::move(via));
}

// ── VIARULE ──────────────────────────────────────────────────────────────────

void LefParser::parse_via_rule(LefLibrary& lib) {
    LefViaRule rule;
    rule.name = advance();

    if (peek() == "GENERATE") {
        advance();
        rule.is_generate = true;
    }

    while (pos_ < tokens_.size()) {
        const auto& tok = peek();

        if (tok == "END") {
            advance();
            if (pos_ < tokens_.size() && peek() == rule.name) advance();
            break;
        }

        if (tok == "LAYER") {
            advance();
            LefViaRule::RuleLayer rl;
            rl.layer_name = advance();
            match(";");

            // Parse optional sub-properties before next LAYER or END
            while (pos_ < tokens_.size()) {
                const auto& sub = peek();
                if (sub == "LAYER" || sub == "END") break;

                if (sub == "DIRECTION") {
                    advance(); advance(); match(";");
                }
                else if (sub == "WIDTH") {
                    advance();
                    rl.width_lo = next_double();
                    if (peek() == "TO") { advance(); rl.width_hi = next_double(); }
                    match(";");
                }
                else if (sub == "OVERHANG") {
                    advance();
                    rl.overhang1 = next_double();
                    match(";");
                }
                else if (sub == "METALOVERHANG") {
                    advance();
                    rl.overhang2 = next_double();
                    match(";");
                }
                else if (sub == "ENCLOSURE") {
                    advance();
                    rl.enc_x = next_double();
                    rl.enc_y = next_double();
                    match(";");
                }
                else if (sub == "SPACING") {
                    advance();
                    rl.spacing_x = next_double();
                    if (peek() == "BY") { advance(); rl.spacing_y = next_double(); }
                    else rl.spacing_y = rl.spacing_x;
                    match(";");
                }
                else if (sub == "RECT") {
                    // Cut layer rect definition — skip
                    advance(); skip_to_semicolon();
                }
                else {
                    advance(); skip_to_semicolon();
                }
            }

            rule.layers.push_back(std::move(rl));
        }
        else {
            advance();
            skip_to_semicolon();
        }
    }

    lib.via_rules.push_back(std::move(rule));
}

// ── SITE ─────────────────────────────────────────────────────────────────────

void LefParser::parse_site(LefLibrary& lib) {
    LefSite site;
    site.name = advance();

    while (pos_ < tokens_.size()) {
        const auto& tok = peek();

        if (tok == "END") {
            advance();
            if (pos_ < tokens_.size() && peek() == site.name) advance();
            break;
        }

        if (tok == "CLASS") {
            advance();
            const auto& c = advance();
            if      (c == "CORE") site.site_class = LefSite::CORE;
            else if (c == "PAD")  site.site_class = LefSite::PAD;
            else if (c == "IO")   site.site_class = LefSite::IO;
            match(";");
        }
        else if (tok == "SYMMETRY") {
            advance();
            const auto& s = advance();
            if      (s == "X")   site.symmetry = LefSite::X;
            else if (s == "Y")   site.symmetry = LefSite::Y;
            else if (s == "R90") site.symmetry = LefSite::R90;
            // Additional symmetry tokens before semicolon are consumed below
            skip_to_semicolon();
        }
        else if (tok == "SIZE") {
            advance();
            site.width = next_double();
            expect("BY");
            site.height = next_double();
            match(";");
        }
        else {
            advance();
            skip_to_semicolon();
        }
    }

    lib.sites.push_back(std::move(site));
}

// ── MACRO ────────────────────────────────────────────────────────────────────

static LefMacro::Class parse_macro_class_str(const std::string& a,
                                              const std::string& b) {
    if (a == "CORE") {
        if (b == "TIEHIGH")      return LefMacro::CORE_TIEHIGH;
        if (b == "TIELOW")       return LefMacro::CORE_TIELOW;
        if (b == "ANTENNACELL")  return LefMacro::CORE_ANTENNACELL;
        if (b == "WELLTAP")      return LefMacro::CORE_WELLTAP;
        return LefMacro::CORE;
    }
    if (a == "PAD") {
        if (b == "INPUT")   return LefMacro::PAD_INPUT;
        if (b == "OUTPUT")  return LefMacro::PAD_OUTPUT;
        if (b == "INOUT")   return LefMacro::PAD_INOUT;
        if (b == "POWER")   return LefMacro::PAD_POWER;
        if (b == "SPACER")  return LefMacro::PAD_SPACER;
        if (b == "AREAIO")  return LefMacro::PAD_AREAIO;
        return LefMacro::PAD;
    }
    if (a == "BLOCK")       return LefMacro::BLOCK;
    if (a == "RING")        return LefMacro::RING;
    if (a == "COVER")       return LefMacro::COVER;
    if (a == "ENDCAP") {
        if (b == "PRE")  return LefMacro::ENDCAP_PRE;
        if (b == "POST") return LefMacro::ENDCAP_POST;
    }
    return LefMacro::CORE;
}

void LefParser::parse_macro(LefLibrary& lib) {
    LefMacro macro;
    macro.name = advance();

    while (pos_ < tokens_.size()) {
        const auto& tok = peek();

        if (tok == "END") {
            advance();
            if (pos_ < tokens_.size() && peek() == macro.name) advance();
            break;
        }

        if (tok == "CLASS") {
            advance();
            std::string a = advance();
            std::string b;
            if (peek() != ";") b = advance();
            macro.macro_class = parse_macro_class_str(a, b);
            // Detect filler cells
            if (a == "CORE" && b == "SPACER") macro.is_filler = true;
            match(";");
        }
        else if (tok == "SIZE") {
            advance();
            macro.width = next_double();
            expect("BY");
            macro.height = next_double();
            match(";");
        }
        else if (tok == "ORIGIN") {
            advance();
            macro.origin_x = next_double();
            macro.origin_y = next_double();
            match(";");
        }
        else if (tok == "SYMMETRY") {
            advance();
            std::string sym;
            while (peek() != ";") {
                if (!sym.empty()) sym += ' ';
                sym += advance();
            }
            macro.symmetry = sym;
            match(";");
        }
        else if (tok == "SITE") {
            advance();
            macro.site_ref.name = advance();
            // Try to resolve from already-parsed sites
            if (auto* s = lib.find_site(macro.site_ref.name))
                macro.site_ref = *s;
            match(";");
        }
        else if (tok == "SOURCE") {
            advance();
            macro.source = advance();
            match(";");
        }
        else if (tok == "PIN") {
            advance();
            parse_macro_pin(macro);
        }
        else if (tok == "OBS") {
            advance();
            parse_macro_obs(macro);
        }
        else if (tok == "PROPERTY") {
            advance();
            // PROPERTY key value ... ;
            std::string key = advance();
            std::string val;
            while (peek() != ";") {
                if (!val.empty()) val += ' ';
                val += advance();
            }
            macro.properties[key] = val;
            match(";");
        }
        else if (tok == "FOREIGN") {
            advance(); skip_to_semicolon();
        }
        else {
            advance(); skip_to_semicolon();
        }
    }

    lib.macros.push_back(std::move(macro));
}

// ── MACRO PIN ────────────────────────────────────────────────────────────────

void LefParser::parse_macro_pin(LefMacro& macro) {
    LefPin pin;
    pin.name = advance();

    while (pos_ < tokens_.size()) {
        const auto& tok = peek();

        if (tok == "END") {
            advance();
            if (pos_ < tokens_.size() && peek() == pin.name) advance();
            break;
        }

        if (tok == "DIRECTION") {
            advance();
            const auto& d = advance();
            if      (d == "INPUT")   pin.direction = LefPin::INPUT;
            else if (d == "OUTPUT")  pin.direction = LefPin::OUTPUT;
            else if (d == "INOUT")   pin.direction = LefPin::INOUT;
            else if (d == "FEEDTHRU") pin.direction = LefPin::FEEDTHRU;
            match(";");
        }
        else if (tok == "USE") {
            advance();
            const auto& u = advance();
            if      (u == "SIGNAL") pin.use = LefPin::SIGNAL;
            else if (u == "POWER")  pin.use = LefPin::POWER;
            else if (u == "GROUND") pin.use = LefPin::GROUND;
            else if (u == "CLOCK")  pin.use = LefPin::CLOCK;
            else if (u == "ANALOG") { pin.use = LefPin::ANALOG; pin.is_analog = true; }
            match(";");
        }
        else if (tok == "SHAPE") {
            advance();
            pin.shape = advance();
            match(";");
        }
        else if (tok == "ANTENNAMODEL" || tok == "ANTENNAGATEAREA"
              || tok == "ANTENNADIFFAREA" || tok == "ANTENNAPARTIALMETALAREA"
              || tok == "ANTENNAPARTIALMETALSIDEAREA") {
            advance(); skip_to_semicolon();
        }
        else if (tok == "PORT") {
            advance();
            LefPin::Port port;

            while (pos_ < tokens_.size()) {
                const auto& pt = peek();

                if (pt == "END") {
                    advance(); // END (of PORT)
                    break;
                }

                if (pt == "LAYER") {
                    advance();
                    // New port per LAYER if we already have rects for the previous
                    if (!port.layer.empty() && !port.rects.empty()) {
                        pin.ports.push_back(std::move(port));
                        port = LefPin::Port{};
                    }
                    port.layer = advance();
                    match(";");
                }
                else if (pt == "RECT") {
                    advance();
                    double x0 = next_double();
                    double y0 = next_double();
                    double x1 = next_double();
                    double y1 = next_double();
                    port.rects.push_back({x0, y0, x1, y1});
                    match(";");
                }
                else if (pt == "POLYGON") {
                    // Approximate polygon as bounding box
                    advance();
                    double xmin = 1e30, ymin = 1e30, xmax = -1e30, ymax = -1e30;
                    while (peek() != ";") {
                        double x = next_double();
                        double y = next_double();
                        xmin = std::min(xmin, x); ymin = std::min(ymin, y);
                        xmax = std::max(xmax, x); ymax = std::max(ymax, y);
                    }
                    port.rects.push_back({xmin, ymin, xmax, ymax});
                    match(";");
                }
                else if (pt == "VIA") {
                    advance(); skip_to_semicolon();
                }
                else {
                    advance(); skip_to_semicolon();
                }
            }

            if (!port.layer.empty()) pin.ports.push_back(std::move(port));
        }
        else if (tok == "CAPACITANCE") {
            advance();
            pin.capacitance = next_double();
            match(";");
        }
        else if (tok == "MAXFANOUT") {
            advance();
            pin.max_fanout = next_double();
            match(";");
        }
        else {
            advance(); skip_to_semicolon();
        }
    }

    macro.pins.push_back(std::move(pin));
}

// ── MACRO OBS ────────────────────────────────────────────────────────────────

void LefParser::parse_macro_obs(LefMacro& macro) {
    LefObs::ObsLayer current;

    while (pos_ < tokens_.size()) {
        const auto& tok = peek();

        if (tok == "END") {
            advance(); // END (of OBS)
            break;
        }

        if (tok == "LAYER") {
            advance();
            if (!current.layer.empty()) macro.obs.layers.push_back(std::move(current));
            current = LefObs::ObsLayer{};
            current.layer = advance();
            match(";");
        }
        else if (tok == "RECT") {
            advance();
            double x0 = next_double();
            double y0 = next_double();
            double x1 = next_double();
            double y1 = next_double();
            current.rects.push_back({x0, y0, x1, y1});
            match(";");
        }
        else if (tok == "POLYGON") {
            advance();
            double xmin = 1e30, ymin = 1e30, xmax = -1e30, ymax = -1e30;
            while (peek() != ";") {
                double x = next_double();
                double y = next_double();
                xmin = std::min(xmin, x); ymin = std::min(ymin, y);
                xmax = std::max(xmax, x); ymax = std::max(ymax, y);
            }
            current.rects.push_back({xmin, ymin, xmax, ymax});
            match(";");
        }
        else {
            advance(); skip_to_semicolon();
        }
    }

    if (!current.layer.empty()) macro.obs.layers.push_back(std::move(current));
}

// ── SPACING ──────────────────────────────────────────────────────────────────

void LefParser::parse_spacing(LefLibrary& lib) {
    while (pos_ < tokens_.size()) {
        const auto& tok = peek();

        if (tok == "END") {
            advance(); // END
            if (pos_ < tokens_.size() && peek() == "SPACING") advance();
            return;
        }

        if (tok == "SAMENET") {
            advance();
            LefSpacing sp;
            sp.layer1      = advance();
            sp.layer2      = advance();
            sp.min_spacing = next_double();
            if (peek() == "STACK") { advance(); sp.stack = true; }
            match(";");
            lib.spacings.push_back(std::move(sp));
        }
        else {
            advance();
            skip_to_semicolon();
        }
    }
}

// ── PROPERTYDEFINITIONS ──────────────────────────────────────────────────────

void LefParser::parse_property(LefLibrary& lib) {
    while (pos_ < tokens_.size()) {
        const auto& tok = peek();
        if (tok == "END") {
            advance();
            if (pos_ < tokens_.size() && peek() == "PROPERTYDEFINITIONS") advance();
            return;
        }

        // Each definition: <object_type> <name> <type> [default] ;
        std::string obj = advance();
        if (pos_ < tokens_.size() && peek() != "END") {
            std::string name = advance();
            std::string rest;
            while (peek() != ";") {
                if (!rest.empty()) rest += ' ';
                rest += advance();
            }
            lib.properties[obj + "." + name] = rest;
            match(";");
        }
    }
}

} // namespace sf
