#include "einstar/model/photo.hpp"

#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <format>
#include <string>
#include <utility>

namespace einstar::model {
namespace {

constexpr std::array<std::pair<AnnotationKind, const char*>, 5> kKinds = {{{AnnotationKind::dimension, "dimension"},
                                                                         {AnnotationKind::diameter, "diameter"},
                                                                         {AnnotationKind::angle, "angle"},
                                                                         {AnnotationKind::callout, "callout"},
                                                                         {AnnotationKind::note, "note"}}};

struct Scanner {
    std::string_view s;
    std::size_t i = 0;
    void spaces() {
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    }
    bool eat(std::string_view token, bool fold_case = false) {
        if (s.size() - i < token.size()) return false;
        for (std::size_t k = 0; k < token.size(); ++k) {
            const char a = s[i + k], b = token[k];
            if (fold_case ? std::tolower(static_cast<unsigned char>(a)) != std::tolower(static_cast<unsigned char>(b)) : a != b) return false;
        }
        i += token.size();
        return true;
    }
    [[nodiscard]] bool at_digit() const { return i < s.size() && (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '.'); }
    std::optional<double> number() {
        if (!at_digit()) return std::nullopt;
        // Apple's libc++ has no floating-point std::from_chars, so parse with strtod. The view may not be
        // null-terminated, so copy the remainder (these strings are short); at_digit() ruled out a sign/inf/nan.
        const std::string rest(s.substr(i));
        const char* begin = rest.c_str();
        char* end = nullptr;
        const double v = std::strtod(begin, &end);
        if (end == begin) return std::nullopt;
        i += static_cast<std::size_t>(end - begin);
        return v;
    }
    // 1.5, 3/8, 1 1/2
    std::optional<double> quantity() {
        const auto a = number();
        if (!a) return std::nullopt;
        const std::size_t after_a = i;
        spaces();
        if (eat("/")) {
            spaces();
            const auto b = number();
            if (!b || *b == 0) return std::nullopt;
            return *a / *b;
        }
        // A mixed fraction: a whole number, a space, then b/c.
        if (i > after_a && at_digit()) {
            const std::size_t save = i;
            const auto b = number();
            spaces();
            if (b && eat("/")) {
                spaces();
                if (const auto c = number(); c && *c != 0) return *a + *b / *c;
            }
            i = save;
        }
        i = after_a;
        return a;
    }
    // A unit after a quantity: the factor to mm (lengths) or degrees (angles).
    std::optional<std::pair<double, bool>> unit() {
        spaces();
        if (eat("mm", true)) return std::pair{1.0, false};
        if (eat("cm", true)) return std::pair{10.0, false};
        if (eat("inches", true) || eat("inch", true) || eat("in", true) || eat("\"") || eat("''") || eat("”")) return std::pair{25.4, false};
        if (eat("°") || eat("degrees", true) || eat("deg", true)) return std::pair{1.0, true};
        const std::size_t save = i;
        if (eat("m", true) && (i >= s.size() || !std::isalpha(static_cast<unsigned char>(s[i])))) return std::pair{1000.0, false};
        i = save;
        return std::nullopt;
    }
};

}  // namespace

std::string_view annotation_kind_name(AnnotationKind k) {
    for (const auto& [kind, name] : kKinds)
        if (kind == k) return name;
    return "?";
}

std::optional<AnnotationKind> annotation_kind_from_name(std::string_view s) {
    for (const auto& [kind, name] : kKinds)
        if (s == name) return kind;
    return std::nullopt;
}

std::string_view annotation_prefix(AnnotationKind k) {
    switch (k) {
        case AnnotationKind::dimension: return "D";
        case AnnotationKind::diameter: return "DIA";
        case AnnotationKind::angle: return "A";
        case AnnotationKind::callout: return "C";
        case AnnotationKind::note: return "N";
    }
    return "?";
}

std::pair<int, int> annotation_points(AnnotationKind k) {
    switch (k) {
        case AnnotationKind::dimension: return {2, 2};
        case AnnotationKind::diameter: return {2, 3};
        case AnnotationKind::angle: return {3, 3};
        case AnnotationKind::callout: return {2, 2};
        case AnnotationKind::note: return {1, 1};
    }
    return {1, 1};
}

namespace {
nlohmann::json pixel_json(const Vec2& v) { return {v.x(), v.y()}; }
Vec2 pixel_from(const nlohmann::json& j) { return {j.at(0).get<double>(), j.at(1).get<double>()}; }
}  // namespace

nlohmann::json annotation_to_file(const Annotation& a) {
    nlohmann::json j = {{"id", a.id}, {"name", a.name}, {"kind", annotation_kind_name(a.kind)}, {"points", nlohmann::json::array()},
                        {"entered", a.entered}, {"text", a.text}, {"links", a.links}, {"applied", a.applied}, {"author", a.author}};
    for (const auto& q : a.points) j["points"].push_back(pixel_json(q));
    if (a.value) j["value"] = *a.value;
    if (a.tolerance) j["tolerance"] = *a.tolerance;
    return j;
}

Annotation annotation_from_file(const nlohmann::json& j) {
    Annotation a;
    a.id = j.at("id");
    a.name = j.at("name");
    a.kind = annotation_kind_from_name(j.at("kind").get<std::string>()).value_or(AnnotationKind::note);
    for (const auto& q : j.at("points")) a.points.push_back(pixel_from(q));
    a.entered = j.value("entered", "");
    a.text = j.value("text", "");
    a.links = j.value("links", std::vector<int>{});
    a.applied = j.value("applied", 0);
    a.author = j.value("author", "user");
    if (j.contains("value")) a.value = j["value"].get<double>();
    if (j.contains("tolerance")) a.tolerance = j["tolerance"].get<double>();
    return a;
}

nlohmann::json photo_to_file(const Photo& p) {
    nlohmann::json j = {{"id", p.id},         {"name", p.name},     {"caption", p.caption},       {"blob", p.blob ? p.blob->hash : ""},
                        {"width", p.width},   {"height", p.height}, {"exif", p.exif},             {"annotations", nlohmann::json::array()},
                        {"correspondences", nlohmann::json::array()}, {"use_for_colour", p.use_for_colour}};
    for (const auto& a : p.annotations) j["annotations"].push_back(annotation_to_file(a));
    for (const auto& c : p.correspondences)
        j["correspondences"].push_back({{"id", c.id}, {"pixel", pixel_json(c.pixel)}, {"point", {c.point.x(), c.point.y(), c.point.z()}}});
    if (p.camera) {
        const auto& c = *p.camera;
        nlohmann::json T = nlohmann::json::array();
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 4; ++k) T.push_back(c.T_camera_world.matrix()(r, k));
        j["camera"] = {{"focal_px", c.focal_px}, {"principal", pixel_json(c.principal)}, {"k1", c.k1}, {"T_camera_world", T}, {"rms_px", c.rms_px}};
    }
    return j;
}

Photo photo_from_file(const nlohmann::json& j, std::shared_ptr<const PhotoBlob> blob) {
    Photo p;
    p.id = j.at("id");
    p.name = j.at("name");
    p.caption = j.value("caption", "");
    p.blob = std::move(blob);
    p.width = j.at("width");
    p.height = j.at("height");
    p.exif = j.value("exif", nlohmann::json::object());
    p.use_for_colour = j.value("use_for_colour", true);
    for (const auto& a : j.value("annotations", nlohmann::json::array())) p.annotations.push_back(annotation_from_file(a));
    for (const auto& c : j.value("correspondences", nlohmann::json::array()))
        p.correspondences.push_back({c.at("id").get<int>(), pixel_from(c.at("pixel")),
                                     Vec3(c.at("point").at(0).get<double>(), c.at("point").at(1).get<double>(), c.at("point").at(2).get<double>())});
    if (j.contains("camera")) {
        const auto& c = j["camera"];
        PhotoCamera cam;
        cam.focal_px = c.at("focal_px");
        cam.principal = pixel_from(c.at("principal"));
        cam.k1 = c.value("k1", 0.0);
        cam.rms_px = c.value("rms_px", 0.0);
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 4; ++k) cam.T_camera_world.matrix()(r, k) = c.at("T_camera_world").at(static_cast<std::size_t>(r * 4 + k)).get<double>();
        p.camera = cam;
    }
    return p;
}

std::optional<std::pair<Vec2, double>> annotation_circle(const Annotation& a) {
    if (a.kind != AnnotationKind::diameter) return std::nullopt;
    if (a.points.size() == 2) return std::pair{a.points[0], (a.points[1] - a.points[0]).norm()};
    if (a.points.size() != 3) return std::nullopt;
    // The circle through three points: the intersection of two chords' perpendicular bisectors.
    const Vec2 &p = a.points[0], &q = a.points[1], &r = a.points[2];
    const double d = 2 * (p.x() * (q.y() - r.y()) + q.x() * (r.y() - p.y()) + r.x() * (p.y() - q.y()));
    if (std::abs(d) < 1e-9) return std::nullopt;
    const double p2 = p.squaredNorm(), q2 = q.squaredNorm(), r2 = r.squaredNorm();
    const Vec2 c((p2 * (q.y() - r.y()) + q2 * (r.y() - p.y()) + r2 * (p.y() - q.y())) / d,
                 (p2 * (r.x() - q.x()) + q2 * (p.x() - r.x()) + r2 * (q.x() - p.x())) / d);
    return std::pair{c, (p - c).norm()};
}

std::string annotation_value_text(const Annotation& a) {
    if (!a.value) return {};
    std::string s = a.kind == AnnotationKind::angle ? std::format("{:.1f}\u00B0", *a.value)
                    : a.kind == AnnotationKind::diameter ? std::format("\u00D8{:.2f} mm", *a.value)
                                                         : std::format("{:.2f} mm", *a.value);
    if (a.tolerance) s += std::format(" \u00B1{:g}", *a.tolerance);
    return s;
}

std::string annotation_summary(const Annotation& a) {
    std::string s = a.name;
    if (const auto v = annotation_value_text(a); !v.empty()) s += " " + v;
    if (!a.text.empty()) s += (a.value ? ": " : " ") + a.text;
    return s;
}

std::optional<ParsedValue> parse_value(std::string_view text, bool angle_default) {
    Scanner sc{text};
    ParsedValue out;
    sc.spaces();
    if (sc.eat("Ø") || sc.eat("⌀") || sc.eat("diam", true) || sc.eat("dia", true)) {
        out.form = 'D';
    } else if ((sc.eat("R") || sc.eat("r")) && (sc.spaces(), sc.at_digit())) {
        out.form = 'R';
    } else {
        sc.i = 0;
    }
    sc.spaces();
    const auto q = sc.quantity();
    if (!q) return std::nullopt;
    const auto u = sc.unit();
    out.angle = u ? u->second : angle_default;
    const double factor = u ? u->first : 1.0;
    out.value = *q * factor;
    sc.spaces();
    if (sc.eat("±") || sc.eat("+/-") || sc.eat("+-")) {
        sc.spaces();
        if (const auto t = sc.quantity()) {
            const auto tu = sc.unit();
            out.tolerance = *t * (tu ? tu->first : factor);
        }
    }
    return out;
}

std::string base64_encode(std::string_view in) {
    static constexpr char kTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((in.size() + 2) / 3 * 4);
    std::size_t i = 0;
    for (; i + 2 < in.size(); i += 3) {
        const std::uint32_t v = static_cast<std::uint32_t>(static_cast<std::uint8_t>(in[i])) << 16 |
                                static_cast<std::uint32_t>(static_cast<std::uint8_t>(in[i + 1])) << 8 | static_cast<std::uint8_t>(in[i + 2]);
        out += kTable[v >> 18 & 63];
        out += kTable[v >> 12 & 63];
        out += kTable[v >> 6 & 63];
        out += kTable[v & 63];
    }
    if (i < in.size()) {
        std::uint32_t v = static_cast<std::uint32_t>(static_cast<std::uint8_t>(in[i])) << 16;
        if (i + 1 < in.size()) v |= static_cast<std::uint32_t>(static_cast<std::uint8_t>(in[i + 1])) << 8;
        out += kTable[v >> 18 & 63];
        out += kTable[v >> 12 & 63];
        out += i + 1 < in.size() ? kTable[v >> 6 & 63] : '=';
        out += '=';
    }
    return out;
}

std::optional<std::string> base64_decode(std::string_view in) {
    const auto value = [](char c) -> int {
        if (c >= 'A' && c <= 'Z') return c - 'A';
        if (c >= 'a' && c <= 'z') return c - 'a' + 26;
        if (c >= '0' && c <= '9') return c - '0' + 52;
        if (c == '+' || c == '-') return 62;
        if (c == '/' || c == '_') return 63;
        return -1;
    };
    std::string out;
    out.reserve(in.size() / 4 * 3);
    std::uint32_t acc = 0;
    int bits = 0;
    for (const char c : in) {
        if (c == '=' || std::isspace(static_cast<unsigned char>(c))) continue;
        const int v = value(c);
        if (v < 0) return std::nullopt;
        acc = acc << 6 | static_cast<std::uint32_t>(v);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out += static_cast<char>(acc >> bits & 0xFF);
        }
    }
    return out;
}

}  // namespace einstar::model
