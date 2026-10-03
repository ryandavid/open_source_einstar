#pragma once

// Photos of the part, with what the user marked on them, and notes about the part.
//
// Photos may be taken anywhere (another room, another day, the part in its assembly): nothing assumes they show
// the scan's setting. Each keeps its file's bytes as imported. Annotations are drawn in the photo's pixels (as
// shown: EXIF orientation applied, origin top left, y down):
//   dimension  two points and a length;
//   diameter   three points on a rim, or a centre and a point on the rim;
//   angle      three points, the middle one the vertex;
//   callout    an arrow from its text (second point) to what it points at (first point), with text and/or a value;
//   note       text pinned at one point.
// An annotation can name labels, holes or fillets it is about (links).

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "einstar/core/se3.hpp"

namespace einstar::model {

// A photo file's bytes, shared between undo snapshots.
struct PhotoBlob {
    std::string bytes;
    std::string mime;
    std::string hash;
};

enum class AnnotationKind { dimension, diameter, angle, callout, note };
[[nodiscard]] std::string_view annotation_kind_name(AnnotationKind k);
[[nodiscard]] std::optional<AnnotationKind> annotation_kind_from_name(std::string_view s);
[[nodiscard]] std::string_view annotation_prefix(AnnotationKind k);  // names: D1, DIA2, A3, C4, N5
// How many points the kind takes: {min, max}.
[[nodiscard]] std::pair<int, int> annotation_points(AnnotationKind k);

struct Annotation {
    int id = 0;
    std::string name;
    AnnotationKind kind = AnnotationKind::dimension;
    std::vector<Vec2> points;          // photo pixels
    std::optional<double> value;       // mm (a length or diameter) or degrees (an angle)
    std::string entered;               // the value as typed ("1 1/2\"", "Ø6 H7")
    std::optional<double> tolerance;   // +- in the value's unit
    std::string text;
    std::vector<int> links;            // label, hole or fillet ids
    int applied = 0;                   // the constraint (or hole / fillet) the value was applied to
    std::string author;                // "user" / "agent" / "script"
};

// A point on the part seen in the photo and the same point on the scan (for the photo's camera).
struct Correspondence {
    int id = 0;
    Vec2 pixel = Vec2::Zero();
    Vec3 point = Vec3::Zero();
};

// Where a photo was taken from, relative to the part: a pinhole camera with one radial distortion term.
struct PhotoCamera {
    double focal_px = 0;
    Vec2 principal = Vec2::Zero();
    double k1 = 0;
    SE3 T_camera_world = SE3::Identity();
    double rms_px = 0;
};

struct Photo {
    int id = 0;
    std::string name;     // the file's name without extension by default
    std::string caption;
    std::shared_ptr<const PhotoBlob> blob;
    int width = 0, height = 0;  // as shown
    nlohmann::json exif = nlohmann::json::object();  // make, model, taken, focal_mm, focal_35mm (when recorded)
    std::vector<Annotation> annotations;
    std::vector<Correspondence> correspondences;
    std::optional<PhotoCamera> camera;
    bool use_for_colour = true;  // when its camera is known: may colour the scan
};

// A fact about the part, from the user or the agent: material, finish, "the bottom is flat", "M6 tapped".
struct Note {
    int id = 0;
    std::string text;
    std::string author;
};

// A diameter annotation's circle in the photo (centre, radius px): through its three rim points, or about its
// centre through its rim point.
[[nodiscard]] std::optional<std::pair<Vec2, double>> annotation_circle(const Annotation& a);
// Its value as shown: "42.00 mm", "Ø6.00 mm", "30.0°" (empty without a value).
[[nodiscard]] std::string annotation_value_text(const Annotation& a);
// One line for lists and the agent: "D1 42.00 mm: overall length".
[[nodiscard]] std::string annotation_summary(const Annotation& a);

// As stored in an .emodel's DOCU record (the bytes go in PHOT records, by hash).
[[nodiscard]] nlohmann::json photo_to_file(const Photo& p);
[[nodiscard]] Photo photo_from_file(const nlohmann::json& j, std::shared_ptr<const PhotoBlob> blob);  // throws json exceptions
[[nodiscard]] nlohmann::json annotation_to_file(const Annotation& a);
[[nodiscard]] Annotation annotation_from_file(const nlohmann::json& j);

// A value as typed: "42", "42 mm", "4.2cm", "1.5in", "1 1/2\"", "3/8 in", "Ø6", "R2.5", "30°", "30 deg",
// optionally followed by a tolerance ("±0.05", "+/-0.1"). Lengths come back in mm, angles in degrees.
struct ParsedValue {
    double value = 0;
    bool angle = false;
    char form = 0;  // 'D' (Ø, a diameter), 'R' (a radius), or 0
    std::optional<double> tolerance;
};
[[nodiscard]] std::optional<ParsedValue> parse_value(std::string_view text, bool angle_default = false);

// Standard base64 (for photos sent and received as text).
[[nodiscard]] std::string base64_encode(std::string_view bytes);
[[nodiscard]] std::optional<std::string> base64_decode(std::string_view text);

}  // namespace einstar::model
