#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <format>
#include <fstream>
#include <print>

#include <unistd.h>

#include "einstar/image/image.hpp"
#include "einstar/model/document.hpp"
#include "einstar/model/photo_geometry.hpp"
#include "einstar/model/photo_render.hpp"

using namespace einstar;
using model::json;
using Catch::Approx;

namespace {

json run(model::Document& doc, std::string_view command, const json& params = json::object(), model::Author author = model::Author::script) {
    const model::Outcome o = doc.apply(command, params, author);
    INFO(command << " " << params.dump());
    INFO(o.error);
    REQUIRE(o.ok);
    return o.result;
}

bool fails(model::Document& doc, std::string_view command, const json& params) { return !doc.apply(command, params).ok; }

// A photo file: w x h of a colour gradient, as a phone might store it (orientation 6: on its side).
std::filesystem::path write_photo(const std::filesystem::path& dir, const std::string& name, int w, int h, int orientation = 1) {
    image::Rgba img;
    img.width = w;
    img.height = h;
    img.pixels.resize(static_cast<std::size_t>(w * h * 4));
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            auto* p = &img.pixels[static_cast<std::size_t>((y * w + x) * 4)];
            p[0] = static_cast<std::uint8_t>(255 * x / w);
            p[1] = static_cast<std::uint8_t>(255 * y / h);
            p[2] = 128;
            p[3] = 255;
        }
    const auto path = dir / name;
    std::ofstream(path, std::ios::binary) << image::encode_jpeg(img, 0.9, orientation);
    return path;
}

struct TempDir {
    std::filesystem::path path = std::filesystem::temp_directory_path() / std::format("einstar_photos_{}", ::getpid());
    TempDir() { std::filesystem::create_directories(path); }
    ~TempDir() { std::filesystem::remove_all(path); }
};

}  // namespace

TEST_CASE("values are read as the user types them") {
    const auto v = [](std::string_view s, bool angle = false) { return model::parse_value(s, angle); };
    CHECK(v("42")->value == Approx(42));
    CHECK(v("42 mm")->value == Approx(42));
    CHECK(v("4.2cm")->value == Approx(42));
    CHECK(v("1.5in")->value == Approx(38.1));
    CHECK(v("1 1/2\"")->value == Approx(38.1));
    CHECK(v("3/8 in")->value == Approx(9.525));
    CHECK(v("0.1 m")->value == Approx(100));
    CHECK(v("Ø6")->form == 'D');
    CHECK(v("Ø6")->value == Approx(6));
    CHECK(v("R2.5")->form == 'R');
    CHECK(v("dia 8")->form == 'D');
    CHECK(v("30°")->angle);
    CHECK(v("30 deg")->value == Approx(30));
    CHECK(v("30", true)->angle);
    const auto t = v("Ø6 ±0.02");
    REQUIRE(t->tolerance);
    CHECK(*t->tolerance == Approx(0.02));
    CHECK(v("12 +/- 0.1 mm")->tolerance.value_or(0) == Approx(0.1));
    CHECK(!v("about six"));
    CHECK(!v(""));
}

TEST_CASE("a photo library: imported, annotated, linked, saved with the model, undone") {
    TempDir tmp;
    const auto front = write_photo(tmp.path, "front.jpg", 400, 300);
    const auto side = write_photo(tmp.path, "IMG_0042.jpg", 300, 200, 6);  // a phone photo on its side

    model::Document doc;
    CHECK(fails(doc, "photo.import", {{"path", front.string()}}));  // no scan yet
    run(doc, "open_demo");
    run(doc, "detect");
    const json imported = run(doc, "photo.import", {{"paths", {front.string(), side.string()}}});
    REQUIRE(imported["photos"].size() == 2);
    CHECK(imported["photos"][0]["name"] == "front");
    CHECK(imported["photos"][1]["name"] == "IMG_0042");
    CHECK(imported["photos"][1]["size"] == json({200, 300}));  // as shown: upright
    // The same file again shares its bytes, under another name.
    CHECK(run(doc, "photo.import", {{"path", front.string()}})["photos"][0]["name"] == "front 2");
    run(doc, "photo.delete", {{"photo", "front 2"}});
    CHECK(fails(doc, "photo.import", {{"path", (tmp.path / "missing.jpg").string()}}));

    // Annotations, with the value as typed.
    const std::string hole = run(doc, "summary")["holes"][0]["name"];
    const json d1 = run(doc, "photo.annotate", {{"photo", "front"}, {"kind", "dimension"}, {"points", {{20, 30}, {380, 30}}},
                                                {"value", "1 1/2 in"}, {"text", "flange width"}});
    CHECK(d1["name"] == "D1");
    CHECK(d1["value"].get<double>() == Approx(38.1));
    CHECK(d1["entered"] == "1 1/2 in");
    const json dia = run(doc, "photo.annotate", {{"photo", "front"}, {"kind", "diameter"}, {"points", {{100, 100}, {120, 120}, {140, 100}}},
                                                 {"value", "Ø5.5 ±0.05"}, {"links", {hole}}},
                         model::Author::agent);
    CHECK(dia["name"] == "DIA1");
    CHECK(dia["links"] == json({hole}));
    CHECK(dia["author"] == "agent");
    CHECK(dia["circle"]["radius_px"].get<double>() == Approx(20));
    CHECK(dia["tolerance"].get<double>() == Approx(0.05));
    run(doc, "photo.annotate", {{"photo", "IMG_0042"}, {"kind", "angle"}, {"points", {{10, 10}, {100, 100}, {190, 10}}}, {"value", "90"}});
    run(doc, "photo.annotate", {{"photo", "IMG_0042"}, {"kind", "callout"}, {"points", {{50, 250}, {150, 280}}}, {"text", "M6 thread"}});
    run(doc, "note.add", {{"text", "6061 aluminium, bead blasted"}});

    // Refused: off the photo, the wrong number of points, a length as an angle, a callout saying nothing.
    CHECK(fails(doc, "photo.annotate", {{"photo", "IMG_0042"}, {"kind", "note"}, {"points", {{250, 10}}}, {"text", "x"}}));  // 200 wide
    CHECK(fails(doc, "photo.annotate", {{"photo", "front"}, {"kind", "dimension"}, {"points", {{1, 1}}}, {"value", 3}}));
    CHECK(fails(doc, "photo.annotate", {{"photo", "front"}, {"kind", "angle"}, {"points", {{1, 1}, {2, 2}, {3, 1}}}, {"value", "5 mm"}}));
    CHECK(fails(doc, "photo.annotate", {{"photo", "front"}, {"kind", "callout"}, {"points", {{1, 1}, {2, 2}}}}));

    const json summary = run(doc, "summary");
    REQUIRE(summary["photos"].size() == 2);
    CHECK(summary["photos"][0]["annotations"][0] == "D1 38.10 mm: flange width");
    CHECK(summary["notes"][0]["text"] == "6061 aluminium, bead blasted");
    const json listed = run(doc, "photo.list", {{"photo", "front"}});
    CHECK(listed["photos"][0]["annotations"].size() == 2);

    // The agent's view of a photo: scaled, cropped, drawn on.
    const model::Photo& photo = doc.state().photos[0];
    model::PhotoRenderOptions small;
    small.max_size = 200;
    const auto shown = model::render_photo(photo, small);
    REQUIRE(shown);
    CHECK((shown->width == 200 && shown->height == 150));
    CHECK(image::probe(shown->jpeg)->width == 200);
    model::PhotoRenderOptions part;
    part.crop = std::array<double, 4>{100, 50, 100, 100};
    part.grid = true;
    const auto crop = model::render_photo(photo, part);
    REQUIRE(crop);
    CHECK((crop->width == 100 && crop->height == 100));

    // Saved with the model: the bytes as imported, the annotations as made.
    const auto path = tmp.path / "part.emodel";
    run(doc, "save", {{"path", path.string()}});
    model::Document again;
    run(again, "open", {{"path", path.string()}});
    REQUIRE(again.state().photos.size() == 2);
    std::ifstream f(front, std::ios::binary);
    const std::string front_bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    CHECK(again.state().photos[0].blob->bytes == front_bytes);
    CHECK(run(again, "photo.list")["photos"] == run(doc, "photo.list")["photos"]);
    CHECK(run(again, "summary")["notes"] == summary["notes"]);

    // A deleted hole leaves the annotation, without its link; undo brings the link back.
    run(doc, "hole.delete", {{"hole", hole}});
    CHECK(!run(doc, "photo.list")["photos"][0]["annotations"][1].contains("links"));
    run(doc, "undo");
    CHECK(run(doc, "photo.list")["photos"][0]["annotations"][1]["links"] == json({hole}));

    // Undoing a photo's deletion brings it back, bytes and all.
    run(doc, "photo.delete", {{"photo", "front"}});
    CHECK(run(doc, "summary")["photos"].size() == 1);
    run(doc, "undo");
    REQUIRE(doc.state().photos.size() == 2);
    CHECK(doc.state().photos[0].blob->bytes == front_bytes);

    run(doc, "photo.annotation.update", {{"annotation", "D1"}, {"value", nullptr}, {"text", "flange width (not measured)"}});
    CHECK(!run(doc, "photo.list")["photos"][0]["annotations"][0].contains("value"));
    run(doc, "photo.annotation.delete", {{"annotation", "D1"}});
    CHECK(run(doc, "photo.list")["photos"][0]["annotations"].size() == 1);
}

TEST_CASE("a photo registered to the scan: its camera from matched points, the scan's reading of what is marked on it") {
    TempDir tmp;
    model::Document doc;
    run(doc, "open_demo");
    run(doc, "detect");
    // A 1600 x 1200 photo, taken from above and to one side with a 28 mm equivalent lens.
    const int W = 1600, H = 1200;
    fit::PinholeCamera truth;
    truth.focal = fit::focal_from_35mm(28, W, H);
    truth.principal = Vec2(W / 2.0, H / 2.0);
    const Vec3 eye(110, -150, 160), target(0, 0, 8);
    const Vec3 z = (target - eye).normalized(), x = z.cross(Vec3::UnitZ()).normalized(), y = z.cross(x);
    Mat3 R;
    R.row(0) = x.transpose();
    R.row(1) = y.transpose();
    R.row(2) = z.transpose();
    truth.T_camera_world.linear() = R;
    truth.T_camera_world.translation() = -R * eye;
    run(doc, "photo.import", {{"path", write_photo(tmp.path, "side.jpg", W, H).string()}});

    // Points the user would match: where the camera sees the part through chosen pixels.
    const auto seen_at = [&](const Vec2& px) {
        const auto [o, d] = truth.ray(px);
        const auto hit = doc.bvh().raycast(o.cast<float>(), d.cast<float>());
        REQUIRE(hit);
        return hit->point.cast<double>();
    };
    // Corners and edges of the part, located where the true camera sees them.
    const std::vector<Vec3> corners = {{-50, -25, 4}, {50, -25, 4}, {50, 25, 4}, {-30, -20, 20}, {30, -20, 20}, {30, 20, 20}, {-30, 20, 20}, {50, -25, 1.5}};
    json last;
    for (std::size_t i = 0; i < corners.size(); ++i) {
        // The scan's own point nearest the corner (the scan rounds edges), and where the photo shows it.
        const Vec3 on = doc.bvh().closest(corners[i].cast<float>())->point.cast<double>();
        const Vec2 px = *truth.project(on);
        last = run(doc, "photo.correspond", {{"photo", "side"}, {"pixel", {px.x(), px.y()}}, {"point", {on.x(), on.y(), on.z()}}});
        CHECK(last["registered"].get<bool>() == (i + 1 >= 6));
    }
    std::println("registered: {}", last["camera"].dump());
    CHECK(last["camera"]["rms_px"].get<double>() < 0.5);
    const auto& cam = *doc.state().photos[0].camera;
    CHECK((model::pinhole_of(cam).center() - eye).norm() < 1.0);

    // A dimension across the flange's front edge, and a hole's rim: the scan's reading and the suggested links.
    const Vec2 a = *truth.project(seen_at(*truth.project(Vec3(-45, -25, 4)))), b = *truth.project(seen_at(*truth.project(Vec3(45, -25, 4))));
    const json dim = run(doc, "photo.annotate", {{"photo", "side"}, {"kind", "dimension"}, {"points", {{a.x(), a.y()}, {b.x(), b.y()}}}, {"value", 90}});
    REQUIRE(dim.contains("scan_value"));
    CHECK(std::abs(dim["scan_value"].get<double>() - 90) < 1.0);
    json rim = json::array();
    for (const double t : {0.3, 2.2, 4.1}) {
        const Vec2 q = *truth.project(Vec3(42 + 2.75 * std::cos(t), -19 + 2.75 * std::sin(t), 4));
        rim.push_back({q.x(), q.y()});
    }
    const json dia = run(doc, "photo.annotate", {{"photo", "side"}, {"kind", "diameter"}, {"points", rim}, {"value", "5.5"}});
    std::println("hole rim from the photo: scan {:.2f} mm, links {}", dia["scan_value"].get<double>(), dia["suggested_links"].dump());
    CHECK(std::abs(dia["scan_value"].get<double>() - 5.5) < 0.6);
    const std::string hole = [&] {
        for (const auto& h : run(doc, "summary")["holes"])
            if (std::hypot(h["center"][0].get<double>() - 42, h["center"][1].get<double>() + 19) < 1) return h["name"].get<std::string>();
        return std::string();
    }();
    CHECK(std::ranges::find(dia["suggested_links"], json(hole)) != dia["suggested_links"].end());

    // Projections both ways; the model drawn over the photo.
    const Vec2 top = *truth.project(Vec3(0, -10, 20));
    const json proj = run(doc, "photo.project", {{"photo", "side"}, {"point", {30, -20, 20}}, {"pixel", {top.x(), top.y()}}});
    CHECK(proj["in_photo"]["visible"].get<bool>());
    CHECK(!proj["on_scan"].is_null());
    CHECK(model::overlay_lines(doc, doc.state().photos[0]).size() > 100);

    // Background is not the part: no matching on it, and it hides nothing.
    const std::string under = proj["on_scan"]["label"];
    run(doc, "label.update", {{"label", under}, {"role", "ignore"}});
    const auto bgv = proj["on_scan"]["point"].get<std::vector<double>>();
    const Vec3 bg(bgv[0], bgv[1], bgv[2]);
    const auto o = doc.apply("photo.correspond", {{"photo", "side"}, {"pixel", {top.x(), top.y()}}, {"point", {bg.x(), bg.y(), bg.z()}}});
    CHECK(!o.ok);
    CHECK(o.error.find("not the part") != std::string::npos);

    // Saved and read back with its camera.
    const auto path = tmp.path / "registered.emodel";
    run(doc, "save", {{"path", path.string()}});
    model::Document again;
    run(again, "open", {{"path", path.string()}});
    REQUIRE(again.state().photos[0].camera);
    CHECK((model::pinhole_of(*again.state().photos[0].camera).center() - model::pinhole_of(cam).center()).norm() < 1e-9);
    CHECK(again.state().photos[0].correspondences.size() == corners.size());
}

TEST_CASE("a measurement on a photo applied to the model: by its links, a distance, a pitch, a diameter, a radius, an angle") {
    TempDir tmp;
    model::Document doc;
    run(doc, "open_demo");
    run(doc, "detect");
    run(doc, "photo.import", {{"path", write_photo(tmp.path, "bench.jpg", 400, 300).string()}});
    const json s = run(doc, "summary");
    // The faces and holes by where they are.
    const auto label_at = [&](json origin, json dir) { return run(doc, "raycast", {{"origin", origin}, {"direction", dir}})["label"].get<std::string>(); };
    const std::string top = label_at({10, 8, 100}, {0, 0, -1}), flange = label_at({45, 0, 100}, {0, 0, -1});
    const std::string right = label_at({80, 3, 12}, {-1, 0, 0}), front = label_at({3, -80, 12}, {0, 1, 0});
    std::string h1, h2, blind;
    for (const auto& h : s["holes"]) {
        const double x = h["center"][0], y = h["center"][1];
        if (std::hypot(x - 42, y + 19) < 1) h1 = h["name"];
        if (std::hypot(x - 42, y - 19) < 1) h2 = h["name"];
        if (std::hypot(x, y) < 1) blind = h["name"];
    }
    REQUIRE(!s["fillets"].empty());
    const std::string fillet = s["fillets"][0]["name"];
    const auto annotate = [&](const char* kind, json points, json value, json links) {
        return run(doc, "photo.annotate", {{"photo", "bench"}, {"kind", kind}, {"points", points}, {"value", value}, {"links", links}})["name"].get<std::string>();
    };
    const json two = {{10, 10}, {200, 10}};
    const auto applied = [&](const std::string& name) { return run(doc, "photo.apply", {{"annotation", name}})["applied"]; };

    CHECK(applied(annotate("dimension", two, "16 mm", {top, flange}))["constraint"]["type"] == "distance");
    CHECK(applied(annotate("dimension", two, "38", {h1, h2}))["constraint"]["type"] == "axis_distance");
    const json dia = applied(annotate("diameter", {{50, 50}, {60, 60}, {70, 50}}, "Ø8.0", {blind}));
    CHECK(dia["constraint"]["type"] == "diameter");
    CHECK(dia["constraint"]["value"].get<double>() == Approx(8.0));
    const json r = applied(annotate("callout", two, "R2", {fillet}));
    CHECK(r["fillet"] == fillet);
    CHECK(r["radius"].get<double>() == Approx(2.0));
    CHECK(applied(annotate("angle", {{10, 10}, {100, 100}, {190, 10}}, "90", {right, front}))["constraint"]["type"] == "angle");

    // Not enough to go on: said so.
    const std::string vague = annotate("dimension", two, "12", {top});
    const auto o = doc.apply("photo.apply", {{"annotation", vague}});
    CHECK(!o.ok);
    CHECK(o.error.find("Link two faces") != std::string::npos);

    // Applying again replaces; after a solve, the annotation shows how its constraint fared.
    const std::size_t before = run(doc, "summary")["constraints"].size();
    run(doc, "photo.annotation.update", {{"annotation", "D1"}, {"value", "16.0 mm"}});
    applied("D1");
    CHECK(run(doc, "summary")["constraints"].size() == before);
    run(doc, "solve");
    const json listed = run(doc, "photo.list", {{"photo", "bench"}})["photos"][0]["annotations"];
    REQUIRE(listed[0].contains("applied"));
    CHECK(listed[0]["applied"]["last_solve"]["status"].is_string());
    std::println("D1 after the solve: {}", listed[0]["applied"]["last_solve"].dump());
}
