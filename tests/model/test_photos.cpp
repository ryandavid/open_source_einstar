#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <format>
#include <fstream>

#include <unistd.h>

#include "einstar/image/image.hpp"
#include "einstar/model/document.hpp"
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
