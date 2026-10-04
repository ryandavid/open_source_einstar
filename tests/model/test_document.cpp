#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <print>
#include <set>

#include <unistd.h>

#include "einstar/agent/protocol.hpp"
#include "einstar/brep/brep.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/fit/synthetic_part.hpp"
#include "einstar/model/document.hpp"
#include "einstar/recon/mesh.hpp"

using namespace einstar;
using model::json;

namespace {

// Applies a command, failing the test with its error if it fails.
json run(model::Document& doc, std::string_view command, const json& params = json::object(), model::Author author = model::Author::script) {
    const model::Outcome o = doc.apply(command, params, author);
    INFO(command << " " << params.dump());
    INFO(o.error);
    REQUIRE(o.ok);
    return o.result;
}

// The label of the scan under a vertical ray from above.
std::string label_below(model::Document& doc, double x, double y) {
    const json hit = run(doc, "raycast", {{"origin", {x, y, 100}}, {"direction", {0, 0, -1}}});
    REQUIRE(hit["hit"].get<bool>());
    REQUIRE(hit["label"].is_string());
    return hit["label"];
}
std::string label_along(model::Document& doc, json origin, json direction) {
    const json hit = run(doc, "raycast", {{"origin", origin}, {"direction", direction}});
    REQUIRE(hit["hit"].get<bool>());
    REQUIRE(hit["label"].is_string());
    return hit["label"];
}

}  // namespace

TEST_CASE("the agent's method table and the document's commands are the same list") {
    std::set<std::string> specs, commands;
    for (const auto& s : agent::method_specs())
        if (s.app == agent::App::model && s.name != "model.view" && s.name != "model.photo.get") specs.insert(s.name.substr(std::string("model.").size()));  // view: the app's
    for (const auto& c : model::Document::commands()) commands.insert(c);
    CHECK(specs == commands);
}

TEST_CASE("the flanged box from scan to STEP through the commands alone") {
    model::Document doc;
    run(doc, "open_demo");

    // Auto-detect: faces, fillets, hole walls and holes.
    Stopwatch sw;
    const json det = run(doc, "detect");
    std::println("detect: {} labels, {} role changes, {} holes in {:.0f} ms", det["labels"].size(), det["roles"].size(), det["holes"].size(),
                 sw.elapsed_ms());
    CHECK(det["holes"].size() == 5);
    json summary = run(doc, "summary");
    const auto count_role = [&](const char* role) {
        return std::ranges::count_if(summary["labels"], [&](const json& l) { return l["role"] == role; });
    };
    std::println("labels: {} faces, {} fillets, {} hole walls; {} fillet edges", count_role("face"), count_role("fillet"), count_role("hole"),
                 summary["fillets"].size());
    CHECK(count_role("face") == 10);
    CHECK(summary["fillets"].size() == 8);
    CHECK(count_role("hole") == 5);

    // Name the faces the way a user would talk about them.
    const std::string top = label_below(doc, 10, 8), flange = label_below(doc, 40, -10);
    const std::string side = label_along(doc, {80, 3, 12}, {-1, 0, 0});
    run(doc, "label.update", {{"label", top}, {"name", "top"}});
    run(doc, "label.update", {{"label", flange}, {"name", "flange"}});
    run(doc, "label.update", {{"label", side}, {"name", "right"}});
    const std::string left = label_along(doc, {-80, 3, 12}, {1, 0, 0});
    run(doc, "label.update", {{"label", left}, {"name", "left"}});

    // "Make the box square", in one undoable change as an agent would.
    run(doc, "begin_change", {{"description", "square the part to a datum"}}, model::Author::agent);
    run(doc, "datum.create", {{"name", "part"}, {"z", "top"}, {"x", "right"}}, model::Author::agent);
    const json sq = run(doc, "square", {{"datum", "part"}}, model::Author::agent);
    run(doc, "end_change", {}, model::Author::agent);
    CHECK(sq["added"].size() == 10);
    CHECK(sq["skipped"].empty());
    // The part stood on its bottom: 20 mm below the top (the datum's origin lies on the top face).
    run(doc, "face.add_plane", {{"name", "bottom"}, {"datum", "part"}, {"axis", "z"}, {"offset", -20.0}, {"facing", "-"}});

    // Measurements from the real part.
    run(doc, "constraint.add", {{"type", "distance"}, {"a", "right"}, {"b", "left"}, {"value", 60.0}});
    run(doc, "constraint.add", {{"type", "offset"}, {"label", "flange"}, {"datum", "part"}, {"axis", "z"}, {"value", -16.0}});
    for (const auto& h : summary["holes"])
        if (std::abs(h["measured_diameter"].get<double>() - 5.5) < 0.2)
            run(doc, "constraint.add", {{"type", "diameter"}, {"hole", h["name"]}, {"value", 5.5}});
    for (const auto& f : summary["fillets"]) run(doc, "fillet.update", {{"fillet", f["name"]}, {"radius", 2.0}});
    // The 8 mm hole's floor was not scanned (its wall only to ~7 mm), so it reads as through; it is 10 mm deep.
    for (const auto& h : summary["holes"])
        if (std::abs(h["measured_diameter"].get<double>() - 8.0) < 0.2) {
            CHECK(h["through"].get<bool>());
            CHECK(h["wall_seen_to_depth"].get<double>() > 6.0);
            run(doc, "hole.update", {{"hole", h["name"]}, {"depth", 10.0}});
        }

    sw.reset();
    const json solved = run(doc, "solve");
    std::println("solve: converged {} in {} iterations, {:.0f} ms", solved["converged"].get<bool>(), solved["iterations"].get<int>(), sw.elapsed_ms());
    CHECK(solved["converged"].get<bool>());
    for (const auto& c : solved["constraints"]) {
        INFO(c.dump());
        if (c.contains("last_solve")) CHECK(c["last_solve"]["status"] == "satisfied");
    }

    sw.reset();
    const json built = run(doc, "build");
    std::println("build: ok {} closed {} volume {:.1f} mm3 coverage {:.3f}, {:.0f} ms; deviation p95 {:.4f}", built["ok"].get<bool>(),
                 built["closed"].get<bool>(), built["volume_mm3"].get<double>(), built["scan_coverage"].get<double>(), sw.elapsed_ms(),
                 built["deviation"]["overall"]["p95_mm"].get<double>());
    for (const auto& line : built["log"]) std::println("  {}", line.get<std::string>());
    REQUIRE(built["ok"].get<bool>());
    CHECK(built["closed"].get<bool>());
    CHECK(std::abs(built["volume_mm3"].get<double>() - 57295.95) < 0.005 * 57295.95);
    CHECK(built["scan_coverage"].get<double>() > 0.95);
    CHECK(built["deviation"]["overall"]["p95_mm"].get<double>() < 0.1);

    const auto step = std::filesystem::temp_directory_path() / "einstar_model_test.step";
    const json exported = run(doc, "export_step", {{"path", step.string()}});
    CHECK(exported["read_back"]["valid"].get<bool>());
    CHECK(exported["read_back"]["solids"] == 1);

    // Undo takes back a whole change at once; redo brings it back.
    summary = run(doc, "summary");
    const auto constraints = summary["constraints"].size();
    const json hist = run(doc, "history");
    REQUIRE(hist["undo"].size() >= 3);
    run(doc, "undo");  // build
    run(doc, "undo");  // solve
    for (int i = 0; i < 16; ++i) run(doc, "undo");  // the hole depth, fillet radii, measurements, the bottom
    run(doc, "undo");  // the squaring change: datum and all its constraints
    summary = run(doc, "summary");
    CHECK(summary["datums"].empty());
    CHECK(summary["constraints"].empty());
    for (int i = 0; i < 19; ++i) run(doc, "redo");
    summary = run(doc, "summary");
    CHECK(summary["constraints"].size() == constraints);

    // Saved and opened again: the same document.
    const auto path = std::filesystem::temp_directory_path() / "einstar_model_test.emodel";
    run(doc, "save", {{"path", path.string()}});
    model::Document again;
    run(again, "open", {{"path", path.string()}});
    const json s2 = run(again, "summary");
    CHECK(s2["labels"].size() == summary["labels"].size());
    CHECK(s2["constraints"].size() == summary["constraints"].size());
    CHECK(s2["holes"].size() == summary["holes"].size());
    CHECK(s2["scan"]["triangles"] == summary["scan"]["triangles"]);
    run(again, "solve");
    const json b2 = run(again, "build");
    CHECK(std::abs(b2["volume_mm3"].get<double>() - built["volume_mm3"].get<double>()) < 1.0);
}

TEST_CASE("bad commands fail without changing the document") {
    model::Document doc;
    CHECK(doc.apply("summary", {}).ok);
    CHECK_FALSE(doc.apply("grow", {}).ok);  // no scan
    run(doc, "open_demo");
    const auto revision = doc.revision();
    const model::Outcome a = doc.apply("constraint.add", {{"type", "parallel"}, {"a", "nothing"}, {"b", "else"}});
    CHECK_FALSE(a.ok);
    CHECK(a.error.find("no label") != std::string::npos);
    CHECK_FALSE(doc.apply("label.create", {{"role", "wall"}}).ok);
    CHECK_FALSE(doc.apply("frobnicate", {}).ok);
    CHECK(doc.revision() == revision);
    CHECK(doc.history()["undo"].empty());
}

namespace {

// The hole whose centre is nearest (x, y).
json hole_near(const json& summary, double x, double y) {
    json best;
    double d = 1e9;
    for (const auto& h : summary["holes"]) {
        const double e = std::hypot(h["center"][0].get<double>() - x, h["center"][1].get<double>() - y);
        if (e < d) {
            d = e;
            best = h;
        }
    }
    return best;
}

// (Positions come back rounded to 0.1 um, so distances between them hold to about 2e-4 mm.)
double centre_distance(const json& a, const json& b) {
    return std::hypot(a["center"][0].get<double>() - b["center"][0].get<double>(), a["center"][1].get<double>() - b["center"][1].get<double>(),
                      a["center"][2].get<double>() - b["center"][2].get<double>());
}

// The demo part squared to a datum, for constraint tests.
void squared_demo(model::Document& doc) {
    run(doc, "detect");
    run(doc, "label.update", {{"label", label_below(doc, 10, 8)}, {"name", "top"}});
    run(doc, "label.update", {{"label", label_along(doc, {80, 3, 12}, {-1, 0, 0})}, {"name", "right"}});
    run(doc, "datum.create", {{"name", "part"}, {"z", "top"}, {"x", "right"}});
    run(doc, "square", {{"datum", "part"}});
}

}  // namespace

TEST_CASE("hole positions: pitches between holes hold exactly, a wrong one shows its cost") {
    model::Document doc;
    run(doc, "open_demo");
    squared_demo(doc);
    json s = run(doc, "summary");
    const std::string a = hole_near(s, 42, -19)["name"], b = hole_near(s, 42, 19)["name"], c = hole_near(s, -42, -19)["name"];
    run(doc, "constraint.add", {{"type", "axis_distance"}, {"a", a}, {"b", b}, {"value", 38.0}});
    run(doc, "constraint.add", {{"type", "axis_distance"}, {"a", a}, {"b", c}, {"value", 84.0}});
    const json solved = run(doc, "solve");
    REQUIRE(solved["converged"].get<bool>());
    s = run(doc, "summary");
    for (const auto& con : s["constraints"])
        if (con["constraint"]["type"] == "axis_distance") {
            INFO(con.dump());
            CHECK(con["last_solve"]["status"] == "satisfied");
            CHECK(con["last_solve"]["moves_scan_fit_mm"].get<double>() < 0.05);  // the true pitches
        }
    const auto ha = hole_near(s, 42, -19), hb = hole_near(s, 42, 19), hc = hole_near(s, -42, -19);
    CHECK(std::abs(centre_distance(ha, hb) - 38.0) < 2e-4);
    CHECK(std::abs(centre_distance(ha, hc) - 84.0) < 2e-4);
    std::println("hole centres: ({:.4f}, {:.4f}) ({:.4f}, {:.4f})", ha["center"][0].get<double>(), ha["center"][1].get<double>(),
                 hb["center"][0].get<double>(), hb["center"][1].get<double>());

    // A pitch 0.4 mm off: still held, and it says what it costs (the far hole, held by the other pitches, moves the
    // most).
    run(doc, "constraint.add", {{"type", "axis_distance"}, {"a", c}, {"b", hole_near(s, -42, 19)["name"]}, {"value", 38.4}});
    run(doc, "solve");
    s = run(doc, "summary");
    const json& wrong = s["constraints"].back();
    INFO(wrong.dump());
    CHECK(wrong["last_solve"]["status"] == "satisfied");
    CHECK(wrong["last_solve"]["moves_scan_fit_mm"].get<double>() > 0.15);
    CHECK(wrong["last_solve"]["moves_scan_fit_mm"].get<double>() < 0.45);
    CHECK(std::abs(centre_distance(hole_near(s, -42, -19), hole_near(s, -42, 19)) - 38.4) < 2e-4);
}

TEST_CASE("holes seen only at their rim take part in the solve by their opening") {
    fit::PartSpec spec = fit::flanged_box_spec();
    spec.hole_wall_depth = 0.02;  // only the rims were scanned
    const fit::SyntheticPart part = fit::make_synthetic_part(spec);
    model::Document doc;
    doc.set_scan(part.mesh, model::ScanSource{{}, "demo", 0, 0, {}});
    squared_demo(doc);
    json s = run(doc, "summary");
    REQUIRE(s["holes"].size() >= 4);
    const std::string a = hole_near(s, 42, -19)["name"], b = hole_near(s, 42, 19)["name"];
    CHECK(hole_near(s, 42, -19)["measured_from"] == "opening");
    run(doc, "constraint.add", {{"type", "axis_distance"}, {"a", a}, {"b", b}, {"value", 38.0}});
    const json solved = run(doc, "solve");
    CHECK(solved["converged"].get<bool>());
    s = run(doc, "summary");
    CHECK(s["constraints"].back()["last_solve"]["status"] == "satisfied");
    const auto ha = hole_near(s, 42, -19), hb = hole_near(s, 42, 19);
    CHECK(std::abs(centre_distance(ha, hb) - 38.0) < 2e-4);
    CHECK(std::hypot(ha["center"][0].get<double>() - 42, ha["center"][1].get<double>() + 19) < 0.08);
}

TEST_CASE("hole forms through the document: detected, built, named in the STEP file") {
    const fit::SyntheticPart part = fit::make_synthetic_part(fit::hole_forms_spec());
    model::Document doc;
    doc.set_scan(part.mesh, model::ScanSource{{}, "demo", 0, 0, {}});
    run(doc, "detect");
    json s = run(doc, "summary");
    REQUIRE(s["holes"].size() == 3);
    const auto count_role = [&](const char* role) { return std::ranges::count_if(s["labels"], [&](const json& l) { return l["role"] == role; }); };
    CHECK(count_role("face") == 6);  // the plate's faces; the holes' surfaces belong to the holes
    for (const auto& h : s["holes"]) std::println("  {}", h.dump());
    CHECK(hole_near(s, -18, 0).contains("counterbore_diameter"));
    CHECK(hole_near(s, 0, 0).contains("countersink_diameter"));
    CHECK(hole_near(s, 18, 0).contains("point_angle_deg"));

    const json built = run(doc, "build");
    for (const auto& line : built["log"]) std::println("  {}", line.get<std::string>());
    REQUIRE(built["ok"].get<bool>());
    // The plate less: the bores, the counterbore around one, the countersink's cone, the drilled hole and its point.
    const double pi = std::numbers::pi;
    const double t45 = 1.0, t59 = std::tan(59 * pi / 180);
    const double cs_h = (4.5 - 2.25) / t45;
    const double volume = 60 * 40 * 12 - pi * 3.3 * 3.3 * 12 - pi * (5.5 * 5.5 - 3.3 * 3.3) * 6.5 - pi * 2.25 * 2.25 * 12 -
                          (pi * cs_h / 3 * (4.5 * 4.5 + 4.5 * 2.25 + 2.25 * 2.25) - pi * 2.25 * 2.25 * cs_h) - pi * 2.5 * 2.5 * 8 - pi * 2.5 * 2.5 * (2.5 / t59) / 3;
    std::println("hole forms: volume {:.2f} vs {:.2f} ({:+.3f}%)", built["volume_mm3"].get<double>(), volume, 100 * (built["volume_mm3"].get<double>() / volume - 1));
    CHECK(std::abs(built["volume_mm3"].get<double>() - volume) < 0.004 * volume);

    const auto step = std::filesystem::temp_directory_path() / "einstar_hole_forms.step";
    const json exported = run(doc, "export_step", {{"path", step.string()}});
    CHECK(exported["read_back"]["valid"].get<bool>());
    const brep::StepSummary summary = brep::read_step(step);
    CHECK(summary.surface_kinds.at("cone") >= 2);
    for (const char* form : {"counterbore", "countersink", "point"})
        CHECK(std::ranges::any_of(summary.face_names, [&](const std::string& n) { return n.find(form) != std::string::npos; }));
}

TEST_CASE("the measurements reveal a scan 0.4% too large; scaling corrects it") {
    fit::SyntheticPart part = fit::make_synthetic_part(fit::flanged_box_spec());
    for (auto& v : part.mesh.vertices) v *= 1.004f;  // a scanner whose calibration is a little off
    model::Document doc;
    doc.set_scan(part.mesh, model::ScanSource{{}, "demo", 0, 0, {}});
    squared_demo(doc);
    run(doc, "label.update", {{"label", label_along(doc, {-80, 3, 12}, {1, 0, 0})}, {"name", "left"}});
    json s = run(doc, "summary");
    // What the user measured on the part.
    run(doc, "constraint.add", {{"type", "distance"}, {"a", "right"}, {"b", "left"}, {"value", 60.0}});
    run(doc, "constraint.add", {{"type", "axis_distance"}, {"a", hole_near(s, 42, -19)["name"]}, {"b", hole_near(s, 42, 19)["name"]}, {"value", 38.0}});
    run(doc, "constraint.add", {{"type", "axis_distance"}, {"a", hole_near(s, 42, -19)["name"]}, {"b", hole_near(s, -42, -19)["name"]}, {"value", 84.0}});
    run(doc, "constraint.add", {{"type", "diameter"}, {"hole", hole_near(s, 0, 0)["name"]}, {"value", 8.0}});
    const json solved = run(doc, "solve");
    const json& scale = solved["scale"];
    std::println("scale check: {}", scale.dump());
    REQUIRE(scale.is_object());
    CHECK(std::abs(scale["factor"].get<double>() - 1 / 1.004) < 5e-4);
    CHECK(scale["significant"].get<bool>());

    run(doc, "scale", {{"factor", scale["factor"]}});
    const json again = run(doc, "solve");
    std::println("after scaling: {}", again["scale"].dump());
    CHECK(std::abs(again["scale"]["factor"].get<double>() - 1) < 5e-4);
    CHECK(!again["scale"]["significant"].get<bool>());
    CHECK(std::abs(run(doc, "summary")["scan"]["scale_applied"].get<double>() - scale["factor"].get<double>()) < 1e-12);
}

TEST_CASE("a freeform face: a domed top fitted as a B-spline, built into the solid and exported") {
    const fit::SyntheticPart part = fit::make_synthetic_part(fit::domed_block_spec());
    model::Document doc;
    doc.set_scan(part.mesh, model::ScanSource{{}, "demo", 0, 0, {}});
    run(doc, "label.create", {{"name", "top"}});
    run(doc, "paint", {{"label", "top"}, {"point", {0, 0, 15}}, {"radius", 3.0}});
    Stopwatch sw;
    const json ff = run(doc, "freeform", {{"label", "top"}});
    std::println("freeform: {} triangles, sigma {:.4f} rms {:.4f} mm, grid {} in {:.0f} ms", ff["triangles"].get<int>(), ff["sigma_mm"].get<double>(),
                 ff["rms_mm"].get<double>(), ff["fit"]["control_grid"].dump(), sw.elapsed_ms());
    CHECK(ff["rms_mm"].get<double>() < 0.04);
    // The dome stops at the block's edges: about its footprint's share of the scan.
    const auto top_truth = std::ranges::count(part.triangle_face, std::ranges::find(part.faces, std::string("box0 +z"), &fit::TruthFace::name)->id);
    CHECK(std::abs(ff["triangles"].get<double>() - static_cast<double>(top_truth)) < 0.05 * static_cast<double>(top_truth));

    run(doc, "detect");
    const json s = run(doc, "summary");
    const auto faces = std::ranges::count_if(s["labels"], [](const json& l) { return l["role"] == "face"; });
    CHECK(faces == 6);  // the dome, four sides, the bottom

    const json built = run(doc, "build");
    for (const auto& line : built["log"]) std::println("  {}", line.get<std::string>());
    REQUIRE(built["ok"].get<bool>());
    CHECK(built["closed"].get<bool>());
    const double volume = 50 * 30 * 10 + 4.0 * 25 * 15 * 16 / 9;
    std::println("domed block: volume {:.2f} vs {:.2f} ({:+.3f}%), deviation p95 {:.4f}", built["volume_mm3"].get<double>(), volume,
                 100 * (built["volume_mm3"].get<double>() / volume - 1), built["deviation"]["overall"]["p95_mm"].get<double>());
    CHECK(std::abs(built["volume_mm3"].get<double>() - volume) < 0.003 * volume);
    CHECK(built["deviation"]["overall"]["p95_mm"].get<double>() < 0.1);

    const auto step = std::filesystem::temp_directory_path() / "einstar_domed.step";
    CHECK(run(doc, "export_step", {{"path", step.string()}})["read_back"]["valid"].get<bool>());
    const brep::StepSummary summary = brep::read_step(step);
    CHECK(summary.surface_kinds.contains("other"));  // the B-spline
    CHECK(std::ranges::count(summary.face_names, std::string("top")) == 1);

    // Saved and read back with its control heights.
    const auto path = std::filesystem::temp_directory_path() / "einstar_domed.emodel";
    run(doc, "save", {{"path", path.string()}});
    model::Document again;
    run(again, "open", {{"path", path.string()}});
    CHECK(std::abs(run(again, "build")["volume_mm3"].get<double>() - built["volume_mm3"].get<double>()) < 1e-6);
}

TEST_CASE("sketch-like constraints: fillets tangent to their faces, all of one radius; sides symmetric about the datum") {
    model::Document doc;
    run(doc, "open_demo");
    squared_demo(doc);
    run(doc, "label.update", {{"label", label_along(doc, {-80, 3, 12}, {1, 0, 0})}, {"name", "left"}});
    json s = run(doc, "summary");
    REQUIRE(s["fillets"].size() == 8);
    std::string first;
    for (const auto& f : s["fillets"]) {
        const std::string lab = f["label"];
        run(doc, "constraint.add", {{"type", "tangent"}, {"a", lab}, {"b", f["between"][0]}});
        run(doc, "constraint.add", {{"type", "tangent"}, {"a", lab}, {"b", f["between"][1]}});
        if (first.empty()) first = lab;
        else run(doc, "constraint.add", {{"type", "equal_radius"}, {"a", first}, {"b", lab}});
    }
    run(doc, "constraint.add", {{"type", "symmetric"}, {"a", "right"}, {"b", "left"}, {"datum", "part"}, {"axis", "x"}});
    const json solved = run(doc, "solve");
    REQUIRE(solved["converged"].get<bool>());
    s = run(doc, "summary");
    double worst_cost = 0;
    for (const auto& c : s["constraints"]) {
        const std::string type = c["constraint"]["type"];
        if (type != "tangent" && type != "equal_radius" && type != "symmetric") continue;
        INFO(c.dump());
        CHECK(c["last_solve"]["status"] == "satisfied");
        CHECK(c["last_solve"]["violation"].get<double>() < 1e-9);
        worst_cost = std::max(worst_cost, c["last_solve"]["moves_scan_fit_mm"].get<double>());
    }
    std::println("tangent / equal radius / symmetric: worst cost {:.4f} mm", worst_cost);
    CHECK(worst_cost < 0.1);
    // One radius for all fillets, near the part's 2 mm.
    double r0 = -1;
    for (const auto& l : s["labels"])
        if (l["role"] == "fillet" && l.contains("solved") && l["solved"]["kind"] == "cylinder") {
            const double r = l["solved"]["radius"];
            if (r0 < 0) r0 = r;
            CHECK(std::abs(r - r0) < 1e-9);
        }
    std::println("common fillet radius {:.4f}", r0);
    CHECK(std::abs(r0 - 2.0) < 0.1);
    // The datum's origin moved to the middle between the sides (x = 0 on the part).
    CHECK(std::abs(s["datums"][0]["origin"][0].get<double>()) < 0.02);
}

TEST_CASE("a scan whose source changed is processed again with its modelling carried over") {
    const auto dir = std::filesystem::temp_directory_path() / std::format("einstar_reprocess_{}", ::getpid());
    std::filesystem::create_directories(dir);
    const auto path = dir / "part.stl";
    auto spec = fit::flanged_box_spec();
    REQUIRE(recon::save_mesh(fit::make_synthetic_part(spec).mesh, path));

    model::Document doc;
    run(doc, "open", {{"path", path.string()}});
    squared_demo(doc);
    run(doc, "face.add_plane", {{"name", "bottom"}, {"datum", "part"}, {"axis", "z"}, {"offset", -20}, {"facing", "-"}});
    run(doc, "hole.update", {{"hole", hole_near(run(doc, "summary"), 0, 0)["name"]}, {"diameter", 8.0}});
    const json before = run(doc, "summary");
    CHECK(before["scan"]["source"] == "unchanged");
    REQUIRE(run(doc, "build")["ok"].get<bool>());

    // The scan is processed again at another resolution (another noise draw): every triangle is new.
    spec.voxel_mm = 0.4;
    spec.seed = 2;
    REQUIRE(recon::save_mesh(fit::make_synthetic_part(spec).mesh, path));
    std::filesystem::last_write_time(path, std::filesystem::last_write_time(path) + std::chrono::seconds(2));
    CHECK(run(doc, "summary")["scan"]["source"] == "changed");

    Stopwatch sw;
    const json re = run(doc, "reprocess");
    std::println("reprocess: {} triangles carried in {:.0f} ms", re["carried_triangles"].get<std::size_t>(), sw.elapsed_ms());
    const json after = run(doc, "summary");
    CHECK(after["scan"]["source"] == "unchanged");
    CHECK(after["scan"]["triangles"].get<std::size_t>() > before["scan"]["triangles"].get<std::size_t>());
    CHECK(run(doc, "history")["undo"].empty());
    REQUIRE(after["labels"].size() == before["labels"].size());
    for (std::size_t i = 0; i < before["labels"].size(); ++i) {
        const auto& a = before["labels"][i];
        const auto& b = after["labels"][i];
        CHECK(a["name"] == b["name"]);
        // The same area, in smaller triangles: (0.5 / 0.4)^2 as many, give or take the edges.
        const double ratio = b["triangles"].get<double>() / std::max(1.0, a["triangles"].get<double>());
        INFO(a["name"] << ": " << a["triangles"] << " -> " << b["triangles"]);
        if (a["triangles"].get<int>() > 200) CHECK((ratio > 1.2 && ratio < 1.9));
    }
    for (const auto& r : re["labels"])
        if (r.contains("moved_mm")) CHECK(r["moved_mm"].get<double>() < 0.1);
    CHECK(after["holes"].size() == before["holes"].size());
    CHECK(hole_near(after, 0, 0)["diameter"].get<double>() == 8.0);

    REQUIRE(run(doc, "solve")["converged"].get<bool>());
    const json built = run(doc, "build");
    CHECK(built["ok"].get<bool>());
    CHECK(built["closed"].get<bool>());
    CHECK(run(doc, "deviation")["overall"]["p95_mm"].get<double>() < 0.15);
    std::filesystem::remove_all(dir);
}

TEST_CASE("blocks: a boss the scan never saw added on planes of its own, a pocket cut from it, the rest of the part untouched") {
    model::Document doc;
    run(doc, "open_demo");
    squared_demo(doc);
    run(doc, "face.add_plane", {{"name", "bottom"}, {"datum", "part"}, {"axis", "z"}, {"offset", -20.0}, {"facing", "-"}});
    const double base = run(doc, "build")["volume_mm3"];

    // Planes square to the datum at world coordinates (the datum's axes are the world's here).
    const json datum = run(doc, "summary")["datums"][0];
    const auto plane = [&](const std::string& name, std::size_t axis, double at, const char* facing) {
        const std::string key(1, "xyz"[axis]);
        const double origin = datum["origin"][axis].get<double>();
        REQUIRE(std::abs(datum[key][axis].get<double>()) > 0.999);
        run(doc, "face.add_plane", {{"name", name}, {"datum", "part"}, {"axis", key}, {"offset", at - origin}, {"facing", facing}});
    };
    // A boss 16 x 10 standing 6 mm on the top (z = 20), down into the body to the flange (z = 4) so they overlap.
    plane("boss -x", 0, 8, "-");
    plane("boss +x", 0, 24, "+");
    plane("boss -y", 1, -5, "-");
    plane("boss +y", 1, 5, "+");
    plane("boss top", 2, 26, "+");
    const std::string flange = label_below(doc, 45, 0);
    CHECK(!doc.apply("block.add", {{"faces", {"boss -x", "boss +x"}}}).ok);  // encloses nothing
    run(doc, "block.add", {{"name", "boss"}, {"faces", {"boss -x", "boss +x", "boss -y", "boss +y", "boss top"}},
                           {"bounds", {{{"label", flange}, {"side", "outside"}}}}});
    json built = run(doc, "build");
    std::println("with the boss: {}", built["log"].dump());
    REQUIRE(built["ok"].get<bool>());
    CHECK(built["volume_mm3"].get<double>() == Catch::Approx(base + 16 * 10 * 6).epsilon(1e-4));

    // A pocket 6 x 4 down to z = 15, open at the top: its walls face out of it.
    plane("pocket -x", 0, 12, "-");
    plane("pocket +x", 0, 18, "+");
    plane("pocket -y", 1, -2, "-");
    plane("pocket +y", 1, 2, "+");
    plane("pocket floor", 2, 15, "-");
    run(doc, "block.add", {{"name", "pocket"}, {"faces", {"pocket -x", "pocket +x", "pocket -y", "pocket +y", "pocket floor"}}, {"cut", true}});
    built = run(doc, "build");
    REQUIRE(built["ok"].get<bool>());
    CHECK(built["volume_mm3"].get<double>() == Catch::Approx(base + 16 * 10 * 6 - 6 * 4 * 11).epsilon(1e-4));

    // Saved and read back; a block goes with a face it uses.
    const auto path = std::filesystem::temp_directory_path() / std::format("einstar_blocks_{}.emodel", ::getpid());
    run(doc, "save", {{"path", path.string()}});
    model::Document again;
    run(again, "open", {{"path", path.string()}});
    std::filesystem::remove(path);
    REQUIRE(run(again, "summary")["blocks"].size() == 2);
    CHECK(run(again, "build")["volume_mm3"].get<double>() == Catch::Approx(built["volume_mm3"].get<double>()).epsilon(1e-6));
    CHECK(run(again, "label.delete", {{"label", "pocket floor"}})["blocks_removed"] == 1);
    CHECK(run(again, "build")["volume_mm3"].get<double>() == Catch::Approx(base + 16 * 10 * 6).epsilon(1e-4));
}
