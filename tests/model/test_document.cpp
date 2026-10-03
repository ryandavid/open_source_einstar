#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <print>
#include <set>

#include "einstar/agent/protocol.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/model/document.hpp"

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
        if (s.app == agent::App::model && s.name != "model.view") specs.insert(s.name.substr(std::string("model.").size()));  // view: the app's
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
