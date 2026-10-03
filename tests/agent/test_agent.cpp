// Agent control (libs/agent): the protocol, the dispatcher, the ImGui inspector and input, against a real
// (headless) ImGui context and a real socket.

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <filesystem>
#include <set>
#include <thread>

#include <imgui.h>

#include "einstar/agent/client.hpp"
#include "einstar/agent/server.hpp"
#include "einstar/core/log.hpp"

using namespace einstar;
using namespace einstar::agent;

namespace {

// A headless ImGui app: two windows, some buttons, a checkbox. Each frame() pumps the server first.
struct TestUi {
    Server server{App::scan, "test"};
    int go_clicks = 0, other_clicks = 0;
    bool flag = false;
    TestUi() {
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize = ImVec2(800, 600);
        io.IniFilename = nullptr;
        io.Fonts->AddFontDefault();
        io.Fonts->Build();
    }
    ~TestUi() { ImGui::DestroyContext(); }
    void frame() {
        server.pump();
        ImGui::GetIO().DeltaTime = 1.0f / 60.0f;
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(10, 10));
        ImGui::SetNextWindowSize(ImVec2(300, 300));
        ImGui::Begin("Panel");
        if (ImGui::Button("Go")) ++go_clicks;
        ImGui::Checkbox("Flag", &flag);
        ImGui::Button("Same##1");
        ImGui::Button("Same##2");
        ImGui::Button("Export (.stl / .ply)");
        ImGui::End();
        ImGui::SetNextWindowPos(ImVec2(400, 10));
        ImGui::SetNextWindowSize(ImVec2(200, 200));
        ImGui::Begin("Other");
        if (ImGui::Button("Go")) ++other_clicks;
        ImGui::End();
        ImGui::Render();
    }
    // A request, answered after pumping frames (at most `max_frames`).
    json call(std::string_view method, json params = json::object(), int max_frames = 200, int* frames_taken = nullptr) {
        auto f = server.submit(request_line(1, method, params));
        for (int i = 0; i < max_frames; ++i) {
            if (f.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                if (frames_taken) *frames_taken = i;
                return json::parse(f.get());
            }
            frame();
        }
        FAIL("no answer to " << method);
        return {};
    }
};

}  // namespace

TEST_CASE("agent protocol: every method has a unique name and an object schema") {
    std::set<std::string> names;
    for (const auto& s : method_specs()) {
        CHECK(names.insert(s.name).second);
        CHECK(s.name.find('.') != std::string::npos);
        CHECK(s.params.value("type", "") == "object");
        CHECK_FALSE(s.description.empty());
    }
    CHECK(find_spec("ui.snapshot") != nullptr);
    CHECK(find_spec("nope.nope") == nullptr);
    // Framing: one line, errors carry their code and data.
    const auto line = response_line(7, error(ErrorCode::ambiguous, "two\nlines", json{{"a", 1}}));
    CHECK(line.find('\n') == line.size() - 1);
    const auto r = parse_response(line);
    REQUIRE(std::holds_alternative<Error>(r));
    CHECK(std::get<Error>(r).code == ErrorCode::ambiguous);
    CHECK(std::get<Error>(r).data["a"] == 1);
    CHECK(std::holds_alternative<Error>(parse_response("not json")));
}

TEST_CASE("agent server: dispatch, errors, settling and the inspector") {
    TestUi ui;
    ui.frame();
    ui.frame();
    // Bad requests are answered, never dropped.
    auto f = ui.server.submit("{nope");
    ui.frame();
    CHECK(json::parse(f.get())["error"]["code"] == static_cast<int>(ErrorCode::parse));
    CHECK(ui.call("no.such")["error"]["code"] == static_cast<int>(ErrorCode::method_not_found));
    const auto calib = ui.call("calib.state");  // spec'd, but for the other app
    CHECK(calib["error"]["message"].get<std::string>().find("calibration app") != std::string::npos);
    CHECK(ui.call("ui.find")["error"]["code"] == static_cast<int>(ErrorCode::invalid_params));
    CHECK_THROWS_AS(ui.server.handle("made.up", [](const json&) -> Server::Outcome { return json(); }), std::logic_error);

    // The inspector: items by window/label, refs, ambiguity with candidates.
    const auto snap = ui.call("ui.snapshot")["result"];
    CHECK(snap["windows"].size() >= 2);
    const auto go = ui.call("ui.find", {{"target", "Panel/Go"}})["result"];
    CHECK(go["window"] == "Panel");
    CHECK(ui.call("ui.find", {{"target", go["ref"]}})["result"]["ref"] == go["ref"]);
    const auto amb = ui.call("ui.find", {{"target", "Go"}})["error"];
    CHECK(amb["code"] == static_cast<int>(ErrorCode::ambiguous));
    CHECK(amb["data"].size() == 2);
    CHECK(ui.call("ui.find", {{"target", "Panel/Same"}})["error"]["code"] == static_cast<int>(ErrorCode::ambiguous));
    CHECK(ui.call("ui.find", {{"target", "Panel/Same##2"}})["result"]["full_label"] == "Same##2");
    CHECK(ui.call("ui.find", {{"target", "Panel/Export (.stl / .ply)"}})["result"]["label"] == "Export (.stl / .ply)");
    const auto missing = ui.call("ui.find", {{"target", "Panel/Gox"}})["error"];
    CHECK(missing["code"] == static_cast<int>(ErrorCode::not_found));

    // Input reaches widgets; a mutating method answers only after the effect has been drawn.
    int frames = 0;
    const auto click = ui.call("input.click", {{"target", "Other/Go"}}, 200, &frames);
    REQUIRE(click.contains("result"));
    CHECK(ui.other_clicks == 1);
    CHECK(ui.go_clicks == 0);
    CHECK(frames >= 4);  // played over frames (move, press, release, ...) and settled
    ui.call("input.click", {{"target", "Panel/Flag"}});
    CHECK(ui.flag);
    CHECK(ui.call("ui.find", {{"target", "Panel/Flag"}})["result"]["checked"] == true);
    CHECK(ui.call("input.key", {{"key", "NoSuchKey"}})["error"]["code"] == static_cast<int>(ErrorCode::invalid_params));
    CHECK(ui.call("input.key", {{"key", "Escape"}, {"modifiers", {"shift"}}}).contains("result"));
    CHECK(ui.call("input.click", {{"x", 1}, {"y", 1}, {"modifiers", {"hyper"}}})["error"]["code"] == static_cast<int>(ErrorCode::invalid_params));

    // Waits.
    const auto before = ui.server.frames();
    ui.call("frame.wait", {{"frames", 5}});
    CHECK(ui.server.frames() >= before + 5);
    CHECK(ui.call("ui.wait_for", {{"target", "Panel/Go"}})["result"]["window"] == "Panel");
    CHECK(ui.call("ui.wait_for", {{"target", "Panel/Never"}, {"timeout_ms", 50}}, 100000)["error"]["code"] ==
          static_cast<int>(ErrorCode::timeout));
}

TEST_CASE("agent server: logs with a cursor, and screenshots") {
    TestUi ui;
    log::info("first line");
    log::warn("second line");
    const auto all = ui.call("app.logs", {{"grep", "line"}})["result"];
    REQUIRE(all["records"].size() >= 2);
    const auto next = all["next_seq"].get<std::uint64_t>();
    log::info("third line");
    const auto more = ui.call("app.logs", {{"since_seq", next}})["result"]["records"];
    REQUIRE(more.size() == 1);
    CHECK(more[0]["message"] == "third line");
    CHECK(ui.call("app.logs", {{"level", "warn"}, {"grep", "line"}})["result"]["records"].size() == 1);

    // A screenshot waits for the next frame the app draws; crop and scale are reported.
    auto f = ui.server.submit(request_line(3, "ui.screenshot", {{"rect", {10, 20, 100, 50}}}));
    for (int i = 0; i < 10 && f.wait_for(std::chrono::seconds(0)) != std::future_status::ready; ++i) {
        ui.frame();
        if (ui.server.capture_wanted()) {
            Frame fr{1600, 1200, 2.0f, std::vector<std::uint8_t>(1600ull * 1200 * 4, 128)};
            ui.server.provide_capture(std::move(fr));
        }
    }
    const auto shot = json::parse(f.get())["result"];
    CHECK(shot["scale"] == 2.0);
    CHECK(shot["image_size"] == json({200, 100}));
    CHECK(shot["logical_rect"] == json({10.0, 20.0, 100.0, 50.0}));
    const auto png = shot["png_base64"].get<std::string>();
    CHECK(png.starts_with("iVBORw0KGgo"));  // "\x89PNG\r\n\x1a\n" in base64
}

TEST_CASE("agent server: over a socket, and busy when the UI thread is stuck") {
    TestUi ui;
    const auto path = (std::filesystem::temp_directory_path() / "einstar_agent_test.sock").string();
    std::string why;
    REQUIRE(ui.server.start(path, why));
    auto c = Client::connect(path);
    REQUIRE(std::holds_alternative<std::unique_ptr<Client>>(c));
    auto& client = *std::get<std::unique_ptr<Client>>(c);
    // The UI thread pumps while a client thread calls (polls counted, not timed).
    auto call_while_pumping = [&](std::string_view method, json params) {
        Result r;
        std::atomic<bool> done{false};
        std::thread caller([&] {
            r = client.call(method, params);
            done = true;
        });
        for (int polls = 0; polls < 20000 && !done; ++polls) {
            ui.frame();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        caller.join();
        return r;
    };
    const auto methods = call_while_pumping("rpc.methods", json::object());
    REQUIRE(std::holds_alternative<json>(methods));
    CHECK(std::get<json>(methods)["app"] == "test");
    // Nobody pumping: the call is answered `busy`, not hung.
    const auto busy = client.call("app.info", {{"_timeout_ms", 200}}, 5000);
    REQUIRE(std::holds_alternative<Error>(busy));
    CHECK(std::get<Error>(busy).code == ErrorCode::busy);
    // The late answer to that call is skipped by the next one.
    const auto info = call_while_pumping("app.info", json::object());
    REQUIRE(std::holds_alternative<json>(info));
    CHECK(std::get<json>(info)["app"] == "test");
    ui.server.stop();
    CHECK_FALSE(std::filesystem::exists(path));
}
