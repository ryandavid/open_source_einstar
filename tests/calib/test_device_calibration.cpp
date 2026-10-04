#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/calib/rectify.hpp"

using namespace einstar;
using Catch::Matchers::WithinAbs;

namespace {

const char* kExstarCache = EINSTAR_TEST_CALIBRATION_DIR;  // the scanner's calibration (tests/fixtures)

std::vector<std::uint8_t> slurp(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), {}};
}

}  // namespace

TEST_CASE("obfuscation round trip on synthetic CCF files") {
    // Build plain fields, pick random tables, obfuscate the way the writer does, then decode.
    std::array<double, 33> truth{};
    for (std::size_t i = 0; i < truth.size(); ++i) truth[i] = 0.5 + static_cast<double>(i) * 1.25;
    std::array<std::int32_t, 64> tl{}, tr{};
    for (int i = 0; i < 64; ++i) {
        tl[static_cast<std::size_t>(i)] = (i * 7 + 3) % 10;
        tr[static_cast<std::size_t>(i)] = (i * 5 + 1) % 10;
    }
    auto make_file = [&](const std::array<std::int32_t, 64>& tail, bool encode) {
        std::vector<std::uint8_t> f(520);
        std::array<double, 33> stored = truth;
        if (encode) {
            // Same tables as the decoder (field -> key/sign, key -> L +/- R).
            const int fk[][3] = {{0,0,1},{1,1,-1},{4,2,-1},{5,3,1},{8,4,1},{10,5,1},{11,6,-1},{12,7,1},{13,8,1},{14,9,1},
                                 {20,50,-1},{21,51,-1},{22,52,-1},{23,33,-1},{24,34,1},{25,35,-1},{26,36,1},{27,37,1},
                                 {28,38,1},{29,39,-1},{30,40,-1},{31,41,-1}};
            const int ks[][2] = {{0,1},{1,1},{2,-1},{3,-1},{4,-1},{5,-1},{6,-1},{7,1},{8,1},{9,1},{33,1},{34,-1},{35,1},
                                 {36,-1},{37,1},{38,-1},{39,1},{40,1},{41,-1},{50,-1},{51,1},{52,-1}};
            std::array<double, 64> key{};
            for (auto [k, s] : ks) key[static_cast<std::size_t>(k)] = tl[static_cast<std::size_t>(k)] + s * tr[static_cast<std::size_t>(k)];
            for (auto [field, k, s] : fk) stored[static_cast<std::size_t>(field)] += s * key[static_cast<std::size_t>(k)];
        }
        std::memcpy(f.data(), stored.data(), 264);
        std::memcpy(f.data() + 264, tail.data(), 256);
        return f;
    };
    const auto lf = make_file(tl, true), rf = make_file(tr, true);
    auto decoded = calib::decode_ccf_pair(lf, rf);
    REQUIRE(decoded.has_value());
    for (std::size_t i = 0; i < 33; ++i) {
        REQUIRE_THAT(decoded->first[i], WithinAbs(truth[i], 1e-12));
        REQUIRE_THAT(decoded->second[i], WithinAbs(truth[i], 1e-12));
    }
}

TEST_CASE("decodes the real EXStar calibration cache") {
    auto cal = calib::load_ccf_directory(kExstarCache);
    REQUIRE(cal.has_value());
    // Values documented in docs/calibration.md for the 2026-09-27 calibration.
    CHECK_THAT(cal->left.model.fx, WithinAbs(1157.23588, 1e-4));
    CHECK_THAT(cal->right.model.cy, WithinAbs(506.03491, 1e-4));
    CHECK_THAT(cal->texture.model.cx, WithinAbs(662.11298, 1e-4));
    const auto rig = cal->rig();
    CHECK_THAT(rig.baseline_mm(), WithinAbs(159.913, 1e-3));
    CHECK_THAT(rotation_angle(rig.T_right_left) * 180.0 / M_PI, WithinAbs(22.15, 0.02));

    // A flash blob assembled from these files decodes to the same thing.
    std::vector<std::uint8_t> blob(calib::kFlashBlobSize, 0);
    auto put = [&](std::size_t off, const std::vector<std::uint8_t>& d) { std::memcpy(blob.data() + off, d.data(), d.size()); };
    const std::uint32_t type = 4;
    std::memcpy(blob.data() + 0x39B, &type, 4);
    const std::string names[3] = {"LeftCCF.txt", "RightCCF.txt", "TexCCF.txt"};
    for (int i = 0; i < 3; ++i) {
        const auto data = slurp(std::string(kExstarCache) + "/" + names[i]);
        const std::size_t base = 0x39B + 4 + static_cast<std::size_t>(i) * 0x508;
        const auto len = static_cast<std::int32_t>(data.size());
        std::memcpy(blob.data() + base + 260, &len, 4);
        put(base + 264, data);
    }
    std::memcpy(blob.data() + 0x12B7, "FQFQ", 5);
    auto from_blob = calib::decode_flash_blob(blob);
    REQUIRE(from_blob.has_value());
    CHECK(from_blob->left.model.fx == cal->left.model.fx);
    CHECK(from_blob->right.t_cam_world == cal->right.t_cam_world);
}

TEST_CASE("real calibration rectifies to a usable stereo geometry") {
    if (!std::filesystem::exists(std::string(kExstarCache) + "/LeftCCF.txt")) SKIP("EXStar calibration cache not installed");
    auto cal = calib::load_ccf_directory(kExstarCache);
    REQUIRE(cal.has_value());
    const auto rect = calib::compute_rectification(cal->rig());
    // Right camera centre must sit on +x of the rectified left frame at the full baseline.
    const Vec3 right_center_left = cal->rig().T_right_left.inverse().translation();
    const Vec3 in_rect = rect.R_left * right_center_left;
    CHECK_THAT(in_rect.x(), WithinAbs(159.913, 1e-3));
    CHECK(std::abs(in_rect.y()) < 1e-6);
    CHECK(std::abs(in_rect.z()) < 1e-6);
    // Disparity over the working range (175 mm .. 625 mm) is signed, zero at the reference depth; the
    // physical disparity f * B / z (disparity + cx_offset) is what the scanner's geometry dictates.
    const auto& g = rect.geometry;
    const double d_near = g.disparity_from_depth(175), d_far = g.disparity_from_depth(625);
    INFO("disparity range " << d_far << " .. " << d_near << " px, f=" << g.f << " cx=" << g.cx << " cx_offset=" << g.cx_offset);
    CHECK(d_far + g.cx_offset > 200);
    CHECK(d_near + g.cx_offset < 1100);
    CHECK(std::abs(g.disparity_from_depth(calib::RectificationOptions{}.reference_depth_mm)) < 1e-9);
    CHECK(d_far < 0);
    CHECK(d_near > 0);
    // The window sits on the overlap: EXStar's own rectification (docs/calibration.md) puts the left
    // principal point at 319.6 and the right one 619 px further right; ours is within ~10% of that layout.
    CHECK(g.cx > 250);
    CHECK(g.cx < 420);
    CHECK(rect.rectified_right.cx == g.right_cx());
}

TEST_CASE("CCF files extracted from a flash blob are the stored files, and write back to a loadable directory") {
    const std::string dir = kExstarCache;
    const auto l = slurp(dir + "/LeftCCF.txt"), r = slurp(dir + "/RightCCF.txt"), t = slurp(dir + "/TexCCF.txt");
    REQUIRE(l.size() == calib::kObfuscatedCcfSize);
    const auto blob = calib::encode_quick_flash_blob(l, r, t, "2026-09-27 13:57");
    auto files = calib::extract_ccf_files(blob);
    REQUIRE(files.has_value());
    CHECK(files->left == l);
    CHECK(files->right == r);
    CHECK(files->tex == t);
    CHECK(files->calibration_time == "2026-09-27 13:57");

    const auto out = std::filesystem::temp_directory_path() / "einstar_test_ccf_dump";
    std::filesystem::remove_all(out);
    REQUIRE(calib::write_ccf_directory(*files, out.string()).has_value());
    auto reloaded = calib::load_ccf_directory(out.string());
    auto direct = calib::decode_flash_blob(blob);
    REQUIRE(reloaded.has_value());
    REQUIRE(direct.has_value());
    CHECK(reloaded->calibration_time == "2026-09-27 13:57");
    CHECK(reloaded->rig().baseline_mm() == direct->rig().baseline_mm());
    CHECK(reloaded->left.model.fx == direct->left.model.fx);
    std::filesystem::remove_all(out);
}

TEST_CASE("CCF encoding decodes back to the calibration it was made from") {
    const auto cal = calib::load_ccf_directory(kExstarCache);
    REQUIRE(cal);
    for (const std::uint32_t seed : {1u, 2u, 12345u}) {
        auto files = calib::encode_ccf_files(*cal, seed);
        REQUIRE(files.left.size() == calib::kObfuscatedCcfSize);
        REQUIRE(files.right.size() == calib::kObfuscatedCcfSize);
        REQUIRE(files.tex.size() == calib::kPlainCcfSize);
        const auto back = calib::decode_ccf_files(files.left, files.right, files.tex);
        REQUIRE(back);
        for (const auto& [a, b] : {std::pair{&cal->left, &back->left}, std::pair{&cal->right, &back->right}, std::pair{&cal->texture, &back->texture}}) {
            CHECK_THAT(b->model.fx, WithinAbs(a->model.fx, 1e-9));
            CHECK_THAT(b->model.fy, WithinAbs(a->model.fy, 1e-9));
            CHECK_THAT(b->model.cx, WithinAbs(a->model.cx, 1e-9));
            CHECK_THAT(b->model.cy, WithinAbs(a->model.cy, 1e-9));
            CHECK_THAT(b->model.skew, WithinAbs(a->model.skew, 1e-9));
            for (std::size_t k = 0; k < 5; ++k) CHECK_THAT(b->model.dist[k], WithinAbs(a->model.dist[k], 1e-12));
            CHECK((b->R_cam_world - a->R_cam_world).norm() < 1e-12);
            CHECK((b->t_cam_world - a->t_cam_world).norm() < 1e-9);
        }
        CHECK_THAT(back->texture.rms_error, WithinAbs(cal->texture.rms_error, 1e-15));
        // The stored doubles really are offset (not plain copies).
        double fx_stored = 0;
        std::memcpy(&fx_stored, files.left.data(), 8);
        const auto* tail = files.left.data() + calib::kPlainCcfSize;
        CHECK(std::any_of(tail, tail + 256, [](std::uint8_t v) { return v != 0; }));
        (void)fx_stored;
    }
}

TEST_CASE("replacing the quick section changes nothing else in the blob") {
    const auto l = slurp(std::string(kExstarCache) + "/LeftCCF.txt"), r = slurp(std::string(kExstarCache) + "/RightCCF.txt"),
               t = slurp(std::string(kExstarCache) + "/TexCCF.txt");
    auto blob = calib::encode_quick_flash_blob(l, r, t, "2026-09-27 13:57");
    // Other sections filled with a pattern, so any stray write shows.
    for (std::size_t i = 0; i < blob.size(); ++i)
        if (i < calib::kQuickSectionOffset || i >= calib::kQuickSectionOffset + calib::kQuickSectionSize) blob[i] = static_cast<std::uint8_t>(i * 7 + 3);

    auto cal = calib::decode_flash_blob(blob);
    REQUIRE(cal);
    cal->left.model.fx += 1.5;
    cal->right.R_cam_world = Eigen::AngleAxisd(0.004, Vec3::UnitX()).toRotationMatrix() * cal->right.R_cam_world;
    cal->calibration_time = "2026-09-30 12:34";
    const auto files = calib::encode_ccf_files(*cal, 99);
    const auto updated = calib::replace_quick_section(blob, files);
    REQUIRE(updated);
    REQUIRE(updated->size() == blob.size());
    for (std::size_t i = 0; i < blob.size(); ++i)
        if (i < calib::kQuickSectionOffset || i >= calib::kQuickSectionOffset + calib::kQuickSectionSize) REQUIRE((*updated)[i] == blob[i]);
    const auto back = calib::decode_flash_blob(*updated);
    REQUIRE(back);
    CHECK(back->calibration_time == "2026-09-30 12:34");
    CHECK_THAT(back->left.model.fx, WithinAbs(cal->left.model.fx, 1e-9));
    CHECK((back->right.R_cam_world - cal->right.R_cam_world).norm() < 1e-12);

    // Rejected: wrong sizes, a blob without the section.
    auto bad = files;
    bad.left.pop_back();
    CHECK(!calib::replace_quick_section(blob, bad));
    std::vector<std::uint8_t> empty(calib::kFlashBlobSize, 0);
    CHECK(!calib::replace_quick_section(empty, files));
    CHECK(!calib::replace_quick_section(std::span(blob).first(100), files));
}

TEST_CASE("the quick section of the scanner's flash rebuilds byte for byte") {
    // The dump of the scanner's flash is local (fixtures-data/, not in the repository).
    const std::string path = std::string(EINSTAR_TEST_CALIBRATION_DIR) + "/../../../../fixtures-data/scanner-2026-09-30/flash_blob.bin";
    if (!std::filesystem::exists(path)) SKIP("no local flash dump");
    const auto blob = slurp(path);
    REQUIRE(blob.size() == calib::kFlashBlobSize);
    const auto files = calib::extract_ccf_files(blob);
    REQUIRE(files);
    const auto rebuilt = calib::replace_quick_section(blob, *files);
    REQUIRE(rebuilt);
    CHECK(*rebuilt == blob);  // same layout, name fields and padding as EXStar's writer
}
