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

const char* kExstarCache = "/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/200x150";

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
    if (!std::filesystem::exists(std::string(kExstarCache) + "/LeftCCF.txt")) SKIP("EXStar calibration cache not installed");
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
    // Disparity at the working range: 175 mm .. 625 mm.
    const double d_near = rect.geometry.disparity_from_depth(175), d_far = rect.geometry.disparity_from_depth(625);
    INFO("disparity range " << d_far << " .. " << d_near << " px, f=" << rect.geometry.f << " cx=" << rect.rectified.cx);
    CHECK(d_far > 200);
    CHECK(d_near < 1100);
}
