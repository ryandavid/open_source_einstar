#include <catch2/catch_test_macros.hpp>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numeric>

#include "einstar/fixtures/packaging.hpp"

using namespace einstar;
namespace fs = std::filesystem;

namespace {

std::vector<char> slurp(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), {}};
}

// A frame block as the reader indexes it: "BLOCK", u64 size at +8, payload.
std::vector<char> block(std::uint64_t size, char fill) {
    std::vector<char> b(size, fill);
    std::memcpy(b.data(), "BLOCK", 5);
    std::memcpy(b.data() + 8, &size, 8);
    return b;
}

}  // namespace

TEST_CASE("fixture packaging: project subsets keep the original bytes, zstd round trip, STL crop") {
    const fs::path dir = fs::temp_directory_path() / "einstar_test_packaging";
    fs::remove_all(dir);
    fs::create_directories(dir / "src");
    const auto b0 = block(0x40, 'a'), b1 = block(0x80, 'b'), b2 = block(0x60, 'c');
    {
        std::ofstream f(dir / "src/P.data_base", std::ios::binary);
        std::vector<char> head(0xA00000, 0);
        std::memcpy(head.data() + 8, "\x20\0\0\0\0\0\0\0", 8);  // head block size (no marker sections)
        for (const std::vector<char>* b : {static_cast<const std::vector<char>*>(&head), &b0, &b1, &b2}) f.write(b->data(), static_cast<std::streamsize>(b->size()));
        std::ofstream(dir / "src/P.data_cm", std::ios::binary) << std::string(692, '\1');
        std::ofstream(dir / "src/P.ir_E10_prj") << "<project/>";
    }
    auto src = fixtures::ExstarProject::open(dir / "src/P.ir_E10_prj");
    REQUIRE(src.has_value());
    REQUIRE((*src)->frame_count() == 3);

    const std::size_t pick[] = {2, 0};
    REQUIRE(fixtures::write_project_subset(dir / "src/P.ir_E10_prj", pick, dir / "Q").has_value());
    auto sub = fixtures::ExstarProject::open(dir / "Q.ir_E10_prj");
    REQUIRE(sub.has_value());
    REQUIRE((*sub)->frame_count() == 2);
    const auto bytes = slurp(dir / "Q.data_base");
    REQUIRE(bytes.size() == 0xA00000 + b2.size() + b0.size());
    CHECK(std::equal(b2.begin(), b2.end(), bytes.begin() + 0xA00000));
    CHECK(std::equal(b0.begin(), b0.end(), bytes.begin() + 0xA00000 + static_cast<std::ptrdiff_t>(b2.size())));
    CHECK(slurp(dir / "Q.data_cm") == slurp(dir / "src/P.data_cm"));

    // Packed (zstd) and unpacked again: identical files; the 10 MB zero head compresses to almost nothing.
    fs::create_directories(dir / "packed");
    for (const char* ext : {".data_base", ".data_cm", ".ir_E10_prj"})
        REQUIRE(fixtures::compress_file(dir / (std::string("Q") + ext), dir / "packed" / (std::string("Q") + ext + ".zst")).has_value());
    CHECK(fs::file_size(dir / "packed/Q.data_base.zst") < 4096);
    REQUIRE(fixtures::unpack_directory(dir / "packed", dir / "unpacked").has_value());
    CHECK(slurp(dir / "unpacked/Q.data_base") == bytes);
    auto reopened = fixtures::ExstarProject::open(dir / "unpacked/Q");
    REQUIRE(reopened.has_value());
    CHECK((*reopened)->frame_count() == 2);
    CHECK_FALSE(fixtures::decompress_file(dir / "Q.data_cm", dir / "not_zstd").has_value());

    // STL: crop keeps triangles touching the box; write + load round trip.
    fixtures::Mesh mesh;
    for (int t = 0; t < 4; ++t) {
        const float x = 10.0f * static_cast<float>(t);
        for (const Vec3f& v : {Vec3f(x, 0, 0), Vec3f(x + 1, 0, 0), Vec3f(x, 1, 0)}) mesh.vertices.push_back(v);
    }
    const auto cropped = fixtures::crop(mesh, Vec3f(-1, -1, -1), Vec3f(12, 2, 1));
    CHECK(cropped.vertices.size() == 6);
    REQUIRE(fixtures::write_stl(cropped, dir / "m.stl").has_value());
    auto loaded = fixtures::load_stl(dir / "m.stl");
    REQUIRE(loaded.has_value());
    CHECK(loaded->vertices == cropped.vertices);
    fs::remove_all(dir);
}

TEST_CASE("the mustang fixture excerpt holds every frame the tests read") {
    const auto frames = fixtures::mustang_test_frames();
    for (const auto f : {0uz, 1uz, 2uz, 42uz, 476uz, 1500uz, 6000uz}) CHECK(std::ranges::binary_search(frames, f));
    CHECK(frames.size() == 137);
}
