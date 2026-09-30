#pragma once

// Packaging real EXStar data as small test fixtures (tests/fixtures/external/README.md): excerpts that
// keep the original bytes, compressed with zstd, and the unpacking the tests use.

#include <cstddef>
#include <filesystem>
#include <span>
#include <vector>

#include "einstar/core/error.hpp"
#include "einstar/fixtures/exstar_project.hpp"

namespace einstar::fixtures {

// Frames of the mustang_differential recording that the test suite reads (original indices).
[[nodiscard]] std::vector<std::size_t> mustang_test_frames();
// Frames whose surface the mesh-agreement test compares with EXStar's STL.
[[nodiscard]] std::vector<std::size_t> mustang_mesh_frames();

// Writes an EXStar project holding only `frames` (indices into `src`, a *.ir_E10_prj or its base
// path): the head region and those frame blocks byte for byte, the calibration snapshot and the XML.
// ExstarProject opens the result like the original; frame i of it is frames[i] of the source.
Result<void> write_project_subset(const std::filesystem::path& src, std::span<const std::size_t> frames,
                                  const std::filesystem::path& out_base);

// Triangles with at least one vertex inside [lo, hi].
[[nodiscard]] Mesh crop(const Mesh& mesh, const Vec3f& lo, const Vec3f& hi);
Result<void> write_stl(const Mesh& mesh, const std::filesystem::path& path);

// Whole-file zstd.
Result<void> compress_file(const std::filesystem::path& in, const std::filesystem::path& out, int level = 19);
Result<void> decompress_file(const std::filesystem::path& in, const std::filesystem::path& out);
// Copies `dir` into `out` (created), decompressing every *.zst to the name without the suffix.
Result<void> unpack_directory(const std::filesystem::path& dir, const std::filesystem::path& out);

}  // namespace einstar::fixtures
