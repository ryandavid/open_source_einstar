#pragma once

// Scans in, and the project file.
//
// A project (.emodel) refers to its scan's source (an .estr recording, or an STL/PLY mesh) and keeps a copy
// of the mesh it was modelled on: labels point at that mesh's triangles, so a later change to processing
// cannot move them. The file is a sequence of tagged, length-prefixed records like an .estr (readers skip
// tags they do not know): SCAN (source, JSON), MESH (vertices, normals, triangles; zstd), LABL (paint and
// region per triangle; zstd), DOCU (labels, holes, fillets, datums, constraints, solve report; JSON).

#include <filesystem>
#include <utility>

#include "einstar/core/error.hpp"
#include "einstar/model/document.hpp"

namespace einstar::model {

// A mesh from an .stl (binary or ASCII), a .ply (as recon::save_mesh writes it, or ASCII) or an .estr
// (processed, as the scanning app's Process step does; `fine`: its 0.3 mm voxels).
[[nodiscard]] Result<std::pair<recon::TriangleMesh, ScanSource>> load_scan_mesh(const std::filesystem::path& path, bool fine = false);

}  // namespace einstar::model
