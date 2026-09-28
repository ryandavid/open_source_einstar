# EXStar project format (Einstar, EXStar 1.2.2.0)

Status: reverse-engineered for interoperability, read-only, from the user's three
projects in `~/Documents/EXStar/` and from static analysis of
`libSn3DFile.dylib` (EXStar macOS arm64 build, 2024-07; Ghidra project folder
`/einstar`). This is a clean-room description. The tables and prose restate the
formats in our own words, and no vendor code is reproduced.

Confidence tags: **[H]** means verified against data or read directly from code.
**[M]** means strongly inferred. **[L]** means a guess.

All numbers are little-endian. Lengths are in mm.

---

## 0. Summary for implementers

* A project is an XML descriptor (`*.ir_E10_prj`) plus four binary files that
  share its base name: `.data_base`, `.data_cm`, `.data_fusion` and `.data_tb`.
* **`.data_base`** holds everything per frame. For each frame it stores:
  * a **640×512 float32 depth map (Z in mm)** in the *rectified left IR camera*
    at half resolution;
  * a per-pixel "deleted" flag byte;
  * the **camera→world pose** as 12 doubles (a 3×3 rotation stored
    column-major, then t);
  * the frame id;
  * optionally, the frame's marker observations (camera frame) with global ids.
* Unproject a depth pixel `(u, v)` (integer column/row, no half-pixel offset) as
  `X = (u−cx)·Z/fx`, `Y = (v−cy)·Z/fy`, `Z`. Then `p_world = R·X + t`.
  Use only pixels with `Z > 0` and flag == 0.
  * The depth section stores `fx, fy, cx, cy`. In one project they are all
    zero. In that case use half of the rectified values in `.data_cm`
    (`cx = cm[0]/2, cy = cm[1]/2, f = cm[2]/2`).
* Validation: 20 frames spread over the 6055-frame `mustang_differential`
  project give **median 0.128 mm and RMS 0.235 mm** point-to-mesh distance
  against EXStar's own STL, with no trimming (details in §7).
* Every stored frame has a valid pose [H]. Frames lost during tracking are not
  stored [M]. Frame ids are consecutive, starting at 1 [H].
* `.data_fusion` is EXStar's fused point cloud (points, normals, weights).
* `.data_tb` holds a denser point cloud with normals. The `.asc` exports are
  bit-for-bit copies of its point and normal arrays. `.data_cm` is the
  calibration snapshot (see `calibration.md` §3.5, with the correction in §4
  below).
* No `.p3` (`_global_mark.p3`, `_frameMarker.p3`) or `.rge` files exist in any of
  the three projects (§6).

---

## 1. Directory layout and XML files

```
<group>/                          e.g. mustang_differential/, ProjectGroup_2/
  <group>.sln_E10_ir              solution/group XML
  ProjectN.ir_E10_prj             project XML
  ProjectN.data_base              per-frame data (GBs)
  ProjectN.data_cm                calibration snapshot (692 B)
  ProjectN.data_fusion            fused cloud
  ProjectN.data_tb                dense surface cloud
  *.stl / *.asc                   user exports (optional, anywhere)
```

**`.sln_E10_ir`** [H]: `<solution>` with `version` (0.1), `point_dis` (0.5),
`scan_mode` (2), `haveTexture`, `scan_option`, and a list of `<project>`. Each
project has a `path` to its prj, an `rt` block and a `groupId`. The `rt` block
holds `rot00..rot22` and `tran00..tran02`. It is identity in all our data. It is
presumably the placement of each project within the group (multi-project
alignment) [M].

**`.ir_E10_prj`** [H]: `<PROJECT>` with these fields:
* `FRAMES`: equals the number of frame blocks in `.data_base`.
* `POINTS`: close to, but not equal to, the `.data_fusion` point count.
  * mustang: 553397 vs 550344.
* `DIS`: point distance in mm.
* `OBJECT_ALIGN_TYPE`:
  * 1 = mustang, no markers;
  * 8 = PG2/Project1, markers used;
  * 5 = PG2/Project2, hybrid/feature.
  * These meanings are [M] from the data.
* `GENERATE_CLOUD`, `PROJECT_TYPE`, `SERIALCODE`, `FIRMWARE`, `CALIBTIME`, …

---

## 2. `.data_base`

### 2.1 File layout [H]

```
0x000000  BLOCK  "head" block (type UELaserHeaderV1), ~0.8–8 KB
          zero padding up to 0xA00000 (10 MiB reserved for header growth)
0xA00000  BLOCK  frame 0 (type HandleFrameInfoV4)
          BLOCK  frame 1          (blocks are contiguous, no padding)
          ...
          BLOCK  frame N-1        (end of last block == file size)
```

Walk it by starting at 0xA00000 and advancing by each block's `size` field.
In all three projects the last block ends exactly at EOF.

* The mustang frames are all exactly 1,638,817 bytes because they carry no
  markers. So `file size = 0xA00000 + 6055 × 1638817`.
* Frames in marker projects grow by 118 bytes per marker.

### 2.2 Generic block ("BLOCK") [H]

| Offset | Size | Field |
|---|---|---|
| 0x00 | 5 | ASCII `BLOCK` |
| 0x05 | 3 | block version major, minor, patch (u8 each). Head = 1.0.0; frame = 4.0.0 |
| 0x08 | u64 | total block size in bytes, header included |
| 0x10 | u32 + u32 pad | CRC-32 (standard zlib/IEEE, reflected, init and xorout 0xFFFFFFFF) of bytes `[0x20, size)` |
| 0x18 | u32 + u32 pad | type key = CRC-32 of the C++ mangled type name: `N10ProjParser15UELaserHeaderV1E` → 0x52BB5F73 (head), `N10ProjParser17HandleFrameInfoV4E` → 0x72AF89E1 (frame) |
| 0x20 | u8 | always 0xBC in our files (meaning unknown) [L: constant/magic] |
| 0x21 | u64[n] | section offset table. Each entry is the byte offset of section *k*, **relative to block+0x20**. The first entry equals `1 + 8n`, so `n = (entry0 − 1)/8`. The head block has n = 10 and frame blocks have n = 11 |
| … | | sections back-to-back. Section *k* runs to the start of section *k+1*, and the last one runs to the block end |

The CRC matches for head blocks. It does **not** match for any frame block we
tested, even after zeroing the deletion flags. Frames are rewritten in place
after capture (poses optimized, flags set), so the CRC is apparently left stale.
A reader must not reject frames on CRC. [H for the mismatch, M for the reason]

Each section begins with a **u64 "member key"**: the byte offset of the
corresponding member inside the C++ object (0x08, 0x18, …). This makes sections
self-identifying. Look sections up by key, not by position. [H]

### 2.3 Head block (UELaserHeaderV1, block version 1.0.0) [H layout, M names]

In the head, sections are *not* optional. Each is `u64 key` followed directly by
the value.

| Key | Value encoding | Content |
|---|---|---|
| 0x008 | u64 len + chars | creation timestamp, e.g. `2024-11-16 11:40:02` |
| 0x020 | u64 len + chars | string, empty in all files |
| 0x038 | u64 len + chars | string, empty |
| 0x050 | u64 len + chars | string, empty |
| 0x068 | u64 | frame count (equals `FRAMES`) |
| 0x070 | f64[40] | 10×4 matrix, all zero (laser-plane parameters for laser models) [M] |
| 0x1B0 | f64[24] | 6×4 matrix, all zero [M] |
| 0x270 | u64 count + count × MarkerRecord | global marker map, list A |
| 0x288 | u64 count + count × MarkerRecord | global marker map, list B |
| 0x2A0 | u64 count + count × i32 | int list, empty in all files |

Marker lists A and B:
* PG2/Project1 has 31 markers in both, and the lists are identical.
* PG2/Project2 has 25 markers in B only.
* mustang has none.
* The library has separate `readGlobalMarkers` and `readTotalMarkers`
  functions. Which list is which is [L].

### 2.4 MarkerRecord (118 bytes, packed) [H layout, M meanings]

| Offset | Type | Meaning |
|---|---|---|
| 0 | f64[3] | marker centre (mm). Per-frame records are in the frame's camera coordinates. Header records are in world coordinates |
| 24 | f64[3] | unit normal |
| 48 | f64 | per-frame records: 1.0. Global records: number of observations (e.g. 661, 689) [M] |
| 56 | f64 | diameter, 6.0 mm |
| 64 | f64 | 1.0 usually. Some global markers in PG2/P2 hold 6.29–6.82 (measured diameter? quality?) [L] |
| 72 | i32 | marker id. In frames this is the id of the matched global marker [H] |
| 76 | i32 | −1 in all records [L: code id / group] |
| 80 | u8, u8 | 0, 0 |
| 82 | i32 | 0 |
| 86 | 16 bytes | zero |
| 102 | 16 bytes | zero |

Check [H]:
* PG2/Project1: transforming a frame's markers by the frame pose lands on the
  global marker with the same id, 0.05–0.21 mm away.
* PG2/Project2: 0.2–0.7 mm, and one id disagreed with the nearest global marker
  in frame 1.

### 2.5 Frame block (HandleFrameInfoV4, block version 4.0.0) [H]

Sections are `u64 key`, `u32 present` (0/1), then the value if present. An absent
section is just 12 bytes.

| Key | Member type | Present in our data | Payload when present |
|---|---|---|---|
| 0x08 | PointImage (organized point image) | never | – (a separate deserializer exists) |
| 0x18 | CloudWithWeight | never | – |
| 0x28 | vector of MarkerRecord | PG2 projects (every frame, count may be 0) | u64 count, then count × 118 B |
| 0x38 | RigidMatrix (pose) | **always** | f64[12], see §2.6 |
| 0x48 | RealTexData (texture image) | never | – |
| 0x58 | TextureFeatureParam | never | – |
| 0x68 | float (point distance) | never | – |
| 0x70 | int (frame id) | always | i32, 1-based, consecutive |
| 0x80 | clip-plane data | never | – |
| 0x90 | DepthInfo | never | – |
| 0xA0 | **DepthImage** | always | see §2.7 |

The serializer supports several "anchor" subsets of these members, so other
EXStar versions or modes may populate PointImage, CloudWithWeight or texture.
Readers should dispatch on the key and the present flag. [H]

Size check for a marker-less frame:
`32 (hdr) + 89 (table) + 8×12 (absent) + 108 (RT) + 16 (id) + 1638476 (depth) = 1638817`.

### 2.6 Pose (key 0x38) [H]

The value is 12 doubles:
* `a[0..8]` is the rotation `R_cw` in **column-major** order (Eigen default).
  So `R[row][col] = a[col*3 + row]`.
* `a[9..11]` is the translation `t` in mm.
* The pose maps **camera → world**: `p_world = R_cw · p_cam + t`.
* So `t` is the camera centre in world coordinates.
* The camera frame is the frame the depth map lives in: the rectified left IR
  camera.

Checks:
* All 6055 mustang rotations are orthonormal (error < 2e-15) with det = +1.
* The alternative interpretations (row-major, or inverse) put the points
  hundreds of mm away from the model.
* The world frame is the first frame's camera when there are no markers:
  frame 0 is exactly identity in mustang and PG2/Project2.
* With a marker map (PG2/Project1) the world frame is the marker map's frame,
  and frame 0 is not identity.

### 2.7 DepthImage (key 0xA0) [H]

| Offset (in value) | Type | Content |
|---|---|---|
| 0 | i32, i32 | width = 640, height = 512 |
| 8 | f64 ×4 | fx, fy, cx, cy for the 640×512 grid. mustang: 578.976, 578.976, 160.417, 257.751. PG2/Project2: 579.148, 579.148, 159.799, 257.378. **PG2/Project1: all zero** (see below) |
| 40 | u64 | N = w·h = 327680 |
| 48 | f32[N] | depth Z (mm) row-major (row = v, column = u). 0 = no data. Typical 200–320 mm |
| 48+4N | u64 | M, always 0 in files. A second float array that is never written [H] |
| 56+4N | u64 | K = w·h |
| 64+4N | u8[K] | per-pixel **VertexFlag**. 0 = keep, 1 = deleted (removed by background, clip or edit before fusion) |

What the depth values are [H]:
* The value is the **Z coordinate** (distance along the optical axis), not the
  range along the ray.
  * Treated as Z: median 0.13–0.20 mm to the mesh.
  * Treated as range: median 8–14 mm.
* The intrinsics are **exactly half** of the rectified full-resolution camera in
  `.data_cm`. mustang: `cm = (cx 320.833, cy 515.502, f 1157.953)`, and ÷2 gives
  the header values.
* Pixel convention: `u_half = u_full / 2`, with integer pixel indices and no
  0.5 offset. Shifting cx and cy by ±0.5 or ±1 px worsened the fit
  (RMS 0.228 → 0.245–0.32 mm).
* This matches the 640×512 "every second rectified-left pixel" point image
  described in `algorithms.md` §1.7.
* PG2/Project1 stores zeros for fx..cy. Substituting `.data_cm`/2 gives a fit of
  the same quality as the other projects, so a reader should fall back to
  `.data_cm` when fx == 0.

The deletion flags:
* Flagged pixels are exactly the points that are missing from the fused cloud
  and the STL. In frame 2549, 99.8% of the points more than 1 mm from the STL
  were flagged, against 1% of the inliers.
* Totals: mustang 27.5 M of 483.7 M valid pixels are flagged, PG2/P1 has 0, and
  PG2/P2 has 30.1 M of 85.3 M.
* The count of valid, unflagged pixels over all frames is stored in `.data_cm`
  (§4).

Organized grid vs point list: the data is an **organized depth map**. There is
no per-point normal or weight on disk. Those were not persisted: the
CloudWithWeight and PointImage sections are absent.

---

## 3. `.data_fusion` (fused cloud) [H]

The layout comes from `CloudFile::read`/`write` in libSn3DFile, and the whole
file parses with 0 bytes left over in all three projects.

| Offset | Type | Content |
|---|---|---|
| 0 | u32 | frame count (6055 / 2057 / 2025) |
| 4 | u32 | point count P |
| 8 | u32 | 0 [L] |
| 12 | f32 | max weight (equals max of the array below) |
| 16 | i32 | W = P |
| 20 | f32[W] | per-point fusion weight (0.2 … 1698) |
| … | i32 | −1 (cloud field) [L] |
| … | u8 | 0 (cloud flag) [L] |
| … | i32 n, f32[3n] | points (world, mm) |
| … | i32 n, f32[3n] | unit normals |
| … | i32 n, f32[3n] | colours, all zero (no texture) |

Point counts are 550344 (mustang), 277360 (PG2/P1) and 61270 (PG2/P2). The
fused points lie in the same world frame as the posed depth maps (§7).

---

## 4. `.data_cm` (692 bytes)

The full field decoding is in `docs/calibration.md` §3.5:
* rectified camera `(cx, cy, f, f)` at offset 0;
* texture, identity, left and right `CameraCalibParam`;
* `f_rect`, baseline, Δcx.

This analysis adds one correction [H]:

* `CameraDataFile::writeValidPoint` seeks to **0x29C (668)** and writes a
  **u64**. The value is the **total number of valid, unflagged depth pixels over
  all frames**, and it matches our count exactly in all three projects:

  | Project | Value |
  |---|---|
  | mustang | 456,153,529 |
  | PG2/P1 | 136,026,981 |
  | PG2/P2 | 55,215,961 |

* So the "per-project hash u32 at 668" plus the leading 0 of the int array
  in calibration.md is really this u64.
* It is followed by `ImageSizeInfo` = i32 (640, 512, 320, 256) at 0x2A4.

The calibration differs slightly per project: for example, the rectified cx is
320.833 in mustang and 319.598 in PG2. Always pair a project's depth maps with
its own `.data_cm`.

---

## 5. `.data_tb` (dense surface cloud) [H for the parts listed, L beyond]

| Offset | Type | Content |
|---|---|---|
| 0 | u32 len + chars | `"1.0"` |
| 7 | u8 | 0xEC (mustang) / 0x86 (PG2) [unknown] |
| 8 | i32 | −1 |
| 12 | u32 len + chars | `"v2.1"` (mustang) / `"v2.2"` (PG2) |
| 20 | u8 | 0 |
| 21 | u32 n | point count: 1,640,988 / 1,046,674 / 233,237 |
| 25 | f32[3n] | points (world, mm) |
| 25+12n | u32 n, f32[3n] | unit normals |
| … | … | v2.2: four zero u32s, then a named per-point property block (`u64 1`, `u32 6 "weight"`, type byte, data). The property's encoding is not decoded yet. mustang (v2.1) differs |
| EOF−16 | u64, u64 | footer: offset of a trailing record (a u32-length 11-byte key, repeated twice), then the file size |

What the data is:
* Many coordinates are snapped to a 0.5 mm lattice (e.g. x = …​.448/.552),
  which suggests TSDF/voxel-edge surface points [M].
* **`IC_Project1.asc` is exactly PG2/Project1's `.data_tb` points and normals**:
  1,046,674 rows, max deviation 5e-4 mm, which is the text rounding.
* `rccd_Project2.asc` is likewise PG2/Project2's (233,237 rows).
* `.data_tb` does **not** contain poses.

---

## 6. `.p3`, `.rge`, `.data_fusion_s`, `.data_tb_s`

* None of these exist in the three projects. `find ~ -maxdepth 6` found no
  `*.p3` or `*.rge`.
* In this EXStar version, global and per-frame markers live inside
  `.data_base` (§2.3–2.5).
* `libcommon` still contains `.rge` readers and writers (`read_rge_V2/V3`,
  `CRangeDataIO`), for legacy RangeData.
* libSn3DFile knows the `_s` suffixes, which are probably save or temp copies
  [L].
* Not decoded.

---

## 7. Validation

Method:
* Unproject depth pixels with Z > 0 and flag == 0 using the section intrinsics,
  falling back to `.data_cm`/2.
* Apply the pose to get world coordinates.
* Measure exact point-to-triangle distance against
  `mustang_differential_simplified.stl` (992,188 triangles, uniform-grid
  acceleration). For the PG2 projects, measure point-to-plane distance to the
  nearest `.asc` point, using the `.asc` normals.
* Use 4000 random points per frame, and no outlier trimming unless stated.

| Project | Reference | Frames | Points | Median | Mean | RMS | RMS of d<1 mm | Share > 1 mm |
|---|---|---|---|---|---|---|---|---|
| mustang | STL | 20 (ids 2…6055, even spread) | 80,000 | **0.128 mm** | 0.170 | **0.235 mm** | 0.226 | 0.2% |
| mustang | `.data_fusion` (point-to-plane) | same 20, flags ignored | 80,000 | 0.143 | – | 0.249 | 0.238 | 4.0% (all flagged pixels) |
| mustang | STL | **all 6055** (300 pts each) | 1.8 M | per-frame median 0.128; worst frame 0.36 | | | | no frame > 20% outliers |
| PG2/Project1 | `IC_Project1.asc` | 20 | 80,000 | 0.168 | – | 0.326 | 0.289 | 2.5% |
| PG2/Project2 | `rccd_Project2.asc` | 20 | 76,142 | 0.137 | – | 0.237 | 0.230 | 0.3% |

Per-frame RMS for the 20 mustang frames ranges from 0.16 to 0.32 mm.

Without the flag mask, mustang RMS rises to 19.7 mm because deleted background
points are included. This shows the flag byte must be honoured.

A depth-as-ray-range interpretation gives a median of 13.7 mm, and
row-major or inverse pose interpretations are off by hundreds of mm, so these
are ruled out.

Scripts were kept outside the repo in the session scratchpad (`fmtre/`), and
they are not part of the project.

---

## 8. Per-project statistics [H]

| | mustang_differential/Project1 | ProjectGroup_2/Project1 | ProjectGroup_2/Project2 |
|---|---|---|---|
| Frames (blocks = `FRAMES` = head count) | 6055 | 2057 | 2025 |
| Frames with pose | 6055 (100%) | 2057 | 2025 |
| Frame ids | 1…6055, no gaps | 1…2057 | 1…2025 |
| Depth intrinsics in frame | yes | **zero** (use `.data_cm`) | yes |
| Valid depth px/frame (mean, min–max) | 79,879 (14k–206k) | 66,129 (4.8k–223k) | 42,138 (7k–101k) |
| Flagged (deleted) px total | 27.5 M | 0 | 30.1 M |
| Global markers in header | 0 | 31 (lists A and B identical) | 25 (list B) |
| Frames with ≥1 marker | 0 | 2057 (mean 5.4, max 13) | 677 (mean 1.8, max 11) |
| Median inter-frame step | 4.1 mm / 0.87° | 3.0 mm / 0.77° | 3.3 mm / 0.82° |
| Pose jumps > 50 mm (relocalizations) | 16 (max 667 mm, 172°) | 32 (max 709 mm) | 10 (max 172 mm) |
| Fused points (`.data_fusion`) | 550,344 | 277,360 | 61,270 |
| Dense points (`.data_tb`) | 1,640,988 | 1,046,674 | 233,237 |

"Which frames were tracked":
* No per-frame "tracked" or "lost" flag exists.
* Frames captured while tracking is lost are simply not stored, and ids are
  renumbered consecutively [M].
* A pose jump between consecutive ids marks a relocalization. For example, the
  PG2/P1 frames around ids 238–242 and 335–337 were checked individually
  against the model (RMS 0.29–0.45 mm), and they are correct.
* For tracker development, treat pose discontinuities > ~50 mm (or > ~20°) as
  segment boundaries.

---

## 9. Suggested reader (outline)

1. Parse `.ir_E10_prj` (FRAMES) and `.data_cm`. Take `(cx, cy, f)` from offset 0
   and halve them.
2. Open `.data_base`.
   * Read the head block at 0: check `BLOCK`, size, CRC, and type key
     0x52BB5F73. Extract the global markers (keys 0x270/0x288).
   * Starting at 0xA00000, loop: read the 0x28-byte block header (magic,
     version, size). Read the offset table at +0x21. Index sections by u64 key.
     `pread` only the sections you need. The depth value alone is 1.64 MB, so
     memory-map or seek.
3. For each frame, read:
   * the pose (key 0x38);
   * the id (key 0x70);
   * the depth plus flags (key 0xA0);
   * the markers (key 0x28), if present.
4. Unproject as in §0.

---

## 10. Open questions

1. The byte 0xBC at block+0x20 (constant in all blocks).
2. Why the frame-block CRCs do not validate. Stale after in-place modification
   is the working theory. Which regions are re-written (pose, flags) is unverified.
3. Head marker lists A (0x270) and B (0x288): semantics (global vs total or
   fixed), and the meaning of the f64 at record offset 64 (1.0 vs ~6.3–6.8) and
   the i32 at 76 (−1).
4. Why PG2/Project1 stores zero intrinsics in every depth header (marker-mode
   project, `PROJECT_TYPE` 6?).
5. `.data_tb`:
   * the encoding of the "weight" property block;
   * the rest of the v2.2 body and the trailing 11-byte key record;
   * how v2.1 differs;
   * the byte at offset 7.
6. `.data_fusion` u32 at offset 8, and the i32 −1 and u8 before the cloud arrays.
7. The meaning of `OBJECT_ALIGN_TYPE` and `GENERATE_CLOUD` values. The mapping
   given is inferred from three samples only.
8. The PointImage, CloudWithWeight, texture, clip-plane and DepthInfo sections
   are never populated here, so their payload layouts are not documented.
   (`deserializePointImage` and `deserializeDepthInfo` exist.)
9. The `.p3` and `.rge` formats were not observed in these projects.
