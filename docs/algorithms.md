# EXStar / Einstar algorithms — depth reconstruction, marker detection, configuration

Status: static-analysis notes for interoperability and as a quality baseline for the open-source
C++23 / Metal replacement. **Clean-room rule:** this document describes behaviour in our own words,
tables and maths. No vendor kernel source or decompiled code is reproduced here. The raw OpenCL
strings extracted from the vendor binary live only in the git-ignored scratch area
`.re/out/opencl/` (reference only, never to be copied into `libs/`).

Analysed build: EXStar.app (Frameworks dated 2024-07-22), arm64 Mach-O.
Addresses below are image-relative offsets inside the named dylib (Ghidra project `app_re`, folder `/einstar`).

Confidence tags: **[H]** read directly from kernel source / literal constant; **[M]** inferred from
host-side argument plumbing (decompiler parameter mapping can be off by one); **[L]** educated guess.

---

## 0. Where things live

| Component | Library | Notes |
|---|---|---|
| IR speckle depth (stereo) | `libSn3DSpeckleFusion.dylib` | 10 OpenCL program strings; host classes `SpeckleRebuildGPUGPU`, `InferRedPreProcessManager`, `PixMatchManager`, `CostCompute`, `CostAggregation`, `SubPixelMatchManager`, `ZnccProcess`, `DataProcess`, `DetilRefineManager`, `MultiPlaneRebuild`, `CaliParReadIn` |
| Caller / parameter plumbing | `libSn3DAlgorithmWrapper.dylib` | `IRSpeckleRebuildAlgManager`, `SpeckleTrackAndFusionAlgManager` read `alg`/`scan` tables of `config.db` |
| Marker (retro-reflective dot) detection & tracking | `libalgorithmLzy.dylib` | see §2 |
| Tuning DB | `Resources/res/config.db` | SQLite3-Multiple-Ciphers (ChaCha20/sqleet) — key recovered, see §3 |
| Device ini | `Resources/res/Einscan-E10/ini/*.ini` | mostly legacy EinScan fringe/marker parameters, see §4 |

Extracted kernel programs (`.re/out/opencl/src_NN_<offset>.cl`, pretty-printed copies `*.pretty.cl`):

| File offset | Program (host class) | Kernels |
|---|---|---|
| 0x6626a | CostAggregation | 4 directional SGM passes, 4-path sum + WTA + parabola, `device_Tranform_device2D` |
| 0x6ad79 | CostCompute | 5×5 census, census Hamming cost volume |
| 0x6b98d | DataProcess | good-cube test, edge/occlusion deletion, right-view collision (multiple-match), normals (3 variants), 5×5 median, disparity→point, weights, CCL (float-threshold variant), island deletion, hole fill, inverse-depth Gaussian smoothing, normal box filter |
| 0x79f1a | CudaUtilFusion (CCL) | union-find connected components on a binary mask + component-size fill |
| 0x7b78a | MultiPlaneRebuild | 2× up-sampling over "good cubes", result transform |
| 0x7f671 | SubPixelMatch | slanted-window ZNCC with uniqueness / correlation threshold (hair mode) |
| 0x815b4, 0x8f846, 0x9be82 | ZnccProcess | slanted-window ZNCC sweeps (buffer and image2d variants), LR check, near-edge LR check, "fine" 4-quadrant-plane ZNCC, 21×21 box mean, bilinear copy |
| 0xa50d9 | imageProcess | stereo rectification (remap + bilinear), Gaussian pyramid down, colour undistort |

---

## 1. IR speckle depth pipeline (`libSn3DSpeckleFusion`)

### 1.1 Top-level flow

Entry: `speckle_rebuildFusion` → `SpeckleRebuildGPUGPU::gpu_operate_infred_speckleFusion` (0x38b1c). Per frame:

1. **Upload** left/right IR frames (8-bit, 1280×1024) and optional colour frame.
2. **IR pre-process** `InferRedPreProcessManager::run` (0x1e908): rectify both images, build one pyramid level (640×512), optional colour undistortion.
3. **Coarse integer matching** `PixMatchManager::PixMatch_InferRed` (0x21fe0): census + 4-path SGM at 320×256, median, consistency deletion, (hair mode: island removal / fill / smoothing).
4. **Slanted-window ZNCC refinement** `SubPixelMatchManager::SubPixMatch_InferRed` (0x45fa8) on the 320×256 grid in full-resolution disparity units; LR check; uniqueness (many-to-one) check; median; planarity ("good cube") + CCL cleanup.
5. **Multi-plane detail refinement** `DetilRefineManager::multi_PlaneRefine_withWeight` (0x16c6c): 2× up-sample to 640×512 and two/three more ZNCC passes with per-quadrant planes; normals; per-point weights.
6. **Download** point image (XYZ + normal + weight) → `PointImage` / `CloudWithWeight` / `DepthImage`.

### 1.2 Parameters and where they come from

`SpeckleFusionParameter` (ctor 0x2fc94, printer `operator<<` 0x24b64) — defaults vs. values the wrapper loads from `config.db` table `alg`, prefix `IRSpeckle.rebuild.*`:

| Field (offset) | Log label | Built-in default | config.db (`IRSpeckle.rebuild.*`) | Meaning |
|---|---|---|---|---|
| +0x00 | `sampleRata` | 4 | `sampleRatio=4` | coarse depth-grid stride in full-res pixels (forced even; ≥1) |
| +0x04 | `resultSample` | 2 | – | stride of the final output grid (640×512) |
| +0x08 | `depthImageSize` | 1280×1024 | `imgSize=1280,1024` | IR frame size |
| +0x10 | `colorImageSize` | 1280×1024 | `colorImgSize=1280,1024` | texture camera size |
| +0x18 | `pixCorrWin` / `subPixCorrWin` | 9 / 9 | `pixMatchWin=9,9`, `subPixMatchWin=9,9` | forced odd; `halfWin = max/2 = 4` defines the valid border |
| +0x20 | `imageRange` | (9,1271, 9,1015) | `10,1270,10,1014` | x-min, x-max, y-min, y-max of processed region (full-res px) |
| +0x30 | `edgeWeight` | 0.1 | `edgeWeight=0.1` | weight multiplier for points flagged as near a hole/edge [M] |
| +0x34 | `rawPointDis` | 2.0 | `rawPointDis=2.0` | nominal point spacing (mm); host derives `edgeDis = 7 × rawPointDis = 14` which is the 14.0 threshold seen in normal/smoothing calls [M] |
| +0xe8 | (filter window) | 3 | `newAlgRebuild.filterWindow=3` | final Gaussian smoothing window (hair mode) [M] |
| +0xec | `depthOfField` | 300, 600 mm | overridden at runtime by `reset_depth_range(near, far)` | valid Z range for the final cleanup |
| – | `findRange` | – | `-480,480` | **ignored**: `set_findRange` is an empty stub (0x26990); disparity search is hard-coded (§1.4) |
| – | `gradientTh`, `scanType=5` | – | present | not consumed by this library [L] |

Other fixed internals: invalid-disparity sentinel **1270**; the SGM code path always uses **4 paths**; buffers are sized for 1280×1024 input.

### 1.3 Stage A — IR pre-processing (rectify + pyramid)

- **Calibration input** (`CaliParReadIn`, 0x55810…; single-camera text reader 0x572d0): Bouguet-toolbox layout — `fc[2]`, `cc[2]`, `alpha`, `kc[5]`, `T[3]`, `R[9]` (22 numbers per camera; the ini path demands exactly 22) [H for fields, M for vector order]. Each camera's (R,T) is an extrinsic w.r.t. a common calibration frame; the stereo pose is composed as `R_rl = R_r·R_lᵀ`, `T_rl = T_r − R_rl·T_l` (`Read_Camera_Para`, 0x55e30) [H]. Rectification uses OpenCV-3.4.8 `stereoRectify` + `initUndistortRectifyMap` semantics (OpenCV assertion strings present) producing float2 remap tables uploaded to the GPU.
- **Rectification kernel**: per output pixel, bilinear sample of the raw image at the remap coordinate; samples whose integer corner falls outside `[0, W−2]×[0, H−2]` are left untouched; result rounded half-away-from-zero to uint8 (`IsRound=1`) [H].
- **Q-vector** used everywhere (`get_qMatrix` 0x33d44): `q = (f, B, Δc)` with `f = Q[2][3]`, `B = 1/Q[3][2]`, `Δc = Q[3][3]/Q[3][2]` (OpenCV Q). Disparity is signed **d = x_right − x_left**, and
  - depth `Z = f·B / (Δc − d)`,
  - `X = Z (u − cx)/fx`, `Y = Z (v − cy)/fy` using the rectified left intrinsics scaled to the grid (`u = col·s`, `v = row·s`).
- **Pyramid**: one level down, stride-2 decimation after a normalised **3×3 Gaussian, σ = 1** (window from `InferRedPreProcessPar.GaussianWin = 3`), rounded → 640×512 [H kernel, M window].
- No histogram equalisation, no CLAHE, no background subtraction on the IR images.
- **Auto-exposure hooks**: `speckle_AE_set(int2 a, int2 b, int2 c, int& v, bool en)` / `speckle_AE_get` only store/return values; defaults set in `SpeckleRebuildGPUGPU::ini` (0x309c4) are a=(80,120), b=(50,80), c=(0,20), v=5 [H values, L semantics — look like grey-level target bands]. No code in the analysed `run` path reads them, and the wrapper does **not** import `speckle_AutoExposure_*`. Exposure is instead driven by the host's brightness-level tables (`config.db` `scan:E10.cam_para_config.*`, §3).

### 1.4 Stage B — census + SGM (coarse integer disparity)

Resolutions: census on the 640×512 pyramid image; cost volume and SGM on a **320×256** grid (every 2nd pyramid pixel = every 4th full-res pixel, matching `sampleRata=4`).

| Item | Value | Conf. |
|---|---|---|
| Census window | 5×5, bit = 1 when neighbour < centre (25 comparisons shifted into a uint32; centre bit always 0) | H |
| Matching cost | Hamming distance of census words (0…24) stored as uint8; a zero cost **and** any out-of-image sample are stored as 255 (i.e. an exact census match is treated as invalid) | H |
| Disparity search | fixed **[−320, +320)** at pyramid scale = 640 hypotheses (`PixMatch_SetDepth` writes the constant regardless of depth arguments); 4 hypotheses per work-item, 160 work-items per row | H |
| Max supported range | template buckets 128/256/512/1024; >1024 → "disparity size must be less 1024" | H |
| Aggregation | 4 scan-line paths: L→R, R→L, T→B, B→T; 8-bit saturating path costs | H |
| Recurrence | `Lr(p,d) = C(p,d) + min( Lr(p−r,d), Lr(p−r,d±1)+P1, Lr(p−r,d±2)+P1, minₖLr(p−r,k)+max(P1,P2) ) − minₖLr(p−r,k)`, result clamped to 255. Note the ±2 neighbours also use P1 (non-standard). Hypotheses 0,1 and D−2,D−1 are never updated (stay 255). | H |
| P1 / P2 | **P1 = 0, P2 = 30**, constant (no intensity-adaptive P2: the image argument is unused) | M (from `PixMatchOption` bytes `+0x24=0`, `+0x28=30`) |
| WTA | sum of the 4 paths as uint16, arg-min (ties → lowest index) | H |
| Border rejection | best index 0 or D−1 → invalid | H |
| Sub-pixel | parabola: `d* = d + (C₋ − C₊) / (2(C₋ + C₊ − 2C₀))` | H |
| Uniqueness / LR at this stage | none | H |
| Output scaling | `disp_full = 2 × (d* + dmin)` → full-resolution disparity units on the 320×256 grid | H |

### 1.5 Stage C — coarse cleanup (`PixMatch_InferRed`)

1. **Median 5×5** (partial selection sort; invalid sentinel participates) inside `imageRange`; outside → sentinel. [H]
2. Disparity → points (`generate_Point`).
3. **Consistency deletion** `delete_through_Disparity_consist` (0x1202c), threshold `2·s = 8` px (s = sampleRata):
   - edge: if any 8-neighbour differs by > 2s → delete; occlusion ordering: if `d(x+1) + s < d(x)` delete both;
   - right-view collision: for pixels bordering holes, project into the right grid (`col + d/s`); if two left pixels land on the same right cell with disparities differing by > 2s → delete both. [H]
4. **Hair-enhance mode only** (`set_infrared_scan_object_type(true)`, wrapper key `hairEnhance`):
   island removal via CCL with neighbour-difference threshold **8.0** and min island size **100** grid cells; median 5×5; 4-neighbour hole fill; 3×3 "save invalid" fill; Gaussian smoothing (window 3) that averages **inverse** values of neighbours within **14** of the centre. [H kernels, M constants]

### 1.6 Stage D — slanted-window ZNCC refinement (`SubPixMatch_InferRed`, normal mode)

All matching is done on the rectified **full-resolution** 1280×1024 images, evaluated at the 320×256 grid positions (`u = 4·col`, `v = 4·row`).

**Plane-induced warp.** For a hypothesis disparity `d` at pixel (u,v) with unit normal **n**: back-project the point **P**(d); the right-image column of every window pixel (u′,v′) is
`x_R(u′,v′) = u′ + Δc − B · n·(u′−cx, v′−cy, f) / (n·P)`,
i.e. the disparity varies linearly across the window following the local tangent plane. Right intensities are linearly interpolated along the row. Score = ZNCC over the window (sums in fp32; the classic `Σxy − ΣxΣy/N` form).

**Hypothesis sweep.** `K` hypotheses `d_k = d₀ + (k − K/2)·step`. Winner = largest ZNCC among strict local maxima (k ∉ {0, K−1}); reject if none, NaN or > 1; sub-step parabola `δ = step·(c₋ − c₊)/(2c₋ − 4c₀ + 2c₊)`. ZNCC peak value is written as a per-pixel weight.

Sequence (normal objects):

| # | Operation | Window | K | Step (px) | Other |
|---|---|---|---|---|---|
| D1 | normals from disparity (drop neighbours with |Δd| > 14), then 5×5 normal mean | – | – | – | – |
| D2 | ZNCC sweep | 9×9 | 16 | 2.0 | ±16 px search |
| D3 | normals using point-to-plane distance ≤ 14 (mm), 5×5 normal mean | | | | |
| D4 | ZNCC sweep | 9×9 | 16 | 1.0 | ±8 px |
| D5 | normals + 5×5 mean | | | | |
| D6 | **LR check** (`leftRight_Check_new`, 0x528dc): re-match from the right view with the same plane warp, 9×9, 8 hypotheses, step 2.0; reject if the best hypothesis is more than `round(2/step)=1` index from centre | 9×9 | 8 | 2.0 | [H kernel, M params] |
| D7 | **multiple-match check** (`Check_Unique`, 0x10ad8): link-list of left pixels per right cell; if a right cell is claimed by left pixels > 3 grid columns apart, delete both | | | | H |
| D8 | median 5×5; recompute points + normals | | | | |
| D9 | **good-cube + CCL cleanup** (`Is_GoodCube`, 0x10600): a 2×2 cell is "good" if the two diagonal triangle-pair normals agree (cos ≥ **0.848**, ≈32°); edge flagging (5×5 window of invalid count); CCL over good cells; a point survives if one of its 4 adjacent cells is good **and** belongs to a component larger than **10** cells (5 in hair mode) **and** Z ∈ depth range | | | | H/M |

Hair-enhance mode replaces D2/D4 by `SubPixelMatch::SubPix_Match` (kernel `subPix_match_impl_update`: 9×9 or 15×15 window, 16 then 8 hypotheses, optional ratio test `cMax − c2nd > cMax·(1−uniqueRatio)`, final-pass ZNCC floor `coorValueThr`, clamp |Δd| ≤ 1) and D6 by `leftRight_Check_Hair`. Values passed include 1.0 and 0.1 (probably step and ratio/threshold) [L].

### 1.7 Stage E — multi-plane detail refinement (`DetilRefineManager`, 0x16c6c)

| # | Operation | Grid | Window | K | Step | Accept |
|---|---|---|---|---|---|---|
| E1 | 2× up-sampling over good cubes (`kernel_intePolation_twoSampleCube`): copy corners, edge mid-points = mean of 2, centre = mean of 4 (or of the diagonal pair in the `_new` variant); normals averaged and re-normalised; non-good cubes left empty | 640×512 | – | – | – | – |
| E2 | uniform ZNCC sweep | 640×512 | 5×5 | 5 | 1.0 | as §1.6 |
| E3 | normals | | | | | |
| E4 | **"fine" ZNCC** — each of the 4 window quadrants is warped with the normal of the corresponding diagonal neighbour (piecewise-planar = "multi plane") | 640×512 | 5×5 | 5 | 0.5 | only update if |Δd| ≤ **1.3**, else keep previous |
| E5 | normals | | | | | |
| E6 | (normal mode only) fine ZNCC again | 640×512 | 3×3 [M] | 5 | 0.5 | |Δd| ≤ 1.3 |
| E7 | normals, weights, `result_Transform` → output | | | | | |

**Output resolution: 640×512** point image (every 2nd rectified-left pixel), XYZ in the rectified-left camera frame (mm), normals and weights. The fusion side then works on 320×256 (`IRSpeckle.fusion.init.fusionDataSize`) or 640×512 depending on the point-distance "resolution strategy" (§3).

### 1.8 Normals

Central-difference cross products over the 3×3 neighbourhood: four triangles (up/left-right, down/right-left, and the two diagonals) summed and normalised; neighbours with z≈0 are replaced by the centre point. Variants reject neighbours by (a) disparity jump > threshold (14) or (b) point-to-tangent-plane distance > threshold (14 mm, first iteration: 3.0 in the refine stage). A result with fewer than 2 (or 1) valid neighbours → zero normal (point dropped). Optional 5×5 mean filter of normals (`normal_filter_kernel`). [H]

### 1.9 Per-point weight (confidence)

`w = max(0, min(cos θ_L, cos θ_R)) × w_depth(Z)` where `cos θ = n·(−P)/|P|` for the left camera and for the right camera (`P − (B,0,0)`), and [H]

| Z (mm) | w_depth |
|---|---|
| < 250 | 1.0 |
| 250–350 | linear 1.0 → 0.8 |
| 350–600 | 0.8 − (0.4/350)(Z−350) (vendor slope uses 700 as the end point; value at 600 ≈ 0.51) |
| 600–900 | 0.4 → 0.1 linear |
| > 900 | 0.1 |

Points near holes are additionally scaled by `edgeWeight = 0.1` [M]. The ZNCC peak is also stored per pixel.

### 1.10 Things we should do better / notes for the Metal port

- The coarse SGM has P1=0, uses only 4 paths, maps exact census matches to 255, and has no LR/uniqueness check — recoverable by the ZNCC stages but wasteful. An 8-path SGM with P1≈3–5/P2 adaptive on the 640×512 level, plus LR check, is a cheap improvement.
- Disparity range is fixed (±640 full-res px) regardless of `reset_depth_range`; deriving it from the working volume (e.g. 250–700 mm) would shrink the cost volume ~3–4×.
- Everything is float32 and single-precision ZNCC; Metal `simdgroup` reductions map well to the vendor's "sub-warp per pixel, one thread per hypothesis" layout (16 or 5 hypotheses per pixel).

### 1.11 Other code paths present but not used for Einstar IR

- `depth_Trans_Disp_kernel` / `PixMatch_TransferTX3`: converts a 16-bit Q8.8 disparity map from another device (invalid = 0xFF00) into disparity ×2 — hardware-depth product, not Einstar.
- `speckle_SubPixRefine` / `gpu_operate_infred_subPixRefine(_New)`: refinement given an externally supplied depth map.
- `gaussian_image_kernel`: 21×21 box mean of a 1280×1024 image (debug/test).

---

## 2. Marker detection and tracking (`libalgorithmLzy`)

Sources: `libalgorithmLzy.dylib` (Ghidra `/einstar/libalgorithmLzy.dylib`) and the caller `libSn3DAlgorithmWrapper.dylib` (`/einstar/libSn3DAlgorithmWrapper.dylib`). Findings come from a delegated static-analysis pass that I cross-checked against the decrypted `config.db`.

### 2.1 Configuration path (which values are live)

- **`BuildSetting.ini` `[RefPointSection]` is not used.** No EXStar binary references `BuildSetting.ini` or the string `RefPointSection` [H for the strings]. Marker parameters come from `config.db` through `libCommonCfg` `CommonCfgStrategy::getMarkerRebuild`. For Einstar IR the `alg` keys are `IRSpeckle.marker.*` (profile "marker") and `IRSpeckle.framemarker.*`. Each key falls back to a compiled-in CommonCfg default, then to the library's own `ConfigParam` default.
- The wrapper `resetTrackHandle` (0x442a8) fills `Sn3DAlgorithm::ConfigParam` (printer `print_self` 0x1b9460, ctor 0x10e0c0) and calls `marker_tracker_ini`.
- **`useTyAlg`**: CommonCfg default is true, and the key does not exist in `config.db`, so the default applies. When it is true the wrapper does the following:
  - forces `extrType = 2`, `bCalibType = 0` (Ty calibration format) and `bRebType = 0`;
  - does **not** forward `highThresh`, `lowThresh`, `alignRange`, `dimError` or `extrType`, so library defaults apply to those;
  - never forwards `maxCircumference`.
- **Marker size list `dimList`** maps nominal diameter (mm) to tolerance. In `config.db` the candidates are `scan:IRSpeckle.dimStr0/dimErr0` = 6.0/1.8 and `dimStr1/dimErr1` = 3.0/0.5, with `marDimSize = 1`. That suggests only the 6 mm class is enabled [M]. The wrapper fallback is {6.0 ± 1.0, 3.0 ± 0.5}.
- **Depth limits are overridden at runtime** with `setDepthLimit`; one observed log shows `zMin:160, zMax:250`. The configured 250/700 is therefore not what runs [H from log].

### 2.2 Parameter mapping

Columns: E10 `BuildSetting.ini` (legacy, unused); `config.db` `IRSpeckle.marker.*` (live); CommonCfg fallback; library default; meaning.

| Key | ini | config.db | CommonCfg | lib | Meaning (units) |
|---|---|---|---|---|---|
| minGrayTH | 100 | **150** | 100 | 100 | Binarisation threshold (grey level, strict `>`) |
| ellipseMinD | 4.0 | **2.5** | 4.0 | 3.0 | Fast path: min blob box size and min semi-minor axis (px); clamped up to 3, or 4 when width ≥ 1000. Contour path: min arc length / 2π |
| ellipseMaxD | 150 | **40** | 200 | 120 | Max blob size / semi-major axis (px) |
| ellipseRatio | 5.0 | **2.5** | 3.0 | 3.0 | Max a/b (fast path) or max circularity L²/(4πA) (contour path) |
| ellipseError | 0.5 | **0.5** | 0.3 | 0.5 | Max std-dev of radial fit residuals (px) |
| minCircumference / maxCircumference | 10 / 800 | 25 / 200 | 15 / – | 25 / 200 | Contour point-count limits (contour path only) |
| highThresh / lowThresh | 90 / 30 | 200 / 25 (not forwarded) | – | 150 / 50 | cvCanny thresholds, contour path only. The fast path hard-codes 120/50 |
| epiErr | 0.5 | **0.35** | 0.5 | 0.8 | Max point-to-epipolar-line distance of the right centre (px, undistorted) |
| alignErr | 0.05 | **0.1** | 0.1 | 0.06 | Pair-distance match tolerance (mm); inlier radius = 3 × alignErr |
| alignRange | 500 | 400 (not forwarded) | – | 400 | Max pair distance used in signatures (mm); min is hard-coded 5 mm |
| zMinLimit / zMaxLimit | 250 / 700 | 250 / 700 | 150 / 800 | 250 / 700 | Allowed triangulated Z, exclusive (mm); overridden at runtime |
| dimError | 0.4 | 0.5 (not forwarded) | – | 0.5 | Diameter tolerance in `dim_vote`/smoothing; per-marker acceptance uses the `dimList` tolerances |
| bIsFourPtAlign | 1 | **true** | false | 0 | 4-point instead of 3-point alignment; min markers per frame becomes 4 |
| bIsRansacFilter | 0 | – | – | 0 | RANSAC ellipse fit (contour path only) |
| bRebType | 0 | true (forced 0) | – | 0 | Reconstruct in the rectified frame and rotate back [M] |
| bDebugMode | 0 | 0 | false | 0 | Debug archives |
| bIsReflectFMarks | 0 | – | – | 1 | Contour-path variant selector [L] |
| maxAngle | – | – | – | 150 | Passed to the aligner; use not traced |
| extrType | – | 3 (forced 2) | – | 2 | Detector selection (§2.3) |
| firstFrameCycleTimes | – | 5 | – | – | Retries for the first frame [M] |

Legacy ini keys that no EXStar binary references (unused):
- `pointRadius0..2`, `radiusError0..2`, `pointQuality`, `circularity`, `eripolarError`, `minDiameter`, `maxDiameter`, `lengthEpsilon`, `removeEpsilon`, `minSamePointCount`, `bweight`;
- all of `[CannySection]` and `[CodePointSection]`.

### 2.3 Single-image detection (Einstar path: `extrType = 2`)

`CMarkerReconstruct::marker_reconstruct` (0x15b014) processes both images in parallel and dispatches on `extrType`:

| extrType | Path |
|---|---|
| 1, 2, 6 | `SnMarkerCenExtract::marker_center_fast_detect` (0x12e34c) |
| 0 | OpenCV contour path |
| 3 | fixScan variant |
| 4 | fixScan + coded targets |
| 5 | dot-target / GSI mode, the only user of `DotTargetRecognitionGaussion` |

The AAMED detector (`Ell_detect_AAMED`) has no callers, so it is dead code [M].

Fast path, in order [H]:

1. **Blob seeding** (0x1210b8).
   - Scan rows y ∈ [10, H−10) and columns x ∈ [10, W−15).
   - A run starts where 2 consecutive pixels are > `minGrayTH`. Runs are merged across rows by x-overlap into bounding boxes with pixel counts.
   - Keep a blob only if all of these hold: `minS ≤ h−1` and `h < maxS`; `minS < w < maxS`; aspect < 4:1; fill ratio > 50% of the box; more than 3 px from the border.
   - The ROI is the box grown by 3 px.
2. **Patch processing** (0x129b90).
   - Gaussian smoothing, then threshold at `minGrayTH`.
   - Morphological opening with an elliptical kernel of radius 1 (radius 2 when W ≥ 1000). Pixels that the opening removes are classified as thin laser-line structures and excluded.
   - Canny on the opened mask, dilated by 2, gives a boundary band.
   - Canny on the grey patch. Keep edge pixels inside the band that are not laser pixels; at least 8 are required.
3. **Canny** (0x1267d4): L1 Sobel magnitude, 4 direction sectors, NMS, hysteresis with **hard-coded** thresholds 50 (weak) and 120 (strong).
4. **Sub-pixel edges** (0x127248): sample the bilinear gradient magnitude along the gradient direction at t ∈ [−1, 1] in 0.2 steps and fit a least-squares parabola. Accept the peak if |t\*| ≤ 1.
5. **Ellipse fit** (0x127f58): algebraic conic fit `ax²+bxy+cy²+dx+ey+1 = 0` via 5×5 normal equations, then conversion to centre, semi-axes and angle. The fit is rejected if any of these fail:
   - the ellipse is real;
   - b > ellipseMinD;
   - a < ellipseMaxD;
   - a/b < ellipseRatio;
   - edge count ≥ 0.8πa (about 40% of the perimeter);
   - radial-residual std-dev ≤ ellipseError;
   - at least 60% of twelve 30° sectors are covered.
6. **Centre = fitted ellipse centre.** There is no grey centroid or Gaussian fit on this path.

Contour path (`extrType = 0`, 0x145268), for reference:
- threshold at minGrayTH, then findContours;
- filters: arc length in [2π·minD, 2π·maxD]; point count in [minCirc, maxCirc]; 1 ≤ L²/(4πA) ≤ ratio; area ≥ 5; at least 7 px from the border;
- ROI grown by 7 px, Gaussian 3×3, `cvCanny(low, high)`, then ellipse fit (RANSAC when `bIsRansacFilter` is set).

### 2.4 Stereo matching and triangulation

`CMarkerReconstruct::reconstruct` (0x15267c) [H]:

1. Each image needs at least 3 ellipses. Centres are undistorted.
2. **Epipolar gating** on the *unrectified* images: for each left centre, compute the line l = F·x_L. A right centre is a candidate if |l·x_R|/√(l₁²+l₂²) < `epiErr`.
3. For each candidate pair, reconstruct a 3D circle (centre, normal, radius) from the two image ellipses (`ellipse_reb_ty`, internal constants 0.75 and 0.5). The measured diameter 2r must fall within a `dimList` class tolerance.
4. **Triangulation** (`re_constrction`, 0x14200c): linear least squares on the 4 inhomogeneous DLT rows from both 3×4 projection matrices (3×3 normal equations; det < 1e-6 fails). The result is expressed in the reference camera frame.
5. The depth check requires zMin < Z < zMax.
6. Ambiguity resolution: each left marker keeps the candidate whose diameter is closest to nominal. Right ellipses claimed twice are dropped.
7. The optional elliptical ROI (`cutEllipseBox`) is applied. At least 3 markers are needed.

Each output marker (`MarkerPtCloud`, 0x80 bytes) holds position, normal, weight = 1, nominal and measured diameter, id, code, deleted flag, and the left/right image centres.

### 2.5 Frame-to-frame tracking

`MarkerTrackerManager::marker_tracker_cpu` (0x170ae8). Nmin = 3, or 4 with `bIsFourPtAlign`.

1. **First frame:** seeds the global map (`set_marker_ref`). Failure returns 5.
2. **Match against recent frames** (`align_from_frameset`, 0x16e7cc): tries up to about 51 recent frame marker sets, newest first, using the 3-point aligner (0x136ea4) or the 4-point aligner (0x1382ec). A 3-point success is refined with the 4-point aligner when both sets have more than 3 markers.
3. **Aligner ("descriptor")**, a hypothesise-and-verify scheme. There is no RANSAC loop and no triangle-area or angle descriptors.
   - Signature = all intra-set pairwise distances d with 5 mm < d < alignRange.
   - Candidate correspondences are distance pairs with |d_A − d_B| < alignErr; at least 3 are needed (6 for the 4-point version).
   - Vote for a "hub" point shared by the most matches, then verify its star of matches (≥ 2 or ≥ 3 edges) for geometric consistency. This yields a pose hypothesis.
   - Greedily associate points within 3·alignErr. At least 3 (or 4) inliers are required; strict mode also requires ≥ min(nA, nB) − 2 when n ≥ 5.
   - The final pose comes from **Horn's closed-form absolute orientation** (quaternion / eigen-decomposition). The mean residual is kept.
4. **Relocation fallback** (`markers_relocation`, 0x16ecbc): aligns against the global map.
   - 3-point tolerance 0.2 (0.5 in one mode).
   - 4-point tolerance 0.1; accepted when at least 4 markers and at least 60% of the frame lie within 2 mm (5 mm) with normal dot product > 0.97.
   - Failure returns 4.
5. **Repeat-frame skip:** if `skipRepeatFrame` is on and motion < `skipRepeatFrameDisThre` (0.15 or 0.05), return 8.
6. **Frame-to-model refinement** (`frame_to_model_optim_rt`, 0x16f470): transform the frame into the map and find nearest neighbours within r = 1.5 mm (nominal diameter < 2 mm), otherwise 5 mm (10 mm in GSI mode). Normal dot product must be > 0.2. Horn is re-solved on at least Nmin pairs; failure returns 4.

### 2.6 Global marker map

The map is updated during refinement:

- Matched markers get a running mean: p ← (w·p + p_new)/(w+1), the same for the normal (then renormalised), and w ← w+1.
- Unmatched markers are appended with new ids and w = 1.
- Duplicates merge only through the NN radius.

This path has no bundle adjustment. `marker_integrate_id_with_remove_repeat` (0x1803a4) and `get_hole_global_fitting` exist for other modes and were not analysed.

### 2.7 Error codes and why markers get rejected

Library return codes (`marker_tracker_cpu`):

| Code | Meaning |
|---|---|
| 0 | OK |
| 2 | Null image or handle |
| 3 | `marker_reconstruct` failed |
| 4 | Alignment, relocation or refinement failed |
| 5 | Reference (first frame) initialisation failed |
| 8 | Repeated frame skipped |
| 17 | Jaw mode: returns the last pose |

Code 3 has four possible causes:
- `dimList` is empty, or its smallest diameter is outside [0.5, 50] mm;
- no ellipses in either image (`MK_NO_FOUND_MARKER`);
- extraction failed in one image;
- fewer than 3 stereo markers after the epipolar, diameter and Z gates.

Wrapper (`MarkerAlgManagerPrivate::track`, 0x4b9f4) [H]:

- The reported code is `ret == 0 ? 0 : 0x2A100000 | ret`. The 0x2A1 prefix is hard-coded in the wrapper; `libSn3DErrorCode` maps `EM_Sn3DMarkerAlg` to 0x35000000, which is a different scheme. A missing handle gives −1.
- **0x2A100001 (705691649):** the wrapper sets ret to 1 whenever the frame ends with fewer than Nmin markers (3, or 4 with four-point alignment), after clearing them. The library never returns 1. "0 markers + 0x2A100001" therefore just means "fewer than Nmin stereo markers". The real cause is the inner code in the `marker track ret:` log line; the observed log shows ret 3, i.e. reconstruction produced fewer than 3.
- **0x2A100009:** under track flag 2, the global map has fewer than Nmin markers.
- Track flags: 0 = pre-scan, 1 = tracking, 2 = rebuild plus align to the global map, 3 = pre-scan plus coded, 4 = coded tracking, 5 = rebuild only. Flags 2 and 5 call rebuild with (320.0, 700.0), which look like a depth range [L].

Likely causes of "0 markers", in order:

1. The runtime depth window (e.g. 160–250 mm) excludes the markers.
2. The `dimList` classes don't match the stickers. Einstar ships 6 mm (outer) markers, and possibly only the 6.0 ± 1.8 class is active.
3. The IR image is below `minGrayTH` = 150. Marker mode drops projector brightness to `marLaserBrightness` = 60 and uses the `gray_mark_extract_*` exposure table.
4. Epipolar error > 0.35 px, from a calibration or undistortion mismatch.

### 2.8 Open questions (markers)

- How `ellipse_reb_ty` recovers the circle normal and radius (constants 0.75 and 0.5).
- `set_marker_ref`, `search_tri/fourpoints_match` internals, and the aligner's use of `maxAngle`.
- `dim_vote`/`dimError` and `get_repeat_flag`.
- Gaussian kernel size and sigma in patch processing.
- `bRebType` rotation and the `bIsReflectFMarks` semantics.
- Whether `marDimSize = 1` really limits `dimList` to the 6 mm class.

---

### 2.9 Our implementation (differences from EXStar)

Written from the notes above, not from EXStar's code.

- **Detection** (`libs/markers/detect`): threshold blobs → sub-pixel contour at the half level between the local background and the peak, restricted to the blob's own box → Fitzgibbon ellipse fit with residual, axis-ratio and angular-coverage checks → **dark-ring test** (samples at 1.4× the ellipse must stay within 20% of the disc contrast above the background). The ring test is what separates stickers from laser speckle dots, which pass every shape test.
- **Stereo** (`libs/markers/stereo`): centres are undistorted and rectified. Candidate pairs are gated by row error, depth range, left/right size agreement and physical diameter (6 mm by default). A depth prior comes from the dense depth in an annulus just outside the sticker ring, because the sticker itself leaves a hole in the speckle depth. Markers with no surface around them are rejected unless dense stereo can't exist there (outside either rectified view). Pairings that aren't unique are dropped rather than guessed.
- **Map** (`libs/markers/marker_map`):
  - Association is RANSAC over triplets with a least-squares refit and re-association. EXStar's greedy association lets a few wrong pairs drag the pose.
  - New markers are candidates until they are re-observed in place. Candidates can help tracking near a pose guess, but they are never used for relocalisation, bundle adjustment or display.
  - Relocalisation uses pairwise-distance triangle signatures, verified on all frame markers.
- **Tracking**: joint marker + point-to-plane ICP. Marker terms are weighted and centred like the surface terms, in both the CPU and Metal solvers. Markers can override a failing surface check only with at least 4 inliers.
- **Global markers**: a markers-only capture pass collects keyframes (pose, identified markers, rectified left/right centres). A Ceres bundle adjustment then minimises the stereo reprojection error of every marker centre over keyframe poses and marker positions, with a Huber loss, outlier removal and the first keyframe fixed. EXStar keeps only a running mean with no adjustment. The result becomes a fixed map; a later scan starts by relocalising onto it and tracks in its frame.

## 3. Tuning database `config.db`

- Opened by `libSn3DConfig.dylib` `Sn3DConfigSQLite::open` (0x7598) through the Qt SQL driver **`SQLITECIPHER`** (`PlugIns/sqldrivers/libsqlitecipher.dylib` = *SQLite3 Multiple Ciphers 1.3.5*, SQLite 3.37.0).
- The default strategy (FUN_00008dac) is **static**: `dbFile = ../Resources/res/config.db`, **`dbPassword = "2021shining3D"`**, `dbTableName = common`, prefix empty. [H]
- Cipher = sqlite3mc default **ChaCha20-Poly1305 ("sqleet")**: key = PBKDF2-HMAC-SHA256(password, salt = first 16 file bytes, 64007 iterations, 32 bytes); 4096-byte pages with 32 reserved bytes (16-byte nonce + 16-byte Poly1305 tag); page 1 bytes 16–23 stay plaintext; per-page one-time key = ChaCha20(key, nonce[0:12], counter = LE32(nonce[12:16]) ⊕ pgno), data keystream at counter+1, page-1 payload starts at byte 24. Verified: all 50 pages authenticate and the plaintext opens with stock SQLite. A decrypted dump (cloud-account tokens redacted) is in `.re/out/configdb/config_dump.txt` (scratch only).
- Schema: 17 tables, each `(key varchar, value varchar, comment text)`: `project, seqFusion, seqScan, scan, calibration, deviceadjust, postprocess, measure, common, alg, scenecontrol, ui, navigation, Sn3DCloudDisk, ModePage, restore, config`. Einstar = device prefix **`E10`** plus the shared **`IRSpeckle.*`** keys.
- The app rewrites this file at runtime (modification time changes while running) — our tools must treat it as read-only input and never write it.

### 3.1 Keys relevant to depth/tracking/fusion (Einstar)

`alg` table:

| Key | Value | Use |
|---|---|---|
| `IRSpeckle.rebuild.pixMatchWin` / `subPixMatchWin` | 9,9 | §1.2 |
| `IRSpeckle.rebuild.findRange` | −480,480 | ignored by the lib (§1.4) |
| `IRSpeckle.rebuild.imageRange` | 10,1270,10,1014 | processed ROI |
| `IRSpeckle.rebuild.sampleRatio` | 4 | coarse grid stride |
| `IRSpeckle.rebuild.rawPointDis` / `edgeWeight` | 2.0 / 0.1 | §1.2 |
| `IRSpeckle.newAlgRebuild.*` | same + `filterWindow=3`, `downSampleTime=0`, `sampleONPyramid=0` | alternative config set |
| `IRSpeckle.marker.*` (see §2.2) | ellipseMinD 2.5, ellipseMaxD 40, ellipseRatio 2.5, ellipseError 0.5, epiErr 0.35, minGrayTH 150, bIsFourPtAlign true, alignErr 0.1, alignRange 400, minCircumference 25, maxCircumference 200, zMinLimit 250, zMaxLimit 700, highThresh 200, lowThresh 25, dimError 0.5, bRebType true, firstFrameCycleTimes 5, extrType 3 | **the marker parameters actually used for Einstar IR marker mode** (they override BuildSetting.ini) |
| `IRSpeckle.framemarker.*` | ellipseMinD 3.0, ellipseMaxD 120, ellipseRatio 3.5, ellipseError 0.5, epiErr 0.5, minGrayTH 100, highThresh 220, lowThresh 150, alignErr 0.1, alignRange 500, dimError 1.0, min/maxCircumference 25/200, zMin/zMax 250/700 | second marker profile ("frame marker") |
| `IRSpeckle.fusion.{init,smallInit,bigInit}.*` | sampleRata 4; icpIterNum 9; icpDis 5,200 (big: 10,200); icpRataRay 0.25,0.25 (small: 0.25,0.15); icpParticleSize 20; icpDisAvgThread 0.5; disUniform 0.3; particleTrackPtAfterSample 1500; particleCubeDistribution 5,5,1; treeDepth 13; pointDis 0.5 (small: 1.0); fusionDataSize 320,256; viewBox 300,1200; fusionLimitNum 5000 (small 2000); maxMatchNum 2000; histogramEqualization 1; checkStitch 1; ORB vocabulary `./res/mixRes/` | tracking/fusion (ICP + feature) setup |
| `IRSpeckle.fusion.pointDis_sub_*`, `pseudo_thre_*` | point-distance ladder 0.1…10 mm and matching confidence thresholds 15…1 | fusion quality tiers |
| `E10.generateData.*` | marker_radius 6, max_marker_normal_angle 90, min_smallpart_area 25, denoise iterations, spike sensitivities | post-processing |

`scan` table (Einstar-relevant):

| Key | Value | Use |
|---|---|---|
| `IRSpeckle.triggorPeriod` / `texTriggorPeriod` | 68 / 90 (ms) | frame period without / with texture |
| `IRSpeckle.LaserBrightness` / `marLaserBrightness` | 100 / 60 | IR projector brightness (lower in marker mode) |
| `IRSpeckle.textureLedValue` / `smallTextureLedValue` | 1000 / 400 | texture LED |
| `IRSpeckle.fusionLimitNum`, `validNumThresshold` | 4000, 2000 | min points per frame |
| `IRSpeckle.planeRata/planeProp/planeNumThresshold/planeNumMinValue` | 0.3 / 0.8 / 8000 / 2000 | "flat scene" detection (tracking degeneracy) |
| `IRSpeckle.dis0/dis1` + `resulutionStrategy0/1` | 3.0 → gray 640×512 / scan 320×256; 10.0 → gray 320×256 / scan 320×256 | resolution by point distance |
| `IRSpeckle.marDimSize`, `dimStr0/dimErr0`, `dimStr1/dimErr1` | 1; 6.0/1.8; 3.0/0.5 | marker diameter classes (mm) and tolerances |
| `IRSpeckle.rotaAlignThre/tranDisThre`, `rotaAngleThreSlow/transDisThreSlow` | 1.0/1.0; 5.0/2.0 | duplicate-frame / slow-motion thresholds (deg/mm) |
| `IRSpeckle.GPUTypeThreshold`, `lowGPU*` | 1050; skip 2 frames; sampling 2 | low-end GPU fallbacks |
| `E10.cam_para_config.*_{0..20}` | 21 brightness levels × {gray, gray_mark_extract, gray_small_extract, gray_small_nm} × {left,right} × {gain, expo}, plus `gray_led_N`, `project_dutiratio_N` | the brightness slider → sensor gain/exposure (exposure in ms: ~0.33–8.5 normal, 1.1–5.6 marker mode; gain 0.3–7). This table, not an image-based AE loop, sets IR exposure. |

`seqScan` / `seqFusion` hold generic sequential-scan defaults (fail counters, plane thresholds, pointDis ladder).

---

## 4. Device ini files (`Resources/res/Einscan-E10/ini/`)

These are largely inherited from the EinScan fringe-projection product line; for Einstar the `config.db` `IRSpeckle.*` values take precedence where both exist.

### BuildSetting.ini

| Section / key | Value | Meaning |
|---|---|---|
| `[SystemSection] saveBmp/saveBmps/readBmp` | 1/0/0 | debug image dumping |
| `[CannySection]` filter 5×5, gaussSigma 1, sigma 1, thresholdLow 0.12, thresholdHigh 0.30 | | Canny for legacy circle finder (relative thresholds) |
| `[RefPointSection]` | see §2.2 | legacy marker parameters — **not read by any EXStar binary**; live values come from `config.db` |
| `[CodePointSection]` | filter 5×5, σ 2.1, Canny 70/150, quality 0.2, big 1.0, diameter 20–300, circularity 3, ring band 0.5–0.8, division 15 | coded targets (not used by Einstar) |
| `[RangeSection]` | nearClip 150, farClip −150, boxMask, viewAngleThreshold 85°, depthThreshold 3, pointDistanceThrehold 3, minSurfacePoints 500, maxDistance 0.9, deleteBoundary 1, L/M/H scale 6.5/3/1 | legacy fringe point filtering; view-angle 85° and min-surface 500 are reasonable defaults for us too |
| `[FastSection]`, `[SmoothSection]`, `[RebuildParam]` | fringe/phase settings (ccol 1280, crow 1024, Gaussian 3/σ1 and 7/σ2) | legacy |
| `[ProcessDataSection]` | maxCameraAngle 70°, maxDist 3, maxRatio 3, boundaryWidth 2, maxDihedral 60°, maxCellAngle 45°, markerRadius 3, minPatchSize 100, buildNormal 1 | mesh/point post-processing: reject points seen at > 70°, marker hole radius 3 mm |

### CameraSetting.ini

1280×1024 capture (`width/height`, `widthFull/heightFull`), camera ids, per-mode exposure/gain lists (legacy fringe values), `exposureType=2`, trigger mode 1, `doubleExposure=0`, binning parameters (off). Einstar actual exposure comes from `config.db` (§3.1).

### CalibrateSetting.ini

Lookup-table filenames, circle detection for the calibration board (quality 0.2, circularity 3, diameter 4–200 px, Canny 0.15/0.40), board geometry (`aa=66.27`, `bb=44.14`, 225/575 points), projector-camera block (1280×1536, 16 mm lenses, 5.2 µm pixels — legacy projector). `CalibrateSection.cas` is binary.

### CaptureSetting*.ini

Fringe projector step tables (pixelWidth / fringe counts / move counts) — not applicable to the IR speckle Einstar.

---

## 5. Open questions

1. **P1 = 0** in the SGM is read from option-struct bytes; confirm with a runtime trace (e.g. dtrace/lldb on `clSetKernelArg` index 5/6 of `kernel_cost_aggregate_*`).
2. Which depth range the wrapper passes to `reset_depth_range`. Candidates: marker `zMin/zMax` 250/700; the (320, 700) pair the marker wrapper passes for rebuild; fusion `viewBox` 300/1200; the default 300/600.
3. Exact semantic of the stored AE triple (80,120)/(50,80)/(0,20)/5 — appears unused in this build.
4. Whether `SubPixMatch_InferRed_Mac`/`Is_GoodCube_Mac` (distance-based planarity test, 0.4 × pixel footprint) are selected on Apple GPUs in some code path; the analysed default path uses the angle test.
5. Template parameter meaning for the final `zncc_subPix_macth_fine<3,5,6>` (window 3 vs. something else).
6. Hair-mode constants (`uniqueRatio`, `coorValueThr`) — only 1.0 and 0.1 are visible at the call site.
7. Order of the 22 per-camera calibration numbers in the vector form (text form is certain).
