# Einstar calibration storage and decoding

Status: reverse-engineered from EXStar (macOS arm64 build, 2024-07) by static
analysis. The decoder was checked against real data from one device (two
calibrations). This is a clean-room description: the tables and prose below
restate the formats in our own words. No vendor code is reproduced.

Libraries and addresses cited below are Ghidra addresses. Image base is 0, so
they equal the `nm` offsets:

| Library | Role |
|---|---|
| `libSn3DCalibDevFile.dylib` | Splits the 6568-byte flash blob into sections and writes the cache files (`DeviceDataHandler`, `DeviceDataManager`) |
| `libSn3DCalibrationJR.dylib` | Writes and reads the obfuscated CCF files (`WriteCcf_bin`, `GenerateOffset`, `read_ccf_encode_write_ccf`). Also contains AES code that the calibration path does not use |
| `libSn3DCalibrateAlgorithmWrapper.dylib` | Calibration-session glue: board grid generator, activeBox defaults, focal seed |

Confidence tags used below: **[H]** means verified against data or read directly
from code, **[M]** means strongly inferred, **[L]** means a guess.

---

## 1. Flash blob (6568 bytes = 0x19A8)

`DeviceDataHandler::setDevData` (0x13c1c) copies the 6568 bytes read from flash
unchanged into the handler object at `this+0x1420`. Every section below is then
read at a fixed offset. The per-section offset/size table is a static array
used by `getDataInfo` (0x13d48; data at 0x2391c for sizes and 0x23938 for
offsets). Each section ends in a 4-character ASCII magic plus a NUL. A section
is valid only if its magic matches (`strcmp`). **The blob has no CRC, no header
and no encryption.** **[H]**

| Section id | Blob offset | Size | Magic (at section end) | Contents | Written to cache as |
|---|---|---|---|---|---|
| 0 | 0x0000 | 0x19A8 | – | whole blob | – |
| 1 | 0x0000 | 0x02F9 (761) | `FAFA` @0x2F4 | "Factory" CCF (plain doubles) | text CCFs (Bouguet-style) |
| 2 | 0x02F9 | 0x00A2 (162) | `FBFB` ×2 | colour: 2 × {CCM, dark} | `ParaCCM_0.txt`, `ParaDark_0.txt`, `ParaCCM_noled_0.txt`, `ParaDark_noled_0.txt` |
| 3 | – | – | – | invalid (bitmask 0x77 excludes it) | – |
| 4 | 0x039B | 0x0F21 (3873) | `FQFQ` @0x12B7 | "Quick" CCF: raw copies of the three cache files | `LeftCCF.txt`, `RightCCF.txt`, `TexCCF.txt` |
| 5 | 0x12BC | 0x0099 (153) | `FWFW` @0x1350 | white balance, two 3×3 double matrices | `wbdata.txt`, `wbdata_noled.txt` |
| 6 | 0x1355 | 0x0011 (17) | `AABB` @0x1361 | 2 floats (AABB / depth parameters) | small file via `readAABBDataToFile` |
| – | 0x1366–0x19A7 | 1602 | – | unused / reserved | – |

All integers and floats are little-endian.

### 1.1 Section 1: Factory CCF (`FAFA`) **[H] layout, [M] camera order**

Read by `readFactoryCCFDataToFile` (0x16b00) and `readFactoryCCFDataToBinary`
(0x142c8). Written by `writeFactoryCCFFileToDev` (0x152dc) and
`writeBinaryFacCCFFileToDev` (0x13d90).

| Offset | Type | Meaning |
|---|---|---|
| 0x000 | u32 | type/version = 1 |
| 0x004 | i32 | image width, forced to 1280 on read |
| 0x008 | i32 | image height, forced to 1024 on read |
| 0x00C | CamSlot | camera 0 (left) |
| 0x104 | CamSlot | camera 1 (right) |
| 0x1FC | CamSlot | camera 2 (texture) |
| 0x2F4 | char[5] | `"FAFA\0"` |

A `CamSlot` is 248 bytes. The first 176 bytes are a `CameraCalibParam`
(22 doubles, §3.1). The remaining 72 bytes are reserved and not read.
Bytes 0x004–0x2F3 form the 752-byte `SrcCalibParam` structure, which is also
the `.bin` format of `readFactoryCCFDataToBinary`. This section stores plain,
unobfuscated doubles. When it is valid, EXStar expands it into Bouguet-style
text files (§3.4).

### 1.2 Section 4: Quick CCF (`FQFQ`) — the path the Einstar actually uses **[H]**

`readQuickCCFDataToFile` (0x146d4) and `writeQuickCCFFileToDev` (0x15ebc)
handle this section. The data is treated as valid only if the magic matches
**and** the first u32 is 4.

| Section offset | Blob offset | Type | Meaning |
|---|---|---|---|
| 0x000 | 0x039B | u32 | type = 4 |
| 0x004 | 0x039F | char[260] | entry 0 name (the writer never fills it) |
| 0x108 | 0x04A3 | i32 | entry 0 length (1..1024) |
| 0x10C | 0x04A7 | u8[1024] | entry 0 data = **LeftCCF.txt** (520 bytes) |
| 0x50C | 0x08A7 | char[260] | entry 1 name |
| 0x610 | 0x09AB | i32 | entry 1 length |
| 0x614 | 0x09AF | u8[1024] | entry 1 data = **RightCCF.txt** (520 bytes) |
| 0xA14 | 0x0DAF | char[260] | entry 2 name field. The writer puts the calibration time here as ASCII `"yyyy-MM-dd hh:mm"` |
| 0xB18 | 0x0EB3 | i32 | entry 2 length |
| 0xB1C | 0x0EB7 | u8[1024] | entry 2 data = **TexCCF.txt** (264 bytes) |
| 0xF1C | 0x12B7 | char[5] | `"FQFQ\0"` |

Each entry is `{char name[260]; i32 len; u8 data[1024]}` (0x508 bytes).
EXStar copies the data bytes unchanged into `<cachedir>/200x150/{Left,Right,Tex}CCF.txt`.
It writes them first as `*_dev` temp files and then renames them
(`readDevCalDataToDiskEx`, 0x1d12c). **So the cache files are the flash
contents byte-for-byte.** The flash blob itself carries no extra decoding
layer. Only the CCF-level obfuscation in §2 applies.

### 1.3 Section 2: colour (`FBFB`) **[H] layout, [M] semantics**

This section holds two consecutive 81-byte records:

| Record offset | Type | Meaning |
|---|---|---|
| +0x00 | u32 | type (record A = 2, record B = 3) |
| +0x04 | f32[15] | CCM block, 60 bytes; this is `ParaCCM*.txt` exactly |
| +0x40 | f32[3] | dark level; this is `ParaDark*.txt` exactly |
| +0x4C | char[5] | `"FBFB\0"` |

`readColorData` (0x17e84) validates each record: all floats must be non-NaN and
the dark values must lie in [0,1]. `readColorDataToFile` (0x17d04) writes
A.CCM, A.dark, B.CCM and B.dark to four files in that order. The files on this
device have identical LED and no-LED content.

### 1.4 Section 5: white balance (`FWFW`) **[H]**

| Offset | Type | Meaning |
|---|---|---|
| +0x00 | u32 | type = 5 |
| +0x04 | f64[9] | WB matrix #1 (3×3, row-major, diagonal in practice) → `wbdata.txt` **[M]** |
| +0x4C | f64[9] | WB matrix #2 → `wbdata_noled.txt` **[M]** |
| +0x94 | char[5] | `"FWFW\0"` |

If any element is NaN, EXStar substitutes defaults (`readQuickWbData`, 0x178f0):
diag(1.93312, 1, 1.69474) for #1 and diag(1.98274, 1, 1.68089) for #2.

### 1.5 Section 6: AABB **[H] layout, [L] meaning**

The layout is `{u32 type = 6; f32 a; f32 b; "AABB\0"}`. It is written by
`writeAABBToDev(float,float)` (0x1690c). `readAABBDataToFile` (0x18030) writes
the two floats (8 bytes) to a file. The meaning is probably a depth or size
limit pair; it has not been confirmed.

---

## 2. "Encryption" of LeftCCF/RightCCF: additive integer obfuscation (not AES)

**AES is not used for calibration.** **[H]** The `AES_*` routines in
libSn3DCalibrationJR (`AES_jiami`/`AES_jiemi` are 16-byte block
encrypt/decrypt, `AES_file_decrypt` at 0x3b6ce0 is plain ECB over 16-byte
blocks with a caller-supplied key) are called only from `run_ple`
(photogrammetry export). No other dylib imports them. The globals `key1`
(0x678460) and `key2` (0x678f88) are small-int tables used by that path.

Left/Right CCFs are protected by **per-field integer offsets**:

* At write time, `GenerateOffset()` (0x470f84) fills two 64-entry int32 tables,
  `offsetcalib_l` and `offsetcalib_r` (both in BSS), with `rand() % 10`.
  Each value is therefore in 0..9.
* It then derives a 64-entry key: `K[i] = L[i] + R[i]` or `K[i] = L[i] − R[i]`.
  The sign depends on i and follows a fixed pattern (table below).
* `WriteCcf_bin` (0x471134) writes each protected field as `stored = true ± K[j]`.
  It then appends **the file's own 64-int table**: `offsetcalib_l` to
  LeftCCF and `offsetcalib_r` to RightCCF.
* To decode, you need **both** files: `K` is built from the tail of LeftCCF and
  the tail of RightCCF. EXStar decodes with `read_ccf_encode_write_ccf`
  (0x471978).

Because each file carries its own key half, no secret is needed. The key only
changes between calibrations.

### 2.1 Binary CCF file layout (Left/Right: 520 B; Tex: 264 B) **[H]**

The file is 33 little-endian doubles (264 bytes). LeftCCF and RightCCF then add
int32[64] (256 bytes).

| Index | Field | Left/Right stored as | K index | K sign (L±R) |
|---|---|---|---|---|
| 0 | fx | true **+** K0 | 0 | + |
| 1 | fy | true **−** K1 | 1 | + |
| 2,3 | fx_err, fy_err | plain (0) | – | |
| 4 | cx | true **−** K2 | 2 | − |
| 5 | cy | true **+** K3 | 3 | − |
| 6,7 | cc_err | plain (0) | – | |
| 8 | alpha (skew coefficient) | true **+** K4 | 4 | − |
| 9 | alpha_err | plain (0) | – | |
| 10 | k1 | true **+** K5 | 5 | − |
| 11 | k2 | true **−** K6 | 6 | − |
| 12 | p1 | true **+** K7 | 7 | + |
| 13 | p2 | true **+** K8 | 8 | + |
| 14 | k3 | true **+** K9 | 9 | + |
| 15–19 | kc_err[5] | plain (0) | – | |
| 20 | Tx | true **−** K50 | 50 | − |
| 21 | Ty | true **−** K51 | 51 | + |
| 22 | Tz | true **−** K52 | 52 | − |
| 23 | R00 | true **−** K33 | 33 | + |
| 24 | R01 | true **+** K34 | 34 | − |
| 25 | R02 | true **−** K35 | 35 | + |
| 26 | R10 | true **+** K36 | 36 | − |
| 27 | R11 | true **+** K37 | 37 | + |
| 28 | R12 | true **+** K38 | 38 | − |
| 29 | R20 | true **−** K39 | 39 | + |
| 30 | R21 | true **−** K40 | 40 | + |
| 31 | R22 | true **−** K41 | 41 | − |
| 32 | reprojection error (px) | plain (0 for L/R) | – | |
| 33.. | int32[64] offset table (L or R half) | – | | |

The "K sign" column gives `K[i] = tailL[i] + tailR[i]` for "+" and
`tailL[i] − tailR[i]` for "−". Here `tailL` is read from LeftCCF and `tailR`
from RightCCF. The other K entries (10–32, 42–49, 53–63) exist but are unused.
Their signs, for completeness: 32 +, 42 +, 43 −, 48 +, 49 +, 53 +.

**TexCCF.txt is not obfuscated.** It has the same 33-double layout with no
integer tail, and its field 32 holds the RMS error.

Decoding, in the file's own terms: `true = stored − op·K[j]`, where `op` is the
+ or − in the "stored as" column.

Reference decode (Python, our own code):

```python
S  = {0:+1,1:+1,2:-1,3:-1,4:-1,5:-1,6:-1,7:+1,8:+1,9:+1,33:+1,34:-1,35:+1,36:-1,
      37:+1,38:-1,39:+1,40:+1,41:-1,50:-1,51:+1,52:-1}          # K sign
F  = {0:(0,+1),1:(1,-1),4:(2,-1),5:(3,+1),8:(4,+1),10:(5,+1),11:(6,-1),12:(7,+1),
      13:(8,+1),14:(9,+1),20:(50,-1),21:(51,-1),22:(52,-1),23:(33,-1),24:(34,+1),
      25:(35,-1),26:(36,+1),27:(37,+1),28:(38,+1),29:(39,-1),30:(40,-1),31:(41,-1)}
def decode(dL, tL, dR, tR):        # d*: 33 float64, t*: 64 int32
    K = {i: tL[i] + S[i]*tR[i] for i in S}
    fix = lambda d: [d[i] - F[i][1]*K[F[i][0]] if i in F else d[i] for i in range(33)]
    return fix(dL), fix(dR)
```

This was verified on two independent calibrations: the current cache and
`backup/.bak`. Decoded values match the plain copies in the project files
`*.data_cm` to **0.0 difference** (bit-exact). **[H]**

### 2.2 How the writer fills the fields **[H]**

`Sn3D_write_encrpt_ccffile` (0x2b75c) receives, per camera, `K = [f, cx, cy, aspect, skew]`.
It writes fx = f and fy = f·aspect, with alpha = skew. It also receives
distortion[5], the left pose as a quaternion + translation, and the left→right
pose as a quaternion + translation. The right camera's pose is written as the
composition (LR ∘ L). The error fields are written as 0.

---

## 3. Decoded parameter semantics

### 3.1 Camera model **[H]**

The model is the Bouguet (Caltech toolbox) model, which is also the OpenCV
model:

* `K = [[fx, alpha·fx, cx], [0, fy, cy], [0, 0, 1]]`. Here alpha is the
  dimensionless skew coefficient; skew in pixels is alpha·fx.
* Distortion `kc = [k1, k2, p1, p2, k3]` is Brown–Conrady with 3 radial and 2
  tangential terms, in the same order as OpenCV `distCoeffs`.
* Pixel coordinates are **0-based, OpenCV convention**. Using cx,cy as-is gives
  a mean epipolar |dy| of 0.083 px. Shifting by ±1 px worsens it to 0.10–0.11 px.
* Image size is 1280×1024. There is no binning or ROI (`widthFull = 1280`).
* Extrinsics: `X_cam = R · X_world + T`. R is a 3×3 matrix, row-major
  (R00 R01 R02 R10 …), and T is in **mm**. The world frame is the calibration
  board at the reference pose (board plane z = 0, about 190–205 mm in front of
  the cameras). The inverse convention was tested and fails completely (§5).
* In-memory `CameraCalibParam` order, used by the factory section and
  `.data_cm`: `fc[2], cc[2], kc[5], alpha, R[9], T[3]` (22 doubles, 176 B).

### 3.2 Decoded values (current calibration, 2026-09-27) **[H]**

| | Left (IR) | Right (IR) | Texture (RGB) |
|---|---|---|---|
| fx | 1157.23588 | 1159.08122 | 1141.85538 |
| fy | 1157.35240 | 1159.24004 | 1142.21913 |
| cx | 625.36487 | 633.79275 | 662.11298 |
| cy | 522.36893 | 506.03491 | 508.88210 |
| alpha | 2.6411e-4 | 1.2154e-4 | −8.157e-6 |
| k1 | −0.156139 | −0.157356 | −0.150166 |
| k2 | 0.158111 | 0.174904 | 0.162076 |
| p1 | −5.3155e-5 | 6.8048e-4 | −2.1995e-5 |
| p2 | 2.9988e-4 | 1.2562e-3 | −3.5483e-4 |
| k3 | 0.039162 | 0.010014 | 0.045726 |
| T (mm) | (52.512, −2.119, 204.132) | (−31.356, −2.710, 199.940) | (−12.164, −2.026, 192.140) |
| RMS err | 0 (not stored) | 0 | 0.0555 px |

Rotation matrices (world→cam):

```
R_L = [-0.036271 -0.975675  0.216202; -0.993475  0.011795 -0.113440;  0.108130 -0.218906 -0.969736]
R_R = [ 0.006117 -0.986202 -0.165435; -0.993641  0.012606 -0.111885;  0.112427  0.165068 -0.979853]
R_T = [-0.003627 -0.996997 -0.077350; -0.993317  0.012515 -0.114738;  0.115361  0.076416 -0.990380]
```

Derived rig geometry:

* **Left→Right**: `R_LR = R_R·R_Lᵀ`, `t_LR = T_R − R_LR·T_L = (−156.945, −0.221, 30.667) mm`.
  **Baseline = 159.913 mm**. The relative rotation is 22.15°, almost pure yaw
  (rvec = (0.091°, 22.147°, −0.044°)): the cameras toe in, and their optical
  axes cross about 416 mm in front of the rig.
* **Left→Tex**: |t| = 122.78 mm, yaw 17.03°. **Right→Tex**: |t| = 37.14 mm,
  yaw −5.12°. The texture camera sits on the L–R baseline, since
  122.78 + 37.14 ≈ 159.9.
* FOV (left): 57.9° × 47.7°.
* The previous calibration (`backup/.bak`, Nov 2024) gave a baseline of
  159.842 mm, fy_mean 1157.953.

### 3.3 Focal length sanity check (1157 px vs "16 mm / 0.0052 mm")

The `[ProjectoCamera]` section of `CalibrateSetting.ini` (`focus_mm = 16`,
`pixel_size = 0.0052`, `imageHeight = 1536`) is a leftover template from other
EinScan models. It does not describe the E10. The E10 pose seed in
`resetSn3DComputeE10FactoryPose` uses its own defaults (focal 1800 px as a seed,
principal point at the image centre). The calibrated f ≈ 1157 px with 5.2 µm
pixels means a **~6.0 mm lens** (or ~5.6 mm if the pitch is 4.8 µm). No binning
is involved: images are native 1280×1024, and the principal point lies near
(640, 512). **[M]**

### 3.4 Text CCF format (factory path and decrypted export) **[H]**

The factory path and the decrypted export use a Bouguet-style ASCII file,
written by `writeDecryptCCFTxt` (0x16dac) and parsed by
`readDecryptCalibrationFile` (0x1847c). It has these lines, in order:
`Focal Length: fc = [fx fy]`, its error line, `Principal point: cc = [...]`,
its error line, `alpha: alpha = [a] [err]`, `Distortion: kc = [5]`, its error
line, `Translation vector: T = [3]`, `Rotation Matrix: R = [9]` (row-major), and
`Standard Error: error = [e]`. Values are printed as `%20.15lf`.

### 3.5 `.data_cm` (project copy, 692 bytes) **[H] layout, [M] labels**

| Offset | Type | Content |
|---|---|---|
| 0 | f64[4] | rectified camera: cx_rect (319.598), cy_rect (514.755), f_rect, f_rect. Here f_rect = mean(fy_L, fy_R) = 1158.2962 |
| 32 | f64[22] | Texture CameraCalibParam (fc, cc, kc, alpha, R, T) |
| 208 | f64[12] | identity R (9) + zero T (3) — reference/device transform |
| 304 | f64[22] | Left CameraCalibParam (decoded) |
| 480 | f64[22] | Right CameraCalibParam (decoded) |
| 656 | f32[3] | f_rect (1158.296), **baseline (159.913)**, Δcx_rect (619.36) |
| 668 | u32 | per-project ID/hash (differs between projects) |
| 672 | i32[5] | 0, 640, 512, 320, 256 (half- and quarter-resolution sizes) |

OpenCV `stereoRectify(alpha=-1)` gives f = 1158.296, cx₁ = 319.75, cy = 514.78
and Δcx = 619.1, which matches the header closely.

---

## 4. Other cache files

| File | Format | Meaning |
|---|---|---|
| `wbdata.txt`, `wbdata_noled.txt` | 9 × f64 (72 B), 3×3 row-major | Diagonal white-balance gains. Current: diag(1.62328, 1, 1.66557) and diag(1.64140, 1, 1.67277). G is the reference; channel order is probably R,G,B **[M]** |
| `ParaCCM_0.txt`, `ParaCCM_noled_0.txt` | 15 × f32 (60 B) | [0..8] a 3×3 colour-correction matrix, [9..11] per-channel offsets, [12..14] per-channel gains (1.659, 1.0, 1.735). Each **column** of the 3×3 plus its offset sums to 1.000 (1.0173 − 0.0173). So output channel j = Σᵢ M[3i+j]·inᵢ + off[j], which preserves white. **[M]** |
| `ParaDark_0.txt`, `ParaDark_noled_0.txt` | 3 × f32 | Per-channel dark/black level, normalised: 0.0451, 0.0458, 0.0448 (≈ 11.5/255). Validated as lying in [0,1] **[H]** |
| `activeBox.dat` | 6 × f64 | 3-D working-volume box in mm, stored as (xmin, xmax, ymin, ymax, zmin, zmax). The calibration-session writer (`CameraCalibrateAlg::configFileAlg` call at 0x2d564 in AlgorithmWrapper) passes min = max = (−3000, −3000, 200). That produces the current file [−3000, −3000, −3000, −3000, 200, 200], which is a degenerate or disabled box. The device-path default (`writeActiveBoxFile`, 0x14d50) is x, y ∈ [−1500, 1500], z ∈ [200, 1500] **[H]** |

---

## 5. Calibration board

* **XML/INI:** `calibrateConfig/Einscan-EA/rapidCameraCalibrate.xml` defines
  only the capture procedure. It has 5 groups × 5 steps (25 image triplets).
  Group 0 is face-on; groups 1–4 are tilted ±18° about x and y (`angle z=180`).
  Distances are `distanceMid=200`, `distanceMax=600`, with 1280×1024 images and
  pose algorithm `runRapidCameraCalibratePose`. It does **not** give the board
  geometry. `BuildSetting.ini [RefPointSection]` holds marker-detector
  thresholds only (minDiameter 6, maxDiameter 100, circularity 3, epiErr 0.5,
  …).
* **Code:** `CalibrateBoard3DAlg::get3D` (AlgorithmWrapper 0x2b1ee4/0x2b1c18)
  generates a generic grid: cols × rows at an integer pitch p (mm), centred on
  the origin, with X = (i − ⌊(cols−1)/2⌋)·p, Y = (⌊(rows−1)/2⌋ − j)·p and Z = 0.
  The E10 path instead receives an opaque `boardData` blob
  (`PoseAlgManagerPrivate::setCalibBoardData`, 0x48ac8) from upstream, and its
  source was not located. **[M]**
* **From the images (measured):** the board has white retro-reflective dots,
  each with a dark ring, on grey. The dots form a **square grid with a pitch of
  27.89 ± 0.08 mm** triangulated (diagonal 39.5 mm), so the nominal pitch is
  most likely **28 mm**. Four **larger dots** act as orientation markers: three
  sit on one grid column and one sits between rows, off-grid. The visible grid
  is ≥ 7 × 6 (41 dots matched per frame). **[M]**

---

## 6. Validation performed

Tools: python3 with numpy 2.5.2 and OpenCV 5.0.0. Scratch scripts are not kept
in the repo.

1. **Decode check:** the decoded Left/Right/Tex CCFs match `.data_cm` bit-exactly
   for two different calibrations. The same `.data_cm` stores a baseline float
   of 159.913 (and 159.842 for the older calibration), identical to the value
   computed from the decoded extrinsics.
2. **Epipolar check** on the 25 stored calibration triplets
   (`calibrate_image_read_rapid/`, captured in the same session as the current
   CCFs):
   * Dots were detected by background-subtracted threshold and an
     intensity-weighted centroid.
   * Left↔right correspondences came from a homography seeded by the 4 big
     markers and refined with RANSAC. This matching does not use the
     calibration, so the check is unbiased.
   * Points were undistorted and rectified with `cv2.stereoRectify`.
   * Result over 11 images and 451 points: **mean |dy| = 0.083 px, RMS 0.114 px**.
     The worst single image has a mean of 0.14 px. Depths ranged 262–565 mm.
     (113 mismatches, from images where marker ordering failed, were rejected
     at |dy| > 3 px.)
   * With the inverse extrinsic convention (cam→world), 0 of 11 images pass.
   * With the older calibration (`backup/.bak`), every point shows a constant
     dy ≈ −3.9 px. The rig has drifted since then, which confirms the images
     belong to the new calibration.
3. **Metric check:** the nearest-neighbour spacing of triangulated dots is
   **27.89 mm ± 0.08 mm**, consistent across depths of 300–500 mm, and the
   diagonal is 39.5 mm. Board planarity RMS is 0.03–0.10 mm.
4. **Texture camera:** projecting the triangulated points (via the world frame)
   into `imageTex*` gives **0.56 px** mean distance to the detected dots
   (410 points). This confirms that all three CCFs share one world frame.

---

## 7. Open questions

1. **Board definition.** The source of the E10 `boardData` blob is unknown (it
   may be a file shipped per board or downloaded). The pitch of 28 mm is
   inferred, and the rows/cols count and big-marker coordinates are not
   recovered. It is worth checking `libSn3DCalibCaptureImageProcess`
   (`rotateBoard`), `libSn3DEACalibrateController` and the device-side blob.
2. **Why 27.89 mm and not 28.00 mm?** The −0.4 % scale could be real board
   pitch, centroid bias on the ringed retro-dots, or a scale bias in the
   calibration.
3. **Factory `FAFA` section.** No raw flash dump was available, so it is
   unknown whether this unit populates it. The cache files match the Quick
   `FQFQ` format. The meaning of the 72 reserved bytes per camera slot is
   unknown, and the camera order L/R/T is inferred from call order.
4. **LED vs no-LED assignment.** It is not verified which of the two WB/CCM
   records maps to `*_noled`. Channel order (RGB vs BGR) of the WB/CCM gains is
   also unverified.
5. **AABB section (6).** The meaning of its two floats and which file consumes
   them are unknown.
6. **`.data_cm` extras.** The meaning of the identity block, the u32 at
   offset 668, and the leading 0 of the int array are unknown.
7. **Offset-table sign pattern.** Signs for K indices that no field uses were
   read from `GenerateOffset` but have no bearing on decoding.
8. **Write-back.** To write a new calibration to the device, generate fresh
   tails (each value 0..9) and apply the §2.1 table. EXStar's reader does not
   check whether the tail values are random, but this is untested on hardware.
