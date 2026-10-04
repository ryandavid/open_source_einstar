# Status

## Working (verified without hardware)

| Area | State | Evidence |
|---|---|---|
| USB transport (libusb) | done | codec/mask round trip on documented example; packet reassembly + resync tests |
| Device control | done | full command set via typed API; dangerous opcodes blocked at compile time; emulator tests match EXStar's logged bytes; reconnects and replays every setting when the scanner leaves the bus (it reboots or restarts its USB side on its own, docs/firmware.md 5) |
| Firmware flashing | done, not yet run on hardware | `einstar-firmware version / inspect / flash`: package checks, reboot first, EXStar's packets sent once each, reconnect and read the version; the host's recorded conversations (a full flash with our package, a typical session) replayed into the vendor firmware and our build in fx3emu agree reply for reply |
| Device emulator | done | modelled on the firmware source (`firmware/src`): payload-length checks, status codes, reply and bulk sizes, shared IR exposure, gain rounding, laser halving, strobe wrap, the restart after a cleared image-endpoint halt, reboots; streams packetised frames, serves flash, buttons, temperature |
| Calibration | done | decodes EXStar CCF / flash blob; matches documented values; baseline 159.913 mm |
| Rectification | done | row alignment < 1e-6 px on synthetic rig with distortion + toe-in; each camera has its own principal point, placed so the scene at 320 mm lands on the same columns in both images (the cameras converge 22°: one shared principal point, as before 2026-10-03, left only ~half the overlap in frame at 300 mm). Disparities are signed. Emulated scan: valid depth per frame 34% -> 56% |
| Stereo depth | done (CPU reference + Metal) | census + SGM (slant steps) + slanted-window ZNCC; 0.056 mm median error on synthetic speckle; Metal 5.7 ms/frame (CPU 21 ms), 99.8% of disparities within 0.25 px of CPU |
| Tracking | working | robust point-to-plane ICP, normal-space balancing, degeneracy-aware updates (degenerate frames tracked but not fused live), global FPFH relocalisation on a background worker with 3-frame confirmation |
| Fusion | done (CPU reference + Metal) | sparse voxel-hash TSDF; Metal: GPU hash table + brick pool, integrate 0.7 ms, raycast 1.2 ms, full surface extraction 0.4 ms |
| ICP | done (CPU reference + Metal) | Metal runs all Gauss-Newton iterations in one command buffer, one dispatch each (the last threadgroup to finish solves; 1.3 ms vs 3.7 ms CPU); identical poses and Hessians to the CPU solver |
| Live view | done | Metal splats, frustum/trail, ghost frustum when lost, HUD, IR previews (`--snapshot` for headless capture), marker discs (map / global / current frame) and preview detections |
| Markers | done | detection on raw IR (blob + half-level contour + ellipse fit + dark-ring test), rectified stereo matching gated by a dense-depth prior, marker map with candidate confirmation, RANSAC association, triangle-signature relocalisation, joint marker + surface ICP (CPU and Metal) |
| Session recording | done | every processed frame (depth, confidence, pose, flags, markers, capture settings, tracking diagnostics), the scanner and its calibration, every frame that could not be processed, optional raw IR (`einstar-cli replay` runs it through the live pipeline again: none dropped, as this build does it, so one scan with raw IR is a fixed test case for depth front-end changes); frame numbers stay unique across stream restarts; `.estr`, zstd, crash-tolerant; a recording opens again as a paused scan (view, edit, process, or resume with the same scanner, appended to the same file); see docs/process.md |
| Process step | done | frame-level pose graph (chain + free-space-verified fragment registrations + loop closures + marker landmarks), island exclusion, lost-frame recovery, re-fusion, surface-nets mesh, cleanup, parallel quadric simplification, STL/PLY/OBJ export; CLI `process`, app Process panel. Mustang: 18.6 s; 93.0% of EXStar's reference within 0.5 mm |
| Scan to CAD (Einstar Model) | working, first real part done | labels, primitive fits, holes and forms, constraint solver, OpenCASCADE solid, named STEP; re-processing a changed scan carries the labels over; blocks for features the scan barely saw or hollow ones (clips, trays with walls); first real part: a car display reversed from a rough scan with draft, mirrored tabs and measured dimensions (docs/model.md) |
| Photo library | working, tried on real photos | photos in the `.emodel`, annotations with typed units, registration to the scan (camera recovered to <1 mm on synthetic views; 9-27 px rms on real phone photos), the scan's reading of each annotation, apply as constraints, holes placed from a marked diameter where the scan bridged over them, photo colour on the scan |
| Global markers | done | markers-only constellation capture -> keyframes -> Ceres stereo-reprojection bundle adjustment -> fixed map that later scans start on; save/load as text |

### Verified on the scanner (`einstar-cli hw-test`, 2026-09-29, firmware SC130_FX3_V2.10_FPGA_V3.7, USB 2.0)
- Identification, sensor ranges (1280x1024, 8-bit, exposure 1..10000, gain 1..800) and the flash calibration
  (2026-09-27 13:57, baseline 159.913 mm, 22.147 deg) as documented. Flash reads are consistent. A session's
  first request could repeat the last number of the previous session, which the firmware drops
  (docs/firmware.md 5): sessions now start at a random sequence number, and device status errors are
  reported with their meaning and not retried.
- **Sensor 1 is mounted upside down**: its frames arrive rotated 180 degrees relative to the calibration
  (only that orientation gives stereo depth; rectified rows then agree to ~1 px on the markers). The device
  layer turns it upright; the emulator sends it rotated. Sensor 0 is the left camera.
- **Group ids count modulo 256** (the header field is 32 bits): the device layer extends them so frame
  indices keep increasing (a 20 s stream: 300 of 300 groups, no gaps across the wrap).
- Image-endpoint stalls are cleared and the stream resumed. Clearing the halt arms the firmware's full
  restart on the next 00/07 (when FPGA state bit 17 is set): the scanner leaves the bus and comes back with
  its FPGA and sensors reset, and the device layer reopens it, checks the serial and replays its settings.
  Not yet provoked on hardware; the emulator models it.
- Exposure and gain write + read back on all three sensors; state polling; 10/62 accepted for DISTANCE 0/1/2
  (which selects FPGA laser modes 4/1/2, docs/firmware.md 5, not only indicator LEDs).
- Scan streaming: 14.77 Hz (68 ms trigger), every group complete, no frame-id gaps, resyncs or bad packets.
  Texture mode (1 IR + RGB): 10.04 Hz, all three sensors in every group. USB 2.0 carries both.
- Strobe route 0 lights the scene (IR level 12 -> 200-252 at exposure 4400 / gain 120). The "LD" register
  (10/68, `set_laser_percent`) alone changed nothing in the images, with DISTANCE 1 (laser mode 1) selected
  and the scanner face down on a table (scene out of focus and saturated).
- Register 10/5D ("colour mode") read 8 on all three sensors (the firmware leaves that reply byte
  unwritten; no longer queried). Temperature (10/50) reads 0x0000 idle and streaming, on both firmwares: no sensor answers on that
  I2C channel (protocol-device.md 3.11); the app shows "n/a" instead of 0 °C. Device-state reply byte
  21 is a run state: 01/02 idle, 08 scan streaming, 40 texture streaming.
- With the scanner's light on (a bucket lid with marker stickers at ~45 cm, no ambient IR): brightness
  above black is linear in exposure (6.3 / 12.7 / 25.2 at 1100 / 2200 / 4400; 42 at 8800) and in gain as a
  percentage (x1.9 at 240, x3.4 at 480 vs 120). 10/68 alone adds a fine speckle texture; 10/62 DISTANCE
  (laser mode) made no visible difference. Depth: 11-14% of pixels valid (347-535 mm), limited by the lid's
  smooth plastic and a specular highlight, not by exposure. Marker detection: 10 of 10 stickers (was 4-5)
  after giving saturated blobs a looser dark-ring test and raising the minimum size to 6 px (on the
  scanner's IR the sticker surround is barely darker than the surface; real speckle and glints are
  under 7 px).
- **The scanner no longer matches its stored calibration.** With the flash calibration, rectified rows of
  matching markers disagree by up to ~7 px (stereo markers fail, depth is sparse), while EXStar's own
  captures from that calibration run (tests/fixtures/external/calibration_board) fit it to 0.05 px.
  EXStar forms frames exactly as we do (its saved right images carry the per-frame metadata at the end,
  reversed: the same 180-degree turn; nothing else in its configuration or command log touches the
  pixel geometry). A stereo bundle adjustment (`einstar-cli board-check ... --ba`) fits EXStar's views
  alone and ours alone equally well (0.31 / 0.35 px) but no single model fits both (ours 1.1+ px): the
  cameras' geometry changed after 2026-09-27, mainly a ~0.2-0.3 degree rotation of one camera relative
  to the other plus small intrinsic shifts. A calibration fitted to three of our board poses brings the
  rows of independent captures to ~0.2 px, markers match (7 of 10 close, 9 of 10 far) and valid depth
  triples. Fix: recalibrate (EXStar's calibration, which rewrites the flash; or a host-side calibration
  with `EinstarCalibration.app`, which writes it into the scanner like EXStar; docs/calibration.md §8). Raw captures: `einstar-cli hw-capture` (local,
  fixtures-data/).
- **First live scans (2026-10-01, after recalibrating with `EinstarCalibration.app`)**: the calibration
  read back from the scanner is the one solved, and it aligns rows to 0.2-0.4 px on saved scan frames, but
  ICP failed on ~99% of frames. Causes, found by replaying the recordings (`einstar-cli track-session`):
  the scanner's first frames after the lights switch on have no depth, and the tracker started its model
  on one, so nothing could ever align (also marker capture starting without markers: nothing registered);
  and the stream header's timestamp is a constant, so motion prediction never ran. Now a scan starts on a
  frame with depth over 5% of the view (or 3 markers; marker capture needs the markers), restarts itself
  if it loses track within its first 15 frames, and frame times come from the trigger schedule. Replayed
  with these, one recording went from 1 to 764 of 955 frames tracked.
- 10/62 DISTANCE 0/1/2 turns the top LED red / green / blue (idle and scanning). Button codes (`einstar-cli
  hw-ui`): 1 single, 2 double, 3 long on start/pause, each cleared once read; the brightness buttons report
  a double click as two singles and a long press as 3 then, 1-2 s later, 2.
- Still open: marker detection on real IR, RGB colour content; the longest image packet (the firmware's DMA
  buffer allows 41 000 bytes, `hw-test` now prints it); reconnection on hardware (unplug/replug, host sleep).
  The scanner is USB 2.0 only (its cable and connector carry no SuperSpeed lines), so the firmware's
  SuperSpeed paths never run.

The scanner's calibration is in `tests/fixtures/calibration/einstar_e10` (`einstar-cli calib-dump`), so the
tests run on it everywhere. Tests that need real recordings or calibration captures use
`tests/fixtures/external` (`einstar-cli fixture-pack`, see its README) and skip without them.

### Discrete GPUs (Intel Mac, Radeon Pro 560X)
Buffers are placed by access (`gpu::Context::gpu_buffer` / `mirrored_buffer`): private (VRAM) for GPU-only
data and managed with explicit syncs for data the CPU also touches; on unified memory both are the
shared buffers as before. Kernels that were slow on AMD have variants selected on non-Apple GPUs
(barrier-free SGM paths, fused row winner-takes-all, 8x8 raycast tiles; identical results); Apple GPUs
keep their kernels. Blob moments are exact integer sums everywhere (float atomic adds are emulated on AMD).
`einstar-bench pipeline` on the 560X: 583 -> 42 ms per frame (markers and recording on), within the 68 ms
frame period; the live view adds ~5.8 ms of GPU per frame at 3200x2000.

### Tracking on EXStar's own recordings (`einstar-cli track-fixture`)
mustang_differential, 6055 frames, depth-only (no markers/texture), GPU path. A single replay is not a
reliable measure: starting one frame later swung the old tracker between 74.5% and 94.5% tracked, because
whether a slide on the part's surface of revolution plants a ghost surface depends on tiny numerical
differences. Numbers below are over six start frames (0-5):
- 94.1-95.2% of frames tracked (was 74.5-94.5% before degenerate frames stopped extending the live
  model and relocalisation moved to a background worker); 30-64 frames more than 20 mm from EXStar
- the recording has 25 large jumps where EXStar itself lost the scanner (up to 667 mm / 158 deg): we
  re-acquire after a median of 5 frames (3 of them the confirmation window), worst 25-49, never at a
  wrong pose; paced at the scanner's 14.7 Hz with live (non-waiting) relocalisation: median 4-5, worst 34-46
- tracking-thread time per frame p95 ~9 ms, worst ~25 ms including lost frames (was 460-660 ms when
  descriptor rebuilds and global registration ran inline); the 68 ms frame period is never exceeded
- `--diagnose-frame N` / `--probe-frame P` / `--oracle` show where a frame disagrees with the model and
  when a ghost entered it

PG2/Project1 (markers on the part), 2057 frames, GPU path:
- geometry only: 6.6% tracked (the part slides); hybrid (surface + markers): 85.8% tracked, pose p95 0.56 mm / 0.09 deg vs EXStar, 5.8 ms per frame
- hybrid on EXStar's own final marker map as fixed global markers (`--global-markers`): **100% tracked**, frame-to-frame error p95 0.22 mm; absolute agreement 0.75 mm median (a constant offset from EXStar's final map)
- the remaining losses without global markers follow the recording's discontinuities into areas whose markers were never seen before

### Global markers end to end (synthetic, `test_global_markers`)
Marker stickers rendered into the emulated IR images -> live detection -> markers-only capture (21/21 frames tracked, nothing fused) -> bundle adjustment (16 markers, 0.07 px, max error vs truth 0.20 mm) -> surface scan started on the fixed map: 30/30 frames tracked with **absolute** pose error <= 0.31 mm / 0.074 deg (the open-loop sweep without markers drifts ~2 deg), no phantom markers in the map.

ICP is linearised about the centroid of the observed surface (not the world origin), which keeps
the float GPU solve well conditioned and makes the degeneracy analysis independent of where the
scan started; the degeneracy/relocalisation thresholds were recalibrated for it.

## Known issues / next steps
1. **CPU use** (`einstar-bench pipeline`, emulated frames with markers, recording on): 24.1 → 3.6 ms of CPU per frame across all threads, now 8.9 ms (M4 Max, 2026-09-30): the real-IR marker detection (looser ring test for saturated blobs, a GPU pre-check kept as its superset) raised the candidates reaching the CPU fit from ~100 to ~440 per frame, and fitting them over the whole TBB pool had cost 19 ms (workers spinning while the frame waits for the GPU); the fits now run in a 2-thread arena. Tightening the GPU pre-check again is the next lever. The work moved to the GPU:
   - marker blob search: threshold, connected components, per-blob box, size, peak and moments, gates, and a dark-ring pre-check on the moment ellipse; about 100 candidates per frame reached the CPU's sub-pixel fit (~440 since the real-IR changes);
   - camera previews: textures copied GPU to GPU;
   - raw IR: the camera frames are used by the GPU in place (page-aligned image storage, no-copy buffers);
   - recording: depth packing on the GPU, zstd level 1.
   What remains on the CPU: the ellipse fits (~1.6 ms), zstd (~1 ms), tracker bookkeeping, marker hole filling, and relocalisation while lost.
2. **Speed**: done. Frames stay GPU-resident from stereo through ICP, fusion and rendering (speckle filter, points/normals, balancing weights, raycasts, surface extraction and the live overlay are all GPU buffers; the CPU only touches them for relocalisation and fallbacks). Full frame path 8.6 ms (was 12.9 ms with CPU round trips, ~50 ms all-CPU).
2. **Open-loop drift** on young models (synthetic sweep: ~0.03 deg/frame). Fix in the process step: keyframe pose graph + loop closure + re-fusion.
3. **Surfaces of revolution** are geometrically ambiguous; depth-only tracking holds but may slide. Degenerate frames are no longer fused live (surface fused at a slid pose became a ghost that broke tracking thousands of frames later); on long featureless parts the live model therefore stops growing and tracking is eventually lost. Use markers (hybrid / global markers) or, later, texture.
4. **Markers on real IR**: tuned on EXStar's calibration captures and synthetic stickers only. Real scans may need the detection threshold / ring test adjusted (live speckle brightness vs. retro-reflective return under the strobe is unknown until hardware). 3 mm markers are disabled by default (too close to speckle size).
5. **Process step**: done (docs/process.md). It includes lost-frame recovery, free-space-verified loop closures, island exclusion and error-bounded simplification; marker holes are filled in the frontend. Open: watertight (Poisson) meshing.
6. **Session recording**: done. Raw IR is optional (off by default, ~20-25 MB/s); RGB is not recorded.
9. **Reproducibility**: fixed. Replays are bit-identical: GPU surface extraction returns a canonical order. That exposed the real problem: global relocalisation onto the 180°-symmetric counterpart of the mustang part. Global registration now reports ambiguous matches, and the tracker takes the candidate near where tracking was lost (or waits). Mustang: 95.7% tracked, p95 1.05 mm vs EXStar (a flipped run was 74%, p95 57 mm).
10. **Stereo outliers**: measured on synthetic speckle (`debug_depth_errors`), they are not concentrated at silhouettes (5% of the >2 mm errors are). About 7% of pixels have ≥1 px disparity errors spread over textured areas, which may be specific to our synthetic speckle. Fusion averages them out (0.16 mm mesh error). Retune only once real IR captures exist.
11. **IR-intensity / colour ICP term**: not implemented. The IR images are lit by the laser speckle, which moves with the scanner, so their intensity is not surface texture. A photometric term needs the RGB texture camera (or strobe-only IR frames), and there are no such recordings to validate it on.
7. Stereo outlier blobs at silhouettes need an extra consistency filter.
8. Hardware-only unknowns: what the "LD" register drives exactly, and the laser modes beyond the LED colour (see "Verified on the scanner").
12. **Firmware**: the update package, flash layout, boot chain, runtime and FPGA loading are
    described in docs/firmware.md, with an assessment of an open replacement (FX3 side feasible,
    FPGA side needs hardware access; the FPGA vendor is unidentified).

## First contact with a real scanner
`einstar-cli probe --verbose` performs read-only identification plus a calibration read and prints a full hex transcript.
`einstar-cli hw-test` exercises the rest (registers, indication LEDs, scan and texture streaming, light and exposure
sweeps, depth and markers on the captured frames, optionally buttons) and turns the projector and strobe on;
`einstar-cli calib-dump <dir>` writes the scanner's calibration as EXStar's cache files.
