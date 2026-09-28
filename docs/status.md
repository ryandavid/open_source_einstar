# Status

## Working (verified without hardware)

| Area | State | Evidence |
|---|---|---|
| USB transport (libusb) | done | codec/mask round trip on documented example; packet reassembly + resync tests |
| Device control | done | full command set via typed API; dangerous opcodes blocked at compile time; emulator tests match EXStar's logged bytes |
| Device emulator | done | speaks the protocol, streams packetised frames, serves flash, buttons, temperature |
| Calibration | done | decodes EXStar CCF / flash blob; matches documented values; baseline 159.913 mm |
| Rectification | done | row alignment < 1e-6 px on synthetic rig with distortion + toe-in |
| Stereo depth | done (CPU reference + Metal) | census + SGM (slant steps) + slanted-window ZNCC; 0.056 mm median error on synthetic speckle; Metal 5.7 ms/frame (CPU 21 ms), 99.8% of disparities within 0.25 px of CPU |
| Tracking | working | robust point-to-plane ICP, normal-space balancing, degeneracy-aware updates (degenerate frames tracked but not fused live), global FPFH relocalisation on a background worker with 3-frame confirmation |
| Fusion | done (CPU reference + Metal) | sparse voxel-hash TSDF; Metal: GPU hash table + brick pool, integrate 0.7 ms, raycast 1.2 ms, full surface extraction 0.4 ms |
| ICP | done (CPU reference + Metal) | Metal runs all Gauss-Newton iterations in one command buffer, one dispatch each (the last threadgroup to finish solves; 1.3 ms vs 3.7 ms CPU); identical poses and Hessians to the CPU solver |
| Live view | done | Metal splats, frustum/trail, ghost frustum when lost, HUD, IR previews (`--snapshot` for headless capture), marker discs (map / global / current frame) and preview detections |
| Markers | done | detection on raw IR (blob + half-level contour + ellipse fit + dark-ring test), rectified stereo matching gated by a dense-depth disparity prior, marker map with candidate confirmation, RANSAC association, triangle-signature relocalisation, joint marker + surface ICP (CPU and Metal) |
| Session recording | done | every processed frame (depth, confidence, pose, flags, markers) to `.estr`, zstd, crash-tolerant; see docs/process.md |
| Process step | done | frame-level pose graph (chain + free-space-verified fragment registrations + loop closures + marker landmarks), island exclusion, lost-frame recovery, re-fusion, surface-nets mesh, cleanup, parallel quadric simplification, STL/PLY/OBJ export; CLI `process`, app Process panel. Mustang: 18.6 s; 93.0% of EXStar's reference within 0.5 mm |
| Global markers | done | markers-only constellation capture -> keyframes -> Ceres stereo-reprojection bundle adjustment -> fixed map that later scans start on; save/load as text |

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
1. **CPU use** (`einstar-bench pipeline`, emulated frames with markers, recording on): 24.1 → 3.6 ms of CPU per frame across all threads. The work moved to the GPU:
   - marker blob search: threshold, connected components, per-blob box, size, peak and moments, gates, and a dark-ring pre-check on the moment ellipse; about 100 candidates per frame reach the CPU's sub-pixel fit;
   - camera previews: textures copied GPU to GPU;
   - raw IR: the camera frames are used by the GPU in place (page-aligned image storage, no-copy buffers);
   - recording: depth packing on the GPU, zstd level 1.
   What remains on the CPU: the ellipse fits (~1.6 ms), zstd (~1 ms), tracker bookkeeping, marker hole filling, and relocalisation while lost.
2. **Speed**: done. Frames stay GPU-resident from stereo through ICP, fusion and rendering (speckle filter, points/normals, balancing weights, raycasts, surface extraction and the live overlay are all GPU buffers; the CPU only touches them for relocalisation and fallbacks). Full frame path 8.6 ms (was 12.9 ms with CPU round trips, ~50 ms all-CPU).
2. **Open-loop drift** on young models (synthetic sweep: ~0.03 deg/frame). Fix in the process step: keyframe pose graph + loop closure + re-fusion.
3. **Surfaces of revolution** are geometrically ambiguous; depth-only tracking holds but may slide. Degenerate frames are no longer fused live (surface fused at a slid pose became a ghost that broke tracking thousands of frames later); on long featureless parts the live model therefore stops growing and tracking is eventually lost. Use markers (hybrid / global markers) or, later, texture.
4. **Markers on real IR**: tuned on EXStar's calibration captures and synthetic stickers only. Real scans may need the detection threshold / ring test adjusted (live speckle brightness vs. retro-reflective return under the strobe is unknown until hardware). 3 mm markers are disabled by default (too close to speckle size).
5. **Process step**: done (docs/process.md). It includes lost-frame recovery, free-space-verified loop closures, island exclusion and error-bounded simplification; marker holes are filled in the frontend. Open: watertight (Poisson) meshing.
6. **Session recording**: done (depth, not raw IR; raw IR would be ~60 MB/s).
9. **Reproducibility**: fixed. Replays are bit-identical: GPU surface extraction returns a canonical order. That exposed the real problem: global relocalisation onto the 180°-symmetric counterpart of the mustang part. Global registration now reports ambiguous matches, and the tracker takes the candidate near where tracking was lost (or waits). Mustang: 95.7% tracked, p95 1.05 mm vs EXStar (a flipped run was 74%, p95 57 mm).
10. **Stereo outliers**: measured on synthetic speckle (`debug_depth_errors`), they are not concentrated at silhouettes (5% of the >2 mm errors are). About 7% of pixels have ≥1 px disparity errors spread over textured areas, which may be specific to our synthetic speckle. Fusion averages them out (0.16 mm mesh error). Retune only once real IR captures exist.
11. **IR-intensity / colour ICP term**: not implemented. The IR images are lit by the laser speckle, which moves with the scanner, so their intensity is not surface texture. A photometric term needs the RGB texture camera (or strobe-only IR frames), and there are no such recordings to validate it on.
7. Stereo outlier blobs at silhouettes need an extra consistency filter.
8. Hardware-only unknowns: exact LED distance-zone semantics, button codes 2/3, exposure units.

## First contact with a real scanner
`einstar-cli probe --verbose` performs read-only identification plus a calibration read and prints a full hex transcript.
