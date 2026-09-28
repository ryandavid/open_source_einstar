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
| Tracking | working | robust point-to-plane ICP, normal-space balancing, degeneracy-aware updates, extend-only fusion for weak frames, global FPFH relocalisation with 3-frame confirmation |
| Fusion | done (CPU reference + Metal) | sparse voxel-hash TSDF; Metal: GPU hash table + brick pool, integrate 0.7 ms, raycast 1.2 ms, full surface extraction 0.4 ms |
| ICP | done (CPU reference + Metal) | Metal runs all Gauss-Newton iterations in one command buffer (1.6 ms vs 3.7 ms); identical poses and Hessians to the CPU solver |
| Live view | done | Metal splats, frustum/trail, ghost frustum when lost, HUD, IR previews (`--snapshot` for headless capture), marker discs (map / global / current frame) and preview detections |
| Markers | done | detection on raw IR (blob + half-level contour + ellipse fit + dark-ring test), rectified stereo matching gated by a dense-depth disparity prior, marker map with candidate confirmation, RANSAC association, triangle-signature relocalisation, joint marker + surface ICP (CPU and Metal) |
| Session recording | done | every processed frame (depth, confidence, pose, flags, markers) to `.estr`, zstd, crash-tolerant; see docs/process.md |
| Process step | done | frame-level pose graph (chain + free-space-verified fragment registrations + loop closures + marker landmarks), island exclusion, lost-frame recovery, re-fusion, surface-nets mesh, cleanup, parallel quadric simplification, STL/PLY/OBJ export; CLI `process`, app Process panel. Mustang: 18.6 s; 93.0% of EXStar's reference within 0.5 mm |
| Global markers | done | markers-only constellation capture -> keyframes -> Ceres stereo-reprojection bundle adjustment -> fixed map that later scans start on; save/load as text |

### Tracking on EXStar's own recordings (`einstar-cli track-fixture`)
mustang_differential, 6055 frames, depth-only (no markers/texture), GPU path:
- 92.3% of frames tracked; pose within 1.62 mm (p95) of EXStar's globally optimised poses
- tracked frames land on EXStar's final mesh with median 0.18 mm; 2.3% of sampled frames misfit
- 6.7 ms per frame for tracking + fusion (CPU reference: ~28 ms)
- lost frames are dominated by the recording's own discontinuities (EXStar did not store the frames it lost), which live capture does not have

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
1. **Speed**: done. Frames stay GPU-resident from stereo through ICP, fusion and rendering (speckle filter, points/normals, balancing weights, raycasts, surface extraction and the live overlay are all GPU buffers; the CPU only touches them for relocalisation and fallbacks). Full frame path 8.6 ms (was 12.9 ms with CPU round trips, ~50 ms all-CPU).
2. **Open-loop drift** on young models (synthetic sweep: ~0.03 deg/frame). Fix in the process step: keyframe pose graph + loop closure + re-fusion.
3. **Surfaces of revolution** are geometrically ambiguous; depth-only tracking holds but may slide. Use markers (hybrid / global markers) or, later, texture.
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
