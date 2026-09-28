# Status

## Working (verified without hardware)

| Area | State | Evidence |
|---|---|---|
| USB transport (libusb) | done | codec/mask round trip on documented example; packet reassembly + resync tests |
| Device control | done | full command set via typed API; dangerous opcodes blocked at compile time; emulator tests match EXStar's logged bytes |
| Device emulator | done | speaks the protocol, streams packetised frames, serves flash, buttons, temperature |
| Calibration | done | decodes EXStar CCF / flash blob; matches documented values; baseline 159.913 mm |
| Rectification | done | row alignment < 1e-6 px on synthetic rig with distortion + toe-in |
| Stereo depth (CPU) | done | census + SGM (slant steps) + slanted-window ZNCC; 0.056 mm median error on synthetic speckle; ~20-25 ms at 640x512 output |
| Tracking | working | robust point-to-plane ICP, normal-space balancing, degeneracy-aware updates, extend-only fusion for weak frames, global FPFH relocalisation with 3-frame confirmation |
| Fusion | done | sparse voxel-hash TSDF, tile-bounded raycast, per-brick point extraction |
| Live view | done | Metal splats, frustum/trail, ghost frustum when lost, HUD, IR previews (`--snapshot` for headless capture) |

### Tracking on EXStar's own recordings (`einstar-cli track-fixture`)
mustang_differential, 6055 frames, depth-only (no markers/texture):
- tracked frames land on EXStar's final mesh with median 0.17 mm; ~95% of sampled frames fit cleanly
- per-frame relative error median 0.066 mm / 0.019 deg
- first 2000 frames: 90.6% tracked, pose within 1.26 mm (p95) of EXStar's optimised poses
- lost frames are dominated by the recording's own discontinuities (EXStar did not store the frames it lost), which live capture does not have

## Known issues / next steps
1. **Speed**: stereo + tracking run on the CPU (~50-60 ms/frame idle). Port stereo and TSDF raycast/integration to Metal (target < 30 ms total).
2. **Open-loop drift** on young models (synthetic sweep: ~0.03 deg/frame). Fix in the process step: keyframe pose graph + loop closure + re-fusion.
3. **Surfaces of revolution** are geometrically ambiguous; depth-only tracking holds but may slide. Needs markers (next) or texture.
4. **Markers**: detection, stereo matching, marker map, joint marker + ICP tracking — not started (EXStar's parameters are in docs/algorithms.md).
5. **Process step**: global optimisation, Poisson meshing, export (STL/PLY/OBJ) — not started.
6. **Session recording** of raw frames — not started.
7. Stereo outlier blobs at silhouettes need an extra consistency filter.
8. Hardware-only unknowns: exact LED distance-zone semantics, button codes 2/3, exposure units.

## First contact with a real scanner
`einstar-cli probe --verbose` performs read-only identification plus a calibration read and prints a full hex transcript.
