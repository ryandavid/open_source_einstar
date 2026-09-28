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
| Live view | done | Metal splats, frustum/trail, ghost frustum when lost, HUD, IR previews (`--snapshot` for headless capture) |

### Tracking on EXStar's own recordings (`einstar-cli track-fixture`)
mustang_differential, 6055 frames, depth-only (no markers/texture), GPU path:
- 92.3% of frames tracked; pose within 1.62 mm (p95) of EXStar's globally optimised poses
- tracked frames land on EXStar's final mesh with median 0.18 mm; 2.3% of sampled frames misfit
- 6.7 ms per frame for tracking + fusion (CPU reference: ~28 ms)
- lost frames are dominated by the recording's own discontinuities (EXStar did not store the frames it lost), which live capture does not have

ICP is linearised about the centroid of the observed surface (not the world origin), which keeps
the float GPU solve well conditioned and makes the degeneracy analysis independent of where the
scan started; the degeneracy/relocalisation thresholds were recalibrated for it.

## Known issues / next steps
1. **Speed**: done for the main path (stereo 5.7 ms + tracking/fusion 6.7 ms). Remaining: keep depth frames on the GPU between stereo, ICP and fusion (today they round-trip through CPU memory, ~1 ms each); renderer could draw the extracted surface buffer directly.
2. **Open-loop drift** on young models (synthetic sweep: ~0.03 deg/frame). Fix in the process step: keyframe pose graph + loop closure + re-fusion.
3. **Surfaces of revolution** are geometrically ambiguous; depth-only tracking holds but may slide. Needs markers (next) or texture.
4. **Markers**: detection, stereo matching, marker map, joint marker + ICP tracking — not started (EXStar's parameters are in docs/algorithms.md).
5. **Process step**: global optimisation, Poisson meshing, export (STL/PLY/OBJ) — not started.
6. **Session recording** of raw frames — not started.
7. Stereo outlier blobs at silhouettes need an extra consistency filter.
8. Hardware-only unknowns: exact LED distance-zone semantics, button codes 2/3, exposure units.

## First contact with a real scanner
`einstar-cli probe --verbose` performs read-only identification plus a calibration read and prints a full hex transcript.
