# Recording and the process step

## Session recording (`.estr`, `libs/session`)

Every frame the live pipeline processes is written to a session file. The app writes to
`~/Documents/Einstar/Scans/scan-<date>-<time>.estr`, or to `$EINSTAR_SCAN_DIR` if that is set. Each
frame record holds:
- its depth, as a 16-bit value in 1/50 mm steps, delta-coded along rows and zstd-compressed;
- its stereo confidence (8-bit);
- the live pose and tracking flags: accepted, degenerate, relocalised, marker pose, fused, global-marker capture;
- its markers: camera-frame position, normal, diameter, live map id, and rectified left/right centres.

Around 170 KB per frame, about 2.5 MB/s at the scan rate.

A `GMRK` record stores the global-marker map in use. Records are length-prefixed, so a file truncated
by a crash still loads up to its last complete frame.

A new file starts whenever the tracker's world frame restarts: Clear model (which also deletes the
discarded recording), a new marker capture, or loading or discarding a global-marker map. Pausing
and resuming a scan continues the same file.

## Process step (`libs/recon`, `einstar-cli process`, the app's Process panel)

1. **Fragments.** Consecutive tracked frames are grouped into fragments of 40 frames; a tracking gap
   also starts a new fragment. Each fragment becomes an oriented point cloud (1 mm voxels) in the
   frame of its middle frame, called the *anchor*.
2. **Pose graph** (`optim::PoseGraph`, Ceres), with one node per tracked frame:
   - **Chain edges** link consecutive frames with their live relative pose, with σ 0.15° / 0.3 mm per
     frame. The drift they carry is systematic, so they must stay loose. Across a relocalisation or a
     gap the edge is 100× weaker.
   - **Registrations** between overlapping fragments, as edges between their anchors: consecutive
     fragments, plus loop closures between any pair whose clouds overlap at the current estimate.
     Each is point-to-plane ICP and is accepted on fitness, rms, conditioning and a maximum
     correction. Its information matrix comes from the ICP Hessian: σ 0.15 mm, with at most 100
     correspondences counted as independent. Weakly constrained directions (sliding surfaces) are
     therefore down-weighted automatically.
   - **Markers** are landmarks observed by every frame that identified them (σ 0.08 mm). They are
     fixed when a global-marker map was recorded, so the scan is anchored to it.
   - Loop closures whose whitened error exceeds 25 after a solve are pruned one at a time, and the
     graph is re-solved from the live poses.
   - Loop closures are searched again after each solve: once nearer loops are closed, far ones come
     within reach.
   - The fragments are then rebuilt from the optimised poses, re-registered and re-solved (two
     iterations). This removes the bias of fragments that were smeared by drift inside them.
3. **Islands.** A group of fragments linked to the rest only by live tracking across a
   relocalisation has no verified registration and no shared marker. If it contradicts the main
   model where they overlap, it is left out of the fusion. This is the signature of a wrong live
   relocalisation.
4. **Re-fusion** of every tracked frame at its optimised pose (Metal TSDF, 0.5 mm voxels by default;
   0.3 mm in the app's fine mode). Frames the tracker flagged as degenerate count half.
5. **Mesh.** Surface nets on the zero level: one vertex per surface cell, placed by one Newton step
   onto the trilinear zero level, with quads split along the diagonal that agrees with the SDF
   normals. Pieces smaller than 2% of the largest are removed. Optional Taubin smoothing. Export to
   binary STL, binary PLY (with normals) or OBJ.

### Results

| Case | Result |
|---|---|
| Synthetic closed loop (72 frames, drift injected into the live poses: 1.4° / 3.5 mm around the loop) | 0.29 mm / 0.08° after processing; mesh within 0.06 mm of the analytic scene |
| Emulator end to end (global-marker capture, surface scan, recording, process) | mesh 0.16 mm from the scene: the stereo depth limit, same as fusing at the live poses, which are already anchored by the markers |
| mustang_differential (5310 tracked frames, 141 fragments, ~950 loop closures, ~40 s), vs EXStar's reference STL | accuracy median 0.14 mm, p95 0.39 mm; 91.8% of the reference within 0.5 mm, 95.7% within 1 mm |
| same, vs EXStar's poses after rigid alignment | median 0.44 → 0.31 mm, p95 1.00 → 0.60 mm |
| PG2/Project1 (markers) | marker observations are more self-consistent under the processed poses (spread 0.040 mm median) than under EXStar's own (0.063 mm) |

### Known limits
- Frames live tracking never accepted are not recovered; registering them against the final model
  is a natural next step.
- The mesh keeps a small dimple where each marker sticker left a hole in the depth; EXStar fills
  these.
- Holes are not filled: there is no watertight (Poisson) mode yet.
