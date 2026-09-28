# Recording and the process step

## Session recording (`.estr`, `libs/session`)

Every frame the live pipeline processes is written to a session file. The app writes to
`~/Documents/Einstar/Scans/scan-<date>-<time>.estr`, or to `$EINSTAR_SCAN_DIR` if that is set. Each
frame record holds:
- its depth, as a 16-bit value in 1/50 mm steps, delta-coded along rows and zstd-compressed;
- its stereo confidence (8-bit);
- the live pose and tracking flags: accepted, degenerate, relocalised, marker pose, fused, global-marker capture;
- its markers: camera-frame position, normal, diameter, live map id, and rectified left/right centres.

Around 170-250 KB per frame, about 2.5-3.7 MB/s at the scan rate.

Alongside the frames:
- `DEVC` (once per file): the scanner's vendor, product, serial and firmware, its calibration block
  as read from flash (6568 bytes), the decoded rig, and the rectification used live. Enough to
  re-rectify raw images and to tie a scan to a unit.
- `FXTR` (after each frame): which stream sensor is the left camera, the capture settings last sent
  (exposure and gain read back per sensor, laser, strobe, trigger period, last temperature reading)
  and the tracker's view of the frame (state, ICP rms / inlier ratio / coverage / eigen ratio,
  correspondences, degenerate directions, markers seen, stereo and tracking time, rejection reason).
- `DROP`: every frame that reached the host but has no frame record, with the reason (the live
  queue overflowed, or the image group was incomplete), so the timeline is complete.
- `RAWI` (optional, the app's "Keep raw IR images", off by default): both raw 1280x1024 IR images
  of every frame that reaches the host, recorded on arrival (so dropped frames keep theirs), each
  row delta-coded and zstd-compressed: 1.5-1.9x, about 20-25 MB/s. With these, future stereo and
  marker algorithms can be re-run on the scan. The writer drops raw frames rather than growing
  memory if the disk falls behind, and the HUD counts them.
- `GMRK`: the global-marker map in use.

Readers skip record types they do not know, so new kinds can be added without breaking old files or
old readers. Records are length-prefixed, so a file truncated by a crash still loads up to its last
complete record. `einstar-cli inspect <file>` summarises a session file.

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
   - **Free-space check.** Every registration, odometry or loop, must also pass a free-space
     test. Points of either fragment may not lie in space the other fragment's anchor camera
     observed as empty. A dominant plane (a table) can make a wrong alignment fit very well, and
     the rest of the geometry then floats in observed free space. At most 10% of such points are
     tolerated, within 3 mm.
   - **Loop candidates.** Each fragment tries only its 10 nearest overlapping partners, and a pair
     that failed is retried only after its relative-pose estimate has moved.
3. **Islands.** A group of fragments linked to the rest only by live tracking across a
   relocalisation has no verified registration and no shared marker. It is left out of the fusion
   if it violates the main model's observed free space; that is the signature of a wrong live
   relocalisation.
   - **Lost-frame recovery.** Frames live tracking rejected are re-tracked against the complete
     model. Each run of lost frames is walked inward from both ends, and a frame is accepted only
     on good fitness, rms and conditioning.
4. **Re-fusion** of every tracked frame at its optimised pose (Metal TSDF, 0.5 mm voxels by default;
   0.3 mm in the app's fine mode). Frames the tracker flagged as degenerate count half.
5. **Mesh.** Surface nets on the zero level: one vertex per surface cell, placed by one Newton step
   onto the trilinear zero level, with quads split along the diagonal that agrees with the SDF
   normals. Pieces smaller than 2% of the largest are removed. Optional Taubin smoothing.
   - **Simplification** (on by default): quadric edge collapse bounded at 0.02 mm area-averaged
     deviation, keeping the surface manifold and borders fixed. Large meshes are processed in
     parallel 40 mm blocks with shared vertices locked, then a half-shifted pass cleans up the seams.
   - Export to binary STL, binary PLY (with normals) or OBJ.

### Results

| Case | Result |
|---|---|
| Synthetic closed loop (72 frames, drift injected into the live poses: 1.4° / 3.5 mm around the loop) | 0.29 mm / 0.08° after processing; mesh within 0.06 mm of the analytic scene |
| Emulator end to end (global-marker capture, surface scan, recording, process) | mesh 0.16 mm from the scene: the stereo depth limit, same as fusing at the live poses, which are already anchored by the markers |
| mustang_differential (5796 tracked + 247 recovered frames, 148 fragments, 627 loop closures, 18.6 s), vs EXStar's reference STL | accuracy median 0.12 mm, p95 0.39 mm; 93.0% of the reference within 0.5 mm, 96.1% within 1 mm; 5.6 M triangles simplified to 0.63 M within 0.02 mm |
| same, vs EXStar's poses after rigid alignment | median 0.36 → 0.32 mm, p95 0.66 → 0.61 mm; recovered frames 0.47 mm median |
| synthetic segment placed by a wrong relocalisation | detected (25% free-space violation) and excluded exactly |
| PG2/Project1 (markers) | marker observations are more self-consistent under the processed poses (spread 0.040 mm median) than under EXStar's own (0.063 mm) |

### Known limits
- Holes are not filled: there is no watertight (Poisson) mode yet.
- Marker holes are filled live (in the stereo frontend), so recordings replayed from EXStar projects
  keep whatever EXStar recorded there.
