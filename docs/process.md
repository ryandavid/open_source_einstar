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
  marker algorithms can be re-run on the scan: `einstar-cli replay <scan.estr> -o <new.estr>` feeds
  them through this build's live pipeline (stereo, markers, tracking, fusion) in order, none dropped,
  into a new recording (a recorded marker capture is replayed and bundle-adjusted as one, or
  `--recorded-map` keeps the recorded map and replays only the surface). Replay the same scan before
  and after a front-end change, then `process` / `inspect` both. The writer drops raw frames rather
  than growing memory if the disk falls behind, and the HUD counts them.

The scanner starts its frame count and clock over whenever streaming restarts (a settings change,
the marker capture ending); the pipeline continues both past the last frame instead, so frame numbers
in a recording are unique and time only moves forward.
- `GMRK`: the global-marker map in use.
- `ERAS` / `UNDO`: a paused scan's lasso delete and its undo. An erase holds the lasso strokes (each a
  screen polygon with the view it was drawn in; everything that projects inside it, front to back, is
  selected; later strokes add or subtract). It applies to every frame before it in the file: reading a
  tracked frame drops the depth whose point, at the frame's live pose, the selection contains; frames
  live tracking lost get it at the pose processing recovers for them. Frames after it are untouched,
  so rescanning an erased area brings it back. The writer flushes before an erase, so it always follows
  the frames it covers.

- `RESM`: where a recording opened again (below) continues: how many frame records came before, and a
  note.

Readers skip record types they do not know, so new kinds can be added without breaking old files or
old readers. Records are length-prefixed, so a file truncated by a crash still loads up to its last
complete record. `einstar-cli inspect <file>` summarises a session file.

**Editing a paused scan.** The live volume clears the selected voxels (the surface and its truncation
band, so tracking no longer aligns to them either) and keeps them for undo until the next frame is
fused; the same erase goes to the recording as above, so processing never sees the deleted data. The
selection test is shared by the CPU, the live volume and the renderer's highlight
(`core/lasso.hpp`, `lasso.metal.inc`): each stroke is rasterised once into a pixel mask. Markers are
not edited. After a delete, a remnant cut off from the rest of the surface may be dropped by the process
step's clean-up of small pieces.

A new file starts whenever the tracker's world frame restarts: Clear model (which also deletes the
discarded recording), a new marker capture, or loading or discarding a global-marker map. Pausing
and resuming a scan continues the same file.

**Opening a recording** (Open a recorded scan..., `scan.open`). Opening a recording makes it a paused scan
again, with or without a scanner (`ScanPipeline::load_recording`):
- The live model is rebuilt from the frames live tracking fused, at their recorded poses, with erases
  applied. The marker map is rebuilt from the recorded global map plus every other marker, averaged over
  its sightings.
- It can be viewed, edited with the lasso, and processed with no scanner connected. Its edits are appended
  to the same file.
- With the scanner it was made with (the same serial and rectification), Resume continues it. The tracker
  starts out lost at the last recorded pose (`Tracker::resume`) and relocalises against the rebuilt model,
  so the new frames share the old ones' world frame. They are appended to the same file
  (`SessionWriter::append`, which first trims a record cut short by a crash).
- The first new frame writes a `RESM` mark. New frames number on from the last recorded index, and their
  timestamps start 10 s after the last one. The process step therefore sees a tracking gap at the join and
  starts a new fragment there; loop closures tie the two sessions together.
- A recording made with another scanner or calibration can be viewed, edited and processed, but not
  continued; New scan starts a new one.

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
   - **Edge noise.** Stereo depth is least reliable along object outlines (the foreground's disparity
     smeared over the silhouette, pixels straddling a depth step), which fused into a fringe of flakes
     around objects. Settings follow EXStar's E10 range-image processing (BuildSetting.ini
     `[ProcessDataSection]`: maxCameraAngle 70, boundaryWidth 2 with boundaryAngleOnly, minPatchSize
     100) and its realtime fusion, whose voxels record the frames that saw them:
     - per frame (`track::filter_depth_edges`, `track::filter_grazing`): depth within 2 px of a depth
       step removed; connected regions under 100 px dropped; surface seen more obliquely than 70
       degrees dropped (also pixels too steep to have a normal); the 2 px border of a depth region
       dropped where the surface there is steeper than 45 degrees (a face-on border stays);
     - fusion weight = the recorded confidence, which the depth front end computes with the viewing
       angle in it (stereo score x cos(viewing angle)); recordings without one get cos(viewing angle).
       (The process step used to multiply by the cosine again; `--grazing-weight` still does.)
     - extraction: surface only where at least 5 frames observed it (`TsdfParams::count_observations`,
       `ExtractParams::min_observations`; the scanner gives ~40 frames per second).
     On a real scan (display + glossy bucket, 2026-10-01) the display's outline went from a ragged
     fringe to a clean edge and the floating debris went; the glossy bucket lost some sparse coverage.
     Tried and rejected: eroding the border of every depth region (the depth is sparse, so every hole
     has a border; it cost surface and changed registration) and a 10-frame minimum (it fragmented
     glossy surface, so the small-piece cleanup, relative to the largest piece, kept a stray flap).
     `einstar-cli process` switches: `--no-edge-filter`, `--edge-radius`, `--rim-radius`, `--min-region`,
     `--no-grazing-filter`, `--max-view-angle`, `--steep-rim`, `--steep-rim-angle`, `--grazing-weight`,
     `--min-observations`, `--min-weight`, and `--render out.pgm [--render-frame view.txt]` (a shaded
     view, identical across runs) to compare.
   - **Model consistency** (`recon::ConsistencyModel`). The fused model, meshed as the output would
     be, is the consensus every frame is then checked against from its own camera (a z-buffer of the
     mesh, rendered after a 0.02 mm decimation); the pixels it contradicts are dropped and all frames
     are fused again. A pixel goes when the model surface its ray meets faces the camera and the
     pixel lies in front of it (it floats in space the views that built that surface looked through)
     or behind it (that surface should have hidden it), by more than 0.3 mm + 0.7 mm x (z / 400 mm)^2
     (pose and calibration residuals plus stereo noise, which grows with z^2; along the surface
     normal, and at most four tolerances along the ray), unless another model surface facing the
     camera lies that close within 1.5 mm (a silhouette the model places slightly differently, a
     surface behind a flake of the model). A pixel on the surface whose normal is more than 60
     degrees off the model's goes too (flying pixels at steps). A pixel whose ray meets no model, or
     the back of a model surface, is kept: the camera then looks at the far side of a part the model
     only has one face of (a thin wall), and nothing says the space is empty. EXStar does the same
     job with a distance field of its live model (every pixel not near it is deleted); this test
     knows from which side each surface was seen. On the car display 3.8-4.0% of the depth pixels
     go (1.5% of EXStar's own depth), most of them along depth borders; the glass's tail beyond
     0.5 mm went from 0.51% to 0.17% of its area, the dark back plate's from 6.3% to 5.4% (044655)
     and 4.0% to 3.1% (044347), with the same completeness; it costs ~5 s on 2357 frames. A second
     round against the cleaner model changed nothing measurable; a 0.6 mm tolerance at 400 mm cleaned
     the back plate a little more but thinned the glass, 1.5 mm kept more of the tails; a 45 degree
     normal test cost sparse surface on the glossy bucket. Switches: `--no-consistency`,
     `--consistency-tol`, `--consistency-noise`, `--consistency-radius`, `--consistency-normal`.
5. **Mesh.** Surface nets on the zero level: one vertex per surface cell, placed by one Newton step
   onto the trilinear zero level, with quads split along the diagonal that agrees with the SDF
   normals. Optional Taubin smoothing.
   - **Cleanup.** Pieces are triangles joined across edges two triangles share, so a flap on a
     non-manifold edge is a piece of its own (surface nets make a few hundred where sheets touch).
     Pieces under 25 mm^2 are removed (EXStar's figure at 0.5 mm), and so are pieces up to 1% of the
     total area with no larger piece within ~50 mm (floaters, the background). The rule relative to
     the largest piece (2%) is off: on the car display it removed real surface next to the model (parts
     seen through openings in the back plate, the bucket's rim; `--min-component 0.02` restores it).
     No spike removal: no vertex of a surface-nets mesh lies more than an edge length off its
     neighbours' plane. Switches: `--min-area`, `--isolation`, `--min-component`.
   - **Marker stickers.** The front end fills a sticker's hole in the depth with a plane, but the
     fused surface over it still had a bump or dent (0.6-1.7 mm over ~10 mm on the bucket of
     044347). Each identified marker's surface, out to 2.2 sticker radii, is replaced by a quadric
     fitted to the 5 mm annulus outside, blended in over 2 mm, where that annulus is smooth (rms <=
     0.2 mm, data on all sides): 17 of 62 markers on 044347 (`--no-marker-flatten`). EXStar cuts the
     surface within 6 mm of a marker and leaves the hole.
   - **Pinholes**: holes whose boundary is a simple loop of at most 2.5 mm are closed with a fan
     (one or two missing cells; ~260 per scan). A real 1 mm hole, the smallest the model app looks for,
     is 3.1 mm round. `--fill-holes MM` (EXStar's meshing closes up to 10 mm).
   - **Watertight** (optional, `ProcessParams::watertight`, `--watertight closed.stl`): a second,
     closed mesh by screened Poisson reconstruction of the mesh's vertices and normals (PoissonRecon,
     MIT, finest cell = the voxel size), cleaned and decimated like the mesh: 044655 in 17.6 s, no
     boundary or non-manifold edges.
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
- Holes larger than pinholes stay open in the mesh; the watertight mesh closes everything, with a
  smooth guess where the scan saw nothing.
- Marker holes are filled live (in the stereo frontend), so recordings replayed from EXStar projects
  keep whatever EXStar recorded there.
