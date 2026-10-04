# Einstar Model: scan to CAD

`build/apps/EinstarModel.app` turns a scan into a CAD solid with real faces, holes, fillets and edges, exported
as STEP. The user, an agent (through MCP), or both, label the scan's faces and state what they know: which
faces are square to each other, measured sizes, depths the scanner could not see. The app fits every face
under those constraints and builds the solid. A deviation map shows where the model departs from the scan.

It needs OpenCASCADE (`brew install opencascade`). Without it, CMake skips `libs/brep`, `libs/model` and the
app.

## Workflow

1. **Open** an `.estr` recording (processed first, as in the scanning app's Process step), an STL/PLY mesh, or
   a saved `.emodel`. The demo part, a box with mounting flanges, fillets and holes, needs no file.
2. **Label the faces.**
   - *Detect faces* proposes them all. It finds planes first, then cylinders, cones, spheres and tori. It
     sorts out fillets (cylinders running along the edge between two planes), hole walls (concave cylinders
     along a plane's normal) and fillet corners, and finds the holes in each plane.
   - To label a face by hand, Shift-drag paints a few mm of it into the selected label (Option-drag
     erases), and *Grow* extends the paint to the whole face. All labels grow at once, so neighbours meet
     at their common edge.
   - Name labels after what you call the faces; the agent and the STEP file use those names.
3. **Square the part.** Create a datum (z from the main face, x from a side) and use *Square to datum*. Every
   plane face is aligned to the nearest datum axis: one frame instead of many pairwise constraints, and the
   exported part sits in it.
4. **Add what the scan cannot know:**
   - the bottom the part stood on (*Add unseen face*);
   - a blind hole's depth, when its floor was not scanned;
   - measured hole diameters and fillet radii;
   - distances and offsets between faces.
5. **Solve.** Every face is fitted together, with the constraints held exactly. Each constraint reports:
   - its status: *satisfied*, *redundant* (implied by those before it), *conflict*, or *invalid*;
   - what it costs: how far it moved the surfaces off the scan.
6. **Build.** The solid is built, and the deviation map shows the scan against it:
   - green: within tolerance;
   - yellow to red: the scan lies outside the model;
   - cyan to blue: the scan lies inside the model.

   Hot spots list where the model departs from the scan. Thin *edge bands*, where the scan rounds sharp edges,
   are listed separately. If the faces do not enclose the part, the build says how much of the scan bounds the
   solid and asks for the missing face.
7. **Export STEP:** AP242, in mm, with each face named after its label. The file is read back to check it.

Every change is one undo step. An agent's change is one step for each request it carries out
(`model.begin_change` .. `model.end_change`), shown as the agent's in the History panel.

## How it works

- **`libs/fit`**: scan geometry.
  - Mesh topology, and a BVH for ray casts and closest points.
  - Robust primitive fits. The simplest surface kind that clearly explains the data wins.
  - Region growing from seeds, and automatic detection.
  - Holes, found from their opening in the host face.
  - The constraint solver: equality-constrained Gauss-Newton, which holds constraints exactly. Datums are
    solved with the faces. Direction conflicts are caught before the solve; covariance gives the
    uncertainties.
  - Deviation statistics and hot spots.
  - Synthetic parts with known geometry, for the tests.
- **`libs/brep`**: the solid (OpenCASCADE, kept private).
  - The faces are extended past the part, and they cut space into cells.
  - A graph cut picks the material cells. Each piece of a face votes with its scan points, through the scan
    normals, for the cell on its material side. Making a boundary where the scan saw nothing costs that
    piece's area, so cells hidden inside the part follow their neighbours.
  - Blocks are added or cut after the cells are joined (see below).
  - Fillets are rolled on the edges between their faces, and holes are cut, after the cells are joined.
  - STEP is written with face names.
- **`libs/model`**: the document.
  - Labels, holes, fillets, datums and constraints.
  - The commands, which are the only way to change the document.
  - Undo.
  - `.emodel` files: the source's path, and a copy of the mesh the labels point at, in tagged records like an
    `.estr`.
  - Settings, kept in `~/Library/Application Support/Einstar/`.

## Agents

Every command is a `model.*` agent method (`libs/agent/src/specs.cpp`), so it is also an `einstar-mcp` tool.
`model.view` and `ui.screenshot` let an agent look at the result. There are two ways in:
- **Headless:** `app_launch(app="model")`.
- **Alongside the user:** `app_attach(app="model")` works in the window the user has open. The app listens on
  `~/Library/Application Support/Einstar/model-agent.sock`, which only this user can open.

The MCP server's instructions describe the modelling workflow. A typical exchange:
1. "Make the box faces perpendicular": `model_datum_create` + `model_square`.
2. "The flange holes measure 5.5 mm": `model_hole_update`.
3. `model_solve`, `model_build`.
4. `model_deviation`, a screenshot, and a report of any conflicts and of what each measurement cost.

## Photos

Photos of the part keep what the scan cannot show: caliper readings, threads, the material, how it fits its
assembly. They can be taken anywhere. Nothing assumes they show the scan's setting.
- **Storage:** the photos live in the `.emodel`, as the files were (JPEG, HEIC, PNG), in one `PHOT` record per file.
- **Annotations:** each one is drawn in the photo's pixels (as shown, after EXIF orientation) and named:
  - a dimension (two points and a length), named D1, D2, ...;
  - a diameter (three points on a rim, or a centre and the rim), named DIA1, ...;
  - an angle, named A1, ...;
  - a callout (an arrow with text and/or a value), named C1, ...;
  - a note pinned to a point, named N1, ....
- **Values** are read as typed: `42`, `1 1/2"`, `3/8 in`, `Ø6 ±0.02`, `R2`, `30°`. A typed value is also kept as
  entered.
- **Links:** an annotation can name the labels, holes or fillets it is about.
- **Notes** (`model.note.*`) keep facts about the part as a whole.
- **For the agent:**
  - `model.summary` lists the photos, their annotations and the notes;
  - `model.photo.list` gives the annotations in full;
  - `model.photo.get` returns a photo as an image, scaled down, with its annotations drawn and named, optionally
    cropped or with a grid of pixel coordinates.

  The agent annotates too, to keep what the user tells it attached to a picture.

### Registering a photo to the scan

*Match to scan* (`model.photo.correspond`) pairs a point of the part in the photo with the same point on the
scan: click it in the photo, then on the scan in the 3D view. With 6 pairs, or 4 when EXIF gives the focal
length, the photo's camera is solved, relative to the part only.
- **Spread the points:** pick corners and edges that are easy to place exactly, spread over the part.
- **Background is left out:** points on scan labelled `ignore` are refused, and that scan hides nothing. Label
  leftover background `ignore` when a photo was taken somewhere else.
- **How it's solved:** `fit::fit_camera` is a pinhole with one radial term and the principal point at the image
  centre.
  - It starts from views all round the part, each with a weak-perspective fit for roll, distance and offset.
    This works with coplanar points and with 4 points.
  - It then refines with Levenberg–Marquardt and a Huber loss.
  - A pair the others disagree with is flagged as an outlier.

**Once registered:**
- The model's edges, or the label boundaries before a build, are drawn over the photo. `model.photo.get` can
  draw them too, with `overlay`.
- Each annotation reads on the scan:
  - its points on the part;
  - what the scan measures. A diameter is measured where its rim rays cross the face, because the scan rounds
    rims;
  - suggested links (the labels under its points, and a hole at its centre).
- `model.photo.project` maps a pixel to a scan point and label, and a scan point back to a pixel.
- The photo's camera is drawn in the 3D view, and *View from photo* takes the 3D view to it.

### Photo colour on the scan

The *Photos* view mode (`model.view mode=photos`) colours the scan from its registered photos. Each vertex
takes its colour from the photo that sees it most head-on, where nothing of the part hides it.
- Background (scan labelled `ignore`) stays grey, as does what no photo sees.
- A photo where something covers the part (a hand, the assembly) can be left out with *use for colour*
  (`model.photo.update`).
- The colours are made in the background, and again only when a camera or the scan changes.

### Applying a measurement

*Apply* (`model.photo.apply`) makes an annotation's value hold on the model. What it becomes depends on its
links:

| Linked to | Becomes |
|---|---|
| two plane faces | a `distance` |
| two holes or cylinders | an `axis_distance` (a pitch) |
| a hole, with a diameter or a value typed `Ø…` | the hole's `diameter` |
| a cylinder | its `diameter`, or its `radius` when typed `R…` |
| a fillet, typed `R…` | the fillet's radius |
| two faces, with an angle | an `angle` |

- After a solve, the annotation shows how its constraint fared: its status and how far it moved the scan fit.
- Applying again replaces what was applied before.
- Applied photo measurements count toward the scale check like any other measurement.

## When the scan changes

The model keeps its own copy of the mesh, so a changed source cannot move the labels underneath it. When
the source file has changed since, e.g. an `.estr` opened again in the scanning app and scanned further or
edited, the panel says so (`model.summary`'s `scan.source` is `changed`). *Process it again*
(`model.reprocess`) then:
1. makes the mesh again;
2. gives each new triangle the label of the nearest old triangle (within two voxels, facing the same way);
3. moves hole openings to the nearest new vertices;
4. refits every fitted face to its carried region, reporting how far each one moved.

Datums, constraints, measurements and unseen faces stay as they are. Solve and build again afterwards. A
`model.scale` correction already applied carries over to the new mesh. The undo history is cleared.

## Holes

- **Finding them:** holes are found as round openings in a plane face. The diameter comes from the scanned
  wall, or from the opening when little wall was seen.
- **Holes the scan bridged over:** a small hole often comes out filled in, so there is no opening to find.
  `model.hole.add` (*Make a hole* on a diameter marked on a photo) places one:
  - from a diameter on a registered photo. Its rim is taken where the photo's rays cross the face, so what the
    scan shows there does not matter. The annotation then links the hole, and its value becomes the hole's size;
  - or at a centre point, with a diameter.

  The face is the one given, else the one the annotation links, else the plane face under its rim. A placed
  hole's diameter is not a scan measurement, so the scale check leaves it out.
- **Forms:** the scan's counterbores, countersinks and drill points are measured. `model.hole.update` sets
  or corrects them, and they are built and named in the STEP file.
- **In the solve:** holes take part through their axis: the wall's label, or a cylinder fitted to the
  opening, or for a placed hole an axis held by its constraints alone. They are held parallel to their face's
  normal. Constraints that mean an axis can name a hole, e.g. `axis_distance` for a pitch or a bolt circle's
  radius, `symmetric` for a mirrored pair, or `offset` from the datum.

## Blocks

Every face splits the whole part into cells, and the scan votes on which cells are material. That cannot
make a feature the scan barely saw, such as a clip whose faces are mostly hidden, and it cannot make a hollow
one. An unseen face added across the part (`model.face.add_plane`) would also cut through everything else.

A block (`model.block.add`) is a region bounded by planes:
- **Faces:** its own plane labels, usually unseen faces added with `model.face.add_plane`. The block lies on
  each one's material side. Its faces bound only the block: they do not split the rest of the part.
- **Bounds:** other planes that limit it, which stay in the part. `side: outside` puts the block past one.
  For example, a clip on the back is bounded past the back's step, so it overlaps the body it stands on.
- **Pocket:** `cut: true` removes the region instead of adding it, e.g. the inside of a hollow tab. Give a
  pocket's walls facing out of the pocket. A side with no wall is open.

Blocks are added or cut after the cells are joined, before fillets and holes.

## Constraints

| Constraint | Meaning |
|---|---|
| aligned | along a datum axis |
| parallel, perpendicular, angle | between directions |
| coplanar | two planes on one plane |
| coaxial | two axes on one line |
| distance | between parallel planes |
| axis_distance | between parallel axes |
| offset | a position along a datum axis |
| radius, diameter | a size |
| tangent | a plane and a cylinder or sphere, or two cylinders |
| symmetric | mirrored about a datum plane |
| equal_radius | the same radius |

These give, in 3D, what a sketch would give a prismatic or turned part.

## Freeform faces

`model.freeform` makes a label a B-spline height field over its best-fit plane. It fits a region no primitive
describes, such as a domed or sculpted top. The face reaches past its region so that it meets its neighbours.
It is held as fitted in the solve, built by OpenCASCADE as the same spline, and exported as a B-spline face.

## Scale

`model.solve` compares the user's measured lengths with what the scan alone gives, and reports the factor
they imply and whether it is significant. `model.scale` applies a factor to the scan and everything on it.
A significant factor is also a reason to check the scanner's calibration.

## Known limits

- **Freeform faces must be height fields over a plane.** A region that folds over itself has to be split.
- **Sketch-and-extrude is not built as such.** Its relations are the constraints above, on 3D faces.
- **Calibration is synthetic only:** tolerances and the rim correction (`fit::HoleOptions::rim_bias_voxels`) are
  tuned on synthetic surface-nets scans, and real scans may need their own values.
- **Untested on a real machined part:** the real scans tried so far (a display and bucket, a cast differential)
  are not machined parts. Detection runs on them in 6-19 s.
