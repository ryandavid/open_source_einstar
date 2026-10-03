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

## Known limits

- Only analytic surfaces: freeform (B-spline) patches for regions no primitive fits are not done yet.
- Holes are round, through or flat-bottomed. Countersinks, counterbores and a drill point's cone are not
  modelled yet.
- Hole positions are measured, not yet constrainable (e.g. "on a 38 mm pitch").
- Tolerances and the rim correction (`fit::HoleOptions::rim_bias_voxels`) are calibrated on synthetic
  surface-nets scans; real scans may need their own values.
