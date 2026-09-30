# Real-data test fixtures

Excerpts of real Einstar / EXStar data for the tests that need them. They are optional: without
them those tests are skipped. `tests/support/real_data.hpp` finds them in this order:

1. `$EINSTAR_FIXTURES` (a directory laid out like this one),
2. this directory,
3. the original data: `~/Documents/EXStar/mustang_differential` and EXStar's calibration captures in
   `/Applications/EXStar.app`.

| Directory | Tests | Contents |
|---|---|---|
| `mustang/` | `EXStar fixture frames reproduce EXStar's own mesh`, `fixture frames: model raycast and ICP between consecutive frames`, `global registration finds fixture frames in a model built from other frames` | The 137 frames of the `mustang_differential` EXStar recording that the tests read (`fixtures::mustang_test_frames()`), byte for byte as an EXStar project, with `manifest.txt` mapping them to the recording's frame indices, and EXStar's mesh cropped to the compared frames. |
| `calibration_board/` | `markers on the real calibration board triangulate to the board pitch` | EXStar's calibration-board captures (`imageLeft<k>.bmp`, `imageRight<k>.bmp`, k = 1..25). They must come from the scanner whose calibration is in `../calibration/einstar_e10`. |

All files except `manifest.txt` are zstd-compressed (`*.zst`) and unpacked into a temporary
directory by the tests.

## Making them

On the machine with EXStar and the recording:

    einstar-cli fixture-pack --out tests/fixtures/external \
        --mustang ~/Documents/EXStar/mustang_differential/Project1.ir_E10_prj \
        --stl ~/Documents/EXStar/mustang_differential/mustang_differential_simplified.stl \
        --board /Applications/EXStar.app/Contents/Frameworks/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/calibrate_image_read_rapid

It prints the size of each directory.
