# einstar

Open-source replacement for Shining3D's EXStar software for the Einstar handheld 3D scanner.
C++23, macOS / Apple Silicon first (Metal; Intel Macs with discrete GPUs work too), libusb device layer portable to Linux.

Workflow: connect -> live scan (depth, tracking, fusion, live view) -> process -> export.

## Build
    cmake --preset default && cmake --build --preset default && ctest --preset default

Requires Homebrew `ninja libusb eigen ceres-solver tbb zstd nlohmann-json` and the Xcode Command Line Tools.
GLFW, Dear ImGui, Catch2, metal-cpp and nanoflann are fetched by CMake. On Intel Macs Homebrew has no
prebuilt packages for several of these; build the missing ones with `brew install --build-from-source`.
Older Command Line Tools keep `std::jthread` experimental; CMake detects that and adds `-fexperimental-library`.

## Run
- `build/default/apps/Einstar.app` — GUI; uses the scanner if attached, otherwise the built-in emulator.
- `build/default/apps/EinstarCalibration.app` — guided calibration of the IR pair from the calibration board
  (EXStar's 25 views: live guidance, auto-capture, solve, comparison with the scanner's stored calibration,
  and "Write to scanner…", which stores the result in the scanner as EXStar's calibration does, after a
  backup, with read-back verification). Works with the emulator, or offline on a folder of captures. See
  docs/calibration.md §8.
- `build/default/apps/einstar-cli calib-solve <captures dir> --reference <calibration>` — the same solve and
  comparison from the command line (einstar-calibrate's or EXStar's `imageLeftN` folders).
- `build/default/apps/einstar-cli probe --verbose` — read-only scanner check (first contact with hardware).
- `build/default/apps/einstar-firmware version | inspect <package> | flash <package>` — the scanner's firmware
  version; check an update package offline; flash one (ours from `firmware/`, or EXStar's to go back).
  See docs/firmware.md §4.7.
- `build/default/apps/einstar-cli hw-test` — full scanner check (streams; turns the projector and strobe on).
- `build/default/apps/einstar-cli track-fixture <Project.ir_E10_prj> --stl <ref.stl>` — evaluate tracking on an EXStar recording.

## Safety
Flash writes, firmware update, reboot and bootloader commands are not sendable (compile-time guard,
`libs/device/include/einstar/device/opcodes.hpp`), except through the two dedicated, verified paths: the
calibration write (pages 0-1) and `einstar-firmware flash`. Laser/strobe are clamped to EXStar's maxima and
switched off on disconnect or destruction.

## Docs
Protocol and format notes from static analysis are in `docs/` (clean-room: behaviour described,
no vendor code). Current state: `docs/status.md`.
