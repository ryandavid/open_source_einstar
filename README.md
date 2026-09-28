# einstar

Open-source replacement for Shining3D's EXStar software for the Einstar handheld 3D scanner.
C++23, macOS / Apple Silicon first (Metal), libusb device layer portable to Linux.

Workflow: connect -> live scan (depth, tracking, fusion, live view) -> process -> export.

## Build
    cmake --preset default && cmake --build --preset default && ctest --preset default

Requires Homebrew `libusb eigen ceres-solver tbb zstd nlohmann-json` and the Xcode Command Line Tools.
GLFW, Dear ImGui, Catch2, metal-cpp and nanoflann are fetched by CMake.

## Run
- `build/default/apps/Einstar.app` — GUI; uses the scanner if attached, otherwise the built-in emulator.
- `build/default/apps/einstar-cli probe --verbose` — read-only scanner check (first contact with hardware).
- `build/default/apps/einstar-cli track-fixture <Project.ir_E10_prj> --stl <ref.stl>` — evaluate tracking on an EXStar recording.

## Safety
Flash writes, firmware update, reboot and bootloader commands are not sendable (compile-time guard,
`libs/device/include/einstar/device/opcodes.hpp`). Laser/strobe are clamped to EXStar's maxima and
switched off on disconnect or destruction.

## Docs
Protocol and format notes from static analysis are in `docs/` (clean-room: behaviour described,
no vendor code). Current state: `docs/status.md`.
