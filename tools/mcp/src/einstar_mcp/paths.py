"""Where things are: the repository, its build, the apps, the method table."""

from __future__ import annotations

import os
import pathlib


def repo_root() -> pathlib.Path:
    """EINSTAR_REPO_ROOT, else the first directory up from here with .git and CMakeLists.txt."""
    if env := os.environ.get("EINSTAR_REPO_ROOT"):
        return pathlib.Path(env).resolve()
    here = pathlib.Path(__file__).resolve()
    for d in here.parents:
        if (d / ".git").exists() and (d / "CMakeLists.txt").exists():
            return d
    raise RuntimeError(f"cannot find the einstar repository above {here}; set EINSTAR_REPO_ROOT")


def build_dir() -> pathlib.Path:
    """EINSTAR_BUILD_DIR, else build/ in the repository (a plain `cmake -B build`)."""
    if env := os.environ.get("EINSTAR_BUILD_DIR"):
        return pathlib.Path(env).resolve()
    return repo_root() / "build"


# app -> (binary in the build, CMake target, environment variable for its data folder or None)
APPS = {
    "scan": ("Einstar.app/Contents/MacOS/Einstar", "einstar-app", "EINSTAR_SCAN_DIR"),
    "calibration": ("EinstarCalibration.app/Contents/MacOS/EinstarCalibration", "einstar-calibrate", "EINSTAR_CALIBRATION_DIR"),
    "model": ("EinstarModel.app/Contents/MacOS/EinstarModel", "einstar-model", None),
}

# Apps the user can start and an agent attach to: where each one listens.
ATTACHABLE = {"model": "model-agent.sock"}


def settings_dir() -> pathlib.Path:
    """Where the apps keep their settings (EINSTAR_SETTINGS_DIR, else ~/Library/Application Support/Einstar)."""
    if env := os.environ.get("EINSTAR_SETTINGS_DIR"):
        return pathlib.Path(env)
    return pathlib.Path.home() / "Library" / "Application Support" / "Einstar"


def attach_socket(app: str) -> pathlib.Path:
    return settings_dir() / ATTACHABLE[app]


def app_binary(app: str) -> pathlib.Path:
    return build_dir() / "apps" / APPS[app][0]


def methods_json() -> pathlib.Path:
    """The agent methods (generated from the C++ table, kept equal to it by test_agent_methods)."""
    return pathlib.Path(__file__).resolve().parents[2] / "methods.json"
