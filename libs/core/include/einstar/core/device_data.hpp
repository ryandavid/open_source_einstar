#pragma once

// Device-resident (GPU) copies of per-frame data. On Apple Silicon these live in shared memory, so
// the CPU can read them through the pointers below without copying. Layouts: xyzw floats per pixel
// (w = 1 for valid points / hits, 0 otherwise), row-major, no padding.

namespace einstar {

class DeviceFrameData {
public:
    virtual ~DeviceFrameData() = default;
    [[nodiscard]] virtual int width() const = 0;
    [[nodiscard]] virtual int height() const = 0;
    [[nodiscard]] virtual const float* points_xyzw() const = 0;   // camera frame, z = 0 invalid
    [[nodiscard]] virtual const float* normals_xyzw() const = 0;  // zero where unknown
    [[nodiscard]] virtual const float* weights() const = 0;       // 0..1
};

class DeviceRaycastData {
public:
    virtual ~DeviceRaycastData() = default;
    [[nodiscard]] virtual int width() const = 0;
    [[nodiscard]] virtual int height() const = 0;
    [[nodiscard]] virtual const float* points_xyzw() const = 0;   // world frame, w = 1 where valid
    [[nodiscard]] virtual const float* normals_xyzw() const = 0;
};

}  // namespace einstar
