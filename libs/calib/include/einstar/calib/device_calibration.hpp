#pragma once

// Decoding of the Einstar calibration stored in device flash (docs/calibration.md).
//
// The 6568-byte blob holds tagged sections at fixed offsets. The Einstar uses the "quick"
// section (FQFQ), which embeds three CCF files: Left/Right (33 doubles + 64-int offset table,
// fields obfuscated with additive integer offsets) and Tex (33 plain doubles).
// Extrinsics are world->camera (X_cam = R X_world + T, mm), the world being the calibration board.

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "einstar/core/camera.hpp"
#include "einstar/core/error.hpp"

namespace einstar::calib {

inline constexpr std::size_t kFlashBlobSize = 6568;
inline constexpr std::size_t kCcfDoubles = 33;
inline constexpr std::size_t kCcfTailInts = 64;
inline constexpr std::size_t kObfuscatedCcfSize = kCcfDoubles * 8 + kCcfTailInts * 4;  // 520
inline constexpr std::size_t kPlainCcfSize = kCcfDoubles * 8;                          // 264

// One camera as stored: intrinsics + pose relative to the calibration board.
struct CameraCalibration {
    CameraModel model;
    Mat3 R_cam_world = Mat3::Identity();
    Vec3 t_cam_world = Vec3::Zero();  // mm
    double rms_error = 0;

    [[nodiscard]] SE3 T_cam_world() const {
        SE3 t = SE3::Identity();
        t.linear() = R_cam_world;
        t.translation() = t_cam_world;
        return t;
    }
};

struct ColorRecord {
    std::array<float, 9> ccm{};     // 3x3 colour-correction matrix
    std::array<float, 3> offset{};
    std::array<float, 3> gain{};
    std::array<float, 3> dark{};
};

struct DeviceCalibration {
    CameraCalibration left, right, texture;
    std::string calibration_time;             // "yyyy-MM-dd hh:mm" if present
    std::optional<std::array<ColorRecord, 2>> color;       // [0] with LED, [1] without (unverified)
    std::optional<std::array<Mat3, 2>> white_balance;       // same order

    [[nodiscard]] RigCalibration rig() const;  // left-camera-referenced rig
};

// Raw 33-double CCF parsing.
[[nodiscard]] Result<std::array<double, kCcfDoubles>> read_ccf_doubles(std::span<const std::uint8_t> bytes);
// Removes the Left/Right obfuscation (both files are needed: each carries half of the key).
[[nodiscard]] Result<std::pair<std::array<double, kCcfDoubles>, std::array<double, kCcfDoubles>>>
decode_ccf_pair(std::span<const std::uint8_t> left_file, std::span<const std::uint8_t> right_file);
[[nodiscard]] CameraCalibration camera_from_ccf(const std::array<double, kCcfDoubles>& d, int width, int height);

[[nodiscard]] Result<DeviceCalibration> decode_ccf_files(std::span<const std::uint8_t> left_file,
                                                         std::span<const std::uint8_t> right_file,
                                                         std::span<const std::uint8_t> tex_file);
[[nodiscard]] Result<DeviceCalibration> decode_flash_blob(std::span<const std::uint8_t> blob);

// The three CCF files stored in the flash blob's quick-calibration section: byte-for-byte what EXStar
// writes to its cache directory (LeftCCF.txt, RightCCF.txt, TexCCF.txt).
struct CcfFiles {
    std::vector<std::uint8_t> left, right, tex;
    std::string calibration_time;  // "yyyy-MM-dd hh:mm" if present
};
[[nodiscard]] Result<CcfFiles> extract_ccf_files(std::span<const std::uint8_t> blob);
// Writes them (plus calibration_time.txt) to `dir`, creating it; load_ccf_directory() reads them back.
[[nodiscard]] Result<void> write_ccf_directory(const CcfFiles& files, const std::string& dir);

// Builds a flash blob holding the three CCF files in the quick-calibration section (for the device
// emulator; never written to a real device).
[[nodiscard]] std::vector<std::uint8_t> encode_quick_flash_blob(std::span<const std::uint8_t> left_file,
                                                                std::span<const std::uint8_t> right_file,
                                                                std::span<const std::uint8_t> tex_file,
                                                                const std::string& calibration_time);
[[nodiscard]] Result<std::vector<std::uint8_t>> encode_quick_flash_blob_from_directory(const std::string& dir);

// Reads LeftCCF.txt / RightCCF.txt / TexCCF.txt from a directory (e.g. EXStar's cache).
[[nodiscard]] Result<DeviceCalibration> load_ccf_directory(const std::string& dir);

}  // namespace einstar::calib
