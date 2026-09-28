#pragma once

// Scan session recording (`.estr`): every processed frame's depth and confidence, its live pose
// and tracking verdict, and its markers. The live view is only a preview; the process step
// (libs/recon) re-reads the session to optimise poses, re-fuse and mesh, so nothing seen while
// scanning is lost.
//
// File layout (little endian): "ESTR" u32 version, then records [u32 tag][u32 reserved][u64 size][payload].
//   HEAD  session header (depth intrinsics, rectified stereo geometry)
//   FRAM  one frame (metadata + zstd-compressed u16 depth in 1/50 mm + u8 confidence)
//   GMRK  a global-marker map in use from that point on
// A reader tolerates a truncated last record (e.g. after a crash).

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <span>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "einstar/core/error.hpp"
#include "einstar/core/image.hpp"
#include "einstar/core/se3.hpp"
#include "einstar/markers/marker_map.hpp"
#include "einstar/track/frame.hpp"

namespace einstar::session {

struct SessionHeader {
    track::Intrinsics depth_intrinsics;  // of the stored depth images (rectified left camera)
    // Full-resolution rectified stereo geometry (for marker bundle adjustment).
    double rect_f = 0, rect_cx = 0, rect_cy = 0, baseline_mm = 0;
    std::string description;
};

enum FrameFlags : std::uint32_t {
    frame_accepted = 1u << 0,
    frame_degenerate = 1u << 1,
    frame_relocalized = 1u << 2,
    frame_marker_pose = 1u << 3,
    frame_integrated = 1u << 4,
    frame_global_marker_capture = 1u << 5,  // markers-only phase (no surface fused)
};

struct FrameMarker {
    Vec3 position;  // camera frame, mm
    Vec3 normal;
    double diameter = 0;
    int map_id = -1;  // id in the live marker map (-1 = not identified)
    Vec2 left_rect = Vec2::Constant(-1), right_rect = Vec2::Constant(-1);
};

// A frame's depth and confidence already in the file's uncompressed layout (u16 depth in 1/50 mm,
// delta-coded along rows, then u8 confidence), e.g. produced on the GPU. The writer calls `ready()`
// (which may block until the GPU finished) before reading `bytes`.
struct PackedImages {
    int width = 0, height = 0;
    std::function<void()> ready;
    std::span<const std::uint8_t> bytes;
    std::shared_ptr<void> owner;  // keeps `bytes` alive
};

struct FrameRecord {
    std::uint64_t index = 0;
    double timestamp_s = 0;
    std::uint32_t flags = 0;
    SE3 T_world_camera = SE3::Identity();  // live pose (meaningful when accepted)
    std::vector<FrameMarker> markers;
    ImageF32 depth;       // mm, 0 = none
    ImageF32 confidence;  // 0..1 (empty = unknown)
    std::shared_ptr<PackedImages> packed;  // writing only: used instead of depth/confidence when set

    [[nodiscard]] bool accepted() const { return (flags & frame_accepted) != 0; }
    // Points, normals and weights for fusion / registration.
    [[nodiscard]] track::DepthFrame depth_frame(const track::Intrinsics& k) const;
};

// Depth / confidence of a frame from its CPU images or GPU buffers.
void capture_depth(const track::DepthFrame& frame, ImageF32& depth, ImageF32& confidence);

// Appends records from any thread; compression and I/O happen on a writer thread.
class SessionWriter {
public:
    static Result<std::unique_ptr<SessionWriter>> create(const std::string& path, const SessionHeader& header);
    ~SessionWriter();

    void write(FrameRecord frame);
    void write_global_markers(const std::vector<markers::MapMarker>& map);
    void flush();  // waits until everything submitted so far is on disk (the file stays open)
    void close();  // flushes; idempotent

    [[nodiscard]] const std::string& path() const { return path_; }
    [[nodiscard]] std::uint64_t frames_written() const { return frames_written_; }
    [[nodiscard]] std::uint64_t bytes_written() const { return bytes_written_; }
    [[nodiscard]] std::size_t backlog() const;

private:
    SessionWriter() = default;
    void run();
    void write_record(std::uint32_t tag, const std::vector<std::uint8_t>& payload);

    std::string path_;
    std::ofstream out_;
    mutable std::mutex mutex_;
    std::condition_variable cv_, drained_cv_;
    std::size_t in_flight_ = 0;  // records taken by the writer thread but not yet written
    std::deque<std::vector<std::uint8_t>> queue_;  // serialised records (tag first)
    std::deque<FrameRecord> frames_;
    bool closing_ = false;
    std::atomic<std::uint64_t> frames_written_{0}, bytes_written_{0};
    std::thread thread_;
};

class SessionReader {
public:
    static Result<std::unique_ptr<SessionReader>> open(const std::string& path);

    [[nodiscard]] const SessionHeader& header() const { return header_; }
    [[nodiscard]] std::size_t frame_count() const { return frames_.size(); }
    // Metadata only (no depth): cheap, all frames.
    [[nodiscard]] const FrameRecord& meta(std::size_t i) const { return frames_[i]; }
    // Full frame including depth (decompressed on demand; thread-safe).
    [[nodiscard]] Result<FrameRecord> read(std::size_t i) const;
    // The last global-marker map recorded (empty if none).
    [[nodiscard]] const std::vector<markers::MapMarker>& global_markers() const { return global_markers_; }

private:
    SessionHeader header_;
    std::string path_;
    struct ImageBlock {
        std::uint64_t offset = 0, size = 0;  // compressed images in the file
        int width = 0, height = 0;
        bool has_confidence = false;
    };
    std::vector<FrameRecord> frames_;  // metadata (no images)
    std::vector<ImageBlock> blocks_;
    std::vector<markers::MapMarker> global_markers_;
    mutable std::mutex file_mutex_;
    mutable std::ifstream in_;
};

}  // namespace einstar::session
