#pragma once

// Scan session recording (`.estr`): every processed frame's depth and confidence, its live pose
// and tracking verdict, and its markers. The live view is only a preview; the process step
// (libs/recon) re-reads the session to optimise poses, re-fuse and mesh, so nothing seen while
// scanning is lost.
//
// File layout (little endian): "ESTR" u32 version, then records [u32 tag][u32 reserved][u64 size][payload].
//   HEAD  session header (depth intrinsics, rectified stereo geometry)
//   DEVC  the scanner: identity, calibration blob as read from flash, decoded rig, rectification
//   FRAM  one frame (metadata + zstd-compressed u16 depth in 1/50 mm + u8 confidence)
//   FXTR  the preceding frame's capture settings and tracking diagnostics
//   DROP  a frame that reached the host but has no FRAM (live queue overflow, no depth)
//   RAWI  optional raw IR images of one trigger (row delta-coded 8-bit, zstd)
//   GMRK  a global-marker map in use from that point on
//   ERAS  an erase (a paused scan's lasso delete): removes, from every frame before it in the file, the
//         depth whose point (at the frame's live pose) the selection contains; it follows every frame
//         submitted before it (the writer flushes first)
//   UNDO  cancels an erase
//   RESM  the scan was loaded and scanning resumed: frames after it continue the same model (in the same world
//         frame, relocalised against it), their timestamps moved past the earlier ones by a gap, so processing
//         treats the join like a tracking gap
// Readers skip record types they do not know, so new kinds can be added without breaking old
// files or old readers. A reader tolerates a truncated last record (e.g. after a crash).

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "einstar/core/camera.hpp"
#include "einstar/core/error.hpp"
#include "einstar/core/image.hpp"
#include "einstar/core/lasso.hpp"
#include "einstar/core/se3.hpp"
#include "einstar/depth/point_image.hpp"
#include "einstar/markers/marker_map.hpp"
#include "einstar/track/frame.hpp"

namespace einstar::session {

// The scanner a session was recorded with: enough to re-rectify raw images and tie results to a unit.
struct DeviceRecord {
    std::string vendor, product, serial, firmware;
    std::vector<std::uint8_t> calibration_blob;  // as read from the scanner's flash (empty for replays)
    RigCalibration rig;                          // decoded from the blob
    Mat3 R_rect_left = Mat3::Identity();         // rectifying rotations used live (original -> rectified)
    Mat3 R_rect_right = Mat3::Identity();
    CameraModel rectified;                       // full-resolution rectified left camera (no distortion)
    CameraModel rectified_right;                 // the right one (its cx differs; older files: = rectified)
};

// Settings the frame was captured with (as last sent to the scanner).
struct CaptureSettings {
    std::array<std::uint32_t, 2> exposure{};  // per IR sensor (device units, assumed us)
    std::array<std::uint16_t, 2> gain{};
    int laser_percent = 0;
    int strobe = 0;
    std::uint32_t trigger_period_us = 0;
    float temperature_c = std::numeric_limits<float>::quiet_NaN();  // last reading
};

// How the live tracker saw the frame (the process step recomputes what it needs; this is for tuning
// and diagnosis without re-running tracking).
struct TrackingDiagnostics {
    std::uint8_t state = 0;  // track::TrackState
    float icp_rms_mm = 0, inlier_ratio = 0, coverage = 0, eigen_ratio = 0, marker_rms_mm = 0;
    std::int32_t correspondences = 0, degenerate_directions = 0, markers_seen = 0;
    float stereo_ms = 0, track_ms = 0;
    std::string reason;  // why the frame was rejected, if it was
};

struct FrameExtras {
    std::int32_t left_sensor = -1;  // which stream sensor is the left IR camera
    CaptureSettings capture;
    TrackingDiagnostics tracking;
};

// A frame that reached the host without a FRAM record.
struct DroppedFrame {
    std::uint64_t index = 0;
    double timestamp_s = 0;
    std::string reason;
};

// Raw IR images of one trigger, as streamed (sensor index as delivered; see FrameExtras::left_sensor).
struct RawFrame {
    std::uint64_t index = 0;
    double timestamp_s = 0;
    std::vector<std::pair<int, ImageU8>> images;
};

struct SessionHeader {
    track::Intrinsics depth_intrinsics;  // of the stored depth images (rectified left camera)
    // Full-resolution rectified stereo geometry (for marker bundle adjustment).
    double rect_f = 0, rect_cx = 0, rect_cy = 0, baseline_mm = 0;
    double rect_cx_offset = 0;  // right camera's cx - left cx (0 in older files, which had one cx)
    std::string description;
    [[nodiscard]] depth::RectifiedGeometry rectified_geometry() const { return {rect_f, rect_cx, rect_cy, baseline_mm, rect_cx_offset}; }
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
    std::optional<FrameExtras> extras;     // FXTR (absent in older files)

    [[nodiscard]] bool accepted() const { return (flags & frame_accepted) != 0; }
    // Points, normals and weights for fusion / registration.
    [[nodiscard]] track::DepthFrame depth_frame(const track::Intrinsics& k) const;
};

// An erase in effect (not undone): frames [0, frames_before) lose the depth the selection contains.
struct SessionErase {
    std::uint32_t id = 0;
    std::size_t frames_before = 0;
    LassoSelection selection;
};

// Where a recording was resumed.
struct ResumeMark {
    std::size_t frames_before = 0;  // frames in the file before it
    std::string note;
};

// Depth / confidence of a frame from its CPU images or GPU buffers.
void capture_depth(const track::DepthFrame& frame, ImageF32& depth, ImageF32& confidence);

// Appends records from any thread; compression and I/O happen on a writer thread.
class SessionWriter {
public:
    static Result<std::unique_ptr<SessionWriter>> create(const std::string& path, const SessionHeader& header);
    // Continues an existing recording: records go after its last complete one (a record cut short by a crash is
    // trimmed first, or it would hide everything written after it).
    static Result<std::unique_ptr<SessionWriter>> append(const std::string& path);
    ~SessionWriter();

    void write(FrameRecord frame);
    void write_global_markers(const std::vector<markers::MapMarker>& map);
    void write_device(const DeviceRecord& device);
    void write_dropped(const DroppedFrame& frame);
    // Edits: an erase covers every frame submitted before it (waits until they are on disk first).
    void write_erase(std::uint32_t id, const LassoSelection& selection);
    void write_undo(std::uint32_t id);
    // Marks where scanning resumed on a loaded recording.
    void write_resume(std::uint64_t frames_before, const std::string& note);
    // Raw images are compressed on the writer thread. Returns false (and drops the frame) when more
    // than `max_raw_backlog` are waiting, so a slow disk cannot grow memory without bound.
    bool write_raw(RawFrame frame, std::size_t max_raw_backlog = 48);
    void flush();  // waits until everything submitted so far is on disk (the file stays open)
    void close();  // flushes; idempotent

    [[nodiscard]] const std::string& path() const { return path_; }
    [[nodiscard]] std::uint64_t frames_written() const { return frames_written_; }
    [[nodiscard]] std::uint64_t raw_frames_written() const { return raw_written_; }
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
    std::deque<RawFrame> raws_;
    bool closing_ = false;
    std::atomic<std::uint64_t> frames_written_{0}, bytes_written_{0}, raw_written_{0};
    std::thread thread_;
};

class SessionReader {
public:
    static Result<std::unique_ptr<SessionReader>> open(const std::string& path);

    [[nodiscard]] const SessionHeader& header() const { return header_; }
    [[nodiscard]] std::size_t frame_count() const { return frames_.size(); }
    // Metadata only (no depth): cheap, all frames.
    [[nodiscard]] const FrameRecord& meta(std::size_t i) const { return frames_[i]; }
    // Full frame including depth (decompressed on demand; thread-safe). The session's erases are applied
    // to a tracked frame at its live pose (what the lasso was drawn against).
    [[nodiscard]] Result<FrameRecord> read(std::size_t i) const;
    // Erases in effect (undone ones dropped), and applying them to frame i at a given pose (a frame
    // live tracking lost, at the pose processing recovered for it).
    [[nodiscard]] const std::vector<SessionErase>& erasures() const { return erasures_; }
    void apply_erasures(std::size_t i, FrameRecord& frame, const SE3& T_world_camera) const;
    // The last global-marker map recorded (empty if none).
    [[nodiscard]] const std::vector<markers::MapMarker>& global_markers() const { return global_markers_; }
    [[nodiscard]] const std::optional<DeviceRecord>& device() const { return device_; }
    [[nodiscard]] const std::vector<DroppedFrame>& dropped() const { return dropped_; }
    [[nodiscard]] const std::vector<ResumeMark>& resumes() const { return resumes_; }
    // Bytes up to the end of the last complete record (the rest is a record cut short).
    [[nodiscard]] std::uint64_t complete_bytes() const { return complete_bytes_; }
    // Raw IR frames (optional): index / timestamp without decoding, and the full images on demand.
    [[nodiscard]] std::size_t raw_count() const { return raws_.size(); }
    [[nodiscard]] std::uint64_t raw_index(std::size_t i) const { return raws_[i].index; }
    [[nodiscard]] double raw_timestamp(std::size_t i) const { return raws_[i].timestamp_s; }
    [[nodiscard]] std::uint64_t raw_bytes() const;  // on disk, all raw frames
    [[nodiscard]] Result<RawFrame> read_raw(std::size_t i) const;

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
    std::optional<DeviceRecord> device_;
    std::vector<DroppedFrame> dropped_;
    std::vector<SessionErase> erasures_;
    std::vector<ResumeMark> resumes_;
    std::uint64_t complete_bytes_ = 8;
    struct RawBlock {
        std::uint64_t index = 0;
        double timestamp_s = 0;
        std::uint64_t offset = 0, size = 0;  // the RAWI payload in the file
    };
    std::vector<RawBlock> raws_;
    mutable std::mutex file_mutex_;
    mutable std::ifstream in_;
};

}  // namespace einstar::session
