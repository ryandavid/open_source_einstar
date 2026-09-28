#include "einstar/session/session.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <format>
#include <optional>

#include <zstd.h>

#include "einstar/core/log.hpp"

namespace einstar::session {
namespace {

constexpr std::uint32_t kMagic = 0x52545345;  // "ESTR"
constexpr std::uint32_t kVersion = 1;
constexpr std::uint32_t tag(const char (&s)[5]) {
    return static_cast<std::uint32_t>(s[0]) | static_cast<std::uint32_t>(s[1]) << 8 | static_cast<std::uint32_t>(s[2]) << 16 |
           static_cast<std::uint32_t>(s[3]) << 24;
}
constexpr std::uint32_t kHead = tag("HEAD");
constexpr std::uint32_t kFrame = tag("FRAM");
constexpr std::uint32_t kGlobal = tag("GMRK");
constexpr std::uint32_t kDevice = tag("DEVC");
constexpr std::uint32_t kExtras = tag("FXTR");
constexpr std::uint32_t kDropped = tag("DROP");
constexpr std::uint32_t kRaw = tag("RAWI");
constexpr double kDepthScale = 50.0;  // u16 units per mm (0.02 mm, up to 1310 mm)
constexpr int kZstdLevel = 1;  // ~2x faster than 3 for 2% larger files (recording runs every frame)

struct Writer {
    std::vector<std::uint8_t> b;
    template <typename T>
    void put(const T& v) {
        const auto* p = reinterpret_cast<const std::uint8_t*>(&v);
        b.insert(b.end(), p, p + sizeof(T));
    }
    void put_vec(const Vec3& v) { put(v.x()), put(v.y()), put(v.z()); }
    void put_vec(const Vec2& v) { put(v.x()), put(v.y()); }
    void put_string(const std::string& s) {
        put(static_cast<std::uint32_t>(s.size()));
        b.insert(b.end(), s.begin(), s.end());
    }
    void put_bytes(std::span<const std::uint8_t> bytes) {
        put(static_cast<std::uint64_t>(bytes.size()));
        b.insert(b.end(), bytes.begin(), bytes.end());
    }
    void put_mat3(const Mat3& m) {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) put(m(r, c));
    }
    void put_camera(const CameraModel& c) {
        put(static_cast<std::int32_t>(c.width)), put(static_cast<std::int32_t>(c.height));
        put(c.fx), put(c.fy), put(c.cx), put(c.cy), put(c.skew);
        for (const double d : c.dist) put(d);
    }
};

struct Reader {
    const std::uint8_t* p;
    const std::uint8_t* end;
    bool ok = true;
    template <typename T>
    T get() {
        T v{};
        if (end - p < static_cast<std::ptrdiff_t>(sizeof(T))) {
            ok = false;
            return v;
        }
        std::memcpy(&v, p, sizeof(T));
        p += sizeof(T);
        return v;
    }
    Vec3 get_vec3() {
        const double x = get<double>(), y = get<double>(), z = get<double>();
        return {x, y, z};
    }
    Vec2 get_vec2() {
        const double x = get<double>(), y = get<double>();
        return {x, y};
    }
    std::string get_string() {
        const auto n = get<std::uint32_t>();
        if (end - p < static_cast<std::ptrdiff_t>(n)) {
            ok = false;
            return {};
        }
        std::string s(reinterpret_cast<const char*>(p), n);
        p += n;
        return s;
    }
    std::vector<std::uint8_t> get_bytes() {
        const auto n = get<std::uint64_t>();
        if (static_cast<std::uint64_t>(end - p) < n) {
            ok = false;
            return {};
        }
        std::vector<std::uint8_t> v(p, p + n);
        p += n;
        return v;
    }
    Mat3 get_mat3() {
        Mat3 m;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) m(r, c) = get<double>();
        return m;
    }
    CameraModel get_camera() {
        CameraModel c;
        c.width = get<std::int32_t>(), c.height = get<std::int32_t>();
        c.fx = get<double>(), c.fy = get<double>(), c.cx = get<double>(), c.cy = get<double>(), c.skew = get<double>();
        for (double& d : c.dist) d = get<double>();
        return c;
    }
};

void put_pose(Writer& w, const SE3& T) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) w.put(T.matrix()(r, c));
}

SE3 get_pose(Reader& r) {
    SE3 T = SE3::Identity();
    for (int i = 0; i < 3; ++i)
        for (int c = 0; c < 4; ++c) T.matrix()(i, c) = r.get<double>();
    return T;
}

// Depth as horizontally delta-coded u16 (smooth surfaces compress far better), then confidence.
std::vector<std::uint8_t> compress(std::span<const std::uint8_t> raw) {
    std::vector<std::uint8_t> out(ZSTD_compressBound(raw.size()));
    const std::size_t sz = ZSTD_compress(out.data(), out.size(), raw.data(), raw.size(), kZstdLevel);
    if (ZSTD_isError(sz)) return {};
    out.resize(sz);
    return out;
}

std::vector<std::uint8_t> encode_images(const ImageF32& depth, const ImageF32& conf) {
    const int w = depth.width(), h = depth.height();
    const auto n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    std::vector<std::uint8_t> raw(n * 2 + (conf.empty() ? 0 : n));
    auto* d16 = reinterpret_cast<std::uint16_t*>(raw.data());
    for (int y = 0; y < h; ++y) {
        std::uint16_t prev = 0;
        for (int x = 0; x < w; ++x) {
            const float z = depth(x, y);
            const auto q = static_cast<std::uint16_t>(z > 0 ? std::clamp(std::lround(z * kDepthScale), 1L, 65535L) : 0);
            d16[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = static_cast<std::uint16_t>(q - prev);
            prev = q;
        }
    }
    if (!conf.empty())
        for (std::size_t i = 0; i < n; ++i)
            raw[n * 2 + i] = static_cast<std::uint8_t>(std::clamp(std::lround(conf.data()[i] * 255.0f), 0L, 255L));
    return compress(raw);
}

bool decode_images(const std::uint8_t* data, std::size_t size, int w, int h, bool has_conf, ImageF32& depth, ImageF32& conf) {
    const auto n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    std::vector<std::uint8_t> raw(n * 2 + (has_conf ? n : 0));
    const std::size_t got = ZSTD_decompress(raw.data(), raw.size(), data, size);
    if (ZSTD_isError(got) || got != raw.size()) return false;
    depth = ImageF32(w, h, 0.0f);
    const auto* d16 = reinterpret_cast<const std::uint16_t*>(raw.data());
    for (int y = 0; y < h; ++y) {
        std::uint16_t acc = 0;
        for (int x = 0; x < w; ++x) {
            acc = static_cast<std::uint16_t>(acc + d16[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)]);
            depth(x, y) = acc ? static_cast<float>(acc / kDepthScale) : 0.0f;
        }
    }
    if (has_conf) {
        conf = ImageF32(w, h, 0.0f);
        for (std::size_t i = 0; i < n; ++i) conf.data()[i] = raw[n * 2 + i] / 255.0f;
    } else {
        conf = {};
    }
    return true;
}

std::vector<std::uint8_t> encode_device(const DeviceRecord& d) {
    Writer p;
    p.put_string(d.vendor), p.put_string(d.product), p.put_string(d.serial), p.put_string(d.firmware);
    p.put_bytes(d.calibration_blob);
    p.put_camera(d.rig.left), p.put_camera(d.rig.right), p.put_camera(d.rig.texture);
    put_pose(p, d.rig.T_right_left), put_pose(p, d.rig.T_texture_left);
    p.put_mat3(d.R_rect_left), p.put_mat3(d.R_rect_right);
    p.put_camera(d.rectified);
    return std::move(p.b);
}

DeviceRecord decode_device(Reader& rd) {
    DeviceRecord d;
    d.vendor = rd.get_string(), d.product = rd.get_string(), d.serial = rd.get_string(), d.firmware = rd.get_string();
    d.calibration_blob = rd.get_bytes();
    d.rig.left = rd.get_camera(), d.rig.right = rd.get_camera(), d.rig.texture = rd.get_camera();
    d.rig.T_right_left = get_pose(rd), d.rig.T_texture_left = get_pose(rd);
    d.R_rect_left = rd.get_mat3(), d.R_rect_right = rd.get_mat3();
    d.rectified = rd.get_camera();
    return d;
}

std::vector<std::uint8_t> encode_extras(std::uint64_t index, const FrameExtras& e) {
    Writer p;
    p.put(index);
    p.put(e.left_sensor);
    const auto& c = e.capture;
    p.put(c.exposure[0]), p.put(c.exposure[1]), p.put(c.gain[0]), p.put(c.gain[1]);
    p.put(static_cast<std::int32_t>(c.laser_percent)), p.put(static_cast<std::int32_t>(c.strobe));
    p.put(c.trigger_period_us), p.put(c.temperature_c);
    const auto& t = e.tracking;
    p.put(t.state);
    p.put(t.icp_rms_mm), p.put(t.inlier_ratio), p.put(t.coverage), p.put(t.eigen_ratio), p.put(t.marker_rms_mm);
    p.put(t.correspondences), p.put(t.degenerate_directions), p.put(t.markers_seen);
    p.put(t.stereo_ms), p.put(t.track_ms);
    p.put_string(t.reason);
    return std::move(p.b);
}

FrameExtras decode_extras(Reader& rd) {
    FrameExtras e;
    e.left_sensor = rd.get<std::int32_t>();
    auto& c = e.capture;
    c.exposure[0] = rd.get<std::uint32_t>(), c.exposure[1] = rd.get<std::uint32_t>();
    c.gain[0] = rd.get<std::uint16_t>(), c.gain[1] = rd.get<std::uint16_t>();
    c.laser_percent = rd.get<std::int32_t>(), c.strobe = rd.get<std::int32_t>();
    c.trigger_period_us = rd.get<std::uint32_t>(), c.temperature_c = rd.get<float>();
    auto& t = e.tracking;
    t.state = rd.get<std::uint8_t>();
    t.icp_rms_mm = rd.get<float>(), t.inlier_ratio = rd.get<float>(), t.coverage = rd.get<float>();
    t.eigen_ratio = rd.get<float>(), t.marker_rms_mm = rd.get<float>();
    t.correspondences = rd.get<std::int32_t>(), t.degenerate_directions = rd.get<std::int32_t>(), t.markers_seen = rd.get<std::int32_t>();
    t.stereo_ms = rd.get<float>(), t.track_ms = rd.get<float>();
    t.reason = rd.get_string();
    return e;
}

// Raw IR: each row delta-coded from its left neighbour (1.9x with zstd on real IR images vs 1.5x
// without), then zstd.
std::vector<std::uint8_t> encode_raw(const RawFrame& f) {
    Writer p;
    p.put(f.index);
    p.put(f.timestamp_s);
    p.put(static_cast<std::uint32_t>(f.images.size()));
    std::vector<std::uint8_t> delta;
    for (const auto& [sensor, img] : f.images) {
        const int w = img.width(), h = img.height();
        delta.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
        for (int y = 0; y < h; ++y) {
            const std::uint8_t* row = img.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(w);
            std::uint8_t* out = delta.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(w);
            std::uint8_t prev = 0;
            for (int x = 0; x < w; ++x) {
                out[x] = static_cast<std::uint8_t>(row[x] - prev);
                prev = row[x];
            }
        }
        p.put(static_cast<std::int32_t>(sensor)), p.put(static_cast<std::int32_t>(w)), p.put(static_cast<std::int32_t>(h));
        p.put_bytes(compress(delta));
    }
    return std::move(p.b);
}

}  // namespace

track::DepthFrame FrameRecord::depth_frame(const track::Intrinsics& k) const {
    auto f = track::make_depth_frame(depth, k);
    if (!confidence.empty())
        for (std::size_t i = 0; i < f.weights.pixels().size(); ++i)
            if (f.weights.data()[i] > 0) f.weights.data()[i] = std::max(0.02f, confidence.data()[i]);
    f.index = index;
    f.timestamp_s = timestamp_s;
    for (const auto& m : markers) {
        track::MarkerPoint mp;
        mp.position = m.position;
        mp.normal = m.normal;
        mp.diameter = m.diameter;
        mp.id = m.map_id;
        mp.left_rect = m.left_rect;
        mp.right_rect = m.right_rect;
        f.markers.push_back(mp);
    }
    return f;
}

void capture_depth(const track::DepthFrame& frame, ImageF32& depth, ImageF32& confidence) {
    const int w = frame.width(), h = frame.height();
    depth = ImageF32(w, h, 0.0f);
    confidence = ImageF32(w, h, 0.0f);
    if (frame.device) {
        const float* p = frame.device->points_xyzw();
        const float* wt = frame.device->weights();
        for (std::size_t i = 0; i < static_cast<std::size_t>(w) * static_cast<std::size_t>(h); ++i) {
            depth.data()[i] = std::max(0.0f, p[4 * i + 2]);
            confidence.data()[i] = wt ? wt[i] : 1.0f;
        }
        return;
    }
    for (std::size_t i = 0; i < frame.points.pixels().size(); ++i) {
        depth.data()[i] = std::max(0.0f, frame.points.data()[i].z());
        confidence.data()[i] = frame.weights.empty() ? 1.0f : frame.weights.data()[i];
    }
}

// ---------------------------------------------------------------------------------------------

Result<std::unique_ptr<SessionWriter>> SessionWriter::create(const std::string& path, const SessionHeader& header) {
    std::error_code ec;
    if (const auto dir = std::filesystem::path(path).parent_path(); !dir.empty()) std::filesystem::create_directories(dir, ec);
    auto w = std::unique_ptr<SessionWriter>(new SessionWriter());
    w->path_ = path;
    w->out_.open(path, std::ios::binary | std::ios::trunc);
    if (!w->out_) return make_error(Errc::io, "cannot create " + path);
    Writer h;
    h.put(kMagic);
    h.put(kVersion);
    w->out_.write(reinterpret_cast<const char*>(h.b.data()), static_cast<std::streamsize>(h.b.size()));
    Writer p;
    const auto& k = header.depth_intrinsics;
    p.put(static_cast<std::int32_t>(k.width));
    p.put(static_cast<std::int32_t>(k.height));
    p.put(k.fx), p.put(k.fy), p.put(k.cx), p.put(k.cy);
    p.put(header.rect_f), p.put(header.rect_cx), p.put(header.rect_cy), p.put(header.baseline_mm);
    p.put_string(header.description);
    w->write_record(kHead, p.b);
    w->thread_ = std::thread([raw = w.get()] { raw->run(); });
    return w;
}

SessionWriter::~SessionWriter() { close(); }

void SessionWriter::write(FrameRecord frame) {
    {
        std::lock_guard lock(mutex_);
        if (closing_) return;
        frames_.push_back(std::move(frame));
    }
    cv_.notify_one();
}

void SessionWriter::write_global_markers(const std::vector<markers::MapMarker>& map) {
    Writer p;
    p.put(kGlobal);
    p.put(static_cast<std::uint32_t>(map.size()));
    for (const auto& m : map) {
        p.put(static_cast<std::int32_t>(m.id));
        p.put_vec(m.position);
        p.put(m.diameter);
    }
    {
        std::lock_guard lock(mutex_);
        if (closing_) return;
        queue_.push_back(std::move(p.b));
    }
    cv_.notify_one();
}

void SessionWriter::write_device(const DeviceRecord& device) {
    Writer p;
    p.put(kDevice);
    const auto payload = encode_device(device);
    p.b.insert(p.b.end(), payload.begin(), payload.end());
    {
        std::lock_guard lock(mutex_);
        if (closing_) return;
        queue_.push_back(std::move(p.b));
    }
    cv_.notify_one();
}

void SessionWriter::write_dropped(const DroppedFrame& frame) {
    Writer p;
    p.put(kDropped);
    p.put(frame.index);
    p.put(frame.timestamp_s);
    p.put_string(frame.reason);
    {
        std::lock_guard lock(mutex_);
        if (closing_) return;
        queue_.push_back(std::move(p.b));
    }
    cv_.notify_one();
}

bool SessionWriter::write_raw(RawFrame frame, std::size_t max_raw_backlog) {
    {
        std::lock_guard lock(mutex_);
        if (closing_ || raws_.size() >= max_raw_backlog) return false;
        raws_.push_back(std::move(frame));
    }
    cv_.notify_one();
    return true;
}

void SessionWriter::flush() {
    std::unique_lock lock(mutex_);
    // The writer thread flushes the stream when it drains (only it touches the stream).
    drained_cv_.wait(lock, [&] { return (frames_.empty() && queue_.empty() && raws_.empty() && in_flight_ == 0) || !thread_.joinable(); });
}

std::size_t SessionWriter::backlog() const {
    std::lock_guard lock(mutex_);
    return frames_.size() + queue_.size() + raws_.size();
}

void SessionWriter::close() {
    {
        std::lock_guard lock(mutex_);
        if (closing_ && !thread_.joinable()) return;
        closing_ = true;
    }
    cv_.notify_one();
    if (thread_.joinable()) thread_.join();
    out_.flush();
    out_.close();
}

void SessionWriter::write_record(std::uint32_t t, const std::vector<std::uint8_t>& payload) {
    Writer h;
    h.put(t);
    h.put(std::uint32_t{0});
    h.put(static_cast<std::uint64_t>(payload.size()));
    out_.write(reinterpret_cast<const char*>(h.b.data()), static_cast<std::streamsize>(h.b.size()));
    out_.write(reinterpret_cast<const char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
    bytes_written_ += h.b.size() + payload.size();
}

void SessionWriter::run() {
    for (;;) {
        std::optional<FrameRecord> frame;
        std::optional<RawFrame> raw;
        std::vector<std::uint8_t> other;
        {
            std::unique_lock lock(mutex_);
            cv_.wait(lock, [&] { return closing_ || !frames_.empty() || !queue_.empty() || !raws_.empty(); });
            // Keep records in submission order within each kind; small records (global markers,
            // device, drops) go first, then processed frames, then raw images.
            if (!queue_.empty()) {
                other = std::move(queue_.front());
                queue_.pop_front();
                ++in_flight_;
            } else if (!frames_.empty()) {
                frame = std::move(frames_.front());
                frames_.pop_front();
                ++in_flight_;
            } else if (!raws_.empty()) {
                raw = std::move(raws_.front());
                raws_.pop_front();
                ++in_flight_;
            } else if (closing_) {
                return;
            }
        }
        auto done = [&] {
            std::lock_guard lock(mutex_);
            --in_flight_;
            if (frames_.empty() && queue_.empty() && raws_.empty() && in_flight_ == 0) {
                out_.flush();
                drained_cv_.notify_all();
            }
        };
        if (!other.empty()) {
            std::uint32_t t;
            std::memcpy(&t, other.data(), 4);
            write_record(t, std::vector<std::uint8_t>(other.begin() + 4, other.end()));
            done();
            continue;
        }
        if (raw) {
            write_record(kRaw, encode_raw(*raw));
            ++raw_written_;
            done();
            continue;
        }
        if (!frame) continue;
        Writer p;
        p.put(frame->index);
        p.put(frame->timestamp_s);
        p.put(frame->flags);
        put_pose(p, frame->T_world_camera);
        p.put(static_cast<std::uint32_t>(frame->markers.size()));
        for (const auto& m : frame->markers) {
            p.put_vec(m.position);
            p.put_vec(m.normal);
            p.put(m.diameter);
            p.put(static_cast<std::int32_t>(m.map_id));
            p.put_vec(m.left_rect);
            p.put_vec(m.right_rect);
        }
        std::vector<std::uint8_t> blob;
        int w = frame->depth.width(), h = frame->depth.height();
        bool conf = !frame->confidence.empty();
        if (frame->packed) {
            frame->packed->ready();
            blob = compress(frame->packed->bytes);
            w = frame->packed->width;
            h = frame->packed->height;
            conf = true;
        } else {
            blob = encode_images(frame->depth, frame->confidence);
        }
        p.put(static_cast<std::int32_t>(w));
        p.put(static_cast<std::int32_t>(h));
        p.put(static_cast<std::uint32_t>(conf ? 1 : 0));
        p.put(static_cast<std::uint64_t>(blob.size()));
        p.b.insert(p.b.end(), blob.begin(), blob.end());
        write_record(kFrame, p.b);
        if (frame->extras) write_record(kExtras, encode_extras(frame->index, *frame->extras));
        ++frames_written_;
        done();
    }
}

// ---------------------------------------------------------------------------------------------

Result<std::unique_ptr<SessionReader>> SessionReader::open(const std::string& path) {
    auto r = std::unique_ptr<SessionReader>(new SessionReader());
    r->path_ = path;
    r->in_.open(path, std::ios::binary);
    if (!r->in_) return make_error(Errc::not_found, "cannot open " + path);
    std::uint32_t magic = 0, version = 0;
    r->in_.read(reinterpret_cast<char*>(&magic), 4);
    r->in_.read(reinterpret_cast<char*>(&version), 4);
    if (!r->in_ || magic != kMagic) return make_error(Errc::protocol, path + " is not an einstar session");
    if (version != kVersion) return make_error(Errc::unsupported, std::format("session version {} is not supported", version));
    bool have_header = false;
    std::error_code ec;
    const auto file_size = std::filesystem::file_size(path, ec);
    std::vector<std::uint8_t> buf;
    for (;;) {
        std::uint32_t t = 0, reserved = 0;
        std::uint64_t size = 0;
        const auto record_start = static_cast<std::uint64_t>(r->in_.tellg());
        r->in_.read(reinterpret_cast<char*>(&t), 4);
        r->in_.read(reinterpret_cast<char*>(&reserved), 4);
        r->in_.read(reinterpret_cast<char*>(&size), 8);
        if (!r->in_) break;
        const std::uint64_t payload_at = record_start + 16;
        if (!ec && payload_at + size > file_size) break;  // truncated last record
        if (t == kRaw) {
            // Index only; images are decoded on demand.
            buf.resize(16);
            r->in_.read(reinterpret_cast<char*>(buf.data()), 16);
            if (!r->in_) break;
            Reader rd{buf.data(), buf.data() + buf.size()};
            RawBlock b;
            b.index = rd.get<std::uint64_t>();
            b.timestamp_s = rd.get<double>();
            b.offset = payload_at;
            b.size = size;
            r->raws_.push_back(b);
        } else if (t == kFrame) {
            // Metadata only; the image block is read on demand.
            const std::size_t meta_max = std::min<std::uint64_t>(size, 1u << 20);
            buf.resize(meta_max);
            r->in_.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(meta_max));
            if (!r->in_) break;
            Reader rd{buf.data(), buf.data() + buf.size()};
            FrameRecord f;
            f.index = rd.get<std::uint64_t>();
            f.timestamp_s = rd.get<double>();
            f.flags = rd.get<std::uint32_t>();
            f.T_world_camera = get_pose(rd);
            const auto nm = rd.get<std::uint32_t>();
            for (std::uint32_t i = 0; i < nm && rd.ok; ++i) {
                FrameMarker m;
                m.position = rd.get_vec3();
                m.normal = rd.get_vec3();
                m.diameter = rd.get<double>();
                m.map_id = rd.get<std::int32_t>();
                m.left_rect = rd.get_vec2();
                m.right_rect = rd.get_vec2();
                f.markers.push_back(m);
            }
            const auto w = rd.get<std::int32_t>(), h = rd.get<std::int32_t>();
            const auto has_conf = rd.get<std::uint32_t>();
            const auto blob = rd.get<std::uint64_t>();
            if (!rd.ok) break;
            const auto header_bytes = static_cast<std::uint64_t>(rd.p - buf.data());
            if (header_bytes + blob > size) break;
            r->frames_.push_back(std::move(f));
            r->blocks_.push_back({payload_at + header_bytes, blob, w, h, has_conf != 0});
        } else {
            buf.resize(size);
            r->in_.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(size));
            if (!r->in_) break;
            Reader rd{buf.data(), buf.data() + buf.size()};
            if (t == kHead) {
                auto& k = r->header_.depth_intrinsics;
                k.width = rd.get<std::int32_t>();
                k.height = rd.get<std::int32_t>();
                k.fx = rd.get<double>(), k.fy = rd.get<double>(), k.cx = rd.get<double>(), k.cy = rd.get<double>();
                r->header_.rect_f = rd.get<double>();
                r->header_.rect_cx = rd.get<double>();
                r->header_.rect_cy = rd.get<double>();
                r->header_.baseline_mm = rd.get<double>();
                r->header_.description = rd.get_string();
                have_header = rd.ok;
            } else if (t == kGlobal) {
                const auto n = rd.get<std::uint32_t>();
                std::vector<markers::MapMarker> map;
                for (std::uint32_t i = 0; i < n && rd.ok; ++i) {
                    markers::MapMarker m;
                    m.id = rd.get<std::int32_t>();
                    m.position = rd.get_vec3();
                    m.diameter = rd.get<double>();
                    m.fixed = true;
                    m.observations = 1;
                    map.push_back(m);
                }
                if (rd.ok) r->global_markers_ = std::move(map);
            } else if (t == kDevice) {
                auto d = decode_device(rd);
                if (rd.ok) r->device_ = std::move(d);
            } else if (t == kExtras) {
                const auto index = rd.get<std::uint64_t>();
                auto e = decode_extras(rd);
                // Written right after its frame.
                if (rd.ok && !r->frames_.empty() && r->frames_.back().index == index) r->frames_.back().extras = std::move(e);
            } else if (t == kDropped) {
                DroppedFrame d;
                d.index = rd.get<std::uint64_t>();
                d.timestamp_s = rd.get<double>();
                d.reason = rd.get_string();
                if (rd.ok) r->dropped_.push_back(std::move(d));
            }
        }
        r->in_.seekg(static_cast<std::streamoff>(payload_at + size));
        if (!r->in_) break;
    }
    r->in_.clear();
    if (!have_header) return make_error(Errc::protocol, path + " has no session header");
    return r;
}

Result<FrameRecord> SessionReader::read(std::size_t i) const {
    if (i >= frames_.size()) return make_error(Errc::invalid_argument, "frame index out of range");
    FrameRecord f = frames_[i];
    const ImageBlock& b = blocks_[i];
    std::vector<std::uint8_t> data(b.size);
    {
        std::lock_guard lock(file_mutex_);
        in_.seekg(static_cast<std::streamoff>(b.offset));
        in_.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(b.size));
        if (!in_) {
            in_.clear();
            return make_error(Errc::io, std::format("frame {}: truncated image data", i));
        }
    }
    if (!decode_images(data.data(), data.size(), b.width, b.height, b.has_confidence, f.depth, f.confidence))
        return make_error(Errc::io, std::format("frame {}: corrupt image data", i));
    return f;
}

std::uint64_t SessionReader::raw_bytes() const {
    std::uint64_t n = 0;
    for (const auto& b : raws_) n += b.size;
    return n;
}

Result<RawFrame> SessionReader::read_raw(std::size_t i) const {
    if (i >= raws_.size()) return make_error(Errc::invalid_argument, "raw frame index out of range");
    const RawBlock& b = raws_[i];
    std::vector<std::uint8_t> data(b.size);
    {
        std::lock_guard lock(file_mutex_);
        in_.seekg(static_cast<std::streamoff>(b.offset));
        in_.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(b.size));
        if (!in_) {
            in_.clear();
            return make_error(Errc::io, std::format("raw frame {}: truncated", i));
        }
    }
    Reader rd{data.data(), data.data() + data.size()};
    RawFrame f;
    f.index = rd.get<std::uint64_t>();
    f.timestamp_s = rd.get<double>();
    const auto n = rd.get<std::uint32_t>();
    for (std::uint32_t k = 0; k < n && rd.ok; ++k) {
        const auto sensor = rd.get<std::int32_t>(), w = rd.get<std::int32_t>(), h = rd.get<std::int32_t>();
        const auto blob = rd.get_bytes();
        if (!rd.ok || w <= 0 || h <= 0) break;
        ImageU8 img(w, h, 0);
        const std::size_t got = ZSTD_decompress(img.data(), img.pixels().size(), blob.data(), blob.size());
        if (ZSTD_isError(got) || got != img.pixels().size()) return make_error(Errc::io, std::format("raw frame {}: corrupt image", i));
        for (int y = 0; y < h; ++y) {
            std::uint8_t* row = img.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(w);
            std::uint8_t acc = 0;
            for (int x = 0; x < w; ++x) row[x] = acc = static_cast<std::uint8_t>(acc + row[x]);
        }
        f.images.emplace_back(sensor, std::move(img));
    }
    if (!rd.ok) return make_error(Errc::io, std::format("raw frame {}: corrupt record", i));
    return f;
}

}  // namespace einstar::session
