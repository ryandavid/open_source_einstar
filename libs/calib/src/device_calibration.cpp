#include "einstar/calib/device_calibration.hpp"

#include <bit>
#include <cstring>
#include <format>
#include <fstream>
#include <iterator>
#include <algorithm>

namespace einstar::calib {
namespace {

// Per-field obfuscation: field index -> (key index, sign used when the value was stored).
struct FieldKey {
    int field, key, sign;
};
constexpr FieldKey kFieldKeys[] = {
    {0, 0, +1},   {1, 1, -1},   {4, 2, -1},   {5, 3, +1},   {8, 4, +1},   {10, 5, +1},  {11, 6, -1},  {12, 7, +1},
    {13, 8, +1},  {14, 9, +1},  {20, 50, -1}, {21, 51, -1}, {22, 52, -1}, {23, 33, -1}, {24, 34, +1}, {25, 35, -1},
    {26, 36, +1}, {27, 37, +1}, {28, 38, +1}, {29, 39, -1}, {30, 40, -1}, {31, 41, -1},
};
// How the key is formed from the two files' tables: K[i] = L[i] + sign * R[i].
struct KeySign {
    int key, sign;
};
constexpr KeySign kKeySigns[] = {
    {0, +1},  {1, +1},  {2, -1},  {3, -1},  {4, -1},  {5, -1},  {6, -1},  {7, +1},  {8, +1},  {9, +1},  {33, +1},
    {34, -1}, {35, +1}, {36, -1}, {37, +1}, {38, -1}, {39, +1}, {40, +1}, {41, -1}, {50, -1}, {51, +1}, {52, -1},
};

template <typename T>
T load_le(const std::uint8_t* p) {
    static_assert(std::endian::native == std::endian::little);
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

std::array<std::int32_t, kCcfTailInts> read_tail(std::span<const std::uint8_t> b) {
    std::array<std::int32_t, kCcfTailInts> t{};
    for (std::size_t i = 0; i < kCcfTailInts; ++i) t[i] = load_le<std::int32_t>(b.data() + kPlainCcfSize + 4 * i);
    return t;
}

bool tag_at(std::span<const std::uint8_t> blob, std::size_t off, std::string_view tag) {
    return off + tag.size() <= blob.size() && std::memcmp(blob.data() + off, tag.data(), tag.size()) == 0;
}

Result<std::vector<std::uint8_t>> read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return make_error(Errc::not_found, std::format("cannot open {}", path));
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(f), {});
}

}  // namespace

Result<std::array<double, kCcfDoubles>> read_ccf_doubles(std::span<const std::uint8_t> bytes) {
    if (bytes.size() < kPlainCcfSize)
        return make_error(Errc::invalid_argument, std::format("CCF too short ({} bytes)", bytes.size()));
    std::array<double, kCcfDoubles> d{};
    for (std::size_t i = 0; i < kCcfDoubles; ++i) d[i] = load_le<double>(bytes.data() + 8 * i);
    return d;
}

Result<std::pair<std::array<double, kCcfDoubles>, std::array<double, kCcfDoubles>>>
decode_ccf_pair(std::span<const std::uint8_t> left_file, std::span<const std::uint8_t> right_file) {
    if (left_file.size() < kObfuscatedCcfSize || right_file.size() < kObfuscatedCcfSize)
        return make_error(Errc::invalid_argument, "Left/Right CCF must be 520 bytes");
    auto l = read_ccf_doubles(left_file);
    auto r = read_ccf_doubles(right_file);
    if (!l || !r) return make_error(Errc::invalid_argument, "bad CCF");
    const auto tl = read_tail(left_file), tr = read_tail(right_file);
    std::array<double, kCcfTailInts> key{};
    for (const auto [k, sign] : kKeySigns) key[static_cast<std::size_t>(k)] = tl[static_cast<std::size_t>(k)] + sign * tr[static_cast<std::size_t>(k)];
    for (const auto [field, k, sign] : kFieldKeys) {
        (*l)[static_cast<std::size_t>(field)] -= sign * key[static_cast<std::size_t>(k)];
        (*r)[static_cast<std::size_t>(field)] -= sign * key[static_cast<std::size_t>(k)];
    }
    return std::pair{*l, *r};
}

CameraCalibration camera_from_ccf(const std::array<double, kCcfDoubles>& d, int width, int height) {
    CameraCalibration c;
    c.model.width = width;
    c.model.height = height;
    c.model.fx = d[0];
    c.model.fy = d[1];
    c.model.cx = d[4];
    c.model.cy = d[5];
    c.model.skew = d[8] * d[0];  // alpha is dimensionless (Bouguet); skew in pixels = alpha * fx
    c.model.dist = {d[10], d[11], d[12], d[13], d[14]};
    c.t_cam_world = Vec3(d[20], d[21], d[22]);
    for (int i = 0; i < 9; ++i) c.R_cam_world(i / 3, i % 3) = d[static_cast<std::size_t>(23 + i)];
    c.rms_error = d[32];
    return c;
}

RigCalibration DeviceCalibration::rig() const {
    RigCalibration r;
    r.left = left.model;
    r.right = right.model;
    r.texture = texture.model;
    const SE3 T_world_left = left.T_cam_world().inverse();
    r.T_right_left = right.T_cam_world() * T_world_left;
    r.T_texture_left = texture.T_cam_world() * T_world_left;
    return r;
}

Result<DeviceCalibration> decode_ccf_files(std::span<const std::uint8_t> left_file, std::span<const std::uint8_t> right_file,
                                           std::span<const std::uint8_t> tex_file) {
    auto lr = decode_ccf_pair(left_file, right_file);
    if (!lr) return std::unexpected(lr.error());
    auto tex = read_ccf_doubles(tex_file);
    if (!tex) return std::unexpected(tex.error());
    DeviceCalibration cal;
    cal.left = camera_from_ccf(lr->first, 1280, 1024);
    cal.right = camera_from_ccf(lr->second, 1280, 1024);
    cal.texture = camera_from_ccf(*tex, 1280, 1024);
    // Sanity: rotations must be orthonormal, otherwise the key/offset table was wrong.
    for (const auto* c : {&cal.left, &cal.right, &cal.texture}) {
        const double err = (c->R_cam_world * c->R_cam_world.transpose() - Mat3::Identity()).norm();
        if (err > 1e-3 || c->model.fx < 100 || c->model.fx > 10000)
            return make_error(Errc::protocol, std::format("decoded calibration is implausible (R error {:.3g}, fx {:.1f})",
                                                          err, c->model.fx));
    }
    return cal;
}

Result<DeviceCalibration> decode_flash_blob(std::span<const std::uint8_t> blob) {
    if (blob.size() < kFlashBlobSize) return make_error(Errc::invalid_argument, "flash blob must be 6568 bytes");
    // Quick CCF section: type u32 @0x39B, three {name[260], i32 len, data[1024]} entries, tag FQFQ @0x12B7.
    constexpr std::size_t kQuick = 0x39B;
    if (load_le<std::uint32_t>(blob.data() + kQuick) != 4 || !tag_at(blob, 0x12B7, "FQFQ"))
        return make_error(Errc::not_found, "flash blob has no quick-calibration section");
    auto entry = [&](int i) -> std::pair<std::span<const std::uint8_t>, std::string> {
        const std::size_t base = kQuick + 4 + static_cast<std::size_t>(i) * 0x508;
        const auto len = load_le<std::int32_t>(blob.data() + base + 260);
        const char* name = reinterpret_cast<const char*>(blob.data() + base);
        std::string label(name, strnlen(name, 260));
        const auto n = static_cast<std::size_t>(std::clamp(len, 0, 1024));
        return {blob.subspan(base + 264, n), label};
    };
    const auto [lf, ln] = entry(0);
    const auto [rf, rn] = entry(1);
    const auto [tf, tn] = entry(2);
    auto cal = decode_ccf_files(lf, rf, tf);
    if (!cal) return cal;
    cal->calibration_time = tn;

    // Colour section: two 81-byte records {u32 type, f32[15] ccm+offset+gain, f32[3] dark, "FBFB"}.
    if (tag_at(blob, 0x2F9 + 0x4C, "FBFB") && tag_at(blob, 0x2F9 + 81 + 0x4C, "FBFB")) {
        std::array<ColorRecord, 2> recs{};
        for (int r = 0; r < 2; ++r) {
            const std::uint8_t* p = blob.data() + 0x2F9 + 81 * r + 4;
            auto& rec = recs[static_cast<std::size_t>(r)];
            for (int i = 0; i < 9; ++i) rec.ccm[static_cast<std::size_t>(i)] = load_le<float>(p + 4 * i);
            for (int i = 0; i < 3; ++i) rec.offset[static_cast<std::size_t>(i)] = load_le<float>(p + 36 + 4 * i);
            for (int i = 0; i < 3; ++i) rec.gain[static_cast<std::size_t>(i)] = load_le<float>(p + 48 + 4 * i);
            for (int i = 0; i < 3; ++i) rec.dark[static_cast<std::size_t>(i)] = load_le<float>(p + 60 + 4 * i);
        }
        cal->color = recs;
    }
    // White balance: {u32 type = 5, f64[9], f64[9], "FWFW"} at 0x12BC.
    if (tag_at(blob, 0x12BC + 0x94, "FWFW")) {
        std::array<Mat3, 2> wb;
        for (int m = 0; m < 2; ++m)
            for (int i = 0; i < 9; ++i)
                wb[static_cast<std::size_t>(m)](i / 3, i % 3) = load_le<double>(blob.data() + 0x12BC + 4 + 72 * m + 8 * i);
        cal->white_balance = wb;
    }
    return cal;
}

std::vector<std::uint8_t> encode_quick_flash_blob(std::span<const std::uint8_t> l, std::span<const std::uint8_t> r,
                                                  std::span<const std::uint8_t> t, const std::string& time) {
    std::vector<std::uint8_t> blob(kFlashBlobSize, 0);
    const std::uint32_t type = 4;
    std::memcpy(blob.data() + 0x39B, &type, 4);
    const std::span<const std::uint8_t> files[3] = {l, r, t};
    for (int i = 0; i < 3; ++i) {
        const std::size_t base = 0x39B + 4 + static_cast<std::size_t>(i) * 0x508;
        if (i == 2) std::memcpy(blob.data() + base, time.data(), std::min<std::size_t>(time.size(), 259));
        const auto len = static_cast<std::int32_t>(std::min<std::size_t>(files[i].size(), 1024));
        std::memcpy(blob.data() + base + 260, &len, 4);
        std::memcpy(blob.data() + base + 264, files[i].data(), static_cast<std::size_t>(len));
    }
    std::memcpy(blob.data() + 0x12B7, "FQFQ", 5);
    return blob;
}

Result<std::vector<std::uint8_t>> encode_quick_flash_blob_from_directory(const std::string& dir) {
    auto l = read_file(dir + "/LeftCCF.txt");
    auto r = read_file(dir + "/RightCCF.txt");
    auto t = read_file(dir + "/TexCCF.txt");
    if (!l || !r || !t) return make_error(Errc::not_found, "CCF files not found in " + dir);
    return encode_quick_flash_blob(*l, *r, *t, "emulated");
}

Result<DeviceCalibration> load_ccf_directory(const std::string& dir) {
    auto l = read_file(dir + "/LeftCCF.txt");
    auto r = read_file(dir + "/RightCCF.txt");
    auto t = read_file(dir + "/TexCCF.txt");
    if (!l) return std::unexpected(l.error());
    if (!r) return std::unexpected(r.error());
    if (!t) return std::unexpected(t.error());
    return decode_ccf_files(*l, *r, *t);
}

}  // namespace einstar::calib
