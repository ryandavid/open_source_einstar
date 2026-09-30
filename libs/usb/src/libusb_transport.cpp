#include <libusb.h>

#include <algorithm>
#include <atomic>
#include <format>
#include <chrono>
#include <mutex>
#include <optional>
#include <thread>

#include "einstar/core/log.hpp"
#include "einstar/usb/constants.hpp"
#include "einstar/usb/transport.hpp"

namespace einstar::usb {
namespace {

Error usb_error(int rc, std::string_view what) {
    const Errc code = rc == LIBUSB_ERROR_TIMEOUT      ? Errc::timeout
                      : rc == LIBUSB_ERROR_NO_DEVICE  ? Errc::disconnected
                      : rc == LIBUSB_ERROR_NOT_FOUND  ? Errc::not_found
                      : rc == LIBUSB_ERROR_BUSY       ? Errc::busy
                                                      : Errc::io;
    return Error{code, std::format("{}: {}", what, libusb_error_name(rc))};
}

std::string port_path(libusb_device* dev) {
    std::uint8_t ports[8];
    const int n = libusb_get_port_numbers(dev, ports, 8);
    std::string s = std::format("{}", libusb_get_bus_number(dev));
    for (int i = 0; i < n; ++i) s += std::format("{}{}", i == 0 ? '-' : '.', ports[i]);
    return s;
}

struct ContextDeleter {
    void operator()(libusb_context* c) const { libusb_exit(c); }
};
using ContextPtr = std::unique_ptr<libusb_context, ContextDeleter>;

class LibusbTransport final : public Transport {
public:
    LibusbTransport(ContextPtr ctx, libusb_device_handle* handle, UsbDeviceInfo info)
        : ctx_(std::move(ctx)), handle_(handle), info_(std::move(info)) {}

    ~LibusbTransport() override {
        stop_stream();
        libusb_release_interface(handle_, kInterface);
        libusb_close(handle_);
    }

    std::string description() const override {
        return std::format("USB {:04x}:{:04x} at {}", info_.vendor_id, info_.product_id, info_.path);
    }

    Result<std::vector<std::uint8_t>> command(std::span<const std::uint8_t> request, std::size_t reply_capacity,
                                              unsigned timeout_ms) override {
        std::lock_guard lock(command_mutex_);
        ++stats_.commands;
        std::vector<std::uint8_t> out(request.begin(), request.end());
        int transferred = 0;
        int rc = libusb_interrupt_transfer(handle_, kEpCommandOut, out.data(), static_cast<int>(out.size()), &transferred,
                                           timeout_ms);
        if (rc != 0) {
            ++stats_.command_failures;
            return std::unexpected(usb_error(rc, "command send"));
        }
        std::vector<std::uint8_t> reply(reply_capacity);
        rc = libusb_interrupt_transfer(handle_, kEpCommandIn, reply.data(), static_cast<int>(reply.size()), &transferred,
                                       timeout_ms);
        if (rc != 0) {
            ++stats_.command_failures;
            return std::unexpected(usb_error(rc, "command reply"));
        }
        reply.resize(static_cast<std::size_t>(transferred));
        return reply;
    }

    Result<std::vector<std::uint8_t>> bulk(std::span<const std::uint8_t> request, std::size_t reply_size,
                                           unsigned timeout_ms) override {
        std::lock_guard lock(bulk_mutex_);
        // The firmware only sees a request once its 5120-byte DMA buffer is full, so pad to exactly that.
        if (request.size() > kBulkRequestSize)
            return make_error(Errc::invalid_argument, std::format("bulk request of {} bytes exceeds {}", request.size(), kBulkRequestSize));
        std::vector<std::uint8_t> out(request.begin(), request.end());
        out.resize(kBulkRequestSize, 0);
        for (std::size_t off = 0; off < out.size(); off += kBulkChunk) {
            int transferred = 0;
            const int rc = libusb_bulk_transfer(handle_, kEpBulkOut, out.data() + off, static_cast<int>(kBulkChunk),
                                                &transferred, timeout_ms);
            if (rc != 0) return std::unexpected(usb_error(rc, "bulk send"));
        }
        // Read exactly the reply size in 1 KiB chunks (a short packet also ends it).
        std::vector<std::uint8_t> reply;
        reply.reserve(reply_size);
        std::vector<std::uint8_t> chunk(kBulkChunk);
        while (reply.size() < reply_size) {
            const auto want = std::min(kBulkChunk, reply_size - reply.size());
            int transferred = 0;
            const int rc = libusb_bulk_transfer(handle_, kEpBulkIn, chunk.data(), static_cast<int>(want), &transferred,
                                                timeout_ms);
            if (rc != 0) {
                if (rc == LIBUSB_ERROR_TIMEOUT) libusb_clear_halt(handle_, kEpBulkIn);
                return std::unexpected(usb_error(rc, "bulk reply"));
            }
            reply.insert(reply.end(), chunk.begin(), chunk.begin() + transferred);
            if (static_cast<std::size_t>(transferred) < want) break;
        }
        return reply;
    }

    Result<void> reset_command_pipe() override {
        std::lock_guard lock(command_mutex_);
        libusb_clear_halt(handle_, kEpCommandOut);
        libusb_clear_halt(handle_, kEpCommandIn);
        return {};
    }

    Result<void> reset_bulk_pipe() override {
        std::lock_guard lock(bulk_mutex_);
        libusb_clear_halt(handle_, kEpBulkOut);
        libusb_clear_halt(handle_, kEpBulkIn);
        return {};
    }

    Result<void> start_stream(PacketHandler handler) override {
        if (streaming_) return make_error(Errc::busy, "stream already running");
        handler_ = std::move(handler);
        resubmit_ = true;
        in_flight_ = 0;
        transfers_.clear();
        for (std::size_t i = 0; i < kStreamTransfersInFlight; ++i) {
            Slot slot;
            slot.buffer.resize(kStreamTransferSize);
            slot.transfer = libusb_alloc_transfer(0);
            transfers_.push_back(std::move(slot));
        }
        for (auto& slot : transfers_) {
            libusb_fill_bulk_transfer(slot.transfer, handle_, kEpStreamIn, slot.buffer.data(),
                                      static_cast<int>(slot.buffer.size()), &LibusbTransport::on_transfer, this,
                                      kStreamTransferTimeoutMs);
            const int rc = libusb_submit_transfer(slot.transfer);
            if (rc != 0) {
                log::error("stream submit failed: {}", libusb_error_name(rc));
                continue;
            }
            ++in_flight_;
        }
        if (in_flight_ == 0) {
            free_transfers();
            return make_error(Errc::io, "no stream transfers could be submitted");
        }
        streaming_ = true;
        event_thread_ = std::jthread([this](std::stop_token st) { event_loop(st); });
        return {};
    }

    void stop_stream() override {
        if (!streaming_) return;
        resubmit_ = false;
        for (auto& slot : transfers_) libusb_cancel_transfer(slot.transfer);
        // The event thread keeps pumping until every transfer has come back.
        event_thread_.request_stop();
        event_thread_.join();
        free_transfers();
        streaming_ = false;
    }

    TransportStats stats() const override {
        TransportStats s = stats_;
        s.stream_packets = packets_.load(std::memory_order_relaxed);
        s.stream_bytes = bytes_.load(std::memory_order_relaxed);
        s.stream_timeouts = timeouts_.load(std::memory_order_relaxed);
        s.stream_errors = errors_.load(std::memory_order_relaxed);
        s.stream_stalls = stalls_.load(std::memory_order_relaxed);
        return s;
    }

private:
    struct Slot {
        libusb_transfer* transfer = nullptr;
        std::vector<std::uint8_t> buffer;
    };

    static void LIBUSB_CALL on_transfer(libusb_transfer* t) {
        auto* self = static_cast<LibusbTransport*>(t->user_data);
        switch (t->status) {
            case LIBUSB_TRANSFER_COMPLETED:
                self->packets_.fetch_add(1, std::memory_order_relaxed);
                self->bytes_.fetch_add(static_cast<std::uint64_t>(t->actual_length), std::memory_order_relaxed);
                if (self->handler_) self->handler_(std::span(t->buffer, static_cast<std::size_t>(t->actual_length)));
                break;
            case LIBUSB_TRANSFER_TIMED_OUT:
                self->timeouts_.fetch_add(1, std::memory_order_relaxed);
                break;
            case LIBUSB_TRANSFER_CANCELLED:
                --self->in_flight_;
                return;
            case LIBUSB_TRANSFER_STALL: {
                // The image endpoint halted: resubmitting cannot succeed until the halt is cleared. Park the
                // transfer; the event thread clears the halt (a synchronous request, not allowed here) and
                // resubmits. The firmware's CLEAR_FEATURE handler itself restarts GPIF, the DMA channel and the
                // endpoint, so the stream resumes. It also arms a full restart: the next device-state read
                // (00/07) that sees FPGA state bit 17 disconnects from USB, reloads the FPGA and the sensor
                // tables and re-enumerates (docs/firmware.md 5). EinstarDevice reconnects and replays its
                // settings when that happens (ConnectOptions::reopen).
                self->errors_.fetch_add(1, std::memory_order_relaxed);
                std::lock_guard lock(self->stall_mutex_);
                self->stalled_.push_back(t);
                --self->in_flight_;
                return;
            }
            default:
                self->errors_.fetch_add(1, std::memory_order_relaxed);
                break;
        }
        if (self->resubmit_ && libusb_submit_transfer(t) == 0) return;
        --self->in_flight_;
    }

    void event_loop(std::stop_token st) {
        timeval tv{0, 100000};
        std::optional<std::chrono::steady_clock::time_point> deadline;
        while (!st.stop_requested() || in_flight_ > 0) {
            libusb_handle_events_timeout_completed(ctx_.get(), &tv, nullptr);
            std::vector<libusb_transfer*> stalled;
            {
                std::lock_guard lock(stall_mutex_);
                stalled.swap(stalled_);
            }
            if (!stalled.empty() && resubmit_ && !st.stop_requested()) {
                log::warn("image endpoint stalled; clearing the halt and resuming");
                stalls_.fetch_add(1, std::memory_order_relaxed);
                libusb_clear_halt(handle_, kEpStreamIn);
                for (auto* t : stalled)
                    if (libusb_submit_transfer(t) == 0) ++in_flight_;
            }
            if (st.stop_requested() && in_flight_ > 0) {
                if (!deadline) deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
                if (std::chrono::steady_clock::now() > *deadline) {
                    log::warn("stream stop: {} transfers did not return", in_flight_.load());
                    break;
                }
                // Keep cancelling until all transfers are back (bounded by the 500 ms transfer timeout).
                for (auto& slot : transfers_) libusb_cancel_transfer(slot.transfer);
            }
        }
    }

    void free_transfers() {
        for (auto& slot : transfers_) libusb_free_transfer(slot.transfer);
        transfers_.clear();
    }

    ContextPtr ctx_;
    libusb_device_handle* handle_;
    UsbDeviceInfo info_;
    std::mutex command_mutex_, bulk_mutex_;
    TransportStats stats_;

    PacketHandler handler_;
    std::vector<Slot> transfers_;
    std::atomic<bool> resubmit_{false};
    std::mutex stall_mutex_;
    std::vector<libusb_transfer*> stalled_;  // (guarded by stall_mutex_)
    std::atomic<int> in_flight_{0};
    bool streaming_ = false;
    std::jthread event_thread_;
    std::atomic<std::uint64_t> packets_{0}, bytes_{0}, timeouts_{0}, errors_{0}, stalls_{0};
};

}  // namespace

Result<std::vector<UsbDeviceInfo>> enumerate_devices() {
    libusb_context* raw = nullptr;
    if (const int rc = libusb_init(&raw); rc != 0) return std::unexpected(usb_error(rc, "libusb_init"));
    ContextPtr ctx(raw);
    libusb_device** list = nullptr;
    const auto n = libusb_get_device_list(ctx.get(), &list);
    if (n < 0) return std::unexpected(usb_error(static_cast<int>(n), "get_device_list"));
    std::vector<UsbDeviceInfo> out;
    for (ssize_t i = 0; i < n; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(list[i], &desc) != 0 || desc.idVendor != kVendorId) continue;
        out.push_back({desc.idVendor, desc.idProduct, libusb_get_bus_number(list[i]), libusb_get_port_number(list[i]),
                       port_path(list[i])});
    }
    libusb_free_device_list(list, 1);
    return out;
}

Result<std::unique_ptr<Transport>> open_libusb(const UsbDeviceInfo& info) {
    libusb_context* raw = nullptr;
    if (const int rc = libusb_init(&raw); rc != 0) return std::unexpected(usb_error(rc, "libusb_init"));
    ContextPtr ctx(raw);

    libusb_device** list = nullptr;
    const auto n = libusb_get_device_list(ctx.get(), &list);
    if (n < 0) return std::unexpected(usb_error(static_cast<int>(n), "get_device_list"));
    libusb_device_handle* handle = nullptr;
    int open_rc = LIBUSB_ERROR_NOT_FOUND;
    for (ssize_t i = 0; i < n; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(list[i], &desc) != 0) continue;
        if (desc.idVendor == info.vendor_id && desc.idProduct == info.product_id && port_path(list[i]) == info.path) {
            open_rc = libusb_open(list[i], &handle);
            break;
        }
    }
    libusb_free_device_list(list, 1);
    if (open_rc != 0) return std::unexpected(usb_error(open_rc, "open"));

    if (const int rc = libusb_claim_interface(handle, kInterface); rc != 0) {
        libusb_close(handle);
        return std::unexpected(usb_error(rc, "claim interface 0"));
    }
    // Drain stale replies left from a previous session, then clear both IN pipes.
    std::vector<std::uint8_t> scratch(kBulkChunk);
    int transferred = 0;
    for (int i = 0; i < 5; ++i)
        if (libusb_interrupt_transfer(handle, kEpCommandIn, scratch.data(), static_cast<int>(scratch.size()), &transferred, 100) != 0) break;
    for (int i = 0; i < 30; ++i)
        if (libusb_bulk_transfer(handle, kEpBulkIn, scratch.data(), static_cast<int>(scratch.size()), &transferred, 100) != 0) break;
    libusb_clear_halt(handle, kEpCommandIn);
    libusb_clear_halt(handle, kEpBulkIn);

    return std::unique_ptr<Transport>(new LibusbTransport(std::move(ctx), handle, info));
}

Result<std::unique_ptr<Transport>> reopen_libusb(const UsbDeviceInfo& previous) {
    auto devices = enumerate_devices();
    if (!devices) return std::unexpected(devices.error());
    if (devices->empty()) return make_error(Errc::not_found, "no Shining3D device attached");
    const auto same = std::ranges::find_if(*devices, [&](const UsbDeviceInfo& d) { return d.path == previous.path; });
    return open_libusb(same != devices->end() ? *same : devices->front());
}

}  // namespace einstar::usb
