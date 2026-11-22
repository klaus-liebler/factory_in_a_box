// The PLC project's source on the SD card (see plc_project.hpp).
#include "plc_project.hpp"

#include <array>
#include <span>

#include "rtos/log/log.hpp"
#include "sd_card.hpp"
#include "system.hpp"
#include "web.hpp"
#include "ws_protocol.hh"

namespace fcu::plc::project {
namespace {

namespace ws = WsProtocol::plc;
using sd::fx;

constexpr const char* directory = "/PLC";
constexpr const char* path = "/PLC/project.fcuplc.json";
constexpr const char* tmp_path = "/PLC/project.tmp";

web::FrameQueue<ws::PutProjectRequest::PutProjectRequest_MAX_SIZE, 2> requests{"plc project"};
std::array<std::uint8_t, ws::GetProjectResponse::GetProjectResponse_MAX_SIZE> answer;
std::array<std::uint8_t, 1024> chunk;  ///< a GetProject chunk (static: not on the I/O thread's stack)

std::uint32_t prepared = 0;  ///< the card (sd::generation()) /PLC was created on
FX_FILE tmp{};
bool writing = false;        ///< tmp is open: a PutProject is under way
std::uint32_t written = 0;   ///< bytes of it in tmp

/// The card mounted and /PLC on it; else why not.
const char* ready() noexcept {
    if (!sd::mount()) return "no SD card in the device";
    if (prepared != sd::generation()) {
        if (!sd::ensure_directory(directory)) {
            sd::lost("plc project");
            return "/PLC could not be created on the SD card";
        }
        prepared = sd::generation();
    }
    return nullptr;
}

void abandon() noexcept {
    if (writing) {
        (void)fx_file_close(&tmp);
        (void)fx_file_delete(sd::media(), fx(tmp_path));
        writing = false;
    }
}

/// A chunk of the new project; the last one replaces the stored project. nullptr: done.
const char* put(const ws::PutProjectRequest::Payload& q) noexcept {
    if (q.totalBytes == 0 || q.totalBytes > max_bytes) return "the project is too large for the device";
    if (const char* e = ready()) {
        writing = false;  // a lost card closed every file
        return e;
    }
    FX_MEDIA* m = sd::media();
    if (q.offset == 0) {
        abandon();
        (void)fx_file_delete(m, fx(tmp_path));
        if (fx_file_create(m, fx(tmp_path)) != FX_SUCCESS || fx_file_open(m, &tmp, fx(tmp_path), FX_OPEN_FOR_WRITE) != FX_SUCCESS) {
            sd::lost("plc project");
            return "the project could not be written to the SD card";
        }
        writing = true;
        written = 0;
    }
    if (!writing || q.offset != written || q.dataCount == 0 || q.offset + q.dataCount > q.totalBytes) {
        abandon();
        return "chunk out of order";
    }
    if (fx_file_write(&tmp, const_cast<std::uint8_t*>(q.dataData), static_cast<ULONG>(q.dataCount)) != FX_SUCCESS) {
        abandon();
        sd::lost("plc project");
        return "the project could not be written to the SD card";
    }
    written += static_cast<std::uint32_t>(q.dataCount);
    if (written < q.totalBytes) return nullptr;
    writing = false;
    if (fx_file_close(&tmp) != FX_SUCCESS) {
        sd::lost("plc project");
        return "the project could not be written to the SD card";
    }
    (void)fx_file_delete(m, fx(path));
    if (fx_file_rename(m, fx(tmp_path), fx(path)) != FX_SUCCESS || !sd::flush()) {
        sd::lost("plc project");
        return "the project could not be stored on the SD card";
    }
    RTOS_LOG_INFO("plc: project source stored on the SD card (%lu bytes)", static_cast<unsigned long>(written));
    return nullptr;
}

/// A chunk of the stored project into `out`; total: its size. nullptr: done.
const char* get(std::uint32_t offset, std::span<std::uint8_t> out, std::uint32_t& total, std::size_t& got) noexcept {
    total = 0;
    got = 0;
    if (writing) return "the project is being written";
    if (const char* e = ready()) return e;
    FX_FILE f{};
    if (const UINT s = fx_file_open(sd::media(), &f, fx(path), FX_OPEN_FOR_READ); s != FX_SUCCESS) {
        if (s == FX_NOT_FOUND) return "no project stored on the device";
        sd::lost("plc project");
        return "the project could not be read from the SD card";
    }
    total = static_cast<std::uint32_t>(f.fx_file_current_file_size);
    const char* e = nullptr;
    if (offset > total) {
        e = "offset beyond the project";
    } else if (offset < total) {
        ULONG n = 0;
        const UINT r = fx_file_seek(&f, offset);
        if (r != FX_SUCCESS || (fx_file_read(&f, out.data(), static_cast<ULONG>(out.size()), &n) != FX_SUCCESS)) {
            e = "the project could not be read from the SD card";
        }
        got = n;
    }
    (void)fx_file_close(&f);
    if (e != nullptr && got == 0 && offset <= total) sd::lost("plc project");
    return e;
}

void put_project(WsProtocol::Responder& reply, const ws::PutProjectRequest::Payload& q) noexcept {
    const char* e = put(q);
    reply.Send(ws::PutProjectResponse::Encode, ws::PutProjectResponse::Payload{.ok = e == nullptr, .message = e ? e : ""});
}

void get_project(WsProtocol::Responder& reply, const ws::GetProjectRequest::Payload& q) noexcept {
    std::uint32_t total = 0;
    std::size_t got = 0;
    const char* e = get(q.offset, chunk, total, got);
    reply.Send(ws::GetProjectResponse::Encode,
               ws::GetProjectResponse::Payload{.ok = e == nullptr, .totalBytes = total, .offset = q.offset, .dataData = chunk.data(),
                                               .dataCount = got, .message = e ? e : ""});
}

void handle(WsProtocol::Responder& reply, const WsProtocol::FrameHeader& h, std::span<const std::uint8_t> frame) noexcept {
    if (h.messageTypeId == ws::PutProjectRequest::TYPE_ID) {
        WsProtocol::Dispatch(reply, frame.data(), frame.size(), ws::PutProjectRequest::Decode, put_project);
    } else {
        WsProtocol::Dispatch(reply, frame.data(), frame.size(), ws::GetProjectRequest::Decode, get_project);
    }
}

}  // namespace

bool post(net::Server::SessionId id, WsProtocol::Responder& reply, std::span<const std::uint8_t> frame) noexcept {
    const std::uint16_t type = reply.Request().messageTypeId;
    if (type != ws::PutProjectRequest::TYPE_ID && type != ws::GetProjectRequest::TYPE_ID) return false;
    requests.post(id, reply, frame);
    return true;
}

void cycle() noexcept {
    // one request per cycle: a chunk is one FileX write of 1 KB
    requests.drain(answer, handle, 1);
}

}  // namespace fcu::plc::project
