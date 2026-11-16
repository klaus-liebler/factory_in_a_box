// Task manager (see task_monitor.hpp).
#include "task_monitor.hpp"

#include <algorithm>
#include <array>
#include <cstring>

#include "network.hpp"
#include "rtos/net/tls_memory.hpp"
#include "board/chip.hpp"
#include "ws_protocol.hh"

namespace fcu::task_monitor {
namespace {

namespace ws = WsProtocol::tasks;

struct Entry {
    Kernel::Thread* thread = nullptr;
    std::span<std::byte> stack;
    std::uint64_t last_cycles = 0;
};
std::array<Entry, KernelConfig::max_threads> entries{};
std::size_t count = 0;
std::uint32_t last_sample = 0;
bool sampled = false;

template <std::size_t N>
void fill(std::uint8_t (&out)[N], const char* text) noexcept {
    std::memset(out, 0, N);
    std::strncpy(reinterpret_cast<char*>(out), text, N - 1);
}

void tasks(WsProtocol::Responder& reply, const ws::TaskManagerRequest::Payload&) noexcept {
    const std::uint32_t now = board::CycleCounter::now();
    const std::uint32_t wall = sampled ? now - last_sample : 0;
    std::array<std::uint8_t, KernelConfig::max_threads * ws::TaskListMessageItems_ELEMENT_SIZE> items{};
    std::size_t pos = 0;
    for (std::size_t i = 0; i < count; ++i) {
        Entry& e = entries[i];
        const Kernel::Thread& t = *e.thread;
        ws::TaskInfo info{};
        fill(info.name, t.name);
        info.priority = t.priority;
        info.stackSizeBytes = static_cast<std::uint32_t>(e.stack.size());
        const auto sp = reinterpret_cast<std::uintptr_t>(t.sp);
        const auto end = reinterpret_cast<std::uintptr_t>(e.stack.data() + e.stack.size());
        info.stackUsedCurrentBytes = sp != 0 && sp <= end ? static_cast<std::uint32_t>(end - sp) : 0;
        info.stackUsedHighWaterBytes = static_cast<std::uint32_t>(stack_used(e.stack));
        info.runCount = t.run_count;
        const std::uint64_t cycles = t.run_cycles;
        info.cpuPermille = wall != 0 ? static_cast<std::uint16_t>(std::min<std::uint64_t>(1000, (cycles - e.last_cycles) * 1000 / wall)) : 0;
        e.last_cycles = cycles;
        pos += ws::EncodeTaskListMessageItemsElement(info, items.data() + pos, items.size() - pos);
    }
    last_sample = now;
    sampled = true;
    reply.Send(ws::TaskListMessage::Encode, ws::TaskListMessage::Payload{.itemsData = items.data(), .itemsCount = count});
}

void pools(WsProtocol::Responder& reply, const ws::PoolListRequest::Payload&) noexcept {
    std::array<ws::PoolInfo, 2> info{};
    const net::PoolUse packets = net::packet_pool_use();
    fill(info[0].name, "Packets (buffers)");
    info[0].capacity = packets.capacity;
    info[0].free = packets.available;
    info[0].minFree = packets.available;
    const rtos::net::TlsPoolStats& tls = rtos::net::tls_pool_stats();
    fill(info[1].name, "wolfSSL (bytes)");
    info[1].capacity = tls.total_bytes;
    info[1].free = tls.total_bytes - tls.used_bytes;
    info[1].minFree = tls.total_bytes - tls.peak_bytes;  // sizes the pool: the peak of real use
    std::array<std::uint8_t, 2 * ws::PoolListMessagePools_ELEMENT_SIZE> data{};
    std::size_t pos = 0;
    for (const auto& i : info) {
        pos += ws::EncodePoolListMessagePoolsElement(i, data.data() + pos, data.size() - pos);
    }
    reply.Send(ws::PoolListMessage::Encode,
               ws::PoolListMessage::Payload{.poolsData = data.data(), .poolsCount = info.size(), .freeHeapBytes = 0});
}

}  // namespace

void add(Kernel::Thread& thread, std::span<std::byte> stack) noexcept {
    if (count < entries.size()) {
        entries[count++] = {&thread, stack, 0};
    }
}

std::size_t stack_used(std::span<const std::byte> stack) noexcept {
    const auto untouched = std::find_if(stack.begin(), stack.end(), [](std::byte b) { return b != paint; });
    return static_cast<std::size_t>(stack.end() - untouched);
}

void handle_message(WsProtocol::Responder& reply, std::span<const std::uint8_t> frame) noexcept {
    switch (reply.Request().messageTypeId) {
        case ws::TaskManagerRequest::TYPE_ID: return WsProtocol::Dispatch(reply, frame.data(), frame.size(), ws::TaskManagerRequest::Decode, tasks);
        case ws::PoolListRequest::TYPE_ID: return WsProtocol::Dispatch(reply, frame.data(), frame.size(), ws::PoolListRequest::Decode, pools);
        default: return reply.Error(WsProtocol::ErrorCode::UnknownMessage, "unknown message");
    }
}

}  // namespace fcu::task_monitor
