// Which role a WebSocket request needs (docs/security.md "Roles"): one table for every message the
// device accepts, checked on the device before the request is dispatched (web.cpp) - the web UI
// only hides what a role cannot do. A message missing here needs the admin role: a new request type
// is closed until it is classified. Host test: tests/host/ws_permissions_test.cpp.
//
//   read              Observer    status, values, logs, lists
//   operate           Operator    write registers, move actuators, robots and axes
//   program           Programmer  PLC project, online changes, debugging, missions, axis settings
//   program_critical  + presence  PLC download/commit, forcing
//   admin             Admin       the terminal (its commands reach everything)
#pragma once

#include <cstdint>

#include "rtos/auth/access.hpp"
#include "ws_protocol.hh"

namespace fcu::auth {

namespace perm = rtos::auth::permission;

[[nodiscard]] constexpr rtos::auth::Permission ws_permission(std::uint16_t ns, std::uint16_t type) noexcept {
    namespace w = WsProtocol;
    switch (ns) {
        case w::system::NAMESPACE_ID:
            if (type == w::system::SystemInfoRequest::TYPE_ID) return perm::read;
            break;
        case w::modbus::NAMESPACE_ID:
            if (type == w::modbus::GetRegistersRequest::TYPE_ID) return perm::read;
            if (type == w::modbus::WriteHoldingRequest::TYPE_ID) return perm::operate;
            break;
        case w::ethercat::NAMESPACE_ID:
            if (type == w::ethercat::StatusRequest::TYPE_ID) return perm::read;
            break;
        case w::tasks::NAMESPACE_ID:
            if (type == w::tasks::TaskManagerRequest::TYPE_ID || type == w::tasks::PoolListRequest::TYPE_ID) return perm::read;
            break;
        case w::terminal::NAMESPACE_ID:
            if (type == w::terminal::LookupRequest::TYPE_ID) return perm::read;
            if (type == w::terminal::CommandRequest::TYPE_ID) return perm::admin;
            break;
        case w::plc::NAMESPACE_ID:
            switch (type) {
                case w::plc::StatusRequest::TYPE_ID:
                case w::plc::GetProjectRequest::TYPE_ID:
                case w::plc::WatchRequest::TYPE_ID: return perm::read;
                case w::plc::ControlRequest::TYPE_ID:  // RUN/STOP, control source, autostart
                case w::plc::WriteRequest::TYPE_ID:    // online change of a variable
                case w::plc::PutProjectRequest::TYPE_ID:
                case w::plc::UnforceRequest::TYPE_ID:
                case w::plc::BreakpointsRequest::TYPE_ID:
                case w::plc::HaltRequest::TYPE_ID: return perm::program;
                case w::plc::UploadRequest::TYPE_ID:  // the program download
                case w::plc::CommitRequest::TYPE_ID:
                case w::plc::ForceRequest::TYPE_ID: return perm::program_critical;
                default: break;
            }
            break;
        case w::roarm::NAMESPACE_ID:
            switch (type) {
                case w::roarm::ListMissionsRequest::TYPE_ID:
                case w::roarm::GetMissionRequest::TYPE_ID:
                case w::roarm::GetMissionGpioListRequest::TYPE_ID: return perm::read;
                case w::roarm::StartTeachModeRequest::TYPE_ID:
                case w::roarm::StopTeachModeRequest::TYPE_ID:
                case w::roarm::JointJogTarget::TYPE_ID:
                case w::roarm::CartesianJogTarget::TYPE_ID:
                case w::roarm::Mission::TYPE_ID: return perm::operate;
                case w::roarm::SaveMissionRequest::TYPE_ID:
                case w::roarm::DeleteMissionRequest::TYPE_ID: return perm::program;
                default: break;
            }
            break;
        case w::stepper::NAMESPACE_ID:
            switch (type) {
                case w::stepper::StatusRequest::TYPE_ID: return perm::read;
                case w::stepper::ManualRequest::TYPE_ID:
                case w::stepper::MoveRequest::TYPE_ID:
                case w::stepper::StopRequest::TYPE_ID:
                case w::stepper::HomeRequest::TYPE_ID: return perm::operate;
                case w::stepper::SettingsRequest::TYPE_ID: return perm::program;
                default: break;
            }
            break;
        case w::malibotti::NAMESPACE_ID:
            if (type == w::malibotti::Command::TYPE_ID) return perm::operate;
            break;
        default: break;
    }
    return perm::admin;  // not classified: closed
}

}  // namespace fcu::auth
