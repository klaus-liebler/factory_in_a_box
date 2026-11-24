// The role table of the WebSocket requests (src/ws_permissions.hpp, docs/security.md "Roles"):
// each request type needs the role the concept gives it, the critical ones also physical presence,
// and a message the table does not know needs the admin role (closed by default).
#include <gtest/gtest.h>

#include "ws_permissions.hpp"

namespace {

namespace w = WsProtocol;
using fcu::auth::ws_permission;
using rtos::auth::Decision;
using rtos::auth::Permission;
using rtos::auth::Role;
using rtos::auth::decide;

bool same(Permission a, Permission b) { return a.least == b.least && a.presence == b.presence; }
namespace perm = rtos::auth::permission;

struct Row {
    std::uint16_t ns, type;
    Permission need;
    const char* name;
};

const Row rows[] = {
    {w::system::NAMESPACE_ID, w::system::SystemInfoRequest::TYPE_ID, perm::read, "system info"},
    {w::modbus::NAMESPACE_ID, w::modbus::GetRegistersRequest::TYPE_ID, perm::read, "read registers"},
    {w::modbus::NAMESPACE_ID, w::modbus::WriteHoldingRequest::TYPE_ID, perm::operate, "write registers"},
    {w::ethercat::NAMESPACE_ID, w::ethercat::StatusRequest::TYPE_ID, perm::read, "EtherCAT status"},
    {w::tasks::NAMESPACE_ID, w::tasks::TaskManagerRequest::TYPE_ID, perm::read, "task list"},
    {w::tasks::NAMESPACE_ID, w::tasks::PoolListRequest::TYPE_ID, perm::read, "pool list"},
    {w::terminal::NAMESPACE_ID, w::terminal::LookupRequest::TYPE_ID, perm::read, "terminal completion"},
    {w::terminal::NAMESPACE_ID, w::terminal::CommandRequest::TYPE_ID, perm::admin, "terminal command"},
    {w::plc::NAMESPACE_ID, w::plc::StatusRequest::TYPE_ID, perm::read, "PLC status"},
    {w::plc::NAMESPACE_ID, w::plc::WatchRequest::TYPE_ID, perm::read, "PLC watch"},
    {w::plc::NAMESPACE_ID, w::plc::GetProjectRequest::TYPE_ID, perm::read, "PLC project upload"},
    {w::plc::NAMESPACE_ID, w::plc::ControlRequest::TYPE_ID, perm::program, "PLC run/stop"},
    {w::plc::NAMESPACE_ID, w::plc::WriteRequest::TYPE_ID, perm::program, "PLC online write"},
    {w::plc::NAMESPACE_ID, w::plc::PutProjectRequest::TYPE_ID, perm::program, "PLC project save"},
    {w::plc::NAMESPACE_ID, w::plc::BreakpointsRequest::TYPE_ID, perm::program, "breakpoints"},
    {w::plc::NAMESPACE_ID, w::plc::HaltRequest::TYPE_ID, perm::program, "halt/step"},
    {w::plc::NAMESPACE_ID, w::plc::UnforceRequest::TYPE_ID, perm::program, "unforce"},
    {w::plc::NAMESPACE_ID, w::plc::UploadRequest::TYPE_ID, perm::program_critical, "PLC download"},
    {w::plc::NAMESPACE_ID, w::plc::CommitRequest::TYPE_ID, perm::program_critical, "PLC commit"},
    {w::plc::NAMESPACE_ID, w::plc::ForceRequest::TYPE_ID, perm::program_critical, "force"},
    {w::roarm::NAMESPACE_ID, w::roarm::ListMissionsRequest::TYPE_ID, perm::read, "list missions"},
    {w::roarm::NAMESPACE_ID, w::roarm::JointJogTarget::TYPE_ID, perm::operate, "jog the arm"},
    {w::roarm::NAMESPACE_ID, w::roarm::Mission::TYPE_ID, perm::operate, "play a mission"},
    {w::roarm::NAMESPACE_ID, w::roarm::SaveMissionRequest::TYPE_ID, perm::program, "save a mission"},
    {w::stepper::NAMESPACE_ID, w::stepper::StatusRequest::TYPE_ID, perm::read, "stepper status"},
    {w::stepper::NAMESPACE_ID, w::stepper::MoveRequest::TYPE_ID, perm::operate, "move an axis"},
    {w::stepper::NAMESPACE_ID, w::stepper::StopRequest::TYPE_ID, perm::operate, "stop an axis"},
    {w::stepper::NAMESPACE_ID, w::stepper::SettingsRequest::TYPE_ID, perm::program, "axis settings"},
};

TEST(WsPermissions, EveryRequestHasTheRoleOfTheConcept) {
    for (const Row& r : rows) {
        EXPECT_TRUE(same(ws_permission(r.ns, r.type), r.need)) << r.name;
    }
}

TEST(WsPermissions, UnknownMessagesAndResponsesNeedAdmin) {
    EXPECT_TRUE(same(ws_permission(0xFFFF, 1), perm::admin));
    EXPECT_TRUE(same(ws_permission(w::plc::NAMESPACE_ID, 0xFFFF), perm::admin));
    // a response type sent to the device is no request: closed as well
    EXPECT_TRUE(same(ws_permission(w::modbus::NAMESPACE_ID, w::modbus::RegistersMessage::TYPE_ID), perm::admin));
    EXPECT_TRUE(same(ws_permission(w::pneumatics::NAMESPACE_ID, w::pneumatics::PressureControlFeedback::TYPE_ID), perm::admin));
}

// The escalation paths of docs/security.md: who may do the critical things, from where.
TEST(WsPermissions, RolesAndPresenceTogether) {
    const auto download = ws_permission(w::plc::NAMESPACE_ID, w::plc::UploadRequest::TYPE_ID);
    const auto write = ws_permission(w::modbus::NAMESPACE_ID, w::modbus::WriteHoldingRequest::TYPE_ID);
    EXPECT_EQ(decide({.role = Role::Observer}, write), Decision::RoleTooLow);
    EXPECT_EQ(decide({.role = Role::Operator}, write), Decision::Allowed);
    EXPECT_EQ(decide({.role = Role::Operator, .present = true}, download), Decision::RoleTooLow);
    EXPECT_EQ(decide({.role = Role::Admin}, download), Decision::PresenceRequired);     // LAN
    EXPECT_EQ(decide({.role = Role::Programmer, .present = true}, download), Decision::Allowed);  // USB
    EXPECT_EQ(decide({.role = Role::Admin, .must_change_password = true, .present = true}, write),
              Decision::PasswordChangeRequired);
}

}  // namespace
