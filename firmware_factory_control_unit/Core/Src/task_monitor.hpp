#pragma once
// Beantwortet tasks.TaskManagerRequest (s. best_binary_buffers_schema/tasks.cs) -- s.
// task_monitor.cpp fuer die Begruendung des Request/Response- statt Broadcast-Ansatzes.
#include <cstdint>
#include "http_websocket_server.hpp"

// request_id: unveraendert aus TaskManagerRequest::Payload.requestId uebernommen und in
// TaskListMessage::Payload.requestId zurueckgegeben (vom generierten Code automatisch zu jedem
// Request/Response-Paar hinzugefuegt, s. Core/generated/ws_protocol.hh).
void HandleTaskManagerRequest(Http::WebSocketConnection &conn, uint16_t request_id);
