// Entry-Point: baut die Dashboard-Shell auf und haengt die Apps ein (Reihenfolge der
// RegisterApp()-Aufrufe bestimmt die Reihenfolge in der Sidebar). Kein Login -- jede App spricht
// bei Bedarf schlankes HTTP/Fetch (s. api.ts); daneben laeuft dauerhaft die WebSocket-Verbindung
// (s. ws-client.ts) fuer Server-Push-Daten (aktuell: gespiegelte Firmware-Logs).
import "./shell/app-shell.js";
import "./apps/modbus-register-app.js";
import "./apps/power-management-app.js";
import "./apps/system-info-app.js";
import type { AppShell } from "./shell/app-shell.js";
import { startWebSocketClient } from "./ws-client.js";

document.addEventListener("DOMContentLoaded", () => {
	const shell = document.querySelector("app-shell") as AppShell;

	shell.RegisterApp("/", "Modbus-Register", "🔧", document.createElement("modbus-register-app"));
	shell.RegisterApp("/power", "Power Management", "⚡", document.createElement("power-management-app"));
	shell.RegisterApp("/system", "System", "🖥️", document.createElement("system-info-app"));

	shell.Start();
	startWebSocketClient();
});
