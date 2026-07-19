// Entry-Point: baut die Dashboard-Shell auf und haengt die drei Apps ein (Reihenfolge der
// RegisterApp()-Aufrufe bestimmt die Reihenfolge in der Sidebar). Kein Websocket, kein Login --
// jede App spricht bei Bedarf direkt schlankes HTTP/Fetch (s. api.ts).
import "./shell/app-shell.js";
import "./apps/modbus-register-app.js";
import "./apps/power-management-app.js";
import "./apps/system-info-app.js";
import type { AppShell } from "./shell/app-shell.js";

document.addEventListener("DOMContentLoaded", () => {
	const shell = document.querySelector("app-shell") as AppShell;

	shell.RegisterApp("/", "Modbus-Register", "🔧", document.createElement("modbus-register-app"));
	shell.RegisterApp("/power", "Power Management", "⚡", document.createElement("power-management-app"));
	shell.RegisterApp("/system", "System", "🖥️", document.createElement("system-info-app"));

	shell.Start();
});
