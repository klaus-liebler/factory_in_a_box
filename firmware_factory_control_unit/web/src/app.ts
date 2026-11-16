// Entry point: builds the application window (shell/app-shell.ts) and registers the apps as workspaces;
// the order of the RegisterApp() calls is the order in the workspace switcher, grouped by its section.
// The first workspace ("/", the dashboard) is shown at the start. Next to them the WebSocket connection
// (ws-client.ts) runs for the pushed data (e.g. the firmware log, log-store.ts).
import "./shell/app-shell.js";
import "./apps/overview-app.js";
import "./apps/modbus-register-app.js";
import "./apps/power-management-app.js";
import "./apps/system-info-app.js";
import "./apps/roarm-teach-app.js";
import "./apps/task-manager-app.js";
import "./apps/druckregelstrecke-app.js";
import "./apps/terminal-app.js";
import "./plc/ide/plc-ide-app.ts";
import "./plc/hmi/hmi-runtime-app.ts";
import "./apps/stepper-lab-app.js";
import type { AppShell } from "./shell/app-shell.js";
import { startWebSocketClient } from "./ws-client.js";
import { startLogStore } from "./log-store.js";

document.addEventListener("DOMContentLoaded", () => {
	const shell = document.querySelector("app-shell") as AppShell;
	const add = (path: string, title: string, icon: Parameters<AppShell["RegisterApp"]>[2], tag: string, group: string, description: string) =>
		shell.RegisterApp(path, title, icon, document.createElement(tag), { group, description });

	add("/", "Dashboard", "dashboard", "overview-app", "Übersicht", "Systemzustand auf einen Blick");
	add("/plc", "SPS-Entwicklung", "plc", "plc-ide-app", "Automatisierung", "IEC 61131-3: ST, FUP, AS, Visualisierung");
	add("/hmi", "Bedienen & Beobachten", "hmi", "hmi-runtime-app", "Automatisierung", "Bedienseiten, Meldungen, Linienschreiber");
	add("/registers", "Register", "registers", "modbus-register-app", "Anlage", "Modbus-Register lesen und schreiben");
	add("/druckregelstrecke", "Druckregelstrecke", "pressure", "druckregelstrecke-app", "Anlage", "Kennlinie, Sprungantwort, Reglerentwurf");
	add("/steppers", "Schrittmotoren", "stepper", "stepper-lab-app", "Anlage", "Achsen fahren, referenzieren, parametrieren");
	add("/roarm-teach", "RoArm Teach", "robot", "roarm-teach-app", "Anlage", "Roboterarm teachen und Missionen abspielen");
	add("/system", "System", "chip", "system-info-app", "Gerät", "Board, Build, Netzwerk, I²C");
	add("/power", "Power Management", "power", "power-management-app", "Gerät", "Spannung, Strom, Leistung im Verlauf");
	add("/tasks", "Task Manager", "tasks", "task-manager-app", "Gerät", "Threads, CPU-Last, Stacks, Speicherpools");
	add("/terminal", "Terminal", "terminal", "terminal-app", "Gerät", "Testkommandos und Firmware-Log");

	shell.Start();
	startLogStore();
	startWebSocketClient();
});
