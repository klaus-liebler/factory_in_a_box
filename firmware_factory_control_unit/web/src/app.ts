// Entry point: builds the application window (@klaus-liebler/ui-shell, shell/app-shell.ts) and registers the apps as workspaces;
// the order of the RegisterApp() calls is the order in the workspace switcher, grouped by its section.
// The first workspace ("/", the dashboard) is shown at the start. Next to them the WebSocket connection
// (ws-client.ts) runs for the pushed data (e.g. the firmware log, log-store.ts).
import "@klaus-liebler/ui-shell/ui/base.css";
import "@klaus-liebler/ui-shell/ui/components.css";
import "@klaus-liebler/ui-shell/shell/app-shell.ts";
import "./workspaces/dashboard/overview-app.ts";
import "./workspaces/actuators/actuator-test-app.ts";
import "./workspaces/registers/modbus-register-app.ts";
import "./workspaces/power/power-management-app.ts";
import "./workspaces/system/system-info-app.ts";
import "./workspaces/roarm/roarm-teach-app.ts";
import "./workspaces/tasks/task-manager-app.ts";
import "./workspaces/druckregelstrecke/druckregelstrecke-app.ts";
import "./workspaces/terminal/terminal-app.ts";
import "./plc-fcu/setup.ts";
import "@fiab/plc/ide/plc-ide-app.ts";
import "@fiab/plc/hmi/hmi-runtime-app.ts";
import "./workspaces/steppers/stepper-lab-app.ts";
import type { AppShell } from "@klaus-liebler/ui-shell/shell/app-shell.ts";
import { onWsStateChange, startWebSocketClient, wsState } from "./core/ws-client.ts";
import { logEntries, startLogStore, subscribeLog } from "./core/log-store.ts";
import { system } from "../generated/ws-protocol.ts";
import { DEVICE_HOSTNAME } from "../generated/build-info.ts";
import logoSvg from "./assets/factory-control-unit-logo.svg?raw";

const isWarning = (level: number) => level >= system.LogLevel.LOG_WARN;

document.addEventListener("DOMContentLoaded", () => {
	const shell = document.querySelector("app-shell") as AppShell;
	const add = (path: string, title: string, icon: Parameters<AppShell["RegisterApp"]>[2], tag: string, group: string, description: string) =>
		shell.RegisterApp(path, title, icon, document.createElement(tag), { group, description });

	add("/", "Dashboard", "dashboard", "overview-app", "Übersicht", "Systemzustand auf einen Blick");
	add("/plc", "SPS-Entwicklung", "plc", "plc-ide-app", "Automatisierung", "IEC 61131-3: ST, FUP, AS, Visualisierung");
	add("/hmi", "Bedienen & Beobachten", "hmi", "hmi-runtime-app", "Automatisierung", "Bedienseiten, Meldungen, Linienschreiber");
	add("/actuators", "Aktorentest", "actuator", "actuator-test-app", "Anlage", "Förderband, Kompressor, Ventile, Beleuchtung direkt bedienen");
	add("/registers", "Register", "registers", "modbus-register-app", "Anlage", "Modbus-Register lesen und schreiben");
	add("/druckregelstrecke", "Druckregelstrecke", "pressure", "druckregelstrecke-app", "Anlage", "Kennlinie, Sprungantwort, Reglerentwurf");
	add("/steppers", "Schrittmotoren", "stepper", "stepper-lab-app", "Anlage", "Achsen fahren, referenzieren, parametrieren");
	add("/roarm-teach", "RoArm Teach", "robot", "roarm-teach-app", "Anlage", "Roboterarm teachen und Missionen abspielen");
	add("/system", "System", "chip", "system-info-app", "Gerät", "Board, Build, Netzwerk, I²C");
	add("/power", "Power Management", "power", "power-management-app", "Gerät", "Spannung, Strom, Leistung im Verlauf");
	add("/tasks", "Task Manager", "tasks", "task-manager-app", "Gerät", "Threads, CPU-Last, Stacks, Speicherpools");
	add("/terminal", "Terminal", "terminal", "terminal-app", "Gerät", "Testkommandos und Firmware-Log");

	shell.Start({
		productName: "Factory Control Unit",
		logoSvg,
		hostLabel: `${DEVICE_HOSTNAME}.local`,
		connection: { state: wsState, onChange: (cb) => void onWsStateChange(cb), title: "WebSocket-Verbindung zur Control Unit" },
		notifications: {
			initial: () => logEntries().filter((e) => isWarning(e.level)).length,
			subscribe: (cb) => void subscribeLog((added) => cb(added.filter((e) => isWarning(e.level)).length)),
			path: "/terminal",
			title: (n) => (n ? `${n} neue Warnungen/Fehler im Firmware-Log` : "Firmware-Log"),
		},
	});
	startLogStore();
	startWebSocketClient();
});
