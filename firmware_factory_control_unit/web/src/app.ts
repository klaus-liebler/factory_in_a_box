// Entry point: builds the application window (@klaus-liebler/ui-shell, shell/app-shell.ts) and registers the apps as workspaces;
// the order of the RegisterApp() calls is the order in the workspace switcher, grouped by its section.
// The first workspace ("/", the dashboard) is shown at the start. Next to them the WebSocket connection
// (ws-client.ts) runs for the pushed data (e.g. the firmware log, log-store.ts).
//
// Login first (core/auth.ts, docs/security.md): the page itself loads without one, but the device opens the
// WebSocket only for a logged-in session. The workspaces a role cannot use are not offered (the device
// refuses their requests anyway, per message).
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
import "./workspaces/users/user-admin-app.ts";
import "./plc-fcu/setup.ts";
import "@fiab/plc/ide/plc-ide-app.ts";
import "@fiab/plc/hmi/hmi-runtime-app.ts";
import "./workspaces/steppers/stepper-lab-app.ts";
import type { AppShell } from "@klaus-liebler/ui-shell/shell/app-shell.ts";
import { onWsStateChange, setConnectGuard, startWebSocketClient, wsState } from "./core/ws-client.ts";
import { logEntries, startLogStore, subscribeLog } from "./core/log-store.ts";
import { hasRole, logout, refreshSession, type Role, ROLE_LABEL, type Session } from "./core/auth.ts";
import { showLogin } from "./core/login-view.ts";
import { system } from "../generated/ws-protocol.ts";
import { DEVICE_HOSTNAME } from "../generated/build-info.ts";
import logoSvg from "./assets/factory-control-unit-logo.svg?raw";

const isWarning = (level: number) => level >= system.LogLevel.LOG_WARN;

async function sessionOrLogin(): Promise<Session> {
	const s = await refreshSession().catch(() => null);
	return s && !s.mustChangePassword ? s : showLogin();
}

document.addEventListener("DOMContentLoaded", async () => {
	const s = await sessionOrLogin();
	const shell = document.querySelector("app-shell") as AppShell;
	const add = (least: Role, path: string, title: string, icon: Parameters<AppShell["RegisterApp"]>[2], tag: string, group: string, description: string) => {
		if (hasRole(least)) shell.RegisterApp(path, title, icon, document.createElement(tag), { group, description });
	};

	add("observer", "/", "Dashboard", "dashboard", "overview-app", "Übersicht", "Systemzustand auf einen Blick");
	add("programmer", "/plc", "SPS-Entwicklung", "plc", "plc-ide-app", "Automatisierung", "IEC 61131-3: ST, FUP, AS, Visualisierung");
	add("observer", "/hmi", "Bedienen & Beobachten", "hmi", "hmi-runtime-app", "Automatisierung", "Bedienseiten, Meldungen, Linienschreiber");
	add("operator", "/actuators", "Aktorentest", "actuator", "actuator-test-app", "Anlage", "Förderband, Kompressor, Ventile, Beleuchtung direkt bedienen");
	add("observer", "/registers", "Register", "registers", "modbus-register-app", "Anlage", "Modbus-Register lesen und schreiben");
	add("operator", "/druckregelstrecke", "Druckregelstrecke", "pressure", "druckregelstrecke-app", "Anlage", "Kennlinie, Sprungantwort, Reglerentwurf");
	add("operator", "/steppers", "Schrittmotoren", "stepper", "stepper-lab-app", "Anlage", "Achsen fahren, referenzieren, parametrieren");
	add("operator", "/roarm-teach", "RoArm Teach", "robot", "roarm-teach-app", "Anlage", "Roboterarm teachen und Missionen abspielen");
	add("observer", "/system", "System", "chip", "system-info-app", "Gerät", "Board, Build, Netzwerk, I²C");
	add("observer", "/power", "Power Management", "power", "power-management-app", "Gerät", "Spannung, Strom, Leistung im Verlauf");
	add("observer", "/tasks", "Task Manager", "tasks", "task-manager-app", "Gerät", "Threads, CPU-Last, Stacks, Speicherpools");
	add("admin", "/terminal", "Terminal", "terminal", "terminal-app", "Gerät", "Testkommandos und Firmware-Log");
	add("admin", "/users", "Benutzer", "user", "user-admin-app", "Gerät", "Konten und Rollen der Web-Oberfläche");

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
		user: {
			name: `${s.user} · ${ROLE_LABEL[s.role]}${s.present ? " · USB" : ""}`,
			logout: () => void logout().finally(() => location.reload()),
		},
	});
	// a reconnect needs a login: one that expired or was ended elsewhere shows the login again
	setConnectGuard(async () => {
		const now = await refreshSession().catch(() => undefined);
		if (now === null) location.reload();
		return now !== null;
	});
	startLogStore();
	startWebSocketClient();
});
