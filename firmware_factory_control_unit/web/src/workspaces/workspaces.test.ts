// @vitest-environment happy-dom
// Safety net for every workspace (app) of the web UI: it renders, publishes its ribbon, polls the device while
// it is shown, and sends nothing any more once it is hidden. The WebSocket layer is replaced: every request
// fails at once ("offline"), so polling loops keep running as they do without a device.
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

const net = vi.hoisted(() => ({ requests: 0, sent: 0, listeners: 0 }));

vi.mock("../core/ws-client.ts", () => ({
	wsState: () => "closed",
	onWsStateChange: () => () => undefined,
	registerWsProtocolListener: () => {
		net.listeners++;
		return () => void net.listeners--;
	},
	sendBinary: () => {
		net.sent++;
		return false;
	},
	startWebSocketClient: () => undefined,
	wsRequest: () => {
		net.requests++;
		return Promise.reject(new Error("offline (test)"));
	},
}));

// happy-dom has no WebGL (RoArm 3D view) and does not parse HTML inside an SVG <foreignObject> set by innerHTML
// (Druckregelstrecke schematic): both views become no-op stand-ins; the rest of those apps runs for real.
const { noopView } = vi.hoisted(() => ({ noopView: () => new Proxy({}, { get: () => () => undefined }) }));
vi.mock("./roarm/roarm-3d-view.ts", async (importOriginal) => ({ ...(await importOriginal<object>()), createRoArm3DView: noopView }));
vi.mock("./druckregelstrecke/druckregelstrecke-view.ts", async (importOriginal) => ({ ...(await importOriginal<object>()), createDruckregelstreckeView: noopView }));

import { appRibbonOf } from "@klaus-liebler/ui-shell/shell/ribbon.ts";
import type { WorkspaceApp } from "@klaus-liebler/ui-shell/shell/workspace-app.ts";
import "./dashboard/overview-app.ts";
import "./registers/modbus-register-app.ts";
import "./actuators/actuator-test-app.ts";
import "./power/power-management-app.ts";
import "./system/system-info-app.ts";
import "./tasks/task-manager-app.ts";
import "./druckregelstrecke/druckregelstrecke-app.ts";
import "./steppers/stepper-lab-app.ts";
import "./terminal/terminal-app.ts";
import "./roarm/roarm-teach-app.ts";
import "../plc-fcu/setup.ts";
import "@fiab/plc/ide/plc-ide-app.ts";
import "@fiab/plc/hmi/hmi-runtime-app.ts";

/** tag, does it poll the device while shown */
const WORKSPACES: [string, boolean][] = [
	["overview-app", true],
	["modbus-register-app", true],
	["actuator-test-app", true],
	["power-management-app", true],
	["system-info-app", true],
	["task-manager-app", true],
	["druckregelstrecke-app", true],
	["stepper-lab-app", true],
	["terminal-app", false],
	["roarm-teach-app", false],
	["plc-ide-app", false],
	["hmi-runtime-app", false],
];

type App = HTMLElement & WorkspaceApp & { updateComplete: Promise<boolean> };

beforeEach(() => {
	vi.useFakeTimers();
	net.requests = net.sent = 0;
	document.body.innerHTML = "";
});
afterEach(() => {
	vi.useRealTimers();
});

describe.each(WORKSPACES)("workspace <%s>", (tag, polls) => {
	it("renders, publishes a ribbon, polls only while shown", async () => {
		// happy-dom cannot parse lit templates that start with <tr> (a real browser can): start the IDE in a
		// view without table rows
		window.history.replaceState(null, "", tag === "plc-ide-app" ? "/plc?view=lib" : "/");
		const app = document.createElement(tag) as App;
		document.body.appendChild(app);
		await app.updateComplete;
		app.onShow();
		await vi.advanceTimersByTimeAsync(5000);
		await app.updateComplete;
		expect(app.childElementCount, "rendered nothing").toBeGreaterThan(0);
		const tabs = appRibbonOf(app);
		expect(tabs.length, "no ribbon").toBeGreaterThan(0);
		expect(tabs.every((t) => t.groups.length > 0 && t.groups.every((g) => g.commands.length > 0))).toBe(true);
		if (polls) expect(net.requests + net.sent, "no request while shown").toBeGreaterThan(0);

		app.onHide();
		await vi.advanceTimersByTimeAsync(100);  // a request already on its way may finish
		net.requests = net.sent = 0;
		await vi.advanceTimersByTimeAsync(10_000);
		expect(net.requests + net.sent, "requests after onHide").toBe(0);
		app.remove();
	});
});
