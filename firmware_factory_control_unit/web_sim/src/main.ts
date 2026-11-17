// The Factory Control Unit's PLC without hardware: the IDE (ST, FBD, SFC, HMI editor) and the HMI runtime
// of @fiab/plc, with the simulator and its plant model as the only device - no FCU is registered, so
// IDE and HMI offer no device connection. The I/O variables are those of the FCU (its register map), so a
// program written here runs unchanged on the device.
//
// A static site for any web server: hash routing ("#/hmi"), the projects stay in the browser (localStorage).
import "@klaus-liebler/ui-shell/ui/base.css";
import "@klaus-liebler/ui-shell/ui/components.css";
import "@klaus-liebler/ui-shell/shell/app-shell.ts";
import { useHashRouting } from "@klaus-liebler/ui-shell/shell/router.ts";
import type { AppShell } from "@klaus-liebler/ui-shell/shell/app-shell.ts";
import logoSvg from "../../web/src/assets/factory-control-unit-logo.svg?raw";
import "./register-map.ts";
import "@fiab/plc/ide/plc-ide-app.ts";
import "@fiab/plc/hmi/hmi-runtime-app.ts";

useHashRouting();

document.addEventListener("DOMContentLoaded", () => {
	const shell = document.querySelector("app-shell") as AppShell;
	const add = (path: string, title: string, icon: Parameters<AppShell["RegisterApp"]>[2], tag: string, description: string) =>
		shell.RegisterApp(path, title, icon, document.createElement(tag), { group: "Automatisierung", description });

	add("/", "SPS-Entwicklung", "plc", "plc-ide-app", "IEC 61131-3: ST, FUP, AS, Visualisierung – im Simulator");
	add("/hmi", "Bedienen & Beobachten", "hmi", "hmi-runtime-app", "Bedienseiten, Meldungen, Linienschreiber");

	shell.Start({ productName: "Factory Control Unit – SPS-Simulator", logoSvg, hostLabel: "Simulation im Browser" });
});
