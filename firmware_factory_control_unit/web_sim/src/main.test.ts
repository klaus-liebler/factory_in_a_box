// @vitest-environment jsdom
// (jsdom, not happy-dom: happy-dom cannot parse lit templates with a <tr> root - the IDE's declaration table)
// The simulation-only app: it starts without any device, offers only the simulator (no FCU command in the
// ribbons of the IDE and the HMI runtime), routes by hash and runs the example project in the simulator.
import { describe, expect, it } from "vitest";
import { appRibbonOf } from "@fiab/ui-shell/shell/ribbon.ts";
import { remoteDevice, simulator } from "@fiab/plc/ide/devices.ts";
import { build, exampleProject } from "@fiab/plc/ide/project.ts";
import { IO_VARS } from "@fiab/plc/ide/io-list.ts";
import "./main.ts";

const tick = () => new Promise((r) => setTimeout(r, 0));

function commandIds(el: Element | null): string[] {
	return appRibbonOf(el as HTMLElement).flatMap((t) => t.groups.flatMap((g) => g.commands.map((c) => c.id)));
}

describe("simulation-only app", () => {
	it("starts with the IDE and the HMI runtime, simulator only", async () => {
		window.history.replaceState(null, "", "/");
		document.body.innerHTML = "<app-shell></app-shell>";
		document.dispatchEvent(new Event("DOMContentLoaded"));
		await tick();
		await tick();
		expect(remoteDevice()).toBeNull();
		expect(document.querySelector(".tb-conn")).toBeNull();
		const ide = document.querySelector("plc-ide-app");
		const hmi = document.querySelector("hmi-runtime-app");
		expect(ide).not.toBeNull();
		expect(hmi).not.toBeNull();
		expect(commandIds(ide)).toContain("sim");
		expect(commandIds(ide)).not.toContain("fcu");

		window.location.hash = "#/hmi";
		window.dispatchEvent(new PopStateEvent("popstate"));
		await tick();
		expect((hmi as HTMLElement).hidden).toBe(false);
		expect((ide as HTMLElement).hidden).toBe(true);
		expect(commandIds(hmi)).not.toContain("fcu");
		expect(commandIds(hmi)).not.toContain("src-device");
	});

	it("has the FCU's I/O and runs the example project in the simulator", () => {
		expect(IO_VARS.length).toBeGreaterThan(0);
		expect(simulator.download(build(exampleProject()).result)).toBe(true);
		simulator.singleCycle();
		expect(simulator.cycles).toBe(1);
		expect(simulator.error).toBeNull();
	});
});
