// @vitest-environment happy-dom
// The application window: workspaces in the switcher, the ribbon of the active workspace (plus the shell's
// "Ansicht" tab), commands, group menus, collapsing, and the lifecycle of the apps when the workspace changes.
import { describe, expect, it } from "vitest";
import "./app-shell.ts";
import type { AppShell } from "./app-shell.ts";
import { setAppRibbon } from "./ribbon.ts";

class FakeApp extends HTMLElement {
	shown = 0;
	hidden_ = 0;
	ran: string[] = [];
	onShow(): void {
		this.shown++;
		setAppRibbon(this, [{ id: "t", label: `Tab ${this.id}`, groups: [{ label: "Gruppe", commands: [
			{ id: "a", label: "Aktion", icon: "run", run: () => this.ran.push("a") },
			{ id: "b", label: "Gesperrt", icon: "stop", disabled: true, run: () => this.ran.push("b") },
			{ id: "c", label: "Klein", icon: "add", size: "small", run: () => this.ran.push("c") },
		] }] }]);
	}
	onHide(): void {
		this.hidden_++;
	}
}
customElements.define("fake-app", FakeApp);

const tick = () => new Promise((r) => setTimeout(r, 0));

async function setup() {
	window.history.replaceState(null, "", "/");
	document.body.innerHTML = "";
	const shell = document.createElement("app-shell") as AppShell;
	document.body.appendChild(shell);
	const a = document.createElement("fake-app") as FakeApp;
	a.id = "A";
	const b = document.createElement("fake-app") as FakeApp;
	b.id = "B";
	shell.RegisterApp("/", "Dashboard", "dashboard", a, { group: "Übersicht", description: "Start" });
	shell.RegisterApp("/b", "Zweites", "plc", b, { group: "Automatisierung", description: "Zweiter Arbeitsbereich" });
	shell.Start();
	await shell.updateComplete;
	await tick();
	await shell.updateComplete;
	return { shell, a, b };
}

describe("app-shell", () => {
	it("starts with the first workspace and shows its ribbon and the Ansicht tab", async () => {
		const { shell, a, b } = await setup();
		expect(a.shown).toBe(1);
		expect(a.hidden).toBe(false);
		expect(b.hidden).toBe(true);
		expect(shell.querySelector(".rb-switcher-name")?.textContent).toBe("Dashboard");
		expect([...shell.querySelectorAll(".rb-tab")].map((t) => t.textContent)).toEqual(["Tab A", "Ansicht"]);
		expect(shell.querySelectorAll(".rb-large")).toHaveLength(2);
		expect(shell.querySelectorAll(".rb-small")).toHaveLength(1);
	});

	it("runs commands, not disabled ones, also from the group menu", async () => {
		const { shell, a } = await setup();
		(shell.querySelector(".rb-large") as HTMLButtonElement).click();
		expect(a.ran).toEqual(["a"]);
		expect((shell.querySelectorAll(".rb-large")[1] as HTMLButtonElement).disabled).toBe(true);
		(shell.querySelector(".rb-group-caption") as HTMLButtonElement).click();
		await shell.updateComplete;
		const items = [...shell.querySelectorAll(".rb-menu-item")] as HTMLButtonElement[];
		expect(items.map((i) => i.textContent?.trim())).toEqual(["Aktion", "Gesperrt", "Klein"]);
		items[2].click();
		await shell.updateComplete;
		expect(a.ran).toEqual(["a", "c"]);
		expect(shell.querySelector(".rb-menu")).toBeNull();
	});

	it("switches the workspace from the switcher: hide/show, ribbon, URL", async () => {
		const { shell, a, b } = await setup();
		(shell.querySelector(".rb-switcher") as HTMLButtonElement).click();
		await shell.updateComplete;
		const items = [...shell.querySelectorAll(".ws-menu-item")] as HTMLButtonElement[];
		expect(items).toHaveLength(2);
		expect([...shell.querySelectorAll(".ws-menu-group")].map((g) => g.textContent)).toEqual(["Übersicht", "Automatisierung"]);
		items[1].click();
		await tick();
		await shell.updateComplete;
		await tick();
		await shell.updateComplete;
		expect(window.location.pathname).toBe("/b");
		expect([a.hidden_, b.shown, a.hidden, b.hidden]).toEqual([1, 1, true, false]);
		expect(shell.querySelector(".rb-switcher-name")?.textContent).toBe("Zweites");
		expect(shell.querySelector(".rb-tab")?.textContent).toBe("Tab B");
		expect(shell.querySelector(".ws-switcher-menu")).toBeNull();
	});

	it("collapses the ribbon (Ansicht tab or double click) and expands it with a tab click", async () => {
		const { shell } = await setup();
		const tabs = () => [...shell.querySelectorAll(".rb-tab")] as HTMLButtonElement[];
		tabs()[0].dispatchEvent(new MouseEvent("dblclick", { bubbles: true }));
		await shell.updateComplete;
		expect(shell.querySelector(".rb-groups")).toBeNull();
		expect(shell.classList.contains("shell-ribbon-collapsed")).toBe(true);
		tabs()[1].click();
		await shell.updateComplete;
		expect(shell.querySelector(".rb-groups")).not.toBeNull();
		const collapse = [...shell.querySelectorAll(".rb-cmd")].find((c) => c.textContent?.includes("Ribbon einklappen")) as HTMLButtonElement;
		collapse.click();
		await shell.updateComplete;
		expect(shell.querySelector(".rb-groups")).toBeNull();
	});
});
