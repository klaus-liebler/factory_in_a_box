// Polling of the device for a workspace, as a lit reactive controller: the task runs every `intervalMs`
// while the workspace is shown and not paused, never overlapping (the next run is scheduled when the last one
// has settled), and it stops when the element leaves the DOM. It also knows the connection state for the
// status bar, so the apps do not keep their own status message:
//
//   private poller = new Poller(this, () => this.poll(), 1000);
//   onShow() { this.poller.start(); }   onHide() { this.poller.stop(); }
//   willUpdate() { setAppStatus(this, this.poller.status()); }
//
// A task that only sends (the answer comes as a pushed message) calls `poller.ok()` when it arrives.
import type { ReactiveController, ReactiveControllerHost } from "lit";
import type { AppStatus } from "../shell/status-bar.ts";
import type { RibbonGroup } from "../shell/ribbon.ts";
import { clock } from "./format.ts";

/** The ribbon group "Daten" of a polling workspace: refresh now, pause/resume (all its pollers together). */
export function pollingGroup(pollers: Poller | readonly Poller[], what = "die Werte"): RibbonGroup {
	const ps = Array.isArray(pollers) ? pollers : [pollers as Poller];
	const paused = ps[0].paused;
	return { label: "Daten", commands: [
		{ id: "refresh", label: "Aktualisieren", icon: "refresh", title: `${what} jetzt abfragen`, run: () => ps.forEach((p) => p.refresh()) },
		{ id: "pause", label: paused ? "Fortsetzen" : "Anhalten", icon: paused ? "run" : "pause", active: paused,
			title: `Die laufende Abfrage anhalten (${what} bleiben stehen)`, run: () => ps.forEach((p) => p.setPaused(!paused)) },
	] };
}

export type PollState = "idle" | "loading" | "ok" | "error";

export class Poller implements ReactiveController {
	state: PollState = "idle";
	/** When the last run succeeded (null: never). */
	lastOk: Date | null = null;
	private running = false;
	private paused_ = false;
	private busy = false;
	private timer: ReturnType<typeof setTimeout> | null = null;
	/** Bumped by stop(): a run that settles after it schedules nothing. */
	private generation = 0;

	constructor(private readonly host: ReactiveControllerHost, private readonly task: () => Promise<unknown> | unknown, public intervalMs: number) {
		host.addController(this);
	}

	hostDisconnected(): void {
		this.stop();
	}

	get paused(): boolean {
		return this.paused_;
	}

	/** The workspace is shown: poll now and then every interval (unless paused). */
	start(): void {
		this.running = true;
		if (!this.paused_) this.schedule(0);
	}

	/** The workspace is hidden: no further run (a run on its way finishes, but schedules nothing). */
	stop(): void {
		this.running = false;
		this.generation++;
		this.clear();
	}

	setPaused(on: boolean): void {
		this.paused_ = on;
		if (on) {
			this.generation++;
			this.clear();
		} else if (this.running) {
			this.schedule(0);
		}
		this.host.requestUpdate();
	}

	/** Runs the task at once (a refresh button), then on the usual interval. */
	refresh(): void {
		if (this.running && !this.paused_) this.schedule(0);
		else void this.run();
	}

	/** For tasks whose answer is pushed: it arrived. */
	ok(): void {
		this.state = "ok";
		this.lastOk = new Date();
		this.host.requestUpdate();
	}

	/** The state for the status bar (shell/status-bar.ts). */
	status(loading = "Lade …"): AppStatus {
		if (this.paused_) return { text: "Angehalten", kind: "idle" };
		switch (this.state) {
			case "ok": return { text: `Verbunden – Stand ${this.lastOk ? clock(this.lastOk) : ""}`, kind: "ok" };
			case "error": return { text: "Keine Verbindung zur Control Unit", kind: "error" };
			default: return { text: loading, kind: "busy" };
		}
	}

	private clear(): void {
		if (this.timer !== null) clearTimeout(this.timer);
		this.timer = null;
	}

	private schedule(delayMs: number): void {
		this.clear();
		const gen = this.generation;
		this.timer = setTimeout(() => {
			this.timer = null;
			void this.run().finally(() => {
				if (gen === this.generation && this.running && !this.paused_ && this.timer === null) this.schedule(this.intervalMs);
			});
		}, delayMs);
	}

	private async run(): Promise<void> {
		if (this.busy) return;
		this.busy = true;
		if (this.state === "idle") this.state = "loading";
		try {
			await this.task();
			this.ok();
		} catch {
			this.state = "error";
			this.host.requestUpdate();
		} finally {
			this.busy = false;
		}
	}
}
