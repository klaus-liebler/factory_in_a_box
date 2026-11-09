// SFC (sequential function chart, IEC 61131-3 Ed. 3, 8.3) - the model of the editor and its lowering
// to ST (docs/plc-iec61131.md 5.4). The chart is a tree, as the editor lays it out: a sequence of
// steps and transitions, with alternative branches (one path, the leftmost true transition wins) and
// parallel branches (all paths, joined by one transition), and jumps. The lowering turns it into a
// graph of transitions (source steps -> target steps) and that into ST: every step is an SFC_STEP
// (.X active, .T time since activation), every cycle the transitions are evaluated on the old state,
// the enabled ones fire together, then the actions of the active steps run. DOM-free.

export type Qualifier = "N" | "S" | "R" | "P";

/** The step type of every SFC (a source file of its own, with the I/O list and the FCU library). */
export const SFC_FILE = "<sfc>";
export const SFC_STEP_SOURCE = "TYPE SFC_STEP : STRUCT X : BOOL; T : TIME; END_STRUCT; END_TYPE\n";

export interface SfcAction {
	qualifier: Qualifier;
	/** A BOOL variable (N: it is TRUE while the step is active; S/R: set/reset it) or ST statements. */
	body: string;
}

export type SfcItem =
	| { kind: "step"; id: string; name: string; actions: SfcAction[] }
	| { kind: "trans"; id: string; condition: string }
	| { kind: "branch"; id: string; type: "alt" | "par"; branches: SfcItem[][] }
	| { kind: "jump"; id: string; target: string };

export interface SfcPou {
	name: string;
	/** "PROGRAM <name>" and the VAR sections of the user's variables (the steps are added). */
	decl: string;
	/** The main sequence; its first item is the initial step. */
	chart: SfcItem[];
}

export interface SfcTransition {
	id: string;
	condition: string;
	sources: string[];
	targets: string[];
	/** Alternative branches: the transitions of the same divergence before this one (priority). */
	before: string[];
}

export interface SfcProblem {
	item?: string;
	message: string;
}

const IDENT = /^[A-Za-z_][A-Za-z0-9_]*$/;

/** The graph of the chart; problems for a structure the lowering cannot take. */
export function sfcGraph(pou: SfcPou): { steps: (SfcItem & { kind: "step" })[]; transitions: SfcTransition[]; problems: SfcProblem[] } {
	const steps: (SfcItem & { kind: "step" })[] = [];
	const transitions: SfcTransition[] = [];
	const problems: SfcProblem[] = [];
	const jumps: { t: SfcTransition[]; target: string; id: string }[] = [];

	/** Walks a sequence: `prev` the steps whose transition comes next, `open` transitions waiting for
	 *  their target steps. Returns both at the end of the sequence. */
	const walk = (seq: SfcItem[], prev: string[], open: SfcTransition[]): { prev: string[]; open: SfcTransition[] } => {
		for (const it of seq) {
			switch (it.kind) {
				case "step": {
					if (prev.length && !open.length) problems.push({ item: it.id, message: `zwischen zwei Schritten fehlt eine Transition (vor ${it.name})` });
					steps.push(it);
					for (const t of open) t.targets.push(it.name);
					open = [];
					prev = [it.name];
					break;
				}
				case "trans": {
					if (!prev.length) problems.push({ item: it.id, message: "eine Transition braucht einen Schritt davor" });
					const t: SfcTransition = { id: it.id, condition: it.condition, sources: prev, targets: [], before: [] };
					transitions.push(t);
					open = [t];
					prev = [];
					break;
				}
				case "branch": {
					if (it.branches.length < 2) problems.push({ item: it.id, message: "eine Verzweigung braucht mindestens zwei Zweige" });
					if (it.type === "alt") {
						// after a step; every path begins and ends with a transition
						if (!prev.length) problems.push({ item: it.id, message: "eine Alternativ-Verzweigung folgt auf einen Schritt" });
						const ends: SfcTransition[] = [];
						const firsts: string[] = [];
						for (const b of it.branches) {
							if (b[0]?.kind !== "trans") problems.push({ item: it.id, message: "jeder Alternativ-Zweig beginnt mit einer Transition" });
							const before = transitions.length;
							const r = walk(b, prev, []);
							const first = transitions[before];
							if (first) {
								first.before = [...firsts];
								firsts.push(first.id);
							}
							if (r.prev.length) problems.push({ item: it.id, message: "jeder Alternativ-Zweig endet mit einer Transition (oder einem Sprung)" });
							ends.push(...r.open);
						}
						open = ends;
						prev = [];
					} else {
						// after a transition; every path begins and ends with a step; joined by the next transition
						if (!open.length) problems.push({ item: it.id, message: "eine Parallel-Verzweigung folgt auf eine Transition" });
						const sync: string[] = [];
						for (const b of it.branches) {
							if (b[0]?.kind !== "step") problems.push({ item: it.id, message: "jeder Parallel-Zweig beginnt mit einem Schritt" });
							const r = walk(b, [], open);
							if (!r.prev.length) problems.push({ item: it.id, message: "jeder Parallel-Zweig endet mit einem Schritt" });
							sync.push(...r.prev);
						}
						open = [];
						prev = sync;
					}
					break;
				}
				case "jump": {
					if (!open.length) problems.push({ item: it.id, message: "ein Sprung folgt auf eine Transition" });
					jumps.push({ t: open, target: it.target, id: it.id });
					open = [];
					prev = [];
					break;
				}
			}
		}
		return { prev, open };
	};

	if (pou.chart[0]?.kind !== "step") problems.push({ message: "die Ablaufkette beginnt mit dem Initialschritt" });
	const end = walk(pou.chart, [], []);
	const initial = pou.chart[0]?.kind === "step" ? pou.chart[0].name : "";
	// the end of the chain: back to the initial step (as a jump would)
	for (const t of end.open) t.targets.push(initial);
	if (end.prev.length) problems.push({ message: "die Ablaufkette endet mit einer Transition oder einem Sprung, nicht mit einem Schritt" });
	const names = new Set<string>();
	for (const s of steps) {
		if (!IDENT.test(s.name)) problems.push({ item: s.id, message: `Schrittname "${s.name}" ist kein Bezeichner` });
		else if (names.has(s.name.toUpperCase())) problems.push({ item: s.id, message: `Schritt ${s.name} doppelt` });
		names.add(s.name.toUpperCase());
	}
	for (const j of jumps) {
		if (!names.has(j.target.toUpperCase())) problems.push({ item: j.id, message: `Sprungziel ${j.target} gibt es nicht` });
		for (const t of j.t) t.targets.push(j.target);
	}
	for (const t of transitions) if (!t.condition.trim()) problems.push({ item: t.id, message: "Transition ohne Bedingung" });
	return { steps, transitions, problems };
}

/** SFC -> ST: the step type, the POU with the user's declaration plus the steps, and the evolution. */
export function sfcSource(pou: SfcPou, end = "END_PROGRAM"): { text: string; problems: SfcProblem[] } {
	const g = sfcGraph(pou);
	const initial = g.steps[0]?.name ?? "";
	const tn = (t: SfcTransition, i: number) => `Trans${i + 1}`;
	const vars: string[] = ["    SfcStarted : BOOL;"];
	for (const s of g.steps) vars.push(`    ${s.name} : SFC_STEP;`, `    ${s.name}_Time : TON;`);
	g.transitions.forEach((t, i) => vars.push(`    ${tn(t, i)} : BOOL;`));
	const pulses = g.steps.filter((s) => s.actions.some((a) => a.qualifier === "P"));
	for (const s of pulses) vars.push(`    ${s.name}_Entered : R_TRIG;`);

	const out: string[] = [];
	out.push("(* the initial step *)", `IF NOT SfcStarted THEN SfcStarted := TRUE; ${initial}.X := TRUE; END_IF;`);
	out.push("(* step times *)");
	for (const s of g.steps) out.push(`${s.name}_Time(IN := ${s.name}.X, PT := T#24d); ${s.name}.T := ${s.name}_Time.ET;`);
	out.push("(* transitions: evaluated on the old state, the leftmost alternative wins *)");
	const idx = new Map(g.transitions.map((t, i) => [t.id, i]));
	g.transitions.forEach((t, i) => {
		const src = t.sources.map((s) => `${s}.X`).join(" AND ") || "FALSE";
		const prior = t.before.map((b) => ` AND NOT ${tn(g.transitions[idx.get(b)!], idx.get(b)!)}`).join("");
		out.push(`${tn(t, i)} := ${src} AND (${t.condition.trim() || "FALSE"})${prior};`);
	});
	out.push("(* the enabled transitions fire together: first leave, then enter *)");
	g.transitions.forEach((t, i) => {
		if (t.sources.length) out.push(`IF ${tn(t, i)} THEN ${t.sources.map((s) => `${s}.X := FALSE;`).join(" ")} END_IF;`);
	});
	g.transitions.forEach((t, i) => {
		if (t.targets.length) out.push(`IF ${tn(t, i)} THEN ${t.targets.map((s) => `${s}.X := TRUE;`).join(" ")} END_IF;`);
	});
	out.push("(* actions *)");
	const nVars = new Map<string, string[]>(); // N with a variable: TRUE while any of its steps is active
	for (const s of g.steps) {
		for (const a of s.actions) {
			const body = a.body.trim();
			if (!body) continue;
			const isVar = /^[A-Za-z_][A-Za-z0-9_.]*$/.test(body);
			switch (a.qualifier) {
				case "N":
					if (isVar) nVars.set(body, [...(nVars.get(body) ?? []), s.name]);
					else out.push(`IF ${s.name}.X THEN ${withSemicolon(body)} END_IF;`);
					break;
				case "S": out.push(`IF ${s.name}.X THEN ${isVar ? `${body} := TRUE;` : withSemicolon(body)} END_IF;`); break;
				case "R": out.push(`IF ${s.name}.X THEN ${isVar ? `${body} := FALSE;` : withSemicolon(body)} END_IF;`); break;
				case "P":
					out.push(`${s.name}_Entered(CLK := ${s.name}.X);`, `IF ${s.name}_Entered.Q THEN ${isVar ? `${body} := TRUE;` : withSemicolon(body)} END_IF;`);
					break;
			}
		}
	}
	for (const [v, steps] of nVars) out.push(`${v} := ${steps.map((s) => `${s}.X`).join(" OR ")};`);

	// the declaration unchanged: line n of it is line n of the text (diagnostics map back to it); the step
	// type SFC_STEP comes with the device sources (SFC_STEP_SOURCE), once for every SFC of the project
	const text = `${pou.decl}\nVAR (* SFC *)\n${vars.join("\n")}\nEND_VAR\n${out.join("\n")}\n${end}\n`;
	return { text, problems: g.problems };
}

const withSemicolon = (s: string) => (s.endsWith(";") ? s : `${s};`);

let counter = 0;
export function sfcId(): string {
	return `s${++counter}${Math.random().toString(36).slice(2, 6)}`;
}

/** Every item of the chart (depth first), with the sequence it is in and its index there. */
export function* sfcWalk(seq: SfcItem[]): Generator<{ item: SfcItem; seq: SfcItem[]; index: number }> {
	for (let i = 0; i < seq.length; i++) {
		const item = seq[i];
		yield { item, seq, index: i };
		if (item.kind === "branch") for (const b of item.branches) yield* sfcWalk(b);
	}
}

/** The example: an ejector station with an alternative and a parallel branch. */
export function exampleSfc(): SfcPou {
	const step = (name: string, actions: SfcAction[] = []): SfcItem => ({ kind: "step", id: sfcId(), name, actions });
	const trans = (condition: string): SfcItem => ({ kind: "trans", id: sfcId(), condition });
	return {
		name: "SFC_PRG",
		decl: "PROGRAM SFC_PRG\nVAR\n    Pieces : INT;\nEND_VAR",
		chart: [
			step("Start", [{ qualifier: "N", body: "ConveyorPwm := 600" }]),
			trans("Lightbarrier1"),
			step("Detect"),
			{
				kind: "branch", id: sfcId(), type: "alt", branches: [
					[trans("Lightbarrier2"), step("Eject", [{ qualifier: "N", body: "Valve1" }]), trans("Eject.T >= T#300ms")],
					[trans("Detect.T > T#2s"), step("Pass"), trans("TRUE")],
				],
			},
			step("Count", [{ qualifier: "P", body: "Pieces := Pieces + 1" }]),
			trans("TRUE"),
			{
				kind: "branch", id: sfcId(), type: "par", branches: [
					[step("Pump", [{ qualifier: "N", body: "VacuumPump" }]), trans("Pump.T > T#500ms"), step("PumpDone")],
					[step("Hold", [{ qualifier: "S", body: "Valve3" }])],
				],
			},
			trans("TRUE"),
			step("Release", [{ qualifier: "R", body: "Valve3" }]),
			trans("Release.T > T#200ms"),
			{ kind: "jump", id: sfcId(), target: "Start" },
		],
	};
}
