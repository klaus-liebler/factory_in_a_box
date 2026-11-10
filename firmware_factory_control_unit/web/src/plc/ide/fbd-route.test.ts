// The FBD wire router (fbd-route.ts): wires around foreign boxes, no two sources on the same line.
import { describe, expect, it } from "vitest";
import { routeWires, type Pt, type Rect, type WireEnds } from "./fbd-route.ts";

/** Every problem of a routing: a segment through a box that is not an end of its wire, two segments of
 *  different sources on top of each other, a non-orthogonal step, wrong ends. */
function problems(boxes: Map<string, Rect>, wires: WireEnds[], routes: Pt[][]): string[] {
	const out: string[] = [];
	const segs: { src: string; v: boolean; fixed: number; lo: number; hi: number; wire: number }[] = [];
	routes.forEach((pts, i) => {
		const w = wires[i];
		if (pts[0].x !== w.a.x || pts[0].y !== w.a.y || pts.at(-1)!.x !== w.b.x || pts.at(-1)!.y !== w.b.y) out.push(`wire ${i}: wrong ends`);
		for (let k = 0; k + 1 < pts.length; k++) {
			const p = pts[k], q = pts[k + 1];
			if (p.x !== q.x && p.y !== q.y) out.push(`wire ${i}: diagonal`);
			const v = p.x === q.x;
			const fixed = v ? p.x : p.y, lo = Math.min(v ? p.y : p.x, v ? q.y : q.x), hi = Math.max(v ? p.y : p.x, v ? q.y : q.x);
			if (lo === hi) continue;
			for (const [id, r] of boxes) {
				if (id === w.from || id === w.to) continue;
				const inside = v ? fixed > r.x && fixed < r.x + r.w && hi > r.y && lo < r.y + r.h
					: fixed > r.y && fixed < r.y + r.h && hi > r.x && lo < r.x + r.w;
				if (inside) out.push(`wire ${i}: through box ${id}`);
			}
			segs.push({ src: w.source, v, fixed, lo, hi, wire: i });
		}
	});
	for (const s of segs) {
		for (const t of segs) {
			if (s.wire >= t.wire || s.src === t.src || s.v !== t.v) continue;
			if (s.fixed === t.fixed && Math.min(s.hi, t.hi) > Math.max(s.lo, t.lo)) out.push(`wires ${s.wire} and ${t.wire} overlap`);
		}
	}
	return out;
}

const box = (x: number, y: number, h = 60): Rect => ({ x, y, w: 120, h });

/** Crossings of wires of different sources: a vertical segment of one through a horizontal of the other. */
function crossings(wires: WireEnds[], routes: Pt[][]): string[] {
	const out: string[] = [];
	const segs = routes.flatMap((pts, i) => pts.slice(0, -1).map((p, k) => ({ i, p, q: pts[k + 1] })));
	for (const v of segs.filter((s) => s.p.x === s.q.x && s.p.y !== s.q.y)) {
		for (const h of segs.filter((s) => s.p.y === s.q.y && s.p.x !== s.q.x)) {
			if (v.i === h.i || wires[v.i].source === wires[h.i].source) continue;
			const [y0, y1] = [Math.min(v.p.y, v.q.y), Math.max(v.p.y, v.q.y)], [x0, x1] = [Math.min(h.p.x, h.q.x), Math.max(h.p.x, h.q.x)];
			if (v.p.x > x0 && v.p.x < x1 && h.p.y > y0 && h.p.y < y1) out.push(`wire ${v.i} crosses wire ${h.i}`);
		}
	}
	return out;
}

describe("FBD wire routing", () => {
	it("a wire from a left box to a box far right goes around the box between them", () => {
		const boxes = new Map([["L", box(20, 20)], ["M", box(220, 20)], ["R", box(420, 20)]]);
		const wires: WireEnds[] = [
			{ a: { x: 140, y: 52 }, b: { x: 420, y: 52 }, from: "L", to: "R", source: "L.Q" },  // straight through M without routing
			{ a: { x: 140, y: 52 }, b: { x: 220, y: 52 }, from: "L", to: "M", source: "L.Q" },
			{ a: { x: 340, y: 52 }, b: { x: 420, y: 72 }, from: "M", to: "R", source: "M.Q" },
		];
		const routes = routeWires(boxes, wires);
		expect(problems(boxes, wires, routes)).toEqual([]);
		expect(routes[1]).toEqual([{ x: 140, y: 52 }, { x: 220, y: 52 }]);  // the direct neighbour: straight
	});

	it("many wires into one box get their own verticals; wires of one output may share", () => {
		const boxes = new Map([["A", box(20, 20)], ["B", box(20, 120)], ["C", box(20, 220)], ["T", box(300, 100, 100)]]);
		const wires: WireEnds[] = [
			{ a: { x: 140, y: 52 }, b: { x: 300, y: 132 }, from: "A", to: "T", source: "A.Q" },
			{ a: { x: 140, y: 152 }, b: { x: 300, y: 152 }, from: "B", to: "T", source: "B.Q" },
			{ a: { x: 140, y: 252 }, b: { x: 300, y: 172 }, from: "C", to: "T", source: "C.Q" },
			{ a: { x: 140, y: 52 }, b: { x: 300, y: 192 }, from: "A", to: "T", source: "A.Q" },
		];
		const routes = routeWires(boxes, wires);
		expect(problems(boxes, wires, routes)).toEqual([]);
	});

	it("the bend is close to the target, not in the middle; a straight wire has none", () => {
		const boxes = new Map([["A", box(20, 20)], ["B", box(520, 100)]]);
		const wires: WireEnds[] = [{ a: { x: 140, y: 52 }, b: { x: 520, y: 132 }, from: "A", to: "B", source: "A.Q" }];
		const [r] = routeWires(boxes, wires);
		expect(r).toHaveLength(4);
		expect(r[1].x).toBe(r[2].x);
		expect(520 - r[1].x).toBeLessThanOrEqual(20);
		const [flat] = routeWires(boxes, [{ ...wires[0], b: { x: 520, y: 52 } }]);
		expect(flat).toEqual([{ x: 140, y: 52 }, { x: 520, y: 52 }]);
	});

	// Regression: the example network "Förderband" - the constants 0 and 600 below the SEL box, wired to
	// its pins IN0 and IN1. Their verticals were in the wrong order: the wire of 600 crossed the wire of
	// 0 right before the box (and looked connected to it).
	it("wires into one box from below do not cross each other (lanes): the constants at SEL", () => {
		const boxes = new Map<string, Rect>([
			["and", { x: 300, y: 20, w: 120, h: 68 }], ["zero", { x: 300, y: 110, w: 60, h: 24 }], ["six", { x: 300, y: 140, w: 60, h: 24 }],
			["sel", { x: 450, y: 30, w: 120, h: 88 }],
		]);
		const wires: WireEnds[] = [
			{ a: { x: 420, y: 54 }, b: { x: 450, y: 64 }, from: "and", to: "sel", source: "and.OUT" },
			{ a: { x: 360, y: 122 }, b: { x: 450, y: 84 }, from: "zero", to: "sel", source: "zero.OUT" },
			{ a: { x: 360, y: 152 }, b: { x: 450, y: 104 }, from: "six", to: "sel", source: "six.OUT" },
		];
		const routes = routeWires(boxes, wires);
		expect(problems(boxes, wires, routes)).toEqual([]);
		expect(crossings(wires, routes)).toEqual([]);
		// the same from above: three sources above their pins
		const above: WireEnds[] = [
			{ a: { x: 200, y: 20 }, b: { x: 450, y: 64 }, from: "p", to: "sel", source: "p.OUT" },
			{ a: { x: 200, y: 30 }, b: { x: 450, y: 84 }, from: "q", to: "sel", source: "q.OUT" },
			{ a: { x: 200, y: 40 }, b: { x: 450, y: 104 }, from: "r", to: "sel", source: "r.OUT" },
		];
		const only = new Map<string, Rect>([["sel", boxes.get("sel")!]]);
		const up = routeWires(only, above);
		expect(problems(only, above, up)).toEqual([]);
		expect(crossings(above, up)).toEqual([]);
	});

	it("a feedback (the input left of the output) goes around below, out of the boxes", () => {
		const boxes = new Map([["A", box(20, 20)], ["B", box(220, 20)]]);
		const wires: WireEnds[] = [
			{ a: { x: 140, y: 52 }, b: { x: 220, y: 52 }, from: "A", to: "B", source: "A.Q" },
			{ a: { x: 340, y: 52 }, b: { x: 20, y: 72 }, from: "B", to: "A", source: "B.Q" },
		];
		const routes = routeWires(boxes, wires);
		expect(problems(boxes, wires, routes)).toEqual([]);
	});

	it("a grid of boxes with crossing connections: no wire through a box, no overlap", () => {
		const boxes = new Map<string, Rect>();
		for (let c = 0; c < 4; c++) for (let r = 0; r < 3; r++) boxes.set(`${c}${r}`, box(20 + c * 200, 20 + r * 100));
		const wires: WireEnds[] = [];
		for (let r = 0; r < 3; r++) {
			for (let t = 0; t < 3; t++) {
				// column 0 row r -> column 3 row t (crosses columns 1 and 2), and column 1 -> column 2
				wires.push({ a: { x: 140, y: 40 + r * 100 }, b: { x: 620, y: 35 + t * 100 + r * 10 }, from: `0${r}`, to: `3${t}`, source: `0${r}.Q` });
				wires.push({ a: { x: 340, y: 60 + r * 100 }, b: { x: 420, y: 65 + t * 100 - r * 10 + 20 }, from: `1${r}`, to: `2${t}`, source: `1${r}.Q` });
			}
		}
		const routes = routeWires(boxes, wires);
		expect(problems(boxes, wires, routes)).toEqual([]);
	});
});
