// Orthogonal routing of the wires of one FBD network (docs/plc-iec61131.md 5.3). Every wire leaves its
// output to the right and enters its input from the left. The router keeps wires out of boxes they do
// not belong to and gives every vertical segment and every horizontal track its own line: wires from
// different outputs never run on top of each other (wires from the same output may share, they carry
// the same value). Shorter wires are routed first and get the direct ways. DOM-free.

export interface Pt {
	x: number;
	y: number;
}

export interface Rect {
	x: number;
	y: number;
	w: number;
	h: number;
}

export interface WireEnds {
	/** The output pin and the input pin. */
	a: Pt;
	b: Pt;
	/** The nodes of the two ends (their boxes are no obstacles for this wire). */
	from: string;
	to: string;
	/** The source (node.pin): wires of one source may share segments. */
	source: string;
}

/** Clearance around boxes; distance between parallel segments of different sources. */
const CLEAR = 6;
const PITCH = 7;

interface Seg {
	src: string;
	fixed: number;  // x of a vertical, y of a horizontal segment
	lo: number;
	hi: number;
}

const span = (p: number, q: number): [number, number] => (p < q ? [p, q] : [q, p]);

function crossesBox(vertical: boolean, fixed: number, from: number, to: number, r: Rect): boolean {
	const [lo, hi] = span(from, to);
	if (vertical) return fixed > r.x - CLEAR && fixed < r.x + r.w + CLEAR && hi > r.y - CLEAR && lo < r.y + r.h + CLEAR;
	return fixed > r.y - CLEAR && fixed < r.y + r.h + CLEAR && hi > r.x - CLEAR && lo < r.x + r.w + CLEAR;
}

export function routeWires(boxes: ReadonlyMap<string, Rect>, wires: readonly WireEnds[]): Pt[][] {
	const vUsed: Seg[] = [];
	const hUsed: Seg[] = [];
	const out: Pt[][] = new Array(wires.length);

	const order = wires.map((w, i) => i).sort((i, j) => Math.abs(wires[i].b.x - wires[i].a.x) - Math.abs(wires[j].b.x - wires[j].a.x) ||
		Math.abs(wires[i].b.y - wires[i].a.y) - Math.abs(wires[j].b.y - wires[j].a.y));

	for (const i of order) {
		const w = wires[i];
		const obstacles = [...boxes].filter(([id]) => id !== w.from && id !== w.to).map(([, r]) => r);
		// a segment is free: no foreign box, no segment of another source on the same line
		const free = (vertical: boolean, fixed: number, from: number, to: number): boolean => {
			if (obstacles.some((r) => crossesBox(vertical, fixed, from, to, r))) return false;
			const [lo, hi] = span(from, to);
			return !(vertical ? vUsed : hUsed).some((s) => s.src !== w.source && Math.abs(s.fixed - fixed) < PITCH - 1 && hi > s.lo - 2 && lo < s.hi + 2);
		};
		const take = (pts: Pt[]) => {
			for (let k = 0; k + 1 < pts.length; k++) {
				const p = pts[k], q = pts[k + 1];
				if (p.x === q.x && p.y !== q.y) vUsed.push({ src: w.source, fixed: p.x, ...lohi(p.y, q.y) });
				else if (p.y === q.y && p.x !== q.x) hUsed.push({ src: w.source, fixed: p.y, ...lohi(p.x, q.x) });
			}
			out[i] = pts;
		};
		const { a, b } = w;
		// 1. one vertical: out at a.y, down/up at x, in at b.y. Preferred: the middle of the free gap before
		// the target (the right edge would block the way of wires that enter the target at the height this
		// one leaves its source), then alternately around it.
		if (b.x - a.x >= 2 * CLEAR + 4) {
			const [ylo, yhi] = span(a.y, b.y);
			const wall = Math.max(a.x, ...obstacles.filter((r) => r.x + r.w < b.x && r.x + r.w > a.x && r.y < yhi + CLEAR && r.y + r.h > ylo - CLEAR)
				.map((r) => r.x + r.w + CLEAR));
			const mid = Math.round((wall + b.x) / 2);
			const xs: number[] = [];
			for (let k = 0; ; k++) {
				const l = mid - k * PITCH, r = mid + k * PITCH;
				const lOk = l >= a.x + 8, rOk = r <= b.x - 8;
				if (!lOk && !rOk) break;
				if (rOk) xs.push(r);
				if (lOk && k > 0) xs.push(l);
			}
			let done = false;
			for (const x of xs) {
				if (done) break;
				if (free(false, a.y, a.x, x) && free(true, x, a.y, b.y) && free(false, b.y, x, b.x)) {
					take(a.y === b.y ? [a, b] : [a, { x, y: a.y }, { x, y: b.y }, b]);
					done = true;
				}
			}
			if (done) continue;
		}
		// 2. around: out to x1, along a free track y, to x2 left of the target, in at b.y
		const left = Math.min(a.x, b.x) - 40, right = Math.max(a.x, b.x) + 40;
		const inRange = obstacles.filter((r) => r.x + r.w > left && r.x < right);
		const top = Math.min(a.y, b.y, ...inRange.map((r) => r.y)) - 12;
		const bottom = Math.max(a.y, b.y, ...inRange.map((r) => r.y + r.h)) + 12;
		const tracks: number[] = [];
		for (let y = Math.min(a.y, b.y); y <= Math.max(a.y, b.y); y += PITCH) tracks.push(y);  // between the ends first
		for (let k = 0; k < 24; k++) tracks.push(bottom + k * PITCH, top - k * PITCH);
		tracks.sort((p, q) => Math.abs(p - a.y) + Math.abs(p - b.y) - (Math.abs(q - a.y) + Math.abs(q - b.y)));
		let routed: Pt[] | null = null;
		for (const y of tracks) {
			if (routed) break;
			for (let x1 = a.x + 10; x1 <= a.x + 10 + 8 * PITCH && !routed; x1 += PITCH) {
				if (!free(false, a.y, a.x, x1) || !free(true, x1, a.y, y)) continue;
				for (let x2 = b.x - 10; x2 >= b.x - 10 - 8 * PITCH; x2 -= PITCH) {
					if (free(false, y, x1, x2) && free(true, x2, y, b.y) && free(false, b.y, x2, b.x)) {
						routed = [a, { x: x1, y: a.y }, { x: x1, y }, { x: x2, y }, { x: x2, y: b.y }, b];
						break;
					}
				}
			}
		}
		// 3. nothing free (a crowded network): the plain way, overlaps accepted
		take(routed ?? [a, { x: a.x + 12, y: a.y }, { x: a.x + 12, y: bottom }, { x: b.x - 12, y: bottom }, { x: b.x - 12, y: b.y }, b]);
	}
	return out;
}

function lohi(p: number, q: number): { lo: number; hi: number } {
	const [lo, hi] = span(p, q);
	return { lo, hi };
}

/** The SVG path of a route. */
export function pathOf(pts: readonly Pt[]): string {
	return pts.map((p, k) => `${k === 0 ? "M" : "L"} ${p.x} ${p.y}`).join(" ");
}
