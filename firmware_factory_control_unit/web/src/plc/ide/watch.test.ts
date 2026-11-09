// Watch subscriptions (docs/plc-iec61131.md 10.3): the variables the IDE shows become few ranges of the
// data segment within the device's limits (128 ranges, 1024 bytes).
import { describe, expect, it } from "vitest";
import { WATCH_MAX_BYTES, WATCH_MAX_RANGES, watchRanges } from "./device.ts";

describe("watch ranges", () => {
	it("merges overlapping and close variables, keeps distant ones apart", () => {
		const r = watchRanges(new Map([[100, 4], [102, 2], [108, 1], [200, 8], [0, 0]]));
		expect(r).toMatchObject({ addresses: [100, 200], lengths: [9, 8] });
		expect(r!.key).toBe("100:9,200:8");
	});

	it("nothing watched: no subscription", () => {
		expect(watchRanges(new Map())).toBeNull();
		expect(watchRanges(new Map([[5, 0]]))).toBeNull();
	});

	it("stays within the device's limits: ranges and bytes", () => {
		const many = new Map<number, number>();
		for (let i = 0; i < 300; i++) many.set(i * 100, 2);
		const r = watchRanges(many)!;
		expect(r.addresses.length).toBe(WATCH_MAX_RANGES);
		const big = new Map<number, number>([[0, 600], [1000, 600]]);
		const b = watchRanges(big)!;
		expect(b.lengths.reduce((s, n) => s + n, 0)).toBeLessThanOrEqual(WATCH_MAX_BYTES);
		expect(b.addresses).toEqual([0]);
		// a merge never makes one range larger than the byte limit
		const chain = new Map<number, number>();
		for (let i = 0; i < 200; i++) chain.set(i * 8, 8);
		const c = watchRanges(chain)!;
		expect(Math.max(...c.lengths)).toBeLessThanOrEqual(WATCH_MAX_BYTES);
		expect(c.lengths.reduce((s, n) => s + n, 0)).toBeLessThanOrEqual(WATCH_MAX_BYTES);
	});

	it("the same variables give the same key (no new subscription without a change)", () => {
		const a = watchRanges(new Map([[10, 2], [40, 4]]))!;
		const b = watchRanges(new Map([[40, 4], [10, 2]]))!;
		expect(a.key).toBe(b.key);
	});
});
