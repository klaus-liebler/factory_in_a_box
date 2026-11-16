// The line recorders' history: one buffer per device and trend element, sampled on the device's updates
// (at most every 100 ms) for every trend of every page - also of pages that are not shown, so switching
// pages keeps the curves. The history lives in this browser tab only.
import type { PlcDevice } from "../ide/device.ts";
import { MAX_PENS, toNumber, TrendBuffer, type HmiElement, type HmiPage } from "./hmi.ts";

const buffers = new WeakMap<PlcDevice, Map<string, TrendBuffer>>();
const lastSample = new WeakMap<PlcDevice, number>();
const SAMPLE_MS = 100;

/** The buffer of a trend element on a device (created empty). */
export function trendOf(dev: PlcDevice, pageId: string, e: HmiElement): TrendBuffer {
	let m = buffers.get(dev);
	if (!m) buffers.set(dev, (m = new Map()));
	const key = `${pageId}/${e.id}/${(e.pens ?? []).map((p) => p.var).join("|")}`;
	let b = m.get(key);
	if (!b) m.set(key, (b = new TrendBuffer(MAX_PENS)));
	return b;
}

/** Samples every trend of the pages (call it on each device update; it limits itself to 10 Hz). */
export function sampleTrends(dev: PlcDevice, pages: readonly HmiPage[], now = Date.now()): void {
	if (now - (lastSample.get(dev) ?? 0) < SAMPLE_MS) return;
	lastSample.set(dev, now);
	if (!dev.debug || dev.state === "NO_PROGRAM") return;
	for (const p of pages) {
		for (const e of p.elements) {
			if (e.kind !== "trend" || !e.pens?.length) continue;
			const values = e.pens.map((pen) => (pen.var.trim() ? toNumber(dev.read(pen.var.trim()).value) : null));
			trendOf(dev, p.id, e).push(now, values, Math.max(1, e.span ?? 60) * 1000);
		}
	}
}

/** Forgets a device's history (a new program was loaded). */
export function clearTrends(dev: PlcDevice): void {
	buffers.delete(dev);
}
