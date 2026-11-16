// Hand-rolled canvas line chart renderer - deliberately no chart library (Chart.js or similar), to keep the
// single-file bundle small that is compiled completely into the firmware flash (see
// web/scripts/embed-into-firmware.mjs). Only one series per call (one chart = one axis) - several quantities
// (voltage/current/power) are drawn as several separate charts side by side instead of a multi-axis chart,
// see power-management-app.ts.
import { COLORS } from "./colors.ts";

export interface LineChartPoint {
	t: number;
	v: number;
}

export interface LineChartOptions {
	color: string;
	unit: string;
	decimals?: number;
	/** The x axis labels (default: t is a wall-clock time in ms, shown as a time of day). */
	formatTime?: (t: number) => string;
	/** The last value as a dot and a number at the top right (default true: a live series). */
	showLast?: boolean;
}

const MUTED_INK = COLORS.muted;
const GRID_LINE = COLORS.grid;
const PRIMARY_INK = COLORS.text;
const FONT = "system-ui, -apple-system, 'Segoe UI', sans-serif";

export function drawLineChart(canvas: HTMLCanvasElement, points: readonly LineChartPoint[], opts: LineChartOptions): void {
	const ctx = canvas.getContext("2d");
	if (!ctx) return;

	const cssWidth = canvas.clientWidth;
	const cssHeight = canvas.clientHeight;
	if (cssWidth === 0 || cssHeight === 0) return;

	const dpr = window.devicePixelRatio || 1;
	const wantWidth = Math.round(cssWidth * dpr);
	const wantHeight = Math.round(cssHeight * dpr);
	if (canvas.width !== wantWidth || canvas.height !== wantHeight) {
		canvas.width = wantWidth;
		canvas.height = wantHeight;
	}

	ctx.save();
	ctx.scale(dpr, dpr);
	ctx.clearRect(0, 0, cssWidth, cssHeight);

	const decimals = opts.decimals ?? 1;
	const paddingLeft = 44;
	const paddingRight = 8;
	const paddingTop = 20;
	const paddingBottom = 18;
	const plotWidth = Math.max(1, cssWidth - paddingLeft - paddingRight);
	const plotHeight = Math.max(1, cssHeight - paddingTop - paddingBottom);

	if (points.length < 2) {
		ctx.fillStyle = MUTED_INK;
		ctx.font = `12px ${FONT}`;
		ctx.textAlign = "left";
		ctx.textBaseline = "middle";
		ctx.fillText("Sammle Messwerte…", paddingLeft, paddingTop + plotHeight / 2);
		ctx.restore();
		return;
	}

	let min = points[0].v;
	let max = points[0].v;
	for (const p of points) {
		if (p.v < min) min = p.v;
		if (p.v > max) max = p.v;
	}
	if (min === max) {
		min -= 1;
		max += 1;
	} else {
		const pad = (max - min) * 0.1;
		min -= pad;
		max += pad;
	}

	const t0 = points[0].t;
	const t1 = points[points.length - 1].t;
	const tSpan = Math.max(1, t1 - t0);
	const xFor = (t: number) => paddingLeft + ((t - t0) / tSpan) * plotWidth;
	const yFor = (v: number) => paddingTop + (1 - (v - min) / (max - min)) * plotHeight;

	ctx.strokeStyle = GRID_LINE;
	ctx.lineWidth = 1;
	ctx.fillStyle = MUTED_INK;
	ctx.font = `11px ${FONT}`;
	ctx.textAlign = "right";
	ctx.textBaseline = "middle";
	for (const v of [min, (min + max) / 2, max]) {
		const y = yFor(v);
		ctx.beginPath();
		ctx.moveTo(paddingLeft, y);
		ctx.lineTo(paddingLeft + plotWidth, y);
		ctx.stroke();
		ctx.fillText(v.toFixed(decimals), paddingLeft - 6, y);
	}

	// x axis: baseline + three time labels (start/middle/end of the visible window), like the three y axis
	// labels above.
	const axisY = paddingTop + plotHeight;
	ctx.strokeStyle = GRID_LINE;
	ctx.beginPath();
	ctx.moveTo(paddingLeft, axisY);
	ctx.lineTo(paddingLeft + plotWidth, axisY);
	ctx.stroke();

	ctx.fillStyle = MUTED_INK;
	ctx.font = `11px ${FONT}`;
	ctx.textBaseline = "top";
	const labelY = axisY + 4;
	const formatTime = opts.formatTime ?? ((t: number) => new Date(t).toLocaleTimeString("de-DE"));

	ctx.textAlign = "left";
	ctx.fillText(formatTime(t0), paddingLeft, labelY);

	ctx.textAlign = "right";
	ctx.fillText(formatTime(t1), paddingLeft + plotWidth, labelY);

	if (tSpan > 0) {
		const tMid = t0 + tSpan / 2;
		ctx.textAlign = "center";
		ctx.fillText(formatTime(tMid), paddingLeft + plotWidth / 2, labelY);
	}

	ctx.strokeStyle = opts.color;
	ctx.lineWidth = 2;
	ctx.lineJoin = "round";
	ctx.lineCap = "round";
	ctx.beginPath();
	points.forEach((p, i) => {
		const x = xFor(p.t);
		const y = yFor(p.v);
		if (i === 0) ctx.moveTo(x, y);
		else ctx.lineTo(x, y);
	});
	ctx.stroke();

	if (opts.showLast === false) {
		ctx.restore();
		return;
	}
	const last = points[points.length - 1];
	ctx.fillStyle = opts.color;
	ctx.beginPath();
	ctx.arc(xFor(last.t), yFor(last.v), 3.5, 0, Math.PI * 2);
	ctx.fill();

	ctx.fillStyle = PRIMARY_INK;
	ctx.font = `bold 13px ${FONT}`;
	ctx.textAlign = "right";
	ctx.textBaseline = "alphabetic";
	ctx.fillText(`${last.v.toFixed(decimals)} ${opts.unit}`, paddingLeft + plotWidth, paddingTop - 6);

	ctx.restore();
}
