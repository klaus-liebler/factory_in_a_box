// Zwei handgerollte Canvas-Diagramme fuer die Druckregelstrecke-Modi (s.
// docs/druckregelstrecke-modes.md) -- bewusst eigenstaendig statt line-chart.ts
// wiederzuverwenden: line-chart.ts zeichnet genau EINE zeitbasierte Serie mit einer Y-Achse
// (s. dortiger Kommentar), hier werden aber (a) zwei Serien mit UNTERSCHIEDLICHEN Skalen auf
// gemeinsamer Zeitachse (Trend: Druck 0..4095 links, Kompressorleistung 0..1000 rechts) bzw.
// (b) mehrere XY-Kurven ohne Zeitachse (Kennlinien) gebraucht.
const MUTED_INK = "#898781";
const GRID_LINE = "#e1e0d9";
const FONT = "system-ui, -apple-system, 'Segoe UI', sans-serif";
const PRESSURE_COLOR = "#2a78d6";
const COMPRESSOR_COLOR = "#e08a1e";

function resizeForDpr(canvas: HTMLCanvasElement): CanvasRenderingContext2D | null {
	const ctx = canvas.getContext("2d");
	if (!ctx) return null;
	const cssWidth = canvas.clientWidth;
	const cssHeight = canvas.clientHeight;
	if (cssWidth === 0 || cssHeight === 0) return null;
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
	return ctx;
}

export interface TrendSample {
	t: number;
	pressureRaw: number;
	compressorPermille: number;
}

/** Zwei Linien (Druck links, Kompressorleistung rechts) ueber ein festes Zeitfenster
 * (durationSec, endend bei "nowMs") -- fuer "Freies Experiment", "Sprungantwort" und
 * "Reglerbetrieb" (s. docs/druckregelstrecke-modes.md, jeweils "Liniendiagramm ... der letzten
 * 60 Sekunden"). Optional eine dritte, gestrichelte Linie fuer den Sollwert (Reglerbetrieb). */
export function drawTrendChart(
	canvas: HTMLCanvasElement,
	samples: readonly TrendSample[],
	nowMs: number,
	durationSec: number,
	setpointRaw?: number,
): void {
	const ctx = resizeForDpr(canvas);
	if (!ctx) return;
	const cssWidth = canvas.clientWidth;
	const cssHeight = canvas.clientHeight;

	const paddingLeft = 44;
	const paddingRight = 48;
	const paddingTop = 12;
	const paddingBottom = 18;
	const plotWidth = Math.max(1, cssWidth - paddingLeft - paddingRight);
	const plotHeight = Math.max(1, cssHeight - paddingTop - paddingBottom);

	const t0 = nowMs - durationSec * 1000;
	const xFor = (t: number) => paddingLeft + ((t - t0) / (durationSec * 1000)) * plotWidth;
	const yForPressure = (raw: number) => paddingTop + (1 - raw / 4095) * plotHeight;
	const yForCompressor = (permille: number) => paddingTop + (1 - permille / 1000) * plotHeight;

	ctx.strokeStyle = GRID_LINE;
	ctx.lineWidth = 1;
	ctx.fillStyle = MUTED_INK;
	ctx.font = `11px ${FONT}`;
	ctx.textBaseline = "middle";
	for (const f of [0, 0.5, 1]) {
		const y = paddingTop + (1 - f) * plotHeight;
		ctx.beginPath();
		ctx.moveTo(paddingLeft, y);
		ctx.lineTo(paddingLeft + plotWidth, y);
		ctx.stroke();
		ctx.textAlign = "right";
		ctx.fillStyle = PRESSURE_COLOR;
		ctx.fillText(String(Math.round(f * 4095)), paddingLeft - 6, y);
		ctx.textAlign = "left";
		ctx.fillStyle = COMPRESSOR_COLOR;
		ctx.fillText(String(Math.round(f * 1000)), paddingLeft + plotWidth + 6, y);
	}

	const visible = samples.filter((s) => s.t >= t0 && s.t <= nowMs);
	if (visible.length < 2) {
		ctx.fillStyle = MUTED_INK;
		ctx.font = `12px ${FONT}`;
		ctx.textAlign = "center";
		ctx.fillText("Sammle Messwerte…", paddingLeft + plotWidth / 2, paddingTop + plotHeight / 2);
		ctx.restore();
		return;
	}

	function drawLine(color: string, yFor: (v: number) => number, valueOf: (s: TrendSample) => number): void {
		ctx!.strokeStyle = color;
		ctx!.lineWidth = 2;
		ctx!.lineJoin = "round";
		ctx!.lineCap = "round";
		ctx!.beginPath();
		visible.forEach((s, i) => {
			const x = xFor(s.t);
			const y = yFor(valueOf(s));
			if (i === 0) ctx!.moveTo(x, y);
			else ctx!.lineTo(x, y);
		});
		ctx!.stroke();
	}

	drawLine(COMPRESSOR_COLOR, yForCompressor, (s) => s.compressorPermille);
	drawLine(PRESSURE_COLOR, yForPressure, (s) => s.pressureRaw);

	if (setpointRaw !== undefined) {
		ctx.strokeStyle = PRESSURE_COLOR;
		ctx.globalAlpha = 0.5;
		ctx.setLineDash([5, 4]);
		ctx.lineWidth = 1.5;
		const y = yForPressure(setpointRaw);
		ctx.beginPath();
		ctx.moveTo(paddingLeft, y);
		ctx.lineTo(paddingLeft + plotWidth, y);
		ctx.stroke();
		ctx.setLineDash([]);
		ctx.globalAlpha = 1;
	}

	ctx.fillStyle = PRESSURE_COLOR;
	ctx.font = `bold 11px ${FONT}`;
	ctx.textAlign = "left";
	ctx.textBaseline = "alphabetic";
	ctx.fillText("Druck (Rohwert)", paddingLeft, paddingTop + 10);
	ctx.fillStyle = COMPRESSOR_COLOR;
	ctx.textAlign = "right";
	ctx.fillText("Kompressor (‰)", paddingLeft + plotWidth, paddingTop + 10);

	ctx.restore();
}

export interface CharacteristicCurve {
	label: string;
	color: string;
	points: readonly { compressorPermille: number; maxPressureRaw: number }[];
}

/** Kennfelddiagramm: X = Kompressorleistung (0..1000), Y = max. erreichter Druck (0..4095), eine
 * Kurve je Ventilkombination (s. docs/druckregelstrecke-modes.md, Abschnitt "Kennlinie"). */
export function drawCharacteristicChart(canvas: HTMLCanvasElement, curves: readonly CharacteristicCurve[]): void {
	const ctx = resizeForDpr(canvas);
	if (!ctx) return;
	const cssWidth = canvas.clientWidth;
	const cssHeight = canvas.clientHeight;

	const paddingLeft = 48;
	const paddingRight = 12;
	const paddingTop = 12;
	const paddingBottom = 28;
	const plotWidth = Math.max(1, cssWidth - paddingLeft - paddingRight);
	const plotHeight = Math.max(1, cssHeight - paddingTop - paddingBottom);

	const xFor = (permille: number) => paddingLeft + (permille / 1000) * plotWidth;
	const yFor = (raw: number) => paddingTop + (1 - raw / 4095) * plotHeight;

	ctx.strokeStyle = GRID_LINE;
	ctx.lineWidth = 1;
	ctx.fillStyle = MUTED_INK;
	ctx.font = `11px ${FONT}`;
	ctx.textBaseline = "middle";
	for (const f of [0, 0.25, 0.5, 0.75, 1]) {
		const y = paddingTop + (1 - f) * plotHeight;
		ctx.beginPath();
		ctx.moveTo(paddingLeft, y);
		ctx.lineTo(paddingLeft + plotWidth, y);
		ctx.stroke();
		ctx.textAlign = "right";
		ctx.fillText(String(Math.round(f * 4095)), paddingLeft - 6, y);
	}
	ctx.textBaseline = "top";
	ctx.textAlign = "center";
	for (const f of [0, 0.25, 0.5, 0.75, 1]) {
		const x = paddingLeft + f * plotWidth;
		ctx.fillText(String(Math.round(f * 1000)), x, paddingTop + plotHeight + 6);
	}
	ctx.fillText("Kompressorleistung (‰) →", paddingLeft + plotWidth / 2, paddingTop + plotHeight + 18);

	if (curves.every((c) => c.points.length === 0)) {
		ctx.fillStyle = MUTED_INK;
		ctx.font = `12px ${FONT}`;
		ctx.textAlign = "center";
		ctx.textBaseline = "middle";
		ctx.fillText("Noch keine Punkte übernommen…", paddingLeft + plotWidth / 2, paddingTop + plotHeight / 2);
		ctx.restore();
		return;
	}

	for (const curve of curves) {
		if (curve.points.length === 0) continue;
		const sorted = [...curve.points].sort((a, b) => a.compressorPermille - b.compressorPermille);
		ctx.strokeStyle = curve.color;
		ctx.fillStyle = curve.color;
		ctx.lineWidth = 2;
		ctx.lineJoin = "round";
		ctx.beginPath();
		sorted.forEach((p, i) => {
			const x = xFor(p.compressorPermille);
			const y = yFor(p.maxPressureRaw);
			if (i === 0) ctx.moveTo(x, y);
			else ctx.lineTo(x, y);
		});
		if (sorted.length >= 2) ctx.stroke();
		for (const p of sorted) {
			ctx.beginPath();
			ctx.arc(xFor(p.compressorPermille), yFor(p.maxPressureRaw), 3.5, 0, Math.PI * 2);
			ctx.fill();
		}
	}

	ctx.font = `11px ${FONT}`;
	ctx.textAlign = "left";
	ctx.textBaseline = "top";
	let legendY = paddingTop;
	for (const curve of curves) {
		if (curve.points.length === 0) continue;
		ctx.fillStyle = curve.color;
		ctx.fillRect(paddingLeft + plotWidth - 110, legendY, 10, 10);
		ctx.fillStyle = MUTED_INK;
		ctx.fillText(curve.label, paddingLeft + plotWidth - 96, legendY);
		legendY += 14;
	}

	ctx.restore();
}
