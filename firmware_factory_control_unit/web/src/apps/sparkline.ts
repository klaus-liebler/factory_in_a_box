// Minimalistischer Sparkline-Renderer (nur Pfad + Endpunkt, keine Achsen/Labels/Gitternetz) fuer
// kompakte Inline-Verlaufsanzeigen in Tabellenzellen -- s. drawLineChart() (line-chart.ts) fuer
// die "grosse" Variante mit Achsenbeschriftung; deren Achsen-Paddings (44px links) waeren fuer
// eine ~56x18px-Tabellenzelle unbrauchbar.
export interface SparklineOptions {
	color?: string;
	// Fester oberer Skalenwert (z.B. 1000 fuer 0..100.0% CPU-Last in Permille) -- bewusst KEINE
	// automatische Min/Max-Anpassung an die sichtbaren Werte, sonst wuerde sich die Skala bei
	// jedem neuen Sample "unter" der Kurve verschieben und die Linie taeuschend gleich hoch wirken
	// lassen, egal ob 5% oder 80% Last.
	max: number;
}

const DEFAULT_COLOR = "#2a78d6";

export function drawSparkline(canvas: HTMLCanvasElement, values: readonly number[], opts: SparklineOptions): void {
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

	if (values.length < 2) {
		ctx.restore();
		return;
	}

	const pad = 2;
	const plotWidth = Math.max(1, cssWidth - pad * 2);
	const plotHeight = Math.max(1, cssHeight - pad * 2);
	const xFor = (i: number) => pad + (i / (values.length - 1)) * plotWidth;
	const yFor = (v: number) => pad + (1 - Math.min(v, opts.max) / opts.max) * plotHeight;

	const color = opts.color ?? DEFAULT_COLOR;
	ctx.strokeStyle = color;
	ctx.lineWidth = 1.5;
	ctx.lineJoin = "round";
	ctx.lineCap = "round";
	ctx.beginPath();
	values.forEach((v, i) => {
		const x = xFor(i);
		const y = yFor(v);
		if (i === 0) ctx.moveTo(x, y);
		else ctx.lineTo(x, y);
	});
	ctx.stroke();

	ctx.fillStyle = color;
	ctx.beginPath();
	ctx.arc(xFor(values.length - 1), yFor(values[values.length - 1]), 1.8, 0, Math.PI * 2);
	ctx.fill();

	ctx.restore();
}
