// Small charts as lit templates (SVG, scale with their box, no drawing after the render): a sparkline and a
// ring gauge. The time-series chart with axes is ui/line-chart.ts (canvas). Charts with a special purpose stay
// with their workspace (Druckregelstrecke: trend with set point and characteristic; HMI: line recorder).
import { html, svg, type TemplateResult } from "lit";
import { COLORS } from "./colors.ts";
import "./charts.css";

export interface SparklineOptions {
	color?: string;
	/** A fixed scale (e.g. 0..1000 per mille CPU): without it the curve is scaled to its own min/max, which
	 *  makes 5 % and 80 % look the same - give it whenever the quantity has a natural range. */
	min?: number;
	max?: number;
	/** The number of samples the full width stands for (fewer: the curve grows from the right). */
	capacity?: number;
	/** Fill the area below the curve. */
	area?: boolean;
}

let gradientId = 0;

/** A sparkline of the values (oldest first) over the full width of its box. */
export function sparkline(values: readonly number[], opts: SparklineOptions = {}): TemplateResult {
	if (values.length < 2) return html`<div class="ui-spark ui-spark-empty"></div>`;
	const color = opts.color ?? COLORS.accent;
	const cap = Math.max(opts.capacity ?? values.length, values.length);
	const lo = opts.min ?? Math.min(...values), hi = opts.max ?? Math.max(...values);
	const span = hi - lo || 1;
	const x = (i: number) => ((cap - values.length + i) / (cap - 1)) * 100;
	const y = (v: number) => 29 - (Math.min(hi, Math.max(lo, v)) - lo) / span * 28;
	const pts = values.map((v, i) => `${x(i).toFixed(2)},${y(v).toFixed(2)}`).join(" ");
	const id = `ui-spark-${gradientId++}`;
	return html`<svg class="ui-spark" viewBox="0 0 100 30" preserveAspectRatio="none" aria-hidden="true">
		${opts.area === false ? svg`` : svg`<defs><linearGradient id=${id} x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color=${color} stop-opacity=".35"/><stop offset="1" stop-color=${color} stop-opacity="0"/></linearGradient></defs>
		<polygon points="${x(0).toFixed(2)},30 ${pts} 100,30" fill="url(#${id})"/>`}
		${svg`<polyline points=${pts} fill="none" stroke=${color} stroke-width="1.6" vector-effect="non-scaling-stroke"/>`}
	</svg>`;
}

/** A ring gauge: the fraction 0..1 as an arc, the text in the middle. */
export function ring(fraction: number, color: string, text: string, sub = ""): TemplateResult {
	const r = 34, c = 2 * Math.PI * r, v = Math.max(0, Math.min(1, fraction));
	return html`<svg class="ui-ring" viewBox="0 0 84 84" aria-label=${text}>
		${svg`<circle cx="42" cy="42" r=${r} fill="none" stroke=${COLORS.track} stroke-width="8"/>
		<circle cx="42" cy="42" r=${r} fill="none" stroke=${color} stroke-width="8" stroke-linecap="round"
			stroke-dasharray="${(c * v).toFixed(1)} ${c.toFixed(1)}" transform="rotate(-90 42 42)"/>
		<text x="42" y="44" text-anchor="middle" class="ui-ring-value">${text}</text>
		<text x="42" y="58" text-anchor="middle" class="ui-ring-sub">${sub}</text>`}
	</svg>`;
}
