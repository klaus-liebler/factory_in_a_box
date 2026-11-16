// Inline SVG schematic of a pneumatic pressure control plant (compressor -> pressure vessel with manometer ->
// manifold -> 3x valve -> consumer/silencer), drawn with symbols based on ISO 1219 (circle + triangle for the
// compressor, capsule for the pressure vessel, circle with pointer for the manometer, 2/2-way poppet valve
// with solenoid + spring for the open/close valves, funnel + fins for a silencer/consumer outlet). Like
// roarm-3d-view.ts this module only knows geometry and state-to-display, no Modbus/register details - the
// caller (druckregelstrecke-app.ts) wires it to real values.
//
// Deliberately static SVG markup (template string, set once via innerHTML) instead of element by element via
// createElementNS as in the WebGL scene of roarm-3d-view.ts - SVG DOM is much less code per symbol, and
// everything here is flat anyway (no transform hierarchy needed). setState() afterwards only updates the few
// dynamic attributes (pointer rotation, spool displacement, text) via querySelector(). The compressor
// controls (on/off switch + power slider) sit as a <foreignObject> with real HTML form elements directly in
// the control panel symbol - so the controls are "inline" in the drawing as required, not in a separate HTML
// panel next to it (druckregelstrecke-app.ts therefore renders no compressor/valve control area of its own).
export type ValveIndex = 0 | 1 | 2;

export interface DruckregelstreckeState {
	/* Raw value of the analog pressure sensor (ADC1 CH10, 12 bit) - see the PRESSURE_RAW register; the physical
	   scaling is still open in the firmware (see src/io.cpp, inputs()), so it is deliberately shown only as raw
	   value/percent of full scale, no invented bar calibration. */
	pressureRaw: number;
	valveOpen: readonly [boolean, boolean, boolean];
	/** COMPRESSOR_PWM-Register, 0..1000 Promille -- 0 = aus. */
	compressorPwmPromille: number;
}

export interface DruckregelstreckeCallbacks {
	onValveClick?(index: ValveIndex): void;
	/* Control panel toggle switch - the caller decides which value "on" means (0 vs. last power), since it knows
	   the last power written. */
	onCompressorToggle?(): void;
	/* Power slider, reported only on "change" (release), see the comment at the registration further down - like
	   the sliderPreview pattern in register-panel.ts. */
	onCompressorPowerChange?(promille: number): void;
}

export interface DruckregelstreckeViewHandles {
	setState(state: DruckregelstreckeState): void;
	dispose(): void;
}

export const PRESSURE_RAW_FULL_SCALE = 4095;

const VALVE_X: readonly number[] = [500, 650, 800];
const HEADER_Y = 170;
const VALVE_MID_Y = 245;
const WINDOW_HALF = 15; // Sichtfenster des Ventilschiebers: 30x30
const WINDOW_SHIFT = WINDOW_HALF * 2; // Verschiebeweg Feder- <-> Magnetstellung
const ACTUATOR_SIZE = 16; // Feder-/Magnetsymbol, quadratisch
const CONSUMER_TOP_Y = 320;
const CONSUMER_BOTTOM_Y = 362;

const COLOR_VALVE_OPEN = "#16a34a";
const COLOR_VALVE_CLOSED = "#b91c1c";
const COLOR_ACCENT = "#2a78d6";
const COLOR_IDLE = "#94a3b8";
const COLOR_INK = "#334155";
const COLOR_PIPE = "#475569";

// Pointer deflection of a classic manometer: -120..+120 degrees around the 12 o'clock position, 240 degrees
// total scale (5 ticks at 0/25/50/75/100%).
const GAUGE_MIN_DEG = -120;
const GAUGE_MAX_DEG = 120;
const GAUGE_CX = 285;
const GAUGE_CY = 340;
const GAUGE_R = 46;

function gaugeTick(fraction: number): { x1: number; y1: number; x2: number; y2: number; lx: number; ly: number } {
	const deg = GAUGE_MIN_DEG + fraction * (GAUGE_MAX_DEG - GAUGE_MIN_DEG);
	const rad = ((deg - 90) * Math.PI) / 180;
	const outer = GAUGE_R - 4;
	const inner = GAUGE_R - 12;
	const labelR = GAUGE_R - 20;
	return {
		x1: GAUGE_CX + outer * Math.cos(rad),
		y1: GAUGE_CY + outer * Math.sin(rad),
		x2: GAUGE_CX + inner * Math.cos(rad),
		y2: GAUGE_CY + inner * Math.sin(rad),
		lx: GAUGE_CX + labelR * Math.cos(rad),
		ly: GAUGE_CY + labelR * Math.sin(rad),
	};
}

function gaugeTicksMarkup(): string {
	return [0, 0.25, 0.5, 0.75, 1].map((f) => {
		const t = gaugeTick(f);
		return `<line x1="${t.x1.toFixed(1)}" y1="${t.y1.toFixed(1)}" x2="${t.x2.toFixed(1)}" y2="${t.y2.toFixed(1)}" stroke="${COLOR_INK}" stroke-width="2" />
			<text x="${t.lx.toFixed(1)}" y="${t.ly.toFixed(1)}" font-size="8" fill="${COLOR_INK}" text-anchor="middle" dominant-baseline="middle">${Math.round(f * 100)}</text>`;
	}).join("\n");
}

// 2/2-way poppet valve, solenoid operated, closed at rest (NC) - DIN ISO 1219 symbols: two square switching
// position fields (spring/rest position = closed, solenoid/working position = open), of which, because of the
// fixed line connections, only ONE is ever in the "window" between the line stubs - just like a real spool
// valve whose spool moves inside the fixed housing. The click animation therefore moves exactly this spool
// element (see the ".valve-spool" transform in setState()) instead of only switching a fill color - the
// actual "displacement" is visible too. The spring is on the left (pushes the spool into the rest position =
// window shows "closed"), the solenoid on the right (moves the spool one window step to the left when
// energized, the open field slides into the window). Name and state labels deliberately sit to the side,
// OUTSIDE pipe, window and actuator symbols (instead of above/below as before), so that the manifold/branch
// line no longer covers them.
function valveMarkup(index: ValveIndex, x: number): string {
	const windowLeft = x - WINDOW_HALF;
	const windowTop = VALVE_MID_Y - WINDOW_HALF;
	const clipId = `valve-window-clip-${index}`;
	const springLeft = windowLeft - ACTUATOR_SIZE;
	const solenoidLeft = x + WINDOW_HALF;
	const actuatorTop = VALVE_MID_Y - ACTUATOR_SIZE / 2;

	// Closed field: two line stubs that do NOT meet in the middle, with a crossbar at their ends (T shape) - the
	// usual symbol for a blocked path.
	const blockedField = `
		<rect x="${windowLeft}" y="${windowTop}" width="${WINDOW_SHIFT}" height="${WINDOW_SHIFT}" fill="#f8fafc" stroke="${COLOR_INK}" stroke-width="2" />
		<line x1="${x}" y1="${windowTop}" x2="${x}" y2="${windowTop + 9}" stroke="${COLOR_VALVE_CLOSED}" stroke-width="2.5" />
		<line x1="${x - 5}" y1="${windowTop + 9}" x2="${x + 5}" y2="${windowTop + 9}" stroke="${COLOR_VALVE_CLOSED}" stroke-width="2.5" />
		<line x1="${x}" y1="${windowTop + WINDOW_SHIFT}" x2="${x}" y2="${windowTop + WINDOW_SHIFT - 9}" stroke="${COLOR_VALVE_CLOSED}" stroke-width="2.5" />
		<line x1="${x - 5}" y1="${windowTop + WINDOW_SHIFT - 9}" x2="${x + 5}" y2="${windowTop + WINDOW_SHIFT - 9}" stroke="${COLOR_VALVE_CLOSED}" stroke-width="2.5" />`;

	// Durchgangsfeld: durchgehende Linie mit Pfeilspitze (Durchflussrichtung).
	const openX = x + WINDOW_SHIFT;
	const openField = `
		<rect x="${windowLeft + WINDOW_SHIFT}" y="${windowTop}" width="${WINDOW_SHIFT}" height="${WINDOW_SHIFT}" fill="#f8fafc" stroke="${COLOR_INK}" stroke-width="2" />
		<line data-valve-flowline="${index}" x1="${openX}" y1="${windowTop}" x2="${openX}" y2="${windowTop + WINDOW_SHIFT}" stroke="${COLOR_VALVE_OPEN}" stroke-width="3" />
		<polygon points="${openX - 4},${VALVE_MID_Y + 2} ${openX + 4},${VALVE_MID_Y + 2} ${openX},${VALVE_MID_Y + 9}" fill="${COLOR_VALVE_OPEN}" />`;

	return `
		<g class="valve-group" data-valve-index="${index}" role="button" aria-label="Ventil ${index + 1}" tabindex="0">
			<rect x="${springLeft - 4}" y="${actuatorTop - 4}" width="${solenoidLeft + ACTUATOR_SIZE - springLeft + 8}" height="${ACTUATOR_SIZE + 8}" fill="transparent" />

			<!-- Anschlussbezeichnungen (DIN ISO 1219): 2 = Ausgang (oben), 1 = Eingang (unten). -->
			<text x="${x + 7}" y="${windowTop - 4}" font-size="9" fill="${COLOR_INK}" text-anchor="start">2</text>
			<text x="${x + 7}" y="${windowTop + WINDOW_SHIFT + 12}" font-size="9" fill="${COLOR_INK}" text-anchor="start">1</text>

			<!-- Feder (Ruhestellung) -->
			<polyline points="${springLeft},${VALVE_MID_Y} ${springLeft + 4},${VALVE_MID_Y - 6} ${springLeft + 8},${VALVE_MID_Y + 6} ${springLeft + 12},${VALVE_MID_Y - 6} ${springLeft + 16},${VALVE_MID_Y}"
				fill="none" stroke="${COLOR_INK}" stroke-width="1.5" />

			<!-- Sichtfenster: nur EIN Schaltfeld sichtbar, der Rest wird weggeclippt. -->
			<clipPath id="${clipId}"><rect x="${windowLeft}" y="${windowTop}" width="${WINDOW_SHIFT}" height="${WINDOW_SHIFT}" /></clipPath>
			<g clip-path="url(#${clipId})">
				<g class="valve-spool" data-valve-spool="${index}">
					${blockedField}
					${openField}
				</g>
			</g>

			<!-- Elektromagnet (Arbeitsstellung): Rechteck mit Diagonale, DIN-ISO-1219-Kurzzeichen fuer elektrische Betaetigung. -->
			<rect x="${solenoidLeft}" y="${actuatorTop}" width="${ACTUATOR_SIZE}" height="${ACTUATOR_SIZE}" fill="#f8fafc" stroke="${COLOR_INK}" stroke-width="1.5" />
			<line x1="${solenoidLeft}" y1="${actuatorTop + ACTUATOR_SIZE}" x2="${solenoidLeft + ACTUATOR_SIZE}" y2="${actuatorTop}" stroke="${COLOR_INK}" stroke-width="1.5" />

			<text x="${springLeft - 6}" y="${VALVE_MID_Y + 4}" font-size="12" font-weight="600" fill="${COLOR_INK}" text-anchor="end">V${index + 1}</text>
			<text class="valve-state-label" data-valve-label="${index}" x="${solenoidLeft + ACTUATOR_SIZE + 6}" y="${VALVE_MID_Y + 4}" font-size="11" font-weight="600" text-anchor="start"></text>
		</g>`;
}

// Consumer/silencer symbol: funnel (tip at the pipe) with fins at the open side - standard symbol for
// venting/throttling to atmosphere, fitting a control plant setup whose "consumers" are simply adjustable
// blow-offs/disturbances.
function consumerMarkup(index: ValveIndex, x: number): string {
	const halfW = 20;
	return `
		<g>
			<line x1="${x}" y1="${VALVE_MID_Y + WINDOW_HALF}" x2="${x}" y2="${CONSUMER_TOP_Y}" stroke="${COLOR_PIPE}" stroke-width="4" data-branch-pipe="${index}" />
			<polygon points="${x},${CONSUMER_TOP_Y} ${x - halfW},${CONSUMER_BOTTOM_Y} ${x + halfW},${CONSUMER_BOTTOM_Y}" fill="#f1f5f9" stroke="${COLOR_INK}" stroke-width="2" />
			<line x1="${x - 12}" y1="${CONSUMER_TOP_Y + 20}" x2="${x - 4}" y2="${CONSUMER_TOP_Y + 30}" stroke="${COLOR_INK}" stroke-width="1.5" />
			<line x1="${x}" y1="${CONSUMER_TOP_Y + 18}" x2="${x + 8}" y2="${CONSUMER_TOP_Y + 30}" stroke="${COLOR_INK}" stroke-width="1.5" />
			<line x1="${x + 12}" y1="${CONSUMER_TOP_Y + 16}" x2="${x + 18}" y2="${CONSUMER_TOP_Y + 30}" stroke="${COLOR_INK}" stroke-width="1.5" />
			<text x="${x}" y="${CONSUMER_BOTTOM_Y + 18}" font-size="11" fill="${COLOR_INK}" text-anchor="middle">Verbraucher ${index + 1}</text>
		</g>`;
}

// Control panel: indicator lamp (information only) + embedded HTML form (toggle switch + power slider) via
// <foreignObject> - so the compressor controls are part of the drawing itself instead of a separate panel
// next to it. Reuses the global .toggle-switch/.panel-slider classes from ui/components.css, which apply
// inside the foreignObject too thanks to light DOM (no shadow root, see createRenderRoot() in the *-app.ts
// files).
function panelMarkup(compressorCx: number): string {
	const panelX = compressorCx - 85;
	const panelY = 16;
	const panelW = 170;
	return `
		<g>
			<rect x="${panelX}" y="${panelY}" width="${panelW}" height="100" rx="8" fill="#f8fafc" stroke="${COLOR_INK}" stroke-width="2" />
			<text x="${compressorCx}" y="${panelY + 14}" font-size="11" fill="${COLOR_INK}" text-anchor="middle">Bedientableau</text>
			<circle id="panel-lamp" cx="${panelX + 18}" cy="${panelY + 32}" r="7" fill="${COLOR_IDLE}" stroke="${COLOR_INK}" stroke-width="1.5" />
			<text x="${panelX + 32}" y="${panelY + 36}" font-size="11" fill="${COLOR_INK}">Kompressor</text>
			<foreignObject x="${panelX + 12}" y="${panelY + 46}" width="${panelW - 24}" height="46">
				<div xmlns="http://www.w3.org/1999/xhtml" style="display:flex; flex-direction:column; gap:6px; font:11px 'Segoe UI', Roboto, Arial, sans-serif; color:${COLOR_INK};">
					<div style="display:flex; align-items:center; gap:8px;">
						<label class="toggle-switch">
							<input type="checkbox" id="compressor-toggle" />
							<span class="toggle-slider-track"></span>
						</label>
						<span id="compressor-toggle-label">Aus</span>
					</div>
					<div style="display:flex; align-items:center; gap:6px;">
						<input type="range" min="0" max="1000" value="0" id="compressor-slider" class="panel-slider" style="flex:1 1 auto; min-width:0;" />
						<span id="compressor-power-label" style="min-width:3.2em; text-align:right; font-variant-numeric:tabular-nums;">0 ‰</span>
					</div>
				</div>
			</foreignObject>
			<line x1="${compressorCx}" y1="${panelY + 100}" x2="${compressorCx}" y2="130" stroke="${COLOR_INK}" stroke-width="2" stroke-dasharray="4 3" />
		</g>`;
}

function svgMarkup(): string {
	const compressorCx = 95;
	const compressorCy = HEADER_Y;
	const compressorR = 40;
	const tankX = 230;
	const tankW = 110;
	const tankY = 60;
	const tankH = 220;
	const tankRight = tankX + tankW;

	const branches = VALVE_X.map((x, i) => {
		const idx = i as ValveIndex;
		return `
			<line x1="${x}" y1="${HEADER_Y}" x2="${x}" y2="${VALVE_MID_Y - WINDOW_HALF}" stroke="${COLOR_PIPE}" stroke-width="4" data-header-branch="${idx}" />
			${valveMarkup(idx, x)}
			${consumerMarkup(idx, x)}`;
	}).join("\n");

	return `
		<style>
			.pk-flow-active { stroke-dasharray: 8 7; animation: pk-flow-dash 0.7s linear infinite; }
			@keyframes pk-flow-dash { to { stroke-dashoffset: -15; } }
			.valve-group { cursor: pointer; }
			.valve-spool { transition: transform 0.25s ease; }
			.valve-group:focus-visible { outline: 2px solid ${COLOR_ACCENT}; outline-offset: 2px; }
		</style>

		${panelMarkup(compressorCx)}

		<!-- Kompressor-Symbol: Kreis mit Dreieck (Foerderrichtung), ISO-1219-aehnlich. -->
		<g>
			<circle cx="${compressorCx}" cy="${compressorCy}" r="${compressorR}" fill="#f8fafc" stroke="${COLOR_INK}" stroke-width="2.5" />
			<polygon id="compressor-triangle" points="${compressorCx - 14},${compressorCy - 18} ${compressorCx - 14},${compressorCy + 18} ${compressorCx + 22},${compressorCy}" fill="${COLOR_IDLE}" stroke="${COLOR_INK}" stroke-width="1.5" />
			<text x="${compressorCx}" y="${compressorCy + compressorR + 18}" font-size="12" font-weight="600" fill="${COLOR_INK}" text-anchor="middle">Kompressor</text>
			<line x1="${compressorCx + compressorR}" y1="${compressorCy}" x2="${tankX}" y2="${HEADER_Y}" stroke="${COLOR_PIPE}" stroke-width="4" data-compressor-pipe="1" />
		</g>

		<!-- Druckspeicher/Tank-Symbol: stehende Kapsel (rx=Breite/2 -> halbrunde Kappen). -->
		<g>
			<rect x="${tankX}" y="${tankY}" width="${tankW}" height="${tankH}" rx="${tankW / 2}" ry="${tankW / 2}" fill="#dbeafe" stroke="${COLOR_INK}" stroke-width="2.5" />
			<text x="${tankX + tankW / 2}" y="${tankY - 10}" font-size="12" font-weight="600" fill="${COLOR_INK}" text-anchor="middle">Druckspeicher</text>
		</g>

		<!-- Manometer (Druck-Messinstrument), am Tankboden angeschlagen. -->
		<g>
			<line x1="${tankX + tankW / 2}" y1="${tankY + tankH}" x2="${GAUGE_CX}" y2="${GAUGE_CY - GAUGE_R}" stroke="${COLOR_INK}" stroke-width="2.5" />
			<circle cx="${GAUGE_CX}" cy="${GAUGE_CY}" r="${GAUGE_R}" fill="#ffffff" stroke="${COLOR_INK}" stroke-width="2.5" />
			${gaugeTicksMarkup()}
			<line id="gauge-needle" x1="${GAUGE_CX}" y1="${GAUGE_CY}" x2="${GAUGE_CX}" y2="${GAUGE_CY - GAUGE_R + 10}" stroke="#dc2626" stroke-width="2.5" stroke-linecap="round" transform="rotate(${GAUGE_MIN_DEG} ${GAUGE_CX} ${GAUGE_CY})" />
			<circle cx="${GAUGE_CX}" cy="${GAUGE_CY}" r="4" fill="${COLOR_INK}" />
			<text x="${GAUGE_CX}" y="${GAUGE_CY + GAUGE_R + 20}" font-size="12" fill="${COLOR_INK}" text-anchor="middle">Manometer</text>
			<text id="pressure-readout" x="${GAUGE_CX}" y="${GAUGE_CY + GAUGE_R + 36}" font-size="12" font-weight="700" fill="${COLOR_INK}" text-anchor="middle">– / ${PRESSURE_RAW_FULL_SCALE}</text>
		</g>

		<!-- Sammelleitung + 3x Ventil + Verbraucher. -->
		<g>
			<line x1="${tankRight}" y1="${HEADER_Y}" x2="${VALVE_X[VALVE_X.length - 1]}" y2="${HEADER_Y}" stroke="${COLOR_PIPE}" stroke-width="4" data-header-pipe="1" />
			${branches}
		</g>
	`;
}

export function createDruckregelstreckeView(container: HTMLElement, callbacks: DruckregelstreckeCallbacks = {}): DruckregelstreckeViewHandles {
	const svgNs = "http://www.w3.org/2000/svg";
	const svg = document.createElementNS(svgNs, "svg");
	svg.setAttribute("viewBox", "0 0 900 440");
	svg.setAttribute("width", "100%");
	svg.setAttribute("height", "100%");
	svg.innerHTML = svgMarkup();
	container.appendChild(svg);

	const needle = svg.querySelector<SVGLineElement>("#gauge-needle")!;
	const pressureReadout = svg.querySelector<SVGTextElement>("#pressure-readout")!;
	const panelLamp = svg.querySelector<SVGCircleElement>("#panel-lamp")!;
	const compressorTriangle = svg.querySelector<SVGPolygonElement>("#compressor-triangle")!;
	const compressorToggle = svg.querySelector<HTMLInputElement>("#compressor-toggle")!;
	const compressorToggleLabel = svg.querySelector<HTMLSpanElement>("#compressor-toggle-label")!;
	const compressorSlider = svg.querySelector<HTMLInputElement>("#compressor-slider")!;
	const compressorPowerLabel = svg.querySelector<HTMLSpanElement>("#compressor-power-label")!;
	const flowPipes = Array.from(svg.querySelectorAll<SVGElement>("[data-compressor-pipe], [data-header-pipe]"));
	const branchPipes: SVGElement[][] = [0, 1, 2].map((i) => [
		...Array.from(svg.querySelectorAll<SVGElement>(`[data-header-branch="${i}"]`)),
		...Array.from(svg.querySelectorAll<SVGElement>(`[data-branch-pipe="${i}"]`)),
		...Array.from(svg.querySelectorAll<SVGElement>(`[data-valve-flowline="${i}"]`)),
	]);
	const valveSpools = [0, 1, 2].map((i) => svg.querySelector<SVGGElement>(`[data-valve-spool="${i}"]`)!);
	const valveLabels = [0, 1, 2].map((i) => svg.querySelector<SVGTextElement>(`[data-valve-label="${i}"]`)!);
	const valveGroups = Array.from(svg.querySelectorAll<SVGGElement>(".valve-group"));

	const listeners: Array<{ el: EventTarget; type: string; fn: EventListenerOrEventListenerObject }> = [];
	function on(el: EventTarget, type: string, fn: EventListenerOrEventListenerObject): void {
		el.addEventListener(type, fn);
		listeners.push({ el, type, fn });
	}

	valveGroups.forEach((group) => {
		const index = Number(group.dataset.valveIndex) as ValveIndex;
		on(group, "click", () => callbacks.onValveClick?.(index));
		on(group, "keydown", (ev: Event) => {
			const key = (ev as KeyboardEvent).key;
			if (key === "Enter" || key === " ") {
				ev.preventDefault();
				callbacks.onValveClick?.(index);
			}
		});
	});

	on(compressorToggle, "change", () => callbacks.onCompressorToggle?.());

	// While dragging, only update the local display (no register write per pixel of mouse movement);
	// "sliderDragging" also keeps the next poll tick (see druckregelstrecke-app.ts, setState() about every 1s)
	// from making the slider jump back to the last confirmed server value while dragging - like the
	// sliderPreview pattern in register-panel.ts, only kept locally here instead of in a lit @state.
	let sliderDragging = false;
	on(compressorSlider, "pointerdown", () => {
		sliderDragging = true;
	});
	on(compressorSlider, "input", () => {
		compressorPowerLabel.textContent = `${compressorSlider.value} ‰`;
	});
	on(compressorSlider, "change", () => {
		sliderDragging = false;
		callbacks.onCompressorPowerChange?.(Number(compressorSlider.value));
	});

	return {
		setState(state: DruckregelstreckeState): void {
			const fraction = Math.min(1, Math.max(0, state.pressureRaw / PRESSURE_RAW_FULL_SCALE));
			const deg = GAUGE_MIN_DEG + fraction * (GAUGE_MAX_DEG - GAUGE_MIN_DEG);
			needle.setAttribute("transform", `rotate(${deg} ${GAUGE_CX} ${GAUGE_CY})`);
			pressureReadout.textContent = `${Math.round(state.pressureRaw)} / ${PRESSURE_RAW_FULL_SCALE}`;

			const running = state.compressorPwmPromille > 0;
			const pwm = Math.min(1000, Math.max(0, state.compressorPwmPromille));
			panelLamp.setAttribute("fill", running ? "#22c55e" : COLOR_IDLE);
			compressorTriangle.setAttribute("fill", running ? COLOR_ACCENT : COLOR_IDLE);
			compressorToggle.checked = running;
			compressorToggleLabel.textContent = running ? "Ein" : "Aus";
			if (!sliderDragging) {
				compressorSlider.value = String(pwm);
				compressorPowerLabel.textContent = `${pwm} ‰`;
			}
			for (const pipe of flowPipes) pipe.classList.toggle("pk-flow-active", running);

			state.valveOpen.forEach((open, i) => {
				valveSpools[i].setAttribute("transform", open ? `translate(${-WINDOW_SHIFT}, 0)` : "translate(0, 0)");
				valveLabels[i].textContent = open ? "AUF" : "ZU";
				valveLabels[i].setAttribute("fill", open ? COLOR_VALVE_OPEN : COLOR_VALVE_CLOSED);
				for (const pipe of branchPipes[i]) pipe.classList.toggle("pk-flow-active", running && open);
			});
		},
		dispose(): void {
			for (const { el, type, fn } of listeners) el.removeEventListener(type, fn);
			container.removeChild(svg);
		},
	};
}
