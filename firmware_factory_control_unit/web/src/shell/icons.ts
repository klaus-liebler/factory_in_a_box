// The icons of the ribbon, the workspace switcher and the dashboard: two-tone SVG (32 x 32), a coloured
// body with a darker outline, in the style of desktop engineering tools. `icon(name, size)` renders one.
import { html, svg, type SVGTemplateResult, type TemplateResult } from "lit";

const B = "#3b82f6", BD = "#1d4ed8", BL = "#bfdbfe";
const O = "#f59e0b", OD = "#b45309";
const G = "#22c55e", GD = "#15803d";
const R = "#ef4444", RD = "#b91c1c";
const Y = "#facc15";
const P = "#a855f7", PD = "#7e22ce";
const T = "#14b8a6", TD = "#0f766e";
const S = "#94a3b8", SD = "#475569", SL = "#e2e8f0";
const K = "#334155";

const ICONS = {
	dashboard: svg`<rect x="3" y="4" width="12" height="10" rx="2" fill=${B} stroke=${BD}/><rect x="17" y="4" width="12" height="16" rx="2" fill=${T} stroke=${TD}/><rect x="3" y="16" width="12" height="12" rx="2" fill=${O} stroke=${OD}/><rect x="17" y="22" width="12" height="6" rx="2" fill=${G} stroke=${GD}/>`,
	registers: svg`<rect x="4" y="4" width="24" height="24" rx="2" fill=${SL} stroke=${SD}/><path d="M4 12h24M4 20h24M13 4v24" stroke=${SD}/><rect x="15" y="14" width="11" height="4" fill=${B}/>`,
	power: svg`<path d="M18 3 7 18h8l-2 11 12-16h-8z" fill=${Y} stroke=${OD} stroke-linejoin="round"/>`,
	chip: svg`<rect x="8" y="8" width="16" height="16" rx="2" fill=${SD} stroke=${K}/><rect x="12" y="12" width="8" height="8" fill=${B}/><path d="M12 4v4M16 4v4M20 4v4M12 24v4M16 24v4M20 24v4M4 12h4M4 16h4M4 20h4M24 12h4M24 16h4M24 20h4" stroke=${SD} stroke-width="2"/>`,
	robot: svg`<rect x="5" y="24" width="14" height="4" rx="1" fill=${SD}/><path d="M12 24V15l9-7 5 4" fill="none" stroke=${O} stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/><circle cx="12" cy="15" r="2.5" fill=${K}/><circle cx="21" cy="8" r="2.5" fill=${K}/><path d="M26 12l3 3M26 12l-1 4" stroke=${K} stroke-width="2" stroke-linecap="round"/>`,
	tasks: svg`<rect x="4" y="5" width="24" height="22" rx="2" fill="#fff" stroke=${SD}/><rect x="7" y="9" width="12" height="3" fill=${B}/><rect x="7" y="15" width="17" height="3" fill=${G}/><rect x="7" y="21" width="8" height="3" fill=${O}/>`,
	pressure: svg`<circle cx="16" cy="17" r="11" fill="#fff" stroke=${SD} stroke-width="2"/><path d="M16 17l6-6" stroke=${R} stroke-width="2.5" stroke-linecap="round"/><circle cx="16" cy="17" r="2" fill=${K}/><rect x="13" y="2" width="6" height="4" fill=${SD}/>`,
	stepper: svg`<rect x="5" y="9" width="16" height="16" rx="2" fill=${S} stroke=${SD}/><circle cx="13" cy="17" r="4" fill=${SL} stroke=${SD}/><rect x="21" y="15" width="8" height="4" fill=${O} stroke=${OD}/>`,
	plc: svg`<rect x="4" y="6" width="24" height="20" rx="2" fill=${G} stroke=${GD}/><rect x="7" y="10" width="7" height="12" fill="#fff" opacity=".85"/><path d="M17 11h8M17 15h8M17 19h5" stroke="#fff" stroke-width="2"/>`,
	hmi: svg`<rect x="3" y="5" width="26" height="18" rx="2" fill=${K}/><rect x="5" y="7" width="22" height="14" fill=${BL}/><circle cx="11" cy="14" r="4" fill=${B}/><rect x="18" y="10" width="7" height="2" fill=${G}/><rect x="18" y="15" width="5" height="2" fill=${O}/><path d="M12 23l-2 5h12l-2-5" fill=${SD}/>`,
	terminal: svg`<rect x="3" y="5" width="26" height="22" rx="2" fill=${K}/><path d="M8 12l4 4-4 4" fill="none" stroke=${G} stroke-width="2"/><path d="M15 21h8" stroke="#fff" stroke-width="2"/>`,
	refresh: svg`<path d="M25 11a10 10 0 1 0 1 9" fill="none" stroke=${B} stroke-width="3" stroke-linecap="round"/><path d="M27 4v8h-8z" fill=${BD}/>`,
	build: svg`<circle cx="16" cy="16" r="7" fill=${S} stroke=${SD} stroke-width="2"/><path d="M16 3v5M16 24v5M3 16h5M24 16h5M7 7l3.5 3.5M21.5 21.5 25 25M25 7l-3.5 3.5M10.5 21.5 7 25" stroke=${SD} stroke-width="3"/><circle cx="16" cy="16" r="3" fill="#fff"/>`,
	download: svg`<rect x="5" y="22" width="22" height="6" rx="1" fill=${S} stroke=${SD}/><path d="M16 3v14" stroke=${B} stroke-width="4"/><path d="M9 13l7 8 7-8z" fill=${BD}/>`,
	"download-run": svg`<rect x="3" y="22" width="16" height="6" rx="1" fill=${S} stroke=${SD}/><path d="M11 3v13" stroke=${B} stroke-width="4"/><path d="M5 12l6 7 6-7z" fill=${BD}/><path d="M20 12v14l10-7z" fill=${G} stroke=${GD}/>`,
	run: svg`<path d="M9 5v22l18-11z" fill=${G} stroke=${GD} stroke-linejoin="round"/>`,
	stop: svg`<rect x="7" y="7" width="18" height="18" rx="2" fill=${R} stroke=${RD}/>`,
	cycle: svg`<path d="M6 6v20l12-10z" fill=${G} stroke=${GD}/><rect x="21" y="6" width="5" height="20" fill=${SD}/>`,
	reset: svg`<path d="M8 11a10 10 0 1 1-1 9" fill="none" stroke=${O} stroke-width="3" stroke-linecap="round"/><path d="M5 4v8h8z" fill=${OD}/>`,
	continue: svg`<rect x="5" y="6" width="5" height="20" fill=${O}/><path d="M13 6v20l15-10z" fill=${G} stroke=${GD}/>`,
	"step-over": svg`<path d="M5 18c0-9 15-11 20-3" fill="none" stroke=${B} stroke-width="3"/><path d="M28 9l-1 9-8-3z" fill=${BD}/><circle cx="16" cy="25" r="3" fill=${O}/>`,
	"step-into": svg`<path d="M16 3v16" stroke=${B} stroke-width="3"/><path d="M10 15l6 7 6-7z" fill=${BD}/><circle cx="16" cy="27" r="3" fill=${O}/>`,
	"step-out": svg`<path d="M16 25V9" stroke=${B} stroke-width="3"/><path d="M10 13l6-8 6 8z" fill=${BD}/><circle cx="16" cy="28" r="2.5" fill=${O}/>`,
	breakpoint: svg`<circle cx="16" cy="16" r="10" fill=${R} stroke=${RD} stroke-width="2"/><path d="M11 21l10-10" stroke="#fff" stroke-width="3"/>`,
	force: svg`<path d="M11 4h10l-2 10h5L13 29l2-11h-5z" fill=${P} stroke=${PD} stroke-linejoin="round"/>`,
	export: svg`<path d="M7 3h12l6 6v20H7z" fill="#fff" stroke=${SD}/><path d="M16 26V14" stroke=${G} stroke-width="3"/><path d="M11 18l5-6 5 6z" fill=${GD}/>`,
	import: svg`<path d="M7 3h12l6 6v20H7z" fill="#fff" stroke=${SD}/><path d="M16 11v12" stroke=${B} stroke-width="3"/><path d="M11 19l5 6 5-6z" fill=${BD}/>`,
	"from-device": svg`<rect x="3" y="4" width="16" height="12" rx="2" fill=${G} stroke=${GD}/><path d="M9 3h12l6 6v20H9z" fill="#fff" stroke=${SD} transform="translate(2 0)"/><path d="M19 12v11" stroke=${B} stroke-width="3"/><path d="M14 19l5 6 5-6z" fill=${BD}/>`,
	example: svg`<path d="M6 5h20v22H6z" fill=${Y} stroke=${OD}/><path d="M10 11h12M10 16h12M10 21h8" stroke=${OD} stroke-width="2"/>`,
	"new-object": svg`<path d="M6 3h12l6 6v20H6z" fill="#fff" stroke=${SD}/><circle cx="23" cy="23" r="7" fill=${G}/><path d="M23 19v8M19 23h8" stroke="#fff" stroke-width="2.5"/>`,
	"new-folder": svg`<path d="M3 8h10l3 3h13v16H3z" fill=${Y} stroke=${OD}/><circle cx="23" cy="22" r="6" fill=${G}/><path d="M23 19v6M20 22h6" stroke="#fff" stroke-width="2.5"/>`,
	"new-page": svg`<rect x="3" y="5" width="22" height="16" rx="2" fill=${BL} stroke=${BD}/><circle cx="23" cy="23" r="7" fill=${G}/><path d="M23 19v8M19 23h8" stroke="#fff" stroke-width="2.5"/>`,
	simulator: svg`<rect x="3" y="5" width="26" height="18" rx="2" fill=${P} stroke=${PD}/><path d="M7 17l4-6 4 4 4-7 5 9" fill="none" stroke="#fff" stroke-width="2"/><path d="M10 27h12" stroke=${SD} stroke-width="3"/>`,
	device: svg`<rect x="4" y="6" width="24" height="20" rx="3" fill=${G} stroke=${GD}/><circle cx="10" cy="12" r="2" fill=${Y}/><circle cx="16" cy="12" r="2" fill="#fff"/><path d="M8 19h16" stroke="#fff" stroke-width="2"/><path d="M8 26v3M24 26v3" stroke=${SD} stroke-width="3"/>`,
	fieldbus: svg`<path d="M4 16h24" stroke=${SD} stroke-width="3"/><rect x="5" y="5" width="7" height="7" fill=${B} stroke=${BD}/><rect x="20" y="5" width="7" height="7" fill=${B} stroke=${BD}/><rect x="12.5" y="20" width="7" height="7" fill=${B} stroke=${BD}/><path d="M8.5 12v4M23.5 12v4M16 16v4" stroke=${SD} stroke-width="2"/>`,
	iec: svg`<rect x="4" y="6" width="24" height="20" rx="2" fill=${G} stroke=${GD}/><text x="16" y="21" font-size="11" font-weight="700" text-anchor="middle" fill="#fff" font-family="sans-serif">IEC</text>`,
	autostart: svg`<circle cx="16" cy="17" r="11" fill="none" stroke=${G} stroke-width="3"/><path d="M16 3v12" stroke=${GD} stroke-width="3" stroke-linecap="round"/>`,
	unforce: svg`<path d="M11 4h10l-2 10h5L13 29l2-11h-5z" fill=${S} stroke=${SD}/><path d="M5 27 27 5" stroke=${R} stroke-width="3"/>`,
	messages: svg`<rect x="3" y="5" width="26" height="22" rx="2" fill="#fff" stroke=${SD}/><circle cx="8" cy="11" r="2" fill=${R}/><circle cx="8" cy="17" r="2" fill=${O}/><circle cx="8" cy="23" r="2" fill=${B}/><path d="M12 11h13M12 17h13M12 23h10" stroke=${SD} stroke-width="2"/>`,
	watch: svg`<path d="M2 16s5-9 14-9 14 9 14 9-5 9-14 9S2 16 2 16z" fill=${BL} stroke=${BD}/><circle cx="16" cy="16" r="5" fill=${B}/><circle cx="16" cy="16" r="2" fill=${K}/>`,
	log: svg`<path d="M7 3h18v26H7z" fill="#fff" stroke=${SD}/><path d="M10 9h12M10 14h12M10 19h12M10 24h8" stroke=${S} stroke-width="2"/>`,
	plant: svg`<rect x="3" y="20" width="26" height="6" rx="3" fill=${SD}/><circle cx="7" cy="23" r="2" fill=${SL}/><circle cx="25" cy="23" r="2" fill=${SL}/><rect x="9" y="11" width="7" height="8" fill=${O} stroke=${OD}/><rect x="18" y="13" width="6" height="6" fill=${B} stroke=${BD}/>`,
	stack: svg`<rect x="6" y="5" width="20" height="6" fill=${B} stroke=${BD}/><rect x="6" y="13" width="20" height="6" fill=${BL} stroke=${BD}/><rect x="6" y="21" width="20" height="6" fill=${BL} stroke=${BD}/>`,
	xref: svg`<circle cx="13" cy="13" r="8" fill=${BL} stroke=${BD} stroke-width="2"/><path d="M19 19l8 8" stroke=${K} stroke-width="4" stroke-linecap="round"/>`,
	fullscreen: svg`<path d="M4 12V4h8M28 12V4h-8M4 20v8h8M28 20v8h-8" fill="none" stroke=${K} stroke-width="3"/><rect x="10" y="10" width="12" height="12" fill=${BL} stroke=${BD}/>`,
	theme: svg`<circle cx="16" cy="16" r="11" fill=${Y} stroke=${OD}/><path d="M16 5a11 11 0 0 1 0 22z" fill=${K}/>`,
	collapse: svg`<rect x="3" y="5" width="26" height="22" rx="2" fill="#fff" stroke=${SD}/><rect x="3" y="5" width="26" height="7" fill=${BL} stroke=${SD}/><path d="M11 20l5-5 5 5" fill="none" stroke=${BD} stroke-width="2.5"/>`,
	ack: svg`<circle cx="16" cy="16" r="12" fill=${G} stroke=${GD}/><path d="M10 16l4 4 8-9" fill="none" stroke="#fff" stroke-width="3"/>`,
	alarm: svg`<path d="M16 3 29 27H3z" fill=${Y} stroke=${OD} stroke-linejoin="round"/><path d="M16 11v8" stroke=${K} stroke-width="3"/><circle cx="16" cy="23" r="1.8" fill=${K}/>`,
	page: svg`<rect x="3" y="5" width="26" height="20" rx="2" fill=${BL} stroke=${BD}/><rect x="6" y="9" width="8" height="6" fill=${B}/><rect x="17" y="9" width="9" height="3" fill=${G}/><rect x="17" y="14" width="6" height="3" fill=${O}/><rect x="6" y="18" width="20" height="4" fill="#fff"/>`,
	clear: svg`<path d="M8 9h16l-2 19H10z" fill=${SL} stroke=${SD}/><path d="M5 9h22M13 5h6" stroke=${SD} stroke-width="2.5"/><path d="M13 13v11M19 13v11" stroke=${SD} stroke-width="2"/>`,
	save: svg`<path d="M5 4h18l4 4v20H5z" fill=${B} stroke=${BD}/><rect x="9" y="4" width="12" height="8" fill="#fff"/><rect x="9" y="18" width="14" height="10" fill=${BL}/>`,
	open: svg`<path d="M3 8h10l3 3h13v16H3z" fill=${Y} stroke=${OD}/><path d="M3 27l5-11h23l-4 11z" fill="#fde68a" stroke=${OD}/>`,
	add: svg`<circle cx="16" cy="16" r="12" fill=${G} stroke=${GD}/><path d="M16 10v12M10 16h12" stroke="#fff" stroke-width="3"/>`,
	pose: svg`<path d="M8 26V16l8-6 6 5" fill="none" stroke=${O} stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/><circle cx="24" cy="24" r="6" fill=${G}/><path d="M24 21v6M21 24h6" stroke="#fff" stroke-width="2.5"/>`,
	vacuum: svg`<path d="M16 3v12" stroke=${SD} stroke-width="4"/><path d="M7 17h18l-3 8H10z" fill=${T} stroke=${TD}/><path d="M11 28h10" stroke=${TD} stroke-width="2"/>`,
	"vacuum-off": svg`<path d="M16 3v12" stroke=${SD} stroke-width="4"/><path d="M7 17h18l-3 8H10z" fill=${S} stroke=${SD}/><path d="M5 29 27 9" stroke=${R} stroke-width="3"/>`,
	delay: svg`<circle cx="16" cy="17" r="11" fill="#fff" stroke=${SD} stroke-width="2"/><path d="M16 10v7l5 3" fill="none" stroke=${B} stroke-width="2.5" stroke-linecap="round"/><rect x="13" y="2" width="6" height="3" fill=${SD}/>`,
	home: svg`<path d="M4 16 16 5l12 11" fill="none" stroke=${BD} stroke-width="3" stroke-linejoin="round"/><path d="M8 14v13h16V14" fill=${BL} stroke=${BD}/><rect x="13" y="19" width="6" height="8" fill=${B}/>`,
	left: svg`<path d="M27 16H9" stroke=${B} stroke-width="4"/><path d="M13 8 4 16l9 8z" fill=${BD}/>`,
	right: svg`<path d="M5 16h18" stroke=${B} stroke-width="4"/><path d="M19 8l9 8-9 8z" fill=${BD}/>`,
	apply: svg`<rect x="4" y="4" width="24" height="24" rx="3" fill=${BL} stroke=${BD}/><path d="M9 16l5 5 9-10" fill="none" stroke=${GD} stroke-width="3"/>`,
	mode: svg`<rect x="3" y="7" width="12" height="18" rx="2" fill=${B} stroke=${BD}/><rect x="17" y="7" width="12" height="18" rx="2" fill="#fff" stroke=${SD}/><path d="M20 12h6M20 16h6M20 20h4" stroke=${S} stroke-width="2"/>`,
	chart: svg`<rect x="3" y="4" width="26" height="24" rx="2" fill="#fff" stroke=${SD}/><path d="M6 22l6-7 5 4 9-10" fill="none" stroke=${B} stroke-width="2.5"/><path d="M6 25h20" stroke=${S}/>`,
	pause: svg`<rect x="8" y="6" width="6" height="20" rx="1" fill=${O} stroke=${OD}/><rect x="18" y="6" width="6" height="20" rx="1" fill=${O} stroke=${OD}/>`,
	keyboard: svg`<rect x="3" y="9" width="26" height="15" rx="2" fill=${SL} stroke=${SD}/><path d="M7 13h2M11 13h2M15 13h2M19 13h2M23 13h2M7 17h2M23 17h2M11 20h10" stroke=${SD} stroke-width="2"/>`,
	filter: svg`<path d="M4 5h24l-9 11v9l-6 3V16z" fill=${B} stroke=${BD} stroke-linejoin="round"/>`,
	network: svg`<rect x="12" y="3" width="8" height="7" rx="1" fill=${B} stroke=${BD}/><rect x="3" y="22" width="8" height="7" rx="1" fill=${T} stroke=${TD}/><rect x="21" y="22" width="8" height="7" rx="1" fill=${T} stroke=${TD}/><path d="M16 10v6M7 22v-6h18v6" fill="none" stroke=${SD} stroke-width="2"/>`,
	info: svg`<circle cx="16" cy="16" r="12" fill=${B} stroke=${BD}/><path d="M16 14v9" stroke="#fff" stroke-width="3"/><circle cx="16" cy="10" r="1.8" fill="#fff"/>`,
	temperature: svg`<path d="M13 5a3 3 0 0 1 6 0v13a6 6 0 1 1-6 0z" fill="#fff" stroke=${SD} stroke-width="2"/><circle cx="16" cy="22" r="3.5" fill=${R}/><path d="M16 22V11" stroke=${R} stroke-width="3"/>`,
	memory: svg`<rect x="3" y="9" width="26" height="14" rx="1" fill=${G} stroke=${GD}/><rect x="6" y="12" width="4" height="8" fill=${K}/><rect x="12" y="12" width="4" height="8" fill=${K}/><rect x="18" y="12" width="4" height="8" fill=${K}/><path d="M6 23v3M10 23v3M14 23v3M18 23v3M22 23v3M26 23v3" stroke=${GD} stroke-width="1.5"/>`,
	bell: svg`<path d="M8 22V14a8 8 0 0 1 16 0v8l3 3H5z" fill=${Y} stroke=${OD} stroke-linejoin="round"/><circle cx="16" cy="27" r="2.5" fill=${OD}/>`,
	actuator: svg`<rect x="4" y="10" width="14" height="12" rx="2" fill=${S} stroke=${SD}/><rect x="18" y="14" width="10" height="4" fill=${O} stroke=${OD}/><path d="M8 10V6h6v4" fill="none" stroke=${SD} stroke-width="2"/>`,
	sensor: svg`<circle cx="9" cy="16" r="5" fill=${R} stroke=${RD}/><path d="M15 16h12" stroke=${R} stroke-width="2" stroke-dasharray="3 2"/><rect x="25" y="9" width="4" height="14" fill=${SD}/>`,
	can: svg`<path d="M3 13h26M3 19h26" stroke=${O} stroke-width="2.5"/><rect x="6" y="6" width="6" height="5" fill=${SD}/><rect x="20" y="21" width="6" height="5" fill=${SD}/>`,
	git: svg`<circle cx="9" cy="8" r="3.5" fill=${O} stroke=${OD}/><circle cx="9" cy="24" r="3.5" fill=${O} stroke=${OD}/><circle cx="23" cy="13" r="3.5" fill=${O} stroke=${OD}/><path d="M9 11.5v9M23 16.5c0 5-8 3-12 6" fill="none" stroke=${OD} stroke-width="2"/>`,
	i2c: svg`<path d="M3 11h26M3 21h26" stroke=${T} stroke-width="2.5"/><rect x="6" y="13" width="5" height="6" fill=${SD}/><rect x="14" y="13" width="5" height="6" fill=${SD}/><rect x="22" y="13" width="5" height="6" fill=${SD}/>`,
} as const;

export type IconName = keyof typeof ICONS;

/** The <svg> element is an html template (an svg template must not contain the outer <svg>); the shapes are svg fragments. */
export function icon(name: IconName, size = 32, cls = ""): TemplateResult {
	const body: SVGTemplateResult = ICONS[name] ?? ICONS.info;
	return html`<svg class="icon ${cls}" width=${size} height=${size} viewBox="0 0 32 32" aria-hidden="true" stroke-width="1.5">${body}</svg>`;
}
