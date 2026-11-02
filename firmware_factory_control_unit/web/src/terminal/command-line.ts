// The Terminal page's command line logic without DOM: words and cursor, the command meant, the
// parameter being typed, value matching and completion (tests: command-line.test.ts).
import { REGIONS } from "../../generated/register-map.js";
import { TERMINAL_COMMANDS, type TerminalCommand, type TerminalParam, type TerminalValue } from "../../generated/terminal-commands.js";

export interface Word {
	text: string;
	start: number;
	end: number;
}

/** The words of the line and the one the cursor is in (or a new, empty one after a space). */
export function analyse(line: string, cursor: number): { words: Word[]; current: Word; index: number } {
	const words: Word[] = [];
	for (const m of line.matchAll(/\S+/g)) words.push({ text: m[0], start: m.index, end: m.index + m[0].length });
	let index = words.findIndex((w) => cursor >= w.start && cursor <= w.end);
	if (index < 0) {
		index = words.filter((w) => w.end < cursor).length;
		words.splice(index, 0, { text: "", start: cursor, end: cursor });
	}
	return { words, current: words[index], index };
}

export function findCommand(name: string): TerminalCommand | undefined {
	return TERMINAL_COMMANDS.find((c) => c.name === name);
}

/** The command meant: the exact name, or, while typing it, the only one with that prefix. */
export function meantCommand(words: Word[], index: number): TerminalCommand | undefined {
	const first = words[0]?.text ?? "";
	const exact = findCommand(first);
	if (exact || index > 0 || first.length === 0) return exact;
	const candidates = TERMINAL_COMMANDS.filter((c) => c.name.startsWith(first));
	return candidates.length === 1 ? candidates[0] : undefined;
}

/**
 * The parameter at word `index` (1 = first parameter). `tooMany` only when a word was really
 * typed beyond the last parameter: the empty word after a trailing space is no parameter yet.
 */
export function paramAt(command: TerminalCommand, index: number, current: Word): { param?: TerminalParam; tooMany: boolean } {
	if (index === 0) return { tooMany: false };
	const param = command.params[index - 1];
	return { param, tooMany: !param && current.text.length > 0 };
}

/** After completing word `index`: a space only when another parameter can follow. */
export function spaceAfterCompletion(command: TerminalCommand | undefined, index: number): boolean {
	return command !== undefined && index < command.params.length;
}

export function matches(values: readonly TerminalValue[], prefix: string): TerminalValue[] {
	const p = prefix.toLowerCase();
	const byValue = values.filter((v) => v.value.toLowerCase().startsWith(p));
	if (byValue.length > 0 || p.length < 2) return byValue;
	return values.filter((v) => v.doc.toLowerCase().includes(p)); // e.g. a register by its name
}

export function commonPrefix(values: string[]): string {
	let prefix = values[0] ?? "";
	for (const v of values) while (!v.startsWith(prefix)) prefix = prefix.slice(0, -1);
	return prefix;
}

/** Values computed in the browser (TerminalParam.source). */
export function sourceValues(param: TerminalParam, words: Word[]): TerminalValue[] {
	switch (param.source) {
		case "commands":
			return TERMINAL_COMMANDS.map((c) => ({ value: c.name, doc: c.doc }));
		case "registers": {
			const bank = words[1]?.text === "h" ? "holding" : words[1]?.text === "i" ? "input" : undefined;
			if (!bank) return [];
			return REGIONS.flatMap((r) => r.registers)
				.filter((r) => r.bank === bank)
				.sort((a, b) => a.address - b.address)
				.map((r) => ({ value: String(r.address), doc: `${r.name}${r.description ? ` - ${r.description}` : ""}` }));
		}
		default:
			return [];
	}
}

/**
 * Tab completion of the word at the cursor with the candidate values: the new line and cursor,
 * or the candidates to list when they share no longer prefix; null when nothing matches.
 */
export function complete(
	line: string,
	cursor: number,
	values: readonly TerminalValue[],
): { line: string; cursor: number } | { list: string[] } | null {
	const { words, current, index } = analyse(line, cursor);
	const prefix = line.slice(current.start, cursor);
	const found = matches(values, prefix);
	if (found.length === 0) return null;
	const replace = (text: string, done: boolean) => {
		const command = index === 0 ? findCommand(text) : findCommand(words[0].text);
		const space = done && spaceAfterCompletion(command, index);
		const before = line.slice(0, current.start) + text;
		const after = line.slice(current.end);
		const sep = space && !after.startsWith(" ") ? " " : "";
		return { line: before + sep + after, cursor: before.length + (space ? 1 : 0) };
	};
	if (found.length === 1) return replace(found[0].value, true);
	const common = commonPrefix(found.map((v) => v.value));
	if (common.length > prefix.length && common.toLowerCase().startsWith(prefix.toLowerCase())) return replace(common, false);
	return { list: found.map((v) => v.value) };
}
