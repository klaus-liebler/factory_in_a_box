import { describe, expect, it } from "vitest";
import { TERMINAL_COMMANDS } from "../../generated/terminal-commands.js";
import { analyse, complete, findCommand, matches, paramAt, sourceValues } from "./command-line.js";

const commandNames = TERMINAL_COMMANDS.map((c) => ({ value: c.name, doc: c.doc }));
const valuesOf = (command: string, param: number) => findCommand(command)!.params[param].values!;

describe("analyse", () => {
	it("finds the word at the cursor, or an empty one after a space", () => {
		expect(analyse("pwm co", 6).index).toBe(1);
		expect(analyse("pwm co", 6).current.text).toBe("co");
		const after = analyse("pwm ", 4);
		expect(after.index).toBe(1);
		expect(after.current.text).toBe("");
		expect(analyse("pwm", 3).index).toBe(0);
	});
});

// Regression: completing a command without parameters appended a space, and the empty word
// after that space counted as a parameter -> "too many parameters" for a correct line.
describe("completion and trailing space", () => {
	it("completes a command without parameters without a space", () => {
		expect(complete("inf", 3, commandNames)).toEqual({ line: "info", cursor: 4 });
		expect(complete("temp", 4, commandNames)).toEqual({ line: "temp", cursor: 4 });
	});

	it("completes a command with parameters with a space", () => {
		expect(complete("pw", 2, commandNames)).toEqual({ line: "pwm ", cursor: 4 });
	});

	it("adds a space after a value only when another parameter follows", () => {
		expect(complete("pwm con", 7, valuesOf("pwm", 0))).toEqual({ line: "pwm conveyor ", cursor: 13 });
		expect(complete("reset n", 7, valuesOf("reset", 0))).toEqual({ line: "reset now", cursor: 9 });
	});

	it("does not count the empty word after a trailing space as a parameter", () => {
		const info = findCommand("info")!;
		const line = analyse("info ", 5);
		expect(paramAt(info, line.index, line.current).tooMany).toBe(false);
		const typed = analyse("info x", 6);
		expect(paramAt(info, typed.index, typed.current).tooMany).toBe(true);
	});
});

describe("values", () => {
	it("lists the candidates when they share no longer prefix", () => {
		expect(complete("i", 1, commandNames)).toEqual({ list: ["info", "in", "i2c"] });
	});

	it("finds a register by its name", () => {
		const words = analyse("reg h conveyor", 14).words;
		const param = findCommand("reg")!.params[1];
		const found = matches(sourceValues(param, words), "conveyor_pwm");
		expect(found.map((v) => v.value)).toContain("40");
	});
});
