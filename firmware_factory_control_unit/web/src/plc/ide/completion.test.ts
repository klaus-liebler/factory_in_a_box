// Code completion in the ST editor: where the caret is (completionContext).
import { describe, expect, it } from "vitest";
import { completionContext } from "./st-editor.ts";

const at = (text: string) => completionContext(text.replace("|", ""), text.indexOf("|"));

describe("ST code completion: the context at the caret", () => {
	it("an identifier begun, after an operator, at the start of a line", () => {
		expect(at("x := Light|")).toEqual({ prefix: "Light", object: null });
		expect(at("IF a AND Pie|")).toEqual({ prefix: "Pie", object: null });
		expect(at("a := 1;\nCon|")).toEqual({ prefix: "Con", object: null });
	});

	it("after a dot: the members of the object (also a path)", () => {
		expect(at("x := Pieces.|")).toEqual({ prefix: "", object: "Pieces" });
		expect(at("x := Pieces.C|")).toEqual({ prefix: "C", object: "Pieces" });
		expect(at("y := PLC_PRG.Ejector.Va|")).toEqual({ prefix: "Va", object: "PLC_PRG.Ejector" });
	});

	it("nothing in comments, strings and literals", () => {
		expect(at("x := 1; // Light|")).toBeNull();
		expect(at("x := 1; (* Light|")).toBeNull();
		expect(at("s := 'Light|")).toBeNull();
		expect(at("x := 16#F|")).toBeNull();
		expect(at("t := T#1s|")).toBeNull();
		expect(at("x := 12|")).toBeNull();
	});

	it("after a closed comment or string it completes again", () => {
		expect(at("(* note *) Light|")).toEqual({ prefix: "Light", object: null });
		expect(at("s := 'a'; Light|")).toEqual({ prefix: "Light", object: null });
	});
});
