// @vitest-environment happy-dom
//
// Regression: the prompt started near the middle of the console. .term-line is
// white-space: pre-wrap (output keeps its spaces), and the prompt line's template had line
// breaks and tabs inside the <div> - rendered as text, they indented the prompt.
import { describe, expect, it } from "vitest";
import "./terminal-app.js";
import type { TerminalApp } from "./terminal-app.js";

describe("terminal-app", () => {
	it("renders the prompt line without template whitespace", async () => {
		const app = document.createElement("terminal-app") as TerminalApp;
		document.body.appendChild(app);
		await app.updateComplete;
		const prompt = app.querySelector(".term-prompt");
		expect(prompt?.textContent).toBe("fcu>  "); // the prompt and the block cursor on an empty line
		app.remove();
	});
});
