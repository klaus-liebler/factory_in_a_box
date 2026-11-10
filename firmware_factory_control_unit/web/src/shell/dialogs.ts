// Modal dialogs in the look of the web application, for every app - instead of the browser's
// confirm(), prompt() and alert(). Built on <dialog> (showModal: focus stays inside, Esc cancels,
// the page behind is inert). Each function returns a promise of the answer:
//
//   if (await confirmDialog({ title: "Löschen", message: "Wirklich?", confirm: "Löschen", danger: true })) ...
//   const name = await promptDialog({ title: "Umbenennen", label: "Name", value: old, validate: (v) => v ? null : "leer" });
//   const what = await choiceDialog({ title: "Ordner löschen", message: "...", choices: [{ id: "all", label: "Alles löschen", danger: true }, ...] });
//   await messageDialog({ title: "Hinweis", message: "..." });
//
// Styles: .ui-dialog in styles.css.

export interface DialogChoice {
	id: string;
	label: string;
	/** The default action (Enter). */
	primary?: boolean;
	/** A destructive action: red. */
	danger?: boolean;
}

interface DialogSpec {
	title: string;
	/** Paragraphs (a string: one). */
	message?: string | string[];
	choices: DialogChoice[];
	/** A text field: its label, initial value and check (the reason it is not accepted, or null). */
	input?: { label: string; value: string; placeholder?: string; validate?: (value: string) => string | null };
	/** The choice Esc and a click outside give (null: the dialog's result is null). */
	cancelId?: string | null;
}

/** Shows the dialog; resolves with the chosen id (and the field's text), or id null when cancelled. */
function show(spec: DialogSpec): Promise<{ id: string | null; text: string }> {
	return new Promise((resolve) => {
		const dialog = document.createElement("dialog");
		dialog.className = "ui-dialog";
		const form = document.createElement("form");
		form.method = "dialog";
		form.noValidate = true;
		const h = document.createElement("h2");
		h.textContent = spec.title;
		form.append(h);
		for (const text of spec.message === undefined ? [] : Array.isArray(spec.message) ? spec.message : [spec.message]) {
			const p = document.createElement("p");
			p.textContent = text;
			form.append(p);
		}
		let field: HTMLInputElement | null = null;
		let problem: HTMLElement | null = null;
		if (spec.input) {
			const label = document.createElement("label");
			label.className = "ui-dialog-field";
			const span = document.createElement("span");
			span.textContent = spec.input.label;
			field = document.createElement("input");
			field.type = "text";
			field.value = spec.input.value;
			field.spellcheck = false;
			field.autocomplete = "off";
			if (spec.input.placeholder) field.placeholder = spec.input.placeholder;
			problem = document.createElement("div");
			problem.className = "ui-dialog-problem";
			label.append(span, field);
			form.append(label, problem);
		}
		const buttons = document.createElement("div");
		buttons.className = "ui-dialog-buttons";
		let result: string | null = spec.cancelId ?? null;
		let primaryButton: HTMLButtonElement | null = null;
		const check = (): boolean => {
			const reason = field && spec.input?.validate ? spec.input.validate(field.value.trim()) : null;
			// an empty field: the button is off, but no complaint before anything is typed
			if (problem) problem.textContent = field && field.value.trim() === "" ? "" : reason ?? "";
			if (primaryButton) primaryButton.disabled = reason !== null;
			return reason === null;
		};
		for (const c of spec.choices) {
			const b = document.createElement("button");
			b.type = c.primary ? "submit" : "button";
			b.textContent = c.label;
			b.className = `${c.primary ? "ui-dialog-primary" : ""} ${c.danger ? "ui-dialog-danger" : ""}`.trim();
			if (c.primary) primaryButton = b;
			b.addEventListener("click", (e) => {
				e.preventDefault();
				if (c.primary && !check()) return;
				result = c.id;
				dialog.close();
			});
			buttons.append(b);
		}
		form.append(buttons);
		field?.addEventListener("input", check);
		form.addEventListener("submit", (e) => {
			// Enter in the field: the primary choice
			e.preventDefault();
			primaryButton?.click();
		});
		dialog.append(form);
		// a click on the backdrop (the dialog itself, outside the form): cancel
		dialog.addEventListener("click", (e) => {
			if (e.target === dialog) dialog.close();
		});
		dialog.addEventListener("close", () => {
			const text = field?.value.trim() ?? "";
			dialog.remove();
			resolve({ id: result, text });
		});
		document.body.append(dialog);
		dialog.showModal();
		check();
		if (field) {
			field.focus();
			field.select();
		} else (primaryButton ?? buttons.querySelector("button"))?.focus();
	});
}

/** A question with two answers; true: confirmed. */
export async function confirmDialog(o: { title: string; message?: string | string[]; confirm?: string; cancel?: string; danger?: boolean }): Promise<boolean> {
	const r = await show({
		title: o.title, message: o.message,
		choices: [{ id: "cancel", label: o.cancel ?? "Abbrechen" }, { id: "ok", label: o.confirm ?? "OK", primary: true, danger: o.danger }],
	});
	return r.id === "ok";
}

/** A line of text; null: cancelled. `validate` gives the reason a text is not accepted (shown below the field). */
export async function promptDialog(o: {
	title: string; message?: string | string[]; label: string; value?: string; placeholder?: string; confirm?: string;
	validate?: (value: string) => string | null;
}): Promise<string | null> {
	const r = await show({
		title: o.title, message: o.message,
		input: { label: o.label, value: o.value ?? "", placeholder: o.placeholder, validate: o.validate },
		choices: [{ id: "cancel", label: "Abbrechen" }, { id: "ok", label: o.confirm ?? "OK", primary: true }],
	});
	return r.id === "ok" ? r.text : null;
}

/** One of several answers (their ids); null: cancelled. */
export async function choiceDialog(o: { title: string; message?: string | string[]; choices: DialogChoice[] }): Promise<string | null> {
	return (await show({ title: o.title, message: o.message, choices: [{ id: "\0cancel", label: "Abbrechen" }, ...o.choices] }).then((r) => (r.id === "\0cancel" ? null : r.id)));
}

/** A message to acknowledge. */
export async function messageDialog(o: { title: string; message: string | string[] }): Promise<void> {
	await show({ title: o.title, message: o.message, choices: [{ id: "ok", label: "OK", primary: true }] });
}
