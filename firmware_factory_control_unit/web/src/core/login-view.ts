// The login screen in front of the app (core/auth.ts): user and password, then - for the one-time password from
// the device label - the forced password change. Resolves showLogin() with the session once the app may start.
import { LitElement, css, html, nothing } from "lit";
import { customElement, state } from "lit/decorators.js";
import { ApiError, changePassword, login, passwordProblem, ROLE_LABEL, type Session, session } from "./auth.ts";

@customElement("fcu-login")
export class LoginView extends LitElement {
	static styles = css`
		:host { position: fixed; inset: 0; display: grid; place-items: center; background: var(--surface-2, #eef1f5); z-index: 1000;
			font-family: var(--font, system-ui, sans-serif); }
		form { background: var(--surface-1, #fff); padding: 28px 32px; border-radius: 10px; width: 340px;
			box-shadow: 0 6px 30px rgba(0, 0, 0, 0.12); display: grid; gap: 12px; }
		h1 { font-size: 18px; margin: 0 0 4px; }
		p { margin: 0; font-size: 13px; color: var(--text-2, #555); }
		label { display: grid; gap: 4px; font-size: 13px; }
		input { font: inherit; padding: 7px 9px; border: 1px solid var(--border, #bbb); border-radius: 6px; }
		button { font: inherit; padding: 8px; border: 0; border-radius: 6px; background: var(--accent, #1f6feb); color: #fff; cursor: pointer; }
		button[disabled] { opacity: 0.6; cursor: default; }
		.error { color: #b3261e; font-size: 13px; }
	`;

	@state() private mode: "login" | "change" = "login";
	@state() private busy = false;
	@state() private error = "";
	private user = "admin";
	private currentPassword = "";
	done: (s: Session) => void = () => {};

	private async submitLogin(e: Event) {
		e.preventDefault();
		const f = new FormData(e.target as HTMLFormElement);
		this.user = String(f.get("user") ?? "");
		this.currentPassword = String(f.get("password") ?? "");
		this.busy = true;
		this.error = "";
		try {
			const s = await login(this.user, this.currentPassword);
			if (s.mustChangePassword) {
				this.mode = "change";
			} else {
				this.currentPassword = "";
				this.done(s);
			}
		} catch (err) {
			this.error = err instanceof ApiError && err.status === 429
				? `Zu viele Fehlversuche - in ${err.retryAfterS} s erneut versuchen.`
				: err instanceof ApiError && err.status === 401
					? "Benutzer oder Passwort falsch."
					: `Anmeldung fehlgeschlagen: ${(err as Error).message}`;
		} finally {
			this.busy = false;
		}
	}

	private async submitChange(e: Event) {
		e.preventDefault();
		const f = new FormData(e.target as HTMLFormElement);
		const next = String(f.get("new") ?? "");
		const repeat = String(f.get("repeat") ?? "");
		const problem = passwordProblem(next, this.user);
		if (problem) return void (this.error = `Neues Passwort: ${problem}.`);
		if (next !== repeat) return void (this.error = "Die beiden Eingaben stimmen nicht überein.");
		this.busy = true;
		this.error = "";
		try {
			await changePassword(this.currentPassword, next);
			this.currentPassword = "";
			this.done(session()!);
		} catch (err) {
			this.error = `Passwort nicht geändert: ${(err as Error).message}`;
		} finally {
			this.busy = false;
		}
	}

	render() {
		const err = this.error ? html`<div class="error" role="alert">${this.error}</div>` : nothing;
		if (this.mode === "change") {
			return html`<form @submit=${this.submitChange}>
				<h1>Neues Passwort festlegen</h1>
				<p>Das Passwort vom Geräte-Etikett gilt nur für die erste Anmeldung. Bitte ein eigenes festlegen (mindestens 10 Zeichen).</p>
				<label>Neues Passwort <input name="new" type="password" autocomplete="new-password" required autofocus /></label>
				<label>Wiederholen <input name="repeat" type="password" autocomplete="new-password" required /></label>
				${err}
				<button ?disabled=${this.busy}>${this.busy ? "Wird gespeichert …" : "Passwort speichern"}</button>
			</form>`;
		}
		return html`<form @submit=${this.submitLogin}>
			<h1>Factory Control Unit</h1>
			<p>Anmelden, um Werte zu sehen und das Gerät zu bedienen. Rollen: ${Object.values(ROLE_LABEL).join(", ")}.</p>
			<label>Benutzer <input name="user" autocomplete="username" .value=${this.user} required /></label>
			<label>Passwort <input name="password" type="password" autocomplete="current-password" required autofocus /></label>
			${err}
			<button ?disabled=${this.busy}>${this.busy ? "Prüfe …" : "Anmelden"}</button>
		</form>`;
	}
}

/** Shows the login screen until a session may start the app. */
export function showLogin(): Promise<Session> {
	return new Promise((resolve) => {
		const view = document.createElement("fcu-login") as LoginView;
		view.done = (s) => {
			view.remove();
			resolve(s);
		};
		document.body.append(view);
	});
}
