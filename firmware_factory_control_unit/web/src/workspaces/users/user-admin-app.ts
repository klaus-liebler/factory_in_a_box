// Accounts of the device (admin only; firmware src/auth.cpp /api/users): list, add with a one-time password,
// change the role, delete. Setting another user's password is a password reset: the device allows it only with
// physical presence (USB). The device refuses the last admin's removal and an admin's own deletion.
import { LitElement, css, html } from "lit";
import { customElement, state } from "lit/decorators.js";
import { listUsers, manageUser, passwordProblem, type Role, ROLE_LABEL, session, type UserEntry } from "../../core/auth.ts";

const ROLES = Object.keys(ROLE_LABEL) as Role[];

@customElement("user-admin-app")
export class UserAdminApp extends LitElement {
	static styles = css`
		:host { display: block; padding: 16px; font-size: 14px; }
		table { border-collapse: collapse; margin-bottom: 18px; }
		th, td { text-align: left; padding: 6px 12px; border-bottom: 1px solid var(--border, #ddd); }
		form { display: flex; gap: 8px; align-items: end; flex-wrap: wrap; }
		label { display: grid; gap: 3px; font-size: 12px; }
		input, select, button { font: inherit; padding: 5px 8px; }
		.msg { margin-top: 10px; }
		.error { color: #b3261e; }
	`;

	@state() private users: UserEntry[] = [];
	@state() private message = "";
	@state() private failed = false;

	onShow(): void {
		void this.load();
	}
	connectedCallback(): void {
		super.connectedCallback();
		void this.load();
	}

	private async run(what: () => Promise<void>, ok: string) {
		try {
			await what();
			this.failed = false;
			this.message = ok;
		} catch (e) {
			this.failed = true;
			this.message = (e as Error).message;
		}
		await this.load();
	}

	private async load() {
		try {
			this.users = await listUsers();
		} catch (e) {
			this.failed = true;
			this.message = (e as Error).message;
		}
	}

	private add(e: Event) {
		e.preventDefault();
		const f = new FormData(e.target as HTMLFormElement);
		const user = String(f.get("user"));
		const password = String(f.get("password"));
		const problem = passwordProblem(password, user);
		if (problem) {
			this.failed = true;
			this.message = `Einmalpasswort: ${problem}`;
			return;
		}
		void this.run(() => manageUser("add", user, { role: f.get("role") as Role, password }),
			`${user} angelegt - muss das Passwort bei der ersten Anmeldung ändern.`);
	}

	render() {
		const me = session()?.user;
		return html`
			<table>
				<tr><th>Benutzer</th><th>Rolle</th><th>Status</th><th></th></tr>
				${this.users.map((u) => html`<tr>
					<td>${u.user}${u.user === me ? " (Sie)" : ""}</td>
					<td><select ?disabled=${u.user === me} @change=${(e: Event) => void this.run(() => manageUser("role", u.user, { role: (e.target as HTMLSelectElement).value as Role }), `Rolle von ${u.user} geändert (neu anmelden).`)}>
						${ROLES.map((r) => html`<option value=${r} ?selected=${r === u.role}>${ROLE_LABEL[r]}</option>`)}
					</select></td>
					<td>${u.mustChangePassword ? "Einmalpasswort" : "aktiv"}</td>
					<td><button ?disabled=${u.user === me} @click=${() => confirm(`${u.user} löschen?`) && void this.run(() => manageUser("delete", u.user), `${u.user} gelöscht.`)}>Löschen</button></td>
				</tr>`)}
			</table>
			<form @submit=${this.add}>
				<label>Benutzer (a-z 0-9 . _ -) <input name="user" pattern="[a-z0-9._-]{1,15}" required /></label>
				<label>Rolle <select name="role">${ROLES.map((r) => html`<option value=${r}>${ROLE_LABEL[r]}</option>`)}</select></label>
				<label>Einmalpasswort <input name="password" type="password" autocomplete="new-password" required /></label>
				<button>Anlegen</button>
			</form>
			${this.message ? html`<div class="msg ${this.failed ? "error" : ""}">${this.message}</div>` : ""}
		`;
	}
}
