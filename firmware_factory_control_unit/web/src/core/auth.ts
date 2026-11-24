// Login against the device (firmware src/auth.hpp, docs/security.md): the HTTPS API under /api/ with an
// HttpOnly session cookie the page never sees. The static app loads without login; every data and command
// goes over the WebSocket, which the device opens for a logged-in session only, and checks each request
// against the role there. This module only mirrors the role for the UI: what a role may not do is hidden or
// greyed, but the device decides.

export type Role = "observer" | "operator" | "programmer" | "admin";
export const ROLE_LABEL: Record<Role, string> = {
	observer: "Beobachter",
	operator: "Bediener",
	programmer: "Programmierer",
	admin: "Administrator",
};
const RANK: Record<Role, number> = { observer: 1, operator: 2, programmer: 3, admin: 4 };

/** WebSocket error codes of refused requests (WsProtocol::ErrorCode::Application + n, firmware auth.hpp). */
export const WS_ERROR_NOT_AUTHENTICATED = 0x101;
export const WS_ERROR_ROLE = 0x102;
export const WS_ERROR_PRESENCE = 0x103;
export const WS_ERROR_PASSWORD_CHANGE = 0x104;

export interface Session {
	user: string;
	role: Role;
	mustChangePassword: boolean;
	/** physical presence: logged in over USB (or the presence button) */
	present: boolean;
	via: "usb" | "lan";
	idleTimeoutS: number;
}

export class ApiError extends Error {
	constructor(
		readonly status: number,
		message: string,
		readonly retryAfterS = 0,
	) {
		super(message);
	}
}

let current: Session | null = null;
const listeners = new Set<(s: Session | null) => void>();

export function session(): Session | null {
	return current;
}
export function onSessionChange(cb: (s: Session | null) => void): () => void {
	listeners.add(cb);
	return () => listeners.delete(cb);
}
function set(s: Session | null): void {
	current = s;
	for (const l of listeners) l(s);
}

/** The logged-in role is at least `least`. */
export function hasRole(least: Role): boolean {
	return current !== null && RANK[current.role] >= RANK[least];
}

async function api<T>(method: "GET" | "POST", path: string, body?: unknown): Promise<T> {
	const r = await fetch(path, {
		method,
		credentials: "same-origin",
		headers: body !== undefined ? { "Content-Type": "application/json" } : {},
		body: body !== undefined ? JSON.stringify(body) : undefined,
		cache: "no-store",
	});
	let json: Record<string, unknown> = {};
	try {
		json = (await r.json()) as Record<string, unknown>;
	} catch {
		// an empty or non-JSON body: the status says it
	}
	if (!r.ok) {
		const retry = Number(r.headers.get("Retry-After") ?? json.retryAfter ?? 0);
		throw new ApiError(r.status, String(json.error ?? r.statusText), Number.isFinite(retry) ? retry : 0);
	}
	return json as T;
}

/** The current login (null: none). */
export async function refreshSession(): Promise<Session | null> {
	try {
		set(await api<Session>("GET", "/api/session"));
	} catch (e) {
		if (e instanceof ApiError && e.status === 401) set(null);
		else throw e;
	}
	return current;
}

export async function login(user: string, password: string): Promise<Session> {
	const s = await api<Session>("POST", "/api/login", { user, password });
	set(s);
	return s;
}

export async function changePassword(currentPassword: string, newPassword: string): Promise<void> {
	await api("POST", "/api/password", { current: currentPassword, new: newPassword });
	await refreshSession();
}

export async function logout(): Promise<void> {
	try {
		await api("POST", "/api/logout", {});
	} finally {
		set(null);
	}
}

export interface UserEntry {
	user: string;
	role: Role;
	mustChangePassword: boolean;
}
export async function listUsers(): Promise<UserEntry[]> {
	return (await api<{ users: UserEntry[] }>("GET", "/api/users")).users;
}
export async function manageUser(action: "add" | "delete" | "role" | "password", user: string, extra: { role?: Role; password?: string } = {}): Promise<void> {
	await api("POST", "/api/users", { action, user, ...extra });
}

/** Text for a refused WebSocket request (WsErrorReply.code), or null for other errors. */
export function refusalText(code: number): string | null {
	switch (code) {
		case WS_ERROR_NOT_AUTHENTICATED: return "Nicht (mehr) angemeldet.";
		case WS_ERROR_ROLE: return "Ihre Rolle darf das nicht.";
		case WS_ERROR_PRESENCE: return "Nur mit physischer Anwesenheit: über die USB-Verbindung am Gerät anmelden.";
		case WS_ERROR_PASSWORD_CHANGE: return "Erst das Passwort ändern.";
		default: return null;
	}
}

/** New password rules of the device (rtos/auth/credentials.hpp password_problem). */
export function passwordProblem(pw: string, user: string): string | null {
	if (pw.length < 10) return "mindestens 10 Zeichen";
	if (new TextEncoder().encode(pw).length > 64) return "höchstens 64 Byte";
	if (pw === user) return "nicht gleich dem Benutzernamen";
	if (/[\u0000-\u001f\u007f]/.test(pw)) return "keine Steuerzeichen";
	return null;
}
