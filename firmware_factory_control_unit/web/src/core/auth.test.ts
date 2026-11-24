// core/auth.ts: the session mirror of the UI (roles, refusals, the API calls with their errors) against a
// fetch stub that answers like the firmware (src/auth.cpp).
import { afterEach, describe, expect, it, vi } from "vitest";
import {
	ApiError, changePassword, hasRole, login, logout, passwordProblem, refreshSession, refusalText, session,
	WS_ERROR_PRESENCE, WS_ERROR_ROLE,
} from "./auth.ts";

function reply(status: number, body: unknown, headers: Record<string, string> = {}) {
	return Promise.resolve(new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json", ...headers } }));
}

afterEach(() => {
	vi.unstubAllGlobals();
});

describe("auth", () => {
	it("mirrors the session and its role", async () => {
		const calls: Array<[string, RequestInit | undefined]> = [];
		vi.stubGlobal("fetch", (url: string, init?: RequestInit) => {
			calls.push([url, init]);
			return reply(200, { user: "op", role: "operator", mustChangePassword: false, present: false, via: "lan", idleTimeoutS: 1800 });
		});
		const s = await login("op", "Passwort-1234");
		expect(s.role).toBe("operator");
		expect(calls[0][0]).toBe("/api/login");
		expect(calls[0][1]?.method).toBe("POST");
		expect(JSON.parse(String(calls[0][1]?.body))).toEqual({ user: "op", password: "Passwort-1234" });
		expect(calls[0][1]?.credentials).toBe("same-origin");
		expect(hasRole("observer")).toBe(true);
		expect(hasRole("operator")).toBe(true);
		expect(hasRole("programmer")).toBe(false);
	});

	it("a 401 on /api/session means logged out, other errors are errors", async () => {
		vi.stubGlobal("fetch", () => reply(401, { error: "not logged in" }));
		expect(await refreshSession()).toBeNull();
		expect(session()).toBeNull();
		expect(hasRole("observer")).toBe(false);
		vi.stubGlobal("fetch", () => reply(503, { error: "busy" }));
		await expect(refreshSession()).rejects.toBeInstanceOf(ApiError);
	});

	it("throttled logins carry Retry-After", async () => {
		vi.stubGlobal("fetch", () => reply(429, { error: "too many failed logins", retryAfter: 8 }, { "Retry-After": "8" }));
		const e = await login("admin", "x").catch((x: unknown) => x);
		expect(e).toBeInstanceOf(ApiError);
		expect((e as ApiError).status).toBe(429);
		expect((e as ApiError).retryAfterS).toBe(8);
	});

	it("password change and logout", async () => {
		const paths: string[] = [];
		vi.stubGlobal("fetch", (url: string) => {
			paths.push(url);
			return url === "/api/session"
				? reply(200, { user: "admin", role: "admin", mustChangePassword: false, present: true, via: "usb", idleTimeoutS: 1800 })
				: reply(200, { ok: true });
		});
		await changePassword("old-label-pw", "Ganz-neues-Passwort");
		expect(session()?.mustChangePassword).toBe(false);
		await logout();
		expect(session()).toBeNull();
		expect(paths).toEqual(["/api/password", "/api/session", "/api/logout"]);
	});

	it("knows the device's rules and refusal codes", () => {
		expect(passwordProblem("short", "admin")).not.toBeNull();
		expect(passwordProblem("adminadmin", "adminadmin")).not.toBeNull();
		expect(passwordProblem("lang genug 123", "admin")).toBeNull();
		expect(refusalText(WS_ERROR_ROLE)).toMatch(/Rolle/);
		expect(refusalText(WS_ERROR_PRESENCE)).toMatch(/USB/);
		expect(refusalText(2)).toBeNull();
	});
});
