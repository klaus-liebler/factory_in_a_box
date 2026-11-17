// The simulation library against analytic solutions and energy/mass balances (docs/sim-library.md).
import { describe, expect, it } from "vitest";
import { LibRun } from "./lib-harness.ts";

const one = (type: string, dtMs = 50) => new LibRun(`b : SIM.${type};`, "b();", dtMs);

describe("drives", () => {
	it("MOTOR: no-load speed, speed under load, current, time constant", () => {
		const r = one("MOTOR").set("b.On", true).run(2000);
		expect(r.num("b.Speed")).toBeCloseTo(3000 / (1 + 0.01 / 1000 * 3000 / 2), -1);  // friction lowers it a little
		r.set("b.Friction", 0).set("b.LoadTorque", 1).run(2000);  // half the stall torque: half speed
		expect(r.num("b.Speed")).toBeCloseTo(1500, 0);
		expect(r.num("b.Torque")).toBeCloseTo(1, 3);
		expect(r.num("b.Current")).toBeCloseTo(5.3, 2);
		expect(r.num("b.Power")).toBeCloseTo(1 * 1500 * 2 * Math.PI / 60, 0);
		// time constant J * n0 / Ts in rad: 0.0005 * 314.16 / 2 = 78.5 ms; 63 % after one
		const t = new LibRun("b : SIM.MOTOR;", "b(On := TRUE, Friction := 0.0);", 1);
		t.cycle(1 + 79);
		expect(t.num("b.Speed") / 3000).toBeCloseTo(1 - Math.exp(-79 / 78.54), 2);
	});
	it("MOTOR: a load larger than the stall torque holds it; reverse; coasting; brake", () => {
		const r = one("MOTOR").set("b.On", true).set("b.LoadTorque", 3).run(1000);
		expect(r.num("b.Speed")).toBe(0);
		expect(r.num("b.Current")).toBeCloseTo(10);  // stall current
		r.set("b.LoadTorque", 0).set("b.U", -50).run(2000);
		expect(r.num("b.Speed")).toBeLessThan(-1400);
		r.set("b.On", false).set("b.LoadTorque", 0.5).run(5000);  // coasts down against the load and stays
		expect(r.num("b.Speed")).toBe(0);
		expect(r.num("b.Current")).toBe(0);
		r.set("b.On", true).set("b.U", 100).set("b.LoadTorque", 0).set("b.Brake", true).run(500);
		expect(r.num("b.Speed")).toBe(0);
	});
	it("MOTOR stays stable with a tiny inertia and a long cycle", () => {
		const r = new LibRun("b : SIM.MOTOR;", "b(On := TRUE, Inertia := 0.000001);", 1000);
		r.cycle(5);
		expect(r.num("b.Speed")).toBeGreaterThan(2900);
		expect(r.num("b.Speed")).toBeLessThan(3001);
	});
	it("SPINDLE integrates the motion, stops at the ends, homes", () => {
		const r = new LibRun("b : SIM.SPINDLE; n : REAL := 600.0;", "b(Speed := n, Ratio := 2.0, Pitch := 5.0, MaxPos := 100.0);", 100);
		r.cycle(11);  // 10 steps of 0.1 s at 600 rpm / 2 * 5 mm = 25 mm/s
		expect(r.num("b.Pos")).toBeCloseTo(25, 3);
		expect(r.num("b.Velocity")).toBeCloseTo(25);
		r.run(5000);
		expect([r.num("b.Pos"), r.get("b.AtMax"), r.get("b.Blocked"), r.num("b.Velocity")]).toEqual([100, true, true, 0]);
		r.set("n", -600).cycle();
		expect(r.get("b.Blocked")).toBe(false);
		r.set("b.SetPos", 0).set("b.Set", true).cycle();
		expect(r.num("b.Pos")).toBeLessThan(0.01);
		expect(r.get("b.AtMin")).toBe(true);
	});
	it("ACTUATOR: 3-point and continuous", () => {
		const r = new LibRun("b : SIM.ACTUATOR;", "b();", 100).set("b.RunTime", 10_000).set("b.Open", true).cycle(51);
		expect(r.num("b.Pos")).toBeCloseTo(50, 3);
		r.set("b.Close", true).cycle(5);
		expect(r.get("b.Moving")).toBe(false);  // both: no motion
		r.set("b.Open", false).set("b.Close", false).set("b.Continuous", true).set("b.Y", 20).run(5000);
		expect([r.num("b.Pos"), r.get("b.AtClosed")]).toEqual([20, false]);
	});
});

describe("pneumatics", () => {
	it("COMPRESSOR: delivery falls with pressure, adiabatic power and temperature", () => {
		const r = one("COMPRESSOR").set("b.On", true).set("b.Pout", 5).run(10_000);
		expect(r.num("b.Speed")).toBeCloseTo(100, 1);
		expect(r.num("b.Flow")).toBeCloseTo(25, 1);
		const ratio = (5 + 1.01325) / 1.01325;
		const rk = ratio ** 0.2857143 - 1;
		expect(r.num("b.Power")).toBeCloseTo(3.5 * 101325 * 25 / 60000 * rk / 0.6, 0);
		expect(r.num("b.Tout")).toBeCloseTo(293.15 * (1 + rk / 0.6) - 273.15, 1);
		r.set("b.Pout", 12).cycle();
		expect([r.num("b.Flow"), r.get("b.Overpressure")]).toEqual([0, true]);
		r.set("b.On", false).run(1000);
		expect(r.num("b.Speed")).toBeCloseTo(100 * Math.exp(-1), 0);  // RunUp 1 s
	});
	it("TANK: filling by mass balance, safety valve, choked outflow", () => {
		const r = one("TANK").set("b.InFlow", 60).cycle(201);  // 60 Nl/min = 10 Nl in 10 s into 10 l: +1.013 bar
		expect(r.num("b.Pressure")).toBeCloseTo(1.01325, 2);
		expect(r.num("b.StoredAir")).toBeCloseTo(10, 1);
		r.run(200_000);
		expect([r.num("b.Pressure"), r.get("b.Relief")]).toEqual([11, true]);
		// valve open, no inflow: choked flow C * p1 while p1 >= 2 * pamb
		r.set("b.InFlow", 0).set("b.Valve", 100).cycle();
		expect(r.num("b.OutFlow")).toBeCloseTo(100 * (11 + 1.01325), -1);
		r.run(120_000);
		expect(r.num("b.Pressure")).toBeLessThan(0.01);
		expect(r.num("b.OutFlow")).toBeLessThan(20);
	});
	it("TANK stays stable with a tiny volume and a wide valve", () => {
		const r = new LibRun("b : SIM.TANK;", "b(InFlow := 100.0, Volume := 0.01, Valve := 100.0, ValveC := 1000.0);", 1000);
		r.cycle(5);
		const p = r.num("b.Pressure");
		expect(p).toBeGreaterThanOrEqual(0);
		expect(p).toBeLessThan(0.2);  // the balance of 100 Nl/min against the valve
	});
	it("VALVE: linear and equal-percentage characteristic", () => {
		const r = one("VALVE").set("b.Pos", 100).set("b.Dp", 1).cycle();
		expect(r.num("b.Flow")).toBeCloseTo(2.5);
		r.set("b.Pos", 50).cycle();
		expect(r.num("b.Kv")).toBeCloseTo(2.5 / Math.sqrt(50), 3);
		r.set("b.EqualPercentage", false).cycle();
		expect(r.num("b.Kv")).toBeCloseTo(1.25);
		r.set("b.Pos", 0.5).cycle();
		expect(r.num("b.Flow")).toBe(0);
	});
});

describe("heat", () => {
	it("HEATER: steady state from convection, exponential approach, energy", () => {
		const r = new LibRun("b : SIM.HEATER;", "b(On := TRUE, RatedPower := 100.0, Emissivity := 0.0);", 1000);
		r.run(3_600_000);
		expect(r.num("b.Temp")).toBeCloseTo(70, 1);  // 20 + 100 W / 2 W/K
		expect(r.num("b.Energy")).toBeCloseTo(100 * (3600 - 1) / 3600, 1);
		const t = new LibRun("b : SIM.HEATER;", "b(On := TRUE, RatedPower := 100.0, Emissivity := 0.0);", 100);
		t.run(125_100);  // tau = 250 J/K / 2 W/K = 125 s
		expect(t.num("b.Temp")).toBeCloseTo(20 + 50 * (1 - Math.exp(-1)), 0);
	});
	it("HEATER with radiation: the steady state balances P = hA dT + e s A (T^4 - Ta^4)", () => {
		const r = new LibRun("b : SIM.HEATER;", "b(On := TRUE);", 1000);
		r.run(7_200_000);
		const T = r.num("b.Temp") + 273.15, Ta = 293.15;
		expect(2 * (T - Ta) + 0.8 * 5.670374e-8 * 0.02 * (T ** 4 - Ta ** 4)).toBeCloseTo(1000, -1);
	});
	it("THERMALMASS: exact exponential, any cycle; without UA it integrates", () => {
		const r = new LibRun("b : SIM.THERMALMASS;", "b(Q1 := 100.0, Capacity := 10000.0, UA := 10.0);", 1000);
		r.cycle(1001);
		expect(r.num("b.Temp")).toBeCloseTo(30 - 10 * Math.exp(-1), 1);
		const i = new LibRun("b : SIM.THERMALMASS;", "b(Q1 := 100.0, Capacity := 10000.0, UA := 0.0);", 1000);
		i.cycle(101);
		expect(i.num("b.Temp")).toBeCloseTo(21, 3);
		expect(i.num("b.Stored")).toBeCloseTo(10, 2);
	});
	it("HEATTRANSFER: conduction, convection, radiation add up", () => {
		const r = one("HEATTRANSFER").set("b.T1", 100).set("b.T2", 20).set("b.UA", 1).set("b.Conductivity", 50).set("b.Thickness", 0.01)
			.set("b.Area", 0.1).set("b.Alpha", 10).cycle();
		expect(r.num("b.UAeff")).toBeCloseTo(1 + 500 + 1);
		expect(r.num("b.Q")).toBeCloseTo(502 * 80, 0);
		r.set("b.Emissivity", 1).set("b.UA", 0).set("b.Conductivity", 0).set("b.Alpha", 0).cycle();
		expect(r.num("b.Q")).toBeCloseTo(5.670374e-8 * 0.1 * (373.15 ** 4 - 293.15 ** 4), 1);
	});
	it("heater -> heat transfer -> thermal mass conserves energy", () => {
		const r = new LibRun("h : SIM.HEATER; x : SIM.HEATTRANSFER; m : SIM.THERMALMASS;",
			"h(On := TRUE, RatedPower := 200.0, HA := 0.0, Emissivity := 0.0, QExt := x.Q);\n" +
			"m(Q1 := x.Q, Capacity := 50000.0, UA := 0.0);\nx(T1 := h.Temp, T2 := m.Temp, UA := 5.0);", 100);
		r.run(600_000);
		const stored = (r.num("h.Temp") - 20) * 250 + (r.num("m.Temp") - 20) * 50000;
		expect(stored / 1000).toBeCloseTo(200 * 600 / 1000, -1);  // 120 kJ in (1 cycle of lag)
		expect(r.num("m.Temp")).toBeGreaterThan(21);
	});
	it("HEATEXCHANGER: energy balance and the effectiveness limits", () => {
		const r = one("HEATEXCHANGER").set("b.Tau", 0).cycle();
		const q = r.num("b.Q");
		expect(q).toBeCloseTo(0.1 * 4190 * (70 - r.num("b.THotOut")), -1);
		expect(q).toBeCloseTo(0.5 * 1006 * (r.num("b.TColdOut") - 10), -1);
		expect(r.num("b.Effectiveness")).toBeGreaterThan(0);
		expect(r.num("b.Effectiveness")).toBeLessThan(1);
		r.set("b.UA", 1e6).cycle();
		expect(r.num("b.Effectiveness")).toBeCloseTo(1, 2);
		r.set("b.MCold", 0).cycle();
		expect(r.num("b.Q")).toBe(0);
		r.set("b.Counterflow", false).set("b.MCold", 0.419 * 4190 / 1006).set("b.MHot", 0.419).set("b.UA", 1e6).cycle();
		expect(r.num("b.Effectiveness")).toBeCloseTo(0.5, 2);  // parallel flow, Cr = 1
	});
	it("PT1 and DEADTIME", () => {
		const p = new LibRun("b : SIM.PT1;", "b(In := 1.0, Tau := 2.0, K := 3.0);", 100).cycle(21);
		expect(p.num("b.Out")).toBeCloseTo(3 * (1 - Math.exp(-1)), 3);
		const d = new LibRun("b : SIM.DEADTIME; x : REAL;", "b(In := x, Delay := T#1s);", 10);
		d.cycle(10).set("x", 5).cycle(99);
		expect(d.num("b.Out")).toBe(0);
		d.cycle(2);
		expect(d.num("b.Out")).toBe(5);
	});
});
