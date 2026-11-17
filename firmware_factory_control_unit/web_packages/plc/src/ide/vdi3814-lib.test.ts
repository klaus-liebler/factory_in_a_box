// The VDI3814 library: the behaviour documented in docs/vdi3814-library.md, especially the edge cases.
// Cycle 50 ms unless a test says otherwise.
import { describe, expect, it } from "vitest";
import { LibRun } from "./lib-harness.ts";

const one = (type: string, body = "b();") => new LibRun(`b : VDI3814.${type};`, body);

describe("1.1 physical I/O", () => {
	it("AI scales, offsets, filters; a fault holds the last good value; out of service", () => {
		const r = one("AI").set("b.Raw", 2047.5).set("b.Offset", 1).cycle();
		expect(r.num("b.PV")).toBeCloseTo(51);
		r.set("b.Filter", 1).set("b.Raw", 4095).run(1000);  // one time constant: 63 %
		expect(r.num("b.PV")).toBeCloseTo(51 + 50 * (1 - Math.exp(-1)), 0);
		r.set("b.FaultHi", 4000).cycle();
		expect(r.get("b.Fault")).toBe(true);
		const held = r.num("b.PV");
		r.cycle(10);
		expect(r.num("b.PV")).toBe(held);
		r.set("b.OutOfService", true).set("b.Manual", -3).cycle();
		expect(r.num("b.PV")).toBe(-3);
	});
	it("AI with RawLo = RawHi gives EngLo + Offset (no division by zero)", () => {
		const r = one("AI").set("b.RawHi", 0).set("b.Raw", 7).cycle();
		expect(r.num("b.PV")).toBe(0);
	});
	it("AI faulty from the start keeps PV at 0 until a good value", () => {
		const r = one("AI").set("b.Raw", -5).set("b.FaultLo", 0).cycle(3);
		expect(r.get("b.Fault")).toBe(true);
		expect(r.num("b.PV")).toBe(0);
	});
	it("BI debounces, counts rising edges, accumulates hours", () => {
		const r = one("BI").set("b.Debounce", 200).set("b.In", true).cycle(3);
		expect(r.get("b.PV")).toBe(false);
		r.cycle(2);
		expect(r.get("b.PV")).toBe(true);
		r.set("b.In", false).cycle(2).set("b.In", true).cycle(2);  // a 100 ms gap is filtered out
		expect(r.get("b.PV")).toBe(true);
		expect(r.num("b.Changes")).toBe(1);
		r.set("b.Invert", true).cycle(10);
		expect(r.get("b.PV")).toBe(false);
		expect(r.num("b.ActiveTime")).toBeGreaterThan(0);
		r.set("b.ResetCount", true).cycle();
		expect(r.num("b.Changes")).toBe(0);
	});
	it("AO ramps, limits and scales; Min > Max is clamped by LIMIT's order", () => {
		const r = one("AO").set("b.Auto", 150).cycle();
		expect(r.num("b.PV")).toBe(100);  // the first call jumps (no ramp from 0)
		expect(r.num("b.Out")).toBeCloseTo(10);
		r.set("b.Ramp", 10).set("b.Auto", 0).run(1000);
		expect(r.num("b.PV")).toBeCloseTo(90, 0);
		r.set("b.ManualMode", true).set("b.Manual", 90).cycle();
		expect(r.num("b.PV")).toBeCloseTo(90, 0);
	});
	it("BO keeps the minimum on/off times; the first switch-on does not wait", () => {
		const r = one("BO").set("b.MinOn", 1000).set("b.MinOff", 2000).set("b.Auto", true).cycle();
		expect(r.get("b.Q")).toBe(true);
		r.set("b.Auto", false).run(500);
		expect(r.get("b.Q")).toBe(true);
		r.run(600);
		expect(r.get("b.Q")).toBe(false);
		r.set("b.Auto", true).run(1500);
		expect(r.get("b.Q")).toBe(false);
		r.run(600);
		expect(r.get("b.Q")).toBe(true);
		expect(r.num("b.Switches")).toBe(3);
	});
});

describe("1.2 values", () => {
	it("AV: default, Set/Reset edges, direct writes clamped", () => {
		const r = one("AV").set("b.Default", 20).set("b.Min", 10).set("b.Max", 30).cycle();
		expect(r.num("b.Value")).toBe(20);
		r.set("b.In", 25).set("b.Set", true).cycle();
		expect(r.num("b.Value")).toBe(25);
		expect(r.get("b.Changed")).toBe(true);
		r.cycle();
		expect(r.get("b.Changed")).toBe(false);
		r.set("b.Value", 99).cycle();  // HMI writes beyond Max
		expect(r.num("b.Value")).toBe(30);
		expect(r.get("b.Changed")).toBe(true);
		r.cycle().set("b.Value", 12).cycle();  // an operator's write within the limits is reported too
		expect([r.num("b.Value"), r.get("b.Changed")]).toEqual([12, true]);
		r.set("b.Reset", true).set("b.Set", false).cycle();
		expect(r.num("b.Value")).toBe(20);
	});
	it("BV: reset beats set beats toggle", () => {
		const r = one("BV").set("b.Toggle", true).cycle();
		expect(r.get("b.Value")).toBe(true);
		r.set("b.Toggle", false).cycle().set("b.Set", true).set("b.Reset", true).set("b.Toggle", true).cycle();
		expect(r.get("b.Value")).toBe(false);
	});
	it("MV stays within 1..States, also for a bad States", () => {
		const r = one("MV").set("b.Up", true).cycle().set("b.Up", false).cycle().set("b.Up", true).cycle().set("b.Up", false).cycle()
			.set("b.Up", true).cycle();
		expect(r.num("b.Value")).toBe(3);
		expect(r.get("b.Q3")).toBe(true);
		r.set("b.States", 0).cycle();
		expect(r.num("b.Value")).toBe(1);
	});
});

describe("1.3 complex", () => {
	it("CLOCK: date arithmetic, weekday, day of year, midnight and year change", () => {
		const r = one("CLOCK").set("b.Year", 2024).set("b.Month", 12).set("b.Day", 31).set("b.Hour", 23).set("b.Minute", 59).set("b.Second", 59).cycle();
		expect([r.num("b.Y"), r.num("b.Mo"), r.num("b.D"), r.num("b.Weekday"), r.num("b.DayOfYear")]).toEqual([2024, 12, 31, 2, 366]);
		r.run(1000);
		expect([r.num("b.Y"), r.num("b.Mo"), r.num("b.D"), r.num("b.H"), r.num("b.Weekday"), r.num("b.DayOfYear")]).toEqual([2025, 1, 1, 0, 3, 1]);
		r.set("b.Year", 2028).set("b.Month", 2).set("b.Day", 28).set("b.Hour", 12).set("b.Set", true).cycle().set("b.Set", false).set("b.Speed", 3600).run(12_000);
		expect([r.num("b.Mo"), r.num("b.D")]).toEqual([2, 29]);  // leap year
		expect(r.num("b.Weekday")).toBe(2);
	});
	it("SCH: periods, over midnight, On = Off, Saturday, holiday as Sunday", () => {
		const r = one("SCH").set("b.TimeOfDay", 7 * 3600_000).set("b.Weekday", 3).cycle();
		expect(r.get("b.Q")).toBe(true);
		r.set("b.TimeOfDay", 18 * 3600_000).cycle();
		expect(r.get("b.Q")).toBe(false);  // Off is excluded
		r.set("b.WorkOn2", 22 * 3600_000).set("b.WorkOff2", 2 * 3600_000).set("b.TimeOfDay", 1 * 3600_000).cycle();
		expect(r.get("b.Q")).toBe(true);
		r.set("b.Weekday", 6).cycle();
		expect(r.get("b.Q")).toBe(false);  // Saturday: On = Off = 0 -> never
		r.set("b.Weekday", 3).set("b.Holiday", true).set("b.SunOn", 0).set("b.SunOff", 2 * 3600_000).cycle();
		expect(r.get("b.Q")).toBe(true);
		expect(r.num("b.Value")).toBe(1);
	});
	it("CAL: German holidays from Easter, special days, a range over the year end", () => {
		const r = one("CAL").set("b.Year", 2026).set("b.Month", 4).set("b.Day", 3).cycle();  // Easter 2026: 5 April
		expect(r.get("b.Holiday")).toBe(true);  // Good Friday
		r.set("b.Day", 6).cycle();
		expect(r.get("b.Holiday")).toBe(true);  // Easter Monday
		r.set("b.Day", 5).cycle();
		expect(r.get("b.Q")).toBe(false);  // Easter Sunday is no extra day (a Sunday anyway)
		r.set("b.Month", 5).set("b.Day", 14).cycle();
		expect(r.get("b.Holiday")).toBe(true);  // Ascension
		r.set("b.Month", 1).set("b.Day", 3).set("b.RangeFrom", 1224).set("b.RangeTo", 106).cycle();
		expect(r.get("b.Special")).toBe(true);
		r.set("b.Month", 7).set("b.Day", 1).set("b.D3", 701).cycle();
		expect(r.get("b.Special")).toBe(true);
		r.set("b.GermanHolidays", false).set("b.Month", 10).set("b.Day", 3).set("b.D3", 0).cycle();
		expect(r.get("b.Q")).toBe(false);
	});
	it("NC: delay, acknowledgement, the four states, counting", () => {
		const r = one("NC").set("b.Delay", 100).set("b.Event", true).cycle();
		expect(r.get("b.Active")).toBe(false);
		r.cycle(2);
		expect(r.num("b.State")).toBe(1);
		r.set("b.Ack", true).cycle().set("b.Ack", false);
		expect(r.num("b.State")).toBe(2);
		expect(r.get("b.Lamp")).toBe(true);
		r.set("b.Event", false).cycle();
		expect(r.num("b.State")).toBe(0);
		r.set("b.Event", true).cycle(3).set("b.Event", false).cycle();
		expect(r.num("b.State")).toBe(3);  // gone, not acknowledged
		expect(r.get("b.Alarm")).toBe(true);
		r.set("b.Ack", true).cycle();
		expect(r.num("b.State")).toBe(0);
		expect(r.num("b.Count")).toBe(2);
	});
	it("NC: an acknowledgement before the event does not pre-acknowledge it", () => {
		const r = one("NC").set("b.Ack", true).cycle().set("b.Event", true).cycle();
		expect(r.num("b.State")).toBe(1);
	});
	it("LOG: stores per interval, statistics, wraps after 60 values", () => {
		const r = one("LOG").set("b.Interval", 100).set("b.In", 5).cycle();
		expect(r.num("b.Count")).toBe(1);
		for (let i = 0; i < 70; i++) r.set("b.In", i).cycle(2);
		expect(r.num("b.Count")).toBe(60);
		expect(r.num("b.Max")).toBe(69);
		expect(r.num("b.Min")).toBe(10);
		r.set("b.Clear", true).cycle();
		expect(r.num("b.Count")).toBe(0);
	});
});

describe("2.1/2.2 logic, monitoring and control", () => {
	it("MEAN4 uses N inputs, N clamped to 1..4", () => {
		const r = one("MEAN4").set("b.In1", 1).set("b.In2", 3).set("b.In3", 8).set("b.N", 3).cycle();
		expect([r.num("b.Avg"), r.num("b.Min"), r.num("b.Max")]).toEqual([4, 1, 8]);
		r.set("b.N", 0).cycle();
		expect(r.num("b.Avg")).toBe(1);
	});
	it("TONOF delays both edges; a pulse shorter than TOn does nothing", () => {
		const r = one("TONOF").set("b.TOn", 200).set("b.TOff", 300).set("b.IN", true).cycle(2).set("b.IN", false).cycle(10);
		expect(r.get("b.Q")).toBe(false);
		r.set("b.IN", true).cycle(5);
		expect(r.get("b.Q")).toBe(true);
		r.set("b.IN", false).cycle(5);
		expect(r.get("b.Q")).toBe(true);
		r.cycle(2);
		expect(r.get("b.Q")).toBe(false);
	});
	it("LIMITMON: hysteresis below High, delay, disable resets", () => {
		const r = one("LIMITMON").set("b.High", 50).set("b.Hyst", 5).set("b.In", 51).cycle();
		expect(r.get("b.QHigh")).toBe(true);
		r.set("b.In", 46).cycle();
		expect(r.get("b.QHigh")).toBe(true);
		r.set("b.In", 44).cycle();
		expect(r.get("b.QHigh")).toBe(false);
		r.set("b.Low", 0).set("b.In", -1).set("b.Delay", 500).cycle(5);
		expect(r.get("b.QLow")).toBe(false);
		r.cycle(6);
		expect(r.get("b.Q")).toBe(true);
		r.set("b.Enable", false).cycle();
		expect(r.get("b.Q")).toBe(false);
	});
	it("RUNHOURS: hours, starts, service interval, preset", () => {
		const r = new LibRun("b : VDI3814.RUNHOURS;", "b();", 1000);
		r.set("b.Run", true).set("b.ServiceInterval", 0.0005).run(4000);
		expect(r.num("b.Hours")).toBeCloseTo(3 / 3600, 6);  // the first call has no time step
		expect(r.get("b.ServiceDue")).toBe(true);
		r.set("b.ServiceDone", true).cycle();
		expect(r.num("b.SinceService")).toBeCloseTo(1 / 3600, 6);
		r.set("b.Run", false).cycle().set("b.Run", true).cycle();
		expect(r.num("b.Starts")).toBe(2);
		r.set("b.Preset", 1000).set("b.SetPreset", true).set("b.Run", false).cycle();
		expect(r.num("b.Hours")).toBe(1000);
	});
	it("EVENTCOUNT: limit and frequency per window", () => {
		const r = one("EVENTCOUNT").set("b.Limit", 3).set("b.Window", 1000).set("b.MaxPerWindow", 2);
		for (let i = 0; i < 3; i++) r.set("b.In", true).cycle().set("b.In", false).cycle();
		expect(r.get("b.LimitReached")).toBe(true);
		expect(r.get("b.Frequent")).toBe(true);
		r.run(1000);
		expect(r.num("b.InWindow")).toBe(0);
		expect(r.num("b.Count")).toBe(3);
	});
	it("CMDMON latches both faults until Reset", () => {
		const r = one("CMDMON").set("b.OnDelay", 200).set("b.Cmd", true).cycle(5);
		expect(r.get("b.NoFeedback")).toBe(true);
		r.set("b.Feedback", true).cycle();
		expect(r.get("b.Fault")).toBe(true);
		r.set("b.Reset", true).cycle();
		expect(r.get("b.Fault")).toBe(false);
	});
	it("DEVSTATUS: the highest state wins", () => {
		const r = one("DEVSTATUS").set("b.Cmd", true).cycle();
		expect(r.num("b.Status")).toBe(2);
		r.set("b.Running", true).set("b.LocalManual", true).set("b.Fault", true).cycle();
		expect(r.num("b.Status")).toBe(5);
	});
	it("MOTOR: modes, release, feedback fault latches, overload, reset only without the cause", () => {
		const r = one("MOTOR").set("b.Auto", true).set("b.FbDelay", 200).cycle();
		expect(r.get("b.Q")).toBe(true);
		r.cycle(5);
		expect([r.get("b.Fault"), r.num("b.FaultCode"), r.get("b.Q")]).toEqual([true, 2, false]);
		r.set("b.Feedback", true).cycle(5);  // feedback without command - the first fault keeps its code
		expect(r.num("b.FaultCode")).toBe(2);
		r.set("b.Feedback", false).set("b.Reset", true).cycle().set("b.Reset", false).set("b.Feedback", true).cycle();
		expect([r.get("b.Running"), r.num("b.Status")]).toEqual([true, 1]);
		r.set("b.Release", false).cycle();
		expect(r.get("b.Q")).toBe(false);
		r.set("b.Release", true).set("b.Overload", true).cycle().set("b.Reset", true).cycle();
		expect(r.num("b.FaultCode")).toBe(1);  // reset refused while the protection is tripped
		r.set("b.Mode", 0).set("b.Overload", false).set("b.Feedback", false).set("b.Reset", false).cycle().set("b.Reset", true).cycle();
		expect([r.get("b.Fault"), r.get("b.Q")]).toEqual([false, false]);
	});
	it("ANTISEIZE exercises after the idle interval only", () => {
		const r = one("ANTISEIZE").set("b.Interval", 1000).set("b.Duration", 200).run(1000);
		expect(r.get("b.Exercising")).toBe(false);
		r.cycle(2);
		expect(r.get("b.Q")).toBe(true);
		r.run(250);
		expect(r.get("b.Q")).toBe(false);
		r.set("b.Cmd", true).run(2000).set("b.Cmd", false).run(500);
		expect(r.get("b.Exercising")).toBe(false);  // the idle time restarted after running
	});
	it("ASWITCH ramps between values; an unknown selector picks In0", () => {
		const r = one("ASWITCH").set("b.In0", 10).set("b.In2", 30).set("b.Ramp", 20).cycle();
		expect(r.num("b.Out")).toBe(10);
		r.set("b.Sel", 2).run(500);
		expect(r.num("b.Out")).toBeCloseTo(20, 0);
		r.set("b.Sel", 9).run(2000);
		expect(r.num("b.Out")).toBeCloseTo(10);
	});
	it("SEQUENCE adds and removes stages with delay, rotates the lead, skips unavailable ones", () => {
		const r = one("SEQUENCE").set("b.Stages", 3).set("b.OnDelay", 100).set("b.OffDelay", 100).set("b.Demand", 80).run(1000);
		expect(r.num("b.Active")).toBe(3);
		r.set("b.Demand", 50).run(1000);
		expect(r.num("b.Active")).toBe(2);  // stage 3 off at <= 66.7 %
		expect([r.get("b.Q1"), r.get("b.Q2"), r.get("b.Q3")]).toEqual([true, true, false]);
		r.set("b.Rotate", true).cycle();
		expect([r.get("b.Q1"), r.get("b.Q2"), r.get("b.Q3")]).toEqual([false, true, true]);
		r.set("b.Avail3", false).cycle();
		expect([r.get("b.Q1"), r.get("b.Q2"), r.get("b.Q3")]).toEqual([true, true, false]);
		r.set("b.Avail1", false).set("b.Avail2", false).cycle();
		expect(r.num("b.Active")).toBe(2);  // demanded stages stay counted, no unit left to run
		expect([r.get("b.Q1"), r.get("b.Q2"), r.get("b.Q3")]).toEqual([false, false, false]);
	});
	it("FROST: proportional valve, trip latches, reset needs warmth", () => {
		const r = one("FROST").set("b.Temp", 6.5).cycle();
		expect(r.num("b.Y")).toBeCloseTo(50);
		expect(r.get("b.PreAlarm")).toBe(true);
		r.set("b.Temp", 4).run(2100);
		expect([r.get("b.Trip"), r.get("b.Release"), r.num("b.Y")]).toEqual([true, false, 100]);
		r.set("b.Temp", 6).set("b.Reset", true).cycle();
		expect(r.get("b.Trip")).toBe(true);  // not above TripLimit + Hyst
		r.set("b.Reset", false).set("b.Temp", 7.5).cycle().set("b.Reset", true).cycle();
		expect(r.get("b.Trip")).toBe(false);
	});
	it("SAFETY: first out, reset only with every contact closed", () => {
		const r = one("SAFETY").set("b.OK3", false).cycle().set("b.OK1", false).cycle();
		expect([r.get("b.Trip"), r.num("b.FirstOut")]).toEqual([true, 3]);
		r.set("b.OK3", true).set("b.OK1", true).set("b.Reset", true).cycle();
		expect([r.get("b.Release"), r.num("b.FirstOut")]).toEqual([true, 0]);
	});
	it("PRIORITY: the smallest enabled level wins, else Default", () => {
		const r = one("PRIORITY").set("b.Default", 7).cycle();
		expect([r.num("b.Out"), r.num("b.Level")]).toEqual([7, 0]);
		r.set("b.En8", true).set("b.V8", 3).set("b.En5", true).set("b.V5", 0).cycle();
		expect([r.num("b.Out"), r.num("b.Level"), r.get("b.Q")]).toEqual([0, 5, false]);
	});
});

describe("2.3 control", () => {
	it("PID: P-only with offset, PI reaches zero error, anti-windup, bumpless manual", () => {
		const r = one("PID").set("b.SP", 20).set("b.PV", 18).set("b.Kp", 10).set("b.Offset", 30).cycle();
		expect(r.num("b.Y")).toBeCloseTo(50);
		// PI against a first-order plant: PV := PV + (Y/10 - PV) * dt/5 s
		const p = new LibRun("c : VDI3814.PID; pv : REAL;", "c(SP := 5.0, PV := pv, Kp := 2.0, Tn := 5.0);\npv := pv + (c.Y / 10.0 - pv) * 0.05 / 5.0;");
		p.run(120_000);
		expect(p.num("pv")).toBeCloseTo(5, 1);
		// windup: an unreachable setpoint keeps Y at YMax; the integral does not grow beyond it
		const w = one("PID").set("b.SP", 100).set("b.Kp", 1).set("b.Tn", 1).run(10_000);
		expect(w.num("b.Y")).toBe(100);
		w.set("b.SP", 0).cycle(2);
		expect(w.num("b.Y")).toBeLessThan(5);  // leaves the limit at once
		const m = one("PID").set("b.SP", 10).set("b.Kp", 1).set("b.Tn", 10).set("b.Manual", true).set("b.YManual", 40).cycle(3)
			.set("b.Manual", false).cycle();
		expect(m.num("b.Y")).toBeCloseTo(40, 0);  // no jump
		m.set("b.Enable", false).set("b.YOff", 0).cycle();
		expect(m.num("b.Y")).toBe(0);
		m.set("b.Reverse", true).set("b.Enable", true).set("b.PV", 15).set("b.SP", 10).set("b.Deadband", 6).set("b.Tn", 0).cycle();
		expect(m.num("b.E")).toBe(0);
	});
	it("SPCURVE: linear between points, constant beyond, shift and limits", () => {
		const r = one("SPCURVE").set("b.X", -7.5).cycle();
		expect(r.num("b.Y")).toBeCloseTo(61);
		r.set("b.X", -30).cycle();
		expect(r.num("b.Y")).toBe(70);
		r.set("b.X", 30).set("b.Shift", 3).set("b.YMax", 24).cycle();
		expect(r.num("b.Y")).toBe(24);
	});
	it("OUTCONT: limits, inversion, safety position over the limits", () => {
		const r = one("OUTCONT").set("b.Y", 50).set("b.OutLo", 2).cycle();
		expect(r.num("b.Out")).toBeCloseTo(6);
		r.set("b.Invert", true).cycle();
		expect(r.num("b.Out")).toBeCloseTo(6);
		r.set("b.Y", 0).set("b.YMin", 20).cycle();
		expect(r.num("b.Pos")).toBe(20);
		r.set("b.Safe", true).set("b.YSafe", 0).cycle();
		expect(r.num("b.Pos")).toBe(0);
	});
	it("OUT2P: thresholds and minimum times", () => {
		const r = one("OUT2P").set("b.Y", 61).cycle();
		expect(r.get("b.Q")).toBe(true);
		r.set("b.Y", 50).cycle();
		expect(r.get("b.Q")).toBe(true);
		r.set("b.MinOn", 1000).set("b.Y", 30).cycle();
		expect(r.get("b.Q")).toBe(true);
		r.run(1000);
		expect(r.get("b.Q")).toBe(false);
	});
	it("OUT3P: initial sync, positioning within the dead band, end-stop sync", () => {
		const s = new LibRun("b : VDI3814.OUT3P; y : REAL := 50.0;", "b(Y := y, RunTime := T#10s);", 100);
		s.cycle();
		expect([s.get("b.Close"), s.get("b.Syncing")]).toEqual([true, true]);
		s.run(12_100);  // 100 % + 2 * 10 % of the runtime
		expect(s.get("b.Syncing")).toBe(false);
		s.run(6000);
		expect(s.num("b.Pos")).toBeGreaterThan(48);
		expect(s.num("b.Pos")).toBeLessThan(52);
		expect([s.get("b.Open"), s.get("b.Close")]).toEqual([false, false]);
		s.set("y", 51).run(1000);
		expect(s.get("b.Open")).toBe(false);  // within the dead band
		// regression: the end-stop sync ran past 100 %, the model was clamped back to 100 %, and with Y still at
		// 100 % that looked like a new deviation - the drive kept opening forever (sync + clamp + held Y)
		s.set("y", 100).run(7000);
		expect(s.num("b.Pos")).toBe(100);
		expect(s.get("b.Open")).toBe(false);
		s.run(10_000);
		expect(s.get("b.Open")).toBe(false);
		s.set("y", 0).run(13_000);
		expect([s.num("b.Pos"), s.get("b.Close")]).toEqual([0, false]);
		s.run(10_000);
		expect(s.get("b.Close")).toBe(false);
	});
	it("OUT3PVAR: pulse length from the deviation, limited to MinPulse..Period", () => {
		const r = one("OUT3PVAR").set("b.E", 5).set("b.Kp", 1).cycle();
		expect(r.num("b.Pulse")).toBe(6000);
		expect(r.get("b.Open")).toBe(true);
		r.set("b.E", -100).run(10_000);
		expect(r.num("b.Pulse")).toBe(10_000);
		expect(r.get("b.Close")).toBe(true);
		r.set("b.E", 0.01).set("b.Deadband", 0).run(10_000);
		expect(r.num("b.Pulse")).toBe(200);
	});
	it("PWM: duty cycle, minimum pulse at both ends", () => {
		const r = one("PWM").set("b.Y", 30).set("b.Period", 1000).cycle();
		expect(r.num("b.OnTime")).toBe(300);
		let on = 0;
		for (let i = 0; i < 20; i++) on += r.cycle().get("b.Q") ? 1 : 0;
		expect(on).toBe(6);
		r.set("b.MinPulse", 100).set("b.Y", 5).run(1000);
		expect(r.num("b.OnTime")).toBe(0);
		r.set("b.Y", 95).run(1000);
		expect(r.num("b.OnTime")).toBe(1000);
	});
	it("LIMITSP: limits and rates", () => {
		const r = one("LIMITSP").set("b.Hi", 50).set("b.In", 80).cycle();
		expect([r.num("b.Out"), r.get("b.AtHi")]).toEqual([50, true]);
		r.set("b.RateDown", 10).set("b.In", 0).run(1000);
		expect(r.num("b.Out")).toBeCloseTo(40, 0);
	});
	it("PARAMSET: out-of-range selector takes set 1", () => {
		const r = one("PARAMSET").set("b.A1", 1).set("b.A3", 3).set("b.Sel", 3).cycle();
		expect(r.num("b.A")).toBe(3);
		r.set("b.Sel", 7).cycle();
		expect(r.num("b.A")).toBe(1);
	});
});

describe("2.4 optimisation", () => {
	it("ENERGYLEVEL: levels, presence, window", () => {
		const r = one("ENERGYLEVEL").set("b.Level", 2).cycle();
		expect([r.num("b.HeatSP"), r.num("b.CoolSP")]).toEqual([19, 26]);
		r.set("b.Presence", true).cycle();
		expect(r.num("b.Active")).toBe(1);
		r.set("b.Level", 3).cycle();
		expect(r.num("b.Active")).toBe(3);  // presence does not end economy
		r.set("b.Window", true).cycle();
		expect([r.num("b.HeatSP"), r.num("b.CoolSP")]).toEqual([7, 35]);
		r.set("b.Window", false).set("b.Level", 9).cycle();
		expect(r.num("b.Active")).toBe(4);
	});
	it("HEATRECOVERY: heating, cooling, nothing when useless, icing", () => {
		const r = one("HEATRECOVERY").set("b.Toa", 0).set("b.Tex", 22).set("b.SupplySP", 7.7).cycle();
		expect(r.num("b.Y")).toBeCloseTo(50, 0);
		expect(r.num("b.Tsupply")).toBeCloseTo(7.7, 1);
		r.set("b.Toa", 30).set("b.Tex", 24).set("b.SupplySP", 18).cycle();
		expect([r.get("b.Cooling"), r.num("b.Y")]).toEqual([true, 100]);
		r.set("b.Tex", 31).cycle();
		expect(r.num("b.Y")).toBe(0);
		r.set("b.Toa", -10).set("b.Tex", 22).set("b.SupplySP", 20).cycle();
		expect(r.num("b.Y")).toBe(50);
	});
	it("HXSETPOINT: psychrometrics against reference values", () => {
		const r = one("HXSETPOINT").set("b.T", 20).set("b.RH", 50).cycle();
		expect(r.num("b.X")).toBeCloseTo(7.26, 1);
		expect(r.num("b.H")).toBeCloseTo(38.5, 0);
		expect(r.num("b.DewPoint")).toBeCloseTo(9.3, 0);
		expect([r.get("b.Humidify"), r.get("b.Dehumidify")]).toEqual([false, false]);
		r.set("b.RH", 20).cycle();
		expect(r.get("b.Humidify")).toBe(true);
		expect(r.num("b.RHSet")).toBeCloseTo(41.5, 1);
	});
	it("OPTSTART: starts Lead before occupancy and learns the rate", () => {
		const r = one("OPTSTART").set("b.Troom", 17).set("b.TimeOfDay", 4 * 3600_000).cycle();
		expect(r.num("b.Lead")).toBe(2 * 3600_000);
		expect(r.get("b.Start")).toBe(false);
		r.set("b.TimeOfDay", 5 * 3600_000 + 60_000).cycle();
		expect(r.get("b.Start")).toBe(true);
		r.set("b.Troom", 21).set("b.TimeOfDay", 6 * 3600_000 + 60_000).cycle();  // reached in 1 h: 4 K/h
		expect(r.num("b.LearnedRate")).toBeCloseTo(0.7 * 2 + 0.3 * 4, 3);
		r.set("b.TimeOfDay", 7 * 3600_000).cycle();
		expect(r.get("b.Start")).toBe(false);
	});
	it("NIGHTCOOL: conditions with hysteresis", () => {
		const r = one("NIGHTCOOL").set("b.Night", true).set("b.Troom", 25).set("b.Toa", 15).cycle();
		expect(r.get("b.Q")).toBe(true);
		r.set("b.Toa", 21.5).cycle();
		expect(r.get("b.Q")).toBe(true);
		r.set("b.Toa", 22.5).cycle();
		expect(r.get("b.Q")).toBe(false);
	});
});

describe("2.5 lighting, 2.6 sun protection", () => {
	it("LIGHT: short press toggles, long press dims, absence switches off", () => {
		const r = one("LIGHT").set("b.Button", true).cycle(2).set("b.Button", false).cycle();
		expect([r.get("b.Q"), r.num("b.Level")]).toEqual([true, 100]);
		r.set("b.Button", true).run(1500).set("b.Button", false).cycle();
		expect(r.get("b.Q")).toBe(true);
		expect(r.num("b.Level")).toBeLessThan(80);
		r.set("b.Presence", false).set("b.AbsenceDelay", 1000).run(1100);
		expect([r.get("b.Q"), r.num("b.Level")]).toEqual([false, 0]);
	});
	it("STAIRLIGHT: duration, retrigger, warning blink, permanent light", () => {
		const r = one("STAIRLIGHT").set("b.Duration", 5000).set("b.Warn", 1000).set("b.Trigger", true).cycle().set("b.Trigger", false).run(3000);
		expect(r.get("b.Q")).toBe(true);
		r.set("b.Trigger", true).cycle().set("b.Trigger", false).run(4100);
		expect([r.get("b.Q"), r.get("b.Warning")]).toEqual([false, true]);  // the warning blink
		r.run(600);
		expect([r.get("b.Q"), r.num("b.Level")]).toEqual([true, 50]);
		r.run(600);
		expect(r.get("b.Q")).toBe(false);
		r.set("b.Trigger", true).run(2100).set("b.Trigger", false).run(10_000);
		expect(r.get("b.Permanent")).toBe(true);
		r.set("b.Trigger", true).cycle().set("b.Trigger", false).cycle();
		expect(r.get("b.Q")).toBe(false);
	});
	it("DAYLIGHT and CONSTLIGHT", () => {
		const d = one("DAYLIGHT").set("b.Lux", 100).set("b.Delay", 100).run(200);
		expect(d.get("b.Q")).toBe(true);
		d.set("b.Lux", 400).run(1000);
		expect(d.get("b.Q")).toBe(true);
		// constant light against a room: lux = daylight + 8 * level
		const c = new LibRun("b : VDI3814.CONSTLIGHT; day : REAL := 100.0;", "b(Lux := day + 8.0 * b.Level);");
		c.run(30_000);
		expect(c.num("b.Level")).toBeCloseTo(50, 0);
		c.set("day", 2000).set("b.OffDelay", 1000).run(60_000);
		expect(c.get("b.Q")).toBe(false);
	});
	it("SUNBLIND: priorities, manual timeout, drive with slats", () => {
		const r = one("SUNBLIND").set("b.AutoPos", 100).set("b.AutoSlat", 50).set("b.TravelTime", 2000).set("b.SlatTime", 500).cycle();
		expect(r.get("b.Down")).toBe(true);
		r.run(400);
		expect(r.num("b.ActPos")).toBeLessThan(5);  // the slats close first
		r.run(5000);
		expect(r.num("b.ActPos")).toBe(100);
		expect(r.num("b.ActSlat")).toBeCloseTo(50, -1);
		r.set("b.ManPos", 20).set("b.ManSet", true).set("b.ManTimeout", 1000).cycle();
		expect(r.num("b.Mode")).toBe(1);
		r.set("b.Protect", true).cycle();
		expect([r.num("b.Mode"), r.num("b.Pos")]).toEqual([2, 0]);
		r.set("b.Protect", false).run(1100);
		expect(r.num("b.Mode")).toBe(0);
	});
	it("SUNDUSK, THERMOAUTO, WEATHERPROT", () => {
		const s = one("SUNDUSK").set("b.Lux", 40000).set("b.SunOnDelay", 100).run(200);
		expect(s.get("b.Sun")).toBe(true);
		const t = one("THERMOAUTO").set("b.Sun", true).set("b.Troom", 26).cycle();
		expect([t.get("b.Close"), t.num("b.Pos")]).toEqual([true, 100]);
		t.set("b.Occupied", true).cycle();
		expect(t.get("b.Active")).toBe(false);
		const w = one("WEATHERPROT").set("b.Wind", 12).set("b.Release", 1000).cycle();
		expect(w.get("b.Protect")).toBe(true);
		w.set("b.Wind", 8).run(2000);
		expect(w.get("b.WindAlarm")).toBe(true);  // hysteresis: not below 7 m/s
		w.set("b.Wind", 6).run(900);
		expect(w.get("b.Protect")).toBe(true);
		w.run(200);
		expect(w.get("b.Protect")).toBe(false);
		w.set("b.Rain", true).set("b.Toa", 1).cycle().set("b.Rain", false).cycle();
		expect(w.get("b.FrostAlarm")).toBe(true);
	});
	it("SUNPOS against the NOAA calculator", () => {
		// 21 June, solar noon in Berlin (52.52 N, 13.40 E) at 13:08 CEST: elevation 60.9°, azimuth 180°
		const r = one("SUNPOS").set("b.DayOfYear", 172).set("b.TimeOfDay", (13 * 60 + 8) * 60_000).set("b.UtcOffset", 2)
			.set("b.Latitude", 52.52).set("b.Longitude", 13.4).cycle();
		expect(r.num("b.Elevation")).toBeCloseTo(60.9, 0);
		expect(Math.abs(r.num("b.Azimuth") - 180)).toBeLessThan(1.5);
		r.set("b.TimeOfDay", 6 * 3600_000).cycle();  // morning: east
		expect(r.num("b.Azimuth")).toBeLessThan(90);
		r.set("b.TimeOfDay", 1 * 3600_000).cycle();
		expect(r.get("b.Day")).toBe(false);
	});
});
