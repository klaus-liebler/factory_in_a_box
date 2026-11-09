// The FCU library blocks against the device's rules for command registers (src/plc.cpp, copy_inputs /
// copy_outputs): before each cycle register -> variable, after it variable -> register only when the
// program changed it, and only while the control source is IEC. The "firmware" here is the test,
// answering in the register between cycles like io (homing) and roarm (missions).
import { describe, expect, it } from "vitest";
import { compile, type DebugInfo } from "../compiler/compiler.ts";
import { formatDiagnostic } from "../compiler/diagnostics.ts";
import { IoKind } from "../image.ts";
import { Vm } from "../vm.ts";
import { COMMAND_VARS, ioBindings, ioMapHash } from "./io-list.ts";
import { deviceSources } from "./project.ts";
import { readValue, writeValue, type Value } from "./values.ts";

const HOMING1 = 69, MISSION = 110;

const PROGRAM = `PROGRAM PLC_PRG
VAR
    home : FCU_StepperHome;
    mission : FCU_RoArmRunMission;
    goHome : BOOL;
    axis : UINT := 1;
    goMission : BOOL;
    index : INT := 1;
END_VAR
home(Axis := axis, Execute := goHome);
mission(Index := index, Execute := goMission);
END_PROGRAM
`;

/** The program on a device: holding registers, the control source, the cycle with the copy rules. */
class Device {
	readonly holding = new Map<number, number>();
	iec = true;
	private readonly vm: Vm;
	private readonly debug: DebugInfo;
	private readonly data = new Uint8Array(32 * 1024);
	private readonly dv = new DataView(this.data.buffer);
	private t = 0;

	constructor() {
		const r = compile(deviceSources("PLC_PRG", PROGRAM), undefined, { io: ioBindings(), ioMapHash: ioMapHash() });
		if (!r.ok) throw new Error(r.diagnostics.map(formatDiagnostic).join("\n"));
		this.vm = new Vm(this.data);
		expect(this.vm.load(r.image!)).toBe("none");
		this.debug = r.debug!;
	}

	get(path: string): Value | null {
		return readValue(this.data, this.debug.resolve(path)!);
	}
	set(path: string, v: Value): void {
		writeValue(this.data, this.debug.resolve(path)!, v);
	}
	reg(r: number): number {
		return this.holding.get(r) ?? 0;
	}

	cycle(n = 1): void {
		for (let i = 0; i < n; i++) {
			const commands = this.vm.io.filter((b) => b.kind === IoKind.Command);
			const before = commands.map((b) => {
				const v = this.reg(b.reg);
				this.dv.setUint16(b.addr, v, true);
				return v;
			});
			expect(this.vm.runCycle(0, (this.t += 50)).state).not.toBe("trapped");
			if (!this.iec) continue;
			commands.forEach((b, k) => {
				const v = this.dv.getUint16(b.addr, true);
				if (v !== before[k]) this.holding.set(b.reg, v);
			});
		}
	}

	outputs(fb: string): { done: Value | null; busy: Value | null; error: Value | null; id: Value | null } {
		return { done: this.get(`PLC_PRG.${fb}.Done`), busy: this.get(`PLC_PRG.${fb}.Busy`), error: this.get(`PLC_PRG.${fb}.Error`),
			id: this.get(`PLC_PRG.${fb}.ErrorID`) };
	}
}

describe("FCU library: command registers", () => {
	it("binds the schema's command registers (kind command), and they are no I/O variables", () => {
		expect(COMMAND_VARS.map((c) => [c.name, c.register, c.type])).toEqual([
			["__Stepper1StartHoming", 69, "UINT"], ["__Stepper2StartHoming", 70, "UINT"], ["__RoArmMissionControl", 110, "INT"]]);
		const commands = ioBindings().filter((b) => b.kind === "command");
		expect(commands.map((b) => b.register)).toEqual([69, 70, 110]);
	});

	it("FCU_StepperHome: the start reaches the register once, the answer 0 is Done", () => {
		const d = new Device();
		d.set("PLC_PRG.goHome", true);
		d.cycle();
		expect(d.reg(HOMING1)).toBe(1);
		expect(d.outputs("home")).toMatchObject({ busy: true, done: false });
		d.cycle(5);
		expect(d.outputs("home")).toMatchObject({ busy: true, done: false });
		d.holding.set(HOMING1, 0); // io: homed
		d.cycle();
		expect(d.outputs("home")).toEqual({ done: true, busy: false, error: false, id: 0 });
		d.cycle(5);
		expect(d.reg(HOMING1)).toBe(0); // written on the edge only: the answer stays
		expect(d.outputs("home").done).toBe(true); // while Execute is TRUE
		d.set("PLC_PRG.goHome", false);
		d.cycle();
		expect(d.outputs("home")).toEqual({ done: false, busy: false, error: false, id: 0 });
	});

	it("FCU_StepperHome: an error answer at once (TMC2209 does not answer: 3)", () => {
		const d = new Device();
		d.set("PLC_PRG.goHome", true);
		d.cycle();
		d.holding.set(HOMING1, 3); // io answers before the next cycle sees the 1
		d.cycle();
		expect(d.outputs("home")).toEqual({ done: false, busy: false, error: true, id: 3 });
	});

	it("FCU_StepperHome: no such axis", () => {
		const d = new Device();
		d.set("PLC_PRG.axis", 3);
		d.set("PLC_PRG.goHome", true);
		d.cycle();
		expect(d.outputs("home")).toEqual({ done: false, busy: false, error: true, id: 0x100 });
		expect(d.reg(HOMING1)).toBe(0);
	});

	// Regression: the register's idle value 0 is also the answer "done". While the control source is
	// the fieldbus the start never reaches the register, which stays 0 - the block must not read that
	// as success (it takes 0 only after it has seen its start value in the register).
	it("FCU_StepperHome: control source fieldbus - nothing starts, no false Done", () => {
		const d = new Device();
		d.iec = false;
		d.set("PLC_PRG.goHome", true);
		d.cycle(10);
		expect(d.reg(HOMING1)).toBe(0);
		expect(d.outputs("home")).toMatchObject({ busy: true, done: false, error: false });
	});

	it("FCU_RoArmRunMission: runs, then Done; a missing mission is ErrorID 1", () => {
		const d = new Device();
		d.set("PLC_PRG.index", 4);
		d.set("PLC_PRG.goMission", true);
		d.cycle();
		expect(d.reg(MISSION)).toBe(4);
		d.cycle(3);
		expect(d.outputs("mission")).toMatchObject({ busy: true, done: false });
		d.holding.set(MISSION, 0); // roarm: mission done
		d.cycle();
		expect(d.outputs("mission")).toEqual({ done: true, busy: false, error: false, id: 0 });

		d.set("PLC_PRG.goMission", false);
		d.cycle();
		d.set("PLC_PRG.index", 9);
		d.set("PLC_PRG.goMission", true);
		d.cycle();
		expect(d.reg(MISSION)).toBe(9);
		d.holding.set(MISSION, 0xffff); // roarm: -1, not found
		d.cycle();
		expect(d.outputs("mission")).toEqual({ done: false, busy: false, error: true, id: 1 });
	});
});
