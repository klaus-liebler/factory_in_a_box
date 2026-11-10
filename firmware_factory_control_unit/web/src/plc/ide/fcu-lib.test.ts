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

const HOMING1 = 69, MISSION = 110, MALIBOTTI = 111;

const PROGRAM = `PROGRAM PLC_PRG
VAR
    home : FCU_StepperHome;
    mission : FCU_RoArmRunMission;
    goHome : BOOL;
    axis : UINT := 1;
    goMission : BOOL;
    index : INT := 1;
    move : FCU_StepperMoveAbsolute;
    goMove : BOOL;
    moveAxis : UINT := 2;
    goal : DINT;
    speed : UINT;
    bot : FCU_MalibottiRunMission;
    goBot : BOOL;
    botMission : INT := 1;
END_VAR
home(Axis := axis, Execute := goHome);
mission(Index := index, Execute := goMission);
bot(Mission := botMission, Execute := goBot);
move(Axis := moveAxis, Execute := goMove, Position := goal, Velocity := speed);
END_PROGRAM
`;

/** The program on a device: holding and input registers (16-bit words), the control source, the cycle
 *  with the copy rules of src/plc.cpp (32-bit values high word first). */
class Device {
	readonly holding = new Map<number, number>();
	readonly input = new Map<number, number>();
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
	/** A 32-bit holding value (two registers, high word first). */
	reg32(r: number): number {
		return ((this.reg(r) << 16) | this.reg(r + 1)) | 0;
	}
	setInput32(r: number, v: number): void {
		this.input.set(r, (v >>> 16) & 0xffff);
		this.input.set(r + 1, v & 0xffff);
	}

	cycle(n = 1): void {
		for (let i = 0; i < n; i++) {
			// registers -> variables: inputs, parameters, the commands' answers
			const before = new Map<number, number>();
			for (const b of this.vm.io) {
				if (b.kind === IoKind.Output) continue;
				const table = b.area === 0 ? this.input : this.holding;
				const w = (r: number) => table.get(r) ?? 0;
				const v = b.bits === 32 ? ((w(b.reg) << 16) | w(b.reg + 1)) >>> 0 : w(b.reg);
				if (b.bits === 1) this.data[b.addr] = v !== 0 ? 1 : 0;
				else if (b.bits === 16) this.dv.setUint16(b.addr, v, true);
				else this.dv.setUint32(b.addr, v, true);
				if (b.kind === IoKind.Command) before.set(b.addr, v);
			}
			expect(this.vm.runCycle(0, (this.t += 50)).state).not.toBe("trapped");
			if (!this.iec) continue;
			// variables -> registers: every output, a command only when the program changed it
			for (const b of this.vm.io) {
				if (b.kind !== IoKind.Output && b.kind !== IoKind.Command) continue;
				const v = b.bits === 1 ? this.data[b.addr] : b.bits === 16 ? this.dv.getUint16(b.addr, true) : this.dv.getUint32(b.addr, true);
				if (b.kind === IoKind.Command && v === before.get(b.addr)) continue;
				if (b.bits === 32) {
					this.holding.set(b.reg, (v >>> 16) & 0xffff);
					this.holding.set(b.reg + 1, v & 0xffff);
				} else {
					this.holding.set(b.reg, v);
				}
			}
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
			["__Stepper1StartHoming", 69, "UINT"], ["__Stepper2StartHoming", 70, "UINT"], ["__RoArmMissionControl", 110, "INT"],
			["__MalibottiMissionControl", 111, "INT"]]);
		const commands = ioBindings().filter((b) => b.kind === "command");
		expect(commands.map((b) => b.register)).toEqual([69, 70, 110, 111]);
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

	it("FCU_MalibottiRunMission: runs, then Done; the robot's errors are the ErrorID", () => {
		const d = new Device();
		d.set("PLC_PRG.botMission", 3);
		d.set("PLC_PRG.goBot", true);
		d.cycle();
		expect(d.reg(MALIBOTTI)).toBe(3);
		d.cycle(3);
		expect(d.outputs("bot")).toMatchObject({ busy: true, done: false });
		d.holding.set(MALIBOTTI, 0); // malibotti: mission done
		d.cycle();
		expect(d.outputs("bot")).toEqual({ done: true, busy: false, error: false, id: 0 });

		d.set("PLC_PRG.goBot", false);
		d.cycle();
		d.set("PLC_PRG.goBot", true);
		d.cycle();
		d.holding.set(MALIBOTTI, 0xfffe); // malibotti: -2, unknown mission
		d.cycle();
		expect(d.outputs("bot")).toEqual({ done: false, busy: false, error: true, id: 2 });
	});
});

describe("FCU library: FCU_StepperMoveAbsolute", () => {
	const ENABLE = 60, TARGET2 = 65, SPEED2 = 67, POSITION2 = 243, STATUS2 = 245;
	const ENABLED = 1, MOVING = 2, IN_POSITION = 8, REFERENCED = 32;

	/** Axis 2 standing referenced at `position`; the program enables the drivers (an output). */
	function ready(position = 0): Device {
		const d = new Device();
		d.set("StepperEnable", true);
		d.setInput32(POSITION2, position);
		d.input.set(STATUS2, ENABLED | REFERENCED | IN_POSITION);  // standing at the old target
		d.cycle();
		expect(d.reg(ENABLE)).toBe(1);
		return d;
	}
	const out = (d: Device) => ({ done: d.get("PLC_PRG.move.Done"), busy: d.get("PLC_PRG.move.Busy"),
		aborted: d.get("PLC_PRG.move.CommandAborted"), error: d.get("PLC_PRG.move.Error"), id: d.get("PLC_PRG.move.ErrorID") });

	// The status bit "in position" still describes the old target in the cycle that writes the new
	// one: the block takes Done only from the position register reaching the goal.
	it("writes the target (32 bits) and the speed, Done when the axis stands at the goal", () => {
		const d = ready();
		d.set("PLC_PRG.goal", -70000);
		d.set("PLC_PRG.speed", 800);
		d.set("PLC_PRG.goMove", true);
		d.cycle();
		expect(d.reg32(TARGET2)).toBe(-70000);
		expect(d.reg(SPEED2)).toBe(800);
		expect(out(d)).toMatchObject({ busy: true, done: false });
		d.cycle(3);  // the firmware has not even started: InPosition is still set for the old target
		expect(out(d)).toMatchObject({ busy: true, done: false });
		d.setInput32(POSITION2, -30000);
		d.input.set(STATUS2, ENABLED | REFERENCED | MOVING);
		d.cycle();
		expect(out(d)).toMatchObject({ busy: true, done: false });
		d.setInput32(POSITION2, -70000);
		d.input.set(STATUS2, ENABLED | REFERENCED | IN_POSITION);
		d.cycle();
		expect(out(d)).toEqual({ done: true, busy: false, aborted: false, error: false, id: 0 });
	});

	it("speed 0 keeps the speed register", () => {
		const d = ready();
		d.set("Stepper2Speed", 1234);
		d.set("PLC_PRG.goal", 10);
		d.set("PLC_PRG.goMove", true);
		d.cycle();
		expect(d.reg(SPEED2)).toBe(1234);
	});

	it("refuses an axis that is not referenced (ErrorID 1) or drivers that are off (2)", () => {
		const d = ready();
		d.input.set(STATUS2, ENABLED);  // not referenced
		d.set("PLC_PRG.goal", 500);
		d.set("PLC_PRG.goMove", true);
		d.cycle();
		expect(out(d)).toMatchObject({ busy: false, error: true, id: 1 });
		expect(d.reg32(TARGET2)).toBe(0);  // nothing written

		const e = ready();
		e.set("StepperEnable", false);
		e.set("PLC_PRG.goal", 500);
		e.set("PLC_PRG.goMove", true);
		e.cycle();
		expect(out(e)).toMatchObject({ busy: false, error: true, id: 2 });
	});

	it("CommandAborted when another target is written while it moves", () => {
		const d = ready();
		d.set("PLC_PRG.goal", 500);
		d.set("PLC_PRG.goMove", true);
		d.cycle();
		d.set("Stepper2Target", 900);  // the program (or another block) sets a new target
		d.cycle();
		expect(out(d)).toMatchObject({ busy: false, done: false, aborted: true });
	});
});
