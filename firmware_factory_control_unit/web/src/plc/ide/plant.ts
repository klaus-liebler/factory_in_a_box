// A small plant model for the simulator (mockup): a conveyor with workpieces passing two light
// barriers, a compressor with a pressure tank and valves, two steppers that drive to their targets.
// It reads the outputs of the program and writes its inputs once per cycle - so a program can be
// tried without hardware. It is a toy, not a physical simulation of the Factory-in-a-Box.

export interface PlantIo {
	get(name: string): number | boolean | undefined;
	set(name: string, v: number | boolean): void;
}

interface Piece {
	pos: number; // 0..100 % of the belt
}

export class Plant {
	private pieces: Piece[] = [];
	private spawnIn = 0;
	private pressure = 0; // raw 0..4095
	private steppers = [0, 0];
	private homingMs = [0, 0];
	/** Referenced axes: the device homes at its start when the motor supply is up. */
	private homed = [true, true];
	private missionMs = 0;
	private botMs = 0;
	private elapsed = 0;
	/** Workpieces pushed off at light barrier 2 (valve 1) and those that reached the end. */
	ejected = 0;
	delivered = 0;

	reset(): void {
		this.pieces = [];
		this.spawnIn = 0;
		this.pressure = 0;
		this.steppers = [0, 0];
		this.homingMs = [0, 0];
		this.homed = [true, true];
		this.missionMs = 0;
		this.botMs = 0;
		this.elapsed = 0;
		this.ejected = 0;
		this.delivered = 0;
	}

	get state(): { pieces: number[]; pressure: number } {
		return { pieces: this.pieces.map((p) => p.pos), pressure: this.pressure };
	}

	step(dtMs: number, io: PlantIo): void {
		this.elapsed += dtMs;
		const num = (n: string) => Number(io.get(n) ?? 0);
		const bool = (n: string) => Boolean(io.get(n));

		// conveyor: full speed (1000 per mille) = 25 % of the belt per second
		const speed = (Math.min(Math.max(num("ConveyorPwm"), 0), 1000) / 1000) * 25;
		if (speed > 0) {
			this.spawnIn -= dtMs;
			if (this.spawnIn <= 0 && (this.pieces.length === 0 || this.pieces[this.pieces.length - 1].pos > 22)) {
				this.pieces.push({ pos: 0 });
				this.spawnIn = 1800;
			}
		}
		for (const p of this.pieces) p.pos += (speed * dtMs) / 1000;
		// valve 1 pushes a piece off while it is at light barrier 2
		if (bool("Valve1")) {
			const before = this.pieces.length;
			this.pieces = this.pieces.filter((p) => !(p.pos > 66 && p.pos < 76));
			this.ejected += before - this.pieces.length;
		}
		const before = this.pieces.length;
		this.pieces = this.pieces.filter((p) => p.pos < 100);
		this.delivered += before - this.pieces.length;
		const at = (lo: number, hi: number) => this.pieces.some((p) => p.pos >= lo && p.pos <= hi);
		io.set("Lightbarrier1", at(28, 34));
		io.set("Lightbarrier2", at(68, 74));
		io.set("Lightbarrier3", at(94, 99));

		// compressor and tank: the valves let air out
		const inflow = (Math.min(Math.max(num("CompressorPwm"), 0), 1000) / 1000) * 900;
		const open = Number(bool("Valve1")) + Number(bool("Valve2")) + Number(bool("Valve3"));
		const outflow = (this.pressure * (0.05 + 0.6 * open)) / 1;
		this.pressure = Math.min(4095, Math.max(0, this.pressure + ((inflow - outflow) * dtMs) / 1000));
		io.set("PressureRaw", Math.round(this.pressure));

		// steppers: 2000 steps/s toward the target while enabled and referenced (as the firmware)
		const enabled = bool("StepperEnable");
		const targets = [num("Stepper1Target"), num("Stepper2Target")];
		const moving = [false, false];
		for (let i = 0; i < 2; i++) {
			if (!enabled || !this.homed[i]) continue;
			const d = targets[i] - this.steppers[i];
			const step = Math.sign(d) * Math.min(Math.abs(d), (2000 * dtMs) / 1000);
			this.steppers[i] = Math.round(this.steppers[i] + step);
			moving[i] = this.steppers[i] !== targets[i];
		}
		// the command registers (FCU library): homing takes 1 s, then position 0 and the answer 0;
		// a mission runs 2 s (mission 99 does not exist: -1)
		for (let i = 0; i < 2; i++) {
			const cmd = `__Stepper${i + 1}StartHoming`;
			if (num(cmd) !== 1) {
				this.homingMs[i] = 0;
				continue;
			}
			this.homingMs[i] += dtMs;
			this.homed[i] = false;  // as the firmware: no longer referenced while homing
			if (this.homingMs[i] >= 1000) {
				this.steppers[i] = 0;
				this.homed[i] = true;
				io.set(cmd, 0);
			}
		}
		const mission = num("__RoArmMissionControl");
		if (mission <= 0) {
			this.missionMs = 0;
		} else if (mission === 99) {
			io.set("__RoArmMissionControl", -1);
		} else if ((this.missionMs += dtMs) >= 2000) {
			io.set("__RoArmMissionControl", 0);
		}
		// the Malibotti robot: a mission takes 3 s, number 99 it does not know (-2)
		const bot = num("__MalibottiMissionControl");
		if (bot <= 0) {
			this.botMs = 0;
		} else if (bot === 99) {
			io.set("__MalibottiMissionControl", -2);
		} else if ((this.botMs += dtMs) >= 3000) {
			io.set("__MalibottiMissionControl", 0);
		}
		io.set("MalibottiStatus", 1 | (bot > 0 && bot !== 99 ? 2 : 0));  // connected, busy
		io.set("MalibottiActiveMission", bot > 0 && bot !== 99 ? bot : 0);
		// status: bit 0 enabled, 1 moving, 3 in position, 4 homing, 5 referenced (StepperStatus.cs)
		for (let i = 0; i < 2; i++) {
			const status = (enabled ? 1 : 0) | (moving[i] ? 2 : 0) | (!moving[i] && this.steppers[i] === targets[i] ? 8 : 0) |
				(this.homingMs[i] > 0 ? 16 : 0) | (this.homed[i] ? 32 : 0);
			io.set(`Stepper${i + 1}Status`, status);
		}
		io.set("Stepper1Position", this.steppers[0]);
		io.set("Stepper2Position", this.steppers[1]);

		// a few signs of life
		io.set("EthLinkStatus", true);
		io.set("TimerTick", Math.floor(this.elapsed) % 65536);
		io.set("CpuTemperatureC", 41);
	}
}
