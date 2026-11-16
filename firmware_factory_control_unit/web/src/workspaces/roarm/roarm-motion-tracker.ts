// Client-side counterpart of the firmware's JointTracker (src/roarm/motion.hpp): an acceleration-limited
// (trapezoidal) velocity profile per joint that decides anew on every tick from the remaining distance
// whether to accelerate/cruise/brake - no time profile fixed in advance, so it also reacts smoothly to a
// target changing during the movement (live jogging). Also useful here to check/tune the feel of motion in
// the browser without hardware, before the same logic is written in C++.

export class JointTracker {
	current: number;
	private velocity = 0;
	target: number;

	constructor(
		initial: number,
		public maxVelocityPerSec: number,
		public maxAccelerationPerSec2: number,
	) {
		this.current = initial;
		this.target = initial;
	}

	get isMoving(): boolean {
		return Math.abs(this.target - this.current) > 1e-4 || Math.abs(this.velocity) > 1e-4;
	}

	// Sets actual AND target angle as well as the velocity immediately, without going through the acceleration
	// profile - see MultiJointTracker.snapTo().
	snapTo(value: number): void {
		this.current = value;
		this.target = value;
		this.velocity = 0;
	}

	// dtSeconds: time elapsed since the last tick. The braking distance is derived from the current velocity
	// (v^2 / 2a), so that braking starts in time before the target instead of overshooting it.
	tick(dtSeconds: number): number {
		if (dtSeconds <= 0) return this.current;

		const distance = this.target - this.current;
		const direction = Math.sign(distance);
		const absDistance = Math.abs(distance);

		const brakingDistance = (this.velocity * this.velocity) / (2 * this.maxAccelerationPerSec2);

		let desiredVelocity: number;
		if (absDistance <= 1e-4 && Math.abs(this.velocity) < this.maxAccelerationPerSec2 * dtSeconds) {
			// Target practically reached and already braked to (almost) standstill - snap in exactly.
			this.current = this.target;
			this.velocity = 0;
			return this.current;
		} else if (absDistance <= brakingDistance) {
			desiredVelocity = 0;
		} else {
			desiredVelocity = direction * this.maxVelocityPerSec;
		}

		const maxDeltaV = this.maxAccelerationPerSec2 * dtSeconds;
		const deltaV = Math.max(-maxDeltaV, Math.min(maxDeltaV, desiredVelocity - this.velocity));
		this.velocity += deltaV;

		let next = this.current + this.velocity * dtSeconds;
		// Avoid overshooting because of coarse time steps: never move beyond the target.
		if ((direction >= 0 && next > this.target) || (direction < 0 && next < this.target)) {
			next = this.target;
			this.velocity = 0;
		}
		this.current = next;
		return this.current;
	}
}

export class MultiJointTracker {
	private readonly trackers: JointTracker[];

	constructor(initialRad: readonly number[], maxVelocityRadPerSec: number, maxAccelerationRadPerSec2: number) {
		this.trackers = initialRad.map((v) => new JointTracker(v, maxVelocityRadPerSec, maxAccelerationRadPerSec2));
	}

	setTargets(targetsRad: readonly number[]): void {
		targetsRad.forEach((v, i) => {
			this.trackers[i].target = v;
		});
	}

	// Like setTargets(), but with a maximum speed given per call instead of the constructor default - for
	// mission playback, where every step brings its own maxSpeedDegPerSec (see playMission() in
	// roarm-teach-app.ts).
	setTargetsWithSpeed(targetsRad: readonly number[], maxVelocityRadPerSec: number): void {
		targetsRad.forEach((v, i) => {
			this.trackers[i].maxVelocityPerSec = maxVelocityRadPerSec;
			this.trackers[i].target = v;
		});
	}

	// Sets actual AND target angles immediately, without going through the acceleration-limited profile - meant
	// for direct jogging/gizmo dragging (the user expects 1:1 following there, no inertia), while
	// tick()/setTargets() stay available for a later mission playback simulation with a realistic feel of
	// motion.
	snapTo(anglesRad: readonly number[]): void {
		anglesRad.forEach((v, i) => this.trackers[i].snapTo(v));
	}

	get isMoving(): boolean {
		return this.trackers.some((t) => t.isMoving);
	}

	tick(dtSeconds: number): number[] {
		return this.trackers.map((t) => t.tick(dtSeconds));
	}

	get currentRad(): number[] {
		return this.trackers.map((t) => t.current);
	}
}
