// Client-seitiger Vorgeschmack auf den in Firmware geplanten JointTracker (Plan Abschnitt 3,
// spaeter Core/Src/setup_and_loops/roarm_motion.hh): ein beschleunigungsbegrenztes
// (trapezfoermiges) Geschwindigkeitsprofil pro Gelenk, das bei jedem Tick aus der verbleibenden
// Distanz neu entscheidet, ob beschleunigt/konstant gefahren/gebremst wird -- kein vorab fixes
// Zeitprofil noetig, reagiert daher auch glatt auf ein sich waehrend der Fahrt aenderndes Ziel
// (Live-Jogging). Hier zusaetzlich nuetzlich, um das Bewegungsgefuehl schon ohne Hardware im
// Browser zu pruefen/abzustimmen, bevor dieselbe Logik in C++ entsteht.

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

	/// Ist- UND Zielwinkel sowie die Geschwindigkeit sofort setzen, ohne das Beschleunigungsprofil
	/// zu durchlaufen -- s. MultiJointTracker.snapTo().
	snapTo(value: number): void {
		this.current = value;
		this.target = value;
		this.velocity = 0;
	}

	/// dtSeconds: seit dem letzten Tick vergangene Zeit. Bremsweg wird aus der aktuellen
	/// Geschwindigkeit hergeleitet (v^2 / 2a), damit rechtzeitig vor dem Ziel abgebremst wird,
	/// statt es zu ueberschwingen.
	tick(dtSeconds: number): number {
		if (dtSeconds <= 0) return this.current;

		const distance = this.target - this.current;
		const direction = Math.sign(distance);
		const absDistance = Math.abs(distance);

		const brakingDistance = (this.velocity * this.velocity) / (2 * this.maxAccelerationPerSec2);

		let desiredVelocity: number;
		if (absDistance <= 1e-4 && Math.abs(this.velocity) < this.maxAccelerationPerSec2 * dtSeconds) {
			// Ziel praktisch erreicht und schon (fast) zum Stillstand gebremst -- exakt einrasten.
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
		// Ueberschiessen durch grobe Zeitschritte vermeiden: nie ueber das Ziel hinausfahren.
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

	/// Wie setTargets(), aber mit einer je Aufruf vorgegebenen Hoechstgeschwindigkeit statt der
	/// Konstruktor-Vorgabe -- fuer die Missions-Wiedergabe, wo jeder Schritt sein eigenes
	/// maxSpeedDegPerSec mitbringt (s. playMission() in roarm-teach-app.ts).
	setTargetsWithSpeed(targetsRad: readonly number[], maxVelocityRadPerSec: number): void {
		targetsRad.forEach((v, i) => {
			this.trackers[i].maxVelocityPerSec = maxVelocityRadPerSec;
			this.trackers[i].target = v;
		});
	}

	/// Setzt Ist- UND Zielwinkel sofort, ohne das beschleunigungsbegrenzte Profil zu durchlaufen --
	/// fuers direkte Jogging/Gizmo-Ziehen gedacht (der Nutzer erwartet dort 1:1-Nachfuehren, keine
	/// Traegheit), waehrend tick()/setTargets() fuer eine spaetere Missions-Wiedergabe-Simulation
	/// mit realistischem Bewegungsgefuehl verfuegbar bleiben.
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
