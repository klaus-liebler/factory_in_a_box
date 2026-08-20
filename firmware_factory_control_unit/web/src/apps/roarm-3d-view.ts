// 3D-Modell des RoArm-M3 auf Basis von OGL (npm-Paket "ogl" -- s. Plan Abschnitt 9), an MoveIt2/
// RViz' MotionPlanning-Anzeige angelehnt: heller Armkoerper mit dunklen Gelenkgehaeusen (statt
// bunter Balken) plus ein 6-DOF-Interactive-Marker-Gizmo am Werkzeugkopf (3 RGB-Translationspfeile
// + 2 Dreh-Ringe fuer Pitch/Roll -- KEIN dritter Ring fuer "Yaw am Kopf": dieser Arm hat dort
// kinematisch keinen unabhaengigen 3. Rotationsfreiheitsgrad, die Werkzeug-Ausrichtung um die
// Welt-Z-Achse ergibt sich zwingend aus der Basis-Drehung zur Zielposition, s.
// roarm-kinematics.ts InverseKinematics()) plus je einen Dreh-Ring an jedem Gelenk der Kette.
//
// Die Verschachtelung der Transform-Knoten macht die eigentliche Vorwaertskinematik implizit
// (jede Rotation/Translation vererbt sich an die Kinder) -- fuers Rendering wird keine der Formeln
// aus roarm-kinematics.ts gebraucht, nur dieselben Linklaengen.
//
// Interaktion: Klick+Drag auf einen Gelenk-Ring dreht dieses eine Gelenk (Drag-Richtung = Tangente
// der Rotationskreisbahn, aus Kamerablickrichtung projiziert, s. onPointerDown()); Klick+Drag auf
// einen der 3 Kopf-Pfeile verschiebt die Werkzeugspitze entlang genau dieser einen Weltachse
// (klassische Translations-Gizmo-Technik); Klick+Drag auf den Pitch-/Roll-Ring am Kopf dreht die
// Werkzeug-Ausrichtung um genau diese Achse (dieselbe Tangenten-Technik wie bei den Gelenk-Ringen).
// Jedes interaktive Element hat zusaetzlich einen unsichtbaren, groesseren "Hit-Proxy" (separates,
// nicht gerendertes Mesh nur fuers Raycasting, s. hitProxySphere()/hitProxyCylinder()) -- ohne den
// war das sichtbar duenne Ring-/Pfeil-Geometrie in der Praxis kaum zuverlaessig zu treffen.
// Die tatsaechliche Umrechnung in Gelenkwinkel/kartesische IK-Ziele passiert NICHT hier, sondern im
// Aufrufer (roarm-teach-app.ts) -- dieses Modul liefert nur rohe Deltas (Winkel bzw.
// Szenen-Einheiten-Vektor), um kinematikfrei zu bleiben.
import { Renderer, Camera, Transform, Mesh, Program, Cylinder, Torus, Sphere, Orbit, Vec3, Raycast } from "ogl";
import { JOINT_COUNT } from "./roarm-kinematics.js";

// mm -> Szeneneinheiten (1 Einheit = 100mm), rein fuers handlichere Kamera-/Zoom-Werte. Von
// roarm-teach-app.ts beim Umrechnen von Kopf-Drag-Deltas (Szeneneinheiten) in mm wiederverwendet.
export const SCENE_UNITS_PER_MM = 1 / 100;
const MM = SCENE_UNITS_PER_MM;
const L1 = 126.06 * MM;
const L2 = Math.hypot(236.82, 30.0) * MM;
const L3 = 144.49 * MM;
const LE = Math.hypot(171.67, 13.69) * MM;
// Gesamthoehe (Base+L1+L2+L3+LE) ca. 7.1 Einheiten -- bestimmt Kamera-Framing weiter unten.
const TOTAL_HEIGHT_ESTIMATE = 0.32 + L1 + L2 + L3 + LE;

const LINK_RADIUS_BOTTOM = 0.14;
const LINK_RADIUS_TOP = 0.1;
const JOINT_RADIUS = 0.19;
const JOINT_HOUSING_LEN = 0.22;
const BASE_HEIGHT = 0.32;
const BASE_RADIUS = 0.55;

const JOINT_RING_RADIUS = JOINT_RADIUS + 0.08;
const JOINT_RING_TUBE = 0.035;
const JOINT_RING_HIT_RADIUS = JOINT_RING_RADIUS + 0.12; // s. hitProxySphere()-Kommentar
const ARROW_SHAFT_LEN = 0.85;
const ARROW_SHAFT_RADIUS = 0.035;
const ARROW_TIP_LEN = 0.2;
const ARROW_TIP_RADIUS = 0.08;
const ARROW_HIT_RADIUS = 0.16;
const HEAD_ORIENTATION_RING_RADIUS = 0.4;
const HEAD_ORIENTATION_RING_TUBE = 0.035;

const PITCH_ROLL_COLORS = { pitch: [0.85, 0.55, 0.15] as [number, number, number], roll: [0.6, 0.25, 0.85] as [number, number, number] };

// Panda-artige Farbgebung (helles Armgehaeuse, dunkle Gelenke/Greifer) statt bunter Glieder.
const LINK_COLOR_A: [number, number, number] = [0.88, 0.88, 0.86];
const LINK_COLOR_B: [number, number, number] = [0.83, 0.83, 0.81];
const JOINT_HOUSING_COLOR: [number, number, number] = [0.17, 0.18, 0.2];
const BASE_COLOR: [number, number, number] = [0.28, 0.29, 0.32];
const GRIPPER_COLOR: [number, number, number] = [0.2, 0.2, 0.22];

const GIZMO_RING_COLOR: [number, number, number] = [0.98, 0.66, 0.12];
const AXIS_COLORS: Record<"x" | "y" | "z", [number, number, number]> = {
	x: [0.92, 0.18, 0.2],
	y: [0.22, 0.82, 0.28],
	z: [0.2, 0.45, 0.95],
};

const vertex = /* glsl */ `
	attribute vec3 position;
	attribute vec3 normal;
	uniform mat4 modelViewMatrix;
	uniform mat4 projectionMatrix;
	uniform mat3 normalMatrix;
	varying vec3 vNormal;
	varying vec3 vViewPos;
	void main() {
		vNormal = normalize(normalMatrix * normal);
		vec4 viewPos = modelViewMatrix * vec4(position, 1.0);
		vViewPos = viewPos.xyz;
		gl_Position = projectionMatrix * viewPos;
	}
`;

// Zwei Richtungslichter (Haupt+Fuelllicht) + einfaches Blinn-Phong-Glanzlicht -- rein kosmetisch,
// damit die Formen (insb. Zylinder) plastischer wirken als mit reiner Flat-/Normal-Shading.
const fragment = /* glsl */ `
	precision mediump float;
	uniform vec3 uColor;
	varying vec3 vNormal;
	varying vec3 vViewPos;
	void main() {
		vec3 n = normalize(vNormal);
		vec3 key = normalize(vec3(0.5, 0.9, 0.6));
		vec3 fill = normalize(vec3(-0.6, 0.2, -0.4));
		float diffuse = max(dot(n, key), 0.0) * 0.7 + max(dot(n, fill), 0.0) * 0.25;
		vec3 viewDir = normalize(-vViewPos);
		vec3 halfDir = normalize(key + viewDir);
		float spec = pow(max(dot(n, halfDir), 0.0), 24.0) * 0.35;
		float ambient = 0.32;
		gl_FragColor = vec4(uColor * (ambient + diffuse) + vec3(spec), 1.0);
	}
`;

// Halbtransparentes Gizmo-Material (Ringe/Pfeile) -- bewusst ohne Beleuchtungsrechnung (reine
// Rim-Aufhellung), damit die Marker als "UI-Overlay" statt als Teil der Mechanik lesbar bleiben,
// analog zu MoveIts durchscheinenden Interactive-Marker-Ringen/-Pfeilen.
const gizmoFragment = /* glsl */ `
	precision mediump float;
	uniform vec3 uColor;
	varying vec3 vNormal;
	void main() {
		float rim = 0.55 + 0.45 * max(dot(normalize(vNormal), vec3(0.0, 0.0, 1.0)), 0.0);
		gl_FragColor = vec4(uColor * rim + vec3(0.1), 0.62);
	}
`;

export type HandleId = { type: "joint"; jointIndex: number } | { type: "head" } | { type: "orientation"; axis: "pitch" };

export interface RoArm3DCallbacks {
	onDragStart?(handle: HandleId): void;
	/** deltaRad ist kumulativ seit onDragStart (nicht seit dem letzten Aufruf). */
	onJointDrag?(jointIndex: number, deltaRadSinceDragStart: number): void;
	/** In Szeneneinheiten (SCENE_UNITS_PER_MM), kumulativ seit onDragStart. */
	onHeadDrag?(deltaSceneUnitsSinceDragStart: { x: number; y: number; z: number }): void;
	/** Pitch-Ring am Kopf -- "Roll" am Kopf meldet sich stattdessen ueber onJointDrag(4, ...), s.
	 * Moduskommentar (derselbe Rollgelenk-Freiheitsgrad, nur ein zweiter, bequemerer Anfasser). */
	onHeadOrientationDrag?(axis: "pitch", deltaRadSinceDragStart: number): void;
	onDragEnd?(): void;
}

export interface RoArm3DHandles {
	setJointAnglesRad(anglesRad: readonly number[]): void;
	/** Ausschliesslich fuers Kalibrieren/Verifizieren der Szene<->Kinematik-Achsenzuordnung (s.
	 * roarm-teach-app.ts' sceneDeltaToKinematicsMm()) sowie als Drag-Basispunkt nuetzlich. */
	getHeadWorldPositionSceneUnits(): { x: number; y: number; z: number };
	resize(): void;
	dispose(): void;
}

function material(gl: WebGL2RenderingContext, color: [number, number, number]): Program {
	return new Program(gl as any, { vertex, fragment, uniforms: { uColor: { value: color } } });
}

function gizmoMaterial(gl: WebGL2RenderingContext, color: [number, number, number]): Program {
	return new Program(gl as any, { vertex, fragment: gizmoFragment, uniforms: { uColor: { value: color } }, transparent: true, depthWrite: false });
}

// Zylindrisches Glied zwischen zwei Gelenken -- leicht konisch (Radius nimmt zur Spitze hin ab),
// wirkt dadurch weniger wie ein blosser Balken als ein flacher Box-Querschnitt.
function linkMesh(gl: WebGL2RenderingContext, length: number, color: [number, number, number]): Mesh {
	const geometry = new Cylinder(gl as any, { radiusTop: LINK_RADIUS_TOP, radiusBottom: LINK_RADIUS_BOTTOM, height: length, radialSegments: 20 });
	const mesh = new Mesh(gl as any, { geometry, program: material(gl, color) });
	mesh.position.set(0, length / 2, 0);
	return mesh;
}

// Servo-/Getriebe-Gehaeuse an einem Gelenk -- ein rotationssymmetrischer Zylinder um die
// Drehachse ist unabhaengig vom aktuellen Gelenkwinkel immer gleich ausgerichtet, daher genuegt
// eine feste lokale Rotation je nach Achse (Y bleibt Y, Z/X werden per 90Grad-Vorrotation aus der
// Standard-Y-Achse der Cylinder-Geometrie erzeugt).
function jointHousing(gl: WebGL2RenderingContext, axis: "x" | "y" | "z"): Mesh {
	const geometry = new Cylinder(gl as any, { radiusTop: JOINT_RADIUS, radiusBottom: JOINT_RADIUS, height: JOINT_HOUSING_LEN, radialSegments: 24 });
	const mesh = new Mesh(gl as any, { geometry, program: material(gl, JOINT_HOUSING_COLOR) });
	if (axis === "z") mesh.rotation.x = Math.PI / 2;
	if (axis === "x") mesh.rotation.z = Math.PI / 2;
	return mesh;
}

// Halbtransparenter Dreh-Ring um ein Gelenk (Interactive-Marker-Stil) -- Torus liegt per Default
// in der XY-Ebene (Lochachse = Z), daher dieselbe Vorrotations-Logik wie jointHousing().
function jointRing(gl: WebGL2RenderingContext, axis: "x" | "y" | "z"): Mesh {
	const geometry = new Torus(gl as any, { radius: JOINT_RING_RADIUS, tube: JOINT_RING_TUBE, radialSegments: 28, tubularSegments: 10 });
	const mesh = new Mesh(gl as any, { geometry, program: gizmoMaterial(gl, GIZMO_RING_COLOR) });
	if (axis === "y") mesh.rotation.x = Math.PI / 2;
	if (axis === "x") mesh.rotation.y = Math.PI / 2;
	return mesh;
}

// Ring + eigener, deutlich fetterer (aber gleich orientierter) Torus als Kollisionskoerper, fuer
// die beiden Kopf-Orientierungsringe (Pitch/Roll) gebraucht: die dort verwendete Kugel-Hitbox
// (s. hitProxySphere()) kann nicht zwischen zwei verschieden ausgerichteten Ringen AM SELBEN
// Punkt unterscheiden (eine Kugel ist richtungslos) -- ein fetter Torus in exakt derselben Ebene
// dagegen schon (seine Bounding-Box ist in Lochachsen-Richtung sehr flach, ein Klick weit
// ausserhalb der Ringebene faellt so durch, s. Raycast.intersectBounds()' Box-Pfad).
function ringWithFatHitProxy(
	gl: WebGL2RenderingContext,
	axis: "x" | "y" | "z",
	radius: number,
	tube: number,
	color: [number, number, number],
): { ring: Mesh; hit: Mesh } {
	const ring = new Mesh(gl as any, {
		geometry: new Torus(gl as any, { radius, tube, radialSegments: 28, tubularSegments: 10 }),
		program: gizmoMaterial(gl, color),
	});
	const hit = new Mesh(gl as any, {
		geometry: new Torus(gl as any, { radius, tube: tube + 0.16, radialSegments: 20, tubularSegments: 8 }),
		program: gizmoMaterial(gl, color),
	});
	hit.visible = false;
	if (axis === "y") {
		ring.rotation.x = Math.PI / 2;
		hit.rotation.x = Math.PI / 2;
	}
	if (axis === "x") {
		ring.rotation.y = Math.PI / 2;
		hit.rotation.y = Math.PI / 2;
	}
	return { ring, hit };
}

// Ein RGB-Translationspfeil (Schaft+Spitze) fuer den Kopf-Gizmo, Default zeigt entlang +Y wie die
// zugrunde liegenden Cylinder-Geometrien; toWorldAxis dreht die ganze Gruppe auf die Zielachse.
function translationArrow(gl: WebGL2RenderingContext, axis: "x" | "y" | "z"): { group: Transform; parts: Mesh[]; hit: Mesh } {
	const color = AXIS_COLORS[axis];
	const shaft = new Mesh(gl as any, {
		geometry: new Cylinder(gl as any, { radiusTop: ARROW_SHAFT_RADIUS, radiusBottom: ARROW_SHAFT_RADIUS, height: ARROW_SHAFT_LEN, radialSegments: 12 }),
		program: gizmoMaterial(gl, color),
	});
	shaft.position.set(0, ARROW_SHAFT_LEN / 2, 0);
	const tip = new Mesh(gl as any, {
		geometry: new Cylinder(gl as any, { radiusTop: 0, radiusBottom: ARROW_TIP_RADIUS, height: ARROW_TIP_LEN, radialSegments: 16 }),
		program: gizmoMaterial(gl, color),
	});
	tip.position.set(0, ARROW_SHAFT_LEN + ARROW_TIP_LEN / 2, 0);

	// Unsichtbarer, deutlich fetterer Kollisionskoerper ueber der vollen Pfeillaenge -- die
	// duenne sichtbare Geometrie allein war in der Praxis kaum zuverlaessig zu treffen (s.
	// Moduskommentar). visible=false blendet ihn vom Rendering aus, Raycast.intersectBounds()
	// (s. weiter unten) ignoriert dieses Flag bewusst nicht, sondern arbeitet unabhaengig davon
	// direkt auf der Geometrie -- genau das macht diesen Trick moeglich.
	const hit = new Mesh(gl as any, {
		geometry: new Cylinder(gl as any, { radiusTop: ARROW_HIT_RADIUS, radiusBottom: ARROW_HIT_RADIUS, height: ARROW_SHAFT_LEN + ARROW_TIP_LEN, radialSegments: 8 }),
		program: gizmoMaterial(gl, color),
	});
	hit.position.set(0, (ARROW_SHAFT_LEN + ARROW_TIP_LEN) / 2, 0);
	hit.visible = false;

	const group = new Transform();
	shaft.setParent(group);
	tip.setParent(group);
	hit.setParent(group);
	if (axis === "x") group.rotation.z = -Math.PI / 2;
	if (axis === "z") group.rotation.x = Math.PI / 2;
	return { group, parts: [shaft, tip], hit };
}

// Unsichtbare Kollisions-Kugel fuer einen Dreh-Ring (Torus) -- exaktes, grosszuegiges
// Kugel-Raycasting (geometry.raycast='sphere') statt der knappen Box-Naeherung, die ein duenner
// Torus sonst bekaeme. Wird als eigenes Geschwister-Mesh (nicht als Kind) an denselben Transform
// gehaengt wie der sichtbare Ring, s. Aufrufstellen.
function hitProxySphere(gl: WebGL2RenderingContext, radius: number): Mesh {
	const geometry = new Sphere(gl as any, { radius, widthSegments: 10, heightSegments: 8 });
	(geometry as any).raycast = "sphere";
	const mesh = new Mesh(gl as any, { geometry, program: gizmoMaterial(gl, [1, 1, 1]) });
	mesh.visible = false;
	return mesh;
}

export function createRoArm3DView(container: HTMLElement, callbacks: RoArm3DCallbacks = {}): RoArm3DHandles {
	const renderer = new Renderer({ dpr: Math.min(window.devicePixelRatio || 1, 2), alpha: true });
	const gl = renderer.gl as WebGL2RenderingContext;
	const canvasEl = gl.canvas as HTMLCanvasElement;
	gl.clearColor(0, 0, 0, 0);
	container.appendChild(canvasEl);

	// Kamera so platziert/ausgerichtet, dass die gesamte Kette INKLUSIVE des Kopf-Gizmos (Pfeile
	// ragen ARROW_SHAFT_LEN+ARROW_TIP_LEN ueber den Kopf hinaus) ins Bild passt -- ohne den
	// Gizmo-Zuschlag ragte der Pfeil-/Ring-Cluster bei einer nahezu vertikalen Pose deutlich ueber
	// den oberen Bildrand hinaus (per Playwright-Klicktest gefunden: errechnete Bildschirm-Y-
	// Koordinaten der Anfasser lagen ausserhalb des Canvas, s. Commit-Historie).
	const gizmoMargin = ARROW_SHAFT_LEN + ARROW_TIP_LEN + HEAD_ORIENTATION_RING_RADIUS;
	const effectiveHeight = TOTAL_HEIGHT_ESTIMATE + gizmoMargin;
	const lookAtHeight = effectiveHeight * 0.5;
	const camera = new Camera(gl as any, { fov: 38, near: 0.05, far: 60 });
	camera.position.set(effectiveHeight * 1.05, lookAtHeight + effectiveHeight * 0.25, effectiveHeight * 1.05);
	camera.lookAt(new Vec3(0, lookAtHeight, 0));

	const controls = new Orbit(camera, { element: canvasEl, target: new Vec3(0, lookAtHeight, 0), minDistance: 2, maxDistance: 40 });

	const scene = new Transform();

	const groundMarker = new Mesh(gl as any, {
		geometry: new Cylinder(gl as any, { radiusTop: 0.01, radiusBottom: 0.01, height: 1, radialSegments: 6 }),
		program: material(gl, [0.65, 0.65, 0.65]),
	});
	groundMarker.rotation.z = Math.PI / 2;
	groundMarker.scale.set(1, 10, 1);
	groundMarker.setParent(scene);

	const base = new Mesh(gl as any, {
		geometry: new Cylinder(gl as any, { radiusTop: BASE_RADIUS, radiusBottom: BASE_RADIUS * 1.08, height: BASE_HEIGHT, radialSegments: 28 }),
		program: material(gl, BASE_COLOR),
	});
	base.position.set(0, BASE_HEIGHT / 2, 0);
	base.setParent(scene);

	// --- Gelenkkette: Base(Y) -> Shoulder(Z) -> Elbow(Z) -> Wrist(Z) -> Roll(X) -> Gripper ---
	const baseJoint = new Transform();
	baseJoint.position.set(0, BASE_HEIGHT, 0);
	baseJoint.setParent(scene);
	jointHousing(gl, "y").setParent(baseJoint);

	const link1 = linkMesh(gl, L1, LINK_COLOR_A);
	link1.setParent(baseJoint);

	const shoulderJoint = new Transform();
	shoulderJoint.position.set(0, L1, 0);
	shoulderJoint.setParent(baseJoint);
	jointHousing(gl, "z").setParent(shoulderJoint);

	const link2 = linkMesh(gl, L2, LINK_COLOR_B);
	link2.setParent(shoulderJoint);

	const elbowJoint = new Transform();
	elbowJoint.position.set(0, L2, 0);
	elbowJoint.setParent(shoulderJoint);
	jointHousing(gl, "z").setParent(elbowJoint);

	const link3 = linkMesh(gl, L3, LINK_COLOR_A);
	link3.setParent(elbowJoint);

	const wristJoint = new Transform();
	wristJoint.position.set(0, L3, 0);
	wristJoint.setParent(elbowJoint);
	jointHousing(gl, "z").setParent(wristJoint);

	const link4 = linkMesh(gl, LE, LINK_COLOR_B);
	link4.setParent(wristJoint);

	const rollJoint = new Transform();
	rollJoint.position.set(0, LE, 0);
	rollJoint.setParent(wristJoint);
	jointHousing(gl, "x").setParent(rollJoint);

	const gripperMount = new Mesh(gl as any, {
		geometry: new Cylinder(gl as any, { radiusTop: 0.16, radiusBottom: 0.16, height: 0.1, radialSegments: 20 }),
		program: material(gl, GRIPPER_COLOR),
	});
	gripperMount.position.set(0, 0.05, 0);
	gripperMount.setParent(rollJoint);

	const fingerL = new Mesh(gl as any, {
		geometry: new Cylinder(gl as any, { radiusTop: 0.02, radiusBottom: 0.05, height: 0.32, radialSegments: 10 }),
		program: material(gl, GRIPPER_COLOR),
	});
	fingerL.position.set(-0.09, 0.1 + 0.16, 0);
	fingerL.setParent(rollJoint);
	const fingerR = new Mesh(gl as any, {
		geometry: new Cylinder(gl as any, { radiusTop: 0.02, radiusBottom: 0.05, height: 0.32, radialSegments: 10 }),
		program: material(gl, GRIPPER_COLOR),
	});
	fingerR.position.set(0.09, 0.1 + 0.16, 0);
	fingerR.setParent(rollJoint);

	// Kopf-Anker genau an der Werkzeugspitze (etwas ueber den Fingerspitzen) -- folgt der
	// Gelenkkette automatisch (Kind von rollJoint). Traegt KEINE eigene Geometrie -- dient nur als
	// Referenzpunkt, an den headGizmo (s.u., separat/nicht-rotierend) jeden Tick nachgefuehrt wird.
	const headAnchor = new Transform();
	headAnchor.position.set(0, 0.1 + 0.16 + 0.34, 0);
	headAnchor.setParent(rollJoint);

	const headMarker = new Mesh(gl as any, {
		geometry: new Sphere(gl as any, { radius: 0.05, widthSegments: 12, heightSegments: 8 }),
		program: gizmoMaterial(gl, [0.9, 0.9, 0.9]),
	});
	headMarker.setParent(headAnchor);

	// Pitch-/Roll-Ring am Kopf: Kinder von headAnchor (folgen also automatisch dessen Position
	// UND aktueller Orientierung). "Roll" ist geometrisch identisch mit dem Rollgelenk-Ring oben
	// (headAnchor erbt exakt rollJoints Rotation, keine eigene) -- ein zweiter, bequemerer
	// Anfasser fuer denselben Freiheitsgrad, s. Moduskommentar. "Pitch" liegt quer dazu (Y statt
	// X als Lochachse) -- rein optisch; der tatsaechliche Drehsinn wird in onPointerDown() ueber
	// die aktuelle Weltachse von wristJoint (s. dort) berechnet, nicht ueber diese lokale
	// Ausrichtung (kann bei aktivem Roll dadurch leicht "schief" wirken, s. Datei-Kommentar zur
	// vereinfachten Darstellung).
	const rollAtHead = ringWithFatHitProxy(gl, "x", HEAD_ORIENTATION_RING_RADIUS, HEAD_ORIENTATION_RING_TUBE, GIZMO_RING_COLOR);
	rollAtHead.ring.setParent(headAnchor);
	rollAtHead.hit.setParent(headAnchor);

	const pitchAtHead = ringWithFatHitProxy(gl, "y", HEAD_ORIENTATION_RING_RADIUS, HEAD_ORIENTATION_RING_TUBE, PITCH_ROLL_COLORS.pitch);
	pitchAtHead.ring.setParent(headAnchor);
	pitchAtHead.hit.setParent(headAnchor);

	// Kopf-Gizmo (3 Translationspfeile): bewusst NICHT Kind von rollJoint/headAnchor, sondern ein
	// eigener, nicht rotierender Transform-Knoten direkt unter scene -- die Pfeile sollen immer
	// weltachsenparallel zeigen (nicht mit Handgelenk/Roll mitdrehen), nur die POSITION wird in
	// setJointAnglesRad() an headAnchor nachgefuehrt.
	const headGizmo = new Transform();
	headGizmo.setParent(scene);
	const arrowX = translationArrow(gl, "x");
	const arrowY = translationArrow(gl, "y");
	const arrowZ = translationArrow(gl, "z");
	arrowX.group.setParent(headGizmo);
	arrowY.group.setParent(headGizmo);
	arrowZ.group.setParent(headGizmo);

	const jointHandleDefs: { transform: Transform; axis: "x" | "y" | "z" }[] = [
		{ transform: baseJoint, axis: "y" },
		{ transform: shoulderJoint, axis: "z" },
		{ transform: elbowJoint, axis: "z" },
		{ transform: wristJoint, axis: "z" },
		{ transform: rollJoint, axis: "x" },
	];
	const jointRingMeshes: Mesh[] = jointHandleDefs.flatMap((def, i) => {
		const ring = jointRing(gl, def.axis);
		ring.setParent(def.transform);
		(ring as any).__roarmJointIndex = i;
		const hit = hitProxySphere(gl, JOINT_RING_HIT_RADIUS);
		hit.setParent(def.transform);
		(hit as any).__roarmJointIndex = i;
		return [ring, hit];
	});
	// Gripper (Joint 6) hat kein eigenes Scharnier-Gelenk in dieser vereinfachten Kette (Oeffnen/
	// Schliessen wird ueber den Fingerabstand dargestellt, s. setJointAnglesRad) -- daher nur 5
	// Dreh-Ringe (Base..Roll) statt 6.
	for (const p of [...arrowX.parts, arrowX.hit]) (p as any).__roarmHeadAxis = "x";
	for (const p of [...arrowY.parts, arrowY.hit]) (p as any).__roarmHeadAxis = "y";
	for (const p of [...arrowZ.parts, arrowZ.hit]) (p as any).__roarmHeadAxis = "z";
	(rollAtHead.hit as any).__roarmJointIndex = 4;
	(pitchAtHead.hit as any).__roarmOrientation = "pitch";
	const allHandleMeshes: Mesh[] = [...jointRingMeshes, arrowX.hit, arrowY.hit, arrowZ.hit, rollAtHead.hit, pitchAtHead.hit];

	function resize(): void {
		const width = container.clientWidth || 1;
		const height = container.clientHeight || 1;
		renderer.setSize(width, height);
		camera.perspective({ aspect: width / height });
	}
	resize();

	let disposed = false;
	function raf(): void {
		if (disposed) return;
		requestAnimationFrame(raf);
		controls.update();
		renderer.render({ scene, camera });
	}
	requestAnimationFrame(raf);

	// --- Pointer-Handling fuer die Gizmo-Elemente (s. Moduskommentar oben) ---
	const raycast = new Raycast();

	function ndcFromEvent(e: PointerEvent): [number, number] {
		const rect = canvasEl.getBoundingClientRect();
		const x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
		const y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
		return [x, y];
	}

	type DragState =
		| { kind: "joint"; jointIndex: number; startX: number; startY: number; tangentPx: [number, number] }
		| { kind: "head"; startX: number; startY: number; axisWorld: Vec3; axisPx: [number, number]; scale: number }
		| { kind: "orientation"; axis: "pitch"; startX: number; startY: number; tangentPx: [number, number] };
	let drag: DragState | null = null;

	function worldMatrixColumn(m: Transform, col: number): Vec3 {
		const wm = m.worldMatrix;
		return new Vec3(wm[col * 4], wm[col * 4 + 1], wm[col * 4 + 2]);
	}

	// Mat4.getTranslation() (anders als z.B. Vec3.copy()) mutiert das uebergebene Vec3 und gibt
	// zum Verketten "this" (das Mat4) zurueck, NICHT den Vec3 -- daher hier als eigene Hilfsfunktion
	// statt des Rueckgabewerts direkt inline zu verwenden.
	function worldTranslation(t: Transform): Vec3 {
		const v = new Vec3();
		t.worldMatrix.getTranslation(v);
		return v;
	}

	function projectToPixels(worldPos: Vec3): [number, number] {
		const clip = new Vec3(worldPos.x, worldPos.y, worldPos.z);
		camera.project(clip);
		return [((clip.x + 1) / 2) * canvasEl.clientWidth, ((1 - clip.y) / 2) * canvasEl.clientHeight];
	}

	// Projiziert 'axisWorld' (Richtung, an 'pivot' verankert) auf eine normierte 2D-Bildschirmrichtung
	// -- gemeinsam genutzt fuer die Gelenk-Tangente UND die Kopf-Pfeilachsen.
	function axisToScreenDirection(pivot: Vec3, axisWorld: Vec3): [number, number] {
		const [px, py] = projectToPixels(pivot);
		const [tx, ty] = projectToPixels(new Vec3().copy(pivot).add(new Vec3().copy(axisWorld).multiply(0.05)));
		let dir: [number, number] = [tx - px, ty - py];
		const len = Math.hypot(dir[0], dir[1]) || 1;
		return [dir[0] / len, dir[1] / len];
	}

	// Tangente der Rotationskreisbahn um 'axisWorld' bei 'pivot', aus der aktuellen
	// Kamerablickrichtung projiziert -- gemeinsam genutzt von Gelenk-Ringen UND den beiden
	// Kopf-Orientierungsringen (Pitch/Roll), s. onPointerDown().
	function rotationTangentPx(pivot: Vec3, axisWorld: Vec3): [number, number] {
		const toCam = new Vec3().copy(camera.worldPosition).sub(pivot).normalize();
		const tangentWorld = new Vec3().cross(axisWorld, toCam);
		if (tangentWorld.len() < 1e-5) tangentWorld.set(1, 0, 0);
		tangentWorld.normalize();
		return axisToScreenDirection(pivot, tangentWorld);
	}

	function onPointerDown(e: PointerEvent): void {
		const [ndcX, ndcY] = ndcFromEvent(e);
		raycast.castMouse(camera, [ndcX, ndcY]);
		const hits = raycast.intersectBounds(allHandleMeshes);
		if (hits.length === 0) return;
		const hit = hits[0] as any;

		controls.enabled = false;
		canvasEl.setPointerCapture(e.pointerId);

		if (hit.__roarmHeadAxis) {
			const axis = hit.__roarmHeadAxis as "x" | "y" | "z";
			const axisWorld = axis === "x" ? new Vec3(1, 0, 0) : axis === "y" ? new Vec3(0, 1, 0) : new Vec3(0, 0, 1);
			const pivot = worldTranslation(headGizmo);
			const distance = camera.worldPosition.distance(pivot);
			const k = (2 * distance * Math.tan(((camera.fov / 2) * Math.PI) / 180)) / canvasEl.clientHeight;
			drag = { kind: "head", startX: e.clientX, startY: e.clientY, axisWorld, axisPx: axisToScreenDirection(pivot, axisWorld), scale: k };
			callbacks.onDragStart?.({ type: "head" });
		} else if (hit.__roarmOrientation === "pitch") {
			// Pitch-Achse = aktuelle Weltachse von wristJoint (upstream von Roll, s.
			// ringWithFatHitProxy()-Kommentar) -- NICHT headAnchors eigene Z-Achse, die sich mit
			// dem Rollwinkel mitdreht und damit fuer "reines Pitch" ungeeignet waere.
			const pivot = worldTranslation(headAnchor);
			const axisWorld = worldMatrixColumn(wristJoint, 2).normalize();
			drag = { kind: "orientation", axis: "pitch", startX: e.clientX, startY: e.clientY, tangentPx: rotationTangentPx(pivot, axisWorld) };
			callbacks.onDragStart?.({ type: "orientation", axis: "pitch" });
		} else {
			const jointIndex: number = hit.__roarmJointIndex;
			const def = jointHandleDefs[jointIndex];
			const pivot = worldTranslation(def.transform);
			const axisCol = def.axis === "x" ? 0 : def.axis === "y" ? 1 : 2;
			const axisWorld = worldMatrixColumn(def.transform, axisCol).normalize();

			drag = { kind: "joint", jointIndex, startX: e.clientX, startY: e.clientY, tangentPx: rotationTangentPx(pivot, axisWorld) };
			callbacks.onDragStart?.({ type: "joint", jointIndex });
		}
		e.preventDefault();
	}

	const ANGLE_PER_PIXEL = 0.012; // rad/px -- ca. 0.7 Grad pro Pixel, per Augenmass abgestimmt.

	function onPointerMove(e: PointerEvent): void {
		if (!drag) return;
		const dx = e.clientX - drag.startX;
		const dy = e.clientY - drag.startY;

		if (drag.kind === "joint") {
			const along = dx * drag.tangentPx[0] + dy * drag.tangentPx[1];
			callbacks.onJointDrag?.(drag.jointIndex, along * ANGLE_PER_PIXEL);
		} else if (drag.kind === "orientation") {
			const along = dx * drag.tangentPx[0] + dy * drag.tangentPx[1];
			callbacks.onHeadOrientationDrag?.(drag.axis, along * ANGLE_PER_PIXEL);
		} else {
			const along = dx * drag.axisPx[0] + dy * drag.axisPx[1];
			const worldDelta = new Vec3().copy(drag.axisWorld).multiply(along * drag.scale);
			callbacks.onHeadDrag?.({ x: worldDelta.x, y: worldDelta.y, z: worldDelta.z });
		}
	}

	function onPointerUp(e: PointerEvent): void {
		if (!drag) return;
		drag = null;
		controls.enabled = true;
		canvasEl.releasePointerCapture(e.pointerId);
		callbacks.onDragEnd?.();
	}

	canvasEl.addEventListener("pointerdown", onPointerDown);
	canvasEl.addEventListener("pointermove", onPointerMove);
	canvasEl.addEventListener("pointerup", onPointerUp);
	canvasEl.addEventListener("pointercancel", onPointerUp);

	return {
		setJointAnglesRad(anglesRad: readonly number[]): void {
			if (anglesRad.length < JOINT_COUNT) return;
			const [base_, shoulder, elbow, wrist, roll, gripper] = anglesRad;
			baseJoint.rotation.y = base_;
			shoulderJoint.rotation.z = shoulder;
			elbowJoint.rotation.z = elbow;
			wristJoint.rotation.z = wrist;
			rollJoint.rotation.x = roll;
			const opening = 0.02 + Math.max(0, Math.sin(gripper)) * 0.12;
			fingerL.position.x = -0.09 - opening;
			fingerR.position.x = 0.09 + opening;

			scene.updateMatrixWorld();
			headGizmo.position.copy(worldTranslation(headAnchor));
		},
		getHeadWorldPositionSceneUnits(): { x: number; y: number; z: number } {
			scene.updateMatrixWorld();
			const p = worldTranslation(headAnchor);
			return { x: p.x, y: p.y, z: p.z };
		},
		resize,
		dispose(): void {
			disposed = true;
			controls.remove();
			canvasEl.removeEventListener("pointerdown", onPointerDown);
			canvasEl.removeEventListener("pointermove", onPointerMove);
			canvasEl.removeEventListener("pointerup", onPointerUp);
			canvasEl.removeEventListener("pointercancel", onPointerUp);
			container.removeChild(canvasEl);
		},
	};
}
