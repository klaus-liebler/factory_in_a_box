// WebGL mesh rendering via OGL (github.com/oframe/ogl) — a small, well-
// tested WebGL wrapper, used here instead of a hand-rolled Canvas2D
// rasterizer after that approach kept surfacing rendering artifacts
// (faceted shading, seams). True per-pixel lighting and a real depth
// buffer come for free with a GPU pipeline.
//
// The scene graph is built once (one OGL Mesh per robot part, plus one
// per ghost part) and reused every frame — only transforms and the
// ghost's visibility are updated per frame, not the graph itself.

import { Renderer as GLRenderer, Camera as GLCamera, Transform, Mesh as GLMesh, Program, Geometry } from 'ogl'
import type { Camera as AppCamera } from './renderer.ts'
import type { Pose } from './math.ts'
import type { Mesh as AppMesh } from './robot-data.ts'

const VERTEX = `
  attribute vec3 position;
  attribute vec3 normal;
  uniform mat4 modelMatrix;
  uniform mat4 viewMatrix;
  uniform mat4 projectionMatrix;
  varying vec3 vNormal;
  varying vec3 vWorldPos;
  void main() {
    // mat3(modelMatrix) is enough (no non-uniform scale anywhere in this scene).
    vNormal = mat3(modelMatrix) * normal;
    vec4 worldPos = modelMatrix * vec4(position, 1.0);
    vWorldPos = worldPos.xyz;
    gl_Position = projectionMatrix * viewMatrix * worldPos;
  }
`

// A small metallic/roughness PBR shader (Cook-Torrance GGX specular +
// Fresnel + a procedural "sky gradient" standing in for image-based
// lighting) rather than the flat Blinn-Phong this had before — a single
// fixed-direction highlight on a flat-shaded surface reads as a cheap
// 3D-demo look; a Fresnel rim plus an environment reflection is most of
// what actually sells "this is metal/plastic under real light," and both
// are inexpensive per-pixel math, no texture/cubemap asset required (kept
// the whole thing analytic to stay asset-free for the single-file build).
const FRAGMENT = `
  precision mediump float;
  uniform vec3 uColor;
  uniform float uOpacity;
  uniform float uMetallic;
  uniform float uRoughness;
  uniform vec3 cameraPosition;
  varying vec3 vNormal;
  varying vec3 vWorldPos;

  const vec3 keyDir = vec3(0.4699, 0.5169, 0.7147);
  const vec3 keyColor = vec3(3.2, 3.1, 2.95);
  const vec3 fillDir = vec3(-0.7998, -0.3999, 0.4666);
  const vec3 fillColor = vec3(0.32, 0.38, 0.48);
  const float PI = 3.14159265;

  // Cheap stand-in for image-based lighting: a sky/ground gradient plus a
  // bright "sun" glint near the key light's own direction, sampled at a
  // direction instead of a real environment cubemap. The glint matters
  // more than it might look like it should: a single point-light specular
  // highlight is only visible in a razor-thin cone of viewing angles, which
  // reads as "flat" from everywhere else — a bright, wide-ish glint in the
  // *environment* itself means a curved metal surface shows a visible
  // reflection streak across a much wider sweep of angles, which is doing
  // most of the work of "this looks like real metal, not a grey box."
  vec3 skyEnv(vec3 dir) {
    vec3 sky = vec3(0.88, 0.92, 1.0);
    vec3 ground = vec3(0.10, 0.09, 0.08);
    vec3 horizon = vec3(0.5, 0.5, 0.52);
    float t = clamp(dir.z, -1.0, 1.0); // world is Z-up
    vec3 base = t > 0.0 ? mix(horizon, sky, pow(t, 0.6)) : mix(horizon, ground, pow(-t, 0.6));
    float sun = pow(max(dot(dir, keyDir), 0.0), 40.0) * 5.0;
    return base + vec3(sun);
  }

  float distributionGGX(float NdotH, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d + 0.0001);
  }
  float geometrySmith(float NdotV, float NdotL, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return (NdotV / (NdotV * (1.0 - k) + k)) * (NdotL / (NdotL * (1.0 - k) + k));
  }
  vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
  }

  void main() {
    vec3 N = normalize(vNormal);
    vec3 V = normalize(cameraPosition - vWorldPos);
    vec3 F0 = mix(vec3(0.04), uColor, uMetallic);

    float NdotV = max(dot(N, V), 0.0001);

    // Key light: full Cook-Torrance (diffuse + GGX specular).
    vec3 H = normalize(V + keyDir);
    float NdotL = max(dot(N, keyDir), 0.0);
    float NdotH = max(dot(N, H), 0.0);
    float VdotH = max(dot(V, H), 0.0);
    float D = distributionGGX(NdotH, uRoughness);
    float G = geometrySmith(NdotV, NdotL, uRoughness);
    vec3 F = fresnelSchlick(VdotH, F0);
    vec3 spec = (D * G * F) / max(4.0 * NdotV * NdotL, 0.001);
    vec3 kd = (1.0 - F) * (1.0 - uMetallic);
    vec3 direct = (kd * uColor / PI + spec) * keyColor * NdotL;

    // Fill light: diffuse only, cheaper and it's meant to be subtle anyway.
    direct += kd * uColor * fillColor * max(dot(N, fillDir), 0.0);

    // Fake IBL: reflect the view ray off the surface and sample the sky
    // gradient — this is what makes metal read as metal (mirrors its
    // surroundings) instead of a flat grey highlight.
    vec3 R = reflect(-V, N);
    vec3 envF = fresnelSchlick(NdotV, F0);
    vec3 envSpec = skyEnv(R) * envF * (1.0 - uRoughness * 0.7);
    vec3 envDiffuse = skyEnv(N) * uColor * (1.0 - uMetallic) * 0.35;

    vec3 color = direct + envSpec + envDiffuse;
    color = color / (color + vec3(1.0)); // Reinhard tonemap
    color = pow(color, vec3(1.0 / 2.2)); // gamma

    gl_FragColor = vec4(color, uOpacity);
  }
`

export interface PartHandle {
  glMesh: InstanceType<typeof GLMesh>
}

// Flat, unlit line shader — for reference geometry (the floor grid) that should be depth-tested
// against the robot mesh (so it's correctly occluded when the robot is in front of it from the
// camera's angle) but otherwise needs no lighting at all. A separate tiny program rather than
// reusing VERTEX/FRAGMENT above: those require a `normal` attribute the grid has no use for.
const LINE_VERTEX = `
  attribute vec3 position;
  uniform mat4 modelViewMatrix;
  uniform mat4 projectionMatrix;
  void main() {
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`
const LINE_FRAGMENT = `
  precision mediump float;
  uniform vec3 uColor;
  uniform float uOpacity;
  void main() {
    gl_FragColor = vec4(uColor, uOpacity);
  }
`

export interface MaterialOptions {
  metallic?: number
  roughness?: number
}

export class OglRenderer {
  private renderer: GLRenderer
  private camera: GLCamera
  private scene: Transform
  private geometryCache = new WeakMap<AppMesh, InstanceType<typeof Geometry>>()

  constructor(canvas: HTMLCanvasElement) {
    this.renderer = new GLRenderer({
      canvas,
      alpha: true,
      antialias: true,
      dpr: window.devicePixelRatio || 1,
    })
    // OGL's constructor internally calls setSize(300, 150) (its own defaults), which explicitly sets
    // canvas.style.width/height to "300px"/"150px" - this overrides the position:absolute;inset:0 already set
    // by roarm-3d-view.ts (an explicit width/height wins over inset, see the CSS over-constraint rules for
    // absolutely positioned boxes). Since resize() below deliberately NEVER touches style.width/height again
    // (the CSS box must come entirely from flexbox/inset:0, see the comment there), this one-time constructor
    // side effect has to be removed here once, otherwise the canvas stays stuck at 300x150 for good.
    canvas.style.removeProperty('width')
    canvas.style.removeProperty('height')
    this.renderer.gl.clearColor(0, 0, 0, 0)
    this.camera = new GLCamera(this.renderer.gl, { near: 0.01, far: 50, fov: 50 })
    this.scene = new Transform()
  }

  // OGL's own renderer.setSize() ALSO sets canvas.style.width/height as a side effect - exactly what we do
  // not want here (see the Overlay2D.resize() comment in renderer.ts: the CSS box must come only from
  // flexbox/inset:0, never from a pixel value set by JS that can differ from the actual container layout
  // because of timing). So the bitmap resolution is set manually here (identical to what setSize() does
  // internally, see node_modules/ogl/src/core/Renderer.js), but without its style lines.
  resize(width: number, height: number, dpr: number) {
    this.renderer.dpr = dpr
    this.renderer.width = width
    this.renderer.height = height
    this.renderer.gl.canvas.width = width * dpr
    this.renderer.gl.canvas.height = height * dpr
    this.camera.perspective({ aspect: width / height })
  }

  private getGeometry(mesh: AppMesh) {
    let geometry = this.geometryCache.get(mesh)
    if (!geometry) {
      geometry = new Geometry(this.renderer.gl, {
        position: { size: 3, data: mesh.positions },
        normal: { size: 3, data: computeVertexNormals(mesh) },
        index: { data: mesh.indices },
      })
      this.geometryCache.set(mesh, geometry)
    }
    return geometry
  }

  createPart(
    mesh: AppMesh,
    color: [number, number, number],
    opacity = 1,
    material: MaterialOptions = {},
  ): PartHandle {
    const gl = this.renderer.gl
    const program = new Program(gl, {
      vertex: VERTEX,
      fragment: FRAGMENT,
      uniforms: {
        uColor: { value: [color[0] / 255, color[1] / 255, color[2] / 255] },
        uOpacity: { value: opacity },
        uMetallic: { value: material.metallic ?? 0 },
        uRoughness: { value: material.roughness ?? 0.5 },
      },
      transparent: opacity < 1,
      depthWrite: opacity >= 1,
    })
    const glMesh = new GLMesh(gl, { geometry: this.getGeometry(mesh), program })
    glMesh.setParent(this.scene)
    return { glMesh }
  }

  /** Depth-tested reference lines (the floor grid) — drawn as real scene geometry instead of a 2D
   * overlay so the robot correctly occludes them when it's in front from the camera's angle. */
  createLines(lines: { a: [number, number, number]; b: [number, number, number] }[], color: [number, number, number], opacity = 1): PartHandle {
    const gl = this.renderer.gl
    const positions = new Float32Array(lines.length * 6)
    let w = 0
    for (const { a, b } of lines) {
      positions[w++] = a[0]; positions[w++] = a[1]; positions[w++] = a[2]
      positions[w++] = b[0]; positions[w++] = b[1]; positions[w++] = b[2]
    }
    const geometry = new Geometry(gl, { position: { size: 3, data: positions } })
    const program = new Program(gl, {
      vertex: LINE_VERTEX,
      fragment: LINE_FRAGMENT,
      uniforms: {
        uColor: { value: [color[0] / 255, color[1] / 255, color[2] / 255] },
        uOpacity: { value: opacity },
      },
      transparent: opacity < 1,
    })
    const glMesh = new GLMesh(gl, { geometry, program, mode: gl.LINES })
    glMesh.setParent(this.scene)
    return { glMesh }
  }

  setPose(handle: PartHandle, pose: Pose) {
    handle.glMesh.position.set(pose.pos[0], pose.pos[1], pose.pos[2])
    handle.glMesh.quaternion.set(pose.quat[0], pose.quat[1], pose.quat[2], pose.quat[3])
  }

  setVisible(handle: PartHandle, visible: boolean) {
    handle.glMesh.visible = visible
  }

  /* Recolours an already created part (e.g. the floor grid) live - for signal colours that can change per
     frame (see the approaching-the-motion-limit warning in roarm-3d-view.ts), without rebuilding the
     geometry. Works for every part created via createPart()/createLines()/createTexturedQuad(), whose
     programs all have a "uColor" uniform. */
  setColor(handle: PartHandle, color: [number, number, number]) {
    const uColor = handle.glMesh.program.uniforms.uColor
    if (uColor) uColor.value = [color[0] / 255, color[1] / 255, color[2] / 255]
  }

  render(camera: AppCamera) {
    this.camera.position.set(camera.pose.pos[0], camera.pose.pos[1], camera.pose.pos[2])
    this.camera.quaternion.set(camera.pose.quat[0], camera.pose.quat[1], camera.pose.quat[2], camera.pose.quat[3])
    this.camera.fov = (camera.fovY * 180) / Math.PI
    this.camera.perspective()
    this.renderer.render({ scene: this.scene, camera: this.camera })
  }
}

// Smooth (area-weighted) vertex normals from local mesh geometry — computed
// once per mesh at scene-build time, not baked into robotData.ts (keeps the
// shipped data file exactly as small as the raw geometry, nothing extra).
function computeVertexNormals(mesh: AppMesh): Float32Array {
  const { positions, indices } = mesh
  const normals = new Float32Array(positions.length)
  for (let i = 0; i < indices.length; i += 3) {
    const ia = indices[i] * 3
    const ib = indices[i + 1] * 3
    const ic = indices[i + 2] * 3
    const ux = positions[ib] - positions[ia]
    const uy = positions[ib + 1] - positions[ia + 1]
    const uz = positions[ib + 2] - positions[ia + 2]
    const vx = positions[ic] - positions[ia]
    const vy = positions[ic + 1] - positions[ia + 1]
    const vz = positions[ic + 2] - positions[ia + 2]
    const nx = uy * vz - uz * vy
    const ny = uz * vx - ux * vz
    const nz = ux * vy - uy * vx
    for (const idx of [ia, ib, ic]) {
      normals[idx] += nx
      normals[idx + 1] += ny
      normals[idx + 2] += nz
    }
  }
  for (let i = 0; i < normals.length; i += 3) {
    const len = Math.hypot(normals[i], normals[i + 1], normals[i + 2]) || 1
    normals[i] /= len
    normals[i + 1] /= len
    normals[i + 2] /= len
  }
  return normals
}
