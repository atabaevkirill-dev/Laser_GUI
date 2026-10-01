// Hexabot Scout: browser simulation of the robot on a test range.
// The legs run the JavaScript port of the ROS 2 gait (gait.js); device panels
// show the real UART packets of the LRF and the IR illuminator drivers.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Geometry, GaitGenerator, stanceToBase, rotateZ, clamp } from './gait.js';

const $ = (id) => document.getElementById(id);
const DEG = Math.PI / 180;
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const lerp = (a, b, t) => a + (b - a) * t;
const hex = (b) => b.toString(16).padStart(2, '0').toUpperCase();
const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

// ---------------------------------------------------------------------------
// Assets
// ---------------------------------------------------------------------------

const MESHES = [
  'base_printed', 'base_servos', 'base_electronics', 'base_sensors',
  'coxa_printed', 'coxa_servo', 'femur_printed', 'femur_servo',
  'tibia_printed', 'tibia_foot', 'head_pan_printed', 'head_pan_servo',
  'head_tilt_printed', 'head_tilt_payload',
];

function parseSTL(buffer) {
  const dv = new DataView(buffer);
  const n = dv.getUint32(80, true);
  const pos = new Float32Array(n * 9);
  const nor = new Float32Array(n * 9);
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
  let o = 84;
  for (let i = 0; i < n; i++) {
    o += 12; // stored normal: recomputed below
    for (let v = 0; v < 9; v++) { pos[i * 9 + v] = dv.getFloat32(o, true); o += 4; }
    o += 2;
    a.fromArray(pos, i * 9); b.fromArray(pos, i * 9 + 3); c.fromArray(pos, i * 9 + 6);
    c.sub(b); a.sub(b); c.cross(a).normalize();
    for (let v = 0; v < 3; v++) c.toArray(nor, i * 9 + v * 3);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  g.scale(0.001, 0.001, 0.001); // CAD is in millimetres
  return g;
}

async function loadAssets() {
  const [geoRes, meshRes, courseRes] = await Promise.all([fetch('geometry.json'), fetch('meshes.json'), fetch('course.json')]);
  if (!geoRes.ok || !meshRes.ok || !courseRes.ok) throw new Error('geometry.json, meshes.json or course.json is missing (run build.mjs)');
  const geoJson = await geoRes.json();
  COURSE = await courseRes.json();
  OBSTACLES = COURSE.obstacles;
  RAMPS = COURSE.ramps;
  PATROL = COURSE.patrol;
  ZONE = COURSE.zone;
  const pack = await meshRes.json();
  const meshes = {};
  for (const name of MESHES) {
    if (!pack[name]) throw new Error(`mesh ${name} missing in meshes.json`);
    const bin = atob(pack[name]);
    const bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    meshes[name] = parseSTL(bytes.buffer);
  }
  return { geo: new Geometry(geoJson), meshes };
}

// ---------------------------------------------------------------------------
// Test range: terrain, obstacles and actors
// ---------------------------------------------------------------------------

// Course (obstacles, ramps, patrol, zone, actors) from sim/course.json, shared
// with the Gazebo world.  Filled in loadAssets().
let OBSTACLES = [];
let RAMPS = [];
let PATROL = [];
let ZONE = { name: '', pts: [] };
let COURSE = null;

function terrainHeight(x, y) {
  let h = 0;
  for (const o of OBSTACLES) {
    if (x >= o.x0 && x <= o.x1 && y >= o.y0 && y <= o.y1) h = Math.max(h, o.h);
  }
  for (const r of RAMPS) {
    if (x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1) {
      h = Math.max(h, lerp(r.h0, r.h1, (x - r.x0) / (r.x1 - r.x0)));
    }
  }
  return h;
}

/** True if a circle of radius r at (x, y) touches an obstacle too tall to step on. */
function blockedAt(x, y, r, groundHere, stepMax) {
  for (const o of OBSTACLES) {
    if (o.h - groundHere <= stepMax + 0.005) continue;
    const cx = clamp(x, o.x0, o.x1), cy = clamp(y, o.y0, o.y1);
    if ((x - cx) ** 2 + (y - cy) ** 2 < r * r) return true;
  }
  return false;
}

function insidePolygon(x, y, pts) {
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const [xi, yi] = pts[i], [xj, yj] = pts[j];
    if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

// ---------------------------------------------------------------------------
// Scene construction
// ---------------------------------------------------------------------------

const MAT = {
  printed: new THREE.MeshStandardMaterial({ color: 0x3a403c, roughness: 0.82, metalness: 0.05 }),
  accent: new THREE.MeshStandardMaterial({ color: 0xd9480f, roughness: 0.6, metalness: 0.05 }),
  servo: new THREE.MeshStandardMaterial({ color: 0x161818, roughness: 0.38, metalness: 0.2 }),
  electronics: new THREE.MeshStandardMaterial({ color: 0x1f5c3a, roughness: 0.6, metalness: 0.2 }),
  sensors: new THREE.MeshStandardMaterial({ color: 0x2c3036, roughness: 0.45, metalness: 0.35 }),
  payload: new THREE.MeshStandardMaterial({ color: 0x24272b, roughness: 0.35, metalness: 0.5 }),
  foot: new THREE.MeshStandardMaterial({ color: 0x0f1010, roughness: 0.95 }),
};

// Temperatures (deg C) for the thermal camera, per material.
const TEMP = { printed: 24, accent: 24, servo: 38, electronics: 44, sensors: 33, payload: 30, foot: 20 };

function meshFor(geometry, matName) {
  const m = new THREE.Mesh(geometry, MAT[matName]);
  m.castShadow = true;
  m.receiveShadow = true;
  m.userData.temp = TEMP[matName];
  return m;
}

class RobotModel {
  constructor(geo, meshes) {
    this.geo = geo;
    this.root = new THREE.Group();
    this.body = new THREE.Group();
    this.root.add(this.body);
    const add = (parent, name, mat) => parent.add(meshFor(meshes[name], mat));
    add(this.body, 'base_printed', 'printed');
    add(this.body, 'base_servos', 'servo');
    add(this.body, 'base_electronics', 'electronics');
    add(this.body, 'base_sensors', 'sensors');

    this.legs = geo.mounts.map((m) => {
      const mount = new THREE.Group();
      mount.position.set(m.x, m.y, 0);
      mount.rotation.z = m.yaw;
      const coxa = new THREE.Group();
      mount.add(coxa);
      add(coxa, 'coxa_printed', 'printed');
      add(coxa, 'coxa_servo', 'servo');
      const femur = new THREE.Group();
      femur.position.x = geo.coxa;
      coxa.add(femur);
      add(femur, 'femur_printed', 'printed');
      add(femur, 'femur_servo', 'servo');
      const tibia = new THREE.Group();
      tibia.position.x = geo.femur;
      femur.add(tibia);
      add(tibia, 'tibia_printed', 'printed');
      add(tibia, 'tibia_foot', 'foot');
      this.body.add(mount);
      return { coxa, femur, tibia };
    });

    const j = geo.raw;
    this.pan = new THREE.Group();
    this.pan.position.set(...j.head_pan_xyz);
    this.body.add(this.pan);
    add(this.pan, 'head_pan_printed', 'accent');
    add(this.pan, 'head_pan_servo', 'servo');
    this.tilt = new THREE.Group();
    this.tilt.position.z = j.head_tilt_height;
    this.pan.add(this.tilt);
    add(this.tilt, 'head_tilt_printed', 'accent');
    add(this.tilt, 'head_tilt_payload', 'payload');

    // Optical anchors in head_tilt_link (+X is the boresight).
    const anchor = (xyz, dx) => {
      const o = new THREE.Object3D();
      o.position.set(xyz[0] + dx, xyz[1], xyz[2]);
      this.tilt.add(o);
      return o;
    };
    this.lrfAnchor = anchor(j.payload_lrf_xyz, 0.025);
    this.lowlightAnchor = anchor(j.payload_lowlight_xyz, 0.03);
    this.thermalAnchor = anchor(j.payload_thermal_xyz, 0.016);
    this.illAnchor = anchor(j.payload_illuminator_xyz, 0.045);
  }

  setLeg(i, q) {
    const l = this.legs[i];
    l.coxa.rotation.z = q.coxa;
    l.femur.rotation.y = -q.femur;
    l.tibia.rotation.y = -q.tibia;
  }

  setHead(pan, tilt) {
    this.pan.rotation.z = pan;
    this.tilt.rotation.y = -tilt;
  }
}

function groundTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 512;
  const g = c.getContext('2d');
  g.fillStyle = '#b9b29f';
  g.fillRect(0, 0, 512, 512);
  // Speckle so the ground does not look like a render.
  for (let i = 0; i < 9000; i++) {
    const v = 150 + Math.floor(Math.random() * 60);
    g.fillStyle = `rgba(${v},${v - 8},${v - 26},0.35)`;
    g.fillRect(Math.random() * 512, Math.random() * 512, 2, 2);
  }
  g.strokeStyle = 'rgba(70,64,52,0.28)';
  g.lineWidth = 2;
  for (let k = 0; k <= 512; k += 256) {
    g.beginPath(); g.moveTo(k, 0); g.lineTo(k, 512); g.stroke();
    g.beginPath(); g.moveTo(0, k); g.lineTo(512, k); g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(200, 200); // one tile = 2 m, a line every metre
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}

function buildWorld(scene) {
  const world = new THREE.Group();
  scene.add(world);
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(400, 400),
    new THREE.MeshStandardMaterial({ map: groundTexture(), roughness: 0.96 }),
  );
  ground.receiveShadow = true;
  ground.userData.temp = 12;
  ground.userData.env = true;
  world.add(ground);

  const mats = {
    rock: new THREE.MeshStandardMaterial({ color: 0x7d7468, roughness: 0.9 }),
    beam: new THREE.MeshStandardMaterial({ color: 0x8a6a45, roughness: 0.8 }),
    platform: new THREE.MeshStandardMaterial({ color: 0x9a9a92, roughness: 0.92 }),
    wall: new THREE.MeshStandardMaterial({ color: 0xa7a59c, roughness: 0.95 }),
  };
  for (const o of OBSTACLES) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(o.x1 - o.x0, o.y1 - o.y0, o.h), mats[o.kind]);
    m.position.set((o.x0 + o.x1) / 2, (o.y0 + o.y1) / 2, o.h / 2);
    m.castShadow = m.receiveShadow = true;
    m.userData.temp = 13;
    m.userData.env = true;
    world.add(m);
  }
  for (const r of RAMPS) {
    // Wedge / slab following the ramp height function.
    const s = new THREE.Shape([
      new THREE.Vector2(r.x0, 0), new THREE.Vector2(r.x1, 0),
      new THREE.Vector2(r.x1, Math.max(r.h1, 0.002)), new THREE.Vector2(r.x0, Math.max(r.h0, 0.002)),
    ]);
    const g = new THREE.ExtrudeGeometry(s, { depth: r.y1 - r.y0, bevelEnabled: false });
    g.rotateX(Math.PI / 2);
    g.translate(0, r.y1, 0);
    const m = new THREE.Mesh(g, mats.platform);
    m.castShadow = m.receiveShadow = true;
    m.userData.temp = 13;
    m.userData.env = true;
    world.add(m);
  }

  // Trees and bushes.
  const trunk = new THREE.MeshStandardMaterial({ color: 0x5b4733, roughness: 0.9 });
  const leaves = new THREE.MeshStandardMaterial({ color: 0x55663f, roughness: 0.9 });
  const trees = [];
  const rnd = mulberry32(COURSE.trees_seed);
  for (let i = 0; i < COURSE.trees_count; i++) {
    const a = rnd() * Math.PI * 2;
    const d = 16 + rnd() * 90;
    const x = Math.cos(a) * d + 20, y = Math.sin(a) * d;
    if (insidePolygon(x, y, ZONE.pts)) continue;
    const h = 4 + rnd() * 5;
    const t = new THREE.Group();
    const tr = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.22, h * 0.45, 8), trunk);
    tr.rotation.x = Math.PI / 2; tr.position.z = h * 0.22;
    const lv = new THREE.Mesh(new THREE.ConeGeometry(h * 0.28, h * 0.75, 9), leaves);
    lv.rotation.x = Math.PI / 2; lv.position.z = h * 0.62;
    for (const m of [tr, lv]) { m.castShadow = true; m.userData.temp = 9; m.userData.env = true; t.add(m); }
    t.position.set(x, y, 0);
    world.add(t);
    trees.push({ x, y, r: h * 0.22, h });
  }

  // Zone A outline.
  const zoneShape = new THREE.Shape(ZONE.pts.map(([x, y]) => new THREE.Vector2(x, y)));
  const zone = new THREE.Mesh(new THREE.ShapeGeometry(zoneShape),
    new THREE.MeshBasicMaterial({ color: 0xe0a43a, transparent: true, opacity: 0.18, depthWrite: false }));
  zone.position.z = 0.01;
  zone.layers.set(1);
  world.add(zone);
  const postMat = new THREE.MeshStandardMaterial({ color: 0xe0a43a, roughness: 0.6 });
  for (const [x, y] of ZONE.pts) {
    const p = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 1.2, 6), postMat);
    p.rotation.x = Math.PI / 2; p.position.set(x, y, 0.6); p.castShadow = true; p.userData.temp = 12; p.userData.env = true;
    world.add(p);
  }
  return { world, trees };
}

function mulberry32(seed) {
  return () => {
    seed |= 0; seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Actors: people walking, a drone and a parked vehicle.
function makePerson(color) {
  const g = new THREE.Group();
  const skin = new THREE.MeshStandardMaterial({ color: 0xc69c7a, roughness: 0.8 });
  const cloth = new THREE.MeshStandardMaterial({ color, roughness: 0.85 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x2b2e33, roughness: 0.9 });
  const part = (geo, mat, x, y, z, temp) => {
    const m = new THREE.Mesh(geo, mat);
    m.position.set(x, y, z); m.castShadow = true; m.userData.temp = temp; g.add(m); return m;
  };
  const legGeo = new THREE.CylinderGeometry(0.07, 0.06, 0.82, 8); legGeo.rotateX(Math.PI / 2); legGeo.translate(0, 0, -0.41);
  const armGeo = new THREE.CylinderGeometry(0.05, 0.045, 0.62, 8); armGeo.rotateX(Math.PI / 2); armGeo.translate(0, 0, -0.31);
  const legL = part(legGeo, dark, 0, 0.1, 0.86, 30);
  const legR = part(legGeo, dark, 0, -0.1, 0.86, 30);
  const torso = part(new THREE.CylinderGeometry(0.19, 0.16, 0.62, 10), cloth, 0, 0, 1.18, 31);
  torso.rotation.x = Math.PI / 2;
  part(new THREE.SphereGeometry(0.11, 12, 10), skin, 0, 0, 1.62, 35);
  const armL = part(armGeo, cloth, 0, 0.24, 1.45, 31);
  const armR = part(armGeo, cloth, 0, -0.24, 1.45, 31);
  g.userData.limbs = { legL, legR, armL, armR };
  return g;
}

function makeDrone() {
  const g = new THREE.Group();
  const body = new THREE.MeshStandardMaterial({ color: 0x2a2d31, roughness: 0.5, metalness: 0.3 });
  const prop = new THREE.MeshBasicMaterial({ color: 0x9aa0a6, transparent: true, opacity: 0.35 });
  const b = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.14, 0.08), body);
  b.userData.temp = 33; g.add(b);
  const rotors = [];
  for (let k = 0; k < 4; k++) {
    const a = Math.PI / 4 + k * Math.PI / 2;
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.02, 0.02), body);
    arm.position.set(Math.cos(a) * 0.1, Math.sin(a) * 0.1, 0); arm.rotation.z = a; arm.userData.temp = 30; g.add(arm);
    const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.035, 10), body);
    motor.rotation.x = Math.PI / 2; motor.position.set(Math.cos(a) * 0.2, Math.sin(a) * 0.2, 0.02); motor.userData.temp = 52; g.add(motor);
    const disc = new THREE.Mesh(new THREE.CircleGeometry(0.1, 20), prop);
    disc.position.set(Math.cos(a) * 0.2, Math.sin(a) * 0.2, 0.045); disc.userData.temp = 18; g.add(disc);
    rotors.push(disc);
  }
  g.traverse((m) => { if (m.isMesh) m.castShadow = true; });
  g.userData.rotors = rotors;
  return g;
}

function makeVehicle() {
  const g = new THREE.Group();
  const paint = new THREE.MeshStandardMaterial({ color: 0x55606b, roughness: 0.4, metalness: 0.5 });
  const glass = new THREE.MeshStandardMaterial({ color: 0x1b2229, roughness: 0.15, metalness: 0.6 });
  const tyre = new THREE.MeshStandardMaterial({ color: 0x111214, roughness: 0.9 });
  const add = (geo, mat, x, y, z, temp) => {
    const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); m.castShadow = true; m.userData.temp = temp; g.add(m); return m;
  };
  add(new THREE.BoxGeometry(4.5, 1.9, 0.75), paint, 0, 0, 0.78, 16);
  add(new THREE.BoxGeometry(1.4, 1.85, 0.6), paint, 1.55, 0, 0.8, 58); // engine bay is hot
  add(new THREE.BoxGeometry(2.5, 1.75, 0.6), glass, -0.3, 0, 1.45, 14);
  for (const [x, y] of [[1.45, 0.9], [1.45, -0.9], [-1.45, 0.9], [-1.45, -0.9]]) {
    const w = add(new THREE.CylinderGeometry(0.38, 0.38, 0.28, 16), tyre, x, y, 0.38, 26);
    w.rotation.x = 0;
  }
  return g;
}

class Actor {
  constructor(kind, label, obj, path, speed, opts = {}) {
    this.kind = kind; this.label = label; this.obj = obj; this.path = path; this.speed = speed;
    this.s = opts.s0 || 0; this.alt = opts.alt || 0; this.pause = 0;
    this.radius = kind === 'person' ? 0.25 : kind === 'drone' ? 0.25 : 2.3;
    this.height = kind === 'person' ? 1.75 : kind === 'drone' ? 0.12 : 1.75;
    this.pos = new THREE.Vector3();
    this.inZone = false;
    this.update(0, 0);
  }

  center() { return new THREE.Vector3(this.pos.x, this.pos.y, this.pos.z + (this.kind === 'drone' ? 0 : this.height * 0.55)); }

  update(t, dt) {
    if (this.kind === 'drone') {
      const c = this.path;
      const a = c.phase + (t * this.speed) / c.r;
      this.pos.set(c.x + c.r * Math.cos(a), c.y + c.r * Math.sin(a), c.alt + Math.sin(t * 0.7) * 2);
      this.obj.position.copy(this.pos);
      this.obj.rotation.z = a + Math.PI / 2;
      for (const r of this.obj.userData.rotors) r.rotation.z += dt * 60;
      return;
    }
    if (!this.path) { this.obj.position.copy(this.pos); return; }
    // Walk a closed polyline at constant speed, pausing at corners.
    if (this.pause > 0) { this.pause -= dt; } else { this.s += this.speed * dt; }
    const pts = this.path;
    let total = 0;
    const segs = pts.map((p, i) => { const q = pts[(i + 1) % pts.length]; const L = Math.hypot(q[0] - p[0], q[1] - p[1]); total += L; return L; });
    let s = this.s % total;
    let i = 0;
    while (s > segs[i]) { s -= segs[i]; i = (i + 1) % pts.length; }
    const p = pts[i], q = pts[(i + 1) % pts.length];
    const f = s / segs[i];
    if (segs[i] - s < this.speed * dt * 1.5 && this.pause <= 0 && Math.random() < 0.5) this.pause = 1.5 + Math.random() * 3;
    this.pos.set(lerp(p[0], q[0], f), lerp(p[1], q[1], f), 0);
    this.obj.position.copy(this.pos);
    this.obj.rotation.z = Math.atan2(q[1] - p[1], q[0] - p[0]);
    const swing = this.pause > 0 ? 0 : Math.sin(this.s * 3.1) * 0.45;
    const l = this.obj.userData.limbs;
    if (l) { l.legL.rotation.y = swing; l.legR.rotation.y = -swing; l.armL.rotation.y = -swing * 0.8; l.armR.rotation.y = swing * 0.8; }
  }
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------

const CAMERAS = {
  color: { label: 'LOW-LIGHT · IMX462', tag: 'low-light · HFOV 26°', hfov: 26 },
  nir: { label: 'LOW-LIGHT · NO IR-CUT · 850 nm', tag: 'IR illuminated · HFOV 26°', hfov: 26 },
  thermal: { label: 'THERMAL · 256×192 · 8–14 µm', tag: 'thermal · HFOV 56°', hfov: 56 },
};

// Detection range (m) of the simulated detector per camera mode and class.
const DETECT_RANGE = {
  color: { person: 220, drone: 260, vehicle: 600 },
  nir: { person: 160, drone: 120, vehicle: 300 },
  thermal: { person: 120, drone: 150, vehicle: 350 },
};

const IRONBOW = [
  [0.0, [0, 0, 4]], [0.22, [40, 11, 84]], [0.42, [125, 23, 110]], [0.6, [205, 63, 76]],
  [0.75, [246, 129, 40]], [0.88, [252, 207, 72]], [1.0, [255, 255, 238]],
];
function ironbow(t) {
  t = clamp(t, 0, 1);
  for (let i = 1; i < IRONBOW.length; i++) {
    if (t <= IRONBOW[i][0]) {
      const [t0, c0] = IRONBOW[i - 1], [t1, c1] = IRONBOW[i];
      const f = (t - t0) / (t1 - t0);
      return new THREE.Color(...c0.map((v, k) => lerp(v, c1[k], f) / 255));
    }
  }
  return new THREE.Color(1, 1, 1);
}

// LRF packet helpers (protocol from the module manual, section 6).
function lrfFrame(cmd, params = []) {
  const body = [0x03, cmd, ...params];
  const sum = body.reduce((a, b) => a + b, 0) & 0xff;
  return [0xee, 0x16, body.length, ...body, sum];
}

function lrfRangeResponse(range, status) {
  const r = Math.max(0, range);
  const whole = Math.floor(r);
  const dec = Math.round((r - whole) * 10) % 10;
  return lrfFrame(0x02, [status, (whole >> 8) & 0xff, whole & 0xff, dec]);
}

// Illuminator packets (protocol of the former Laser_GUI device).
function illPacket(cmd1, cmd2, d1 = 0, d2 = 0) {
  const p = [0xff, 0x01, cmd1, cmd2, d1, d2];
  p.push(p.slice(1).reduce((a, b) => a + b, 0) & 0xff);
  return p;
}
const illSpotInternal = (deg) => clamp(Math.round(((71 - deg) / (71 - 1.8)) * 0x4000), 0, 0x4000);

// ---------------------------------------------------------------------------
// Application
// ---------------------------------------------------------------------------

async function main() {
  const { geo, meshes } = await loadAssets();

  const canvas = $('main-canvas');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(42, 1, 0.02, 2500);
  camera.up.set(0, 0, 1);
  camera.position.set(-0.62, -0.98, 0.42);
  camera.layers.enable(1);
  const controls = new OrbitControls(camera, canvas);
  controls.target.set(0.22, 0.02, 0.1);
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI * 0.495;
  controls.minDistance = 0.4;
  controls.maxDistance = 120;

  const hemi = new THREE.HemisphereLight(0xe3ecf2, 0x8a8170, 1.25);
  hemi.up.set(0, 0, 1);
  scene.add(hemi);
  const sun = new THREE.DirectionalLight(0xfff4e2, 2.3);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -4, right: 4, top: 4, bottom: -4, near: 0.5, far: 40 });
  sun.shadow.bias = -0.0004;
  scene.add(sun, sun.target);

  const { world, trees } = buildWorld(scene);

  // Stars for the night sky.
  const starGeo = new THREE.BufferGeometry();
  const sp = [];
  const rnd = mulberry32(3);
  for (let i = 0; i < 900; i++) {
    const a = rnd() * Math.PI * 2, e = Math.asin(0.08 + rnd() * 0.92);
    sp.push(Math.cos(a) * Math.cos(e) * 900, Math.sin(a) * Math.cos(e) * 900, Math.sin(e) * 900);
  }
  starGeo.setAttribute('position', new THREE.Float32BufferAttribute(sp, 3));
  const stars = new THREE.Points(starGeo, new THREE.PointsMaterial({ color: 0xdfe6f5, size: 1.6, sizeAttenuation: false, fog: false }));
  stars.visible = false;
  stars.userData.temp = -40;
  scene.add(stars);

  // Robot.
  const robot = new RobotModel(geo, meshes);
  scene.add(robot.root);
  const gen = new GaitGenerator(geo);

  // Head optics: inset camera, LRF beam, IR illuminator.
  const headCam = new THREE.PerspectiveCamera(20, 4 / 3, 0.05, 1500);
  headCam.matrixAutoUpdate = true;
  const camBasis = new THREE.Matrix4().makeBasis(
    new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(-1, 0, 0));
  headCam.quaternion.setFromRotationMatrix(camBasis);
  robot.lowlightAnchor.add(headCam);

  const spot = new THREE.SpotLight(0xffffff, 0, 180, 6 * DEG, 0.35, 0);
  spot.position.set(0, 0, 0);
  robot.illAnchor.add(spot);
  const spotTarget = new THREE.Object3D();
  spotTarget.position.set(10, 0, 0);
  robot.illAnchor.add(spotTarget);
  spot.target = spotTarget;
  const cone = new THREE.Mesh(
    new THREE.ConeGeometry(1, 1, 32, 1, true),
    new THREE.MeshBasicMaterial({ color: 0xb8a6ff, transparent: true, opacity: 0.07, depthWrite: false, side: THREE.DoubleSide }),
  );
  cone.layers.set(1);
  robot.illAnchor.add(cone);

  const beamGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]);
  const beam = new THREE.Line(beamGeo, new THREE.LineDashedMaterial({ color: 0xff6a2c, dashSize: 0.25, gapSize: 0.18, transparent: true, opacity: 0.95 }));
  beam.layers.set(1);
  beam.frustumCulled = false;
  beam.visible = false;
  scene.add(beam);
  const hitMark = new THREE.Mesh(new THREE.SphereGeometry(0.12, 12, 10), new THREE.MeshBasicMaterial({ color: 0xff6a2c }));
  hitMark.layers.set(1);
  hitMark.visible = false;
  scene.add(hitMark);

  const selRing = new THREE.Mesh(new THREE.RingGeometry(0.55, 0.68, 40), new THREE.MeshBasicMaterial({ color: 0x33b3a6, transparent: true, opacity: 0.9, side: THREE.DoubleSide }));
  selRing.layers.set(1);
  selRing.visible = false;
  scene.add(selRing);

  // Actors.
  const C = COURSE;
  const actors = [
    ...C.people.map((p) => new Actor('person', 'person', makePerson(new THREE.Color(p.color).getHex()), p.path, p.speed, { s0: p.s0 || 0 })),
    new Actor('drone', 'drone', makeDrone(), { x: C.drone.x, y: C.drone.y, r: C.drone.r, alt: C.drone.alt, phase: C.drone.phase }, C.drone.speed),
    new Actor('vehicle', 'vehicle', makeVehicle(), null, 0),
  ];
  const vehicle = actors[actors.length - 1];
  vehicle.pos.set(C.vehicle.x, C.vehicle.y, 0);
  vehicle.obj.rotation.z = C.vehicle.yaw;
  for (const a of actors) scene.add(a.obj);

  // Inset renderer for the head camera.
  const camCanvas = $('cam-canvas');
  const camRenderer = new THREE.WebGLRenderer({ canvas: camCanvas, antialias: false });
  camRenderer.setPixelRatio(1);
  camRenderer.setSize(320, 240, false);
  const overlay = $('cam-overlay').getContext('2d');

  // Thermal materials, cached per temperature.
  const thermalCache = new Map();
  const thermalMat = (temp) => {
    const key = Math.round(temp);
    if (!thermalCache.has(key)) thermalCache.set(key, new THREE.MeshBasicMaterial({ color: ironbow(key / 44) }));
    return thermalCache.get(key);
  };
  // Scenery is sun-warmed by day and cold at night; living things are not.
  const sceneTemp = (m) => {
    const t = m.userData.temp ?? 16;
    return m.userData.env ? t + (st.night ? -5 : 9) : t;
  };
  const thermalPoints = new THREE.PointsMaterial({ color: 0x000000, size: 1, sizeAttenuation: false });

  // ------------------------------------------------------------------ state
  const st = {
    t: 0,
    x: 0, y: 0, yaw: 0,
    bodyHeight: geo.bodyHeight, userHeight: geo.bodyHeight, ground: 0,
    pose: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
    stepBase: geo.stepHeight, autoStep: true, crouch: false, speedLimit: 0.7,
    patrol: !reducedMotion, wp: 0, observe: 0, observeName: '',
    follow: true, night: false, camMode: 'color',
    ill: { power: false, brightness: 200, spot: 12 },
    lrf: { mode: 1, shots: 0, last: null, lastPacket: null, busy: 0 },
    head: { pan: 0, tilt: 0, auto: true, scanT: 0 },
    tracks: new Map(), nextTrackId: 1, selected: null,
    battery: 1.0, events: [], blocked: false,
    keys: new Set(), stick: { x: 0, y: 0 },
  };

  // Joint angles after IK, kept for the UI.
  const q = new Array(6).fill(null).map(() => ({ coxa: 0, femur: 0, tibia: 0 }));

  // ---------------------------------------------------------------- helpers
  const logEvent = (type, text) => {
    st.events.unshift({ t: st.t, type, text });
    st.events.length = Math.min(st.events.length, 40);
    renderEvents();
  };

  const fmtTime = (t) => `${String(Math.floor(t / 60)).padStart(2, '0')}:${(t % 60).toFixed(1).padStart(4, '0')}`;

  function worldFromStance(p) {
    const r = rotateZ(p, st.yaw);
    return { x: st.x + r.x, y: st.y + r.y };
  }

  function headPose() {
    robot.root.updateMatrixWorld(true);
    const o = new THREE.Vector3(), d = new THREE.Vector3(1, 0, 0);
    robot.lrfAnchor.getWorldPosition(o);
    const qw = new THREE.Quaternion();
    robot.tilt.getWorldQuaternion(qw);
    d.applyQuaternion(qw);
    return { origin: o, dir: d };
  }

  /** All ray hits (sorted) for the LRF and the line of sight. */
  function raycast(origin, dir, maxT = 4500) {
    const hits = [];
    // Ground (z = 0 beyond the obstacle course).
    if (dir.z < -1e-6) {
      const t = -origin.z / dir.z;
      if (t > 0 && t < maxT) hits.push({ t, what: 'ground' });
    }
    const box = (x0, x1, y0, y1, z0, z1, what, actor) => {
      let t0 = 0, t1 = maxT;
      const o = [origin.x, origin.y, origin.z], dv = [dir.x, dir.y, dir.z];
      const lo = [x0, y0, z0], hi = [x1, y1, z1];
      for (let k = 0; k < 3; k++) {
        if (Math.abs(dv[k]) < 1e-9) { if (o[k] < lo[k] || o[k] > hi[k]) return; continue; }
        let a = (lo[k] - o[k]) / dv[k], b = (hi[k] - o[k]) / dv[k];
        if (a > b) [a, b] = [b, a];
        t0 = Math.max(t0, a); t1 = Math.min(t1, b);
        if (t0 > t1) return;
      }
      hits.push({ t: t0, what, actor });
    };
    for (const o of OBSTACLES) box(o.x0, o.x1, o.y0, o.y1, 0, o.h, 'obstacle');
    const cylinder = (cx, cy, r, z0, z1, what, actor) => {
      const ox = origin.x - cx, oy = origin.y - cy;
      const a = dir.x * dir.x + dir.y * dir.y;
      if (a < 1e-12) return;
      const b = 2 * (ox * dir.x + oy * dir.y), c = ox * ox + oy * oy - r * r;
      const disc = b * b - 4 * a * c;
      if (disc < 0) return;
      for (const t of [(-b - Math.sqrt(disc)) / (2 * a), (-b + Math.sqrt(disc)) / (2 * a)]) {
        const z = origin.z + dir.z * t;
        if (t > 0 && z >= z0 && z <= z1) { hits.push({ t, what, actor }); return; }
      }
    };
    for (const tr of trees) cylinder(tr.x, tr.y, tr.r, 0, tr.h, 'tree');
    for (const a of actors) {
      if (a.kind === 'person') cylinder(a.pos.x, a.pos.y, a.radius, 0, a.height, 'person', a);
      else if (a.kind === 'drone') {
        const c = a.pos;
        const oc = origin.clone().sub(c);
        const bb = oc.dot(dir), cc = oc.lengthSq() - 0.3 * 0.3;
        const disc = bb * bb - cc;
        if (disc >= 0) { const t = -bb - Math.sqrt(disc); if (t > 0) hits.push({ t, what: 'drone', actor: a }); }
      } else {
        box(a.pos.x - 2.25, a.pos.x + 2.25, a.pos.y - 2.25, a.pos.y + 2.25, 0, 1.75, 'vehicle', a);
      }
    }
    hits.sort((p, r) => p.t - r.t);
    return hits;
  }

  // ------------------------------------------------------------------ LRF
  function fireLRF(reason = 'manual') {
    const { origin, dir } = headPose();
    // Pointing jitter: servo resolution (1.5 mrad) and tracking noise.
    const jitter = () => (Math.random() - 0.5) * 0.0016;
    dir.applyAxisAngle(new THREE.Vector3(0, 0, 1), jitter()).normalize();
    dir.z += jitter();
    dir.normalize();
    const hits = raycast(origin, dir).filter((h) => h.t > 0.3);
    const valid = hits.filter((h) => h.t >= 15);
    let chosen = [];
    if (st.lrf.mode === 1) chosen = valid.slice(0, 1);
    else if (st.lrf.mode === 2) chosen = valid.slice(-1);
    else chosen = valid.slice(0, 3);
    const noise = () => (Math.random() - 0.5) * 0.8;
    const ranges = chosen.map((h) => Math.max(15, h.t + noise()));
    const tx = lrfFrame(0x02);
    let status = 0x04; // out of range
    if (ranges.length) {
      status = st.lrf.mode === 3 ? ((ranges.length > 1 ? 0x3 : 0x0)) : (valid.length > 1 ? 0x01 : 0x00);
    }
    const rx = lrfRangeResponse(ranges[0] || 0, status);
    st.lrf.shots += 1;
    st.lrf.last = { ranges, hit: chosen[0], t: st.t, tooClose: hits.length > 0 && valid.length === 0 && hits[0].t < 15 };
    st.lrf.lastPacket = { tx, rx, status };
    const shown = chosen[0] ? chosen[0].t : (hits[0] ? hits[0].t : 300);
    beamGeo.setFromPoints([origin, origin.clone().addScaledVector(dir, shown)]);
    beam.computeLineDistances();
    beam.visible = true;
    hitMark.visible = !!chosen[0];
    if (chosen[0]) hitMark.position.copy(origin.clone().addScaledVector(dir, chosen[0].t));
    st.lrf.busy = 0.35;

    // Attach the range to the track that was hit.
    const hitActor = chosen[0] && chosen[0].actor;
    if (hitActor) {
      for (const tr of st.tracks.values()) {
        if (tr.actor === hitActor) {
          tr.range = ranges[0];
          tr.rangeSource = 'LRF';
          tr.rangedAt = st.t;
          const p = origin.clone().addScaledVector(dir, ranges[0]);
          tr.enu = { e: p.x, n: p.y, u: p.z };
          if (reason !== 'track' || !tr.reportedRange) {
            logEvent(hitActor.kind === 'drone' ? 'drone' : 'range', `${tr.name} ranged ${ranges[0].toFixed(1)} m · E ${p.x.toFixed(1)} N ${p.y.toFixed(1)} U ${p.z.toFixed(1)}`);
            tr.reportedRange = true;
          }
        }
      }
    }
    renderPackets();
    return ranges;
  }

  // -------------------------------------------------------------- detection
  const projected = new THREE.Vector3();
  function detect(dt) {
    const mode = st.camMode;
    headCam.updateMatrixWorld();
    const camPos = new THREE.Vector3();
    headCam.getWorldPosition(camPos);
    const seen = new Set();
    const boxes = [];
    for (const a of actors) {
      const c = a.center();
      const d = c.distanceTo(camPos);
      projected.copy(c).project(headCam);
      if (projected.z > 1 || Math.abs(projected.x) > 1 || Math.abs(projected.y) > 1) continue;
      let range = DETECT_RANGE[mode][a.kind];
      if (st.night && mode === 'color') range *= 0.06;
      if (mode === 'nir') {
        if (!st.ill.power && st.night) range *= 0.08;
        else if (st.ill.power && st.night) {
          // Only what the illuminator lights is visible.
          const { origin, dir } = headPose();
          const v = c.clone().sub(origin).normalize();
          const inside = Math.acos(clamp(v.dot(dir), -1, 1)) < (st.ill.spot / 2) * DEG * 1.15;
          range *= inside ? Math.sqrt(st.ill.brightness / 255) * Math.sqrt(12 / st.ill.spot) * 1.4 : 0.08;
        }
      }
      if (d > range) continue;
      // Line of sight (trees and walls block).
      const dir = c.clone().sub(camPos).normalize();
      const block = raycast(camPos, dir, d).find((h) => h.what !== a.kind && h.what !== 'ground' && h.t < d - 0.5);
      if (block) continue;
      const conf = clamp(1.04 - (d / range) * 0.7 + (Math.random() - 0.5) * 0.04, 0.3, 0.98);
      seen.add(a);
      let tr = [...st.tracks.values()].find((x) => x.actor === a);
      if (!tr) {
        const prefix = a.kind === 'person' ? 'P' : a.kind === 'drone' ? 'D' : 'V';
        tr = { id: st.nextTrackId++, actor: a, name: `${prefix}-${String(st.nextTrackId - 1).padStart(2, '0')}`, range: null, rangeSource: null, first: st.t, reportedRange: false };
        st.tracks.set(tr.id, tr);
        logEvent(a.kind === 'drone' ? 'drone' : 'track', a.kind === 'drone'
          ? `Drone detected (${mode}), ${tr.name}`
          : `New ${a.kind} ${tr.name} (${mode}, ${(conf * 100).toFixed(0)} %)`);
      }
      tr.last = st.t;
      tr.conf = conf;
      tr.mode = mode;
      // Bearing / elevation from the robot.
      const rel = c.clone().sub(new THREE.Vector3(st.x, st.y, st.bodyHeight));
      tr.bearing = wrap(Math.atan2(rel.y, rel.x) - st.yaw);
      tr.elev = Math.atan2(rel.z, Math.hypot(rel.x, rel.y));
      if (tr.rangeSource !== 'LRF' || st.t - tr.rangedAt > 6) {
        // Monocular estimate for people and vehicles: depression angle of the
        // foot point over a flat ground plane.  Pixel quantisation and
        // terrain make it a few percent off, hence the per-track bias.
        if (a.kind !== 'drone') {
          tr.bias = tr.bias ?? 1 + (Math.random() - 0.5) * 0.14;
          const horiz = Math.hypot(a.pos.x - camPos.x, a.pos.y - camPos.y);
          const depression = Math.atan2(camPos.z - a.pos.z, horiz);
          tr.range = (camPos.z / Math.tan(depression)) * tr.bias;
          tr.rangeSource = 'ground';
        } else {
          tr.range = null;
          tr.rangeSource = null;
        }
      }
      // Bounding box in the inset image.
      const s = a.kind === 'vehicle' ? [4.5, 1.9, 1.8] : a.kind === 'drone' ? [0.5, 0.5, 0.15] : [0.55, 0.55, 1.75];
      let x0 = 1, x1 = -1, y0 = 1, y1 = -1;
      for (const dx of [-0.5, 0.5]) for (const dy of [-0.5, 0.5]) for (const dz of [0, 1]) {
        const p = new THREE.Vector3(a.pos.x + dx * s[0], a.pos.y + dy * s[1], (a.kind === 'drone' ? a.pos.z - s[2] / 2 : 0) + dz * s[2]).project(headCam);
        x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x); y0 = Math.min(y0, p.y); y1 = Math.max(y1, p.y);
      }
      boxes.push({ tr, x0, x1, y0, y1, conf });
    }
    // Zone analytics: works on tracked people positions.
    for (const a of actors) {
      if (a.kind !== 'person') continue;
      const inZone = insidePolygon(a.pos.x, a.pos.y, ZONE.pts);
      const tr = [...st.tracks.values()].find((x) => x.actor === a && st.t - x.last < 2);
      if (inZone && !a.inZone && tr) logEvent('zone', `${tr.name} entered ${ZONE.name}`);
      if (tr) a.inZone = inZone;
    }
    for (const [id, tr] of st.tracks) {
      if (st.t - tr.last > 8) {
        st.tracks.delete(id);
        if (st.selected === id) st.selected = null;
      }
    }
    return boxes;
  }

  // -------------------------------------------------------------- head aim
  function aimAt(target, dt) {
    // Convert the target to pan/tilt angles of the head.
    robot.root.updateMatrixWorld(true);
    const local = robot.pan.parent.worldToLocal(target.clone());
    const panOrigin = robot.pan.position;
    const v = local.sub(panOrigin);
    const pan = Math.atan2(v.y, v.x);
    const tilt = Math.atan2(v.z - geo.raw.head_tilt_height, Math.hypot(v.x, v.y));
    moveHead(pan, tilt, dt);
    return Math.hypot(wrap(pan - st.head.pan), tilt - st.head.tilt);
  }

  function moveHead(pan, tilt, dt) {
    const lim = geo.raw.head_pan_limits, tl = geo.raw.head_tilt_limits;
    pan = clamp(pan, lim[0], lim[1]);
    tilt = clamp(tilt, tl[0], tl[1]);
    const step = 0.088 * DEG; // STS3215 resolution
    const quant = (v) => Math.round(v / step) * step;
    st.head.pan += clamp(wrap(pan - st.head.pan), -2.2 * dt, 2.2 * dt);
    st.head.tilt += clamp(tilt - st.head.tilt, -1.6 * dt, 1.6 * dt);
    robot.setHead(quant(st.head.pan), quant(st.head.tilt));
  }

  function updateHead(dt) {
    if (!st.head.auto) { robot.setHead(st.head.pan, st.head.tilt); return; }
    let tr = st.selected != null ? st.tracks.get(st.selected) : null;
    if (!tr) {
      const live = [...st.tracks.values()].filter((x) => st.t - x.last < 1.0);
      const prio = { drone: 0, person: 1, vehicle: 2 };
      live.sort((a, b) => (prio[a.actor.kind] - prio[b.actor.kind]) || ((a.rangedAt || -99) - (b.rangedAt || -99)));
      tr = live.find((x) => !x.rangedAt || st.t - x.rangedAt > 7) || null;
    }
    if (tr && st.t - tr.last < 1.5) {
      const err = aimAt(tr.actor.center(), dt);
      tr.aimTime = (tr.aimTime || 0) + (err < 0.006 ? dt : 0);
      if (tr.aimTime > 0.6 && (!tr.rangedAt || st.t - tr.rangedAt > 2.5) && st.lrf.busy <= 0) {
        fireLRF('track');
        tr.aimTime = 0;
      }
      st.head.focus = tr.id;
    } else {
      // Search pattern: wide pan sweep, alternate horizon and sky.
      st.head.scanT += dt;
      const s = st.head.scanT;
      const pan = Math.sin(s * 0.42) * 2.3;
      const tilt = (Math.sin(s * 0.13) > 0.35 ? 0.42 : 0.02);
      moveHead(pan, tilt, dt);
      st.head.focus = null;
    }
  }

  // -------------------------------------------------------------- driving
  function driveCommand(dt) {
    const k = st.keys;
    const manual = ['w', 's', 'a', 'd', 'q', 'e', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].some((x) => k.has(x)) ||
      Math.hypot(st.stick.x, st.stick.y) > 0.05;
    if (manual && st.patrol) setPatrol(false);
    const lim = gen.maxCommand();
    const sl = st.speedLimit;
    let cmd = { vx: 0, vy: 0, wz: 0 };
    if (manual) {
      const f = (k.has('w') || k.has('arrowup') ? 1 : 0) - (k.has('s') || k.has('arrowdown') ? 1 : 0) - st.stick.y;
      const s = (k.has('a') ? 1 : 0) - (k.has('d') ? 1 : 0);
      const r = (k.has('q') || k.has('arrowleft') ? 1 : 0) - (k.has('e') || k.has('arrowright') ? 1 : 0) - st.stick.x;
      cmd = { vx: clamp(f, -1, 1) * lim.vx * sl, vy: clamp(s, -1, 1) * lim.vy * sl, wz: clamp(r, -1, 1) * lim.wz * sl };
    } else if (st.patrol) {
      if (st.observe > 0) {
        st.observe -= dt;
        if (st.observe <= 0) { setCrouch(false); logEvent('patrol', `${st.observeName} done, moving on`); }
      } else {
        const wp = PATROL[st.wp];
        const dx = wp.x - st.x, dy = wp.y - st.y;
        const dist = Math.hypot(dx, dy);
        if (dist < 0.22) {
          if (wp.observe) {
            st.observe = 9; st.observeName = wp.observe; setCrouch(true);
            logEvent('patrol', `${wp.observe}: crouch and scan`);
          }
          st.wp = (st.wp + 1) % PATROL.length;
        } else {
          const e = wrap(Math.atan2(dy, dx) - st.yaw);
          cmd.wz = clamp(1.6 * e, -lim.wz, lim.wz) * sl;
          cmd.vx = lim.vx * sl * Math.max(0, Math.cos(e)) ** 3 * clamp(dist / 0.6, 0.35, 1);
        }
      }
    }
    // Do not walk into walls: cancel motion that would enter one.
    const c = Math.cos(st.yaw), s = Math.sin(st.yaw);
    const nx = st.x + (cmd.vx * c - cmd.vy * s) * 0.35, ny = st.y + (cmd.vx * s + cmd.vy * c) * 0.35;
    st.blocked = blockedAt(nx, ny, 0.26, st.ground, geo.stepHeightMax);
    if (st.blocked) { cmd.vx = 0; cmd.vy = 0; }
    return cmd;
  }

  function terrainLookahead() {
    // Highest point in a corridor ahead of the walking direction (terrain node logic).
    const c = gen.command;
    const sp = Math.hypot(c.vx, c.vy);
    const dirB = sp > 0.01 ? Math.atan2(c.vy, c.vx) : 0;
    const a = st.yaw + dirB;
    let maxRise = 0;
    for (let d = 0.12; d <= 0.5; d += 0.04) {
      for (let w = -0.2; w <= 0.2; w += 0.05) {
        const x = st.x + Math.cos(a) * d - Math.sin(a) * w, y = st.y + Math.sin(a) * d + Math.cos(a) * w;
        maxRise = Math.max(maxRise, terrainHeight(x, y) - st.ground);
      }
    }
    return maxRise;
  }

  // ---------------------------------------------------------------- step
  function step(dt) {
    st.t += dt;
    for (const a of actors) a.update(st.t, dt);
    const cmd = driveCommand(dt);
    gen.setCommand(cmd);

    // Terrain: ground under every foot (stance) or under its touchdown point (swing).
    const stance = gen.inStance();
    for (let i = 0; i < 6; i++) {
      const p = stance[i] ? gen.feet[i] : gen.swingTarget(i);
      const w = worldFromStance(p);
      gen.setGroundHeight(i, terrainHeight(w.x, w.y));
    }
    if (st.autoStep) {
      const rise = terrainLookahead();
      gen.setStepHeight(rise > 0.012 ? Math.min(geo.stepHeightMax, Math.max(st.stepBase, rise + 0.035)) : st.stepBase);
    } else {
      gen.setStepHeight(st.stepBase);
    }
    gen.update(dt);

    // Integrate the body pose with the executed command (stance feet stay put).
    const c = gen.command;
    const cy = Math.cos(st.yaw), sy = Math.sin(st.yaw);
    st.x += (c.vx * cy - c.vy * sy) * dt;
    st.y += (c.vx * sy + c.vy * cy) * dt;
    st.yaw = wrap(st.yaw + c.wz * dt);
    // Body follows the mean ground smoothly, height target with rate limit.
    st.ground += (gen.meanGroundHeight() - st.ground) * Math.min(1, dt / 0.25);
    const target = st.crouch ? geo.bodyHeightRange[0] + 0.012 : st.userHeight;
    st.bodyHeight += clamp(target - st.bodyHeight, -0.06 * dt, 0.06 * dt);

    st.battery = Math.max(0, st.battery - dt * (gen.active ? 1 / 5400 : 1 / 14400) - (st.ill.power ? dt / 20000 : 0));
    st.lrf.busy -= dt;
    if (st.lrf.busy <= 0) { beam.visible = false; hitMark.visible = false; }
  }

  function poseRobot() {
    robot.root.position.set(st.x, st.y, 0);
    robot.root.rotation.z = st.yaw;
    const h = st.bodyHeight + st.ground;
    robot.body.position.set(st.pose.x, st.pose.y, h + st.pose.z);
    robot.body.rotation.set(st.pose.roll, st.pose.pitch, st.pose.yaw, 'ZYX');
    for (let i = 0; i < 6; i++) {
      const r = geo.inverse(i, stanceToBase(gen.feet[i], st.pose, h));
      q[i] = r.q;
      robot.setLeg(i, r.q);
    }
  }

  // ------------------------------------------------------------- lighting
  function applyTimeOfDay() {
    if (st.night) {
      scene.background = new THREE.Color(0x070a10);
      scene.fog = new THREE.Fog(0x070a10, 25, 170);
      hemi.intensity = 0.06; hemi.color.set(0x334466); hemi.groundColor.set(0x0b0d10);
      sun.intensity = 0.14; sun.color.set(0x9fb4ff);
      stars.visible = true;
    } else {
      scene.background = new THREE.Color(0xc5d2db);
      scene.fog = new THREE.Fog(0xc5d2db, 70, 420);
      hemi.intensity = 1.25; hemi.color.set(0xe3ecf2); hemi.groundColor.set(0x8a8170);
      sun.intensity = 2.3; sun.color.set(0xfff4e2);
      stars.visible = false;
    }
  }

  function setIlluminatorPhysics() {
    spot.angle = clamp((st.ill.spot / 2) * DEG, 0.5 * DEG, 40 * DEG);
    const len = 14;
    const r = Math.tan(spot.angle) * len;
    cone.scale.set(r, len, r);
    cone.rotation.set(0, 0, Math.PI / 2);
    cone.position.set(len / 2, 0, 0);
    cone.visible = st.ill.power && st.night;
  }

  // ------------------------------------------------------------ rendering
  function renderInset(boxes) {
    const mode = st.camMode;
    const cfg = CAMERAS[mode];
    const anchor = mode === 'thermal' ? robot.thermalAnchor : robot.lowlightAnchor;
    if (headCam.parent !== anchor) anchor.add(headCam);
    headCam.fov = (2 * Math.atan(Math.tan((cfg.hfov / 2) * DEG) * 0.75)) / DEG;
    headCam.updateProjectionMatrix();

    const savedBg = scene.background, savedFog = scene.fog;
    if (mode === 'thermal') {
      const swapped = [];
      scene.traverse((m) => {
        if (m.isMesh && m.layers.test(headCam.layers)) {
          swapped.push([m, m.material]);
          m.material = thermalMat(sceneTemp(m));
        } else if (m.isPoints) {
          swapped.push([m, m.material]);
          m.material = thermalPoints;
        }
      });
      scene.background = ironbow(0.03);
      scene.fog = new THREE.Fog(ironbow(0.18).getHex(), 35, 260);
      camRenderer.render(scene, headCam);
      for (const [m, mat] of swapped) m.material = mat;
    } else {
      // Color camera has an IR-cut filter: the illuminator is invisible to it.
      const ir = mode === 'nir' && st.ill.power;
      spot.intensity = ir ? 14 * (st.ill.brightness / 255) * Math.sqrt(12 / st.ill.spot) : 0;
      const savedHemi = hemi.intensity;
      if (mode === 'nir' && st.night) hemi.intensity = 0.12;
      camRenderer.render(scene, headCam);
      hemi.intensity = savedHemi;
    }
    scene.background = savedBg;
    scene.fog = savedFog;
    spot.intensity = st.ill.power && st.night ? 2.2 * (st.ill.brightness / 255) : 0;

    // Overlay: reticle, detections, LRF readout.
    const W = 640, H = 480;
    overlay.clearRect(0, 0, W, H);
    overlay.strokeStyle = 'rgba(255,176,138,0.85)';
    overlay.lineWidth = 1.5;
    overlay.beginPath();
    overlay.moveTo(W / 2 - 22, H / 2); overlay.lineTo(W / 2 - 6, H / 2);
    overlay.moveTo(W / 2 + 6, H / 2); overlay.lineTo(W / 2 + 22, H / 2);
    overlay.moveTo(W / 2, H / 2 - 22); overlay.lineTo(W / 2, H / 2 - 6);
    overlay.moveTo(W / 2, H / 2 + 6); overlay.lineTo(W / 2, H / 2 + 22);
    overlay.stroke();
    overlay.font = '500 17px "IBM Plex Mono", monospace';
    for (const b of boxes) {
      const x0 = (b.x0 + 1) / 2 * W, x1 = (b.x1 + 1) / 2 * W;
      const y0 = (1 - b.y1) / 2 * H, y1 = (1 - b.y0) / 2 * H;
      const col = b.tr.actor.kind === 'drone' ? '#ff7a3d' : b.tr.actor.kind === 'person' ? '#ffd166' : '#5ee0d2';
      overlay.strokeStyle = col;
      overlay.lineWidth = b.tr.id === st.head.focus ? 3 : 2;
      overlay.strokeRect(x0, y0, Math.max(4, x1 - x0), Math.max(4, y1 - y0));
      const label = `${b.tr.name} ${(b.conf * 100).toFixed(0)}%${b.tr.range ? ` ${b.tr.rangeSource === 'LRF' ? '' : '~'}${b.tr.range.toFixed(0)}m` : ''}`;
      const tw = overlay.measureText(label).width + 8;
      overlay.fillStyle = 'rgba(0,0,0,0.6)';
      overlay.fillRect(x0, Math.max(0, y0 - 22), tw, 20);
      overlay.fillStyle = col;
      overlay.fillText(label, x0 + 4, Math.max(15, y0 - 6));
    }
    const last = st.lrf.last;
    $('cam-range').textContent = !last ? '— m'
      : last.ranges.length ? `${last.ranges.map((r) => r.toFixed(1)).join(' / ')} m`
        : last.tooClose ? '< 15 m' : 'no echo';
  }

  // -------------------------------------------------------------- UI glue
  const setPressed = (el, on) => el.setAttribute('aria-pressed', on ? 'true' : 'false');
  function setPatrol(on) {
    st.patrol = on; setPressed($('btn-patrol'), on);
    if (!on && st.observe > 0) { st.observe = 0; setCrouch(false); }
    if (on) {
      // Resume from the nearest waypoint.
      let best = 0, bd = 1e9;
      PATROL.forEach((w, i) => { const d = Math.hypot(w.x - st.x, w.y - st.y); if (d < bd) { bd = d; best = i; } });
      st.wp = best;
    }
  }
  function setCrouch(on) { st.crouch = on; setPressed($('btn-crouch'), on); }
  function setGait(name) {
    gen.requestGait(name);
    for (const b of $('gait-seg').querySelectorAll('button')) setPressed(b, b.dataset.gait === name);
  }
  function setNight(on) {
    st.night = on;
    for (const b of $('tod-seg').querySelectorAll('button')) setPressed(b, (b.dataset.tod === 'night') === on);
    applyTimeOfDay(); setIlluminatorPhysics();
    logEvent('scene', on ? 'Night: switch to Night IR or Thermal' : 'Day');
    if (on && st.camMode === 'color') setCam('thermal');
  }
  function setCam(mode) {
    st.camMode = mode;
    for (const b of $('cam-seg').querySelectorAll('button')) setPressed(b, b.dataset.cam === mode);
    $('cam').classList.toggle('nir', mode === 'nir');
    $('cam-label').textContent = CAMERAS[mode].label;
    $('cam-tag').textContent = CAMERAS[mode].tag;
  }
  let lastIllPacket = null;
  function sendIll(kind) {
    if (kind === 'power') lastIllPacket = { p: illPacket(0x01, 0x01, st.ill.power ? 1 : 0, 0), what: `power ${st.ill.power ? 'on' : 'off'}` };
    if (kind === 'bright') lastIllPacket = { p: illPacket(0x01, 0x03, st.ill.brightness, 0), what: `brightness ${st.ill.brightness}` };
    if (kind === 'spot') {
      const v = illSpotInternal(st.ill.spot);
      lastIllPacket = { p: illPacket(0x08, 0x01, v >> 8, v & 0xff), what: `spot ${st.ill.spot.toFixed(1)}° → motor 0x${v.toString(16).toUpperCase().padStart(4, '0')}` };
    }
    renderPackets();
  }
  function setIll(on) {
    st.ill.power = on; setPressed($('btn-ill'), on);
    $('ill-tag').textContent = on ? 'on' : 'off';
    setIlluminatorPhysics(); sendIll('power');
    if (on && st.night && st.camMode !== 'thermal') setCam('nir');
  }

  function renderPackets() {
    const fmt = (arr) => arr.map(hex).join(' ');
    if (lastIllPacket) {
      $('ill-packet').innerHTML = `<span>TX</span> <b>${fmt(lastIllPacket.p)}</b> <span>· ${lastIllPacket.what}</span>`;
    } else {
      $('ill-packet').innerHTML = '<span>TCP 192.168.0.7:20108 or RS-485 9600 · idle</span>';
    }
    const lp = st.lrf.lastPacket;
    if (lp) {
      const r = st.lrf.last.ranges;
      $('lrf-packet').innerHTML = `<span>TX</span> <b>${fmt(lp.tx)}</b><br><span>RX</span> <b>${fmt(lp.rx)}</b> <span>· ${r.length ? r.map((v) => v.toFixed(1)).join(' / ') + ' m' : 'status 0x04 out of range'}</span>`;
    } else {
      $('lrf-packet').innerHTML = '<span>UART 115200 8N1 · single ranging = EE 16 02 03 02 05</span>';
    }
  }

  function renderEvents() {
    const ol = $('events');
    ol.replaceChildren(...st.events.slice(0, 18).map((e) => {
      const li = document.createElement('li');
      li.className = e.type;
      const t = document.createElement('time'); t.textContent = fmtTime(e.t);
      const s = document.createElement('span'); s.textContent = e.text;
      li.append(t, s);
      return li;
    }));
  }

  function renderTargets() {
    const tb = $('targets').querySelector('tbody');
    const rows = [...st.tracks.values()].sort((a, b) => (a.range ?? 1e9) - (b.range ?? 1e9));
    $('targets-tag').textContent = `${rows.length} tracked`;
    $('targets-empty').hidden = rows.length > 0;
    tb.replaceChildren(...rows.map((tr) => {
      const r = document.createElement('tr');
      if (tr.id === st.selected) r.className = 'sel';
      const brg = tr.bearing ?? 0;
      const cells = [
        `<span class="cls ${tr.actor.kind}">${tr.name}</span>`,
        tr.range == null ? '—' : `${tr.rangeSource === 'LRF' ? '' : '~'}${tr.range.toFixed(tr.rangeSource === 'LRF' ? 1 : 0)}`,
        `${brg >= 0 ? 'L' : 'R'} ${Math.abs(brg / DEG).toFixed(0)}°`,
        `${tr.elev >= 0 ? '+' : ''}${((tr.elev ?? 0) / DEG).toFixed(1)}°`,
      ];
      r.innerHTML = cells.map((c) => `<td>${c}</td>`).join('');
      r.title = tr.rangeSource === 'LRF' ? 'Range from the laser rangefinder' : 'Estimated from the ground plane';
      r.addEventListener('click', () => { st.selected = st.selected === tr.id ? null : tr.id; st.head.auto = true; setPressed($('btn-autotrack'), true); renderTargets(); });
      return r;
    }));
  }

  // Leg diagram.
  const svg = $('legs-svg');
  const S = 230; // px per metre in the diagram
  const feetEls = [];
  {
    const ns = 'http://www.w3.org/2000/svg';
    const body = document.createElementNS(ns, 'polygon');
    body.setAttribute('class', 'body');
    const order = [0, 1, 2, 5, 4, 3]; // lf lm lr rr rm rf around the body
    body.setAttribute('points', order.map((i) => `${(-geo.mounts[i].y * S).toFixed(1)},${(-geo.mounts[i].x * S).toFixed(1)}`).join(' '));
    svg.append(body);
    geo.names.forEach((n, i) => {
      const f = geo.neutralFoot(i);
      const line = document.createElementNS(ns, 'line');
      line.setAttribute('class', 'leg');
      line.setAttribute('x1', -geo.mounts[i].y * S); line.setAttribute('y1', -geo.mounts[i].x * S);
      line.setAttribute('x2', -f.y * S); line.setAttribute('y2', -f.x * S);
      const c = document.createElementNS(ns, 'circle');
      c.setAttribute('class', 'foot'); c.setAttribute('r', 7);
      c.setAttribute('cx', -f.y * S); c.setAttribute('cy', -f.x * S);
      const t = document.createElementNS(ns, 'text');
      t.textContent = n.toUpperCase();
      t.setAttribute('x', -f.y * S + (f.y > 0 ? -9 : 9)); t.setAttribute('y', -f.x * S + 3);
      t.setAttribute('text-anchor', f.y > 0 ? 'end' : 'start');
      svg.append(line, c, t);
      feetEls.push({ c, line });
    });
  }
  const jointsBody = $('joints').querySelector('tbody');
  const jointRows = geo.names.map((n) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${n.toUpperCase()}</td><td></td><td></td><td></td>`;
    jointsBody.append(tr);
    return tr.children;
  });
  const headRow = document.createElement('tr');
  headRow.innerHTML = '<td title="pan / tilt">HEAD</td><td title="pan"></td><td title="tilt"></td><td></td>';
  jointsBody.append(headRow);

  function renderUI() {
    const c = gen.command;
    const speed = Math.hypot(c.vx, c.vy);
    $('speed-tag').textContent = `${speed.toFixed(2)} m/s`;
    $('phase-tag').textContent = `phase ${gen.phase.toFixed(2)} · ${gen.gait}`;
    const stance = gen.inStance();
    feetEls.forEach((f, i) => {
      f.c.setAttribute('class', stance[i] ? 'foot on' : 'foot');
      const p = gen.feet[i];
      f.c.setAttribute('cx', (-p.y * S).toFixed(1)); f.c.setAttribute('cy', (-p.x * S).toFixed(1));
      f.line.setAttribute('x2', (-p.y * S).toFixed(1)); f.line.setAttribute('y2', (-p.x * S).toFixed(1));
    });
    jointRows.forEach((cells, i) => {
      cells[1].textContent = (q[i].coxa / DEG).toFixed(1);
      cells[2].textContent = (q[i].femur / DEG).toFixed(1);
      cells[3].textContent = (q[i].tibia / DEG).toFixed(1);
    });
    headRow.children[1].textContent = (st.head.pan / DEG).toFixed(1);
    headRow.children[2].textContent = (st.head.tilt / DEG).toFixed(1);
    headRow.children[3].textContent = '—';

    const volts = 9.9 + 2.7 * st.battery;
    const kv = [
      ['Mode', st.patrol ? (st.observe > 0 ? `observe ${st.observeName}` : `patrol → WP${st.wp + 1}`) : 'manual'],
      ['Position', `${st.x.toFixed(2)}, ${st.y.toFixed(2)} m`],
      ['Heading', `${((st.yaw / DEG + 360) % 360).toFixed(0)}°`],
      ['Body height', `${((st.bodyHeight) * 1000).toFixed(0)} mm`],
      ['Ground under feet', `${(st.ground * 1000).toFixed(0)} mm`],
      ['Step height', `${(gen.stepHeight * 1000).toFixed(0)} mm`],
      ['Command', `${c.vx.toFixed(2)} ${c.vy.toFixed(2)} ${c.wz.toFixed(2)}`],
      ['Battery 3S', `${volts.toFixed(2)} V · ${(st.battery * 100).toFixed(0)} %`],
      ['LRF shots', String(st.lrf.shots)],
    ];
    $('telemetry').innerHTML = kv.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');

    const chips = [
      { t: st.patrol ? (st.observe > 0 ? `OBSERVE ${st.observeName}` : 'PATROL') : 'MANUAL', c: '' },
      { t: gen.gait.toUpperCase(), c: '' },
      { t: `${speed.toFixed(2)} m/s`, c: '' },
      { t: st.night ? 'NIGHT' : 'DAY', c: st.night ? 'warn' : '' },
      { t: `BAT ${(st.battery * 100).toFixed(0)}%`, c: st.battery < 0.25 ? 'warn' : '' },
    ];
    if (st.blocked) chips.push({ t: 'BLOCKED', c: 'warn' });
    if (st.lrf.last && st.t - st.lrf.last.t < 4) chips.push({ t: `LRF ${st.lrf.last.ranges.length ? st.lrf.last.ranges[0].toFixed(1) + ' m' : '—'}`, c: 'laser' });
    $('hud').innerHTML = chips.map((x) => `<span class="chip ${x.c}"><i></i>${x.t}</span>`).join('');
    renderTargets();
  }

  // --------------------------------------------------------------- inputs
  window.addEventListener('keydown', (e) => {
    if (e.target instanceof HTMLInputElement) return;
    const k = e.key.toLowerCase();
    st.keys.add(k);
    if (k === ' ') { e.preventDefault(); fireLRF('manual'); }
    if (k === '1') setGait('tripod');
    if (k === '2') setGait('ripple');
    if (k === '3') setGait('wave');
    if (k === 'p') setPatrol(!st.patrol);
    if (k === 'c') setCrouch(!st.crouch);
    if (k === 'n') setNight(!st.night);
    if (k === 'l') setIll(!st.ill.power);
    if (k === 't') { st.head.auto = !st.head.auto; setPressed($('btn-autotrack'), st.head.auto); }
    if (k === 'g') { st.autoStep = !st.autoStep; setPressed($('btn-autostep'), st.autoStep); }
    if (k === 'f') { st.follow = !st.follow; setPressed($('btn-follow'), st.follow); }
    if (k === 'v') setCam({ color: 'nir', nir: 'thermal', thermal: 'color' }[st.camMode]);
    if (['arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k)) e.preventDefault();
  });
  window.addEventListener('keyup', (e) => st.keys.delete(e.key.toLowerCase()));
  window.addEventListener('blur', () => st.keys.clear());

  $('gait-seg').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) setGait(b.dataset.gait); });
  $('tod-seg').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) setNight(b.dataset.tod === 'night'); });
  $('cam-seg').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) setCam(b.dataset.cam); });
  $('lrf-seg').addEventListener('click', (e) => {
    const b = e.target.closest('button'); if (!b) return;
    st.lrf.mode = Number(b.dataset.mode);
    for (const x of $('lrf-seg').querySelectorAll('button')) setPressed(x, x === b);
    const frame = lrfFrame(0x03, [st.lrf.mode]);
    st.lrf.lastPacket = null;
    $('lrf-packet').innerHTML = `<span>TX</span> <b>${frame.map(hex).join(' ')}</b> <span>· set ${['', 'first', 'last', 'multi'][st.lrf.mode]} target</span>`;
  });
  $('btn-patrol').addEventListener('click', () => setPatrol(!st.patrol));
  $('btn-crouch').addEventListener('click', () => setCrouch(!st.crouch));
  $('btn-autostep').addEventListener('click', () => { st.autoStep = !st.autoStep; setPressed($('btn-autostep'), st.autoStep); });
  $('btn-follow').addEventListener('click', () => { st.follow = !st.follow; setPressed($('btn-follow'), st.follow); });
  $('btn-fire').addEventListener('click', () => fireLRF('manual'));
  $('btn-autotrack').addEventListener('click', () => { st.head.auto = !st.head.auto; setPressed($('btn-autotrack'), st.head.auto); });
  $('btn-ill').addEventListener('click', () => setIll(!st.ill.power));

  const bindRange = (id, out, fmt, apply) => {
    const el = $(id);
    const upd = () => { const v = Number(el.value); $(out).textContent = fmt(v); apply(v); };
    el.addEventListener('input', upd);
    upd();
  };
  bindRange('in-height', 'out-height', (v) => `${v} mm`, (v) => { st.userHeight = v / 1000; });
  bindRange('in-step', 'out-step', (v) => `${v} mm`, (v) => { st.stepBase = v / 1000; });
  bindRange('in-speed', 'out-speed', (v) => `${v} %`, (v) => { st.speedLimit = v / 100; });
  bindRange('in-bright', 'out-bright', (v) => String(v), (v) => { st.ill.brightness = v; if (lastIllPacket) sendIll('bright'); });
  bindRange('in-spot', 'out-spot', (v) => `${v.toFixed(1)}°`, (v) => { st.ill.spot = v; setIlluminatorPhysics(); if (lastIllPacket) sendIll('spot'); });

  // Touch joystick: up/down walks, left/right turns.
  const stick = $('stick');
  let stickId = null;
  const stickMove = (e) => {
    const r = stick.getBoundingClientRect();
    const dx = (e.clientX - (r.left + r.width / 2)) / (r.width / 2);
    const dy = (e.clientY - (r.top + r.height / 2)) / (r.height / 2);
    const n = Math.max(1, Math.hypot(dx, dy));
    st.stick = { x: dx / n, y: dy / n };
    stick.style.setProperty('--sx', `${(dx / n) * 34}px`);
    stick.style.setProperty('--sy', `${(dy / n) * 34}px`);
  };
  stick.addEventListener('pointerdown', (e) => { stickId = e.pointerId; stick.setPointerCapture(e.pointerId); stickMove(e); });
  stick.addEventListener('pointermove', (e) => { if (e.pointerId === stickId) stickMove(e); });
  const stickEnd = () => { stickId = null; st.stick = { x: 0, y: 0 }; stick.style.setProperty('--sx', '0px'); stick.style.setProperty('--sy', '0px'); };
  stick.addEventListener('pointerup', stickEnd);
  stick.addEventListener('pointercancel', stickEnd);

  // Narrow layout tabs.
  for (const b of document.querySelectorAll('.tabs button')) {
    b.addEventListener('click', () => {
      for (const x of document.querySelectorAll('.tabs button')) x.setAttribute('aria-selected', x === b ? 'true' : 'false');
      $('rail-left').dataset.hidden = b.dataset.tab !== 'left';
      $('rail-right').dataset.hidden = b.dataset.tab !== 'right';
    });
  }
  $('rail-right').dataset.hidden = 'true';
  $('rail-left').dataset.hidden = 'false';

  // Resize.
  const view = $('view');
  const resize = () => {
    const w = view.clientWidth, h = view.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(view);
  resize();

  // ----------------------------------------------------------------- loop
  applyTimeOfDay();
  setIlluminatorPhysics();
  renderPackets();
  setCam('color');
  logEvent('scene', 'Robot standing at the start line. Patrol route has 3 observation points.');
  $('loading').hidden = true;

  let last = performance.now();
  let acc = 0;
  let frame = 0;
  let boxes = [];
  let lastRobot = new THREE.Vector3();
  poseRobot();
  window.hexabot = { st, gen, fireLRF, setNight, setCam, setIll, setPatrol, step };

  function frameLoop(now) {
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    acc += dt;
    while (acc >= 0.01) { step(0.01); acc -= 0.01; }
    updateHead(dt);
    poseRobot();

    // Camera follows the robot without fighting the user's orbit.
    const rp = new THREE.Vector3(st.x, st.y, 0.12 + st.ground);
    if (st.follow) {
      const d = rp.clone().sub(lastRobot);
      camera.position.add(d);
      controls.target.add(d);
    }
    lastRobot.copy(rp);
    controls.update();
    sun.position.set(st.x - 6, st.y - 9, 14);
    sun.target.position.set(st.x, st.y, 0);

    if (frame % 3 === 0) {
      boxes = detect(dt * 3);
      renderInset(boxes);
    }
    const focus = st.tracks.get(st.selected ?? st.head.focus ?? -1);
    selRing.visible = !!focus;
    if (focus) {
      const p = focus.actor.pos;
      selRing.position.set(p.x, p.y, focus.actor.kind === 'drone' ? p.z : 0.03);
      const k = focus.actor.kind === 'vehicle' ? 4 : 1;
      selRing.scale.setScalar(k);
    }
    renderer.render(scene, camera);
    if (frame % 6 === 0) renderUI();
    frame++;
    window.DONE = true;
    requestAnimationFrame(frameLoop);
  }
  requestAnimationFrame(frameLoop);
}

main().catch((err) => {
  console.error(err);
  $('loading').textContent = `Could not start the simulation: ${err.message}`;
});
