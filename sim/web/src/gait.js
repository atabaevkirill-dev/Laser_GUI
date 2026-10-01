// JavaScript port of the hexabot_locomotion core (geometry.cpp, gait.cpp).
// Keep it line-for-line with the C++: sim/web/test/check_gait.mjs replays the
// script of tools/gait_trace.cpp and compares every joint angle.

export const NUM_LEGS = 6;

export const GAITS = {
  tripod: { duty: 0.5, offset: [0.0, 0.5, 0.0, 0.5, 0.0, 0.5], cycle: 0.8 },
  ripple: { duty: 2 / 3, offset: [0.0, 1 / 3, 2 / 3, 0.5, 5 / 6, 1 / 6], cycle: 1.05 },
  wave: { duty: 5 / 6, offset: [0.5, 2 / 3, 5 / 6, 0.0, 1 / 6, 1 / 3], cycle: 1.4 },
};

export const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const frac = (v) => { v %= 1.0; return v < 0 ? v + 1.0 : v; };

export function rotateZ(v, a) {
  const c = Math.cos(a), s = Math.sin(a);
  return { x: c * v.x - s * v.y, y: s * v.x + c * v.y, z: v.z };
}

/** Geometry from geometry.json (metres / radians). */
export class Geometry {
  constructor(j) {
    this.names = j.leg_names;
    this.mounts = j.mount_x.map((x, i) => ({ x, y: j.mount_y[i], yaw: j.mount_yaw[i] }));
    this.coxa = j.coxa_length;
    this.femur = j.femur_length;
    this.tibia = j.tibia_length;
    this.coxaLim = j.coxa_limits;
    this.femurLim = j.femur_limits;
    this.tibiaLim = j.tibia_limits;
    this.stanceRadius = j.stance_radius;
    this.bodyHeight = j.body_height;
    this.bodyHeightRange = j.body_height_range;
    this.stepHeight = j.step_height;
    this.stepHeightMax = j.step_height_max;
    this.raw = j;
  }

  neutralFoot(leg) {
    const m = this.mounts[leg];
    return { x: m.x + this.stanceRadius * Math.cos(m.yaw), y: m.y + this.stanceRadius * Math.sin(m.yaw), z: 0 };
  }

  legFrameFromBase(leg, p) {
    const m = this.mounts[leg];
    return rotateZ({ x: p.x - m.x, y: p.y - m.y, z: p.z }, -m.yaw);
  }

  baseFromLegFrame(leg, p) {
    const m = this.mounts[leg];
    const r = rotateZ(p, m.yaw);
    return { x: r.x + m.x, y: r.y + m.y, z: r.z };
  }

  forwardLeg(q) {
    const reach = this.coxa + this.femur * Math.cos(q.femur) + this.tibia * Math.sin(q.femur + q.tibia);
    const z = this.femur * Math.sin(q.femur) - this.tibia * Math.cos(q.femur + q.tibia);
    return { x: reach * Math.cos(q.coxa), y: reach * Math.sin(q.coxa), z };
  }

  forward(leg, q) { return this.baseFromLegFrame(leg, this.forwardLeg(q)); }

  /** Analytic IK, knee-up branch, clamped to the joint limits. */
  inverse(leg, footBase) {
    const p = this.legFrameFromBase(leg, footBase);
    const L2 = this.femur, L3 = this.tibia;
    const coxa = Math.atan2(p.y, p.x);
    const r = Math.hypot(p.x, p.y) - this.coxa;
    const z = p.z;
    let c = (r * r + z * z - L2 * L2 - L3 * L3) / (2 * L2 * L3);
    const reachable = c >= -1 && c <= 1;
    c = clamp(c, -1, 1);
    const knee = -Math.acos(c);
    const femur = Math.atan2(z, r) - Math.atan2(L3 * Math.sin(knee), L2 + L3 * Math.cos(knee));
    const tibia = knee + Math.PI / 2;
    const inside = (v, lim) => v >= lim[0] - 1e-9 && v <= lim[1] + 1e-9;
    return {
      q: {
        coxa: clamp(coxa, ...this.coxaLim),
        femur: clamp(femur, ...this.femurLim),
        tibia: clamp(tibia, ...this.tibiaLim),
      },
      reachable,
      withinLimits: inside(coxa, this.coxaLim) && inside(femur, this.femurLim) && inside(tibia, this.tibiaLim),
    };
  }
}

/** Stance frame -> base_link for a body pose (R = Rz(yaw) Ry(pitch) Rx(roll)). */
export function stanceToBase(p, pose, height) {
  const v = { x: p.x - pose.x, y: p.y - pose.y, z: p.z - (height + pose.z) };
  const cr = Math.cos(pose.roll), sr = Math.sin(pose.roll);
  const cp = Math.cos(pose.pitch), sp = Math.sin(pose.pitch);
  const cy = Math.cos(pose.yaw), sy = Math.sin(pose.yaw);
  const a = { x: cy * v.x + sy * v.y, y: -sy * v.x + cy * v.y, z: v.z };
  const b = { x: cp * a.x - sp * a.z, y: a.y, z: sp * a.x + cp * a.z };
  return { x: b.x, y: cr * b.y + sr * b.z, z: -sr * b.y + cr * b.z };
}

export const ZERO_POSE = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };

export function swingHeightProfile(s) {
  s = clamp(s, 0, 1);
  return 0.5 * (1 - Math.cos(2 * Math.PI * s));
}

export function minJerk(s) {
  s = clamp(s, 0, 1);
  return s * s * s * (10 + s * (-15 + 6 * s));
}

const slew = (cur, tgt, step) => cur + clamp(tgt - cur, -step, step);
const isZero = (c, eps = 1e-6) => Math.abs(c.vx) < eps && Math.abs(c.vy) < eps && Math.abs(c.wz) < eps;

export class GaitGenerator {
  constructor(geo, cfg = {}) {
    this.geo = geo;
    this.cfg = {
      maxStride: 0.08, linearAccel: 0.6, angularAccel: 2.5,
      tripodCycle: 0.8, rippleCycle: 1.05, waveCycle: 1.4,
      settleTolerance: 0.004, searchDepth: 0.04, searchSpeed: 0.08, ...cfg,
    };
    this.gait = 'tripod';
    this.requestedGait = 'tripod';
    this.pattern = this.patternFor('tripod');
    this.target = { vx: 0, vy: 0, wz: 0 };
    this.command = { vx: 0, vy: 0, wz: 0 };
    this.stepHeight = geo.stepHeight;
    this.phase = 0;
    this.active = false;
    this.contactSensing = false;
    this.ground = new Array(NUM_LEGS).fill(0);
    this.expectedGround = new Array(NUM_LEGS).fill(0);
    this.contact = new Array(NUM_LEGS).fill(false);
    this.reset();
  }

  patternFor(name) {
    const p = GAITS[name];
    const cycle = { tripod: this.cfg.tripodCycle, ripple: this.cfg.rippleCycle, wave: this.cfg.waveCycle }[name];
    return { duty: p.duty, offset: p.offset.slice(), cycle };
  }

  reset() {
    this.feet = [];
    this.liftoff = [];
    this.wasStance = [];
    this.touched = [];
    this.hold = [];
    this.swingS0 = [];
    for (let i = 0; i < NUM_LEGS; i++) {
      const n = this.geo.neutralFoot(i);
      n.z = this.ground[i];
      this.feet.push(n);
      this.liftoff.push({ ...n });
      this.wasStance.push(true);
      this.touched.push(false);
      this.hold.push(false);
      this.swingS0.push(0);
    }
    this.command = { vx: 0, vy: 0, wz: 0 };
    this.active = false;
  }

  setCommand(c) { this.target = { vx: c.vx, vy: c.vy, wz: c.wz }; }
  requestGait(name) { if (GAITS[name]) this.requestedGait = name; }
  setStepHeight(h) { this.stepHeight = clamp(h, 0.01, this.geo.stepHeightMax); }
  setGroundHeight(leg, z) {
    this.expectedGround[leg] = z;
    if (!this.contactSensing) this.ground[leg] = z;
  }
  setContactSensing(on) { this.contactSensing = on; }
  setFootContact(leg, c) { this.contact[leg] = c; }

  inStance() {
    return this.feet.map((_, i) => !this.active || this.hold[i] || frac(this.phase + this.pattern.offset[i]) < this.pattern.duty);
  }

  meanGroundHeight() { return this.ground.reduce((a, b) => a + b, 0) / NUM_LEGS; }

  strideVector(leg, cmd) {
    const p = this.geo.neutralFoot(leg);
    const t = this.pattern.duty * this.pattern.cycle;
    return { x: (cmd.vx - cmd.wz * p.y) * t, y: (cmd.vy + cmd.wz * p.x) * t, z: 0 };
  }

  maxCommand() {
    const t = this.pattern.duty * this.pattern.cycle;
    let rMax = 0;
    for (let i = 0; i < NUM_LEGS; i++) {
      const n = this.geo.neutralFoot(i);
      rMax = Math.max(rMax, Math.hypot(n.x, n.y));
    }
    const v = this.cfg.maxStride / t;
    return { vx: v, vy: v, wz: v / rMax };
  }

  limitCommand(cmd) {
    const lim = this.maxCommand();
    const c = {
      vx: clamp(cmd.vx, -lim.vx, lim.vx),
      vy: clamp(cmd.vy, -lim.vy, lim.vy),
      wz: clamp(cmd.wz, -lim.wz, lim.wz),
    };
    let worst = 0;
    for (let i = 0; i < NUM_LEGS; i++) {
      const s = this.strideVector(i, c);
      worst = Math.max(worst, Math.hypot(s.x, s.y));
    }
    if (worst > this.cfg.maxStride) {
      const k = this.cfg.maxStride / worst;
      c.vx *= k; c.vy *= k; c.wz *= k;
    }
    return c;
  }

  swingTarget(leg) {
    const n = this.geo.neutralFoot(leg);
    const s = this.strideVector(leg, this.command);
    return { x: n.x + 0.5 * s.x, y: n.y + 0.5 * s.y, z: this.ground[leg] };
  }

  footSettled(leg) {
    const n = this.geo.neutralFoot(leg);
    const f = this.feet[leg];
    return Math.hypot(f.x - n.x, f.y - n.y) <= this.cfg.settleTolerance &&
      Math.abs(f.z - this.ground[leg]) <= this.cfg.settleTolerance;
  }

  settled() {
    for (let i = 0; i < NUM_LEGS; i++) if (!this.footSettled(i)) return false;
    return true;
  }

  update(dt) {
    if (dt <= 0) return;
    const gaitChange = this.requestedGait !== this.gait;
    const target = gaitChange ? { vx: 0, vy: 0, wz: 0 } : this.limitCommand(this.target);
    const c = this.command;
    c.vx = slew(c.vx, target.vx, this.cfg.linearAccel * dt);
    c.vy = slew(c.vy, target.vy, this.cfg.linearAccel * dt);
    c.wz = slew(c.wz, target.wz, this.cfg.angularAccel * dt);
    const wantMotion = !isZero(c, 1e-4) || !isZero(target, 1e-4);

    if (!this.active) {
      if (gaitChange) {
        this.gait = this.requestedGait;
        this.pattern = this.patternFor(this.gait);
        return;
      }
      if (!wantMotion) {
        if (!this.contactSensing) for (let i = 0; i < NUM_LEGS; i++) this.feet[i].z = this.ground[i];
        return;
      }
      this.active = true;
      for (let i = 0; i < NUM_LEGS; i++) {
        const psi = frac(this.phase + this.pattern.offset[i]);
        this.liftoff[i] = { ...this.feet[i] };
        this.touched[i] = false;
        this.swingS0[i] = 0;
        this.hold[i] = false;
        this.wasStance[i] = psi < this.pattern.duty;
        if (!this.wasStance[i]) {
          const s = (psi - this.pattern.duty) / (1 - this.pattern.duty);
          if (1 - s < 0.3) this.hold[i] = true; else this.swingS0[i] = s;
        }
      }
    }

    this.phase = frac(this.phase + dt / this.pattern.cycle);
    const duty = this.pattern.duty;

    for (let i = 0; i < NUM_LEGS; i++) {
      const psi = frac(this.phase + this.pattern.offset[i]);
      const stance = psi < duty;
      if (stance) {
        this.hold[i] = false;
      } else if (this.wasStance[i] && !wantMotion && this.footSettled(i)) {
        this.hold[i] = true;
      }
      const f = this.feet[i];
      if (stance || this.hold[i]) {
        if (!this.wasStance[i]) {
          if (this.contactSensing && !this.touched[i] && !this.contact[i]) this.ground[i] = f.z;
          this.touched[i] = false;
        }
        if (this.contactSensing && !this.contact[i]) {
          this.ground[i] = Math.max(this.ground[i] - this.cfg.searchSpeed * dt,
            this.expectedGround[i] - this.cfg.searchDepth);
        }
        const moved = rotateZ({ x: f.x, y: f.y, z: 0 }, -c.wz * dt);
        f.x = moved.x - c.vx * dt;
        f.y = moved.y - c.vy * dt;
        f.z = this.ground[i];
      } else {
        if (this.wasStance[i]) {
          this.liftoff[i] = { ...f };
          this.touched[i] = false;
          this.swingS0[i] = 0;
          if (this.contactSensing) this.ground[i] = this.expectedGround[i];
        }
        const sRaw = (psi - duty) / (1 - duty);
        const s = clamp((sRaw - this.swingS0[i]) / (1 - this.swingS0[i]), 0, 1);
        if (this.contactSensing && s > 0.5 && this.contact[i] && !this.touched[i]) {
          this.touched[i] = true;
          this.ground[i] = f.z;
        }
        const t = this.swingTarget(i);
        const b = minJerk(s);
        const lo = this.liftoff[i];
        f.x = lo.x + (t.x - lo.x) * b;
        f.y = lo.y + (t.y - lo.y) * b;
        if (this.touched[i]) {
          f.z = this.ground[i];
        } else {
          const base = lo.z + (this.ground[i] - lo.z) * b;
          f.z = base + this.stepHeight * swingHeightProfile(s);
        }
      }
      this.wasStance[i] = stance;
    }

    if (!wantMotion && this.settled()) {
      this.active = false;
      for (let i = 0; i < NUM_LEGS; i++) {
        this.feet[i].z = this.ground[i];
        this.wasStance[i] = true;
      }
    }
  }
}
