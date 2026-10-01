// Replays the script of src/hexabot_locomotion/tools/gait_trace.cpp with the
// JavaScript gait and compares every joint angle with the C++ output.
//   node sim/web/test/check_gait.mjs [trace.json]
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Geometry, GaitGenerator, stanceToBase, ZERO_POSE } from '../src/gait.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const tracePath = process.argv[2] || path.join(here, 'gait_trace.json');
const trace = JSON.parse(fs.readFileSync(tracePath, 'utf8'));
const geo = new Geometry(JSON.parse(fs.readFileSync(path.join(here, '../assets/geometry.json'), 'utf8')));
const gen = new GaitGenerator(geo);

const script = [
  [0.5, [0.0, 0.0, 0.0], 'tripod'],
  [2.0, [0.15, 0.0, 0.0], 'tripod'],
  [1.5, [0.05, 0.05, 0.4], 'tripod'],
  [1.5, [0.0, 0.0, 0.0], 'tripod'],
  [2.5, [0.08, -0.03, 0.0], 'ripple'],
  [2.0, [0.05, 0.0, -0.2], 'wave'],
  [2.0, [0.0, 0.0, 0.0], 'wave'],
];

const dt = trace.dt;
let step = 0;
let idx = 0;
let worst = 0;
for (const [duration, [vx, vy, wz], gait] of script) {
  gen.requestGait(gait);
  gen.setCommand({ vx, vy, wz });
  const n = Math.round(duration / dt);
  for (let k = 0; k < n; k++, step++) {
    gen.update(dt);
    if (step % 10 !== 0) continue;
    const ref = trace.samples[idx++];
    for (let i = 0; i < 6; i++) {
      const r = geo.inverse(i, stanceToBase(gen.feet[i], ZERO_POSE, trace.body_height));
      const q = [r.q.coxa, r.q.femur, r.q.tibia];
      for (let j = 0; j < 3; j++) worst = Math.max(worst, Math.abs(q[j] - ref.q[3 * i + j]));
    }
  }
}
if (idx !== trace.samples.length) {
  console.error(`sample count mismatch: js ${idx}, c++ ${trace.samples.length}`);
  process.exit(1);
}
console.log(`compared ${idx} samples, max joint difference ${worst.toExponential(2)} rad`);
if (worst > 1e-5) {
  console.error('JavaScript gait differs from the C++ gait');
  process.exit(1);
}
