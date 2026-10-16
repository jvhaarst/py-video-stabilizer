// Check that the browser port reproduces stabilize.py's camera-path maths.
//
// Two implementations of one algorithm drift apart silently. This pulls the
// pure functions straight out of index.html and runs them on the same
// synthetic input the Python used, so any divergence fails the build.
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const here = path.dirname(fileURLToPath(import.meta.url));
const html = fs.readFileSync(path.join(here, '..', 'index.html'), 'utf8');
const ref = JSON.parse(fs.readFileSync(path.join(here, 'fixture.json'), 'utf8'));

const slice = (from, to) => {
  const i = html.indexOf(from), j = html.indexOf(to);
  if (i < 0 || j < 0) throw new Error(`cannot find section: ${i < 0 ? from : to}`);
  return html.slice(i, j);
};
const source =
  slice('const rad = d =>', '// ------------------------------------------------------ feature tracking ----') +
  slice('function localAveragePose(', 'async function renderOutput(');

const g = globalThis;
g.srcW = ref.width; g.srcH = ref.height;
g.steps = ref.steps; g.cuts = [];
g.staticTail = () => null;
new Function(`${source}\nObject.assign(globalThis, {I3, mul, inv, trans, sim, decompose, unwrap, deg,
  buildPoses, posesToMatrices, boundsOf});`)();

const cx = ref.width / 2, cy = ref.height / 2;
const toC = g.trans(-cx, -cy), fromC = g.trans(cx, cy);
g.cumulative = [g.I3()];
for (let i = 1; i < g.steps.length; i++)
  g.cumulative.push(g.mul(g.mul(toC, g.mul(g.steps[i], fromC)), g.cumulative[i - 1]));
g.rollDeg = g.unwrap(g.cumulative.map(m => g.decompose(m).angle)).map(g.deg);

const fail = [];
const check = (what, got, want, tol) => {
  const d = Math.abs(got - want);
  if (!(d <= tol)) fail.push(`${what}: ${got} vs ${want} (diff ${d.toExponential(2)} > ${tol})`);
  return d;
};

let maxRoll = 0;
for (let i = 0; i < g.rollDeg.length; i++)
  maxRoll = Math.max(maxRoll, check(`roll[${i}]`, g.rollDeg[i], ref.rollDeg[i], 1e-9));

const poses = g.buildPoses(ref.lock, ref.sigma, ref.offsetDeg);
let maxPose = 0;
poses.forEach((p, i) => {
  const r = ref.poses[i];
  maxPose = Math.max(maxPose,
    check(`pose[${i}].angle`, p.angle, r[0], 1e-9),
    check(`pose[${i}].scale`, p.scale, r[1], 1e-9),
    check(`pose[${i}].tx`, p.tx, r[2], 1e-7),
    check(`pose[${i}].ty`, p.ty, r[3], 1e-7));
});

const b = g.boundsOf(g.posesToMatrices(poses), poses.length);
const maxBox = Math.max(
  check('bbox.x0', b.x0, ref.bbox[0], 1e-6), check('bbox.x1', b.x1, ref.bbox[1], 1e-6),
  check('bbox.y0', b.y0, ref.bbox[2], 1e-6), check('bbox.y1', b.y1, ref.bbox[3], 1e-6));

console.log(`frames        ${g.steps.length}`);
console.log(`roll  max diff ${maxRoll.toExponential(2)} deg`);
console.log(`pose  max diff ${maxPose.toExponential(2)}`);
console.log(`bbox  max diff ${maxBox.toExponential(2)} px`);
console.log(`canvas        ${Math.round(b.x1 - b.x0)}x${Math.round(b.y1 - b.y0)}`);

if (fail.length) {
  console.error(`\nFAILED — the browser port no longer matches stabilize.py:`);
  for (const f of fail.slice(0, 12)) console.error('  ' + f);
  if (fail.length > 12) console.error(`  ...and ${fail.length - 12} more`);
  process.exit(1);
}
console.log('\nOK — index.html reproduces stabilize.py');
