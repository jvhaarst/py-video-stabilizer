// Keep vendor/ identical to what package.json pins, so Renovate's version
// bumps actually reach the page instead of sitting in node_modules unused.
//
//   node tools/sync-vendor.mjs          copy from node_modules into vendor/
//   node tools/sync-vendor.mjs --check  fail if they differ (used by CI)
import crypto from 'crypto';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const files = [
  ['node_modules/mp4-muxer/build/mp4-muxer.js', 'vendor/mp4-muxer.js'],
  ['node_modules/mp4-muxer/LICENSE', 'vendor/mp4-muxer.LICENSE'],
];

const check = process.argv.includes('--check');
let bad = 0;

for (const [from, to] of files) {
  const src = path.join(root, from), dst = path.join(root, to);
  if (!fs.existsSync(src)) {
    console.error(`missing ${from} — run npm ci first`);
    process.exit(1);
  }
  const a = fs.readFileSync(src);
  const b = fs.existsSync(dst) ? fs.readFileSync(dst) : null;
  if (b && a.equals(b)) {
    console.log(`ok       ${to}`);
  } else if (check) {
    console.error(`STALE    ${to} does not match ${from}`);
    bad++;
  } else {
    fs.writeFileSync(dst, a);
    console.log(`updated  ${to}`);
  }
}

if (bad) {
  console.error('\nRun `node tools/sync-vendor.mjs` and commit the result.');
  process.exit(1);
}

const pkg = JSON.parse(fs.readFileSync(path.join(root, 'node_modules/mp4-muxer/package.json')));
console.log(`\nmp4-muxer ${pkg.version}`);

// opencv.js is built from source rather than taken from a registry, so there is
// no package to compare it with. Check it against the hash its provenance file
// records instead, which catches a corrupt or swapped copy.
const prov = fs.readFileSync(path.join(root, 'vendor/opencv.js.provenance'), 'utf8');
const want = /sha256\s+([0-9a-f]{64})/.exec(prov);
if (!want) {
  console.error('vendor/opencv.js.provenance records no sha256');
  process.exit(1);
}
const got = crypto.createHash('sha256')
  .update(fs.readFileSync(path.join(root, 'vendor/opencv.js'))).digest('hex');
if (got !== want[1]) {
  console.error(`STALE    vendor/opencv.js does not match its provenance`);
  console.error(`  recorded ${want[1]}`);
  console.error(`  actual   ${got}`);
  console.error('\nRebuild with tools/build-opencv-js.sh and update the provenance file.');
  process.exit(1);
}
console.log(`opencv.js matches its recorded sha256`);
