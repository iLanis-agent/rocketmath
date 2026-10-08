const e = require('./engine.js');
const exp = require('./expected.json');
let pass = 0, fail = 0;
const close = (a, b, tol) => (a === null && b === null) || (typeof a === 'number' && typeof b === 'number' && Math.abs(a - b) <= tol * Math.max(1, Math.abs(b)));
function cmp(g, w, path){
  if (Array.isArray(w)){ g.forEach((g2, i) => cmp(g2, w[i], path + '[' + i + ']')); return; }
  if (typeof w === 'object' && w !== null){ Object.keys(w).forEach(k => cmp(g[k], w[k], path + '.' + k)); return; }
  if (typeof w === 'string'){ if (g !== w) throw new Error(path + ': got ' + g + ' want ' + w); return; }
  if (!close(g, w, 1e-9)) throw new Error(path + ': got ' + g + ' want ' + w);
}
exp.cases.forEach((c, i) => {
  try {
    let got;
    if (c.kind === 'flight'){ const r = e.flight(...c.args); delete r.samples; got = r; }
    else if (c.kind === 'delayPick') got = e.delayPick(...c.args);
    else got = e.descent(...c.args);
    cmp(got, c.out, 'case' + i);
    pass++;
  } catch (err){ fail++; console.log('FAIL case', i, c.kind, err.message); }
});
// Anchors: no propellant mass change, no drag (tiny dia * tiny Cd won't work - use zero-CdA limit via small dia? Instead: Cd tiny)
(function(){
  // With Cd -> 0 and no prop mass, burnout speed = (T - m g)/m * burn; coast adds v^2/2g.
  const m = 0.05, T = 8, burn = 1.5;
  const r = e.flight(50, 0, T, burn, 1, 0.0001);
  const a = (T - m * 9.81) / m;
  const vB = a * burn, altB = 0.5 * a * burn * burn;
  if (!close(r.vBurn, vB, 1e-3)) throw new Error('anchor vBurn ' + r.vBurn + ' vs ' + vB);
  if (!close(r.altBurn, altB, 3e-3)) throw new Error('anchor altBurn'); // fixed-step Euler overshoots distance by ~dt/t (~0.2%)
  const coastT = vB / 9.81;
  if (!close(r.coastS, coastT, 2e-3)) throw new Error('anchor coast');
  const apogee = altB + vB * vB / (2 * 9.81);
  if (!close(r.apogeeM, apogee, 3e-3)) throw new Error('anchor apogee ' + r.apogeeM + ' vs ' + apogee); // same step bias
  // Descent terminal velocity exact.
  const d = e.descent(40, 450, 300);
  const vT = Math.sqrt(2 * 0.04 * 9.81 / (1.225 * 0.8 * Math.PI * Math.pow(0.225, 2)));
  if (!close(d.vTerm, vT, 1e-12)) throw new Error('anchor vTerm');
  if (!close(d.descentS, 300 / vT, 1e-12)) throw new Error('anchor descentS');
  pass += 6;
})();
// Properties.
(function(){
  // Heavier rocket, same motor: lower apogee.
  const a = e.flight(40, 3.1, 6, 1.6, 24.8, 0.6);
  const b = e.flight(90, 3.1, 6, 1.6, 24.8, 0.6);
  if (!(b.apogeeM < a.apogeeM)) throw new Error('mass monotonicity');
  // More thrust: higher apogee.
  const c = e.flight(40, 3.1, 12, 1.6, 24.8, 0.6);
  if (!(c.apogeeM > a.apogeeM)) throw new Error('thrust monotonicity');
  // Bigger chute: slower descent.
  const d1 = e.descent(40, 300, 300), d2 = e.descent(40, 600, 300);
  if (!(d2.vTerm < d1.vTerm)) throw new Error('chute monotonicity');
  // Delay pick best is never worse than any row.
  const dp = e.delayPick(40, 3.1, 6, 1.6, 24.8, 0.6, [3, 5, 7]);
  dp.rows.forEach(rw => { if (rw.speedMps !== null && !(dp.bestSpeedMps <= rw.speedMps + 1e-12)) throw new Error('best delay'); });
  pass += 4;
})();
// Error cases.
(function(){
  const bad = [
    () => e.flight(0, 3.1, 6, 1.6, 24.8, 0.6),
    () => e.flight(40, 3.1, 0, 1.6, 24.8, 0.6),
    () => e.flight(500, 3.1, 1, 1.6, 24.8, 0.6),
    () => e.delayPick(40, 3.1, 6, 1.6, 24.8, 0.6, []),
    () => e.delayPick(40, 3.1, 6, 1.6, 24.8, 0.6, [-1]),
    () => e.descent(40, 0, 300),
    () => e.drift(-1, 60),
  ];
  bad.forEach((f2, i) => {
    try { f2(); fail++; console.log('FAIL error case', i, 'did not throw'); }
    catch (err){ pass++; }
  });
})();
console.log(pass + '/' + (pass + fail) + ' checks pass');
process.exit(fail ? 1 : 0);
