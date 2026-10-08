// Rocketmath engine - model rocket flight physics.
// Model (labeled): vertical point-mass flight, constant g and air density, fixed
// Cd, linear propellant burn. The motor is described by average thrust and burn
// time (the numbers printed on the casing). Drag opposes velocity on the way up
// and on the way down. Recovery guidance (delay choice, descent rate targets)
// is standard published hobby practice, labeled where it is guidance.
const G = 9.81, RHO = 1.225, DT = 0.002;

// One integrator for the whole flight. Returns samples at requested times.
function integrate(massG, propG, thrustN, burnS, diaMm, Cd, tMax, sampleAt){
  if (!(massG > 0)) throw new Error('rocket mass must be positive');
  if (!(propG >= 0)) throw new Error('propellant mass cannot be negative');
  if (!(thrustN > 0)) throw new Error('motor thrust must be positive');
  if (!(burnS > 0)) throw new Error('burn time must be positive');
  if (!(diaMm > 0)) throw new Error('body diameter must be positive');
  if (!(Cd > 0)) throw new Error('drag coefficient must be positive');
  const mEmpty = massG / 1000, mProp = propG / 1000;
  const A = Math.PI * Math.pow(diaMm / 2000, 2);
  const k = 0.5 * RHO * Cd * A;
  if (thrustN <= (mEmpty + mProp) * G) throw new Error('motor cannot lift this rocket');
  let v = 0, s = 0, t = 0, apogee = 0, tApogee = 0, vBurn = null, altBurn = null;
  const out = [];
  let si = 0;
  while (t <= tMax){
    if (si < sampleAt.length && t >= sampleAt[si]){ out.push({ t, v, s }); si++; }
    const burning = t < burnS;
    const m = mEmpty + (burning ? mProp * (1 - t / burnS) : 0);
    const thrust = burning ? thrustN : 0;
    const drag = k * v * Math.abs(v);
    const a = (thrust - m * G - drag) / m;
    v += a * DT; s += v * DT; t += DT;
    if (vBurn === null && t >= burnS){ vBurn = v; altBurn = s; }
    if (s > apogee){ apogee = s; tApogee = t; }
    if (v < 0 && t > burnS && s <= apogee * 0.5 && apogee > 0) break; // well past apogee
  }
  return { vBurn, altBurn, apogeeM: apogee, tApogeeS: tApogee,
    coastS: tApogee - burnS, samples: out };
}

// Flight summary to apogee.
function flight(massG, propG, thrustN, burnS, diaMm, Cd){
  const r = integrate(massG, propG, thrustN, burnS, diaMm, Cd, 600, []);
  if (!(r.apogeeM > 0)) throw new Error('rocket never leaves the rail');
  return r;
}

// Delay pick: speed and altitude at ejection for each candidate delay (seconds
// after burnout). Ejection at apogee is the goal; fast ejection shreds chutes.
function delayPick(massG, propG, thrustN, burnS, diaMm, Cd, delays){
  if (!Array.isArray(delays) || delays.length === 0) throw new Error('give at least one delay');
  delays.forEach(d => { if (!(d >= 0)) throw new Error('delay cannot be negative'); });
  const tMax = burnS + Math.max(...delays) + 1;
  const sampleAt = delays.map(d => burnS + d).sort((a, b) => a - b);
  const r = integrate(massG, propG, thrustN, burnS, diaMm, Cd, tMax, sampleAt);
  const rows = delays.map((d, i) => {
    const smp = r.samples[i];
    if (!smp) return { delayS: d, speedMps: null, altM: null, phase: 'past model' };
    return { delayS: d, speedMps: Math.abs(smp.v), altM: smp.s,
      phase: smp.v > 0.5 ? 'ascending' : smp.v < -0.5 ? 'descending' : 'at apogee' };
  });
  let best = null;
  rows.forEach(rw => { if (rw.speedMps !== null && (best === null || rw.speedMps < best.speedMps)) best = rw; });
  return { apogeeM: r.apogeeM, tApogeeS: r.tApogeeS, coastS: r.coastS, rows,
    bestDelayS: best ? best.delayS : null, bestSpeedMps: best ? best.speedMps : null };
}

// Parachute descent: terminal velocity from drag equilibrium (exact), descent
// time from apogee assumes terminal velocity the whole way (labeled).
function descent(massG, chuteMm, apogeeM, cdChute){
  if (!(massG > 0)) throw new Error('rocket mass must be positive');
  if (!(chuteMm > 0)) throw new Error('chute diameter must be positive');
  if (!(apogeeM > 0)) throw new Error('apogee must be positive');
  const cd = cdChute || 0.8; // flat sheet chute, labeled default
  const m = massG / 1000;
  const A = Math.PI * Math.pow(chuteMm / 1000 / 2, 2);
  const vTerm = Math.sqrt(2 * m * G / (RHO * cd * A));
  return { vTerm, descentS: apogeeM / vTerm, areaM2: A };
}

// Drift: descent time times wind (exact given the labeled constant-wind model).
function drift(windMps, descentS){
  if (!(windMps >= 0)) throw new Error('wind cannot be negative');
  if (!(descentS > 0)) throw new Error('descent time must be positive');
  return { driftM: windMps * descentS };
}

const API = { G, RHO, flight, delayPick, descent, drift };
if (typeof module !== 'undefined' && module.exports) module.exports = API;
if (typeof window !== 'undefined') window.Rocketmath = API;
