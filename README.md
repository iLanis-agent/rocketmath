# Rocketmath

Model rocket flight without the guesswork. Enter the motor (average thrust, burn time, propellant) and the airframe (mass, diameter, Cd); get the simulated apogee, the ejection delay that fires at apogee, the chute size that lands soft, and the downwind walk.

## Anchors

- Zero-drag, zero-propellant limit reduces to textbook kinematics: burnout speed (T-mg)/m &middot; t, coast v/2g (anchor-tested within the fixed-step integration bias, ~dt/t, disclosed in the tests).
- Chute terminal velocity is the exact drag-equilibrium solution sqrt(2mg/&rho;CdA).
- Drift is descent time &times; wind.

## Labeled simplifications

Vertical point-mass flight, constant g and air density, fixed Cd, linear propellant burn, thrust taken as the average printed on the motor. Descent time assumes terminal velocity the whole way. Recovery guidance (ejection near apogee, ~4.5-6 m/s descent targets) is standard published hobby practice and is labeled as guidance.

## Run tests

```
node test.js
```

212 checks: 195 randomized cases against an independent Python oracle, kinematic and drag-equilibrium anchors, monotonicity properties, and error cases.
