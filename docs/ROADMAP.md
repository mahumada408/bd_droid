# BD droid roadmap (after v3)

State at commit 414fe0b: v3 gimbal hip inside the torso, all-RS05 direct drive,
torso-on-ground sit, WBC balancing crouch to -0.160 m plus sit mode. All
benchmarks pass in sim. The CAD is a single display model (`CAD/bd_droid_rs05.fs`).

Suggested order: 1 and 2 first. They decide whether this becomes a real robot,
and 1 may change the masses and dimensions that 2 depends on.

## 1. Make the CAD buildable — done (v3.1, see docs/BUILD.md)

Open follow-ups: RS05 connector position and count, rotor dowel-hole depth, IMU mount, weighing the parts.

Original scope:

- Split every printed part into a separate body, with real bolt bosses,
  printable split lines, and heat-set insert pockets.
- Size the bearing and insert pockets for the printer's tolerance.
- Route cables through the gimbal (the roll joint makes this tricky).
- Produce a bill of materials (bearings, M3 hardware, inserts, connectors) and
  a print list with orientations.
- Check whether the printed gimbal plates are strong enough, and whether the
  thigh or shin need carbon or aluminium inserts.

## 2. Close the sim-to-real gap

- RS05 torque falls off with speed at 48 V. Model it, along with the
  5.5 N·m peak and 1.6 N·m rated limits, and a thermal estimate for the knees in
  deep crouch.
- CAN bus latency and sensor noise.
- Randomise mass, battery position, inertia and friction.
- Run the benchmark suite with all of the above on and fix what breaks.

## 3. Embedded bring-up

- Nucleo H743 talking to the RS05s over CAN, and reading the IMU.
- Safety handling: e-stop and low-battery (33 V) cutoff.
- Run the C++ WBC onboard (it was built for this).
- Start with a single leg on the bench.

## 4. New controller behaviours

- Leaning into turns — done (carving, see below).
- Small jumps and hops off the crouch.
- Handling bumps and steps.
- Push recovery.
- A smooth shift from sitting straight into driving.

Each is mostly controller work plus a new benchmark scenario.

Carving (BD droid, `wbc_carve`): the whole robot leans into turns like a
motorcycle, up to 26 deg and 5 m/s^2, with both wheels equally loaded. The lean
leads the turn, the robot crouches so the outer leg can reach down, and the head
looks into the turn and stays level. Benchmarks: carve, carve_slalom,
carve_exit, carve_crouch. Clip: docs/media/bd_carve.gif.

Open follow-ups for carving:
- Robustness. Nominal sim is clean, but with every perturbation on at once
  (`--robust 8`) the four carve scenarios survive 38-62 %. Falls track leg-joint
  friction above about 1.5x nominal (the posture corrections get stuck) and
  floor friction below about 0.5. Measure the RS05 joint friction on the bench,
  and consider a slip-based grip estimate (traction control) that lowers
  a_lat_max on slick floors.
- In sustained hard carves the hips still splay outward (up to 0.33 rad on the
  slalom, the limit is 0.35). This comes from the posture task balancing
  against the height and roll tasks.
- deep_crouch_drive is marginal under perturbation: in the baseline it was in
  limits in 0 of 8 trials, and seeds 3 and 7 now fall. This predates carving.

## 5. Head and neck

- Bring the 3-DOF neck from the earlier design into v3.
- Style the chest panel area.
