# BD droid v3.1 build guide

Everything here matches `CAD/bd_droid_rs05.fs` (v3.1, the default design).

- **Onshape build:** https://cad.onshape.com/documents/1adfb587f203f54c3d5717b2/w/a37d5d8438d81bff1410c3a9/e/b2aae361fe8bb2b1c49e988a
- **Clearance checks (in Onshape):**
  - No part overlaps another at ten poses: hip heights 33 (sitting), 60, 100, 150 and 210 mm, plus hip roll −0.35 and +0.12 rad at 60, 120 and 210 mm.
  - Every hip part was moved along its install path below, in 1 to 4 mm steps, and touches nothing on the way.

## What changed from v3 to make it buildable

| Area | v3 (display model) | v3.1 (buildable) |
|---|---|---|
| Gimbal | Closed 4-plate box: the pitch motor could not be fitted | Open on the inner side. The pitch motor drops in and slides outward through the outer-plate bore |
| Pitch motor | x 14..58 | x 9.5..53.5. Every gimbal edge is inside the torso wall line (|x| ≤ 56.5), so the gimbal drops in from the top |
| Thigh | Plate x 61..70 | Plus a hub boss (x 56.5..61, D30) reaching through the outer-plate bore |
| Mid bulkhead | Closed D36 bore | D31 bore with a top-open U-slot, so the roll motor's shaft slides down into it. Keeps 6 of the 8 stator screws, countersunk |
| Rear wall | Flat | 0.5 mm pocket, open to the top, that locates the roll motor's rear face |
| Front bulkhead | Closed 6800 bore | Split seat with a bolt-on bearing cap (2× M3 into inserts) |
| Lower side walls | Notched only at the hip socket | Socket runs forward to the front, and the wall steps in to x ±48 (front recess). Without this, hip roll pushed the thigh into the wall (up to 1.2 cm³ overlap) |
| Lid | 4 corner screws | 3 screws: rear-centre plus the two front corners. The rear corner bosses blocked the roll motors' drop-in path |
| Battery shelf | 4 side ledges | 4 full-height columns (front corners, rear-centre, mid-centre) that print without overhangs. Rear notches pass the leg harnesses |
| Power board | Posts at y −56 / 9 | Posts at y −56 / −30, outboard of the roll motors and behind the gimbals |
| Shin | Plain plate | 6 × 3 mm harness channel on the outer face, closed by the orange stripe cover |
| Wheel hub | 4 mm disc with a 1.5 mm hub cap over the screws | 6 mm disc with counterbored screws, plus an outer lip that holds the tyre in place |
| Motor interfaces | Clearance holes only | Real RS05 interfaces (below), screw grips sized to the RS05 thread depths |

Sim: `CAD_bd/gen_robot.py` was updated (pitch motor position, gimbal and hub mass). Total mass is still 4.34 kg, and every benchmark scenario passes.

## RS05 interfaces used everywhere

From the RS05 user manual, section 1.1 outline drawing.

- **Output (rotor):**
  - The D30 shaft stands 3 mm proud of the housing.
  - 6× M4 on PCD 24, **threads only 3.5 deep**: screw length = grip + 3.
  - 3× D4 H7 dowel holes on PCD 19. Printed parts take D4 h6 pins in 3.9 mm press holes; the pins carry the torque.
  - D9 centre bore.
- **Output-side housing (stator):**
  - 8× M3 on PCD 41.5, 7 deep.
  - A D35 × 0.5 step around the shaft. Mating bores are D35.3.
- **Rear face:** 4× M3 on PCD 38.5, 10 deep.
- **Connector:** power and CAN on an AMASS XT30PB(2+2) (mate: XT30(2+2)-F).
  - The manual's drawing does not show where the connector sits, or whether there are two (for daisy-chaining).
  - Check this on the first motor before routing harnesses (see Wiring).

## Printer fit check (do this first)

1. Run `python3 tools/export_fit_coupon.py` and print `CAD/print/fit_coupon.stl` in the material you will use for the leg parts.
2. Test each feature against real parts:

| Feature | Test | CAD value |
|---|---|---|
| A | Lay it on an RS05 stator face: bore over the step, 8× M3 line up | D35.3, 8× D3.4 |
| B | Bolt it to an RS05 output (M4 × 8): dowel pins press in, plate sits flat on the shaft | D9.5, 6× D4.5, 3× D3.9 |
| C | A 6800-2RS drops into the seat with no play | D19.1 × 5 |
| D | M3 × 5.7 and M3 × 4 heat-set inserts go in straight and hold | D4.0 |
| E | M3 and M4 countersunk heads sit flush | 90°, D6.4 and D8.2 |
| F | The D10 peg (printed horizontal, like the gimbal stub) takes a 6800 with a light press | D10 |

3. If a fit is off, change that diameter in `CAD/bd_droid_rs05.fs` (every hole size is written out at its call) and rebuild in Onshape.

## Printed parts

To export STLs: in the Onshape Part Studio, select the printed parts in the parts list, then Right-click → Export → STL, units mm, "Export each part as a separate file". Parts tagged "(ref)" are purchased items and are not printed.

**Materials:**
- **PA-CF** (or PETG-CF) for anything that carries leg loads. Use 6 perimeters and 40–60 % infill; the gimbal uses 100 %.
- **PETG or ASA** for the shell parts: 4 perimeters, 15–20 % gyroid.
- **TPU 95A** for the tyres.

L/R parts are mirror images; Onshape builds both.

| Part | Qty | Material | Print orientation | Supports | Notes |
|---|---|---|---|---|---|
| Torso shell | 1 | PETG / ASA | Floor on the bed | Yes, under the side-wall ledge over the sockets and recesses (z = hip + 38) and the r30 front-bottom round | 120 × 158 × 162 mm. Needs a ≥ 180 mm bed |
| Hip gimbal L/R | 2 | PA-CF, 100 % | Outer plate on the bed | None (stub and pad overhangs are short) | Most stressed printed part (see Strength) |
| Gimbal bearing cap L/R | 2 | PA-CF | Top face on the bed | None | |
| Thigh L/R | 2 | PA-CF | Outer face (cap pocket side) on the bed | None | Hub boss prints upward |
| Shin L/R | 2 | PA-CF | Outer face (channel side) on the bed | None | |
| Wheel hub | 2 | PA-CF / PETG | Outer lip on the bed | None | |
| Tyre | 2 | TPU 95A | Axis vertical | None | 3 walls, 15 % gyroid for some give. Glue onto the rim |
| Hip cap inlay, knee cap, shin stripe cover | 2 each | PETG orange | Flat | None | Inlay: CA glue. Stripe cover: 3M VHB tape |
| Battery shelf | 1 | PETG | Flat, lips up | None | |
| Battery dock carrier | 1 | PETG | Lid side on the bed | None | Takes 4× M3 × 5.7 inserts for the lid screws |
| Torso lid | 1 | PETG / ASA | Top face on the bed | None | |
| Chest panel, chest stripe/bars, chest LED | 1 set | PETG dark / orange | Flat | None | Glue or VHB to the front wall |
| Neck collar, yaw turntable, neck pitch bracket, head mount | 1 each | PETG | Flange / largest face down | Light, on the pitch bracket arms | Drill Dynamixel horn holes from the XL430 drawing, or use stock FR12 frames |
| Head shell | 1 | PETG / ASA | Open back on the bed | Yes, under the brow chamfer | 176 × 78 × 72 mm |
| Head rear cover, eye bezel, secondary eye, ear pods, antenna bases, bands | 1 set | PETG | Flat | None | |

## Hardware

**Fasteners** (ISO 4762 socket head unless noted):

| Item | Qty | Where |
|---|---|---|
| M4 × 10 countersunk (DIN 7991) | 12 | Gimbal back plate → roll output (grip 7, 3 mm thread) |
| M4 × 12 | 24 | Thigh → pitch output, shin → knee output (grip 9 in counterbores) |
| M4 × 6 low head (DIN 7984) | 12 | Wheel hub → wheel output (grip 3) |
| M3 × 8 countersunk (DIN 7991) | 12 | Roll stator → mid bulkhead (6 per side) |
| M3 × 14 countersunk (DIN 7991) | 24 | Knee stator → thigh (16), wheel rear face → shin (8) |
| M3 × 8 | 43 | Roll rear face → rear wall (8), pitch stator → gimbal (16), shelf (4), lid (3), neck collar (4), head rear cover (4), head mount (4) |
| M3 × 20 | 4 | Gimbal bearing caps |
| M3 × 10 | 4 | Lid → dock carrier |
| M3 × 6 | 5 | NUCLEO-H743ZI |
| M3 × 5 | 4 | Power / CAN board |
| M2.5 × 8 plastite (self-tapping for plastic) | 12 | Knee caps (6), eye bezels (6) |
| M3 × 5.7 heat-set insert | 28 | Bearing caps 4, shelf 4, lid 3, Nucleo 5, dock carrier 4, head rear cover 4, head mount 4 |
| M3 × 4 heat-set insert (short) | 8 | Power board posts 4, neck collar (in the 4 mm lid) 4 |

**Bearings and pins:**

| Item | Qty | Where |
|---|---|---|
| 6800-2RS (10 × 19 × 5) | 2 | Gimbal front pivot |
| Dowel pin D4 h6 × 10 | 6 | Gimbal ↔ roll output |
| Dowel pin D4 h6 × 12 | 12 | Thigh ↔ pitch output, shin ↔ knee output |
| Dowel pin D4 h6 × 8 | 6 | Wheel hub ↔ wheel output |

Pins should stand about 3 mm proud of the printed part. The manual does not give the rotor's dowel-hole depth, so check it before ordering, and shorten the pins if the holes are shallower.

**Electrical:**

| Item | Qty | Notes |
|---|---|---|
| RobStride 05 | 8 | 4 per leg |
| Dynamixel XL430-W250-T | 3 | Neck |
| NUCLEO-H743ZI | 1 | Has 2 FDCAN peripherals: one CAN bus per leg (see Wiring) |
| CAN transceiver (TJA1051T/3 or similar) + 120 Ω | 2 | One per bus. The CAD shows one transceiver block; fit two on the power board |
| IMU breakout (e.g. ICM-42688-P or BNO085) | 1 | Not in the CAD yet. Mount it rigidly on the Nucleo standoffs or the shelf, near the CoM |
| Makita XGT BL4025 + third-party XGT dock adapter with 12 AWG leads | 1 | Check whether the pack needs an enable handshake |
| DC-DC 36 V → 12 V 30 W (e.g. Mean Well SKMW30F-12), DC-DC 36 V → 5 V (R-78HB5.0-0.5) | 1 each | 12 V for the Dynamixels, 5 V for the Nucleo |
| Mini blade fuse holder + 20 A fuse | 1 | |
| XT30(2+2)-F plugs | 8–16 | One or two per motor, depending on its connector count |
| Silicone wire 18 AWG (power), 24 AWG twisted pair (CAN) | ~3 m each | |

## Assembly order

Every hip step below drops in from the open top of the torso. The Onshape install-path check confirms that steps 3 to 5 clear each other, with the other side already installed.

1. **Inserts.** Press the heat-set inserts:
   - torso: 2 per bearing cap, 4 shelf, 3 lid, 5 Nucleo, 4 power board
   - lid: 4 neck collar
   - dock carrier: 4
   - head: rear cover 4, head mount 4
2. **Motors on the bench.** For each RS05: set its CAN ID (1..4 on each leg's bus), set the mechanical zero, fit the plug, and label it.
3. **Hip roll motor.**
   1. Output forward. Lower it into the rear bay so its shaft slides down the mid-bulkhead U-slot and its rear face drops into the rear-wall pocket.
   2. Fit 4× M3 × 8 through the rear wall.
   3. Fit 6× M3 × 8 countersunk from the front of the mid bulkhead.
4. **Gimbal.**
   1. Press 3 dowels (D4 × 10) into the back plate and a 6800-2RS onto the stub.
   2. Lower the gimbal between the bulkheads: the pad lands on the roll output and the bearing drops into the front-bulkhead seat.
   3. Fit 6× M4 × 10 countersunk from inside the open gimbal (a short hex key fits).
   4. Fit the bearing cap with 2× M3 × 20.
5. **Hip pitch motor.**
   1. Output outboard. Lower it into the gimbal about 6 mm inboard of its final spot, then slide it outward until the shaft passes through the outer-plate bore.
   2. Fit 8× M3 × 8 from outside, through the hip socket.
6. **Other side.** Repeat steps 3–5 mirrored.
7. **Harness and shelf.** Run the leg harnesses (see Wiring), then fit the battery shelf (4× M3 × 8). The harnesses go up through the shelf's rear corner notches.
8. **Electronics.**
   1. Power / CAN board on its 4 posts; fuse holder beside the battery.
   2. Nucleo on its 5 front standoffs.
   3. Dock adapter into the carrier, carrier to the lid (4× M3 × 10).
9. **Legs, per side.**
   1. **Wheel:** glue the tyre onto the hub rim, press 3 dowels (D4 × 8) into the hub, and fit the hub to the wheel motor with 6× M4 × 6 low-head.
   2. **Wheel motor:** fit it to the shin with 4× M3 × 14 countersunk from the shin's inner face. The heads sit flush, so the shin can fold over the thigh.
   3. **Knee motor:** fit it to the thigh with 8× M3 × 14 countersunk from the thigh's outer face.
   4. **Shin to knee:** press 3 dowels (D4 × 12) into the shin hub and fit 6× M4 × 12 into the knee output. Fit the knee cap with 3× M2.5.
   5. **Thigh to hip:** press 3 dowels (D4 × 12) into the thigh hub. Through the hip socket, fit 6× M4 × 12 into the pitch output, then glue the hip-cap inlay over the screw heads.
10. **Finish.** Lid (3× M3 × 8), neck collar, neck, head.

## Wiring

**Two CAN buses, one per leg**, on the H743's FDCAN1 and FDCAN2, each terminated with 120 Ω at both ends. At 1 Mbit/s, one command plus one reply frame (about 130 bits each) for 4 motors at the 500 Hz control rate uses about 52 % of a bus. Putting all 8 motors on one bus would need over 100 %.

**Power:** battery dock leads (12 AWG) → 20 A fuse → power board → one 18 AWG pair per leg, plus the DC-DCs.

Leg harness route, from the wheel to the power board:

1. Wheel motor plug → **shin channel** (6 × 3 mm under the stripe cover, from just outside the motor's rear rim to below the knee).
2. **Knee service loop** behind the knee: about 2.5 rad of travel, so leave about 60 mm of slack and zip-tie both ends.
3. Knee motor plug joins → along the **thigh slot** (18 mm wide, zip-tie around the ligaments).
4. **Hip service loop** in the socket, around the pitch axis (1.6 rad of travel; about 50 mm of slack).
5. Into the lower bay under the gimbal. The pitch motor plug (at its rear face, near the torso centre) and the roll motor plug (at the rear wall; D14 access hole) join here. The gimbal's roll travel (±0.35 rad) needs only about 15 mm of slack.
6. Through the **harness pass** in the mid bulkhead (8 × 8 mm, outboard, near the top) → up through the **shelf notch** at that rear corner → to the power board.

The neck wiring leaves through the 12 mm hole in the lid.

## Strength (hand estimates; check after the first drop tests)

**Loads assumed:** 4.34 kg robot, 3 g dynamic load, so about 64 N per wheel. The tyre centre is about 40 mm outboard of the thigh and shin plates.

| Part | Worst case | Section | Stress | PA-CF in-plane strength* |
|---|---|---|---|---|
| Shin | 2.6 N·m out-of-plane bending | 27 × 8 mm web, minus the channel | ≈ 11 MPa | 60–100 MPa |
| Thigh | Same moment | Two 17 × 9 mm flanges around the slot | ≈ 6 MPa | 60–100 MPa |
| Thigh | 5.5 N·m (knee peak) in-plane | | < 2 MPa | |
| Gimbal outer plate (3 mm) | Stator ring carries the leg's roll moment (≈ 4.7 N·m at 3 g) | | ≈ 12 MPa | |

\* Strength is for printed layers loaded in-plane; all the leg parts are printed flat so that bending stays in-plane.

- **Gimbal outer plate:** this has the lowest margin of any printed part. If it cracks or flexes, add a 2 mm 6061 doubler plate under the stator screws (same hole pattern, 8 mm longer screws).
- **Roll motor:** at a 3 g load it needs about 4.7 N·m (peak rating 5.5 N·m). The 1 g steady load is about 1.55 N·m, consistent with the sim.
- **Not checked:** the RS05 output-bearing moment rating (radial and tilting) is not in the manual. The wheel and hip-pitch outputs carry the full wheel offset moment, so ask RobStride for it.

## Still open

- **Motor connector:** where the RS05 connector sits, and whether there is one or two.
- **Rotor dowel holes:** their depth, which sets the dowel pin lengths.
- **IMU:** mounting is not modelled yet.
- **Dynamixel screw patterns:** not modelled. Drill them from the ROBOTIS drawing.
- **Mass:** weigh the printed parts and update `CAD_bd/gen_robot.py` (it assumes 4.34 kg total).
