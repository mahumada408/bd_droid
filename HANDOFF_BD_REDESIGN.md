# Handoff: restyle the robot as a BD-series droid

**Goal:** change how the robot looks (head, neck, body shell, leg styling) to resemble a BD-series droid from Star Wars, without breaking the whole-body controller (WBC) that balances and drives it.

**Branch:** `wbc-cpp-robustness`. Make a new branch from it.

**Read first:** `wbc_report.html` (repo root) has the full maths of the controller. This document covers only what a redesign can break.

---

## 1. Current state

- The robot is a wheel-legged biped of about 1.94 kg. It balances on two wheels like a Segway, while its legs keep the base level and set the ride height.
- It has 3 actuated leg joints per side (hip roll, hip pitch, knee) and a driven wheel at each foot.
- The model is `CAD/robot.xml`. Both `CAD/scene.xml` (flat floor and a platform) and `CAD/wavy_scene.xml` (wavy runway) include it.
- The meshes are in `CAD/assets/*.stl`. They were exported from Onshape with onshape-to-robot (`CAD/config.json`).
- **Do not re-run the Onshape export over `robot.xml` blindly.** The file has been hand-edited since the export: motors and torque limits, joint ranges, the `home` keyframe and the collision classes. Re-exporting would overwrite them. Re-export into a scratch file and merge by hand.
- **Controller:** C++ in `src/wbc/` (built as `//:libwbc.so`), with a Python reference and wrapper in `controllers/wbc/wbc_controller.py`.
  - The C++ core does **not** use MuJoCo. It has its own copy of the robot's kinematic tree and masses in `src/wbc/robot_model.h`, generated from `robot.xml` by `tools/gen_robot_model.py`. It also cross-compiles for the robot's STM32H743 (`embedded/`).
  - **After every change to `robot.xml`, re-run `python3 tools/gen_robot_model.py` and `bazel build //:libwbc.so`.** The Python wrapper refuses to start the C++ backend with a stale model, and `tests/wbc_rbd_check.py` checks the generated dynamics against MuJoCo.
- **GUI:** `bazel run //:sim_gui`, which runs `run_interactive_sim.py`.
- **Tests:**
  - `bazel test //:wbc_regression_test` runs `tests/wbc_benchmark.py`, a set of named driving and push scenarios with pass limits.
  - `python3 tests/wbc_benchmark.py --robust 20` runs every scenario 20 times under random model, sensor and actuator errors. It currently survives 100% (0 falls in 380 runs).

The current visual design is a plain base box with grey and blue parts and black wheels, so any restyle starts from scratch visually.

---

## 2. Hard contract: what the controller assumes about the model

Breaking any of these breaks balance, often silently. Either keep each one, or update the code listed next to it.

| Assumption | Where it is used |
|---|---|
| Body 1 is the base (`body_base`) and has the only `freejoint`. The base is the IMU: the controller reads attitude from body 1. | `wbc_controller.cpp` (`d->xmat + 9`, `qvel[0..5]`) and the benchmark |
| The robot drives forward along the base's **+y** axis, and +z is up at the home pose. The heading frame is built from column 1 of the base rotation matrix. | Heading frame, pitch and roll formulas |
| The wheel bodies are named `wheel` (right) and `wheel_2` (left). | `wbc_controller.py:174-175` |
| Joints are named `right_wheel`, `left_wheel`, `right_hip_roll` and `left_hip_roll`. | `wbc_controller.py:181-182` |
| Every actuator is a `motor` with gear 1, so `ctrl` is torque. The wheel actuators are the ones driving the wheel joints. **Every actuator that is not a wheel is treated as a "leg" joint.** It gets a posture task, a joint-limit look-ahead and safe-fall tucking. | `leg_idx`, `act_dof`, `wheel_act` |
| Every non-floating degree of freedom is actuated. A **passive joint (spring neck, wobbling antenna, free head) is not supported**: the QP would leave its acceleration unconstrained. Weld decorative parts rigidly into their parent body, or give them a motor. | QP structure |
| A `home` keyframe exists, its `qpos` matches the model, and it is a balanced standing pose. The nominal height, the lateral CoM offset, the posture table and the LQR pendulum model are all computed from it at start-up. | `PyWholeBodyController.__init__` |
| Wheel radius is 0.05 m (hard-coded). | `wbc_controller.py` (`self.wheel_radius`) |
| At most 16 actuators and 12 "leg" joints. | `src/wbc/wbc_params.h` |
| Model structure the C++ core supports: one joint per body; only free (body 1) and hinge joints; hinge anchors at the body origin (no joint `pos`) with zero `ref`; actuators are gear-1 joint motors. `tools/gen_robot_model.py` refuses anything else. | `src/wbc/rbd.cpp` |
| Contact is modelled as a thin wheel disk on flat ground: the contact point is the rim point under the hub. The wheel joint axis must be the wheel's spin axis. | `src/wbc/wbc_core.cpp` |
| The controller's working memory is fixed at compile time from `robot_model.h`. More joints means more microcontroller RAM (currently 241 KB of the H743's 512 KB). | `embedded/` |

Other code hard-codes the **current qpos and ctrl layout**: `qpos[7:15]` holds the 8 joints and `ctrl[0..7]` the 8 motors. This affects:
- the manual-drive, backflip and multi-flip modes and the reset code in `run_interactive_sim.py` (about 90 indexed accesses);
- `HOME_LEGS` and the start heights in `tests/wbc_benchmark.py`;
- the NMPC path (`export_acados.py`, `src/main_mpc.cpp`).

**To keep all of that working, any new joints and bodies must come *after* the existing ones.** MuJoCo orders qpos by tree order, so declare new child bodies of `body_base` after the two legs, and add new actuators at the end of the `<actuator>` list. The WBC itself adapts to extra actuators. The non-WBC modes in `run_interactive_sim.py` will ignore them, but those modes must then not crash.

---

## 3. Design considerations for a BD-style body

### Mass and centre of mass

The controller derives its balance model (mass, CoM height, pitch inertia) from the model automatically, so no gains need retuning just because mass moved. Some things still matter.

**Put the CoM over the axle at the home pose with a level base.**
- The controller holds the base level (pitch 0) and the CoM over the wheel axle at the same time.
- A big head on a forward-leaning neck moves the CoM forward. Those two goals then conflict, and you get a standing tilt, a creep, or the legs pushed toward their joint limits.
- The controller's slow lean-offset learning can only absorb about ±0.06 rad (3.4°).
- **Re-solve the `home` keyframe** so the whole-body CoM (`subtree_com[1]`) sits directly above the midpoint of the two wheel hubs, with the base level. Adjust hip and knee pitch, or move internal mass such as a battery to counterweight the head. Section 5 has a check.

**Keep total mass and CoM height in a sane range.**
- Today the total is 1.94 kg, with the CoM 0.135 m above the axle.
- A taller CoM (long neck, raised head) makes balancing *slower and easier*, but it raises wheel torque in hard braking and cornering. Wheels are limited to ±5 N·m and hips and knees to ±15 N·m.
- A heavy head far from the base adds pitch inertia the hips have to fight to keep the base level.
- Target: head plus neck under about 0.5 kg, and total mass under about 2.6 kg. The contact force bound `Fz_max = 60 N` per wheel holds up to about 6 kg. If you go much heavier, re-check the torque limits.

**Give every new body an explicit `<inertial>` (mass, CoM, inertia).** All current bodies have one, so MuJoCo ignores geom volumes for mass. A new body with no `<inertial>` gets its mass from geom density, including the visual-only meshes, and can easily come out wrong by 10×. Visual-only geoms should stay in the `visual` class.

**Lateral symmetry.**
- The current model is slightly asymmetric, and the controller copes because it measures the offset at the home pose.
- Keep the head on the centreline. A big sideways CoM offset forces a constant roll correction, and with it a hip-roll offset.

### Legs (the BD backward-bent knee look)

- BD-style legs bend backward (digitigrade). The current legs are a hip pitch plus knee chain ending in a wheel.
- **Easiest path:** change the visual meshes of `femur` and `tibia` so they *look* reverse-jointed, and keep the joint axes, positions and ranges.
- **If you change the real kinematics** (link lengths, knee direction, joint ranges):
  - re-solve `home`;
  - make sure the posture table still covers base-height offsets from −0.08 to about +0.015 m (it is built automatically by IK from `home`; check `c.dh_grid` after construction);
  - keep at least 0.05 rad between `home` and every joint limit.
- Keep the hip-roll joints. The controller uses them for roll balance, banking and the wavy runway.
- The track width (distance between wheels) sets roll stability. Narrowing it to look more like the droid makes sideways pushes and fast turns worse. Do not narrow it below the current width without re-running the robustness benchmark.

### Wheels

- The droid in the films walks, so the wheels will need styling: hub caps or "feet" around the wheels, for example.
- **Visual only.** Any foot or shroud geometry must not touch the ground. Give it `contype="0" conaffinity="0"` (the `visual` class) or keep it well clear of the tyre's contact patch.
- **Keep a wheel radius of 0.05 m**, or update `self.wheel_radius` and re-run all tests.
- Recommended, but separate work: collide the wheel with a `cylinder` instead of the mesh. The convex mesh hull makes the wheel load chatter between 0 and 18 N. That hasn't caused falls since the solver fix, but it is a known simulation artifact. Changing it alters the dynamics, so do it as its own commit with a before/after benchmark.

### Head and neck

- **Rigid head (recommended first step):** model it as more geoms in `body_base`, or as a child body with no joint (welded). That needs no controller changes beyond the mass and CoM work above.
- **Actuated neck (later):** add hinge joints *after* the legs, with motors at the end of the actuator list. The WBC will hold them at their `home` angle through the posture task, and safe fall will drive them to the posture row's value.
  - To animate the head (looking around), you will need a new input to the controller to set neck targets. Today the posture targets come only from the leg IK table, and the neck angles in that table equal `home`.
  - Gains: `kp_post = 60`, `kd_post = 12` (acceleration-level). A heavy head on a weak motor will lag.
  - Head motion is a disturbance to balance. Keep neck speeds modest.
- **Do not put the IMU in the head.** Base attitude comes from body 1, so leave the base as body 1.

### Collision geometry

- **The benchmark now counts any non-wheel body touching the ground as a fall.** A low-hanging head, antennae or tail that brushes the ground on the `ledge_drop` (an 8 cm drop) or the wavy runway will fail tests.
- Keep at least about 3 cm of ground clearance for every non-wheel part at the lowest crouch (base-height offset −0.08 m), and at about ±10° of pitch and roll.
- **Use simple collision shapes** (box, capsule, sphere) for the head, neck and shell, not the detailed visual meshes. Put fine detail (eyes, panels, greebles) in visual-only geoms.
- In a fall the head will hit first. Safe fall tucks the legs to the lowest posture and brakes the wheels. A head that sticks far forward lengthens the lever arm and raises impact speed. The safe-fall scenarios check that impact speed stays under 3.0 m/s (2.5 m/s for `fall_trip`).

### Heights and start poses

- If the base origin moves vertically, or the standing height changes, update every start height and expected height:
  - `body_base pos` and the `home` keyframe `z` (0.1172 on the platform) in `robot.xml`;
  - `PLATFORM`, `FLOOR` and the wavy start `z` in `tests/wbc_benchmark.py`, and `HOME_LEGS` there if the joint home angles change;
  - `z_start`, and `x_start` for the wavy runway, in `run_interactive_sim.py`.
- The robot must spawn with both wheels on the surface and nothing else touching it.

### Visual identity

These can change freely: materials and colours (the droid's colour scheme), lights, eye glow (emissive materials), decals and textures, and any geoms with `contype="0" conaffinity="0"` and no mass.

---

## 4. Suggested order of work

1. Build the head, shell and styling as **visual-only, massless geoms** on the existing bodies. Check in `bazel run //:sim_gui` that nothing about behaviour changed: run the benchmark, which should be identical.
2. Add mass: give the head or shell bodies real `<inertial>` values.
3. Re-solve `home` so the CoM is over the axle with the base level (section 5), and update start heights.
4. Add simple collision geoms for the new parts, then check ground clearance (the `ledge_drop`, `wavy` and `fall_*` scenarios).
5. Optional: change the leg kinematics or add an actuated neck, following the rules above.
6. Run the full validation after every step that touches mass, collisions or kinematics.

---

## 5. Validation (all must pass before handing back)

```bash
python3 tools/gen_robot_model.py                      # after any robot.xml change
bazel build //:libwbc.so
python3 tests/wbc_rbd_check.py                        # C++ dynamics vs MuJoCo
python3 tests/wbc_benchmark.py --backend cpp          # nominal suite
python3 tests/wbc_benchmark.py --backend python       # reference backend, should match closely
python3 tests/wbc_benchmark.py --robust 20 --jobs 8   # robustness; currently 100% survival
bazel test //:wbc_regression_test
bazel run //:sim_gui -- --headless --wavy --duration 8   # no "[SAFE FALL]" lines, Up-Z near 1
embedded/build.sh                                     # must still fit the STM32H743
```

**Balanced home check.** Both numbers should be about zero, within a few mm:

```python
import mujoco, numpy as np
m = mujoco.MjModel.from_xml_path("CAD/scene.xml"); d = mujoco.MjData(m)
d.qpos[:] = m.key_qpos[0]; mujoco.mj_forward(m, d)
w = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in ("wheel", "wheel_2")]
axle = 0.5 * (d.xpos[w[0]] + d.xpos[w[1]]); R = d.xmat[1].reshape(3, 3)
rel = d.subtree_com[1] - axle
print("fore-aft CoM offset (m):", rel @ R[:, 1], "  base pitch (rad):", np.arctan2(R[2, 1], R[2, 2]))
```

The current robot gives `0.00004 m` and `0.0 rad`. Match that.

**Notes on the benchmark limits:**
- The pass limits in `SCENARIOS` sit about 1.5× above the *current* robot's measured values. A heavier, taller robot may legitimately need some re-baselined, for example pitch RMS.
- Only change a limit if the new robot is physically different. Write down why in the scenario comment.
- Never loosen a limit to hide a fall, or safe fall triggering in a normal scenario.
- Any `fell` or `safe fall triggered` result in a normal scenario is a real regression.

**If balance gets worse after the redesign,** look at the mass layout first: home CoM, total mass, head inertia. Do not retune controller gains first. The controller's model is rebuilt from `robot.xml` at start-up, so most problems come from a home pose that isn't balanced or from a passive joint.

---

## 6. Do not

- Add unactuated joints (springs, free-swinging parts).
- Rename `body_base`, `wheel`, `wheel_2`, or the wheel and hip-roll joints. If you must rename them, update `wbc_controller.py` too.
- Insert new joints or actuators *before* the existing ones (it breaks the hard-coded indices).
- Overwrite `CAD/robot.xml` with a fresh Onshape export.
- Give visual meshes collision, or let any styling geometry touch the ground.
- Move the base frame or change its forward axis away from +y.
