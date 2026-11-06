# Whole-body controller on the STM32H743 (NUCLEO-H743ZI2)

The controller core in `src/wbc/` is plain C++17 with no MuJoCo, no heap
allocation and no OS calls. The simulator runs it through `//:libwbc.so`, and
this directory cross-compiles the same sources for the Cortex-M7.

| File | Role |
|---|---|
| `../src/wbc/rbd.{h,cpp}` | rigid-body dynamics: mass matrix, bias forces, Jacobians, Jdot·qvel, CoM (matches MuJoCo to ~1e-14) |
| `../src/wbc/robot_model.h` | kinematic tree and mass properties, **generated** from `CAD/robot.xml` |
| `../src/wbc/wbc_default_params.h` | WbcParams (LQR gain, posture table, gains), **generated** from the Python setup |
| `../src/wbc/wbc_core.{h,cpp}` | the controller: `init(params)`, `reset()`, `step(qpos, qvel, command) -> torques` |
| `../src/wbc/wbc_capi.cpp` | host-only C API for the simulator (adds heap allocation and timing) |
| `CMakeLists.txt`, `cmake/arm-none-eabi.cmake` | cross build: BLASFEO + HPIPM (generic C target) + core + image |
| `stm32h743/` | startup code, linker script, and a bring-up `main.cpp` that times `step()` |

## Build

```bash
embedded/build.sh
```

This needs the Arm GNU toolchain. It is expected in
`~/.local/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi`; otherwise set
`ARM_TOOLCHAIN_DIR`. BLASFEO and HPIPM come from `~/software/acados/external`
(the same versions the simulator uses), or are fetched from GitHub at the same
commits.

Current footprint:

| Region | Used | Contents |
|---|---|---|
| Flash | 202 KB of 2 MB | code and the default parameter table |
| AXI SRAM | 241 KB of 512 KB | the `WbcCore` object, including a 192 KB HPIPM arena (174 KB used on x86) |
| DTCM | 16 KB reserved | stack |

## After changing the robot model

```bash
python3 tools/gen_robot_model.py    # regenerates robot_model.h and wbc_default_params.h
bazel build //:libwbc.so
python3 tests/wbc_rbd_check.py      # standalone dynamics vs MuJoCo
python3 tests/wbc_benchmark.py
embedded/build.sh
```

The Python wrapper refuses to use the C++ backend if `robot_model.h` does not
match the MuJoCo model. The generator enforces the model rules the core relies
on:
- one joint per body;
- only free and hinge joints;
- hinge anchors at the body origin;
- gear-1 joint motors.

## What the bring-up image does

`stm32h743/main.cpp` enables the caches and the DWT cycle counter, initialises
the controller from `wbc_default_params.h`, then calls `step()` forever on the
home pose with a small changing disturbance. It records min, max and total
cycles in the global `g_bench`, which you read with a debugger:

```
arm-none-eabi-gdb embedded/build/wbc_h743.elf
(gdb) target extended-remote :3333        # OpenOCD or ST-LINK gdbserver
(gdb) load
(gdb) continue        # then Ctrl-C
(gdb) p g_bench
```

The image runs on the reset clock (64 MHz HSI) because there is no PLL setup
yet. Multiply the times by 64/480 for full speed, or add clock setup first.

## Not done yet

- **Clock tree:** PLL to 480 MHz, with voltage scale 0 on rev V silicon.
- **Fast memory placement:** move the hot data and code into DTCM and ITCM.
- **Hardware interface:** IMU and encoder drivers, a state estimator (the
  controller needs `qpos` and `qvel` as MuJoCo lays them out), the motor torque
  interface, and a 500 Hz timer interrupt that calls `step()`.
- **Unused library code:** HPIPM and newlib error paths link `puts` and
  `malloc`. Neither is called in the control loop.
