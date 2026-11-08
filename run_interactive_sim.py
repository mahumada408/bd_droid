#!/usr/bin/env python3
"""
Interactive Real-Time MuJoCo Simulation for the 8-DOF Wheeled Biped Balancer Robot.
Features:
  - Dual Hip Roll Abduction/Adduction for dynamic cornering bank-lean
  - Independent 100mm hub wheel torque control with 3D orientation invariance
  - Active suspension with landing impact damping for the 8cm platform ledge drop
  - Interactive keyboard driving (W/A/S/D/X) and vertical posture control (C/T/N)
  - Robust repeatable 360-degree backflip stunts with auto re-settling & multi-flip chaining
"""

import math
import os
import sys
import time
import argparse
import contextlib
import ctypes
import numpy as np
import mujoco
import mujoco.viewer


def load_acados_mpc():
    """Attempts to load the compiled acados C++ NMPC shared library."""
    candidates = [
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "bazel-bin", "libbalancer_mpc.so"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "libbalancer_mpc.so"),
        "bazel-bin/libbalancer_mpc.so",
        "libbalancer_mpc.so",
    ]
    if "RUNFILES_DIR" in os.environ:
        candidates.insert(0, os.path.join(os.environ["RUNFILES_DIR"], "_main", "libbalancer_mpc.so"))

    for path in candidates:
        if os.path.isfile(path):
            try:
                lib = ctypes.CDLL(path)
                lib.mpc_create.restype = ctypes.c_void_p
                lib.mpc_destroy.argtypes = [ctypes.c_void_p]
                lib.mpc_step.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_double, ctypes.c_double,
                    ctypes.c_double, ctypes.c_double,
                    ctypes.c_double, ctypes.c_double,
                    ctypes.c_double, ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_int,
                    ctypes.POINTER(ctypes.c_double),
                    ctypes.POINTER(ctypes.c_double)
                ]
                handle = lib.mpc_create()
                if handle:
                    return lib, handle, path
            except Exception as e:
                pass
    return None, None, None


def main():
    parser = argparse.ArgumentParser(description="Interactive MuJoCo simulation for wheeled biped robot.")
    parser.add_argument("--auto-flip", action="store_true", help="Automatically trigger backflip stunt")
    parser.add_argument("--repeat-flips", type=int, default=1, help="Number of consecutive backflips to execute (default: 1)")
    parser.add_argument("--flip-time", type=float, default=0.5, help="Simulation time to trigger backflip (default: 0.5s)")
    parser.add_argument("--duration", type=float, default=None, help="Maximum simulation duration in seconds")
    parser.add_argument("--log-pitch", type=str, default=None, help="Path to write base pitch log CSV")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode without GUI window")
    parser.add_argument("--wavy", action="store_true", help="Load dual-lane offset wavy terrain runway (<cad>/wavy_scene.xml)")
    parser.add_argument("--speed", type=float, default=None, help="Target forward cruising speed in m/s")
    parser.add_argument("--controller", type=str, choices=["wbc", "nmpc"], default="wbc", help="Active controller: 'wbc' (Hierarchical Whole-Body QP) or 'nmpc' (acados SQP-RTI NMPC)")
    parser.add_argument("--cad", type=str, default="CAD", help="Robot model directory: CAD (original robot, default) or CAD_bd (BD droid)")
    args, unknown = parser.parse_known_args()

    if args.repeat_flips > 1:
        args.auto_flip = True

    here = os.path.dirname(os.path.abspath(__file__))
    cad_dir = args.cad if os.path.isabs(args.cad) else os.path.join(here, args.cad)
    if not os.path.isdir(cad_dir):
        cad_dir = args.cad
    # The backflip, the cascaded fallback balancer and the acados NMPC are
    # hand-tuned for the original robot (CAD/): 1.9 kg, 15 N m leg motors, no
    # neck. Any other model runs on the whole-body controller only, which builds
    # its model from the MuJoCo XML.
    orig_robot = os.path.basename(os.path.normpath(cad_dir)) == "CAD"
    if not orig_robot:
        if args.controller != "wbc":
            parser.error(f"--controller {args.controller} is only tuned for CAD/; use the WBC with {args.cad}")
        if args.auto_flip:
            parser.error(f"backflips are only tuned for CAD/, not {args.cad}")

    scene_file = "wavy_scene.xml" if args.wavy else "scene.xml"
    model_xml = os.path.join(cad_dir, scene_file)

    print(f"[Init] Loading MuJoCo model from: {model_xml}")
    m = mujoco.MjModel.from_xml_path(model_xml)
    d = mujoco.MjData(m)
    dt_sim = m.opt.timestep

    # Calibrated Nominal Stance for Rev 3 (aligns Center of Mass directly over wheel axles)
    qh_R_nom = 0.314503    # Right hip pitch (+18.02 deg)
    qk_R_nom = -0.052919   # Right knee (-3.03 deg)
    qh_L_nom = -0.212551   # Left hip pitch (-12.18 deg)
    qk_L_nom = 0.253279    # Left knee (+14.51 deg)

    # Initial platform position and home targets
    # Wheel track center is at x_root = +0.0705m. Spawning at -0.28m gives symmetric margin on 1.2m platform.
    is_multi_flip = args.auto_flip or args.repeat_flips > 1
    if args.wavy:
        x_start = 0.0705
        y_home = -0.80
        z_start = 0.0772
        x_home = 0.0705
    else:
        x_start = -0.28 if is_multi_flip else 0.0
        y_home = -0.12 if is_multi_flip else 0.0
        z_start = 0.1172
        x_home = 0.07 if is_multi_flip else 0.0

    # Joint layout shared by every model: legs at qpos[7:15] / ctrl[0:8]
    # (right roll, pitch, knee, wheel; then left), extra joints (the BD neck)
    # after them, held at their home pose by the WBC.
    leg_joints = ("right_hip_roll", "right_hip_pitch", "right_knee", "right_wheel",
                  "left_hip_roll", "left_hip_pitch", "left_knee", "left_wheel")
    # The v2 BD droid has no hip roll: legs at qpos[7:13] (right pitch, knee, wheel; then left).
    leg_joints_v2 = ("right_hip_pitch", "right_knee", "right_wheel", "left_hip_pitch", "left_knee", "left_wheel")
    def _adr(names):
        ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in names]
        return None if min(ids) < 0 else [m.jnt_qposadr[i] for i in ids]
    has_roll = _adr(leg_joints) == list(range(7, 15))
    if not has_roll and _adr(leg_joints_v2) != list(range(7, 13)):
        raise SystemExit(f"{model_xml}: leg joints must be qpos[7:15] in the order {leg_joints}"
                         f" or qpos[7:13] in the order {leg_joints_v2}")
    wheel_R_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel")
    wheel_L_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "wheel_2")
    home_qpos = np.array(m.key_qpos[0]) if m.nkey else np.zeros(m.nq)

    def spawn():
        """Resting on ground/platform in the nominal stance, zero velocity."""
        d.qpos[:] = home_qpos
        d.qpos[0] = x_start
        d.qpos[1] = y_home
        d.qpos[2] = z_start
        d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        if not has_roll:            # v2 layout: legs from the keyframe home pose
            d.qvel[:] = 0.0
            mujoco.mj_forward(m, d)
            return

        d.qpos[7] = 0.0         # right_hip_roll
        d.qpos[8] = qh_R_nom    # right_hip_pitch
        d.qpos[9] = qk_R_nom    # right_knee
        d.qpos[10] = 0.0        # right_wheel
        d.qpos[11] = 0.0        # left_hip_roll
        d.qpos[12] = qh_L_nom   # left_hip_pitch
        d.qpos[13] = qk_L_nom   # left_knee
        d.qpos[14] = 0.0        # left_wheel
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)

    spawn()

    print("\n" + "=" * 75)
    print("      Acrobat / BDX Wheeled Biped: 8-DOF Drive, Lean & Active Suspension")
    print(f"      Robot: {model_xml} ({m.nu} actuators, {float(np.sum(m.body_mass)):.2f} kg)")
    print("=" * 75)
    print("  Driving & Cornering Controls:")
    print("    - DRIVE FORWARD:    Press 'W' or Up Arrow     (+0.2 m/s per tap)")
    print("    - SLOW / REVERSE:   Press 'S' or Down Arrow   (-0.2 m/s per tap)")
    print("    - STEER LEFT (LEAN):Press 'A' or Left Arrow   (+0.35 rad/s + Active Bank; BD droid: +0.5, carves)")
    print("    - STEER RIGHT(LEAN):Press 'D' or Right Arrow  (-0.35 rad/s + Active Bank; BD droid: -0.5, carves)")
    print("    - BRAKE / STOP:     Press 'X'                 (Locks stationary position)")
    print("  Vertical Posture Controls:")
    print("    - CROUCH DOWN:      Press 'C'                 (Tap repeatedly to crouch lower; BD droid: down to sitting)")
    print("    - STAND TALL:       Press 'T'                 (Tap repeatedly to stand taller)")
    print("    - NOMINAL HEIGHT:   Press 'N'                 (Reset to nominal stance)")
    if orig_robot:
        print("  Acrobatic Stunt:")
        print(f"    - 360 BACKFLIP:     Press 'B'                 (Explosive jump, tucked flip & landing) [Repeat: {args.repeat_flips}]")
    if args.wavy:
        print("  Terrain Mode:")
        print("    - WAVY RUNWAY:      Dual offset wave lanes active (180° out-of-phase)!")
        print("                        Left & Right wheels drive at different heights at different times.")
        print("                        Active suspension roll leveling keeps chassis level during traversal.")
    print("  Simulation Controls:")
    print("    - POKE / PERTURB:   Right-Click + Drag on robot body in viewer")
    print("    - RESET TO PLATFORM:Backspace key")
    print("    - PAUSE / RESUME:   Space bar")
    print("    - QUIT:             Close window or press Ctrl+C")
    print("=" * 75 + "\n")

    # Balancer gains (Ascento-tuned optimal whole-body control)
    kp_th = 58.0
    kd_th = 7.8
    kp_v = 0.35
    kd_v = 0.12
    ki_v = 0.08
    max_tilt = 0.18

    # Steering gains
    kp_yaw = 2.2
    kd_yaw = 0.35
    k_roll_bank = 0.35   # rad roll per rad/s yaw rate (interactive keyboard banking)
    k_knee_diff = 0.18

    # Leg stance & suspension gains
    kp_leg = 70.0
    kd_leg = 5.0
    kp_roll = 85.0
    kd_roll = 5.5
    kd_knee_R = 4.5
    kd_knee_L = 3.5

    # Initialize Selected Controller (Ascento Whole-Body QP or acados NMPC)
    wbc_solver = None
    if args.controller == "wbc":
        try:
            from controllers.wbc import WholeBodyController
            wbc_solver = WholeBodyController(m)
            qp_name = "C++/HPIPM" if wbc_solver.backend == "cpp" else "Python/OSQP"
            print(f"[Controller] LQR-assisted Whole-Body QP active ({qp_name}, 20 vars: qdd + contact forces, "
                  "friction cones, joint limits, heading hold, 500 Hz)")
        except Exception as e:
            if not orig_robot:
                raise SystemExit(f"[Error] WholeBodyController failed on {model_xml}: {e}")
            print(f"[Warning] Could not initialize WholeBodyController ({e}). Falling back to NMPC.")
            wbc_solver = None

    mpc_lib, mpc_handle, mpc_so_path = load_acados_mpc() if orig_robot else (None, None, None)
    if args.controller == "nmpc" or wbc_solver is None:
        if mpc_lib is not None:
            print(f"[Controller] acados SQP-RTI Nonlinear MPC Active ({os.path.basename(mpc_so_path)}, 8 states, 3 controls, ~3.8 kHz)")
            mpc_out_ctrl = (ctypes.c_double * 4)()
            mpc_solve_time = ctypes.c_double()
        else:
            print("[Controller] acados NMPC library not found; using calibrated cascaded balancer.")
            mpc_out_ctrl = None
            mpc_solve_time = None
    else:
        mpc_out_ctrl = None
        mpc_solve_time = None

    last_sim_time = d.time
    step_count = 0

    if args.speed is not None:
        v_target = args.speed
    elif args.wavy and args.headless:
        v_target = 0.35
    else:
        v_target = 0.0

    yaw_rate_target = 0.0
    v_filtered = 0.0
    yaw_filtered = 0.0
    v_int = 0.0

    qk_target = 0.0
    qk_current = 0.0
    # Crouch range in posture units (delta_h = 0.10 * qk). Models with a deep posture table and the
    # sit mode (CAD_bd) go down to their balancing floor and one press further sits on the ground.
    qk_min, qk_step = -0.55, 0.15
    if wbc_solver is not None and getattr(wbc_solver, "sit_enable", 0):
        qk_min, qk_step = (wbc_solver.dh_min - 0.02) / 0.10, 0.25

    # Multi-flip state tracking
    target_flips = args.repeat_flips
    completed_flips = 0
    flip_records = []

    # Current flip variables
    is_backflipping = False
    bf_timer = 0.0
    bf_state = "idle"
    bf_step_count = 0
    bf_t_land = 0.0
    bf_unwrapped_pitch = 0.0
    bf_last_raw_pitch = 0.0
    settle_timer = 0.0
    target_yaw = 0.0

    current_flip_apex_z = 0.0
    current_flip_max_rot = 0.0
    current_flip_touchdown_deg = 0.0
    current_flip_air_collisions = 0

    # Milestone tracking
    milestones = {
        "crouch": False,
        "whip": False,
        "pop": False,
        "quarter": False,
        "half": False,
        "three_quarter": False,
        "landed": False,
        "settled": False,
    }

    # Carving models (wbc_carve) lean the whole robot into turns and can turn much harder
    carving = bool(getattr(wbc_solver, "carve", False))
    yaw_max, yaw_step = (3.0, 0.5) if carving else (1.80, 0.35)

    def print_status(action_msg):
        if carving:
            lean_deg = math.degrees(math.atan(min(abs(v_target * yaw_rate_target), wbc_solver.a_lat_max) / 9.81))
            lean_deg = min(lean_deg, math.degrees(wbc_solver.max_bank))
            side = "Left" if v_target * yaw_rate_target > 0 else "Right"
            lean_str = f" (Carve {side} ~{lean_deg:.0f}°)" if abs(yaw_rate_target) > 0.05 and abs(v_target) > 0.05 else ""
        else:
            lean_str = " (Deep Bank Left)" if yaw_rate_target > 0.05 else (" (Deep Bank Right)" if yaw_rate_target < -0.05 else "")
        if 0.10 * qk_target < getattr(wbc_solver, "dh_min", -1):
            posture_str = "Sitting"
        elif qk_target < -0.35:
            posture_str = f"Deep Crouch ({qk_target:+.2f})"
        elif qk_target < -0.05:
            posture_str = f"Crouch ({qk_target:+.2f})"
        elif qk_target > 0.05:
            posture_str = f"Tall ({qk_target:+.2f})"
        else:
            posture_str = "Nominal"
        print(f"[{action_msg:<14}] Speed: {v_target:+.2f} m/s | Turn: {yaw_rate_target:+.2f} rad/s{lean_str:<18} | Stance: {posture_str}")

    def trigger_flip():
        nonlocal is_backflipping, bf_timer, bf_step_count, bf_t_land, bf_state
        nonlocal bf_unwrapped_pitch, bf_last_raw_pitch, v_target, yaw_rate_target
        nonlocal qk_target, v_filtered, v_int, target_yaw
        nonlocal current_flip_apex_z, current_flip_max_rot, current_flip_touchdown_deg, current_flip_air_collisions
        nonlocal milestones

        if is_backflipping:
            return

        R_curr = d.xmat[1].reshape(3, 3)
        th_curr = float(np.arctan2(R_curr[2, 1], R_curr[2, 2]))
        yaw_head = float(np.arctan2(R_curr[1, 0], R_curr[0, 0]))

        flip_num = completed_flips + 1
        print(f"\n>>> [FLIP #{flip_num:02d} / {target_flips}] Launching at t={d.time:.3f}s (Pos X: {d.qpos[0]:+.3f}m, Pos Y: {d.qpos[1]:+.3f}m, Heading: {np.degrees(yaw_head):+.1f}°, Pitch: {np.degrees(th_curr):+.1f}°) <<<")

        is_backflipping = True
        bf_timer = 0.0
        bf_step_count = 0
        bf_t_land = 0.0
        bf_state = "crouch"
        bf_unwrapped_pitch = 0.0
        bf_last_raw_pitch = float(np.arctan2(R_curr[2, 1], R_curr[2, 2]))
        current_flip_apex_z = float(d.qpos[2])
        current_flip_max_rot = 0.0
        current_flip_touchdown_deg = 0.0
        current_flip_air_collisions = 0

        v_target = 0.0
        yaw_rate_target = 0.0
        qk_target = 0.0
        target_yaw = 0.0
        v_filtered = 0.0
        v_int = 0.0

        for k in milestones:
            milestones[k] = False

    def key_callback(keycode):
        nonlocal qk_target, v_target, yaw_rate_target

        if keycode in (ord('W'), ord('w'), 265):
            v_target = np.clip(v_target + 0.25, -1.0, 2.0)
            print_status("Speed Up")
        elif keycode in (ord('S'), ord('s'), 264):
            v_target = np.clip(v_target - 0.25, -1.0, 2.0)
            print_status("Slow/Reverse")
        elif keycode in (ord('A'), ord('a'), 263):
            yaw_rate_target = np.clip(yaw_rate_target + yaw_step, -yaw_max, yaw_max)
            print_status("Steer Left")
        elif keycode in (ord('D'), ord('d'), 262):
            yaw_rate_target = np.clip(yaw_rate_target - yaw_step, -yaw_max, yaw_max)
            print_status("Steer Right")
        elif keycode in (ord('X'), ord('x')):
            v_target = 0.0
            yaw_rate_target = 0.0
            print_status("BRAKE / STOP")
        elif keycode in (ord('C'), ord('c')):
            qk_target = np.clip(qk_target - qk_step, qk_min, 0.20)
            print_status("Sit (body on the ground)" if 0.10 * qk_target < getattr(wbc_solver, "dh_min", -1) else "Crouch Lower")
        elif keycode in (ord('T'), ord('t')):
            qk_target = np.clip(qk_target + 0.10, qk_min, 0.20)
            print_status("Stand Taller")
        elif keycode in (ord('N'), ord('n')):
            qk_target = 0.0
            print_status("Nominal Height")
        elif keycode in (ord('B'), ord('b')):
            if orig_robot:
                trigger_flip()
            else:
                print("[STUNT] Backflips are only tuned for the original robot (CAD/).")

    log_fp = None
    if args.log_pitch:
        log_fp = open(args.log_pitch, "w")
        log_fp.write("sim_time,pitch_deg,unwrapped_pitch_deg,omega_pitch,up_z,pos_y,pos_z,state\n")

    global_unwrapped_pitch = 0.0
    global_last_pitch = None
    wbc_was_active = False
    safe_fall_shown = 0

    try:
        if args.headless:
            class NullViewer:
                def is_running(self):
                    return True
                def sync(self):
                    pass
            viewer_ctx = contextlib.nullcontext(NullViewer())
        else:
            viewer_ctx = mujoco.viewer.launch_passive(m, d, key_callback=key_callback)

        with viewer_ctx as viewer:
            if not args.headless:
                base_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "body_base")
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                viewer.cam.trackbodyid = base_body_id
                viewer.cam.distance = 1.35
                viewer.cam.elevation = -18.0
                viewer.cam.azimuth = 135.0

            while viewer.is_running():
                step_start = time.time()

                # Handle Reset (Backspace key in viewer)
                if not args.headless and d.time < last_sim_time:
                    spawn()
                    step_count = 0
                    v_target = 0.0
                    yaw_rate_target = 0.0
                    v_filtered = 0.0
                    yaw_filtered = 0.0
                    v_int = 0.0
                    qk_target = 0.0
                    qk_current = 0.0
                    is_backflipping = False
                    bf_timer = 0.0
                    bf_state = "idle"
                    settle_timer = 0.0
                    if wbc_solver is not None:
                        wbc_solver.reset()
                    safe_fall_shown = 0
                    print("\n[Reset] Robot reset to platform surface (z = 0.08m, zero velocity).")

                last_sim_time = d.time

                # 3D Invariant Coordinate Tracking using Base Rotation Matrix
                R = d.xmat[1].reshape(3, 3)
                fwd = R[:, 1]

                theta = float(np.arctan2(R[2, 1], R[2, 2]))
                omega_pitch = float(d.qvel[3])
                omega_yaw = float(d.qvel[5])
                v_fwd = float(np.dot(d.qvel[0:3], fwd))
                yaw_heading = float(np.arctan2(R[1, 0], R[0, 0]))

                # Global pitch tracking
                raw_pitch = float(np.arctan2(R[2, 1], R[1, 1]))
                if global_last_pitch is None:
                    global_last_pitch = raw_pitch
                dp = raw_pitch - global_last_pitch
                if dp < -np.pi: dp += 2 * np.pi
                elif dp > np.pi: dp -= 2 * np.pi
                global_unwrapped_pitch += dp
                global_last_pitch = raw_pitch

                pitch_deg = float(np.degrees(theta))
                pitch_rot_deg = float(np.degrees(global_unwrapped_pitch))

                # Autonomous re-settling and flip chaining between stunts
                if not is_backflipping:
                    settle_timer += dt_sim
                    ey = d.qpos[1] - y_home
                    ex = d.qpos[0] - x_home

                    if is_multi_flip:
                        # Multi-flip active re-centering on platform
                        if ey > 0.015:
                            target_yaw = float(np.clip(- 2.5 * ex, -0.32, 0.32))
                            v_target = float(np.clip(-1.5 * ey, -0.16, 0.16))
                        elif ey < -0.015:
                            target_yaw = float(np.clip(2.5 * ex, -0.32, 0.32))
                            v_target = float(np.clip(-1.5 * ey, -0.16, 0.16))
                        else:
                            target_yaw = 0.0
                            if abs(yaw_heading) > 0.035:
                                v_target = 0.0
                            else:
                                v_target = float(np.clip(-1.5 * ey, -0.12, 0.12)) if abs(ey) > 0.008 else 0.0

                        is_ready = (settle_timer >= 2.20 and
                                    -0.50 <= d.qpos[0] <= 0.50 and
                                    abs(ey) < 0.025 and
                                    abs(yaw_heading) < 0.035 and
                                    abs(v_fwd) < 0.008 and
                                    abs(theta) < 0.010 and
                                    abs(omega_pitch) < 0.04 and
                                    abs(d.qvel[9]) < 0.08 and
                                    abs(d.qvel[13]) < 0.08)

                        if is_ready and completed_flips < target_flips:
                            trigger_flip()

                # Pitch and collision tracking during stunt
                if is_backflipping:
                    raw_pitch_curr = float(np.arctan2(R[2, 1], R[2, 2]))
                    dp_f = raw_pitch_curr - bf_last_raw_pitch
                    if dp_f < -np.pi: dp_f += 2 * np.pi
                    elif dp_f > np.pi: dp_f -= 2 * np.pi
                    bf_unwrapped_pitch += dp_f
                    bf_last_raw_pitch = raw_pitch_curr
                    deg_flip = float(np.degrees(bf_unwrapped_pitch))

                    if deg_flip > current_flip_max_rot:
                        current_flip_max_rot = deg_flip
                    if d.qpos[2] > current_flip_apex_z:
                        current_flip_apex_z = float(d.qpos[2])

                    if bf_state in ("pop", "tuck", "untuck") and bf_timer > 0.03:
                        for c in range(d.ncon):
                            b1 = m.geom_bodyid[d.contact[c].geom1]
                            b2 = m.geom_bodyid[d.contact[c].geom2]
                            if b1 not in (wheel_R_id, wheel_L_id) and b2 not in (wheel_R_id, wheel_L_id):
                                current_flip_air_collisions += 1

                    # Milestone logging
                    if not milestones["crouch"] and bf_state == "crouch":
                        milestones["crouch"] = True
                        print(f"  [Sim t={d.time:6.3f}s] CROUCH: Squatting to load suspension stroke | Pitch: {pitch_deg:+.1f}° | Up-Z: {R[2, 2]:.4f}")
                    elif not milestones["whip"] and bf_state == "whip":
                        milestones["whip"] = True
                        print(f"  [Sim t={d.time:6.3f}s] WHIP: Explosive torso back-whip torque | Pitch: {pitch_deg:+.1f}° | Up-Z: {R[2, 2]:.4f}")
                    elif not milestones["pop"] and bf_state == "pop":
                        milestones["pop"] = True
                        print(f"  [Sim t={d.time:6.3f}s] POP: Knee extension liftoff into air | Pitch: {pitch_deg:+.1f}° | Base Z: {d.qpos[2]:.3f}m")
                    elif not milestones["quarter"] and deg_flip >= 90.0:
                        milestones["quarter"] = True
                        print(f"  [Sim t={d.time:6.3f}s] FLIP 90°: Torso horizontal in mid-air | Continuous Pitch: {deg_flip:.1f}° | Base Z: {d.qpos[2]:.3f}m")
                    elif not milestones["half"] and deg_flip >= 180.0:
                        milestones["half"] = True
                        print(f"  [Sim t={d.time:6.3f}s] FLIP 180°: INVERTED APEX (Upside Down) | Continuous Pitch: {deg_flip:.1f}° | Base Z: {d.qpos[2]:.3f}m | Up-Z: {R[2, 2]:.4f}")
                    elif not milestones["three_quarter"] and deg_flip >= 270.0:
                        milestones["three_quarter"] = True
                        print(f"  [Sim t={d.time:6.3f}s] FLIP 270°: Untucking legs for touchdown | Continuous Pitch: {deg_flip:.1f}° | Base Z: {d.qpos[2]:.3f}m")
                    elif not milestones["landed"] and bf_state == "landed":
                        milestones["landed"] = True
                        print(f"  [Sim t={d.time:6.3f}s] TOUCHDOWN: Wheels contacted ground | Pitch: {pitch_deg:+.1f}° (Flip: {deg_flip:.1f}°) | Base Y: {d.qpos[1]:.3f}m")

                if milestones["landed"] and not milestones["settled"] and not is_backflipping:
                    milestones["settled"] = True
                    print(f"  [Sim t={d.time:6.3f}s] SETTLED: Balanced & upright in original state | Pitch: {pitch_deg:+.1f}° | Up-Z: {R[2, 2]:.4f} | Base Z: {d.qpos[2]:.3f}m | Base Y: {d.qpos[1]:.3f}m")

                if log_fp is not None:
                    log_fp.write(f"{d.time:.4f},{pitch_deg:.2f},{pitch_rot_deg:.2f},{omega_pitch:.4f},{R[2, 2]:.4f},{d.qpos[1]:.4f},{d.qpos[2]:.4f},{bf_state}\n")

                # Filter target velocity
                alpha = min(1.0, 4.0 * dt_sim)
                v_filtered += alpha * (v_target - v_filtered)
                yaw_filtered += alpha * (yaw_rate_target - yaw_filtered)

                # Airborne Anti-Windup
                is_airborne = (d.qpos[2] < 0.11 and d.qpos[2] > 0.035 and d.qvel[2] < -0.2)
                if not is_airborne and not is_backflipping:
                    v_err = v_fwd - v_filtered
                    v_int += v_err * dt_sim
                    v_int = float(np.clip(v_int, -0.4, 0.4))
                else:
                    v_int = 0.0

                # Ascento-Inspired LQR Balancing Motion Task (Klemm et al. 2020 Eq 36-37)
                v_err = v_fwd - v_filtered
                v_int += v_err * dt_sim
                v_int = float(np.clip(v_int, -0.30, 0.30))

                th_ref = kp_v * v_err + ki_v * v_int
                th_ref = float(np.clip(th_ref, -max_tilt, max_tilt))

                tau_wheel = - (kp_th * (theta - th_ref) + kd_th * omega_pitch)
                tau_wheel = float(np.clip(tau_wheel, -14.0, 14.0))

                tau_roll = 0.0
                psi_ref = 0.0

                if args.controller == "nmpc" and mpc_lib is not None and not is_backflipping:
                    # 3D Invariant State Mapping for acados NMPC (Local relative receding horizon)
                    p_curr = float(d.qpos[1])
                    v_curr = float(v_fwd)
                    theta_mpc = - float(theta)
                    omega_mpc = - float(omega_pitch)

                    qk_avg = 0.5 * ((d.qpos[9] - qk_R_nom) - (d.qpos[13] - qk_L_nom))
                    l_curr = 0.148 - 0.08 * qk_avg
                    dl_curr = -0.08 * 0.5 * (d.qvel[8] - d.qvel[12])

                    roll_curr = float(np.arctan2(R[0, 2], R[2, 2]))
                    roll_rate_curr = float(d.qvel[4])

                    l_target_ref = 0.148 - 0.08 * qk_current

                    mpc_lib.mpc_step(
                        mpc_handle,
                        p_curr, v_curr,
                        theta_mpc, omega_mpc,
                        l_curr, dl_curr,
                        roll_curr, roll_rate_curr,
                        float(v_filtered),
                        l_target_ref,
                        float(yaw_filtered),
                        p_curr,  # Local relative horizon prevents global position runaway
                        0,
                        mpc_out_ctrl,
                        ctypes.byref(mpc_solve_time)
                    )
                    tau_roll = mpc_out_ctrl[2]
                    psi_ref = mpc_out_ctrl[3]

                # Heading regulation
                if is_multi_flip:
                    yaw_heading_err = yaw_heading - target_yaw
                    if abs(yaw_heading_err) > np.pi:
                        yaw_heading_err -= 2 * np.pi * np.sign(yaw_heading_err)
                    tau_diff = - (kp_yaw * yaw_heading_err + kd_yaw * omega_yaw)
                    tau_diff = float(np.clip(tau_diff, -2.5, 2.5))
                elif args.wavy and abs(yaw_rate_target) < 0.05:
                    # Autonomous lane-keeping on center runway (x_home = 0.0705)
                    x_err = float(d.qpos[0] - x_home)
                    yaw_target = float(np.clip(+ 1.2 * x_err, -0.20, 0.20))
                    yaw_err = yaw_heading - yaw_target
                    tau_diff = float(np.clip(- (2.8 * yaw_err + 0.40 * omega_yaw), -2.5, 2.5))
                else:
                    yaw_err = omega_yaw - yaw_filtered
                    tau_diff = - (kp_yaw * yaw_err + kd_yaw * omega_yaw)
                    tau_diff = float(np.clip(tau_diff, -3.5, 3.5))

                # Actuator Command Generation
                wbc_ran_this_tick = False
                if is_backflipping:
                    bf_timer += dt_sim
                    if bf_state == "crouch":
                        frac = min(1.0, max(0.0, bf_timer - dt_sim) / 0.20)
                        qk_c = -0.60 * frac
                        d_qh_R_c = -0.515 * qk_c
                        d_qh_L_c = +0.705 * qk_c

                        d.ctrl[0] = -85.0 * d.qpos[7] - 5.5 * d.qvel[6]
                        d.ctrl[1] = -90.0 * (d.qpos[8] - (qh_R_nom + d_qh_R_c)) - 6.0 * d.qvel[7]
                        d.ctrl[2] = -90.0 * (d.qpos[9] - (qk_R_nom - qk_c)) - 6.0 * d.qvel[8]
                        d.ctrl[3] = 0.0
                        d.ctrl[4] = -85.0 * d.qpos[11] - 5.5 * d.qvel[10]
                        d.ctrl[5] = -90.0 * (d.qpos[12] - (qh_L_nom + d_qh_L_c)) - 6.0 * d.qvel[11]
                        d.ctrl[6] = -90.0 * (d.qpos[13] - (qk_L_nom + qk_c)) - 6.0 * d.qvel[12]
                        d.ctrl[7] = 0.0

                        if bf_timer >= 0.20:
                            bf_state = "whip"
                            bf_step_count = 0

                    elif bf_state == "whip":
                        bf_step_count += 1
                        d.ctrl[0] = -90.0 * d.qpos[7] - 6.0 * d.qvel[6]
                        d.ctrl[1] = -9.80
                        d.ctrl[2] = -6.0
                        d.ctrl[3] = 0.4
                        d.ctrl[4] = -90.0 * d.qpos[11] - 6.0 * d.qvel[10]
                        d.ctrl[5] = +9.80
                        d.ctrl[6] = +6.0
                        d.ctrl[7] = 0.4

                        if bf_step_count >= 8:
                            bf_state = "pop"
                            bf_step_count = 0

                    elif bf_state == "pop":
                        bf_step_count += 1
                        tau_roll_comp = -2.2 * d.qvel[4]
                        d.ctrl[0] = float(np.clip(-90.0 * d.qpos[7] - 6.0 * d.qvel[6] + tau_roll_comp, -15.0, 15.0))
                        d.ctrl[1] = -15.0
                        d.ctrl[2] = -4.0
                        d.ctrl[3] = 0.4
                        d.ctrl[4] = float(np.clip(-90.0 * d.qpos[11] - 6.0 * d.qvel[10] - tau_roll_comp, -15.0, 15.0))
                        d.ctrl[5] = +15.0
                        d.ctrl[6] = +4.0
                        d.ctrl[7] = 0.4

                        if bf_step_count >= 18:
                            bf_state = "tuck"
                            print(f"  [FLIP #{completed_flips+1} TAKEOFF] z={d.qpos[2]:.3f}, vz={d.qvel[2]:.3f}, pitch_rate={d.qvel[3]:.2f}, deg={deg_flip:.1f}")

                    elif bf_state in ("tuck", "untuck"):
                        if bf_state == "tuck":
                            if deg_flip > 218.0:
                                bf_state = "untuck"
                            else:
                                qk_t = -0.55
                                d_qh_R_t = -0.515 * qk_t
                                d_qh_L_t = +0.705 * qk_t
                                d.ctrl[0] = -85.0 * d.qpos[7] - 5.5 * d.qvel[6]
                                d.ctrl[1] = -35.0 * (d.qpos[8] - (qh_R_nom + d_qh_R_t)) - 2.0 * d.qvel[7]
                                d.ctrl[2] = -35.0 * (d.qpos[9] - (qk_R_nom - qk_t)) - 2.0 * d.qvel[8]
                                d.ctrl[3] = 0.0
                                d.ctrl[4] = -85.0 * d.qpos[11] - 5.5 * d.qvel[10]
                                d.ctrl[5] = -35.0 * (d.qpos[12] - (qh_L_nom + d_qh_L_t)) - 2.0 * d.qvel[11]
                                d.ctrl[6] = -35.0 * (d.qpos[13] - (qk_L_nom + qk_t)) - 2.0 * d.qvel[12]
                                d.ctrl[7] = 0.0

                        if bf_state == "untuck":
                            d.ctrl[0] = -85.0 * d.qpos[7] - 5.5 * d.qvel[6]
                            d.ctrl[1] = -80.0 * (d.qpos[8] - qh_R_nom) - 6.0 * d.qvel[7]
                            d.ctrl[2] = -80.0 * (d.qpos[9] - qk_R_nom) - 6.0 * d.qvel[8]
                            d.ctrl[3] = 0.0
                            d.ctrl[4] = -85.0 * d.qpos[11] - 5.5 * d.qvel[10]
                            d.ctrl[5] = -80.0 * (d.qpos[12] - qh_L_nom) - 6.0 * d.qvel[11]
                            d.ctrl[6] = -80.0 * (d.qpos[13] - qk_L_nom) - 6.0 * d.qvel[12]
                            d.ctrl[7] = 0.0

                            wheel_touch = False
                            for c in range(d.ncon):
                                b1 = m.geom_bodyid[d.contact[c].geom1]
                                b2 = m.geom_bodyid[d.contact[c].geom2]
                                if (b1 in (5, 9) or b2 in (5, 9)):
                                    wheel_touch = True
                                    break

                            if wheel_touch and deg_flip > 260.0:
                                bf_state = "landed"
                                bf_t_land = 0.0
                                current_flip_touchdown_deg = deg_flip

                        if bf_timer > 1.2:
                            print(f"DEBUG: flip timed out in state {bf_state}, deg={deg_flip:.1f}")
                            break

                    elif bf_state == "landed":
                        bf_t_land += dt_sim
                        theta_l = float(np.arctan2(R[2, 1], R[2, 2]))
                        omega_pitch_l = float(d.qvel[3])

                        alpha_imp = min(1.0, bf_t_land / 0.25)
                        kp_leg_land = 50.0 + 35.0 * alpha_imp
                        kd_knee_l = 18.0 - 12.0 * alpha_imp

                        d.ctrl[0] = -85.0 * d.qpos[7] - 5.5 * d.qvel[6]
                        d.ctrl[1] = -kp_leg_land * (d.qpos[8] - qh_R_nom) - 6.0 * d.qvel[7]
                        d.ctrl[2] = -kp_leg_land * (d.qpos[9] - qk_R_nom) - kd_knee_l * d.qvel[8]
                        d.ctrl[4] = -85.0 * d.qpos[11] - 5.5 * d.qvel[10]
                        d.ctrl[5] = -kp_leg_land * (d.qpos[12] - qh_L_nom) - 6.0 * d.qvel[11]
                        d.ctrl[6] = -kp_leg_land * (d.qpos[13] - qk_L_nom) - kd_knee_l * d.qvel[12]

                        tau_bal = float(np.clip(- (60.0 * theta_l + 7.5 * omega_pitch_l), -14.0, 14.0))
                        d.ctrl[3] = 0.5 * tau_bal
                        d.ctrl[7] = 0.5 * tau_bal

                        if bf_t_land >= 0.80:
                            completed_flips += 1
                            record = {
                                "flip": completed_flips,
                                "apex_z": current_flip_apex_z,
                                "touchdown_deg": current_flip_touchdown_deg,
                                "max_rot": current_flip_max_rot,
                                "air_collisions": current_flip_air_collisions,
                                "land_x": d.qpos[0],
                                "land_y": d.qpos[1],
                                "up_z": R[2, 2],
                                "pitch_err": np.degrees(theta_l),
                                "heading": np.degrees(yaw_heading),
                            }
                            flip_records.append(record)
                            print(f"  [RESULT #{completed_flips:02d}] Apex Z: {current_flip_apex_z:.3f}m | Touchdown: {current_flip_touchdown_deg:.1f}° | Pos X: {d.qpos[0]:+.3f}m | Pos Y: {d.qpos[1]:+.3f}m | Heading: {np.degrees(yaw_heading):+.1f}° | Up-Z: {R[2, 2]:.4f} | Status: SUCCESS")

                            is_backflipping = False
                            bf_state = "idle"
                            settle_timer = 0.0
                            v_target = 0.0
                            yaw_rate_target = 0.0
                            v_filtered = 0.0
                            yaw_filtered = 0.0
                            v_int = 0.0

                            if is_multi_flip and completed_flips >= target_flips:
                                print("\n" + "=" * 80)
                                print(f"              {completed_flips} / {target_flips} BACKFLIPS COMPLETED TELEMETRY TABLE")
                                print("=" * 80)
                                print(f" {'Flip':<5} | {'Apex Z (m)':<10} | {'Touchdown':<10} | {'Air Touch':<10} | {'End X (m)':<10} | {'End Y (m)':<10} | {'End Up-Z':<10} | {'Status'}")
                                print("-" * 80)
                                for r in flip_records:
                                    print(f" #{r['flip']:02d}   | {r['apex_z']:<10.3f} | {r['touchdown_deg']:<9.1f}° | {r['air_collisions']:<10d} | {r['land_x']:<+10.3f} | {r['land_y']:<+10.3f} | {r['up_z']:<10.4f} | SUCCESS")
                                print("=" * 80)
                                print(f"\n>>> ALL {target_flips}/{target_flips} CONSECUTIVE BACKFLIPS COMPLETED & VERIFIED ON PLATFORM! <<<\n")
                                if args.headless:
                                    break
                                else:
                                    print("[STUNT] Auto-flip sequence complete! Robot settled on platform. Press <B> to flip again or driving keys to drive.\n")
                            else:
                                if is_multi_flip:
                                    print("[STUNT] Backflip landed successfully! Settling for next flip...\n")
                                else:
                                    print("[STUNT] Backflip landed successfully! Balanced & ready for driving or next flip (<B>).\n")
                else:
                    if is_multi_flip:
                        # Autonomous multi-flip platform settling
                        d.ctrl[0] = -kp_roll * d.qpos[7] - kd_roll * d.qvel[6]
                        d.ctrl[1] = -kp_leg * (d.qpos[8] - qh_R_nom) - kd_leg * d.qvel[7]
                        d.ctrl[2] = -kp_leg * (d.qpos[9] - qk_R_nom) - kd_knee_R * d.qvel[8]
                        d.ctrl[3] = 0.5 * tau_wheel - tau_diff
                        d.ctrl[4] = -kp_roll * d.qpos[11] - kd_roll * d.qvel[10]
                        d.ctrl[5] = -kp_leg * (d.qpos[12] - qh_L_nom) - kd_leg * d.qvel[11]
                        d.ctrl[6] = -kp_leg * (d.qpos[13] - qk_L_nom) - kd_knee_L * d.qvel[12]
                        d.ctrl[7] = 0.5 * tau_wheel + tau_diff
                    else:
                        if wbc_solver is not None:
                            # Ascento Hierarchical Whole-Body QP Control
                            yaw_cmd = float(yaw_filtered)
                            if args.wavy and abs(yaw_rate_target) < 0.05:
                                # Lane keeping on the runway centre line: x_err > 0 needs a CCW
                                # (positive) heading; close the loop on heading, not just rate.
                                x_err = float(d.qpos[0] - x_home)
                                heading_cmd = float(np.clip(1.2 * x_err, -0.20, 0.20))
                                yaw_cmd = float(np.clip(2.5 * (heading_cmd - yaw_heading), -0.6, 0.6))

                            roll_terrain_offset = 0.0
                            if args.wavy:
                                roll_now = float(np.arctan2(-R[2, 0], R[2, 2]))   # about the forward axis
                                roll_rate_now = float(d.qvel[4])
                                roll_terrain_offset = float(np.clip(- (0.60 * roll_now + 0.05 * roll_rate_now), -0.25, 0.25))

                            # C/T/N posture keys: qk_target in [-0.55, +0.20] -> base height offset.
                            # (Negative = crouch; the WBC clamps to its reachable range.)
                            alpha_q = min(1.0, 3.0 * dt_sim)
                            qk_current += alpha_q * (qk_target - qk_current)
                            delta_h_cmd = 0.10 * qk_current

                            # WBC keeps internal state (odometry, filters); restart it whenever it
                            # takes over from another mode (start, backflip, multi-flip settling).
                            if not wbc_was_active:
                                wbc_solver.reset()
                                safe_fall_shown = 0
                            wbc_ran_this_tick = True

                            tau_wbc, wbc_info = wbc_solver.step(
                                d,
                                v_target=float(v_filtered),
                                yaw_target=yaw_cmd,
                                delta_h=delta_h_cmd,
                                roll_target_offset=roll_terrain_offset,
                                dt=dt_sim
                            )
                            d.ctrl[:] = tau_wbc
                            sf = wbc_info.get("safe_fall", 0)
                            if sf != safe_fall_shown:
                                if sf == 5:
                                    print(f"\n[SIT] t={d.time:.2f}s sitting on the ground (press T or N to stand up).")
                                elif safe_fall_shown == 5 and not sf:
                                    print(f"\n[SIT] t={d.time:.2f}s standing up, balancing.")
                                elif sf:
                                    cause = {1: "lean", 2: "roll", 3: "tilt", 4: "QP failure"}.get(sf, sf)
                                    print(f"\n[SAFE FALL] t={d.time:.2f}s fall detected ({cause}): wheels braking, "
                                          f"legs tucked. Press Backspace to reset.")
                                else:
                                    print(f"\n[SAFE FALL] t={d.time:.2f}s upright again, balancing resumed.")
                                safe_fall_shown = sf
                        else:
                            # Manual keyboard driving mode: active hip banking, differential knees, and hip pitch coordination
                            alpha_q = min(1.0, 3.0 * dt_sim)
                            qk_current += alpha_q * (qk_target - qk_current)

                            # Ascento-inspired ZMP-regulated dynamic curve leaning: psi_ref = arctan(v * omega_yaw / g)
                            a_lat = v_fwd * omega_yaw
                            psi_zmp = float(np.arctan2(a_lat, 9.81))
                            bank_cmd = k_roll_bank * yaw_filtered + 0.50 * psi_zmp
                            knee_diff_cmd = k_knee_diff * yaw_filtered + 0.25 * psi_zmp
                            roll = float(np.arctan2(R[0, 2], R[2, 2]))
                            roll_rate = float(d.qvel[4])

                            if args.wavy:
                                # Active suspension roll leveling and knee compliance for out-of-phase wave lanes
                                k_roll_hip = 0.75
                                kd_roll_hip = 0.06
                                k_knee_diff_terrain = 0.85
                                kd_knee_diff_terrain = 0.06

                                roll_terrain_cmd = float(np.clip(k_roll_hip * roll + kd_roll_hip * roll_rate, -0.30, 0.30))
                                q_roll_R_cmd = - roll_terrain_cmd - bank_cmd
                                q_roll_L_cmd = + roll_terrain_cmd + bank_cmd

                                delta_qk_terrain = float(np.clip(k_knee_diff_terrain * roll + kd_knee_diff_terrain * roll_rate, -0.35, 0.35))
                                if delta_qk_terrain > 0:
                                    qk_R_cmd = qk_R_nom - qk_current - knee_diff_cmd + delta_qk_terrain
                                    qk_L_cmd = qk_L_nom + qk_current - knee_diff_cmd
                                else:
                                    qk_R_cmd = qk_R_nom - qk_current - knee_diff_cmd
                                    qk_L_cmd = qk_L_nom + qk_current - knee_diff_cmd + delta_qk_terrain
                            else:
                                q_roll_R_cmd = - bank_cmd
                                q_roll_L_cmd = + bank_cmd
                                qk_R_cmd = qk_R_nom - qk_current - knee_diff_cmd
                                qk_L_cmd = qk_L_nom + qk_current - knee_diff_cmd

                            # Ascento Active Hip Pitch Coordination: dynamically tilt torso into acceleration/braking
                            k_hip_pitch = 0.40
                            d_qh_pitch = k_hip_pitch * th_ref
                            d_qh_R = -0.515 * (qk_R_cmd - qk_R_nom)
                            d_qh_L = -0.705 * (qk_L_cmd - qk_L_nom)
                            qh_R_cmd = qh_R_nom + d_qh_pitch + d_qh_R
                            qh_L_cmd = qh_L_nom - d_qh_pitch + d_qh_L
                            kd_knee_R_man = 9.0 if abs(d.qvel[8]) > 1.2 else kd_knee_R
                            kd_knee_L_man = 9.0 if abs(d.qvel[12]) > 1.2 else kd_knee_L

                            d.ctrl[0] = -0.5 * tau_roll - kp_roll * (d.qpos[7] - q_roll_R_cmd) - kd_roll * d.qvel[6]
                            d.ctrl[1] = -kp_leg * (d.qpos[8] - qh_R_cmd) - kd_leg * d.qvel[7]
                            d.ctrl[2] = -kp_leg * (d.qpos[9] - qk_R_cmd) - kd_knee_R_man * d.qvel[8]
                            d.ctrl[3] = 0.5 * tau_wheel - tau_diff
                            d.ctrl[4] = +0.5 * tau_roll - kp_roll * (d.qpos[11] - q_roll_L_cmd) - kd_roll * d.qvel[10]
                            d.ctrl[5] = -kp_leg * (d.qpos[12] - qh_L_cmd) - kd_leg * d.qvel[11]
                            d.ctrl[6] = -kp_leg * (d.qpos[13] - qk_L_cmd) - kd_knee_L_man * d.qvel[12]
                            d.ctrl[7] = 0.5 * tau_wheel + tau_diff

                wbc_was_active = wbc_ran_this_tick
                mujoco.mj_step(m, d)
                step_count += 1

                if args.wavy and step_count % 250 == 0:
                    r_wheel_z = d.xpos[wheel_R_id][2]
                    l_wheel_z = d.xpos[wheel_L_id][2]
                    h_diff = l_wheel_z - r_wheel_z
                    roll_deg = np.degrees(np.arctan2(-R[2, 0], R[2, 2]))
                    ctrl_stat = f" | WBC={wbc_info['solve_time_us']:.0f}µs" if wbc_solver is not None else (f" | MPC={mpc_solve_time.value:.0f}µs" if mpc_lib is not None else "")
                    print(f"  [WAVY RUNWAY] t={d.time:5.2f}s | Pos X={d.qpos[0]:+.3f}m | Pos Y={d.qpos[1]:+.2f}m | R-wheel Z={r_wheel_z*100:4.1f}cm | L-wheel Z={l_wheel_z*100:4.1f}cm | Diff={h_diff*100:+4.1f}cm | Roll={roll_deg:+5.1f}° | Up-Z={R[2, 2]:.4f}{ctrl_stat}")

                if step_count % 8 == 0:
                    viewer.sync()

                if not args.headless:
                    elapsed = time.time() - step_start
                    sleep_time = dt_sim - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)

                if args.duration is not None and d.time >= args.duration:
                    print(f"\n[Done] Reached target simulation duration ({args.duration:.2f}s).")
                    break

    except KeyboardInterrupt:
        print("\n[Exit] User interrupted simulation.")
    finally:
        if mpc_lib is not None and mpc_handle is not None:
            mpc_lib.mpc_destroy(mpc_handle)
        if log_fp is not None:
            log_fp.close()
            print(f"[Log] Successfully wrote pitch telemetry log to: {args.log_pitch}")
        print("[Exit] Simulation terminated cleanly.")


if __name__ == "__main__":
    main()
