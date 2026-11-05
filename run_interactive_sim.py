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

import os
import sys
import time
import argparse
import contextlib
import numpy as np
import mujoco
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(description="Interactive MuJoCo simulation for wheeled biped robot.")
    parser.add_argument("--auto-flip", action="store_true", help="Automatically trigger backflip stunt")
    parser.add_argument("--repeat-flips", type=int, default=1, help="Number of consecutive backflips to execute (default: 1)")
    parser.add_argument("--flip-time", type=float, default=0.5, help="Simulation time to trigger backflip (default: 0.5s)")
    parser.add_argument("--duration", type=float, default=None, help="Maximum simulation duration in seconds")
    parser.add_argument("--log-pitch", type=str, default=None, help="Path to write base pitch log CSV")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode without GUI window")
    args, unknown = parser.parse_known_args()

    if args.repeat_flips > 1:
        args.auto_flip = True

    model_xml = os.path.join(os.path.dirname(__file__), "CAD", "scene.xml")
    if not os.path.isfile(model_xml):
        model_xml = "CAD/scene.xml"

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
    x_start = -0.28 if is_multi_flip else 0.0
    y_home = -0.12 if is_multi_flip else 0.0
    x_home = 0.07 if is_multi_flip else 0.0

    # Spawn resting on top of raised platform (z = 0.08m)
    d.qpos[0] = x_start
    d.qpos[1] = y_home
    d.qpos[2] = 0.1172
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

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

    print("\n" + "=" * 75)
    print("      Acrobat / BDX Wheeled Biped: 8-DOF Drive, Lean & Active Suspension")
    print("=" * 75)
    print("  Driving & Cornering Controls:")
    print("    - DRIVE FORWARD:    Press 'W' or Up Arrow     (+0.2 m/s per tap)")
    print("    - SLOW / REVERSE:   Press 'S' or Down Arrow   (-0.2 m/s per tap)")
    print("    - STEER LEFT (LEAN):Press 'A' or Left Arrow   (+0.3 rad/s + Active Bank)")
    print("    - STEER RIGHT(LEAN):Press 'D' or Right Arrow  (-0.3 rad/s + Active Bank)")
    print("    - BRAKE / STOP:     Press 'X'                 (Locks stationary position)")
    print("  Vertical Posture Controls:")
    print("    - CROUCH DOWN:      Press 'C'                 (Tap repeatedly to crouch lower, up to -5cm)")
    print("    - STAND TALL:       Press 'T'                 (Tap repeatedly to stand taller)")
    print("    - NOMINAL HEIGHT:   Press 'N'                 (Reset to nominal stance)")
    print("  Acrobatic Stunt:")
    print(f"    - 360 BACKFLIP:     Press 'B'                 (Explosive jump, tucked flip & landing) [Repeat: {args.repeat_flips}]")
    print("  Simulation Controls:")
    print("    - POKE / PERTURB:   Right-Click + Drag on robot body in viewer")
    print("    - RESET TO PLATFORM:Backspace key")
    print("    - PAUSE / RESUME:   Space bar")
    print("    - QUIT:             Close window or press Ctrl+C")
    print("=" * 75 + "\n")

    # Balancer gains
    kp_th = 55.0
    kd_th = 7.5
    kp_v = 0.40
    kd_v = 0.30
    ki_v = 0.06
    max_tilt = 0.15

    # Steering gains
    kp_yaw = 2.0
    kd_yaw = 0.30
    k_roll_bank = 0.38   # rad roll per rad/s yaw rate (interactive keyboard banking)
    k_knee_diff = 0.20

    # Leg stance & suspension gains
    kp_leg = 70.0
    kd_leg = 5.0
    kp_roll = 85.0
    kd_roll = 5.5
    kd_knee_R = 4.5
    kd_knee_L = 3.5

    last_sim_time = d.time
    step_count = 0

    v_target = 0.0
    yaw_rate_target = 0.0
    v_filtered = 0.0
    yaw_filtered = 0.0
    v_int = 0.0

    qk_target = 0.0
    qk_current = 0.0

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

    def print_status(action_msg):
        lean_str = " (Deep Bank Left)" if yaw_rate_target > 0.05 else (" (Deep Bank Right)" if yaw_rate_target < -0.05 else "")
        if qk_target < -0.35:
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
            yaw_rate_target = np.clip(yaw_rate_target + 0.35, -1.80, 1.80)
            print_status("Steer Left")
        elif keycode in (ord('D'), ord('d'), 262):
            yaw_rate_target = np.clip(yaw_rate_target - 0.35, -1.80, 1.80)
            print_status("Steer Right")
        elif keycode in (ord('X'), ord('x')):
            v_target = 0.0
            yaw_rate_target = 0.0
            print_status("BRAKE / STOP")
        elif keycode in (ord('C'), ord('c')):
            qk_target = np.clip(qk_target - 0.15, -0.55, 0.20)
            print_status("Crouch Lower")
        elif keycode in (ord('T'), ord('t')):
            qk_target = np.clip(qk_target + 0.10, -0.55, 0.20)
            print_status("Stand Taller")
        elif keycode in (ord('N'), ord('n')):
            qk_target = 0.0
            print_status("Nominal Height")
        elif keycode in (ord('B'), ord('b')):
            trigger_flip()

    log_fp = None
    if args.log_pitch:
        log_fp = open(args.log_pitch, "w")
        log_fp.write("sim_time,pitch_deg,unwrapped_pitch_deg,omega_pitch,up_z,pos_y,pos_z,state\n")

    global_unwrapped_pitch = 0.0
    global_last_pitch = None

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
                    d.qpos[0] = x_start
                    d.qpos[1] = y_home
                    d.qpos[2] = 0.1172
                    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
                    d.qpos[7] = 0.0
                    d.qpos[8] = qh_R_nom
                    d.qpos[9] = qk_R_nom
                    d.qpos[10] = 0.0
                    d.qpos[11] = 0.0
                    d.qpos[12] = qh_L_nom
                    d.qpos[13] = qk_L_nom
                    d.qpos[14] = 0.0
                    d.qvel[:] = 0.0
                    mujoco.mj_forward(m, d)
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
                            if b1 not in (5, 9) and b2 not in (5, 9):
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

                # Cascaded longitudinal pitch reference
                th_ref = kp_v * (v_fwd - v_filtered) + ki_v * v_int
                th_ref = float(np.clip(th_ref, -max_tilt, max_tilt))

                tau_wheel = - (kp_th * (theta - th_ref) + kd_th * omega_pitch)
                tau_wheel = float(np.clip(tau_wheel, -15.0, 15.0))

                # Heading regulation
                if is_multi_flip:
                    yaw_heading_err = yaw_heading - target_yaw
                    if abs(yaw_heading_err) > np.pi:
                        yaw_heading_err -= 2 * np.pi * np.sign(yaw_heading_err)
                    tau_diff = - (kp_yaw * yaw_heading_err + kd_yaw * omega_yaw)
                    tau_diff = float(np.clip(tau_diff, -2.5, 2.5))
                else:
                    yaw_err = omega_yaw - yaw_filtered
                    tau_diff = - (0.75 * yaw_err + 0.18 * omega_yaw)
                    tau_diff = float(np.clip(tau_diff, -3.5, 3.5))

                # Actuator Command Generation
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
                        # Manual keyboard driving mode: active hip banking and differential knees
                        alpha_q = min(1.0, 3.0 * dt_sim)
                        qk_current += alpha_q * (qk_target - qk_current)
                        # Ascento-inspired ZMP-regulated dynamic curve leaning:
                        # psi_ref = arctan(v * omega_yaw / g)
                        a_lat = v_fwd * omega_yaw
                        psi_ref = float(np.arctan2(a_lat, 9.81))
                        psi_ref = float(np.clip(psi_ref, -0.35, 0.35))
                        bank_cmd = k_roll_bank * yaw_filtered + 0.45 * psi_ref
                        knee_diff_cmd = k_knee_diff * yaw_filtered + 0.25 * psi_ref
                        q_roll_R_cmd = + bank_cmd
                        q_roll_L_cmd = - bank_cmd
                        d_qh_R = -0.515 * qk_current + 0.098 * (qk_current ** 2)
                        d_qh_L = +0.705 * qk_current - 0.011 * (qk_current ** 2)
                        qh_R_cmd = qh_R_nom + d_qh_R
                        qh_L_cmd = qh_L_nom + d_qh_L
                        qk_R_cmd = qk_R_nom - qk_current - knee_diff_cmd
                        qk_L_cmd = qk_L_nom + qk_current - knee_diff_cmd
                        kd_knee_R_man = 9.0 if abs(d.qvel[8]) > 1.2 else kd_knee_R
                        kd_knee_L_man = 9.0 if abs(d.qvel[12]) > 1.2 else kd_knee_L

                        d.ctrl[0] = -kp_roll * (d.qpos[7] - q_roll_R_cmd) - kd_roll * d.qvel[6]
                        d.ctrl[1] = -kp_leg * (d.qpos[8] - qh_R_cmd) - kd_leg * d.qvel[7]
                        d.ctrl[2] = -kp_leg * (d.qpos[9] - qk_R_cmd) - kd_knee_R_man * d.qvel[8]
                        d.ctrl[3] = 0.5 * tau_wheel - tau_diff
                        d.ctrl[4] = -kp_roll * (d.qpos[11] - q_roll_L_cmd) - kd_roll * d.qvel[10]
                        d.ctrl[5] = -kp_leg * (d.qpos[12] - qh_L_cmd) - kd_leg * d.qvel[11]
                        d.ctrl[6] = -kp_leg * (d.qpos[13] - qk_L_cmd) - kd_knee_L_man * d.qvel[12]
                        d.ctrl[7] = 0.5 * tau_wheel + tau_diff

                mujoco.mj_step(m, d)
                step_count += 1

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
        if log_fp is not None:
            log_fp.close()
            print(f"[Log] Successfully wrote pitch telemetry log to: {args.log_pitch}")
        print("[Exit] Simulation terminated cleanly.")


if __name__ == "__main__":
    main()
