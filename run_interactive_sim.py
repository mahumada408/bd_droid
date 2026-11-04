#!/usr/bin/env python3
"""
Interactive Real-Time MuJoCo Simulation for the 8-DOF Wheeled Biped Balancer Robot.
Features:
  - Dual Hip Roll Abduction/Adduction for dynamic cornering bank-lean
  - Independent 100mm hub wheel torque control with 3D orientation invariance
  - Active suspension with landing impact damping for the 8cm platform ledge drop
  - Interactive keyboard driving (W/A/S/D/X) and vertical posture control (C/T/N)
"""

import os
import sys
import time
import numpy as np
import mujoco
import mujoco.viewer


def main():
    model_xml = os.path.join(os.path.dirname(__file__), "CAD", "scene.xml")
    if not os.path.isfile(model_xml):
        model_xml = "CAD/scene.xml"

    print(f"[Init] Loading MuJoCo model from: {model_xml}")
    m = mujoco.MjModel.from_xml_path(model_xml)
    d = mujoco.MjData(m)

    # Calibrated Nominal Stance for Rev 3 (aligns Center of Mass directly over wheel axles)
    qh_R_nom = 0.314503    # Right hip pitch (+18.02 deg)
    qk_R_nom = -0.052919   # Right knee (-3.03 deg)
    qh_L_nom = -0.212551   # Left hip pitch (-12.18 deg)
    qk_L_nom = 0.253279    # Left knee (+14.51 deg)

    # Spawn resting on top of raised platform (z = 0.08m)
    d.qpos[0] = 0.0
    d.qpos[1] = 0.0
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

    # Steering and hip roll banking gains
    kp_yaw = 0.75
    kd_yaw = 0.18
    k_roll_bank = 0.38   # rad roll per rad/s yaw rate (aggressive active hip banking)
    k_knee_diff = 0.20   # differential knee extension/crouch for deep motorcycle banking

    # Leg stance & suspension gains
    kp_leg = 70.0
    kd_leg = 5.0
    kp_roll = 85.0       # stiffened roll to hold aggressive lean angle against ground reaction
    kd_roll = 5.5
    kd_susp_impact = 9.0 # extra damping on touchdown to dissipate landing shock

    dt_sim = m.opt.timestep
    last_sim_time = d.time
    step_count = 0

    v_target = 0.0
    yaw_rate_target = 0.0
    v_filtered = 0.0
    yaw_filtered = 0.0
    v_int = 0.0

    qk_target = 0.0
    qk_current = 0.0

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
        print(f"[{action_msg:<14}] Speed: {v_target:+.2f} m/s | Turn: {yaw_rate_target:+.2f} rad/s{lean_str:<18} | Height: {posture_str}")

    def key_callback(keycode):
        nonlocal qk_target, v_target, yaw_rate_target

        if keycode in (ord('W'), ord('w'), 265):
            v_target = np.clip(v_target + 0.20, -0.50, 0.80)
            print_status("Speed Up")
        elif keycode in (ord('S'), ord('s'), 264):
            v_target = np.clip(v_target - 0.20, -0.50, 0.80)
            print_status("Slow/Reverse")
        elif keycode in (ord('A'), ord('a'), 263):
            yaw_rate_target = np.clip(yaw_rate_target + 0.30, -1.20, 1.20)
            print_status("Steer Left")
        elif keycode in (ord('D'), ord('d'), 262):
            yaw_rate_target = np.clip(yaw_rate_target - 0.30, -1.20, 1.20)
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

    try:
        with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
            # Configure default camera to track the robot base smoothly
            base_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "body_base")
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = base_body_id
            viewer.cam.distance = 1.35
            viewer.cam.elevation = -18.0
            viewer.cam.azimuth = 135.0
            while viewer.is_running():
                step_start = time.time()

                # Handle Reset (Backspace key)
                if d.time < last_sim_time:
                    d.qpos[0] = 0.0
                    d.qpos[1] = 0.0
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
                    print("\n[Reset] Robot reset to platform surface (z = 0.08m, zero velocity).")

                last_sim_time = d.time

                # Smooth rate limiting on velocity and steering
                alpha = min(1.0, 4.0 * dt_sim)
                v_filtered += alpha * (v_target - v_filtered)
                yaw_filtered += alpha * (yaw_rate_target - yaw_filtered)

                alpha_q = min(1.0, 3.0 * dt_sim)
                qk_current += alpha_q * (qk_target - qk_current)

                # 3D Invariant Coordinate Tracking using Base Rotation Matrix
                R = d.xmat[1].reshape(3, 3)
                fwd = R[:, 1]
                lat = R[:, 0]

                theta = float(np.arcsin(np.clip(fwd[2], -1.0, 1.0)))
                omega_pitch = float(d.qvel[3])
                omega_yaw = float(d.qvel[5])
                v_fwd = float(np.dot(d.qvel[0:3], fwd))

                # Airborne Anti-Windup during Platform Drop:
                is_airborne = (d.qpos[2] < 0.11 and d.qpos[2] > 0.035 and d.qvel[2] < -0.2)
                if not is_airborne:
                    v_err = v_fwd - v_filtered
                    v_int += v_err * dt_sim
                    v_int = np.clip(v_int, -0.4, 0.4)
                else:
                    v_int = 0.0

                # Cascaded longitudinal pitch tilt reference
                th_ref = kp_v * (v_fwd - v_filtered) + ki_v * v_int
                th_ref = np.clip(th_ref, -max_tilt, max_tilt)

                # Wheel balancing torque
                tau_wheel = - (kp_th * (theta - th_ref) + kd_th * omega_pitch)
                tau_wheel = np.clip(tau_wheel, -15.0, 15.0)

                # Steering differential wheel torque
                yaw_err = omega_yaw - yaw_filtered
                tau_diff = - (kp_yaw * yaw_err + kd_yaw * omega_yaw)
                tau_diff = np.clip(tau_diff, -3.5, 3.5)

                # Direct Hip Roll Banking Lean into Turns:
                # Right roll joint (+Y) and Left roll joint (-Y)
                bank_cmd = k_roll_bank * yaw_filtered
                q_roll_R_cmd = + bank_cmd
                q_roll_L_cmd = - bank_cmd

                # Differential knee flexion for deep motorcycle-style cornering lean:
                # Inside leg drops (crouches) while outside leg extends
                knee_diff_cmd = k_knee_diff * yaw_filtered

                # Coordinated Sagittal Kinematics:
                # Hip pitch counter-rotates as knees flex to keep Center of Mass
                # precisely over the wheel axles across the full deep squat stroke (-5cm).
                d_qh_R = -0.515 * qk_current + 0.098 * (qk_current ** 2)
                d_qh_L = +0.705 * qk_current - 0.011 * (qk_current ** 2)
                qh_R_cmd = qh_R_nom + d_qh_R
                qh_L_cmd = qh_L_nom + d_qh_L

                # Blended vertical height and cornering knee posture
                qk_R_cmd = qk_R_nom - qk_current - knee_diff_cmd
                qk_L_cmd = qk_L_nom + qk_current - knee_diff_cmd

                # Active Suspension Impact Shock Dissipation:
                # Damping stiffens during touchdown impact knee compression
                kd_knee_R = kd_susp_impact if abs(d.qvel[8]) > 1.2 else kd_leg
                kd_knee_L = kd_susp_impact if abs(d.qvel[12]) > 1.2 else kd_leg

                # Apply torques to all 8 actuators:
                # 0: right_hip_roll
                d.ctrl[0] = -kp_roll * (d.qpos[7] - q_roll_R_cmd) - kd_roll * d.qvel[6]
                # 1: right_hip_pitch
                d.ctrl[1] = -kp_leg * (d.qpos[8] - qh_R_cmd) - kd_leg * d.qvel[7]
                # 2: right_knee
                d.ctrl[2] = -kp_leg * (d.qpos[9] - qk_R_cmd) - kd_knee_R * d.qvel[8]
                # 3: right_wheel
                d.ctrl[3] = 0.5 * tau_wheel - tau_diff
                # 4: left_hip_roll
                d.ctrl[4] = -kp_roll * (d.qpos[11] - q_roll_L_cmd) - kd_roll * d.qvel[10]
                # 5: left_hip_pitch
                d.ctrl[5] = -kp_leg * (d.qpos[12] - qh_L_cmd) - kd_leg * d.qvel[11]
                # 6: left_knee
                d.ctrl[6] = -kp_leg * (d.qpos[13] - qk_L_cmd) - kd_knee_L * d.qvel[12]
                # 7: left_wheel
                d.ctrl[7] = 0.5 * tau_wheel + tau_diff

                mujoco.mj_step(m, d)
                step_count += 1

                if step_count % 8 == 0:
                    viewer.sync()

                elapsed = time.time() - step_start
                sleep_time = dt_sim - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[Exit] User interrupted simulation.")
    finally:
        print("[Exit] Simulation terminated cleanly.")


if __name__ == "__main__":
    main()
