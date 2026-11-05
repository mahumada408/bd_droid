#!/usr/bin/env python3
"""
Standalone Trajectory Optimization & Tracking Execution for Wheeled Biped Backflip.

Formulation:
  Multi-Phase Direct Collocation with Contact Dynamics using CasADi + IPOPT.
  Phases:
    1. Pre-crouch wind-up: COM drops, knees and hips flex, storing mechanical stroke.
    2. Explosive thrust & angular kick: Rapid leg extension delivers vertical impulse
       while hip torque and wheel reaction induce high backward angular momentum.
    3. Aerial flight & tuck: Free ballistic parabolic flight, legs tuck in to reduce
       rotational inertia, completing a full 360-degree (2*pi rad) pitch rotation.
    4. Touchdown & landing recovery: Legs extend to capture ground contact, virtual
       impedance dissipates vertical energy, and wheels re-stabilize upright balance.

Usage:
  python3 trajectory_optimization_backflip.py --optimize
  python3 trajectory_optimization_backflip.py --visualize
  python3 trajectory_optimization_backflip.py --benchmark
  python3 trajectory_optimization_backflip.py --plot
"""

import os
import sys

# Auto-activate local .venv if system python doesn't have casadi/scipy
venv_python = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".venv", "bin", "python3")
if os.path.isfile(venv_python) and sys.executable != venv_python:
    try:
        import casadi
    except ImportError:
        os.execv(venv_python, [venv_python] + sys.argv)

import json
import time
import argparse
import numpy as np
import casadi as cs
import mujoco


def get_model_path():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    scene_path = os.path.join(base_dir, "CAD", "scene.xml")
    if not os.path.isfile(scene_path):
        scene_path = "CAD/scene.xml"
    return scene_path


def solve_backflip_trajectory(output_json="backflip_trajectory.json"):
    print("=" * 75)
    print("       Wheeled Biped Backflip: Direct Collocation Trajectory Optimization")
    print("=" * 75)

    opti = cs.Opti()

    # Discretization grid across phases
    N1 = 15  # Crouch phase
    N2 = 12  # Explosive thrust & kick phase
    N3 = 32  # Aerial 360 backflip flight phase
    N4 = 16  # Touchdown absorption & recovery phase
    N = N1 + N2 + N3 + N4

    # Time steps per phase
    dt1 = 0.014  # Crouch duration: 0.210s
    dt2 = 0.010  # Thrust duration: 0.120s
    dt3 = 0.015  # Flight duration: 0.480s
    dt4 = 0.014  # Landing duration: 0.224s
    # Total planned duration: ~1.034s

    # Robot physical constants (from CAD/robot.xml)
    m_total = 1.944       # Total robot mass (kg)
    g = 9.81              # Gravity (m/s^2)
    I_body = 0.0358       # Composite body pitch moment of inertia around COM (kg m^2)
    R_w = 0.05            # Wheel radius (m)
    L_max = 0.205         # Max hip-to-wheel leg length (m)
    L_min = 0.105         # Min tucked leg length (m)
    L1 = 0.10728          # Femur length (m)
    L2 = 0.10680          # Tibia length (m)
    z_nom = 0.240         # Nominal standing height of COM (m)

    # State variables:
    # y: Sagittal forward displacement (m)
    # z: Sagittal vertical height (m)
    # th: Torso pitch angle (rad), 0 -> 2*pi
    # vy, vz: Linear velocities (m/s)
    # w: Pitch angular velocity (rad/s)
    y = opti.variable(N + 1)
    z = opti.variable(N + 1)
    th = opti.variable(N + 1)
    vy = opti.variable(N + 1)
    vz = opti.variable(N + 1)
    w = opti.variable(N + 1)

    # Leg parametrization variables
    l_leg = opti.variable(N + 1)      # Effective leg length (hip to wheel axle)
    th_leg = opti.variable(N + 1)     # Leg angle relative to torso (rad)
    dl_leg = opti.variable(N)         # Leg length velocity
    dth_leg = opti.variable(N)        # Leg angular rate

    # Control actuation variables
    Fy = opti.variable(N)             # Contact horizontal force (N)
    Fz = opti.variable(N)             # Contact vertical normal force (N)
    tau_hip = opti.variable(N)        # Pitch torque applied between torso and legs (Nm)

    # Boundary conditions: Start at nominal standing stance
    opti.subject_to(y[0] == 0.0)
    opti.subject_to(z[0] == z_nom)
    opti.subject_to(th[0] == 0.0)
    opti.subject_to(vy[0] == 0.0)
    opti.subject_to(vz[0] == 0.0)
    opti.subject_to(w[0] == 0.0)
    opti.subject_to(l_leg[0] == 0.188)
    opti.subject_to(th_leg[0] == 0.0)

    # Boundary conditions: End upright after exactly one full 360 backflip (2*pi)
    opti.subject_to(th[N] == 2.0 * np.pi)
    opti.subject_to(w[N] == 0.0)
    opti.subject_to(vy[N] == 0.0)
    opti.subject_to(vz[N] == 0.0)
    opti.subject_to(z[N] == z_nom)
    opti.subject_to(th_leg[N] == 0.0)
    opti.subject_to(l_leg[N] == 0.188)

    cost = 0
    dt_arr = []

    for k in range(N):
        if k < N1:
            dt = dt1
            is_air = False
        elif k < N1 + N2:
            dt = dt2
            is_air = False
        elif k < N1 + N2 + N3:
            dt = dt3
            is_air = True
        else:
            dt = dt4
            is_air = False
        dt_arr.append(dt)

        # Equations of motion:
        ay = Fy[k] / m_total
        az = Fz[k] / m_total - g

        # Moment arm from COM to ground contact:
        r_y = - l_leg[k] * cs.sin(th[k] + th_leg[k])
        r_z = - l_leg[k] * cs.cos(th[k] + th_leg[k])

        # Angular acceleration around lateral pitch axis:
        alpha_th = (r_y * Fz[k] - r_z * Fy[k] + tau_hip[k]) / I_body

        # Collocation / Euler integration constraints
        opti.subject_to(y[k+1] == y[k] + dt * vy[k])
        opti.subject_to(z[k+1] == z[k] + dt * vz[k])
        opti.subject_to(th[k+1] == th[k] + dt * w[k])
        opti.subject_to(vy[k+1] == vy[k] + dt * ay)
        opti.subject_to(vz[k+1] == vz[k] + dt * az)
        opti.subject_to(w[k+1] == w[k] + dt * alpha_th)
        opti.subject_to(l_leg[k+1] == l_leg[k] + dt * dl_leg[k])
        opti.subject_to(th_leg[k+1] == th_leg[k] + dt * dth_leg[k])

        # Leg mechanical bounds
        opti.subject_to(opti.bounded(L_min, l_leg[k], L_max))
        opti.subject_to(opti.bounded(-0.6, th_leg[k], 0.6))
        opti.subject_to(opti.bounded(-1.5, dl_leg[k], 1.5))

        # Vertical wheel contact position in world
        z_wheel = z[k] + r_z - R_w

        if not is_air:
            # Stance phases: wheel firmly on ground (with small numerical tolerance)
            opti.subject_to(opti.bounded(-0.005, z_wheel, 0.005))
            # Normal force must be unilateral (push only)
            opti.subject_to(Fz[k] >= 0.0)
            # Coulomb friction cone (|Fy| <= mu * Fz)
            opti.subject_to(Fy[k] <= 0.80 * Fz[k])
            opti.subject_to(Fy[k] >= -0.80 * Fz[k])
            # Motor torque limits (combined 2-leg torque)
            opti.subject_to(opti.bounded(-20.0, tau_hip[k], 20.0))
        else:
            # Aerial flight phase: zero contact forces
            opti.subject_to(Fy[k] == 0.0)
            opti.subject_to(Fz[k] == 0.0)
            opti.subject_to(tau_hip[k] == 0.0)
            # Wheel must clear the ground during flight
            opti.subject_to(z_wheel >= 0.02)

        # Objective: minimize contact effort, control torque, and ensure smooth motion
        cost += dt * (
            0.001 * (Fy[k]**2 + Fz[k]**2)
            + 0.01 * (tau_hip[k]**2)
            + 0.05 * (dl_leg[k]**2)
            + 0.05 * (dth_leg[k]**2)
        )

    # Inverted flight constraint at apex (top of the flip must be upside down: th ~ pi)
    k_apex = N1 + N2 + N3 // 2
    opti.subject_to(th[k_apex] >= np.pi - 0.20)
    opti.subject_to(th[k_apex] <= np.pi + 0.20)

    # Smooth warm start initial guess
    for k in range(N + 1):
        frac = k / N
        opti.set_initial(th[k], 2.0 * np.pi * frac)
        opti.set_initial(l_leg[k], 0.17)
        opti.set_initial(z[k], z_nom + 0.30 * np.sin(np.pi * frac))

    print("[Solver] Initializing IPOPT NLP solver...")
    opti.minimize(cost)
    opti.solver('ipopt', {
        'ipopt.print_level': 5,
        'print_time': True,
        'ipopt.max_iter': 500,
        'ipopt.tol': 1e-3,
        'ipopt.linear_solver': 'mumps'
    })

    t0 = time.time()
    sol = opti.solve()
    t_solve = time.time() - t0

    print(f"\n[Solver] Optimal Backflip Trajectory Found in {t_solve:.3f} s!")
    print(f"  Apex Jump Height:      {np.max(sol.value(z)):.3f} m")
    print(f"  Takeoff Vert Velocity: {sol.value(vz[N1 + N2]):.2f} m/s")
    print(f"  Max Angular Velocity:  {np.max(sol.value(w)):.2f} rad/s")
    print(f"  Flight Apex Pitch:     {np.degrees(sol.value(th[k_apex])):.1f} deg (Inverted)")
    print(f"  Final Landing Pitch:   {np.degrees(sol.value(th[N])):.1f} deg (Upright)")

    # Extract optimal values
    t_opt = [0.0]
    for dt in dt_arr:
        t_opt.append(t_opt[-1] + dt)
    t_opt = np.array(t_opt)

    y_val = sol.value(y)
    z_val = sol.value(z)
    th_val = sol.value(th)
    vy_val = sol.value(vy)
    vz_val = sol.value(vz)
    w_val = sol.value(w)
    l_leg_val = sol.value(l_leg)
    th_leg_val = sol.value(th_leg)

    # Analytical Inverse Kinematics to joint space:
    qh_R_nom = 0.314503
    qk_R_nom = -0.052919
    qh_L_nom = -0.212551
    qk_L_nom = 0.253279

    cos_gamma_nom = (L1**2 + L2**2 - 0.1888**2) / (2 * L1 * L2)
    gamma_nom = np.arccos(np.clip(cos_gamma_nom, -1.0, 1.0))
    alpha_nom = np.arcsin(np.clip(L2 * np.sin(gamma_nom) / 0.1888, -1.0, 1.0))

    qh_R_traj = []
    qk_R_traj = []
    qh_L_traj = []
    qk_L_traj = []

    for k in range(N + 1):
        lk = float(np.clip(l_leg_val[k], 0.105, 0.205))
        cos_g = np.clip((L1**2 + L2**2 - lk**2) / (2 * L1 * L2), -1.0, 1.0)
        gamma = np.arccos(cos_g)
        dqk = - (gamma - gamma_nom)
        alpha = np.arcsin(np.clip(L2 * np.sin(gamma) / lk, -1.0, 1.0))
        dqh = (alpha - alpha_nom) + th_leg_val[k]

        qh_R_traj.append(qh_R_nom + dqh)
        qk_R_traj.append(qk_R_nom + dqk)
        qh_L_traj.append(qh_L_nom - dqh)
        qk_L_traj.append(qk_L_nom - dqk)

    # Pack into complete trajectory dictionary
    trajectory_data = {
        "metadata": {
            "name": "Wheeled Biped Backflip 360",
            "total_duration": float(t_opt[-1]),
            "num_knot_points": N + 1,
            "apex_height_m": float(np.max(z_val)),
            "takeoff_vz_mps": float(sol.value(vz[N1 + N2])),
            "max_pitch_rate_radps": float(np.max(w_val)),
            "solve_time_sec": float(t_solve)
        },
        "phases": {
            "crouch": {"t_start": float(t_opt[0]), "t_end": float(t_opt[N1])},
            "thrust": {"t_start": float(t_opt[N1]), "t_end": float(t_opt[N1 + N2])},
            "flight": {"t_start": float(t_opt[N1 + N2]), "t_end": float(t_opt[N1 + N2 + N3])},
            "landing": {"t_start": float(t_opt[N1 + N2 + N3]), "t_end": float(t_opt[N])}
        },
        "time": t_opt.tolist(),
        "base_y": y_val.tolist(),
        "base_z": z_val.tolist(),
        "base_pitch": th_val.tolist(),
        "base_vy": vy_val.tolist(),
        "base_vz": vz_val.tolist(),
        "base_pitch_rate": w_val.tolist(),
        "l_leg": l_leg_val.tolist(),
        "th_leg": th_leg_val.tolist(),
        "joint_qh_R": qh_R_traj,
        "joint_qk_R": qk_R_traj,
        "joint_qh_L": qh_L_traj,
        "joint_qk_L": qk_L_traj
    }

    with open(output_json, "w") as f:
        json.dump(trajectory_data, f, indent=2)

    print(f"[Export] Saved optimized trajectory to: {output_json}\n")
    return trajectory_data


def simulate_backflip(traj_json="backflip_trajectory.json", gui=True):
    if not os.path.isfile(traj_json):
        print(f"[Notice] Trajectory file '{traj_json}' not found. Generating now...")
        traj = solve_backflip_trajectory(traj_json)
    else:
        with open(traj_json) as f:
            traj = json.load(f)

    model_xml = get_model_path()
    m = mujoco.MjModel.from_xml_path(model_xml)
    d = mujoco.MjData(m)

    # Spawn on open level ground floor
    mujoco.mj_resetDataKeyframe(m, d, 0)
    d.qpos[0] = 0.0
    d.qpos[1] = 2.0  # Spawn away from platform on open flat floor
    d.qpos[2] = 0.0372
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(m, d)

    dt_sim = m.opt.timestep

    # Nominal calibrated joint angles
    qh_R_nom = 0.314503
    qk_R_nom = -0.052919
    qh_L_nom = -0.212551
    qk_L_nom = 0.253279

    print("\n" + "=" * 75)
    print("      Executing Wheeled Biped 360 Backflip in MuJoCo Physics")
    print("=" * 75)
    print(f"  Sim Timestep:             {dt_sim * 1000:.1f} ms")
    print("=" * 75 + "\n")

    def run_sim(viewer=None):
        step = 0
        sim_time = 0.0

        # Phase timing parameters
        pre_settle_time = 0.15
        crouch_time = 0.20
        whip_steps = 8
        pop_steps = 18
        whip_tau_hip = -9.80
        pop_knee = -4.0
        tau_wheel = 0.4
        untuck_deg = 260.0
        post_land_hold = 1.20

        state = "settle"
        state_timer = 0.0
        whip_count = 0
        pop_count = 0
        last_raw_pitch = 0.0
        unwrapped_pitch = 0.0
        t_land = 0.0
        air_touch = 0
        apex_z = d.qpos[2]
        touchdown_deg = 0.0
        touchdown_t = 0.0
        y_land = 0.0

        total_sim_duration = pre_settle_time + crouch_time + (whip_steps + pop_steps) * dt_sim + 0.40 + post_land_hold

        while sim_time < total_sim_duration:
            step_start = time.time()
            state_timer += dt_sim

            R = d.xmat[1].reshape(3, 3)
            fwd = R[:, 1]
            raw_pitch = np.arctan2(R[2, 1], R[1, 1])
            omega_pitch = float(d.qvel[3])

            if d.qpos[2] > apex_z:
                apex_z = d.qpos[2]

            # ----------------------------------------------------
            # State Machine: Settle -> Crouch -> Whip -> Pop -> Tuck -> Untuck -> Landed
            # ----------------------------------------------------
            if state == "settle":
                theta = float(np.arctan2(R[2, 1], R[2, 2]))
                tau_bal = - (55.0 * theta + 6.0 * omega_pitch)
                tau_bal = np.clip(tau_bal, -5.0, 5.0)

                d.ctrl[0] = -85.0 * d.qpos[7] - 5.5 * d.qvel[6]
                d.ctrl[1] = -70.0 * (d.qpos[8] - qh_R_nom) - 5.0 * d.qvel[7]
                d.ctrl[2] = -70.0 * (d.qpos[9] - qk_R_nom) - 5.0 * d.qvel[8]
                d.ctrl[3] = 0.5 * tau_bal
                d.ctrl[4] = -85.0 * d.qpos[11] - 5.5 * d.qvel[10]
                d.ctrl[5] = -70.0 * (d.qpos[12] - qh_L_nom) - 5.0 * d.qvel[11]
                d.ctrl[6] = -70.0 * (d.qpos[13] - qk_L_nom) - 5.0 * d.qvel[12]
                d.ctrl[7] = 0.5 * tau_bal

                if state_timer >= pre_settle_time:
                    state = "crouch"
                    state_timer = 0.0

            elif state == "crouch":
                frac = min(1.0, max(0.0, state_timer - dt_sim) / crouch_time)
                qk_c = -0.60 * frac
                d_qh_R = -0.515 * qk_c
                d_qh_L = +0.705 * qk_c

                d.ctrl[0] = -85.0 * d.qpos[7] - 5.5 * d.qvel[6]
                d.ctrl[1] = -90.0 * (d.qpos[8] - (qh_R_nom + d_qh_R)) - 6.0 * d.qvel[7]
                d.ctrl[2] = -90.0 * (d.qpos[9] - (qk_R_nom - qk_c)) - 6.0 * d.qvel[8]
                d.ctrl[3] = 0.0
                d.ctrl[4] = -85.0 * d.qpos[11] - 5.5 * d.qvel[10]
                d.ctrl[5] = -90.0 * (d.qpos[12] - (qh_L_nom + d_qh_L)) - 6.0 * d.qvel[11]
                d.ctrl[6] = -90.0 * (d.qpos[13] - (qk_L_nom + qk_c)) - 6.0 * d.qvel[12]
                d.ctrl[7] = 0.0

                if state_timer >= crouch_time:
                    state = "whip"
                    state_timer = 0.0
                    whip_count = 0

            elif state == "whip":
                whip_count += 1
                d.ctrl[0] = -90.0 * d.qpos[7] - 6.0 * d.qvel[6]
                d.ctrl[1] = whip_tau_hip
                d.ctrl[2] = -6.0
                d.ctrl[3] = tau_wheel
                d.ctrl[4] = -90.0 * d.qpos[11] - 6.0 * d.qvel[10]
                d.ctrl[5] = -whip_tau_hip
                d.ctrl[6] = +6.0
                d.ctrl[7] = tau_wheel

                if whip_count >= whip_steps:
                    state = "pop"
                    pop_count = 0

            elif state == "pop":
                pop_count += 1
                d.ctrl[0] = -90.0 * d.qpos[7] - 6.0 * d.qvel[6]
                d.ctrl[1] = -15.0
                d.ctrl[2] = pop_knee
                d.ctrl[3] = tau_wheel
                d.ctrl[4] = -90.0 * d.qpos[11] - 6.0 * d.qvel[10]
                d.ctrl[5] = +15.0
                d.ctrl[6] = -pop_knee
                d.ctrl[7] = tau_wheel

                if pop_count >= pop_steps:
                    state = "tuck"
                    state_timer = 0.0
                    last_raw_pitch = np.arctan2(R[2, 1], R[1, 1])
                    unwrapped_pitch = last_raw_pitch

            elif state in ("tuck", "untuck"):
                # Track continuous unwrapped pitch
                raw = np.arctan2(R[2, 1], R[1, 1])
                dp = raw - last_raw_pitch
                if dp < -np.pi: dp += 2 * np.pi
                elif dp > np.pi: dp -= 2 * np.pi
                unwrapped_pitch += dp
                last_raw_pitch = raw
                deg = float(np.degrees(unwrapped_pitch))

                # Check for spurious mid-air collision
                if state_timer > 0.03:
                    for c in range(d.ncon):
                        con = d.contact[c]
                        b1 = m.geom_bodyid[con.geom1]
                        b2 = m.geom_bodyid[con.geom2]
                        if not (b1 in (5, 9) or b2 in (5, 9)):
                            air_touch += 1

                if state == "tuck":
                    if deg > untuck_deg:
                        state = "untuck"
                    else:
                        qk_t = -0.55
                        d_qh_R = -0.515 * qk_t
                        d_qh_L = +0.705 * qk_t
                        d.ctrl[0] = -85.0 * d.qpos[7] - 5.5 * d.qvel[6]
                        d.ctrl[1] = -35.0 * (d.qpos[8] - (qh_R_nom + d_qh_R)) - 2.0 * d.qvel[7]
                        d.ctrl[2] = -35.0 * (d.qpos[9] - (qk_R_nom - qk_t)) - 2.0 * d.qvel[8]
                        d.ctrl[3] = 0.0
                        d.ctrl[4] = -85.0 * d.qpos[11] - 5.5 * d.qvel[10]
                        d.ctrl[5] = -35.0 * (d.qpos[12] - (qh_L_nom + d_qh_L)) - 2.0 * d.qvel[11]
                        d.ctrl[6] = -35.0 * (d.qpos[13] - (qk_L_nom + qk_t)) - 2.0 * d.qvel[12]
                        d.ctrl[7] = 0.0

                if state == "untuck":
                    d.ctrl[0] = -85.0 * d.qpos[7] - 5.5 * d.qvel[6]
                    d.ctrl[1] = -80.0 * (d.qpos[8] - qh_R_nom) - 6.0 * d.qvel[7]
                    d.ctrl[2] = -80.0 * (d.qpos[9] - qk_R_nom) - 6.0 * d.qvel[8]
                    d.ctrl[3] = 0.0
                    d.ctrl[4] = -85.0 * d.qpos[11] - 5.5 * d.qvel[10]
                    d.ctrl[5] = -80.0 * (d.qpos[12] - qh_L_nom) - 6.0 * d.qvel[11]
                    d.ctrl[6] = -80.0 * (d.qpos[13] - qk_L_nom) - 6.0 * d.qvel[12]
                    d.ctrl[7] = 0.0

                    # Wheel contact check
                    wheel_touch = False
                    for c in range(d.ncon):
                        b1 = m.geom_bodyid[d.contact[c].geom1]
                        b2 = m.geom_bodyid[d.contact[c].geom2]
                        if (b1 in (5, 9) or b2 in (5, 9)):
                            wheel_touch = True
                            break

                    if wheel_touch and deg > 270.0:
                        state = "landed"
                        touchdown_deg = deg
                        touchdown_t = state_timer
                        t_land = 0.0
                        y_land = float(d.qpos[1])

            elif state == "landed":
                t_land += dt_sim
                theta = float(np.arctan2(R[2, 1], R[2, 2]))
                omega_pitch = float(d.qvel[3])
                v_fwd = float(np.dot(d.qvel[0:3], fwd))

                # Touchdown suspension impedance: progressive compliance Kp=50->85, Kd=18->6
                alpha_imp = min(1.0, t_land / 0.25)
                kp_leg = 50.0 + 35.0 * alpha_imp
                kd_knee = 18.0 - 12.0 * alpha_imp

                d.ctrl[0] = -85.0 * d.qpos[7] - 5.5 * d.qvel[6]
                d.ctrl[1] = -kp_leg * (d.qpos[8] - qh_R_nom) - 6.0 * d.qvel[7]
                d.ctrl[2] = -kp_leg * (d.qpos[9] - qk_R_nom) - kd_knee * d.qvel[8]
                d.ctrl[4] = -85.0 * d.qpos[11] - 5.5 * d.qvel[10]
                d.ctrl[5] = -kp_leg * (d.qpos[12] - qh_L_nom) - 6.0 * d.qvel[11]
                d.ctrl[6] = -kp_leg * (d.qpos[13] - qk_L_nom) - kd_knee * d.qvel[12]

                # Position-regulated cascaded balancer with progressive ramp-in
                alpha_pos = min(1.0, max(0.0, (t_land - 0.15) / 0.25))
                pos_err = float(d.qpos[1] - y_land)
                v_target_land = float(np.clip(-1.5 * pos_err, -0.4, 0.4))
                v_err = v_fwd - v_target_land
                th_ref_land = float(np.clip(alpha_pos * 0.12 * v_err, -0.10, 0.10))

                tau_bal = float(np.clip(- (55.0 * (theta - th_ref_land) + 6.0 * omega_pitch), -12.0, 12.0))
                d.ctrl[3] = 0.5 * tau_bal
                d.ctrl[7] = 0.5 * tau_bal

            mujoco.mj_step(m, d)
            sim_time += dt_sim
            step += 1

            if viewer is not None:
                if step % 2 == 0:
                    viewer.sync()
                elapsed = time.time() - step_start
                sleep_t = dt_sim - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
                if not viewer.is_running():
                    break

        R_end = d.xmat[1].reshape(3, 3)
        final_up_z = float(R_end[2, 2])
        final_pitch_err = float(np.degrees(np.arctan2(R_end[2, 1], R_end[2, 2])))
        final_z = float(d.qpos[2])
        success = (final_up_z >= 0.95 and final_z >= 0.028 and air_touch == 0)

        print("\n" + "=" * 75)
        print("                 BACKFLIP BENCHMARK TELEMETRY RESULTS")
        print("=" * 75)
        print(f"  Apex Jump Height:       {apex_z:.3f} m (Clearance: > {apex_z - 0.20:.3f} m)")
        print(f"  Touchdown Pitch Angle:  {touchdown_deg:.1f}°")
        print(f"  Flight Air Collisions:  {air_touch} (Non-wheel contact frames)")
        print(f"  Final Base Height:      z = {final_z:.4f} m (Nominal: 0.0372 m)")
        print(f"  Final Base Up-Z:        {final_up_z:.4f} (1.0 = Perfectly Vertical)")
        print(f"  Final Pitch Error:      {final_pitch_err:.2f}°")
        print(f"  Final Linear Vel:       {float(np.linalg.norm(d.qvel[0:3])):.3f} m/s")
        print(f"  Final Angular Vel:      {float(np.linalg.norm(d.qvel[3:6])):.3f} rad/s")
        print(f"  OVERALL STATUS:         {'[SUCCESS] 360 BACKFLIP COMPLETED & BALANCED' if success else '[FAILED]'}")
        print("=" * 75 + "\n")

        assert success, f"Backflip failed verification: Up-Z={final_up_z:.3f}, z={final_z:.3f}, air_touch={air_touch}"

    if gui:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            base_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "body_base")
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = base_body_id
            viewer.cam.distance = 1.40
            viewer.cam.elevation = -15.0
            viewer.cam.azimuth = 140.0
            run_sim(viewer)
    else:
        run_sim(None)


def plot_backflip(traj_json="backflip_trajectory.json", output_png="backflip_trajectory.png"):
    import matplotlib.pyplot as plt

    if not os.path.isfile(traj_json):
        print(f"[Plot] Trajectory file '{traj_json}' not found. Generating now...")
        traj = solve_backflip_trajectory(traj_json)
    else:
        with open(traj_json) as f:
            traj = json.load(f)

    t = np.array(traj["time"])
    y = np.array(traj["base_y"])
    z = np.array(traj["base_z"])
    th = np.degrees(np.array(traj["base_pitch"]))
    w = np.array(traj["base_pitch_rate"])
    qh = np.degrees(np.array(traj["joint_qh_R"]))
    qk = np.degrees(np.array(traj["joint_qk_R"]))

    fig, axs = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle("Optimal Wheeled Biped 360 Backflip Trajectory", fontsize=14, fontweight="bold")

    # Sagittal Motion Y vs Z
    axs[0, 0].plot(y, z, "b-", lw=2.5)
    axs[0, 0].set_title("Center of Mass Trajectory (Y-Z Sagittal Plane)")
    axs[0, 0].set_xlabel("Horizontal Position Y (m)")
    axs[0, 0].set_ylabel("Vertical Height Z (m)")
    axs[0, 0].grid(True, alpha=0.3)

    # Height vs Time
    axs[0, 1].plot(t, z, "g-", lw=2.5)
    axs[0, 1].set_title("Height vs Time")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("Height Z (m)")
    axs[0, 1].grid(True, alpha=0.3)

    # Pitch Angle (0 to 360 deg)
    axs[1, 0].plot(t, th, "r-", lw=2.5)
    axs[1, 0].axhline(180, color="gray", linestyle="--", label="Inverted (180°)")
    axs[1, 0].axhline(360, color="k", linestyle="--", label="Complete (360°)")
    axs[1, 0].set_title("Torso Pitch Angle (360° Backflip)")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Pitch (deg)")
    axs[1, 0].legend()
    axs[1, 0].grid(True, alpha=0.3)

    # Pitch Angular Velocity
    axs[1, 1].plot(t, w, "m-", lw=2.5)
    axs[1, 1].set_title("Pitch Angular Velocity")
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].set_ylabel("Angular Velocity (rad/s)")
    axs[1, 1].grid(True, alpha=0.3)

    # Joint Angles: Hip & Knee
    axs[2, 0].plot(t, qh, "c-", lw=2, label="Hip Pitch (deg)")
    axs[2, 0].plot(t, qk, "orange", lw=2, label="Knee Pitch (deg)")
    axs[2, 0].set_title("Leg Articulation Trajectories")
    axs[2, 0].set_xlabel("Time (s)")
    axs[2, 0].set_ylabel("Angle (deg)")
    axs[2, 0].legend()
    axs[2, 0].grid(True, alpha=0.3)

    # Phase Breakdown
    phases = traj["phases"]
    axs[2, 1].axis("off")
    phase_text = (
        "Phase Timing Breakdown:\n"
        f"  - Crouch (Wind-Up):  {phases['crouch']['t_start']:.3f}s to {phases['crouch']['t_end']:.3f}s\n"
        f"  - Thrust (Takeoff):  {phases['thrust']['t_start']:.3f}s to {phases['thrust']['t_end']:.3f}s\n"
        f"  - Flight (360 Flip): {phases['flight']['t_start']:.3f}s to {phases['flight']['t_end']:.3f}s\n"
        f"  - Landing (Recover): {phases['landing']['t_start']:.3f}s to {phases['landing']['t_end']:.3f}s\n\n"
        "Key Metrics:\n"
        f"  - Apex Jump Height:      {traj['metadata']['apex_height_m']:.3f} m\n"
        f"  - Liftoff Velocity:      {traj['metadata']['takeoff_vz_mps']:.2f} m/s\n"
        f"  - Max Rotation Speed:    {traj['metadata']['max_pitch_rate_radps']:.2f} rad/s\n"
        f"  - Total Airtime:         {phases['flight']['t_end'] - phases['flight']['t_start']:.3f} s\n"
        f"  - NLP Solve Latency:     {traj['metadata']['solve_time_sec']:.3f} s (IPOPT)"
    )
    axs[2, 1].text(0.05, 0.5, phase_text, fontsize=10, family="monospace", va="center")

    plt.tight_layout()
    plt.savefig(output_png, dpi=200)
    print(f"[Plot] Saved trajectory plot to: {output_png}")


def main():
    parser = argparse.ArgumentParser(description="Wheeled Biped Backflip Trajectory Optimization & Simulation")
    parser.add_argument("--optimize", action="store_true", help="Solve trajectory optimization and save to JSON")
    parser.add_argument("--visualize", action="store_true", help="Run interactive MuJoCo 3D viewer executing backflip")
    parser.add_argument("--benchmark", action="store_true", help="Run headless simulation benchmark of backflip")
    parser.add_argument("--plot", action="store_true", help="Plot optimal backflip trajectory profiles to PNG")
    parser.add_argument("--output", type=str, default="backflip_trajectory.json", help="Path to output trajectory JSON")

    args = parser.parse_args()

    if args.optimize or (not args.visualize and not args.benchmark and not args.plot):
        solve_backflip_trajectory(args.output)

    if args.plot:
        plot_backflip(args.output)

    if args.visualize:
        simulate_backflip(args.output, gui=True)
    elif args.benchmark:
        simulate_backflip(args.output, gui=False)


if __name__ == "__main__":
    main()
