#!/usr/bin/env python3
"""
Ascento-Inspired Dynamic Whole-Body Model Predictive Control (NMPC)
Symbolic Equations of Motion & acados SQP-RTI C-Code Generator
for the Wheeled Biped / Acrobat Balancer Robot (CAD/robot.xml).

Inspired by:
"LQR-Assisted Whole-Body Control of a Wheeled Bipedal Robot with Kinematic Loops"
(Klemm, Morra, Gulich, Mannhart, Rohr, Kamel, de Viragh, Siegwart - ETH Zurich / ASL, IEEE RAL 2020)

Core Innovations from the Ascento Architecture:
1. Coupled Non-Minimum Phase Pitch & Velocity Dynamics:
   Directly captures the fundamental physics of wheeled balancing: to accelerate
   forward rapidly, the controller commands an initial counter-steering pulse
   to tilt the body forward into the direction of motion, avoiding tip-over.
2. Dynamic Variable Pendulum Length & Variable Pitch Inertia:
   The effective pendulum length l(t) and moment of inertia I_y(l) = I_0 + M*l^2
   are state variables, enabling dynamic crouching into turns and explosive extensions.
3. ZMP-Regulated Active Roll Lean (Curve Banking):
   Calculates lateral centrifugal acceleration a_lat = v * omega_yaw and actively
   regulates the roll lean angle to psi_ref = arctan(v * omega_yaw / g), shifting the
   Zero-Moment Point (ZMP) directly to the center of the Line of Support (G) to prevent
   tripping and maximize cornering traction at high speed.
4. Active Vertical Force Modulation:
   Directly optimizes total vertical leg extension force F_l for active suspension
   damping, disturbance rejection, and terrain height compliance.

States (nx = 8):
  [p, v, theta, omega, l, dl, psi, dpsi]
    - p: Forward displacement along path (m)
    - v: Forward ground velocity (m/s)
    - theta: Base pitch angle from vertical (rad)
    - omega: Base pitch angular velocity (rad/s)
    - l: Effective pendulum leg length (m)
    - dl: Leg extension velocity (m/s)
    - psi: Base roll lean angle (rad)
    - dpsi: Base roll angular velocity (rad/s)

Controls (nu = 3):
  [tau_w, F_l, tau_roll]
    - tau_w: Net symmetric wheel drive torque (Nm)
    - F_l: Total vertical leg thrust / extension force (N)
    - tau_roll: Active lateral roll torque for ZMP banking (Nm)

Online Parameters (np = 1):
  [omega_yaw]
    - omega_yaw: Current yaw angular rate (rad/s) for centrifugal coupling
"""

import os
import sys
import numpy as np
import casadi as cs
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


def export_robot_model() -> AcadosModel:
    model_name = "biped_balancer"

    # States (nx = 8)
    p = cs.SX.sym('p')
    v = cs.SX.sym('v')
    theta = cs.SX.sym('theta')
    omega = cs.SX.sym('omega')
    l = cs.SX.sym('l')
    dl = cs.SX.sym('dl')
    psi = cs.SX.sym('psi')
    dpsi = cs.SX.sym('dpsi')
    x = cs.vertcat(p, v, theta, omega, l, dl, psi, dpsi)

    # Controls (nu = 3)
    tau_w = cs.SX.sym('tau_w')
    F_l = cs.SX.sym('F_l')
    tau_roll = cs.SX.sym('tau_roll')
    u = cs.vertcat(tau_w, F_l, tau_roll)

    # Online Parameter (np = 1)
    omega_yaw = cs.SX.sym('omega_yaw')
    p_param = cs.vertcat(omega_yaw)

    # Physical Robot Parameters directly extracted from CAD/robot.xml:
    # Total robot mass: 1.943 kg
    # Nominal COM distance from wheel axle: 0.148 m
    # Total pitch inertia about root: 0.067 kg m^2
    # Total roll inertia about root: 0.076 kg m^2
    M = 1.943          # Torso + leg links lumped mass (kg)
    m_w = 0.0864       # Two wheels combined mass (kg)
    R = 0.04           # Wheel radius (m)
    I_w = 1.08e-4      # Wheel rotational inertia (kg m^2)
    M_tot = M + m_w + (I_w / (R**2))  # 2.097 kg

    l_nom = 0.148      # Nominal pendulum length (m)
    I_y0 = 0.0668      # Base pitch inertia (kg m^2)
    I_x0 = 0.0765      # Base roll inertia (kg m^2)
    g = 9.81           # Gravity (m/s^2)
    b_l = 6.0          # Leg extension damping (N s / m)
    b_roll = 0.45      # Roll lean damping (N m s / rad)

    # Coupled Pitch-Longitudinal Non-Minimum Phase Dynamics
    # Variable inertia I_y(l) as function of instantaneous leg height via parallel axis theorem
    I_y = I_y0 + M * (l**2)
    det_M = M_tot * I_y - (M * l * cs.cos(theta))**2

    # Forward force from wheel torque + centripetal and Coriolis cross-terms
    f_v = (tau_w / R) + M * l * (omega**2) * cs.sin(theta) - 2.0 * M * dl * omega * cs.cos(theta)
    # Pitch moment: gravity destabilizes, wheel torque reaction -tau_w counter-steers
    f_th = M * g * l * cs.sin(theta) - tau_w - 2.0 * M * l * dl * omega

    v_dot = (I_y * f_v - (M * l * cs.cos(theta)) * f_th) / det_M
    omega_dot = (-(M * l * cs.cos(theta)) * f_v + M_tot * f_th) / det_M

    # Vertical Suspension & Dynamic Height Modulation
    # Leg acceleration accounts for applied thrust F_l, gravity, and centrifugal expansion
    l_ddot = (F_l - M * g - b_l * dl) / M + l * (omega**2) + l * (dpsi**2)

    # Lateral ZMP-Regulated Roll Lean Dynamics
    # Lateral centrifugal acceleration in curves: a_lat = v * omega_yaw
    a_lat = v * omega_yaw
    I_x = I_x0 + M * (l**2)
    psi_ddot = (tau_roll - M * g * l * cs.sin(psi) + M * a_lat * l * cs.cos(psi) - b_roll * dpsi) / I_x

    f_expl = cs.vertcat(v, v_dot, omega, omega_dot, dl, l_ddot, dpsi, psi_ddot)

    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.u = u
    model.p = p_param
    model.f_expl_expr = f_expl
    model.xdot = cs.SX.sym('xdot', 8)
    model.f_impl_expr = model.xdot - f_expl
    return model


def generate_solver():
    if 'ACADOS_SOURCE_DIR' not in os.environ:
        os.environ['ACADOS_SOURCE_DIR'] = '/home/manuel/software/acados'

    ocp = AcadosOcp()
    model = export_robot_model()
    ocp.model = model

    # Horizon parameters: 20 stages over 0.40 seconds (20ms step)
    # Provides fast MPC preview for non-minimum phase counter-pitching and ZMP roll anticipation
    N = 20
    Tf = 0.40
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    nx = 8
    nu = 3
    ny = nx + nu
    ny_e = nx

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    # Ascento-Tuned Weight Matrices:
    # High pitch authority + agile velocity tracking + precise ZMP roll lean
    Q = np.array([
        5.0,    # p:    position tracking / holding
        25.0,   # v:    forward velocity tracking authority
        220.0,  # theta: firm pitch balance regulation
        28.0,   # omega: pitch rate damping
        180.0,  # l:    active suspension height tracking
        12.0,   # dl:   suspension vertical velocity damping
        90.0,   # psi:  ZMP active roll banking tracking
        10.0    # dpsi: roll rate damping
    ])

    R_weight = np.array([
        0.005,  # tau_w:    wheel torque penalization
        0.001,  # F_l:      leg thrust penalization (centered at M*g)
        0.005   # tau_roll: roll lean torque penalization
    ])

    ocp.cost.W = np.diag(np.concatenate([Q, R_weight]))
    ocp.cost.W_e = np.diag(Q * 2.5)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(ny_e)

    # Nominal reference: upright standing at nominal height l = 0.148m, thrust = M*g (~19.06 N)
    M_nominal = 1.943
    g_nominal = 9.81
    F_l_nominal = M_nominal * g_nominal

    yref_nominal = np.zeros(ny)
    yref_nominal[4] = 0.148        # l_ref = 0.148m
    yref_nominal[9] = F_l_nominal  # F_l_ref = M*g
    ocp.cost.yref = yref_nominal

    yref_e_nominal = np.zeros(ny_e)
    yref_e_nominal[4] = 0.148
    ocp.cost.yref_e = yref_e_nominal

    # Actuator limits (symmetric wheel torque +/-15 Nm, leg thrust 0..100 N, roll torque +/-15 Nm)
    ocp.constraints.lbu = np.array([-15.0, 0.0, -15.0])
    ocp.constraints.ubu = np.array([15.0, 100.0, 15.0])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.148, 0.0, 0.0, 0.0])

    # Initial parameter value: zero yaw rate
    ocp.parameter_values = np.array([0.0])

    # High-Performance Real-Time Iteration (SQP-RTI) settings
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'

    # Export C solver code to c_generated_code/
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp_balancer.json')
    print("[acados] Code generation complete in c_generated_code/")
    return solver


if __name__ == "__main__":
    generate_solver()
