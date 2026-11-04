#!/usr/bin/env python3
"""
Symbolic Equations of Motion & acados SQP-RTI C-Code Generator
for the Wheeled Biped / Acrobat Balancer Robot (CAD/robot.xml).

States (nx = 8):
  [p, v, theta, omega, qh, dqh, qk, dqk]
    - p: Forward displacement along Y-axis (m)
    - v: Forward velocity (m/s)
    - theta: Torso pitch angle from vertical (rad)
    - omega: Torso pitch angular rate (rad/s)
    - qh: Symmetrical hip joint angle from nominal stance (rad)
    - dqh: Hip joint angular rate (rad/s)
    - qk: Symmetrical knee joint angle from nominal stance (rad)
    - dqk: Knee joint angular rate (rad/s)

Controls (nu = 3):
  [tau_w, tau_h, tau_k]
    - tau_w: Wheel motor torque (Nm)
    - tau_h: Hip motor torque (Nm)
    - tau_k: Knee motor torque (Nm)
"""

import os
import sys
import numpy as np
import casadi as cs
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


def export_robot_model() -> AcadosModel:
    model_name = "biped_balancer"

    # States
    p = cs.SX.sym('p')
    v = cs.SX.sym('v')
    theta = cs.SX.sym('theta')
    omega = cs.SX.sym('omega')
    qh = cs.SX.sym('qh')
    dqh = cs.SX.sym('dqh')
    qk = cs.SX.sym('qk')
    dqk = cs.SX.sym('dqk')
    x = cs.vertcat(p, v, theta, omega, qh, dqh, qk, dqk)

    # Controls
    tau_w = cs.SX.sym('tau_w')
    tau_h = cs.SX.sym('tau_h')
    tau_k = cs.SX.sym('tau_k')
    u = cs.vertcat(tau_w, tau_h, tau_k)

    # Robot Physical Parameters (derived from CAD/robot.xml)
    # Total mass: ~2.12 kg (Torso: 1.30 kg, Femurs: 0.38 kg, Tibias: 0.38 kg, Wheels: 0.056 kg)
    M = 1.68       # Torso + upper legs effective mass (kg)
    m = 0.056      # Wheels mass (kg)
    R = 0.04       # Wheel radius (m)
    I_w = 0.5 * m * (R**2)
    l = 0.22       # COM distance from wheel axle (m)
    I_y = 0.015    # Pitch moment of inertia (kg m^2)
    g = 9.81

    # Leg joint parameters
    I_h = 0.005    # Hip inertia (kg m^2)
    I_k = 0.005    # Knee inertia (kg m^2)
    b_h = 0.05     # Hip joint damping (Nms/rad)
    b_k = 0.05     # Knee joint damping (Nms/rad)

    # Coupled Balancer Dynamics
    M_tot = M + m + (I_w / (R**2))
    det_M = M_tot * (I_y + M * (l**2)) - (M * l * cs.cos(theta))**2

    # Forward force from wheel torque + centripetal terms
    f_v = (tau_w / R) + M * l * (omega**2) * cs.sin(theta)
    # Pitch moment: gravity destabilizes, tau_w creates reaction moment -tau_w
    f_th = M * g * l * cs.sin(theta) - tau_w - tau_h

    v_dot = ((I_y + M * (l**2)) * f_v - (M * l * cs.cos(theta)) * f_th) / det_M
    omega_dot = (-(M * l * cs.cos(theta)) * f_v + M_tot * f_th) / det_M

    # Hip & Knee joint dynamics
    qh_ddot = (tau_h - b_h * dqh - 0.1 * g * cs.sin(qh)) / I_h
    qk_ddot = (tau_k - b_k * dqk - 0.1 * g * cs.sin(qk)) / I_k

    f_expl = cs.vertcat(v, v_dot, omega, omega_dot, dqh, qh_ddot, dqk, qk_ddot)

    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.u = u
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

    Tf = 0.5
    N = 25
    ocp.dims.N = N
    ocp.solver_options.tf = Tf

    nx = 8
    nu = 3
    ny = nx + nu
    ny_e = nx

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    # Weight matrices: Q for state error, R for control effort
    # High pitch damping ensures stable balance; hip/knee weights maintain stance
    Q = np.array([
        20.0,  5.0,     # p, v
        150.0, 25.0,    # theta, omega
        250.0, 15.0,    # qh, dqh
        250.0, 15.0     # qk, dqk
    ])
    R_weight = np.array([
        0.01,  # tau_w
        0.005, # tau_h
        0.005  # tau_k
    ])

    ocp.cost.W = np.diag(np.concatenate([Q, R_weight]))
    ocp.cost.W_e = np.diag(Q * 2.5)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(ny_e)

    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # Actuator limits
    ocp.constraints.lbu = np.array([-15.0, -15.0, -15.0])
    ocp.constraints.ubu = np.array([15.0, 15.0, 15.0])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.x0 = np.zeros(nx)

    # Real-Time Iteration (SQP-RTI) solver settings
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
