"""
Hierarchical Whole-Body Quadratic Programming (WBC QP) Controller
for the 8-DOF Wheeled Biped Balancer Robot.

Inspired by:
"LQR-Assisted Whole-Body Control of a Wheeled Bipedal Robot with Kinematic Loops"
(Klemm, Morra, Gulich, Mannhart, Rohr, Kamel, de Viragh, Siegwart - ETH Zurich / ASL, IEEE RAL 2020)

Optimization Variables (nx = 28):
  x = [qddot (14), F_c (6), tau (8)]
    - qddot: Generalized accelerations (6 floating base + 8 actuated joints)
    - F_c: 3D Contact forces at [Right_Wheel, Left_Wheel] in world frame
    - tau: Actuation torques [hip_roll_R, hip_pitch_R, knee_R, wheel_R,
                              hip_roll_L, hip_pitch_L, knee_L, wheel_L]

Physics & Constraints:
  1. Multibody Dynamics: M(q) qddot - J_c^T F_c - S^T tau = -h(q, qdot)
  2. Full 3D Contact Kinematics:
     - X: Lateral no-slip
     - Y: Longitudinal rolling contact (J_c,y qddot = -kd v_c,y)
     - Z: Vertical terrain contact damping
  3. Coulomb Friction Pyramids: |F_x| <= mu * F_z, |F_y| <= mu * F_z
  4. Unilateral Contact & Gravity Support: F_z >= F_z_min > 0, centered on mg/2
  5. Actuator Saturation: tau_min <= tau <= tau_max
"""

import time
import numpy as np
import scipy.sparse as sp
import mujoco
import osqp


class WholeBodyController:
    def __init__(self, model: mujoco.MjModel, mu: float = 0.85):
        self.m = model
        self.nv = model.nv   # 14
        self.nu = model.nu   # 8
        self.nc = 6          # 2 wheels * 3D contact forces
        self.nx = self.nv + self.nc + self.nu # 28

        self.mu = mu
        self.Fz_min = 1.0    # Minimum contact force (N)
        self.Fz_max = 60.0   # Maximum contact force (N)
        self.wheel_radius = 0.05

        # Actuation selection matrix S (8 x 14)
        self.S = np.zeros((self.nu, self.nv))
        self.S[:, 6:14] = np.eye(self.nu)

        # Actuator torque limits from model (Nm)
        self.tau_max = np.array([15.0, 15.0, 15.0, 5.0, 15.0, 15.0, 15.0, 5.0])
        self.tau_min = -self.tau_max

        # Calibrated Nominal Stance for Rev 3
        self.qh_R_nom = 0.314503
        self.qk_R_nom = -0.052919
        self.qh_L_nom = -0.212551
        self.qk_L_nom = 0.253279

        # Wheel body IDs
        self.r_wheel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wheel")
        self.l_wheel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wheel_2")

        # Nominal relative base height above wheel axle midpoint
        self.h_rel_nom = -0.0128
        self.total_mass = float(np.sum(model.body_mass))
        self.Fz_gravity_per_wheel = 0.5 * self.total_mass * 9.81

        # Control Gains
        self.kp_pitch = 140.0
        self.kd_pitch = 18.0
        self.kp_v = 0.35
        self.ki_v = 0.08
        self.max_tilt = 0.20

        self.kp_h = 120.0
        self.kd_h = 18.0

        self.kp_roll = 120.0
        self.kd_roll = 18.0

        self.kd_yaw = 25.0

        # Integral & hold state
        self.v_int = 0.0
        self.y_hold = None
        self.last_tau = np.zeros(self.nu)

    def reset(self):
        self.v_int = 0.0
        self.y_hold = None
        self.last_tau = np.zeros(self.nu)

    def step(self, d: mujoco.MjData, v_target: float, yaw_target: float,
             delta_h: float = 0.0, roll_target_offset: float = 0.0,
             dt: float = 0.002) -> tuple:
        """
        Solves one iteration of the Whole-Body QP and returns optimal torques.
        Args:
            d: MuJoCo data instance
            v_target: Forward ground velocity target (m/s)
            yaw_target: Heading yaw angular rate target (rad/s)
            delta_h: Relative base height adjustment (m) (e.g. -0.03 for crouch, +0.02 for stand)
            roll_target_offset: Roll bank offset (rad) for terrain compensation
            dt: Simulation timestep (s)
        Returns:
            tau_cmd (np.ndarray): 8 motor torques
            info (dict): Solve metrics
        """
        t0 = time.time()

        # 1. 3D Invariant Robot Kinematics
        R = d.xmat[1].reshape(3, 3)
        fwd = R[:, 1]
        theta = float(np.arctan2(R[2, 1], R[2, 2]))
        omega_pitch = float(d.qvel[3])
        omega_yaw = float(d.qvel[5])
        v_fwd = float(np.dot(d.qvel[0:3], fwd))
        roll = float(np.arctan2(R[0, 2], R[2, 2]))
        roll_rate = float(d.qvel[4])

        # Stationary position holding lock
        if abs(v_target) < 0.05 and abs(v_fwd) < 0.15:
            if self.y_hold is None:
                self.y_hold = float(d.qpos[1])
            v_target_adj = float(np.clip(- 1.5 * (d.qpos[1] - self.y_hold), -0.25, 0.25))
        else:
            self.y_hold = None
            v_target_adj = v_target

        # Velocity anti-windup integration
        v_err = v_fwd - v_target_adj
        self.v_int += v_err * dt
        self.v_int = float(np.clip(self.v_int, -0.30, 0.30))

        # 2. Ascento LQR Balancing Motion Task (Klemm et al. 2020 Eq 36-37)
        th_ref = self.kp_v * v_err + self.ki_v * self.v_int
        th_ref = float(np.clip(th_ref, -self.max_tilt, self.max_tilt))
        a_pitch_des = - (self.kp_pitch * (theta - th_ref) + self.kd_pitch * omega_pitch)

        # 3. Base Height Task relative to wheel midpoint (Ascento Eq 33)
        pt_R = d.xpos[self.r_wheel_id].copy(); pt_R[2] -= self.wheel_radius
        pt_L = d.xpos[self.l_wheel_id].copy(); pt_L[2] -= self.wheel_radius
        mid_w_z = 0.5 * (pt_R[2] + pt_L[2]) + self.wheel_radius
        h_rel = d.qpos[2] - mid_w_z
        target_h_rel = self.h_rel_nom + delta_h
        a_h_des = - (self.kp_h * (h_rel - target_h_rel) + self.kd_h * d.qvel[2])

        # 4. Base Roll & ZMP Curve Banking Task (Ascento Sec III-B.3)
        a_lat = v_fwd * omega_yaw
        psi_zmp = float(np.arctan2(a_lat, 9.81))
        # Curve bank is active only when driving forward; stationary turns stay upright
        k_bank = float(np.clip(abs(v_fwd) / 0.50, 0.0, 1.0))
        psi_ref = float(np.clip(k_bank * (0.35 * yaw_target + 0.50 * psi_zmp) + roll_target_offset, -0.35, 0.35))
        a_roll_des = - (self.kp_roll * (roll - psi_ref) + self.kd_roll * roll_rate)

        # 5. Base Yaw Steering Task (Ascento Sec III-B.5)
        a_yaw_des = - self.kd_yaw * (omega_yaw - yaw_target)

        # 6. Extract Analytical Multibody Dynamics from MuJoCo
        M = np.zeros((self.nv, self.nv))
        mujoco.mj_fullM(self.m, d, M)
        h = d.qfrc_bias.copy()

        jac_R = np.zeros((3, self.nv))
        jac_L = np.zeros((3, self.nv))
        mujoco.mj_jac(self.m, d, jac_R, None, pt_R, self.r_wheel_id)
        mujoco.mj_jac(self.m, d, jac_L, None, pt_L, self.l_wheel_id)
        Jc = np.vstack([jac_R, jac_L]) # 6 x 14

        # Contact velocities in world frame
        v_c_R = jac_R @ d.qvel
        v_c_L = jac_L @ d.qvel

        # 7. Rigid-Body Dynamics Equality Constraint
        # M(q) qddot - J_c^T F_c - S^T tau = -h(q, qdot)
        A_dyn = np.hstack([M, -Jc.T, -self.S.T])
        b_dyn = -h

        # 8. Friction Pyramids & Bounds: |Fx| <= mu*Fz, |Fy| <= mu*Fz
        A_fric = np.zeros((8, self.nx))
        A_fric[0, self.nv+0] = +1.0; A_fric[0, self.nv+2] = -self.mu
        A_fric[1, self.nv+0] = -1.0; A_fric[1, self.nv+2] = -self.mu
        A_fric[2, self.nv+1] = +1.0; A_fric[2, self.nv+2] = -self.mu
        A_fric[3, self.nv+1] = -1.0; A_fric[3, self.nv+2] = -self.mu

        A_fric[4, self.nv+3] = +1.0; A_fric[4, self.nv+5] = -self.mu
        A_fric[5, self.nv+3] = -1.0; A_fric[5, self.nv+5] = -self.mu
        A_fric[6, self.nv+4] = +1.0; A_fric[6, self.nv+5] = -self.mu
        A_fric[7, self.nv+4] = -1.0; A_fric[7, self.nv+5] = -self.mu

        # Normal contact force bounds
        A_fz = np.zeros((2, self.nx))
        A_fz[0, self.nv+2] = 1.0 # F_z,R
        A_fz[1, self.nv+5] = 1.0 # F_z,L

        # Actuator torque limits
        A_tau = np.zeros((self.nu, self.nx))
        A_tau[:, self.nv+self.nc:] = np.eye(self.nu)

        A_all = np.vstack([A_dyn, A_fric, A_fz, A_tau])
        l_all = np.concatenate([b_dyn, -np.inf * np.ones(8), [self.Fz_min, self.Fz_min], self.tau_min])
        u_all = np.concatenate([b_dyn, np.zeros(8), [self.Fz_max, self.Fz_max], self.tau_max])

        # 9. Quadratic Cost Function
        P = np.zeros((self.nx, self.nx))
        q_cost = np.zeros(self.nx)

        # 3D Contact Kinematics (No-slip X, rolling contact Y, terrain contact Z)
        w_cont = 150.0
        kd_cont = 15.0
        a_cont_des_R = - kd_cont * v_c_R
        a_cont_des_L = - kd_cont * v_c_L
        P[:self.nv, :self.nv] += w_cont * (jac_R.T @ jac_R + jac_L.T @ jac_L)
        q_cost[:self.nv] -= w_cont * (jac_R.T @ a_cont_des_R + jac_L.T @ a_cont_des_L)

        # Gravity support force prior: maintain positive support force ~mg/2
        w_fz = 5.0
        P[self.nv+2, self.nv+2] += w_fz; q_cost[self.nv+2] -= w_fz * self.Fz_gravity_per_wheel
        P[self.nv+5, self.nv+5] += w_fz; q_cost[self.nv+5] -= w_fz * self.Fz_gravity_per_wheel

        # Relative Base Height Task
        w_height = 60.0
        P[2, 2] += w_height
        q_cost[2] -= w_height * a_h_des

        # Balancing Motion Task (Pitch)
        w_pitch = 140.0
        P[3, 3] += w_pitch
        q_cost[3] -= w_pitch * a_pitch_des

        # Base Roll Task
        w_roll = 110.0
        P[4, 4] += w_roll
        q_cost[4] -= w_roll * a_roll_des

        # Base Yaw Steering Task
        w_yaw = 40.0
        P[5, 5] += w_yaw
        q_cost[5] -= w_yaw * a_yaw_des

        # Normal Force Equalization on rough / wavy terrain
        w_fb = 2.0
        P[self.nv+2, self.nv+2] += w_fb; P[self.nv+5, self.nv+5] += w_fb
        P[self.nv+2, self.nv+5] -= w_fb; P[self.nv+5, self.nv+2] -= w_fb

        # Stiff Leg Joint Stance Posture Regularization:
        # Hip roll joints (indices 6 and 10) must be held tightly near 0 to avoid hitting 0.6 rad limits!
        w_post_roll = 35.0
        P[6, 6] += w_post_roll; q_cost[6] -= w_post_roll * (- 45.0 * d.qpos[7] - 8.0 * d.qvel[6])
        P[10, 10] += w_post_roll; q_cost[10] -= w_post_roll * (- 45.0 * d.qpos[11] - 8.0 * d.qvel[10])

        w_post_leg = 25.0
        qh_R_post = self.qh_R_nom + 0.40 * th_ref
        qh_L_post = self.qh_L_nom - 0.40 * th_ref
        P[7, 7] += w_post_leg; q_cost[7] -= w_post_leg * (- 45.0 * (d.qpos[8] - qh_R_post) - 8.0 * d.qvel[7])
        P[8, 8] += w_post_leg; q_cost[8] -= w_post_leg * (- 45.0 * (d.qpos[9] - self.qk_R_nom) - 8.0 * d.qvel[8])
        P[11, 11] += w_post_leg; q_cost[11] -= w_post_leg * (- 45.0 * (d.qpos[12] - qh_L_post) - 8.0 * d.qvel[11])
        P[12, 12] += w_post_leg; q_cost[12] -= w_post_leg * (- 45.0 * (d.qpos[13] - self.qk_L_nom) - 8.0 * d.qvel[12])

        # Actuation Torque Minimization
        P[self.nv+self.nc:, self.nv+self.nc:] += 1e-3 * np.eye(self.nu)

        # 10. Solve with OSQP
        solver = osqp.OSQP()
        solver.setup(P=sp.csc_matrix(P), q=q_cost, A=sp.csc_matrix(A_all), l=l_all, u=u_all,
                     verbose=False, polish=False, eps_abs=1e-3, eps_rel=1e-3, max_iter=400)
        results = solver.solve()

        solve_time_us = (time.time() - t0) * 1e6

        if results.info.status_val in [1, 2]: # optimal or optimal_inaccurate
            x_opt = results.x
            tau_cmd = x_opt[self.nv+self.nc:]
            Fc = x_opt[self.nv:self.nv+self.nc]
            self.last_tau = tau_cmd.copy()
        else:
            # Fallback to last valid command rather than zeroing torques
            tau_cmd = self.last_tau.copy()
            Fc = np.zeros(self.nc)

        info = {
            "status": results.info.status,
            "solve_time_us": solve_time_us,
            "th_ref": th_ref,
            "psi_ref": psi_ref,
            "Fc": Fc,
            "pitch_deg": np.degrees(theta),
            "roll_deg": np.degrees(roll),
        }
        return tau_cmd, info
