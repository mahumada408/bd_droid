// Parameter block shared between the C++ whole-body controller and its Python
// (ctypes) front-end in controllers/wbc/wbc_controller.py. Plain C so it can be
// filled from Python, from a generated initializer (wbc_default_params.h) or
// from firmware.
//
// Layout contract: the field order here MUST match WbcParams._fields_ in Python.
// wbc_params_size() is checked against ctypes.sizeof() at load time.
#pragma once

#define WBC_MAX_ACT 16
#define WBC_MAX_LEG 12
#define WBC_MAX_GRID 64

#ifdef __cplusplus
extern "C" {
#endif

typedef struct WbcParams {
  // ---- Indices (computed in Python from the MuJoCo model)
  int nu;                          // actuators
  int n_leg;                       // non-wheel actuators
  int n_grid;                      // posture table rows
  int pad0;
  int act_dof[WBC_MAX_ACT];        // qvel index per actuator
  int act_qpos[WBC_MAX_ACT];       // qpos index per actuator
  int leg_idx[WBC_MAX_ACT];        // actuator index per leg joint
  int is_roll[WBC_MAX_ACT];        // per leg joint: 1 if hip roll
  int wheel_body[2];               // right, left (body index in robot_model.h)
  int wheel_act[2];                // actuator index per wheel body

  // ---- Model constants
  double tau_min[WBC_MAX_ACT];
  double tau_max[WBC_MAX_ACT];
  double fric[WBC_MAX_ACT];        // wheel frictionloss feed-forward (0 on legs)
  double q_lo[WBC_MAX_ACT];        // leg joint limits incl. margin (leg order)
  double q_hi[WBC_MAX_ACT];
  double dh_grid[WBC_MAX_GRID];
  double q_grid[WBC_MAX_GRID * WBC_MAX_ACT];   // row-major [n_grid][nu], actuator order
  double K[4];                     // LQR gain on [s, v, phi, phid]
  double h_rel_nom;
  double y_lat_nom;
  double total_mass;
  double mu;
  double Fz_max;
  double wheel_radius;
  double fric_vel;

  // ---- Gains / weights (names match the Python class attributes)
  double a_max, s_err_max, a_ff_max, use_ff;
  double kp_pitch, kd_pitch, kp_roll, kd_roll, kp_yaw;
  double kp_heading, heading_err_max;
  double kp_h, kd_h, kp_lat, kd_lat, kp_post, kd_post;
  double max_bank, lim_T, bank_gain, a_lat_max, contact_hold, kd_wheel_air;
  double w_contact, w_axle, w_pitch, w_roll, w_yaw, w_height, w_lat;
  double w_post, w_post_roll, w_tau, w_qdd, w_force, w_ftan, kv_slip;
  double obs_K, fz_on, fz_off;
  double ki_lean, lean_off_max, lean_learn_vmax;
  double dist_comp, dist_T, dist_max;
  double fall_detect, fall_lean, fall_roll, fall_tilt, fall_T, fall_qp_T;
  double fall_kp, fall_kd, fall_kd_wheel, fall_tau_wheel, fall_recover_T;
  double qp_warm_start, qp_iter_max, qp_mu0;   // HPIPM: 0 cold / 1 primal / 2 primal+dual
  double sit_enable, sit_kp, sit_kd;   // sit: delta_h below the posture table -> legs to its lowest row, wheels braked
  // Lean-led turning (see the "Bank" block in wbc_core.cpp)
  double lean_rate;                // rad/s, slew limit on the bank reference
  double lean_band;                // m/s^2, |g tan(lean) - a_lat| allowed: keeps the ZMP inside the track
  double lean_lead;                // 1: turn rate limited by the lean actually reached (lean leads the turn)
  double lean_T;                   // s, low-pass on the measured lateral acceleration
  double half_track;               // m, half the hub-to-hub distance
  double lean_room;                // m, extension kept in reserve on the outer leg while banked
  double lean_crouch;              // m of extra crouch per rad of bank (style)
  double neck_look, neck_look_max; // head yaw into the turn: rad per rad/s of yaw rate, clamp
  double neck_level;               // fraction of the base roll the neck roll cancels (0: head rolls with the body)
  double wheel_crown;              // m, tyre half width of a crowned (ellipsoid) tyre; 0: thin disk
  double leg_fric_comp, leg_fric_T; // leg dry-friction feed-forward: fraction, look-ahead on the QP's qdd (s)
  double fz_min;                   // N, least normal load per wheel the QP keeps while both are down
  double fric_leg[WBC_MAX_ACT];    // leg joint dry friction per actuator (0 on wheels), Nm
  int leg_side[WBC_MAX_ACT];       // per leg joint: +1 right leg, -1 left leg, 0 other (neck)
  int neck_k[2];                   // leg-joint index of neck yaw, neck roll (-1: none)
} WbcParams;

// Layout of the info vector filled by wbc_step()
enum {
  WBC_INFO_STATUS = 0,       // >=0 HPIPM status (0 ok), <0 failed / fallback
  WBC_INFO_SOLVE_US,         // filled by the host wrapper (the core does no timing)
  WBC_INFO_PHI,
  WBC_INFO_PSI_REF,
  WBC_INFO_A_AXLE,
  WBC_INFO_PITCH,            // rad
  WBC_INFO_ROLL,             // rad
  WBC_INFO_CONTACTS,
  WBC_INFO_HEADING_ERR,      // rad
  WBC_INFO_QP_ITER,
  WBC_INFO_FN_R,             // estimated normal load, N
  WBC_INFO_FN_L,
  WBC_INFO_PHI_OFF,          // learned lean offset, rad
  WBC_INFO_DIST_R,           // estimated unmodeled wheel torque, Nm
  WBC_INFO_DIST_L,
  WBC_INFO_SAFE_FALL,        // 0 balancing, else latched safe-fall cause (1 lean, 2 roll, 3 tilt, 4 QP)
  WBC_INFO_A_LAT,            // measured lateral acceleration v*omega (low-passed), m/s^2, + = turning left
  WBC_INFO_DH_EFF,           // height offset actually used (command, lowered while banked), m
  WBC_INFO_V_FWD,            // axle forward speed, m/s
  WBC_INFO_FC,               // 6 values
  WBC_INFO_QDD = WBC_INFO_FC + 6,   // nv values
};

#ifdef __cplusplus
}
#endif
