// LQR-assisted whole-body QP controller: portable core.
//
// Input is the robot state (qpos, qvel in MuJoCo layout) and the operator
// command; output is joint torques. No MuJoCo, no heap allocation, no OS calls:
// the same code runs in the simulator (through wbc_capi.cpp) and on a
// microcontroller. Dynamics come from rbd.h, the QP is solved with HPIPM's
// dense interior-point solver (warm-started from the previous tick).
//
// Memory: one WbcCore holds everything, including the HPIPM workspaces (an
// arena of WBC_ARENA_BYTES; the three HPIPM instances need ~174 kB on x86).
// It is large (~230 kB in total), so make it a static or
// heap object, not a stack variable.
#pragma once

#include <cstddef>

extern "C" {
#include "hpipm_d_dense_qp.h"
#include "hpipm_d_dense_qp_dim.h"
#include "hpipm_d_dense_qp_ipm.h"
#include "hpipm_d_dense_qp_sol.h"
}

#include "rbd.h"
#include "robot_model_select.h"
#include "wbc_params.h"

#ifndef WBC_ARENA_BYTES
#define WBC_ARENA_BYTES (192 * 1024)
#endif

namespace wbc {

enum WbcInitError {
  WBC_OK = 0,
  WBC_ERR_SIZES = -1,      // params do not match robot_model.h / compile-time limits
  WBC_ERR_ARENA = -2,      // WBC_ARENA_BYTES too small for the HPIPM workspaces
};

class WbcCore {
 public:
  static constexpr int NV = model::NV;
  static constexpr int NQ = model::NQ;
  static constexpr int NU = model::NU;
  static constexpr int NC = 6;             // 2 wheels x 3D contact force
  static constexpr int NX = NV + NC;       // QP variables [qdd, F]
  static constexpr int NTASK = 12;
  static constexpr int NG = NU + 8;        // torque rows + 8 friction rows
  static constexpr int NE_MAX = 6 + 8;     // dynamics + 4 per airborne wheel
  static constexpr int NB_MAX = WBC_MAX_LEG + 2;
  static constexpr int INFO_SIZE = WBC_INFO_QDD + NV;

  // Returns WBC_OK or a WbcInitError. Must be called before step().
  int init(const WbcParams& params);
  void reset();
  // One control tick. tau_out: NU torques. info (optional): INFO_SIZE values,
  // layout in wbc_params.h. Returns 0 if tau_out is a fresh command (QP
  // solved, or the safe-fall law), -1 if the QP failed and the previous
  // command was repeated.
  int step(const double* qpos, const double* qvel, double v_target, double yaw_target, double delta_h,
           double roll_target_offset, double dt, double* tau_out, double* info);

  std::size_t arena_used() const { return arena_used_; }
  const WbcParams& params() const { return p; }

 private:
  struct Qp {
    d_dense_qp_dim dim;
    d_dense_qp qp;
    d_dense_qp_sol sol;
    d_dense_qp_ipm_arg arg;
    d_dense_qp_ipm_ws ws;
    void* ws_mem;
    void* sol_mem;
    hpipm_size_t ws_size, sol_size;
    void reset_ws();
  };

  void* take(std::size_t n);
  int create_qp(Qp& Q, int ne);
  void point_jac(int body, const double* pt, double* J, double* jdqv) const;

  WbcParams p{};
  int n_leg = 0, nb = 0;
  bool ready = false;
  Rbd rbd;

  // ---- per-tick state
  double s = 0, s_ref = 0, v_ref_prev = 0, a_ref = 0;
  bool have_v_prev = false;
  double heading_ref = 0;
  bool have_heading = false;
  double last_tau[NU] = {0};
  bool have_obs = false;
  double p_int[6] = {0}, r_obs[6] = {0};
  double M_prev[NV * NV] = {0};
  bool contact_state[2] = {true, true};
  double t_low[2] = {0, 0};
  double Fn_est[2] = {0, 0};
  double phi_off = 0;
  double pw_int[2] = {0, 0}, tau_dist[2] = {0, 0};
  int n_fail = 0;
  int fall_mode = 0;
  double t_fall_cond = 0, t_upright = 0;
  double x_prev[NX] = {0};
  bool have_x_prev = false;

  // ---- work buffers
  double M[NV * NV], Jhub[2][3 * NV], Jc[NC * NV], Jk[NC * NV], Jw[3 * NV];
  double JT[NTASK * NV], P[NX * NX], q[NX], Atau[NU * NX], btau[NU], x[NX];
  double H_cm[NX * NX], A_eq_cm[NE_MAX * NX], C_cm[NG * NX], b_eq[NE_MAX];
  double lb[NB_MAX], ub[NB_MAX], lg[NG], ug[NG];
  int idxb[NB_MAX];

  // ---- HPIPM: one instance per number of airborne wheels (0, 1, 2)
  Qp qp_[3];
  alignas(64) unsigned char arena_[WBC_ARENA_BYTES];
  std::size_t arena_used_ = 0;
};

}  // namespace wbc
