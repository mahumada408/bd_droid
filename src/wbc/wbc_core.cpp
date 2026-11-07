// LQR-assisted whole-body QP controller: portable core. See wbc_core.h and the
// formulation in controllers/wbc/wbc_controller.py (and wbc_report.html).

#include "wbc_core.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace wbc {
namespace {

constexpr double G = 9.81;
constexpr double PI = 3.14159265358979323846;
constexpr double BIG = 1e4;     // "unbounded" for rows that are switched off

inline double clampd(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }
inline double dot3(const double* a, const double* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline void cross3(const double* a, const double* b, double* o) {
  o[0] = a[1] * b[2] - a[2] * b[1];
  o[1] = a[2] * b[0] - a[0] * b[2];
  o[2] = a[0] * b[1] - a[1] * b[0];
}
inline double wrap_pi(double a) { return std::remainder(a, 2.0 * PI); }
inline void matvec3(const double* J, const double* v, double* o, int nv) {
  for (int i = 0; i < 3; ++i) {
    double acc = 0;
    for (int j = 0; j < nv; ++j) acc += J[i * nv + j] * v[j];
    o[i] = acc;
  }
}

}  // namespace

// ---------------------------------------------------------------- HPIPM setup
void* WbcCore::take(std::size_t n) {
  std::size_t off = (arena_used_ + 63) & ~std::size_t(63);
  if (off + n > sizeof(arena_)) return nullptr;
  arena_used_ = off + n;
  return arena_ + off;
}

void WbcCore::Qp::reset_ws() {
  // A NaN solve can leave NaNs in the workspace / solution that poison every
  // later solve (HPIPM then fails after one iteration forever). Rebuild both.
  std::memset(ws_mem, 0, ws_size);
  d_dense_qp_ipm_ws_create(&dim, &arg, &ws, ws_mem);
  std::memset(sol_mem, 0, sol_size);
  d_dense_qp_sol_create(&dim, &sol, sol_mem);
}

int WbcCore::create_qp(Qp& Q, int ne) {
  void* m_dim = take(d_dense_qp_dim_memsize());
  if (!m_dim) return WBC_ERR_ARENA;
  d_dense_qp_dim_create(&Q.dim, m_dim);
  d_dense_qp_dim_set_all(NX, ne, nb, NG, 0, 0, &Q.dim);
  void* m_qp = take(d_dense_qp_memsize(&Q.dim));
  Q.sol_size = d_dense_qp_sol_memsize(&Q.dim);
  Q.sol_mem = take(Q.sol_size);
  void* m_arg = take(d_dense_qp_ipm_arg_memsize(&Q.dim));
  if (!m_qp || !Q.sol_mem || !m_arg) return WBC_ERR_ARENA;
  d_dense_qp_create(&Q.dim, &Q.qp, m_qp);
  d_dense_qp_sol_create(&Q.dim, &Q.sol, Q.sol_mem);
  d_dense_qp_ipm_arg_create(&Q.dim, &Q.arg, m_arg);
  d_dense_qp_ipm_arg_set_default(BALANCE, &Q.arg);
  int iter_max = int(p.qp_iter_max);
  double tol = 1e-7;
  double mu0 = p.qp_mu0;
  int warm = int(p.qp_warm_start);
  d_dense_qp_ipm_arg_set_iter_max(&iter_max, &Q.arg);
  d_dense_qp_ipm_arg_set_tol_stat(&tol, &Q.arg);
  d_dense_qp_ipm_arg_set_tol_eq(&tol, &Q.arg);
  d_dense_qp_ipm_arg_set_tol_ineq(&tol, &Q.arg);
  d_dense_qp_ipm_arg_set_tol_comp(&tol, &Q.arg);
  d_dense_qp_ipm_arg_set_mu0(&mu0, &Q.arg);
  d_dense_qp_ipm_arg_set_warm_start(&warm, &Q.arg);
  Q.ws_size = d_dense_qp_ipm_ws_memsize(&Q.dim, &Q.arg);
  Q.ws_mem = take(Q.ws_size);
  if (!Q.ws_mem) return WBC_ERR_ARENA;
  std::memset(Q.ws_mem, 0, Q.ws_size);
  d_dense_qp_ipm_ws_create(&Q.dim, &Q.arg, &Q.ws, Q.ws_mem);
  return WBC_OK;
}

int WbcCore::init(const WbcParams& prm) {
  ready = false;
  p = prm;
  if (p.nu != NU || p.n_leg > WBC_MAX_LEG || p.n_leg != NU - 2 || p.n_grid < 2 || p.n_grid > WBC_MAX_GRID)
    return WBC_ERR_SIZES;
  for (int w = 0; w < 2; ++w)
    if (p.wheel_body[w] <= 0 || p.wheel_body[w] >= model::NBODY || model::JNT_TYPE[p.wheel_body[w]] != 1)
      return WBC_ERR_SIZES;
  for (int a = 0; a < NU; ++a)
    if (p.act_dof[a] < 6 || p.act_dof[a] >= NV || p.act_qpos[a] < 7 || p.act_qpos[a] >= NQ) return WBC_ERR_SIZES;
  n_leg = p.n_leg;
  nb = n_leg + 2;
  for (int k = 0; k < n_leg; ++k) idxb[k] = p.act_dof[p.leg_idx[k]];
  idxb[n_leg] = NV + 2;          // F_z right
  idxb[n_leg + 1] = NV + 5;      // F_z left

  // Static general-constraint rows: friction pyramids (rows NU .. NU+7).
  // HPIPM stores C column-major: C_cm[col * NG + row].
  std::memset(C_cm, 0, sizeof(C_cm));
  double lg_mask[NG];
  for (int r = 0; r < NG; ++r) lg_mask[r] = 1.0;
  for (int w = 0; w < 2; ++w) {
    const int fx = NV + 3 * w, fy = fx + 1, fz = fx + 2;
    const int ii[4] = {fx, fx, fy, fy};
    const double sg[4] = {1, -1, 1, -1};
    for (int k = 0; k < 4; ++k) {
      const int r = NU + 4 * w + k;
      C_cm[ii[k] * NG + r] = sg[k];
      C_cm[fz * NG + r] = -p.mu;
      lg_mask[r] = 0.0;         // one-sided: lower bound inactive
      lg[r] = -BIG;
    }
  }
  arena_used_ = 0;
  for (int na = 0; na < 3; ++na) {
    const int rc = create_qp(qp_[na], 6 + 4 * na);
    if (rc != WBC_OK) return rc;
    d_dense_qp_set_idxb(idxb, &qp_[na].qp);
    d_dense_qp_set_lg_mask(lg_mask, &qp_[na].qp);
  }
  ready = true;
  reset();
  return WBC_OK;
}

void WbcCore::reset() {
  s = s_ref = 0;
  have_v_prev = false;
  v_ref_prev = 0;
  a_ref = 0;
  have_heading = false;
  have_obs = false;
  contact_state[0] = contact_state[1] = true;
  t_low[0] = t_low[1] = 0;
  Fn_est[0] = Fn_est[1] = 0;
  phi_off = 0;
  pw_int[0] = pw_int[1] = 0;
  tau_dist[0] = tau_dist[1] = 0;
  n_fail = 0;
  fall_mode = 0;
  t_fall_cond = t_upright = 0;
  have_x_prev = false;
  for (double& t : last_tau) t = 0.0;
}

// J (3 x NV) and Jdot*qvel at a world point fixed to `body`
void WbcCore::point_jac(int body, const double* pt, double* J, double* jdqv) const {
  rbd.jac_point(body, pt, J);
  rbd.jacdot_qvel(body, pt, jdqv);
}

// ---------------------------------------------------------------- one tick
int WbcCore::step(const double* qpos, const double* qv, double v_target, double yaw_target, double delta_h,
                  double roll_off, double dt, double* tau_out, double* info) {
  if (!ready) return -2;
  rbd.forward(qpos, qv);

  // ---- Base state (heading frame)
  const double* R = rbd.xmat(1);
  double nf = std::hypot(R[1], R[4]);
  if (nf == 0) nf = 1e-6;
  const double fh[3] = {R[1] / nf, R[4] / nf, 0.0};
  const double rh[3] = {fh[1], -fh[0], 0.0};
  const double theta = std::atan2(R[7], R[8]);
  // Roll about the body's forward axis (heading-invariant).
  const double roll = std::atan2(-R[6], R[8]);
  const double yaw = std::atan2(fh[1], fh[0]);
  const double wx = qv[3], wy = qv[4], wz = qv[5];

  // ---- Wheel hubs / axle
  const int wid[2] = {p.wheel_body[0], p.wheel_body[1]};
  double Jd_hub[2][3];
  for (int w = 0; w < 2; ++w) point_jac(wid[w], rbd.xpos(wid[w]), Jhub[w], Jd_hub[w]);
  double J_ax[3 * NV], Jd_ax[3], axle[3], v_ax[3];
  for (int i = 0; i < 3 * NV; ++i) J_ax[i] = 0.5 * (Jhub[0][i] + Jhub[1][i]);
  for (int i = 0; i < 3; ++i) {
    Jd_ax[i] = 0.5 * (Jd_hub[0][i] + Jd_hub[1][i]);
    axle[i] = 0.5 * (rbd.xpos(wid[0])[i] + rbd.xpos(wid[1])[i]);
  }
  matvec3(J_ax, qv, v_ax, NV);
  const double v_fwd = dot3(fh, v_ax);

  // Turn-rate limiter: |v * omega| <= a_lat_max, using the larger of commanded
  // and actual speed so an overspeeding robot cannot turn harder than intended.
  const double w_cap = p.a_lat_max / std::max(std::max(std::fabs(v_target), std::fabs(v_fwd)), 1e-3);
  yaw_target = clampd(yaw_target, -w_cap, w_cap);

  // ---- Wheel contact geometry: flat ground (normal +z) under a thin disk.
  // The contact point is the rim point closest to the ground: the hub minus
  // r times the unit vector along the projection of +z onto the wheel plane.
  double nrm_w[2][3], bk[NC];
  std::memset(Jc, 0, sizeof(Jc));
  std::memset(Jk, 0, sizeof(Jk));
  for (int w = 0; w < 2; ++w) {
    const double* a = rbd.axis(wid[w]);
    const double* hub = rbd.xpos(wid[w]);
    const double nrm[3] = {0.0, 0.0, 1.0};
    double u[3] = {-a[2] * a[0], -a[2] * a[1], 1.0 - a[2] * a[2]};
    double un = std::sqrt(dot3(u, u));
    if (un < 1e-6) {          // wheel lying flat: no meaningful rim point
      u[0] = u[1] = 0.0;
      u[2] = 1.0;
      un = 1.0;
    }
    double pt[3];
    for (int k = 0; k < 3; ++k) pt[k] = hub[k] - p.wheel_radius * u[k] / un;
    std::copy(nrm, nrm + 3, nrm_w[w]);
    double Jdw[3], vc[3], vh[3];
    point_jac(wid[w], pt, Jw, Jdw);
    std::copy(Jw, Jw + 3 * NV, Jc + 3 * w * NV);
    double t1[3], t2[3];
    cross3(nrm, fh, t1);
    double n1 = std::sqrt(dot3(t1, t1));
    if (n1 == 0) n1 = 1e-6;
    for (int k = 0; k < 3; ++k) t1[k] /= n1;
    cross3(t1, nrm, t2);
    matvec3(Jw, qv, vc, NV);
    matvec3(Jhub[w], qv, vh, NV);
    const double* tm[2] = {t1, t2};
    for (int r = 0; r < 2; ++r) {
      double* row = &Jk[(3 * w + r) * NV];
      for (int j = 0; j < NV; ++j) row[j] = tm[r][0] * Jw[j] + tm[r][1] * Jw[NV + j] + tm[r][2] * Jw[2 * NV + j];
      bk[3 * w + r] = -(dot3(tm[r], Jdw) + p.kv_slip * dot3(tm[r], vc));
    }
    double* row = &Jk[(3 * w + 2) * NV];
    for (int j = 0; j < NV; ++j)
      row[j] = nrm[0] * Jhub[w][j] + nrm[1] * Jhub[w][NV + j] + nrm[2] * Jhub[w][2 * NV + j];
    bk[3 * w + 2] = -(dot3(nrm, Jd_hub[w]) + p.kv_slip * dot3(nrm, vh));
  }

  // ---- Wheel load estimate. Generalized-momentum observer on the 6
  // floating-base rows: r -> tau_ext,b = Jc_b^T F (no actuator torque or joint
  // friction there). The wheel-joint rows give the unmodeled wheel torque.
  rbd.mass_matrix(M);
  double h[NV];
  rbd.bias(h);
  {
    double pb[6], mdq[6];
    for (int i = 0; i < 6; ++i) {
      double a = 0, b = 0;
      for (int j = 0; j < NV; ++j) {
        a += M[i * NV + j] * qv[j];
        b += (M[i * NV + j] - M_prev[i * NV + j]) * qv[j];
      }
      pb[i] = a;
      mdq[i] = have_obs ? b / dt : 0.0;
    }
    double pw[2], mdqw[2];
    for (int w = 0; w < 2; ++w) {
      const int j = p.act_dof[p.wheel_act[w]];
      double a = 0, b = 0;
      for (int k = 0; k < NV; ++k) {
        a += M[j * NV + k] * qv[k];
        b += (M[j * NV + k] - M_prev[j * NV + k]) * qv[k];
      }
      pw[w] = a;
      mdqw[w] = have_obs ? b / dt : 0.0;
    }
    if (!have_obs) {                 // assume resting on the ground: tau_ext,b = h_b
      for (int i = 0; i < 6; ++i) {
        r_obs[i] = h[i];
        p_int[i] = pb[i] - r_obs[i] / p.obs_K;
      }
      for (int w = 0; w < 2; ++w) pw_int[w] = pw[w];
      have_obs = true;
    }
    for (int i = 0; i < 6; ++i) r_obs[i] = p.obs_K * (pb[i] - p_int[i]);
    for (int i = 0; i < 6; ++i) p_int[i] += dt * (r_obs[i] - h[i] + mdq[i]);
    double rw[2];
    for (int w = 0; w < 2; ++w) {
      const int a = p.wheel_act[w], j = p.act_dof[a];
      rw[w] = p.obs_K * (pw[w] - pw_int[w]);
      pw_int[w] += dt * (rw[w] - h[j] + mdqw[w] + last_tau[a]);
    }
    std::memcpy(M_prev, M, sizeof(M));
    // Solve (A^T A + lam I) F = A^T r with A[i][k] = Jc[k][i] (6x6)
    double N[36], rhs[6];
    for (int a = 0; a < 6; ++a) {
      rhs[a] = 0;
      for (int i = 0; i < 6; ++i) rhs[a] += Jc[a * NV + i] * r_obs[i];
      for (int b = 0; b < 6; ++b) {
        double acc = 0;
        for (int i = 0; i < 6; ++i) acc += Jc[a * NV + i] * Jc[b * NV + i];
        N[a * 6 + b] = acc + (a == b ? 1e-6 : 0.0);
      }
    }
    for (int c = 0; c < 6; ++c) {    // Gaussian elimination with partial pivoting
      int piv = c;
      for (int r = c + 1; r < 6; ++r)
        if (std::fabs(N[r * 6 + c]) > std::fabs(N[piv * 6 + c])) piv = r;
      if (piv != c) {
        for (int k = 0; k < 6; ++k) std::swap(N[c * 6 + k], N[piv * 6 + k]);
        std::swap(rhs[c], rhs[piv]);
      }
      for (int r = c + 1; r < 6; ++r) {
        const double f = N[r * 6 + c] / N[c * 6 + c];
        for (int k = c; k < 6; ++k) N[r * 6 + k] -= f * N[c * 6 + k];
        rhs[r] -= f * rhs[c];
      }
    }
    double F[6];
    for (int c = 5; c >= 0; --c) {
      double acc = rhs[c];
      for (int k = c + 1; k < 6; ++k) acc -= N[c * 6 + k] * F[k];
      F[c] = acc / N[c * 6 + c];
    }
    for (int w = 0; w < 2; ++w) Fn_est[w] = dot3(&F[3 * w], nrm_w[w]);
    // Unmodeled wheel torque = wheel-row residual - contact contribution
    // (Jc^T F) - the friction the feed-forward already assumes. Low-passed and
    // cancelled on the output.
    if (p.dist_comp != 0 && contact_state[0] && contact_state[1]) {
      for (int w = 0; w < 2; ++w) {
        const int a = p.wheel_act[w], j = p.act_dof[a];
        double jf = 0;
        for (int k = 0; k < 6; ++k) jf += Jc[k * NV + j] * F[k];
        const double e = rw[w] - jf + p.fric[a] * std::tanh(qv[j] / p.fric_vel);
        tau_dist[w] += std::min(1.0, dt / p.dist_T) * (e - tau_dist[w]);
        tau_dist[w] = clampd(tau_dist[w], -p.dist_max, p.dist_max);
      }
    }
  }
  bool in_contact[2];
  for (int w = 0; w < 2; ++w) {      // contact from load, with hysteresis
    if (contact_state[w]) {
      t_low[w] = Fn_est[w] < p.fz_off ? t_low[w] + dt : 0.0;
      if (t_low[w] > p.contact_hold) contact_state[w] = false;
    } else if (Fn_est[w] > p.fz_on) {
      contact_state[w] = true;
      t_low[w] = 0.0;
    }
    in_contact[w] = contact_state[w];
  }
  const int n_on = int(in_contact[0]) + int(in_contact[1]);

  // Airborne wheels: fixed torque that spins them to ground speed
  double tau_air[2] = {0, 0};
  for (int w = 0; w < 2; ++w) {
    if (in_contact[w]) continue;
    const int a = p.wheel_act[w], dof = p.act_dof[a];
    const double k_w = Jk[(3 * w + 1) * NV + dof];
    const double rate_ref = std::fabs(k_w) > 1e-4 ? -v_fwd / k_w : 0.0;
    tau_air[w] = clampd(-p.kd_wheel_air * (qv[dof] - rate_ref), p.tau_min[a], p.tau_max[a]);
  }

  // ---- LQR balance layer on the whole-body CoM lean over the axle
  const double* Jcom = rbd.com_jac();
  double rel[3], rel_v[3], vcom[3];
  matvec3(Jcom, qv, vcom, NV);
  for (int k = 0; k < 3; ++k) {
    rel[k] = rbd.com()[k] - axle[k];
    rel_v[k] = vcom[k] - v_ax[k];
  }
  const double rf = dot3(fh, rel), rz = rel[2];
  const double phi = std::atan2(rf, rz);
  const double phid = (dot3(fh, rel_v) * rz - rf * rel_v[2]) / std::max(rf * rf + rz * rz, 1e-6);

  if (n_on) {
    s += v_fwd * dt;
    s_ref += v_target * dt;
  } else {
    s_ref = s;
  }
  double e_s = s - s_ref;
  if (std::fabs(e_s) > p.s_err_max) {
    e_s = std::copysign(p.s_err_max, e_s);
    s_ref = s - e_s;
  }
  if (have_v_prev) {
    const double a_raw = (v_target - v_ref_prev) / dt;
    a_ref += std::min(1.0, 30.0 * dt) * (a_raw - a_ref);
  }
  v_ref_prev = v_target;
  have_v_prev = true;
  const double a_ff = p.use_ff != 0 ? clampd(a_ref, -p.a_ff_max, p.a_ff_max) : 0.0;
  const double* K = p.K;
  // Learned lean offset (PI on position through the lean channel), only near
  // standstill with both wheels down and the position clamp inactive.
  if (n_on == 2 && std::fabs(s - s_ref) < p.s_err_max && K[2] != 0.0 &&
      std::fabs(v_target) < p.lean_learn_vmax && std::fabs(yaw_target) < 0.1 && std::fabs(a_ref) < 0.1) {
    phi_off -= dt * p.ki_lean * (K[0] / K[2]) * e_s;
    phi_off = clampd(phi_off, -p.lean_off_max, p.lean_off_max);
  }
  const double u_fb = -(K[0] * e_s + K[1] * (v_fwd - v_target) +
                        K[2] * (phi - phi_off - std::atan(a_ff / G)) + K[3] * phid);
  const double a_axle = clampd(a_ff + u_fb, -p.a_max, p.a_max);

  // ---- Heading hold
  if (!have_heading || !n_on) {
    heading_ref = yaw;
    have_heading = true;
  } else {
    heading_ref = wrap_pi(heading_ref + yaw_target * dt);
  }
  double e_head = wrap_pi(yaw - heading_ref);
  if (std::fabs(e_head) > p.heading_err_max) {        // anti-windup: drag reference
    e_head = std::copysign(p.heading_err_max, e_head);
    heading_ref = wrap_pi(yaw - e_head);
  }
  const double yaw_rate_ref = yaw_target - p.kp_heading * e_head;

  // ---- Task references
  const double psi_bank = -p.bank_gain * std::atan2(v_target * yaw_target, G);
  const double psi_ref = clampd(psi_bank + roll_off, -p.max_bank, p.max_bank);
  const double dh_lo = p.dh_grid[0], dh_hi = p.dh_grid[p.n_grid - 1];
  const bool sit_cmd = p.sit_enable != 0 && delta_h < dh_lo - 0.003;
  delta_h = clampd(delta_h, dh_lo, dh_hi);
  const double rel_rh = dot3(rh, rel);
  const double h_rel = qpos[2] - axle[2];
  const double hd_rel = qv[2] - v_ax[2];

  auto fill_info = [&](double* inf, double status_code, int iters_, bool ok_) {
    inf[WBC_INFO_STATUS] = status_code;
    inf[WBC_INFO_SOLVE_US] = 0.0;
    inf[WBC_INFO_PHI] = phi;
    inf[WBC_INFO_PSI_REF] = psi_ref;
    inf[WBC_INFO_A_AXLE] = a_axle;
    inf[WBC_INFO_PITCH] = theta;
    inf[WBC_INFO_ROLL] = roll;
    inf[WBC_INFO_CONTACTS] = n_on;
    inf[WBC_INFO_HEADING_ERR] = e_head;
    inf[WBC_INFO_QP_ITER] = iters_;
    inf[WBC_INFO_FN_R] = Fn_est[0];
    inf[WBC_INFO_FN_L] = Fn_est[1];
    inf[WBC_INFO_PHI_OFF] = phi_off;
    inf[WBC_INFO_DIST_R] = tau_dist[0];
    inf[WBC_INFO_DIST_L] = tau_dist[1];
    inf[WBC_INFO_SAFE_FALL] = fall_mode;
    for (int k = 0; k < NC; ++k) inf[WBC_INFO_FC + k] = ok_ ? x[NV + k] : 0.0;
    for (int j = 0; j < NV; ++j) inf[WBC_INFO_QDD + j] = ok_ ? x[j] : 0.0;
  };

  // ---- Sit: a height command below the posture table lowers the body onto the ground.
  // The legs go softly to the table's lowest row (a few mm under it the torso rests on the
  // floor), the wheels are braked and the QP is skipped. Raising the command stands back up.
  if (sit_cmd) {
    sitting = true;
    for (int k = 0; k < n_leg; ++k) {
      const int a = p.leg_idx[k];
      const double e = p.q_grid[a] - qpos[p.act_qpos[a]];
      last_tau[a] = clampd(p.sit_kp * e - p.sit_kd * qv[p.act_dof[a]], p.tau_min[a], p.tau_max[a]);
    }
    for (int w = 0; w < 2; ++w) {
      const int a = p.wheel_act[w];
      const double lim = std::min(p.fall_tau_wheel, std::min(-p.tau_min[a], p.tau_max[a]));
      last_tau[a] = clampd(-p.fall_kd_wheel * qv[p.act_dof[a]], -lim, lim);
    }
    std::copy(last_tau, last_tau + NU, tau_out);
    std::memset(x, 0, sizeof(x));
    have_x_prev = false;
    if (info) {
      fill_info(info, 0, 0, false);
      info[WBC_INFO_SAFE_FALL] = 5;
    }
    return 0;
  }
  if (sitting) {          // standing up: restart the balance state from the seated pose
    reset();
    sitting = false;
  }

  // ---- Safe fall: latch into a protective mode once a fall is unrecoverable
  // (lean error, roll error or base tilt past its limit for fall_T, or the QP
  // failing for fall_qp_T): brake the wheels, tuck the legs, skip the QP.
  if (p.fall_detect != 0 && !fall_mode) {
    const double lean_err = phi - phi_off - std::atan(a_ff / G);
    int cause = 0;
    if (std::fabs(lean_err) > p.fall_lean) cause = 1;
    else if (std::fabs(roll - psi_ref) > p.fall_roll) cause = 2;
    else if (R[8] < std::cos(p.fall_tilt)) cause = 3;
    t_fall_cond = cause ? t_fall_cond + dt : 0.0;
    if (cause && t_fall_cond >= p.fall_T) fall_mode = cause;
    if (n_fail * dt >= p.fall_qp_T) fall_mode = 4;
    t_upright = 0.0;
  }
  if (fall_mode) {
    for (int k = 0; k < n_leg; ++k) {
      const int a = p.leg_idx[k];
      const double e = p.q_grid[a] - qpos[p.act_qpos[a]];       // lowest posture row
      last_tau[a] = clampd(p.fall_kp * e - p.fall_kd * qv[p.act_dof[a]], p.tau_min[a], p.tau_max[a]);
    }
    for (int w = 0; w < 2; ++w) {
      const int a = p.wheel_act[w];
      const double lim = std::min(p.fall_tau_wheel, std::min(-p.tau_min[a], p.tau_max[a]));
      last_tau[a] = clampd(-p.fall_kd_wheel * qv[p.act_dof[a]], -lim, lim);
    }
    std::copy(last_tau, last_tau + NU, tau_out);
    std::memset(x, 0, sizeof(x));
    have_x_prev = false;
    if (info) fill_info(info, 0, 0, false);
    const bool upright = std::fabs(theta) < 0.15 && std::fabs(roll) < 0.15 && std::fabs(phi) < 0.15 &&
                         n_on == 2 && std::fabs(wx) < 0.5 && std::fabs(wy) < 0.5;
    t_upright = upright ? t_upright + dt : 0.0;
    if (p.fall_recover_T > 0 && t_upright >= p.fall_recover_T) reset();
    return 0;
  }

  // ---- Tasks: rows of J (qdd), targets b, weights w
  double bT[NTASK] = {0}, wT[NTASK] = {0};
  std::memset(JT, 0, sizeof(JT));
  for (int w = 0; w < 2; ++w) {
    if (!in_contact[w]) continue;
    for (int r = 3 * w; r < 3 * w + 3; ++r) {
      std::copy(&Jk[r * NV], &Jk[r * NV] + NV, &JT[r * NV]);
      bT[r] = bk[r];
      wT[r] = p.w_contact;
    }
  }
  if (n_on) {
    for (int j = 0; j < NV; ++j) JT[6 * NV + j] = fh[0] * J_ax[j] + fh[1] * J_ax[NV + j];
    bT[6] = a_axle - dot3(fh, Jd_ax);
    wT[6] = p.w_axle;
  }
  JT[7 * NV + 3] = 1.0;
  bT[7] = -p.kp_pitch * theta - p.kd_pitch * wx;
  wT[7] = p.w_pitch;
  JT[8 * NV + 4] = 1.0;
  bT[8] = -p.kp_roll * (roll - psi_ref) - p.kd_roll * wy;
  wT[8] = p.w_roll;
  JT[9 * NV + 5] = 1.0;
  bT[9] = p.kp_yaw * (yaw_rate_ref - wz);
  wT[9] = p.w_yaw;
  for (int j = 0; j < NV; ++j) JT[10 * NV + j] = -J_ax[2 * NV + j];
  JT[10 * NV + 2] += 1.0;
  bT[10] = p.kp_h * (p.h_rel_nom + delta_h - h_rel) - p.kd_h * hd_rel + Jd_ax[2];
  wT[10] = p.w_height;
  for (int j = 0; j < NV; ++j)
    JT[11 * NV + j] = rh[0] * (Jcom[j] - J_ax[j]) + rh[1] * (Jcom[NV + j] - J_ax[NV + j]);
  bT[11] = p.kp_lat * (p.y_lat_nom + rz * std::tan(psi_ref) - rel_rh) - p.kd_lat * dot3(rh, rel_v) -
           dot3(rh, Jd_ax);
  wT[11] = p.w_lat;

  // ---- Cost: P (NX x NX, full symmetric), q
  std::memset(P, 0, sizeof(P));
  std::memset(q, 0, sizeof(q));
  for (int i = 0; i < NV; ++i) P[i * NX + i] = p.w_qdd;
  for (int i = NV; i < NX; ++i) P[i * NX + i] = p.w_force;
  for (int t = 0; t < NTASK; ++t) {
    if (wT[t] == 0.0) continue;
    const double* Jt = &JT[t * NV];
    for (int i = 0; i < NV; ++i) {
      if (Jt[i] == 0.0) continue;
      const double wi = wT[t] * Jt[i];
      for (int j = 0; j < NV; ++j) P[i * NX + j] += wi * Jt[j];
      q[i] -= wi * bT[t];
    }
  }
  // Posture from the leg-IK table
  int gi = 0;
  while (gi < p.n_grid - 2 && p.dh_grid[gi + 1] < delta_h) ++gi;
  const double gf = (delta_h - p.dh_grid[gi]) / (p.dh_grid[gi + 1] - p.dh_grid[gi]);
  for (int k = 0; k < n_leg; ++k) {
    const int a = p.leg_idx[k];
    const int dof = p.act_dof[a];
    const double qref = (1.0 - gf) * p.q_grid[gi * NU + a] + gf * p.q_grid[(gi + 1) * NU + a];
    const double a_post = p.kp_post * (qref - qpos[p.act_qpos[a]]) - p.kd_post * qv[dof];
    const double wp = p.is_roll[k] ? p.w_post_roll : p.w_post;
    P[dof * NX + dof] += wp;
    q[dof] -= wp * a_post;
  }
  // Tangential contact forces (heading frame)
  for (int w = 0; w < 2; ++w) {
    const int o = NV + 3 * w;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) P[(o + i) * NX + o + j] += p.w_ftan * (fh[i] * fh[j] + rh[i] * rh[j]);
  }

  // ---- Dynamics: tau = M_a qdd + h_a - Jc_a^T F
  for (int a = 0; a < NU; ++a) {
    const int dof = p.act_dof[a];
    double* row = &Atau[a * NX];
    for (int j = 0; j < NV; ++j) row[j] = M[dof * NV + j];
    for (int k = 0; k < NC; ++k) row[NV + k] = -Jc[k * NV + dof];
    btau[a] = h[dof];
  }
  for (int a = 0; a < NU; ++a) {
    const double* row = &Atau[a * NX];
    for (int i = 0; i < NX; ++i) {
      if (row[i] == 0.0) continue;
      const double wi = p.w_tau * row[i];
      for (int j = 0; j < NX; ++j) P[i * NX + j] += wi * row[j];
      q[i] += wi * btau[a];
    }
  }
  if (n_on == 2) {
    const double share = 0.5 * p.total_mass * G;
    P[(NV + 2) * NX + NV + 2] += 1e-4;
    P[(NV + 5) * NX + NV + 5] += 1e-4;
    q[NV + 2] -= 1e-4 * share;
    q[NV + 5] -= 1e-4 * share;
  }

  // ---- Constraints (HPIPM: A x = b, lb <= x[idxb] <= ub, lg <= C x <= ug)
  const int n_air = 2 - n_on;
  const int ne = 6 + 4 * n_air;
  std::memset(A_eq_cm, 0, sizeof(double) * ne * NX);
  for (int r = 0; r < 6; ++r) {                         // floating-base dynamics
    for (int j = 0; j < NV; ++j) A_eq_cm[j * ne + r] = M[r * NV + j];
    for (int k = 0; k < NC; ++k) A_eq_cm[(NV + k) * ne + r] = -Jc[k * NV + r];
    b_eq[r] = -h[r];
  }
  int er = 6;
  for (int w = 0; w < 2; ++w) {
    if (in_contact[w]) continue;
    const int a = p.wheel_act[w], dof = p.act_dof[a];
    for (int j = 0; j < NX; ++j) A_eq_cm[j * ne + er] = Atau[a * NX + j];
    b_eq[er] = tau_air[w] - btau[a] - p.fric[a] * std::tanh(qv[dof] / p.fric_vel);
    ++er;
    for (int k = 0; k < 3; ++k, ++er) {                 // no force on an airborne wheel
      A_eq_cm[(NV + 3 * w + k) * ne + er] = 1.0;
      b_eq[er] = 0.0;
    }
  }
  for (int a = 0; a < NU; ++a) {                        // torque limits
    for (int j = 0; j < NX; ++j) C_cm[j * NG + a] = Atau[a * NX + j];
    lg[a] = p.tau_min[a] - btau[a];
    ug[a] = p.tau_max[a] - btau[a];
  }
  for (int w = 0; w < 2; ++w) {
    if (!in_contact[w]) {
      const int a = p.wheel_act[w];
      lg[a] = -BIG;
      ug[a] = BIG;
    }
    for (int k = 0; k < 4; ++k) ug[NU + 4 * w + k] = in_contact[w] ? 0.0 : BIG;
    lb[n_leg + w] = in_contact[w] ? 0.0 : -1.0;
    ub[n_leg + w] = in_contact[w] ? p.Fz_max : 1.0;
  }
  // Joint limits: keep q + qd*T + 0.5*qdd*T^2 inside [lo, hi]
  const double T = p.lim_T, amax = 2.0 / (T * T);
  for (int k = 0; k < n_leg; ++k) {
    const int a = p.leg_idx[k];
    const double qp_ = qpos[p.act_qpos[a]] + qv[p.act_dof[a]] * T;
    double l = clampd(amax * (p.q_lo[k] - qp_), -1e3, 1e3);
    double u = clampd(amax * (p.q_hi[k] - qp_), -1e3, 1e3);
    if (l > u - 1.0) {               // past a margin: keep a small feasible band
      const double mid = 0.5 * (l + u);
      l = mid - 0.5;
      u = mid + 0.5;
    }
    lb[k] = l;
    ub[k] = u;
  }
  for (int i = 0; i < NX; ++i)
    for (int j = 0; j < NX; ++j) H_cm[j * NX + i] = 0.5 * (P[i * NX + j] + P[j * NX + i]);

  Qp& Q = qp_[n_air];
  d_dense_qp_set_H(H_cm, &Q.qp);
  d_dense_qp_set_g(q, &Q.qp);
  d_dense_qp_set_A(A_eq_cm, &Q.qp);
  d_dense_qp_set_b(b_eq, &Q.qp);
  d_dense_qp_set_lb(lb, &Q.qp);
  d_dense_qp_set_ub(ub, &Q.qp);
  d_dense_qp_set_C(C_cm, &Q.qp);
  d_dense_qp_set_lg(lg, &Q.qp);
  d_dense_qp_set_ug(ug, &Q.qp);

  // Warm start from the previous tick's solution (primal; the instance keeps
  // its own duals). The retry after a failure is a cold start on a rebuilt
  // workspace.
  const int warm = int(p.qp_warm_start);
  int status = 0, iters = 0;
  bool ok = false;
  for (int attempt = 0; attempt < 2 && !ok; ++attempt) {
    int ws_mode = warm;
    if (attempt) {
      Q.reset_ws();
      ws_mode = 0;
    } else if (warm && have_x_prev) {
      d_dense_qp_sol_set_v(x_prev, &Q.sol);
    } else {
      ws_mode = 0;
    }
    d_dense_qp_ipm_arg_set_warm_start(&ws_mode, &Q.arg);
    d_dense_qp_ipm_solve(&Q.qp, &Q.sol, &Q.arg, &Q.ws);
    d_dense_qp_ipm_get_status(&Q.ws, &status);
    d_dense_qp_ipm_get_iter(&Q.ws, &iters);
    d_dense_qp_sol_get_v(&Q.sol, x);
    ok = status <= 2;   // SUCCESS, MAX_ITER, MIN_STEP (NAN_SOL = 3)
    for (int i = 0; ok && i < NX; ++i) ok = std::isfinite(x[i]);
  }
  if (!ok) Q.reset_ws();   // never carry a poisoned workspace into the next tick
  n_fail = ok ? 0 : n_fail + 1;
  have_x_prev = ok;
  if (ok) std::memcpy(x_prev, x, sizeof(x));

  if (ok) {
    for (int a = 0; a < NU; ++a) {
      double t = btau[a];
      for (int j = 0; j < NX; ++j) t += Atau[a * NX + j] * x[j];
      t += p.fric[a] * std::tanh(qv[p.act_dof[a]] / p.fric_vel);
      if (p.dist_comp != 0) {
        if (a == p.wheel_act[0]) t -= tau_dist[0];
        if (a == p.wheel_act[1]) t -= tau_dist[1];
      }
      last_tau[a] = clampd(t, p.tau_min[a], p.tau_max[a]);
    }
  }
  std::copy(last_tau, last_tau + NU, tau_out);
  if (info) fill_info(info, ok ? status : -1 - status, iters, ok);
  return ok ? 0 : -1;
}

}  // namespace wbc
