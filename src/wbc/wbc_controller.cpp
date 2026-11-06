// LQR-assisted whole-body QP controller -- native implementation.
//
// Line-for-line port of the per-tick logic in controllers/wbc/wbc_controller.py
// (see that file's docstring for the formulation). Setup-time work that needs
// scipy (LQR Riccati solve, leg-IK posture table) stays in Python and arrives
// here through WbcParams. The QP is solved with HPIPM's dense IPM (shipped with
// acados), which handles equality rows natively, so airborne-wheel conditions
// are expressed as equalities instead of collapsed inequality bounds.
//
// The controller reads the caller's mjModel / mjData directly (pointers come
// from the Python bindings' `_address`), so there is no per-tick marshalling.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <mujoco/mujoco.h>

extern "C" {
#include "hpipm_d_dense_qp.h"
#include "hpipm_d_dense_qp_dim.h"
#include "hpipm_d_dense_qp_ipm.h"
#include "hpipm_d_dense_qp_sol.h"
}

#include "wbc_params.h"

namespace {

constexpr double G = 9.81;
constexpr double BIG = 1e4;     // "unbounded" for rows that are switched off
constexpr int NC = 6;           // 2 wheels x 3D contact force
constexpr int NTASK = 12;

inline double clampd(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }
inline double dot3(const double* a, const double* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline void cross3(const double* a, const double* b, double* o) {
  o[0] = a[1] * b[2] - a[2] * b[1];
  o[1] = a[2] * b[0] - a[0] * b[2];
  o[2] = a[0] * b[1] - a[1] * b[0];
}
inline double wrap_pi(double a) { return std::remainder(a, 2.0 * M_PI); }

// One HPIPM dense-QP instance for a fixed (ne, nb, ng).
struct HpipmQp {
  d_dense_qp_dim dim;
  d_dense_qp qp;
  d_dense_qp_sol sol;
  d_dense_qp_ipm_arg arg;
  d_dense_qp_ipm_ws ws;
  void* mem = nullptr;

  void create(int nv, int ne, int nb, int ng) {
    hpipm_size_t s_dim = d_dense_qp_dim_memsize();
    void* m_dim = std::malloc(s_dim);
    d_dense_qp_dim_create(&dim, m_dim);
    d_dense_qp_dim_set_all(nv, ne, nb, ng, 0, 0, &dim);
    hpipm_size_t s_qp = d_dense_qp_memsize(&dim);
    hpipm_size_t s_sol = d_dense_qp_sol_memsize(&dim);
    hpipm_size_t s_arg = d_dense_qp_ipm_arg_memsize(&dim);
    // ws memsize needs a created arg; allocate the arg first
    void* m_qp = std::calloc(1, s_qp);
    void* m_sol = std::calloc(1, s_sol);
    void* m_arg = std::calloc(1, s_arg);
    d_dense_qp_create(&dim, &qp, m_qp);
    d_dense_qp_sol_create(&dim, &sol, m_sol);
    d_dense_qp_ipm_arg_create(&dim, &arg, m_arg);
    d_dense_qp_ipm_arg_set_default(BALANCE, &arg);
    int iter_max = 40;
    double tol = 1e-7;
    double mu0 = 1e2;
    int warm = 0;
    d_dense_qp_ipm_arg_set_iter_max(&iter_max, &arg);
    d_dense_qp_ipm_arg_set_tol_stat(&tol, &arg);
    d_dense_qp_ipm_arg_set_tol_eq(&tol, &arg);
    d_dense_qp_ipm_arg_set_tol_ineq(&tol, &arg);
    d_dense_qp_ipm_arg_set_tol_comp(&tol, &arg);
    d_dense_qp_ipm_arg_set_mu0(&mu0, &arg);
    d_dense_qp_ipm_arg_set_warm_start(&warm, &arg);
    hpipm_size_t s_ws = d_dense_qp_ipm_ws_memsize(&dim, &arg);
    void* m_ws = std::calloc(1, s_ws);
    d_dense_qp_ipm_ws_create(&dim, &arg, &ws, m_ws);
    blocks_ = {m_dim, m_qp, m_sol, m_arg, m_ws};
  }
  ~HpipmQp() {
    for (void* p : blocks_) std::free(p);
  }
  std::vector<void*> blocks_;
};

}  // namespace

struct WbcHandle {
  WbcParams p;
  int nv = 0, nu = 0, nx = 0, n_leg = 0;

  // per-tick state
  double s = 0, s_ref = 0, v_ref_prev = 0, a_ref = 0;
  bool have_v_prev = false;
  double heading_ref = 0;
  bool have_heading = false;
  double t_since_contact[2] = {1.0, 1.0};
  std::vector<double> last_tau;
  // contact-load observer (generalized momentum, floating-base rows only)
  bool have_obs = false;
  double p_int[6] = {0}, r_obs[6] = {0};
  std::vector<double> M_prev;
  bool contact_state[2] = {true, true};
  double t_low[2] = {0, 0};
  double Fn_est[2] = {0, 0};
  double phi_off = 0;        // learned lean offset (IMU bias / CoM error)

  // work buffers
  std::vector<double> M, jp, jpd, Jhub[2], Jc, Jk, Jcom, Jw, JT, P, q, Atau, btau;
  std::vector<double> H_cm, A_eq_cm, C_cm, b_eq, lb, ub, lg, ug, x;
  std::vector<int> idxb;
  HpipmQp qp[3];          // indexed by number of airborne wheels

  explicit WbcHandle(const mjModel* m, const WbcParams& prm) : p(prm) {
    nv = m->nv;
    nu = p.nu;
    nx = nv + NC;
    n_leg = p.n_leg;
    last_tau.assign(nu, 0.0);
    M.assign(nv * nv, 0);
    M_prev.assign(nv * nv, 0);
    jp.assign(3 * nv, 0);
    jpd.assign(3 * nv, 0);
    Jhub[0].assign(3 * nv, 0);
    Jhub[1].assign(3 * nv, 0);
    Jc.assign(NC * nv, 0);
    Jk.assign(NC * nv, 0);
    Jcom.assign(3 * nv, 0);
    Jw.assign(3 * nv, 0);
    JT.assign(NTASK * nv, 0);
    P.assign(nx * nx, 0);
    q.assign(nx, 0);
    Atau.assign(nu * nx, 0);
    btau.assign(nu, 0);
    x.assign(nx, 0);
    const int nb = n_leg + 2, ng = nu + 8;
    H_cm.assign(nx * nx, 0);
    A_eq_cm.assign(14 * nx, 0);
    C_cm.assign(ng * nx, 0);
    b_eq.assign(14, 0);
    lb.assign(nb, 0);
    ub.assign(nb, 0);
    lg.assign(ng, 0);
    ug.assign(ng, 0);
    idxb.assign(nb, 0);
    for (int k = 0; k < n_leg; ++k) idxb[k] = p.act_dof[p.leg_idx[k]];
    idxb[n_leg] = nv + 2;
    idxb[n_leg + 1] = nv + 5;

    // Static general-constraint rows: friction pyramids (rows nu .. nu+7)
    std::vector<double> lg_mask(ng, 1.0);
    for (int w = 0; w < 2; ++w) {
      const int fx = nv + 3 * w, fy = fx + 1, fz = fx + 2;
      const int ii[4] = {fx, fx, fy, fy};
      const double sg[4] = {1, -1, 1, -1};
      for (int k = 0; k < 4; ++k) {
        const int r = nu + 4 * w + k;
        C_cm[ii[k] * ng + r] = sg[k];
        C_cm[fz * ng + r] = -p.mu;
        lg_mask[r] = 0.0;         // one-sided: lower bound inactive
        lg[r] = -BIG;
      }
    }
    for (int na = 0; na < 3; ++na) {
      qp[na].create(nx, 6 + 4 * na, nb, ng);
      d_dense_qp_set_idxb(idxb.data(), &qp[na].qp);
      d_dense_qp_set_lg_mask(lg_mask.data(), &qp[na].qp);
    }
    reset();
  }

  void reset() {
    s = s_ref = 0;
    have_v_prev = false;
    a_ref = 0;
    have_heading = false;
    t_since_contact[0] = t_since_contact[1] = 1.0;
    have_obs = false;
    contact_state[0] = contact_state[1] = true;
    t_low[0] = t_low[1] = 0;
    Fn_est[0] = Fn_est[1] = 0;
    phi_off = 0;
    std::fill(last_tau.begin(), last_tau.end(), 0.0);
  }

  // J (3 x nv) and Jdot*qvel at a world point fixed to `body`
  void point_jac(const mjModel* m, mjData* d, const double* pt, int body, double* J, double* jdqv) {
    mj_jac(m, d, J, nullptr, pt, body);
    mj_jacDot(m, d, jpd.data(), nullptr, pt, body);
    for (int i = 0; i < 3; ++i) {
      double acc = 0;
      for (int j = 0; j < nv; ++j) acc += jpd[i * nv + j] * d->qvel[j];
      jdqv[i] = acc;
    }
  }
  void matvec3(const double* J, const double* v, double* o) const {
    for (int i = 0; i < 3; ++i) {
      double acc = 0;
      for (int j = 0; j < nv; ++j) acc += J[i * nv + j] * v[j];
      o[i] = acc;
    }
  }

  int step(const mjModel* m, mjData* d, double v_target, double yaw_target, double delta_h,
           double roll_off, double dt, double* tau_out, double* info) {
    const auto t0 = std::chrono::steady_clock::now();
    const double* qv = d->qvel;

    // Turn-rate limiter: |v * omega| <= a_lat_max
    const double w_cap = p.a_lat_max / std::max(std::fabs(v_target), 1e-3);
    yaw_target = clampd(yaw_target, -w_cap, w_cap);

    // ---- Base state (heading frame)
    const double* R = d->xmat + 9;    // body 1, row-major
    double nf = std::hypot(R[1], R[4]);
    if (nf == 0) nf = 1e-6;
    const double fh[3] = {R[1] / nf, R[4] / nf, 0.0};
    const double rh[3] = {fh[1], -fh[0], 0.0};
    const double theta = std::atan2(R[7], R[8]);
    const double roll = std::atan2(R[2], R[8]);
    const double yaw = std::atan2(fh[1], fh[0]);
    const double wx = qv[3], wy = qv[4], wz = qv[5];

    // ---- Wheel hubs / axle
    const int wid[2] = {p.wheel_body[0], p.wheel_body[1]};
    double Jd_hub[2][3];
    for (int w = 0; w < 2; ++w) point_jac(m, d, d->xpos + 3 * wid[w], wid[w], Jhub[w].data(), Jd_hub[w]);
    std::vector<double>& J_ax = jp;   // reuse as axle Jacobian
    double Jd_ax[3], axle[3], v_ax[3];
    for (int i = 0; i < 3 * nv; ++i) J_ax[i] = 0.5 * (Jhub[0][i] + Jhub[1][i]);
    for (int i = 0; i < 3; ++i) {
      Jd_ax[i] = 0.5 * (Jd_hub[0][i] + Jd_hub[1][i]);
      axle[i] = 0.5 * (d->xpos[3 * wid[0] + i] + d->xpos[3 * wid[1] + i]);
    }
    matvec3(J_ax.data(), qv, v_ax);
    const double v_fwd = dot3(fh, v_ax);

    // ---- Wheel contacts (mean point / normal per wheel)
    double cpos[2][3] = {{0}}, cnrm[2][3] = {{0}}, nrm_w[2][3];
    int ccount[2] = {0, 0};
    bool ccount_close[2] = {false, false};
    for (int i = 0; i < d->ncon; ++i) {
      const mjContact& c = d->contact[i];
      if (c.dist >= p.contact_geom_dist || c.geom[0] < 0 || c.geom[1] < 0) continue;
      const int b1 = m->geom_bodyid[c.geom[0]], b2 = m->geom_bodyid[c.geom[1]];
      for (int w = 0; w < 2; ++w) {
        if (b1 == wid[w] || b2 == wid[w]) {
          for (int k = 0; k < 3; ++k) {
            cpos[w][k] += c.pos[k];
            cnrm[w][k] += c.frame[k];
          }
          ++ccount[w];
          if (c.dist < 2e-3) ccount_close[w] = true;
        }
      }
    }
    bool in_contact[2];
    double bk[NC];
    std::fill(Jc.begin(), Jc.end(), 0.0);
    std::fill(Jk.begin(), Jk.end(), 0.0);
    for (int w = 0; w < 2; ++w) {
      double pt[3], nrm[3];
      const bool on = ccount[w] > 0;
      if (on) {
        double nn = std::sqrt(dot3(cnrm[w], cnrm[w]));
        if (nn == 0) nn = 1;
        const double sgn = cnrm[w][2] < 0 ? -1.0 : 1.0;   // normal points geom1 -> geom2
        for (int k = 0; k < 3; ++k) {
          pt[k] = cpos[w][k] / ccount[w];
          nrm[k] = sgn * cnrm[w][k] / nn;
        }
      } else {
        for (int k = 0; k < 3; ++k) pt[k] = d->xpos[3 * wid[w] + k];
        pt[2] -= p.wheel_radius;
        nrm[0] = nrm[1] = 0;
        nrm[2] = 1;
      }
      std::copy(nrm, nrm + 3, nrm_w[w]);
      double Jdw[3], vc[3], vh[3];
      point_jac(m, d, pt, wid[w], Jw.data(), Jdw);
      std::copy(Jw.begin(), Jw.end(), Jc.begin() + 3 * w * nv);
      double t1[3], t2[3];
      cross3(nrm, fh, t1);
      double n1 = std::sqrt(dot3(t1, t1));
      if (n1 == 0) n1 = 1e-6;
      for (int k = 0; k < 3; ++k) t1[k] /= n1;
      cross3(t1, nrm, t2);
      matvec3(Jw.data(), qv, vc);
      matvec3(Jhub[w].data(), qv, vh);
      const double* tm[2] = {t1, t2};
      for (int r = 0; r < 2; ++r) {
        double* row = &Jk[(3 * w + r) * nv];
        for (int j = 0; j < nv; ++j)
          row[j] = tm[r][0] * Jw[j] + tm[r][1] * Jw[nv + j] + tm[r][2] * Jw[2 * nv + j];
        bk[3 * w + r] = -(dot3(tm[r], Jdw) + p.kv_slip * dot3(tm[r], vc));
      }
      double* row = &Jk[(3 * w + 2) * nv];
      for (int j = 0; j < nv; ++j)
        row[j] = nrm[0] * Jhub[w][j] + nrm[1] * Jhub[w][nv + j] + nrm[2] * Jhub[w][2 * nv + j];
      bk[3 * w + 2] = -(dot3(nrm, Jd_hub[w]) + p.kv_slip * dot3(nrm, vh));
      // geometric contact (only used when contact_from_load == 0)
      t_since_contact[w] = (on && ccount_close[w]) ? 0.0 : t_since_contact[w] + dt;
    }

    // ---- Wheel load estimate. Generalized-momentum observer on the 6
    // floating-base rows: r -> tau_ext,b = Jc_b^T F. These rows carry no
    // actuator torque or joint friction, so the estimate does not depend on the
    // motor model. F follows from a damped least-squares solve; the normal
    // components decide contact (robust to attitude bias that fools geometry).
    mj_fullM(m, d, M.data());
    const double* h = d->qfrc_bias;
    {
      double pb[6], mdq[6];
      for (int i = 0; i < 6; ++i) {
        double a = 0, b = 0;
        for (int j = 0; j < nv; ++j) {
          a += M[i * nv + j] * qv[j];
          b += (M[i * nv + j] - M_prev[i * nv + j]) * qv[j];
        }
        pb[i] = a;
        mdq[i] = have_obs ? b / dt : 0.0;
      }
      if (!have_obs) {                 // assume resting on the ground: tau_ext,b = h_b
        for (int i = 0; i < 6; ++i) {
          r_obs[i] = h[i];
          p_int[i] = pb[i] - r_obs[i] / p.obs_K;
        }
        have_obs = true;
      }
      for (int i = 0; i < 6; ++i) r_obs[i] = p.obs_K * (pb[i] - p_int[i]);
      for (int i = 0; i < 6; ++i) p_int[i] += dt * (r_obs[i] - h[i] + mdq[i]);
      std::copy(M.begin(), M.end(), M_prev.begin());
      // Solve (A^T A + lam I) F = A^T r with A[i][k] = Jc[k][i] (6x6)
      double N[36], rhs[6];
      for (int a = 0; a < 6; ++a) {
        rhs[a] = 0;
        for (int i = 0; i < 6; ++i) rhs[a] += Jc[a * nv + i] * r_obs[i];
        for (int b = 0; b < 6; ++b) {
          double acc = 0;
          for (int i = 0; i < 6; ++i) acc += Jc[a * nv + i] * Jc[b * nv + i];
          N[a * 6 + b] = acc + (a == b ? 1e-6 : 0.0);
        }
      }
      // Gaussian elimination with partial pivoting
      for (int c = 0; c < 6; ++c) {
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
    }
    for (int w = 0; w < 2; ++w) {
      if (p.contact_from_load != 0) {
        if (contact_state[w]) {
          t_low[w] = Fn_est[w] < p.fz_off ? t_low[w] + dt : 0.0;
          if (t_low[w] > p.contact_hold) contact_state[w] = false;
        } else if (Fn_est[w] > p.fz_on) {
          contact_state[w] = true;
          t_low[w] = 0.0;
        }
        in_contact[w] = contact_state[w];
      } else {
        in_contact[w] = t_since_contact[w] < p.contact_hold;
      }
    }
    const int n_on = int(in_contact[0]) + int(in_contact[1]);

    // Airborne wheels: fixed torque that spins them to ground speed
    double tau_air[2] = {0, 0};
    for (int w = 0; w < 2; ++w) {
      if (in_contact[w]) continue;
      const int a = p.wheel_act[w], dof = p.act_dof[a];
      const double k_w = Jk[(3 * w + 1) * nv + dof];
      const double rate_ref = std::fabs(k_w) > 1e-4 ? -v_fwd / k_w : 0.0;
      tau_air[w] = clampd(-p.kd_wheel_air * (qv[dof] - rate_ref), p.tau_min[a], p.tau_max[a]);
    }

    // ---- LQR balance layer on the whole-body CoM lean over the axle
    mj_jacSubtreeCom(m, d, Jcom.data(), 1);
    double rel[3], rel_v[3], vcom[3];
    matvec3(Jcom.data(), qv, vcom);
    for (int k = 0; k < 3; ++k) {
      rel[k] = d->subtree_com[3 + k] - axle[k];
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
    // Learned lean offset: integral action on the position error routed through
    // the lean channel (PI on position). Absorbs IMU tilt bias and CoM error that
    // would otherwise leave a standing offset or a slow creep. Frozen while the
    // position error is clamped (pushes), a wheel is off the ground, or moving.
    // Only learned near standstill: while driving, e_s is dominated by tracking
    // lag, which is not an offset and must not be integrated.
    if (n_on == 2 && std::fabs(s - s_ref) < p.s_err_max && K[2] != 0.0 &&
        std::fabs(v_target) < p.lean_learn_vmax && std::fabs(yaw_target) < 0.1 &&
        std::fabs(a_ref) < 0.1) {
      phi_off -= dt * p.ki_lean * (K[0] / K[2]) * e_s;
      phi_off = clampd(phi_off, -p.lean_off_max, p.lean_off_max);
    }
    const double u_fb = -(K[0] * e_s + K[1] * (v_fwd - v_target) +
                          K[2] * (phi - phi_off - std::atan(a_ff / G)) + K[3] * phid);
    const double a_axle = clampd(a_ff + u_fb, -p.a_max, p.a_max);

    // ---- Heading hold: integrate the yaw-rate command into a heading reference
    // so yaw disturbances (pushes, wheel slip, bumps) are steered back out.
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
    double dh_lo = p.dh_grid[0], dh_hi = p.dh_grid[p.n_grid - 1];
    delta_h = clampd(delta_h, dh_lo, dh_hi);
    const double rel_rh = dot3(rh, rel);
    const double h_rel = d->qpos[2] - axle[2];
    const double hd_rel = qv[2] - v_ax[2];

    double bT[NTASK] = {0}, wT[NTASK] = {0};
    std::fill(JT.begin(), JT.end(), 0.0);
    for (int w = 0; w < 2; ++w) {
      if (!in_contact[w]) continue;
      for (int r = 3 * w; r < 3 * w + 3; ++r) {
        std::copy(&Jk[r * nv], &Jk[r * nv] + nv, &JT[r * nv]);
        bT[r] = bk[r];
        wT[r] = p.w_contact;
      }
    }
    if (n_on) {
      for (int j = 0; j < nv; ++j) JT[6 * nv + j] = fh[0] * J_ax[j] + fh[1] * J_ax[nv + j];
      bT[6] = a_axle - dot3(fh, Jd_ax);
      wT[6] = p.w_axle;
    }
    JT[7 * nv + 3] = 1.0;
    bT[7] = -p.kp_pitch * theta - p.kd_pitch * wx;
    wT[7] = p.w_pitch;
    JT[8 * nv + 4] = 1.0;
    bT[8] = -p.kp_roll * (roll - psi_ref) - p.kd_roll * wy;
    wT[8] = p.w_roll;
    JT[9 * nv + 5] = 1.0;
    bT[9] = p.kp_yaw * (yaw_rate_ref - wz);
    wT[9] = p.w_yaw;
    for (int j = 0; j < nv; ++j) JT[10 * nv + j] = -J_ax[2 * nv + j];
    JT[10 * nv + 2] += 1.0;
    bT[10] = p.kp_h * (p.h_rel_nom + delta_h - h_rel) - p.kd_h * hd_rel + Jd_ax[2];
    wT[10] = p.w_height;
    for (int j = 0; j < nv; ++j)
      JT[11 * nv + j] = rh[0] * (Jcom[j] - J_ax[j]) + rh[1] * (Jcom[nv + j] - J_ax[nv + j]);
    bT[11] = p.kp_lat * (p.y_lat_nom + rz * std::tan(psi_ref) - rel_rh) - p.kd_lat * dot3(rh, rel_v) - dot3(rh, Jd_ax);
    wT[11] = p.w_lat;

    // ---- Cost: P (nx x nx, full symmetric), q
    std::fill(P.begin(), P.end(), 0.0);
    std::fill(q.begin(), q.end(), 0.0);
    for (int i = 0; i < nv; ++i) P[i * nx + i] = p.w_qdd;
    for (int i = nv; i < nx; ++i) P[i * nx + i] = p.w_force;
    for (int t = 0; t < NTASK; ++t) {
      if (wT[t] == 0.0) continue;
      const double* Jt = &JT[t * nv];
      for (int i = 0; i < nv; ++i) {
        if (Jt[i] == 0.0) continue;
        const double wi = wT[t] * Jt[i];
        for (int j = 0; j < nv; ++j) P[i * nx + j] += wi * Jt[j];
        q[i] -= wi * bT[t];
      }
    }
    // Posture from the leg-IK table
    int gi = 0;
    {
      const int ng_ = p.n_grid;
      while (gi < ng_ - 2 && p.dh_grid[gi + 1] < delta_h) ++gi;
    }
    const double gf = (delta_h - p.dh_grid[gi]) / (p.dh_grid[gi + 1] - p.dh_grid[gi]);
    for (int k = 0; k < n_leg; ++k) {
      const int a = p.leg_idx[k];
      const int dof = p.act_dof[a];
      const double qref = (1.0 - gf) * p.q_grid[gi * nu + a] + gf * p.q_grid[(gi + 1) * nu + a];
      const double a_post = p.kp_post * (qref - d->qpos[p.act_qpos[a]]) - p.kd_post * qv[dof];
      const double wp = p.is_roll[k] ? p.w_post_roll : p.w_post;
      P[dof * nx + dof] += wp;
      q[dof] -= wp * a_post;
    }
    // Tangential contact forces (heading frame)
    for (int w = 0; w < 2; ++w) {
      const int o = nv + 3 * w;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P[(o + i) * nx + o + j] += p.w_ftan * (fh[i] * fh[j] + rh[i] * rh[j]);
    }

    // ---- Dynamics: tau = M_a qdd + h_a - Jc_a^T F
    for (int a = 0; a < nu; ++a) {
      const int dof = p.act_dof[a];
      double* row = &Atau[a * nx];
      for (int j = 0; j < nv; ++j) row[j] = M[dof * nv + j];
      for (int k = 0; k < NC; ++k) row[nv + k] = -Jc[k * nv + dof];
      btau[a] = h[dof];
    }
    for (int a = 0; a < nu; ++a) {
      const double* row = &Atau[a * nx];
      for (int i = 0; i < nx; ++i) {
        if (row[i] == 0.0) continue;
        const double wi = p.w_tau * row[i];
        for (int j = 0; j < nx; ++j) P[i * nx + j] += wi * row[j];
        q[i] += wi * btau[a];
      }
    }
    if (n_on == 2) {
      const double share = 0.5 * p.total_mass * G;
      P[(nv + 2) * nx + nv + 2] += 1e-4;
      P[(nv + 5) * nx + nv + 5] += 1e-4;
      q[nv + 2] -= 1e-4 * share;
      q[nv + 5] -= 1e-4 * share;
    }

    // ---- Constraints (HPIPM: A x = b, lb <= x[idxb] <= ub, lg <= C x <= ug)
    const int n_air = 2 - n_on;
    const int ne = 6 + 4 * n_air;
    const int ng = nu + 8;
    std::fill(A_eq_cm.begin(), A_eq_cm.begin() + ne * nx, 0.0);
    for (int r = 0; r < 6; ++r) {                         // floating-base dynamics
      for (int j = 0; j < nv; ++j) A_eq_cm[j * ne + r] = M[r * nv + j];
      for (int k = 0; k < NC; ++k) A_eq_cm[(nv + k) * ne + r] = -Jc[k * nv + r];
      b_eq[r] = -h[r];
    }
    int er = 6;
    for (int w = 0; w < 2; ++w) {
      if (in_contact[w]) continue;
      const int a = p.wheel_act[w], dof = p.act_dof[a];
      for (int j = 0; j < nx; ++j) A_eq_cm[j * ne + er] = Atau[a * nx + j];
      b_eq[er] = tau_air[w] - btau[a] - p.fric[a] * std::tanh(qv[dof] / p.fric_vel);
      ++er;
      for (int k = 0; k < 3; ++k, ++er) {                 // no force on an airborne wheel
        A_eq_cm[(nv + 3 * w + k) * ne + er] = 1.0;
        b_eq[er] = 0.0;
      }
    }
    for (int a = 0; a < nu; ++a) {                        // torque limits
      for (int j = 0; j < nx; ++j) C_cm[j * ng + a] = Atau[a * nx + j];
      lg[a] = p.tau_min[a] - btau[a];
      ug[a] = p.tau_max[a] - btau[a];
    }
    for (int w = 0; w < 2; ++w) {
      if (!in_contact[w]) {
        const int a = p.wheel_act[w];
        lg[a] = -BIG;
        ug[a] = BIG;
      }
      for (int k = 0; k < 4; ++k) ug[nu + 4 * w + k] = in_contact[w] ? 0.0 : BIG;
      lb[n_leg + w] = in_contact[w] ? 0.0 : -1.0;
      ub[n_leg + w] = in_contact[w] ? p.Fz_max : 1.0;
    }
    // Joint limits: keep q + qd*T + 0.5*qdd*T^2 inside [lo, hi]
    const double T = p.lim_T, amax = 2.0 / (T * T);
    for (int k = 0; k < n_leg; ++k) {
      const int a = p.leg_idx[k];
      const double qp_ = d->qpos[p.act_qpos[a]] + qv[p.act_dof[a]] * T;
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
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < nx; ++j) H_cm[j * nx + i] = 0.5 * (P[i * nx + j] + P[j * nx + i]);

    HpipmQp& Q = qp[n_air];
    d_dense_qp_set_H(H_cm.data(), &Q.qp);
    d_dense_qp_set_g(q.data(), &Q.qp);
    d_dense_qp_set_A(A_eq_cm.data(), &Q.qp);
    d_dense_qp_set_b(b_eq.data(), &Q.qp);
    d_dense_qp_set_lb(lb.data(), &Q.qp);
    d_dense_qp_set_ub(ub.data(), &Q.qp);
    d_dense_qp_set_C(C_cm.data(), &Q.qp);
    d_dense_qp_set_lg(lg.data(), &Q.qp);
    d_dense_qp_set_ug(ug.data(), &Q.qp);
    d_dense_qp_ipm_solve(&Q.qp, &Q.sol, &Q.arg, &Q.ws);
    int status = 0, iters = 0;
    d_dense_qp_ipm_get_status(&Q.ws, &status);
    d_dense_qp_ipm_get_iter(&Q.ws, &iters);
    d_dense_qp_sol_get_v(&Q.sol, x.data());
    bool ok = status <= 2;   // SUCCESS, MAX_ITER, MIN_STEP (NAN_SOL = 3)
    for (int i = 0; ok && i < nx; ++i) ok = std::isfinite(x[i]);

    if (ok) {
      for (int a = 0; a < nu; ++a) {
        double t = btau[a];
        for (int j = 0; j < nx; ++j) t += Atau[a * nx + j] * x[j];
        t += p.fric[a] * std::tanh(qv[p.act_dof[a]] / p.fric_vel);
        last_tau[a] = clampd(t, p.tau_min[a], p.tau_max[a]);
      }
    }
    std::copy(last_tau.begin(), last_tau.end(), tau_out);

    if (info) {
      const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
      info[WBC_INFO_STATUS] = ok ? status : -1 - status;
      info[WBC_INFO_SOLVE_US] = us;
      info[WBC_INFO_PHI] = phi;
      info[WBC_INFO_PSI_REF] = psi_ref;
      info[WBC_INFO_A_AXLE] = a_axle;
      info[WBC_INFO_PITCH] = theta;
      info[WBC_INFO_ROLL] = roll;
      info[WBC_INFO_CONTACTS] = n_on;
      info[WBC_INFO_HEADING_ERR] = e_head;
      info[WBC_INFO_QP_ITER] = iters;
      info[WBC_INFO_FN_R] = Fn_est[0];
      info[WBC_INFO_FN_L] = Fn_est[1];
      info[WBC_INFO_PHI_OFF] = phi_off;
      for (int k = 0; k < NC; ++k) info[WBC_INFO_FC + k] = ok ? x[nv + k] : 0.0;
      for (int j = 0; j < nv; ++j) info[WBC_INFO_QDD + j] = ok ? x[j] : 0.0;
    }
    return ok ? 0 : -1;
  }
};

extern "C" {

int wbc_params_size() { return int(sizeof(WbcParams)); }

WbcHandle* wbc_create(const mjModel* m, const WbcParams* prm) {
  if (!m || !prm || prm->nu > WBC_MAX_ACT || prm->n_leg > WBC_MAX_LEG || prm->n_grid < 2 ||
      prm->n_grid > WBC_MAX_GRID)
    return nullptr;
  return new WbcHandle(m, *prm);
}

void wbc_destroy(WbcHandle* h) { delete h; }

void wbc_reset(WbcHandle* h) {
  if (h) h->reset();
}

// Returns 0 on a successful QP solve (tau_out holds the new command) and -1 if
// the solve failed (tau_out holds the last good command).
int wbc_step(WbcHandle* h, const mjModel* m, mjData* d, double v_target, double yaw_target,
             double delta_h, double roll_target_offset, double dt, double* tau_out, double* info) {
  if (!h || !m || !d || !tau_out) return -2;
  return h->step(m, d, v_target, yaw_target, delta_h, roll_target_offset, dt, tau_out, info);
}

}  // extern "C"
