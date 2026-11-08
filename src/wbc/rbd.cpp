#include "rbd.h"

#include <cmath>
#include <cstring>

namespace wbc {
namespace {

using namespace model;

inline void cross(const double* a, const double* b, double* o) {
  const double x = a[1] * b[2] - a[2] * b[1];
  const double y = a[2] * b[0] - a[0] * b[2];
  const double z = a[0] * b[1] - a[1] * b[0];
  o[0] = x;
  o[1] = y;
  o[2] = z;
}

inline void quat2mat(const double* q, double* R) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1 - 2 * (y * y + z * z);
  R[1] = 2 * (x * y - w * z);
  R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z);
  R[4] = 1 - 2 * (x * x + z * z);
  R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y);
  R[7] = 2 * (y * z + w * x);
  R[8] = 1 - 2 * (x * x + y * y);
}

// Normalized quaternion -> rotation (MuJoCo normalizes free-joint quaternions)
inline void quat2mat_norm(const double* q, double* R) {
  double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-15) n = 1.0;
  const double qn[4] = {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
  quat2mat(qn, R);
}

inline void matmul3(const double* A, const double* B, double* C) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
}

inline void matvec3(const double* A, const double* v, double* o) {
  for (int i = 0; i < 3; ++i) o[i] = A[3 * i] * v[0] + A[3 * i + 1] * v[1] + A[3 * i + 2] * v[2];
}

// Rotation by `angle` about unit axis `a` (Rodrigues)
inline void axis_angle(const double* a, double angle, double* R) {
  const double c = std::cos(angle), s = std::sin(angle), t = 1 - c;
  const double x = a[0], y = a[1], z = a[2];
  R[0] = t * x * x + c;
  R[1] = t * x * y - s * z;
  R[2] = t * x * z + s * y;
  R[3] = t * x * y + s * z;
  R[4] = t * y * y + c;
  R[5] = t * y * z - s * x;
  R[6] = t * x * z - s * y;
  R[7] = t * y * z + s * x;
  R[8] = t * z * z + c;
}

// a + w x (w x r) style helper: o = w x (w x r)
inline void cross2(const double* w, const double* r, double* o) {
  double t[3];
  cross(w, r, t);
  cross(w, t, o);
}

}  // namespace

Rbd::Rbd() {
  // Ancestor dof lists: a body's Jacobian is non-zero only on the dofs of the
  // joints between it and the world.
  for (int b = 0; b < NB; ++b) {
    ndof_[b] = 0;
    for (int k = b; k > 0; k = PARENT[k]) {
      if (JNT_TYPE[k] < 0) continue;
      const int nd = JNT_TYPE[k] == 0 ? 6 : 1;
      for (int i = 0; i < nd; ++i) dofs_[b][ndof_[b]++] = DOF_ADR[k] + i;
    }
  }
  std::memset(xpos_, 0, sizeof(xpos_));
  std::memset(xmat_, 0, sizeof(xmat_));
  xmat_[0][0] = xmat_[0][4] = xmat_[0][8] = 1.0;
}

void Rbd::forward(const double* qpos, const double* qvel) {
  qvel_ = qvel;
  std::memset(w_[0], 0, sizeof(w_[0]));
  std::memset(v_[0], 0, sizeof(v_[0]));
  std::memset(alpha_[0], 0, sizeof(alpha_[0]));
  std::memset(acc_[0], 0, sizeof(acc_[0]));
  for (int b = 1; b < NB; ++b) {
    const int p = PARENT[b];
    double* R = xmat_[b];
    if (JNT_TYPE[b] == 0) {
      // Free joint: pose straight from qpos; linear velocity is world-frame,
      // angular velocity body-frame. With qacc = 0 the origin does not
      // accelerate and R*w_body has zero derivative.
      const int qa = QPOS_ADR[b], da = DOF_ADR[b];
      for (int i = 0; i < 3; ++i) xpos_[b][i] = qpos[qa + i];
      quat2mat_norm(qpos + qa + 3, R);
      matvec3(R, qvel + da + 3, w_[b]);
      for (int i = 0; i < 3; ++i) {
        v_[b][i] = qvel[da + i];
        alpha_[b][i] = 0.0;
        acc_[b][i] = 0.0;
      }
      for (int i = 0; i < 3; ++i) axis_[b][i] = 0.0;
    } else {
      double Rb[9], R0[9], r[3];
      matvec3(xmat_[p], BODY_POS[b], r);
      for (int i = 0; i < 3; ++i) xpos_[b][i] = xpos_[p][i] + r[i];
      quat2mat(BODY_QUAT[b], Rb);
      matmul3(xmat_[p], Rb, R0);
      double qd = 0.0;
      if (JNT_TYPE[b] == 1) {
        double Rj[9];
        axis_angle(JNT_AXIS[b], qpos[QPOS_ADR[b]], Rj);
        matmul3(R0, Rj, R);
        matvec3(R0, JNT_AXIS[b], axis_[b]);
        qd = qvel[DOF_ADR[b]];
      } else {
        std::memcpy(R, R0, sizeof(R0));
        for (int i = 0; i < 3; ++i) axis_[b][i] = 0.0;
      }
      // Velocities; the hinge anchor is the body origin, so the joint adds no
      // linear velocity at the origin.
      double t[3], u[3];
      cross(w_[p], r, t);
      for (int i = 0; i < 3; ++i) {
        w_[b][i] = w_[p][i] + axis_[b][i] * qd;
        v_[b][i] = v_[p][i] + t[i];
      }
      // Bias accelerations (qacc = 0): the hinge axis turns with the parent.
      double aq[3] = {axis_[b][0] * qd, axis_[b][1] * qd, axis_[b][2] * qd};
      cross(w_[p], aq, t);
      for (int i = 0; i < 3; ++i) alpha_[b][i] = alpha_[p][i] + t[i];
      cross(alpha_[p], r, t);
      cross2(w_[p], r, u);
      for (int i = 0; i < 3; ++i) acc_[b][i] = acc_[p][i] + t[i] + u[i];
    }
    // CoM position and world inertia
    double ri[3];
    matvec3(R, IPOS[b], ri);
    for (int i = 0; i < 3; ++i) xipos_[b][i] = xpos_[b][i] + ri[i];
    double T[9];
    matmul3(R, INERTIA[b], T);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        Iw_[b][3 * i + j] = T[3 * i] * R[3 * j] + T[3 * i + 1] * R[3 * j + 1] + T[3 * i + 2] * R[3 * j + 2];
  }
  // Per-body CoM Jacobians, whole-robot CoM and its Jacobian
  double msum = 0.0;
  for (int i = 0; i < 3; ++i) com_[i] = comd_[i] = 0.0;
  std::memset(Jcom_, 0, sizeof(Jcom_));
  for (int b = 1; b < NB; ++b) {
    jac_impl(b, xipos_[b], Jc_[b], Jr_[b]);
    const double m = MASS[b];
    msum += m;
    double ab[3];
    jacdot_qvel(b, xipos_[b], ab);
    for (int i = 0; i < 3; ++i) {
      com_[i] += m * xipos_[b][i];
      comd_[i] += m * ab[i];
    }
    for (int k = 0; k < ndof_[b]; ++k) {
      const int j = dofs_[b][k];
      for (int i = 0; i < 3; ++i) Jcom_[i * NV + j] += m * Jc_[b][i * NV + j];
    }
  }
  if (msum > 0) {
    for (int i = 0; i < 3; ++i) {
      com_[i] /= msum;
      comd_[i] /= msum;
    }
    for (int i = 0; i < 3 * NV; ++i) Jcom_[i] /= msum;
  }
}

void Rbd::jac_impl(int body, const double* pt, double* Jp, double* Jr) const {
  std::memset(Jp, 0, sizeof(double) * 3 * NV);
  if (Jr) std::memset(Jr, 0, sizeof(double) * 3 * NV);
  for (int k = body; k > 0; k = PARENT[k]) {
    if (JNT_TYPE[k] < 0) continue;
    double d[3];
    for (int i = 0; i < 3; ++i) d[i] = pt[i] - xpos_[k][i];
    if (JNT_TYPE[k] == 0) {
      const int da = DOF_ADR[k];
      const double* R = xmat_[k];
      for (int i = 0; i < 3; ++i) Jp[i * NV + da + i] = 1.0;
      for (int c = 0; c < 3; ++c) {          // body-frame angular axes
        const double e[3] = {R[c], R[3 + c], R[6 + c]};
        double t[3];
        cross(e, d, t);
        for (int i = 0; i < 3; ++i) {
          Jp[i * NV + da + 3 + c] = t[i];
          if (Jr) Jr[i * NV + da + 3 + c] = e[i];
        }
      }
    } else {
      const int j = DOF_ADR[k];
      double t[3];
      cross(axis_[k], d, t);
      for (int i = 0; i < 3; ++i) {
        Jp[i * NV + j] = t[i];
        if (Jr) Jr[i * NV + j] = axis_[k][i];
      }
    }
  }
}

void Rbd::jac_point(int body, const double* pt, double* J) const { jac_impl(body, pt, J, nullptr); }

void Rbd::jacdot_qvel(int body, const double* pt, double* out) const {
  double r[3], t[3], u[3];
  for (int i = 0; i < 3; ++i) r[i] = pt[i] - xpos_[body][i];
  cross(alpha_[body], r, t);
  cross2(w_[body], r, u);
  for (int i = 0; i < 3; ++i) out[i] = acc_[body][i] + t[i] + u[i];
}

void Rbd::mass_matrix(double* M) const {
  std::memset(M, 0, sizeof(double) * NV * NV);
  for (int b = 1; b < NB; ++b) {
    const double m = MASS[b];
    const double* Jp = Jc_[b];
    const double* Jr = Jr_[b];
    const double* I = Iw_[b];
    const int n = ndof_[b];
    for (int ka = 0; ka < n; ++ka) {
      const int a = dofs_[b][ka];
      // I * Jr[:, a]
      double IJ[3];
      for (int i = 0; i < 3; ++i)
        IJ[i] = I[3 * i] * Jr[a] + I[3 * i + 1] * Jr[NV + a] + I[3 * i + 2] * Jr[2 * NV + a];
      for (int kb = 0; kb < n; ++kb) {
        const int c = dofs_[b][kb];
        M[a * NV + c] += m * (Jp[a] * Jp[c] + Jp[NV + a] * Jp[NV + c] + Jp[2 * NV + a] * Jp[2 * NV + c]) +
                         IJ[0] * Jr[c] + IJ[1] * Jr[NV + c] + IJ[2] * Jr[2 * NV + c];
      }
    }
  }
  for (int j = 0; j < NV; ++j) M[j * NV + j] += ARMATURE[j];
}

void Rbd::bias(double* h) const {
  std::memset(h, 0, sizeof(double) * NV);
  for (int b = 1; b < NB; ++b) {
    const double m = MASS[b];
    double r[3], t[3], u[3], f[3], n[3], Iw[3], Ia[3];
    for (int i = 0; i < 3; ++i) r[i] = xipos_[b][i] - xpos_[b][i];
    cross(alpha_[b], r, t);
    cross2(w_[b], r, u);
    for (int i = 0; i < 3; ++i) f[i] = m * (acc_[b][i] + t[i] + u[i] - GRAVITY[i]);
    matvec3(Iw_[b], w_[b], Iw);
    matvec3(Iw_[b], alpha_[b], Ia);
    cross(w_[b], Iw, t);
    for (int i = 0; i < 3; ++i) n[i] = Ia[i] + t[i];
    for (int k = 0; k < ndof_[b]; ++k) {
      const int j = dofs_[b][k];
      h[j] += Jc_[b][j] * f[0] + Jc_[b][NV + j] * f[1] + Jc_[b][2 * NV + j] * f[2] +
              Jr_[b][j] * n[0] + Jr_[b][NV + j] * n[1] + Jr_[b][2 * NV + j] * n[2];
    }
  }
}

}  // namespace wbc
