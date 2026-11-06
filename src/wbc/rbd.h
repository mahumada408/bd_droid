// Standalone rigid-body dynamics for the robot in robot_model.h.
//
// Replaces the MuJoCo calls the whole-body controller used (mj_fullM,
// qfrc_bias, mj_jac, mj_jacDot, mj_jacSubtreeCom, subtree_com) so the
// controller can run without MuJoCo, e.g. on a microcontroller. Fixed sizes,
// no heap allocation. Follows MuJoCo conventions exactly so results match it
// to rounding error (checked by tests/wbc_rbd_check.py):
//   * qpos: free joint [pos(3), quat wxyz(4)] then hinge angles
//   * qvel: free joint [linear velocity (world), angular velocity (body)] then rates
//   * matrices are row-major; xmat maps body to world coordinates
#pragma once

#include "robot_model_select.h"

namespace wbc {

class Rbd {
 public:
  static constexpr int NV = model::NV;
  static constexpr int NB = model::NBODY;

  Rbd();

  // Position and velocity kinematics, per-body CoM Jacobians and bias
  // accelerations. Call once per tick before any query below.
  void forward(const double* qpos, const double* qvel);

  // Translational Jacobian (3 x NV) of a world point rigidly attached to `body`.
  void jac_point(int body, const double* pt, double* J) const;
  // Jdot * qvel for the same point (its acceleration at qacc = 0).
  void jacdot_qvel(int body, const double* pt, double* out) const;
  // Joint-space inertia including armature (NV x NV).
  void mass_matrix(double* M) const;
  // Coriolis, centrifugal and gravity forces (MuJoCo qfrc_bias).
  void bias(double* h) const;
  // Whole-robot CoM, its Jacobian (3 x NV) and velocity.
  const double* com() const { return com_; }
  const double* com_jac() const { return Jcom_; }

  // Body frames (world)
  const double* xpos(int b) const { return xpos_[b]; }
  const double* xmat(int b) const { return xmat_[b]; }
  const double* axis(int b) const { return axis_[b]; }   // hinge axis, world

 private:
  const double* qvel_ = nullptr;
  double xpos_[NB][3], xmat_[NB][9], axis_[NB][3];
  double xipos_[NB][3], Iw_[NB][9];
  double w_[NB][3], v_[NB][3];        // angular velocity, origin linear velocity (world)
  double alpha_[NB][3], acc_[NB][3];  // bias angular / origin linear acceleration
  double Jc_[NB][3 * NV], Jr_[NB][3 * NV];   // CoM translational / rotational Jacobians
  double com_[3], Jcom_[3 * NV];
  // ancestor dofs per body (the only non-zero Jacobian columns)
  int ndof_[NB], dofs_[NB][NV];

  void jac_impl(int body, const double* pt, double* Jp, double* Jr) const;
};

}  // namespace wbc
