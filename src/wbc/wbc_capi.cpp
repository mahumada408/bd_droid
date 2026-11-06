// C API over WbcCore for the simulator (loaded by controllers/wbc/wbc_controller.py
// through ctypes as //:libwbc.so). Host-only: adds heap allocation and timing,
// which the core itself avoids so it can run on a microcontroller.

#include <chrono>
#include <cstring>
#include <new>

#include "rbd.h"
#include "robot_model_select.h"
#include "wbc_core.h"

using wbc::WbcCore;

extern "C" {

int wbc_params_size() { return int(sizeof(WbcParams)); }

// Generated-model fingerprint, so the Python side can refuse a stale robot_model.h.
void wbc_model_info(int* nq, int* nv, int* nu, int* nbody, double* total_mass) {
  *nq = wbc::model::NQ;
  *nv = wbc::model::NV;
  *nu = wbc::model::NU;
  *nbody = wbc::model::NBODY;
  *total_mass = wbc::model::TOTAL_MASS;
}

// Returns a handle, or nullptr with *err set (WbcInitError).
WbcCore* wbc_create(const WbcParams* prm, int* err) {
  if (!prm) return nullptr;
  WbcCore* h = new (std::nothrow) WbcCore();
  if (!h) return nullptr;
  const int rc = h->init(*prm);
  if (err) *err = rc;
  if (rc != wbc::WBC_OK) {
    delete h;
    return nullptr;
  }
  return h;
}

void wbc_destroy(WbcCore* h) { delete h; }

void wbc_reset(WbcCore* h) {
  if (h) h->reset();
}

long wbc_arena_used(const WbcCore* h) { return h ? long(h->arena_used()) : -1; }

// Returns 0 if tau_out holds a fresh command, -1 if the QP failed and the
// last command was repeated.
int wbc_step(WbcCore* h, const double* qpos, const double* qvel, double v_target, double yaw_target,
             double delta_h, double roll_target_offset, double dt, double* tau_out, double* info) {
  if (!h || !qpos || !qvel || !tau_out) return -2;
  const auto t0 = std::chrono::steady_clock::now();
  const int rc = h->step(qpos, qvel, v_target, yaw_target, delta_h, roll_target_offset, dt, tau_out, info);
  if (info)
    info[WBC_INFO_SOLVE_US] =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
  return rc;
}

// Test hook: standalone dynamics at (qpos, qvel), compared against MuJoCo by
// tests/wbc_rbd_check.py. M: NV*NV, h: NV, J: 3*NV, jdqv: 3, com: 3, Jcom: 3*NV.
void wbc_rbd_eval(const double* qpos, const double* qvel, int body, const double* pt, double* M, double* h,
                  double* J, double* jdqv, double* com, double* Jcom) {
  static wbc::Rbd rbd;
  rbd.forward(qpos, qvel);
  rbd.mass_matrix(M);
  rbd.bias(h);
  rbd.jac_point(body, pt, J);
  rbd.jacdot_qvel(body, pt, jdqv);
  std::memcpy(com, rbd.com(), sizeof(double) * 3);
  std::memcpy(Jcom, rbd.com_jac(), sizeof(double) * 3 * wbc::model::NV);
}

}  // extern "C"
