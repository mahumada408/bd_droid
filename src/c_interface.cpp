#include <iostream>
#include "acados_mpc_wrapper.hpp"

extern "C" {

struct MpcHandle {
    balancer::AcadosMpcWrapper mpc;
};

MpcHandle* mpc_create() {
    try {
        return new MpcHandle();
    } catch (const std::exception& e) {
        std::cerr << "[Error] mpc_create failed: " << e.what() << std::endl;
        return nullptr;
    }
}

void mpc_destroy(MpcHandle* handle) {
    if (handle) {
        delete handle;
    }
}

void mpc_step(MpcHandle* handle,
              double p, double v,
              double theta, double omega,
              double l, double dl,
              double psi, double dpsi,
              double v_ref,
              double l_ref,
              double omega_yaw,
              double p_ref,
              int hold_position,
              double* out_controls,
              double* solve_time_us) {
    if (!handle || !out_controls) return;
    balancer::State state{p, v, theta, omega, l, dl, psi, dpsi};
    balancer::Target target{v_ref, l_ref, omega_yaw, p_ref, (hold_position != 0)};
    balancer::Controls ctrl = handle->mpc.Step(state, target, solve_time_us);
    out_controls[0] = ctrl.tau_w;
    out_controls[1] = ctrl.F_l;
    out_controls[2] = ctrl.tau_roll;
    out_controls[3] = ctrl.psi_ref;
}

}
