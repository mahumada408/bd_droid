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
              double qh, double dqh,
              double qk, double dqk,
              double target_pos,
              double target_qh,
              double target_qk,
              double* out_controls,
              double* solve_time_us) {
    if (!handle || !out_controls) return;
    balancer::State state{p, v, theta, omega, qh, dqh, qk, dqk};
    balancer::Controls ctrl = handle->mpc.Step(state, target_pos, target_qh, target_qk, solve_time_us);
    out_controls[0] = ctrl.tau_w;
    out_controls[1] = ctrl.tau_h;
    out_controls[2] = ctrl.tau_k;
}

}
