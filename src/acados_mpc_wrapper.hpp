#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <array>
#include <chrono>

#include "acados_solver_biped_balancer.h"

namespace balancer {

struct State {
    double p = 0.0;      // Forward position (m)
    double v = 0.0;      // Forward velocity (m/s)
    double theta = 0.0;  // Pitch angle from upright (rad)
    double omega = 0.0;  // Pitch angular velocity (rad/s)
    double qh = 0.0;     // Symmetrical hip angle error (rad)
    double dqh = 0.0;    // Symmetrical hip angular rate (rad/s)
    double qk = 0.0;     // Symmetrical knee angle error (rad)
    double dqk = 0.0;    // Symmetrical knee angular rate (rad/s)
};

struct Controls {
    double tau_w = 0.0;  // Wheel torque (Nm)
    double tau_h = 0.0;  // Hip torque (Nm)
    double tau_k = 0.0;  // Knee torque (Nm)
};

class AcadosMpcWrapper {
public:
    AcadosMpcWrapper() {
        capsule_ = biped_balancer_acados_create_capsule();
        if (!capsule_) {
            throw std::runtime_error("Failed to allocate acados solver capsule.");
        }

        int status = biped_balancer_acados_create(capsule_);
        if (status != 0) {
            biped_balancer_acados_free_capsule(capsule_);
            throw std::runtime_error("Failed to create acados solver instance, error code: " + std::to_string(status));
        }

        config_ = biped_balancer_acados_get_nlp_config(capsule_);
        dims_ = biped_balancer_acados_get_nlp_dims(capsule_);
        in_ = biped_balancer_acados_get_nlp_in(capsule_);
        out_ = biped_balancer_acados_get_nlp_out(capsule_);
    }

    ~AcadosMpcWrapper() {
        if (capsule_) {
            biped_balancer_acados_free(capsule_);
            biped_balancer_acados_free_capsule(capsule_);
            capsule_ = nullptr;
        }
    }

    // Disable copy
    AcadosMpcWrapper(const AcadosMpcWrapper&) = delete;
    AcadosMpcWrapper& operator=(const AcadosMpcWrapper&) = delete;

    // Enable move
    AcadosMpcWrapper(AcadosMpcWrapper&& other) noexcept
        : capsule_(other.capsule_), config_(other.config_), dims_(other.dims_),
          in_(other.in_), out_(other.out_) {
        other.capsule_ = nullptr;
    }

    AcadosMpcWrapper& operator=(AcadosMpcWrapper&& other) noexcept {
        if (this != &other) {
            if (capsule_) {
                biped_balancer_acados_free(capsule_);
                biped_balancer_acados_free_capsule(capsule_);
            }
            capsule_ = other.capsule_;
            config_ = other.config_;
            dims_ = other.dims_;
            in_ = other.in_;
            out_ = other.out_;
            other.capsule_ = nullptr;
        }
        return *this;
    }

    /**
     * @brief Computes optimal control inputs given current state and target references.
     * @param current_state Current [p, v, theta, omega, qh, dqh, qk, dqk]
     * @param target_pos Target forward displacement (m)
     * @param target_qh Target hip angle error (default: 0.0)
     * @param target_qk Target knee angle error (default: 0.0)
     * @param solve_time_us Optional pointer to receive solver solve time in microseconds.
     * @return Controls containing {tau_w, tau_h, tau_k}.
     */
    Controls Step(const State& current_state,
                  double target_pos = 0.0,
                  double target_qh = 0.0,
                  double target_qk = 0.0,
                  double* solve_time_us = nullptr) {
        // 1. Inject current state x0 into bounds at stage 0
        double x0[BIPED_BALANCER_NX] = {
            current_state.p,
            current_state.v,
            current_state.theta,
            current_state.omega,
            current_state.qh,
            current_state.dqh,
            current_state.qk,
            current_state.dqk
        };
        ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "ubx", x0);

        // 2. Set references: yref = [target_pos, 0, 0, 0, target_qh, 0, target_qk, 0, 0, 0, 0] for stages 0..N-1
        double yref[BIPED_BALANCER_NY] = {
            target_pos, 0.0, 0.0, 0.0,
            target_qh, 0.0, target_qk, 0.0,
            0.0, 0.0, 0.0
        };
        for (int i = 0; i < BIPED_BALANCER_N; ++i) {
            ocp_nlp_cost_model_set(config_, dims_, in_, i, "yref", yref);
        }

        // Terminal reference: yref_e = [target_pos, 0, 0, 0, target_qh, 0, target_qk, 0]
        double yref_e[BIPED_BALANCER_NYN] = {
            target_pos, 0.0, 0.0, 0.0,
            target_qh, 0.0, target_qk, 0.0
        };
        ocp_nlp_cost_model_set(config_, dims_, in_, BIPED_BALANCER_N, "yref", yref_e);

        // 3. Solve single SQP-RTI iteration
        auto t_start = std::chrono::high_resolution_clock::now();
        int status = biped_balancer_acados_solve(capsule_);
        auto t_end = std::chrono::high_resolution_clock::now();

        if (status != 0 && status != 2) { // 0: ACADOS_SUCCESS, 2: ACADOS_MAXITER
            std::cerr << "[Warning] acados solve returned status: " << status << std::endl;
        }

        if (solve_time_us) {
            *solve_time_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
        }

        // 4. Extract optimal controls u_0* = [tau_w, tau_h, tau_k]
        double u_opt[BIPED_BALANCER_NU] = {0.0, 0.0, 0.0};
        ocp_nlp_out_get(config_, dims_, out_, 0, "u", u_opt);

        return Controls{u_opt[0], u_opt[1], u_opt[2]};
    }

private:
    biped_balancer_solver_capsule* capsule_{nullptr};
    ocp_nlp_config* config_{nullptr};
    ocp_nlp_dims* dims_{nullptr};
    ocp_nlp_in* in_{nullptr};
    ocp_nlp_out* out_{nullptr};
};

} // namespace balancer
