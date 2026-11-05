#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <array>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "acados_solver_biped_balancer.h"

namespace balancer {

struct State {
    double p = 0.0;      // Forward position along path (m)
    double v = 0.0;      // Forward velocity (m/s)
    double theta = 0.0;  // Pitch angle from upright (rad)
    double omega = 0.0;  // Pitch angular velocity (rad/s)
    double l = 0.148;    // Effective pendulum leg length (m)
    double dl = 0.0;     // Leg extension rate (m/s)
    double psi = 0.0;    // Roll lean angle (rad)
    double dpsi = 0.0;   // Roll angular rate (rad/s)
};

struct Target {
    double v_ref = 0.0;          // Target forward velocity (m/s)
    double l_ref = 0.148;        // Target leg height (m)
    double omega_yaw = 0.0;      // Current yaw angular rate (rad/s)
    double p_ref = 0.0;          // Target position (m)
    bool hold_position = false;  // Whether to hold stationary position
};

struct Controls {
    double tau_w = 0.0;     // Symmetric wheel drive torque (Nm)
    double F_l = 19.06;     // Leg thrust force (N)
    double tau_roll = 0.0;  // Lateral roll torque (Nm)
    double psi_ref = 0.0;   // Computed ZMP-optimal roll lean angle (rad)
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
     * @brief Computes dynamic MPC control inputs given current state and target references.
     * @param state Current [p, v, theta, omega, l, dl, psi, dpsi]
     * @param target Target references {v_ref, l_ref, omega_yaw, p_ref, hold_position}
     * @param solve_time_us Optional pointer to receive solver solve time in microseconds.
     * @return Controls containing {tau_w, F_l, tau_roll, psi_ref}.
     */
    Controls Step(const State& state,
                  const Target& target,
                  double* solve_time_us = nullptr) {
        // 1. Inject current state x0 into bounds at stage 0
        double x0[BIPED_BALANCER_NX] = {
            state.p,
            state.v,
            state.theta,
            state.omega,
            state.l,
            state.dl,
            state.psi,
            state.dpsi
        };
        ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(config_, dims_, in_, 0, "ubx", x0);

        // 2. Compute ZMP-optimal active roll lean angle (Eth Zurich Ascento Sec III-B3)
        // psi_ref = arctan(v * omega_yaw / g)
        const double g = 9.81;
        double a_lat = target.v_ref * target.omega_yaw;
        double psi_ref = std::atan2(a_lat, g);
        psi_ref = std::clamp(psi_ref, -0.40, 0.40); // clamp to +/- 23 deg

        // 3. Update online parameters (omega_yaw) across all shooting stages
        double p_val[BIPED_BALANCER_NP] = {target.omega_yaw};
        for (int i = 0; i <= BIPED_BALANCER_N; ++i) {
            biped_balancer_acados_update_params(capsule_, i, p_val, BIPED_BALANCER_NP);
        }

        // 4. Update stage references: yref = [p, v, theta, omega, l, dl, psi, dpsi, tau_w, F_l, tau_roll] (ny = 11)
        const double dt_stage = 0.40 / BIPED_BALANCER_N;
        for (int i = 0; i < BIPED_BALANCER_N; ++i) {
            double p_stage = target.hold_position ? target.p_ref : (state.p + target.v_ref * (i * dt_stage));
            double yref[BIPED_BALANCER_NY] = {
                p_stage,
                target.v_ref,
                0.0,
                0.0,
                target.l_ref,
                0.0,
                psi_ref,
                0.0,
                0.0,
                19.06, // Nominal gravity-canceling thrust force F_l = M*g
                0.0
            };
            ocp_nlp_cost_model_set(config_, dims_, in_, i, "yref", yref);
        }

        // Terminal reference: yref_e = [p, v, theta, omega, l, dl, psi, dpsi] (ny_e = 8)
        double p_end = target.hold_position ? target.p_ref : (state.p + target.v_ref * 0.40);
        double yref_e[BIPED_BALANCER_NYN] = {
            p_end,
            target.v_ref,
            0.0,
            0.0,
            target.l_ref,
            0.0,
            psi_ref,
            0.0
        };
        ocp_nlp_cost_model_set(config_, dims_, in_, BIPED_BALANCER_N, "yref", yref_e);

        // 5. Solve single SQP-RTI iteration
        auto t_start = std::chrono::high_resolution_clock::now();
        int status = biped_balancer_acados_solve(capsule_);
        auto t_end = std::chrono::high_resolution_clock::now();

        if (status != 0 && status != 2) { // 0: ACADOS_SUCCESS, 2: ACADOS_MAXITER
            std::cerr << "[Warning] acados solve returned status: " << status << std::endl;
        }

        if (solve_time_us) {
            *solve_time_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
        }

        // 6. Extract optimal controls u_0* = [tau_w, F_l, tau_roll]
        double u_opt[BIPED_BALANCER_NU] = {0.0, 18.15, 0.0};
        ocp_nlp_out_get(config_, dims_, out_, 0, "u", u_opt);

        return Controls{u_opt[0], u_opt[1], u_opt[2], psi_ref};
    }

private:
    biped_balancer_solver_capsule* capsule_{nullptr};
    ocp_nlp_config* config_{nullptr};
    ocp_nlp_dims* dims_{nullptr};
    ocp_nlp_in* in_{nullptr};
    ocp_nlp_out* out_{nullptr};
};

} // namespace balancer
