#include <iostream>
#include <fstream>
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iomanip>

#include <mujoco/mujoco.h>
#include "acados_mpc_wrapper.hpp"

struct SimStepData {
    double time;
    double pos;
    double vel;
    double pitch;
    double pitch_rate;
    double roll;
    double leg_length;
    double tau_w;
    double F_l;
    double tau_roll;
    double solve_time_us;
};

int main(int argc, char** argv) {
    std::string model_path = "CAD/scene.xml";
    if (argc > 1) {
        model_path = argv[1];
    }

    std::cout << "========================================================\n";
    std::cout << "  Ascento-Inspired Wheeled Biped NMPC Benchmark         \n";
    std::cout << "  Coupled Non-Minimum Phase & ZMP Curve Banking         \n";
    std::cout << "========================================================\n";
    std::cout << "[Init] Loading MuJoCo model from: " << model_path << std::endl;

    char error[1000] = "Could not load XML model";
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
    if (!m) {
        std::cerr << "[Error] Failed to load MuJoCo XML: " << error << std::endl;
        return 1;
    }

    mjData* d = mj_makeData(m);
    if (!d) {
        std::cerr << "[Error] Failed to allocate MuJoCo data struct." << std::endl;
        mj_deleteModel(m);
        return 1;
    }

    // Calibrated Nominal Stance for Rev 3 (aligns Center of Mass directly over wheel axles)
    const double qh_R_nom = 0.314503;
    const double qk_R_nom = -0.052919;
    const double qh_L_nom = -0.212551;
    const double qk_L_nom = 0.253279;

    // Reset initial state on raised platform
    mj_resetData(m, d);
    d->qpos[0] = 0.0;
    d->qpos[1] = 0.0;
    d->qpos[2] = 0.1172; // Level wheels on platform (z = 0.08m)
    d->qpos[3] = 1.0;
    d->qpos[4] = 0.0;
    d->qpos[5] = 0.0;
    d->qpos[6] = 0.0;

    d->qpos[7] = 0.0;        // right_hip_roll
    d->qpos[8] = qh_R_nom;   // right_hip_pitch
    d->qpos[9] = qk_R_nom;   // right_knee
    d->qpos[10] = 0.0;       // right_wheel
    d->qpos[11] = 0.0;       // left_hip_roll
    d->qpos[12] = qh_L_nom;  // left_hip_pitch
    d->qpos[13] = qk_L_nom;  // left_knee
    d->qpos[14] = 0.0;       // left_wheel

    mj_forward(m, d);

    // Initialize acados NMPC Solver
    std::cout << "[MPC] Initializing acados Real-Time Iteration (SQP-RTI) solver...\n";
    balancer::AcadosMpcWrapper mpc;

    const double sim_duration = 8.0; // seconds
    const double dt_sim = m->opt.timestep;
    int total_steps = static_cast<int>(sim_duration / dt_sim);

    std::vector<SimStepData> trajectory;
    trajectory.reserve(total_steps);

    // Gains for leg joint coordination
    const double kp_leg = 75.0;
    const double kd_leg = 5.0;
    const double kp_roll = 85.0;
    const double kd_roll = 5.5;
    const double kd_knee_R = 4.5;
    const double kd_knee_L = 3.5;

    std::vector<double> solve_times;
    solve_times.reserve(total_steps);

    std::cout << "[Sim] Starting benchmark (" << sim_duration << "s at " << (1.0 / dt_sim) << " Hz physics)...\n";
    auto sim_start_wall = std::chrono::high_resolution_clock::now();

    for (int step_idx = 0; step_idx < total_steps; ++step_idx) {
        double sim_time = d->time;

        // Dynamic motion profile inspired by Ascento paper experiments:
        // 0.0 - 2.0s: Station-keeping upright at platform center (nominal l = 0.148m)
        // 2.0s:       Heavy horizontal impact disturbance (+1.5 rad/s poke, Fig. 8 in Ascento paper)
        // 2.0 - 4.0s: Shock absorption and rapid upright stabilization
        // 4.0 - 6.0s: Dynamic active suspension height modulation (crouch and stand, Fig. 9 in Ascento paper)
        // 6.0 - 8.0s: Dynamic turning maneuver with active ZMP roll banking (Fig. 10 in Ascento paper)
        double v_target = 0.0;
        double l_target = 0.148;
        double yaw_rate_target = 0.0;
        bool hold_pos = true;
        double target_p = -0.12;

        if (sim_time < 2.0) {
            v_target = 0.0;
            l_target = 0.148;
            hold_pos = true;
        } else if (sim_time < 4.0) {
            v_target = 0.0;
            l_target = 0.148;
            hold_pos = true;
        } else if (sim_time < 6.0) {
            v_target = 0.0;
            // Dynamic suspension modulation between 0.11m (crouch) and 0.18m (tall)
            l_target = 0.148 + 0.035 * std::sin(M_PI * (sim_time - 4.0));
            hold_pos = true;
        } else {
            v_target = 0.0;
            l_target = 0.148;
            yaw_rate_target = 0.80; // 0.8 rad/s dynamic turn
            hold_pos = true;
        }

        // Apply poke disturbance at t = 2.0s
        if (std::abs(sim_time - 2.0) < 0.5 * dt_sim) {
            std::cout << "  >>> [Disturbance t=" << sim_time << "s] Applied +1.5 rad/s horizontal impact pulse!\n";
            d->qvel[3] += 1.5;
        }

        // Extract 3D invariant states
        double p = d->qpos[1];
        double v = d->qvel[1];

        // 3D Invariant Coordinate Tracking using Base Rotation Matrix
        const double* R = &d->xmat[9]; // body_base rotation matrix (3x3)
        double theta = std::atan2(R[7], R[8]); // R[2, 1], R[2, 2]
        double omega_pitch = d->qvel[3];

        double psi = std::atan2(R[2], R[8]); // R[0, 2], R[2, 2]
        double dpsi = d->qvel[4];
        double omega_yaw = d->qvel[5];

        // Effective leg extension l
        double qk_avg = 0.5 * ((d->qpos[9] - qk_R_nom) - (d->qpos[13] - qk_L_nom));
        double l_curr = 0.148 - 0.08 * qk_avg;
        double dl_curr = -0.08 * 0.5 * (d->qvel[8] - d->qvel[12]);

        // Call Ascento-Inspired NMPC
        balancer::State state{p, v, theta, omega_pitch, l_curr, dl_curr, psi, dpsi};
        balancer::Target target{v_target, l_target, yaw_rate_target, target_p, hold_pos};

        double solve_us = 0.0;
        balancer::Controls ctrl = mpc.Step(state, target, &solve_us);
        solve_times.push_back(solve_us);

        // Map optimal MPC outputs [tau_w, F_l, tau_roll, psi_ref] to 8 actuators
        double tau_w = ctrl.tau_w;
        double F_l = ctrl.F_l;
        double tau_roll = ctrl.tau_roll;
        double psi_ref = ctrl.psi_ref;

        // Differential steering for yaw rate regulation
        double yaw_err = omega_yaw - yaw_rate_target;
        double tau_diff = - (2.0 * yaw_err + 0.30 * d->qvel[5]);
        tau_diff = std::clamp(tau_diff, -3.5, 3.5);

        // Suspension knee setpoints from target height + roll banking
        double qk_cmd_base = - (l_target - 0.148) / 0.08;
        double qk_roll_diff = 0.20 * psi_ref;

        double qk_R_cmd = qk_R_nom - (qk_cmd_base - qk_roll_diff);
        double qk_L_cmd = qk_L_nom - (qk_cmd_base + qk_roll_diff);

        double qh_R_cmd = qh_R_nom - 0.515 * (qk_cmd_base - qk_roll_diff);
        double qh_L_cmd = qh_L_nom + 0.705 * (qk_cmd_base + qk_roll_diff);

        // Actuator Torques:
        // 0: right_hip_roll
        d->ctrl[0] = -0.5 * tau_roll - kp_roll * (d->qpos[7] - (+psi_ref)) - kd_roll * d->qvel[6];
        // 1: right_hip_pitch
        d->ctrl[1] = -kp_leg * (d->qpos[8] - qh_R_cmd) - kd_leg * d->qvel[7];
        // 2: right_knee
        d->ctrl[2] = -kp_leg * (d->qpos[9] - qk_R_cmd) - kd_knee_R * d->qvel[8];
        // 3: right_wheel
        d->ctrl[3] = - 0.5 * tau_w - tau_diff;
        // 4: left_hip_roll
        d->ctrl[4] = +0.5 * tau_roll - kp_roll * (d->qpos[11] - (-psi_ref)) - kd_roll * d->qvel[10];
        // 5: left_hip_pitch
        d->ctrl[5] = -kp_leg * (d->qpos[12] - qh_L_cmd) - kd_leg * d->qvel[11];
        // 6: left_knee
        d->ctrl[6] = -kp_leg * (d->qpos[13] - qk_L_cmd) - kd_knee_L * d->qvel[12];
        // 7: left_wheel
        d->ctrl[7] = - 0.5 * tau_w + tau_diff;

        mj_step(m, d);

        if (step_idx % 250 == 0) {
            std::cout << "t=" << std::fixed << std::setprecision(2) << sim_time 
                      << "s | v=" << v << " (ref=" << v_target << ")"
                      << " | pitch=" << (theta * 180.0 / M_PI) << " deg"
                      << " | tau_w=" << tau_w
                      << " | F_l=" << F_l
                      << " | z=" << d->qpos[2]
                      << std::endl;
        }

        trajectory.push_back({
            d->time,
            p,
            v,
            theta,
            omega_pitch,
            psi,
            l_curr,
            tau_w,
            F_l,
            tau_roll,
            solve_us
        });
    }

    auto sim_end_wall = std::chrono::high_resolution_clock::now();
    double total_wall_ms = std::chrono::duration<double, std::milli>(sim_end_wall - sim_start_wall).count();

    double sum_solve = std::accumulate(solve_times.begin(), solve_times.end(), 0.0);
    double avg_solve_us = sum_solve / solve_times.size();
    double max_solve_us = *std::max_element(solve_times.begin(), solve_times.end());

    std::cout << "\n========================================================\n";
    std::cout << "         Ascento NMPC Benchmark Performance              \n";
    std::cout << "========================================================\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Simulated Time:         " << d->time << " s (" << total_steps << " physics steps)\n";
    std::cout << "Wall-Clock Duration:    " << total_wall_ms << " ms (" << (d->time * 1000.0 / total_wall_ms) << "x real-time)\n";
    std::cout << "SQP-RTI Average Solve:  " << avg_solve_us << " us (" << (1e6 / avg_solve_us) << " Hz max control rate)\n";
    std::cout << "SQP-RTI Peak Solve:     " << max_solve_us << " us\n";
    std::cout << "Final Robot Pos:        " << d->qpos[1] << " m\n";
    std::cout << "Final Forward Vel:      " << d->qvel[1] << " m/s\n";
    double final_sinp = 2.0 * (d->qpos[3] * d->qpos[4] + d->qpos[5] * d->qpos[6]);
    double final_pitch = std::asin(std::clamp(final_sinp, -1.0, 1.0));
    std::cout << "Final Pitch:            " << final_pitch << " rad (" << (final_pitch * 180.0 / M_PI) << " deg)\n";
    std::cout << "Final Up-Z:             " << (1.0 - 2.0 * (d->qpos[4]*d->qpos[4] + d->qpos[5]*d->qpos[5])) << "\n";
    std::cout << "========================================================\n";

    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}
