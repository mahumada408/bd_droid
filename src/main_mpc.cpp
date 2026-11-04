#include <iostream>
#include <fstream>
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iomanip>

#include <mujoco/mujoco.h>

struct SimStepData {
    double time;
    double pos;
    double vel;
    double pitch;
    double pitch_rate;
    double tau_w;
};

int main(int argc, char** argv) {
    std::string model_path = "CAD/scene.xml";
    if (argc > 1) {
        model_path = argv[1];
    }

    std::cout << "========================================================\n";
    std::cout << "  Wheeled Biped / Acrobat Balancer Benchmark            \n";
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

    // Reset initial state on raised platform (z = 0.08m)
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

    // Initial pitch disturbance pulse at t = 2.0s
    mj_forward(m, d);

    const double sim_duration = 10.0; // seconds
    int step_idx = 0;

    const double kp_th = 50.0;
    const double kd_th = 7.0;
    const double kp_p = 0.35;
    const double kd_p = 0.30;
    const double ki_p = 0.05;
    const double max_tilt = 0.10;

    const double kp_leg = 50.0;
    const double kd_leg = 4.0;
    const double kp_roll = 60.0;
    const double kd_roll = 4.5;

    std::vector<SimStepData> trajectory;
    trajectory.reserve(static_cast<size_t>(sim_duration / m->opt.timestep));

    double target_pos = 0.0;
    double p_int = 0.0;

    std::cout << "[Sim] Starting benchmark (" << sim_duration << "s at 500 Hz physics)...\n";
    auto sim_start_wall = std::chrono::high_resolution_clock::now();

    while (d->time < sim_duration) {
        // Apply poke disturbance at t = 2.0s
        if (step_idx == 1000) {
            d->qvel[3] += 1.5; // +1.5 rad/s poke
        }

        double p = d->qpos[1];
        double v = d->qvel[1];

        double qw = d->qpos[3];
        double qx = d->qpos[4];
        double qy = d->qpos[5];
        double qz = d->qpos[6];
        double sinp = 2.0 * (qw * qx + qy * qz);
        double theta = std::asin(std::clamp(sinp, -1.0, 1.0));
        double omega = d->qvel[3];

        double p_err = p - target_pos;
        p_int += p_err * m->opt.timestep;
        p_int = std::clamp(p_int, -0.5, 0.5);

        double th_ref = kp_p * p_err + kd_p * v + ki_p * p_int;
        th_ref = std::clamp(th_ref, -max_tilt, max_tilt);

        double tau_wheel = - (kp_th * (theta - th_ref) + kd_th * omega);
        tau_wheel = std::clamp(tau_wheel, -15.0, 15.0);

        // Apply torques to 8 actuators:
        // 0: right_hip_roll
        d->ctrl[0] = -kp_roll * (d->qpos[7] - 0.0) - kd_roll * d->qvel[6];
        // 1: right_hip_pitch
        d->ctrl[1] = -kp_leg * (d->qpos[8] - qh_R_nom) - kd_leg * d->qvel[7];
        // 2: right_knee
        d->ctrl[2] = -kp_leg * (d->qpos[9] - qk_R_nom) - kd_leg * d->qvel[8];
        // 3: right_wheel
        d->ctrl[3] = 0.5 * tau_wheel;
        // 4: left_hip_roll
        d->ctrl[4] = -kp_roll * (d->qpos[11] - 0.0) - kd_roll * d->qvel[10];
        // 5: left_hip_pitch
        d->ctrl[5] = -kp_leg * (d->qpos[12] - qh_L_nom) - kd_leg * d->qvel[11];
        // 6: left_knee
        d->ctrl[6] = -kp_leg * (d->qpos[13] - qk_L_nom) - kd_leg * d->qvel[12];
        // 7: left_wheel
        d->ctrl[7] = 0.5 * tau_wheel;

        mj_step(m, d);

        trajectory.push_back({
            d->time,
            p,
            v,
            theta,
            omega,
            tau_wheel
        });

        step_idx++;
    }

    auto sim_end_wall = std::chrono::high_resolution_clock::now();
    double total_wall_ms = std::chrono::duration<double, std::milli>(sim_end_wall - sim_start_wall).count();

    std::cout << "\n========================================================\n";
    std::cout << "                  Simulation Results                     \n";
    std::cout << "========================================================\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Simulated Time:       " << d->time << " s (" << step_idx << " physics steps)\n";
    std::cout << "Wall-Clock Duration:  " << total_wall_ms << " ms (" << (d->time * 1000.0 / total_wall_ms) << "x real-time)\n";
    std::cout << "Final Robot Pos:      " << d->qpos[1] << " m (Target: " << target_pos << " m)\n";
    double final_sinp = 2.0 * (d->qpos[3] * d->qpos[4] + d->qpos[5] * d->qpos[6]);
    double final_pitch = std::asin(std::clamp(final_sinp, -1.0, 1.0));
    std::cout << "Final Pitch:          " << final_pitch << " rad (" << (final_pitch * 180.0 / M_PI) << " deg)\n";
    std::cout << "Final Forward Vel:    " << d->qvel[1] << " m/s\n";
    std::cout << "Final Pitch Rate:     " << d->qvel[3] << " rad/s\n";
    std::cout << "========================================================\n";

    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}
