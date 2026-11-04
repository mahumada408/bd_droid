#!/usr/bin/env python3
"""
MuJoCo Simulation Runner for the CAD Model (CAD/scene.xml).
Launches an interactive viewer window with real-time physics stepping.
"""

import os
import sys
import time
import argparse
import mujoco
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(description="Run MuJoCo simulation for CAD model")
    parser.add_argument(
        "--model",
        type=str,
        default="CAD/scene.xml",
        help="Path to MuJoCo MJCF XML file (default: CAD/scene.xml)",
    )
    args = parser.parse_args()

    xml_path = os.path.abspath(args.model)
    if not os.path.isfile(xml_path):
        print(f"Error: Model file '{xml_path}' not found.")
        sys.exit(1)

    print(f"[MuJoCo] Loading model from: {xml_path}")
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)

    print("\n--- Model Summary ---")
    print(f"Model Name:       {m.names.decode('utf-8')[:30] if m.names else 'Unnamed'}")
    print(f"Time Step:        {m.opt.timestep} s")
    print(f"Generalized QPos: {m.nq}")
    print(f"Generalized QVel: {m.nv}")
    print(f"Actuators (nu):   {m.nu}")
    print(f"Bodies:           {m.nbody}")
    print(f"Joints:           {m.njnt}")

    print("\nJoints:")
    for j_id in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j_id)
        jnt_type = m.jnt_type[j_id]
        type_str = ["free", "ball", "slide", "hinge"][jnt_type]
        print(f"  [{j_id}] {name} (type: {type_str})")

    print("\nActuators:")
    for a_id in range(m.nu):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a_id)
        print(f"  [{a_id}] {name}")

    print("\nStarting interactive viewer...")
    print("Controls:")
    print("  - Right-Click + Drag: Perturb / apply forces to bodies")
    print("  - Left-Click + Drag:  Rotate camera")
    print("  - Scroll:             Zoom camera")
    print("  - Space:              Pause / Resume simulation")
    print("  - Backspace:          Reset simulation")
    print("  - ESC / Close window: Exit\n")

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(m, d)
            viewer.sync()

            # Maintain real-time simulation speed
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
