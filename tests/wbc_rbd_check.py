#!/usr/bin/env python3
"""Check the C++ core's standalone dynamics (src/wbc/rbd.cpp) against MuJoCo.

Random poses (including tilted bases) and velocities; compares the mass
matrix, bias forces, point Jacobian, Jdot*qvel, whole-body CoM and its
Jacobian. Also fails if src/wbc/robot_model.h is stale.

  python3 tests/wbc_rbd_check.py               # exit 1 on mismatch
  python3 tests/wbc_rbd_check.py --cad CAD_bd  # another robot (its own libwbc_*.so)
"""
import argparse
import ctypes
import os
import sys

import numpy as np
import mujoco

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, ROOT)
from controllers.wbc.wbc_controller import _lib_model, _load_libs  # noqa: E402

TOL = 1e-9   # relative to the largest entry of each quantity


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cad", default=os.path.join(ROOT, "CAD"))
    args = ap.parse_args()
    libs = _load_libs()
    if not libs:
        import controllers.wbc.wbc_controller as W
        print(f"[rbd] libwbc.so unavailable: {W._LIB_ERR}")
        return 1
    m = mujoco.MjModel.from_xml_path(os.path.join(args.cad, "scene.xml"))
    d = mujoco.MjData(m)
    lib = next((l for _, l in libs if _lib_model(l)[0] == (m.nq, m.nv, m.nu, m.nbody)
                and abs(_lib_model(l)[1] - float(np.sum(m.body_mass))) <= 1e-9), None)
    if lib is None:
        print(f"[rbd] FAIL: no controller library matches {args.cad}/scene.xml (generated model is stale); "
              "run python3 tools/gen_robot_model.py and rebuild")
        return 1
    print(f"[rbd] {args.cad}: {m.nbody} bodies, nv={m.nv}")
    nv = m.nv
    P = lambda a: a.ctypes.data_as(ctypes.c_void_p)
    rng = np.random.default_rng(0)
    worst = {}
    home = m.key_qpos[0] if m.nkey else np.r_[0, 0, 0.1, 1, 0, 0, 0, np.zeros(m.nq - 7)]
    for trial in range(300):
        d.qpos[:] = home
        d.qpos[:3] += rng.normal(0, 0.1, 3)
        quat = rng.normal(size=4)
        d.qpos[3:7] = quat / np.linalg.norm(quat) if trial % 2 else [1, 0, 0, 0]
        d.qpos[7:] += rng.uniform(-0.5, 0.5, m.nq - 7)
        d.qvel[:] = rng.normal(0, 2, nv)
        mujoco.mj_forward(m, d)
        b = int(rng.integers(1, m.nbody))
        pt = d.xpos[b] + rng.normal(0, 0.05, 3)
        M, h, J, jd = np.zeros((nv, nv)), np.zeros(nv), np.zeros((3, nv)), np.zeros(3)
        com, Jcom = np.zeros(3), np.zeros((3, nv))
        q, qd = d.qpos.copy(), d.qvel.copy()
        lib.wbc_rbd_eval(P(q), P(qd), b, P(pt), P(M), P(h), P(J), P(jd), P(com), P(Jcom))
        Mr, Jr, Jdr, Jcr = np.zeros((nv, nv)), np.zeros((3, nv)), np.zeros((3, nv)), np.zeros((3, nv))
        mujoco.mj_fullM(m, d, Mr)
        mujoco.mj_jac(m, d, Jr, None, pt, b)
        mujoco.mj_jacDot(m, d, Jdr, None, pt, b)
        mujoco.mj_jacSubtreeCom(m, d, Jcr, 1)
        ref = {"mass matrix": (M, Mr), "bias forces": (h, d.qfrc_bias), "point Jacobian": (J, Jr),
               "Jdot*qvel": (jd, Jdr @ d.qvel), "CoM": (com, d.subtree_com[1]), "CoM Jacobian": (Jcom, Jcr)}
        for k, (a, r) in ref.items():
            e = np.abs(a - r).max() / max(1e-9, np.abs(r).max())
            worst[k] = max(worst.get(k, 0.0), e)
    bad = False
    for k, v in worst.items():
        flag = "ok" if v < TOL else "FAIL"
        bad |= v >= TOL
        print(f"[rbd] {k:<15} max rel error {v:.1e}  {flag}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
