"""
acados C++ NMPC Wrapper Controller Module.
"""

import os
import ctypes
import numpy as np


class NMPCController:
    def __init__(self, lib_path: str = None):
        self.handle = None
        self.lib = None
        self.out_ctrl = (ctypes.c_double * 4)()
        self.solve_time = ctypes.c_double()

        candidates = [
            lib_path,
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../bazel-bin/libbalancer_mpc.so"),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../libbalancer_mpc.so"),
            "bazel-bin/libbalancer_mpc.so",
            "libbalancer_mpc.so",
        ]
        if "RUNFILES_DIR" in os.environ:
            candidates.insert(0, os.path.join(os.environ["RUNFILES_DIR"], "_main", "libbalancer_mpc.so"))

        for path in candidates:
            if path and os.path.isfile(path):
                try:
                    lib = ctypes.CDLL(path)
                    lib.mpc_create.restype = ctypes.c_void_p
                    lib.mpc_destroy.argtypes = [ctypes.c_void_p]
                    lib.mpc_step.argtypes = [
                        ctypes.c_void_p,
                        ctypes.c_double, ctypes.c_double,
                        ctypes.c_double, ctypes.c_double,
                        ctypes.c_double, ctypes.c_double,
                        ctypes.c_double, ctypes.c_double,
                        ctypes.c_double,
                        ctypes.c_double,
                        ctypes.c_double,
                        ctypes.c_double,
                        ctypes.c_int,
                        ctypes.POINTER(ctypes.c_double),
                        ctypes.POINTER(ctypes.c_double)
                    ]
                    handle = lib.mpc_create()
                    if handle:
                        self.lib = lib
                        self.handle = handle
                        self.path = path
                        break
                except Exception:
                    pass

        if self.handle is None:
            raise RuntimeError("Could not load libbalancer_mpc.so. Ensure it is built via `bazel build //:libbalancer_mpc.so`.")

    def __del__(self):
        if self.lib and self.handle:
            self.lib.mpc_destroy(self.handle)
            self.handle = None

    def step(self, p_curr: float, v_curr: float, theta_mpc: float, omega_mpc: float,
             l_curr: float, dl_curr: float, roll_curr: float, roll_rate_curr: float,
             v_target: float, l_target: float, yaw_target: float) -> tuple:
        self.lib.mpc_step(
            self.handle,
            p_curr, v_curr,
            theta_mpc, omega_mpc,
            l_curr, dl_curr,
            roll_curr, roll_rate_curr,
            float(v_target),
            float(l_target),
            float(yaw_target),
            p_curr,  # Relative receding horizon
            0,
            self.out_ctrl,
            ctypes.byref(self.solve_time)
        )
        tau_w = self.out_ctrl[0]
        F_l = self.out_ctrl[1]
        tau_roll = self.out_ctrl[2]
        psi_ref = self.out_ctrl[3]
        return tau_w, F_l, tau_roll, psi_ref, self.solve_time.value
