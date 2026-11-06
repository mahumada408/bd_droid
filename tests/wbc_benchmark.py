#!/usr/bin/env python3
"""Headless whole-body-controller benchmark / regression test.

Drives the WBC through a fixed set of scenarios the same way
run_interactive_sim.py calls it (filtered commands, wavy-runway lane keeping and
terrain roll offset), then prints metrics and checks them against limits.

  python3 tests/wbc_benchmark.py                 # default backend (C++ if built)
  python3 tests/wbc_benchmark.py --backend python
  python3 tests/wbc_benchmark.py --module controllers.wbc.wbc_controller_legacy --no-check
  python3 tests/wbc_benchmark.py --only push,yaw_push
  bazel test //:wbc_regression_test

Exit code is non-zero if any scenario breaks its limits (unless --no-check).
Scenarios marked `known_fail` are reported but never fail the run. A run counts
as a fall when the base tilts past 60 deg or any non-wheel body touches the
world; a safe-fall trigger in a normal scenario is a failure. `expect_fall`
scenarios (nominal mode only) instead require the safe fall to trigger before
impact and bound the impact speed, post-fall travel and wheel speed.

Robustness mode (--robust N): every scenario is run N times against a randomly
perturbed world while the controller keeps the nominal model:
  model     link masses +-15%, base CoM shift +-1 cm, ground friction 0.4-1.0,
            joint friction x0.5-2, joint damping 0-0.02 Nm s/rad
  sensing   controller sees its own nominal-model state built from noisy,
            delayed measurements: 0-4 ms latency, IMU tilt bias +-1 deg,
            attitude/gyro/joint/velocity noise, base position noise
  actuator  torque-speed droop (up to 50% at 40 rad/s wheels / 10 rad/s legs),
            0-5 ms torque lag, 0-5% cogging ripple, +-10% torque-constant error
Ranges above are --severity 1. Trial k uses the same perturbation in every
scenario, so one bad draw shows up as a column of failures. Reproduce a trial
with --robust 1 --seed <seed printed in the report> --only <scenario>.

  python3 tests/wbc_benchmark.py --robust 20                 # all categories
  python3 tests/wbc_benchmark.py --robust 20 --perturb sensing --severity 0.5
"""
import argparse
import importlib
import math
import os
import sys
import time
import zlib
from collections import deque
import multiprocessing as mp
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
import mujoco

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, ROOT)

HOME_LEGS = [0, 0.314503, -0.052919, 0, 0, -0.212551, 0.253279, 0]
FLOOR = (0.0, 2.0, 0.0372)       # flat floor, clear of the spawn platform
PLATFORM = (0.0, 0.0, 0.1172)    # 8 cm platform (spans y in [-0.8, 0.4])
X_LANE = 0.0705                  # wavy-runway centre line


def zero(_t):
    return 0.0


def step_at(t0, val):
    return lambda t: val if t > t0 else 0.0


@dataclass
class Scenario:
    name: str
    T: float
    v: Callable = zero
    yaw: Callable = zero
    dh: Callable = zero
    start: tuple = PLATFORM
    yaw0: float = 0.0                     # initial heading (rad); 0 faces +y
    scene: str = "scene.xml"
    wavy: bool = False
    push: Optional[tuple] = None          # (t0, t1, wrench[6] world frame on the base)
    limits: dict = field(default_factory=dict)
    known_fail: bool = False
    expect_fall: bool = False             # unrecoverable by design: checks the safe fall instead


# Limits sit ~1.5x above the C++ controller's measured values (2026-11).
SCENARIOS = [
    Scenario("stand", 4.0, limits=dict(pitch_rms=0.5, v_rmse=0.03)),
    Scenario("push", 4.0, push=(1.0, 1.1, [0, 6.0, 0, 0, 0, 0]),
             limits=dict(pitch_max=7.0)),
    Scenario("lat_push", 4.0, push=(1.0, 1.1, [4.0, 0, 0, 0, 0, 0]),
             limits=dict(roll_max=3.0)),
    # Facing -y: catches attitude estimates that are only valid at heading 0
    Scenario("roll_torque_back", 5.0, push=(1.0, 4.0, [0, 0, 0, 0, 0.25, 0]), start=FLOOR, yaw0=math.pi,
             limits=dict(roll_max=2.5)),
    Scenario("yaw_push", 4.0, push=(1.0, 1.1, [0, 0, 0, 0, 0, 0.4]),
             limits=dict(heading_final=3.0)),
    Scenario("drive0.5", 5.0, v=step_at(0.5, 0.5), start=FLOOR,
             limits=dict(v_rmse=0.045, heading_final=3.0)),
    Scenario("drive1.0", 5.0, v=step_at(0.5, 1.0), start=FLOOR, limits=dict(v_rmse=0.12)),
    Scenario("drive_yaw_push", 5.0, v=step_at(0.5, 0.5), start=FLOOR,
             push=(2.0, 2.1, [0, 0, 0, 0, 0, 0.4]), limits=dict(heading_final=3.0)),
    Scenario("drive+turn", 6.0, v=step_at(0.5, 0.5), yaw=step_at(2.0, 1.05), start=FLOOR,
             limits=dict(roll_max=3.0, yaw_rmse=0.03)),
    Scenario("spin", 4.0, yaw=step_at(0.5, 1.4), start=FLOOR, limits=dict(yaw_rmse=0.03)),
    Scenario("crouch", 4.0, dh=step_at(1.0, -0.03), limits=dict(pitch_rms=3.5)),
    Scenario("ledge_drop", 5.0, v=step_at(0.5, 0.4), limits=dict(roll_max=5.0)),
    Scenario("wavy", 8.0, v=lambda t: 0.35, start=(X_LANE, -0.8, 0.0772), scene="wavy_scene.xml",
             wavy=True, limits=dict(roll_max=8.0, lane_max=0.25)),
    Scenario("turn_long", 12.0, v=step_at(0.5, 0.5), yaw=step_at(1.0, 1.0), start=FLOOR,
             limits=dict(roll_max=3.0)),
    Scenario("fast_turn", 8.0, v=step_at(0.5, 1.0), yaw=step_at(2.0, 1.4), start=FLOOR,
             limits=dict(roll_max=3.0)),
    Scenario("crouch_drive", 6.0, v=step_at(1.0, 0.6), dh=lambda t: -0.04 * min(1.0, t), start=FLOOR,
             limits=dict(v_rmse=0.1)),
    Scenario("stand_long", 15.0, start=FLOOR, limits=dict(pitch_rms=0.5, hiproll_max=0.05)),
    Scenario("extreme", 8.0, v=step_at(0.5, 1.5), yaw=step_at(3.0, 1.8), start=FLOOR,
             limits=dict(roll_max=5.0)),
    Scenario("slalom", 10.0, v=step_at(0.5, 0.8),
             yaw=lambda t: 1.5 * np.sign(np.sin(1.5 * t)) if t > 2.0 else 0.0, start=FLOOR,
             limits=dict(roll_max=2.0)),
    Scenario("max_cmd", 8.0, v=step_at(0.5, 2.0), yaw=step_at(3.0, 1.8), start=FLOOR,
             known_fail=True),
    # Safe fall (nominal mode only): pushes past what the balance layer can
    # recover. The controller must detect the fall before the body hits the
    # ground, then brake the wheels (no runaway on the ground). fall_wheel_max is
    # the wheel speed from 0.5 s after the trigger on. Without the safe fall
    # these hit at 2.7-4.8 m/s with the wheels spinning at 70-540 rad/s.
    Scenario("fall_fwd", 4.0, push=(1.0, 1.15, [0, 14.0, 0, 0, 0, 0]), start=FLOOR, expect_fall=True,
             limits=dict(hit_v=3.0, fall_travel=0.6, fall_wheel_max=5.0)),
    Scenario("fall_back", 4.0, push=(1.0, 1.15, [0, -12.0, 0, 0, 0, 0]), start=FLOOR, expect_fall=True,
             limits=dict(hit_v=3.0, fall_travel=1.2, fall_wheel_max=5.0)),
    Scenario("fall_trip", 4.0, v=step_at(0.5, 1.0), push=(2.0, 2.3, [0, 0, 0, 3.0, 0, 0]), start=FLOOR,
             expect_fall=True, limits=dict(hit_v=2.5, fall_travel=1.3, fall_wheel_max=5.0)),
]
FALL_POST = 2.0     # s simulated after a fall in expect_fall scenarios


# --------------------------------------------------------------------------
# Perturbations (robustness mode)
# --------------------------------------------------------------------------
CATEGORIES = ("model", "sensing", "actuator")


def sample_perturbation(seed, severity, cats):
    """One random world. Values scale linearly with severity (1 = ranges in the
    module docstring, 0 = nominal)."""
    rng = np.random.default_rng(seed)
    U = lambda a: float(rng.uniform(-a, a))
    s = severity
    p = {"seed": seed}
    if "model" in cats:
        p["mass_scale"] = [1.0 + U(0.15 * s) for _ in range(32)]
        p["com_shift"] = [U(0.01 * s) for _ in range(3)]
        p["ground_mu"] = 1.0 - float(rng.uniform(0.0, 0.6 * s))
        p["fricloss_scale"] = float(np.exp(U(math.log(2.0) * s)))
        p["damping"] = float(rng.uniform(0.0, 0.02 * s))
    if "sensing" in cats:
        p["delay_steps"] = int(rng.integers(0, int(round(2 * s)) + 1))
        p["imu_bias"] = [U(math.radians(1.0) * s), U(math.radians(1.0) * s), 0.0]
        p["sig_att"] = 0.003 * s
        p["sig_gyro"] = 0.02 * s
        p["sig_vlin"] = 0.03 * s
        p["sig_pos"] = 0.001 * s
        p["sig_q"] = 0.001 * s
        p["sig_qd"] = 0.05 * s
    if "actuator" in cats:
        p["droop_wheel"] = float(rng.uniform(0.0, 0.5 * s))
        p["droop_leg"] = float(rng.uniform(0.0, 0.5 * s))
        p["lag_T"] = float(rng.uniform(0.0, 0.005 * s))
        p["ripple"] = float(rng.uniform(0.0, 0.05 * s))
        p["gain"] = [1.0 + U(0.1 * s) for _ in range(32)]
    return p


def describe(p):
    out = []
    if "ground_mu" in p:
        ms = np.array(p["mass_scale"][:9])
        out.append(f"mu={p['ground_mu']:.2f} mass x{ms.min():.2f}-{ms.max():.2f} "
                   f"com=({', '.join('%+.1f' % (1e3 * c) for c in p['com_shift'])})mm "
                   f"fric x{p['fricloss_scale']:.2f} damp={p['damping']:.3f}")
    if "delay_steps" in p:
        out.append(f"delay={p['delay_steps']}st imu_bias=({math.degrees(p['imu_bias'][0]):+.2f},"
                   f"{math.degrees(p['imu_bias'][1]):+.2f})deg")
    if "lag_T" in p:
        out.append(f"droop w/l={p['droop_wheel']:.2f}/{p['droop_leg']:.2f} lag={1e3 * p['lag_T']:.1f}ms "
                   f"ripple={100 * p['ripple']:.1f}%")
    return " | ".join(out)


def apply_model_perturbation(m, p):
    if "ground_mu" not in p:
        return
    for b in range(1, m.nbody):
        k = p["mass_scale"][(b - 1) % 32]
        m.body_mass[b] *= k
        m.body_inertia[b] *= k
    m.body_ipos[1] += p["com_shift"]
    m.geom_friction[:, 0] = p["ground_mu"]
    m.dof_frictionloss[:] *= p["fricloss_scale"]
    m.dof_damping[6:] += p["damping"]


class Sensor:
    """Builds the controller's view of the robot: delayed, noisy measurements
    written into a separate MjData on the NOMINAL model (so mass matrix, CoM and
    contacts all come from the controller's model, not the true world)."""

    def __init__(self, p, m_nom, seed):
        self.p, self.m = p, m_nom
        self.d = mujoco.MjData(m_nom)
        self.rng = np.random.default_rng(seed + 7919)
        self.buf = deque(maxlen=p.get("delay_steps", 0) + 1)
        self.qerr = np.zeros(4)
        self.qtmp = np.zeros(4)

    def measure(self, d_true):
        p, rng = self.p, self.rng
        self.buf.append((d_true.qpos.copy(), d_true.qvel.copy()))
        qp, qv = self.buf[0]
        qp, qv = qp.copy(), qv.copy()
        if "sig_att" in p:
            qp[0:3] += rng.normal(0, p["sig_pos"], 3)
            rv = np.array(p["imu_bias"]) + rng.normal(0, p["sig_att"], 3)
            ang = float(np.linalg.norm(rv))
            if ang > 0:
                mujoco.mju_axisAngle2Quat(self.qerr, rv / ang, ang)
                mujoco.mju_mulQuat(self.qtmp, self.qerr, qp[3:7])
                qp[3:7] = self.qtmp / np.linalg.norm(self.qtmp)
            qp[7:] += rng.normal(0, p["sig_q"], qp.size - 7)
            qv[0:3] += rng.normal(0, p["sig_vlin"], 3)
            qv[3:6] += rng.normal(0, p["sig_gyro"], 3)
            qv[6:] += rng.normal(0, p["sig_qd"], qv.size - 6)
        self.d.qpos[:] = qp
        self.d.qvel[:] = qv
        self.d.time = d_true.time
        mujoco.mj_forward(self.m, self.d)
        return self.d


class Actuators:
    """Turns commanded torques into applied torques: constant error, torque-speed
    droop, cogging ripple and a first-order lag."""

    def __init__(self, p, m):
        self.p = p
        self.dof = np.array([m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in range(m.nu)])
        self.qadr = np.array([m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in range(m.nu)])
        self.tmax = m.actuator_ctrlrange[:, 1].copy()
        wheel = np.array([m.jnt_type[m.actuator_trnid[i, 0]] == mujoco.mjtJoint.mjJNT_HINGE and
                          "wheel" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[i, 0]) or "")
                          for i in range(m.nu)])
        self.droop = np.where(wheel, p.get("droop_wheel", 0.0), p.get("droop_leg", 0.0))
        self.w_ref = np.where(wheel, 40.0, 10.0)
        self.gain = np.array(p.get("gain", [1.0] * 32)[:m.nu])
        self.state = None

    def apply(self, tau, d, dt):
        p = self.p
        tau = tau * self.gain
        lim = self.tmax * np.clip(1.0 - self.droop * np.abs(d.qvel[self.dof]) / self.w_ref, 0.0, 1.0)
        tau = np.clip(tau, -lim, lim)
        if p.get("ripple", 0.0):
            tau = tau * (1.0 + p["ripple"] * np.sin(14.0 * d.qpos[self.qadr]))
        T = p.get("lag_T", 0.0)
        if self.state is None:
            self.state = tau.copy()
        if T > 0:
            self.state += min(1.0, dt / T) * (tau - self.state)
        else:
            self.state = tau
        return self.state


# --------------------------------------------------------------------------
def run(sc: Scenario, WBC, ctor_kw, cad_dir, pert=None):
    path = os.path.join(cad_dir, sc.scene)
    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    m_nom = m
    sensor = act = None
    if pert is not None:
        m_nom = mujoco.MjModel.from_xml_path(path)      # controller keeps the nominal model
        apply_model_perturbation(m, pert)
        sensor = Sensor(pert, m_nom, pert["seed"] * 1000 + zlib.crc32(sc.name.encode()) % 1000)
        act = Actuators(pert, m)
    mujoco.mj_resetData(m, d)
    d.qpos[:3] = sc.start
    d.qpos[3:7] = [math.cos(sc.yaw0 / 2), 0, 0, math.sin(sc.yaw0 / 2)]
    d.qpos[7:15] = HOME_LEGS
    mujoco.mj_forward(m, d)
    c = WBC(m_nom, **ctor_kw)
    dt = m.opt.timestep
    vf = yf = 0.0
    log, ts = [], []
    fell = None
    yaw0 = None
    # Robot bodies that must never touch the world (all but the wheels): a
    # contact means a crash even when the base is not tilted past 60 deg
    # (e.g. lying wedged on its side on the wavy runway).
    is_world = m.body_rootid == 0
    hull = np.array([m.body_rootid[b] != 0 and "wheel" not in m.body(b).name for b in range(m.nbody)])
    safe_t = hit_t = hit_v = p_safe = None
    fall_travel = fall_wheel_max = 0.0
    wheel_dofs = [m.jnt_dofadr[j] for j in range(m.njnt) if "wheel" in m.joint(j).name]
    vel6 = np.zeros(6)
    t_end = sc.T
    while d.time < t_end - 0.5 * dt:
        t = d.time
        R = d.xmat[1].reshape(3, 3)
        theta = math.atan2(R[2, 1], R[2, 2])
        roll = math.atan2(-R[2, 0], R[2, 2])
        heading = math.atan2(R[1, 0], R[0, 0])
        if yaw0 is None:
            yaw0 = heading
        vfwd = float(d.qvel[0:3] @ R[:, 1])
        dc = sensor.measure(d) if sensor else d      # what the controller (and GUI logic) sees
        Rc = dc.xmat[1].reshape(3, 3)
        a = min(1.0, 4.0 * dt)                     # same command filter as the GUI
        vf += a * (sc.v(t) - vf)
        yf += a * (sc.yaw(t) - yf)
        yaw_cmd, roff = yf, 0.0
        if sc.wavy:                                # mirrors run_interactive_sim.py
            if abs(sc.yaw(t)) < 0.05:
                heading_c = math.atan2(Rc[1, 0], Rc[0, 0])
                heading_cmd = float(np.clip(1.2 * (dc.qpos[0] - X_LANE), -0.2, 0.2))
                yaw_cmd = float(np.clip(2.5 * (heading_cmd - heading_c), -0.6, 0.6))
            roll_c = math.atan2(-Rc[2, 0], Rc[2, 2])
            roff = float(np.clip(-(0.6 * roll_c + 0.05 * dc.qvel[4]), -0.25, 0.25))
        d.xfrc_applied[:] = 0
        if sc.push and sc.push[0] <= t < sc.push[1]:
            d.xfrc_applied[1] = sc.push[2]
        t0 = time.perf_counter()
        tau, info = c.step(dc, v_target=vf, yaw_target=yaw_cmd, delta_h=sc.dh(t),
                           roll_target_offset=roff, dt=dt)
        if safe_t is None and info.get("safe_fall"):
            safe_t, p_safe = t, d.qpos[:2].copy()
        ts.append(time.perf_counter() - t0)
        d.ctrl[:] = act.apply(tau, d, dt) if act else tau
        mujoco.mj_step(m, d)
        log.append((t, theta, roll, vfwd, vf, d.qvel[5], yf, d.qpos[7], d.qpos[11],
                    math.remainder(heading - yaw0, 2 * math.pi), d.qpos[0]))
        if hit_t is None:
            for i in range(d.ncon):
                b1, b2 = m.geom_bodyid[d.contact.geom[i]]
                if (hull[b1] and is_world[b2]) or (hull[b2] and is_world[b1]):
                    hit_t = d.time
                    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, 1, vel6, 0)
                    hit_v = float(np.linalg.norm(vel6[3:]))
                    break
        if p_safe is not None:
            fall_travel = max(fall_travel, float(np.linalg.norm(d.qpos[:2] - p_safe)))
            if d.time > safe_t + 0.5:          # after the brake has had time to act
                fall_wheel_max = max(fall_wheel_max, float(np.abs(d.qvel[wheel_dofs]).max()))
        if fell is None and (R[2, 2] < 0.5 or hit_t is not None):
            fell = t
            if not sc.expect_fall:
                break
            t_end = d.time + FALL_POST
    L = np.array(log)
    late = L[:, 0] > 1.0
    if not late.any():
        late[:] = True
    th, ro = L[:, 1], L[:, 2]
    us = np.array(ts) * 1e6
    return dict(
        fell=fell,
        pitch_rms=math.degrees(np.sqrt(np.mean(th[late] ** 2))),
        pitch_max=math.degrees(np.abs(th).max()),
        roll_max=math.degrees(np.abs(ro).max()),
        v_rmse=float(np.sqrt(np.mean((L[:, 3] - L[:, 4])[late] ** 2))),
        yaw_rmse=float(np.sqrt(np.mean((L[:, 5] - L[:, 6])[late] ** 2))),
        hiproll_max=float(np.abs(L[:, 7:9]).max()),
        heading_final=math.degrees(abs(L[-1, 9])),
        lane_max=float(np.abs(L[:, 10] - X_LANE).max()),
        safe_fall=safe_t,
        hit_t=hit_t,
        hit_v=hit_v if hit_v is not None else 0.0,
        fall_travel=fall_travel,
        fall_wheel_max=fall_wheel_max,
        us_mean=float(us.mean()),
        us_p99=float(np.percentile(us, 99)),
    )


def check(sc, r):
    bad = []
    if sc.expect_fall:
        if r["safe_fall"] is None:
            bad.append("no safe fall" + (" (did not fall)" if r["fell"] is None else ""))
        elif r["hit_t"] is not None and r["safe_fall"] > r["hit_t"]:
            bad.append(f"safe fall at {r['safe_fall']:.2f}s after impact at {r['hit_t']:.2f}s")
    else:
        if r["fell"] is not None:
            bad.append(f"fell at {r['fell']:.2f}s")
        if r["safe_fall"] is not None:
            bad.append(f"safe fall triggered at {r['safe_fall']:.2f}s")
    bad += [f"{k}={r[k]:.3g}>{v}" for k, v in sc.limits.items() if r[k] > v]
    return bad


_W = {}


def _worker_init(module, ctor_kw, cad):
    _W["WBC"] = importlib.import_module(module).WholeBodyController
    _W["kw"], _W["cad"] = ctor_kw, cad


def _robust_task(job):
    name, trial, seed, severity, cats = job
    sc = next(s for s in SCENARIOS if s.name == name)
    pert = sample_perturbation(seed, severity, cats)
    r = run(sc, _W["WBC"], _W["kw"], _W["cad"], pert)
    return name, trial, seed, r


def run_robust(args, scen, ctor_kw):
    cats = CATEGORIES if args.perturb == "all" else tuple(args.perturb.split(","))
    for c_ in cats:
        if c_ not in CATEGORIES:
            raise SystemExit(f"unknown perturbation category {c_!r}")
    seeds = [args.seed + k for k in range(args.robust)]
    jobs = [(sc.name, k, seeds[k], args.severity, cats) for sc in scen for k in range(args.robust)]
    print(f"[robust] {len(jobs)} runs | perturb={','.join(cats)} severity={args.severity} "
          f"seeds {seeds[0]}..{seeds[-1]} | {args.jobs} workers", flush=True)
    t0 = time.time()
    res = {}
    with mp.get_context("fork").Pool(args.jobs, _worker_init, (args.module, ctor_kw, args.cad)) as pool:
        for name, k, seed, r in pool.imap_unordered(_robust_task, jobs):
            res[(name, k)] = r
    print(f"[robust] done in {time.time() - t0:.0f}s\n")

    print(f"{'scenario':<16} {'survive':>8} {'in-limits':>9}   worst: pitch  roll   v_rmse  heading   fell (trial)")
    fails_by_trial = {}
    total_fell = 0
    for sc in scen:
        rs = [res[(sc.name, k)] for k in range(args.robust)]
        fell = [k for k, r in enumerate(rs) if r["fell"] is not None]
        okl = sum(1 for r in rs if not check(sc, r))
        total_fell += 0 if sc.known_fail else len(fell)
        for k in fell:
            fails_by_trial.setdefault(k, []).append(sc.name)
        print(f"{sc.name:<16} {100 * (1 - len(fell) / len(rs)):7.0f}% {100 * okl / len(rs):8.0f}%   "
              f"{max(r['pitch_max'] for r in rs):10.1f}° {max(r['roll_max'] for r in rs):5.1f}° "
              f"{max(r['v_rmse'] for r in rs):7.3f} {max(r['heading_final'] for r in rs):6.1f}°   "
              + " ".join(str(k) for k in fell)
              + ("  (known-fail)" if sc.known_fail else ""))
    if fails_by_trial:
        print("\nTrials with falls (worst first):")
        for k, names in sorted(fails_by_trial.items(), key=lambda kv: -len(kv[1])):
            p = sample_perturbation(seeds[k], args.severity, cats)
            print(f"  trial {k:2d} seed {seeds[k]:5d}  falls={len(names):2d}  {describe(p)}")
            print(f"      {', '.join(names)}")
    n = sum(1 for sc in scen if not sc.known_fail) * args.robust
    print(f"\n[robust] survival {100 * (1 - total_fell / max(n, 1)):.1f}% "
          f"({total_fell} falls in {n} runs, known-fail scenarios excluded)")
    return 1 if (args.min_survival and 100 * (1 - total_fell / max(n, 1)) < args.min_survival) else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--module", default="controllers.wbc.wbc_controller",
                    help="module providing WholeBodyController")
    ap.add_argument("--backend", choices=["auto", "cpp", "python"], default=None,
                    help="WBC backend (new controller only)")
    ap.add_argument("--only", default="", help="comma-separated scenario names")
    ap.add_argument("--no-check", action="store_true", help="report only, never fail")
    ap.add_argument("--cad", default=os.environ.get("CAD_DIR", os.path.join(ROOT, "CAD")))
    ap.add_argument("--robust", type=int, default=0, metavar="N", help="randomized trials per scenario")
    ap.add_argument("--perturb", default="all", help="all, or comma list of model,sensing,actuator")
    ap.add_argument("--severity", type=float, default=1.0, help="perturbation range scale (1 = nominal ranges)")
    ap.add_argument("--seed", type=int, default=0, help="first trial seed")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--min-survival", type=float, default=0.0,
                    help="robust mode: fail (exit 1) below this survival percentage")
    args = ap.parse_args()

    WBC = importlib.import_module(args.module).WholeBodyController
    ctor_kw = {"backend": args.backend} if args.backend else {}
    only = {s for s in args.only.split(",") if s}
    scen = [s for s in SCENARIOS if not only or s.name in only]
    if only - {s.name for s in scen}:
        ap.error(f"unknown scenario(s): {sorted(only - {s.name for s in scen})}")

    probe = None
    if hasattr(WBC, "backend") or "backend" in ctor_kw:
        m0 = mujoco.MjModel.from_xml_path(os.path.join(args.cad, "scene.xml"))
        probe = getattr(WBC(m0, **ctor_kw), "backend", None)
    print(f"[bench] {args.module}" + (f" (backend: {probe})" if probe else ""))

    if args.robust > 0:
        return run_robust(args, [s for s in scen if not s.expect_fall], ctor_kw)

    failures = []
    for sc in scen:
        r = run(sc, WBC, ctor_kw, args.cad)
        bad = check(sc, r)
        verdict = "ok" if not bad else ("known-fail" if sc.known_fail else "FAIL")
        if sc.expect_fall:
            f = lambda v: "%.2fs" % v if v is not None else "none"
            print(f"{sc.name:<16} {verdict:<10} safe_fall={f(r['safe_fall']):<6} impact={f(r['hit_t']):<6} "
                  f"impact_v={r['hit_v']:.2f}m/s travel={r['fall_travel']:.2f}m "
                  f"wheel_after={r['fall_wheel_max']:.1f}rad/s"
                  + (f"   [{'; '.join(bad)}]" if bad else ""), flush=True)
            if bad:
                failures.append(sc.name)
            continue
        print(f"{sc.name:<16} {verdict:<10} "
              f"fell={'%.2fs' % r['fell'] if r['fell'] is not None else 'no':<6} "
              f"pitch rms/max={r['pitch_rms']:4.2f}/{r['pitch_max']:4.1f}° "
              f"roll_max={r['roll_max']:4.1f}° v_rmse={r['v_rmse']:.3f} "
              f"yaw_rmse={r['yaw_rmse']:.3f} heading_end={r['heading_final']:5.1f}° "
              f"hiproll={r['hiproll_max']:.2f} step={r['us_mean']:.0f}/{r['us_p99']:.0f}us"
              + (f"   [{'; '.join(bad)}]" if bad else ""), flush=True)
        if bad and not sc.known_fail:
            failures.append(sc.name)
    if failures:
        print(f"[bench] FAILED: {', '.join(failures)}")
        return 0 if args.no_check else 1
    print("[bench] all scenarios within limits")
    return 0


if __name__ == "__main__":
    sys.exit(main())
