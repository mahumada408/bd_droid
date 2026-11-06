#!/usr/bin/env python3
"""Generate CAD_bd/robot.xml: MuJoCo model of the BD-style droid (CAD/bd_droid_rs05.fs, slim inboard-knee layout).

The geometry, joint axes and mass layout come from the same numbers as the Onshape
FeatureScript. Inertias are built from primitive mass elements (motors, battery,
printed parts with their estimated print mass). The model follows the WBC contract
in HANDOFF_BD_REDESIGN.md:

  * body 1 = body_base (freejoint, IMU), drives along +y, +z up
  * wheel bodies `wheel` / `wheel_2`, joints right_/left_ hip_roll, hip_pitch, knee, wheel
  * qpos[7:15] / ctrl[0:8] keep the old leg layout; the 3 neck joints come after them
  * every joint is actuated (motor, gear 1); wheel radius 0.05 m
  * leg bodies are pre-rotated by -HOME_LEGS about their joint axes, so the benchmark's
    hard-coded HOME_LEGS put the robot in the as-designed (CAD) pose (qpos0 stays 0,
    as the C++ model generator requires)
  * base origin sits 0.0372 m above the ground plane at home, so the existing
    FLOOR / PLATFORM / wavy start heights stay valid
  * the wheel goes fore-aft under the whole-body CoM (solved here), base level

Run:  python3 CAD_bd/gen_robot.py        (writes CAD_bd/robot.xml and prints the balance check)
"""
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MM = 1e-3

# ---------------------------------------------------------------- geometry (mm, CAD frame)
HZ = 210.0          # hip pitch axis height
R = 50.0            # wheel radius
LK = 105.0          # thigh = shin length
RZ = HZ + 45        # hip roll axis height
TZ0 = HZ + 18       # torso bottom
TZ1 = HZ + 159      # torso top (lid underside)
TYF = 88.0          # torso front face (rear face -70)
TYC = (TYF - 70.0) / 2   # torso fore-aft centre
WC = 195.0          # wheel (tyre) centre, |x|
KNEE_D = 49.343     # knee-motor axis distance from the hip, along the thigh
LT = TZ1 + 4        # lid top
NY = -6.0           # neck axis y
PZ = LT + 41 + 46.5 - 11.25          # neck pitch axis
CB0 = PZ + 20                        # pitch-bracket crossbar
RZN = CB0 + 3 + 46.5 - 11.25         # neck roll axis
HZ0 = LT + 95                        # head bottom
HZ1 = HZ0 + 72                       # head top
ZB = 37.2           # base origin height above ground at home (keeps benchmark start heights)

HOME_LEGS = [0, 0.314503, -0.052919, 0, 0, -0.212551, 0.253279, 0]
# Joint ranges relative to home. Crouch is -knee / +hip. The extension side is kept short so the
# controller's posture table tops out near +0.015 m (handoff section 3), like the original robot.
HIP_RANGE = (-0.45, 0.75)
KNEE_RANGE = (-1.15, 0.40)

RS05_MASS = 0.191
XL430_MASS = 0.0572
BATTERY_MASS = 0.71  # Makita XGT BL4025 40V max 2.5 Ah (36 V nominal), 118 x 77 x 65 mm

# Knee: the RS05 sits inboard of the thigh, beside the hip-pitch motor, and drives the knee through an
# 18T:36T Gates 5MGT-25 belt, 5MR-250-25 (CAD/bd_droid_rs05.fs).
KNEE_RATIO = 36.0 / 18.0
RS05_PEAK = 5.5                                  # N m at the motor output
RS05_ARMATURE = 0.005                            # reflected rotor inertia at the RS05 output (kg m^2)
KNEE_PEAK = RS05_PEAK * KNEE_RATIO               # 11 N m at the knee joint
KNEE_ARMATURE = RS05_ARMATURE * KNEE_RATIO ** 2  # 0.02 kg m^2


# ---------------------------------------------------------------- inertia helpers
def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def box_I(m, dims):
    a, b, c = (d * MM for d in dims)
    return m / 12.0 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])


def cyl_I(m, r, L, axis, r_in=0.0):
    r, L, r_in = r * MM, L * MM, r_in * MM
    Ia = 0.5 * m * (r * r + r_in * r_in)
    It = m * (3 * (r * r + r_in * r_in) + L * L) / 12.0
    d = {"x": 0, "y": 1, "z": 2}[axis]
    I = np.diag([It, It, It])
    I[d, d] = Ia
    return I


class Body:
    def __init__(self):
        self.el = []        # (mass, com[m, CAD frame], inertia about own com, CAD axes)

    def box(self, m, c, dims, R=np.eye(3), scale=1.0):
        self.el.append((m, np.array(c) * MM, scale * (R @ box_I(m, dims) @ R.T)))

    def cyl(self, m, c, r, L, axis, r_in=0.0):
        self.el.append((m, np.array(c) * MM, cyl_I(m, r, L, axis, r_in)))

    def point(self, m, c):
        self.el.append((m, np.array(c) * MM, np.zeros((3, 3))))

    def total(self):
        M = sum(e[0] for e in self.el)
        com = sum(e[0] * e[1] for e in self.el) / M
        I = np.zeros((3, 3))
        for m, c, Ic in self.el:
            r = c - com
            I += Ic + m * (r @ r * np.eye(3) - np.outer(r, r))
        return M, com, I


def leg_points(e):
    """Hip (0,HZ) -> wheel (e,R), knee forward. Returns knee (y, z)."""
    hy, hz, wy, wz = 0.0, HZ, e, R
    dy, dz = wy - hy, wz - hz
    d = math.hypot(dy, dz)
    h = math.sqrt(max(LK * LK - (d / 2) ** 2, 0.0))
    ny, nz = -dz / d, dy / d              # left normal of hip->wheel; points forward (+y) for a downward leg
    if ny < 0:
        ny, nz = -ny, -nz
    return (hy + wy) / 2 + h * ny, (hz + wz) / 2 + h * nz


def knee_motor(ky, kz):
    """Knee-motor axis (y, z): on the thigh axis, KNEE_D from the hip."""
    L = math.hypot(ky, kz - HZ)
    return KNEE_D * ky / L, HZ + KNEE_D * (kz - HZ) / L


def link_R(y1, z1, y2, z2):
    """Rotation taking the +z axis onto the (y,z) segment direction."""
    return rot_x(math.atan2(-(y2 - y1), z2 - z1))


# ---------------------------------------------------------------- mass model
def build_bodies(e):
    ky, kz = leg_points(e)
    B = {}

    zb = HZ + 79                                # battery underside (slides in from the back onto the rail dock)
    b = Body()                                  # body_base
    b.box(0.29, (0, TYC, (TZ0 + TZ1) / 2), (120, 158, TZ1 - TZ0), scale=1.6)   # torso shell (hollow -> scaled)
    b.box(0.045, (0, TYC, TZ1 + 2), (120, 158, 4))                     # lid
    b.box(0.02, (0, TYF + 2, HZ + 100), (52, 4, 54))                   # chest details
    b.box(0.06, (0, -6, zb - 2), (110, 121, 4))                        # battery shelf
    b.box(BATTERY_MASS, (0, -11, zb + 32.5), (77, 118, 65))            # Makita XGT 40V pack
    b.box(0.11, (0, -11, zb + 65 + 7.5), (90, 112, 15))                # XGT rail dock + carrier
    b.box(0.05, (0, TYF - 22, TZ0 + 72), (70, 18, 133))                # NUCLEO-H743ZI, vertical on the front wall
    b.box(0.07, (-37, 15, TZ0 + 15), (36, 70, 20))                     # power / CAN carrier (DC-DC x2, transceiver)
    b.box(0.04, (-4, -4, TZ0 + 12), (12, 28, 12))                      # fuse, battery leads
    for s in (1, -1):
        b.cyl(RS05_MASS, (s * 26, -45, RZ), 23, 44, "y")               # hip-roll motors
    b.box(0.12, (0, TYC, TZ0 + 70), (110, 150, 130))                   # wiring, fasteners, inserts
    b.box(0.03, (0, NY + 6, LT + 17), (36, 55, 35), scale=1.6)         # neck collar
    b.box(XL430_MASS, (0, NY + 12, LT + 18), (28.5, 46.5, 34))         # yaw XL430 (stator)
    B["body_base"] = b

    for s, side in ((1, "R"), (-1, "L")):
        h = Body()                                                     # hip_roll_*
        h.box(0.045, (s * 40, -75.5, (HZ - 24 + RZ + 24) / 2), (54, 8, 93))
        h.box(0.025, (s * 64, -26.75, HZ - 6), (6, 105, 36))
        h.cyl(RS05_MASS, (s * 89, 0, HZ), 23, 44, "x")                 # hip-pitch motor
        B["hip_" + side] = h

        my, mz = knee_motor(ky, kz)
        f = Body()                                                     # femur*
        Rl = link_R(0, HZ, ky, kz)
        f.box(0.15, (s * 135, ky * 0.55, HZ + (kz - HZ) * 0.55), (42, 68, LK + 60), R=Rl, scale=1.3)  # thigh + cover (hollow)
        f.cyl(RS05_MASS, (s * 92, my, mz), 23, 44, "x")                # knee motor, inboard beside the hip-pitch motor
        f.cyl(0.06, (s * 136, my, mz), 17, 36, "x")                    # 18T pulley, shaft adapter, 6800 + carrier
        f.box(0.025, (s * 135.5, (my + ky) / 2, (mz + kz) / 2), (25, 40, LK - KNEE_D), R=Rl)   # 5MR-250-25 belt
        f.point(0.012, (s * 130, ky, kz))                              # 688 + 6706 knee bearings
        B["femur_" + side] = f

        t = Body()                                                     # tibia* (outboard of the thigh cover)
        Rs = link_R(ky, kz, e, R)
        t.box(0.045, (s * 162, (ky + e) / 2, (kz + R) / 2), (8, 40, LK), R=Rs)
        t.cyl(0.15, (s * 188, e, R), 23, 44, "x")                      # wheel-motor stator
        t.cyl(0.05, (s * 140, ky, kz), 28, 36, "x")                    # knee hub: 36T pulley + D30 boss
        t.cyl(0.01, (s * 167.5, ky, kz), 20, 3, "x")                   # knee cap
        B["tibia_" + side] = t

        w = Body()                                                     # wheel*
        w.cyl(0.09, (s * WC, e, R), 50, 36, "x", r_in=35)              # TPU tyre
        w.cyl(0.06, (s * (WC + 2), e, R), 35, 36, "x", r_in=31)        # hub + rim
        w.cyl(0.04, (s * 204, e, R), 15, 10, "x")                      # motor rotor + output
        B["wheel_" + side] = w

    y = Body()                                                         # neck_yaw
    y.cyl(0.01, (0, NY, LT + 39.5), 19, 3, "z")
    y.box(XL430_MASS, (0, NY, LT + 64.25), (34, 28.5, 46.5))
    B["neck_yaw"] = y

    p = Body()                                                         # neck_pitch
    p.box(0.02, (0, NY, PZ + 14), (46, 28, 30))
    p.box(XL430_MASS, (0, NY, CB0 + 26.25), (28.5, 34, 46.5))
    B["neck_pitch"] = p

    hd = Body()                                                        # head
    hd.box(0.18, (0, 1, HZ0 + 36), (176, 81, 72), scale=1.6)
    hd.point(0.04, (0, 42, HZ0 + 34))                                  # eyes, ear pods, bands, antennas
    hd.box(0.02, (0, NY, HZ1 - 20), (32, 46, 30))                      # head mount
    B["head"] = hd
    return B, (ky, kz)


def solve_balance():
    """Fore-aft wheel offset e such that the whole-body CoM is over the axle."""
    e = 0.0
    for _ in range(30):
        B, _ = build_bodies(e)
        M = sum(b.total()[0] for b in B.values())
        com_y = sum(b.total()[0] * b.total()[1][1] for b in B.values()) / M / MM
        if abs(com_y - e) < 1e-4:
            break
        e = com_y
    return e


# ---------------------------------------------------------------- XML emission
def axq(axis, home):
    """Body quat that pre-rotates by -home about the joint axis, so that qpos = home gives
    the as-designed (CAD-aligned) pose. Used instead of joint `ref`, which the C++ model
    generator (tools/gen_robot_model.py) does not support."""
    c, s_ = math.cos(-home / 2), math.sin(-home / 2)
    return (c, s_, 0, 0) if axis == "x" else (c, 0, s_, 0)


def f(*v):
    return " ".join(f"{x:.10g}" for x in v)


def inertial(b, origin):
    """origin: body frame origin in CAD metres (body frames are axis-aligned with CAD)."""
    M, com, I = b.total()
    c = com - origin
    return (f'<inertial pos="{f(*c)}" mass="{M:.5g}" '
            f'fullinertia="{f(I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2])}"/>')


def P(*xyz):
    return np.array(xyz) * MM


def geom_box(cls, c, half, origin, mat=None, extra=""):
    m = f' material="{mat}"' if mat else ""
    return f'<geom class="{cls}" type="box" pos="{f(*(P(*c) - origin))}" size="{f(*(np.array(half) * MM))}"{m}{extra}/>'


def geom_cyl(cls, a, b, r, origin, mat=None, kind="cylinder"):
    m = f' material="{mat}"' if mat else ""
    return (f'<geom class="{cls}" type="{kind}" fromto="{f(*(P(*a) - origin), *(P(*b) - origin))}" '
            f'size="{r * MM:.6g}"{m}/>')


def link_geom(cls, x, y1, z1, y2, z2, half_w, half_t, origin, mat):
    """Box along a (y,z) segment."""
    L = math.hypot(y2 - y1, z2 - z1)
    a = math.atan2(-(y2 - y1), z2 - z1)
    q = (math.cos(a / 2), math.sin(a / 2), 0, 0)
    c = P(x, (y1 + y2) / 2, (z1 + z2) / 2) - origin
    return (f'<geom class="{cls}" type="box" pos="{f(*c)}" quat="{f(*q)}" '
            f'size="{f(half_t * MM, half_w * MM, L / 2 * MM)}" material="{mat}"/>')


def main():
    e = solve_balance()
    B, (ky, kz) = build_bodies(e)
    base_o = P(0, 0, ZB)
    out = []
    w = out.append
    w('<mujoco model="bd_droid_rs05">')
    w('  <!-- Generated by CAD_bd/gen_robot.py from CAD/bd_droid_rs05.fs. Do not edit by hand. -->')
    w('  <compiler angle="radian" meshdir="../CAD/assets" autolimits="true"/>')
    w('  <default>')
    w('    <default class="CAD">')
    w('      <joint frictionloss="0.1" armature="0.005"/>')
    w('      <default class="neck"><joint frictionloss="0.05" armature="0.002" damping="0.05"/></default>')
    w('      <default class="visual"><geom contype="0" conaffinity="0" group="2" density="0"/></default>')
    w('      <!-- robot collides with the world only (contype 2 never matches another robot geom) -->')
    w('      <default class="collision"><geom contype="2" conaffinity="1" group="3" density="0" rgba="1 0 0 0.3"/></default>')
    w('    </default>')
    w('  </default>')
    w('  <asset>')
    for n, rgba in (("white", ".86 .83 .76 1"), ("orange", ".85 .38 .14 1"), ("dark", ".22 .22 .23 1"),
                    ("motor", ".12 .12 .13 1"), ("tyre", ".08 .08 .08 1"), ("grey", ".55 .56 .58 1"),
                    ("lens", ".1 .3 .85 1"), ("battery", ".15 .17 .2 1")):
        w(f'    <material name="{n}" rgba="{rgba}"/>')
    w('    <material name="eye_glow" rgba=".2 .5 1 1" emission="1"/>')
    w('  </asset>')
    w('  <worldbody>')
    w(f'    <body name="body_base" pos="0 0 {ZB * MM:.4f}" childclass="CAD">')
    w('      <freejoint name="root"/>')
    w('      <camera name="track_robot" mode="trackcom" pos="0 -1.6 0.6" xyaxes="1 0 0 0 0.35 0.94"/>')
    w('      ' + inertial(B["body_base"], base_o))
    o = base_o
    # torso visuals: narrow box, lid, chest details on the front wall, battery end in the rear opening
    cz = HZ + 100
    zb = HZ + 79
    w('      ' + geom_box("visual", (0, TYC, (TZ0 + TZ1) / 2), (60, 79, (TZ1 - TZ0) / 2), o, "white"))
    w('      ' + geom_box("visual", (0, TYC, TZ1 + 2), (60, 79, 2), o, "white"))
    w('      ' + geom_box("visual", (0, TYF + 2, cz), (26, 2, 27), o, "dark"))
    w('      ' + geom_box("visual", (0, TYF + 1, (cz + 33 + TZ1 - 5) / 2), (12, 1, (TZ1 - 5 - cz - 33) / 2), o, "orange"))
    for s in (1, -1):
        w('      ' + geom_box("visual", (s * 42, TYF + 1, cz), (4, 1, 27), o, "orange"))
    w('      ' + geom_box("visual", (0, -70.5, zb + 26.5), (38.5, 1, 26.5), o, "battery"))
    w('      ' + geom_box("visual", (0, NY + 6, LT + 17.5), (18, 27.5, 17.5), o, "orange"))
    w('      ' + geom_box("collision", (0, TYC, (TZ0 + TZ1) / 2), (60, 79, (TZ1 - TZ0) / 2), o))
    w('      ' + geom_box("collision", (0, NY + 6, LT + 17.5), (18, 27.5, 17.5), o))
    w('      <site name="imu" pos="0 0 0" size="0.005"/>')

    for s, side, jn in ((1, "R", "right"), (-1, "L", "left")):
        names = {"hip": f"hip_roll_{jn}", "femur": "femur" if s > 0 else "femur_2",
                 "tibia": "tibia" if s > 0 else "tibia_2", "wheel": "wheel" if s > 0 else "wheel_2"}
        q = HOME_LEGS[0:4] if s > 0 else HOME_LEGS[4:8]
        oh = P(s * 26, -70, RZ)
        w(f'      <body name="{names["hip"]}" pos="{f(*(oh - o))}" quat="{f(*axq("y", q[0]))}">')
        w(f'        <joint name="{jn}_hip_roll" axis="0 1 0" range="-0.6 0.6"/>')
        w('        ' + inertial(B["hip_" + side], oh))
        w('        ' + geom_box("visual", (s * 40, -75.5, (HZ - 24 + RZ + 24) / 2), (27, 4, 46.5), oh, "white"))
        w('        ' + geom_box("visual", (s * 64, -26.75, HZ - 6), (3, 52.75, 18), oh, "white"))
        w('        ' + geom_cyl("visual", (s * 67, 0, HZ), (s * 114, 0, HZ), 23, oh, "motor"))
        w('        ' + geom_box("collision", (s * 40, -75.5, (HZ - 24 + RZ + 24) / 2), (27, 4, 46.5), oh))
        w('        ' + geom_box("collision", (s * 64, -26.75, HZ - 6), (3, 52.75, 18), oh))
        w('        ' + geom_cyl("collision", (s * 67, 0, HZ), (s * 111, 0, HZ), 23, oh))
        of = P(s * 120, 0, HZ)
        w(f'        <body name="{names["femur"]}" pos="{f(*(of - oh))}" quat="{f(*axq("x", q[1]))}">')
        w(f'          <joint name="{jn}_hip_pitch" axis="1 0 0" range="{q[1] + HIP_RANGE[0]:.6f} {q[1] + HIP_RANGE[1]:.6f}"/>')
        w('          ' + inertial(B["femur_" + side], of))
        my, mz = knee_motor(ky, kz)
        w('          ' + link_geom("visual", s * 135, 0, HZ, ky, kz, 30, 21, of, "white"))
        w('          ' + geom_cyl("visual", (s * 114, 0, HZ), (s * 156, 0, HZ), 30, of, "white"))
        w('          ' + geom_cyl("visual", (s * 114, ky, kz), (s * 156, ky, kz), 38, of, "white"))
        w('          ' + geom_cyl("visual", (s * 156, my, mz), (s * 156.5, my, mz), 14, of, "grey"))
        w('          ' + geom_cyl("visual", (s * 70, my, mz), (s * 114, my, mz), 23, of, "motor"))
        w('          ' + geom_cyl("collision", (s * 135, 0, HZ), (s * 135, ky, kz), 30, of, kind="capsule"))
        w('          ' + geom_cyl("collision", (s * 70, my, mz), (s * 114, my, mz), 23, of))
        ot = P(s * 108, ky, kz)
        w(f'          <body name="{names["tibia"]}" pos="{f(*(ot - of))}" quat="{f(*axq("x", q[2]))}">')
        w(f'            <joint name="{jn}_knee" axis="1 0 0" range="{q[2] + KNEE_RANGE[0]:.6f} {q[2] + KNEE_RANGE[1]:.6f}" '
          f'armature="{KNEE_ARMATURE:.6g}" frictionloss="0.15"/>')   # belt reduction: reflected inertia x ratio^2
        w('            ' + inertial(B["tibia_" + side], ot))
        w('            ' + link_geom("visual", s * 162, ky, kz, e, R, 22, 4, ot, "white"))
        w('            ' + geom_cyl("visual", (s * 158, ky, kz), (s * 166, ky, kz), 22, ot, "white"))
        w('            ' + geom_cyl("visual", (s * 158, e, R), (s * 166, e, R), 28, ot, "white"))
        w('            ' + geom_cyl("visual", (s * 166, ky, kz), (s * 169, ky, kz), 20, ot, "orange"))
        w('            ' + geom_cyl("visual", (s * 166, e, R), (s * 210, e, R), 23, ot, "motor"))
        # shin collision stops 40 mm above the axle; the wheel motor sits inside the wheel silhouette
        sy, sz = e + (ky - e) * 40 / LK, R + (kz - R) * 40 / LK
        w('            ' + geom_cyl("collision", (s * 162, ky, kz), (s * 162, sy, sz), 18, ot, kind="capsule"))
        ow = P(s * WC, e, R)
        w(f'            <body name="{names["wheel"]}" pos="{f(*(ow - ot))}">')
        w(f'              <joint name="{jn}_wheel" axis="1 0 0"/>')
        w('              ' + inertial(B["wheel_" + side], ow))
        w('              ' + geom_cyl("visual", (s * (WC - 18), e, R), (s * (WC + 18), e, R), 50, ow, "tyre"))
        w('              ' + geom_cyl("visual", (s * 213, e, R), (s * 217, e, R), 34, ow, "grey"))
        w('              ' + geom_cyl("visual", (s * 217, e, R), (s * 221, e, R), 13, ow, "orange"))
        w('              ' + geom_cyl("collision", (s * (WC - 18), e, R), (s * (WC + 18), e, R), 50, ow))
        w('            </body>')
        w('          </body>')
        w('        </body>')
        w('      </body>')

    # ---- 3-DOF neck (declared after the legs so qpos[7:15] / ctrl[0:8] keep their layout)
    oy = P(0, NY, LT + 38)
    w(f'      <body name="neck_yaw" pos="{f(*(oy - o))}">')
    w('        <joint name="neck_yaw" class="neck" axis="0 0 1" range="-1.5 1.5"/>')
    w('        ' + inertial(B["neck_yaw"], oy))
    w('        ' + geom_cyl("visual", (0, NY, LT + 38), (0, NY, LT + 41), 19, oy, "white"))
    w('        ' + geom_box("visual", (0, NY, LT + 64.25), (17, 14.25, 23.25), oy, "dark"))
    w('        ' + geom_box("collision", (0, NY, LT + 64.25), (17, 14.25, 23.25), oy))
    op = P(0, NY, PZ)
    w(f'        <body name="neck_pitch" pos="{f(*(op - oy))}">')
    w('          <joint name="neck_pitch" class="neck" axis="1 0 0" range="-0.5 0.5"/>')
    w('          ' + inertial(B["neck_pitch"], op))
    w('          ' + geom_box("visual", (21.5, NY, PZ + 11.5), (1.5, 12, 11.5), op, "white"))
    w('          ' + geom_box("visual", (-21.5, NY, PZ + 11.5), (1.5, 12, 11.5), op, "white"))
    w('          ' + geom_box("visual", (0, NY, CB0 + 1.5), (23, 14, 1.5), op, "white"))
    w('          ' + geom_box("visual", (0, NY, CB0 + 26.25), (14.25, 17, 23.25), op, "dark"))
    w('          ' + geom_box("collision", (0, NY, CB0 + 13), (23, 17, 13), op))
    oh = P(0, NY, RZN)
    w(f'          <body name="head" pos="{f(*(oh - op))}">')
    w('            <joint name="neck_roll" class="neck" axis="0 1 0" range="-0.35 0.35"/>')
    w('            ' + inertial(B["head"], oh))
    w('            ' + geom_box("visual", (0, 1, HZ0 + 36), (88, 39, 36), oh, "white"))
    ez = HZ0 + 34
    w('            ' + geom_cyl("visual", (42, 40, ez), (42, 46, ez), 23, oh, "orange"))
    w('            ' + geom_cyl("visual", (42, 40, ez), (42, 46.5, ez), 17, oh, "eye_glow"))
    w('            ' + geom_cyl("visual", (-42, 40, ez), (-42, 48, ez), 18, oh, "grey"))
    w('            ' + geom_cyl("visual", (-42, 48, ez), (-42, 52, ez), 12, oh, "dark"))
    w('            ' + geom_box("visual", (0, 41, ez), (14, 1, 10), oh, "dark"))
    w('            ' + geom_box("visual", (0, 40.2, HZ1 - 10), (20, 1, 4), oh, "orange"))
    for s in (1, -1):
        w('            ' + geom_cyl("visual", (s * 88, 2, ez), (s * 96, 2, ez), 20, oh, "orange"))
        w('            ' + geom_box("visual", (s * 51, 0, HZ1 + 0.5), (11, 40.5, 1), oh, "orange"))
        w('            ' + geom_cyl("visual", (s * 66, -24, HZ1), (s * 76, -30, HZ1 + 75), 1.5, oh, "grey"))
    w('            ' + geom_box("collision", (0, 1, HZ0 + 36), (88, 39, 36), oh))
    w('          </body>')
    w('        </body>')
    w('      </body>')
    w('    </body>')
    w('  </worldbody>')
    w('  <actuator>')
    for jn in ("right", "left"):
        for j in ("hip_roll", "hip_pitch", "knee", "wheel"):
            # RS05 peak 5.5 N m; the knee sees it through the 1.6:1 belt (joint torque, gear 1)
            lim = KNEE_PEAK if j == "knee" else RS05_PEAK
            w(f'    <motor name="{jn}_{j}" joint="{jn}_{j}" gear="1" ctrlrange="{-lim:.4g} {lim:.4g}"/>')
    for j in ("neck_yaw", "neck_pitch", "neck_roll"):
        w(f'    <motor name="{j}" joint="{j}" gear="1" ctrlrange="-1.4 1.4"/>')                  # XL430 stall 1.4 N m @ 11.1 V
    w('  </actuator>')
    home = [0, 0, 0.1172, 1, 0, 0, 0] + HOME_LEGS + [0, 0, 0]
    w('  <keyframe>')
    w(f'    <key name="home" qpos="{f(*home)}"/>')
    w('  </keyframe>')
    w('</mujoco>')
    path = os.path.join(HERE, "robot.xml")
    with open(path, "w") as fh:
        fh.write("\n".join(out) + "\n")

    tot = sum(b.total()[0] for b in B.values())
    print(f"wrote {path}")
    print(f"wheel fore-aft offset under CoM e = {e:.2f} mm, knee at y={ky:.1f} z={kz:.1f} mm")
    print(f"total mass {tot:.3f} kg  (head+neck {sum(B[k].total()[0] for k in ('neck_yaw', 'neck_pitch', 'head')):.3f} kg)")


if __name__ == "__main__":
    main()
