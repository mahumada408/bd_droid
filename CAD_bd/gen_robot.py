#!/usr/bin/env python3
"""Generate CAD_bd/robot.xml: MuJoCo model of the BD-style droid v3 (CAD/bd_droid_rs05.fs).

v3 layout (after inspiration_robot.mp4), all RS05 direct drive, gimbal hip:
  * the hip-roll (fore-aft axis) and hip-pitch (lateral axis) axes intersect at the hip point,
    inside the lower torso, 33 mm above the torso bottom.
  * hip roll: RS05 clamped at both ends (stator ring on the mid bulkhead, rear face on the rear wall).
  * hip pitch: RS05 inside a printed gimbal frame; the frame is bolted to the roll output at the back
    and pivots on a 6800 in the front bulkhead (supported at both ends on the roll axis).
  * thigh: flat plate just outside the torso side wall; the knee RS05 sits on its inner side.
  * shin: slim plate outboard of the thigh; the wheel RS05 sits inside a crowned tyre.
  With nothing below the torso, the legs fold beside it until the torso sits on the ground
  (hip 33 mm above the ground, thigh about horizontal, knee motor ahead of the torso).

The geometry, joint axes and mass layout come from the same numbers as the Onshape
FeatureScript. Inertias are built from primitive mass elements. Model contract:

  * body 1 = body_base (freejoint, IMU), drives along +y, +z up
  * wheel bodies `wheel` / `wheel_2`; joints right_/left_ hip_roll, hip_pitch, knee, wheel
  * legs at qpos[7:15] / ctrl[0:8] (right roll, pitch, knee, wheel; then left); 3 neck joints after them
  * every joint is actuated (motor, gear 1); wheel radius 0.05 m
  * qpos = HOME_LEGS (the `home` keyframe) is the as-designed (CAD) pose
  * base origin sits 0.0372 m above the ground plane at home, so the existing
    FLOOR / PLATFORM / wavy start heights stay valid
  * the wheel goes fore-aft under the whole-body CoM (solved here), base level
  * custom numerics `wbc_dh_min` (lowest balancing base height, m) and `wbc_sit` (sit below it)

Run:  python3 CAD_bd/gen_robot.py        (writes CAD_bd/robot.xml and prints the balance check)
"""
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MM = 1e-3

# ---------------------------------------------------------------- geometry (mm, CAD frame)
HZ = 210.0          # hip axis height (roll and pitch axes intersect here)
R = 50.0            # wheel radius
LK = 105.0          # thigh = shin length
HY = 10.0           # hip axis y (torso spans -70..88): over the CoM, so the thigh lies flat at full crouch
RX = 32.0           # hip point |x| (roll axis)
HIP_DROP = 33.0     # hip axis height above the torso bottom
TZ0 = HZ - HIP_DROP # torso bottom
ZT = HZ + 38        # battery shelf underside (clears the gimbal sweep)
ZB_BATT = ZT + 4    # battery underside
TZ1 = ZB_BATT + 65 + 22    # torso top (lid underside): XGT dock + carrier; clears the Nucleo
TYF = 88.0          # torso front face (rear face -70)
TYC = (TYF - 70.0) / 2   # torso fore-aft centre
# lateral stack (|x|, mm): roll RS05 9..55 (axis y) | pitch RS05 14..58 (+3 boss) in the gimbal 10..60.5 |
# thigh 61..70 | knee RS05 17..61 on the thigh's inner side | shin 71..79 | wheel RS05 79..123 (+3) |
# tyre 81..130 | hub 126..130
XP0, XP1 = 14.0, 58.0      # hip-pitch motor body
XT0, XT1 = 61.0, 70.0      # thigh plate
XK0, XK1 = 17.0, 61.0      # knee motor body
XS0, XS1 = 71.0, 79.0      # shin
XW0, XW1 = 79.0, 123.0     # wheel motor body
WC = 105.5                 # wheel (tyre) centre, |x|
TW = 24.5                  # tyre half width
LT = TZ1 + 4        # lid top
NY = -6.0           # neck axis y
PZ = LT + 41 + 46.5 - 11.25          # neck pitch axis
CB0 = PZ + 20                        # pitch-bracket crossbar
RZN = CB0 + 3 + 46.5 - 11.25         # neck roll axis
HZ0 = LT + 95                        # head bottom
HZ1 = HZ0 + 72                       # head top
ZB = 37.2           # base origin height above ground at home (keeps benchmark start heights)

# Joint ranges about the CAD pose. Crouch is -knee / +hip; the crouch side reaches the full fold
# (torso on the ground: thigh ~horizontal, shin folded back over it).
HIP_RANGE = (-0.45, 1.15)
KNEE_RANGE = (-2.15, 0.40)
# Hip roll about +y: + swings the right foot inward. Inward roll is limited by the knee motors
# (both sit 17 mm off the centreline); outward by the thigh swinging into the hip socket.
ROLL_IN, ROLL_OUT = 0.12, 0.35
# Pose offsets so the CAD pose is qpos = HOME_LEGS (the 8-joint layout's historic home, which
# run_interactive_sim.py spawns into): bodies are pre-rotated by -home about the joint axis.
HOME_LEGS = [0, 0.314503, -0.052919, 0, 0, -0.212551, 0.253279, 0]
# Lowest balancing base height, relative to home (m). The torso touches the ground at
# -(HZ - HIP_DROP) = -0.177; balancing stops 17 mm above it (leaning to accelerate dips the front edge). Commanding lower than DH_MIN makes
# the controller sit (wbc_sit): the body settles onto the ground, wheels braked.
DH_MIN = -0.160

RS05_MASS = 0.191
RS05_PEAK = 5.5     # N m (direct drive at every joint)
XL430_MASS = 0.0572
BATTERY_MASS = 0.71  # Makita XGT BL4025 40V max 2.5 Ah (36 V nominal), 118 x 77 x 65 mm


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


def leg_points(e, hz=HZ):
    """Hip (HY, hz) -> wheel (e, R), knee forward. Returns knee (y, z)."""
    hy, wy, wz = HY, e, R
    dy, dz = wy - hy, wz - hz
    d = math.hypot(dy, dz)
    h = math.sqrt(max(LK * LK - (d / 2) ** 2, 0.0))
    ny, nz = -dz / d, dy / d
    if ny < 0:
        ny, nz = -ny, -nz
    return (hy + wy) / 2 + h * ny, (hz + wz) / 2 + h * nz


def link_R(y1, z1, y2, z2):
    """Rotation taking the +z axis onto the (y,z) segment direction."""
    return rot_x(math.atan2(-(y2 - y1), z2 - z1))


# ---------------------------------------------------------------- mass model
def build_bodies(e):
    ky, kz = leg_points(e)
    B = {}
    zb = ZB_BATT
    b = Body()                                  # body_base
    b.box(0.31, (0, TYC, (TZ0 + TZ1) / 2), (120, 158, TZ1 - TZ0), scale=1.6)   # torso shell (hollow -> scaled)
    b.box(0.045, (0, TYC, TZ1 + 2), (120, 158, 4))                     # lid
    b.box(0.02, (0, TYF + 2, HZ + 70), (52, 4, 54))                    # chest details (above the folded knee motor)
    b.box(0.07, (0, 14, HZ), (115, 70, 70), scale=0.6)                 # mid + front bulkheads, rear-wall pads
    for s in (1, -1):
        b.cyl(RS05_MASS, (s * RX, -43, HZ), 23, 44, "y")               # hip-roll motors
    b.box(0.06, (0, -6, zb - 2), (110, 121, 4))                        # battery shelf
    b.box(BATTERY_MASS, (0, -11, zb + 32.5), (77, 118, 65))            # Makita XGT 40V pack
    b.box(0.11, (0, -11, zb + 65 + 7.5), (90, 112, 15))                # XGT rail dock + carrier
    b.box(0.05, (0, TYF - 22, TZ0 + 15 + 67), (70, 18, 133))           # NUCLEO-H743ZI, vertical on the front wall
    b.box(0.07, (48, -20, zb + 16), (12, 70, 28))                      # power / CAN board, beside the battery
    b.box(0.04, (-48, -26, zb + 8), (11, 28, 11))                      # fuse, battery leads
    b.box(0.12, (0, TYC, TZ0 + 80), (110, 150, 140))                   # wiring, fasteners, inserts
    b.box(0.03, (0, NY + 6, LT + 17), (36, 55, 35), scale=1.6)         # neck collar
    b.box(XL430_MASS, (0, NY + 12, LT + 18), (28.5, 46.5, 34))         # yaw XL430 (stator)
    B["body_base"] = b

    for s, side in ((1, "R"), (-1, "L")):
        h = Body()                                                     # hip_roll_*: gimbal frame + pitch motor
        h.box(0.06, (s * 35, HY + 2, HZ), (50.5, 57, 48), scale=0.5)
        h.cyl(RS05_MASS, (s * (XP0 + XP1) / 2, HY, HZ), 23, 44, "x")   # hip-pitch motor
        B["hip_" + side] = h

        f = Body()                                                     # femur*: thigh plate + knee motor
        Rl = link_R(HY, HZ, ky, kz)
        f.box(0.065, (s * (XT0 + XT1) / 2, (HY + ky) / 2, (HZ + kz) / 2), (XT1 - XT0, 54, LK + 54), R=Rl)
        f.cyl(RS05_MASS, (s * (XK0 + XK1) / 2, ky, kz), 23, 44, "x")   # direct-drive knee RS05
        B["femur_" + side] = f

        t = Body()                                                     # tibia*
        Rs = link_R(ky, kz, e, R)
        t.box(0.04, (s * (XS0 + XS1) / 2, (ky + e) / 2, (kz + R) / 2), (8, 36, LK), R=Rs)
        t.cyl(0.015, (s * 67.5, ky, kz), 16, 7, "x")                   # knee hub through the thigh bore
        t.cyl(0.15, (s * (XW0 + XW1) / 2, e, R), 23, 44, "x")          # wheel-motor stator
        B["tibia_" + side] = t

        w = Body()                                                     # wheel*
        w.cyl(0.12, (s * WC, e, R), 50, 2 * TW, "x", r_in=35)          # TPU tyre
        w.cyl(0.07, (s * (WC + 4), e, R), 35, 2 * TW, "x", r_in=31)    # hub + rim
        w.cyl(0.04, (s * 122, e, R), 15, 10, "x")                      # motor rotor + output
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


def geom_box(cls, c, half, origin, mat=None, extra="", kind="box"):
    m = f' material="{mat}"' if mat else ""
    return f'<geom class="{cls}" type="{kind}" pos="{f(*(P(*c) - origin))}" size="{f(*(np.array(half) * MM))}"{m}{extra}/>'


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


def full_crouch_angles(e):
    """Joint changes (hip pitch, knee) from the CAD pose to the torso-on-ground pose
    (hip HIP_DROP above the ground, wheel at its home fore-aft offset)."""
    def angles(hz):
        ky, kz = leg_points(e, hz)
        th = math.atan2(kz - hz, ky - HY)                 # thigh direction
        sh = math.atan2(R - kz, e - ky)                   # shin direction
        return th, sh - th
    t0, k0 = angles(HZ)
    t1, k1 = angles(HIP_DROP + 0.5)
    dk = (k1 - k0 + math.pi) % (2 * math.pi) - math.pi
    return t1 - t0, dk


def main():
    e = solve_balance()
    B, (ky, kz) = build_bodies(e)
    base_o = P(0, 0, ZB)
    out = []
    w = out.append
    w('<mujoco model="bd_droid_rs05">')
    w('  <!-- Generated by CAD_bd/gen_robot.py from CAD/bd_droid_rs05.fs (v3, gimbal hip). Do not edit by hand. -->')
    w('  <compiler angle="radian" meshdir="../CAD/assets" autolimits="true"/>')
    w('  <custom>')
    w(f'    <numeric name="wbc_dh_min" data="{DH_MIN:g}"/>')
    w('    <numeric name="wbc_sit" data="1"/>')
    w('  </custom>')
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
    # torso visuals: box, lid, chest details on the front wall, battery end in the rear opening
    cz = HZ + 70
    zb = ZB_BATT
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
        rr = (-ROLL_OUT, ROLL_IN) if s > 0 else (-ROLL_IN, ROLL_OUT)
        oh = P(s * RX, HY, HZ)
        w(f'      <body name="{names["hip"]}" pos="{f(*(oh - o))}" quat="{f(*axq("y", q[0]))}">')
        w(f'        <joint name="{jn}_hip_roll" axis="0 1 0" range="{q[0] + rr[0]:.6f} {q[0] + rr[1]:.6f}"/>')
        w('        ' + inertial(B["hip_" + side], oh))
        w('        ' + geom_box("visual", (s * 35.25, HY + 2, HZ), (25.25, 28.5, 24), oh, "dark"))
        w('        ' + geom_cyl("visual", (s * XP0, HY, HZ), (s * XP1, HY, HZ), 23.2, oh, "motor"))
        xt = s * (XT0 + XT1) / 2
        of = P(xt, HY, HZ)
        w(f'        <body name="{names["femur"]}" pos="{f(*(of - oh))}" quat="{f(*axq("x", q[1]))}">')
        w(f'          <joint name="{jn}_hip_pitch" axis="1 0 0" range="{q[1] + HIP_RANGE[0]:.6f} {q[1] + HIP_RANGE[1]:.6f}"/>')
        w('          ' + inertial(B["femur_" + side], of))
        w('          ' + link_geom("visual", xt, HY, HZ, ky, kz, 27, 4.5, of, "white"))
        w('          ' + geom_cyl("visual", (s * XT0, HY, HZ), (s * XT1, HY, HZ), 26, of, "white"))
        w('          ' + geom_cyl("visual", (s * XT0, ky, kz), (s * XT1, ky, kz), 27, of, "white"))
        w('          ' + geom_cyl("visual", (s * (XT1 - 1.4), HY, HZ), (s * (XT1 + 0.1), HY, HZ), 22, of, "orange"))
        w('          ' + geom_cyl("visual", (s * XK0, ky, kz), (s * XK1, ky, kz), 23, of, "motor"))
        w('          ' + geom_cyl("collision", (xt, HY, HZ), (xt, ky, kz), 26, of, kind="capsule"))
        w('          ' + geom_cyl("collision", (s * XK0, ky, kz), (s * XK1, ky, kz), 23, of))
        xs = s * (XS0 + XS1) / 2
        ot = P(xs, ky, kz)
        w(f'          <body name="{names["tibia"]}" pos="{f(*(ot - of))}" quat="{f(*axq("x", q[2]))}">')
        w(f'            <joint name="{jn}_knee" axis="1 0 0" range="{q[2] + KNEE_RANGE[0]:.6f} {q[2] + KNEE_RANGE[1]:.6f}"/>')
        w('            ' + inertial(B["tibia_" + side], ot))
        w('            ' + link_geom("visual", xs, ky, kz, e, R, 18, 4, ot, "white"))
        w('            ' + geom_cyl("visual", (s * XS0, ky, kz), (s * XS1, ky, kz), 17, ot, "white"))
        w('            ' + geom_cyl("visual", (s * XS0, e, R), (s * XS1, e, R), 26, ot, "white"))
        w('            ' + geom_cyl("visual", (s * XS1, ky, kz), (s * (XS1 + 3), ky, kz), 17, ot, "orange"))
        w('            ' + geom_cyl("visual", (s * XW0, e, R), (s * XW1, e, R), 23, ot, "motor"))
        # shin collision stops 40 mm above the axle; the wheel motor sits inside the wheel silhouette
        sy, sz = e + (ky - e) * 40 / LK, R + (kz - R) * 40 / LK
        w('            ' + geom_cyl("collision", (xs, ky, kz), (xs, sy, sz), 16, ot, kind="capsule"))
        ow = P(s * WC, e, R)
        w(f'            <body name="{names["wheel"]}" pos="{f(*(ow - ot))}">')
        w(f'              <joint name="{jn}_wheel" axis="1 0 0"/>')
        w('              ' + inertial(B["wheel_" + side], ow))
        w('              ' + geom_cyl("visual", (s * (WC - TW), e, R), (s * (WC + TW), e, R), 50, ow, "tyre"))
        w('              ' + geom_cyl("visual", (s * (WC + TW - 4), e, R), (s * (WC + TW + 0.05), e, R), 34, ow, "grey"))
        w('              ' + geom_cyl("visual", (s * (WC + TW - 2.5), e, R), (s * (WC + TW + 0.1), e, R), 13, ow, "orange"))
        # crowned TPU tyre (rounded shoulders): one contact patch that stays centred when hip roll
        # cambers the wheel, and no two-edge scrub torque in turns
        w('              ' + geom_box("collision", (s * WC, e, R), (TW, R, R), ow, kind="ellipsoid"))
        w('            </body>')
        w('          </body>')
        w('        </body>')
        w('      </body>')

    # ---- 3-DOF neck (declared after the legs so qpos[7:15] / ctrl[0:8] stay the legs)
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
            w(f'    <motor name="{jn}_{j}" joint="{jn}_{j}" gear="1" ctrlrange="{-RS05_PEAK:.4g} {RS05_PEAK:.4g}"/>')   # RS05, direct drive
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
    dq_hip, dq_knee = full_crouch_angles(e)
    print(f"wrote {path}")
    print(f"wheel fore-aft offset under CoM e = {e:.2f} mm, knee at y={ky:.1f} z={kz:.1f} mm")
    print(f"total mass {tot:.3f} kg  (head+neck {sum(B[k].total()[0] for k in ('neck_yaw', 'neck_pitch', 'head')):.3f} kg)")
    print(f"torso-on-ground pose: hip pitch {dq_hip:+.3f} rad (range up to {HIP_RANGE[1]}), "
          f"knee {dq_knee:+.3f} rad (range down to {KNEE_RANGE[0]})")


if __name__ == "__main__":
    main()
