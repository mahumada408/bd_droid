#!/usr/bin/env python3
"""Export a one-piece, scaled display model of the BD droid (CAD/bd_droid_rs05.fs) as a binary STL.

This rebuilds the visible outer shape of the v3 gimbal-hip design from the same dimensions as the
FeatureScript, without the internals (bearings, electronics) or screw holes, and joins everything with a base
plate into one solid that prints without assembly. Antenna rods are thickened so they survive the
scale-down.

    python3 tools/export_display_stl.py                      # 1:3 -> CAD/print/bd_droid_1to3.stl
    python3 tools/export_display_stl.py --scale 4            # 1:4
    python3 tools/export_display_stl.py --hip 120            # crouched pose (hip axis height, mm, 33..255; 33 = sitting on the torso)

Needs: pip install manifold3d numpy
"""
import argparse
import math
import os
import struct

import numpy as np
from manifold3d import CrossSection, Manifold, OpType

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SEG = 64

# ---------------------------------------------------------------- primitives (mm, CAD frame)


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)]).translate([min(x0, x1), min(y0, y1), min(z0, z1)])


def cyl(a, b, r, seg=SEG):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L = float(np.linalg.norm(d))
    z = d / L
    t = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(t, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    m = np.column_stack([x, y, z, a])
    return Manifold.cylinder(L, r, r, seg).transform(m)


def union(parts):
    return Manifold.batch_boolean(parts, OpType.Add)


def hull_yz(x0, x1, y1, z1, r1, y2, z2, r2):
    return Manifold.batch_hull([cyl((x0, y1, z1), (x1, y1, z1), r1), cyl((x0, y2, z2), (x1, y2, z2), r2)])


def rbox_z(x0, x1, y0, y1, z0, z1, r):
    """Box with its vertical edges rounded."""
    return Manifold.batch_hull([cyl((x, y, z0), (x, y, z1), r) for x in (x0 + r, x1 - r) for y in (y0 + r, y1 - r)])


def rbox_y(x0, x1, y0, y1, z0, z1, r):
    """Box with its edges parallel to y rounded."""
    return Manifold.batch_hull([cyl((x, y0, z), (x, y1, z), r) for x in (x0 + r, x1 - r) for z in (z0 + r, z1 - r)])


def link(x0, x1, y1, z1, y2, z2, r1, r2):
    """Planar link with rounded ends in the y-z plane (FeatureScript mkLink)."""
    L = math.hypot(y2 - y1, z2 - z1)
    w = min(r1, r2) * 1.6
    a = math.atan2(-(y2 - y1), z2 - z1)
    web = box(x0, x1, -w / 2, w / 2, 0, L).rotate([math.degrees(a), 0, 0]).translate([0, y1, z1])
    return union([web, cyl((x0, y1, z1), (x1, y1, z1), r1), cyl((x0, y2, z2), (x1, y2, z2), r2)])


def strip(x0, x1, y1, z1, y2, z2, w, t0, t1):
    """FeatureScript linkBox: a band along the (y,z) segment from distance t0 to t1."""
    a = math.atan2(-(y2 - y1), z2 - z1)
    return box(x0, x1, -w / 2, w / 2, t0, t1).rotate([math.degrees(a), 0, 0]).translate([0, y1, z1])


def rs05(rear, s, y, z):
    return union([cyl((rear, y, z), (rear + s * 44, y, z), 23), cyl((rear + s * 44, y, z), (rear + s * 47, y, z), 15)])


# ---------------------------------------------------------------- robot

TXN, TYB, TYF = 60.0, -70.0, 88.0
RX, HY, HD = 32.0, 10.0, 33.0  # hip point |x|, y; hip axis height above the torso bottom
WC, TW = 105.5, 24.5          # tyre centre |x|, half width


def crowned_tyre(s, R, r_in=35.0, crown=14.0, seg=SEG):
    """Tyre with rounded shoulders (FeatureScript: 14 mm fillets), axis along x, centred at x = s * WC."""
    pts = [(r_in, -TW), (R - crown, -TW)]
    for k in range(1, 12):
        a = -math.pi / 2 + k * (math.pi / 2) / 12
        pts.append((R - crown + crown * math.cos(a), -TW + crown + crown * math.sin(a)))
    pts += [(R, -TW + crown), (R, TW - crown)]
    for k in range(1, 12):
        a = k * (math.pi / 2) / 12
        pts.append((R - crown + crown * math.cos(a), TW - crown + crown * math.sin(a)))
    pts += [(R - crown, TW), (r_in, TW)]
    t = Manifold.revolve(CrossSection([pts]), seg)                # axis z, radius in x
    return t.rotate([0, 90, 0]).translate([s * WC, 0, R])


def leg(s, hz, R, ky, kz):
    """ky: knee y relative to the hip."""
    P = []
    ky = HY + ky
    P.append(cyl((s * RX, -67, hz), (s * RX, -20, hz), 23))                 # hip-roll motor (inside the torso)
    P.append(box(s * 10, s * 60.5, -18, 38, hz - 23, hz + 23))             # gimbal (closed for display)
    P.append(cyl((s * 58, HY, hz), (s * 61.5, HY, hz), 15))                # hip-pitch output boss
    P.append(hull_yz(s * 61, s * 70, HY, hz, 26, ky, kz, 27))              # thigh plate (cap inlay flush)
    P.append(rs05(s * 17, s, ky, kz))                                      # knee motor, inner side of the thigh
    P.append(cyl((s * 63.5, ky, kz), (s * 71.5, ky, kz), 16))              # shin hub through the thigh bore
    P.append(link(s * 71, s * 79, ky, kz, HY, R, 17, 26))                  # shin
    P.append(strip(s * 78.5, s * 80, HY, R, ky, kz, 8, 32, 72))            # shin stripe
    P.append(cyl((s * 78.5, ky, kz), (s * 82, ky, kz), 17))                # knee cap
    P.append(rs05(s * 78.5, s, HY, R))                                     # wheel motor
    P.append(cyl((s * 125.5, HY, R), (s * 130, HY, R), 34))                # hub disc
    P.append(cyl((s * (WC - TW), HY, R), (s * (WC + TW), HY, R), 35.5))    # rim (solid)
    P.append(cyl((s * 129.5, HY, R), (s * 130.5, HY, R), 13))              # flush hub cap
    grooves = [box(s * WC - 12, s * WC + 12, HY - 2.5, HY + 2.5, 2 * R - 2, 2 * R + 1)
               .translate([0, -HY, -R]).rotate([k * 18 + 9, 0, 0]).translate([0, HY, R]) for k in range(20)]
    P.append(crowned_tyre(s, R).translate([0, HY, 0]) - union(grooves))
    return P


def robot(hz=210.0, R=50.0, ant_r=2.5):
    half = (hz - R) / 2
    kz = R + half
    ky = math.sqrt(105 ** 2 - half ** 2)
    P = leg(1, hz, R, ky, kz) + leg(-1, hz, R, ky, kz)

    # torso (solid display shell): rounded front-bottom edge, hip sockets, XGT battery end in the rear opening
    tz0 = hz - HD
    zB = hz + 42
    zD = zB + 65
    tz1 = zD + 22
    torso = rbox_z(-TXN, TXN, TYB, TYF, tz0, tz1 + 4, 8)
    r = 30.0
    torso = torso - (box(-TXN - 1, TXN + 1, TYF - r, TYF + 1, tz0 - 1, tz0 + r)
                     - cyl((-TXN - 2, TYF - r, tz0 + r), (TXN + 2, TYF - r, tz0 + r), r))
    torso = torso - union([box(s * 48, s * (TXN + 1), -19, 39, tz0 - 1, hz + 38) for s in (1, -1)])
    frame = box(-40, 40, TYB - 1, TYB + 1.5, zB - 1, zD + 1) - box(-38.5, 38.5, TYB - 2, TYB + 2, zB, zD)
    torso = torso - frame                                                   # groove around the battery end
    P.append(torso)
    P.append(box(-12, 12, TYB - 1, TYB + 0.5, zB + 30, zB + 44))           # XGT release button
    # chest details
    cz = hz + 70
    chest = box(-26, 26, TYF - 0.5, TYF + 4, cz - 27, cz + 27)
    chest = chest - union([box(-16, 16, TYF + 2, TYF + 5, cz - 18 + k * 8, cz - 14 + k * 8) for k in range(4)])
    P += [chest, box(-6, 6, TYF + 3, TYF + 6, cz + 18, cz + 24), box(-12, 12, TYF - 0.5, TYF + 2, cz + 33, tz1 - 5),
          box(-46, -38, TYF - 0.5, TYF + 2, cz - 27, cz + 27), box(38, 46, TYF - 0.5, TYF + 2, cz - 27, cz + 27)]

    # neck
    Lt = tz1 + 4
    NY = -6
    P.append(rbox_z(-18, 18, -21.5, 33.5, Lt - 0.5, Lt + 35, 8))           # collar (yaw servo inside)
    P.append(rbox_z(-27, 27, -32, 26, Lt - 0.5, Lt + 4, 6))                # collar flange
    P.append(cyl((0, NY, Lt + 34.5), (0, NY, Lt + 38), 11))                # yaw horn
    P.append(union([cyl((0, NY, Lt + 37.5), (0, NY, Lt + 41), 19), box(-19, 19, NY - 16, NY + 16, Lt + 37.5, Lt + 41)]))
    pz = Lt + 41 + 46.5 - 11.25
    P.append(box(-17, 17, NY - 14.25, NY + 14.25, Lt + 40.5, Lt + 87.5))   # pitch servo
    P.append(cyl((16.5, NY, pz), (20.5, NY, pz), 11))
    P.append(cyl((-16.5, NY, pz), (-20.5, NY, pz), 8))
    cb0 = pz + 20
    P.append(union([box(20, 23, NY - 12, NY + 12, pz, cb0 + 3), cyl((20, NY, pz), (23, NY, pz), 12),
                    box(-20, -23, NY - 12, NY + 12, pz, cb0 + 3), cyl((-20, NY, pz), (-23, NY, pz), 12),
                    box(-23, 23, NY - 14, NY + 14, cb0, cb0 + 3)]))
    rzN = cb0 + 3 + 46.5 - 11.25
    P.append(box(-14.25, 14.25, NY - 17, NY + 17, cb0 + 2.5, cb0 + 49.5))  # roll servo
    P.append(cyl((0, NY + 16.5, rzN), (0, NY + 20.5, rzN), 11))
    P.append(cyl((0, NY - 16.5, rzN), (0, NY - 20.5, rzN), 8))

    # head (solid), mount plates reach down to the roll horns
    hz0 = Lt + 95
    hz1 = hz0 + 72
    P += [box(-16, 16, NY + 20, NY + 23, rzN - 14, hz0 + 1), box(-16, 16, NY - 23, NY - 20, rzN - 14, hz0 + 1)]
    head = rbox_y(-88, 88, -41, 40, hz0, hz1, 10)

    def wedge(pts):          # triangular prism across the head width from three (y, z) corners
        return Manifold.batch_hull([box(-90, 90, y, y + 0.01, z, z + 0.01) for y, z in pts])
    head = head - wedge([(30, hz1 + 1), (41, hz1 + 1), (41, hz1 - 10)])   # 9 mm front chamfers, top and bottom
    head = head - wedge([(30, hz0 - 1), (41, hz0 - 1), (41, hz0 + 10)])
    P.append(head)
    P += [box(40, 62, -41.5, 40.5, hz1 - 1, hz1 + 1), box(-62, -40, -41.5, 40.5, hz1 - 1, hz1 + 1),
          box(-20, 20, 30.5, 41, hz1 - 14, hz1 - 6)]                       # bands, brow stripe
    ez = hz0 + 34
    bezel = cyl((42, 39, ez), (42, 46, ez), 23) - cyl((42, 43, ez), (42, 47, ez), 17)
    P += [bezel, cyl((42, 39, ez), (42, 45, ez), 17)]                     # main eye
    P += [cyl((-42, 39, ez), (-42, 48, ez), 18), cyl((-42, 47.5, ez), (-42, 52, ez), 12) - cyl((-42, 50, ez), (-42, 53, ez), 6)]
    P.append(box(-14, 14, 39, 42, ez - 10, ez + 10))                       # face screen
    for s in (1, -1):
        ear = cyl((s * 87.5, 2, ez), (s * 96, 2, ez), 20) - cyl((s * 94, 2, ez), (s * 97, 2, ez), 10)
        P.append(ear)
        P.append(cyl((s * 66, -24, hz1 - 1), (s * 66, -24, hz1 + 6), 4))   # antenna base
        P.append(cyl((s * 66, -24, hz1 + 5), (s * 76, -30, hz1 + 75), ant_r, 24))
        P.append(Manifold.sphere(ant_r * 1.6, 24).translate([s * 76, -30, hz1 + 75]))   # rounded tip
    return P, (hz, ky, kz)


def base_plate(R=50.0, sink=2.0, t=6.0):
    """Display base under the wheels; the tyres sink 'sink' mm into it so the print is one piece.
    A pad under each tyre fills the tread grooves at the contact patch (no trapped voids)."""
    pads = [box(s * (WC - 12), s * (WC + 12), HY - 14, HY + 14, 0, sink + 2) for s in (1, -1)]
    return union([rbox_z(-150, 150, -85, 100, -t + sink, sink, 20)] + pads)


def write_stl(m, path):
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3].astype(np.float32)
    f = np.asarray(mesh.tri_verts)
    tri = v[f]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    with open(path, "wb") as fh:
        fh.write(b"BD droid RS05 display model".ljust(80, b" "))
        fh.write(struct.pack("<I", len(f)))
        fh.write(rec.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scale", type=float, default=3.0, help="reduction factor (3 -> 1:3)")
    ap.add_argument("--hip", type=float, default=210.0, help="hip axis height in mm (pose)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    parts, _ = robot(args.hip)
    parts.append(base_plate())
    m = union(parts).scale([1 / args.scale] * 3)
    comps = m.decompose()
    lo, hi = m.bounding_box()[:3], m.bounding_box()[3:]
    out = args.out or os.path.join(ROOT, "CAD", "print", f"bd_droid_1to{args.scale:g}.stl")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    m = m.translate([0, 0, -lo[2]])
    write_stl(m, out)
    print(f"wrote {out}: {m.num_tri()} triangles, {len(comps)} solid(s), "
          f"size {hi[0] - lo[0]:.1f} x {hi[1] - lo[1]:.1f} x {hi[2] - lo[2]:.1f} mm, volume {m.volume() / 1000:.1f} cm^3")


if __name__ == "__main__":
    main()
