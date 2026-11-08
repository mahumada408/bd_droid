#!/usr/bin/env python3
"""Export the printer fit coupon for the BD droid (CAD/print/fit_coupon.stl), 1:1.

Print it in the same material and settings as the structural parts, then check every fit against real hardware
before printing the robot. Each feature uses the exact size the FeatureScript (CAD/bd_droid_rs05.fs) uses:

  A  RS05 stator ring : D35.3 bore + 8 x D3.4 on PCD 41.5      -> lay it on the motor's output-side housing
  B  RS05 rotor face  : D9.5 + 6 x D4.5 on PCD 24, 3 x D3.9 dowel holes on PCD 19 (D4 x 8 pins press in)
                        -> bolt it to the output with M4 screws (threads only 3.5 deep: use M4 x 8 here)
  C  6800-2RS seat    : D19.1 x 5 pocket over a D12 hole       -> the bearing should drop in and be held by the cap
  D  heat-set inserts : 2 x D4.0 x 6 (M3 x 5.7) and 1 x D4.0 x 4.5 (M3 x 4)
  E  countersinks     : M3 (DIN 7991, head D6) and M4 (head D8)
  F  gimbal stub      : D10 peg printed horizontal, like on the gimbal -> light press into the 6800 inner race

If a fit is off, change the matching number in the FeatureScript (hole diameters are written out at each call)
and re-run this script; the printed size depends on the printer, nozzle and material.

    python3 tools/export_fit_coupon.py           # -> CAD/print/fit_coupon.stl

Needs: pip install manifold3d numpy
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_display_stl import ROOT, box, cyl, union, write_stl  # noqa: E402
from manifold3d import Manifold  # noqa: E402

T = 6.0   # plate thickness


def ring(cx, cy, n, pcd, d, phase_deg):
    return [cyl((cx + pcd / 2 * math.cos(math.radians(phase_deg + k * 360 / n)),
                 cy + pcd / 2 * math.sin(math.radians(phase_deg + k * 360 / n)), -1),
                (cx + pcd / 2 * math.cos(math.radians(phase_deg + k * 360 / n)),
                 cy + pcd / 2 * math.sin(math.radians(phase_deg + k * 360 / n)), T + 1), d / 2) for k in range(n)]


def csk(cx, cy, hr):
    """90 deg countersink from the top face (z = T), plus the clearance hole."""
    return Manifold.cylinder(hr - 0.7, 1.0, hr + 0.3, 48).translate([cx, cy, T - (hr - 1.0)])


def coupon():
    plate = box(0, 110, 0, 60, 0, T)
    riser = box(96, 110, 2, 16, 0, 12)
    peg = cyl((110, 9, 6), (116, 9, 6), 5.0)                          # F: D10 gimbal stub, printed horizontal
    body = union([plate, riser, peg])
    cut = []
    cut += [cyl((27, 30, -1), (27, 30, T + 1), 35.3 / 2)] + ring(27, 30, 8, 41.5, 3.4, 22.5)            # A
    cut += [cyl((70, 18, -1), (70, 18, T + 1), 9.5 / 2)] + ring(70, 18, 6, 24, 4.5, 0) + ring(70, 18, 3, 19, 3.9, 30)  # B
    cut += [cyl((95, 42, T - 5), (95, 42, T + 1), 19.1 / 2), cyl((95, 42, -1), (95, 42, T + 1), 6)]    # C
    cut += [cyl((58, 50, T - 6), (58, 50, T + 1), 2.0), cyl((66, 50, T - 6), (66, 50, T + 1), 2.0),   # D (6 mm plate:
            cyl((74, 50, T - 4.5), (74, 50, T + 1), 2.0)]                                               #  blind 5.9)
    cut += [cyl((52, 8, -1), (52, 8, T + 1), 1.7), csk(52, 8, 3.2),                                    # E
            cyl((52, 20, -1), (52, 20, T + 1), 2.25), csk(52, 20, 4.1)]
    # labels: notches on the plate edge, one per feature group (A..E counted from the left)
    cut += [box(25 + 0, 29, -1, 1.5, T - 1, T + 1), box(68, 72, -1, 1.5, T - 1, T + 1)]
    return body - union(cut)


def main():
    m = coupon()
    out = os.path.join(ROOT, "CAD", "print", "fit_coupon.stl")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    write_stl(m, out)
    lo, hi = m.bounding_box()[:3], m.bounding_box()[3:]
    print(f"wrote {out}: {m.num_tri()} triangles, {len(m.decompose())} solid(s), "
          f"size {hi[0] - lo[0]:.1f} x {hi[1] - lo[1]:.1f} x {hi[2] - lo[2]:.1f} mm, volume {m.volume() / 1000:.1f} cm^3")


if __name__ == "__main__":
    main()
