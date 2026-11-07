FeatureScript 3112;
import(path : "onshape/std/common.fs", version : "3112.0");

// BD-style wheel-legged droid, built around RobStride 05 actuators,
// a Makita XGT 40V max tool battery (BL4025, slides in from the back) and a NUCLEO-H743ZI (Nucleo-144, MB1364).
// Frame: x = robot right, y = forward, z = up, ground at z = 0, axle midpoint at x = 0, y = 0.
// All dimensions in mm.
//
// RS05 interfaces (datasheet 1.1, first-angle projection):
//   output face : rotor flange 6 x M4 on PCD 24 (depth 3.5), pilot D19, boss D30 x 3
//                 stationary ring 8 x M3 on PCD 41.5 (depth 7)
//   rear face   : 4 x M3 on PCD 38.5 (depth 10)
//   1.6 N m rated, 5.5 N m peak, 480 rpm no-load at 48 V, 191 g.
// Fasteners: clearance M3 = 3.4, M4 = 4.5, M2.5 = 2.9; heat-set insert holes M3 = 4.0, pilot M2 = 2.0.
//
// VARIANT (slim inboard-knee, 2:1; was the default until the v2 central-hip design): lane layout of CAD/variants/bd_droid_rs05_inboard_1p6.fs (motors inboard, shin + wheel outboard,
// full knee fold) with a straight, slim thigh: small 2:1 pulleys on a 25 mm belt let the knee motor sit on the
// thigh axis. Set "Hip axis height" to 52 mm to see the full crouch.
//
// Knee transmission (Gates PowerGrip GT3, 5 mm pitch, 25 mm wide):
//   drive  : 18T 5MGT-25 (PD 28.65), keyed on a D10 steel shaft adapter bolted to the knee RS05 rotor (6 x M4);
//            shaft end carried by a 6800-2RS in a sliding carrier in the cover (no overhung load on the motor)
//   driven : 36T 5MGT-25 (PD 57.30), integral with the knee hub that carries the shin
//   belt   : 5MR-250-25 (250 mm, 50 teeth; listed by Gates, made to order), centre distance 55.66 mm,
//            knee-motor axis 49.34 mm from the hip on the thigh axis
//   ratio  : 2:1 -> knee 11.0 N m peak / 3.2 N m continuous, 240 rpm max
//   rating : 18T @ 500 rpm: 6.91 N m (15 mm base) x 1.82 (25 mm) x 0.70 length = 8.8 N m (service factor 1.6 on 5.5 N m)
//   wrap   : 150 deg on the 18T (7.5 teeth in mesh)
//   tension: knee motor + bearing carrier slide +-1.5 mm along the thigh, M3 jack screw;
//            set Tst = 155 N (35 lb) per span: 0.84 mm deflection at mid-span under 12.2-13.2 N.

function P(x, y, z) { return vector(x, y, z) * millimeter; }

function cyl(context is Context, id is Id, a is Vector, b is Vector, r is number)
{
    fCylinder(context, id, { "bottomCenter" : a, "topCenter" : b, "radius" : r * millimeter });
}

function mkBox(context is Context, id is Id, x0, x1, y0, y1, z0, z1)
{
    fCuboid(context, id, { "corner1" : P(min(x0, x1), min(y0, y1), min(z0, z1)), "corner2" : P(max(x0, x1), max(y0, y1), max(z0, z1)) });
}

function bq(id is Id) { return qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID); }

function uni(context is Context, id is Id, q is Query)
{
    opBoolean(context, id, { "tools" : q, "operationType" : BooleanOperationType.UNION });
}

function subtr(context is Context, id is Id, target is Query, tools is Query)
{
    opBoolean(context, id, { "targets" : target, "tools" : tools, "operationType" : BooleanOperationType.SUBTRACTION });
}

function tagPart(context is Context, q is Query, name is string, c is Color)
{
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : name });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE, "value" : c });
}

// Point from axis coordinate a and in-plane coordinates (u, v).
// ax "x": u = y, v = z.  ax "y": u = x, v = z.  ax "z": u = x, v = y.
function axPt(ax is string, a, u, v)
{
    if (ax == "x")
        return P(a, u, v);
    if (ax == "y")
        return P(u, a, v);
    return P(u, v, a);
}

// Cut a bolt circle (n holes, PCD, diameter) through target, along axis ax from a0 to a1,
// centred at (cu, cv). phase in degrees. skipBelow: skip holes within this angle (deg) of skipDir (deg); use -1 for none.
function holesSkip(context is Context, id is Id, target is Query, ax is string, cu, cv, a0, a1, n, pcd, dia, phase, skipDir, skipBelow)
{
    var tools = [];
    for (var k = 0; k < n; k += 1)
    {
        const ang = phase + k * 360 / n;
        if (skipBelow > 0)
        {
            var d = abs(((ang - skipDir) % 360 + 540) % 360 - 180);
            if (d < skipBelow)
                continue;
        }
        const u = cu + pcd / 2 * cos(ang * degree);
        const v = cv + pcd / 2 * sin(ang * degree);
        cyl(context, id + ("h" ~ k), axPt(ax, a0, u, v), axPt(ax, a1, u, v), dia / 2);
        tools = append(tools, bq(id + ("h" ~ k)));
    }
    if (size(tools) > 0)
        subtr(context, id + "cut", target, qUnion(tools));
}

function holes(context is Context, id is Id, target is Query, ax is string, cu, cv, a0, a1, n, pcd, dia, phase)
{
    holesSkip(context, id, target, ax, cu, cv, a0, a1, n, pcd, dia, phase, 0, -1);
}

// Single hole at (u, v)
function hole(context is Context, id is Id, target is Query, ax is string, u, v, a0, a1, dia)
{
    holes(context, id, target, ax, u, v, a0, a1, 1, 0, dia, 0);
}

// Prism from a closed (y, z) polygon, extruded along x from x0 to x1.
function prismYZ(context is Context, id is Id, x0, x1, pts is array)
{
    const sk = newSketchOnPlane(context, id + "sk", { "sketchPlane" : plane(P(min(x0, x1), 0, 0), X_DIRECTION, Y_DIRECTION) });
    var pp = [];
    for (var p in pts)
        pp = append(pp, vector(p[0], p[1]) * millimeter);
    pp = append(pp, pp[0]);
    skPolyline(sk, "poly", { "points" : pp });
    skSolve(sk);
    opExtrude(context, id + "ex", { "entities" : qSketchRegion(id + "sk"), "direction" : X_DIRECTION,
                "endBound" : BoundingType.BLIND, "endDepth" : abs(x1 - x0) * millimeter });
    opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
}

// Prism from a closed (x, z) polygon, extruded along y from y0 to y1.
function prismXZ(context is Context, id is Id, y0, y1, pts is array)
{
    const sk = newSketchOnPlane(context, id + "sk", { "sketchPlane" : plane(P(0, min(y0, y1), 0), -Y_DIRECTION, X_DIRECTION) });
    var pp = [];
    for (var p in pts)
        pp = append(pp, vector(p[0], p[1]) * millimeter);
    pp = append(pp, pp[0]);
    skPolyline(sk, "poly", { "points" : pp });
    skSolve(sk);
    opExtrude(context, id + "ex", { "entities" : qSketchRegion(id + "sk"), "direction" : Y_DIRECTION,
                "endBound" : BoundingType.BLIND, "endDepth" : abs(y1 - y0) * millimeter });
    opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
}

// Convex hull of two circles in the y-z plane, (y1,z1,r1) and (y2,z2,r2), extruded x0..x1. One body under id.
function hullYZ(context is Context, id is Id, x0, x1, y1, z1, r1, y2, z2, r2)
{
    const dy = y2 - y1;
    const dz = z2 - z1;
    const d = sqrt(dy * dy + dz * dz);
    const uy = dy / d;
    const uz = dz / d;
    const ny = -uz;
    const nz = uy;
    const sa = (r1 - r2) / d;
    const ca = sqrt(1 - sa * sa);
    const my = uy * sa + ny * ca;
    const mz = uz * sa + nz * ca;
    const qy = uy * sa - ny * ca;
    const qz = uz * sa - nz * ca;
    prismYZ(context, id + "q", x0, x1, [[y1 + r1 * my, z1 + r1 * mz], [y2 + r2 * my, z2 + r2 * mz],
                [y2 + r2 * qy, z2 + r2 * qz], [y1 + r1 * qy, z1 + r1 * qz]]);
    cyl(context, id + "c1", P(x0, y1, z1), P(x1, y1, z1), r1);
    cyl(context, id + "c2", P(x0, y2, z2), P(x1, y2, z2), r2);
    uni(context, id + "u", qUnion([bq(id + "q"), bq(id + "c1"), bq(id + "c2")]));
}

// Box lying along the y-z segment (y1,z1)->(y2,z2), from distance t0 to t1, width w, x from x0 to x1.
function linkBox(context is Context, id is Id, x0, x1, y1, z1, y2, z2, w, t0, t1)
{
    const dy = y2 - y1;
    const dz = z2 - z1;
    mkBox(context, id, x0, x1, -w / 2, w / 2, t0, t1);
    opTransform(context, id + "rot", { "bodies" : bq(id),
                "transform" : transform(P(0, y1, z1)) * rotationAround(line(P(0, 0, 0), X_DIRECTION), atan2(-dy, dz)) });
}

// Planar link (rounded ends) in the y-z plane between (y1,z1) and (y2,z2), x from x0 to x1. Body id: id + "web".
function mkLink(context is Context, id is Id, x0, x1, y1, z1, y2, z2, r1, r2)
{
    const L = sqrt((y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
    linkBox(context, id + "web", x0, x1, y1, z1, y2, z2, min(r1, r2) * 1.6, 0, L);
    cyl(context, id + "e1", P(x0, y1, z1), P(x1, y1, z1), r1);
    cyl(context, id + "e2", P(x0, y2, z2), P(x1, y2, z2), r2);
    uni(context, id + "u", qUnion([bq(id + "web"), bq(id + "e1"), bq(id + "e2")]));
}

// Timing pulley along x: OD cylinder x0..x1 with n round grooves (approximation of the GT3 profile, 1.9 mm deep).
function pulleyTeeth(context is Context, id is Id, x0, x1, y, z, n, rOD)
{
    cyl(context, id + "b", P(x0, y, z), P(x1, y, z), rOD);
    var gr = [];
    for (var k = 0; k < n; k += 1)
    {
        const a = k * 360 / n * degree;
        const gy = y + (rOD - 0.4) * cos(a);
        const gz = z + (rOD - 0.4) * sin(a);
        cyl(context, id + ("g" ~ k), P(x0 - 1, gy, gz), P(x1 + 1, gy, gz), 1.5);
        gr = append(gr, bq(id + ("g" ~ k)));
    }
    subtr(context, id + "gc", bq(id + "b"), qUnion(gr));
}

// RobStride 05 envelope, axis along x. rearX = rear face, s = direction from rear face toward the output.
function rs05x(context is Context, id is Id, rearX, s, y, z, name is string)
{
    cyl(context, id + "h", P(rearX, y, z), P(rearX + s * 44, y, z), 23);
    cyl(context, id + "o", P(rearX + s * 44, y, z), P(rearX + s * 47, y, z), 15);
    uni(context, id + "u", qUnion([bq(id + "h"), bq(id + "o")]));
    tagPart(context, bq(id + "h"), name, color(0.12, 0.12, 0.13));
}

function rs05y(context is Context, id is Id, x, rearY, s, z, name is string)
{
    cyl(context, id + "h", P(x, rearY, z), P(x, rearY + s * 44, z), 23);
    cyl(context, id + "o", P(x, rearY + s * 44, z), P(x, rearY + s * 47, z), 15);
    uni(context, id + "u", qUnion([bq(id + "h"), bq(id + "o")]));
    tagPart(context, bq(id + "h"), name, color(0.12, 0.12, 0.13));
}

const WHITE = color(0.86, 0.83, 0.76);
const ORANGE = color(0.85, 0.38, 0.14);
const DARK = color(0.22, 0.22, 0.23);
const TYRE = color(0.08, 0.08, 0.08);
const GREY = color(0.55, 0.56, 0.58);
const ALU = color(0.72, 0.73, 0.75);
const STEEL = color(0.45, 0.46, 0.48);
const PCB = color(0.1, 0.45, 0.2);
const BELT = color(0.1, 0.1, 0.1);

// Torso / layout constants
const TXN = 60;     // torso half width
const TYB = -70;    // torso rear outer face (hip-roll motors bolt here)
const TYF = 88;     // torso front outer face
const WALL = 3;

// Knee transmission constants (see header)
const PD_DRIVE = 28.648;
const PD_DRIVEN = 57.296;
const R_DRIVE = PD_DRIVE / 2 - 0.572;     // OD radius 13.75
const R_DRIVEN = PD_DRIVEN / 2 - 0.572;   // OD radius 28.08
const BELT_T = 1.9;                       // belt backing above the OD (3.81 total - 1.91 tooth)
const KNEE_D = 49.343;                    // knee-motor axis distance from the hip axis, along the thigh

function buildLeg(context is Context, id is Id, def is map, s is number, side is string)
{
    const hz = def.hipZ;
    const R = def.wheelR;
    const ky = def.kneeY;
    const kz = def.kneeZ;
    const rz = hz + 45;                       // hip-roll axis height
    const yw = TYB;                           // rear wall outer face (roll output face lies here)
    // Lateral stack (right side, mirrored for left), mm from centreline:
    // yoke arm 61-67 | hip pitch RS05 67-111 (+3 boss) and knee RS05 70-114 (+3 boss), side by side on the thigh axis
    // thigh inner wall 114-119 | belt pocket 119-151 (25 mm belt) | cover 151-156 | shin 158-166 | wheel RS05 166-210
    // hub disc 213-217 inside the rim, flush hub cap | tyre 168-217 (49 mm wide, centre 192.5) covers the wheel motor

    // ---- Hip-roll RS05: output-side ring bolted to the inside of the torso rear wall ----
    rs05y(context, id + "rollM", s * 26, yw + WALL + 44, -1, rz, "RS05 hip roll " ~ side);

    // ---- Hip yoke (printed) ----
    const yp0 = yw - 1.5 - 8;
    mkBox(context, id + "yk1", s * 13, s * 67, yp0, yw - 1.5, hz - 24, rz + 24);
    mkBox(context, id + "yk2", s * 61, s * 67, yp0, 26, hz - 24, hz + 12);
    cyl(context, id + "ykp", P(s * 26, yw - 1.5, rz), P(s * 26, yw, rz), 13);          // 1.5 mm pad onto roll output
    uni(context, id + "yku", qUnion([bq(id + "yk1"), bq(id + "yk2"), bq(id + "ykp")]));
    try silent { opFillet(context, id + "ykf", { "entities" : qParallelEdges(qCreatedBy(id + "yk1", EntityType.EDGE), X_DIRECTION), "radius" : 6 * millimeter }); }
    const yk = bq(id + "yk1");
    holes(context, id + "ykRoll", yk, "y", s * 26, rz, yp0 - 1, yw + 1, 6, 24, 4.5, 0);       // 6 x M4 to roll output
    hole(context, id + "ykRollC", yk, "y", s * 26, rz, yp0 - 1, yw + 1, 10);                  // centre / pilot clearance
    holes(context, id + "ykPitch", yk, "x", 0, hz, s * 60, s * 68, 4, 38.5, 3.4, 45);          // 4 x M3 to pitch-motor rear face
    hole(context, id + "ykPitchC", yk, "x", 0, hz, s * 60, s * 68, 12);                       // cable pass-through
    tagPart(context, yk, "Hip yoke " ~ side, WHITE);

    // ---- Hip-pitch RS05: rear face on the yoke arm, output into the thigh ----
    rs05x(context, id + "pitchM", s * 67, s, 0, hz, "RS05 hip pitch " ~ side);

    // ---- Thigh geometry: straight tapered link, knee motor on the axis ----
    const L = sqrt(ky * ky + (kz - hz) * (kz - hz));
    const uy = ky / L;                        // unit vector hip -> knee
    const uz = (kz - hz) / L;
    const ny = -(kz - hz) / L;                // in-plane normal to the thigh axis
    const nz = ky / L;
    const my = KNEE_D * uy;                   // knee-motor axis, KNEE_D from the hip along the thigh
    const mz = hz + KNEE_D * uz;
    const RO1 = 30;                           // thigh outer radius at the hip
    const RO2 = 38;                           // thigh outer radius at the knee
    const RP1 = 20;                           // pocket radius at the hip (hip-pitch screw access)
    const RP2 = 31.5;                         // pocket radius at the knee (36T + belt)
    const XP1 = 151;                          // pocket outer face / cover inner face
    const XC1 = 156;                          // cover outer face

    hullYZ(context, id + "thigh", s * 114, s * XP1, 0, hz, RO1, ky, kz, RO2);
    hullYZ(context, id + "thighCav", s * 119, s * (XP1 + 1), 0, hz, RP1, ky, kz, RP2);
    const thigh = bq(id + "thigh");
    subtr(context, id + "thighCut", thigh, bq(id + "thighCav"));
    // cover screws: M2.5 countersunk into M2.5 heat-set inserts (3.6 mm holes)
    const cp = [[-25, 0], [L / 2, 29.9], [L / 2, -29.9], [L, 34.75], [L, -34.75], [L + 34.75, 0]];
    var coverPts = [];
    for (var k = 0; k < 6; k += 1)
        coverPts = append(coverPts, [cp[k][0] * uy + cp[k][1] * ny, hz + cp[k][0] * uz + cp[k][1] * nz]);
    for (var k = 0; k < 6; k += 1)
        hole(context, id + ("tbh" ~ k), thigh, "x", coverPts[k][0], coverPts[k][1], s * 140, s * (XP1 + 1), 3.6);
    holes(context, id + "thHip", thigh, "x", 0, hz, s * 113, s * 120, 6, 24, 4.5, 0);          // 6 x M4 to hip-pitch output
    holes(context, id + "thHipCb", thigh, "x", 0, hz, s * 117, s * 120, 6, 24, 8.0, 0);        // countersinks (flush in the pocket)
    hole(context, id + "thHipC", thigh, "x", 0, hz, s * 113, s * 120, 10);
    // knee-motor seat on the inboard face: boss slot + 8 x M3 slots (+-1.5 mm along the thigh for belt tension)
    hullYZ(context, id + "kmSlot", s * 113, s * 120, my - 1.5 * uy, mz - 1.5 * uz, 16, my + 1.5 * uy, mz + 1.5 * uz, 16);
    subtr(context, id + "kmSlotC", thigh, bq(id + "kmSlot"));
    var kmQ = [];
    for (var k = 0; k < 8; k += 1)
    {
        const a = (22.5 + k * 45) * degree;
        const cy = my + 20.75 * cos(a);
        const czz = mz + 20.75 * sin(a);
        hullYZ(context, id + ("kms" ~ k), s * 113, s * 120, cy - 1.5 * uy, czz - 1.5 * uz, 1.7, cy + 1.5 * uy, czz + 1.5 * uz, 1.7);
        hullYZ(context, id + ("kmc" ~ k), s * 116.8, s * 120, cy - 1.5 * uy, czz - 1.5 * uz, 3.25, cy + 1.5 * uy, czz + 1.5 * uz, 3.25);
        kmQ = concatenateArrays([kmQ, [bq(id + ("kms" ~ k)), bq(id + ("kmc" ~ k))]]);
    }
    subtr(context, id + "kmSlots", thigh, qUnion(kmQ));
    // belt-tension jack screw on the inboard face, knee side of the motor: pushes the motor toward the hip
    linkBox(context, id + "jack", s * 104, s * 114, my + 33 * uy, mz + 33 * uz, my + 27 * uy, mz + 27 * uz, 10, 0, 6);
    uni(context, id + "jackU", qUnion([thigh, bq(id + "jack")]));
    cyl(context, id + "jackH", P(s * 108.5, my + 34 * uy, mz + 34 * uz), P(s * 108.5, my + 26 * uy, mz + 26 * uz), 1.25);   // M3 tap drill
    subtr(context, id + "jackHc", thigh, bq(id + "jackH"));
    hole(context, id + "thKnee", thigh, "x", ky, kz, s * 113, s * 120, 16.05);                  // 688-2RS seat for the hub stub
    tagPart(context, thigh, "Thigh " ~ side, WHITE);

    // Cover (printed): flat outer face (the shin folds over it), 6706 for the hub boss, sliding carrier for the drive shaft
    hullYZ(context, id + "cover", s * XP1, s * XC1, 0, hz, RO1, ky, kz, RO2);
    const cover = bq(id + "cover");
    hole(context, id + "cvKnee", cover, "x", ky, kz, s * (XP1 - 1), s * (XC1 - 1), 37.05);      // 6706-2RS seat (30 x 37 x 4)
    hole(context, id + "cvKneeT", cover, "x", ky, kz, s * (XP1 - 1), s * (XC1 + 1), 33);
    // stepped obround seat for the drive-shaft bearing carrier: carrier pocket on the inside, 2 mm lip outside
    hullYZ(context, id + "cvCar", s * (XP1 - 1), s * (XP1 + 3), my - 1.5 * uy, mz - 1.5 * uz, 14.25, my + 1.5 * uy, mz + 1.5 * uz, 14.25);
    hullYZ(context, id + "cvCarO", s * (XP1 - 1), s * (XC1 + 1), my - 1.5 * uy, mz - 1.5 * uz, 10, my + 1.5 * uy, mz + 1.5 * uz, 10);
    subtr(context, id + "cvCarC", cover, qUnion([bq(id + "cvCar"), bq(id + "cvCarO")]));
    for (var k = 0; k < 2; k += 1)
    {
        const e = k == 0 ? 1 : -1;
        hullYZ(context, id + ("cvCs" ~ k), s * (XP1 + 2), s * (XC1 + 1), my + e * 11.75 * ny - 1.5 * uy, mz + e * 11.75 * nz - 1.5 * uz, 1.45,
                    my + e * 11.75 * ny + 1.5 * uy, mz + e * 11.75 * nz + 1.5 * uz, 1.45);
        subtr(context, id + ("cvCsC" ~ k), cover, bq(id + ("cvCs" ~ k)));                      // M2.5 clamp slots through the lip
    }
    for (var k = 0; k < 6; k += 1)
    {
        hole(context, id + ("cvb" ~ k), cover, "x", coverPts[k][0], coverPts[k][1], s * (XP1 - 1), s * (XC1 + 1), 2.9);
        hole(context, id + ("cvk" ~ k), cover, "x", coverPts[k][0], coverPts[k][1], s * (XC1 - 1.5), s * (XC1 + 1), 5.0);   // countersink
    }
    tagPart(context, cover, "Thigh cover " ~ side, WHITE);

    // Drive-shaft bearing carrier (aluminium): 6800-2RS (10 x 19 x 5), slides with the motor, clamped by 2 x M2.5 through the lip
    cyl(context, id + "car", P(s * XP1, my, mz), P(s * (XP1 + 3), my, mz), 14);
    hole(context, id + "carB", bq(id + "car"), "x", my, mz, s * (XP1 - 1), s * (XP1 + 4), 19.05);
    for (var k = 0; k < 2; k += 1)
    {
        const e = k == 0 ? 1 : -1;
        hole(context, id + ("carT" ~ k), bq(id + "car"), "x", my + e * 11.75 * ny, mz + e * 11.75 * nz, s * (XP1 - 1), s * (XP1 + 4), 2.05);
    }
    tagPart(context, bq(id + "car"), "Drive bearing carrier " ~ side, ALU);
    cyl(context, id + "b6800", P(s * XP1, my, mz), P(s * (XP1 + 5), my, mz), 9.5);
    cyl(context, id + "b6800i", P(s * (XP1 - 1), my, mz), P(s * (XP1 + 6), my, mz), 5);
    subtr(context, id + "b6800c", bq(id + "b6800"), bq(id + "b6800i"));
    tagPart(context, bq(id + "b6800"), "Bearing 6800-2RS drive shaft " ~ side, STEEL);

    // ---- Drive shaft adapter (steel): D30 flange on the knee RS05 rotor (6 x M4), D10 keyed shaft to the cover bearing ----
    cyl(context, id + "ad", P(s * 117, my, mz), P(s * 119, my, mz), 15);                       // D30 spigot in the wall slot
    cyl(context, id + "adF", P(s * 119, my, mz), P(s * 121, my, mz), 17);                      // D34 flange in the pocket
    cyl(context, id + "adS", P(s * 121, my, mz), P(s * (XP1 + 5), my, mz), 5);                 // D10 keyed shaft
    uni(context, id + "adU", qUnion([bq(id + "ad"), bq(id + "adF"), bq(id + "adS")]));
    holes(context, id + "adM4", bq(id + "ad"), "x", my, mz, s * 116, s * 122, 6, 24, 4.5, 0);
    holes(context, id + "adCs", bq(id + "ad"), "x", my, mz, s * 119.2, s * 122, 6, 24, 8, 0);     // M4 countersinks, flush under the pulley
    tagPart(context, bq(id + "ad"), "Knee drive shaft adapter " ~ side, STEEL);

    // ---- Knee drive pulley: 18T 5MGT-25 on the keyed D10 shaft ----
    pulleyTeeth(context, id + "pd", s * 122.2, s * 149, my, mz, 18, R_DRIVE);
    cyl(context, id + "pdF1", P(s * 121.2, my, mz), P(s * 122.2, my, mz), 17);
    cyl(context, id + "pdF2", P(s * 149, my, mz), P(s * 150.5, my, mz), 17);
    uni(context, id + "pdU", qUnion([bq(id + "pd"), bq(id + "pdF1"), bq(id + "pdF2")]));
    hole(context, id + "pdB", bq(id + "pd"), "x", my, mz, s * 119, s * 152, 10);
    tagPart(context, bq(id + "pd"), "Knee drive pulley 18T 5MGT-25 " ~ side, ALU);

    // ---- Knee hub: 36T 5MGT-25 + D8 stub in the thigh 688 + D30 boss through the cover 6706 to the shin ----
    pulleyTeeth(context, id + "ph", s * 122.5, s * 148.5, ky, kz, 36, R_DRIVEN);
    cyl(context, id + "phS", P(s * 114, ky, kz), P(s * 122.5, ky, kz), 4);
    cyl(context, id + "phB", P(s * 148.5, ky, kz), P(s * 158, ky, kz), 15);
    uni(context, id + "phU", qUnion([bq(id + "ph"), bq(id + "phS"), bq(id + "phB")]));
    const ph = bq(id + "ph");
    holes(context, id + "phL", ph, "x", ky, kz, s * 122, s * 149, 6, 42, 7, 0);                // lightening holes
    holes(context, id + "phM4", ph, "x", ky, kz, s * 150, s * 159, 4, 22, 3.3, 45);            // 4 x M4 tapped, shin bolts here
    tagPart(context, ph, "Knee hub 36T 5MGT-25 " ~ side, ALU);

    // ---- Belt 5MR-250-25 (50 teeth, 25 mm), backing only (teeth not modelled) ----
    hullYZ(context, id + "beltO", s * 123, s * 148, my, mz, R_DRIVE + BELT_T, ky, kz, R_DRIVEN + BELT_T);
    hullYZ(context, id + "beltI", s * 122, s * 149, my, mz, R_DRIVE, ky, kz, R_DRIVEN);
    subtr(context, id + "beltC", bq(id + "beltO"), bq(id + "beltI"));
    tagPart(context, bq(id + "beltO"), "Belt Gates 5MR-250-25 " ~ side, BELT);

    // ---- Knee bearings ----
    cyl(context, id + "b688", P(s * 114, ky, kz), P(s * 119, ky, kz), 8);
    cyl(context, id + "b688i", P(s * 113, ky, kz), P(s * 120, ky, kz), 4);
    subtr(context, id + "b688c", bq(id + "b688"), bq(id + "b688i"));
    tagPart(context, bq(id + "b688"), "Bearing 688-2RS knee " ~ side, STEEL);
    cyl(context, id + "b6706", P(s * XP1, ky, kz), P(s * (XP1 + 4), ky, kz), 18.5);
    cyl(context, id + "b6706i", P(s * (XP1 - 1), ky, kz), P(s * (XP1 + 5), ky, kz), 15);
    subtr(context, id + "b6706c", bq(id + "b6706"), bq(id + "b6706i"));
    tagPart(context, bq(id + "b6706"), "Bearing 6706-2RS knee " ~ side, STEEL);

    // ---- Knee RS05: inboard, on the thigh axis beside the hip-pitch motor, output outward through the inner wall ----
    rs05x(context, id + "kneeM", s * 70, s, my, mz, "RS05 knee " ~ side);

    // ---- Shin (printed), outboard of the cover: knee hub -> wheel motor ----
    mkLink(context, id + "shin", s * 158, s * 166, ky, kz, 0, R, 22, 28);
    const shin = bq(id + "shin" + "web");
    hole(context, id + "shK", shin, "x", ky, kz, s * 157, s * 167, 10);                        // centre relief
    holes(context, id + "shKc", shin, "x", ky, kz, s * 157, s * 167, 4, 22, 4.5, 45);          // 4 x M4 to the knee hub
    holes(context, id + "shW", shin, "x", 0, R, s * 157, s * 167, 4, 38.5, 3.4, 45);            // 4 x M3 to wheel-motor rear face
    hole(context, id + "shWc", shin, "x", 0, R, s * 157, s * 167, 12);
    tagPart(context, shin, "Shin " ~ side, WHITE);
    linkBox(context, id + "sstr", s * 166, s * 167, 0, R, ky, kz, 10, 30, 70);
    tagPart(context, bq(id + "sstr"), "Shin stripe " ~ side, ORANGE);
    cyl(context, id + "kcap", P(s * 166, ky, kz), P(s * 169, ky, kz), 20);
    holes(context, id + "kcapH", bq(id + "kcap"), "x", ky, kz, s * 165, s * 170, 4, 22, 4.5, 45);
    tagPart(context, bq(id + "kcap"), "Knee cap " ~ side, ORANGE);

    // ---- Wheel RS05 on the shin, output outward into the hub ----
    rs05x(context, id + "wheelM", s * 166, s, 0, R, "RS05 wheel " ~ side);

    const wc = s * 192.5;
    const TW = 24.5;                                                                            // tyre half width
    cyl(context, id + "hubd", P(s * 213, 0, R), P(s * 217, 0, R), 34);
    cyl(context, id + "rim", P(wc - s * TW, 0, R), P(wc + s * TW, 0, R), 35);
    cyl(context, id + "rimi", P(wc - s * (TW + 1), 0, R), P(wc + s * (TW - 0.5), 0, R), 31);
    subtr(context, id + "rimc", bq(id + "rim"), bq(id + "rimi"));
    uni(context, id + "hubu", qUnion([bq(id + "rim"), bq(id + "hubd")]));
    const hub = bq(id + "rim");
    holes(context, id + "hubDec", hub, "x", 0, R, s * 212, s * 218, 6, 46, 10, 30);            // lightening holes
    holes(context, id + "hubM4", hub, "x", 0, R, s * 212, s * 218, 6, 24, 4.5, 0);             // 6 x M4 to wheel-motor output
    hole(context, id + "hubC", hub, "x", 0, R, s * 212, s * 218, 10);
    holes(context, id + "hubCapP", hub, "x", 0, R, s * 212, s * 218, 3, 18, 2.0, 90);          // hub-cap pilots
    hole(context, id + "hubCapR", hub, "x", 0, R, s * 214.5, s * 218, 26.5);                   // recess: hub cap sits flush
    tagPart(context, hub, "Wheel hub " ~ side, GREY);

    cyl(context, id + "hubcap", P(s * 214.5, 0, R), P(s * 217, 0, R), 13);
    holes(context, id + "hubcapH", bq(id + "hubcap"), "x", 0, R, s * 214, s * 218, 3, 18, 2.5, 90);
    tagPart(context, bq(id + "hubcap"), "Hub cap " ~ side, ORANGE);

    cyl(context, id + "tyre", P(wc - s * TW, 0, R), P(wc + s * TW, 0, R), R);
    cyl(context, id + "tyri", P(wc - s * (TW + 1), 0, R), P(wc + s * (TW + 1), 0, R), 35);
    subtr(context, id + "tyrc", bq(id + "tyre"), bq(id + "tyri"));
    var gr = [];
    for (var k = 0; k < 20; k += 1)
    {
        const gid = id + ("g" ~ k);
        mkBox(context, gid, wc - 20, wc + 20, -2.5, 2.5, 2 * R - 2.5, 2 * R + 1);
        opTransform(context, gid + "r", { "bodies" : bq(gid), "transform" : rotationAround(line(P(0, 0, R), X_DIRECTION), k * 18 * degree) });
        gr = append(gr, bq(gid));
    }
    subtr(context, id + "tread", bq(id + "tyre"), qUnion(gr));
    tagPart(context, bq(id + "tyre"), "Tyre " ~ side ~ " (TPU)", TYRE);
}

// NUCLEO-144 (MB1364) mounting holes, board frame: bx across the 70 mm width, by along the 133.34 mm length,
// origin at the corner on the CN14 / user-USB end (UM2407 Fig. 6). The ST-LINK USB is at the far end (by = 133.34).
const NUCLEO_HOLES = [[12.14, 29.21], [57.86, 29.21], [43.89, 53.34], [10.87, 105.39], [59.13, 104.14]];

annotation { "Feature Type Name" : "BD Droid RS05 Inboard Slim" }
export const bdDroidRS05InboardSlim = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Hip axis height (mm)" }
        isReal(definition.hipZ, { (unitless) : [52, 210, 255] } as RealBoundSpec);
        annotation { "Name" : "Wheel radius (mm)" }
        isReal(definition.wheelR, { (unitless) : [20, 50, 100] } as RealBoundSpec);
    }
    {
        var def = definition;
        const hz = def.hipZ;
        const R = def.wheelR;
        const Lk = 105; // thigh = shin = 105 mm
        const half = (hz - R) / 2;
        def.kneeZ = R + half;
        def.kneeY = sqrt(max(Lk * Lk - half * half, 0));

        buildLeg(context, id + "R", def, 1, "R");
        buildLeg(context, id + "L", def, -1, "L");

        // =================== Torso ===================
        // Narrow box (+-60 x -70..+88): hip-roll motors + power/CAN board in the lower bay, Makita XGT 40V battery in the
        // upper bay. The pack slides in from the back onto an XGT rail dock under the lid (like a tool), release button
        // facing out of the rear opening. NUCLEO-H743ZI stands on the front wall, ST-LINK USB up through the lid.
        const tz0 = hz + 18;           // floor underside
        const fl = tz0 + WALL;         // floor top
        const zT = hz + 75;            // battery shelf underside (roll motors top out at hz + 68)
        const zB = zT + 4;             // battery underside
        const BH = 65;                 // XGT pack height including the rails (118 x 77 x 65)
        const zD = zB + BH;            // rail dock underside
        const tz1 = zD + 15;           // torso top (lid underside) = hz + 159
        const rz = hz + 45;
        const yb = TYB + WALL;         // -67 (rear wall inner)
        const yf = TYF - WALL;        //  85 (front wall inner)
        const xn = TXN - WALL;         // 57
        const BY0 = TYB;               // battery rear face, flush with the rear wall
        const BY1 = TYB + 118;         // battery front face

        mkBox(context, id + "torso", -TXN, TXN, TYB, TYF, tz0, tz1);
        try silent { opFillet(context, id + "torsoF", { "entities" : qParallelEdges(qCreatedBy(id + "torso", EntityType.EDGE), Z_DIRECTION), "radius" : 8 * millimeter }); }
        const torso = bq(id + "torso");
        mkBox(context, id + "torsoI", -xn, xn, yb, yf, fl, tz1 + 1);
        try silent { opFillet(context, id + "torsoIF", { "entities" : qParallelEdges(qCreatedBy(id + "torsoI", EntityType.EDGE), Z_DIRECTION), "radius" : 5 * millimeter }); }
        subtr(context, id + "torsoC", torso, bq(id + "torsoI"));

        // Internal features fused to the shell: lid bosses, shelf ledges, Nucleo standoffs, carrier posts
        var feats = [torso];
        const lidPts = [[51.5, -61.5], [-51.5, -61.5], [51.5, 79.5], [-51.5, 79.5]];
        for (var k = 0; k < 4; k += 1)
        {
            cyl(context, id + ("lb" ~ k), P(lidPts[k][0], lidPts[k][1], tz1 - 15), P(lidPts[k][0], lidPts[k][1], tz1), 6);
            feats = append(feats, bq(id + ("lb" ~ k)));
        }
        const shelfPts = [[53.5, -10], [-53.5, -10], [53.5, 45], [-53.5, 45]];
        for (var k = 0; k < 4; k += 1)
        {
            const sx = shelfPts[k][0] > 0 ? 1 : -1;
            mkBox(context, id + ("tl" ~ k), sx * 50, sx * (xn + 0.5), shelfPts[k][1] - 7, shelfPts[k][1] + 7, zT - 8, zT);
            feats = append(feats, bq(id + ("tl" ~ k)));
        }
        // Nucleo: vertical on the front wall, components facing back, ST-LINK USB end (by = 133.34) at the top.
        const nyS = yf - 12;           // board front face (y); 12 mm standoffs clear the morpho pins
        const nb0 = fl + 2;            // board bottom edge (z)
        var nPts = [];
        for (var k = 0; k < 5; k += 1)
        {
            nPts = append(nPts, [NUCLEO_HOLES[k][0] - 35, nb0 + NUCLEO_HOLES[k][1]]);
            cyl(context, id + ("ns" ~ k), P(nPts[k][0], yf + 0.5, nPts[k][1]), P(nPts[k][0], nyS, nPts[k][1]), 3.5);
            feats = append(feats, bq(id + ("ns" ~ k)));
        }
        // power / CAN carrier board posts (36 x 70 board at x -55..-19, y -20..50)
        const pbPts = [[-51.5, -16.5], [-22.5, -16.5], [-51.5, 46.5], [-22.5, 46.5]];
        for (var k = 0; k < 4; k += 1)
        {
            cyl(context, id + ("ps" ~ k), P(pbPts[k][0], pbPts[k][1], fl - 0.5), P(pbPts[k][0], pbPts[k][1], fl + 5), 3.2);
            feats = append(feats, bq(id + ("ps" ~ k)));
        }
        uni(context, id + "torsoU", qUnion(feats));

        for (var k = 0; k < 4; k += 1)
        {
            hole(context, id + ("lbh" ~ k), torso, "z", lidPts[k][0], lidPts[k][1], tz1 - 10, tz1 + 1, 4.0);         // lid inserts
            hole(context, id + ("tlh" ~ k), torso, "z", shelfPts[k][0], shelfPts[k][1], zT - 7, zT + 1, 4.0);        // shelf inserts
        }
        for (var k = 0; k < 5; k += 1)
            hole(context, id + ("nsh" ~ k), torso, "y", nPts[k][0], nPts[k][1], nyS - 1, yf - 2, 4.0);              // Nucleo inserts
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("psh" ~ k), torso, "z", pbPts[k][0], pbPts[k][1], fl + 1, fl + 6, 4.0);             // carrier inserts
        // battery opening in the rear wall, above the hip yokes
        mkBox(context, id + "battOpen", -40, 40, TYB - 1, yb + 1, zB - 1, zD + 1);
        subtr(context, id + "battOpenC", torso, bq(id + "battOpen"));
        // roll-motor bores + 8 x M3 rings in the rear wall, leg cable slots
        for (var s = -1; s <= 1; s += 2)
        {
            const sid = s > 0 ? "R" : "L";
            hole(context, id + ("rb" ~ sid), torso, "y", s * 26, rz, TYB - 1, yb + 1, 36);
            holes(context, id + ("rr" ~ sid), torso, "y", s * 26, rz, TYB - 1, yb + 1, 8, 41.5, 3.4, 0);
            mkBox(context, id + ("cs" ~ sid), s * 49, s * 56, TYB - 1, yb + 1, fl + 3, fl + 13);
            subtr(context, id + ("csC" ~ sid), torso, bq(id + ("cs" ~ sid)));
        }
        tagPart(context, torso, "Torso shell", WHITE);

        // Chest details on the front wall
        const cz = hz + 100;
        mkBox(context, id + "chest", -26, 26, TYF, TYF + 4, cz - 27, cz + 27);
        for (var k = 0; k < 4; k += 1)
            mkBox(context, id + ("slot" ~ k), -16, 16, TYF + 2, TYF + 5, cz - 18 + k * 8, cz - 14 + k * 8);
        subtr(context, id + "chestC", bq(id + "chest"), qUnion([bq(id + "slot0"), bq(id + "slot1"), bq(id + "slot2"), bq(id + "slot3")]));
        tagPart(context, bq(id + "chest"), "Chest panel", DARK);
        mkBox(context, id + "cled", -6, 6, TYF + 4, TYF + 6, cz + 18, cz + 24);
        tagPart(context, bq(id + "cled"), "Chest LED", ORANGE);
        mkBox(context, id + "stripe", -12, 12, TYF, TYF + 2, cz + 33, tz1 - 5);
        tagPart(context, bq(id + "stripe"), "Chest stripe", ORANGE);
        mkBox(context, id + "stripeL", -46, -38, TYF, TYF + 2, cz - 27, cz + 27);
        mkBox(context, id + "stripeR", 38, 46, TYF, TYF + 2, cz - 27, cz + 27);
        tagPart(context, bq(id + "stripeL"), "Chest bar L", ORANGE);
        tagPart(context, bq(id + "stripeR"), "Chest bar R", ORANGE);

        // ---- Battery shelf (printed PETG): catches the pack under the rail, lips guide it in ----
        mkBox(context, id + "tray", -55, 55, yb + 2, BY1 + 6, zT, zB);
        mkBox(context, id + "trR", 39.5, 42.5, yb + 2, BY1 + 6, zB, zB + 10);
        mkBox(context, id + "trL", -39.5, -42.5, yb + 2, BY1 + 6, zB, zB + 10);
        mkBox(context, id + "trF", -42.5, 42.5, BY1 + 3, BY1 + 6, zB, zB + 10);
        uni(context, id + "trU", qUnion([bq(id + "tray"), bq(id + "trR"), bq(id + "trL"), bq(id + "trF")]));
        const tray = bq(id + "tray");
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("trh" ~ k), tray, "z", shelfPts[k][0], shelfPts[k][1], zT - 1, zB + 1, 3.4);
        mkBox(context, id + "wireN", -10, 10, BY1 + 1, BY1 + 7, zT - 1, zB + 1);    // battery leads pass-down
        subtr(context, id + "trayC", tray, bq(id + "wireN"));
        tagPart(context, tray, "Battery shelf", DARK);

        // ---- Makita XGT 40V max battery: BL4025 2.5 Ah (36 V nominal, 90 Wh, 118 x 77 x 65 mm, 0.71 kg), reference ----
        mkBox(context, id + "batt", -38.5, 38.5, BY0, BY1, zB, zB + BH - 12);
        mkBox(context, id + "battR", -27, 27, BY0 + 18, BY1, zB + BH - 12, zB + BH);       // slide rail / terminal block
        uni(context, id + "battU", qUnion([bq(id + "batt"), bq(id + "battR")]));
        tagPart(context, bq(id + "batt"), "Battery Makita XGT BL4025 40V 2.5Ah (ref)", color(0.12, 0.45, 0.5));
        mkBox(context, id + "battBtn", -12, 12, BY0 - 1, BY0, zB + 30, zB + 44);
        tagPart(context, bq(id + "battBtn"), "XGT release button (ref)", DARK);

        // ---- XGT rail dock (third-party battery adapter with 12 AWG leads, reference) + printed carrier on the lid ----
        mkBox(context, id + "dock", -30, 30, BY0 + 16, BY1 + 2, zD - 12, zD + 3);
        mkBox(context, id + "dockS", -27.5, 27.5, BY0 + 15, BY1 + 3, zD - 13, zD);            // rail slot the pack slides into
        subtr(context, id + "dockSC", bq(id + "dock"), bq(id + "dockS"));
        tagPart(context, bq(id + "dock"), "XGT battery dock adapter, 12 AWG leads (ref)", color(0.2, 0.2, 0.22));
        mkBox(context, id + "dockC", -45, 45, BY0 + 12, BY1 + 6, zD + 3, tz1);
        mkBox(context, id + "dockCg", 0, 46, 1, 11, tz1 - 4, tz1 + 1);                 // neck-cable groove to the side gap
        subtr(context, id + "dockCgC", bq(id + "dockC"), bq(id + "dockCg"));
        tagPart(context, bq(id + "dockC"), "Battery dock carrier", DARK);

        // ---- NUCLEO-H743ZI (reference) ----
        mkBox(context, id + "nucleo", -35, 35, nyS - 1.6, nyS, nb0, nb0 + 133.34);
        for (var k = 0; k < 5; k += 1)
            hole(context, id + ("nh" ~ k), bq(id + "nucleo"), "y", nPts[k][0], nPts[k][1], nyS - 3, nyS + 1, 3.2);
        tagPart(context, bq(id + "nucleo"), "NUCLEO-H743ZI (ref)", PCB);
        mkBox(context, id + "nucleoParts", -33, 33, nyS - 1.6 - 16.4, nyS - 1.6, nb0 + 2, nb0 + 131);
        tagPart(context, bq(id + "nucleoParts"), "NUCLEO component envelope (ref)", color(0.1, 0.35, 0.15));

        // ---- Power / CAN carrier board (reference): 48->12 V isolated DC-DC, 48->5 V SIP, CAN transceiver, fuse ----
        mkBox(context, id + "pcb", -55, -19, -20, 50, fl + 5, fl + 6.6);
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("pch" ~ k), bq(id + "pcb"), "z", pbPts[k][0], pbPts[k][1], fl + 4, fl + 8, 3.2);
        tagPart(context, bq(id + "pcb"), "Power-CAN carrier PCB (ref)", PCB);
        mkBox(context, id + "dc12", -49.7, -24.3, -17, 33.8, fl + 6.6, fl + 16.8);
        tagPart(context, bq(id + "dc12"), "DC-DC 36V-12V 30W 2x1in, 18-75V in, e.g. Mean Well SKMW30F-12 (ref)", DARK);
        mkBox(context, id + "dc5", -42, -30.5, 37, 45.5, fl + 6.6, fl + 24.1);
        tagPart(context, bq(id + "dc5"), "DC-DC 36V-5V 0.5A SIP, 9-72V in, e.g. RECOM R-78HB5.0-0.5 (ref)", DARK);
        mkBox(context, id + "can", -27, -20, 36, 48, fl + 6.6, fl + 9.6);
        tagPart(context, bq(id + "can"), "CAN transceiver TJA1051T-3 + 120R (ref)", DARK);
        mkBox(context, id + "fuse", -10, 2, -18, 10, fl + 1, fl + 12);
        tagPart(context, bq(id + "fuse"), "Inline mini blade fuse holder 20 A (ref)", color(0.6, 0.1, 0.1));

        // ---- Lid ----
        mkBox(context, id + "lid", -TXN, TXN, TYB, TYF, tz1, tz1 + 4);
        try silent { opFillet(context, id + "lidF", { "entities" : qParallelEdges(qCreatedBy(id + "lid", EntityType.EDGE), Z_DIRECTION), "radius" : 8 * millimeter }); }
        const lid = bq(id + "lid");
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("lh" ~ k), lid, "z", lidPts[k][0], lidPts[k][1], tz1 - 1, tz1 + 5, 3.4);
        const collarPts = [[22, -27], [-22, -27], [22, 21], [-22, 21]];
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("lnh" ~ k), lid, "z", collarPts[k][0], collarPts[k][1], tz1 - 1, tz1 + 5, 4.0);   // neck-collar inserts
        hole(context, id + "lcab", lid, "z", 0, 6, tz1 - 1, tz1 + 5, 12);                                          // Dynamixel bus / head wiring
        mkBox(context, id + "lusb", 3, 23, nyS - 20, nyS + 2, tz1 - 1, tz1 + 5);                                   // ST-LINK USB access
        subtr(context, id + "lusbC", lid, bq(id + "lusb"));
        const dockPts = [[38, BY0 + 20], [-38, BY0 + 20], [38, BY1], [-38, BY1]];
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("ldh" ~ k), lid, "z", dockPts[k][0], dockPts[k][1], tz1 - 1, tz1 + 5, 3.4);        // M3 into the dock carrier
        tagPart(context, lid, "Torso lid", WHITE);

        // =================== 3-DOF neck: yaw (collar) -> pitch (stalk) -> roll (in head) ===================
        // 3 x Dynamixel XL430-W250-T (28.5 x 34 x 46.5 mm, 57.2 g, 1.4 N m stall @ 11.1 V).
        // Envelope convention: horn axis along the 34 mm side, horn centre 11.25 mm from one end of the 46.5 mm side.
        // Dynamixel horn / body screw patterns are NOT modelled (ROBOTIS drawing not available here): drill from the
        // XL430 drawing or use stock FR12 frames.
        const Lt = tz1 + 4;              // top of lid
        const NY = -6;                   // neck axis y
        // collar (printed) on the lid, houses the yaw servo
        mkBox(context, id + "collar", -18, 18, -21.5, 33.5, Lt, Lt + 35);
        try silent { opFillet(context, id + "collarF", { "entities" : qParallelEdges(qCreatedBy(id + "collar", EntityType.EDGE), Z_DIRECTION), "radius" : 8 * millimeter }); }
        mkBox(context, id + "collarI", -14.5, 14.5, -18.5, 30.5, Lt - 1, Lt + 32);
        subtr(context, id + "collarC", bq(id + "collar"), bq(id + "collarI"));
        // mounting flange with 4 x M3 to the lid inserts
        mkBox(context, id + "collarFl", -27, 27, -32, 26, Lt, Lt + 4);
        try silent { opFillet(context, id + "collarFlF", { "entities" : qParallelEdges(qCreatedBy(id + "collarFl", EntityType.EDGE), Z_DIRECTION), "radius" : 6 * millimeter }); }
        uni(context, id + "collarU", qUnion([bq(id + "collar"), bq(id + "collarFl")]));
        mkBox(context, id + "collarFlI", -14.5, 14.5, -18.5, 30.5, Lt - 1, Lt + 5);
        subtr(context, id + "collarFlC", bq(id + "collar"), bq(id + "collarFlI"));
        hole(context, id + "collarHorn", bq(id + "collar"), "z", 0, NY, Lt + 31, Lt + 36, 26);
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("colh" ~ k), bq(id + "collar"), "z", collarPts[k][0], collarPts[k][1], Lt - 1, Lt + 5, 3.4);
        tagPart(context, bq(id + "collar"), "Neck collar", ORANGE);

        // yaw XL430: horn up, body inside the collar
        mkBox(context, id + "yawS", -14.25, 14.25, NY - 11.25, NY + 35.25, Lt + 1, Lt + 35);
        cyl(context, id + "yawH", P(0, NY, Lt + 35), P(0, NY, Lt + 38), 11);
        tagPart(context, bq(id + "yawS"), "XL430-W250 neck yaw", DARK);
        tagPart(context, bq(id + "yawH"), "XL430 horn (yaw)", GREY);

        // yaw turntable (printed) carrying the pitch servo
        cyl(context, id + "turn", P(0, NY, Lt + 38), P(0, NY, Lt + 41), 19);
        mkBox(context, id + "turnW", -19, 19, NY - 16, NY + 16, Lt + 38, Lt + 41);
        uni(context, id + "turnU", qUnion([bq(id + "turn"), bq(id + "turnW")]));
        hole(context, id + "turnC", bq(id + "turn"), "z", 0, NY, Lt + 37, Lt + 42, 6);
        tagPart(context, bq(id + "turn"), "Neck yaw turntable", WHITE);

        // pitch XL430: standing on the turntable, horn axis along x (horn on +x)
        const pz = Lt + 41 + 46.5 - 11.25;           // pitch axis height
        mkBox(context, id + "pitS", -17, 17, NY - 14.25, NY + 14.25, Lt + 41, Lt + 87.5);
        cyl(context, id + "pitH", P(17, NY, pz), P(20, NY, pz), 11);
        cyl(context, id + "pitI", P(-17, NY, pz), P(-20, NY, pz), 8);
        tagPart(context, bq(id + "pitS"), "XL430-W250 neck pitch", DARK);
        tagPart(context, bq(id + "pitH"), "XL430 horn (pitch)", GREY);
        tagPart(context, bq(id + "pitI"), "XL430 idler (pitch)", GREY);

        // pitch U-bracket (printed): horn + idler arms joined by a crossbar that carries the roll servo
        const cb0 = pz + 20;
        mkBox(context, id + "pbR", 20, 23, NY - 12, NY + 12, pz, cb0 + 3);
        cyl(context, id + "pbRe", P(20, NY, pz), P(23, NY, pz), 12);
        mkBox(context, id + "pbL", -20, -23, NY - 12, NY + 12, pz, cb0 + 3);
        cyl(context, id + "pbLe", P(-20, NY, pz), P(-23, NY, pz), 12);
        mkBox(context, id + "pbX", -23, 23, NY - 14, NY + 14, cb0, cb0 + 3);
        uni(context, id + "pbU", qUnion([bq(id + "pbX"), bq(id + "pbR"), bq(id + "pbRe"), bq(id + "pbL"), bq(id + "pbLe")]));
        tagPart(context, bq(id + "pbX"), "Neck pitch bracket", WHITE);

        // roll XL430: standing on the crossbar, horn axis along y (horn forward), centred under the head
        const rzN = cb0 + 3 + 46.5 - 11.25;          // roll axis height
        mkBox(context, id + "rolS", -14.25, 14.25, NY - 17, NY + 17, cb0 + 3, cb0 + 49.5);
        cyl(context, id + "rolH", P(0, NY + 17, rzN), P(0, NY + 20, rzN), 11);
        cyl(context, id + "rolI", P(0, NY - 17, rzN), P(0, NY - 20, rzN), 8);
        tagPart(context, bq(id + "rolS"), "XL430-W250 neck roll", DARK);
        tagPart(context, bq(id + "rolH"), "XL430 horn (roll)", GREY);
        tagPart(context, bq(id + "rolI"), "XL430 idler (roll)", GREY);

        // =================== Head ===================
        const hz0 = Lt + 95;
        const hz1 = hz0 + 72;
        mkBox(context, id + "head", -88, 88, -38, 40, hz0, hz1);
        try silent { opFillet(context, id + "headF", { "entities" : qParallelEdges(qCreatedBy(id + "head", EntityType.EDGE), Y_DIRECTION), "radius" : 10 * millimeter }); }
        try silent { opChamfer(context, id + "headCh", { "entities" : qCoincidesWithPlane(qParallelEdges(qCreatedBy(id + "head", EntityType.EDGE), X_DIRECTION), plane(P(0, 40, 0), Y_DIRECTION)),
                        "chamferType" : ChamferType.EQUAL_OFFSETS, "width" : 9 * millimeter }); }
        mkBox(context, id + "headI", -85.5, 85.5, -39, 37.5, hz0 + 2.5, hz1 - 2.5);       // open at the back
        try silent { opFillet(context, id + "headIF", { "entities" : qParallelEdges(qCreatedBy(id + "headI", EntityType.EDGE), Y_DIRECTION), "radius" : 7.5 * millimeter }); }
        const head = bq(id + "head");
        subtr(context, id + "headC", head, bq(id + "headI"));
        // bosses: rear-cover inserts in the corners
        var hb = [head];
        const rcPts = [[76, hz0 + 12], [-76, hz0 + 12], [76, hz1 - 12], [-76, hz1 - 12]];
        for (var k = 0; k < 4; k += 1)
        {
            mkBox(context, id + ("hrb" ~ k), rcPts[k][0] - 5, rcPts[k][0] + 5, -38, -28, rcPts[k][1] - 5, rcPts[k][1] + 5);
            hb = append(hb, bq(id + ("hrb" ~ k)));
        }
        // ribs tying the corner bosses to the walls
        mkBox(context, id + "hrr1", 76, 86, -38, -28, hz0 + 1, hz0 + 17);
        mkBox(context, id + "hrr2", -76, -86, -38, -28, hz0 + 1, hz0 + 17);
        mkBox(context, id + "hrr3", 76, 86, -38, -28, hz1 - 17, hz1 - 1);
        mkBox(context, id + "hrr4", -76, -86, -38, -28, hz1 - 17, hz1 - 1);
        hb = concatenateArrays([hb, [bq(id + "hrr1"), bq(id + "hrr2"), bq(id + "hrr3"), bq(id + "hrr4")]]);
        uni(context, id + "headU", qUnion(hb));
        for (var k = 0; k < 4; k += 1)
        {
            hole(context, id + ("hrh" ~ k), head, "y", rcPts[k][0], rcPts[k][1], -39, -30, 4.0);
        }
        // floor opening for the neck pitch bracket / roll servo (clears +-0.35 rad head roll)
        mkBox(context, id + "hfloor", -40, 40, NY - 22, NY + 22, hz0 - 1, hz0 + 3);
        subtr(context, id + "hfloorC", head, bq(id + "hfloor"));
        // 4 x M3 through the head top into the head mount
        const hmPts = [[10, NY - 12], [-10, NY - 12], [10, NY + 12], [-10, NY + 12]];
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("hth" ~ k), head, "z", hmPts[k][0], hmPts[k][1], hz1 - 4, hz1 + 1, 3.4);
        const ez = hz0 + 34;
        holes(context, id + "hEye", head, "y", 42, ez, 36, 41, 3, 40, 2.0, 90);           // main-eye bezel pilots
        hole(context, id + "hEyeW", head, "y", 42, ez, 36, 41, 24);                        // camera / LED window behind lens
        holes(context, id + "hEye2", head, "y", -42, ez, 36, 41, 3, 30, 2.0, 90);         // secondary-eye pilots
        hole(context, id + "hEarR", head, "x", 2, ez, 84, 89, 3.4);
        hole(context, id + "hEarL", head, "x", 2, ez, -89, -84, 3.4);
        hole(context, id + "hAntR", head, "z", 66, -24, hz1 - 4, hz1 + 1, 8.2);
        hole(context, id + "hAntL", head, "z", -66, -24, hz1 - 4, hz1 + 1, 8.2);
        tagPart(context, head, "Head shell", WHITE);

        // head mount (printed): plates on the roll horn and idler, bridged over the servo, screwed to the head top
        mkBox(context, id + "hmF", -16, 16, NY + 20, NY + 23, rzN - 14, hz1 - 2.5);
        mkBox(context, id + "hmR", -16, 16, NY - 23, NY - 20, rzN - 14, hz1 - 2.5);
        mkBox(context, id + "hmT", -16, 16, NY - 23, NY + 23, cb0 + 50.5, hz1 - 2.5);
        uni(context, id + "hmU", qUnion([bq(id + "hmT"), bq(id + "hmF"), bq(id + "hmR")]));
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("hmh" ~ k), bq(id + "hmT"), "z", hmPts[k][0], hmPts[k][1], hz1 - 13, hz1 - 2, 4.0);
        hole(context, id + "hmHornF", bq(id + "hmT"), "y", 0, rzN, NY + 19, NY + 24, 6);
        hole(context, id + "hmHornR", bq(id + "hmT"), "y", 0, rzN, NY - 24, NY - 19, 6);
        tagPart(context, bq(id + "hmT"), "Head mount", WHITE);

        // rear cover
        mkBox(context, id + "hrc", -88, 88, -41, -38, hz0, hz1);
        try silent { opFillet(context, id + "hrcF", { "entities" : qParallelEdges(qCreatedBy(id + "hrc", EntityType.EDGE), Y_DIRECTION), "radius" : 10 * millimeter }); }
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("hrch" ~ k), bq(id + "hrc"), "y", rcPts[k][0], rcPts[k][1], -42, -37, 3.4);
        tagPart(context, bq(id + "hrc"), "Head rear cover", WHITE);

        // Orange bands on head
        mkBox(context, id + "hb1", 40, 62, -41.5, 40.5, hz1 - 1, hz1 + 1);
        mkBox(context, id + "hb2", -62, -40, -41.5, 40.5, hz1 - 1, hz1 + 1);
        mkBox(context, id + "hb3", -20, 20, 39, 41, hz1 - 14, hz1 - 6);
        tagPart(context, bq(id + "hb1"), "Head band R", ORANGE);
        tagPart(context, bq(id + "hb2"), "Head band L", ORANGE);
        tagPart(context, bq(id + "hb3"), "Brow stripe", ORANGE);

        // Big eye (robot right)
        cyl(context, id + "eyeB", P(42, 40, ez), P(42, 46, ez), 23);
        cyl(context, id + "eyeBi", P(42, 43, ez), P(42, 47, ez), 17);
        subtr(context, id + "eyeBc", bq(id + "eyeB"), bq(id + "eyeBi"));
        holes(context, id + "eyeBh", bq(id + "eyeB"), "y", 42, ez, 39, 47, 3, 40, 2.5, 90);
        tagPart(context, bq(id + "eyeB"), "Main eye bezel", ORANGE);
        cyl(context, id + "lens", P(42, 40, ez), P(42, 45, ez), 17);
        tagPart(context, bq(id + "lens"), "Main eye lens", color(0.1, 0.3, 0.85));
        // Small eye (robot left)
        cyl(context, id + "eye2", P(-42, 40, ez), P(-42, 48, ez), 18);
        cyl(context, id + "eye2b", P(-42, 48, ez), P(-42, 52, ez), 12);
        uni(context, id + "eye2u", qUnion([bq(id + "eye2"), bq(id + "eye2b")]));
        hole(context, id + "eye2c", bq(id + "eye2"), "y", -42, ez, 46, 53, 12);
        holes(context, id + "eye2h", bq(id + "eye2"), "y", -42, ez, 39, 49, 3, 30, 2.5, 90);
        tagPart(context, bq(id + "eye2"), "Secondary eye", GREY);
        mkBox(context, id + "screen", -14, 14, 40, 42, ez - 10, ez + 10);
        tagPart(context, bq(id + "screen"), "Face screen", DARK);
        // Ear pods
        for (var s = -1; s <= 1; s += 2)
        {
            const sid = s > 0 ? "R" : "L";
            cyl(context, id + ("ear" ~ sid), P(s * 88, 2, ez), P(s * 96, 2, ez), 20);
            cyl(context, id + ("earI" ~ sid), P(s * 94, 2, ez), P(s * 97, 2, ez), 10);
            subtr(context, id + ("earC" ~ sid), bq(id + ("ear" ~ sid)), bq(id + ("earI" ~ sid)));
            hole(context, id + ("earH" ~ sid), bq(id + ("ear" ~ sid)), "x", 2, ez, s * 87, s * 97, 3.4);
            tagPart(context, bq(id + ("ear" ~ sid)), "Ear pod " ~ sid, ORANGE);
            // Antenna: printed base + 3 mm steel rod
            cyl(context, id + ("antB" ~ sid), P(s * 66, -24, hz1 - 3), P(s * 66, -24, hz1 + 6), 4);
            hole(context, id + ("antBh" ~ sid), bq(id + ("antB" ~ sid)), "z", s * 66, -24, hz1 - 4, hz1 + 7, 3.1);
            tagPart(context, bq(id + ("antB" ~ sid)), "Antenna base " ~ sid, GREY);
            cyl(context, id + ("ant" ~ sid), P(s * 66, -24, hz1 - 2), P(s * 76, -30, hz1 + 75), 1.5);
            tagPart(context, bq(id + ("ant" ~ sid)), "Antenna rod " ~ sid ~ " (3 mm steel)", GREY);
        }

        // remove leftover sketch wire / point bodies from the prism helpers
        const allB = qCreatedBy(id, EntityType.BODY);
        const junk = qSubtraction(allB, qBodyType(allB, BodyType.SOLID));
        if (size(evaluateQuery(context, junk)) > 0)
            opDeleteBodies(context, id + "cleanup", { "entities" : junk });
    }, { hipZ : 210, wheelR : 50 });
