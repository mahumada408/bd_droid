FeatureScript 3112;
import(path : "onshape/std/common.fs", version : "3112.0");

// BD-style wheel-legged droid, built around RobStride 05 actuators,
// a Makita XGT 40V max tool battery (BL4025, slides in from the back) and a NUCLEO-H743ZI (Nucleo-144, MB1364).
// Frame: x = robot right, y = forward, z = up, ground at z = 0, axle midpoint at x = 0, y = 0.
// All dimensions in mm.
//
// RS05 interfaces (user manual 1.1 outline drawing):
//   output (rotor): D30 shaft standing 3 mm proud of the housing, 6 x M4 on PCD 24 (threads only 3.5 deep: screw
//                   length = grip + 3), 3 x D4 H7 dowel holes on PCD 19, D9 centre bore.
//   output-side housing (stator): 8 x M3 on PCD 41.5 (7 deep), D35 x 0.5 step around the shaft.
//   rear face : 4 x M3 on PCD 38.5 (10 deep). Power + CAN on an AMASS XT30PB(2+2) (mate: XT30(2+2)-F).
//   1.6 N m rated, 5.5 N m peak, 480 rpm no-load at 48 V, 191 g, D46 x 44 body.
// Printed fits (calibrate with CAD/print/fit_coupon.stl first): clearance M3 3.4, M4 4.5; heat-set M3 x 5.7
// inserts 4.0; D4 dowel press 3.9; 6800 seat 19.1 (clamped by a cap); stator bores 35.3.
//
// DEFAULT DESIGN v3.1 (after inspiration_robot.mp4): gimbal hip, every joint direct drive, full crouch, buildable.
//   hip       : the roll (fore-aft) and pitch (lateral) axes intersect at the hip point (x +-32, y 10), inside the
//               lower torso 33 mm above its bottom. Nothing hangs below the torso, so the legs fold beside it until
//               the torso sits on the ground (hip 33 mm up, thigh about horizontal, knee motor ahead of the torso).
//   hip roll  : RS05, axis along y, clamped at both ends: rear face in a pocket in the rear wall (4 x M3), stator
//               on the mid bulkhead (6 x M3 countersunk; the bulkhead has a top-open U-slot so it drops in).
//   gimbal    : one printed frame (back, front and outer plates; open on the inner side). Back plate on the roll
//               output (6 x M4 countersunk + 3 dowels), front stub in a 6800-2RS held by a bolt-on bearing cap,
//               pitch stator on the outer plate (8 x M3).
//   thigh     : 9 mm plate (x 61..70) with a hub boss reaching through the outer-plate bore to the pitch output.
//               It swings in a hip socket (side-wall notch); ahead of it the lower side walls step in to x +-48
//               (front recess), so the thigh clears the torso over the full roll range at every crouch height. Flush hip-cap inlay
//               so the shin can fold over the hip.
//   knee      : RS05 on the thigh's inner side at the knee (x 17..61); output through the thigh bore to the shin hub.
//   shin      : 8 mm plate (x 71..79) with a harness channel under the orange stripe cover; wheel RS05 (x 79..123)
//               inside a crowned 49 mm TPU tyre (x 81..130) on a printed hub.
//   Track 211 mm. The torso's front-bottom edge is rounded (r 30) so the knee motor clears it while folding.
// Assembly order and the parts list: docs/BUILD.md. Set "Hip axis height" to 33 mm to see the robot sitting.
// Earlier designs: CAD/variants/ (bd_droid_rs05_cradle_hip.fs = v2, bd_droid_rs05_inboard_slim.fs = belt knee).

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

// 90 deg countersinks for a bolt circle (head radius hr: M3 DIN 7991 3.2, M4 4.1), cut into target from the
// face at a0, going in direction dir (+1 / -1 along the axis).
function csinks(context is Context, id is Id, target is Query, ax is string, cu, cv, a0, dir, n, pcd, phase, hr)
{
    var tools = [];
    for (var k = 0; k < n; k += 1)
    {
        const ang = phase + k * 360 / n;
        const u = cu + pcd / 2 * cos(ang * degree);
        const v = cv + pcd / 2 * sin(ang * degree);
        fCone(context, id + ("c" ~ k), { "topCenter" : axPt(ax, a0 - dir * 0.3, u, v), "bottomCenter" : axPt(ax, a0 + dir * (hr - 1.0), u, v),
                    "topRadius" : (hr + 0.3) * millimeter, "bottomRadius" : 1.0 * millimeter });
        tools = append(tools, bq(id + ("c" ~ k)));
    }
    subtr(context, id + "cut", target, qUnion(tools));
}

// RS05 output (rotor) interface, cut through target along ax from a0 to a1 at (cu, cv):
// 6 x M4 clearance on PCD 24, 3 x D4 dowel holes (printed 3.9 = press fit for D4 x 8 pins) on PCD 19, D9.5 centre.
// Counterbores D8 from cb0 to cb1 (skipped when cb0 == cb1) set the screw grip: the rotor threads are only
// 3.5 deep, so grip + 3 = screw length.
function rotorIface(context is Context, id is Id, target is Query, ax is string, cu, cv, a0, a1, cb0, cb1)
{
    holes(context, id + "m4", target, ax, cu, cv, a0, a1, 6, 24, 4.5, 0);
    holes(context, id + "dw", target, ax, cu, cv, a0, a1, 3, 19, 3.9, 30);
    hole(context, id + "c", target, ax, cu, cv, a0, a1, 9.5);
    if (cb0 != cb1)
        holes(context, id + "cb", target, ax, cu, cv, cb0, cb1, 6, 24, 8, 0);
}

// RS05 stator (output-side housing) interface: D35.3 bore (clears the 30 mm output shaft and locates on the
// 35 mm step), 8 x M3 on PCD 41.5 (threads 7 deep). cdir != 0: countersunk from the face at cf, cdir = direction
// from that face into the part.
function statorIface(context is Context, id is Id, target is Query, ax is string, cu, cv, a0, a1, cf, cdir)
{
    hole(context, id + "b", target, ax, cu, cv, a0, a1, 35.3);
    holes(context, id + "s", target, ax, cu, cv, a0, a1, 8, 41.5, 3.4, 22.5);
    if (cdir != 0)
        csinks(context, id + "k", target, ax, cu, cv, cf, cdir, 8, 41.5, 22.5, 3.2);
}

const WHITE = color(0.86, 0.83, 0.76);
const ORANGE = color(0.85, 0.38, 0.14);
const DARK = color(0.22, 0.22, 0.23);
const TYRE = color(0.08, 0.08, 0.08);
const GREY = color(0.55, 0.56, 0.58);
const STEEL = color(0.45, 0.46, 0.48);
const PCB = color(0.1, 0.45, 0.2);

// Torso / layout constants
const TXN = 60;     // torso half width
const TYB = -70;    // torso rear outer face
const TYF = 88;     // torso front outer face
const WALL = 3;
const RX = 32;      // hip point |x| (roll axis)
const HY = 10;      // hip point y (over the centre of mass, so the thigh lies flat at full crouch)
const HD = 33;      // hip axis height above the torso bottom
const GB0 = 15;     // gimbal back / front plates start (|x|)
const GO0 = 53.5;   // gimbal outer plate inner face = hip-pitch stator face (|x|)
const GO1 = 56.5;   // gimbal outer plate outer face = hip-pitch output face (|x|); torso wall inner face is 57

function buildLeg(context is Context, id is Id, def is map, s is number, side is string)
{
    const hz = def.hipZ;
    const R = def.wheelR;
    const ky = HY + def.kneeY;
    const kz = def.kneeZ;
    const rx = s * RX;

    // ---- Hip-roll RS05: axis along y, rear face in a 0.5 mm pocket in the torso rear wall (y -67.5), output face y -20.5.
    //      Drops in from the top: its output shaft enters the mid bulkhead through a top-open U-slot. ----
    rs05y(context, id + "rollM", rx, -67.5, 1, hz, "RS05 hip roll " ~ side);

    // ---- Gimbal frame (one printed PA-CF part): back plate on the roll output, front plate with a D10 stub in a
    //      6800-2RS in the front bulkhead (bolt-on bearing cap), outer plate on the pitch stator ring.
    //      Open on the inner side, top and bottom: the pitch motor drops in from above and slides outward through
    //      the outer-plate bore. Roll and pitch axes intersect at (rx, HY, hz). Every edge is inside |x| <= 56.5,
    //      so the frame drops in past the torso side walls (inner face 57). ----
    mkBox(context, id + "gbB", s * GB0, s * GO1, -18, -13.5, hz - 23, hz + 23);               // back plate
    mkBox(context, id + "gbF", s * GB0, s * GO1, 33.5, 38, hz - 23, hz + 23);                 // front plate
    try silent { opFillet(context, id + "gbBf", { "entities" : qParallelEdges(qCreatedBy(id + "gbB", EntityType.EDGE), Y_DIRECTION), "radius" : 7 * millimeter }); }
    try silent { opFillet(context, id + "gbFf", { "entities" : qParallelEdges(qCreatedBy(id + "gbF", EntityType.EDGE), Y_DIRECTION), "radius" : 7 * millimeter }); }
    mkBox(context, id + "gbO", s * GO0, s * GO1, -18, 38, hz - 23, hz + 23);                 // outer plate
    cyl(context, id + "gbP", P(rx, -20.5, hz), P(rx, -18, hz), 17);                          // pad onto the roll output
    cyl(context, id + "gbS", P(rx, 38, hz), P(rx, 44, hz), 5);                               // D10 stub into the 6800
    uni(context, id + "gbU", qUnion([bq(id + "gbB"), bq(id + "gbF"), bq(id + "gbO"), bq(id + "gbP"), bq(id + "gbS")]));
    const gb = bq(id + "gbB");
    rotorIface(context, id + "gbRo", gb, "y", rx, hz, -21.5, -12.5, 0, 0);                  // roll output: 6 x M4 + 3 dowels
    csinks(context, id + "gbRk", gb, "y", rx, hz, -13.5, -1, 6, 24, 0, 4.1);                 // flush heads (grip 7: M4 x 10 DIN 7991)
    statorIface(context, id + "gbPo", gb, "x", HY, hz, s * (GO0 - 1), s * (GO1 + 1), 0, 0);   // pitch stator: 8 x M3
    tagPart(context, gb, "Hip gimbal " ~ side, DARK);

    // ---- Hip-pitch RS05 in the gimbal (rear face toward the torso centre, output face flush with the outer plate) ----
    rs05x(context, id + "pitchM", s * (GO0 - 44), s, HY, hz, "RS05 hip pitch " ~ side);

    // ---- Thigh plate (printed PA-CF) with a hub boss through the outer-plate bore onto the pitch output, and a flush
    //      hip-cap inlay (the shin folds over the hip). ----
    hullYZ(context, id + "th", s * 61, s * 70, HY, hz, 26, ky, kz, 27);
    cyl(context, id + "thHb", P(s * GO1, HY, hz), P(s * 61, HY, hz), 15);                    // hub boss to the pitch output
    uni(context, id + "thHbU", qUnion([bq(id + "th" + "q"), bq(id + "thHb")]));
    const th = bq(id + "th" + "q");
    rotorIface(context, id + "thH", th, "x", HY, hz, s * (GO1 - 1), s * 71, s * (GO1 + 9), s * 71);   // grip 9: M4 x 12
    hole(context, id + "thHi", th, "x", HY, hz, s * 68.5, s * 71, 44);                      // inlay pocket
    statorIface(context, id + "thK", th, "x", ky, kz, s * 60, s * 71, s * 70, -s);            // knee stator, countersunk outside
    const Lt = sqrt((ky - HY) * (ky - HY) + (kz - hz) * (kz - hz));
    mkBox(context, id + "thW", s * 60, s * 71, -9, 9, 34, Lt - 34);                         // lightening slot / harness run
    opTransform(context, id + "thWr", { "bodies" : bq(id + "thW"),
                "transform" : transform(P(0, HY, hz)) * rotationAround(line(P(0, 0, 0), X_DIRECTION), atan2(-(ky - HY), kz - hz)) });
    subtr(context, id + "thWC", th, bq(id + "thW"));
    tagPart(context, th, "Thigh " ~ side, WHITE);
    cyl(context, id + "hcap", P(s * 68.5, HY, hz), P(s * 70, HY, hz), 21.8);
    tagPart(context, bq(id + "hcap"), "Hip cap inlay " ~ side, ORANGE);

    // ---- Knee RS05 on the thigh's inner side ----
    rs05x(context, id + "kneeM", s * 17, s, ky, kz, "RS05 knee " ~ side);

    // ---- Shin (printed PA-CF) + hub through the thigh bore onto the knee output ----
    mkLink(context, id + "shin", s * 71, s * 79, ky, kz, HY, R, 17, 26);
    cyl(context, id + "shHub", P(s * 64, ky, kz), P(s * 71, ky, kz), 15);
    uni(context, id + "shU", qUnion([bq(id + "shin" + "web"), bq(id + "shHub")]));
    const shin = bq(id + "shin" + "web");
    rotorIface(context, id + "shK", shin, "x", ky, kz, s * 63, s * 80, s * 73, s * 80);      // grip 9: M4 x 12
    holes(context, id + "shKp", shin, "x", ky, kz, s * 75, s * 80, 3, 30, 2.2, 90);          // knee-cap pilots (M2.5 plastite)
    // wheel-motor rear face: 4 x M3 countersunk from the inner face (flush, the shin folds over the thigh)
    holes(context, id + "shW", shin, "x", HY, R, s * 70, s * 80, 4, 38.5, 3.4, 45);
    csinks(context, id + "shWk", shin, "x", HY, R, s * 71, s, 4, 38.5, 45, 3.2);
    hole(context, id + "shWc", shin, "x", HY, R, s * 70, s * 80, 12);
    // harness channel on the outer face, wheel-motor pigtail to the knee, under the stripe cover
    linkBox(context, id + "shCh", s * 76, s * 80, HY, R, ky, kz, 6, 18, 86);
    subtr(context, id + "shChC", shin, bq(id + "shCh"));
    tagPart(context, shin, "Shin " ~ side, WHITE);
    linkBox(context, id + "sstr", s * 79, s * 80.5, HY, R, ky, kz, 10, 26, 72);
    tagPart(context, bq(id + "sstr"), "Shin stripe cover " ~ side, ORANGE);
    cyl(context, id + "kcap", P(s * 79, ky, kz), P(s * 82, ky, kz), 17);
    holes(context, id + "kcapH", bq(id + "kcap"), "x", ky, kz, s * 78, s * 83, 3, 30, 2.9, 90);
    tagPart(context, bq(id + "kcap"), "Knee cap " ~ side, ORANGE);

    // ---- Wheel RS05 on the shin, output outward into the hub ----
    rs05x(context, id + "wheelM", s * 79, s, HY, R, "RS05 wheel " ~ side);

    const wc = s * 105.5;
    const TW = 24.5;                                                                            // tyre half width
    const xo = s * 130;                                                                         // tyre outer face
    cyl(context, id + "hubd", P(s * 126, HY, R), P(s * 132, HY, R), 34);                       // hub disc on the output
    cyl(context, id + "hubl", P(s * 130, HY, R), P(s * 132, HY, R), 38);                       // outer lip: retains the tyre
    cyl(context, id + "rim", P(wc - s * TW, HY, R), P(xo, HY, R), 35);
    cyl(context, id + "rimi", P(wc - s * (TW + 1), HY, R), P(xo - s * 4, HY, R), 31);
    subtr(context, id + "rimc", bq(id + "rim"), bq(id + "rimi"));
    uni(context, id + "hubu", qUnion([bq(id + "rim"), bq(id + "hubd"), bq(id + "hubl")]));
    const hub = bq(id + "rim");
    holes(context, id + "hubDec", hub, "x", HY, R, s * 125, s * 133, 6, 46, 10, 30);           // lightening holes
    rotorIface(context, id + "hubM4", hub, "x", HY, R, s * 125, s * 133, s * 129, s * 133);   // grip 3: M4 x 6
    tagPart(context, hub, "Wheel hub " ~ side, GREY);

    // crowned TPU tyre: 14 mm shoulders, so the contact patch stays centred when hip roll cambers the wheel
    cyl(context, id + "tyre", P(wc - s * TW, HY, R), P(wc + s * TW, HY, R), R);
    try silent { opFillet(context, id + "tyreF", { "entities" : qCreatedBy(id + "tyre", EntityType.EDGE), "radius" : 14 * millimeter }); }
    cyl(context, id + "tyri", P(wc - s * (TW + 1), HY, R), P(wc + s * (TW + 1), HY, R), 35);
    subtr(context, id + "tyrc", bq(id + "tyre"), bq(id + "tyri"));
    var gr = [];
    for (var k = 0; k < 20; k += 1)
    {
        const gid = id + ("g" ~ k);
        mkBox(context, gid, wc - 12, wc + 12, HY - 2.5, HY + 2.5, 2 * R - 2, 2 * R + 1);
        opTransform(context, gid + "r", { "bodies" : bq(gid), "transform" : rotationAround(line(P(0, HY, R), X_DIRECTION), k * 18 * degree) });
        gr = append(gr, bq(gid));
    }
    subtr(context, id + "tread", bq(id + "tyre"), qUnion(gr));
    tagPart(context, bq(id + "tyre"), "Tyre " ~ side ~ " (TPU, crowned)", TYRE);
}

// Round the edge parallel to x at (y = ye, z = ze) of target with radius r; (sy, sz) = outward signs.
function roundEdgeX(context is Context, id is Id, target is Query, ye, ze, r, sy, sz)
{
    mkBox(context, id + "b", -200, 200, ye - sy * r, ye + sy * 1, ze + sz * 1, ze - sz * r);
    cyl(context, id + "c", P(-201, ye - sy * r, ze - sz * r), P(201, ye - sy * r, ze - sz * r), r);
    subtr(context, id + "bc", bq(id + "b"), bq(id + "c"));
    subtr(context, id + "cut", target, bq(id + "b"));
}

// NUCLEO-144 (MB1364) mounting holes, board frame: bx across the 70 mm width, by along the 133.34 mm length,
// origin at the corner on the CN14 / user-USB end (UM2407 Fig. 6). The ST-LINK USB is at the far end (by = 133.34).
const NUCLEO_HOLES = [[12.14, 29.21], [57.86, 29.21], [43.89, 53.34], [10.87, 105.39], [59.13, 104.14]];

annotation { "Feature Type Name" : "BD Droid RS05" }
export const bdDroidRS05 = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Hip axis height (mm)" }
        isReal(definition.hipZ, { (unitless) : [33, 210, 255] } as RealBoundSpec);
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
        // Box (+-60 x -70..+88), front-bottom edge rounded. Lower bay: the hip-roll motors (rear wall to mid
        // bulkhead) and the gimbals (mid to front bulkhead), hip sockets in the lower side walls.
        // Upper bay: Makita XGT 40V battery, sliding in from the back onto an XGT rail dock under the lid (like a tool),
        // release button facing out of the rear opening; power/CAN board and fuse stand beside the battery.
        // NUCLEO-H743ZI stands on the front wall, USB up through the lid.
        const tz0 = hz - HD;           // torso bottom
        const fl = tz0 + WALL;         // floor top
        const zT = hz + 38;            // battery shelf underside (clears the gimbal sweep)
        const zB = zT + 4;             // battery underside
        const BH = 65;                 // XGT pack height including the rails (118 x 77 x 65)
        const zD = zB + BH;            // rail dock underside
        const tz1 = zD + 22;           // torso top (lid underside) = hz + 129 (clears the 133 mm Nucleo)
        const yb = TYB + WALL;         // -67 (rear wall inner)
        const yf = TYF - WALL;        //  85 (front wall inner)
        const xn = TXN - WALL;         // 57
        const BY0 = TYB;               // battery rear face, flush with the rear wall
        const BY1 = TYB + 118;         // battery front face

        mkBox(context, id + "torso", -TXN, TXN, TYB, TYF, tz0, tz1);
        try silent { opFillet(context, id + "torsoF", { "entities" : qParallelEdges(qCreatedBy(id + "torso", EntityType.EDGE), Z_DIRECTION), "radius" : 8 * millimeter }); }
        const torso = bq(id + "torso");
        roundEdgeX(context, id + "rndO", torso, TYF, tz0, 30, 1, -1);                      // front-bottom edge
        mkBox(context, id + "torsoI", -xn, xn, yb, yf, fl, tz1 + 1);
        try silent { opFillet(context, id + "torsoIF", { "entities" : qParallelEdges(qCreatedBy(id + "torsoI", EntityType.EDGE), Z_DIRECTION), "radius" : 2 * millimeter }); }
        roundEdgeX(context, id + "rndI", bq(id + "torsoI"), yf, fl, 27, 1, -1);
        subtr(context, id + "torsoC", torso, bq(id + "torsoI"));
        // hip sockets: notch the lower side walls and floor edge where the thigh and gimbal swing (before the bulkheads)
        // Forward of the sockets the lower side walls step in to x +-48 (front recess): hip roll swings the thigh's
        // lower edge (standing, roll in) and its upper edge (crouched, roll out) inboard of the main wall line.
        for (var s = -1; s <= 1; s += 2)
        {
            const sd = s > 0 ? "R" : "L";
            mkBox(context, id + ("sock" ~ sd), s * 48, s * (TXN + 1), -19, TYF + 1, tz0 - 1, zT);
            subtr(context, id + ("sockC" ~ sd), torso, bq(id + ("sock" ~ sd)));
        }

        // Internal features fused to the shell: lid bosses, shelf ledges, bulkheads, Nucleo standoffs, carrier posts
        var feats = [torso];
        // lid: 3 screws. The rear one sits between the two roll motors, so nothing overhangs the hip drop-in paths
        const lidPts = [[0, -62], [51.5, 79.5], [-51.5, 79.5]];
        for (var k = 0; k < size(lidPts); k += 1)
        {
            // lid boss with a 45 deg cone under it, so the shell prints upright without supports
            const lr = k == 0 ? 5 : 6;
            cyl(context, id + ("lb" ~ k), P(lidPts[k][0], lidPts[k][1], tz1 - 15), P(lidPts[k][0], lidPts[k][1], tz1), lr);
            fCone(context, id + ("lbc" ~ k), { "topCenter" : P(lidPts[k][0], lidPts[k][1], tz1 - 15), "bottomCenter" : P(lidPts[k][0], lidPts[k][1], tz1 - 15 - lr),
                        "topRadius" : lr * millimeter, "bottomRadius" : 0.5 * millimeter });
            feats = concatenateArrays([feats, [bq(id + ("lb" ~ k)), bq(id + ("lbc" ~ k))]]);
        }
        // Battery-shelf supports: full-height columns (no overhangs), clear of the hip modules' drop-in paths.
        // Front corners beside the front bulkhead; rear-centre on the rear wall and mid-centre on the mid bulkhead,
        // between the two roll motors.
        // recess walls (x 45..48) closing the lower front corners, front-bottom edge rounded like the shell
        for (var s = -1; s <= 1; s += 2)
        {
            const sd = s > 0 ? "R" : "L";
            mkBox(context, id + ("rcW" ~ sd), s * 45, s * 48, 47, TYF, tz0, zT);
            roundEdgeX(context, id + ("rcWr" ~ sd), bq(id + ("rcW" ~ sd)), TYF, tz0, 30, 1, -1);
            feats = append(feats, bq(id + ("rcW" ~ sd)));
        }
        const shelfPts = [[41.5, 50], [-41.5, 50], [0, -63], [0, -26]];
        mkBox(context, id + "tl0", 38, 45.5, 47, 56, fl - 0.5, zT);
        mkBox(context, id + "tl1", -38, -45.5, 47, 56, fl - 0.5, zT);
        mkBox(context, id + "tl2", -6, 6, yb - 0.5, -59, fl - 0.5, zT);
        mkBox(context, id + "tl3", -6, 6, -31, -23, fl - 0.5, zT);
        feats = concatenateArrays([feats, [bq(id + "tl0"), bq(id + "tl1"), bq(id + "tl2"), bq(id + "tl3")]]);
        // Bulkheads: mid (roll stator rings, y -23.5..-20.5) and front (gimbal 6800s), floor to battery shelf
        mkBox(context, id + "bhM", -xn - 0.5, xn + 0.5, -23.5, -20.5, fl - 0.5, zT);
        mkBox(context, id + "bhF", -47, 47, 39, 47, fl - 0.5, zT);
        feats = concatenateArrays([feats, [bq(id + "bhM"), bq(id + "bhF")]]);
        // Nucleo: vertical on the front wall, components facing back, ST-LINK USB end (by = 133.34) at the top.
        const nyS = yf - 12;           // board front face (y); 12 mm standoffs clear the morpho pins
        const nb0 = fl + 12;           // board bottom edge (z), above the rounded front-bottom edge
        var nPts = [];
        for (var k = 0; k < 5; k += 1)
        {
            nPts = append(nPts, [NUCLEO_HOLES[k][0] - 35, nb0 + NUCLEO_HOLES[k][1]]);
            cyl(context, id + ("ns" ~ k), P(nPts[k][0], yf + 0.5, nPts[k][1]), P(nPts[k][0], nyS, nPts[k][1]), 3.5);
            feats = append(feats, bq(id + ("ns" ~ k)));
        }
        // power / CAN board posts on the right side wall (board stands beside the battery, x 53..54.6)
        const pbPts = [[-56, zB + 6], [-30, zB + 6], [-56, zB + 26], [-30, zB + 26]];   // outboard of the roll-motor drop path (x > 55.6), behind the gimbal path
        for (var k = 0; k < 4; k += 1)
        {
            cyl(context, id + ("ps" ~ k), P(55.6, pbPts[k][0], pbPts[k][1]), P(xn + 0.5, pbPts[k][0], pbPts[k][1]), 3.2);
            feats = append(feats, bq(id + ("ps" ~ k)));
        }
        uni(context, id + "torsoU", qUnion(feats));

        for (var k = 0; k < size(lidPts); k += 1)
            hole(context, id + ("lbh" ~ k), torso, "z", lidPts[k][0], lidPts[k][1], tz1 - 10, tz1 + 1, 4.0);             // lid inserts
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("tlh" ~ k), torso, "z", shelfPts[k][0], shelfPts[k][1], zT - 7, zT + 1, 4.0);            // shelf inserts
        for (var k = 0; k < 5; k += 1)
            hole(context, id + ("nsh" ~ k), torso, "y", nPts[k][0], nPts[k][1], nyS - 1, yf - 2, 4.0);              // Nucleo inserts
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("psh" ~ k), torso, "x", pbPts[k][0], pbPts[k][1], 55, 59.4, 4.0);                  // board inserts (M3 x 4)
        // battery opening in the rear wall
        mkBox(context, id + "battOpen", -40, 40, TYB - 1, yb + 1, zB - 1, zD + 1);
        subtr(context, id + "battOpenC", torso, bq(id + "battOpen"));
        for (var s = -1; s <= 1; s += 2)
        {
            const sid = s > 0 ? "R" : "L";
            const rx = s * RX;
            // roll motor rear face: 0.5 mm locating pocket in the rear wall, open to the top so the motor slides
            // down into it; 4 x M3 through (+ D14 centre access)
            cyl(context, id + ("rwP" ~ sid), P(rx, -67.5, hz), P(rx, -66.5, hz), 23.6);
            mkBox(context, id + ("rwPU" ~ sid), rx - 23.6, rx + 23.6, -67.5, -66.5, hz, tz1 + 1);
            subtr(context, id + ("rwPC" ~ sid), torso, qUnion([bq(id + ("rwP" ~ sid)), bq(id + ("rwPU" ~ sid))]));
            holes(context, id + ("rwS" ~ sid), torso, "y", rx, hz, TYB - 1, yb + 1, 4, 38.5, 3.4, 45);
            hole(context, id + ("rwC" ~ sid), torso, "y", rx, hz, TYB - 1, yb + 1, 14);
            // mid bulkhead: D31 bore + top-open U-slot for the roll output shaft (drop-in), 0.6 mm relief for the
            // 35 mm housing step, and 6 of the 8 stator screws (the two top ones fall in the slot), countersunk in front
            cyl(context, id + ("bmB" ~ sid), P(rx, -24, hz), P(rx, -20, hz), 15.5);
            mkBox(context, id + ("bmU" ~ sid), rx - 15.5, rx + 15.5, -24, -20, hz, zT + 1);
            cyl(context, id + ("bmR" ~ sid), P(rx, -24, hz), P(rx, -22.9, hz), 17.65);
            mkBox(context, id + ("bmRU" ~ sid), rx - 17.65, rx + 17.65, -24, -22.9, hz, zT + 1);
            subtr(context, id + ("bmC" ~ sid), torso, qUnion([bq(id + ("bmB" ~ sid)), bq(id + ("bmU" ~ sid)), bq(id + ("bmR" ~ sid)), bq(id + ("bmRU" ~ sid))]));
            holesSkip(context, id + ("bmS" ~ sid), torso, "y", rx, hz, -24.5, -19.5, 8, 41.5, 3.4, 22.5, 90, 30);
            for (var k = 0; k < 8; k += 1)
            {
                const ang = 22.5 + 45 * k;
                if (ang > 60 && ang < 120)
                    continue;
                csinks(context, id + ("bmK" ~ sid ~ k), torso, "y", rx + 20.75 * cos(ang * degree), hz + 20.75 * sin(ang * degree), -20.5, -1, 1, 0, 0, 3.2);
            }
            // front bulkhead: split 6800 seat (pillow block). Notch above the axis so the gimbal + bearing drop in,
            // bolt-on bearing cap (2 x M3 into inserts) closes it.
            mkBox(context, id + ("bfN" ~ sid), rx - 17, rx + 17, 38, 47, hz, zT + 1);
            subtr(context, id + ("bfNC" ~ sid), torso, bq(id + ("bfN" ~ sid)));
            hole(context, id + ("bfB" ~ sid), torso, "y", rx, hz, 38, 44, 19.1);
            hole(context, id + ("bfT" ~ sid), torso, "y", rx, hz, 38, 48, 12);
            hole(context, id + ("bfI1" ~ sid), torso, "z", rx - 13.5, 43, hz - 9, hz + 1, 4.0);
            hole(context, id + ("bfI2" ~ sid), torso, "z", rx + 13.5, 43, hz - 9, hz + 1, 4.0);
            mkBox(context, id + ("bcap" ~ sid), rx - 17, rx + 17, 39, 47, hz, hz + 14);
            try silent { opFillet(context, id + ("bcapF" ~ sid), { "entities" : qParallelEdges(qCreatedBy(id + ("bcap" ~ sid), EntityType.EDGE), Y_DIRECTION), "radius" : 4 * millimeter }); }
            const bcq = bq(id + ("bcap" ~ sid));
            hole(context, id + ("bcB" ~ sid), bcq, "y", rx, hz, 38, 44, 19.1);
            hole(context, id + ("bcT" ~ sid), bcq, "y", rx, hz, 38, 48, 12);
            hole(context, id + ("bcH1" ~ sid), bcq, "z", rx - 13.5, 43, hz - 1, hz + 15, 3.4);
            hole(context, id + ("bcH2" ~ sid), bcq, "z", rx + 13.5, 43, hz - 1, hz + 15, 3.4);
            tagPart(context, bcq, "Gimbal bearing cap " ~ sid, DARK);
            // floor relief under the pitch-motor sweep (roll +-0.35 brings its rear-bottom edge within 1 mm of the floor)
            mkBox(context, id + ("frl" ~ sid), s * 8, s * 30, -14, 34, fl - 1.5, fl + 1);
            subtr(context, id + ("frlC" ~ sid), torso, bq(id + ("frl" ~ sid)));
            // leg-harness pass through the mid bulkhead
            mkBox(context, id + ("cs" ~ sid), s * 48, s * 56, -24, -19, hz + 26, hz + 34);
            subtr(context, id + ("csC" ~ sid), torso, bq(id + ("cs" ~ sid)));
            // gimbal 6800-2RS (10 x 19 x 5), reference
            cyl(context, id + ("b6800" ~ sid), P(rx, 39, hz), P(rx, 44, hz), 9.5);
            hole(context, id + ("b6800h" ~ sid), bq(id + ("b6800" ~ sid)), "y", rx, hz, 38, 45, 10);
            tagPart(context, bq(id + ("b6800" ~ sid)), "Bearing 6800-2RS hip gimbal " ~ sid, STEEL);
        }
        tagPart(context, torso, "Torso shell", WHITE);

        // Chest details on the front wall
        const cz = hz + 70;
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
        mkBox(context, id + "wireR", 44, 56, -62, -50, zT - 1, zB + 1);             // right leg harness up to the power board
        mkBox(context, id + "wireL", -44, -56, -62, -50, zT - 1, zB + 1);           // left leg harness
        subtr(context, id + "trayC", tray, qUnion([bq(id + "wireN"), bq(id + "wireR"), bq(id + "wireL")]));
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
        mkBox(context, id + "dockC", -45, 45, BY0 + 14, BY1 + 6, zD + 3, tz1);
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

        // ---- Power / CAN board (reference), standing beside the battery on the right: DC-DCs + CAN transceiver ----
        mkBox(context, id + "pcb", 54, 55.6, -60, 13, zB + 2, zB + 30);
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("pch" ~ k), bq(id + "pcb"), "x", pbPts[k][0], pbPts[k][1], 53, 57, 3.2);
        tagPart(context, bq(id + "pcb"), "Power-CAN board (ref)", PCB);
        mkBox(context, id + "dc12", 43.8, 54, -46.5, 4.3, zB + 3, zB + 28.4);
        tagPart(context, bq(id + "dc12"), "DC-DC 36V-12V 30W 2x1in, 18-75V in, e.g. Mean Well SKMW30F-12 (ref)", DARK);
        mkBox(context, id + "dc5", 45.5, 54, -59, -47.5, zB + 5, zB + 22.5);
        tagPart(context, bq(id + "dc5"), "DC-DC 36V-5V 0.5A SIP, 9-72V in, e.g. RECOM R-78HB5.0-0.5 (ref)", DARK);
        mkBox(context, id + "can", 51, 54, 5, 12, zB + 5, zB + 17);
        tagPart(context, bq(id + "can"), "CAN transceiver TJA1051T-3 + 120R (ref)", DARK);
        mkBox(context, id + "fuse", -54, -43, -40, -12, zB + 2, zB + 13);
        tagPart(context, bq(id + "fuse"), "Inline mini blade fuse holder 20 A (ref)", color(0.6, 0.1, 0.1));

        // ---- Lid ----
        mkBox(context, id + "lid", -TXN, TXN, TYB, TYF, tz1, tz1 + 4);
        try silent { opFillet(context, id + "lidF", { "entities" : qParallelEdges(qCreatedBy(id + "lid", EntityType.EDGE), Z_DIRECTION), "radius" : 8 * millimeter }); }
        const lid = bq(id + "lid");
        for (var k = 0; k < size(lidPts); k += 1)
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
