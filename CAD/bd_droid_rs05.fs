FeatureScript 3112;
import(path : "onshape/std/common.fs", version : "3112.0");

// BD-style wheel-legged droid, built around RobStride 05 actuators,
// a SuperHandy 48 V 2 Ah tool battery and a NUCLEO-H743ZI (Nucleo-144, MB1364).
// Frame: x = robot right, y = forward, z = up, ground at z = 0, axle midpoint at x = 0, y = 0.
// All dimensions in mm.
//
// RS05 interfaces (datasheet 1.1, first-angle projection):
//   output face : rotor flange 6 x M4 on PCD 24 (depth 3.5), pilot D19, boss D30 x 3
//                 stationary ring 8 x M3 on PCD 41.5 (depth 7)
//   rear face   : 4 x M3 on PCD 38.5 (depth 10)
// Fasteners: clearance M3 = 3.4, M4 = 4.5, M2.5 = 2.9; heat-set insert holes M3 = 4.0, pilot M2 = 2.0.

function P(x, y, z) { return vector(x, y, z) * millimeter; }

function cyl(context is Context, id is Id, a is Vector, b is Vector, r is number)
{
    fCylinder(context, id, { "bottomCenter" : a, "topCenter" : b, "radius" : r * millimeter });
}

function mkBox(context is Context, id is Id, x0, x1, y0, y1, z0, z1)
{
    fCuboid(context, id, { "corner1" : P(min(x0, x1), min(y0, y1), min(z0, z1)), "corner2" : P(max(x0, x1), max(y0, y1), max(z0, z1)) });
}

function bq(id is Id) { return qCreatedBy(id, EntityType.BODY); }

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

const WHITE = color(0.86, 0.83, 0.76);
const ORANGE = color(0.85, 0.38, 0.14);
const DARK = color(0.22, 0.22, 0.23);
const TYRE = color(0.08, 0.08, 0.08);
const GREY = color(0.55, 0.56, 0.58);

// Torso / layout constants
const TX = 60;      // torso half width (outer)
const TYB = -70;    // torso rear outer face
const TYF = 70;     // torso front outer face
const WALL = 3;
const TH = 197;     // torso height (outer, without lid)

function buildLeg(context is Context, id is Id, def is map, s is number, side is string)
{
    const hz = def.hipZ;
    const R = def.wheelR;
    const ky = def.kneeY;
    const kz = def.kneeZ;
    const rz = hz + 45;                       // hip-roll axis height
    const yw = TYB;                           // rear wall outer face (roll output face lies here)
    // Lateral stack (right side, mirrored for left), mm from centreline:
    // yoke arm 61-67 | hip pitch RS05 67-111 (+3 boss) | thigh body 114-129 + cover 129-132 | knee RS05 132-176 | cap 176-182
    // shin 104-112 | wheel RS05 112-156 (+3 boss) | hub disc 159-163 | tyre centred at 151

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

    // ---- Thigh body (printed, open pocket for the knee belt) + bolt-on cover ----
    const r1 = 32;
    const r2 = 24;
    const L = sqrt(ky * ky + (kz - hz) * (kz - hz));
    const ny = -(kz - hz) / L;               // in-plane normal to the thigh axis
    const nz = ky / L;
    const beltDir = atan2(kz - hz, ky) / degree;   // direction hip -> knee in (y,z) as degrees
    mkLink(context, id + "thigh", s * 114, s * 129, 0, hz, ky, kz, r1, r2);
    mkLink(context, id + "thighCav", s * 117, s * 130, 0, hz, ky, kz, r1 - 3, r2 - 3);
    const thigh = bq(id + "thigh" + "web");
    subtr(context, id + "thighCut", thigh, bq(id + "thighCav" + "web"));
    // cover screw bosses (2 at hip, 2 at knee), clear of the belt corridor
    const bossPts = [[26 * ny, hz + 26 * nz], [-26 * ny, hz - 26 * nz], [ky + 17 * ny, kz + 17 * nz], [ky - 17 * ny, kz - 17 * nz]];
    var bossQ = [thigh];
    for (var k = 0; k < 4; k += 1)
    {
        cyl(context, id + ("tb" ~ k), P(s * 116, bossPts[k][0], bossPts[k][1]), P(s * 129, bossPts[k][0], bossPts[k][1]), 3.5);
        bossQ = append(bossQ, bq(id + ("tb" ~ k)));
    }
    uni(context, id + "thighBoss", qUnion(bossQ));
    for (var k = 0; k < 4; k += 1)
    {
        hole(context, id + ("tbh" ~ k), thigh, "x", bossPts[k][0], bossPts[k][1], s * 119, s * 130, 4.0);   // M3 heat-set inserts
    }
    holes(context, id + "thHip", thigh, "x", 0, hz, s * 113, s * 118, 6, 24, 4.5, 0);          // 6 x M4 to hip-pitch output
    hole(context, id + "thHipC", thigh, "x", 0, hz, s * 113, s * 118, 10);                      // centre clearance (M4s sit on PCD 24)
    hole(context, id + "thKnee", thigh, "x", ky, kz, s * 113, s * 118, 16.05);                  // 688ZZ bearing seat
    tagPart(context, thigh, "Thigh " ~ side, WHITE);

    mkLink(context, id + "cover", s * 129, s * 132, 0, hz, ky, kz, r1, r2);
    const cover = bq(id + "cover" + "web");
    hole(context, id + "cvMot", cover, "x", 0, hz, s * 128, s * 133, 31);                      // knee-motor boss
    holesSkip(context, id + "cvRing", cover, "x", 0, hz, s * 128, s * 133, 8, 41.5, 3.4, beltDir + 112.5, beltDir, 30);  // M3 to knee-motor ring (6 of 8, clear of belt)
    hole(context, id + "cvKnee", cover, "x", ky, kz, s * 128, s * 133, 16.05);                  // 688ZZ bearing seat
    for (var k = 0; k < 4; k += 1)
    {
        hole(context, id + ("cvb" ~ k), cover, "x", bossPts[k][0], bossPts[k][1], s * 128, s * 133, 3.4);
    }
    holes(context, id + "cvCap", cover, "x", ky, kz, s * 128, s * 133, 3, 28, 2.0, 90);         // knee-cap pilots
    tagPart(context, cover, "Thigh cover " ~ side, WHITE);

    linkBox(context, id + "tstripe", s * 132, s * 133.5, 0, hz, ky, kz, 12, 30, L - 22);
    tagPart(context, bq(id + "tstripe"), "Thigh stripe " ~ side, ORANGE);

    // Belt pulleys (reference, 1:1 GT2)
    cyl(context, id + "pulH", P(s * 121, 0, hz), P(s * 129, 0, hz), 8);
    tagPart(context, bq(id + "pulH"), "Knee drive pulley " ~ side ~ " (ref)", GREY);
    cyl(context, id + "pulK", P(s * 121, ky, kz), P(s * 129, ky, kz), 8);
    tagPart(context, bq(id + "pulK"), "Knee driven pulley " ~ side ~ " (ref)", GREY);

    // ---- Knee RS05 on the thigh cover at the hip axis, output inward to the drive pulley ----
    rs05x(context, id + "kneeM", s * 176, -s, 0, hz, "RS05 knee " ~ side);
    cyl(context, id + "cap", P(s * 176, 0, hz), P(s * 182, 0, hz), 26);
    cyl(context, id + "capc", P(s * 180, 0, hz), P(s * 183, 0, hz), 12);
    subtr(context, id + "capcut", bq(id + "cap"), bq(id + "capc"));
    holes(context, id + "capH", bq(id + "cap"), "x", 0, hz, s * 175, s * 183, 4, 38.5, 3.4, 45);   // 4 x M3 to knee-motor rear face
    tagPart(context, bq(id + "cap"), "Shoulder cap " ~ side, ORANGE);

    cyl(context, id + "kcap", P(s * 132, ky, kz), P(s * 138, ky, kz), 18);
    cyl(context, id + "kcap2", P(s * 138, ky, kz), P(s * 141, ky, kz), 9);
    uni(context, id + "kcu", qUnion([bq(id + "kcap"), bq(id + "kcap2")]));
    holes(context, id + "kcapH", bq(id + "kcap"), "x", ky, kz, s * 131, s * 139, 3, 28, 2.9, 90);
    tagPart(context, bq(id + "kcap"), "Knee cap " ~ side, ORANGE);

    // ---- Shin (printed): knee shaft -> wheel motor ----
    mkLink(context, id + "shin", s * 104, s * 112, ky, kz, 0, R, 22, 28);
    const shin = bq(id + "shin" + "web");
    hole(context, id + "shK", shin, "x", ky, kz, s * 103, s * 113, 8.1);                       // 8 mm knee shaft
    holes(context, id + "shKc", shin, "x", ky, kz, s * 103, s * 113, 3, 16, 3.4, 30);           // M3 to pulley hub flange
    holes(context, id + "shW", shin, "x", 0, R, s * 103, s * 113, 4, 38.5, 3.4, 45);            // 4 x M3 to wheel-motor rear face
    hole(context, id + "shWc", shin, "x", 0, R, s * 103, s * 113, 12);
    tagPart(context, shin, "Shin " ~ side, WHITE);
    linkBox(context, id + "sstr", s * 112, s * 113, 0, R, ky, kz, 10, 30, 70);
    tagPart(context, bq(id + "sstr"), "Shin stripe " ~ side, ORANGE);

    // ---- Wheel RS05 on the shin, output outward into the hub ----
    rs05x(context, id + "wheelM", s * 112, s, 0, R, "RS05 wheel " ~ side);

    const wc = s * 151;
    cyl(context, id + "hubd", P(s * 159, 0, R), P(s * 163, 0, R), 34);
    cyl(context, id + "rim", P(wc - s * 18, 0, R), P(wc + s * 18, 0, R), 35);
    cyl(context, id + "rimi", P(wc - s * 19, 0, R), P(wc + s * 19, 0, R), 31);
    subtr(context, id + "rimc", bq(id + "rim"), bq(id + "rimi"));
    uni(context, id + "hubu", qUnion([bq(id + "rim"), bq(id + "hubd")]));
    const hub = bq(id + "rim");
    holes(context, id + "hubDec", hub, "x", 0, R, s * 158, s * 164, 6, 42, 12, 30);            // lightening holes
    holes(context, id + "hubM4", hub, "x", 0, R, s * 158, s * 164, 6, 24, 4.5, 0);             // 6 x M4 to wheel-motor output
    hole(context, id + "hubC", hub, "x", 0, R, s * 158, s * 164, 10);
    holes(context, id + "hubCapP", hub, "x", 0, R, s * 158, s * 164, 3, 18, 2.0, 90);          // hub-cap pilots
    tagPart(context, hub, "Wheel hub " ~ side, GREY);

    cyl(context, id + "hubcap", P(s * 163, 0, R), P(s * 167, 0, R), 13);
    holes(context, id + "hubcapH", bq(id + "hubcap"), "x", 0, R, s * 162, s * 168, 3, 18, 2.5, 90);
    tagPart(context, bq(id + "hubcap"), "Hub cap " ~ side, ORANGE);

    cyl(context, id + "tyre", P(wc - s * 18, 0, R), P(wc + s * 18, 0, R), R);
    cyl(context, id + "tyri", P(wc - s * 19, 0, R), P(wc + s * 19, 0, R), 35);
    subtr(context, id + "tyrc", bq(id + "tyre"), bq(id + "tyri"));
    var gr = [];
    for (var k = 0; k < 20; k += 1)
    {
        const gid = id + ("g" ~ k);
        mkBox(context, gid, wc - 14, wc + 14, -2.5, 2.5, 2 * R - 2.5, 2 * R + 1);
        opTransform(context, gid + "r", { "bodies" : bq(gid), "transform" : rotationAround(line(P(0, 0, R), X_DIRECTION), k * 18 * degree) });
        gr = append(gr, bq(gid));
    }
    subtr(context, id + "tread", bq(id + "tyre"), qUnion(gr));
    tagPart(context, bq(id + "tyre"), "Tyre " ~ side ~ " (TPU)", TYRE);
}

// NUCLEO-144 (MB1364) mounting holes, board frame: bx across the 70 mm width, by along the 133.34 mm length,
// origin at the corner on the CN14 / user-USB end (UM2407 Fig. 6).
const NUCLEO_HOLES = [[12.14, 29.21], [57.86, 29.21], [43.89, 53.34], [10.87, 105.39], [59.13, 104.14]];

annotation { "Feature Type Name" : "BD Droid RS05" }
export const bdDroidRS05 = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Hip axis height (mm)" }
        isReal(definition.hipZ, { (unitless) : [100, 210, 400] } as RealBoundSpec);
        annotation { "Name" : "Wheel radius (mm)" }
        isReal(definition.wheelR, { (unitless) : [20, 50, 100] } as RealBoundSpec);
    }
    {
        var def = definition;
        const hz = def.hipZ;
        const R = def.wheelR;
        const Lk = 105; // thigh = shin = 105 mm (matches CAD/robot.xml)
        const half = (hz - R) / 2;
        def.kneeZ = R + half;
        def.kneeY = sqrt(max(Lk * Lk - half * half, 0));

        buildLeg(context, id + "R", def, 1, "R");
        buildLeg(context, id + "L", def, -1, "L");

        // =================== Torso ===================
        const tz0 = hz + 18;
        const tz1 = tz0 + TH;
        const rz = hz + 45;
        const xi = TX - WALL;          // 57
        const yb = TYB + WALL;         // -67 (rear wall inner)
        const yf = TYF - WALL;         //  67 (front wall inner)
        mkBox(context, id + "torso", -TX, TX, TYB, TYF, tz0, tz1);
        try silent { opFillet(context, id + "torsoF", { "entities" : qParallelEdges(qCreatedBy(id + "torso", EntityType.EDGE), Z_DIRECTION), "radius" : 14 * millimeter }); }
        mkBox(context, id + "torsoI", -xi, xi, yb, yf, tz0 + WALL, tz1 + 1);
        try silent { opFillet(context, id + "torsoIF", { "entities" : qParallelEdges(qCreatedBy(id + "torsoI", EntityType.EDGE), Z_DIRECTION), "radius" : 11 * millimeter }); }
        subtr(context, id + "torsoC", bq(id + "torso"), bq(id + "torsoI"));
        const torso = bq(id + "torso");

        // Internal features fused to the shell: lid bosses, door insert blocks, Nucleo standoffs
        var feats = [torso];
        const lidPts = [[51.5, 61.5], [-51.5, 61.5], [51.5, -61.5], [-51.5, -61.5]];
        for (var k = 0; k < 4; k += 1)
        {
            cyl(context, id + ("lb" ~ k), P(lidPts[k][0], lidPts[k][1], tz1 - 15), P(lidPts[k][0], lidPts[k][1], tz1), 6);
            feats = append(feats, bq(id + ("lb" ~ k)));
        }
        const doorZ = [tz0 + 45, tz0 + 110, tz0 + 175];
        for (var k = 0; k < 3; k += 1)
        {
            mkBox(context, id + ("dbR" ~ k), 52.5, xi + 0.5, yf - 3, yf + 0.5, doorZ[k] - 5, doorZ[k] + 5);
            mkBox(context, id + ("dbL" ~ k), -52.5, -xi - 0.5, yf - 3, yf + 0.5, doorZ[k] - 5, doorZ[k] + 5);
            feats = append(feats, bq(id + ("dbR" ~ k)));
            feats = append(feats, bq(id + ("dbL" ~ k)));
        }
        // Nucleo: vertical on the rear wall, components facing forward, ST-LINK USB end at the top.
        const nb0 = tz0 + 52;           // board bottom edge (z)
        const nyS = yb + 12;            // board back face (y), 12 mm standoffs clear the morpho pins
        var nPts = [];
        for (var k = 0; k < 5; k += 1)
        {
            nPts = append(nPts, [NUCLEO_HOLES[k][0] - 35, nb0 + 133.34 - NUCLEO_HOLES[k][1]]);
            cyl(context, id + ("ns" ~ k), P(nPts[k][0], yb - 0.5, nPts[k][1]), P(nPts[k][0], nyS, nPts[k][1]), 3.5);
            feats = append(feats, bq(id + ("ns" ~ k)));
        }
        uni(context, id + "torsoU", qUnion(feats));

        // Holes in the torso shell
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("lbh" ~ k), torso, "z", lidPts[k][0], lidPts[k][1], tz1 - 10, tz1 + 1, 4.0);          // lid inserts
        for (var k = 0; k < 3; k += 1)
        {
            hole(context, id + ("dhR" ~ k), torso, "y", 55, doorZ[k], yf - 4, TYF + 1, 4.0);                         // door inserts
            hole(context, id + ("dhL" ~ k), torso, "y", -55, doorZ[k], yf - 4, TYF + 1, 4.0);
        }
        for (var k = 0; k < 5; k += 1)
            hole(context, id + ("nsh" ~ k), torso, "y", nPts[k][0], nPts[k][1], yb + 2, nyS + 1, 4.0);              // Nucleo inserts
        // battery door opening in the front wall
        mkBox(context, id + "doorOpen", -52, 52, yf - 1, TYF + 1, tz0 + WALL, tz0 + 181);
        subtr(context, id + "doorOpenC", torso, bq(id + "doorOpen"));
        // roll-motor bores + 8 x M3 rings in the rear wall
        for (var s = -1; s <= 1; s += 2)
        {
            const sid = s > 0 ? "R" : "L";
            hole(context, id + ("rb" ~ sid), torso, "y", s * 26, rz, TYB - 1, yb + 1, 36);
            holes(context, id + ("rr" ~ sid), torso, "y", s * 26, rz, TYB - 1, yb + 1, 8, 41.5, 3.4, 0);
            mkBox(context, id + ("cs" ~ sid), s * 30, s * 50, TYB - 1, yb + 1, tz0 + 54, tz0 + 64);                   // leg cable slots
            subtr(context, id + ("csC" ~ sid), torso, bq(id + ("cs" ~ sid)));
        }
        // battery cradle screws through the floor
        const cradlePts = [[45, -10], [-45, -10], [45, 55], [-45, 55]];
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("fh" ~ k), torso, "z", cradlePts[k][0], cradlePts[k][1], tz0 - 1, tz0 + WALL + 1, 3.4);
        tagPart(context, torso, "Torso shell", WHITE);

        // ---- Battery door (front) with chest details ----
        mkBox(context, id + "door", -58, 58, TYF, TYF + 3, tz0 + 2, tz0 + 196);
        try silent { opFillet(context, id + "doorF", { "entities" : qParallelEdges(qCreatedBy(id + "door", EntityType.EDGE), Y_DIRECTION), "radius" : 8 * millimeter }); }
        for (var k = 0; k < 3; k += 1)
        {
            hole(context, id + ("doR" ~ k), bq(id + "door"), "y", 55, doorZ[k], TYF - 1, TYF + 4, 3.4);
            hole(context, id + ("doL" ~ k), bq(id + "door"), "y", -55, doorZ[k], TYF - 1, TYF + 4, 3.4);
        }
        const cz = tz0 + 95;
        holes(context, id + "doChest", bq(id + "door"), "y", 0, cz, TYF - 1, TYF + 4, 4, 2 * sqrt(20 * 20 + 21 * 21), 2.0, 46.4);  // chest-panel pilots
        tagPart(context, bq(id + "door"), "Battery door", WHITE);

        mkBox(context, id + "chest", -26, 26, TYF + 3, TYF + 7, cz - 27, cz + 27);
        for (var k = 0; k < 4; k += 1)
            mkBox(context, id + ("slot" ~ k), -16, 16, TYF + 5, TYF + 8, cz - 18 + k * 8, cz - 14 + k * 8);
        subtr(context, id + "chestC", bq(id + "chest"), qUnion([bq(id + "slot0"), bq(id + "slot1"), bq(id + "slot2"), bq(id + "slot3")]));
        holes(context, id + "chestH", bq(id + "chest"), "y", 0, cz, TYF + 2, TYF + 8, 4, 2 * sqrt(20 * 20 + 21 * 21), 2.5, 46.4);
        tagPart(context, bq(id + "chest"), "Chest panel", DARK);
        mkBox(context, id + "cled", -6, 6, TYF + 7, TYF + 9, cz + 18, cz + 24);
        tagPart(context, bq(id + "cled"), "Chest LED", ORANGE);
        mkBox(context, id + "stripe", -12, 12, TYF + 3, TYF + 5, cz + 35, tz0 + 194);
        tagPart(context, bq(id + "stripe"), "Chest stripe", ORANGE);

        // ---- Battery cradle (printed) ----
        const cb = tz0 + WALL;
        mkBox(context, id + "cradle", -55.5, 55.5, -21, 64, cb, cb + 3);
        mkBox(context, id + "crR", 52.5, 55.5, -21, 64, cb, cb + 28);
        mkBox(context, id + "crL", -52.5, -55.5, -21, 64, cb, cb + 28);
        mkBox(context, id + "crB", -55.5, 55.5, -21, -18, cb, cb + 28);
        uni(context, id + "crU", qUnion([bq(id + "cradle"), bq(id + "crR"), bq(id + "crL"), bq(id + "crB")]));
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("crh" ~ k), bq(id + "cradle"), "z", cradlePts[k][0], cradlePts[k][1], cb - 1, cb + 4, 3.4);
        mkBox(context, id + "strapR", 51, 57, 10, 35, cb + 10, cb + 14);     // hook-and-loop strap slots
        mkBox(context, id + "strapL", -51, -57, 10, 35, cb + 10, cb + 14);
        subtr(context, id + "strapC", bq(id + "cradle"), qUnion([bq(id + "strapR"), bq(id + "strapL")]));
        tagPart(context, bq(id + "cradle"), "Battery cradle", DARK);

        // ---- SuperHandy 48 V 2 Ah battery (reference envelope 103 x 81 x 166, ~0.9 kg) ----
        mkBox(context, id + "batt", -51.5, 51.5, -17, 64, cb + 3, cb + 3 + 166);
        tagPart(context, bq(id + "batt"), "SuperHandy 48V 2Ah battery (ref)", color(0.15, 0.17, 0.2));
        mkBox(context, id + "term", -30, 30, -10, 55, cb + 169, cb + 189);
        tagPart(context, bq(id + "term"), "Battery terminal adapter (ref, 20 mm reserved)", color(0.35, 0.3, 0.25));

        // ---- NUCLEO-H743ZI (reference) ----
        mkBox(context, id + "nucleo", -35, 35, nyS, nyS + 1.6, nb0, nb0 + 133.34);
        for (var k = 0; k < 5; k += 1)
            hole(context, id + ("nh" ~ k), bq(id + "nucleo"), "y", nPts[k][0], nPts[k][1], nyS - 1, nyS + 3, 3.2);
        tagPart(context, bq(id + "nucleo"), "NUCLEO-H743ZI (ref)", color(0.1, 0.45, 0.2));
        mkBox(context, id + "nucleoParts", -33, 33, nyS + 1.6, nyS + 18, nb0 + 2, nb0 + 131);
        tagPart(context, bq(id + "nucleoParts"), "NUCLEO component envelope (ref)", color(0.1, 0.35, 0.15));

        // ---- Lid ----
        mkBox(context, id + "lid", -TX, TX, TYB, TYF, tz1, tz1 + 4);
        try silent { opFillet(context, id + "lidF", { "entities" : qParallelEdges(qCreatedBy(id + "lid", EntityType.EDGE), Z_DIRECTION), "radius" : 14 * millimeter }); }
        const lid = bq(id + "lid");
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("lh" ~ k), lid, "z", lidPts[k][0], lidPts[k][1], tz1 - 1, tz1 + 5, 3.4);
        const collarPts = [[22, -27], [-22, -27], [22, 21], [-22, 21]];
        for (var k = 0; k < 4; k += 1)
            hole(context, id + ("lnh" ~ k), lid, "z", collarPts[k][0], collarPts[k][1], tz1 - 1, tz1 + 5, 4.0);   // neck-collar inserts
        hole(context, id + "lcab", lid, "z", 0, 6, tz1 - 1, tz1 + 5, 12);                                          // Dynamixel bus / head wiring
        mkBox(context, id + "lusb", 3, 23, nyS - 4, nyS + 16, tz1 - 1, tz1 + 5);                                   // ST-LINK USB access
        subtr(context, id + "lusbC", lid, bq(id + "lusb"));
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
        // bosses: neck inserts in the floor, rear-cover inserts in the corners
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
    }, { hipZ : 210, wheelR : 50 });
