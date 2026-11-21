#!/usr/bin/env python3
"""
QuadBot-4 leg v2 ("dog leg") -- thigh / knee / shin silhouette, horn socket that
is screwed to the horn, and a TPU paw boot on an ankle bar.

    python dog_leg_v2.py            # writes ../leg_v2/*.stl and prints the checks

Same servo, horn, body and firmware as QuadBot-4 v1: the hip axis, the hip-to-ground
distance with the leg vertical (66 mm) and the "contact point straight under the hip"
rule the gait relies on are all unchanged.

Print frame (as v1): X fore/aft, Y along the leg (hip at 0, foot negative),
Z = 0 is the OUTER face (on the bed), +Z points toward the robot.
Part A has the knee toward -X; part B is its mirror image.
  Left legs (FL, RL):  X+ = forward  -> A = knee back, B = knee forward
  Right legs (FR, RR): X+ = backward -> B = knee back, A = knee forward
"""
import os
import sys

import numpy as np
import trimesh
from manifold3d import CrossSection, JoinType, Manifold, OpType

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qb4_ref as q  # noqa: E402

# ---------------------------------------------------------------- parameters
LEG_T = 4.0            # plate thickness
LEG_GAP = 0.5          # plate inner face to servo gearbox boss (v1: 0.6, plus 0.4 loose horn)
Y_LEG_IN = q.Y_BOSS_TOP + LEG_GAP
Y_LEG_OUT = Y_LEG_IN + LEG_T

HIP_R = 14.0          # hub disc: carries the screw collar without overhang
KNEE = (-16.0, -31.0)  # knee centre (part A: knee toward -X)
KNEE_R = 6.5
ANKLE = (0.0, -57.0)   # ankle bar centre, straight under the hip
ANKLE_R = 6.0          # plate stays >= 3 mm above the ground (contact at -66)
CONTACT = 66.0         # hip axis to ground, leg vertical (v1 design value)

# horn socket = the horn slot in a printed part known to fit this horn, which
# fits the author's single-arm horn. Measured from that STL (the slot already includes its fit):
#   hub dia 7.5; arm 7.30 wide at the hub centre, tapering 0.236 mm per mm to 4.06 at
#   13.75 mm, round tip r 2.03 (tip end 15.75 mm from the hub centre); 4.0 mm deep there.
PALM_HUB_D = 7.5
PALM_ARM_W0, PALM_ARM_W1, PALM_ARM_L = 7.30, 4.06, 13.75
HORN_CLEAR = 0.10      # extra per side: the palm slot was printed on its side, this one lies
                       # flat on the bed (flat holes print a little small). Set 0.0 if loose.
POCKET_DEPTH = 2.0     # arm thickness up to 2.0 sits flush (a thicker arm just stands proud)
HUB_HOLE_D = PALM_HUB_D + 2 * HORN_CLEAR
EF_CHAMFER = 0.4       # bed-face chamfer on the pocket: eats elephant foot
SLOT_W, SLOT_R0, SLOT_R1 = 1.5, 10.0, 14.0   # screw slot under the arm (horn's own M2 screw)
COLLAR_R0, COLLAR_R1, COLLAR_H = 10.0, 13.5, 3.0   # inner-face collar for screw bite
RIB_W, RIB_H = 3.0, 3.0

# ankle bar (PLA) and paw boot (TPU)
# "small paw" (2026-11-15). The first version was BAR 11 x 8 x 36, SOLE_R 10, wall 2.5,
# a 19.7 x 15.5 x 37.9 mm boot.
BAR_W, BAR_H = 9.0, 7.0      # bar section: X x Y (stadium)
BAR_LEN = 22.0               # from the outer face inward (Z)
BOOT_GAP = 0.5               # boot back face to the plate's inner face
SOLE_R = 8.0                 # sole radius in the swing plane
SOLE_C = -(CONTACT - SOLE_R) # sole arc centre (Y)
BOOT_TOP_WALL = 2.0
BARB_Z = (LEG_T + 4.5, BAR_LEN - 9.0)   # barb positions along the bar
PIN_Z = BAR_LEN - 4.5                   # cross-pin position along the bar
BOOT_CAP = 2.4               # closed inner end
BAR_FIT = 0.0                # bore = bar (TPU stretches over the barbs)
BARB = 0.35                  # retention ridges on the bar
PIN_D = 2.0                  # optional cross pin: a piece of 1.75 mm filament

SEG = 64


# ---------------------------------------------------------------- helpers
def circ(c, r, seg=SEG):
    return CrossSection.circle(r, seg).translate(list(c))


def hull2(*parts):
    return CrossSection.batch_hull(list(parts))


def prism(cs, z0, z1):
    return Manifold.extrude(cs, z1 - z0).translate([0, 0, z0])


def union(ms):
    return Manifold.batch_boolean(list(ms), OpType.Add)


def stadium(cx, cy, w, h, seg=SEG):
    r = min(w, h) / 2
    if w >= h:
        return hull2(circ((cx - (w / 2 - r), cy), r, seg), circ((cx + (w / 2 - r), cy), r, seg))
    return hull2(circ((cx, cy - (h / 2 - r)), r, seg), circ((cx, cy + (h / 2 - r)), r, seg))


def rot2(cs, deg):
    return cs.rotate(deg)


THIGH_DIR = np.degrees(np.arctan2(KNEE[0], -KNEE[1]))   # angle of the thigh from -Y (deg, + toward +X)


def horn_slot_pieces(extra=HORN_CLEAR):
    """The two convex pieces of the horn pocket outline: tapered arm, hub circle."""
    a0, a1 = PALM_ARM_W0 / 2 + extra, PALM_ARM_W1 / 2 + extra
    arm = CrossSection([[(-a0, 0), (-a1, -PALM_ARM_L), (a1, -PALM_ARM_L), (a0, 0)]])
    return [hull2(arm, circ((0, -PALM_ARM_L), a1)), circ((0, 0), PALM_HUB_D / 2 + extra)]


def horn_slot(extra=HORN_CLEAR):
    """Horn pocket outline (palm-slot shape), hub at the origin, arm along -Y."""
    arm, hub = horn_slot_pieces(extra)
    return arm + hub


# ---------------------------------------------------------------- PLA leg
def leg_outline():
    thigh = hull2(circ((0, 0), HIP_R), circ(KNEE, KNEE_R))
    shin = hull2(circ(KNEE, KNEE_R), circ(ANKLE, ANKLE_R))
    return thigh + shin


def segment_bar(p0, p1, w, z0, z1):
    return prism(hull2(circ(p0, w / 2, 24), circ(p1, w / 2, 24)), z0, z1)


def make_leg_A():
    plate = prism(leg_outline(), 0, LEG_T)

    # inner-face collar around the hub (clears the gearbox boss, r <= 9.0) and ribs
    collar = prism(circ((0, 0), COLLAR_R1) - circ((0, 0), COLLAR_R0), LEG_T - 0.01, LEG_T + COLLAR_H)
    k = np.array(KNEE); a = np.array(ANKLE)
    u = k / np.linalg.norm(k)
    rib_a = segment_bar(tuple(u * (COLLAR_R0 + 3.0)), tuple(k), 5.0, LEG_T - 0.01, LEG_T + RIB_H)   # wide: slot runs in it
    rib_b = segment_bar(tuple(k), tuple(k + 0.62 * (a - k)), RIB_W, LEG_T - 0.01, LEG_T + RIB_H)   # stops above the boot
    knee_boss = prism(circ(KNEE, 4.0), LEG_T - 0.01, LEG_T + RIB_H)

    # ankle bar, roof-shaped inner end (prints without support inside the boot)
    bar_cs = stadium(ANKLE[0], ANKLE[1], BAR_W, BAR_H)
    bar = prism(bar_cs, 0, BAR_LEN)
    roof = Manifold.extrude(bar_cs.translate([-ANKLE[0], -ANKLE[1]]), BAR_H / 2, 1, 0, [0.2, 0.2]) \
        .translate([ANKLE[0], ANKLE[1], BAR_LEN - 0.01])
    barbs = []
    for zc in BARB_Z:
        ring = stadium(ANKLE[0], ANKLE[1], BAR_W + 2 * BARB, BAR_H + 2 * BARB)
        barbs.append(Manifold.extrude(ring.translate([-ANKLE[0], -ANKLE[1]]), 1.6, 1, 0, [BAR_W / (BAR_W + 2 * BARB), BAR_H / (BAR_H + 2 * BARB)])
                     .translate([ANKLE[0], ANKLE[1], zc]))

    leg = union([plate, collar, rib_a, rib_b, knee_boss, bar, roof] + barbs)

    # ---- horn socket
    horn = horn_slot().rotate(THIGH_DIR)             # arm lies along the thigh
    pocket = prism(horn, -1, POCKET_DEPTH)
    hub_hole = Manifold.cylinder(LEG_T + COLLAR_H + 2, HUB_HOLE_D / 2, HUB_HOLE_D / 2, SEG).translate([0, 0, -1])
    hub_cham = Manifold.cylinder(EF_CHAMFER + 0.01, HUB_HOLE_D / 2 + EF_CHAMFER, HUB_HOLE_D / 2, SEG).translate([0, 0, -0.01])
    # bed-face chamfer round the pocket (eats elephant foot): a true 45 deg chamfer, built
    # as the hull of each convex piece (offset at the bed, nominal 0.4 mm up)
    layers = []
    for p in horn_slot_pieces():
        p = p.rotate(THIGH_DIR)
        layers.append(Manifold.batch_hull([prism(p.offset(EF_CHAMFER + 0.01, JoinType.Round, 2.0, SEG), -0.02, -0.01),
                                           prism(p, EF_CHAMFER, EF_CHAMFER + 0.01)]))
    # screw slot along the arm, only where the collar/rib gives >= 5 mm of plastic
    slot = prism(stadium(0, -(SLOT_R0 + SLOT_R1) / 2, SLOT_W, SLOT_R1 - SLOT_R0).rotate(THIGH_DIR),
                 POCKET_DEPTH - 0.01, LEG_T + COLLAR_H + 1)
    # optional cross-pin hole through the bar (fore/aft), for a filament pin
    pin = Manifold.cylinder(BAR_W + 6, PIN_D / 2, PIN_D / 2, 24).rotate([0, 90, 0]) \
        .translate([ANKLE[0] - (BAR_W + 6) / 2, ANKLE[1], PIN_Z])
    cuts = union([pocket, hub_hole, hub_cham, slot, pin] + layers)
    return leg - cuts


def mirror_x(m):
    return m.mirror([1, 0, 0])


# ---------------------------------------------------------------- TPU paw boot
def boot_outline():
    """Swing-plane outline of the boot (print-frame XY)."""
    # sole: the lower part of an R10 arc (covers +-80 deg of swing), so the hip-to-ground
    # distance is set by the arc alone; the top is a rounded shell round the bar
    cut_y = SOLE_C - SOLE_R * np.cos(np.radians(80))
    sole = circ((0, SOLE_C), SOLE_R) ^ CrossSection.square([4 * SOLE_R, 4 * SOLE_R]).translate([-2 * SOLE_R, cut_y - 4 * SOLE_R])
    top = stadium(ANKLE[0], ANKLE[1], BAR_W + 2 * BOOT_TOP_WALL, BAR_H + 2 * BOOT_TOP_WALL)  # rounded shoulders
    return hull2(sole, top)


def make_boot(in_leg_frame=True):
    z0 = LEG_T + BOOT_GAP
    z1 = BAR_LEN + BAR_H / 2 + BOOT_CAP          # covers the roof plus a closed cap
    outline = boot_outline()
    body = prism(outline, z0, z1 - 3.0)
    # rounded inner end: hull with a slightly smaller outline 3 mm further in
    end = prism(outline.offset(-2.2, JoinType.Round, 2.0, SEG), z1 - 0.2, z1)
    body = Manifold.batch_hull([prism(outline, z1 - 3.01, z1 - 3.0), end]) + body
    # bore = bar + roof (+ barbs are left to stretch the TPU)
    bar_cs = stadium(ANKLE[0], ANKLE[1], BAR_W + 2 * BAR_FIT, BAR_H + 2 * BAR_FIT)
    bore = prism(bar_cs, z0 - 1, BAR_LEN + 0.2)
    roof = Manifold.extrude(bar_cs.translate([-ANKLE[0], -ANKLE[1]]), BAR_H / 2 + 0.2, 1, 0, [0.2, 0.2]) \
        .translate([ANKLE[0], ANKLE[1], BAR_LEN + 0.19])
    # tread: grooves across the walking direction (parallel to Z), on the sole only
    # five grooves, each normal to the sole, 0.9 wide x 0.8 deep, stopping 1.5 mm short of each end
    grooves = []
    for ang in (-36.0, -18.0, 0.0, 18.0, 36.0):
        gc = Manifold.cube([0.9, 2.0, z1 - z0 - 5.0]).translate([-0.45, -SOLE_R - 1.2, 0]) \
            .rotate([0, 0, ang]).translate([0, SOLE_C, z0 + 1.5])
        grooves.append(gc)
    pin = Manifold.cylinder(30, PIN_D / 2, PIN_D / 2, 24).rotate([0, 90, 0]).translate([-15, ANKLE[1], PIN_Z])
    # 0.6 mm lead-in at the open end (also eats TPU elephant foot)
    lead = Manifold.extrude(stadium(0, 0, BAR_W + 1.2, BAR_H + 1.2), 0.6, 1, 0,
                            [BAR_W / (BAR_W + 1.2), BAR_H / (BAR_H + 1.2)]).translate([ANKLE[0], ANKLE[1], z0 - 0.001])
    boot = body - union([bore, roof, pin, lead] + grooves)
    return boot


# ---------------------------------------------------------------- export + checks
def to_tm(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=False)


def to_tm_export(m, digits=4):
    """Mesh for STL/3MF export. Snap vertices to 0.0001 mm and drop the needle triangles
    that collapse, so the file stays watertight after float32 rounding (stops slivers
    becoming non-manifold edges in the slicer)."""
    tm = to_tm(m)
    tm = trimesh.Trimesh(tm.vertices.round(digits), tm.faces, process=False)
    tm.merge_vertices()
    f = tm.faces
    tm.update_faces((f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2]))
    tm.remove_unreferenced_vertices()
    return tm


def boot_print_pose(boot):
    """Stand the boot on its open end (z0 face) for printing: bore vertical, no supports."""
    z0 = LEG_T + BOOT_GAP
    return boot.translate([0, -SOLE_C, -z0])


def build():
    leg_a = make_leg_A()
    leg_b = mirror_x(leg_a)
    boot = make_boot()
    return leg_a, leg_b, boot


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "..", "leg_v2")
    os.makedirs(out, exist_ok=True)
    leg_a, leg_b, boot = build()
    parts = {
        "leg_v2_A_x2.stl": leg_a,
        "leg_v2_B_x2.stl": leg_b,
        "paw_boot_TPU_x4.stl": boot_print_pose(boot),
    }
    for fn, m in parts.items():
        tm = to_tm_export(m)
        tm.export(os.path.join(out, fn))
        tm = trimesh.load_mesh(os.path.join(out, fn))      # report what the slicer will read
        ext = tm.extents
        print(f"{fn:22s} {ext[0]:6.1f} x {ext[1]:5.1f} x {ext[2]:5.1f} mm  vol {m.volume() / 1000:5.2f} cm3  "
              f"watertight={tm.is_watertight} bodies={len(m.decompose())} zmin={tm.bounds[0][2]:.2f}")


if __name__ == "__main__":
    main()
