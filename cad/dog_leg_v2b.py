#!/usr/bin/env python3
"""
QuadBot-4 leg v2b -- leg v2 with a LONGER horn pocket, so the horn presses in with no
trimming, plus a small fit-test coupon.

    python dog_leg_v2b.py           # writes ../leg_v2b/*.stl ; ../leg_v2/ is left untouched

Why: The author had to cut the v2 pocket longer by hand to get his horn in (2026-11-20), and then
glue it because it was loose after trimming. The v2 pocket copies a slot known to fit this horn: arm tip end 15.75 mm from the hub centre (15.85 with clearance). His horn is longer.

What changes (everything else is leg v2, imported unchanged):
  * the arm keeps v2's proven widths and taper up to 13.75 mm from the hub (that is what
    grips the horn sideways), then runs PARALLEL out to TIP_END, so a longer arm never meets
    a narrowing wall;
  * the screw slot under the arm runs out to SLOT_R1 = 16.5 (v2: 14) so the horn's own
    small screw can go through one of the outer arm holes as well: the horn is clamped by
    the screw, not glue.

The Miuzei MG90S listing he bought (Amazon B0BWJ2CK9D, ordered 2026-10-26) does not publish
horn dimensions. TIP_END = 19.5 is chosen to cover the usual 9 g single-arm horn (about
20 mm overall with a 7 mm hub, i.e. 16.5-17 mm hub centre to tip) with margin. Measure his
horn: TIP_END = (hub centre to the arm's tip end) + 0.3. A longer pocket costs nothing:
the hub hole and the tapered arm locate the horn; the gap past the tip is just air.
"""
import os
import sys

from manifold3d import CrossSection, Manifold

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dog_leg_v2 as d  # noqa: E402

TIP_END = 21.0         # hub centre to the pocket's tip end, before HORN_CLEAR (v2: 15.75).
                       # 2026-11-20: no fit test for MakerWorld builders, so take the longest
                       # coupon size: a longer pocket only leaves air past the arm tip
SLOT_R1 = 16.5         # screw slot outer end (v2: 14.0)
COUPON_TIPS = (18.0, 19.5, 21.0)   # fit-test coupon: three pockets, marked 1, 2, 3 dots


def horn_slot_pieces(extra=d.HORN_CLEAR, tip_end=None):
    """v2 outline (hub circle + tapered arm to 13.75 mm), then a parallel run to tip_end."""
    tip_end = TIP_END if tip_end is None else tip_end
    a0, a1 = d.PALM_ARM_W0 / 2 + extra, d.PALM_ARM_W1 / 2 + extra
    l1 = d.PALM_ARM_L                                  # end of the taper (13.75)
    l2 = max(l1, tip_end - d.PALM_ARM_W1 / 2)          # centre of the round tip
    arm = CrossSection([[(-a0, 0), (-a1, -l1), (-a1, -l2), (a1, -l2), (a1, -l1), (a0, 0)]])
    return [d.hull2(arm, d.circ((0, -l2), a1)), d.circ((0, 0), d.PALM_HUB_D / 2 + extra)]


def apply():
    """Patch leg v2's builders in place (also used by knee_leg_v3.py)."""
    d.horn_slot_pieces = horn_slot_pieces
    d.SLOT_R1 = SLOT_R1


def make_coupon():
    """4 mm plate with three pockets (tip ends COUPON_TIPS) and through hub holes.
    Print flat, pocket side down, like the legs. Press the horn into each: the right one
    seats flush with the hub fully down and holds the horn without glue."""
    pitch, w, top = 13.0, 13.0 * len(COUPON_TIPS) + 3.0, d.LEG_T
    y0, y1 = -max(COUPON_TIPS) - 3.5, d.PALM_HUB_D / 2 + 3.5
    plate = Manifold.cube([w, y1 - y0, top]).translate([-w / 2, y0, 0])
    cuts = []
    for i, tip in enumerate(COUPON_TIPS):
        x = (i - (len(COUPON_TIPS) - 1) / 2) * pitch
        pieces = horn_slot_pieces(tip_end=tip)
        outline = pieces[0] + pieces[1]
        cuts.append(d.prism(outline, -1, d.POCKET_DEPTH).translate([x, 0, 0]))
        cuts.append(Manifold.cylinder(top + 2, d.HUB_HOLE_D / 2, d.HUB_HOLE_D / 2, d.SEG).translate([x, 0, -1]))
        cuts.append(Manifold.cylinder(d.EF_CHAMFER + 0.01, d.HUB_HOLE_D / 2 + d.EF_CHAMFER, d.HUB_HOLE_D / 2, d.SEG)
                    .translate([x, 0, -0.01]))
        for j in range(i + 1):                         # 1, 2, 3 dimples on the top face
            cuts.append(Manifold.cylinder(1.0, 0.8, 0.8, 24).translate([x - 2.0 * i / 2 + 2.0 * j, y1 - 1.8, top - 0.6]))
    return plate - d.union(cuts)


def main():
    apply()
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "..", "leg_v2b")
    os.makedirs(out, exist_ok=True)
    leg_a, leg_b, boot = d.build()
    parts = {
        "leg_v2b_A_x2.stl": leg_a,
        "leg_v2b_B_x2.stl": leg_b,
        "horn_fit_coupon_PLA_x1.stl": make_coupon(),
    }
    import trimesh
    for fn, m in parts.items():
        tm = d.to_tm_export(m)
        tm.export(os.path.join(out, fn))
        tm = trimesh.load_mesh(os.path.join(out, fn))
        e = tm.extents
        print(f"{fn:28s} {e[0]:6.1f} x {e[1]:5.1f} x {e[2]:5.1f} mm  vol {m.volume() / 1000:5.2f} cm3  "
              f"watertight={tm.is_watertight} bodies={len(m.decompose())}")
    print(f"pocket tip end: v2 {d.PALM_ARM_L + d.PALM_ARM_W1 / 2 + d.HORN_CLEAR:.2f} mm -> v2b "
          f"{TIP_END + d.HORN_CLEAR:.2f} mm from the hub centre; screw slot to {SLOT_R1} mm")


if __name__ == "__main__":
    main()
