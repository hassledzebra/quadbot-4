"""Static balance of each QuadBot-4 trick pose, leg v2 (one fore/aft hip servo per leg).
Leg: sole arc centre 58 mm from the hip axis along the leg, sole radius 8, so with leg angle a
(+ = foot forward) the contact is 58 sin a ahead of the hip and the hip stands 58 cos a + 8 high.
Hips at x = +-49.7. The TPU boot touches the floor along a line from |y| 20.0 to 43.4.
COM assumed at the body centre, 12 mm above the hip axes (servos, Uno, battery on the deck)."""
import numpy as np
from shapely.geometry import MultiPoint, Point

HX, R, SR = 49.7, 58.0, 8.0
Y_IN, Y_OUT = 20.0, 43.4
COM_DZ = 12.0
LEGS = {"FL": (+1, +1), "FR": (+1, -1), "RL": (-1, +1), "RR": (-1, -1)}


def pose(ang, up=(), com_dx=0.0, strip=True):
    """ang: dict leg -> deg. up: legs off the floor. Returns (margin mm, pitch deg, hip heights)."""
    hip = {n: (s * HX, R * np.cos(np.radians(ang[n])) + SR) for n, (s, _) in LEGS.items()}
    # body pitch from the front/rear hip heights of the legs on the floor (both sides alike here)
    down = [n for n in LEGS if n not in up]
    zf = np.mean([hip[n][1] for n in down if n[0] == "F"]) if any(n[0] == "F" for n in down) else None
    zr = np.mean([hip[n][1] for n in down if n[0] == "R"])
    pitch = np.degrees(np.arctan2(zf - zr, 2 * HX)) if zf is not None else 0.0   # + = nose up
    pts = []
    for n in down:
        s, side = LEGS[n]
        x = s * HX + R * np.sin(np.radians(ang[n]))
        ys = (Y_IN, Y_OUT) if strip else ((Y_IN + Y_OUT) / 2,)
        pts += [(x, side * y) for y in ys]
    poly = MultiPoint(pts).convex_hull
    # COM projection: body tilted by pitch about the body centre; COM sits COM_DZ above the hips
    cx = com_dx - COM_DZ * np.sin(np.radians(pitch))
    c = Point(cx, 0.0)
    m = poly.exterior.distance(c) if poly.contains(c) else -poly.exterior.distance(c)
    return m, pitch


def show(name, ang, up=()):
    ang = {**dict(FL=0, FR=0, RL=0, RR=0), **ang}
    a, p = pose(ang, up, strip=False)
    b, _ = pose(ang, up, strip=True)
    f, _ = pose(ang, up, com_dx=+10, strip=False)
    print(f"{name:34s} pitch {p:+5.1f}  margin: paw centre line {a:+5.1f} mm | whole boot {b:+5.1f} mm"
          f" | COM 10 mm fwd {f:+5.1f} mm")


L = 25
print("three legs down -- the risky ones")
show("lift FR, no lean (for comparison)", dict(FR=50), up=("FR",))
show("wave FR, lean 25 (paw at 42..60)", dict(FL=L, FR=50, RL=L, RR=L), up=("FR",))
show("wave FL, lean 25", dict(FL=50, FR=L, RL=L, RR=L), up=("FL",))
show("shake FR, lean 25", dict(FL=L, FR=48, RL=L, RR=L), up=("FR",))
show("high five FR, lean 25, paw at 60", dict(FL=L, FR=60, RL=L, RR=L), up=("FR",))
print("four legs down")
show("stand", {})
show("bark lunge (front -16, rear -10)", dict(FL=-16, FR=-16, RL=-10, RR=-10))
show("play bow (front +55, rear 0)", dict(FL=55, FR=55))
show("bow wiggle (RL +12, RR -12)", dict(FL=55, FR=55, RL=12, RR=-12))
show("sit up (front 0, rear -55)", dict(RL=-55, RR=-55))
show("sit up, unsafe variant: rear +55", dict(RL=55, RR=55))
show("stretch front (front +60)", dict(FL=60, FR=60))
show("stretch rear (rear -60)", dict(RL=-60, RR=-60))
show("dance twist (left +15, right -15)", dict(FL=15, RL=15, FR=-15, RR=-15))
show("dance squat (front +20, rear -20)", dict(FL=20, FR=20, RL=-20, RR=-20))
show("tail wag (RL +15, RR -15)", dict(RL=15, RR=-15))
show("lie down (existing sit, +-55)", dict(FL=55, FR=55, RL=-55, RR=-55))
for n, a in (("lift height of the waving paw", None),):
    pass
hh = R * np.cos(np.radians(L)) + SR
for a in (42, 50, 60):
    print(f"wave paw at {a} deg: {hh - (R * np.cos(np.radians(a)) + SR):4.1f} mm off the floor")
