"""
qb4_ref.py -- reference geometry for QuadBot-4 leg work (written for the leg v2 redesign).

Re-states the dimensions of the original build_quadruped.py (read, not run) so the
new leg can be checked against the real body, servo and horn. Same frames:
  robot: +X forward, +Y left, +Z up, Z=0 under the chassis, X=Y=0 body centre.
  leg print frame: X fore/aft, Y along the leg (hip at 0, foot negative),
                   Z=0 the OUTER face (on the bed), +Z toward the robot.
"""
import numpy as np
import trimesh
from manifold3d import Manifold, CrossSection

# ---- body / servo numbers (from build_quadruped.py) ----------------------------
SV_L, SV_W, SV_H = 22.8, 12.4, 22.7
SV_EAR_L, SV_EAR_T, SV_EAR_Z = 32.5, 2.5, 16.0
SV_HOLE_PITCH = 27.8
SV_SHAFT_OFF = 5.3
SV_BUMP_H, SV_SHAFT_H = 5.7, 4.1
HORN_HUB_D, HORN_HUB_H, HORN_ARM_T = 7.0, 4.2, 1.5
HORN_W0, HORN_W1, HORN_LEN = 5.5, 4.0, 16.0

TL, TW, TH = 150.0, 62.0, 24.0
FLOOR = 2.4
DECK_T = 3.0
SERVO_X = 55.0
HIP_X = SERVO_X - SV_SHAFT_OFF            # 49.7
HIP_Z = FLOOR + (TH - FLOOR) / 2          # 13.2
Y_WALL = TW / 2                           # 31.0
Y_BOSS_TOP = Y_WALL + (SV_H - SV_EAR_Z) + SV_BUMP_H   # 43.4, top of the gearbox boss
Y_SHAFT_TOP = Y_BOSS_TOP + SV_SHAFT_H                  # 47.5

LEGS = ["FL", "FR", "RL", "RR"]
LEG_SIGN = {"FL": (1, 1), "FR": (1, -1), "RL": (-1, 1), "RR": (-1, -1)}

SEG = 40


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cylz(x, y, z0, z1, r, seg=SEG):
    return Manifold.cylinder(z1 - z0, r, r, seg).translate([x, y, z0])


def make_servo():
    """MG90S in its own frame (X along body, Z up, bottom Z=0, shaft at X=+off)."""
    body = box(-SV_L / 2, SV_L / 2, -SV_W / 2, SV_W / 2, 0, SV_H)
    ears = box(-SV_EAR_L / 2, SV_EAR_L / 2, -SV_W / 2, SV_W / 2, SV_EAR_Z, SV_EAR_Z + SV_EAR_T)
    # screw heads on top of the ears (M2 pan head ~3.8 dia x 1.5)
    heads = cylz(-SV_HOLE_PITCH / 2, 0, SV_EAR_Z + SV_EAR_T, SV_EAR_Z + SV_EAR_T + 1.6, 1.9) + \
        cylz(SV_HOLE_PITCH / 2, 0, SV_EAR_Z + SV_EAR_T, SV_EAR_Z + SV_EAR_T + 1.6, 1.9)
    top = SV_H + SV_BUMP_H
    boss = cylz(SV_SHAFT_OFF, 0, SV_H - 0.1, top, 5.9) + cylz(SV_SHAFT_OFF - 6.2, 0, SV_H - 0.1, top, 2.8)
    shaft = cylz(SV_SHAFT_OFF, 0, top - 0.1, top + SV_SHAFT_H, 2.4)
    return body + ears + boss + heads, shaft


def place_servo(m, name):
    sx, sy = LEG_SIGN[name]
    return m.transform(np.array([[-sx, 0, 0, sx * SERVO_X],
                                 [0, 0, sy, sy * (Y_WALL - SV_EAR_Z)],
                                 [0, sx * sy, 0, HIP_Z]], dtype=float))


def leg_matrix(name, theta_deg, y_leg_out):
    """Leg print frame -> robot frame, swung theta (deg, + = foot forward)."""
    sx, sy = LEG_SIGN[name]
    t = np.radians(theta_deg)
    c, s = np.cos(t), np.sin(t)
    base = np.array([[sy, 0, 0], [0, 0, -sy], [0, 1, 0]], dtype=float)
    rot = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
    r = rot @ base
    off = np.array([sx * HIP_X, sy * y_leg_out, HIP_Z])
    return np.hstack([r, off[:, None]])


def to_manifold(tm):
    tm = tm.copy()
    tm.merge_vertices()
    from manifold3d import Mesh
    return Manifold(Mesh(vert_properties=np.asarray(tm.vertices, dtype=np.float32),
                         tri_verts=np.asarray(tm.faces, dtype=np.uint32)))


def to_trimesh(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=False)


def horn_outline(extra, length=HORN_LEN, w0=HORN_W0, w1=HORN_W1, hub_d=HORN_HUB_D):
    a0, a1 = w0 / 2 + extra, w1 / 2 + extra
    arm = CrossSection([[(-a0, 0), (-a1, -length), (a1, -length), (a0, 0)]])
    tip = CrossSection.circle(a1, 32).translate([0, -length])
    hub = CrossSection.circle(hub_d / 2 + extra, 48)
    return arm + tip + hub
