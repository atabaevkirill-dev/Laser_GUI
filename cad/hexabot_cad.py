#!/usr/bin/env python3
"""Parametric CAD model of Hexabot (CadQuery).

Builds every printable part and simple models of the bought parts from
``params.py`` and exports:

* ``cad/stl/<part>.stl``                         print-ready parts (mm)
* ``src/hexabot_description/meshes/visual/*.stl`` per-link visual meshes (mm,
  in the URDF link frames; the URDF scales them by 0.001)
* ``cad/step/<part>.step``                       with ``--step``

Usage:  python cad/hexabot_cad.py [--step] [--only PART ...]

Leg architecture (identical parts for all six legs):

    body tray --(coxa servo, vertical axis)--> coxa yoke + femur servo cradle
    coxa      --(femur servo, horizontal) --> femur yoke + tibia servo cradle
    femur     --(tibia servo, horizontal) --> tibia yoke + beam + TPU foot

Every joint is a U-yoke: one arm on the servo horn, the other on a flanged
bearing (F623ZZ) riding an M3 shoulder screw that is coaxial with the servo
shaft, so the servo spline never carries the bending load alone.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cadquery as cq

sys.path.insert(0, str(Path(__file__).resolve().parent))
import params as P  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
STL_DIR = ROOT / "cad" / "stl"
STEP_DIR = ROOT / "cad" / "step"
VISUAL_DIR = ROOT / "src" / "hexabot_description" / "meshes" / "visual"

S = P.SERVO
CLR = P.YOKE_CLEARANCE
W = P.WALL

# Derived servo stack along the shaft axis (servo frame, +Z out of the horn).
HORN_TOP = S["boss_height"] + S["horn_thickness"]       # horn face above the case
HALF_CASE = S["case_height"] / 2                          # servo case centred on the joint
ARM_T = 4.0                                               # yoke arm thickness
IDLER_PLATE = 3.0                                         # plate carrying the idler screw
# Yoke arm planes measured from the joint centre along the shaft axis.
ARM_HORN_IN = HALF_CASE + HORN_TOP                        # 21.75
ARM_HORN_OUT = ARM_HORN_IN + ARM_T                        # 25.75
ARM_IDLER_IN = HALF_CASE + IDLER_PLATE + CLR              # 20.45
ARM_IDLER_OUT = ARM_IDLER_IN + ARM_T                      # 24.45
ARM_R = 14.0                                              # arm end radius round the axis


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def box(x0, x1, y0, y1, z0, z1) -> cq.Workplane:
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    z0, z1 = sorted((z0, z1))
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def cyl(r, h, base=(0, 0, 0), direction=(0, 0, 1)) -> cq.Workplane:
    solid = cq.Solid.makeCylinder(r, h, cq.Vector(*base), cq.Vector(*direction))
    return cq.Workplane("XY").add(solid)


def sphere(r, center=(0, 0, 0)) -> cq.Workplane:
    return cq.Workplane("XY").sphere(r).translate(center)


def rounded_box(x0, x1, y0, y1, z0, z1, r) -> cq.Workplane:
    b = box(x0, x1, y0, y1, z0, z1)
    try:
        return b.edges("|Z").fillet(min(r, (x1 - x0) / 2.01, (y1 - y0) / 2.01))
    except Exception:  # pragma: no cover - fillet failures fall back to sharp
        return b


def slot_profile_xz(points, y0, y1) -> cq.Workplane:
    """Extrude a closed XZ polygon between y0 and y1."""
    wp = cq.Workplane("XZ", origin=(0, y1, 0)).polyline(points).close()
    # Workplane "XZ" has its normal along -Y, so extruding moves towards -Y.
    return wp.extrude(y1 - y0)


def arm_xz(x_end, z_half, y0, y1, r_end=ARM_R) -> cq.Workplane:
    """Flat yoke arm in an XZ plane: disc of r_end round the origin joined to
    a bar that runs to x_end."""
    disc = cyl(r_end, y1 - y0, base=(0, y0, 0), direction=(0, 1, 0))
    bar = box(0, x_end, y0, y1, -z_half, z_half)
    return disc.union(bar)


def horn_holes(plate: cq.Workplane, axis_base, direction, depth) -> cq.Workplane:
    """Centre hole + 4 bolt holes of the servo horn through a yoke arm."""
    plate = plate.cut(cyl(3.2, depth, axis_base, direction))
    bx, by, bz = axis_base
    dx, dy, dz = direction
    r = S["horn_bolt_circle"] / 2
    for k in range(4):
        a = math.radians(45 + 90 * k)
        if abs(dz) > 0.5:          # axis along Z
            off = (r * math.cos(a), r * math.sin(a), 0)
        elif abs(dy) > 0.5:        # axis along Y
            off = (r * math.cos(a), 0, r * math.sin(a))
        else:                      # axis along X
            off = (0, r * math.cos(a), r * math.sin(a))
        plate = plate.cut(cyl(1.1, depth, (bx + off[0], by + off[1], bz + off[2]), direction))
    return plate


def bearing_pocket(plate: cq.Workplane, axis_base, direction, depth) -> cq.Workplane:
    b = P.IDLER_BEARING
    plate = plate.cut(cyl(b["od"] / 2 + 0.1, b["width"], axis_base, direction))
    return plate.cut(cyl(b["bore"] / 2 + 0.3, depth, axis_base, direction))


def servo_model() -> cq.Workplane:
    """STS3215 in its own frame: shaft on Z, horn face at z = HORN_TOP."""
    L, Wd, H = S["length"], S["width"], S["case_height"]
    case = rounded_box(S["shaft_to_front"] - L, S["shaft_to_front"], -Wd / 2, Wd / 2, -H, 0, 2.0)
    boss = cyl(6.0, S["boss_height"])
    horn = cyl(S["horn_diameter"] / 2, S["horn_thickness"], base=(0, 0, S["boss_height"]))
    # Cable exit and label recess give it a recognisable look.
    plug = box(S["shaft_to_front"] - L - 1.0, S["shaft_to_front"] - L, -6, 6, -H + 6, -H + 14)
    return case.union(boss).union(horn).union(plug)


def place_servo(frame_origin, z_dir, x_dir) -> cq.Workplane:
    """Servo model with its +Z along z_dir and +X along x_dir; the joint centre
    (middle of the case along the shaft) at frame_origin."""
    m = servo_model().translate((0, 0, HALF_CASE))
    plane = cq.Plane(origin=cq.Vector(*frame_origin), xDir=cq.Vector(*x_dir),
                     normal=cq.Vector(*z_dir))
    return cq.Workplane("XY").add(m.val().moved(cq.Location(plane)))


# ---------------------------------------------------------------------------
# Leg parts (each in its URDF link frame: x along the leg, z up)
# ---------------------------------------------------------------------------

def coxa_part() -> cq.Workplane:
    """Coxa yoke: arms on the coxa servo (vertical axis at the origin) and a
    cradle holding the femur servo (axis along Y at x = COXA_LENGTH)."""
    L1 = P.COXA_LENGTH
    Wd = S["width"]
    long_end = S["length"] - S["shaft_to_front"]
    # Femur servo: long axis vertical, long end down, case centred on y = 0.
    cx0, cx1 = L1 - Wd / 2 - W, L1 + Wd / 2 + W
    cz0, cz1 = -long_end - W, S["shaft_to_front"] + W
    cradle = box(cx0, cx1, -HALF_CASE - IDLER_PLATE, HALF_CASE, cz0, cz1)
    cradle = cradle.cut(box(cx0 + W, cx1 - W, -HALF_CASE - 0.01, HALF_CASE + 0.01,
                            -long_end - 0.2, S["shaft_to_front"] + 0.2))
    # Horn side stays open; idler side carries the M3 shoulder screw boss.
    cradle = cradle.union(cyl(5.0, CLR, (L1, -HALF_CASE - IDLER_PLATE - CLR, 0), (0, 1, 0)))
    cradle = cradle.cut(cyl(1.6, 12, (L1, -HALF_CASE - IDLER_PLATE - CLR - 1, 0), (0, 1, 0)))

    # Arms above (horn) and below (idler) the coxa servo.
    top = cyl(ARM_R, ARM_T, (0, 0, ARM_HORN_IN)).union(
        box(0, cx0 + W, -13, 13, ARM_HORN_IN, ARM_HORN_OUT))
    top = horn_holes(top, (0, 0, ARM_HORN_IN - 0.1), (0, 0, 1), ARM_T + 0.2)
    bot = cyl(ARM_R, ARM_T, (0, 0, -ARM_IDLER_OUT)).union(
        box(0, cx0 + W, -13, 13, -ARM_IDLER_OUT, -ARM_IDLER_IN))
    bot = bearing_pocket(bot, (0, 0, -ARM_IDLER_IN - P.IDLER_BEARING["width"] + 0.01),
                         (0, 0, 1), ARM_T + 0.2)
    # Web joining the arms to the cradle's inner wall.
    web = box(cx0, cx0 + W + 1.6, -13, 13, -ARM_IDLER_OUT, ARM_HORN_OUT)
    gusset_t = slot_profile_xz([(cx0 - 10, ARM_HORN_IN), (cx0, ARM_HORN_IN),
                                (cx0, ARM_HORN_IN - 10)], -3, 3)
    gusset_b = slot_profile_xz([(cx0 - 10, -ARM_IDLER_IN), (cx0, -ARM_IDLER_IN),
                                (cx0, -ARM_IDLER_IN + 10)], -3, 3)
    part = cradle.union(top).union(bot).union(web).union(gusset_t).union(gusset_b)
    # Cable pass-through in the web.
    part = part.cut(box(cx0 - 1, cx0 + W + 2, -6, 6, -6, 2))
    return part


def femur_part() -> cq.Workplane:
    """Femur yoke round the femur servo and a tube holding the tibia servo
    (long axis along X pointing back, shaft along Y at x = FEMUR_LENGTH)."""
    L2 = P.FEMUR_LENGTH
    Wd = S["width"]
    long_end = S["length"] - S["shaft_to_front"]
    zt = Wd / 2 + W                    # tube half height
    yt = HALF_CASE + W                 # tube half width (inside the yoke arms)
    tube_x0 = 40.0                     # clears the coxa cradle at any femur angle
    # Arms on the femur servo.
    arm_h = arm_xz(tube_x0 + 6, 12.0, ARM_HORN_IN, ARM_HORN_OUT)
    arm_h = horn_holes(arm_h, (0, ARM_HORN_IN - 0.1, 0), (0, 1, 0), ARM_T + 0.2)
    arm_i = arm_xz(tube_x0 + 6, 12.0, -ARM_IDLER_OUT, -ARM_IDLER_IN)
    arm_i = bearing_pocket(arm_i, (0, -ARM_IDLER_IN - P.IDLER_BEARING["width"] + 0.01, 0),
                           (0, 1, 0), ARM_T + 0.2)
    # Jog plates bring the arms in to the tube.
    jog_h = box(tube_x0, tube_x0 + 6, yt - 0.5, ARM_HORN_OUT, -12, 12)
    jog_i = box(tube_x0, tube_x0 + 6, -ARM_IDLER_OUT, -yt + 0.5, -12, 12)
    # Tube with the tibia servo cradle.
    tube = box(tube_x0, L2 + S["shaft_to_front"] + W, -yt, yt, -zt, zt)
    tube = tube.edges("|Y").fillet(3.0)
    pocket = box(L2 - long_end - 0.2, L2 + S["shaft_to_front"] + 0.2,
                 -HALF_CASE - 0.2, HALF_CASE + 0.2, -Wd / 2 - 0.2, Wd / 2 + 0.2)
    tube = tube.cut(pocket)
    # Lightening window in the tube floor and a cable slot.
    tube = tube.cut(box(tube_x0 + 3, L2 - long_end - 3, -yt + W, yt - W, -zt - 1, zt + 1))
    # Horn side opening and idler boss.
    tube = tube.cut(cyl(12.5, W + 0.4, (L2, HALF_CASE - 0.2, 0), (0, 1, 0)))
    tube = tube.union(cyl(5.0, ARM_IDLER_IN - yt, (L2, -ARM_IDLER_IN, 0), (0, 1, 0)))
    tube = tube.cut(cyl(1.6, 12, (L2, -ARM_IDLER_IN - 1, 0), (0, 1, 0)))
    return arm_h.union(arm_i).union(jog_h).union(jog_i).union(tube)


def tibia_part() -> cq.Workplane:
    """Tibia yoke round the tibia servo, tapered beam down to the foot."""
    L3 = P.TIBIA_LENGTH
    merge_z = -42.0
    foot_r = 9.0
    arm_h = cyl(ARM_R, ARM_T, (0, ARM_HORN_IN, 0), (0, 1, 0)).union(
        box(-12, 12, ARM_HORN_IN, ARM_HORN_OUT, merge_z, 0))
    arm_h = horn_holes(arm_h, (0, ARM_HORN_IN - 0.1, 0), (0, 1, 0), ARM_T + 0.2)
    arm_i = cyl(ARM_R, ARM_T, (0, -ARM_IDLER_OUT, 0), (0, 1, 0)).union(
        box(-12, 12, -ARM_IDLER_OUT, -ARM_IDLER_IN, merge_z, 0))
    arm_i = bearing_pocket(arm_i, (0, -ARM_IDLER_IN - P.IDLER_BEARING["width"] + 0.01, 0),
                           (0, 1, 0), ARM_T + 0.2)
    bridge = box(-12, 12, -ARM_IDLER_OUT, ARM_HORN_OUT, merge_z - 8, merge_z)
    # Tapered beam: loft from a wide section to a narrow one.
    beam = (cq.Workplane("XY", origin=(0, 0, merge_z - 8))
            .rect(22, 2 * ARM_HORN_OUT - 6)
            .workplane(offset=-(L3 - foot_r - (-(merge_z - 8))) )
            .rect(12, 10)
            .loft(combine=True))
    socket = cyl(6.5, 14, (0, 0, -L3 + 11.0 - 3.5), (0, 0, 1))
    part = arm_h.union(arm_i).union(bridge).union(beam).union(socket)
    # Lightening slot through the beam.
    part = part.cut(box(-3, 3, -40, 40, merge_z - 4, merge_z - 30))
    return part


def foot_part() -> cq.Workplane:
    """TPU foot cap pushed onto the tibia socket.  Contact point at z = -L3."""
    L3 = P.TIBIA_LENGTH
    r = 11.0
    ball = sphere(r, (0, 0, -L3 + r))
    sleeve = cyl(8.5, 12, (0, 0, -L3 + r), (0, 0, 1))
    hole = cyl(6.6, 14, (0, 0, -L3 + r - 4), (0, 0, 1))
    return ball.union(sleeve).cut(hole)


# ---------------------------------------------------------------------------
# Body
# ---------------------------------------------------------------------------

def _hull(points):
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def tray_outline():
    pts = []
    for name in P.LEG_NAMES:
        x, y, _ = P.LEG_MOUNTS[name]
        for k in range(36):
            a = 2 * math.pi * k / 36
            pts.append((round(x + P.TRAY_EDGE_RADIUS * math.cos(a), 3),
                        round(y + P.TRAY_EDGE_RADIUS * math.sin(a), 3)))
    return _hull(pts)


def coxa_sweep(name, r0=23.5, r1=70.0, z0=-40.0, z1=40.0) -> cq.Workplane:
    """Region swept by a coxa link inside the tray thickness (cut from tray)."""
    x, y, yaw = P.LEG_MOUNTS[name]
    half = math.radians(P.JOINT_LIMITS["coxa"][1] + 45.0)
    a0 = math.radians(yaw) - half
    n = 24
    outer = [(x + r1 * math.cos(a0 + 2 * half * k / n), y + r1 * math.sin(a0 + 2 * half * k / n))
             for k in range(n + 1)]
    inner = [(x + r0 * math.cos(a0 + 2 * half * k / n), y + r0 * math.sin(a0 + 2 * half * k / n))
             for k in range(n, -1, -1)]
    return (cq.Workplane("XY", origin=(0, 0, z0)).polyline(outer + inner).close()
            .extrude(z1 - z0))


def coxa_servo_pose(name):
    x, y, yaw = P.LEG_MOUNTS[name]
    a = math.radians(yaw)
    return (x, y, 0.0), (0, 0, 1), (math.cos(a), math.sin(a), 0)


def tray_part() -> cq.Workplane:
    t = P.TRAY_THICKNESS
    outline = tray_outline()
    shell = cq.Workplane("XY", origin=(0, 0, -t / 2)).polyline(outline).close().extrude(t)
    inner = (cq.Workplane("XY", origin=(0, 0, -t / 2 + W)).polyline(outline).close()
             .extrude(t).translate((0, 0, 0)))
    try:
        inner = (cq.Workplane("XY", origin=(0, 0, -t / 2 + W)).polyline(outline).close()
                 .offset2D(-W).extrude(t))
    except Exception:
        inner = inner.translate((0, 0, 0))
    tray = shell.cut(inner)
    # Servo pockets with walls.
    L, Wd = S["length"], S["width"]
    for name in P.LEG_NAMES:
        (x, y, _), _, xd = coxa_servo_pose(name)
        a = math.atan2(xd[1], xd[0])
        wall = (box(S["shaft_to_front"] - L - W, S["shaft_to_front"] + W,
                    -Wd / 2 - W, Wd / 2 + W, -t / 2, t / 2)
                .rotate((0, 0, 0), (0, 0, 1), math.degrees(a)).translate((x, y, 0)))
        tray = tray.union(wall)
    for name in P.LEG_NAMES:
        (x, y, _), _, xd = coxa_servo_pose(name)
        a = math.atan2(xd[1], xd[0])
        pocket = (box(S["shaft_to_front"] - L - 0.2, S["shaft_to_front"] + 0.2,
                      -Wd / 2 - 0.2, Wd / 2 + 0.2, -t / 2 - 1, t / 2 + 1)
                  .rotate((0, 0, 0), (0, 0, 1), math.degrees(a)).translate((x, y, 0)))
        tray = tray.cut(pocket)
        tray = tray.cut(coxa_sweep(name))
    # Battery bay floor slot (strap) and standoff bosses for the deck.
    for sx in (-1, 1):
        for sy in (-1, 1):
            px, py = sx * (P.DECK_SIZE[0] / 2 - 16), sy * (P.DECK_SIZE[1] / 2 - 16)
            tray = tray.union(cyl(4.5, t, (px, py, -t / 2)))
            tray = tray.cut(cyl(1.4, t + 2, (px, py, -t / 2 - 1)))
    tray = tray.cut(box(-20, 20, -40, 40, -t / 2 - 1, -t / 2 + W + 1))
    return tray


def idler_bridge_part(name) -> cq.Workplane:
    """Plate under the tray carrying the coxa idler screw (in base_link)."""
    (x, y, _), _, xd = coxa_servo_pose(name)
    a = math.degrees(math.atan2(xd[1], xd[0]))
    t = P.TRAY_THICKNESS
    L = S["length"]
    plate = rounded_box(-L + S["shaft_to_front"] - 2, S["shaft_to_front"] + 2, -14, 14,
                        -t / 2 - IDLER_PLATE, -t / 2, 4.0)
    plate = plate.union(cyl(5.0, CLR, (0, 0, -t / 2 - IDLER_PLATE - CLR)))
    plate = plate.cut(cyl(1.6, 10, (0, 0, -t / 2 - IDLER_PLATE - CLR - 1)))
    return plate.rotate((0, 0, 0), (0, 0, 1), a).translate((x, y, 0))


def deck_part() -> cq.Workplane:
    z0 = P.TRAY_THICKNESS / 2 + P.DECK_STANDOFF
    dx, dy = P.DECK_SIZE
    deck = rounded_box(-dx / 2, dx / 2, -dy / 2, dy / 2, z0, z0 + P.DECK_THICKNESS, 14)
    for sx in (-1, 1):
        for sy in (-1, 1):
            deck = deck.cut(cyl(1.7, 10, (sx * (dx / 2 - 16), sy * (dy / 2 - 16), z0 - 1)))
    # Cable windows and computer mounting holes.
    deck = deck.cut(rounded_box(-15, 15, -50, -30, z0 - 1, z0 + 10, 4))
    deck = deck.cut(rounded_box(-15, 15, 30, 50, z0 - 1, z0 + 10, 4))
    cx, cy = 20.0, 0.0
    for hx in (-41.5, 41.5):
        for hy in (-26.0, 26.0):
            deck = deck.cut(cyl(1.4, 10, (cx + hx, cy + hy, z0 - 1)))
    return deck


def standoffs() -> cq.Workplane:
    z0 = P.TRAY_THICKNESS / 2
    out = None
    dx, dy = P.DECK_SIZE
    for sx in (-1, 1):
        for sy in (-1, 1):
            s = cyl(3.0, P.DECK_STANDOFF, (sx * (dx / 2 - 16), sy * (dy / 2 - 16), z0))
            out = s if out is None else out.union(s)
    return out


def head_tower_part() -> cq.Workplane:
    """Tower on the deck holding the pan servo (vertical axis, horn up)."""
    zd = P.deck_z()
    px, py = P.HEAD_PAN_XY
    horn_face = zd + P.HEAD_TOWER_HEIGHT
    case_top = horn_face - HORN_TOP
    L, Wd = S["length"], S["width"]
    x0, x1 = px + S["shaft_to_front"] - L - W, px + S["shaft_to_front"] + W
    tower = rounded_box(x0, x1, py - Wd / 2 - W, py + Wd / 2 + W, zd, case_top, 3)
    tower = tower.cut(box(x0 + W - 0.2, x1 - W + 0.2, py - Wd / 2 - 0.2, py + Wd / 2 + 0.2,
                          case_top - S["case_height"] - 0.2, case_top + 1))
    flange = rounded_box(x0 - 8, x1 + 8, py - Wd / 2 - 10, py + Wd / 2 + 10, zd, zd + 3, 5)
    for hx in (x0 - 4, x1 + 4):
        for hy in (py - Wd / 2 - 6, py + Wd / 2 + 6):
            flange = flange.cut(cyl(1.7, 10, (hx, hy, zd - 1)))
    return tower.union(flange)


def lidar_riser_part() -> cq.Workplane:
    zd = P.deck_z()
    lx, ly = P.LIDAR_XY
    r = cyl(22.0, P.LIDAR_RISER, (lx, ly, zd)).cut(cyl(12.0, P.LIDAR_RISER + 2, (lx, ly, zd - 1)))
    return r


def depth_mount_part() -> cq.Workplane:
    """Bracket on the tray front holding the depth camera tilted down."""
    t = P.TRAY_THICKNESS
    xf = max(p[0] for p in tray_outline())
    plate = box(xf - 8, xf, -40, 40, -t / 2, t / 2)
    arm = box(xf - 8, P.DEPTH_CAMERA_X - 14, -30, 30, -t / 2, -t / 2 + 4)
    return plate.union(arm)


def gnss_pad_part() -> cq.Workplane:
    """Low pad for the GNSS/compass puck: under the lidar plane and the head."""
    zd = P.deck_z()
    gx, gy = P.GNSS_XY
    pad = rounded_box(gx - 24, gx + 24, gy - 24, gy + 24, zd, zd + P.GNSS_MAST_HEIGHT, 6)
    return pad.cut(rounded_box(gx - 20.4, gx + 20.4, gy - 20.4, gy + 20.4,
                               zd + P.GNSS_MAST_HEIGHT - 2, zd + P.GNSS_MAST_HEIGHT + 1, 6))


# ---------------------------------------------------------------------------
# Head (pan-tilt with the scout payload)
# ---------------------------------------------------------------------------

def head_pan_part() -> cq.Workplane:
    """Pan platform with the tilt yoke ears (head_pan_link frame: origin on the
    pan horn face, tilt axis at z = HEAD_TILT_HEIGHT along Y)."""
    h = P.HEAD_TILT_HEIGHT
    ey = P.HEAD_EAR_Y
    plate = cyl(36.0, 4.0)
    plate = horn_holes(plate, (0, 0, -0.1), (0, 0, 1), 4.2)
    plate = plate.union(rounded_box(-16, 16, -(ey - CLR) - HORN_TOP - S["case_height"] - W,
                                    ey + 6, 0, 4, 6))
    # Left ear: idler screw.
    ear_l = slot_profile_xz([(-14, 0), (14, 0), (14, h), (0, h + 14), (-14, h)], ey, ey + 6)
    ear_l = ear_l.union(cyl(14.0, 6.0, (0, ey, h), (0, 1, 0)))
    ear_l = ear_l.cut(cyl(1.6, 10, (0, ey - 1, h), (0, 1, 0)))
    # Right side: box housing the tilt servo, horn facing +Y (into the yoke).
    L, Wd = S["length"], S["width"]
    hx0, hx1 = -L + S["shaft_to_front"] - W, S["shaft_to_front"] + W
    servo_y1 = -(ey - CLR) - HORN_TOP         # case face towards the cradle
    servo_y0 = servo_y1 - S["case_height"]
    house = box(-Wd / 2 - W, Wd / 2 + W, servo_y0 - W, servo_y1, 0,
                h + L - S["shaft_to_front"] + W)
    house = house.cut(box(-Wd / 2 - 0.2, Wd / 2 + 0.2, servo_y0 - 0.2, servo_y1 + 0.1,
                          h - S["shaft_to_front"] - 0.2, h + L - S["shaft_to_front"] + 0.2))
    house = house.cut(cyl(8.0, W + 2, (0, servo_y1 - W - 1, h), (0, 1, 0)))
    del hx0, hx1
    return plate.union(ear_l).union(house)


def head_tilt_part() -> cq.Workplane:
    """Payload cradle (head_tilt_link frame: origin on the tilt axis, +X is the
    common optical axis of LRF, cameras and illuminator)."""
    ey = P.HEAD_EAR_Y
    inner = ey - CLR
    lrf = P.LRF["size"]
    cradle = box(-14, 46, -inner + ARM_T, inner - ARM_T, -24, -21)   # floor
    side_l = box(-14, 30, inner - ARM_T, inner, -24, 30)
    side_r = box(-14, 30, -inner, -inner + ARM_T, -24, 30)
    side_l = side_l.union(cyl(12.0, ARM_T, (0, inner - ARM_T, 0), (0, 1, 0)))
    side_r = side_r.union(cyl(12.0, ARM_T, (0, -inner, 0), (0, 1, 0)))
    side_l = bearing_pocket(side_l, (0, inner - P.IDLER_BEARING["width"] + 0.01, 0), (0, 1, 0), 10)
    side_r = horn_holes(side_r, (0, -inner - 0.1, 0), (0, 1, 0), ARM_T + 0.2)
    # LRF clamp: U channel whose walls continue up to carry the illuminator.
    lx, ly, lz = P.PAYLOAD_FRAMES["lrf"]
    ix, iy, iz = P.PAYLOAD_FRAMES["illuminator"]
    isz = P.ILLUMINATOR["size"]
    saddle_z = iz - isz[2] / 2 - 2.0
    cx0, cx1 = lx - lrf[0] / 2 + 6, lx + lrf[0] / 2 - 6
    clamp = box(cx0, cx1, ly - lrf[1] / 2 - W, ly + lrf[1] / 2 + W, -21, lz - lrf[2] / 2)
    for y0, y1 in ((ly - lrf[1] / 2 - W, ly - lrf[1] / 2), (ly + lrf[1] / 2, ly + lrf[1] / 2 + W)):
        clamp = clamp.union(box(cx0, cx1, y0, y1, -21, saddle_z))
    saddle = box(ix - 20, ix + 20, ly - lrf[1] / 2 - W, ly + lrf[1] / 2 + W, saddle_z, saddle_z + 2)
    # Shelves under the low-light camera and the thermal core.
    cx, cy, cz = P.PAYLOAD_FRAMES["lowlight_camera"]
    cs = P.LOWLIGHT_CAMERA["size"]
    shelf_c = box(cx - 10, cx + 8, cy - cs[0] / 2 + 3, cy + cs[0] / 2 - 3, -21, cz - cs[1] / 2)
    tx, ty, tz = P.PAYLOAD_FRAMES["thermal_camera"]
    ts = P.THERMAL["size"]
    shelf_t = box(tx - ts[0] / 2 + 2, tx + ts[0] / 2 - 2, ty - ts[1] / 2, ty + ts[1] / 2,
                  -21, tz - ts[2] / 2 - 4)
    saddle = saddle.union(shelf_c).union(shelf_t)
    return cradle.union(side_l).union(side_r).union(clamp).union(saddle)


def payload_models() -> cq.Workplane:
    """Bought payload modules in head_tilt_link (visual only)."""
    out = None

    def add(shape):
        nonlocal out
        out = shape if out is None else out.union(shape)

    lx, ly, lz = P.PAYLOAD_FRAMES["lrf"]
    s = P.LRF["size"]
    lrf = rounded_box(lx - s[0] / 2, lx + s[0] / 2, ly - s[1] / 2, ly + s[1] / 2,
                      lz - s[2] / 2, lz + s[2] / 2, 2)
    # Transmit (8 mm) and receive (16 mm) windows.
    lrf = lrf.union(cyl(P.LRF["rx_aperture"] / 2 + 1, 2, (lx + s[0] / 2, ly - 5, lz), (1, 0, 0)))
    lrf = lrf.union(cyl(P.LRF["tx_aperture"] / 2 + 1, 2, (lx + s[0] / 2, ly + 9, lz), (1, 0, 0)))
    add(lrf)
    cx, cy, cz = P.PAYLOAD_FRAMES["lowlight_camera"]
    cs = P.LOWLIGHT_CAMERA["size"]
    cam = rounded_box(cx - cs[2] / 2, cx + cs[2] / 2 - 10, cy - cs[0] / 2, cy + cs[0] / 2,
                      cz - cs[1] / 2, cz + cs[1] / 2, 3)
    cam = cam.union(cyl(8.5, 16, (cx + cs[2] / 2 - 10, cy, cz), (1, 0, 0)))
    add(cam)
    tx, ty, tz = P.PAYLOAD_FRAMES["thermal_camera"]
    ts = P.THERMAL["size"]
    th = rounded_box(tx - ts[0] / 2, tx + ts[0] / 2, ty - ts[1] / 2, ty + ts[1] / 2,
                     tz - ts[2] / 2 - 4, tz + ts[2] / 2 + 4, 2)
    th = th.union(cyl(5.0, 3, (tx + ts[0] / 2, ty, tz), (1, 0, 0)))
    add(th)
    ix, iy, iz = P.PAYLOAD_FRAMES["illuminator"]
    isz = P.ILLUMINATOR["size"]
    ill = cyl(isz[1] / 2, isz[0], (ix - isz[0] / 2, iy, iz), (1, 0, 0))
    ill = ill.union(cyl(isz[1] / 2 + 2, 8, (ix + isz[0] / 2 - 8, iy, iz), (1, 0, 0)))
    for k in range(5):
        ill = ill.union(box(ix - isz[0] / 2 + 6 + 6 * k, ix - isz[0] / 2 + 8 + 6 * k,
                            iy - isz[1] / 2 - 2, iy + isz[1] / 2 + 2, iz - 4, iz + 4))
    add(ill)
    return out


# ---------------------------------------------------------------------------
# Bought parts on the base (visual only)
# ---------------------------------------------------------------------------

def base_servos() -> cq.Workplane:
    out = None
    for name in P.LEG_NAMES:
        org, zd, xd = coxa_servo_pose(name)
        s = place_servo(org, zd, xd)
        out = s if out is None else out.union(s)
    px, py, pz = P.head_pan_xyz()
    s = place_servo((px, py, pz - HORN_TOP - HALF_CASE), (0, 0, 1), (1, 0, 0))
    return out.union(s)


def base_electronics() -> cq.Workplane:
    zd = P.deck_z()
    bx, by, bz = P.LIPO_3S["size"]
    t = P.TRAY_THICKNESS
    battery = rounded_box(-bx / 2, bx / 2, -by / 2, by / 2, -t / 2 + W, -t / 2 + W + bz, 4)
    cx, cy, cz = P.COMPUTER["size"]
    board = rounded_box(20 - cx / 2, 20 + cx / 2, -cy / 2, cy / 2, zd + 5, zd + 7, 3)
    heatsink = box(20 - 30, 20 + 30, -25, 25, zd + 7, zd + 7 + 14)
    for k in range(9):
        heatsink = heatsink.cut(box(20 - 28 + 6.5 * k, 20 - 28 + 6.5 * k + 3, -26, 26,
                                    zd + 11, zd + 22))
    adapter = rounded_box(-40, -10, 35, 60, zd, zd + 10, 2)
    dcdc = rounded_box(-40, -10, -60, -35, zd, zd + 12, 2)
    return battery.union(board).union(heatsink).union(adapter).union(dcdc)


def base_sensors() -> cq.Workplane:
    zd = P.deck_z()
    lx, ly = P.LIDAR_XY
    zl = zd + P.LIDAR_RISER
    lidar = rounded_box(lx - 19.3, lx + 19.3, ly - 19.3, ly + 19.3, zl, zl + 18, 4)
    lidar = lidar.union(cyl(17.0, 16.8, (lx, ly, zl + 18)))
    # Depth camera: rotate a box about Y for the tilt.
    sx, sy, sz = P.DEPTH_CAMERA["size"]
    cam = rounded_box(-sz / 2, sz / 2, -sx / 2, sx / 2, -sy / 2, sy / 2, 6)
    for k, yy in enumerate((-40, -15, 15, 40)):
        cam = cam.union(cyl(4.5 if k % 3 == 0 else 3.5, 1.5, (sz / 2, yy, 0), (1, 0, 0)))
    cam = (cam.rotate((0, 0, 0), (0, 1, 0), P.DEPTH_CAMERA_PITCH)
           .translate((P.DEPTH_CAMERA_X, 0, P.DEPTH_CAMERA_Z)))
    gx, gy = P.GNSS_XY
    gz = zd + P.GNSS_MAST_HEIGHT - 2
    gnss = rounded_box(gx - 20, gx + 20, gy - 20, gy + 20, gz, gz + 12, 8)
    return lidar.union(cam).union(gnss)


# ---------------------------------------------------------------------------
# Assemblies per URDF link
# ---------------------------------------------------------------------------

def coxa_servo_in_coxa() -> cq.Workplane:
    """Femur servo as it sits in the coxa link: shaft along +Y, long end down."""
    return place_servo((P.COXA_LENGTH, 0, 0), (0, 1, 0), (0, 0, 1))


def tibia_servo_in_femur() -> cq.Workplane:
    """Tibia servo in the femur link: shaft along +Y, short end towards +X."""
    return place_servo((P.FEMUR_LENGTH, 0, 0), (0, 1, 0), (1, 0, 0))


def tilt_servo_in_pan() -> cq.Workplane:
    """Tilt servo in the pan link: shaft along +Y (horn into the yoke)."""
    h = P.HEAD_TILT_HEIGHT
    ey = P.HEAD_EAR_Y
    servo_y1 = -(ey - CLR) - HORN_TOP
    centre_y = servo_y1 - HALF_CASE
    return place_servo((0, centre_y, h), (0, 1, 0), (0, 0, -1))


PRINT_PARTS = {
    # name: (builder, quantity, material)
    "body_tray": (tray_part, 1, "PETG or PA6-CF"),
    "deck": (deck_part, 1, "PETG"),
    "head_tower": (head_tower_part, 1, "PETG or PA6-CF"),
    "lidar_riser": (lidar_riser_part, 1, "PETG"),
    "depth_camera_mount": (depth_mount_part, 1, "PETG"),
    "gnss_pad": (gnss_pad_part, 1, "PETG"),
    "coxa": (coxa_part, 6, "PA6-CF or PETG"),
    "femur": (femur_part, 6, "PA6-CF or PETG"),
    "tibia": (tibia_part, 6, "PA6-CF or PETG"),
    "foot": (foot_part, 6, "TPU 95A"),
    "head_pan": (head_pan_part, 1, "PETG"),
    "head_tilt_cradle": (head_tilt_part, 1, "PETG"),
    "coxa_idler_bridge": (lambda: idler_bridge_part("lm").translate(
        (-P.LEG_MOUNTS["lm"][0], -P.LEG_MOUNTS["lm"][1], 0)), 6, "PETG"),
}


def visual_meshes() -> dict:
    """Per-link visual meshes: {file_stem: workplane} in link frames (mm)."""
    bridges = None
    for name in P.LEG_NAMES:
        b = idler_bridge_part(name)
        bridges = b if bridges is None else bridges.union(b)
    base_printed = (tray_part().union(deck_part()).union(standoffs()).union(bridges)
                    .union(head_tower_part()).union(lidar_riser_part())
                    .union(depth_mount_part()).union(gnss_pad_part()))
    return {
        "base_printed": base_printed,
        "base_servos": base_servos(),
        "base_electronics": base_electronics(),
        "base_sensors": base_sensors(),
        "coxa_printed": coxa_part(),
        "coxa_servo": coxa_servo_in_coxa(),
        "femur_printed": femur_part(),
        "femur_servo": tibia_servo_in_femur(),
        "tibia_printed": tibia_part(),
        "tibia_foot": foot_part(),
        "head_pan_printed": head_pan_part(),
        "head_pan_servo": tilt_servo_in_pan(),
        "head_tilt_printed": head_tilt_part(),
        "head_tilt_payload": payload_models(),
    }


def bbox(wp: cq.Workplane):
    bb = wp.val().BoundingBox()
    return (bb.xlen, bb.ylen, bb.zlen)


def fits_bed(dims) -> bool:
    """True if the part fits the bed in some orientation (sorted dims)."""
    a = sorted(dims)
    b = sorted(P.PRINT_BED)
    return all(x <= y for x, y in zip(a, b))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--step", action="store_true", help="also export STEP files")
    ap.add_argument("--only", nargs="*", help="only these print parts / visual meshes")
    ap.add_argument("--no-visual", action="store_true", help="skip per-link visual meshes")
    args = ap.parse_args(argv)

    for p in P.export_geometry(ROOT):
        print("geometry ->", p.relative_to(ROOT))

    STL_DIR.mkdir(parents=True, exist_ok=True)
    VISUAL_DIR.mkdir(parents=True, exist_ok=True)
    ok = True
    report = ["| part | qty | material | size (mm) | fits H2D/H2S |", "|---|---|---|---|---|"]
    for name, (builder, qty, material) in PRINT_PARTS.items():
        if args.only and name not in args.only:
            continue
        wp = builder()
        dims = bbox(wp)
        fit = fits_bed(dims)
        ok &= fit
        cq.exporters.export(wp, str(STL_DIR / f"{name}.stl"), tolerance=0.05, angularTolerance=0.15)
        if args.step:
            STEP_DIR.mkdir(parents=True, exist_ok=True)
            cq.exporters.export(wp, str(STEP_DIR / f"{name}.step"))
        size = " x ".join(f"{d:.0f}" for d in dims)
        report.append(f"| {name} | {qty} | {material} | {size} | {'yes' if fit else 'NO'} |")
        print(f"part {name:20s} {size:>18s}  {'ok' if fit else 'DOES NOT FIT'}")
    (STL_DIR / "PARTS.md").write_text("# Print parts (generated)\n\n" + "\n".join(report) + "\n")

    if not args.no_visual:
        for stem, wp in visual_meshes().items():
            if args.only and stem not in args.only:
                continue
            cq.exporters.export(wp, str(VISUAL_DIR / f"{stem}.stl"),
                                tolerance=0.25, angularTolerance=0.35)
            print(f"visual {stem}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
