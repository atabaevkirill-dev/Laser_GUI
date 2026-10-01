"""Hexabot geometry: the single source of truth.

Every dimension of the robot lives here (millimetres / degrees).  The CAD
script builds the printable parts from it, and ``export_geometry`` writes the
same numbers (metres / radians) for the ROS 2 packages and the web simulation:

    src/hexabot_description/config/geometry.yaml   (xacro + locomotion node)
    sim/web/assets/geometry.json                   (three.js simulation)

Change a number here, run ``python cad/hexabot_cad.py`` and everything stays
consistent.  Dimensions of bought parts are taken from datasheets and marked
"verify" where a real part should be measured before printing.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

# ---------------------------------------------------------------------------
# Bought parts
# ---------------------------------------------------------------------------

# Feetech STS3215 (12 V / 30 kg*cm version).  Frame: origin on the output
# shaft axis at the horn face, +Z out of the horn, +X towards the short end.
SERVO = {
    "length": 45.2,          # along X
    "width": 24.7,           # along Y
    "case_height": 32.5,     # along Z, without the spline boss (verify)
    "boss_height": 2.5,      # spline boss above the case (verify)
    "shaft_to_front": 12.0,  # shaft axis -> short end of the case (verify)
    "horn_diameter": 22.0,
    "horn_thickness": 3.0,
    "horn_bolt_circle": 14.0,  # 4x M2 on this diameter (verify)
    "case_screw": 2.0,         # M2 self-tapping into the case bosses
    "mass": 0.055,             # kg
}

# Flanged bearing used as the idler on the opposite side of every U-yoke.
IDLER_BEARING = {"bore": 3.0, "od": 10.0, "width": 4.0, "flange_od": 11.5}  # F623ZZ

LIPO_3S = {"size": (138.0, 46.0, 30.0), "mass": 0.39}            # 3S 5000 mAh
COMPUTER = {"name": "Firefly ROC-RK3588S-PC", "size": (93.0, 60.0, 22.0), "mass": 0.12}
LIDAR = {"name": "LDROBOT LD19 / STL-19P", "size": (38.6, 38.6, 34.8), "scan_plane": 26.0, "mass": 0.047}
DEPTH_CAMERA = {"name": "Orbbec Gemini 336 / OAK-D Pro W", "size": (124.0, 29.0, 26.0), "mass": 0.10}
LRF = {"name": "3 km eye-safe LRF 1535 nm", "size": (48.0, 31.0, 21.0), "mass": 0.032,
       "tx_aperture": 8.0, "rx_aperture": 16.0}
THERMAL = {"name": "InfiRay P2 Pro (256x192)", "size": (27.0, 18.0, 10.0), "mass": 0.010}
LOWLIGHT_CAMERA = {"name": "IMX462 starlight + M12 12 mm lens", "size": (32.0, 32.0, 28.0), "mass": 0.035}
# Laser IR illuminator (the device driven by the former Laser_GUI).  Size and
# mass are placeholders: measure your unit and update them.
ILLUMINATOR = {"name": "IR laser illuminator (1.8-71 deg zoom)", "size": (70.0, 42.0, 42.0), "mass": 0.18}
GNSS = {"name": "u-blox M10 + compass", "size": (40.0, 40.0, 12.0), "mass": 0.03}

# ---------------------------------------------------------------------------
# Legs
# ---------------------------------------------------------------------------

LEG_NAMES = ["lf", "lm", "lr", "rf", "rm", "rr"]

# Coxa axis position in base_link (mm) and the leg's neutral yaw (deg).
LEG_MOUNTS = {
    "lf": (120.0, 65.0, 45.0),
    "lm": (0.0, 95.0, 90.0),
    "lr": (-120.0, 65.0, 135.0),
    "rf": (120.0, -65.0, -45.0),
    "rm": (0.0, -95.0, -90.0),
    "rr": (-120.0, -65.0, -135.0),
}

COXA_LENGTH = 42.0    # coxa axis -> femur axis
FEMUR_LENGTH = 88.0   # femur axis -> tibia axis
TIBIA_LENGTH = 135.0  # tibia axis -> foot contact point (TPU foot included)

# Joint limits (deg).  0 = femur horizontal, tibia perpendicular to femur.
JOINT_LIMITS = {
    "coxa": (-40.0, 40.0),   # wider makes neighbouring legs collide
    "femur": (-45.0, 95.0),  # the femur yoke hits the coxa cradle below -45
    "tibia": (-65.0, 75.0),  # the tibia beam hits the femur tube below -65
}

# Neutral stance (mm): foot distance from the coxa axis and body height
# (femur axis plane above the ground).
STANCE_RADIUS = 127.0
BODY_HEIGHT = 100.0
BODY_HEIGHT_RANGE = (55.0, 150.0)
STEP_HEIGHT = 45.0
STEP_HEIGHT_MAX = 80.0

# Servo IDs on the bus: (coxa, femur, tibia) per leg, then head pan / tilt.
SERVO_IDS = {
    "lf": (1, 2, 3), "lm": (4, 5, 6), "lr": (7, 8, 9),
    "rf": (10, 11, 12), "rm": (13, 14, 15), "rr": (16, 17, 18),
    "head_pan": 19, "head_tilt": 20,
}

# ---------------------------------------------------------------------------
# Body
# ---------------------------------------------------------------------------

WALL = 2.4                 # default wall thickness of printed parts
TRAY_THICKNESS = SERVO["case_height"]   # coxa servos sit flush in the tray
TRAY_EDGE_RADIUS = 17.0    # tray contour radius around each coxa axis
YOKE_CLEARANCE = 1.2       # gap between a yoke arm and the part it wraps
DECK_STANDOFF = 26.0       # tray top -> deck bottom
DECK_THICKNESS = 4.0
DECK_SIZE = (205.0, 150.0)

# Head (pan-tilt) on the deck, payload boresighted along +X of head_tilt_link.
HEAD_PAN_XY = (-55.0, 0.0)
HEAD_TOWER_HEIGHT = 40.0           # deck top -> pan horn face
HEAD_TILT_HEIGHT = 50.0            # pan horn face -> tilt axis
HEAD_PAN_LIMITS = (-170.0, 170.0)
HEAD_TILT_LIMITS = (-30.0, 85.0)

# Payload optical centres in head_tilt_link (mm): x forward, y left, z up.
PAYLOAD_FRAMES = {
    "lrf": (20.0, 0.0, 0.0),
    "lowlight_camera": (18.0, -36.0, 0.0),
    "thermal_camera": (28.0, 33.0, 0.0),
    "illuminator": (10.0, 0.0, 38.0),
}
HEAD_EAR_Y = 58.0                  # inner face of the tilt yoke ears

LIDAR_XY = (72.0, 0.0)
LIDAR_RISER = 8.0
DEPTH_CAMERA_X = 152.0             # optical centre in front of the tray
DEPTH_CAMERA_Z = 4.0
DEPTH_CAMERA_PITCH = 15.0          # deg, positive = looking down
GNSS_XY = (-88.0, 52.0)            # low on the deck: below the lidar plane and the head sweep
GNSS_MAST_HEIGHT = 6.0

# Estimated masses (kg) for URDF inertials.
MASS = {
    "base": 1.75,   # tray, deck, coxa servos, battery, computer, sensors
    "coxa": 0.095,
    "femur": 0.10,
    "tibia": 0.045,
    "head_pan": 0.085,
    "head_tilt": 0.32,
}

# Bambu Lab H2D (dual nozzle) usable bed; H2S is bigger, so this is the limit.
PRINT_BED = (300.0, 320.0, 325.0)


# ---------------------------------------------------------------------------
# Derived values
# ---------------------------------------------------------------------------

def deck_z() -> float:
    """Deck top surface height in base_link (mm)."""
    return TRAY_THICKNESS / 2 + DECK_STANDOFF + DECK_THICKNESS


def head_pan_xyz() -> tuple[float, float, float]:
    return (HEAD_PAN_XY[0], HEAD_PAN_XY[1], deck_z() + HEAD_TOWER_HEIGHT)


def lidar_xyz() -> tuple[float, float, float]:
    return (LIDAR_XY[0], LIDAR_XY[1], deck_z() + LIDAR_RISER + LIDAR["scan_plane"])


def neutral_foot(leg: str) -> tuple[float, float, float]:
    """Neutral foot position in base_link (mm)."""
    x, y, yaw = LEG_MOUNTS[leg]
    a = math.radians(yaw)
    return (x + STANCE_RADIUS * math.cos(a), y + STANCE_RADIUS * math.sin(a), -BODY_HEIGHT)


def _m(v):
    if isinstance(v, (list, tuple)):
        return [round(x / 1000.0, 6) for x in v]
    return round(v / 1000.0, 6)


def _rad(v):
    if isinstance(v, (list, tuple)):
        return [round(math.radians(x), 6) for x in v]
    return round(math.radians(v), 6)


def geometry_dict() -> dict:
    """Geometry in SI units (metres, radians, kg)."""
    return {
        "leg_names": LEG_NAMES,
        "mount_x": [_m(LEG_MOUNTS[n][0]) for n in LEG_NAMES],
        "mount_y": [_m(LEG_MOUNTS[n][1]) for n in LEG_NAMES],
        "mount_yaw": [_rad(LEG_MOUNTS[n][2]) for n in LEG_NAMES],
        "coxa_length": _m(COXA_LENGTH),
        "femur_length": _m(FEMUR_LENGTH),
        "tibia_length": _m(TIBIA_LENGTH),
        "coxa_limits": _rad(JOINT_LIMITS["coxa"]),
        "femur_limits": _rad(JOINT_LIMITS["femur"]),
        "tibia_limits": _rad(JOINT_LIMITS["tibia"]),
        "stance_radius": _m(STANCE_RADIUS),
        "body_height": _m(BODY_HEIGHT),
        "body_height_range": _m(BODY_HEIGHT_RANGE),
        "step_height": _m(STEP_HEIGHT),
        "step_height_max": _m(STEP_HEIGHT_MAX),
        "tray_thickness": _m(TRAY_THICKNESS),
        "deck_z": _m(deck_z()),
        "head_pan_xyz": _m(head_pan_xyz()),
        "head_tilt_height": _m(HEAD_TILT_HEIGHT),
        "head_pan_limits": _rad(HEAD_PAN_LIMITS),
        "head_tilt_limits": _rad(HEAD_TILT_LIMITS),
        "payload_lrf_xyz": _m(PAYLOAD_FRAMES["lrf"]),
        "payload_lowlight_xyz": _m(PAYLOAD_FRAMES["lowlight_camera"]),
        "payload_thermal_xyz": _m(PAYLOAD_FRAMES["thermal_camera"]),
        "payload_illuminator_xyz": _m(PAYLOAD_FRAMES["illuminator"]),
        "lidar_xyz": _m(lidar_xyz()),
        "depth_camera_xyz": _m((DEPTH_CAMERA_X, 0.0, DEPTH_CAMERA_Z)),
        "depth_camera_pitch": _rad(DEPTH_CAMERA_PITCH),
        "gnss_xyz": _m((GNSS_XY[0], GNSS_XY[1], deck_z() + GNSS_MAST_HEIGHT)),
        "mass_base": MASS["base"],
        "mass_coxa": MASS["coxa"],
        "mass_femur": MASS["femur"],
        "mass_tibia": MASS["tibia"],
        "mass_head_pan": MASS["head_pan"],
        "mass_head_tilt": MASS["head_tilt"],
        "servo_ids": [i for n in LEG_NAMES for i in SERVO_IDS[n]]
        + [SERVO_IDS["head_pan"], SERVO_IDS["head_tilt"]],
    }


def _yaml_value(v) -> str:
    if isinstance(v, list):
        return "[" + ", ".join(_yaml_value(x) for x in v) + "]"
    if isinstance(v, str):
        return v
    if isinstance(v, float):
        return repr(v)
    return str(v)


def export_geometry(repo_root: Path) -> list[Path]:
    """Write geometry.yaml (ROS parameters) and geometry.json (web sim)."""
    geo = geometry_dict()
    yaml_path = repo_root / "src/hexabot_description/config/geometry.yaml"
    json_path = repo_root / "sim/web/assets/geometry.json"
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        "# GENERATED by cad/params.py - do not edit by hand.",
        "# Units: metres, radians, kilograms.",
        "/**:",
        "  ros__parameters:",
        "    geometry:",
    ]
    lines += [f"      {k}: {_yaml_value(v)}" for k, v in geo.items()]
    yaml_path.write_text("\n".join(lines) + "\n")

    json_path.write_text(json.dumps(geo, indent=1) + "\n")

    hpp_path = repo_root / "src/hexabot_locomotion/include/hexabot_locomotion/geometry_defaults.hpp"
    hpp_path.parent.mkdir(parents=True, exist_ok=True)

    def arr(v):
        return "{" + ", ".join(repr(float(x)) for x in v) + "}"

    def scalar(name, key):
        return f"constexpr double {name} = {float(geo[key])!r};"

    def array(name, key, n):
        return f"constexpr std::array<double, {n}> {name}{arr(geo[key])};"

    hpp = [
        "// GENERATED by cad/params.py - do not edit by hand.",
        "#pragma once",
        "",
        "#include <array>",
        "",
        "namespace hexabot_locomotion::defaults",
        "{",
        array("kMountX", "mount_x", 6),
        array("kMountY", "mount_y", 6),
        array("kMountYaw", "mount_yaw", 6),
        scalar("kCoxaLength", "coxa_length"),
        scalar("kFemurLength", "femur_length"),
        scalar("kTibiaLength", "tibia_length"),
        array("kCoxaLimits", "coxa_limits", 2),
        array("kFemurLimits", "femur_limits", 2),
        array("kTibiaLimits", "tibia_limits", 2),
        scalar("kStanceRadius", "stance_radius"),
        scalar("kBodyHeight", "body_height"),
        array("kBodyHeightRange", "body_height_range", 2),
        scalar("kStepHeight", "step_height"),
        scalar("kStepHeightMax", "step_height_max"),
        "}  // namespace hexabot_locomotion::defaults",
        "",
    ]
    hpp_path.write_text("\n".join(hpp))
    return [yaml_path, json_path, hpp_path]


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[1]
    for p in export_geometry(root):
        print("wrote", p.relative_to(root))
