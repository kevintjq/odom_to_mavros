#!/usr/bin/env python3
"""Convert LiDAR_IMU_Init calibration output into YAML-ready extrinsics.

The LiDAR_IMU_Init result usually prints:

  Rotation LiDAR to IMU (degree) = roll pitch yaw
  Translation LiDAR to IMU (meter) = x y z

Edit the config block below, then run:

  python tools/convert_lidar_imu_init.py

For this project, if your "body" frame is the IMU/body frame used by
LiDAR_IMU_Init, these values can be copied directly into:

  config/odom_to_mavros.yaml -> extrinsic.translation / extrinsic.rotation

Use --inverse if you need the opposite transform.
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple


# ===== User config =====
# Path to LiDAR_IMU_Init/result/Initialization_result.txt
RESULT_FILE = r"C:\path\to\LiDAR_IMU_Init\result\Initialization_result.txt"

# False: output LiDAR->IMU, usually what this project wants.
# True: output IMU->LiDAR.
OUTPUT_INVERSE = False

# Output rotation unit: "deg" or "rad".
OUTPUT_UNIT = "deg"

# Output format: "yaml" or "text".
OUTPUT_FORMAT = "yaml"
# =======================


@dataclass
class Extrinsic:
    roll: float
    pitch: float
    yaw: float
    x: float
    y: float
    z: float
    input_unit: str = "deg"


ROT_PATTERNS = (
    re.compile(
        r"Rotation\s+LiDAR\s+to\s+IMU.*?=\s*"
        r"([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s+"
        r"([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s+"
        r"([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)"
        r"(?:\s*(deg|degree|degrees|rad|radian|radians))?",
        re.IGNORECASE,
    ),
)

TRANS_PATTERNS = (
    re.compile(
        r"Translation\s+LiDAR\s+to\s+IMU.*?=\s*"
        r"([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s+"
        r"([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s+"
        r"([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)",
        re.IGNORECASE,
    ),
)


def last_translation(text: str) -> Tuple[float, float, float]:
    matches = []
    for pattern in TRANS_PATTERNS:
        matches.extend(pattern.finditer(text))
    if not matches:
        raise ValueError("Could not find LiDAR_IMU_Init extrinsic lines in the result file.")
    match = max(matches, key=lambda m: m.start())
    return tuple(float(match.group(i)) for i in range(1, 4))  # type: ignore[return-value]


def parse_result(text: str) -> Extrinsic:
    matches = []
    for pattern in ROT_PATTERNS:
        matches.extend(pattern.finditer(text))
    if not matches:
        raise ValueError("Could not find LiDAR_IMU_Init rotation lines in the result file.")

    match = max(matches, key=lambda m: m.start())
    roll = float(match.group(1))
    pitch = float(match.group(2))
    yaw = float(match.group(3))
    unit = (match.group(4) or "deg").lower()
    if unit in {"deg", "degree", "degrees"}:
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        unit = "deg"
    else:
        unit = "rad"

    x, y, z = last_translation(text)
    return Extrinsic(roll=roll, pitch=pitch, yaw=yaw, x=x, y=y, z=z, input_unit=unit)


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def matrix_to_rpy(r: list[list[float]]) -> tuple[float, float, float]:
    pitch = math.atan2(-r[2][0], math.sqrt(r[0][0] ** 2 + r[1][0] ** 2))
    if abs(abs(pitch) - math.pi / 2) < 1e-9:
        roll = 0.0
        yaw = math.atan2(-r[0][1], r[1][1])
    else:
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    return roll, pitch, yaw


def mat_vec_mul(a: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        sum(a[0][k] * v[k] for k in range(3)),
        sum(a[1][k] * v[k] for k in range(3)),
        sum(a[2][k] * v[k] for k in range(3)),
    )


def transpose(a: list[list[float]]) -> list[list[float]]:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def invert_extrinsic(ex: Extrinsic) -> Extrinsic:
    r = rpy_to_matrix(ex.roll, ex.pitch, ex.yaw)
    r_inv = transpose(r)
    t_inv = tuple(-v for v in mat_vec_mul(r_inv, (ex.x, ex.y, ex.z)))
    roll, pitch, yaw = matrix_to_rpy(r_inv)
    return Extrinsic(roll=roll, pitch=pitch, yaw=yaw, x=t_inv[0], y=t_inv[1], z=t_inv[2])


def format_value(value: float, unit: str) -> float:
    if unit == "deg":
        return math.degrees(value)
    return value


def print_yaml(ex: Extrinsic, unit: str) -> None:
    print("extrinsic:")
    print("  translation:")
    print(f"    x: {ex.x:.6f}")
    print(f"    y: {ex.y:.6f}")
    print(f"    z: {ex.z:.6f}")
    print("  rotation:")
    print(f"    unit: {unit}")
    print(f"    roll: {format_value(ex.roll, unit):.6f}")
    print(f"    pitch: {format_value(ex.pitch, unit):.6f}")
    print(f"    yaw: {format_value(ex.yaw, unit):.6f}")


def print_text(ex: Extrinsic, unit: str) -> None:
    print(f"translation: [{ex.x:.6f}, {ex.y:.6f}, {ex.z:.6f}]")
    if unit == "deg":
        print(
            "rotation(deg): "
            f"[{math.degrees(ex.roll):.6f}, {math.degrees(ex.pitch):.6f}, {math.degrees(ex.yaw):.6f}]"
        )
    else:
        print(
            "rotation(rad): "
            f"[{ex.roll:.6f}, {ex.pitch:.6f}, {ex.yaw:.6f}]"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert LiDAR_IMU_Init Initialization_result.txt into YAML-ready extrinsics."
    )
    parser.add_argument(
        "result_file",
        nargs="?",
        type=Path,
        default=Path(RESULT_FILE),
        help="Path to Initialization_result.txt. Defaults to RESULT_FILE in this script.",
    )
    parser.add_argument(
        "--inverse",
        action="store_true",
        default=OUTPUT_INVERSE,
        help="Output the inverse transform instead of the direct LiDAR->IMU result.",
    )
    parser.add_argument(
        "--unit",
        choices=("deg", "rad"),
        default=OUTPUT_UNIT,
        help="Output rotation unit for the YAML snippet.",
    )
    parser.add_argument(
        "--format",
        choices=("yaml", "text"),
        default=OUTPUT_FORMAT,
        help="Output format.",
    )
    args = parser.parse_args()

    if not args.result_file.is_file():
        raise FileNotFoundError(
            f"Result file not found: {args.result_file}\n"
            "Edit RESULT_FILE at the top of this script, or pass the path as an argument."
        )

    text = args.result_file.read_text(encoding="utf-8", errors="ignore")
    extrinsic = parse_result(text)
    if args.inverse:
        extrinsic = invert_extrinsic(extrinsic)

    if args.format == "yaml":
        print_yaml(extrinsic, args.unit)
    else:
        print_text(extrinsic, args.unit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
