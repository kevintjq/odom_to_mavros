#!/usr/bin/env python3

import math
import os
import sys
import threading
import time
from collections import namedtuple

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry


FIELDS = ("X [m]", "Y [m]", "Z [m]", "Roll [deg]", "Pitch [deg]", "Yaw [deg]")
COLUMNS = ("LIO", "VISION OUT", "PX4")
POSE_RX_TIMEOUT = 1.0
STATE_RX_TIMEOUT = 3.0
Sample = namedtuple("Sample", "received_at frame values")


def pose_values(pose):
    position = (pose.position.x, pose.position.y, pose.position.z)
    q = pose.orientation
    quaternion = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in position + quaternion):
        return None
    norm = math.hypot(*quaternion)
    if not math.isfinite(norm) or norm < 1e-6:
        return None

    x, y, z, w = (value / norm for value in quaternion)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return position + tuple(math.degrees(angle) for angle in (roll, pitch, yaw))


class PoseComparison:
    def __init__(self):
        self.lock = threading.Lock()
        self.samples = [None] * len(COLUMNS)
        self.state = None
        bridge_input = rospy.get_param("odom_to_mavros/input_topic", "/Odometry")
        bridge_output = rospy.get_param("odom_to_mavros/output_topic", "/mavros/vision_pose/pose")
        self.subscribers = [
            rospy.Subscriber(
                rospy.get_param("~input_topic", bridge_input),
                Odometry, self.receive_lio, queue_size=1, tcp_nodelay=True,
            ),
            rospy.Subscriber(
                rospy.get_param("~output_topic", bridge_output),
                PoseStamped, self.receive_pose, callback_args=1, queue_size=1, tcp_nodelay=True,
            ),
            rospy.Subscriber(
                rospy.get_param("~px4_pose_topic", "/mavros/local_position/pose"),
                PoseStamped, self.receive_pose, callback_args=2, queue_size=1, tcp_nodelay=True,
            ),
            rospy.Subscriber(
                rospy.get_param("~state_topic", "/mavros/state"),
                State, self.receive_state, queue_size=1,
            ),
        ]

    def receive_lio(self, msg):
        self.save_pose(0, msg.header.frame_id, msg.pose.pose)

    def receive_pose(self, msg, column):
        self.save_pose(column, msg.header.frame_id, msg.pose)

    def save_pose(self, column, frame, pose):
        received_at = time.monotonic()
        sample = Sample(received_at, frame, pose_values(pose))
        with self.lock:
            self.samples[column] = sample

    def receive_state(self, msg):
        with self.lock:
            self.state = (time.monotonic(), msg)

    def snapshot(self):
        with self.lock:
            return tuple(self.samples), self.state


def terminal_label(value, width):
    value = "".join(character if character.isascii() and character.isprintable() else "?"
                    for character in value)
    return value if len(value) <= width else value[:width - 3] + "..."


def format_measurement(value):
    return f"{value:+.3f}" if abs(value) < 1e8 else f"{value:+.3e}"


def render(samples, state, now, color):
    def row(label, values):
        return f"{label:<12}" + " ".join(f"{value:>14}" for value in values)

    def paint(value, ansi):
        return f"\x1b[{ansi}m{value}\x1b[0m" if color else value

    ages = [None if sample is None else now - sample.received_at for sample in samples]
    poses = []
    statuses = []
    for sample, age in zip(samples, ages):
        values = None
        if sample is None:
            status = ("WAIT", "33")
        elif age >= POSE_RX_TIMEOUT:
            status = ("STALE", "31")
        elif sample.values is None:
            status = ("INVALID", "31")
        else:
            status = ("LIVE", "32")
            values = sample.values
        statuses.append(status)
        poses.append(values)

    mode, link, armed = "--", "WAIT", "--"
    if state is not None:
        received_at, msg = state
        if now - received_at >= STATE_RX_TIMEOUT:
            link = "STALE"
        elif msg.connected:
            mode = terminal_label(msg.mode, 24) or "--"
            link = "CONNECTED"
            armed = "YES" if msg.armed else "NO"
        else:
            link = "DISCONNECTED"

    divider = "-" * len(row("", COLUMNS))
    lines = [
        paint("LIO / MAVROS POSE COMPARISON", "1;36"),
        divider,
        paint(row("", COLUMNS), "1;36"),
        divider,
    ]
    for index, field in enumerate(FIELDS):
        lines.append(row(field, ["--" if values is None else format_measurement(values[index])
                                 for values in poses]))
    lines.extend([
        divider,
        "RX status   " + " ".join(paint(f"{status:>14}", ansi) for status, ansi in statuses),
        row("RX age", ["--" if age is None else f"{age:.2f} s" for age in ages]),
        row("Frame", [(terminal_label(sample.frame, 14) or "--") if sample else "--"
                      for sample in samples]),
        divider,
        f"PX4 {mode}  |  Link {link}  |  Armed {armed}",
        "Compare axes only when frames and origins are aligned.",
    ])
    return lines


def main():
    rospy.init_node("pose_compare", anonymous=True)
    comparison = PoseComparison()
    interactive = sys.stdout.isatty() and os.environ.get("TERM") != "dumb"
    color = interactive and "NO_COLOR" not in os.environ
    try:
        if interactive:
            sys.stdout.write("\x1b[?1049h\x1b[?25l")
        while not rospy.is_shutdown():
            samples, state = comparison.snapshot()
            lines = render(samples, state, time.monotonic(), color)
            if interactive:
                sys.stdout.write("\x1b[H" + "\x1b[K\n".join(lines) + "\x1b[K\x1b[J")
            else:
                sys.stdout.write("\n".join(lines) + "\n\n")
            sys.stdout.flush()
            time.sleep(0.2 if interactive else 1.0)
    finally:
        if interactive:
            sys.stdout.write("\x1b[?25h\x1b[?1049l")
            sys.stdout.flush()


if __name__ == "__main__":
    main()
