#!/usr/bin/env python3
"""Record sensor_msgs/Imu to a CSV file, for noise analysis.

Run it INSIDE the container, with the Livox driver publishing and the robot completely still:

    python3 record_imu_csv.py --seconds 300 --label standing
    python3 record_imu_csv.py --seconds 120 --label lying

Output (default folder: /root/robinlab_ws/data):
    imu_<label>_<YYYYmmdd_HHMMSS>.csv

Columns: stamp_ns (header.stamp), recv_ns (wall clock when the message arrived),
         ax, ay, az, gx, gy, gz     (exactly as published: no unit conversion)
The first line is a '#' comment with the topic and frame_id.

Docker note: inside the container you are root, so a file written to a bind-mounted folder would
be owned by root on the host. This script gives the file the same owner as the output folder and
makes it group/other readable, so you can open and attach it from the host without sudo.
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class ImuCsvRecorder(Node):
    def __init__(self, topic, path, seconds):
        super().__init__("imu_csv_recorder")
        self.topic = topic
        self.path = path
        self.seconds = seconds
        self.f = None
        self.rows = 0
        self.t_start = None          # wall-clock time of the first message
        self.first_stamp = None
        self.last_stamp = None
        self.max_gap_ns = 0
        self.big_gaps = 0            # gaps longer than 3x the nominal period (~15 ms at 200 Hz)
        self.last_report = 0.0
        self.done = False

        # Large queue + best effort: Python callbacks must not drop samples at 200 Hz.
        qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Imu, topic, self.cb, qos)
        self.get_logger().info(f"Waiting for messages on {topic} ...")

    def cb(self, m):
        if self.done:
            return
        now = time.time()
        stamp_ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec

        if self.f is None:  # first message: open the file now that frame_id is known
            self.f = open(self.path, "w", buffering=1 << 16)
            self.f.write(f"# topic={self.topic} frame_id={m.header.frame_id} "
                         f"started={time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.f.write("stamp_ns,recv_ns,ax,ay,az,gx,gy,gz\n")
            self.t_start = now
            self.first_stamp = stamp_ns
            self.get_logger().info(f"[SAMPLING IMU] : topic detected (frame_id: {m.header.frame_id}); "
                                   f"recording {self.seconds:.0f} s to {self.path}")

        if self.last_stamp is not None:
            gap = stamp_ns - self.last_stamp
            self.max_gap_ns = max(self.max_gap_ns, gap)
            if gap > 15_000_000:
                self.big_gaps += 1
        self.last_stamp = stamp_ns

        a, g = m.linear_acceleration, m.angular_velocity
        self.f.write(f"{stamp_ns},{time.time_ns()},{a.x!r},{a.y!r},{a.z!r},{g.x!r},{g.y!r},{g.z!r}\n")
        self.rows += 1

        elapsed = now - self.t_start
        if elapsed - self.last_report >= 10.0:
            self.last_report = elapsed
            self.f.flush()
            self.get_logger().info(f"  {elapsed:5.0f}/{self.seconds:.0f} s   rows={self.rows}")
        if elapsed >= self.seconds:
            self.done = True

    def close(self):
        if self.f is not None:
            self.f.flush()
            self.f.close()
            fix_permissions(self.path)


def fix_permissions(path):
    """Give the file the owner of its folder (host user) and make it readable by others."""
    try:
        st = os.stat(os.path.dirname(os.path.abspath(path)))
        os.chown(path, st.st_uid, st.st_gid)
    except Exception as e:  # not root, or filesystem without chown: not fatal
        print(f"(could not chown: {e})", file=sys.stderr)
    try:
        os.chmod(path, 0o664)
    except Exception as e:
        print(f"(could not chmod: {e})", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/livox/imu")
    ap.add_argument("--seconds", type=float, default=300.0, help="recording length (default 300 s)")
    ap.add_argument("--label", default="static", help="e.g. standing, lying, motors_on")
    ap.add_argument("--outdir", default="/root/robinlab_ws/data")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    path = os.path.join(args.outdir, f"imu_{args.label}_{time.strftime('%Y%m%d_%H%M%S')}.csv")

    rclpy.init()
    node = ImuCsvRecorder(args.topic, path, args.seconds)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nInterrupted: saving what was recorded.")
    finally:
        node.close()
        if node.rows > 1 and node.last_stamp > node.first_stamp:
            dur = (node.last_stamp - node.first_stamp) / 1e9
            print(f"\nSaved {node.rows} rows in {dur:.1f} s  (mean rate {node.rows / dur:.1f} Hz)")
            print(f"Largest gap between messages: {node.max_gap_ns / 1e6:.1f} ms;"
                  f" gaps > 15 ms: {node.big_gaps}")
            print(f"File: {path}")
        else:
            print("No data recorded. Check the topic name (ros2 topic list) and that the driver is running.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
