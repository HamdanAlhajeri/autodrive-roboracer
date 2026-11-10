"""Simulated F1TENTH car for testing the Jetson stack without hardware.

Publishes /scan (UST-10LX-like), /odom with drifting wheel odometry, TF odom->base_link
and static base_link->laser. Modes:
  scripted  drives the centreline itself (stands in for a manual teleop lap)
  drive     obeys AckermannDriveStamped on /drive (closed loop with the supervisor)
Ground truth goes to --truth CSV in the frame of the start pose (= the SLAM map frame
when mapping starts there), as t_s,x,y,yaw with ROS time stamps.
"""

import argparse
import math
import sys

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from shapely.geometry import LineString, Polygon, box
from tf2_msgs.msg import TFMessage

WHEELBASE = 0.33
LASER_X = 0.25
TRACK_WIDTH = 1.8


def centreline():
    """Asymmetric closed loop (~32 m): long straight, hairpin, chicane, sweeper."""
    pts = [(0, 0), (6, 0), (9, 0.5), (10.5, 2.5), (10, 4.5), (8, 5.2), (6.5, 4.2),
           (5, 5.5), (3, 6.5), (0.5, 6.0), (-1.5, 4.0), (-2.0, 2.0), (-1.2, 0.6)]
    line = LineString(pts + [pts[0]])
    # Resample densely and smooth with a circular moving average.
    n = 600
    s = np.linspace(0, line.length, n, endpoint=False)
    xy = np.array([line.interpolate(d).coords[0] for d in s])
    k = 25
    kernel = np.ones(k) / k
    pad = np.vstack([xy[-k:], xy, xy[:k]])
    smooth = np.column_stack([np.convolve(pad[:, i], kernel, "same") for i in (0, 1)])
    return smooth[k:-k]


def walls(path):
    ring = Polygon(path)
    shapes = []
    for poly in (ring.buffer(TRACK_WIDTH / 2, resolution=8),
                 ring.buffer(-TRACK_WIDTH / 2, resolution=8)):
        for part in getattr(poly, "geoms", [poly]):
            shapes.append(part.exterior)
            shapes.extend(part.interiors)
    # A few boxes against the outer wall give SLAM distinctive features.
    for (x, y) in ((3.0, -0.95), (8.5, -0.6), (-2.85, 3.0), (2.0, 7.2)):
        shapes.append(box(x - 0.2, y - 0.2, x + 0.2, y + 0.2).exterior)
    segs = []
    for shape in shapes:
        c = np.asarray(shape.coords)
        segs.append(np.hstack([c[:-1], c[1:]]))
    return np.vstack(segs)


class Car(Node):
    def __init__(self, args):
        super().__init__("fake_car")
        self.args = args
        self.path = centreline()
        self.segs = walls(self.path)
        d = self.path[1] - self.path[0]
        self.start = np.array([*self.path[0], math.atan2(d[1], d[0])])
        self.truth = self.start.copy()          # world x, y, yaw of base_link
        self.odom = np.zeros(3)                  # odom frame starts at base_link
        self.v = 0.0
        self.delta = 0.0
        self.cmd = (0.0, 0.0)
        self.idx = 0
        self.travel = 0.0
        self.rng = np.random.default_rng(args.seed)
        n = 1081
        self.angles = np.linspace(-0.75 * math.pi, 0.75 * math.pi, n)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 50)
        static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self.publish_static()
        if args.mode == "drive":
            self.create_subscription(AckermannDriveStamped, "/drive", self.on_drive, 10)
        self.truth_file = open(args.truth, "w") if args.truth else None
        if self.truth_file:
            self.truth_file.write("t_s,x,y,yaw\n")
        self.dt = 0.01
        self.create_timer(0.01, self.step)
        self.create_timer(1 / 40, self.publish_scan)
        self.create_timer(1 / 50, self.publish_odom)
        self.lap_length = LineString(np.vstack([self.path, self.path[:1]])).length

    def stamp(self):
        return self.get_clock().now().to_msg()

    def publish_static(self):
        t = TransformStamped()
        t.header.stamp = self.stamp()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser"
        t.transform.translation.x = LASER_X
        t.transform.rotation.w = 1.0
        self.static_pub.publish(TFMessage(transforms=[t]))

    def on_drive(self, msg):
        self.cmd = (float(msg.drive.speed), float(msg.drive.steering_angle))

    def scripted_command(self):
        """Pure pursuit on the centreline at a fixed speed, like a careful driver."""
        x, y, yaw = self.truth
        d = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)
        window = (self.idx + np.arange(-20, 60)) % len(self.path)
        self.idx = int(window[np.argmin(d[window])])
        look = 1.0
        j = self.idx
        while np.hypot(*(self.path[j] - (x, y))) < look:
            j = (j + 1) % len(self.path)
        gx, gy = self.path[j]
        alpha = math.atan2(gy - y, gx - x) - yaw
        return self.args.speed, math.atan2(2 * WHEELBASE * math.sin(alpha), look)

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.dt = min(0.05, now - self.last) if hasattr(self, "last") else 0.01
        self.last = now
        speed, steer = self.scripted_command() if self.args.mode == "scripted" else self.cmd
        steer = max(-0.4, min(0.4, steer))
        # First-order motor and servo response.
        self.v += (speed - self.v) * min(1.0, self.dt / 0.15)
        self.delta += (steer - self.delta) * min(1.0, self.dt / 0.08)
        w = self.v * math.tan(self.delta) / WHEELBASE
        x, y, yaw = self.truth
        self.truth = np.array([x + self.v * math.cos(yaw) * self.dt,
                               y + self.v * math.sin(yaw) * self.dt,
                               yaw + w * self.dt])
        self.travel += abs(self.v) * self.dt
        # Wheel odometry: 2 % speed scale error and a small yaw-rate bias.
        vo, wo = self.v * 1.02, w + 0.004
        ox, oy, oyaw = self.odom
        self.odom = np.array([ox + vo * math.cos(oyaw) * self.dt,
                              oy + vo * math.sin(oyaw) * self.dt, oyaw + wo * self.dt])
        self.odom_twist = (vo, wo)
        if self.truth_file:
            sx, sy, syaw = self.start
            dx, dy = self.truth[0] - sx, self.truth[1] - sy
            c, s = math.cos(-syaw), math.sin(-syaw)
            t = self.get_clock().now().nanoseconds * 1e-9
            yaw_rel = math.atan2(math.sin(self.truth[2] - syaw), math.cos(self.truth[2] - syaw))
            self.truth_file.write(f"{t:.4f},{c*dx - s*dy:.4f},{s*dx + c*dy:.4f},{yaw_rel:.5f}\n")
        if self.args.laps and self.travel >= self.args.laps * self.lap_length:
            if self.args.mode == "scripted":
                self.args.speed = 0.0
                if abs(self.v) < 0.01:
                    self.get_logger().info("scripted laps done")
                    raise SystemExit(0)

    def publish_scan(self):
        x, y, yaw = self.truth
        lx, ly = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
        a = self.angles + yaw
        dx, dy = np.cos(a)[:, None], np.sin(a)[:, None]
        x1, y1, x2, y2 = (self.segs[:, i][None, :] for i in range(4))
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x1 - lx) * ey - (y1 - ly) * ex) / den
            u = ((x1 - lx) * dy - (y1 - ly) * dx) / den
        t = np.where((den != 0) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf)
        r = t.min(axis=1)
        r = r + self.rng.normal(0, 0.01, r.shape)
        r[(r > 10.0) | ~np.isfinite(r)] = np.inf
        msg = LaserScan()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = "laser"
        msg.angle_min, msg.angle_max = float(self.angles[0]), float(self.angles[-1])
        msg.angle_increment = float(self.angles[1] - self.angles[0])
        msg.scan_time = 1 / 40
        msg.time_increment = msg.scan_time / len(self.angles)
        msg.range_min, msg.range_max = 0.06, 10.0
        msg.ranges = r.astype(np.float32).tolist()
        self.scan_pub.publish(msg)

    def publish_odom(self):
        ox, oy, oyaw = self.odom
        vo, wo = getattr(self, "odom_twist", (0.0, 0.0))
        now = self.stamp()
        q = (0.0, 0.0, math.sin(oyaw / 2), math.cos(oyaw / 2))
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(ox), float(oy)
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        odom.twist.twist.linear.x = float(vo)
        odom.twist.twist.angular.z = float(wo)
        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        t.transform.translation.x, t.transform.translation.y = float(ox), float(oy)
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.tf_pub.publish(TFMessage(transforms=[t]))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=("scripted", "drive"), default="scripted")
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--laps", type=float, default=0.0, help="Stop after this many laps")
    p.add_argument("--truth")
    p.add_argument("--seed", type=int, default=1)
    args = p.parse_args()
    rclpy.init()
    car = Car(args)
    try:
        rclpy.spin(car)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if car.truth_file:
            car.truth_file.close()
        car.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
