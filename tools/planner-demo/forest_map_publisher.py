#!/usr/bin/env python3
"""Publish the official-demo obstacle layout used by Gazebo."""

import math

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from forest_layout import PILLARS, RESOLUTION, RINGS, ring_centerline


PLANNER_MARGIN = 0.0
FLOOR_RESOLUTION = 0.2


def obstacle_points():
    points = []
    for cx, cy, radius, height in PILLARS:
        inflated_radius = radius + PLANNER_MARGIN
        cells = math.ceil(inflated_radius / RESOLUTION)
        for xi in range(-cells, cells + 1):
            for yi in range(-cells, cells + 1):
                dx = xi * RESOLUTION
                dy = yi * RESOLUTION
                if math.hypot(dx, dy) > inflated_radius:
                    continue
                for zi in range(math.ceil((height + PLANNER_MARGIN) / RESOLUTION) + 1):
                    points.append((cx + dx, cy + dy, zi * RESOLUTION))
    for ring in RINGS:
        points.extend(ring_centerline(ring))
    for xi in range(round(70.0 / FLOOR_RESOLUTION) + 1):
        x = -35.0 + xi * FLOOR_RESOLUTION
        for yi in range(round(30.0 / FLOOR_RESOLUTION) + 1):
            points.append((x, -15.0 + yi * FLOOR_RESOLUTION, 0.0))
    return points


def main():
    rospy.init_node("official_demo_obstacle_map")
    publisher = rospy.Publisher("/map_generator/global_cloud", PointCloud2,
                                queue_size=1, latch=True)
    points = obstacle_points()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        header = Header(stamp=rospy.Time.now(), frame_id="world")
        publisher.publish(point_cloud2.create_cloud_xyz32(header, points))
        rospy.loginfo_throttle(
            10.0, "publishing %d points for %d pillars and %d rings",
            len(points), len(PILLARS), len(RINGS))
        rate.sleep()


if __name__ == "__main__":
    main()
