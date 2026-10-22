#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Draw the lifting column collision zone in RViz as markers.

Two boxes, both expressed in TF frame link_base so they ride the lift height:

  - green : telescoping column collision envelope, 212 x 212 x 650 mm
  - red   : xArm base footprint (base.stl bbox), 155 x 126 x 155 mm,
            drawn only so the column overhang past the arm base is visible

Numbers come from the real meshes (see tasks/doc/measurements.md M1 for the
stroke calibration, and the 2026-09-28 mesh analysis for the cross-section).
Visualization only -- nothing here changes the URDF, the collision model or
MoveIt planning.
"""

import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

MARKER_TOPIC = "lift_zone/marker"
FRAME_ID = "link_base"
PUBLISH_HZ = 2.0

# Cross-section measured from 2431_box_link.STL (stable column band
# 0.350~0.575 m): 0.212 x 0.212 m, centred 0.7 mm behind the arm base axis.
# Top sinks 5 mm into the mounting flange; 650 mm length keeps the bottom
# covered for the full 0~600 mm stroke.
COLUMN = {
    "ns": "lift_collision_envelope",
    "id": 0,
    "size": (0.212, 0.212, 0.650),
    "pos": (0.0007, 0.0, -0.320),
    "rgba": (0.10, 0.90, 0.30, 0.35),
}
ARM_BASE = {
    "ns": "arm_base_footprint",
    "id": 1,
    "size": (0.155, 0.126, 0.155),
    "pos": (-0.0147, 0.0, 0.0774),
    "rgba": (0.95, 0.20, 0.20, 0.35),
}


def make_box(spec):
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.ns = spec["ns"]
    marker.id = spec["id"]
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = Pose()
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec["pos"]
    marker.pose.orientation.w = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = spec["size"]
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec["rgba"]
    marker.lifetime = rospy.Duration(0)
    return marker


class LiftZoneViz(object):
    def __init__(self):
        self.pub = rospy.Publisher(MARKER_TOPIC, Marker, queue_size=4, latch=True)
        self.markers = [make_box(COLUMN), make_box(ARM_BASE)]
        rospy.loginfo("Lift zone markers on /%s (frame %s)", MARKER_TOPIC, FRAME_ID)

    def spin(self):
        rate = rospy.Rate(PUBLISH_HZ)
        while not rospy.is_shutdown():
            for marker in self.markers:
                marker.header.stamp = rospy.Time.now()
                self.pub.publish(marker)
            rate.sleep()

    def clear(self):
        # Remove the latched markers so a stopped node does not leave stale
        # boxes in RViz.
        for marker in self.markers:
            marker.action = Marker.DELETE
            marker.header.stamp = rospy.Time.now()
            self.pub.publish(marker)
        rospy.sleep(0.2)


def main():
    rospy.init_node("lift_zone_viz")
    viz = LiftZoneViz()
    rospy.on_shutdown(viz.clear)
    viz.spin()


if __name__ == "__main__":
    main()
