#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Draw the provisional safety plane above the deck in RViz.

One translucent yellow box, expressed in TF frame deck_top (the measured top
surface of the 2431 mounting box) so it rides any future refinement of the
208 mm offset:

  - yellow: provisional safety volume. Bottom face at deck_top + 0.028 m,
    top face at deck_top + 1.298 m (i.e. 1.270 m tall), covering the chassis
    footprint in front of the lift column (0.459 x 0.50 m). Provisional cut:
    the strip at x <= 106 mm (link_base_lift0 frame, ~= the column front edge)
    is removed; y keeps the full +/-0.25 m chassis coverage
    (steering joints at x +0.25 / -0.23, y +/-0.19..0.206).

This file is the SINGLE SOURCE OF TRUTH for the box numbers: on startup they
are written to the parameter server as /safety_plane/box, and agx_plane_target
(tasks layer) reads that parameter for its MoveIt keep-in constraint, so the
drawn box and the enforced box cannot drift apart. The layering direction
tasks -> start is the allowed one (the reverse is forbidden by AGENTS.md), and
a parameter is the sanctioned loose string interface, so the numbers flow this
way rather than start importing tasks code.

Bottom face history: 0.098 m (kept from the original 4 mm slab) -> 0.028 m on
2026-09-29, lowered by 70 mm on request so the gripper may work closer to the
deck surface. The top face stayed at 1.298 m, i.e. the box got taller instead
of being translated. Scope is provisional until the actual safety task is
defined.

Visualization only -- this file does not change the collision model; the
planning effect comes from the consumer of /safety_plane/box.
"""

import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

MARKER_TOPIC = "safety_plane/marker"
FRAME_ID = "deck_top"
PUBLISH_HZ = 2.0

# Where the box numbers are published for other packages to consume.
PLANE_PARAM = "/safety_plane/box"

# The box: bottom face at deck_top + 0.028 m, top face at deck_top + 1.298 m
# (bottom 0.028 + height 1.270). Provisional cut: the strip at x <= 106 mm
# (link_base_lift0 frame) is removed, so it only covers the deck area in front
# of the lift column (106 mm ~= column cross-section front edge, 212/2 + 0.7).
# Old extent in deck_top x was [-0.185, 0.565]; the kept part is [0.106, 0.565]:
# width 0.459, centre 0.3355. y keeps the full +/-0.25 (chassis cover).
# Remember: bottom = pos.z - size.z/2. To lower the bottom by d while keeping
# the top face where it is: size.z += d and pos.z -= d/2.
PLANE = {
    "ns": "safety_plane",
    "id": 0,
    "size": (0.459, 0.50, 1.270),
    "pos": (0.3355, 0.0, 0.663),
    "rgba": (0.95, 0.85, 0.20, 0.25),
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


def publish_box_param():
    """Publish the box so consumers (agx_plane_target's MoveIt keep-in
    constraint) use exactly these numbers instead of a second hard-coded copy."""
    box = {"center": list(PLANE["pos"]), "size": list(PLANE["size"]),
           "frame_id": FRAME_ID, "ns": PLANE["ns"]}
    rospy.set_param(PLANE_PARAM, box)
    bottom = PLANE["pos"][2] - PLANE["size"][2] / 2.0
    top = PLANE["pos"][2] + PLANE["size"][2] / 2.0
    rospy.loginfo(
        "safety box x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] (frame %s) "
        "published to %s", box["center"][0] - box["size"][0] / 2.0,
        box["center"][0] + box["size"][0] / 2.0,
        box["center"][1] - box["size"][1] / 2.0,
        box["center"][1] + box["size"][1] / 2.0, bottom, top, FRAME_ID,
        PLANE_PARAM)
    return box


class SafetyPlaneViz(object):
    def __init__(self):
        self.pub = rospy.Publisher(MARKER_TOPIC, Marker, queue_size=4, latch=True)
        self.markers = [make_box(PLANE)]
        publish_box_param()
        rospy.loginfo("Safety plane marker on /%s (frame %s)", MARKER_TOPIC, FRAME_ID)

    def spin(self):
        rate = rospy.Rate(PUBLISH_HZ)
        while not rospy.is_shutdown():
            for marker in self.markers:
                marker.header.stamp = rospy.Time.now()
                self.pub.publish(marker)
            rate.sleep()

    def clear(self):
        # Remove the latched marker so a stopped node does not leave a stale
        # plane in RViz.
        for marker in self.markers:
            marker.action = Marker.DELETE
            marker.header.stamp = rospy.Time.now()
            self.pub.publish(marker)
        rospy.sleep(0.2)


def main():
    rospy.init_node("safety_plane_viz")
    viz = SafetyPlaneViz()
    rospy.on_shutdown(viz.clear)
    viz.spin()


if __name__ == "__main__":
    main()
