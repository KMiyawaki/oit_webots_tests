#!/usr/bin/env python
# -*- coding: utf_8 -*-
import os
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Pose2D, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry


class OdomToBaseLink(object):
    def __init__(self, topic_odom):
        self.broad_caster = tf.TransformBroadcaster()
        self.sub_odom = rospy.Subscriber(
            topic_odom, Odometry, self.odom_callback)

    def spin(self):
        pass

    def odom_callback(self, odom):
        current_time = rospy.Time.now()
        q = odom.pose.pose.orientation
        translation = (odom.pose.pose.position.x, odom.pose.pose.position.y, 0)
        rotation = (q.x, q.y, q.z, q.w)
        self.broad_caster.sendTransform(
            translation, rotation, current_time, odom.child_frame_id, odom.header.frame_id)


def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)

    process_rate = rospy.get_param("~process_rate", 20.0)
    rate = rospy.Rate(process_rate)
    node = OdomToBaseLink("/odom")
    rospy.loginfo("Start %s with process rate %f Hz",
                  node_name, process_rate)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()
    rospy.loginfo("Exiting %s", node_name)


if __name__ == '__main__':
    main()
