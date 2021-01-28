#!/usr/bin/env python
# -*- coding: utf_8 -*-
import os
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class WebotsVelocityNode(object):
    def __init__(self, tread, wheel_radius, topic_webots_wheel_vels="/webots/wheel_vels", topic_cmd_vel="/cmd_vel"):
        self.pub_webots_wheel_vels = rospy.Publisher(
            topic_webots_wheel_vels, Float32MultiArray, queue_size=1)
        self.sub_cmd_vel = rospy.Subscriber(
            topic_cmd_vel, Twist, self.twist_callback)
        self.tread_2 = tread / 2
        self.wheel_radius = wheel_radius

    def twist_callback(self, message):
        vels = Float32MultiArray()
        vels.data = self.twist_to_vel_wheels(message)
        self.pub_webots_wheel_vels.publish(vels)

    def twist_to_vel_wheels(self, twist):
        vel_wheels = [0, 0]  # left, right
        lin = twist.linear.x
        ang = twist.angular.z
        # http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.html
        vel_wheels[0] = (lin - ang * self.tread_2) / self.wheel_radius  # rad/s
        vel_wheels[1] = (lin + ang * self.tread_2) / self.wheel_radius  # rad/s
        return vel_wheels


def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)

    # https://www.cyberbotics.com/doc/guide/pioneer-3dx?version=master
    process_rate = rospy.get_param("~process_rate", 10.0)
    tread = rospy.get_param("~tread", 0.381)
    wheel_radius = rospy.get_param("~wheel_radius", 0.195/2)
    rate = rospy.Rate(process_rate)
    rospy.loginfo("Start %s with process rate %f Hz, tread %f, wheel_radius %f",
                  node_name, process_rate, tread, wheel_radius)
    _ = WebotsVelocityNode(tread, wheel_radius)
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.loginfo("Exiting %s", node_name)


if __name__ == '__main__':
    main()
