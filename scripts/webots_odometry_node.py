#!/usr/bin/env python
# -*- coding: utf_8 -*-
import math
import os
import rospy
from angles import normalize_angle
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler


class WebotsOdometry(object):
    def __init__(self, tread, wheel_radius, pos_left_motor=0, pos_right_motor=0, frame_id='odom', child_frame_id='base_link'):
        self.tread = tread
        self.wheel_radius = wheel_radius
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.reset(pos_left_motor, pos_right_motor)

    def update_and_get_odom(self, current_time, pos_left_motor, pos_right_motor):
        delta = (current_time - self.time).to_sec()
        self.time = current_time
        result = self.update(pos_left_motor, pos_right_motor, delta)
        ros_odom = build_ros_odometry_2d(current_time, self.frame_id, self.child_frame_id, self.seq,
                                         result[0], result[1], result[2],
                                         result[3], 0, result[4])
        self.seq = self.seq + 1
        return ros_odom

    def update_wheel(self, pos_new, pos_cur):
        d = (pos_new - pos_cur) * self.wheel_radius
        return (d, pos_new)

    def update(self, pos_left_motor, pos_right_motor, delta):
        (d_left, self.pos_left_motor) = self.update_wheel(
            pos_left_motor, self.pos_left_motor)
        (d_right, self.pos_right_motor) = self.update_wheel(
            pos_right_motor, self.pos_right_motor)
        d_total = (d_left + d_right) / 2.0
        if d_right != d_left:
            d_yaw = (d_right - d_left) / self.tread
            new_yaw = self.yaw + d_yaw
            radius = d_total / d_yaw
            self.x += radius * (math.sin(new_yaw) - math.sin(self.yaw))
            self.y -= radius * (math.cos(new_yaw) - math.cos(self.yaw))
            self.yaw = normalize_angle(new_yaw)
        else:
            d_yaw = 0
            self.x += d_total * math.cos(self.yaw)
            self.y += d_total * math.sin(self.yaw)

        linear_x = d_total / delta
        angular_z = d_yaw / delta
        return (self.x, self.y, self.yaw, linear_x, angular_z)

    def reset(self, pos_left_motor, pos_right_motor):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.pos_left_motor = pos_left_motor
        self.pos_right_motor = pos_right_motor
        self.seq = 0
        self.time = rospy.Time.now()


class WebotsOdometryNode(object):
    def __init__(self, webots_odometry, topic_webots_wheel_pos="/webots/wheel_pos", topic_odom="/odom"):
        self.webots_odometry = webots_odometry
        self.sub_webots_wheel_pos = rospy.Subscriber(
            topic_webots_wheel_pos, Float32MultiArray, self.webots_wheel_pos_callback)
        self.pub_odom = rospy.Publisher(topic_odom, Odometry, queue_size=1)

    def webots_wheel_pos_callback(self, message):
        pl = message.data[0]
        pr = message.data[1]
        odom = self.webots_odometry.update_and_get_odom(
            rospy.Time.now(), pl, pr)
        self.pub_odom.publish(odom)


def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)
    # https://www.cyberbotics.com/doc/guide/pioneer-3dx?version=master
    process_rate = rospy.get_param("~process_rate", 10.0)
    tread = rospy.get_param("~tread", 0.381)
    wheel_radius = rospy.get_param("~wheel_radius", 0.195/2)
    frame_id = rospy.get_param("~frame_id", "odom")
    child_frame_id = rospy.get_param("~child_frame_id", "base_link")
    rate = rospy.Rate(process_rate)
    rospy.loginfo("Start %s with process rate %f Hz, tread %f, wheel_radius %f",
                  node_name, process_rate, tread, wheel_radius)
    odom = WebotsOdometry(tread, wheel_radius, 0, 0, frame_id, child_frame_id)
    _ = WebotsOdometryNode(odom)
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.loginfo("Exiting %s", node_name)


def build_ros_odometry_2d(current_time, frame_id, child_frame_id, seq, x, y, yaw, vel_x, vel_y, vel_yaw):
    quat = quaternion_from_euler(0, 0, yaw)
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = frame_id
    # set the position
    odom.pose.pose = Pose(
        Point(x, y, 0.), Quaternion(*quat))

    # set the velocity
    odom.child_frame_id = child_frame_id

    odom.pose.covariance[0] = 0.01
    odom.pose.covariance[7] = 0.01
    odom.pose.covariance[14] = 99999
    odom.pose.covariance[21] = 99999
    odom.pose.covariance[28] = 99999
    odom.pose.covariance[35] = 0.01

    odom.twist.twist.linear.x = vel_x
    odom.twist.twist.linear.y = vel_y
    odom.twist.twist.angular.z = vel_yaw
    odom.twist.covariance = odom.pose.covariance
    return odom


if __name__ == '__main__':
    main()
