#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def odometry_publisher():
    pub = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.init_node('odometry_publisher', anonymous=True)
    broad = tf.TransformBroadcaster()

    r = rospy.Rate(1) # 1Hz
    seq = 0

    x = 0.0
    y = 0.0
    th = 0.0
    vx = 0.1
    vy = -0.1
    vth = 0.1

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        broad.sendTransform((x, y, 0.0), odom_quat, current_time, "base_footprint", "odom")

        odom = Odometry()
        odom.header.seq = seq
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))

        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        pub.publish(odom)
        rospy.loginfo(odom)

        last_time = current_time
        seq += 1

        r.sleep()

if __name__ == '__main__':
    try:
        odometry_publisher()
    except rospy.ROSInterruptException: pass
