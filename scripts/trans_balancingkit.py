#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from tf.transformations import *

tfBroadcaster = ''
odomPublisher = ''
seq = 0
lastMsg = Point(0.0, 0.0, 0.0)

def setup():
    rospy.init_node('balancingkit_listener', anonymous=True)
    global seq
    global tfBroadcaster
    global odomPublisher
    seq = 0
    tfBroadcaster = tf.TransformBroadcaster()
    odomPublisher = rospy.Publisher('odom', Odometry, queue_size=10)

def callback(msg):
    global seq
    global tfBroadcaster
    global odomPublisher
    global lastMsg

    #rospy.loginfo(rospy.get_caller_id()+"I heard %s", msg)

    current_time = rospy.Time.now()

    point = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
#    rospy.loginfo(point)

    orientation = msg.pose.orientation
    quat_array = [orientation.x, orientation.y, orientation.z, orientation.w]
#     vector = (lastMsg.x - point[0], lastMsg.y - point[1], lastMsg.z - point[2])
#     rospy.loginfo(vector)
#     quat_array = tf.transformations.quaternion_about_axis(0.0, vector)
    q_rot = quaternion_from_euler(0, 0, -pi/2)
    quat_array = quaternion_multiply(q_rot, quat_array)
#    rospy.loginfo(quat_array)

    tfBroadcaster.sendTransform(point, quat_array, current_time, "base_footprint", "odom")

    odom = Odometry()
    odom.header.seq = seq
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion())

    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    odomPublisher.publish(odom)
    #rospy.loginfo(odom)

    lastMsg = msg.pose.position


def odometry_publisher():
    pub = rospy.Publisher('odom', Odometry, queue_size=10)

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

def listener():
    rospy.Subscriber("/notify_pos", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    setup()
    listener()
