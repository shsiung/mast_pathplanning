#! /usr/bin/env python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import cos,sin,pi
from tf.broadcaster import TransformBroadcaster
import tf
import rospy
import signal
import time
import numpy

YAW_DOT_MAX = pi/3

def rad_to_degrees(rad):
    return rad*180/pi

class Sim_Plane():

    # Constructor for initializing simulated plane state
    def __init__(self):
        rospy.init_node("plane")

        # x y z
        self.x = 0
        self.y = 0
        self.z = 0
        self.x_dot = 2
        self.y_dot = 0
        self.z_dot = 0

        # roll pitch yaw
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.roll_dot = 0
        self.pitch_dot = 0
        self.yaw_dot = YAW_DOT_MAX

        self.deltaT = 0.01;  #second

        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tfBroadcaster = TransformBroadcaster()
        self.odomMsg = Odometry()
        self.odomMsg.child_frame_id = "odom"

    def update_state(self):
        dist = self.x_dot * self.deltaT
        self.x += dist * cos(self.yaw)
        self.y += dist * sin(self.yaw)
        self.yaw += self.yaw_dot * self.deltaT


    def broadcast_state(self):

        self.odomMsg.header.stamp = rospy.Time.now()
        self.odomMsg.pose.pose.position.x = self.x
        self.odomMsg.pose.pose.position.y = self.y
        self.odomMsg.pose.pose.position.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)

        self.odomMsg.pose.pose.orientation.x = quat[0]
        self.odomMsg.pose.pose.orientation.y = quat[1]
        self.odomMsg.pose.pose.orientation.z = quat[2]
        self.odomMsg.pose.pose.orientation.w = quat[3]
        self.odomPub.publish(self.odomMsg)


        self.tfBroadcaster.sendTransform(
                (self.odomMsg.pose.pose.position.x,
                self.odomMsg.pose.pose.position.y,
                self.odomMsg.pose.pose.position.z),
                (self.odomMsg.pose.pose.orientation.x,self.odomMsg.pose.pose.orientation.y,
                    self.odomMsg.pose.pose.orientation.z,self.odomMsg.pose.pose.orientation.w),
                    rospy.Time.now(),
                    "odom",
                    "map",
        )

    def halt(self, *args):
        self.r.halt()
        rospy.core.signal_shutdown("sigint")

    def get_plane_state(self):
        return (self.x, self.y, self.z)

if __name__== "__main__":
    robot = Sim_Plane()
    while not rospy.is_shutdown():
        robot.get_plane_state()
        robot.update_state()
        robot.broadcast_state()
        rospy.sleep(robot.deltaT)