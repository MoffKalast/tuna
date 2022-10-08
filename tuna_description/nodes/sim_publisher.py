#!/usr/bin/env python3

import numpy as np
import rospy
import time
import math
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import Imu

RATE = 50.0
WAVE_SPEED = 1.0
WAVE_HEIGHT = 0.01

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

#old simulator
class State:
	def __init__(self):
		rospy.init_node('odometry_publisher')

		self.get_tf = tf.TransformListener()
		self.odom_tf = tf.TransformBroadcaster()

		self.motor_sub = rospy.Subscriber("/cmd_vel", Twist, self.motor_data)
		self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)

		self.x = 0
		self.y = 0
		self.angle = 0

		self.speed = 0
		self.angular_speed = 0
		
		self.CMD_speed = 0
		self.CMD_angular_speed = 0

		self.rotation = (0.0, 0.0, 0.0, 1.0)

	def motor_data(self, msg):
		self.CMD_speed = msg.linear.x * 0.85
		self.CMD_angular_speed = msg.angular.z

	def get_covariance(self, val):
		array = np.zeros(36)
		array[0] = val
		array[7] = val
		array[14] = val
		array[21] = val
		array[28] = val
		array[35] = val
		return array

	def publish_state(self):

		#slow acceleration and drifting
		self.angular_speed = self.angular_speed * 0.98 + self.CMD_angular_speed * 0.02
		self.speed = self.speed * 0.98 + self.CMD_speed * 0.02

		current_time = rospy.Time.now()
		linearmove = self.speed * 1.0/RATE

		self.angle += self.angular_speed * 1.0/RATE + abs(math.sin(rospy.get_time()*WAVE_SPEED)*WAVE_HEIGHT*0.2)

		turnx = linearmove
		turny = 0

		cosang = math.cos(self.angle)
		sinang = math.sin(self.angle)

		deltax = turnx * cosang - turny * sinang
		deltay = turnx * sinang + turny * cosang

		self.x += deltax + abs(math.sin(rospy.get_time()*WAVE_SPEED)*WAVE_HEIGHT*0.05)
		self.y += deltay + abs(math.sin(rospy.get_time()*WAVE_SPEED)*WAVE_HEIGHT*0.1)
		self.z = math.sin(rospy.get_time()*WAVE_SPEED)*WAVE_HEIGHT

		self.rotation = euler_to_quaternion(math.sin(rospy.get_time()*WAVE_SPEED)*WAVE_HEIGHT*10.0,math.sin(rospy.get_time()*WAVE_SPEED*1.5)*WAVE_HEIGHT*5.0,self.angle)

		self.odom_tf.sendTransform((self.x, self.y, self.z), self.rotation, current_time, "base_link", "odom")

		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"
		odom.pose.pose = Pose(Point(self.x, self.y, self.z), Quaternion(self.rotation[0], self.rotation[1], self.rotation[2], self.rotation[3]))
		odom.pose.covariance = self.get_covariance(0.5)
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(deltax, deltay, 0), Vector3(0, 0, self.angle))
		odom.twist.covariance = self.get_covariance(0.5)
		self.odom_pub.publish(odom)


try:
	state = State()
	r = rospy.Rate(RATE)
	while not rospy.is_shutdown():
		state.publish_state()
		r.sleep()

except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
