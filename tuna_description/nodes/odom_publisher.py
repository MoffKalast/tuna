#!/usr/bin/env python3

import numpy as np
import rospy
import time
import math
import tf
import random

from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import Imu, MagneticField, JointState

import tf2_ros
import tf2_geometry_msgs

RATE = 20.0
CALIB_FACTOR = 0.85

def transform_quat(orientation, tf_buffer, from_frame, to_frame):
	pose_stamped = tf2_geometry_msgs.PoseStamped()
	pose_stamped.pose.orientation = orientation
	pose_stamped.header.frame_id = from_frame
	pose_stamped.header.stamp = rospy.Time.now()

	try:
		# ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
		output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(0.1))
		return output_pose_stamped.pose

	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		raise

class State:
	def __init__(self):
		rospy.init_node('odometry_publisher')

		self.odom_tf = tf.TransformBroadcaster()

		self.tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf_buffer)

		self.diff_drive_sub = rospy.Subscriber("/diff_drive", JointState, self.diff_drive_data)
		self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_data)

		self.odom_pub = rospy.Publisher("/odometry/propeller", Odometry, queue_size=5)
		self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

		self.x = 0
		self.y = 0
		self.angle = 0

		self.speed = 0

		self.cmd_linear_speed = 0
		self.cmd_prop_left = 0.0
		self.cmd_prop_right = 0.0

		self.prop_left = 0.0
		self.prop_right = 0.0

		self.rotation = (0.0, 0.0, 0.0, 1.0)
		self.imu_angle = 0

	def imu_data(self, msg):
		quat = transform_quat(msg.orientation, self.tf_buffer, "imu_link", "base_link").orientation
		self.rotation = (quat.x, quat.y, quat.z, quat.w)

		angles = tf.transformations.euler_from_quaternion(self.rotation)
		self.imu_angle = angles[2]

	def diff_drive_data(self, msg):
		self.cmd_linear_speed = (msg.velocity[0] + msg.velocity[1]) * 0.5 * CALIB_FACTOR
		self.cmd_prop_left = msg.velocity[0]
		self.cmd_prop_right = msg.velocity[1]

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
		self.speed = self.speed * 0.97 + self.cmd_linear_speed * 0.03

		current_time = rospy.Time.now()
		linearmove = self.speed * 1.0/RATE

		self.angle = self.imu_angle

		turnx = linearmove
		turny = 0

		cosang = math.cos(self.angle)
		sinang = math.sin(self.angle)

		deltax = turnx * cosang - turny * sinang
		deltay = turnx * sinang + turny * cosang

		self.x += deltax
		self.y += deltay

		self.odom_tf.sendTransform((self.x, self.y, 0), self.rotation, current_time, "base_link", "odom")

		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"
		odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(self.rotation[0], self.rotation[1], self.rotation[2], self.rotation[3]))
		odom.pose.covariance = self.get_covariance(1.0)
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(deltax, deltay, 0), Vector3(0, 0, 0))
		odom.twist.covariance = self.get_covariance(1.0)
		self.odom_pub.publish(odom)

		# Joint States for RViz

		self.prop_left += self.cmd_prop_left * 3.1
		self.prop_right += self.cmd_prop_left * 3.1

		state = JointState()
		state.header.stamp = current_time
		state.name = ["base_to_prop_left", "base_to_prop_right"]
		state.position = [self.prop_left  , -self.prop_right]
		self.joint_pub.publish(state)

try:
	state = State()
	r = rospy.Rate(RATE)
	while not rospy.is_shutdown():
		state.publish_state()
		r.sleep()

except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
