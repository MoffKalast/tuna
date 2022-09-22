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
MIN_SPD = 0.15
MAX_SPD = 0.40

def transform_pos(position, tf_buffer, from_frame, to_frame):
	pose_stamped = tf2_geometry_msgs.PoseStamped()
	pose_stamped.pose.position = position
	pose_stamped.header.frame_id = from_frame
	pose_stamped.header.stamp = rospy.Time.now()

	try:
		# ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
		output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(0.1))
		return output_pose_stamped.pose.position

	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		raise

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

def euler_to_quaternion(roll, pitch, yaw):

	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qx, qy, qz, qw]

def quat_mult_vector(q1, v1):
	dist = math.sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2])
	v1 = tf.transformations.unit_vector(v1)
	q2 = list(v1)
	q2.append(0.0)

	vec = tf.transformations.quaternion_multiply(
		tf.transformations.quaternion_multiply(q1, q2), 
		tf.transformations.quaternion_conjugate(q1)
	)[:3]

	vec[0] *= dist
	vec[1] *= dist
	vec[2] *= dist

	return vec 

def clamp(val, minval, maxval):
	if val < minval:
		return minval
	if val > maxval:
		return maxval
	return val

def diffdrive(x,  y):

		x = clamp(x, -MAX_SPD, MAX_SPD)
		y = clamp(y, -MAX_SPD, MAX_SPD)

		# First Compute the angle in deg
		# First hypotenuse
		z = math.sqrt(x * x + y * y)

		if z == 0:
			return [0,0]

		# angle in radians
		rad = math.acos(math.fabs(x) / z)

		# and in degrees
		angle = rad * 180 / math.pi

		# Now angle indicates the measure of turn
		# Along a straight line, with an angle o, the turn co-efficient is same
		# this applies for angles between 0-90, with angle 0 the coeff is -1
		# with angle 45, the co-efficient is 0 and with angle 90, it is 1
		tcoeff = -1 + (angle / 90) * 2
		turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
		turn = round(turn * 100, 0) / 100

		# And max of y or x is the movement
		mov = max(math.fabs(y), math.fabs(x))

		# First and third quadrant
		if (x >= 0 and y >= 0) or (x < 0 and y < 0):
			rawLeft = mov
			rawRight = turn
		else:
			rawRight = mov
			rawLeft = turn

		rawRight = round(rawRight * 100)
		rawLeft = round(rawLeft * 100)

		if y < 0:
			return [-rawLeft, -rawRight]

		return [rawRight, rawLeft]

class State:
	def __init__(self):
		rospy.init_node('odometry_publisher')

		self.odom_tf = tf.TransformBroadcaster()

		self.tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf_buffer)

		self.motor_sub = rospy.Subscriber("/cmd_vel", Twist, self.motor_data)
		self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_data)

		self.odom_pub = rospy.Publisher("/odometry/propeller", Odometry, queue_size=5)
		self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

		self.x = 0
		self.y = 0
		self.angle = 0

		self.speed = 0
		self.angular_speed = 0

		self.cmd_linear_speed = 0
		self.cmd_angular_speed = 0
		self.cmd_vel_msg = None

		self.prop_left = 0.0
		self.prop_right = 0.0

		self.rotation = (0.0, 0.0, 0.0, 1.0)
		self.imu_angle = 0

	def imu_data(self, msg):
		quat = transform_quat(msg.orientation, self.tf_buffer, "imu_link", "base_link").orientation
		self.rotation = (quat.x, quat.y, quat.z, quat.w)

		angles = tf.transformations.euler_from_quaternion(self.rotation)
		self.imu_angle = angles[2]


	def motor_data(self, msg):
		self.cmd_linear_speed = clamp(msg.linear.x * 0.85, -MAX_SPD, MAX_SPD)
		self.cmd_angular_speed = clamp(msg.angular.z, -MAX_SPD, MAX_SPD)
		self.cmd_vel_msg = msg

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
		self.angular_speed = self.angular_speed * 0.98 + self.cmd_angular_speed * 0.02
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

		#self.rotation = euler_to_quaternion(0, 0, self.angle)

		self.odom_tf.sendTransform((self.x, self.y, 0), self.rotation, current_time, "base_link", "odom")

		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"
		odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(0,0,0,1))
		odom.pose.covariance = self.get_covariance(1.0)
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(deltax, deltay, 0), Vector3(0, 0, 0))
		odom.twist.covariance = self.get_covariance(1.0)
		self.odom_pub.publish(odom)

		if not self.cmd_vel_msg is None:
			left_vel, right_vel = diffdrive(self.cmd_vel_msg.angular.z, self.cmd_vel_msg.linear.x)

			self.prop_left += left_vel*0.03
			self.prop_right += right_vel*0.03

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
