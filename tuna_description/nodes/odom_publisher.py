#!/usr/bin/env python3

import numpy as np
import rospy
import time
import math
import tf

from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import Imu, MagneticField

import tf2_ros
import tf2_geometry_msgs

RATE = 20.0

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

class State:
	def __init__(self):
		rospy.init_node('odometry_publisher')

		#self.get_tf = tf.TransformListener()
		self.odom_tf = tf.TransformBroadcaster()

		self.tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf_buffer)

		self.button_init = rospy.Subscriber("/goalpub/init", Bool, self.init)

		self.motor_sub = rospy.Subscriber("/cmd_vel", Twist, self.motor_data)
		self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_data)

		#self.fix_pub = rospy.Subscriber("/fix/tf", Point, self.gps_data)

		self.odom_sub = rospy.Subscriber("/odometry/gps", Odometry, self.gps_data)
		self.odom_pub = rospy.Publisher("/odometry/propeller", Odometry, queue_size=5)
		#self.null_pub = rospy.Publisher("/odometry/null", Odometry, queue_size=50)

		self.anglex = 0
		self.angley = 0
		self.anglez = 0

		self.gx = 0
		self.gy = 0

		self.px_raw = 0
		self.py_raw = 0

		self.gx_raw = 0
		self.gy_raw = 0

		self.offsetx = 0
		self.offsety = 0

		self.x = 0
		self.y = 0
		self.angle = 0

		self.speed = 0
		self.angular_speed = 0

		self.CMD_speed = 0
		self.CMD_angular_speed = 0

		self.rotation = (0.0, 0.0, 0.0, 1.0)

		self.magmin = Vector3()
		self.magmin.x = 9999.9
		self.magmin.y = 9999.9
		self.magmin.z = 9999.9

		self.magmax = Vector3()
		self.magmax.x = -9999.9
		self.magmax.y = -9999.9
		self.magmax.z = -9999.9

		self.imu_angle = 0

	def init(self, msg):

		if not msg.data:
			return

		self.offsetx = self.px_raw
		self.offsety = self.py_raw

		self.x = 0
		self.y = 0
		self.gx = 0
		self.gy = 0
		self.gx_raw = 0
		self.gy_raw = 0

	def gps_data(self, msg):

		self.px_raw = msg.pose.pose.position.x
		self.py_raw = msg.pose.pose.position.y

		if np.isnan(self.px_raw ) or np.isnan(self.py_raw):
			return

		#pos = transform_pos(msg.pose.pose.position, self.tf_buffer, "odom", "base_link")


		#vec = quat_mult_vector(tf.transformations.quaternion_inverse(self.rotation), (px, py, 0))

		#print(vec)

		self.gx_raw = self.px_raw - self.x - self.offsetx
		self.gy_raw = self.py_raw - self.y - self.offsety


	def imu_data(self, msg):
		#pass
		quat = transform_quat(msg.orientation, self.tf_buffer, "imu_link", "base_link").orientation
		self.rotation = (quat.x, quat.y, quat.z, quat.w)

		angles = tf.transformations.euler_from_quaternion(self.rotation)
		self.imu_angle = angles[2]


	def motor_data(self, msg):
		self.CMD_speed = msg.linear.x * 0.7
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

		#gps interpolations
		self.gx = self.gx * 0.997 + 0.003 * self.gx_raw
		self.gy = self.gy * 0.997 + 0.003 * self.gy_raw

		#slow acceleration and drifting
		self.angular_speed = self.angular_speed * 0.98 + self.CMD_angular_speed * 0.02
		self.speed = self.speed * 0.98 + self.CMD_speed * 0.02

		current_time = rospy.Time.now()
		linearmove = self.speed * 1.0/RATE

		#self.angle += self.angular_speed * 1.0/RATE
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
		self.odom_tf.sendTransform((self.gx, self.gy, 0), (0,0,0,1), current_time, "odom", "map")

		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"
		odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(0,0,0,1))
		odom.pose.covariance = self.get_covariance(1.0)
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(deltax, deltay, 0), Vector3(0, 0, 0))
		odom.twist.covariance = self.get_covariance(1.0)
		self.odom_pub.publish(odom)


		#odom1 = Odometry()
		#odom1.header.stamp = current_time
		#odom1.header.frame_id = "odom"
		#odom1.pose.pose = Pose(Point(0, 0, 0), Quaternion(0,0,0,1))
		#odom1.pose.covariance = self.get_covariance(1.0)
		#odom1.child_frame_id = "base_link"
		#odom1.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		#odom1.twist.covariance = self.get_covariance(1.0)
		#self.null_pub.publish(odom1)


try:
	state = State()
	r = rospy.Rate(RATE)
	while not rospy.is_shutdown():
		state.publish_state()
		r.sleep()

except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
