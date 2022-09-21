#!/bin/python3

import rospy
import time
import os
import tf2_ros
import tf
import math
import numpy as np

import tf2_ros
import tf2_geometry_msgs

# High percision for UTM coords
from decimal import *
getcontext().prec = 12

from std_srvs.srv import Empty
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, TransformStamped, PoseStamped, TwistWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

from iris_lama_ros.srv import UTMtoLL, UTMtoLLResponse
from iris_lama_ros.msg import GNSSReference

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_conjugate, quaternion_inverse

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

def clamp(val, vmin, vmax):
	if val > vmax:
		return vmax
	if val < vmin:
		return vmin
	return val

def get_yaw(q):	
	return math.atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x))

def get_pitch(q):
	return math.asin(-2.0*(q.x*q.z - q.w*q.y))

def get_roll(q):
	return math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)

def get_quat(orientation):
	return np.array([orientation.x, orientation.y, orientation.z, orientation.w])

def quaternion_apply(q, v):
	return quaternion_multiply(
		quaternion_multiply(q, np.array([v[0], v[1], v[2], 0])), 
		quaternion_conjugate(q)
	)[:3]

class TFPublisher:

	def __init__(self):
		rospy.init_node('gps_tf_publisher', anonymous=False)

		self.tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf_buffer)

		self.tf_static_pub = tf2_ros.StaticTransformBroadcaster()
		self.tf_pub = tf2_ros.TransformBroadcaster()

		self.origin_pub = rospy.Publisher("odom/gps/origin", Odometry, queue_size=1)

		self.init()

		self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
		self.imu_sub = rospy.Subscriber("imu/data", Imu, self.imu_callback)
		self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
		self.gps_sub = rospy.Subscriber("odom/gps", Odometry, self.gps_callback)
		self.fix_sub = rospy.Subscriber("gnss/fix", NavSatFix, self.fix_callback)

		self.origin_srv = rospy.Service('gps_tf_origin', UTMtoLL, self.origin)
		self.reset_srv = rospy.Service('gps_tf_reset', Empty, self.init)

	def init(self):
		self.tf_data = {}

		self.odom_delayed_x = 0
		self.odom_delayed_y = 0
		self.odom_msg_latest = None
		self.odom_msg = None

		self.imu_msg_latest = None
		self.imu_msg = None

		self.gps_msg = None
		self.fix_msg = None

		self.origin_fix = None
		self.origin_msg = None
		self.origin_x = None
		self.origin_y = None

		self.gps_raw_x = 0
		self.gps_raw_y = 0

		self.gps_x = 0
		self.gps_y = 0

		self.heading = 0
		self.heading_imu = 0
		self.heading_imu_prev = 0

	def origin(self, msg):
		if self.origin_fix is None:
			return UTMtoLLResponse()

		response = UTMtoLLResponse()
		response.nav_sat = self.origin_fix
		return response

	def tf_callback(self, msg):
		for tf in msg.transforms:
			if tf.header.frame_id == "base_link" and tf.child_frame_id == "imu":
				self.tf_data["imu"] = tf
				self.tf_sub.unregister()

	def fix_callback(self, msg):
		if self.origin_fix is None and msg.status.status != -1:
			 self.origin_fix = msg

		self.fix_msg = msg

	def imu_callback(self, msg):
		try:
			quat = transform_quat(msg.orientation, self.tf_buffer, "imu_link", "base_link").orientation
			self.rotation = (quat.x, quat.y, quat.z, quat.w)
		except:
			return;

		angles = euler_from_quaternion(self.rotation)
		self.heading_imu = angles[2]
		self.imu_msg_latest = msg

	def gps_callback(self, msg):
		if math.isnan(msg.pose.pose.position.x) or math.isnan(msg.pose.pose.position.y):
			rospy.logwarn("GPS has no fix!")
			return

		self.gps_last_received_stamp = rospy.Time.now()
		self.gps_msg = msg

		self.odom_msg = self.odom_msg_latest
		self.imu_msg = self.imu_msg_latest

		# define starting origin
		if self.origin_x is None:
			self.origin_x = Decimal(self.gps_msg.pose.pose.position.x)
			self.origin_y = Decimal(self.gps_msg.pose.pose.position.y)
			self.origin_msg = msg

			world_map = TransformStamped()
			world_map.header.stamp = rospy.Time.now()
			world_map.header.frame_id = "world"
			world_map.child_frame_id = "map"
			world_map.transform.translation.x = float(self.origin_x)
			world_map.transform.translation.y = float(self.origin_y)
			world_map.transform.translation.z = 0.0
			world_map.transform.rotation.x = 0.0
			world_map.transform.rotation.y = 0.0
			world_map.transform.rotation.z = 0.0
			world_map.transform.rotation.w = 1.0
			self.tf_static_pub.sendTransform(world_map)

		self.gps_raw_x = float(Decimal(msg.pose.pose.position.x) - self.origin_x)
		self.gps_raw_y = float(Decimal(msg.pose.pose.position.y) - self.origin_y)

		self.origin_pub.publish(self.origin_msg)

	def odom_callback(self, msg):
		self.odom_msg_latest = msg

	def update(self):

		# low pass filtering for the GNSS location
		self.gps_x = self.gps_x * 0.998 + 0.002 * self.gps_raw_x
		self.gps_y = self.gps_y * 0.998 + 0.002 * self.gps_raw_y

		if self.odom_msg is None or self.gps_msg is None or self.imu_msg is None:
			logmsg = "Waiting for: "

			if self.odom_msg is None:
				logmsg += "ODOM "
			if self.gps_msg is None:
				logmsg += "GPS "
			if self.imu_msg is None:
				logmsg += "IMU "

			logmsg += "messages..."
			rospy.logwarn(logmsg)
			rospy.sleep(0.2)
			return

		# delayed response for zeroing out odom position relative to GNSS 
		self.odom_delayed_x = self.odom_delayed_x * 0.9985 + 0.0015 * self.odom_msg.pose.pose.position.x
		self.odom_delayed_y = self.odom_delayed_y * 0.9985 + 0.0015 * self.odom_msg.pose.pose.position.y

		robot_rotation = quaternion_from_euler(0,0,self.heading)
		odom_rotation = quaternion_from_euler(0,0,get_yaw(self.odom_msg.pose.pose.orientation))

		true_rotation = quaternion_multiply(quaternion_inverse(odom_rotation),robot_rotation)
		odom_neg = quaternion_apply(true_rotation, [self.odom_delayed_x, self.odom_delayed_y, 0.0])

		map_odom = TransformStamped()
		map_odom.header.stamp = rospy.Time.now()
		map_odom.header.frame_id = "map"
		map_odom.child_frame_id = "odom"
		map_odom.transform.translation.x = self.gps_x - odom_neg[0]
		map_odom.transform.translation.y = self.gps_y - odom_neg[1]
		map_odom.transform.translation.z = 0.0
		map_odom.transform.rotation.x = true_rotation[0]
		map_odom.transform.rotation.y = true_rotation[1]
		map_odom.transform.rotation.z = true_rotation[2]
		map_odom.transform.rotation.w = true_rotation[3]
		self.tf_pub.sendTransform(map_odom)

try:
	tf = TFPublisher()
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		tf.update()
		rate.sleep()
except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)