#!/usr/bin/env python3

import numpy as np
import rospy
import time
import math
import tf

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, MagneticField

import tf2_ros
import tf2_geometry_msgs

class IMUFilter:
	def __init__(self):
		rospy.init_node('imu_filter', anonymous=True)

		self.mag_angle = 0
		self.mag_angle_x = 0
		self.mag_angle_y = 0
		self.mag_angle_z = 0

		self.rotation = (0.0, 0.0, 0.0, 1.0)

		self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=50)

		self.imu_sub = rospy.Subscriber("/imu/raw", Imu, self.imu_data)
		self.mag_sub = rospy.Subscriber("/imu/mag", MagneticField, self.mag_data)

		self.gyro_callib_y = None

	def mag_data(self, msg):
		m = msg.magnetic_field
		self.mag_angle = math.atan2(m.x, m.z)

	def imu_data(self, msg):
		self.imu_msg = msg

		if self.gyro_callib_y is None:
			self.gyro_callib_y = -msg.angular_velocity.y

		y = -msg.angular_velocity.y - self.gyro_callib_y
		deltarot = tf.transformations.quaternion_from_euler(0, 0, y * 0.0255)	
		self.rotation = tf.transformations.quaternion_multiply(self.rotation, deltarot)

		magquat = tf.transformations.quaternion_from_euler(self.mag_angle_x, self.mag_angle_y, self.mag_angle_z)
		magquat = tf.transformations.quaternion_from_euler(0, 0, self.mag_angle)
		self.rotation = tf.transformations.quaternion_slerp(self.rotation, magquat, 0.005)

		msg.orientation.x = self.rotation[0]
		msg.orientation.y = self.rotation[1]
		msg.orientation.z = self.rotation[2]
		msg.orientation.w = self.rotation[3]
		self.imu_pub.publish(msg)

try:
	b = IMUFilter()
	rospy.spin()
except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
