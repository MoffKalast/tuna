#!/usr/bin/env python3

import numpy as np
import rospy
import time
import math
import tf

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import Imu, MagneticField

import tf2_ros
import tf2_geometry_msgs

class IMUFilter:
	def __init__(self):
		rospy.init_node('imu_filter')

		self.tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf_buffer)

		self.mag_angle = 0
		self.rotation = (0.0, 0.0, 0.0, 1.0)

		self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=50)
		self.id_pub = rospy.Publisher("/imu/identity", Imu, queue_size=50)

		self.imu_sub = rospy.Subscriber("/imu/data_raw", Imu, self.imu_data)
		self.mag_sub = rospy.Subscriber("/imu/mag", MagneticField, self.mag_data)

	def mag_data(self, msg):
		m = msg.magnetic_field
		self.mag_angle = math.atan2(m.x, m.z);


	def imu_data(self, msg):
		deltarot = tf.transformations.quaternion_from_euler(msg.angular_velocity.x * 0.0255, -msg.angular_velocity.z * 0.0255, -msg.angular_velocity.y * 0.0255)	
		self.rotation = tf.transformations.quaternion_multiply(self.rotation, deltarot)

		magquat = tf.transformations.quaternion_from_euler(0, 0, self.mag_angle)
		self.rotation = tf.transformations.quaternion_slerp(self.rotation, magquat, 0.02)

		msg.orientation.x = self.rotation[0]
		msg.orientation.y = self.rotation[1]
		msg.orientation.z = self.rotation[2]
		msg.orientation.w = self.rotation[3]
		self.imu_pub.publish(msg)

		ide = Imu()
		ide.header = msg.header
		ide.orientation.x = 0.0
		ide.orientation.y = 0.0
		ide.orientation.z = 0.0
		ide.orientation.w = 1.0
		self.id_pub.publish(ide)

try:
	b = IMUFilter()
	rospy.spin()
except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
