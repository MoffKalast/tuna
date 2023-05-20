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
		self.rotation = (0.0, 0.0, 0.0, 1.0)
		self.gps_rotation = None
		self.last_time = rospy.Time.now().to_sec()

		self.imu_pub = rospy.Publisher("imu/data", Imu, queue_size=50)

		self.imu_sub = rospy.Subscriber("imu/data_raw", Imu, self.imu_data)
		self.mag_sub = rospy.Subscriber("imu/mag", MagneticField, self.mag_data)
		self.gps_sub = rospy.Subscriber("navsat_simple/heading", PoseWithCovarianceStamped, self.gps_data)

		self.pose_pub = rospy.Publisher('imu/debug_pose', PoseWithCovarianceStamped, queue_size=10)


	def gps_data(self, msg):
		q = msg.pose.pose.orientation

		if q.x == 0 and q.y == 0 and q.z == 0 and q.w == 0:
			self.gps_rotation = None
		else:		
			self.gps_rotation = [q.x, q.y, q.z, q.w]

	def mag_data(self, msg):
		m = msg.magnetic_field

		self.mag_angle = math.atan2(m.x, m.z)

		#Theoretical compensation for tilt, if accel data wasn't complete trash 

		#roll = atan2(accel_y, accel_z)
		#pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z))

		#mag_x_comp = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) + mag_z * cos(roll) * sin(pitch)
		#mag_y_comp = mag_y * cos(roll) - mag_z * sin(roll)

		#self.mag_angle = atan2(-mag_y_comp, mag_x_comp)

	def imu_data(self, msg):

		current_time = msg.header.stamp.to_sec()
		dt = current_time - self.last_time
		self.last_time = current_time

		deltarot = tf.transformations.quaternion_from_euler(msg.angular_velocity.x * dt, -msg.angular_velocity.z * dt, -msg.angular_velocity.y * dt)	
		self.rotation = tf.transformations.quaternion_multiply(self.rotation, deltarot)

		#deltarot = tf.transformations.quaternion_from_euler(msg.angular_velocity.x * 0.0255, -msg.angular_velocity.z * 0.0255, -msg.angular_velocity.y * 0.0255)	
		#self.rotation = tf.transformations.quaternion_multiply(self.rotation, deltarot)

		magquat = tf.transformations.quaternion_from_euler(0, 0, self.mag_angle)
		self.rotation = tf.transformations.quaternion_slerp(self.rotation, magquat, 0.02)

		if not self.gps_rotation is None:
			self.rotation = tf.transformations.quaternion_slerp(self.rotation, self.gps_rotation, 0.04)

		msg.orientation.x = self.rotation[0]
		msg.orientation.y = self.rotation[1]
		msg.orientation.z = self.rotation[2]
		msg.orientation.w = self.rotation[3]
		self.imu_pub.publish(msg)

		test = PoseWithCovarianceStamped()
		test.header.frame_id = "map"
		test.pose.pose.position.x = 1.0
		test.pose.pose.orientation = msg.orientation
		self.pose_pub.publish(test)


b = IMUFilter()
rospy.spin()