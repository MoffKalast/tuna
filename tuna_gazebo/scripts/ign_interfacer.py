#!/usr/bin/env python3

import numpy as np
import rospy
import time
import math
import random
import tf

from std_msgs.msg import Header, String, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Twist

MIN_SPD = 0.15
MAX_SPD = 0.45

DEADMAN = 0.6 # seconds

def clamp(val, minval, maxval):
	if val < minval:
		return minval
	if val > maxval:
		return maxval
	return val

# Stand in for the hardware drivers
class IGNInterfacer:
	def __init__(self):
		rospy.init_node('ign_interfacer')

		self.right_pub = rospy.Publisher("prop_spd_right", Float64, queue_size=1)
		self.left_pub = rospy.Publisher("prop_spd_left", Float64, queue_size=1)

		self.diff_drive_sub = rospy.Subscriber("diff_drive", JointState, self.diff_drive_cb)

		self.m_sleep = True
		self.m_time = time.time()
		self.m_left_vel = 0
		self.m_right_vel = 0

		rospy.loginfo("Motors Ready")

	def diff_drive_cb(self, msg):
		self.m_time = time.time()
		self.m_sleep = False

		left_vel = msg.velocity[0]
		right_vel = msg.velocity[1]

		if left_vel == 0.0 and right_vel == 0.0:
			self.stop()
			return

		# just in case
		left_spd = clamp(abs(left_vel), MIN_SPD, MAX_SPD)
		right_spd = clamp(abs(right_vel), MIN_SPD, MAX_SPD)

		self.m_left_vel = math.copysign(left_spd, left_vel) * 2.0
		self.m_right_vel = math.copysign(right_spd, right_vel) * 2.0

	def update(self):
		if self.m_sleep:
			return

		if rospy.get_time() - self.m_time > DEADMAN:
			self.stop()
			self.m_sleep = True
		else:
			self.right_pub.publish(self.m_left_vel)
			self.left_pub.publish(self.m_right_vel)


	def stop(self):
		self.m_left_vel = 0.0
		self.m_right_vel = 0.0
		self.right_pub.publish(0.0)
		self.left_pub.publish(0.0)
		self.m_sleep = True

try:
	ctrl = IGNInterfacer()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		ctrl.update()
		rate.sleep()

except Exception as e:
	print(e)