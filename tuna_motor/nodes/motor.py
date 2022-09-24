#!/usr/bin/env python3

import rospy
import time
import pigpio
import math
import sys

from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState

PIN_STBY = 17
PIN_A1 = 27
PIN_A2 = 22

PIN_B1 = 24
PIN_B2 = 23

PIN_PWM1 = 12
PIN_PWM2 = 13

MIN_SPD = 15
MAX_SPD = 45

DEADMAN = 0.6 # seconds

def clamp(val, minval, maxval):
	if val < minval:
		return minval
	if val > maxval:
		return maxval
	return val

class Motor:

	def __init__(self, pi, PIN_DIR1, PIN_DIR2, PIN_PWM):
		self.pi = pi

		self.PIN_DIR1 = PIN_DIR1
		self.PIN_DIR2 = PIN_DIR2
		self.PIN_PWM = PIN_PWM

		self.pi.set_mode(self.PIN_DIR1, pigpio.OUTPUT)
		self.pi.write(self.PIN_DIR1, 0)

		self.pi.set_mode(self.PIN_DIR2, pigpio.OUTPUT)
		self.pi.write(self.PIN_DIR2, 0)

		self.pi.set_mode(self.PIN_PWM, pigpio.OUTPUT)
		self.pi.set_PWM_range(self.PIN_PWM, 100)
		self.pi.set_PWM_frequency(self.PIN_PWM, 20000)
		self.pi.set_PWM_dutycycle(self.PIN_PWM, 0)

	def set(self, velocity):

		if velocity > 0:
			self.pi.write(self.PIN_DIR1, 1)
			self.pi.write(self.PIN_DIR2, 0)
		else:
			self.pi.write(self.PIN_DIR1, 0)
			self.pi.write(self.PIN_DIR2, 1)

		self.pi.set_PWM_dutycycle(self.PIN_PWM, clamp(abs(velocity), 0, MAX_SPD))

	def stop(self):
		self.pi.write(self.PIN_DIR1, 0)
		self.pi.write(self.PIN_DIR2, 0)
		self.pi.set_PWM_dutycycle(self.PIN_PWM, 0)

class Controller:
	def __init__(self):
		rospy.init_node('motortest')

		self.pi = pigpio.pi()

		# standby switch
		self.pi.set_mode(PIN_STBY, pigpio.OUTPUT)
		self.pi.write(PIN_STBY, 0)

		#motor setup
		self.motor_right = Motor(self.pi, PIN_A1, PIN_A2, PIN_PWM1)
		self.motor_left = Motor(self.pi, PIN_B1, PIN_B2, PIN_PWM2)

		self.cmdsub = rospy.Subscriber("diff_drive", JointState, self.velocity)

		self.m_sleep = True
		self.m_time = time.time()
		self.m_left_vel = 0
		self.m_right_vel = 0

		rospy.loginfo("Motors Ready")

	def velocity(self, msg):

		self.m_time = time.time()
		self.m_sleep = False

		left_vel = int(msg.velocity[0] * 100)
		right_vel = int(msg.velocity[1] * 100)

		if left_vel == 0 and right_vel == 0:
			self.stop()
			return

		self.pi.write(PIN_STBY, 1)

		# just in case
		left_spd = clamp(abs(left_vel), MIN_SPD, MAX_SPD)
		right_spd = clamp(abs(right_vel), MIN_SPD, MAX_SPD)

		self.m_left_vel = math.copysign(left_spd, left_vel)
		self.m_right_vel = math.copysign(right_spd, right_vel)

	def update(self):
		if self.m_sleep:
			return

		if rospy.get_time() - self.m_time > DEADMAN:
			self.stop()
			self.m_sleep = True
		else:
			self.motor_left.set(self.m_left_vel)
			self.motor_right.set(self.m_right_vel)
			print(self.m_right_vel, self.m_left_vel)


	def stop(self):
		self.pi.write(PIN_STBY, 0)
		self.motor_left.stop()
		self.motor_right.stop()


	def cleanup(self):
		self.stop()
		self.pi.stop()


try:
	ctrl = Controller()
	rospy.on_shutdown(ctrl.cleanup)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		ctrl.update()
		rate.sleep()

except Exception as e:
	print(e)
