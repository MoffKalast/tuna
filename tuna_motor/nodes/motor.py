#!/usr/bin/env python3

import rospy
import time
import pigpio
import math
import sys

from std_msgs.msg import Header
from geometry_msgs.msg import Twist

PIN_STBY = 17
PIN_A1 = 27
PIN_A2 = 22

PIN_B1 = 24
PIN_B2 = 23

PIN_PWM1 = 12
PIN_PWM2 = 13

MIN_SPD = 15
MAX_SPD = 40

DEADMAN = 0.6 # seconds

def clamp(val, minval, maxval):
	if val < minval:
		return minval
	if val > maxval:
		return maxval
	return val

def diffdrive(x,  y):

		x = clamp(x, -1.0, 1.0)
		y = clamp(y, -1.0, 1.0)

		# First Compute the angle in deg
		# First hypotenuse
		z = math.sqrt(x * x + y * y)

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

def map(v, in_min, in_max, out_min, out_max):
	v = clamp(v, in_min, in_max)
	return (v - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

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

		self.cmdsub = rospy.Subscriber("/cmd_vel", Twist, self.velocity)

		self.m_sleep = True
		self.m_time = time.time()
		self.m_left_vel = 0
		self.m_right_vel = 0

		print("Ready")

	def velocity(self, msg):

		self.m_time = time.time()
		self.m_sleep = False

		if msg.angular.z == 0 and msg.linear.x == 0:
			self.stop()
			return

		self.pi.write(PIN_STBY, 1)

		left_vel, right_vel = diffdrive(msg.angular.z, msg.linear.x)

		left_spd = abs(left_vel)
		right_spd = abs(right_vel)

		if left_spd != 0:
			if left_spd < MIN_SPD:
				left_vel = math.copysign(MIN_SPD, left_vel)
			elif left_spd > MAX_SPD:
				left_vel = math.copysign(MAX_SPD, left_vel)

		if right_spd != 0:
			if right_spd < MIN_SPD:
				right_vel = math.copysign(MIN_SPD, right_vel)
			elif right_spd > MAX_SPD:
				right_vel = math.copysign(MAX_SPD, right_vel)

		self.m_left_vel = left_vel
		self.m_right_vel = right_vel

	def update(self):
		if self.m_sleep:
			return

		if rospy.get_time() - self.m_time > DEADMAN:
			self.stop()
			self.m_sleep = True
		else:
			self.motor_left.set(self.m_left_vel)
			self.motor_right.set(self.m_right_vel)


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
