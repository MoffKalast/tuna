#!/usr/bin/env python3

import numpy as np
import rospy
import time
import math
import random
import tf

from geometry_msgs.msg import Vector3

class WaveGen:
	def __init__(self):
		rospy.init_node('gazebo_wave_generator')

		self.wave_magnitude = 0.3
		self.wave_pub = rospy.Publisher("ocean_current", Vector3, queue_size=1)

		rospy.loginfo("Wave Generator started")

	def update(self):
		vel = Vector3()
		vel.x = math.sin(time.time() * 2.5 + random.random()*0.5) * self.wave_magnitude * 0.8
		vel.y = math.cos(time.time() * 2.5 + random.random()*0.4) * self.wave_magnitude * 0.8

		vel.x += math.cos(time.time() * 1.2 + random.random()*0.5) * self.wave_magnitude * 0.4
		vel.y += math.sin(time.time() * 1.2 + random.random()*0.4) * self.wave_magnitude * 0.4

		vel.x += (random.random()*0.1 - 0.05) * self.wave_magnitude
		vel.y += (random.random()*0.1 - 0.05) * self.wave_magnitude
		vel.z = math.cos(time.time() * 0.2) * self.wave_magnitude * 0.5

		#print(vel)
		self.wave_pub.publish(vel)

try:
	wave = WaveGen()
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		wave.update()
		rate.sleep()

except Exception as e:
	print(e)