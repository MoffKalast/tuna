#!/usr/bin/env python3

import numpy as np
import rospy
import time
import math
import random
import tf

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_conjugate, quaternion_inverse

def clamp(num, min, max):
    return min if num < min else max if num > max else num


class WindGen:
	def __init__(self):
		rospy.init_node('gazebo_wave_generator')

		self.wave_magnitude = 0.3
		self.wave_pub = rospy.Publisher("ocean_current", Vector3, queue_size=1)
		self.marker_pub = rospy.Publisher("wave_arrows", MarkerArray, queue_size=1)

		rospy.loginfo("Wave Generator started")

		self.vel = Vector3()
		self.vel.x = 0.0
		self.vel.y = 0.0

		self.dir_x = 0.0
		self.dir_y = 0.0

	def update(self):

		self.dir_x = self.dir_x * 0.97 + (random.random()*0.1 - 0.05) * 0.03
		self.dir_y = self.dir_y * 0.97 + (random.random()*0.1 - 0.05) * 0.03

		self.vel.x = clamp(self.vel.x + self.dir_x, -0.35, 0.35)
		self.vel.y = clamp(self.vel.y - self.dir_x, -0.35, 0.35)
		self.wave_pub.publish(self.vel)

		rotation = quaternion_from_euler(0,0,math.atan2(self.vel.y, self.vel.x))

		markerArray = MarkerArray()

		marker_id = 0;
		for i in range(-2,12,2):
			for j in range(-2,12,2):
				arrow = Marker()
				arrow.header.frame_id = "map"
				arrow.type = arrow.ARROW
				arrow.pose.position.x = i
				arrow.pose.position.y = j
				arrow.pose.position.z = 0.0

				arrow.pose.orientation.x = rotation[0]
				arrow.pose.orientation.y = rotation[1]
				arrow.pose.orientation.z = rotation[2]
				arrow.pose.orientation.w = rotation[3]

				arrow.scale.x = math.sqrt(self.vel.x*self.vel.x + self.vel.y*self.vel.y)*1.5
				arrow.scale.y = arrow.scale.x*0.25
				arrow.scale.z = arrow.scale.x*0.25

				arrow.color.a = 0.2

				arrow.color.r = 0.4
				arrow.color.g = 0.4
				arrow.color.b = 1.0

				arrow.id = marker_id
				markerArray.markers.append(arrow)

				marker_id +=1

		self.marker_pub.publish(markerArray)

try:
	wave = WindGen()
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		wave.update()
		rate.sleep()

	wave.vel.x = 0
	wave.vel.y = 0
	wave.wave_pub.publish(wave.vel)
	wave.marker_pub.publish(MarkerArray())

except Exception as e:
	print(e)