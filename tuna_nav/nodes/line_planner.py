#!/usr/bin/env python3

import rospy
import math
import tf2_ros
import tf
import copy

import numpy as np

from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion

def clamp(num, min, max):
    return min if num < min else max if num > max else num


def project_position(start, end, current, mindist, maxdist):
	
	def unit_vector(vector):
		return vector / np.linalg.norm(vector)
	
	def point_to_np(point):
		return np.array([point.x, point.y])

	def np_to_point(np_array):
		return Point(np_array[0], np_array[1], 0)

	start_pos = point_to_np(start)
	end_pos = point_to_np(end)
	current_pos = point_to_np(current)

	line = end_pos - start_pos
	unit_line = unit_vector(line)
	
	current_to_start = current_pos - start_pos
	projection_length = np.dot(current_to_start, unit_line)
	projection = start_pos + unit_line * projection_length

	if np.dot(projection - start_pos, start_pos - end_pos) > 0:
		projection = start_pos

	deltadist = current_pos - projection 
	distance = mindist + (maxdist - mindist) * clamp(1.0 - np.sqrt(deltadist.dot(deltadist))/1.5,0.0, 1.0)

	new_pos = projection + unit_vector(end_pos - start_pos) * distance

	if np.dot(new_pos - end_pos, end_pos - start_pos) > 0:
		new_pos = end_pos

	return np_to_point(new_pos)

def get_dir(from_vec, to_vec):
	target_direction = [
		to_vec.x - from_vec.x,
		to_vec.y - from_vec.y,
	]
	target_distance = math.sqrt(target_direction[0] ** 2 + target_direction[1] ** 2)

	return [target_direction[0] / target_distance, target_direction[1] / target_distance]

class PID:
	def __init__(self, kp, ki, kd, target=0, windup_guard=20.0):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.target = target

		self.windup_guard = windup_guard

		self.prev_error = 0.0
		self.integral = 0.0
		self.derivative = 0.0

	def reset(self):
		self.prev_error = 0.0
		self.integral = 0.0
		self.derivative = 0.0

	def compute(self, current_value):
		error = self.target - current_value

		self.integral += error
		self.integral = max(min(self.integral, self.windup_guard), -self.windup_guard)

		self.derivative = error - self.prev_error

		output = self.kp * error + self.ki * self.integral + self.kd * self.derivative

		self.prev_error = error

		return output

class LineFollowingController:
	def __init__(self):
		rospy.init_node("line_following_controller")

		self.tf_listener = tf.TransformListener()

		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		self.marker_pub = rospy.Publisher("goal_markers", MarkerArray, queue_size=1)

		self.start_position = Point(0.0, 0.0, 0.0)
		self.end_position = Point(5.0, 5.0, 0.0)

		self.pid = PID(0.8, 0.0001, 0.8)

		self.print_index = 0

	def create_marker(self, position, marker_id, r, g, b):
		marker = Marker()
		marker.header.frame_id = "map"
		marker.type = marker.SPHERE
		marker.pose.position = position
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3
		marker.color.a = 0.5
		marker.color.r = r
		marker.color.g = g
		marker.color.b = b
		marker.id = marker_id
		return marker


	def update(self):
		
		try:
			
			(trans, rot) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
			_, _, current_yaw = euler_from_quaternion(rot)
			current_position = Point(*trans)

			target_position = project_position(
				self.start_position,
				self.end_position,
				current_position,
				0.1,#0.75
				1.5

			)
			
			target_unit_vector = get_dir(current_position, target_position)
			current_unit_vector = [math.cos(current_yaw), math.sin(current_yaw)]

			dot_product = target_unit_vector[0] * current_unit_vector[0] + target_unit_vector[1] * current_unit_vector[1]
			cross_product = current_unit_vector[0] * target_unit_vector[1] - current_unit_vector[1] * target_unit_vector[0]

			angle_error = -math.acos(dot_product) * math.copysign(1, cross_product)
			angular_velocity = self.pid.compute(angle_error)

			deltax = self.end_position.x - current_position.x
			deltay = self.end_position.y - current_position.y

			target_distance = math.sqrt(deltax** 2 + deltay ** 2)

			self.print_index = (self.print_index +1)%15

			if target_distance > 0.6:
				linear_velocity = clamp(0.55 - math.fabs(angle_error) * 0.3, 0.0, 0.45)
			else:
				linear_velocity = 0
				angular_velocity = 0
				self.start_position, self.end_position = self.end_position, self.start_position

			if self.print_index == 0:
				print("Angle:",angle_error,"Vel:",linear_velocity, "Tgt:",target_distance)

			twist = Twist()
			twist.linear.x = linear_velocity
			twist.angular.z = angular_velocity

			self.cmd_vel_pub.publish(twist)

			markerArray = MarkerArray()
			markerArray.markers.append(self.create_marker(self.start_position, 0, 1.0, 0.0, 0.0))
			markerArray.markers.append(self.create_marker(self.end_position, 1, 0.0, 0.0, 1.0))
			markerArray.markers.append(self.create_marker(target_position, 2, 0.0, 1.0, 0.0))
			self.marker_pub.publish(markerArray)


		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn("TF Exception")


ctrl = LineFollowingController()
rate = rospy.Rate(30)

while not rospy.is_shutdown():
	ctrl.update()
	rate.sleep()