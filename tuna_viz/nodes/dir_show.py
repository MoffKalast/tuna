#!/usr/bin/env python
from __future__ import print_function

import rospy
import time
import tf
import tf2_ros
import cv2
import math
import numpy as np

from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import Imu, MagneticField, Image
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix

from cv_bridge import CvBridge, CvBridgeError

def rotate_point(x, y, angle):
	nx = math.cos(angle) * x - math.sin(angle) * y
	ny = math.sin(angle) * x + math.cos(angle) * y
	return [nx, ny]

class DirShow:
	def __init__(self):
		rospy.init_node('dir_show')

		self.bridge = CvBridge()

		self.buffer = tf2_ros.Buffer(rospy.Time(30))
		self.listener = tf2_ros.TransformListener(self.buffer)
		self.old_listener = tf.TransformListener()

		self.button_init = rospy.Subscriber("/goalpub/init", Bool, self.init)
		self.marker_sub = rospy.Subscriber("/goalpub/marker", Marker, self.marker)

		self.gpsraw_sub = rospy.Subscriber("/fix/tf", Point, self.raw)
		self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu)

		self.compass_pub = rospy.Publisher('/debugview/compass', Image, queue_size=1)
		self.tf_pub = rospy.Publisher('/debugview/tf', Image, queue_size=1)

		self.background = cv2.cvtColor(cv2.imread('/home/ubuntu/catkin_ws/src/tuna/tuna_viz/nodes/compass.png'), cv2.COLOR_BGR2RGB)
		self.backgroundtf = cv2.cvtColor(cv2.imread('/home/ubuntu/catkin_ws/src/tuna/tuna_viz/nodes/tf.png'), cv2.COLOR_BGR2RGB)

		self.angle_imu = 0
		self.angle_gps = 0
		self.angle_raw = 0

		self.prev_x = 0
		self.prev_y = 0
		self.prev_angle = 0

		self.prev_rawx = 0
		self.prev_rawy = 0

		self.odom_x = 0
		self.odom_y = 0

		self.has_marker = False;
		self.marker_x = -100;
		self.marker_y = -100;

	def init(self, msg):
		if not msg.data:
			return

		self.has_marker = False;

	def marker(self, msg):
		self.marker_x = int(msg.pose.position.y * 25);
		self.marker_y = int(msg.pose.position.x * 25);
		self.has_marker = True;

	def raw(self, msg):
		x = msg.y - self.prev_rawx
		y = msg.x - self.prev_rawy

		length = math.sqrt(x*x + y*y)

		if length == 0:
			return

		self.angle_raw = math.atan2(x/length, y/length)
		self.prev_rawx = msg.y
		self.prev_rawy = msg.x

	def imu(self, msg):
		q = msg.orientation
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
		self.angle_imu = yaw

	def gps(self):
		try:
			pose = self.get_transform("base_link", "map")

			x = pose.translation.y - self.prev_x
			y = pose.translation.x - self.prev_y
			length = math.sqrt(x*x + y*y) 

			self.angle_gps = math.atan2(x/length, y/length)
			self.prev_x = pose.translation.y
			self.prev_y = pose.translation.x

		except Exception as e:
			print(e)

	def compass(self):
		img = self.background.copy()

		size = 100
		c = (size/2, size/2)
		half = size/2-10
		
		imu_pos = (math.sin(self.angle_imu)*half + c[0], math.cos(self.angle_imu)*half + c[0])
		gps_pos = (math.sin(self.angle_gps)*half + c[0], math.cos(self.angle_gps)*half + c[0])
		raw_pos = (math.sin(self.angle_raw)*half + c[0], math.cos(self.angle_raw)*half + c[0])
		  
		img = cv2.arrowedLine(img, c, (int(raw_pos[0]), int(raw_pos[1])), (50, 50, 255), 2) 
		img = cv2.arrowedLine(img, c, (int(gps_pos[0]), int(gps_pos[1])), (255, 0, 0), 2) 
		img = cv2.arrowedLine(img, c, (int(imu_pos[0]), int(imu_pos[1])), (0, 255, 0), 2) 

		self.compass_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))

	def tf(self):

		try:
			pose = self.get_transform("odom", "map")
			self.odom_x = pose.translation.y * 25
			self.odom_y = pose.translation.x * 25
		except Exception as e:
			print(e)

		img = self.backgroundtf.copy()

		width = 400
		height = 450
		c = (width/2, height/2)

		for i in range(0,16):
			img = cv2.line(img, (i*25,0), (i*25,height), (70, 120, 120), 1)

		for i in range(0,18):
			img = cv2.line(img, (0,i*25), (width,i*25), (70, 120, 120), 1)

		if self.has_marker:
			img = cv2.circle(img, (c[0] + self.marker_x, c[1] + self.marker_y), 25, (200,20,20), 1)

		img = cv2.line(img, c, (c[0]+25,c[1]), (10, 255, 10), 2)
		img = cv2.line(img, c, (c[0],c[1]+25), (255, 10, 10), 2)


		od = (c[0]+int(self.odom_x), c[1]+int(self.odom_y))
		img = cv2.line(img, od, (od[0]+25,od[1]), (10, 255, 10), 2)
		img = cv2.line(img, od, (od[0],od[1]+25), (255, 10, 10), 2)

		x = c[0] + self.prev_x*25
		y = c[1] + self.prev_y*25

		pts = np.array([
			rotate_point(-4,-10,-self.angle_imu),
			rotate_point(4,-10,-self.angle_imu),
			rotate_point(0,15,-self.angle_imu)
		], np.int32)

		for i in range(3):
			pts[i][0] += x
			pts[i][1] += y

		img = cv2.fillPoly(img, [pts], (255,255,255))

		self.tf_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))

	def update(self):
		self.gps()
		self.compass()
		self.tf()


	def get_transform(self, from_, to_):
		try:
			(trans, rot) = self.old_listener.lookupTransform(to_, from_, rospy.Time(0))
			form = Transform()
			form.translation.x = trans[0]
			form.translation.y = trans[1]
			form.translation.z = trans[2]
			form.rotation.x = rot[0]
			form.rotation.y = rot[1]
			form.rotation.z = rot[2]
			form.rotation.w = rot[3]
			return form
		except Exception as e:
			try:
				return self.buffer.lookup_transform(to_, from_, rospy.Time(0), rospy.Duration(1.0)).transform
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logerr("ROUTEMANAGER: Couldn't find transform: "+str(e))

		return -1


try:
	b = DirShow()
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		b.update()
		rate.sleep()


except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
