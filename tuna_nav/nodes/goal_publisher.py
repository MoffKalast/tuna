#!/usr/bin/env python3

import rospy
import time
import tf
import actionlib

import tf2_ros
import tf2_geometry_msgs

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import Imu, MagneticField
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose, GeoPoint

def transform_xy(x, y, tf_buffer, from_frame, to_frame):
	pose_stamped = tf2_geometry_msgs.PoseStamped()
	pose_stamped.pose.position.x = x
	pose_stamped.pose.position.y = y
	pose_stamped.header.frame_id = from_frame
	pose_stamped.header.stamp = rospy.Time.now()

	try:
		# ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
		output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(0.1))
		return output_pose_stamped.pose.position

	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		raise

class GoalPub:
	def __init__(self):
		rospy.init_node('goal_publisher')

		self.tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf_buffer)

		self.is_init = False;
		self.is_send = False;
		self.is_stop = False;

		self.distance = 10.0;

		self.get_tf = tf.TransformListener()
		self.odom_tf = tf.TransformBroadcaster()

		self.dist_sub = rospy.Subscriber("/goalpub/distance", Float32, self.distmsg)

		self.light = rospy.Publisher("/safety_light", Bool, queue_size=1)
		self.info = rospy.Publisher("/info", String, queue_size=1, latch=True)

		self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self.fix)
		self.gps_sub = rospy.Subscriber("/imu/data", Imu, self.imu)

		self.orientation = Quaternion()
		self.position = GeoPoint()

		rospy.wait_for_service('datum')
		self.mandatum = rospy.ServiceProxy('datum', SetDatum)

		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		if self.client.wait_for_server():
			rospy.loginfo("Connection to Action Server established.")
		else:
			rospy.signal_shutdown("Cannot connect to Action Server.")

		self.button_init = rospy.Subscriber("/goalpub/init", Bool, self.init)
		self.button_send = rospy.Subscriber("/goalpub/send", Bool, self.send)
		self.button_stop = rospy.Subscriber("/goalpub/stop", Bool, self.stop)

		self.markerPub = rospy.Publisher('/goalpub/marker', Marker, queue_size=1)

		self.info.publish("Goalpub standing by!")
		self.light.publish(False)

	def distmsg(self, msg):
		self.distance = msg.data
		self.info.publish("Distance set to: "+str(self.distance))

	def init(self, msg):

		if msg.data and self.is_init:
			return

		if not msg.data and self.is_init:
			self.is_init = False
			return

		self.is_init = True

		origin = GeoPose()
		origin.position = self.position
		origin.orientation.x = 0
		origin.orientation.y = 0
		origin.orientation.z = 0.707
		origin.orientation.w = -0.707

		try:
			self.mandatum(origin)
			self.info.publish("Set origin to: "+str(self.position.latitude)+" "+str(self.position.longitude)+" "+str(self.orientation))
		except rospy.ServiceException as e:
			self.info.publish("Setting origin failed: %s"%e)
		

	def send(self, msg):
		if msg.data and self.is_send:
			return

		if not msg.data and self.is_send:
			self.is_send = False
			return

		self.light.publish(True)
		self.is_send = True
		self.is_stop = False

		#origin = transform_xy(0.0,0.0, self.tf_buffer, "base_link", "map")

		start = transform_xy(5.0,0.0, self.tf_buffer, "base_link", "map")
		p1 = transform_xy(5.0,5.0, self.tf_buffer, "base_link", "map")
		p2 = transform_xy(10.0,5.0, self.tf_buffer, "base_link", "map")
		p3 = transform_xy(10.0,-5.0, self.tf_buffer, "base_link", "map")
		p4 = transform_xy(5.0,-5.0, self.tf_buffer, "base_link", "map")

		self.send_goal(start.x,start.y)
		self.send_goal(p1.x,p1.y)
		self.send_goal(p2.x,p2.y)
		self.send_goal(p3.x,p3.y)
		self.send_goal(p4.x,p4.y)
		self.send_goal(start.x,start.y)

		self.light.publish(False)

	def stop(self, msg):
		if msg.data and self.is_stop:
			return

		if not msg.data and self.is_stop:
			self.is_stop = False
			return

		self.is_stop = True

		self.client.cancel_goal()

		self.info.publish("Stopped!")

	def send_goal(self, x, y):

		if self.is_stop:
			return

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y

		goal.target_pose.pose.orientation.x = 0
		goal.target_pose.pose.orientation.y = 0
		goal.target_pose.pose.orientation.z = 0
		goal.target_pose.pose.orientation.w = 0

		self.info.publish("Target goal sent, x="+str(x)+" y="+str(y))

		self.client.send_goal(goal)

		m = Marker()
		m.header.frame_id = "map"
		m.id = 0
		m.type = 2 # sphere
		m.pose.position.x = x
		m.pose.position.y = y
		m.scale.x = 2.0
		m.scale.y = 2.0
		m.scale.z = 2.0
		m.color.r = 1.0
		m.color.a = 1.0
		m.lifetime = rospy.Duration(0)
		self.markerPub.publish(m)

		while self.client.get_state() <= 1:
			rospy.sleep(1.0)

	def imu(self, msg):
		self.orientation = msg.orientation

	def fix(self, msg):
		self.position.latitude = msg.latitude 
		self.position.longitude = msg.longitude 
		self.position.altitude = 0


try:
	b = GoalPub()
	rospy.spin()
except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
