#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool, String

class LightIndicator:
	def __init__(self):
		rospy.init_node('light_nav_indicator')

		self.light_pub = rospy.Publisher("safety_light", Bool, queue_size=1)
		self.route_sub = rospy.Subscriber("route_status", String, self.route_callback)

		self.light_state = False

	def route_callback(self, msg):
		state = msg.data != "stop"
		
		if state != self.light_state:
			self.light_state = state
			self.light_pub.publish(self.light_state)


try:
	l = LightIndicator()
	rospy.spin()
except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
