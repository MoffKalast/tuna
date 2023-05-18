#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool

class LightIndicator:
	def __init__(self):
		rospy.init_node('nav_indicator_light')
		self.light_state = False
		self.light_pub = rospy.Publisher("safety_light", Bool, queue_size=1)
		self.route_sub = rospy.Subscriber("line_planner/active", Bool, self.nav_callback)
		
	def nav_callback(self, msg):
		if msg.data != self.light_state:
			self.light_state = msg.data
			self.light_pub.publish(msg.data)


try:
	l = LightIndicator()
	rospy.spin()
except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
