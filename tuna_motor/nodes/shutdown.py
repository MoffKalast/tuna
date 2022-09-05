#!/usr/bin/env python3

import rospy
import os
from sensor_msgs.msg import BatteryState

class Shutdown:
	def __init__(self):
		rospy.init_node('shutdown')
		self.batt = rospy.Subscriber("/battery_state", BatteryState, self.battery)

		self.running = False
		self.warning = False
		self.shutdown = False
		self.percentage = 1.0

	def battery(self, msg):
		if msg.percentage < 0.1:
			return

		self.percentage = self.percentage * 0.9 + msg.percentage * 0.1
		if self.percentage < 0.2:
			os.system('sudo poweroff')
		
try:
	s = Shutdown()
	rospy.spin()
except Exception as e:
	print(e)
