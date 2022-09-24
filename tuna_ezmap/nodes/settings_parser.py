#!/usr/bin/env python3

import rospy
import time

from std_msgs.msg import Header, String, Float32

class Parser:
	def __init__(self):
		rospy.init_node('settings_parser')
		self.max_speed_pub = rospy.Publisher("diff_drive/max_speed", Float32, queue_size=1)
		self.settingsub = rospy.Subscriber("/web_settings", String, self.settings)

	def settings(self, msg):
		json = msg.data

		if not "\"linear\"" in json:
			return

		split = json.split(",")
		for x in split:
			if "\"linear\"" in x:
				linear = float(x.replace("\"","").split(":")[1])
				self.max_speed_pub.publish(linear)

try:
	parser = Parser()
	rospy.spin()
except Exception as e:
	print(e)
