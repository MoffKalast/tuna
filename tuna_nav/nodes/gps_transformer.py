#!/usr/bin/env python3

import rospy
import time
import tf

from mpmath import mp

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import Imu, MagneticField
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix

mp.dps = 30

m1 = mp.mpf(111132.95255)
m2 = mp.mpf(559.84957)
m3 = mp.mpf(1.17514)
m4 = mp.mpf(-0.00230)
p1 = mp.mpf(111412.87733)
p2 = mp.mpf(-93.50412)
p3 = mp.mpf(0.11774)
p4 = mp.mpf(-0.000165)

class GPSTF:
	def __init__(self):
		rospy.init_node('gps_tf')

		self.init_x = mp.mpf(0)
		self.init_y = mp.mpf(0)

		self.x = mp.mpf(0)
		self.y = mp.mpf(0)

		self.fix_sub = rospy.Subscriber("/fix", NavSatFix, self.fix)
		self.fix_pub = rospy.Publisher('/fix/tf', Point, queue_size=1)

		self.first_msg = True


	def init(self, msg):
		self.init_x = self.x
		self.init_y = self.y

	def fix(self, msg):

		lat = mp.mpf(msg.latitude)
		lon = mp.mpf(msg.longitude)

		m_per_deg_lat = mp.fadd(
			m1, 
			mp.fadd( 
				mp.fmul(m2,mp.cos(mp.fmul(mp.mpf(2),lat))), 
				mp.fadd(
					mp.fmul(m3,mp.cos(mp.fmul(mp.mpf(4),lat))),
					mp.fmul(m4,mp.cos(mp.fmul(mp.mpf(6),lat)))
				)
			)
		)

		m_per_deg_lon = mp.fadd(
			mp.fmul(p1,mp.cos(lat)), 
			mp.fadd( 
				mp.fmul(p2,mp.cos(mp.fmul(mp.mpf(3),lat))), 
				mp.fmul(p3,mp.cos(mp.fmul(mp.mpf(5),lat)))
			)
		)

		self.y = mp.fmul(lat,m_per_deg_lat)
		self.x = mp.fmul(lon,m_per_deg_lon)

		if self.first_msg:
			self.init_x = self.x
			self.init_y = self.y
			self.first_msg = False

		out = Point()
		out.x = mp.fsub(self.x,self.init_x)
		out.y = mp.fsub(self.y,self.init_y)
		self.fix_pub.publish(out)


try:
	b = GPSTF()
	rospy.spin()
except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
