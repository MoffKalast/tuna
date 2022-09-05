#!/usr/bin/env python3

import rospy
import smbus
import time

from sensor_msgs.msg import BatteryState

#http://www.smartypies.com/projects/ads1115-with-raspberrypi-and-python/ads1115runner/

#http://www.ti.com/lit/ds/symlink/ads1115.pdf page 28

#000 : FSR = 6.144 V
#001 : FSR = 4.096 V
#010 : FSR = 2.048 V (default)
#011 : FSR = 1.024 V
#100 : FSR = 0.512 V
#101 : FSR = 0.256 V
#110 : FSR = 0.256 V
#111 : FSR = 0.256 V

#000 : AINP = AIN0 and AINN = AIN1 (default)
#001 : AINP = AIN0 and AINN = AIN3
#010 : AINP = AIN1 and AINN = AIN3
#011 : AINP = AIN2 and AINN = AIN3
#100 : AINP = AIN0 and AINN = GND
#101 : AINP = AIN1 and AINN = GND
#110 : AINP = AIN2 and AINN = GND
#111 : AINP = AIN3 and AINN = GND

LIPO= [
	3.27, # 0
	3.61, # 5
	3.69, # 10
	3.71, # 15
	3.73, # 20
	3.75, # 25
	3.77, # 30
	3.79, # 35
	3.80, # 40
	3.82, # 45
	3.84, # 50
	3.85, # 55
	3.87, # 60
	3.91, # 65
	3.95, # 70
	3.98, # 75
	4.02, # 80
	4.08, # 85
	4.11, # 90
	4.15, # 95
	4.20, # 100
]

LION= [
	3.3, # 0
	3.35,
	3.49,
	3.50,
	3.53,
	3.54,
	3.55,
	3.58,
	3.60,
	3.62,
	3.64,
	3.67,
	3.70,
	3.75,
	3.80,
	3.88,
	3.85,
	3.95,
	4.05,
	4.10,
	4.20, # 100
]

ADDRESS = 0x48
POINTER_CFG = 0x1
POINTER_CONV = 0x0

def swap2Bytes(c):
    '''Revert Byte order for Words (2 Bytes, 16 Bit).'''
    return (c>>8 |c<<8)&0xFFFF

def prepareLEconf(BEconf):
	'''Prepare LittleEndian Byte pattern from BigEndian configuration string, with separators.'''
	c = int(BEconf.replace('-',''), base=2)
	return swap2Bytes(c)

def LEtoBE(c):
	'''Little Endian to BigEndian conversion for signed 2Byte integers (2 complement).'''
	c = swap2Bytes(c)
	if(c >= 2**15):
		c= c-2**16
	return c

class Battery:
	def __init__(self):
		rospy.init_node('battery')
		self.pub = rospy.Publisher('/battery_state', BatteryState, queue_size=1)
		self.bus = smbus.SMBus(1)

	def clamp(self, val, minval, maxval):
		return max(min(val, maxval), minval)

	def capacity(self, voltage, cells, type):
		onecell = voltage / cells

		if onecell >= type[20]:
			return 1.0
		elif onecell <= type[0]:
			return 0.0

		upper = 0
		lower = 0

		for i in range(0,len(type)):
			if onecell > type[i]:
				lower = i
			else:
				upper = i
				break

		deltavoltage = type[upper] - type[lower]
		between_percent = (onecell - type[lower]) / deltavoltage

		upper = float(upper) * 5.0
		lower = float(lower) * 5.0

		return (lower + (upper-lower) * between_percent) / 100

	def read(self):
		try:
			# start single conversion - AIN0/GND - 4.096V - single shot - 8SPS - X - X - X - disable comparator

			conf = prepareLEconf('1-100-001-1-000-0-0-0-11')

			self.bus.write_word_data(ADDRESS, POINTER_CFG, conf)

			rospy.sleep(0.2)

			value_raw = self.bus.read_word_data(ADDRESS, POINTER_CONV)
			value = LEtoBE(value_raw)
			value = value * 0.0004504553 + 1.58421

			battery_msg = BatteryState()
			battery_msg.voltage = value

			battery_msg.current = float('nan')
			battery_msg.current = float('nan')

			battery_msg.percentage = self.clamp(self.capacity(value, 3, LIPO) + 0.03, 0.0, 1.0)
			battery_msg.charge = 5.0 * battery_msg.percentage
			battery_msg.capacity = 5.0
			battery_msg.design_capacity = 5.0
			battery_msg.present = True
			battery_msg.cell_voltage = [value/3.0, value/3.0, value/3.0]

			battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
			battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
			battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO

			self.pub.publish(battery_msg)

		except:
			pass
try:

	b = Battery()
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		b.read()
		rate.sleep()

except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
