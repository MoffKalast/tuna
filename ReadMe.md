# Tuna ROS Boat

## Installation

[Install pigpio](https://abyz.me.uk/rpi/pigpio/download.html)

Apt deps:

`
sudo apt install ros-noetic-nmea-navsat-driver
`

Fix services to set up correct ROS params:

`
cd ~/catkin_ws/src/tuna/tuna_bringup/systemd
sudo cp roscore.service /etc/systemd/system/roscore.service
sudo cp magni-base /usr/sbin/magni-base
sudo cp roscore /usr/sbin/roscore
sudo cp env.sh /etc/ubiquity/env.sh
`


#### Enable Kernel Interfaces

For the safety_light to have the correct access create `/etc/udev/rules.d/99-gpio.rules` with the following contents:

` 
SUBSYSTEM=="bcm2835-gpiomem", KERNEL=="gpiomem", GROUP="gpio", MODE="0660"
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", RUN+="/bin/sh -c 'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", RUN+="/bin/chown root:gpio /sys%p/active_low /sys%p/edge /sys%p/direction /sys%p/value", RUN+="/bin/chmod 660 /sys%p/active_low /sys%p/edge /sys%p/direction /sys%p/value"
` 



