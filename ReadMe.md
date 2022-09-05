# Tuna ROS Boat

## Installation

[Install pigpio](https://abyz.me.uk/rpi/pigpio/download.html)

Install iris_lama and required ezmap packages.

Apt deps:

```
sudo apt install ros-noetic-nmea-navsat-driver
```

Fix services to set up correct ROS params:

```
cd ~/catkin_ws/src/tuna/tuna_bringup/config_scripts
sudo cp roscore.service /etc/systemd/system/roscore.service

sudo cp magni-base /usr/sbin/magni-base
sudo cp roscore /usr/sbin/roscore

sudo cp env.sh /etc/ubiquity/env.sh
sudo cp ros_setup.bash /etc/ubiquity/ros_setup.bash
sudo cp ros_setup.sh /etc/ubiquity/ros_setup.sh
```

Changes to `/boot/config.txt` for i2c and uart and power saving:

```
# disable rainbow splash screen for faster booting
disable_splash=1

# Set up UART and disable BT
dtoverlay=disable-bt
dtoverlay=uart2

# Set up I2C
dtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,bus=1 core_freq=250

# Disable the PWR LED
dtparam=pwr_led_trigger=none
dtparam=pwr_led_activelow=off

# Disable the Activity LED
dtparam=act_led_trigger=none
dtparam=act_led_activelow=off

# Disable ethernet port LEDs
dtparam=eth_led0=4
dtparam=eth_led1=4
```

#### Enable Kernel Interfaces

For the `safety_light` to have the correct access create `/etc/udev/rules.d/99-gpio.rules` with the following contents:

```
SUBSYSTEM=="bcm2835-gpiomem", KERNEL=="gpiomem", GROUP="gpio", MODE="0660"
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", RUN+="/bin/sh -c 'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", RUN+="/bin/chown root:gpio /sys%p/active_low /sys%p/edge /sys%p/direction /sys%p/value", RUN+="/bin/chmod 660 /sys%p/active_low /sys%p/edge /sys%p/direction /sys%p/value"
```



