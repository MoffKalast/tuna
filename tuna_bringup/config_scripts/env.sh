#!/bin/sh

ip="$(hostname -I)"

while [ -z $ip ]
do
	sleep 1
	ip="$(hostname -I)"
done

#export ROS_IP=192.168.1.14
#export ROS_HOSTNAME=$(hostname).local
#export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311

export ROS_IP="$(echo $ip | xargs)"
export ROS_HOSTNAME=$ROS_IP
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
