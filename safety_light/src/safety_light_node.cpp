#include "ros/ros.h"
#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "std_msgs/Bool.h"

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

bool do_blink = false;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
	g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
	int num_params = 0;
	if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
		num_params = params.size();
	if (num_params > 1)
	{
		std::string reason = params[1];
		ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
		g_request_shutdown = 1; // Set flag
	}

	result = ros::xmlrpc::responseInt(1, "", 0);
}

void blink_callback(const std_msgs::Bool::ConstPtr& msg)
{
	do_blink = msg->data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "safety_light", ros::init_options::NoSigintHandler);
	ros::NodeHandle n;
	ros::Subscriber blink_sub = n.subscribe("safety_light", 1, blink_callback);

	signal(SIGINT, mySigIntHandler);
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	// Export the desired pin by writing to /sys/class/gpio/export
	int fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/export");
		exit(1);
	}
	if (write(fd, "26", 2) != 2) {
		perror("Error writing to /sys/class/gpio/export");
		exit(1);
	}
	close(fd);

	ros::Duration(0.5).sleep();

	// Set the pin to be an output by writing "out" to /sys/class/gpio/gpio24/direction
	fd = open("/sys/class/gpio/gpio26/direction", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/gpio26/direction");
		exit(1);
	}
	if (write(fd, "out", 3) != 3) {
		perror("Error writing to /sys/class/gpio/gpio26/direction");
		exit(1);
	}
	close(fd);

	fd = open("/sys/class/gpio/gpio26/value", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/gpio26/value");
		exit(1);
	}

	write(fd, "1", 1);
	usleep(15000);
	write(fd, "0", 1);

	ros::Rate loop_rate(0.8);
	while (ros::ok() && !g_request_shutdown)
	{
		if(do_blink){
			ROS_INFO("1");

			write(fd, "1", 1);
			usleep(15000);
			write(fd, "0", 1);

			ROS_INFO("0");	
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	write(fd, "0", 1);
	close(fd);

	ROS_INFO("shutting down");

	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd == -1) {
		perror("Unable to open /sys/class/gpio/unexport");
		exit(1);
	}
	if (write(fd, "26", 2) != 2) {
		perror("Error writing to /sys/class/gpio/unexport");
		exit(1);
	}
	close(fd);

	ros::shutdown();
}
