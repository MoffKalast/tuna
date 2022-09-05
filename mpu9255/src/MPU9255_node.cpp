#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <err.h>
#include <errno.h>

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <string>
#include "std_msgs/String.h"
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include "sensor_msgs/MagneticField.h"

using namespace std;

//string i2cDeviceName_0 = "/dev/i2c-0";
string i2cDeviceName_1 = "/dev/i2c-1";
//int file_i0;
int file_i1;

#define MPU9250_ADDRESS            0x68
#define MAG_ADDRESS                0x0C

#define GYRO_FULL_SCALE_250_DPS    0x00  
#define GYRO_FULL_SCALE_500_DPS    0x08
#define GYRO_FULL_SCALE_1000_DPS   0x10
#define GYRO_FULL_SCALE_2000_DPS   0x18

#define ACC_FULL_SCALE_2_G        0x00  
#define ACC_FULL_SCALE_4_G        0x08
#define ACC_FULL_SCALE_8_G        0x10
#define ACC_FULL_SCALE_16_G       0x18

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file, I2C_SMBUS, &args);
}

static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file, I2C_SMBUS_READ, command,	I2C_SMBUS_BYTE_DATA, &data))
		return -1;
	else
		return 0x0FF & data.byte;
}

static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command, uint8_t data_in)
{
	union i2c_smbus_data data;
	data.byte = data_in;
	
	if (i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data))
		return -1;
	else
		return 0x0FF & data.byte;
}

uint8_t i2c_read(uint8_t dev_addr, uint8_t reg)
{
	int rc;
	uint8_t read_back;

	rc = ioctl(file_i1, I2C_SLAVE, dev_addr); // Sets the device address
	if (rc < 0)
	{
		ROS_INFO("ERROR - couldn't set device address");
		return false;
	}

	// Actually perform the write and then read back immediately
	read_back = i2c_smbus_read_byte_data(file_i1, reg);
	//ROS_INFO("read_back = 0x%x", read_back);
	
	return read_back;
}

uint8_t i2c_write(uint8_t dev_addr, uint8_t reg, uint8_t data_in)
{
	int rc;
	uint8_t read_back;

	rc = ioctl(file_i1, I2C_SLAVE, dev_addr); // Sets the device address
	if (rc < 0)
	{
		ROS_INFO("ERROR - couldn't set device address");
		return false;
	}

	// Actually perform the write and then read back immediately
	read_back = i2c_smbus_write_byte_data(file_i1, reg, data_in);
	//ROS_INFO("read_back = 0x%x", read_back);
	
	return read_back;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "IMU_pub");
	
	ros::NodeHandle n(""), nh_param("~");
	
	bool print_min_max_mag, print_min_max_acc, print_rolling_mean_acc;
	nh_param.param("print_min_max_mag", print_min_max_mag, false);
	nh_param.param("print_min_max_acc", print_min_max_acc, false);
	nh_param.param("print_rolling_mean_acc", print_rolling_mean_acc, false);

	double imu_covar, mag_covar;
	nh_param.param("imu_covar", imu_covar, 0.01);
	nh_param.param("mag_covar", mag_covar, 0.01);

	double acc_bias_x, acc_bias_y, acc_bias_z;
	nh_param.param("acc_bias_x", acc_bias_x, 0.0);
	nh_param.param("acc_bias_y", acc_bias_y, 0.0);
	nh_param.param("acc_bias_z", acc_bias_z, 0.0);

	double gyr_bias_x, gyr_bias_y, gyr_bias_z;
	nh_param.param("gyr_bias_x", gyr_bias_x, 0.0);
	nh_param.param("gyr_bias_y", gyr_bias_y, 0.0);
	nh_param.param("gyr_bias_z", gyr_bias_z, 0.0);

	double mag_bias_x, mag_bias_y, mag_bias_z;
	nh_param.param("mag_bias_x", gyr_bias_x, 0.0);
	nh_param.param("mag_bias_y", gyr_bias_y, 0.0);
	nh_param.param("mag_bias_z", gyr_bias_z, 0.0);
	 
	ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("imu/data_raw", 2);
	ros::Publisher pub_mag = n.advertise<sensor_msgs::MagneticField>("imu/mag", 2);

	// I2C
	file_i1 = open(i2cDeviceName_1.c_str(), O_RDWR);
	if (file_i1 < 0)
		ROS_INFO("ERROR - i2c 0 file not open!");
		
	// Configure gyroscope range
	// default is - GYRO_FULL_SCALE_250_DPS
	i2c_write(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);

	// Configure accelerometers range
	// default is - ACC_FULL_SCALE_2_G
	i2c_write(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G); //ACC_FULL_SCALE_2_G

	// Set I2C bypass mode for the magnetometer
	i2c_write(MPU9250_ADDRESS,0x37,0x02);

	// Request first magnetometer single measurement
	i2c_write(MAG_ADDRESS,0x0A,0x01);

	double conversion_gyro = 3.1415/(180.0*82.0);  // From deg/s to rad/s. Then 32.8 conststant for 100DPS
	double conversion_acce = 0.001;//9.80665/2048.0f;  //9.8/16384.0f;
	double conversion_magno = 0.15;  // 0.6 uT / LSB  - why did you set 0.15 before?
 
	double min_mag[3] = {1.5, 3.6, -1.95};
	double max_mag[3] = {-21.9, 25.35, 19.65};

	double min_acc[3] = {9999.9, 9999.9, 9999.9};
	double max_acc[3] = {-9999.9, -9999.9, -9999.9};
	
	bool roll_mean_acc_ready = false;
	double roll_mean_acc[3] = {0};

	int16_t InBuffer[9] = {0}; 
	float OutBuffer[9] = {0};
	
	ros::Rate loop_rate(25); 

	while (ros::ok()){
		//http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html 
		//http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html
		sensor_msgs::Imu data_imu;    
		sensor_msgs::MagneticField data_mag;

		data_mag.header.stamp = ros::Time::now();
		data_imu.header.stamp = data_mag.header.stamp;
		data_imu.header.frame_id = "imu_link";

		//datos acelerómetro
		InBuffer[0] = (i2c_read(MPU9250_ADDRESS, 0x3B)<<8)|i2c_read(MPU9250_ADDRESS, 0x3C);
		InBuffer[1] = (i2c_read(MPU9250_ADDRESS, 0x3D)<<8)|i2c_read(MPU9250_ADDRESS, 0x3E);
		InBuffer[2] = (i2c_read(MPU9250_ADDRESS, 0x3F)<<8)|i2c_read(MPU9250_ADDRESS, 0x40);   
		
		//datos giroscopio
		InBuffer[3] = (i2c_read(MPU9250_ADDRESS, 0x43)<<8)|i2c_read(MPU9250_ADDRESS, 0x44);
		InBuffer[4] = (i2c_read(MPU9250_ADDRESS, 0x45)<<8)|i2c_read(MPU9250_ADDRESS, 0x46);
		InBuffer[5] = (i2c_read(MPU9250_ADDRESS, 0x47)<<8)|i2c_read(MPU9250_ADDRESS, 0x48); 
	
		//datos magnetómetro
		InBuffer[6] = (i2c_read(MAG_ADDRESS, 0x04)<<8)|i2c_read(MAG_ADDRESS, 0x03);
		InBuffer[7] = (i2c_read(MAG_ADDRESS, 0x06)<<8)|i2c_read(MAG_ADDRESS, 0x05);
		InBuffer[8] = (i2c_read(MAG_ADDRESS, 0x08)<<8)|i2c_read(MAG_ADDRESS, 0x07);
	
		// Request magnetometer single measurement
		i2c_write(MAG_ADDRESS, 0x0A, 0x01);
		
		// Swap X and Y axis and correct polarity so to align with magnetometer axis
		OutBuffer[0] = (InBuffer[0]*conversion_acce) + acc_bias_x;
		OutBuffer[1] = (InBuffer[1]*conversion_acce) + acc_bias_y;
		OutBuffer[2] = (InBuffer[2]*conversion_acce) + acc_bias_z;

		// Swap X and Y axis and correct polarity so to align with magnetometer axis
		OutBuffer[3] = (InBuffer[3]*conversion_gyro) + gyr_bias_x;
		OutBuffer[4] = (InBuffer[4]*conversion_gyro) + gyr_bias_y;
		OutBuffer[5] = (InBuffer[5]*conversion_gyro) + gyr_bias_z;
		
		// Mag is NED orientation - convert to ENU, swap XY and invert Z
		OutBuffer[6] = InBuffer[7]*conversion_magno;
		OutBuffer[7] = InBuffer[6]*conversion_magno;
		OutBuffer[8] = -InBuffer[8]*conversion_magno;

		for (int i = 0; i < 3; i++)
		{	
			if (OutBuffer[i] < min_acc[i])
				min_acc[i] = OutBuffer[i];
				
			if (OutBuffer[i] > max_acc[i])
				max_acc[i] = OutBuffer[i];	
		}
	
		data_imu.linear_acceleration.x = OutBuffer[0] - (min_acc[0] + max_acc[0]) / 2;
		data_imu.linear_acceleration.y = OutBuffer[1] - (min_acc[1] + max_acc[1]) / 2;
		data_imu.linear_acceleration.z = OutBuffer[2] - (min_acc[2] + max_acc[2]) / 2;

		data_imu.angular_velocity.x = OutBuffer[3];
		data_imu.angular_velocity.y = OutBuffer[4];
		data_imu.angular_velocity.z = OutBuffer[5];
		
		data_imu.orientation_covariance = {-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
		data_imu.linear_acceleration_covariance = {imu_covar, 0.0f, 0.0f, 0.0f, imu_covar, 0.0f, 0.0f, 0.0f, imu_covar};
		data_imu.angular_velocity_covariance = {imu_covar, 0.0f, 0.0f, 0.0f, imu_covar, 0.0f, 0.0f, 0.0f, imu_covar};

		for (int i = 0; i < 3; i++)
		{
			int out_buf_num = i + 6;
			if (min_mag[i] > OutBuffer[out_buf_num])
				min_mag[i] = OutBuffer[out_buf_num];
				
			if (max_mag[i] < OutBuffer[out_buf_num])
				max_mag[i] = OutBuffer[out_buf_num];
		}

		//ROS_INFO("min %f %f %f\n", min_mag[0], min_mag[1], min_mag[2]);
		//ROS_INFO("max %f %f %f\n", max_mag[0], max_mag[1], max_mag[2]);
		
		data_mag.magnetic_field.x = OutBuffer[6] - (min_mag[0] + max_mag[0]) / 2;
		data_mag.magnetic_field.y = OutBuffer[7] - (min_mag[1] + max_mag[1]) / 2;
		data_mag.magnetic_field.z = OutBuffer[8] - (min_mag[2] + max_mag[2]) / 2;//
		data_mag.magnetic_field_covariance = {mag_covar, 0.0f, 0.0f, 0.0f, mag_covar, 0.0f, 0.0f, 0.0f, mag_covar};
			
		pub_imu.publish(data_imu);
		pub_mag.publish(data_mag);
		
		if (print_rolling_mean_acc)
		{
			if (roll_mean_acc_ready)
			{
				for (int i = 0; i < 3; i++)
					roll_mean_acc[i] = (roll_mean_acc[i] + OutBuffer[i]) / 2;
					
				//ROS_INFO("X-Acc - Mean = %f", roll_mean_acc[0]);
				//ROS_INFO("Y-Acc - Mean = %f", roll_mean_acc[1]);
				//ROS_INFO("Z-Acc - Mean = %f", roll_mean_acc[2]);
				//ROS_INFO(" ");
			}
			else
			{
				for (int i = 0; i < 3; i++)
					roll_mean_acc[i] = OutBuffer[i];
					
				roll_mean_acc_ready = true;
			}
		}
	 
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
 }
 