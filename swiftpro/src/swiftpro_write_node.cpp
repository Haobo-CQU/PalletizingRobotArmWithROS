/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>	   
 */
 // Modified in 2020 by YuanHaobo  <yhb0521@126.com>
#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <swiftpro/SwiftproState.h>
#include <swiftpro/status.h>
#include <swiftpro/position.h>

using namespace std;

serial::Serial _serial;				// serial object
swiftpro::SwiftproState pos;
string speed = " F200";


/* 
 * Description: callback when receive data from position_write_topic
 * Inputs: 		msg(float)			3 cartesian coordinates: x, y, z(mm)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void position_write_callback(const swiftpro::position& msg)
{
	std::string Gcode = "";
	std_msgs::String result;
	char x[10];
	char y[10];
	char z[10];

	pos.x = msg.x;
	pos.y = msg.y;
	pos.z = msg.z;
	speed = msg.speed;
	sprintf(x, "%.2f", msg.x);
	sprintf(y, "%.2f", msg.y);
	sprintf(z, "%.2f", msg.z);
	Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + speed + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from swiftpro_status_topic
 * Inputs: 		msg(uint8)			status of motor: attach if 1; detach if 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void swiftpro_status_callback(const swiftpro::status& msg)
{
	std::string Gcode = "";
	std_msgs::String result;

	if (msg.status == 1)
		Gcode = (std::string)"M17\r\n";   // attach
	else if (msg.status == 0)
		Gcode = (std::string)"M2019\r\n";
	else
	{
		ROS_INFO("Error:Wrong swiftpro status input");
		return;
	}
	
	pos.swiftpro_status = msg.status;
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from gripper_topic
 * Inputs: 		msg(uint8)			status of gripper: work if 1; otherwise 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void gripper_callback(const swiftpro::status& msg)
{
	std::string Gcode = "";
	std_msgs::String result;

	if (msg.status == 1)
		Gcode = (std::string)"M2232 V1" + "\r\n";
	else if (msg.status == 0)
		Gcode = (std::string)"M2232 V0" + "\r\n";
	else
	{
		ROS_INFO("Error:Wrong gripper status input");
		return;
	}
	
	pos.gripper = msg.status;
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Node name:
 *	 swiftpro_write_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *	 swiftpro_state_topic
 *   serial_return_topic
 *
 * Topic subscribe: (queue size = 1)
 *	 position_write_topic
 *	 swiftpro_status_topic
 *	 gripper_topic
 */
int main(int argc, char** argv)
{	
	ros::init(argc, argv, "swiftpro_write_node");
	ros::NodeHandle nh;
	swiftpro::SwiftproState swiftpro_state;
	std_msgs::String serial_result;//data back from serial
	swiftpro::status serial_return_status;//if data above is "@9 V0", then it is 1. Otherwise it is 0.
	serial_return_status.status = 1;
	

	ros::Subscriber sub1 = nh.subscribe("/position_write_topic", 1, position_write_callback);
	ros::Subscriber sub2 = nh.subscribe("/swiftpro_status_topic", 1, swiftpro_status_callback);
	// ros::Subscriber sub3 = nh.subscribe("/angle4th_topic", 1, angle4th_callback);
	ros::Subscriber sub4 = nh.subscribe("/gripper_topic", 1, gripper_callback);
	// ros::Subscriber sub5 = nh.subscribe("/pump_topic", 1, pump_callback);
	ros::Publisher  pub  = nh.advertise<swiftpro::SwiftproState>("/SwiftproState_topic", 1);
	ros::Publisher  serial_return_pub = nh.advertise<swiftpro::status>("/serial_return_topic",1);
	ros::Rate loop_rate(20);

	try
	{
		_serial.setPort("/dev/ttyACM0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		ROS_INFO_STREAM("Port has been open successfully by write_node");
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port by write_node");
		return -1;
	}
	

	if (_serial.isOpen())
	{
		ros::Duration(3.5).sleep();				// wait 3.5s
		// _serial.write("M2120 V0\r\n");			// stop report position
		// ros::Duration(0.1).sleep();				// wait 0.1s
		_serial.write("M2122 V1\r\n");			// Start to report "@9 V0" when movement is finished
		ros::Duration(0.1).sleep();				// wait 0.1s
		_serial.write("M17\r\n");				// attach motors
		ros::Duration(0.1).sleep();				// wait 0.1s

		ROS_INFO_STREAM("Write_node: Attach motors and Start to report");
		
	}

	while (ros::ok())
	{
		pub.publish(pos);//swiftpro::SwiftproState pos;
		int serial_result_length = 0;
		if (_serial.available())
		{
			serial_result.data = _serial.read(_serial.available());
			int serial_result_length = serial_result.data.length();
			if (serial_result_length/3 > 0 || serial_result_length/3 < 5)
			{
				// serial_return_status.status = 1;
				serial_return_pub.publish(serial_return_status);
			}

			// else
			// {
			// 	serial_return_status.status = 0;
			// 	serial_return_pub.publish(serial_return_status);
			// }

		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}







                       //////////////////////////////////////
                       ///////////  Abandon Code   //////////
                       //////////////////////////////////////

// #include <swiftpro/angle4th.h>


/* 
 * Description: callback when receive data from angle4th_topic
 * Inputs: 		msg(float)			angle of 4th motor(degree)
 * Outputs:		Gcode				send gcode to control swift pro
 */
// void angle4th_callback(const swiftpro::angle4th& msg)
// {
// 	std::string Gcode = "";
// 	std_msgs::String result;
// 	char m4[10];
	
// 	pos.motor_angle4 = msg.angle4th;
// 	sprintf(m4, "%.2f", msg.angle4th);
// 	Gcode = (std::string)"G2202 N3 V" + m4 + "\r\n";
// 	ROS_INFO("%s", Gcode.c_str());
// 	_serial.write(Gcode.c_str());
// 	result.data = _serial.read(_serial.available());
// }



/* 
 * Description: callback when receive data from pump_topic
 * Inputs: 		msg(uint8)			status of pump: work if 1; otherwise 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
// void pump_callback(const swiftpro::status& msg)
// {
// 	std::string Gcode = "";
// 	std_msgs::String result;

// 	if (msg.status == 1)
// 		Gcode = (std::string)"M2231 V1" + "\r\n";
// 	else if (msg.status == 0)
// 		Gcode = (std::string)"M2231 V0" + "\r\n";
// 	else
// 	{
// 		ROS_INFO("Error:Wrong pump status input");
// 		return;
// 	}
	
// 	pos.pump = msg.status;
// 	ROS_INFO("%s", Gcode.c_str());
// 	_serial.write(Gcode.c_str());
// 	result.data = _serial.read(_serial.available());
// }


