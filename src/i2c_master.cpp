/*
 * Copyright (c) 2019, The Robot Studio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.

 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file can_layer.cpp
 * @author Cyril Jourdan <cyril.jourdan@therobotstudio.com>
 * @date Modified on May 1, 2018
 * @date Created on Apr 31, 2018
 * @version 0.1.1
 * @brief Implementation file for the I2CMaster class
 */

#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "i2c_master.h"

#define NB_DOF 12

using namespace std;
namespace osa_communication
{

I2CMaster::I2CMaster(std::string i2c_device_name) :
i2c_device_name_(i2c_device_name),
i2c_slave_list_(),
motor_cmd_sub_(),
motor_cmd_array_received_(false)
{
}

I2CMaster::~I2CMaster()
{
	if(ros::isStarted())
	{
	  ros::shutdown(); // This is necessary after using ros::start().
	  ros::waitForShutdown();
	}
	wait();
}

bool I2CMaster::init()
{
	ROS_INFO("*** I2C Master Init ***\n");

	int init_argc = 0;
	char** init_argv = 0;
	ros::init(init_argc, init_argv, "osa_i2c_master_node");

	if(!ros::master::check())
	{
		return false;
	}

	ros::start(); //explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh("~");

	ROS_INFO("*** Init Publishers and Subsribers ***\n");
	//Subsribers and publishers
	motor_cmd_sub_ = nh.subscribe("/i2c/motor_cmd_array", 100, &I2CMaster::sendMotorCmdMultiArrayCallback, this);
	//pub_motor_data_ = nh.advertise<osa_msgs::MotorDataMultiArray>("/i2c/motor_data_array", 1); 

	//then start the run method with the main loop
	ROS_INFO("*** Start the run method with the main loop ***");
	run();

	return true;
}

void I2CMaster::run()
{
	ros::Rate r(50);

	//int idx = 8;
	int idx = 0;

	while(ros::ok())
	{
		//idx++;
		//if(idx == 20) idx = 0;
		
		ros::spinOnce();

		//get the sensor value of one node
		//i2c_slave_list_[idx]->requestMotorPosition();
		//publish data topic

		//if(motor_cmd_array_received_)
		//{
			i2c_slave_list_[idx]->requestMotorPosition();
			//motor_cmd_array_received_ = false;
		//}

		idx++;
		if(idx == NB_DOF) idx = 0;

		r.sleep();

		//insert little pause
		//ros::Duration(0, 100000).sleep();
	}
}

void I2CMaster::addI2CSlave(int slave_addr, bool inverted)
{
	I2CSlave* i2c_salve_ptr = new I2CSlave(i2c_device_name_, slave_addr, inverted);
	i2c_slave_list_.push_back(i2c_salve_ptr);	
}

void I2CMaster::sendMotorCmdMultiArrayCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& motor_cmd_array)
{	
	//toggle flag, a message has been received
	motor_cmd_array_received_ = true;

	ROS_DEBUG("motor_cmd_array_received_");

	for(int i=0; i<motor_cmd_array->layout.dim[0].stride; i++)
	//for(std::vector<int>::const_iterator it = motor_cmd_array->motor_cmd.begin(); it != motor_cmd_array->motor_cmd.end(); ++it)
	{
		uint8_t slave_addr = motor_cmd_array->motor_cmd[i].node_id;
		uint8_t command = motor_cmd_array->motor_cmd[i].command;
		int32_t value = motor_cmd_array->motor_cmd[i].value;

		//ROS_INFO("cmd[%d] val[%d]", command, value);

		//find the index in the array which correspond to the node-id
		//auto it = find_if(ptr_robot_description_->getControllerList().begin(), ptr_robot_description_->getControllerList().end(), [&node_id](const EPOSController& obj) {return obj.getNodeID() == node_id;});
		auto it = find_if(i2c_slave_list_.begin(), i2c_slave_list_.end(), [&slave_addr](const I2CSlave* obj)-> bool {return obj->getSlaveAddr() == slave_addr;});

		if(it != i2c_slave_list_.end())
		{
			// found element. it is an iterator to the first matching element.
			// get the index:
			auto index = std::distance(i2c_slave_list_.begin(), it);

			switch(command)
			{
				case SET_TARGET_POSITION:
				{
					ROS_INFO("SET_TARGET_POSITION: cmd[%d] val[%d]", command, value);
					i2c_slave_list_[index]->setMotorPosition(value);					

					break;
				}				

				case SEND_DUMB_MESSAGE:
				{
					ROS_DEBUG("SEND_DUMB_MESSAGE");
					break;
				}

				default:
				{
					//ROS_DEBUG("default");
					break;
				}
			}
		}
		    
		ros::Duration(0.00001).sleep();
	}

	//ROS_INFO("cmd[%d] val[%d]", motor_cmd_array->motor_cmd[0].command, motor_cmd_array->motor_cmd[0].value);
}

} //namespace
