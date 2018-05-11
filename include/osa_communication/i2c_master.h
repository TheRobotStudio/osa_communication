/*
 * Copyright (c) 2018, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
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
 * @file EPOSController.hpp
 * @author Cyril Jourdan
 * @date Apr 31, 2018
 * @version 0.1.0
 * @brief Header file for class EPOSController
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Apr 31, 2018
 */

#ifndef OSA_COMMUNICATION_I2C_MASTER_H
#define OSA_COMMUNICATION_I2C_MASTER_H

#include <ros/ros.h>
#include "osa_msgs/MotorCmdMultiArray.h"
#include "osa_msgs/MotorDataMultiArray.h"
#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <cstring>
#include <string>
#include "osa_communication/i2c_slave.h"
#include "osa_common/enums.h"

namespace osa_communication
{

/**
 * @brief Class
 */
class I2CMaster
{
public:

	/**
	 * @brief Constructor.
	 */
	I2CMaster(std::string i2c_device_name);

	/**
	 * @brief Destructor.
	 */
	~I2CMaster();

	/**
	 * @brief Initialize the ROS node.
	 * @return bool Returns true if the initialization has completed successfully and false otherwise.
	 */
	bool init();

	/**
	 * @brief Run the ROS node.
	 * @return void
	 */
	void run();

	void addI2CSlave(int slave_addr, bool inverted);

	/**
	 *  @brief Callback method that receives EPOS commands and that fits it into the right frame format to be sent over I2C bus.
	 *  @param motor_cmd_array Motor command multi-array.
	 *  @return void
	 */
	void sendMotorCmdMultiArrayCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& motor_cmd_array);

private:
	std::string i2c_device_name_;
	std::vector<I2CSlave*> i2c_slave_list_;
	ros::Subscriber motor_cmd_sub_;
	bool motor_cmd_array_received_;
};

} // osa_communication

#endif // OSA_COMMUNICATION_I2C_MASTER_H
