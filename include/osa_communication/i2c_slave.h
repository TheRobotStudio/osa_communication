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
 * @file i2c_slave.h
 * @author Cyril Jourdan
 * @date Apr 31, 2018
 * @version 0.1.0
 * @brief Header file for class EPOSController
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Apr 31, 2018
 */

#ifndef OSA_COMMUNICATION_I2C_SLAVE_H
#define OSA_COMMUNICATION_I2C_SLAVE_H

#include <ros/ros.h>

#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <cstring>
#include <string>

namespace osa_communication
{

/**
 * @brief Class
 */
class I2CSlave
{
public:
	/**
	 * @brief Constructor.
	 */
	I2CSlave(std::string i2c_device_name, int slave_addr, bool inverted);

	/** @brief Destructor. */
	~I2CSlave();

	int getMotorEncPosition() const { return motor_enc_pos_; };
	int getSlaveAddr() const { return slave_addr_; };

	void setMotorPosition(int position);
	void requestMotorPosition();

private:
	static const int BYTE_NUMBER = 4;	
	std::string i2c_device_name_;
	int slave_addr_;
	char buffer_[BYTE_NUMBER];
	int fd_; //file descriptor
	int motor_enc_pos_;
	int set_motor_pos_;
	bool inverted_; //inverted direction of rotation, use opposite values for commands
};

} //osa_communication

#endif // OSA_COMMUNICATION_I2C_SLAVE_H
