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
 * @file epos_controller.cpp
 * @author Cyril Jourdan <cyril.jourdan@therobotstudio.com>
 * @date Mar 30, 2018
 * @version 0.1.0
 * @brief Implementation file for class I2CSlave
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Aug 3, 2017
 */

#include "i2c_slave.h"

using namespace std;
using namespace osa_communication;

I2CSlave::I2CSlave(std::string i2c_device_name, int slave_addr, bool inverted) :
i2c_device_name_(i2c_device_name),
slave_addr_(slave_addr),
buffer_({0}),
fd_(0),
motor_enc_pos_(0),
set_motor_pos_(0),
inverted_(inverted)
{
	fd_ = open(i2c_device_name.c_str(), O_RDWR);

	if(fd_ < 0)
	{
		ROS_ERROR("Error opening file: %s\n", strerror(errno));
		//return 1;
	}

	if(ioctl(fd_, I2C_SLAVE, slave_addr_) < 0)
	{
		ROS_ERROR("ioctl error: %s\n", strerror(errno));
		//return 1;
	}
}

/**
 * @brief Destructor.
 */
I2CSlave::~I2CSlave()
{

}

void I2CSlave::setMotorPosition(int position)
{
	if(inverted_) position = -1*position;
	
	buffer_[3] = (char)position;
        buffer_[2] = (char)(position >> 8);
        buffer_[1] = (char)(position >> 16);
        buffer_[0] = (char)(position >> 24);

	ROS_INFO("set_pos(%02X)=[%02X %02X %02X %02X]", slave_addr_, buffer_[3], buffer_[2], buffer_[1], buffer_[0]);

	write(fd_, buffer_, BYTE_NUMBER);
}

void I2CSlave::requestMotorPosition()
{
	//for(int i=0; i<BYTE_NUMBER; i++) buffer_[i]=0xFF;
	//read(fd_, buffer_, BYTE_NUMBER);

	char buff[BYTE_NUMBER] = {0x00};
	read(fd_, buff, BYTE_NUMBER);

	motor_enc_pos_ = buff[3] + (buff[2] << 8) + (buff[1] << 16) + (buff[0] << 24);
	
	if(inverted_) motor_enc_pos_ = -1*motor_enc_pos_;

	ROS_INFO("get_pos(%02X)=[%02X %02X %02X %02X]=%d", slave_addr_, buff[3], buff[2], buff[1], buff[0], motor_enc_pos_);
	//ROS_DEBUG("pos(%d)=%d", slave_addr_, motor_enc_pos_);
}

