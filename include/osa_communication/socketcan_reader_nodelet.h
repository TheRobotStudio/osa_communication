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
 * @file socketcan_reader_nodelet.h
 * @author Cyril Jourdan
 * @date Mar 29, 2018
 * @version 0.1.0
 * @brief Header file for class SocketCANReaderNodelet
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 21, 2018
 */

#ifndef OSA_COMMUNICATION_SOCKETCAN_READER_NODELET_H
#define OSA_COMMUNICATION_SOCKETCAN_READER_NODELET_H

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <can_msgs/Frame.h>
#include "registers.h"
#include <boost/thread.hpp>
#include "osa_common/robot_description.h"

namespace osa_communication_nodelet
{

/**
 * @brief Class representing a ROS nodelet for reading CAN frames with SocketCAN.
 */
class SocketCANReaderNodelet : public nodelet::Nodelet
{
public:
	/**
	 * @brief Constructor.
	 */
	SocketCANReaderNodelet();

	/** @brief Destructor. */
	~SocketCANReaderNodelet();

	virtual void onInit();

private:
	int open_port(const char *port);
	int send_port(struct can_frame *frame);
	void read_port(); //thread
	int close_port();

	osa_common::RobotDescription *ptr_robot_description_;
	/*
	std::string robot_namespace_;
	std::string robot_name_;
	std::string robot_can_device_;
	int number_epos_boards_;
	*/
	int soc_;
	bool read_can_port_;
	ros::Publisher pub_rx_can_frame_;
	boost::shared_ptr<boost::thread> socketcan_thread_;
};

} // osa_communication_nodelet

PLUGINLIB_DECLARE_CLASS(osa_communication, SocketCANReaderNodelet, osa_communication_nodelet::SocketCANReaderNodelet, nodelet::Nodelet);

#endif // OSA_COMMUNICATION_SOCKETCAN_READER_NODELET_H
