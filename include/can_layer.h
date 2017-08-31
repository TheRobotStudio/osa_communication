/*
 * Copyright (c) 2017, The Robot Studio
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
 * @file can_layer.h
 * @author Cyril Jourdan
 * @date Aug 29, 2017
 * @version 0.0.1
 * @brief Header file for class CANLayer
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Aug 3, 2017
 */

#ifndef CAN_LAYER_H
#define CAN_LAYER_H

//ROS includes
#include <ros/ros.h>
#include "osa_msgs/MotorCmdMultiArray.h"
#include "osa_msgs/MotorDataMultiArray.h"
#include <can_msgs/Frame.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_interface/socketcan.h>
#include "epos_controller.h"

#define DATA_LENGTH 8

class CANLayer
{
	//methods
	public:
		//constructor
		CANLayer();
		//destructor
		~CANLayer();

		bool init();
		void run();

		//setters

		//getters

		/*! methods */
		void receiveMessagesCallback(const can_msgs::FrameConstPtr& can_msg);
		void sendMotorCmdMultiArrayCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& motor_cmd_ma);
		//void resetMotorCmdMultiArray();

		void display();

	//attributes
	protected:
		std::string robot_name_;
		std::string robot_can_device_;
		std::vector<EPOSController*> epos_controllers_vp_;
		int number_epos_boards_; //size of epos_controllers_vp_
		char data_[DATA_LENGTH];
		ros::Subscriber rx_can_frame_sub_;
		ros::Subscriber motor_cmd_sub_;
		ros::Publisher *tx_can_frame_pub_; //to link it to EPOSController
		ros::Publisher motor_data_pub_;
		bool motor_cmd_received_;
};

#endif /* CAN_LAYER_H */
