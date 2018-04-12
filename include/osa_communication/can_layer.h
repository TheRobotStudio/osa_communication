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
 * @file can_layer.h
 * @author Cyril Jourdan <cyril.jourdan@therobotstudio.com>
 * @date Modified on Apr 3, 2018
 * @date Created on Aug 3, 2017
 * @version 0.1.1
 * @brief Header file for class CANLayer
 */

#ifndef OSA_COMMUNICATION_CAN_LAYER_H
#define OSA_COMMUNICATION_CAN_LAYER_H

#include <ros/ros.h>
#include "osa_msgs/MotorCmdMultiArray.h"
#include "osa_msgs/MotorDataMultiArray.h"
#include "osa_common/robot_description.h"
#include <can_msgs/Frame.h>

#include "epos_controller.h"
//ROS services
#include "osa_communication/SelectMotorController.h"

namespace osa_communication
{

/**
 * @brief Class representing a CAN communication layer.
 */
class CANLayer
{
public:

	/**
	 * @brief Constructor.
	 */
	CANLayer();

	/**
	 * @brief Destructor.
	 */
	~CANLayer();

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

	/**
	 *  @brief Callback method that receives CAN messages from the CAN bus and updates motor data.
	 *  @param can_msg CAN message.
	 *  @return void
	 */
	void receiveCANMessageCallback(const can_msgs::FrameConstPtr& can_msg);

	/**
	 *  @brief Callback method that receives EPOS commands and that fits it into the right frame format to be sent over CAN bus.
	 *  @param motor_cmd_array Motor command multi-array.
	 *  @return void
	 */
	void sendMotorCmdMultiArrayCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& motor_cmd_array);

	/**
	 *  @brief Service server method that setup a given motor controller.
	 *  @param req the request is the node ID on the CAN bus.
	 *  @param res the response is a bool which has the same value as the returned value.
	 *  @return bool whether or not the service call has succedded.
	 */
	bool ssSetupMotorController(osa_communication::SelectMotorController::Request  &req, osa_communication::SelectMotorController::Response &res);

	/**
	 *  @brief Service server method that initialize a given motor controller.
	 *  @param req the request is the node ID on the CAN bus.
	 *  @param res the response is a bool which has the same value as the returned value.
	 *  @return bool whether or not the service call has succedded.
	 */
	bool ssInitMotorController(osa_communication::SelectMotorController::Request  &req, osa_communication::SelectMotorController::Response &res);

private:
	const static int data_length = 8;

	osa_common::RobotDescription *ptr_robot_description_;

	//std::string robot_namespace_;
	//std::string robot_name_;
	//std::string robot_can_device_;
	std::vector<EPOSController*> epos_controller_list_;
	//int number_epos_boards_; /**< Size of epos_controller_list_ */

	char data_[data_length];
	ros::Subscriber rx_can_frame_sub_;
	ros::Subscriber motor_cmd_sub_;
	int* ptr_socket_can_; /**< To link it to EPOSController. */
	ros::Publisher pub_motor_data_;
	bool motor_cmd_array_received_;
	ros::ServiceServer ss_setup_motor_controller_;
	ros::ServiceServer ss_init_motor_controller_;
};

} // osa_communication

#endif // OSA_COMMUNICATION_CAN_LAYER_H
