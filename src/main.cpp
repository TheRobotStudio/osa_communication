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
 * @file main.cpp
 * @author Cyril Jourdan
 * @date Aug 29, 2017
 * @version 0.0.1
 * @brief main function for the Open Source Android CAN layer node.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Aug 3, 2017
 */

#include "can_layer.h"

/*! \fn int main(int argc, char **argv)
 *  \brief main function for the Open Source Android CAN layer node.
 *
 *  This is the main function.
 */
int main(int argc, char **argv)
{
	//ros::init(argc, argv, "osa_can_layer_node");
	//ros::Rate r(15);

	CANLayer *can_layer = new CANLayer(); //ros::this_node::getName());
	if(can_layer->init()) return -1;
	//ros::spin();

/*
	ROS_INFO("*** Grab parameters ***\n");

	// Parameters
	//Robot
	std::string robot_name_str;
	int robot_dof_i;
	std::string robot_can_device_str;
	//Mobile base
	std::string mobile_base_str = "no_mobile_base";
	//Controllers
	//std::vector<Controller> v_controllers;


	// motor1 - right wheel
	std::string controller1_type_str;
	std::string motor1_type_str;
	bool motor1_inverted_bool;
	std::string mode1_str;
	int value1_int;
	//motor2 - left wheel
	std::string controller2_type_str;
	std::string motor2_type_str;
	bool motor2_inverted_bool;
	std::string mode2_str;
	int value2_int;
*/

	//Main loop
	while(ros::ok())
	{
		//getData();
		//ros::spinOnce(); //receive CAN frame : motor data and emergency frames; receive motor commands, publish motor data

		//r.sleep();
	}

	return 0;
}

/*! \fn int main(int argc, char** argv)
 *  \brief main function.
 *  \param argc
 *  \param argv
 *  \return int
 */
/*
int main2(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "osa_socketcan_node");
	ros::NodeHandle nh("~");

	ros::Rate r(15); //15

	ROS_INFO("***** OSA_CAN Layer - The Robot Studio *****\n");

	ROS_INFO("*** Init variables ***\n");
*/
	/*! Variables */
	//ros::Publisher txFrame_pub;/*! Variables */
	//motorArray[NUMBER_OF_MOTORS] = {0, NOT_USED, NONE, false, PROFILE_VELOCITY_MODE, 0};
	//canFrame[NUMBER_OF_MOTORS];
	//numberEposBoards = 0;

	//DigitalOut ledchain[] = {(LED1), (LED2), (LED3), (LED4)}; //used for debugging
	//char data[8];
/*
	activMode[NUMBER_OF_MOTORS] = {PROFILE_VELOCITY_MODE}; //
	rosCmd[NUMBER_OF_MOTORS] = {SEND_DUMB_MESSAGE}; //

	//latest stored commands
	//RPDO1
	targetPosition[NUMBER_OF_MOTORS] = {0}; //int32_t
	targetVelocity[NUMBER_OF_MOTORS] = {0}; //int32_t
	//RPDO2
	profileAcceleration[NUMBER_OF_MOTORS] = {0}; //uint32_t
	profileDeceleration[NUMBER_OF_MOTORS] = {0}; //uint32_t
	//RPDO3
	profileVelocity[NUMBER_OF_MOTORS] = {0}; //uint32_t
	outputCurrentLimit[NUMBER_OF_MOTORS] = {0}; //uint16_t
	currentModeSettingValue[NUMBER_OF_MOTORS] = {0}; //int16_t
	//RPDO4
	maximalSpeedInCurrentMode[NUMBER_OF_MOTORS] = {0}; //uint32_t
	controlword[NUMBER_OF_MOTORS] = {0}; //uint16_t
	//RPDO4 and TPDO3
	modesOfOperation[NUMBER_OF_MOTORS] = {0}; //int8_t

	//sensor variables
	//TPDO1
	position[NUMBER_OF_MOTORS] = {0}; //int32_t
	velocity[NUMBER_OF_MOTORS] = {0}; //int32_t
	//TPDO2
	current[NUMBER_OF_MOTORS] = {0}; //int16_t
	followingError[NUMBER_OF_MOTORS] = {0}; //int16_t
	statusword[NUMBER_OF_MOTORS] = {0}; //uint16_t
	//TPDO3 see modesOfOperation above
	//TPDO4
	incEnc1CntAtIdxPls[NUMBER_OF_MOTORS] = {0}; //uint32_t

	boardStatus[NUMBER_OF_MOTORS] = {0}; //0 = OK, 1 = fault //int8_t
*/

/*
	//configureSlaveBoard - fill motorArray
	motorArray[0].nodeID = 1;
	motorArray[0].controllerType = EPOS4;
	motorArray[0].motorType = EC90;
	motorArray[0].inverted = true;
	motorArray[0].mode = PROFILE_VELOCITY_MODE;
	motorArray[0].value = 0;

	motorArray[1].nodeID = 2;
	motorArray[1].controllerType = EPOS4;
	motorArray[1].motorType = EC90;
	motorArray[1].inverted = false;
	motorArray[1].mode = PROFILE_VELOCITY_MODE;
	motorArray[1].value = 0;
*/

/*
	//Main loop
	while(ros::ok())
	{
		getData();
		ros::spinOnce(); //receive CAN frame : motor data and emergency frames; receive motor commands, publish motor data

		r.sleep();
	}

*/
//}


