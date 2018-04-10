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
 * @file can_layer.cpp
 * @author Cyril Jourdan <cyril.jourdan@therobotstudio.com>
 * @date Modified on Apr 10, 2018
 * @date Created on May 24, 2017
 * @version 0.1.1
 * @brief Implementation file for the CAN communication, using SocketCAN
 */

//Other includes
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include "registers.h"
#include "can_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "nodelet/loader.h"

#include <unistd.h>

//#define LOOP_RATE 15
#define CAN_FRAME_FIFO_SIZE_FACTOR 4 //(2 request + 2 answers) factor for FIFO size: example (2 motors*(2 request + 2 answers) = 8)

using namespace std;
using namespace osa_communication;
//Structure TODO replace with Controller class from osa_gui moved to a new package osa_common

CANLayer::CANLayer() :
robot_name_(""),
robot_can_device_(""),
epos_controller_list_(0),
number_epos_boards_(0),
data_ ({0}),
rx_can_frame_sub_(),
motor_cmd_sub_(),
ptr_socket_can_(nullptr),
pub_motor_data_(),
motor_cmd_array_received_(false)
{
}

CANLayer::~CANLayer()
{
	if(ros::isStarted())
	{
	  ros::shutdown(); // This is necessary after using ros::start().
	  ros::waitForShutdown();
	}
	wait();
}

bool CANLayer::init()
{
	ROS_INFO("*** CANLayer Init ***\n");

	int init_argc = 0;
	char** init_argv = 0;
	ros::init(init_argc, init_argv, "osa_can_layer_node");

	if(!ros::master::check())
	{
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh("~");

	// Grab the namespace parameter
	try
	{
		nh.param("robot_namespace", robot_namespace_, std::string("/my_robot_ns"));
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR(e.what());
		ROS_ERROR("Parameter robot_namespace didn't load correctly!");
		ROS_ERROR("Please check the name and try again.");

		return false;
	}

	ROS_INFO("*** Init Publishers and Subsribers ***\n");
	//Subsribers and publishers
	//ptr_pub_tx_can_frame_ = new ros::Publisher(nh.advertise<can_msgs::Frame>("/sent_messages", 1)); //("/sent_messages", 8, true)); //latch message
	//motor_cmd_sub_ = nh.subscribe("/motor_cmd_array", 1, &CANLayer::sendMotorCmdMultiArrayCallback, this); //receive commands here and translate them into CAN frames and send to /sent_messages
	//FIXME find the right size of the msg buffer, 4 for the 4 steps ? 100 ?
	motor_cmd_sub_ = nh.subscribe(robot_namespace_ + "/motor_cmd_array", 100, &CANLayer::sendMotorCmdMultiArrayCallback, this); //receive commands here and translate them into CAN frames and send to /sent_messages
	pub_motor_data_ = nh.advertise<osa_msgs::MotorDataMultiArray>(robot_namespace_ + "/motor_data_array", 1); //Publish the data received on /rx_can_frame

	ROS_INFO("*** Grab the parameters from the YAML file ***");

	// Grab the parameters
	try
	{
		//load robot parameters
		if(!nh.param(robot_namespace_ + "/robot/name", robot_name_, std::string(robot_namespace_ + "my_robot")))
		{
			ROS_WARN_STREAM("No " << robot_namespace_ << "/robot/name found in YAML config file");
		}

		if(!nh.param(robot_namespace_ + "/robot/dof", number_epos_boards_, int(0)))
		{
			ROS_WARN_STREAM("No " << robot_namespace_ << "/robot/dof found in YAML config file");
		}

		if(!nh.param(robot_namespace_ + "/robot/can_device", robot_can_device_, std::string("can0")))
		{
			ROS_WARN_STREAM("No " << robot_namespace_ << "/robot/can_device found in YAML config file");
		}

		ROS_INFO("Robot name=%s, dof=%d, can=%s", robot_name_.c_str(), number_epos_boards_, robot_can_device_.c_str());

		//load controllers parameters
		//Example:
		//dof1: {node_id: 1, name: 'right wheel', type: 'EPOS4', inverted: true, motor: 'EC90', mode: 'PROFILE_VELOCITY_MODE', value: 0}

		bool dof_exist = true;
		//start with controller 1
		int dof_idx = 1;
		std::string rad_str = robot_namespace_ + "dof"; //common radical name

		while(dof_exist)
		{
			//create the string "controller+index" to search for the controller parameter with that index number
			std::ostringstream dof_idx_path;
			dof_idx_path << rad_str << dof_idx;

			std::string absolute_str = "absolute_str";

			//ROS_INFO("string=%s", dof_idx_path.str().c_str());

			if(nh.searchParam(dof_idx_path.str(), absolute_str))
			{
				//ROS_INFO("%s found in YAML config file", dof_idx_path.str().c_str());
				//ROS_INFO("absolute_str = %s", absolute_str.c_str());

				//create variables to store the controller parameters:
				std:: string name;
				std:: string type;
				int node_id = 0;
				std:: string controller;
				std:: string motor;
				bool inverted;
				std:: string mode;
				int value;

				//grab the parameters of the current controller

				//name
				std::ostringstream name_path;
				name_path << absolute_str << "/name";
				if(!nh.getParam(name_path.str(), name))
				{
					ROS_ERROR("Can't grab param name for %s", dof_idx_path.str().c_str());
					return false;
				}

				//type
				std::ostringstream type_path;
				type_path << absolute_str << "/type";
				if(!nh.getParam(type_path.str(), type))
				{
					ROS_ERROR("Can't grab param type for %s", dof_idx_path.str().c_str());
					return false;
				}

				//node_id
				std::ostringstream node_id_path;
				node_id_path << absolute_str << "/node_id";
				if(!nh.getParam(node_id_path.str(), node_id))
				{
					ROS_ERROR("Can't grab param node_id for %s", dof_idx_path.str().c_str());
					return false;
				}

				//controller
				std::ostringstream controller_path;
				controller_path << absolute_str << "/controller";
				if(!nh.getParam(controller_path.str(), controller))
				{
					ROS_ERROR("Can't grab param controller for %s", dof_idx_path.str().c_str());
					return false;
				}

				//motor
				std::ostringstream motor_path;
				motor_path << absolute_str << "/motor";
				if(!nh.getParam(motor_path.str(), motor))
				{
					ROS_ERROR("Can't grab param motor for %s", dof_idx_path.str().c_str());
					return false;
				}

				//inverted
				std::ostringstream inverted_path;
				inverted_path << absolute_str << "/inverted";
				if(!nh.getParam(inverted_path.str(), inverted))
				{
					ROS_ERROR("Can't grab param inverted for %s", dof_idx_path.str().c_str());
					return false;
				}

				//mode
				std::ostringstream mode_path;
				mode_path << absolute_str << "/mode";
				if(!nh.getParam(mode_path.str(), mode))
				{
					ROS_ERROR("Can't grab param mode for %s", dof_idx_path.str().c_str());
					return false;
				}

				//value
				std::ostringstream value_path;
				value_path << absolute_str << "/value";
				if(!nh.getParam(value_path.str(), value))
				{
					ROS_ERROR("Can't grab param value for %s", dof_idx_path.str().c_str());
					return false;
				}

				//print the dof parameters
				ROS_INFO("%s : name[%s], type[%s], node_id[%d], controller[%s], motor[%s], inverted[%d], mode[%s], value[%d]", dof_idx_path.str().c_str(),
						name.c_str(), type.c_str(), node_id, controller.c_str(), motor.c_str(), inverted, mode.c_str(), value);

				//create a new EPOS controller
				EPOSController *epos_controller = new EPOSController(name, type, node_id, controller, motor, inverted, mode, value, ptr_socket_can_);

				//epos_controller->

				//v_controllers.push_back(controller);
				epos_controller_list_.push_back(epos_controller);

				//increment to search for the next controller
				dof_idx++;
			}
			else
			{
				dof_exist = false;
				//ROS_INFO("No more controllers found in YAML config file");
			}

			//dof_exist = false;
		}

		dof_idx--;
		if(number_epos_boards_ == dof_idx) ROS_INFO("Same number of DOF(%d) and controllers(%d) defined in the YAML config file!", number_epos_boards_, dof_idx);
		else
		{
			ROS_WARN("Not the same number of DOF(%d) and controllers(%d) defined in the YAML config file!", number_epos_boards_, dof_idx);
			throw 1;
		}

		ROS_INFO("Parameters loaded successfully!\n");
	}
	catch(ros::InvalidNameException const &e)
	{
		ROS_ERROR(e.what());
		ROS_ERROR("Parameters didn't load correctly!");
		ROS_ERROR("Please modify your YAML config file and try again.");

		return false;
	}

	//Create the SocketCAN
	ROS_INFO("*** Create the SocketCAN TX ***");
	
	int s;
	struct sockaddr_can addr;
	struct ifreq ifr;

	const char *ifname = robot_can_device_.c_str();

	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
			perror("Error while opening socket");
			return -1;
	}

	ptr_socket_can_ = &s;

	strcpy(ifr.ifr_name, ifname);
	ioctl(*ptr_socket_can_, SIOCGIFINDEX, &ifr);

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if(bind(*ptr_socket_can_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{

			perror("Error in socket bind");
			return -2;
	}
	//End Create the SocketCAN

	//Update the pointer of the epos_controller
	//for(auto it = epos_controller_list_.cbegin(); it != epos_controller_list_.cend(); ++it)
	for(const auto &epos_controller : epos_controller_list_)
	{
		epos_controller->setPtrSocketCAN(ptr_socket_can_);
	}

	ROS_INFO("*** Create the SocketCAN RX nodelet ***");
	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	std::string nodelet_name = "socketcan_reader_nodelet";//ros::this_node::getName();
	nodelet.load(nodelet_name, "osa_communication/SocketCANReaderNodelet", remap, nargv);

	//Subsriber, need the number of EPOS for the FIFO
	//TODO put the define as a static in the namespace
	rx_can_frame_sub_ = nh.subscribe("/rx_can_frame", number_epos_boards_*CAN_FRAME_FIFO_SIZE_FACTOR, &CANLayer::receiveCANMessageCallback, this);

	//Setup Motor Controllers
	ROS_INFO("Do you want to setup the motor controllers ? (y/n)");
	std::string answer_setup;
	std::cin >> answer_setup;

	if(answer_setup.compare("y") == 0)
	{
		ROS_INFO("*** Setup Motor Controllers ***");

		for(int i=0; i<number_epos_boards_; i++)
		{
			if(epos_controller_list_[i]->setup() != EPOS_OK)
			{
				ROS_ERROR("Setup error");
				return false; //exit the main function and return fault if a setup failed
			}
		}
	}
	else
	{
		ROS_INFO("No motor controllers setup will be done.");
	}

	//Setup Motor Controllers
	ROS_INFO("Do you want to initialize the motor controllers with values from the YAML config file ? (y/n)");
	std::string answer_init;
	std::cin >> answer_init;

	if(answer_init.compare("y") == 0)
	{
		ROS_INFO("*** Initialize Motor Controllers ***");

		for(int i=0; i<number_epos_boards_; i++)
		{
			if(epos_controller_list_[i]->initialize() == EPOS_ERROR)
			{
				ROS_ERROR("Initialization error, check the mode/value of your config file, only use Profile modes for EPOS2/EPOS4 and Current mode for EPOS2.");
				return false; //exit the main function and return fault if an initialization failed
			}
		}
	}
	else
	{
		ROS_INFO("No motor controllers initialization will be done.");
	}

	ROS_INFO("*** Getting Motor Data ***\n");
	//gather first pack of data
	//get the sensor values
	for(int i=0; i<number_epos_boards_; i++)
	{
		epos_controller_list_[i]->getData();
	}

	//Services
	CANLayer can_layer;
	ss_setup_motor_controller_ = nh.advertiseService("osa_communication/setup_motor_controller", &CANLayer::ssSetupMotorController, this);
	ss_init_motor_controller_ = nh.advertiseService("osa_communication/init_motor_controller", &CANLayer::ssSetupMotorController, this);

	//then start the run method with the main loop
	ROS_INFO("*** Start the run method with the main loop ***");
	run();

	return true;
}

void CANLayer::run()
{
	ros::Rate r(200);

	int idx = 0;

	while(ros::ok())
	{
		//get the sensor value of one node
		epos_controller_list_[idx]->getData(); //TODO make this automatic with a hearbeat from the CAN
		idx++;

		if(idx == number_epos_boards_) idx = 0;
		
		ros::spinOnce();

		r.sleep();

		//insert little pause
		//ros::Duration(0, 100000).sleep();
	}
}

void CANLayer::receiveCANMessageCallback(const can_msgs::FrameConstPtr& can_msg)
{
	uint64_t data = 0x0000000000000000; //64 bits
	uint8_t node_id = 0;
	int16_t cob_id = 0;

	//ROS_DEBUG("receiveCANMessageCallback");

	//ROS_DEBUG("Interrupt frame : [%02X] [%02X %02X %02X %02X %02X %02X %02X %02X]\n", can_msg->id, can_msg->data[7], can_msg->data[6], can_msg->data[5], can_msg->data[4], can_msg->data[3], can_msg->data[2], can_msg->data[1], can_msg->data[0]);

	node_id = 0x07F & can_msg->id;
	cob_id = 0x0F80 & can_msg->id;

	//ROS_DEBUG("node_id [%02X],  cobID [%02X], can_msg->dlc=%d", node_id, cob_id, can_msg->dlc);
	
	for(int i=0; i<can_msg->dlc; i++) //dlc=len
	{
		//ROS_DEBUG("for i=%d", i);
		data = data | (can_msg->data[i]<<i*8);
	}

	//ROS_DEBUG("data=%ld", data);
	
	//find the index in the array which correspond to the node-id
	auto it = find_if(epos_controller_list_.begin(), epos_controller_list_.end(), [&node_id](const EPOSController* obj)-> bool {return obj->getNodeID() == node_id;});

	if(it != epos_controller_list_.end())
	{
		// found element. it is an iterator to the first matching element.
		// get the index:
		auto index = std::distance(epos_controller_list_.begin(), it);

		//ROS_DEBUG("node_id=%d is at index=%d", node_id, index);

		//set the nodeID
		epos_controller_list_[index]->setNodeID(node_id); //FIXME probably not necessary !

		//check node_id first and that the command is an answer to a request (RTR=false)
		//if((node_id >= 1) && (node_id <= number_epos_boards_) && (can_msg->is_rtr == false))
		if(can_msg->is_rtr == false)
		{
			switch(cob_id)
			{
				case COB_ID_TRANSMIT_PDO_1_ENABLE : //getPositionVelocity
				{
					//ROS_DEBUG("TPDO1 NodeID[%d] COB-ID[%02X] RTR[%d] data[%d]\n", node_id, cob_id, can_msg->is_rtr, data);

					int32_t pos = (can_msg->data[3]<<24 | can_msg->data[2]<<16 | can_msg->data[1]<<8 | can_msg->data[0]); //0x00000000FFFFFFFF & data;
					int32_t vel = (can_msg->data[7]<<24 | can_msg->data[6]<<16 | can_msg->data[5]<<8 | can_msg->data[4]);

					//ROS_DEBUG("NodeID[%d] pos[%d] vel[%d]", node_id, pos, vel);

					if(epos_controller_list_[index]->getInverted() == true) //!< change sign
					{
						//ROS_DEBUG("idx[%d]node[%d] is inverted", index, node_id);

						epos_controller_list_[index]->setPosition(-1*pos);
						epos_controller_list_[index]->setVelocity(-1*vel);
					}
					else
					{
						//ROS_DEBUG("idx[%d]node[%d] is not inverted", index, node_id);
						
						epos_controller_list_[index]->setPosition(pos);
						epos_controller_list_[index]->setVelocity(vel);
					}

					//ROS_DEBUG("1 motor_data_array[%d] position[%d] current[%d] status[%d]", node_id, epos_controller_list_[index]->getPosition(), epos_controller_list_[index]->getCurrent(), epos_controller_list_[index]->getStatusword());

					break;
				}

				case COB_ID_TRANSMIT_PDO_2_ENABLE : //getCurrentFollErrStatusword
				{
					//ROS_DEBUG("TPDO2 NodeID[%d] COB-ID[%02X] RTR[%d] data[%d]\n", node_id, cob_id, can_msg->is_rtr, data);
					//ROS_DEBUG("IT[%02X] [%02X %02X]\n", can_msg->id, can_msg->data[5] ,  can_msg->data[4]);

					int16_t curr  = (can_msg->data[1]<<8 | can_msg->data[0]); //0x000000000000FFFF & data;
					int16_t foll_err = (can_msg->data[3]<<8 | can_msg->data[2]); //(0x00000000FFFF0000 & data) >> 16;
					uint16_t statwrd = (can_msg->data[5]<<8 | can_msg->data[4]); //(0x0000FFFF00000000 & data) >> 32;

					//ROS_DEBUG("NodeID[%d] curr[%d] foll_err[%d] statwrd[%d]", node_id, curr, foll_err, statwrd);

					if(epos_controller_list_[index]->getInverted() == true) curr = -1*curr; //change sign
					epos_controller_list_[index]->setCurrent(curr);
					epos_controller_list_[index]->setFollowingError(foll_err);
					epos_controller_list_[index]->setStatusword(statwrd);

					//ROS_DEBUG("TPDO2 Node-ID[%d] curr[%d] foll_err[%d] statwrd[%d]\n", node_id, curr, foll_err, statwrd);

					//ROS_DEBUG("2 motor_data_array[%d] position[%d] current[%d] status[%d]", node_id, epos_controller_list_[index]->getPosition(), epos_controller_list_[index]->getCurrent(), epos_controller_list_[index]->getStatusword());

					break;
				}

				case COB_ID_TRANSMIT_PDO_3_ENABLE : //getModesOfOperation
				{
					//ROS_DEBUG("TPDO3 Node ID : [%d], PDO COB-ID [%02X], data = %d\n", node_id, cobID, data);

					int8_t modes_of_op = 0x00000000000000FF & data;;

					epos_controller_list_[index]->setModesOfOperation(modes_of_op);

					break;
				}

				case COB_ID_TRANSMIT_PDO_4_ENABLE : //getIncEnc1CntAtIdxPls
				{
					//ROS_DEBUG("TPDO4 Node ID : [%d], PDO COB-ID [%02X], data = %d\n", node_id, cobID, data);

					uint32_t inc_enc1 = 0x00000000FFFFFFFF & data;;

					epos_controller_list_[index]->setIncEnc1CntAtIdxPls(inc_enc1);

					break;
				}

				case COB_ID_EMCY_DEFAULT : //Emergency frame
				{
					//ROS_DEBUG("Emergency frame, Node ID : [%d], PDO COB-ID [%02X], data = %02X\n", node_id, cobID, data);
					//ROS_DEBUG("EF [%02X][%02X %02X %02X %02X %02X %02X %02X %02X]\n", can_msg->id, can_msg->data[7], can_msg->data[6], can_msg->data[5], can_msg->data[4], can_msg->data[3], can_msg->data[2], can_msg->data[1], can_msg->data[0]);
					ROS_WARN("EF%02X-%02X%02X", can_msg->id, can_msg->data[1], can_msg->data[0]);
					//ledchain[1] = 1;
					//nh.logerror("Emergency frame");
					epos_controller_list_[index]->setBoardStatus(1);
					//first step : fault reset on controlword
					//ROS_DEBUG("Node %d - STEP 1 - faultResetControlword\n", node_id);

					//Debug for fault detection on brachii
					epos_controller_list_[index]->faultResetControlword();        //TODO replace with RPDO
					break;
				}

				case COB_ID_SDO_SERVER_TO_CLIENT_DEFAULT : //SDO Acknoledgement frame
				{
					int32_t reg_data = 0x00000000;
					reg_data = (int32_t)data;

					//ROS_DEBUG("Node %d - reg_data [%02X]\n", node_id, reg_data);

					if(reg_data == 0x00604060) //Controlword
					{
						if(epos_controller_list_[index]->getBoardStatus() == 1)
						{
							epos_controller_list_[index]->setBoardStatus(2);
							//second step : shutdown controlword
							//ROS_DEBUG("Node %d - STEP 2 - shutdownControlwordIT\n", node_id);
							epos_controller_list_[index]->shutdownControlwordIT(); //TODO replace with RPDO
						}
						else if(epos_controller_list_[index]->getBoardStatus() == 2)
						{
							epos_controller_list_[index]->setBoardStatus(3);
							//third step : Switch On & Enable Operation on Controlword
							//ROS_DEBUG("Node %d - STEP 3 - switchOnEnableOperationControlwordIT\n", node_id);
							epos_controller_list_[index]->switchOnEnableOperationControlwordIT(); //TODO replace with RPDO
						}
						else if(epos_controller_list_[index]->getBoardStatus() == 3)
						{
							epos_controller_list_[index]->setBoardStatus(4);
							//ask for statusword to check if the board has reset well
							//ROS_DEBUG("Node %d - STEP 4 - getStatusword\n", node_id);
							//TODO getStatusword(node_id);
						}
					}
					else if(reg_data == 0x0060414B) //read Statusword
					{
						//int32_t swData = 0x00000000;

						//ROS_DEBUG("Node %d - Statusword [%02X]\n", node_id, can_msg->data[4]);
						//ROS_DEBUG("Statusword frame : [%02X] [%02X %02X %02X %02X %02X %02X %02X %02X]\n", can_msg->id, can_msg->data[7], can_msg->data[6], can_msg->data[5], can_msg->data[4], can_msg->data[3], can_msg->data[2], can_msg->data[1], can_msg->data[0]);

						if(epos_controller_list_[index]->getBoardStatus() == 4)
						{
							//swData = data >> 32;
							int8_t fault = 0x00;
							fault = (can_msg->data[4] & 0x08) >> 3;

							if(fault == 0) //reset OK
							{
								epos_controller_list_[index]->setBoardStatus(0); //Board is reset and enable OK
								ROS_INFO("%d OK", node_id);
								//ledchain[1] = 0;
							}
							else //try to reset again
							{
								//ROS_DEBUG("Node %d - try to reset again\n", node_id);
								epos_controller_list_[index]->setBoardStatus(1);
								//go back to first step : fault reset on controlword
								//ROS_DEBUG("Node %d - STEP 1 - faultResetControlword\n", node_id);
								epos_controller_list_[index]->faultResetControlword();       //TODO replace with RPDO
							}
						}
					}
					break;
				}
				default :
				{
					ROS_WARN("Unknown frame [%02X][%02X %02X %02X %02X %02X %02X %02X %02X]", can_msg->id, can_msg->data[7], can_msg->data[6], can_msg->data[5], can_msg->data[4], can_msg->data[3], can_msg->data[2], can_msg->data[1], can_msg->data[0]);
				}
			} //end switch

			//publish data
			osa_msgs::MotorDataMultiArray motor_data_array;

			//create the data multi array
			motor_data_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
			motor_data_array.layout.dim[0].size = epos_controller_list_.size();
			motor_data_array.layout.dim[0].stride = epos_controller_list_.size();
			motor_data_array.layout.dim[0].label = "epos";
			motor_data_array.layout.data_offset = 0;
			motor_data_array.motor_data.clear();
			motor_data_array.motor_data.resize(epos_controller_list_.size());

			for(int i=0; i<epos_controller_list_.size(); i++)
			{
				//ROS_DEBUG("3 motor_data_array[%d] position[%d] current[%d] status[%d]", i+1, epos_controller_list_[i]->getPosition(), epos_controller_list_[i]->getCurrent(), epos_controller_list_[i]->getStatusword());

				motor_data_array.motor_data[i].node_id = epos_controller_list_[i]->getNodeID();
				motor_data_array.motor_data[i].position = epos_controller_list_[i]->getPosition();
				motor_data_array.motor_data[i].current = epos_controller_list_[i]->getCurrent();
				motor_data_array.motor_data[i].status = epos_controller_list_[i]->getStatusword();

				//TODO add other data
			}

			//ROS_INFO("Publish motor data\n");
			pub_motor_data_.publish(motor_data_array);
		}
		else
		{
			//TODO check those node ID, make a mechanism to tell a frame has been received and put back the while loop that waits for it in the other EPOSController functions
			//ROS_WARN("NODEID ERROR\n");
		}
	} // if index found
}

void CANLayer::sendMotorCmdMultiArrayCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& motor_cmd_array)
{	
	//toggle flag, a message has been received
	motor_cmd_array_received_ = true;

	ROS_DEBUG("motor_cmd_array_received_");

	for(int i=0; i<epos_controller_list_.size(); i++)
	//for(std::vector<int>::const_iterator it = motor_cmd_array->motor_cmd.begin(); it != motor_cmd_array->motor_cmd.end(); ++it)
	{
		uint8_t node_id = motor_cmd_array->motor_cmd[i].node_id;
		uint8_t command = motor_cmd_array->motor_cmd[i].command;
		int32_t value = motor_cmd_array->motor_cmd[i].value;

/*		uint8_t node_id = it.node_id;
		uint8_t command = it.command;
		int32_t value = it.value;
*/
		//ROS_INFO("cmd[%d] val[%d]", command, value);

		//find the index in the array which correspond to the node-id
		//auto it = find_if(epos_controller_list_.begin(), epos_controller_list_.end(), [&node_id](const EPOSController& obj) {return obj.getNodeID() == node_id;});
		auto it = find_if(epos_controller_list_.begin(), epos_controller_list_.end(), [&node_id](const EPOSController* obj)-> bool {return obj->getNodeID() == node_id;});

		if(it != epos_controller_list_.end())
		{
			// found element. it is an iterator to the first matching element.
			// get the index:
			auto index = std::distance(epos_controller_list_.begin(), it);

			switch(command)
			{
				case SET_TARGET_POSITION:
				{
					//ROS_INFO("SET_TARGET_POSITION");
					ROS_INFO("SET_TARGET_POSITION: cmd[%d] val[%d]", command, value);
					epos_controller_list_[index]->setTargetPosition(value);
					break;
				}

				case SET_TARGET_VELOCITY:
				{
					ROS_INFO("SET_TARGET_VELOCITY: cmd[%d] val[%d]", command, value);
					epos_controller_list_[index]->setTargetVelocity(value);
					break;
				}

				case SET_PROFILE_ACCELERATION:
				{
					epos_controller_list_[index]->setProfileAcceleration(value);
					break;
				}

				case SET_PROFILE_DECELERATION:
				{
					epos_controller_list_[index]->setProfileDeceleration(value);
					break;
				}

				case SET_PROFILE_VELOCITY:
				{
					epos_controller_list_[index]->setProfileVelocity(value);
					break;
				}

				case SET_OUTPUT_CURRENT_LIMIT:
				{
					epos_controller_list_[index]->setOutputCurrentLimit(value);
					break;
				}

				case SET_CONTROLWORD:
				{
					epos_controller_list_[index]->setControlword(value);
					break;
				}

				case SET_CURRENT_MODE_SETTING_VALUE:
				{
					ROS_INFO("SET_CURRENT_MODE_SETTING_VALUE: cmd[%d] val[%d]", command, value);
					epos_controller_list_[index]->setCurrentModeSettingValue(value);
					break;
				}

				case SET_MAXIMAL_SPEED_IN_CURRENT_MODE:
				{
					epos_controller_list_[index]->setMaximalSpeedInCurrentMode(value);
					break;
				}

				case SET_MODES_OF_OPERATION:
				{
					switch(value)
					{
						case INTERPOLATED_POSITION_MODE:
						{
							epos_controller_list_[index]->setModesOfOperation(VALUE_INTERPOLATED_POSITION_MODE);
							break;
						}

						case PROFILE_VELOCITY_MODE:
						{
							epos_controller_list_[index]->setModesOfOperation(VALUE_PROFILE_VELOCITY_MODE);
							break;
						}

						case PROFILE_POSITION_MODE:
						{
							epos_controller_list_[index]->setModesOfOperation(VALUE_PROFILE_POSITION_MODE);
							break;
						}

						case POSITION_MODE:
						{
							epos_controller_list_[index]->setModesOfOperation(VALUE_POSITION_MODE);
							break;
						}

						case VELOCITY_MODE:
						{
							epos_controller_list_[index]->setModesOfOperation(VALUE_VELOCITY_MODE);
							break;
						}

						case CURRENT_MODE:
						{
							epos_controller_list_[index]->setModesOfOperation(VALUE_CURRENT_MODE);
							break;
						}

						case CYCLIC_SYNCHRONOUS_TORQUE_MODE:
						{
							epos_controller_list_[index]->setModesOfOperation(VALUE_CYCLIC_SYNCHRONOUS_TORQUE_MODE);
							break;
						}

						default:
						{
							break;
						}
					}
					//setModesOfOperation(i, value);
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
		    
		ros::Duration(0.00001).sleep(); //10us
	}

	//ROS_INFO("cmd[%d] val[%d]", motor_cmd_array->motor_cmd[0].command, motor_cmd_array->motor_cmd[0].value);
}

bool CANLayer::ssSetupMotorController(osa_communication::SelectMotorController::Request  &req, osa_communication::SelectMotorController::Response &res)
{
	uint8_t node_id = req.node_id;

	//Search the array for the index that has the requested nodeID
	auto it = find_if(epos_controller_list_.begin(), epos_controller_list_.end(), [&node_id](const EPOSController* obj)-> bool {return obj->getNodeID() == node_id;});

	if(it != epos_controller_list_.end())
	{
		// found element. it is an iterator to the first matching element.
		// get the index:
		auto index = std::distance(epos_controller_list_.begin(), it);

		if(epos_controller_list_[index]->setup() != EPOS_OK)
		{
			ROS_ERROR("setup[%d] error", req.node_id);
			res.success = false;

			return false;
		}
	}

	res.success = true;

	return true;
}

bool CANLayer::ssInitMotorController(osa_communication::SelectMotorController::Request  &req, osa_communication::SelectMotorController::Response &res)
{
	uint8_t node_id = req.node_id;

	//Search the array for the index that has the requested nodeID
	auto it = find_if(epos_controller_list_.begin(), epos_controller_list_.end(), [&node_id](const EPOSController* obj)-> bool {return obj->getNodeID() == node_id;});

	if(it != epos_controller_list_.end())
	{
		// found element. it is an iterator to the first matching element.
		// get the index:
		auto index = std::distance(epos_controller_list_.begin(), it);

		if(epos_controller_list_[index]->initialize() != EPOS_OK)
		{
			ROS_ERROR("initialize[%d] error", req.node_id);
			res.success = false;

			return false;
		}
	}

	res.success = true;

	return true;
}
