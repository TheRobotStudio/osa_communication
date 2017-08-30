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
 * @file can_layer.cpp
 * @author Cyril Jourdan
 * @date Aug 29, 2017
 * @version 2.0.0
 * @brief Implementation file for the CAN communication, adaptation from the mbed code
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : May 24, 2017
 */

/*! Includes */

//Other includes
//#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include "registers.h"
#include "can_layer.h"

#define LOOP_RATE 15

//Structure TODO replace with Controller class from osa_gui moved to a new package osa_common

//Motor epos_controllers_vp_[epos_controllers_vp_.size()] = {0, NOT_USED, NONE, false, PROFILE_VELOCITY_MODE, 0};
//can_msgs::Frame canFrame[epos_controllers_vp_.size()];
//bool motor_cmd_received_ = false;
//ros::Publisher data_pub;

CANLayer::CANLayer() :
robot_name_(""),
robot_can_device_(""),
epos_controllers_vp_(0),
number_epos_boards_(0),
data_({0}),
rx_can_frame_sub_(),
motor_cmd_sub_(),
tx_can_frame_pub_(),
motor_data_pub_(),
motor_cmd_received_(false)
{
}

//destructor
CANLayer::~CANLayer()
{
	if(ros::isStarted())
	{
	  ros::shutdown(); // this is necessary after using ros::start().
	  ros::waitForShutdown();
	}
	wait();
}

bool CANLayer::init()
{
	int init_argc = 0;
	char** init_argv = 0;
	ros::init(init_argc, init_argv, "osa_can_layer_node");

	if(!ros::master::check())
	{
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh("~");

	ROS_INFO("*** Init Publishers and Subsribers ***\n");

	//Subsribers and publishers
	rx_can_frame_sub_ = nh.subscribe("/received_messages", 8, &CANLayer::receiveMessagesCallback, this); //check size of FIFO : works with 8 (2 motors*(2 request + 2 answers))
	tx_can_frame_pub_ = new ros::Publisher(nh.advertise<can_msgs::Frame>("/sent_messages", 1)); //("/sent_messages", 8, true)); //latch message
	motor_cmd_sub_ = nh.subscribe("/motor_cmd_array", 1, &CANLayer::sendMotorCmdMultiArrayCallback, this); //receive commands here and translate them into CAN frames and send to /sent_messages
	motor_data_pub_ = nh.advertise<osa_msgs::MotorDataMultiArray>("/motor_data_array", 1); //Publish the data received on /receive_messages

	ROS_INFO("*** Grab the parameters from the YAML file ***\n");

	// Grab the parameters
	try
	{
		//load robot parameters
		if(!nh.param("/robot/name", robot_name_, std::string("my_robot")))
		{
			ROS_WARN("No /robot/name found in YAML config file");
		}

		if(!nh.param("/robot/dof", number_epos_boards_, int(2)))
		{
			ROS_WARN("No /robot/dof found in YAML config file");
		}

		if(!nh.param("/robot/can_device", robot_can_device_, std::string("can0")))
		{
			ROS_WARN("No /robot/can_device found in YAML config file");
		}

		ROS_INFO("Robot name=%s, dof=%d, can=%s", robot_name_.c_str(), number_epos_boards_, robot_can_device_.c_str());
/*
		//load mobile_base parameters
		if(nh.searchParam("/mobile_base", mobile_base_str))
		{
			ROS_INFO("/mobile_base found in YAML config file");
		}
		else
		{
			ROS_WARN("No /mobile_base found in YAML config file");
		}
*/
		//load controllers parameters
		//Example:
		//controller1: {node_id: 1, name: 'right wheel', type: 'EPOS4', inverted: true, motor: 'EC90', mode: 'PROFILE_VELOCITY_MODE', value: 0}

		bool controller_exist = true;
		//start with controller 1
		int controller_idx = 1;
		std::string rad_str = "controller"; //common radical name

		//std::ostringstream controller_idx_path;

		//std::string absolute_str = controller_idx_path.str();

		//ROS_INFO("string=%s", absolute_str.c_str());

		while(controller_exist)
		{
			//create the string "controller+index" to search for the controller parameter with that index number
			std::ostringstream controller_idx_path;
			controller_idx_path << rad_str << controller_idx;

			std::string absolute_str = "absolute_str";

			//ROS_INFO("string=%s", controller_idx_path.str().c_str());

			if(nh.searchParam(controller_idx_path.str(), absolute_str))
			{
				ROS_INFO("%s found in YAML config file", controller_idx_path.str().c_str());
				//ROS_INFO("absolute_str = %s", absolute_str.c_str());

				//create variables to store the controller parameters:
				int node_id = 0;
				std:: string name;
				std:: string type;
				bool inverted;
				std:: string motor;
				std:: string mode;
				int value;

				//grab the parameters of the current controller

				//node_id
				std::ostringstream node_id_path;
				node_id_path << absolute_str << "/node_id";
				if(!nh.getParam(node_id_path.str(), node_id))
				{
					ROS_ERROR("Can't grab param node_id for %s", controller_idx_path.str().c_str());
					return false;
				}

				//name
				std::ostringstream name_path;
				name_path << absolute_str << "/name";
				if(!nh.getParam(name_path.str(), name))
				{
					ROS_ERROR("Can't grab param name for %s", controller_idx_path.str().c_str());
					return false;
				}

				//type
				std::ostringstream type_path;
				type_path << absolute_str << "/type";
				if(!nh.getParam(type_path.str(), type))
				{
					ROS_ERROR("Can't grab param type for %s", controller_idx_path.str().c_str());
					return false;
				}

				//inverted
				std::ostringstream inverted_path;
				inverted_path << absolute_str << "/inverted";
				if(!nh.getParam(inverted_path.str(), inverted))
				{
					ROS_ERROR("Can't grab param inverted for %s", controller_idx_path.str().c_str());
					return false;
				}

				//motor
				std::ostringstream motor_path;
				motor_path << absolute_str << "/motor";
				if(!nh.getParam(motor_path.str(), motor))
				{
					ROS_ERROR("Can't grab param motor for %s", controller_idx_path.str().c_str());
					return false;
				}

				//mode
				std::ostringstream mode_path;
				mode_path << absolute_str << "/mode";
				if(!nh.getParam(mode_path.str(), mode))
				{
					ROS_ERROR("Can't grab param mode for %s", controller_idx_path.str().c_str());
					return false;
				}

				//value
				std::ostringstream value_path;
				value_path << absolute_str << "/value";
				if(!nh.getParam(value_path.str(), value))
				{
					ROS_ERROR("Can't grab param value for %s", controller_idx_path.str().c_str());
					return false;
				}

				//print the controller parameters
				ROS_INFO("Parameters for %s : node_id[%d], name[%s], type[%s], inverted[%d], motor[%s], mode[%s], value[%d]", controller_idx_path.str().c_str(),
						node_id, name.c_str(), type.c_str(), inverted, motor.c_str(), mode.c_str(), value);

				//create a new EPOS controller
				EPOSController *epos_controller = new EPOSController(node_id, name, type, inverted, motor, mode, value, tx_can_frame_pub_);

				//epos_controller->

				//v_controllers.push_back(controller);
				epos_controllers_vp_.push_back(epos_controller);

				//increment to search for the next controller
				controller_idx++;
			}
			else
			{
				controller_exist = false;
				ROS_INFO("No more controllers found in YAML config file");
			}

			//controller_exist = false;
		}

		controller_idx--;
		if(number_epos_boards_ == controller_idx) ROS_INFO("Same number of DOF(%d) and controllers(%d) defined in the YAML config file!", number_epos_boards_, controller_idx);
		else
		{
			ROS_WARN("Not the same number of DOF(%d) and controllers(%d) defined in the YAML config file!", number_epos_boards_, controller_idx);
			throw 1;
		}

		/*
		nh.param("can_device", can_device_str, std::string("can0"));
		//mot1
		nh.param("controller1_type", controller1_type_str, std::string("EPOS4"));
		nh.param("motor1_type", motor1_type_str, std::string("EC90"));
		nh.param("motor1_inverted", motor1_inverted_bool, bool(true));
		nh.param("mode1", mode1_str, std::string("PROFILE_VELOCITY_MODE"));
		nh.param("value1", value1_int, int(0));
		//mot2
		nh.param("controller2_type", controller2_type_str, std::string("EPOS4"));
		nh.param("motor2_type", motor2_type_str, std::string("EC90"));
		nh.param("motor2_inverted", motor2_inverted_bool, bool(false));
		nh.param("mode2", mode2_str, std::string("PROFILE_VELOCITY_MODE"));
		//nh.param("value2", value2_int, int(0));
		nh.param("value2", value2_int);
		*/

		ROS_INFO("Parameters loaded successfully!");
	}
	catch(int exception)
	{
		ROS_ERROR("Parameters didn't load correctly!");
		ROS_ERROR("Please modify your YAML config file and try again.");

		return false;
	}

	//wait for the Publisher/Subscriber to connect
	ROS_INFO("--- wait for the Publisher/Subscriber to connect ---\n");

	ros::Rate poll_rate(100);
	while(tx_can_frame_pub_->getNumSubscribers() == 0)
	{
			poll_rate.sleep();
	}

	int i = 0; //msg
	int j = 0; //byte number

	//TEST assume 2 EPOS4
	//numberEposBoards = 2;

	ROS_INFO("--- Initialise EPOS boards ---\n");
	//power or pushbutton reset
	for(int node_id=1; node_id<=number_epos_boards_; node_id++)
	{
		//ROS_INFO("initEposBoard %d", node_id);

		if(epos_controllers_vp_[node_id-1]->initEposBoard() != EPOS_OK)
		{
			ROS_INFO("initEposBoard error");
			return false; //exit the main function and return fault if an initialization failed
		}
	}

	ROS_INFO("--- Calibrate Arm ---\n");
	for(int node_id=1; node_id<=number_epos_boards_; node_id++)
	{
		if(epos_controllers_vp_[node_id-1]->calibrate() == EPOS_ERROR)
		{
			ROS_INFO("Calibration error, check the mode/value of your config file, only use Profile modes for EPOS2/EPOS4 and Current mode for EPOS2.");
			return false;
		}
	}

	ROS_INFO("--- Getting motor data ---\n");
	//gather first pack of data
	//get the sensor values
	for(int node_id=1; node_id<=number_epos_boards_; node_id++)
	{
		epos_controllers_vp_[node_id-1]->getData();
	}




	/*

	//Publishers
	m_pub_motorCmdArray = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100, true);

	//Subscribers
	m_sub_motorDataArray = nh.subscribe("/motor_data_array", 10, &BasicControlNode::motorDataArray_cb, this);
*/

	ROS_INFO("--- Init motor command array ---\n");

	//create the commands multi array
	motor_cmd_ma_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_ma_.layout.dim[0].size = 1;
	motor_cmd_ma_.layout.dim[0].stride = number_epos_boards_;
	motor_cmd_ma_.layout.dim[0].label = "epos";

	motor_cmd_ma_.layout.data_offset = 0;

	motor_cmd_ma_.motorCmd.clear();
	motor_cmd_ma_.motorCmd.resize(number_epos_boards_);

	resetMotorCmdMultiArray();

/*
	motor_cmd_ma_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_ma_.layout.dim[0].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_ma_.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_ma_.layout.dim[0].label = "motors";
	motor_cmd_ma_.layout.data_offset = 0;
	motor_cmd_ma_.motorCmd.clear();
	motor_cmd_ma_.motorCmd.resize(NUMBER_MAX_EPOS2_PER_SLAVE);


	for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		motor_cmd_ma_.motorCmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_ma_.motorCmd[i].value = 0;
	}
*/

	//then start the main loop
	ROS_INFO("--- Start main loop ---");
	run();

	return true;
}

void CANLayer::run()
{
	ROS_INFO("--- RUN ---");

	ros::Rate r(LOOP_RATE);

	while(ros::ok())
	{
		ROS_INFO("spinOnce");
		ros::spinOnce();

		//publish if enabled
		//if(m_enablePublish)
		//tx_can_frame_pub_->publish(motor_cmd_ma_); //TODO pub motor cmd

		//resetMotorCmdMultiArray();

		ROS_INFO("sleep");
		r.sleep();
	}
}

//callback
void CANLayer::receiveMessagesCallback(const can_msgs::FrameConstPtr& can_msg)
{
	uint64_t data = 0x0000000000000000; //64 bits
	uint8_t node_id = 0;
	int16_t cob_id = 0;

	ROS_INFO("receiveMessagesCallback");

	//ROS_INFO("Interrupt frame : [%02X] [%02X %02X %02X %02X %02X %02X %02X %02X]\n", can_msg->id, can_msg->data[7], can_msg->data[6], can_msg->data[5], can_msg->data[4], can_msg->data[3], can_msg->data[2], can_msg->data[1], can_msg->data[0]);

	node_id = 0x07F & can_msg->id;
	cob_id = 0x0F80 & can_msg->id;

	ROS_INFO("node_id [%02X],  cobID [%02X], can_msg->dlc=%d", node_id, cob_id, can_msg->dlc);
	

	for(int i=0; i<can_msg->dlc; i++) //dlc=len
	{
		//ROS_INFO("for i=%d", i);
		data = data | (can_msg->data[i]<<i*8);
	}

	//ROS_INFO("data=%ld", data);
	
	//check node_id first and that the command is an answer to a request (RTR=false)
	if((node_id >= 1) && (node_id <= number_epos_boards_) && (can_msg->is_rtr==false))
	{
		switch(cob_id)
		{       
		    case COB_ID_TRANSMIT_PDO_1_ENABLE : //getPositionVelocity
		    {
		        ROS_INFO("TPDO1 NodeID[%d] COB-ID[%02X] RTR[%d] data[%d]\n", node_id, cob_id, can_msg->is_rtr, data);
		        
		        int32_t pos = (can_msg->data[3]<<24 | can_msg->data[2]<<16 | can_msg->data[1]<<8 | can_msg->data[0]); //0x00000000FFFFFFFF & data;
		        int32_t vel = (can_msg->data[7]<<24 | can_msg->data[6]<<16 | can_msg->data[5]<<8 | can_msg->data[4]);
		                                       
		        if(epos_controllers_vp_[node_id-1]->getInverted() == true) //!< change sign
		        {
		        	epos_controllers_vp_[node_id-1]->setPosition(-1*pos);
		        	epos_controllers_vp_[node_id-1]->setVelocity(-1*vel);
		        }
		        else
		        {
		        	epos_controllers_vp_[node_id-1]->setPosition(pos);
		        	epos_controllers_vp_[node_id-1]->setVelocity(vel);
		        }
		        
		        break;
		    }
	    
		    case COB_ID_TRANSMIT_PDO_2_ENABLE : //getCurrentFollErrStatusword
		    {
		        ROS_INFO("TPDO2 NodeID[%d] COB-ID[%02X] RTR[%d] data[%d]\n", node_id, cob_id, can_msg->is_rtr, data);
		        //ROS_INFO("IT[%02X] [%02X %02X]\n", can_msg->id, can_msg->data[5] ,  can_msg->data[4]);
		        
		        int16_t curr  = (can_msg->data[1]<<8 | can_msg->data[0]); //0x000000000000FFFF & data;
		        int16_t follErr = (can_msg->data[3]<<8 | can_msg->data[2]); //(0x00000000FFFF0000 & data) >> 16;
		        uint16_t statwrd = (can_msg->data[5]<<8 | can_msg->data[4]); //(0x0000FFFF00000000 & data) >> 32;
		     
		        if(epos_controllers_vp_[node_id-1]->getInverted() == true) curr = -1*curr; //change sign
		        epos_controllers_vp_[node_id-1]->setCurrent(curr);
		        epos_controllers_vp_[node_id-1]->setFollowingError(follErr);
		        epos_controllers_vp_[node_id-1]->setStatusword(statwrd);
		        
		        //ROS_INFO("TPDO2 Node-ID[%d] curr[%d] follErr[%d] statwrd[%d]\n", node_id, curr, follErr, statwrd);
		        
		        break;
		    }
	    
		    case COB_ID_TRANSMIT_PDO_3_ENABLE : //getModesOfOperation
		    {
		        //ROS_INFO("TPDO3 Node ID : [%d], PDO COB-ID [%02X], data = %d\n", node_id, cobID, data);
		        
		        int8_t modesOfOp = 0x00000000000000FF & data;;
		                                      
		        epos_controllers_vp_[node_id-1]->setModesOfOperation(modesOfOp);
		        
		        break;
		    }
	    
		    case COB_ID_TRANSMIT_PDO_4_ENABLE : //getIncEnc1CntAtIdxPls
		    {
		        //ROS_INFO("TPDO4 Node ID : [%d], PDO COB-ID [%02X], data = %d\n", node_id, cobID, data);
		        
		        uint32_t incEnc1 = 0x00000000FFFFFFFF & data;;
		        
		        epos_controllers_vp_[node_id-1]->setIncEnc1CntAtIdxPls(incEnc1);
		        
		        break; 
		    }
	 
		    case COB_ID_EMCY_DEFAULT : //Emergency frame
		    {	
		        //ROS_INFO("Emergency frame, Node ID : [%d], PDO COB-ID [%02X], data = %02X\n", node_id, cobID, data);
		        //ROS_INFO("EF [%02X][%02X %02X %02X %02X %02X %02X %02X %02X]\n", can_msg->id, can_msg->data[7], can_msg->data[6], can_msg->data[5], can_msg->data[4], can_msg->data[3], can_msg->data[2], can_msg->data[1], can_msg->data[0]);
		        ROS_INFO("EF%02X-%02X%02X\n", can_msg->id, can_msg->data[1], can_msg->data[0]);
		        //ledchain[1] = 1;            
		        //nh.logerror("Emergency frame");
		        epos_controllers_vp_[node_id-1]->setBoardStatus(1);
		        //first step : fault reset on controlword
		        //ROS_INFO("Node %d - STEP 1 - faultResetControlword\n", node_id);
		        
		        //Debug for fault detection on brachii
		        epos_controllers_vp_[node_id-1]->faultResetControlword();        //TODO replace with RPDO
		        break;  
		    }
	    
		    case COB_ID_SDO_SERVER_TO_CLIENT_DEFAULT : //SDO Acknoledgement frame
		    {            
	    		int32_t regData = 0x00000000;
		        regData = (int32_t)data;
		        
		        //ROS_INFO("Node %d - regData [%02X]\n", node_id, regData);
		                        
		        if(regData == 0x00604060) //Controlword
		        {
		            if(epos_controllers_vp_[node_id-1]->getBoardStatus() == 1)
		            {
		            	epos_controllers_vp_[node_id-1]->setBoardStatus(2);
		                //second step : shutdown controlword
		                //ROS_INFO("Node %d - STEP 2 - shutdownControlwordIT\n", node_id);
		            	epos_controllers_vp_[node_id-1]->shutdownControlwordIT(); //TODO replace with RPDO
		            }
		            else if(epos_controllers_vp_[node_id-1]->getBoardStatus() == 2)
		            {
		            	epos_controllers_vp_[node_id-1]->setBoardStatus(3);
		                //third step : Switch On & Enable Operation on Controlword
		                //ROS_INFO("Node %d - STEP 3 - switchOnEnableOperationControlwordIT\n", node_id);
		            	epos_controllers_vp_[node_id-1]->switchOnEnableOperationControlwordIT(); //TODO replace with RPDO
		            }
		            else if(epos_controllers_vp_[node_id-1]->getBoardStatus() == 3)
		            {
		            	epos_controllers_vp_[node_id-1]->setBoardStatus(4);
		                //ask for statusword to check if the board has reset well
		                //ROS_INFO("Node %d - STEP 4 - getStatusword\n", node_id);
		                //TODO getStatusword(node_id);
		            }
		        } 
		        else if(regData == 0x0060414B) //read Statusword
		        {
		            //int32_t swData = 0x00000000;
		            
		            //ROS_INFO("Node %d - Statusword [%02X]\n", node_id, can_msg->data[4]);
		            //ROS_INFO("Statusword frame : [%02X] [%02X %02X %02X %02X %02X %02X %02X %02X]\n", can_msg->id, can_msg->data[7], can_msg->data[6], can_msg->data[5], can_msg->data[4], can_msg->data[3], can_msg->data[2], can_msg->data[1], can_msg->data[0]);
		            
		            if(epos_controllers_vp_[node_id-1]->getBoardStatus() == 4)
		            {
		                //swData = data >> 32;
		                int8_t fault = 0x00;
		                fault = (can_msg->data[4] & 0x08) >> 3;
		                                       
		                if(fault == 0) //reset OK
		                {
		                	epos_controllers_vp_[node_id-1]->setBoardStatus(0); //Board is reset and enable OK
		                    ROS_INFO("%d OK\n", node_id);
		                    //ledchain[1] = 0;
		                }
		                else //try to reset again
		                {
		                    //ROS_INFO("Node %d - try to reset again\n", node_id);
		                	epos_controllers_vp_[node_id-1]->setBoardStatus(1);
		                    //go back to first step : fault reset on controlword
		                    //ROS_INFO("Node %d - STEP 1 - faultResetControlword\n", node_id);
		                	epos_controllers_vp_[node_id-1]->faultResetControlword();       //TODO replace with RPDO
		                }
		            }  
		        }                                            
		        break;
		    }               
		    default :
		    {
		        ROS_INFO("Unknown frame [%02X][%02X %02X %02X %02X %02X %02X %02X %02X]\n", can_msg->id, can_msg->data[7], can_msg->data[6], can_msg->data[5], can_msg->data[4], can_msg->data[3], can_msg->data[2], can_msg->data[1], can_msg->data[0]);
		    }                        
		} //end switch

		//publish data
		osa_msgs::MotorDataMultiArray motorData_ma;

		//create the data multi array
		motorData_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
		motorData_ma.layout.dim[0].size = epos_controllers_vp_.size();
		motorData_ma.layout.dim[0].stride = epos_controllers_vp_.size();
		motorData_ma.layout.dim[0].label = "motors";
		motorData_ma.layout.data_offset = 0;
		motorData_ma.motorData.clear();
		motorData_ma.motorData.resize(epos_controllers_vp_.size());

		for(int i=0; i<epos_controllers_vp_.size(); i++)
		{
			motorData_ma.motorData[i].position = epos_controllers_vp_[node_id-1]->getPosition();
			motorData_ma.motorData[i].current = epos_controllers_vp_[node_id-1]->getCurrent();
			motorData_ma.motorData[i].status = epos_controllers_vp_[node_id-1]->getStatusword();
		}

		//ROS_INFO("Publish motor data\n");
		motor_data_pub_.publish(motorData_ma);

	}
	else
	{
		ROS_INFO("NODEID ERROR\n");    
	}
}

/*! \fn void sendMotorCmdMultiArray_cb(const osa_msgs::MotorCmdMultiArrayConstPtr& motorCmd_ma)
 *  \brief callback function that received EPOS commands and that fits it into the right frame format to be sent over CAN bus.
 *  \param motorCmd_ma motor command multi-array.
 *  \return void
 */
void CANLayer::sendMotorCmdMultiArrayCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& motorCmd_ma)
{	
	//toggle flag, a message has been received
	motor_cmd_received_ = true;

	ROS_INFO("sendMotorCmdMultiArrayCallback");

	for(int i=0; i<epos_controllers_vp_.size(); i++)
	{
		uint8_t node_id = motorCmd_ma->motorCmd[i].nodeID;
		uint8_t command = motorCmd_ma->motorCmd[i].command;
		int32_t value = motorCmd_ma->motorCmd[i].value;

		//ROS_INFO("cmd[%d] val[%d]\n", command, value);

		switch(command)
		{
		    case SET_TARGET_POSITION:
		    {
		    	epos_controllers_vp_[node_id-1]->setTargetPosition(value);
				break;
		    }

		    case SET_TARGET_VELOCITY:
		    {
		    	epos_controllers_vp_[node_id-1]->setTargetVelocity(value);
				break;
		    }

		    case SET_PROFILE_ACCELERATION:
		    {
		    	epos_controllers_vp_[node_id-1]->setProfileAcceleration(value);
				break;
		    }

		    case SET_PROFILE_DECELERATION:
		    {
		    	epos_controllers_vp_[node_id-1]->setProfileDeceleration(value);
				break;
		    }

		    case SET_PROFILE_VELOCITY:
		    {
		    	epos_controllers_vp_[node_id-1]->setProfileVelocity(value);
				break;
		    }

		    case SET_OUTPUT_CURRENT_LIMIT:
		    {
		    	epos_controllers_vp_[node_id-1]->setOutputCurrentLimit(value);
				break;
		    }
		    
		    case SET_CONTROLWORD:
		    {
		    	epos_controllers_vp_[node_id-1]->setControlword(value);
				break;
		    }

		    case SET_CURRENT_MODE_SETTING_VALUE:
		    {
		    	epos_controllers_vp_[node_id-1]->setCurrentModeSettingValue(value);
				break;
		    }

		    case SET_MAXIMAL_SPEED_IN_CURRENT_MODE:
		    {
		    	epos_controllers_vp_[node_id-1]->setMaximalSpeedInCurrentMode(value);
				break;
		    }

		    case SET_MODES_OF_OPERATION:
		    {
				switch(value)
				{
					case INTERPOLATED_POSITION_MODE:
					{
						epos_controllers_vp_[node_id-1]->setModesOfOperation(VALUE_INTERPOLATED_POSITION_MODE);
						break;
					}

					case PROFILE_VELOCITY_MODE:
					{
						epos_controllers_vp_[node_id-1]->setModesOfOperation(VALUE_PROFILE_VELOCITY_MODE);
						break;
					}

					case PROFILE_POSITION_MODE:
					{
						epos_controllers_vp_[node_id-1]->setModesOfOperation(VALUE_PROFILE_POSITION_MODE);
						break;
					}

					case POSITION_MODE:
					{
						epos_controllers_vp_[node_id-1]->setModesOfOperation(VALUE_POSITION_MODE);
						break;
					}

					case VELOCITY_MODE:
					{
						epos_controllers_vp_[node_id-1]->setModesOfOperation(VALUE_VELOCITY_MODE);
						break;
					}

					case CURRENT_MODE:
					{
						epos_controllers_vp_[node_id-1]->setModesOfOperation(VALUE_CURRENT_MODE);
						break;
					}

					case CYCLIC_SYNCHRONOUS_TORQUE_MODE:
					{
						epos_controllers_vp_[node_id-1]->setModesOfOperation(VALUE_CYCLIC_SYNCHRONOUS_TORQUE_MODE);
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
				//if(!cmdPlayLED) ledchain[3] = 0;    //switch off when slave is idle, i.e. all cmd in a set are 0xFF.
				break;
		    }

		    default:
		    {
				//if(!cmdPlayLED) ledchain[3] = 0;    //switch off when slave is idle, i.e. all cmd in a set are 0xFF.
				break;
		    }
		}
	  
		//if(cmdPlayLED) ledchain[3] = 1;    //switch on if cmd is applied.
		    
		ros::Duration(0.00001).sleep(); //10us
	}	

	//ROS_INFO("cmd[%d] val[%d]", motorCmd_ma->motorCmd[0].command, motorCmd_ma->motorCmd[0].value);
}

void CANLayer::resetMotorCmdMultiArray()
{
	for(int i=0; i<number_epos_boards_; i++)
	{
		motor_cmd_ma_.motorCmd[i].slaveBoardID = 1;
		motor_cmd_ma_.motorCmd[i].nodeID = i + 1;
		motor_cmd_ma_.motorCmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_ma_.motorCmd[i].value = 0;
	}
}

