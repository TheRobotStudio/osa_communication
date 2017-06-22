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
 * @file socketcan.cpp
 * @author Cyril Jourdan
 * @date May 24, 2017
 * @version 2.0.0
 * @brief Implementation file for the CAN communication, adaptation from the mbed code
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : May 24, 2017
 */

/*! Includes */
#include <ros/ros.h>
#include "osa_msgs/MotorCmdMultiArray.h"
#include "osa_msgs/MotorDataMultiArray.h"
#include <can_msgs/Frame.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_interface/socketcan.h>
#include <sstream>
#include <string>

//#include "robot.h"
//#include "eposCmd.h"
#include "eposCmd.cpp"
#include "registers.h"

//Motor motorArray[NUMBER_OF_MOTORS] = {0, NOT_USED, NONE, false, PROFILE_VELOCITY_MODE, 0};
//can_msgs::Frame canFrame[NUMBER_OF_MOTORS];
bool msgReceived = false;
ros::Publisher data_pub;

/*! \fn void calibrate()
 *  \brief calibration routine.
 *  \return int EPOS_OK or EPOS_ERROR
 */
int calibrate()
{
    uint16_t outCurLmt = 0;
    uint32_t profVel = 0;
    uint32_t profAcc = 0;
    uint32_t profDec = 0; 
    uint32_t maxSpeed = 0;
    
    ROS_INFO("- Start Calibration\n");
    
    for(int nodeID=1; nodeID<=numberEposBoards; nodeID++)
    { 
        printf("NodeID[%d] - ", nodeID);
        
        //set default parameters
        switch(motorArray[nodeID-1].motorType)    
        {    
            case NONE : 
	    {
                return EPOS_ERROR;
                //break;
            }

            case DCX10 :
	    {
                printf("DCX10 - "); 
                               
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    
                    outCurLmt = 2000;
                    profVel = 2000;
                    profAcc = 2000;
                    profDec = 2000; 
                    maxSpeed = 10000;
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    return EPOS_ERROR;
                }
                break;
            }
   
            case DCX14 :
	    {
                printf("DCX14 - ");
                                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    
                    outCurLmt = 2000;
                    profVel = 2000;
                    profAcc = 2000;
                    profDec = 2000; 
                    maxSpeed = 10000;
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    return EPOS_ERROR;
                }
                break;
            }
    
            case DCX16 : //!< return error if it is not a DC type of motor   
	    {         
                printf("DCX16 - ");   
                             
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    
                    outCurLmt = 1000;
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000; 
                    maxSpeed = 1000;
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    return EPOS_ERROR;
                }
                break;
            }
    
            case DCX22 :
	    {
                printf("DCX22 - "); 
                  
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    
                    outCurLmt = 2000;
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000; 
                    maxSpeed = 10000;
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    return EPOS_ERROR;
                }
                break;
            }
    
            case DCX32 :
	    {
                printf("DCX32 - ");  
                          
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    
                    outCurLmt = 1000;
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000; 
                    maxSpeed = 1000;
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4");
                    
                    //outCurLmt = 1000;
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000; 
                    maxSpeed = 1000;
                }
                break;
            }
    
            case RE13 :
	    {
                printf("RE13 - ");  
                          
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    
                    outCurLmt = 2000;
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000; 
                    maxSpeed = 10000;                    
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    return EPOS_ERROR;
                }
                break;  
            }
    
            case RE30 :
	    {
                printf("RE30 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    
                    outCurLmt = 1000;
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000; 
                    maxSpeed = 1000;
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4");
                                        
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000;
                    maxSpeed = 1000;
                }
                break;
            }

            case ECI40 :
	    {
                printf("ECI40 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    
                    outCurLmt = 1000;
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000;
                    maxSpeed = 1000;
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4");
                                       
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000; 
                    maxSpeed = 1000;
                }
                break;
            }
    
            case ECI52 :
	    {
                printf("ECI52 - ");
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    
                    outCurLmt = 1000;
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000;
                    maxSpeed = 1000;
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4");
                    
                    profVel = 800;
                    profAcc = 1000;
                    profDec = 1000;
                    maxSpeed = 1000;
                }
                break;             
            }
    
            case EC90 :
	    {
                printf("EC90 - ");
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    
                    outCurLmt = 1000;
                    profVel = 1500;
                    profAcc = 10000;
                    profDec = 10000; 
                    maxSpeed = 5000;
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4");
                    
                    profVel = 1500;
                    profAcc = 10000;
                    profDec = 10000; 
                    maxSpeed = 5000;
                }
                break;
            }

            default :
	    {
                return EPOS_ERROR;
                //break;
	    }
        }
        
        //Apply values
        //RPDO3           
        setControlword(nodeID, 0x0006); //Shutdown
        ros::Duration(0.01).sleep(); 
        setProfileVelocity(nodeID, profVel); //NEW try this first         
        ros::Duration(0.01).sleep();        
        if(motorArray[nodeID-1].controllerType == EPOS2) setOutputCurrentLimit(nodeID, outCurLmt); //10100 //10000 for EPOS2 70/10, see documentation
        ros::Duration(0.01).sleep();
        
        //RPDO2
        setProfileAcceleration(nodeID, profAcc);
        ros::Duration(0.01).sleep();
        setProfileDeceleration(nodeID, profDec);        
        ros::Duration(0.01).sleep();
        
        //RPDO4
        setMaximalSpeedInCurrentMode(nodeID, maxSpeed); //TODO change function name for fitting with EPOS4 too
        ros::Duration(0.01).sleep();          
        
        //Common to all motor type
        setControlword(nodeID, 0x0006); //Shutdown
        ros::Duration(0.01).sleep();
        setControlword(nodeID, 0x000F); //SwitchOn
        ros::Duration(0.01).sleep();
        
        //Apply a default value based on the selected mode
        switch(motorArray[nodeID-1].mode)
        {
            case INTERPOLATED_POSITION_MODE :
	    {
                return EPOS_ERROR;
                //break;
            }

            case PROFILE_VELOCITY_MODE : 
	    {
                setTargetVelocity(nodeID, motorArray[nodeID-1].value);
                ros::Duration(0.1).sleep();
                setControlword(nodeID, 0x000F); //Start to move
                printf(" - PROFILE_VELOCITY_MODE val[%d] setControlword[0x000F]", motorArray[nodeID-1].value);
                break;
            }    

            case PROFILE_POSITION_MODE :
	    { 
                setTargetPosition(nodeID, motorArray[nodeID-1].value);
                ros::Duration(0.1).sleep();                
                setControlword(nodeID, 0x002F); //Start Positioning (absolute position and start immediately)
                ros::Duration(0.1).sleep();
                setControlword(nodeID, 0x003F);                
                printf(" - PROFILE_POSITION_MODE val[%d] setControlword[rising edge 0x002F to 0x003F]", motorArray[nodeID-1].value);
                break;
            }
    
            case POSITION_MODE : 
	    {
                return EPOS_ERROR;
                //break;
	    }
                
            case VELOCITY_MODE : 
	    {
                return EPOS_ERROR;
                //break;
	    }
                
            case CURRENT_MODE : 
	    {
                setCurrentModeSettingValue(nodeID, motorArray[nodeID-1].value); 
                ros::Duration(0.1).sleep();
                printf(" - CURRENT_MODE val[%d]", motorArray[nodeID-1].value);
                break; 
            }
    
            case CYCLIC_SYNCHRONOUS_TORQUE_MODE : 
	    {
                return EPOS_ERROR; //TODO
                //break; 
            }

            default :
	    {
                return EPOS_ERROR;  
	    }                       
        }
        
       // ledchain[3] = 1;
        ros::Duration(0.01).sleep();
                
        //TPDO : get data once to fill in the arrays
        getPositionVelocity(nodeID);
        ros::Duration(0.01).sleep();
        getCurrentFollErrStatusword(nodeID);
        ros::Duration(0.01).sleep();
        
        //print results to check things are working properly
        printf(" - pos=%d vel=%d cur=%d stwrd=%d\n", position[nodeID-1], velocity[nodeID-1], current[nodeID-1], statusword[nodeID-1]); 
    }
    
    return EPOS_OK;
}


/*! \fn void getData()
 *  \brief 
 *  \return void
 */
void getData()
{
	//printf("getData\n");
	//for(int i=1; i<=numberEposBoards; i++)
	for(int i=1; i<=2; i++)
	{
		//uint8_t node_id = i;
				      
		getPositionVelocity(i);
		//ros::Duration(0.0002).sleep();
		ros::Duration(0.002).sleep();
		//ros::Duration(0.002).sleep();
		getCurrentFollErrStatusword(i);
		//ros::Duration(0.0002).sleep(); 
		ros::Duration(0.002).sleep(); 
		//ros::Duration(0.002).sleep();                    
	}
}

//callback
void receive_messages_cb(const can_msgs::FrameConstPtr& canmsg)
{
	uint64_t data = 0x0000000000000000; //64 bits
	uint8_t nodeID = 0;
	int16_t cobID = 0;

	//ROS_INFO("receive_messages_cb");

	//ROS_INFO("Interrupt frame : [%02X] [%02X %02X %02X %02X %02X %02X %02X %02X]\n", canmsg->id, canmsg->data[7], canmsg->data[6], canmsg->data[5], canmsg->data[4], canmsg->data[3], canmsg->data[2], canmsg->data[1], canmsg->data[0]);

	nodeID = 0x07F & canmsg->id;
	cobID = 0x0F80 & canmsg->id;

	ROS_INFO("nodeID [%02X],  cobID [%02X], canmsg->dlc=%d", nodeID, cobID, canmsg->dlc);
	

	for(int i=0; i<canmsg->dlc; i++) //dlc=len
	{
		//ROS_INFO("for i=%d", i);
		data = data | (canmsg->data[i]<<i*8);        
	}

	//ROS_INFO("data=%ld", data);
	
	//check nodeID first and that the command is an answer to a request (RTR=false)
	if((nodeID >= 1) && (nodeID <= numberEposBoards) && (canmsg->is_rtr==false)) 
	{
		switch(cobID)
		{       
		    case COB_ID_TRANSMIT_PDO_1_ENABLE : //getPositionVelocity
		    {
		        ROS_INFO("TPDO1 NodeID[%d] COB-ID[%02X] RTR[%d] data[%d]\n", nodeID, cobID, canmsg->is_rtr, data);
		        
		        int32_t pos = (canmsg->data[3]<<24 | canmsg->data[2]<<16 | canmsg->data[1]<<8 | canmsg->data[0]); //0x00000000FFFFFFFF & data;                
		        int32_t vel = (canmsg->data[7]<<24 | canmsg->data[6]<<16 | canmsg->data[5]<<8 | canmsg->data[4]);
		                                       
		        if(motorArray[nodeID-1].inverted == true) //!< change sign
		        {
		            position[nodeID-1] = -1*pos;
		            velocity[nodeID-1] = -1*vel;
		        }
		        else
		        {
		            position[nodeID-1] = pos;
		            velocity[nodeID-1] = vel;
		        }
		        
		        break;
		    }
	    
		    case COB_ID_TRANSMIT_PDO_2_ENABLE : //getCurrentFollErrStatusword
		    {
		        ROS_INFO("TPDO2 NodeID[%d] COB-ID[%02X] RTR[%d] data[%d]\n", nodeID, cobID, canmsg->is_rtr, data);
		        //ROS_INFO("IT[%02X] [%02X %02X]\n", canmsg->id, canmsg->data[5] ,  canmsg->data[4]);
		        
		        int16_t curr  = (canmsg->data[1]<<8 | canmsg->data[0]); //0x000000000000FFFF & data;
		        int16_t follErr = (canmsg->data[3]<<8 | canmsg->data[2]); //(0x00000000FFFF0000 & data) >> 16;
		        uint16_t statwrd = (canmsg->data[5]<<8 | canmsg->data[4]); //(0x0000FFFF00000000 & data) >> 32;
		     
		        if(motorArray[nodeID-1].inverted == true) curr = -1*curr; //change sign
		        current[nodeID-1] = curr;
		        followingError[nodeID-1] = follErr;
		        statusword[nodeID-1] = statwrd;
		        
		        //ROS_INFO("TPDO2 Node-ID[%d] curr[%d] follErr[%d] statwrd[%d]\n", nodeID, curr, follErr, statwrd);
		        
		        break;
		    }
	    
		    case COB_ID_TRANSMIT_PDO_3_ENABLE : //getModesOfOperation
		    {
		        //ROS_INFO("TPDO3 Node ID : [%d], PDO COB-ID [%02X], data = %d\n", nodeID, cobID, data);  
		        
		        int8_t modesOfOp = 0x00000000000000FF & data;;
		                                      
		        modesOfOperation[nodeID-1] = modesOfOp;
		        
		        break;
		    }
	    
		    case COB_ID_TRANSMIT_PDO_4_ENABLE : //getIncEnc1CntAtIdxPls
		    {
		        //ROS_INFO("TPDO4 Node ID : [%d], PDO COB-ID [%02X], data = %d\n", nodeID, cobID, data);
		        
		        uint32_t incEnc1 = 0x00000000FFFFFFFF & data;;
		        
		        incEnc1CntAtIdxPls[nodeID-1] = incEnc1;
		        
		        break; 
		    }
	 
		    case COB_ID_EMCY_DEFAULT : //Emergency frame
		    {	
		        //ROS_INFO("Emergency frame, Node ID : [%d], PDO COB-ID [%02X], data = %02X\n", nodeID, cobID, data);
		        //ROS_INFO("EF [%02X][%02X %02X %02X %02X %02X %02X %02X %02X]\n", canmsg->id, canmsg->data[7], canmsg->data[6], canmsg->data[5], canmsg->data[4], canmsg->data[3], canmsg->data[2], canmsg->data[1], canmsg->data[0]);
		        ROS_INFO("EF%02X-%02X%02X\n", canmsg->id, canmsg->data[1], canmsg->data[0]);
		        //ledchain[1] = 1;            
		        //nh.logerror("Emergency frame");
		        boardStatus[nodeID-1] = 1;                
		        //first step : fault reset on controlword
		        //ROS_INFO("Node %d - STEP 1 - faultResetControlword\n", nodeID);
		        
		        //Debug for fault detection on brachii
		        faultResetControlword(nodeID);        //TODO replace with RPDO         
		        break;  
		    }
	    
		    case COB_ID_SDO_SERVER_TO_CLIENT_DEFAULT : //SDO Acknoledgement frame
		    {            
	    		int32_t regData = 0x00000000;
		        regData = (int32_t)data;
		        
		        //ROS_INFO("Node %d - regData [%02X]\n", nodeID, regData);
		                        
		        if(regData == 0x00604060) //Controlword
		        {
		            if(boardStatus[nodeID-1] == 1)
		            {
		                boardStatus[nodeID-1] = 2;                      
		                //second step : shutdown controlword
		                //ROS_INFO("Node %d - STEP 2 - shutdownControlwordIT\n", nodeID);
		                shutdownControlwordIT(nodeID); //TODO replace with RPDO 
		            }
		            else if(boardStatus[nodeID-1] == 2)
		            {
		                boardStatus[nodeID-1] = 3;                      
		                //third step : Switch On & Enable Operation on Controlword
		                //ROS_INFO("Node %d - STEP 3 - switchOnEnableOperationControlwordIT\n", nodeID);
		                switchOnEnableOperationControlwordIT(nodeID); //TODO replace with RPDO 
		            }
		            else if(boardStatus[nodeID-1] == 3)
		            {
		                boardStatus[nodeID-1] = 4;
		                //ask for statusword to check if the board has reset well
		                //ROS_INFO("Node %d - STEP 4 - getStatusword\n", nodeID);
		                //TODO getStatusword(nodeID);
		            }
		        } 
		        else if(regData == 0x0060414B) //read Statusword
		        {
		            //int32_t swData = 0x00000000;
		            
		            //ROS_INFO("Node %d - Statusword [%02X]\n", nodeID, canmsg->data[4]);
		            //ROS_INFO("Statusword frame : [%02X] [%02X %02X %02X %02X %02X %02X %02X %02X]\n", canmsg->id, canmsg->data[7], canmsg->data[6], canmsg->data[5], canmsg->data[4], canmsg->data[3], canmsg->data[2], canmsg->data[1], canmsg->data[0]);
		            
		            if(boardStatus[nodeID-1] == 4)
		            {
		                //swData = data >> 32;
		                int8_t fault = 0x00;
		                fault = (canmsg->data[4] & 0x08) >> 3;
		                                       
		                if(fault == 0) //reset OK
		                {
		                    boardStatus[nodeID-1] = 0; //Board is reset and enable OK
		                    ROS_INFO("%d OK\n", nodeID);
		                    //ledchain[1] = 0;
		                }
		                else //try to reset again
		                {
		                    //ROS_INFO("Node %d - try to reset again\n", nodeID);
		                    boardStatus[nodeID-1] = 1;                
		                    //go back to first step : fault reset on controlword
		                    //ROS_INFO("Node %d - STEP 1 - faultResetControlword\n", nodeID);
		                    faultResetControlword(nodeID);       //TODO replace with RPDO                  
		                }
		            }  
		        }                                            
		        break;
		    }               
		    default :
		    {
		        ROS_INFO("Unknown frame [%02X][%02X %02X %02X %02X %02X %02X %02X %02X]\n", canmsg->id, canmsg->data[7], canmsg->data[6], canmsg->data[5], canmsg->data[4], canmsg->data[3], canmsg->data[2], canmsg->data[1], canmsg->data[0]);    
		    }                        
		} //end switch

		//publish data
		osa_msgs::MotorDataMultiArray motorData_ma;

		//create the data multi array
		motorData_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
		motorData_ma.layout.dim[0].size = NUMBER_OF_MOTORS;
		motorData_ma.layout.dim[0].stride = NUMBER_OF_MOTORS;
		motorData_ma.layout.dim[0].label = "motors";
		motorData_ma.layout.data_offset = 0;
		motorData_ma.motorData.clear();
		motorData_ma.motorData.resize(NUMBER_OF_MOTORS);

		for(int i=0; i<NUMBER_OF_MOTORS; i++)
		{
			motorData_ma.motorData[i].position = position[i];
			motorData_ma.motorData[i].current = current[i];
			motorData_ma.motorData[i].status = statusword[i];	
		}

		//ROS_INFO("Publish motor data\n");
		data_pub.publish(motorData_ma);

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
void sendMotorCmdMultiArray_cb(const osa_msgs::MotorCmdMultiArrayConstPtr& motorCmd_ma)
{	
	//toggle flag, a message has been received
	msgReceived = true;

	ROS_INFO("msgReceived");

	for(int i=0; i<NUMBER_OF_MOTORS; i++)
	{
		uint8_t node_ID = motorCmd_ma->motorCmd[i].nodeID;
		uint8_t command = motorCmd_ma->motorCmd[i].command;
		int32_t value = motorCmd_ma->motorCmd[i].value;

		//ROS_INFO("cmd[%d] val[%d]\n", command, value);

		switch(command)
		{
		    case SET_TARGET_POSITION:
		    {
			setTargetPosition(node_ID, value);
			break;
		    }

		    case SET_TARGET_VELOCITY:
		    {
			setTargetVelocity(node_ID, value);
			break;
		    }

		    case SET_PROFILE_ACCELERATION:
		    {
			setProfileAcceleration(node_ID, value);
			break;
		    }

		    case SET_PROFILE_DECELERATION:
		    {
			setProfileDeceleration(node_ID, value);
			break;
		    }

		    case SET_PROFILE_VELOCITY:
		    {
			setProfileVelocity(node_ID, value);
			break;
		    }

		    case SET_OUTPUT_CURRENT_LIMIT:
		    {
			setOutputCurrentLimit(node_ID, value);
			break;
		    }
		    
		    case SET_CONTROLWORD:
		    {
			setControlword(node_ID, value);
			break;
		    }

		    case SET_CURRENT_MODE_SETTING_VALUE:
		    {
			setCurrentModeSettingValue(node_ID, value);
			break;
		    }

		    case SET_MAXIMAL_SPEED_IN_CURRENT_MODE:
		    {
			setMaximalSpeedInCurrentMode(node_ID, value);
			break;
		    }

		    case SET_MODES_OF_OPERATION:
		    {
			switch(value)         
			{
			    case INTERPOLATED_POSITION_MODE:
			    {
				setModesOfOperation(node_ID, VALUE_INTERPOLATED_POSITION_MODE);    				                      
				break;
			    }
		
			    case PROFILE_VELOCITY_MODE:
			    {
				setModesOfOperation(node_ID, VALUE_PROFILE_VELOCITY_MODE);				
				break;
			    }
		
			    case PROFILE_POSITION_MODE:
			    {				      
				setModesOfOperation(node_ID, VALUE_PROFILE_POSITION_MODE); 				     
				break;
			    }
		
			    case POSITION_MODE:
			    {
				setModesOfOperation(node_ID, VALUE_POSITION_MODE);				                
				break;
			    }
		
			    case VELOCITY_MODE:    
			    {     
				setModesOfOperation(node_ID, VALUE_VELOCITY_MODE);				  
				break;
			    }
		
			    case CURRENT_MODE:
			    {
				setModesOfOperation(node_ID, VALUE_CURRENT_MODE);				
				break;
			    }
		
			    case CYCLIC_SYNCHRONOUS_TORQUE_MODE:  
			    {              
				setModesOfOperation(node_ID, VALUE_CYCLIC_SYNCHRONOUS_TORQUE_MODE);				
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



/*! \fn int main(int argc, char** argv)
 *  \brief main function.
 *  \param argc
 *  \param argv
 *  \return int
 */
int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "osa_socketcan_node");
	ros::NodeHandle nh("~");

	ros::Rate r(15); //15

	ROS_INFO("***** OSA_SlaveBoard - The Robot Studio *****\n"); 

	ROS_INFO("*** Init variables ***\n");

	/*! Variables */
	//ros::Publisher txFrame_pub;/*! Variables */
	motorArray[NUMBER_OF_MOTORS] = {0, NOT_USED, NONE, false, PROFILE_VELOCITY_MODE, 0};
	//canFrame[NUMBER_OF_MOTORS];
	numberEposBoards = 0;

	//DigitalOut ledchain[] = {(LED1), (LED2), (LED3), (LED4)}; //used for debugging
	//char data[8];

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


	ROS_INFO("*** Grab parameters ***\n");

	// Parameters
	std::string can_device_str;
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

	// Grab the parameters
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
	nh.param("value2", value2_int, int(0));

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
	
	ROS_INFO("*** Init Publishers and Subsribers ***\n");
	
	//Subsribers and publishers
	ros::Subscriber rxFrame_sub = nh.subscribe("/received_messages", 8, receive_messages_cb); //check size of FIFO : works with 8 (2 motors*(2 request + 2 answers))
	//txFrame_pub = nh.advertise<can_msgs::Frame>("/sent_messages", 1);
	//txFrame_pub = new ros::Publisher(nh.advertise<can_msgs::Frame>("/sent_messages", 8, true)); //latch message
	txFrame_pub = new ros::Publisher(nh.advertise<can_msgs::Frame>("/sent_messages", 1)); //latch message

	//ROS_INFO("*** Init Publishers and Subsribers1 ***\n");
	ros::Subscriber cmd_sub = nh.subscribe("/motor_cmd_array", 1, sendMotorCmdMultiArray_cb); //receive commands here and translate them into CAN frames and send to /sent_messages
	data_pub = nh.advertise<osa_msgs::MotorDataMultiArray>("/motor_data_array", 1); //Publish the data received on /receive_messages

	//wait for the Publisher/Subscriber to connect
	ROS_INFO("--- wait for the Publisher/Subscriber to connect ---\n"); 

	ros::Rate poll_rate(100);
	while(txFrame_pub->getNumSubscribers() == 0)
	{
    		poll_rate.sleep();
	}

	int i = 0; //msg
	int j = 0; //byte number

	//TEST assume 2 EPOS4
	numberEposBoards = 2;

	ROS_INFO("--- Initialise EPOS boards ---\n");    
	//power or pushbutton reset
        for(int nodeID=1; nodeID<=numberEposBoards; nodeID++)
        {
            if(initEposBoard(nodeID) != EPOS_OK) 
            {
                ROS_INFO("initEposBoard error");
                return -1; //exit the main function and return fault if an initialization failed
            }
        }
	

	ROS_INFO("--- Calibrate Arm ---\n");
        if(calibrate() == EPOS_ERROR) 
        {
            ROS_INFO("Calibration error, check the mode/value of your config file, only use Profile modes for EPOS2/EPOS4 and Current mode for EPOS2.");
            return -1;   
        }

	//gather first pack of data
	//get the sensor values       
	getData();      

	//then start the main loop
	ROS_INFO("--- Start main loop ---\n"); 
    
	//Main loop
	while(ros::ok())
	{    	
		getData();    	
		ros::spinOnce(); //receive CAN frame : motor data and emergency frames; receive motor commands, publish motor data
		
		r.sleep();
	}
}

