/*
 * Copyright (c) 2017, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
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
 * @file eposCmd.cpp
 * @author Cyril Jourdan
 * @date Mar 02, 2017
 * @version 2.0.0
 * @brief implementation file for controlling EPOS2 from Maxon motor, over CAN bus
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 27, 2013
 */
 
/*! Includes */
#include "../include/robot.h"
#include "../include/eposCmd.h"

/*! Variables */
ros::Publisher *txFrame_pub;
/*
//ros::Publisher txFrame_pub;
Motor motorArray[NUMBER_OF_MOTORS] = {0, NOT_USED, NONE, false, PROFILE_VELOCITY_MODE, 0};
can_msgs::Frame canFrame[NUMBER_OF_MOTORS];
uint8_t numberEposBoards = 0;

//DigitalOut ledchain[] = {(LED1), (LED2), (LED3), (LED4)}; //used for debugging
char data[8];

ActivatedModeOfOperation activMode[NUMBER_OF_MOTORS] = {PROFILE_VELOCITY_MODE}; //
ROSCommand rosCmd[NUMBER_OF_MOTORS] = {SEND_DUMB_MESSAGE}; // 

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

//cantoeposWrite publish a CAN frame topic
void cantoeposWrite(int id, char* data, char len)
{
	can_msgs::Frame frame;
	//msgs definition
	//Header header
	//uint32 id
	//bool is_rtr
	//bool is_extended
	//bool is_error
	//uint8 dlc
	//uint8[8] data

	frame.is_extended = false;
	frame.is_rtr = false;
	frame.is_error = false;
	frame.id = id;
	frame.dlc = 8;

	//401#0F0088130000E803
	for(int i=0; i<8; i++)
	{
		frame.data[i] = data[i];
	}


	frame.header.frame_id = "1";  // "0" for no frame.
	frame.header.stamp = ros::Time::now();
	
	//publish the CAN frame
	txFrame_pub->publish(frame);
}

void transmitPDOWrite(int txPDO_cob_id, int nodeID)
{
	can_msgs::Frame frame;
	//msgs definition
	//Header header
	//uint32 id
	//bool is_rtr
	//bool is_extended
	//bool is_error
	//uint8 dlc
	//uint8[8] data

	frame.is_extended = false;
	frame.is_rtr = true; //request only
	frame.is_error = false;
	frame.id = txPDO_cob_id + nodeID;
	
	if(txPDO_cob_id == 0x180)
	{
		frame.dlc = 8;
	}
	else if(txPDO_cob_id == 0x280)
	{
		frame.dlc = 6;
	}

	//printf("frame.id = %X\n", frame.id);
	/*//401#0F0088130000E803
	for(int i=0; i<8; i++)
	{
		frame.data[i] = data[i];
	}
*/

	frame.header.frame_id = "1";  // "0" for no frame.
	frame.header.stamp = ros::Time::now();
	
	//publish the CAN frame
	txFrame_pub->publish(frame);

	ros::Duration(0.01).sleep();    
}

/*! \fn void setNMT(const uint8_t nodeID, uint8_t cs)
 *  \brief 
 *  \param nodeID identifier of the node on the CAN bus.
 *  \return void
 */
void setNMT(const uint8_t nodeID, uint8_t cs)
{
    //Firmware Spec 7.3
    ////CANMessage canmsg; //test
    data[0] = cs;
    data[1] = nodeID;
    cantoeposWrite(CAN_NMT_ID, data, 2);

    //ros::Duration(0.01).sleep();  
	ros::Duration(0.1).sleep();    
}

/*! \fn int8_t setObjectSDO(const uint8_t nodeID, const int32_t object, int32_t value)
 *  \brief 
 *  \param nodeID identifier of the node on the CAN bus.
 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
 */
int8_t setObjectSDO(const uint8_t nodeID, const int32_t object, int32_t value)
{
    ////CANMessage canmsg; 
    uint8_t nbByteObject;
    uint8_t nbByte = 0;
    uint32_t timeout = 0;
    
    //nbByteObject = object & 0x000000FF;
    nbByteObject = (uint8_t)object;
    
    switch(nbByteObject)
    {
        case 0x08 : 
	{
            nbByteObject = WRITING_OBJECT_1_BYTE;
            nbByte = 1;
            break;
        }
    
        case 0x10 : 
	{
            nbByteObject = WRITING_OBJECT_2_BYTE;
            nbByte = 2;
            break;
        }
            
        case 0x20 : 
	{
            nbByteObject = WRITING_OBJECT_4_BYTE;
            nbByte = 4;
            break;
        }
        
        default :
	{
            return EPOS_ERROR;  
        }          
    }   
    
    data[0] = nbByteObject;
    data[1] = (uint8_t)(object >> 16); 
    data[2] = (uint8_t)(object >> 24); 
    data[3] = (uint8_t)(object >> 8);
    data[4] = (uint8_t)value;
    data[5] = (uint8_t)(value >> 8);
    data[6] = (uint8_t)(value >> 16);
    data[7] = (uint8_t)(value >> 24);
    cantoeposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + nodeID, data, 4 + nbByte);

//TODO poll the frame publisher to read the reply
/*   
 while (!(cantoepos.read(canmsg))) 
    {
        ros::Duration(0.000001).sleep(); //1us//use a timeout instead
        
        if(timeout >= TIMEOUT)
        {
            //printf("setObjectSDO TIMEOUT\r\n");
            return EPOS_ERROR;
        }
        
        timeout++;
    }
*/
    //ros::Duration(0.01).sleep();
	ros::Duration(0.1).sleep();
    
    //printf("setObjectSDO ACK received\r\n");
    
    //printf("setSDO[%d] [%02X %02X %02X %02X %02X %02X %02X %02X]\n\r", nodeID, data[7], data[6], data[5], data[4], data[3], data[2], data[1], data[0]);
          
    return EPOS_OK;
}

/*! \fn int8_t getObjectSDO(const uint8_t nodeID, const int32_t object, int32_t value)
 *  \brief 
 *  \param nodeID identifier of the node on the CAN bus.
 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
 */
int8_t getObjectSDO(const uint8_t nodeID, const int32_t object, int32_t value)
{
    //TODO
    return EPOS_OK;
}

/*! \fn int8_t setPDO(const uint8_t nodeID, uint16_t pdoIdx, uint8_t subIdx, uint32_t value, uint8_t nbByte)
 *  \brief 
 *  \param nodeID identifier of the node on the CAN bus.
 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
 */
int8_t setPDO(const uint8_t nodeID, uint16_t pdoIdx, uint8_t subIdx, uint32_t value, uint8_t nbByte)
{
    //CANMessage canmsg; 
    uint8_t nbByteObject;
        
    switch(nbByte)
    {
        case 1 : 
	{
            nbByteObject = WRITING_OBJECT_1_BYTE;
            break;
        }
    
        case 2 : 
	{
            nbByteObject = WRITING_OBJECT_2_BYTE;
            break;
        }
    
        case 4 : 
	{
            nbByteObject = WRITING_OBJECT_4_BYTE;
            break;
        }

        default :
	{
            return EPOS_ERROR;     
	}       
    }   
    
    data[0] = nbByteObject;
    data[1] = (uint8_t)pdoIdx;
    data[2] = (uint8_t)(pdoIdx >> 8);
    data[3] = subIdx;
    data[4] = (uint8_t)value;
    data[5] = (uint8_t)(value >> 8);
    data[6] = (uint8_t)(value >> 16);
    data[7] = (uint8_t)(value >> 24);
    cantoeposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + nodeID, data, 4 + nbByte);
    
/*
    while(!(cantoepos.read(canmsg))) 
    {
        ros::Duration(0.000001).sleep();
    }
*/
    //ros::Duration(0.01).sleep();  
	ros::Duration(0.1).sleep();  
          
    return EPOS_OK;
}

/*! \fn int8_t setModeOfOperationSDO(const uint8_t nodeID, uint8_t mode)
 *  \brief set the mode of operation using SDO.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param mode 
 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
 */
int8_t setModeOfOperationSDO(const uint8_t nodeID, int8_t mode)
{
    //Switch Mode of Operation  (Firmware Spec 8.2.89)
    
    //CANMessage canmsg;
 
    data[0] = WRITING_OBJECT_1_BYTE;
    data[1] = (uint8_t)OBJECT_MODE_OF_OPERATION_INDEX;
    data[2] = (uint8_t)(OBJECT_MODE_OF_OPERATION_INDEX >> 8);
    data[3] = OBJECT_MODE_OF_OPERATION_SUBINDEX;
    data[4] = mode;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    
    cantoeposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + nodeID, data, 5);
/*
    while (!(cantoepos.read(canmsg))) 
    {
        ros::Duration(0.000001).sleep();
    }
*/   
    //set activMode        
    switch(mode)
    {    
        case 7 : //VALUE_INTERPOLATED_POSITION_MODE : 
	{
            activMode[nodeID-1] = INTERPOLATED_POSITION_MODE;
            break;
        }

        case 3 : //VALUE_PROFILE_VELOCITY_MODE : 
	{
            activMode[nodeID-1] = PROFILE_VELOCITY_MODE;
            break;
        }
            
        case 1 : //VALUE_PROFILE_POSITION_MODE : 
	{
            activMode[nodeID-1] = PROFILE_POSITION_MODE;
            break;
        }
            
        case -1 : //VALUE_POSITION_MODE : 
	{
            activMode[nodeID-1] = POSITION_MODE;
            break;
        }
            
        case -2 : //VALUE_VELOCITY_MODE : 
	{
            activMode[nodeID-1] = VELOCITY_MODE;
            break;
        }
            
        case -3 : //VALUE_CURRENT_MODE :  
	{           
            activMode[nodeID-1] = CURRENT_MODE;
            break;
        }
        
        case 10 : //VALUE_CYCLIC_SYNCHRONOUS_TORQUE_MODE : 
	{
            activMode[nodeID-1] = CYCLIC_SYNCHRONOUS_TORQUE_MODE;
            break; 
        }
        
        default : 
	{
            printf("Wrong mode value\r\n");
            return EPOS_ERROR;
        }
    }
    
    ros::Duration(0.01).sleep();
    
    return EPOS_OK;
}

/*! \fn void shutdownControlword(const uint8_t nodeID)
 *  \brief 
 *  \param nodeID identifier of the node on the CAN bus
 *  \return void
 */
void shutdownControlword(const uint8_t nodeID)
{
    //CANMessage canmsg;
 
    //Shutdown Controlword Firmware Spec 8.2.84 bit wise 0xxx x110 so 0x0006
    data[0] = WRITING_OBJECT_2_BYTE;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x06;     //Low Byte
    data[5] = 0x00;     //High Byte
    data[6] = 0x00;
    data[7] = 0x00;
    cantoeposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + nodeID, data, 6);
/*
    while (!(cantoepos.read(canmsg))) 
    {
        ros::Duration(0.000001).sleep();
    }
*/
    //ros::Duration(0.01).sleep();
	ros::Duration(0.1).sleep();
}

/*! \fn void shutdownControlwordIT(const uint8_t nodeID)
 *  \brief 
 *  \param nodeID identifier of the node on the CAN bus
 *  \return void
 */
void shutdownControlwordIT(const uint8_t nodeID)
{
    //CANMessage canmsg;
 
    //Shutdown Controlword Firmware Spec 8.2.84 bit wise 0xxx x110 so 0x0006
    data[0] = WRITING_OBJECT_2_BYTE;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x06;     //Low Byte
    data[5] = 0x00;     //High Byte
    data[6] = 0x00;
    data[7] = 0x00;
    cantoeposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + nodeID, data, 6);    
}

/*! \fn void switchOnEnableOperationControlword(const uint8_t nodeID)
 *  \brief 
 *  \param nodeID identifier of the node on the CAN bus
 *  \return void
 */
void switchOnEnableOperationControlword(const uint8_t nodeID)
{
    //CANMessage canmsg;
 
    //Switch On & Enable Operation Controlword Firmware Spec 8.2.84 bit wise 0xxx 1111 so 0x000F
    data[0] = WRITING_OBJECT_2_BYTE;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x0F;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    cantoeposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + nodeID, data, 6); 
/*
    while(!(cantoepos.read(canmsg))) 
    {
        ros::Duration(0.000001).sleep();  
    }
    ros::Duration(0.126).sleep();     //Tested 5ms not enough, 6ms seems ok
*/
}

/*! \fn void switchOnEnableOperationControlwordIT(const uint8_t nodeID)
 *  \brief 
 *  \param nodeID identifier of the node on the CAN bus
 *  \return void
 */
void switchOnEnableOperationControlwordIT(const uint8_t nodeID)
{
    //CANMessage canmsg;
 
    //Switch On & Enable Operation Controlword Firmware Spec 8.2.84 bit wise 0xxx 1111 so 0x000F
    data[0] = WRITING_OBJECT_2_BYTE;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x0F;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    cantoeposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + nodeID, data, 6);     
}

/*! \fn void faultResetControlword(const uint8_t nodeID)
 *  \brief 
 *  \param nodeID identifier of the node on the CAN bus
 *  \return void
 */
void faultResetControlword(const uint8_t nodeID)
{
    //CANMessage canmsg;
 
    //Fault reset Controlword Firmware Spec 8.2.84 bit wise 0xxx x110 so 0x0006
    data[0] = WRITING_OBJECT_2_BYTE;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x80;     //Low Byte
    data[5] = 0x00;     //High Byte
    data[6] = 0x00;
    data[7] = 0x00;
    cantoeposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + nodeID, data, 6);    
}

/*! \fn int8_t initEposBoard(const uint8_t nodeID)
 *  \brief This funstion initialize the EPOS board of the specified node ID. It will stop if the motor type is set to NONE.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure or if the node is not responding.
 */
int8_t initEposBoard(const uint8_t nodeID)
{
    //CANMessage canmsg;
    
    //if(nodeID ==1) setNMT(nodeID, CS_RESET_NODE); 
    
    printf("\t- initialise board ID = %d...", nodeID);  
       
    //if(cantoepos.frequency(1000000) != 1)  return EPOS_ERROR;
    
    if(motorArray[nodeID-1].controllerType == EPOS2)
    {
        //First reset the node, that also clears the red LED that sometimes appears at the end of the CAN bus
        setNMT(nodeID, CS_RESET_COMMUNICATION);
        ros::Duration(0.1).sleep();
        setNMT(nodeID, CS_RESET_NODE);    
        ros::Duration(0.1).sleep();
    }
    
    setNMT(nodeID, CS_ENTER_PRE_OPERATIONAL);
    ros::Duration(0.1).sleep();
    
    switch(motorArray[nodeID-1].motorType)    
    {    
        case NONE : 
	{
            printf("...set to NONE. STOP here.\n\r");
            printf("\tAll boards are initialised\n\r");
            return EPOS_ERROR;
            //break;
        }

        case DCX10 : //!< return error if it is not a DC type of motor
	{
            if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK) 
            {
                printf("DCX10 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    setObjectSDO(nodeID, OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(nodeID, OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4 - TODO!");
                    return EPOS_ERROR;
                }
                else
                {
                    printf("no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
                printf("...not a DC motor\n\r");
                return EPOS_ERROR;
            }
            break;
        }
            
        case DCX14 : //!< return error if it is not a DC type of motor
	{
            if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK) 
            {
                printf("DCX14 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    setObjectSDO(nodeID, OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(nodeID, OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4 - TODO!");
                    return EPOS_ERROR;
                }
                else
                {
                    printf("no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
                printf("...not a DC motor\n\r");
                return EPOS_ERROR;
            }
            break;
        }
            
        case DCX16 : //!< return error if it is not a DC type of motor
	{
            if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                printf("DCX16 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    setObjectSDO(nodeID, OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(nodeID, OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4 - TODO!");
                    return EPOS_ERROR;
                }
                else
                {
                    printf("no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
                printf("...not a DC motor\n\r");
                return EPOS_ERROR;
            }
            break;
        }
            
        case DCX22 : //!< return error if it is not a DC type of motor
	{
            if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                printf("DCX22 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    setObjectSDO(nodeID, OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(nodeID, OBJECT_OUTPUT_CURRENT_LIMIT, 0x0FA0);
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4 - TODO!");
                    return EPOS_ERROR;
                }
                else
                {
                    printf("no controller selected!");
                    return EPOS_ERROR;
                }
                
                
               /* 
                setObjectSDO(nodeID, OBJECT_MAXIMAL_SPEED_IN_CURRENT_MODE, 0x000030D4);
                setObjectSDO(nodeID, OBJECT_THERMAL_TIME_CONSTANT_WINDING, 0x00B5);
                
                //Miscellaneous configuration : Set bit 0 to 1 to Disable sensor supervision by software, to prevent Position Sensor Breach Error
                setObjectSDO(nodeID, OBJECT_MISCELLANEOUS_CONFIGURATION, 0x0000);
                
                //set Position Mode parameters
                setObjectSDO(nodeID, OBJECT_MAXIMAL_FOLLOWING_ERROR, 0x0000C350);
                //setObjectSDO(nodeID, OBJECT_MAXIMAL_PROFILE_VELOCITY, 0x000061A8); //out of range ??
                setObjectSDO(nodeID, OBJECT_MAXIMAL_ACCELERATION, 0x00001332);*/
            }
            else
            {
                printf("...not a DC motor\n\r");
                return EPOS_ERROR;
            }
            break;
        }
            
        case DCX32 : //!< return error if it is not a DC type of motor
	{
            if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                printf("DCX32 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    setObjectSDO(nodeID, OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(nodeID, OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4 - TODO!");
                    return EPOS_ERROR;
                }
                else
                {
                    printf("no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
                printf("...not a DC motor\n\r");
                return EPOS_ERROR;
            }
            break;
        }
            
        case RE13 : //!< return error if it is not a DC type of motor
	{
            if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                printf("RE13 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    setObjectSDO(nodeID, OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(nodeID, OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4 - TODO!");
                    return EPOS_ERROR;
                }
                else
                {
                    printf("no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
                printf("...not a DC motor\n\r");
                return EPOS_ERROR;
            }
            break;  
        }
            
        case RE30 : //!< return error if it is not a DC type of motor
	{
            if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                printf("RE30 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    setObjectSDO(nodeID, OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(nodeID, OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4 - TODO!");
                    return EPOS_ERROR;
                }
                else
                {
                    printf("no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
                printf("...not a DC motor\n\r");
                return EPOS_ERROR;
            }
            break;
        }
        
        case ECI40 : //!< return error if it is not an EC type of motor
	{
            //if((setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER1) == EPOS_OK) || (setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) || (setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER2) == EPOS_OK))
            if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK)
            {
                printf("ECI40 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    setObjectSDO(nodeID, OBJECT_POLE_PAIR_NUMBER, 0x07);
                    setObjectSDO(nodeID, OBJECT_OUTPUT_CURRENT_LIMIT, 0x2530);
                    setObjectSDO(nodeID, OBJECT_QUICKSTOP_DECELERATION, 0x00002710);
                    setObjectSDO(nodeID, OBJECT_MAXIMAL_FOLLOWING_ERROR, 0x00004E20); //0x4E20 = 20000, 0xFFFFFFFE to disactivate
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4 - TODO!");
                    return EPOS_ERROR;
                }
                else
                {
                    printf("no controller selected!");
                    return EPOS_ERROR;
                }               
            }
            else
            {
                printf("...not an EC motor\n\r");
                return EPOS_ERROR;
            }
            break;   
        }
            
        case ECI52 : //!< return error if it is not an EC type of motor
	{
            //if((setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER1) == EPOS_OK) || (setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) || (setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER2) == EPOS_OK))
            if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK)
            {
                printf("ECI52 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    printf("EPOS2");
                    setObjectSDO(nodeID, OBJECT_POLE_PAIR_NUMBER, 0x07);
                    setObjectSDO(nodeID, OBJECT_OUTPUT_CURRENT_LIMIT, 0x2530);
                    setObjectSDO(nodeID, OBJECT_QUICKSTOP_DECELERATION, 0x00002710);
                    setObjectSDO(nodeID, OBJECT_MAXIMAL_FOLLOWING_ERROR, 0x00004E20); //0x4E20 = 20000, 0xFFFFFFFE to disactivate
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    printf("EPOS4 - TODO!");
                    return EPOS_ERROR;
                }
                else
                {
                    printf("no controller selected!");
                    return EPOS_ERROR;
                }               
            }
            else
            {
                printf("...not an EC motor\n\r");
                return EPOS_ERROR;
            }
            break;             
        }
            
        case EC90 : //!< return error if it is not an EC type of motor
	{
            //if((setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER1) == EPOS_OK) || (setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) || (setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER2) == EPOS_OK))
            //if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) //EC_MOTOR_HALL_ENCODER1 //EC_MOTOR_HALL
            //{
                printf("EC90 - ");
                
                if(motorArray[nodeID-1].controllerType == EPOS2)
                {
                    if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) //EC_MOTOR_HALL_ENCODER1 //EC_MOTOR_HALL
                    {
                        printf("EPOS2");
                        setObjectSDO(nodeID, OBJECT_POLE_PAIR_NUMBER, 0x0C);
                        setObjectSDO(nodeID, OBJECT_OUTPUT_CURRENT_LIMIT, 0x2530);
                        setObjectSDO(nodeID, OBJECT_QUICKSTOP_DECELERATION, 0x00002710);
                        setObjectSDO(nodeID, OBJECT_MAXIMAL_FOLLOWING_ERROR, 0x00004E20); //0x4E20 = 20000, 0xFFFFFFFE to disactivate
                    }
                    else
                    {
                        printf("...not an EC motor\n\r");
                        return EPOS_ERROR;
                    }
                }
                else if(motorArray[nodeID-1].controllerType == EPOS4)
                {
                    if(setObjectSDO(nodeID, OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) //EC_MOTOR_HALL_ENCODER1 //EC_MOTOR_HALL
                    {
                        printf("EPOS4");
                        setObjectSDO(nodeID, OBJECT_EPOS4_POLE_PAIR_NUMBER, 0x0C);
                        setObjectSDO(nodeID, OBJECT_EPOS4_OUTPUT_CURRENT_LIMIT, 0x2530);
                        setObjectSDO(nodeID, OBJECT_QUICKSTOP_DECELERATION, 0x00002710);
                        setObjectSDO(nodeID, OBJECT_EPOS4_FOLLOWING_ERROR_WINDOW, 0x00004E20); //0x4E20 = 20000, 0xFFFFFFFE to disactivate
                    }
                    else
                    {
                        printf("...not an EC motor\n\r");
                        return EPOS_ERROR;
                    }
                }
                else
                {
                    printf("no controller selected!");
                    return EPOS_ERROR;
                }    
            /*}
            else
            {
                printf("...not an EC motor\n\r");
                return EPOS_ERROR;
            }*/
            break;        
        }
        
        default : 
	{
            printf("...config file value incorrect!\n\r");
            return EPOS_ERROR; //!< return ERROR if the value is not correct
            //break;
        }
    }
    
    printf("...param");
    
    if(motorArray[nodeID-1].controllerType == EPOS2)
    {        
        setObjectSDO(nodeID, OBJECT_MAXIMAL_PROFILE_VELOCITY, 0x000061A8); //0x61A8 = 25000, 0x1388 = 5000
        //Miscellaneous configuration : Set bit 0 to 1 to Disable sensor supervision by software, to prevent Position Sensor Breach Error
        setObjectSDO(nodeID, OBJECT_MISCELLANEOUS_CONFIGURATION, 0x00000000);
        setObjectSDO(nodeID, OBJECT_MAXIMAL_FOLLOWING_ERROR, 0x00004E20);
        setObjectSDO(nodeID, OBJECT_MAXIMAL_ACCELERATION, 0x00002EE0); //0x00004332 //0x00002EE0 //0x00001332 //0x00000FFF ; 0x00002FFF //normal value : 0x00001332     
        setObjectSDO(nodeID, OBJECT_MAXIMAL_SPEED_IN_CURRENT_MODE, 0x000061A8);  //3E8 = 1000 rpm  
    }
    else if(motorArray[nodeID-1].controllerType == EPOS4)
    {
        setObjectSDO(nodeID, OBJECT_EPOS4_MAX_PROFILE_VELOCITY, 0x0000C350); //0xC350 = 50000, 0x61A8 = 25000, 0x1388 = 5000
        //Miscellaneous configuration : Set bit 0 to 1 to Disable sensor supervision by software, to prevent Position Sensor Breach Error
        setObjectSDO(nodeID, OBJECT_EPOS4_AXIS_CONFIGURATION_MISCELLANEOUS, 0x00000000);
        setObjectSDO(nodeID, OBJECT_EPOS4_FOLLOWING_ERROR_WINDOW, 0x00004E20);
        setObjectSDO(nodeID, OBJECT_EPOS4_MAX_ACCELERATION, 0x00002EE0); //0x00004332 //0x00002EE0 //0x00001332 //0x00000FFF ; 0x00002FFF //normal value : 0x00001332     
    }
    
    //save all the parameters
    setObjectSDO(nodeID, OBJECT_STORE_PARAMETERS, SAVE_ALL);
    
    setNMT(nodeID, CS_RESET_COMMUNICATION);
    
    //setObjectSDO(nodeID, OBJECT_PROFILE_VELOCITY, 0x000003E8);
    //setObjectSDO(nodeID, OBJECT_PROFILE_ACCELERATION, 0x00002710);
    //setObjectSDO(nodeID, OBJECT_PROFILE_DECELERATION, 0x00002710);
    //setObjectSDO(nodeID, OBJECT_MOTION_PROFILE_TYPE, 0x0001); //0= trapezoidal, 1 = sinusoidal
                                                
    setNMT(nodeID, CS_ENTER_PRE_OPERATIONAL);
    
    //set RxPDO 1
    printf("...RxPDO-1,");
    //Disable the PDO to modify it
    setPDO(nodeID, RECEIVE_PDO_1_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_1_SUBINDEX, COB_ID_RECEIVE_PDO_1_DISABLE + nodeID, 4);
    //printf("...TEST,");
    //Writing 0 first to the number of mapped objects
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_1_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x00, 1);
    //Set object 1
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_1_INDEX, MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX, OBJECT_TARGET_POSITION, 4);  //32
    //Set object 2
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_1_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_TARGET_VELOCITY, 4);    //32
    //enable RxPDO 1 with 2 objects
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_1_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x02, 1);   
    //Enable the PDO once the setup is done
    setPDO(nodeID, RECEIVE_PDO_1_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_1_SUBINDEX, COB_ID_RECEIVE_PDO_1_ENABLE + nodeID, 4); 
    
    //set RxPDO 2
    printf("2,");
    //Disable the PDO to modify it
    setPDO(nodeID, RECEIVE_PDO_2_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_2_SUBINDEX, COB_ID_RECEIVE_PDO_2_DISABLE + nodeID, 4);
    //Writing 0 first to the number of mapped objects
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_2_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x00, 1);
    //Set object 1
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_2_INDEX, MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX, OBJECT_PROFILE_ACCELERATION, 4);  //32
    //Set object 2
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_2_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_PROFILE_DECELERATION, 4);    //32
    //enable RxPDO 1 with 2 objects
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_2_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x02, 1);   
    //Enable the PDO once the setup is done
    setPDO(nodeID, RECEIVE_PDO_2_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_2_SUBINDEX, COB_ID_RECEIVE_PDO_2_ENABLE + nodeID, 4); 
    
    //set RxPDO 3
    printf("3,");
    //Disable the PDO to modify it
    setPDO(nodeID, RECEIVE_PDO_3_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_3_SUBINDEX, COB_ID_RECEIVE_PDO_3_DISABLE + nodeID, 4);
    //Writing 0 first to the number of mapped objects
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x00, 1); 
    //Set object 1
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX, OBJECT_CONTROLWORD, 4); //16  
    //Set object 2
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_PROFILE_VELOCITY, 4); //32
    //Set object 3
    if(motorArray[nodeID-1].controllerType == EPOS2)
    {
        setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, MAPPED_OBJECT_3_RECEIVE_PDO_SUBINDEX, OBJECT_OUTPUT_CURRENT_LIMIT, 4); //16
        //enable RxPDO 1 with 3 objects
        setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x03, 1); 
    }
    else if(motorArray[nodeID-1].controllerType == EPOS4) 
    {
        //setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_EPOS4_OUTPUT_CURRENT_LIMIT, 4); //32 impossible too long
        //enable RxPDO 1 with 2 objects
        setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x02, 1); 
    }
    //Enable the PDO once the setup is done
    setPDO(nodeID, RECEIVE_PDO_3_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_3_SUBINDEX, COB_ID_RECEIVE_PDO_3_ENABLE + nodeID, 4); 
    
    //set RxPDO 4
    printf("4"); 
    //Disable the PDO to modify it
    setPDO(nodeID, RECEIVE_PDO_4_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_4_SUBINDEX, COB_ID_RECEIVE_PDO_4_DISABLE + nodeID, 4); 
    //Writing 0 first to the number of mapped objects
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x00, 1);
    //Set object 1
    if(motorArray[nodeID-1].controllerType == EPOS2) setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX, OBJECT_CURRENT_MODE_SETTING_VALUE, 4);  //16 //Current Mode
    else if(motorArray[nodeID-1].controllerType == EPOS4) setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX, OBJECT_EPOS4_TARGET_TORQUE, 4); //16 //Cyclic Synchronous Torque Mode
    //Set object 2
    if(motorArray[nodeID-1].controllerType == EPOS2) setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_MAXIMAL_SPEED_IN_CURRENT_MODE, 4);  //32
    else if(motorArray[nodeID-1].controllerType == EPOS4) setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_EPOS4_MAX_MOTOR_SPEED, 4);  //32
    //Set object 3
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, MAPPED_OBJECT_3_RECEIVE_PDO_SUBINDEX, OBJECT_MODES_OF_OPERATION, 4); //8    
    //enable RxPDO 1 with 3 objects
    setPDO(nodeID, MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x03, 1);   
    //Enable the PDO once the setup is done
    setPDO(nodeID, RECEIVE_PDO_4_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_4_SUBINDEX, COB_ID_RECEIVE_PDO_4_ENABLE + nodeID, 4); 
    
    //set TxPDO 1
    printf("...TxPDO-1,");
    //Writing 0 first to the number of mapped objects
    setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x00, 1);  
    //Set object 1          
    setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX, MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX, OBJECT_POSITION_ACTUAL_VALUE, 4);    //32
    //Set object 2
    if(motorArray[nodeID-1].controllerType == EPOS2) setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX, MAPPED_OBJECT_2_TRANSMIT_PDO_SUBINDEX, OBJECT_VELOCITY_ACTUAL_VALUE_AVERAGED, 4);  //32
    else if(motorArray[nodeID-1].controllerType == EPOS4) setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX, MAPPED_OBJECT_2_TRANSMIT_PDO_SUBINDEX, OBJECT_EPOS4_VELOCITY_ACTUAL_VALUE_AVERAGED, 4); //32
    //enable TxPDO with 2 objects      
    setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x02, 1);
    
    //set TxPDO 2 
    printf("2,");
    setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x00, 1);  
    
    if(motorArray[nodeID-1].controllerType == EPOS2)
    {
        //Set object 1           
        setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX, OBJECT_CURRENT_ACTUAL_VALUE_AVERAGED, 4);   //16 
        //Set object 2        
        setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, MAPPED_OBJECT_2_TRANSMIT_PDO_SUBINDEX, OBJECT_FOLLOWING_ERROR_ACTUAL_VALUE, 4);   //16
        //Set object 3   
        setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, MAPPED_OBJECT_3_TRANSMIT_PDO_SUBINDEX, OBJECT_STATUSWORD, 4);   //16
        //enable TxPDO with 3 objects                     
        setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x03, 1);   
    }
    else if(motorArray[nodeID-1].controllerType == EPOS4)
    {
        //TODO check the corresponding functions
        //Set object 1           
        setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX, OBJECT_EPOS4_CURRENT_ACTUAL_VALUE_AVERAGED, 4);   //32
        //Set object 2   
        setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, MAPPED_OBJECT_2_TRANSMIT_PDO_SUBINDEX, OBJECT_STATUSWORD, 4);   //16
        //enable TxPDO with 3 objects                     
        setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x02, 1);   
    }
    
    //set TxPDO 3
    printf("3,"); 
    //Writing 0 first to the number of mapped objects
    setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_3_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x00, 1);  
    //Set object 1          
    setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_3_INDEX, MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX, OBJECT_MODES_OF_OPERATION_DISPLAY, 4);  //8
    //enable TxPDO with 1 objects           
    setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_3_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x01, 1);   
                   
    //set TxPDO 4
    printf("4...");
    //Writing 0 first to the number of mapped objects
    setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_4_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x00, 1); 
    //Set object 1           
    setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_4_INDEX, MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX, OBJECT_INCREMENTAL_ENCODER_1_COUNTER_AT_INDEX_PULSE, 4); //32
    //enable TxPDO with 1 objects             
    setPDO(nodeID, MAPPED_OBJECT_TRANSMIT_PDO_4_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x01, 1);  
     
    
    //set TxPDO 1 to RTR mode (request only)       
    setPDO(nodeID, TRANSMIT_PDO_1_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_1_SUBINDEX, COB_ID_TRANSMIT_PDO_1_DISABLE + nodeID, 4);   
    setPDO(nodeID, TRANSMIT_PDO_1_PARAMETER_INDEX, TRANSMISSION_TYPE_TRANSMIT_PDO_1_SUBINDEX, PARAM_TRANSMISSION_TYPE_RTR, 1);                  
    setPDO(nodeID, TRANSMIT_PDO_1_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_1_SUBINDEX, COB_ID_TRANSMIT_PDO_1_ENABLE + nodeID, 4);  
    
    //set TxPDO 2 to RTR mode (request only)       
    setPDO(nodeID, TRANSMIT_PDO_2_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_2_SUBINDEX, COB_ID_TRANSMIT_PDO_2_DISABLE + nodeID, 4);   
    setPDO(nodeID, TRANSMIT_PDO_2_PARAMETER_INDEX, TRANSMISSION_TYPE_TRANSMIT_PDO_2_SUBINDEX, PARAM_TRANSMISSION_TYPE_RTR, 1);                  
    setPDO(nodeID, TRANSMIT_PDO_2_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_2_SUBINDEX, COB_ID_TRANSMIT_PDO_2_ENABLE + nodeID, 4);  
    
    //set TxPDO 3 to RTR mode (request only)       
    setPDO(nodeID, TRANSMIT_PDO_3_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_3_SUBINDEX, COB_ID_TRANSMIT_PDO_3_DISABLE + nodeID, 4);   
    setPDO(nodeID, TRANSMIT_PDO_3_PARAMETER_INDEX, TRANSMISSION_TYPE_TRANSMIT_PDO_3_SUBINDEX, PARAM_TRANSMISSION_TYPE_RTR, 1);                  
    setPDO(nodeID, TRANSMIT_PDO_3_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_3_SUBINDEX, COB_ID_TRANSMIT_PDO_3_ENABLE + nodeID, 4);  
    
    //set TxPDO 4 to RTR mode (request only)       
    setPDO(nodeID, TRANSMIT_PDO_4_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_4_SUBINDEX, COB_ID_TRANSMIT_PDO_4_DISABLE + nodeID, 4);   
    setPDO(nodeID, TRANSMIT_PDO_4_PARAMETER_INDEX, TRANSMISSION_TYPE_TRANSMIT_PDO_4_SUBINDEX, PARAM_TRANSMISSION_TYPE_RTR, 1);                  
    setPDO(nodeID, TRANSMIT_PDO_4_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_4_SUBINDEX, COB_ID_TRANSMIT_PDO_4_ENABLE + nodeID, 4);  
    
    setNMT(nodeID, CS_START_REMOTE_NODE);    
 
    printf("mode...");
    //this also initialise activMode variable
    switch(motorArray[nodeID-1].mode)
    {    
        case INTERPOLATED_POSITION_MODE : 
	{
            setModeOfOperationSDO(nodeID, VALUE_INTERPOLATED_POSITION_MODE);
            break;
        }

        case PROFILE_VELOCITY_MODE : 
	{
            setModeOfOperationSDO(nodeID, VALUE_PROFILE_VELOCITY_MODE);
            break;
        }
            
        case PROFILE_POSITION_MODE : 
	{
            setModeOfOperationSDO(nodeID, VALUE_PROFILE_POSITION_MODE);
            break;
        }
            
        case POSITION_MODE : 
	{
            setModeOfOperationSDO(nodeID, VALUE_POSITION_MODE);
            break;
        }
            
        case VELOCITY_MODE : 
	{
            setModeOfOperationSDO(nodeID, VALUE_VELOCITY_MODE);
            break;
        }
            
        case CURRENT_MODE : 
	{
            setModeOfOperationSDO(nodeID, VALUE_CURRENT_MODE);
            break; 
        }
            
        case CYCLIC_SYNCHRONOUS_TORQUE_MODE : 
	{
            setModeOfOperationSDO(nodeID, VALUE_CYCLIC_SYNCHRONOUS_TORQUE_MODE);
            break; 
        }
        
        default :
	{
            return EPOS_ERROR;  
        }                       
    }   
    
    shutdownControlword(nodeID); 
    printf("ON"); 
    switchOnEnableOperationControlword(nodeID);  
    
    printf("...OK\n\r");
    
    return EPOS_OK;
}

/*! \fn void setTargetPosition(const uint8_t nodeID, int32_t position)
 *  \brief send a CAN frame using the Receive PDO 1, first object Target Position, the controller has to be in Profile Position Mode.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param position Target Position (in steps unit).
 *  \return void
 */
void setTargetPosition(const uint8_t nodeID, int32_t position)
{    
    //!> store the latest command.
    targetPosition[nodeID-1] = position;
    
    //!< change sign if need be.
    if(motorArray[nodeID-1].inverted == true) position = -1*position; 

    //!> fill the data in each byte.
    data[0] = (position & 0x000000FF); //LSB
    data[1] = (position & 0x0000FF00) >> 8;
    data[2] = (position & 0x00FF0000) >> 16;
    data[3] = (position & 0xFF000000) >> 24;
    data[4] = 0x00; //fill with 0 the data which correspond to the others objects defined on the Receive PDO.
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
              
    //!> send the CAN frame.
    cantoeposWrite(COB_ID_RECEIVE_PDO_1_ENABLE + nodeID, data, 8); //write 8 bytes
    
    if(motorArray[nodeID-1].controllerType == EPOS2) ros::Duration(0.0001).sleep();
    else if(motorArray[nodeID-1].controllerType == EPOS4) ros::Duration(0.001).sleep(); //TODO reduce pause
}

/*! \fn void setTargetVelocity(const uint8_t nodeID, int32_t velocity)
 *  \brief send a CAN frame using the Receive PDO 1, second object Target Velocity, the controller has to be in Profile Velocity Mode.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param velocity Target Velocity (in rpm unit).
 *  \return void
 */
void setTargetVelocity(const uint8_t nodeID, int32_t velocity)
{
    //!> store the latest command.
    targetVelocity[nodeID-1] = velocity;
    
    //!< change sign if need be.
    if(motorArray[nodeID-1].inverted == true) velocity = -1*velocity;

    //!> fill the data in each byte.
    data[0] = 0x00; //LSB //fill with 0 the data which correspond to the others objects defined on the Receive PDO.
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = (velocity & 0x000000FF);
    data[5] = (velocity & 0x0000FF00) >> 8;
    data[6] = (velocity & 0x00FF0000) >> 16;
    data[7] = (velocity & 0xFF000000) >> 24;

    //!> send the CAN frame.              
    cantoeposWrite(COB_ID_RECEIVE_PDO_1_ENABLE + nodeID, data, 8); //write 8 bytes
    
    if(motorArray[nodeID-1].controllerType == EPOS2) ros::Duration(0.0001).sleep();
    else if(motorArray[nodeID-1].controllerType == EPOS4) ros::Duration(0.001).sleep();
}

/*! \fn void setProfileAccelerationDeceleration(const uint8_t nodeID, uint32_t acceleration, uint32_t deceleration)
 *  \brief send a CAN frame using the Receive PDO 2, two objects: Profile Acceleration and Profile Deceleration.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param acceleration Profile Acceleration (in rpm/s unit) in a profile move.
 *  \param deceleration Profile Deceleration (in rpm/s unit) in a profile move.
 *  \return void
 */
void setProfileAccelerationDeceleration(const uint8_t nodeID, uint32_t acceleration, uint32_t deceleration)
{
    //!> store the latest command.
    profileAcceleration[nodeID-1] = acceleration;
    profileDeceleration[nodeID-1] = deceleration;
    
    //!> fill the data in each byte.
    data[0] = (acceleration & 0x000000FF); //LSB
    data[1] = (acceleration & 0x0000FF00) >> 8;
    data[2] = (acceleration & 0x00FF0000) >> 16;
    data[3] = (acceleration & 0xFF000000) >> 24;
    data[4] = (deceleration & 0x000000FF);
    data[5] = (deceleration & 0x0000FF00) >> 8;
    data[6] = (deceleration & 0x00FF0000) >> 16;
    data[7] = (deceleration & 0xFF000000) >> 24;
    
    //!> send the CAN frame.          
    cantoeposWrite(COB_ID_RECEIVE_PDO_2_ENABLE + nodeID, data, 8); //write 8 bytes
    
    if(motorArray[nodeID-1].controllerType == EPOS2) ros::Duration(0.0001).sleep();
    else if(motorArray[nodeID-1].controllerType == EPOS4) ros::Duration(0.001).sleep();
}

/*! \fn void setProfileAcceleration(const uint8_t nodeID, uint32_t acceleration)
 *  \brief send a CAN frame using the Receive PDO 2, two objects: user defined Profile Acceleration and stored Profile Deceleration.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param acceleration Profile Acceleration (in rpm/s unit) in a profile move.
 *  \return void
 */
void setProfileAcceleration(const uint8_t nodeID, uint32_t acceleration)
{
    setProfileAccelerationDeceleration(nodeID, acceleration, profileDeceleration[nodeID-1]);
}

/*! \fn void setProfileDeceleration(const uint8_t nodeID, uint32_t deceleration)
 *  \brief send a CAN frame using the Receive PDO 2, two objects: stored Profile Acceleration and user defined Profile Deceleration.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param deceleration Profile Deceleration (in rpm/s unit) in a profile move.
 *  \return void
 */
void setProfileDeceleration(const uint8_t nodeID, uint32_t deceleration)
{
    setProfileAccelerationDeceleration(nodeID, profileAcceleration[nodeID-1], deceleration);
}

/*! \fn void setCrtlWordProVelOutCurrLmt(const uint8_t nodeID, uint16_t ctrlWord, uint32_t velocity, uint16_t outCurrLmt)
 *  \brief send a CAN frame using the Receive PDO 3, three objects: Controlword, Profile Velocity, Output Current Limit (EPOS2 only).
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param ctrlWord Controlword. 
 *  \param velocity Profile Velocity (in rpm unit).
 *  \param outCurrLmt Output Current Limit (in mA unit), only used with EPOS2.
 *  \return void
 */
void setCrtlWordProVelOutCurrLmt(const uint8_t nodeID, uint16_t ctrlWord, uint32_t velocity, uint16_t outCurrLmt)
{
    //!> store the latest command.
    controlword[nodeID-1] = ctrlWord;
    profileVelocity[nodeID-1] = velocity;
    
    if(motorArray[nodeID-1].controllerType == EPOS2)
    {
        outputCurrentLimit[nodeID-1] = outCurrLmt;
        
        //!> fill the data in each byte.
        data[0] = (ctrlWord & 0x000000FF);
        data[1] = (ctrlWord & 0x0000FF00) >> 8;
        data[2] = (velocity & 0x000000FF); //LSB
        data[3] = (velocity & 0x0000FF00) >> 8;
        data[4] = (velocity & 0x00FF0000) >> 16;
        data[5] = (velocity & 0xFF000000) >> 24;
        data[6] = (outCurrLmt & 0x000000FF);
        data[7] = (outCurrLmt & 0x0000FF00) >> 8;
        
        //!> send the CAN frame.
        cantoeposWrite(COB_ID_RECEIVE_PDO_3_ENABLE + nodeID, data, 8); //write 8 bytes
        ros::Duration(0.0001).sleep();    
    } //TODO to finish
    else if(motorArray[nodeID-1].controllerType == EPOS4)
    {
        //!> fill the data in each byte.
        data[0] = (ctrlWord & 0x000000FF);
        data[1] = (ctrlWord & 0x0000FF00) >> 8;
        data[2] = (velocity & 0x000000FF); //LSB
        data[3] = (velocity & 0x0000FF00) >> 8;
        data[4] = (velocity & 0x00FF0000) >> 16;
        data[5] = (velocity & 0xFF000000) >> 24;
        data[6] = 0x00;
        data[7] = 0x00;
        
        //!> send the CAN frame.
        cantoeposWrite(COB_ID_RECEIVE_PDO_3_ENABLE + nodeID, data, 8); //write 8 bytes
        ros::Duration(0.001).sleep(); //TODO timing to check
    }
}

/*! \fn void setProfileVelocity(const uint8_t nodeID, uint32_t velocity)
 *  \brief send a CAN frame using the Receive PDO 3, three objects: user defined Profile Velocity, stored Output Current Limit and stored Controlword.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param velocity Profile Velocity (in rpm unit).
 *  \return void
 */
void setProfileVelocity(const uint8_t nodeID, uint32_t velocity)
{
    setCrtlWordProVelOutCurrLmt(nodeID, controlword[nodeID-1], velocity, outputCurrentLimit[nodeID-1]);
}

/*! \fn void setOutputCurrentLimit(const uint8_t nodeID, uint16_t current)
 *  \brief send a CAN frame using the Receive PDO 3, three objects: stored Profile Velocity, user defined Output Current Limit and stored Controlword.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param current Output Current Limit (in mA unit).
 *  \return void
 */
void setOutputCurrentLimit(const uint8_t nodeID, uint16_t current)
{
    if(motorArray[nodeID-1].controllerType == EPOS2) setCrtlWordProVelOutCurrLmt(nodeID, controlword[nodeID-1], profileVelocity[nodeID-1], current);
}

/*! \fn void setControlword(const uint8_t nodeID, uint16_t ctrlWord)
 *  \brief send a CAN frame using the Receive PDO 3, three objects: stored Profile Velocity, stored Output Current Limit and user defined Controlword.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param ctrlWord Controlword.
 *  \return void
 */
void setControlword(const uint8_t nodeID, uint16_t ctrlWord)
{
    setCrtlWordProVelOutCurrLmt(nodeID, ctrlWord, profileVelocity[nodeID-1], outputCurrentLimit[nodeID-1]);
    
   /* 
    //!> store the latest command.
    //profileVelocity[nodeID-1] = velocity;
    //outputCurrentLimit[nodeID-1] = outCurrLmt;
    controlword[nodeID-1] = ctrlWord;
    
    //!> fill the data in each byte.
    data[0] = (ctrlWord & 0x00FF);
    data[1] = (ctrlWord & 0xFF00) >> 8;
    data[2] = 0x20;
    data[3] = 0x03;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
           
    //!> send the CAN frame.   
    cantoeposWrite(COB_ID_RECEIVE_PDO_3_ENABLE + nodeID, data, 8)); //write 2 bytes
    ros::Duration(0.0001).sleep();*/
}

/*! \fn void setCurrentSpeedMode(const uint8_t nodeID, int16_t current, uint32_t maxSpeedCurr, uint16_t controlword, int8_t mode)
 *  \brief send a CAN frame using the Receive PDO 4, three objects: Current Mode Setting Value, Maximal Speed In Current Mode and Modes Of Operation.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \param current Current Mode Setting Value (in mA unit).
 *  \param maxSpeedCurr Maximal Speed In Current Mode (in rpm unit).
 *  \param mode Modes Of Operation.
 *  \return void
 */
void setCurrentSpeedMode(const uint8_t nodeID, int16_t current, uint32_t maxSpeedCurr, int8_t mode)
{
    //!> store the latest command.
    currentModeSettingValue[nodeID-1] = current; //TODO do a special case for torque mode
    maximalSpeedInCurrentMode[nodeID-1] = maxSpeedCurr;
    modesOfOperation[nodeID-1] = mode;
    
    //!< change sign if need be.
    if(motorArray[nodeID-1].inverted == true) current = -1*current;
    
    //!> fill the data in each byte.
    data[0] = (current & 0x000000FF); //LSB
    data[1] = (current & 0x0000FF00) >> 8;
    data[2] = (maxSpeedCurr & 0x000000FF);
    data[3] = (maxSpeedCurr & 0x0000FF00) >> 8;
    data[4] = (maxSpeedCurr & 0x00FF0000) >> 16;
    data[5] = (maxSpeedCurr & 0xFF000000) >> 24;
    data[6] = (mode & 0x000000FF);
    data[7] = 0x00;
    
    //!> send the CAN frame.         
    cantoeposWrite(COB_ID_RECEIVE_PDO_4_ENABLE + nodeID, data, 8); //write 8 bytes
    
    if(motorArray[nodeID-1].controllerType == EPOS2) ros::Duration(0.0001).sleep();
    else if(motorArray[nodeID-1].controllerType == EPOS4) ros::Duration(0.001).sleep();
}

/*! \fn void setCurrentModeSettingValue(const uint8_t nodeID, int16_t current)
 *  \brief send a CAN frame using the Receive PDO 4, three objects: user defined Current Mode Setting Value, stored Maximal Speed In Current Mode and stored Modes Of Operation.
 *  \param nodeID identifier of the node on the CAN bus
 *  \param current Current Mode Setting Value (in mA unit).
 *  \return void
 */
void setCurrentModeSettingValue(const uint8_t nodeID, int16_t current)
{
    setCurrentSpeedMode(nodeID, current, maximalSpeedInCurrentMode[nodeID-1], modesOfOperation[nodeID-1]);
}

/*! \fn void setMaximalSpeedInCurrentMode(const uint8_t nodeID, uint32_t maxSpeedCurr)
 *  \brief send a CAN frame using the Receive PDO 4, three objects: stored Current Mode Setting Value, user defined Maximal Speed In Current Mode and stored Modes Of Operation.
 *  \param nodeID identifier of the node on the CAN bus
 *  \param maxSpeedCurr Maximal Speed In Current Mode (in rpm unit).
 *  \return void
 */
void setMaximalSpeedInCurrentMode(const uint8_t nodeID, uint32_t maxSpeedCurr)
{
    setCurrentSpeedMode(nodeID, currentModeSettingValue[nodeID-1], maxSpeedCurr, modesOfOperation[nodeID-1]);
}

/*! \fn void setModesOfOperation(const uint8_t nodeID, int16_t current, uint32_t maxSpeedCurr, uint16_t controlword, int8_t mode)
 *  \brief send a CAN frame using the Receive PDO 4, three objects: stored Current Mode Setting Value, stored Maximal Speed In Current Mode and user defined Modes Of Operation.
 *  \param nodeID identifier of the node on the CAN bus
 *  \param mode Modes Of Operation.
 *  \return void
 */
void setModesOfOperation(const uint8_t nodeID, int8_t mode)
{ 
    //set activMode        
    switch(mode)
    {    
        case 7 : //VALUE_INTERPOLATED_POSITION_MODE : 
	{
            activMode[nodeID-1] = INTERPOLATED_POSITION_MODE;
            break;
	}
        
        case 3 : //VALUE_PROFILE_VELOCITY_MODE : 
	{
            activMode[nodeID-1] = PROFILE_VELOCITY_MODE;
            break;
	}
            
        case 1 : //VALUE_PROFILE_POSITION_MODE : 
	{
            activMode[nodeID-1] = PROFILE_POSITION_MODE;
            break;
	}
            
        case -1 : //VALUE_POSITION_MODE : 
	{
            activMode[nodeID-1] = POSITION_MODE;
            break;
	}
            
        case -2 : //VALUE_VELOCITY_MODE : 
	{
            activMode[nodeID-1] = VELOCITY_MODE;
            break;
	}
            
        case -3 : //VALUE_CURRENT_MODE :  
	{           
            activMode[nodeID-1] = CURRENT_MODE;
            break;
	}
        
        case 10 : //VALUE_CYCLIC_SYNCHRONOUS_TORQUE_MODE : 
	{
            activMode[nodeID-1] = CYCLIC_SYNCHRONOUS_TORQUE_MODE;
            break; 
	}
        
        default : 
	{
            printf("Wrong mode value\r\n");
            return;  
	}               
    }
    
    setCurrentSpeedMode(nodeID, currentModeSettingValue[nodeID-1], maximalSpeedInCurrentMode[nodeID-1], mode);
}

/*! \fn void getPositionVelocity(const uint8_t nodeID)
 *  \brief send a CAN frame using the Transmit PDO 1, two objects Position Actual Value and Velocity Actual Value Averaged, the answer frame is caught by interrupt.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \return void
 */
void getPositionVelocity(const uint8_t nodeID)
{    
	//ROS_INFO("nodeID=%d", nodeID);
	transmitPDOWrite(COB_ID_TRANSMIT_PDO_1_ENABLE, nodeID); 
}

/*! \fn void getCurrentFollErrStatusword(const uint8_t nodeID)
 *  \brief send a CAN frame using the Transmit PDO 2, two objects Current Actual Value Averaged, Following Error Actual Value and Statusword, the answer frame is caught by interrupt.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \return void
 */
void getCurrentFollErrStatusword(const uint8_t nodeID)
{    
    transmitPDOWrite(COB_ID_TRANSMIT_PDO_2_ENABLE, nodeID); 
}

/*! \fn void getModesOfOperation(const uint8_t nodeID)
 *  \brief send a CAN frame using the Transmit PDO 3, one object Modes Of Operation Display, the answer frame is caught by interrupt.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \return void
 */
void getModesOfOperation(const uint8_t nodeID)
{    
    transmitPDOWrite(COB_ID_TRANSMIT_PDO_3_ENABLE, nodeID); 
}

/*! \fn void getIncEnc1CntAtIdxPls(const uint8_t nodeID)
 *  \brief send a CAN frame using the Transmit PDO 4, one object Incremental Encoder 1 counter at Index Pulse, the answer frame is caught by interrupt.
 *  \param nodeID identifier of the node on the CAN bus.
 *  \return void
 */
void getIncEnc1CntAtIdxPls(const uint8_t nodeID)
{ 
    transmitPDOWrite(COB_ID_TRANSMIT_PDO_4_ENABLE, nodeID);
}
