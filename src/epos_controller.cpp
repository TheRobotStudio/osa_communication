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
 * @file EPOSController.cpp
 * @author Cyril Jourdan
 * @date Aug 29, 2017
 * @version 0.1.0
 * @brief Implementation file for class EPOSController
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Aug 3, 2017
 */

#include <epos_controller.h>
#include <iostream>
#include <can_msgs/Frame.h>

using namespace std;

/**
 * @brief Constructor.
 */
EPOSController::EPOSController(std::string name, std::string degree_of_freedom_type,
		int node_id, std::string controller_type,
		std::string motor_type, bool inverted,
		std::string mode, int value, ros::Publisher *tx_can_frame_pub) :
name_(name),
degree_of_freedom_type_(TENDON),
node_id_(0),
controller_type_(NOT_USED),
motor_type_(NONE),
inverted_(inverted),
mode_(PROFILE_VELOCITY_MODE),
value_(value),
tx_can_frame_pub_(tx_can_frame_pub),
data_({0}),
activ_mode_(),
ros_cmd_(SEND_DUMB_MESSAGE),
target_position_(0),
target_velocity_(0),
profile_acceleration_(0),
profile_deceleration_(0),
profile_velocity_(0),
output_current_limit_(0),
current_mode_setting_value_(0),
maximal_speed_in_current_mode_(0),
controlword_(0),
modes_of_operation_(0),
position_(0),
velocity_(0),
current_(0),
following_error_(0),
statusword_(0),
inc_enc1_cnt_at_idx_pls_(0),
board_status_(0)
{
	//Init the parameters

	//degree_of_freedom_type_
	if(!degree_of_freedom_type.compare("TENDON")) degree_of_freedom_type_ = TENDON;
	else if(!degree_of_freedom_type.compare("WHEEL")) degree_of_freedom_type_ = WHEEL;
	else if(!degree_of_freedom_type.compare("CLASSICAL")) degree_of_freedom_type_ = CLASSICAL;

	//node_id_
	if((node_id >=0) && (node_id<=255)) node_id_ = node_id; //check that the value is a uint8_t

	//controller_type_
	if(!controller_type.compare("NOT_USED")) controller_type_ = ControllerType(NOT_USED);
	else if(!controller_type.compare("EPOS2")) controller_type_ = ControllerType(EPOS2);
	else if(!controller_type.compare("EPOS4")) controller_type_ = ControllerType(EPOS4);

	//motor_type_
	if(!motor_type.compare("NONE")) motor_type_ = MotorType(NONE);
	else if(!motor_type.compare("DCX10")) motor_type_ = MotorType(DCX10);
	else if(!motor_type.compare("DCX14")) motor_type_ = MotorType(DCX14);
	else if(!motor_type.compare("DCX16")) motor_type_ = MotorType(DCX16);
	else if(!motor_type.compare("DCX22")) motor_type_ = MotorType(DCX22);
	else if(!motor_type.compare("DCX32")) motor_type_ = MotorType(DCX32);
	else if(!motor_type.compare("RE13")) motor_type_ = MotorType(RE13);
	else if(!motor_type.compare("RE30")) motor_type_ = MotorType(RE30);
	else if(!motor_type.compare("ECI40")) motor_type_ = MotorType(ECI40);
	else if(!motor_type.compare("ECI52")) motor_type_ = MotorType(ECI52);
	else if(!motor_type.compare("EC90")) motor_type_ = MotorType(EC90);

	//mode_
	if(!mode.compare("INTERPOLATED_POSITION_MODE")) mode_ = ActivatedModeOfOperation(INTERPOLATED_POSITION_MODE);
	else if(!mode.compare("PROFILE_VELOCITY_MODE")) mode_ = ActivatedModeOfOperation(PROFILE_VELOCITY_MODE);
	else if(!mode.compare("PROFILE_POSITION_MODE")) mode_ = ActivatedModeOfOperation(PROFILE_POSITION_MODE);
	else if(!mode.compare("POSITION_MODE")) mode_ = ActivatedModeOfOperation(POSITION_MODE);
	else if(!mode.compare("VELOCITY_MODE")) mode_ = ActivatedModeOfOperation(VELOCITY_MODE);
	else if(!mode.compare("CURRENT_MODE")) mode_ = ActivatedModeOfOperation(CURRENT_MODE);
	else if(!mode.compare("CYCLIC_SYNCHRONOUS_TORQUE_MODE")) mode_ = ActivatedModeOfOperation(CYCLIC_SYNCHRONOUS_TORQUE_MODE);
}

/**
 * @brief Destructor.
 */
EPOSController::~EPOSController()
{

}

int	EPOSController::setNodeID(uint8_t node_id)
{
	//check the value
	if(node_id > 0)
	{
		node_id_ = node_id;

		return 0;
	}
	else
		return -1;
}

int EPOSController::setPosition(int32_t position)
{
	position_ = position;

	return 0;
}

int EPOSController::setCurrent(int16_t current)
{
	current_ = current;

	return 0;
}

int EPOSController::setVelocity(int32_t velocity)
{
	velocity_ = velocity;

	return 0;
}

int EPOSController::setStatusword(uint16_t statusword)
{
	//check the value
	if(statusword > 0)
	{
		statusword_ = statusword;

		return 0;
	}
	else
		return -1;
}

int EPOSController::setFollowingError(int16_t following_error)
{
	following_error_ = following_error;

	return 0;
}

int EPOSController::setIncEnc1CntAtIdxPls(uint32_t inc_enc1_cnt_at_idx_pls)
{
	//check the value
	if(inc_enc1_cnt_at_idx_pls > 0)
	{
		inc_enc1_cnt_at_idx_pls_ = inc_enc1_cnt_at_idx_pls;

		return 0;
	}
	else
		return -1;
}

int EPOSController::setBoardStatus(int8_t board_status)
{
	board_status_ = board_status;

	return 0;
}

//other methods

//canToEposWrite publish a CAN frame topic
void EPOSController::canToEposWrite(int id, char* data, char len)
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
	tx_can_frame_pub_->publish(frame);
}

void EPOSController::transmitPDOWrite(int tx_pdo_cob_id)
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
	frame.id = tx_pdo_cob_id + node_id_;

	if(tx_pdo_cob_id == 0x180)
	{
		frame.dlc = 8;
	}
	else if(tx_pdo_cob_id == 0x280)
	{
		frame.dlc = 6;
	}

	//ROS_INFO("frame.id = %X", frame.id);
	/*//401#0F0088130000E803
	for(int i=0; i<8; i++)
	{
		frame.data[i] = data[i];
	}
*/

	frame.header.frame_id = "1";  // "0" for no frame.
	frame.header.stamp = ros::Time::now();

	//publish the CAN frame
	tx_can_frame_pub_->publish(frame);

	ros::Duration(0.01).sleep();
	//ros::Duration(0.001).sleep();
}

/*! \fn void setNMT(, uint8_t cs)
 *  \brief
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \return void
 */
void EPOSController::setNMT(uint8_t cs)
{
    //Firmware Spec 7.3
    ////CANMessage canmsg; //test
    data_[0] = cs;
    data_[1] = node_id_;
    canToEposWrite(CAN_NMT_ID, data_, 2);

    //ros::Duration(0.01).sleep();
	ros::Duration(0.1).sleep();
}

/*! \fn int8_t setObjectSDO(const int32_t object, int32_t value)
 *  \brief
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
 */
int8_t EPOSController::setObjectSDO(const int32_t object, int32_t value)
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

    data_[0] = nbByteObject;
    data_[1] = (uint8_t)(object >> 16);
    data_[2] = (uint8_t)(object >> 24);
    data_[3] = (uint8_t)(object >> 8);
    data_[4] = (uint8_t)value;
    data_[5] = (uint8_t)(value >> 8);
    data_[6] = (uint8_t)(value >> 16);
    data_[7] = (uint8_t)(value >> 24);
    canToEposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + node_id_, data_, 4 + nbByte);

//TODO poll the frame publisher to read the reply
/*
 while (!(cantoepos.read(canmsg)))
    {
        ros::Duration(0.000001).sleep(); //1us//use a timeout instead

        if(timeout >= TIMEOUT)
        {
            //ROS_INFO("setObjectSDO TIMEOUT");
            return EPOS_ERROR;
        }

        timeout++;
    }
*/
    //ros::Duration(0.01).sleep();
	ros::Duration(0.1).sleep();

    //ROS_INFO("setObjectSDO ACK received");

    //ROS_INFO("setSDO[%d] [%02X %02X %02X %02X %02X %02X %02X %02X]", node_id_, data[7], data[6], data[5], data[4], data[3], data[2], data[1], data[0]);

    return EPOS_OK;
}

/*! \fn int8_t getObjectSDO(const int32_t object, int32_t value)
 *  \brief
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
 */
int8_t EPOSController::getObjectSDO(const int32_t object, int32_t value)
{
    //TODO
    return EPOS_OK;
}

/*! \fn int8_t setPDO(uint16_t pdoIdx, uint8_t subIdx, uint32_t value, uint8_t nbByte)
 *  \brief
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
 */
int8_t EPOSController::setPDO(uint16_t pdoIdx, uint8_t subIdx, uint32_t value, uint8_t nbByte)
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

    data_[0] = nbByteObject;
    data_[1] = (uint8_t)pdoIdx;
    data_[2] = (uint8_t)(pdoIdx >> 8);
    data_[3] = subIdx;
    data_[4] = (uint8_t)value;
    data_[5] = (uint8_t)(value >> 8);
    data_[6] = (uint8_t)(value >> 16);
    data_[7] = (uint8_t)(value >> 24);
    canToEposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + node_id_, data_, 4 + nbByte);

/*//TODO a mechanism that wait for the returned CAN frame
    while(!(cantoepos.read(canmsg)))
    {
        ros::Duration(0.000001).sleep();
    }
*/
    ros::Duration(0.01).sleep();
	//ros::Duration(0.1).sleep();

    return EPOS_OK;
}

/*! \fn int8_t setModeOfOperationSDO(uint8_t mode)
 *  \brief set the mode of operation using SDO.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param mode
 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure.
 */
int8_t EPOSController::setModeOfOperationSDO(int8_t mode)
{
    //Switch Mode of Operation  (Firmware Spec 8.2.89)

    //CANMessage canmsg;

    data_[0] = WRITING_OBJECT_1_BYTE;
    data_[1] = (uint8_t)OBJECT_MODE_OF_OPERATION_INDEX;
    data_[2] = (uint8_t)(OBJECT_MODE_OF_OPERATION_INDEX >> 8);
    data_[3] = OBJECT_MODE_OF_OPERATION_SUBINDEX;
    data_[4] = mode;
    data_[5] = 0x00;
    data_[6] = 0x00;
    data_[7] = 0x00;

    canToEposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + node_id_, data_, 5);
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
            activ_mode_ = INTERPOLATED_POSITION_MODE;
            break;
        }

        case 3 : //VALUE_PROFILE_VELOCITY_MODE :
        {
            activ_mode_ = PROFILE_VELOCITY_MODE;
            break;
        }

        case 1 : //VALUE_PROFILE_POSITION_MODE :
        {
            activ_mode_ = PROFILE_POSITION_MODE;
            break;
        }

        case -1 : //VALUE_POSITION_MODE :
        {
            activ_mode_ = POSITION_MODE;
            break;
        }

        case -2 : //VALUE_VELOCITY_MODE :
        {
            activ_mode_ = VELOCITY_MODE;
            break;
        }

        case -3 : //VALUE_CURRENT_MODE :
        {
            activ_mode_ = CURRENT_MODE;
            break;
        }

        case 10 : //VALUE_CYCLIC_SYNCHRONOUS_TORQUE_MODE :
        {
            activ_mode_ = CYCLIC_SYNCHRONOUS_TORQUE_MODE;
            break;
        }

        default :
        {
            ROS_INFO("Wrong mode value");
            return EPOS_ERROR;
        }
    }

    ros::Duration(0.01).sleep();

    return EPOS_OK;
}

/*! \fn void shutdownControlword()
 *  \brief
 *  \param node_id_ identifier of the node on the CAN bus
 *  \return void
 */
void EPOSController::shutdownControlword()
{
    //CANMessage canmsg;

    //Shutdown Controlword Firmware Spec 8.2.84 bit wise 0xxx x110 so 0x0006
    data_[0] = WRITING_OBJECT_2_BYTE;
    data_[1] = 0x40;
    data_[2] = 0x60;
    data_[3] = 0x00;
    data_[4] = 0x06;     //Low Byte
    data_[5] = 0x00;     //High Byte
    data_[6] = 0x00;
    data_[7] = 0x00;
    canToEposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + node_id_, data_, 6);
/*
    while (!(cantoepos.read(canmsg)))
    {
        ros::Duration(0.000001).sleep();
    }
*/
    //ros::Duration(0.01).sleep();
	ros::Duration(0.1).sleep();
}

/*! \fn void shutdownControlwordIT()
 *  \brief
 *  \param node_id_ identifier of the node on the CAN bus
 *  \return void
 */
void EPOSController::shutdownControlwordIT()
{
    //CANMessage canmsg;

    //Shutdown Controlword Firmware Spec 8.2.84 bit wise 0xxx x110 so 0x0006
    data_[0] = WRITING_OBJECT_2_BYTE;
    data_[1] = 0x40;
    data_[2] = 0x60;
    data_[3] = 0x00;
    data_[4] = 0x06;     //Low Byte
    data_[5] = 0x00;     //High Byte
    data_[6] = 0x00;
    data_[7] = 0x00;
    canToEposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + node_id_, data_, 6);
}

/*! \fn void switchOnEnableOperationControlword()
 *  \brief
 *  \param node_id_ identifier of the node on the CAN bus
 *  \return void
 */
void EPOSController::switchOnEnableOperationControlword()
{
    //CANMessage canmsg;

    //Switch On & Enable Operation Controlword Firmware Spec 8.2.84 bit wise 0xxx 1111 so 0x000F
    data_[0] = WRITING_OBJECT_2_BYTE;
    data_[1] = 0x40;
    data_[2] = 0x60;
    data_[3] = 0x00;
    data_[4] = 0x0F;
    data_[5] = 0x00;
    data_[6] = 0x00;
    data_[7] = 0x00;
    canToEposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + node_id_, data_, 6);
/*
    while(!(cantoepos.read(canmsg)))
    {
        ros::Duration(0.000001).sleep();
    }
    ros::Duration(0.126).sleep();     //Tested 5ms not enough, 6ms seems ok
*/
}

/*! \fn void switchOnEnableOperationControlwordIT()
 *  \brief
 *  \param node_id_ identifier of the node on the CAN bus
 *  \return void
 */
void EPOSController::switchOnEnableOperationControlwordIT()
{
    //CANMessage canmsg;

    //Switch On & Enable Operation Controlword Firmware Spec 8.2.84 bit wise 0xxx 1111 so 0x000F
    data_[0] = WRITING_OBJECT_2_BYTE;
    data_[1] = 0x40;
    data_[2] = 0x60;
    data_[3] = 0x00;
    data_[4] = 0x0F;
    data_[5] = 0x00;
    data_[6] = 0x00;
    data_[7] = 0x00;
    canToEposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + node_id_, data_, 6);
}

/*! \fn void faultResetControlword()
 *  \brief
 *  \param node_id_ identifier of the node on the CAN bus
 *  \return void
 */
void EPOSController::faultResetControlword()
{
    //CANMessage canmsg;

    //Fault reset Controlword Firmware Spec 8.2.84 bit wise 0xxx x110 so 0x0006
    data_[0] = WRITING_OBJECT_2_BYTE;
    data_[1] = 0x40;
    data_[2] = 0x60;
    data_[3] = 0x00;
    data_[4] = 0x80;     //Low Byte
    data_[5] = 0x00;     //High Byte
    data_[6] = 0x00;
    data_[7] = 0x00;
    canToEposWrite(COB_ID_SDO_CLIENT_TO_SERVER_DEFAULT + node_id_, data_, 6);
}

/*! \fn int8_t initEposBoard()
 *  \brief This funstion initialize the EPOS board of the specified node ID. It will stop if the motor type is set to NONE.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \return int8_t EPOS_OK for success or EPOS_ERROR for failure or if the node is not responding.
 */
int8_t EPOSController::initEposBoard()
{
    //CANMessage canmsg;

    //if(node_id_ ==1) setNMT(CS_RESET_NODE);

    ROS_INFO("- initialise board ID = %d", node_id_);

    //if(cantoepos.frequency(1000000) != 1)  return EPOS_ERROR;

    if(controller_type_ == EPOS2)
    {
        //First reset the node, that also clears the red LED that sometimes appears at the end of the CAN bus
        setNMT(CS_RESET_COMMUNICATION);
        ros::Duration(0.1).sleep();
        setNMT(CS_RESET_NODE);
        ros::Duration(0.1).sleep();
    }

    setNMT(CS_ENTER_PRE_OPERATIONAL);
    ros::Duration(0.1).sleep();

    switch(motor_type_)
    {
        case NONE :
        {
        	ROS_WARN("\t...set to NONE. STOP here.");
        	ROS_INFO("All boards are initialised");
            return EPOS_ERROR;
        }

        case DCX10 : //!< return error if it is not a DC type of motor
        {
            if(setObjectSDO(OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                ROS_INFO("\tmotor type: DCX10");

                if(controller_type_ == EPOS2)
                {
                    ROS_INFO("\tcontroller type: EPOS2");
                    setObjectSDO(OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(controller_type_ == EPOS4)
                {
                	ROS_WARN("\tcontroller type: EPOS4 - not implemented!");
                    return EPOS_ERROR;
                }
                else
                {
                	ROS_WARN("\tcontroller type: no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
                ROS_WARN("\tmotor type: not a DC motor");
                return EPOS_ERROR;
            }
            break;
        }

        case DCX14 : //!< return error if it is not a DC type of motor
        {
            if(setObjectSDO(OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                ROS_INFO("\tmotor type: DCX14");

                if(controller_type_ == EPOS2)
                {
                    ROS_INFO("\tcontroller type: EPOS2");
                    setObjectSDO(OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(controller_type_ == EPOS4)
                {
                	ROS_WARN("\tcontroller type: EPOS4 - not implemented!");
                    return EPOS_ERROR;
                }
                else
                {
                	ROS_WARN("\tcontroller type: no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
            	ROS_WARN("\tmotor type: not a DC motor");
                return EPOS_ERROR;
            }
            break;
        }

        case DCX16 : //!< return error if it is not a DC type of motor
        {
            if(setObjectSDO(OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                ROS_INFO("\tmotor type: DCX16");

                if(controller_type_ == EPOS2)
                {
                    ROS_INFO("\tcontroller type: EPOS2");
                    setObjectSDO(OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(controller_type_ == EPOS4)
                {
                	ROS_WARN("\tcontroller type: EPOS4 - not implemented!");
                    return EPOS_ERROR;
                }
                else
                {
                	ROS_WARN("\tcontroller type: no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
            	ROS_WARN("\tmotor type: not a DC motor");
                return EPOS_ERROR;
            }
            break;
        }

        case DCX22 : //!< return error if it is not a DC type of motor
        {
            if(setObjectSDO(OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                ROS_INFO("\tmotor type: DCX22");

                if(controller_type_ == EPOS2)
                {
                    ROS_INFO("\tcontroller type: EPOS2");
                    setObjectSDO(OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(OBJECT_OUTPUT_CURRENT_LIMIT, 0x0FA0);
                }
                else if(controller_type_ == EPOS4)
                {
                	ROS_WARN("\tcontroller type: EPOS4 - not implemented!");
                    return EPOS_ERROR;
                }
                else
                {
                	ROS_WARN("\tcontroller type: no controller selected!");
                    return EPOS_ERROR;
                }


               /*
                setObjectSDO(OBJECT_MAXIMAL_SPEED_IN_CURRENT_MODE, 0x000030D4);
                setObjectSDO(OBJECT_THERMAL_TIME_CONSTANT_WINDING, 0x00B5);

                //Miscellaneous configuration : Set bit 0 to 1 to Disable sensor supervision by software, to prevent Position Sensor Breach Error
                setObjectSDO(OBJECT_MISCELLANEOUS_CONFIGURATION, 0x0000);

                //set Position Mode parameters
                setObjectSDO(OBJECT_MAXIMAL_FOLLOWING_ERROR, 0x0000C350);
                //setObjectSDO(OBJECT_MAXIMAL_PROFILE_VELOCITY, 0x000061A8); //out of range ??
                setObjectSDO(OBJECT_MAXIMAL_ACCELERATION, 0x00001332);*/
            }
            else
            {
            	ROS_WARN("\tmotor type: not a DC motor");
                return EPOS_ERROR;
            }
            break;
        }

        case DCX32 : //!< return error if it is not a DC type of motor
        {
            if(setObjectSDO(OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                ROS_INFO("\tmotor type: DCX32");

                if(controller_type_ == EPOS2)
                {
                    ROS_INFO("\tcontroller type: EPOS2");
                    setObjectSDO(OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(controller_type_ == EPOS4)
                {
                	ROS_WARN("\tcontroller type: EPOS4 - not implemented!");
                    return EPOS_ERROR;
                }
                else
                {
                	ROS_WARN("\tcontroller type: no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
            	ROS_WARN("\tmotor type: not a DC motor");
                return EPOS_ERROR;
            }
            break;
        }

        case RE13 : //!< return error if it is not a DC type of motor
        {
            if(setObjectSDO(OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                ROS_INFO("\tmotor type: RE13");

                if(controller_type_ == EPOS2)
                {
                    ROS_INFO("\tcontroller type: EPOS2");
                    setObjectSDO(OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(controller_type_ == EPOS4)
                {
                	ROS_WARN("\tcontroller type: EPOS4 - not implemented!");
                    return EPOS_ERROR;
                }
                else
                {
                	ROS_WARN("\tcontroller type: no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
            	ROS_WARN("\tmotor type: not a DC motor");
                return EPOS_ERROR;
            }
            break;
        }

        case RE30 : //!< return error if it is not a DC type of motor
        {
            if(setObjectSDO(OBJECT_MOTOR_TYPE, BRUSHED_DC_MOTOR) == EPOS_OK)
            {
                ROS_INFO("\tmotor type: RE30");

                if(controller_type_ == EPOS2)
                {
                    ROS_INFO("\tcontroller type: EPOS2");
                    setObjectSDO(OBJECT_POLE_PAIR_NUMBER, 0x01);
                    setObjectSDO(OBJECT_OUTPUT_CURRENT_LIMIT, 0x0320);
                }
                else if(controller_type_ == EPOS4)
                {
                	ROS_WARN("\tcontroller type: EPOS4 - not implemented!");
                    return EPOS_ERROR;
                }
                else
                {
                	ROS_WARN("\tcontroller type: no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
            	ROS_WARN("\tmotor type: not a DC motor");
                return EPOS_ERROR;
            }
            break;
        }

        case ECI40 : //!< return error if it is not an EC type of motor
        {
            //if((setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER1) == EPOS_OK) || (setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) || (setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER2) == EPOS_OK))
            if(setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK)
            {
                ROS_INFO("\tmotor type: ECI40");

                if(controller_type_ == EPOS2)
                {
                    ROS_INFO("\tcontroller type: EPOS2");
                    setObjectSDO(OBJECT_POLE_PAIR_NUMBER, 0x07);
                    setObjectSDO(OBJECT_OUTPUT_CURRENT_LIMIT, 0x2530);
                    setObjectSDO(OBJECT_QUICKSTOP_DECELERATION, 0x00002710);
                    setObjectSDO(OBJECT_MAXIMAL_FOLLOWING_ERROR, 0x00004E20); //0x4E20 = 20000, 0xFFFFFFFE to disactivate
                }
                else if(controller_type_ == EPOS4)
                {
                	ROS_WARN("\tcontroller type: EPOS4 - not implemented!");
                    return EPOS_ERROR;
                }
                else
                {
                	ROS_WARN("\tcontroller type: no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
            	ROS_WARN("\tmotor type: not an EC motor");
                return EPOS_ERROR;
            }
            break;
        }

        case ECI52 : //!< return error if it is not an EC type of motor
        {
            //if((setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER1) == EPOS_OK) || (setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) || (setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER2) == EPOS_OK))
            if(setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK)
            {
                ROS_INFO("\tmotor type: ECI52");

                if(controller_type_ == EPOS2)
                {
                    ROS_INFO("\tcontroller type: EPOS2");
                    setObjectSDO(OBJECT_POLE_PAIR_NUMBER, 0x07);
                    setObjectSDO(OBJECT_OUTPUT_CURRENT_LIMIT, 0x2530);
                    setObjectSDO(OBJECT_QUICKSTOP_DECELERATION, 0x00002710);
                    setObjectSDO(OBJECT_MAXIMAL_FOLLOWING_ERROR, 0x00004E20); //0x4E20 = 20000, 0xFFFFFFFE to disactivate
                }
                else if(controller_type_ == EPOS4)
                {
                	ROS_WARN("\tcontroller type: EPOS4 - not implemented!");
                    return EPOS_ERROR;
                }
                else
                {
                	ROS_WARN("\tcontroller type: no controller selected!");
                    return EPOS_ERROR;
                }
            }
            else
            {
            	ROS_WARN("\tmotor type: not an EC motor");
                return EPOS_ERROR;
            }
            break;
        }

        case EC90 : //!< return error if it is not an EC type of motor
        {
            //if((setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER1) == EPOS_OK) || (setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) || (setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL_ENCODER2) == EPOS_OK))
            //if(setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) //EC_MOTOR_HALL_ENCODER1 //EC_MOTOR_HALL
            //{
                ROS_INFO("\tmotor type: EC90");

                if(controller_type_ == EPOS2)
                {
                    if(setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) //EC_MOTOR_HALL_ENCODER1 //EC_MOTOR_HALL
                    {
                        ROS_INFO("\tcontroller type: EPOS2");
                        setObjectSDO(OBJECT_POLE_PAIR_NUMBER, 0x0C);
                        setObjectSDO(OBJECT_OUTPUT_CURRENT_LIMIT, 0x2530);
                        setObjectSDO(OBJECT_QUICKSTOP_DECELERATION, 0x00002710);
                        setObjectSDO(OBJECT_MAXIMAL_FOLLOWING_ERROR, 0x00004E20); //0x4E20 = 20000, 0xFFFFFFFE to disactivate
                    }
                    else
                    {
                    	ROS_WARN("\tmotor type: not an EC motor");
                        return EPOS_ERROR;
                    }
                }
                else if(controller_type_ == EPOS4)
                {
                    if(setObjectSDO(OBJECT_MOTOR_TYPE, EC_MOTOR_HALL) == EPOS_OK) //EC_MOTOR_HALL_ENCODER1 //EC_MOTOR_HALL
                    {
                        ROS_INFO("\tcontroller type: EPOS4");
                        setObjectSDO(OBJECT_EPOS4_POLE_PAIR_NUMBER, 0x0C);
                        setObjectSDO(OBJECT_EPOS4_OUTPUT_CURRENT_LIMIT, 0x2530);
                        setObjectSDO(OBJECT_QUICKSTOP_DECELERATION, 0x00002710);
                        setObjectSDO(OBJECT_EPOS4_FOLLOWING_ERROR_WINDOW, 0x00004E20); //0x4E20 = 20000, 0xFFFFFFFE to disactivate
                    }
                    else
                    {
                    	ROS_WARN("\tmotor type: not an EC motor");
                        return EPOS_ERROR;
                    }
                }
                else
                {
                	ROS_WARN("\tcontroller type: no controller selected!");
                    return EPOS_ERROR;
                }
            /*}
            else
            {
                ROS_INFO("...not an EC motor");
                return EPOS_ERROR;
            }*/
            break;
        }

        default :
        {
        	ROS_WARN("\tmotor type: config file value incorrect!");
            return EPOS_ERROR; //!< return ERROR if the value is not correct
            //break;
        }
    }

    ROS_INFO("\tPDO parameters");

    if(controller_type_ == EPOS2)
    {
        setObjectSDO(OBJECT_MAXIMAL_PROFILE_VELOCITY, 0x000061A8); //0x61A8 = 25000, 0x1388 = 5000
        //Miscellaneous configuration : Set bit 0 to 1 to Disable sensor supervision by software, to prevent Position Sensor Breach Error
        setObjectSDO(OBJECT_MISCELLANEOUS_CONFIGURATION, 0x00000000);
        setObjectSDO(OBJECT_MAXIMAL_FOLLOWING_ERROR, 0x00004E20);
        setObjectSDO(OBJECT_MAXIMAL_ACCELERATION, 0x00002EE0); //0x00004332 //0x00002EE0 //0x00001332 //0x00000FFF ; 0x00002FFF //normal value : 0x00001332
        setObjectSDO(OBJECT_MAXIMAL_SPEED_IN_CURRENT_MODE, 0x000061A8);  //3E8 = 1000 rpm
    }
    else if(controller_type_ == EPOS4)
    {
        setObjectSDO(OBJECT_EPOS4_MAX_PROFILE_VELOCITY, 0x0000C350); //0xC350 = 50000, 0x61A8 = 25000, 0x1388 = 5000
        //Miscellaneous configuration : Set bit 0 to 1 to Disable sensor supervision by software, to prevent Position Sensor Breach Error
        setObjectSDO(OBJECT_EPOS4_AXIS_CONFIGURATION_MISCELLANEOUS, 0x00000000);
        setObjectSDO(OBJECT_EPOS4_FOLLOWING_ERROR_WINDOW, 0x00004E20);
        setObjectSDO(OBJECT_EPOS4_MAX_ACCELERATION, 0x00002EE0); //0x00004332 //0x00002EE0 //0x00001332 //0x00000FFF ; 0x00002FFF //normal value : 0x00001332
    }

    //save all the parameters
    setObjectSDO(OBJECT_STORE_PARAMETERS, SAVE_ALL);

    setNMT(CS_RESET_COMMUNICATION);

    //setObjectSDO(OBJECT_PROFILE_VELOCITY, 0x000003E8);
    //setObjectSDO(OBJECT_PROFILE_ACCELERATION, 0x00002710);
    //setObjectSDO(OBJECT_PROFILE_DECELERATION, 0x00002710);
    //setObjectSDO(OBJECT_MOTION_PROFILE_TYPE, 0x0001); //0= trapezoidal, 1 = sinusoidal

    setNMT(CS_ENTER_PRE_OPERATIONAL);

    //set RxPDO 1
    ROS_INFO("\tRxPDO-1");
    //Disable the PDO to modify it
    setPDO(RECEIVE_PDO_1_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_1_SUBINDEX, COB_ID_RECEIVE_PDO_1_DISABLE + node_id_, 4);
    //ROS_INFO("...TEST,");
    //Writing 0 first to the number of mapped objects
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_1_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x00, 1);
    //Set object 1
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_1_INDEX, MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX, OBJECT_TARGET_POSITION, 4);  //32
    //Set object 2
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_1_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_TARGET_VELOCITY, 4);    //32
    //enable RxPDO 1 with 2 objects
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_1_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x02, 1);
    //Enable the PDO once the setup is done
    setPDO(RECEIVE_PDO_1_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_1_SUBINDEX, COB_ID_RECEIVE_PDO_1_ENABLE + node_id_, 4);

    //set RxPDO 2
    ROS_INFO("\tRxPDO-2");
    //Disable the PDO to modify it
    setPDO(RECEIVE_PDO_2_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_2_SUBINDEX, COB_ID_RECEIVE_PDO_2_DISABLE + node_id_, 4);
    //Writing 0 first to the number of mapped objects
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_2_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x00, 1);
    //Set object 1
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_2_INDEX, MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX, OBJECT_PROFILE_ACCELERATION, 4);  //32
    //Set object 2
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_2_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_PROFILE_DECELERATION, 4);    //32
    //enable RxPDO 1 with 2 objects
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_2_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x02, 1);
    //Enable the PDO once the setup is done
    setPDO(RECEIVE_PDO_2_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_2_SUBINDEX, COB_ID_RECEIVE_PDO_2_ENABLE + node_id_, 4);

    //set RxPDO 3
    ROS_INFO("\tRxPDO-3");
    //Disable the PDO to modify it
    setPDO(RECEIVE_PDO_3_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_3_SUBINDEX, COB_ID_RECEIVE_PDO_3_DISABLE + node_id_, 4);
    //Writing 0 first to the number of mapped objects
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x00, 1);
    //Set object 1
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX, OBJECT_CONTROLWORD, 4); //16
    //Set object 2
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_PROFILE_VELOCITY, 4); //32
    //Set object 3
    if(controller_type_ == EPOS2)
    {
        setPDO(MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, MAPPED_OBJECT_3_RECEIVE_PDO_SUBINDEX, OBJECT_OUTPUT_CURRENT_LIMIT, 4); //16
        //enable RxPDO 1 with 3 objects
        setPDO(MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x03, 1);
    }
    else if(controller_type_ == EPOS4)
    {
        //setPDO(MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_EPOS4_OUTPUT_CURRENT_LIMIT, 4); //32 impossible too long
        //enable RxPDO 1 with 2 objects
        setPDO(MAPPED_OBJECT_RECEIVE_PDO_3_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x02, 1);
    }
    //Enable the PDO once the setup is done
    setPDO(RECEIVE_PDO_3_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_3_SUBINDEX, COB_ID_RECEIVE_PDO_3_ENABLE + node_id_, 4);

    //set RxPDO 4
    ROS_INFO("\tRxPDO-4");
    //Disable the PDO to modify it
    setPDO(RECEIVE_PDO_4_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_4_SUBINDEX, COB_ID_RECEIVE_PDO_4_DISABLE + node_id_, 4);
    //Writing 0 first to the number of mapped objects
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x00, 1);
    //Set object 1
    if(controller_type_ == EPOS2) setPDO(MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX, OBJECT_CURRENT_MODE_SETTING_VALUE, 4);  //16 //Current Mode
    else if(controller_type_ == EPOS4) setPDO(MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, MAPPED_OBJECT_1_RECEIVE_PDO_SUBINDEX, OBJECT_EPOS4_TARGET_TORQUE, 4); //16 //Cyclic Synchronous Torque Mode
    //Set object 2
    if(controller_type_ == EPOS2) setPDO(MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_MAXIMAL_SPEED_IN_CURRENT_MODE, 4);  //32
    else if(controller_type_ == EPOS4) setPDO(MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, MAPPED_OBJECT_2_RECEIVE_PDO_SUBINDEX, OBJECT_EPOS4_MAX_MOTOR_SPEED, 4);  //32
    //Set object 3
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, MAPPED_OBJECT_3_RECEIVE_PDO_SUBINDEX, OBJECT_MODES_OF_OPERATION, 4); //8
    //enable RxPDO 1 with 3 objects
    setPDO(MAPPED_OBJECT_RECEIVE_PDO_4_INDEX, NUMBER_OBJECTS_RECEIVE_PDO_SUBINDEX, 0x03, 1);
    //Enable the PDO once the setup is done
    setPDO(RECEIVE_PDO_4_PARAMETER_INDEX, COB_ID_RECEIVE_PDO_4_SUBINDEX, COB_ID_RECEIVE_PDO_4_ENABLE + node_id_, 4);

    //set TxPDO 1
    ROS_INFO("\tTxPDO-1");
    //Writing 0 first to the number of mapped objects
    setPDO(MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x00, 1);
    //Set object 1
    setPDO(MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX, MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX, OBJECT_POSITION_ACTUAL_VALUE, 4);    //32
    //Set object 2
    if(controller_type_ == EPOS2) setPDO(MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX, MAPPED_OBJECT_2_TRANSMIT_PDO_SUBINDEX, OBJECT_VELOCITY_ACTUAL_VALUE_AVERAGED, 4);  //32
    else if(controller_type_ == EPOS4) setPDO(MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX, MAPPED_OBJECT_2_TRANSMIT_PDO_SUBINDEX, OBJECT_EPOS4_VELOCITY_ACTUAL_VALUE_AVERAGED, 4); //32
    //enable TxPDO with 2 objects
    setPDO(MAPPED_OBJECT_TRANSMIT_PDO_1_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x02, 1);

    //set TxPDO 2
    ROS_INFO("\tTxPDO-2");
    setPDO(MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x00, 1);

    if(controller_type_ == EPOS2)
    {
        //Set object 1
        setPDO(MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX, OBJECT_CURRENT_ACTUAL_VALUE_AVERAGED, 4);   //16
        //Set object 2
        setPDO(MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, MAPPED_OBJECT_2_TRANSMIT_PDO_SUBINDEX, OBJECT_FOLLOWING_ERROR_ACTUAL_VALUE, 4);   //16
        //Set object 3
        setPDO(MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, MAPPED_OBJECT_3_TRANSMIT_PDO_SUBINDEX, OBJECT_STATUSWORD, 4);   //16
        //enable TxPDO with 3 objects
        setPDO(MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x03, 1);
    }
    else if(controller_type_ == EPOS4)
    {
        //TODO check the corresponding functions
        //Set object 1
        setPDO(MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX, OBJECT_EPOS4_CURRENT_ACTUAL_VALUE_AVERAGED, 4);   //32
        //Set object 2
        setPDO(MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, MAPPED_OBJECT_2_TRANSMIT_PDO_SUBINDEX, OBJECT_STATUSWORD, 4);   //16
        //enable TxPDO with 3 objects
        setPDO(MAPPED_OBJECT_TRANSMIT_PDO_2_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x02, 1);
    }

    //set TxPDO 3
    ROS_INFO("\tTxPDO-3");
    //Writing 0 first to the number of mapped objects
    setPDO(MAPPED_OBJECT_TRANSMIT_PDO_3_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x00, 1);
    //Set object 1
    setPDO(MAPPED_OBJECT_TRANSMIT_PDO_3_INDEX, MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX, OBJECT_MODES_OF_OPERATION_DISPLAY, 4);  //8
    //enable TxPDO with 1 objects
    setPDO(MAPPED_OBJECT_TRANSMIT_PDO_3_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x01, 1);

    //set TxPDO 4
    ROS_INFO("\tTxPDO-4");
    //Writing 0 first to the number of mapped objects
    setPDO(MAPPED_OBJECT_TRANSMIT_PDO_4_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x00, 1);
    //Set object 1
    setPDO(MAPPED_OBJECT_TRANSMIT_PDO_4_INDEX, MAPPED_OBJECT_1_TRANSMIT_PDO_SUBINDEX, OBJECT_INCREMENTAL_ENCODER_1_COUNTER_AT_INDEX_PULSE, 4); //32
    //enable TxPDO with 1 objects
    setPDO(MAPPED_OBJECT_TRANSMIT_PDO_4_INDEX, NUMBER_OBJECTS_TRANSMIT_PDO_SUBINDEX, 0x01, 1);


    //set TxPDO 1 to RTR mode (request only)
    setPDO(TRANSMIT_PDO_1_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_1_SUBINDEX, COB_ID_TRANSMIT_PDO_1_DISABLE + node_id_, 4);
    setPDO(TRANSMIT_PDO_1_PARAMETER_INDEX, TRANSMISSION_TYPE_TRANSMIT_PDO_1_SUBINDEX, PARAM_TRANSMISSION_TYPE_RTR, 1);
    setPDO(TRANSMIT_PDO_1_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_1_SUBINDEX, COB_ID_TRANSMIT_PDO_1_ENABLE + node_id_, 4);

    //set TxPDO 2 to RTR mode (request only)
    setPDO(TRANSMIT_PDO_2_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_2_SUBINDEX, COB_ID_TRANSMIT_PDO_2_DISABLE + node_id_, 4);
    setPDO(TRANSMIT_PDO_2_PARAMETER_INDEX, TRANSMISSION_TYPE_TRANSMIT_PDO_2_SUBINDEX, PARAM_TRANSMISSION_TYPE_RTR, 1);
    setPDO(TRANSMIT_PDO_2_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_2_SUBINDEX, COB_ID_TRANSMIT_PDO_2_ENABLE + node_id_, 4);

    //set TxPDO 3 to RTR mode (request only)
    setPDO(TRANSMIT_PDO_3_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_3_SUBINDEX, COB_ID_TRANSMIT_PDO_3_DISABLE + node_id_, 4);
    setPDO(TRANSMIT_PDO_3_PARAMETER_INDEX, TRANSMISSION_TYPE_TRANSMIT_PDO_3_SUBINDEX, PARAM_TRANSMISSION_TYPE_RTR, 1);
    setPDO(TRANSMIT_PDO_3_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_3_SUBINDEX, COB_ID_TRANSMIT_PDO_3_ENABLE + node_id_, 4);

    //set TxPDO 4 to RTR mode (request only)
    setPDO(TRANSMIT_PDO_4_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_4_SUBINDEX, COB_ID_TRANSMIT_PDO_4_DISABLE + node_id_, 4);
    setPDO(TRANSMIT_PDO_4_PARAMETER_INDEX, TRANSMISSION_TYPE_TRANSMIT_PDO_4_SUBINDEX, PARAM_TRANSMISSION_TYPE_RTR, 1);
    setPDO(TRANSMIT_PDO_4_PARAMETER_INDEX, COB_ID_TRANSMIT_PDO_4_SUBINDEX, COB_ID_TRANSMIT_PDO_4_ENABLE + node_id_, 4);

    setNMT(CS_START_REMOTE_NODE);

    ROS_INFO("\tMode of operation:");
    //this also initialise activMode variable
    switch(mode_)
    {
        case INTERPOLATED_POSITION_MODE :
	{
            setModeOfOperationSDO(VALUE_INTERPOLATED_POSITION_MODE);
            break;
        }

        case PROFILE_VELOCITY_MODE :
	{
            setModeOfOperationSDO(VALUE_PROFILE_VELOCITY_MODE);
            break;
        }

        case PROFILE_POSITION_MODE :
	{
            setModeOfOperationSDO(VALUE_PROFILE_POSITION_MODE);
            break;
        }

        case POSITION_MODE :
	{
            setModeOfOperationSDO(VALUE_POSITION_MODE);
            break;
        }

        case VELOCITY_MODE :
	{
            setModeOfOperationSDO(VALUE_VELOCITY_MODE);
            break;
        }

        case CURRENT_MODE :
	{
            setModeOfOperationSDO(VALUE_CURRENT_MODE);
            break;
        }

        case CYCLIC_SYNCHRONOUS_TORQUE_MODE :
	{
            setModeOfOperationSDO(VALUE_CYCLIC_SYNCHRONOUS_TORQUE_MODE);
            break;
        }

        default :
	{
            return EPOS_ERROR;
        }
    }

    shutdownControlword();
    ROS_INFO("\tON");
    switchOnEnableOperationControlword();

    ROS_INFO("\tEnable operation: OK");

    return EPOS_OK;
}

/*! \fn void setTargetPosition(int32_t position)
 *  \brief send a CAN frame using the Receive PDO 1, first object Target Position, the controller has to be in Profile Position Mode.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param position Target Position (in steps unit).
 *  \return void
 */
void EPOSController::setTargetPosition(int32_t position)
{
    //!> store the latest command.
    target_position_ = position;

    //!< change sign if need be.
    if(inverted_ == true) position = -1*position;

    //!> fill the data in each byte.
    data_[0] = (position & 0x000000FF); //LSB
    data_[1] = (position & 0x0000FF00) >> 8;
    data_[2] = (position & 0x00FF0000) >> 16;
    data_[3] = (position & 0xFF000000) >> 24;
    data_[4] = 0x00; //fill with 0 the data which correspond to the others objects defined on the Receive PDO.
    data_[5] = 0x00;
    data_[6] = 0x00;
    data_[7] = 0x00;

    //!> send the CAN frame.
    canToEposWrite(COB_ID_RECEIVE_PDO_1_ENABLE + node_id_, data_, 8); //write 8 bytes

    if(controller_type_ == EPOS2) ros::Duration(0.0001).sleep();
    else if(controller_type_ == EPOS4) ros::Duration(0.001).sleep(); //TODO reduce pause
}

/*! \fn void setTargetVelocity(int32_t velocity)
 *  \brief send a CAN frame using the Receive PDO 1, second object Target Velocity, the controller has to be in Profile Velocity Mode.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param velocity Target Velocity (in rpm unit).
 *  \return void
 */
void EPOSController::setTargetVelocity(int32_t velocity)
{
    //!> store the latest command.
    target_velocity_ = velocity;

    //!< change sign if need be.
    if(inverted_ == true) velocity = -1*velocity;

    //!> fill the data in each byte.
    data_[0] = 0x00; //LSB //fill with 0 the data which correspond to the others objects defined on the Receive PDO.
    data_[1] = 0x00;
    data_[2] = 0x00;
    data_[3] = 0x00;
    data_[4] = (velocity & 0x000000FF);
    data_[5] = (velocity & 0x0000FF00) >> 8;
    data_[6] = (velocity & 0x00FF0000) >> 16;
    data_[7] = (velocity & 0xFF000000) >> 24;

    //!> send the CAN frame.
    canToEposWrite(COB_ID_RECEIVE_PDO_1_ENABLE + node_id_, data_, 8); //write 8 bytes

    if(controller_type_ == EPOS2) ros::Duration(0.0001).sleep();
    else if(controller_type_ == EPOS4) ros::Duration(0.001).sleep();
}

/*! \fn void setProfileAccelerationDeceleration(uint32_t acceleration, uint32_t deceleration)
 *  \brief send a CAN frame using the Receive PDO 2, two objects: Profile Acceleration and Profile Deceleration.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param acceleration Profile Acceleration (in rpm/s unit) in a profile move.
 *  \param deceleration Profile Deceleration (in rpm/s unit) in a profile move.
 *  \return void
 */
void EPOSController::setProfileAccelerationDeceleration(uint32_t acceleration, uint32_t deceleration)
{
    //!> store the latest command.
    profile_acceleration_ = acceleration;
    profile_deceleration_ = deceleration;

    //!> fill the data in each byte.
    data_[0] = (acceleration & 0x000000FF); //LSB
    data_[1] = (acceleration & 0x0000FF00) >> 8;
    data_[2] = (acceleration & 0x00FF0000) >> 16;
    data_[3] = (acceleration & 0xFF000000) >> 24;
    data_[4] = (deceleration & 0x000000FF);
    data_[5] = (deceleration & 0x0000FF00) >> 8;
    data_[6] = (deceleration & 0x00FF0000) >> 16;
    data_[7] = (deceleration & 0xFF000000) >> 24;

    //!> send the CAN frame.
    canToEposWrite(COB_ID_RECEIVE_PDO_2_ENABLE + node_id_, data_, 8); //write 8 bytes

    if(controller_type_ == EPOS2) ros::Duration(0.0001).sleep();
    else if(controller_type_ == EPOS4) ros::Duration(0.001).sleep();
}

/*! \fn void setProfileAcceleration(uint32_t acceleration)
 *  \brief send a CAN frame using the Receive PDO 2, two objects: user defined Profile Acceleration and stored Profile Deceleration.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param acceleration Profile Acceleration (in rpm/s unit) in a profile move.
 *  \return void
 */
void EPOSController::setProfileAcceleration(uint32_t acceleration)
{
    setProfileAccelerationDeceleration(acceleration, profile_deceleration_);
}

/*! \fn void setProfileDeceleration(uint32_t deceleration)
 *  \brief send a CAN frame using the Receive PDO 2, two objects: stored Profile Acceleration and user defined Profile Deceleration.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param deceleration Profile Deceleration (in rpm/s unit) in a profile move.
 *  \return void
 */
void EPOSController::setProfileDeceleration(uint32_t deceleration)
{
    setProfileAccelerationDeceleration(profile_acceleration_, deceleration);
}

/*! \fn void setCrtlWordProVelOutCurrLmt(uint16_t ctrlWord, uint32_t velocity, uint16_t outCurrLmt)
 *  \brief send a CAN frame using the Receive PDO 3, three objects: Controlword, Profile Velocity, Output Current Limit (EPOS2 only).
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param ctrlWord Controlword.
 *  \param velocity Profile Velocity (in rpm unit).
 *  \param outCurrLmt Output Current Limit (in mA unit), only used with EPOS2.
 *  \return void
 */
void EPOSController::setCrtlWordProVelOutCurrLmt(uint16_t ctrlWord, uint32_t velocity, uint16_t outCurrLmt)
{
    //!> store the latest command.
    controlword_ = ctrlWord;
    profile_velocity_ = velocity;

    if(controller_type_ == EPOS2)
    {
        output_current_limit_ = outCurrLmt;

        //!> fill the data in each byte.
        data_[0] = (ctrlWord & 0x000000FF);
        data_[1] = (ctrlWord & 0x0000FF00) >> 8;
        data_[2] = (velocity & 0x000000FF); //LSB
        data_[3] = (velocity & 0x0000FF00) >> 8;
        data_[4] = (velocity & 0x00FF0000) >> 16;
        data_[5] = (velocity & 0xFF000000) >> 24;
        data_[6] = (outCurrLmt & 0x000000FF);
        data_[7] = (outCurrLmt & 0x0000FF00) >> 8;

        //!> send the CAN frame.
        canToEposWrite(COB_ID_RECEIVE_PDO_3_ENABLE + node_id_, data_, 8); //write 8 bytes
        ros::Duration(0.0001).sleep();
    } //TODO to finish
    else if(controller_type_ == EPOS4)
    {
        //!> fill the data in each byte.
        data_[0] = (ctrlWord & 0x000000FF);
        data_[1] = (ctrlWord & 0x0000FF00) >> 8;
        data_[2] = (velocity & 0x000000FF); //LSB
        data_[3] = (velocity & 0x0000FF00) >> 8;
        data_[4] = (velocity & 0x00FF0000) >> 16;
        data_[5] = (velocity & 0xFF000000) >> 24;
        data_[6] = 0x00;
        data_[7] = 0x00;

        //!> send the CAN frame.
        canToEposWrite(COB_ID_RECEIVE_PDO_3_ENABLE + node_id_, data_, 8); //write 8 bytes
        ros::Duration(0.001).sleep(); //TODO timing to check
    }
}

/*! \fn void setProfileVelocity(uint32_t velocity)
 *  \brief send a CAN frame using the Receive PDO 3, three objects: user defined Profile Velocity, stored Output Current Limit and stored Controlword.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param velocity Profile Velocity (in rpm unit).
 *  \return void
 */
void EPOSController::setProfileVelocity(uint32_t velocity)
{
    setCrtlWordProVelOutCurrLmt(controlword_, velocity, output_current_limit_);
}

/*! \fn void setOutputCurrentLimit(uint16_t current)
 *  \brief send a CAN frame using the Receive PDO 3, three objects: stored Profile Velocity, user defined Output Current Limit and stored Controlword.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param current Output Current Limit (in mA unit).
 *  \return void
 */
void EPOSController::setOutputCurrentLimit(uint16_t current)
{
    if(controller_type_ == EPOS2) setCrtlWordProVelOutCurrLmt(controlword_, profile_velocity_, current);
}

/*! \fn void setControlword(uint16_t ctrlWord)
 *  \brief send a CAN frame using the Receive PDO 3, three objects: stored Profile Velocity, stored Output Current Limit and user defined Controlword.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param ctrlWord Controlword.
 *  \return void
 */
void EPOSController::setControlword(uint16_t ctrlWord)
{
    setCrtlWordProVelOutCurrLmt(ctrlWord, profile_velocity_, output_current_limit_);

   /*
    //!> store the latest command.
    //profile_velocity_ = velocity;
    //output_current_limit_ = outCurrLmt;
    controlword_ = ctrlWord;

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
    canToEposWrite(COB_ID_RECEIVE_PDO_3_ENABLE + node_id_, data, 8)); //write 2 bytes
    ros::Duration(0.0001).sleep();*/
}

/*! \fn void setCurrentSpeedMode(int16_t current, uint32_t maxSpeedCurr, uint16_t controlword, int8_t mode)
 *  \brief send a CAN frame using the Receive PDO 4, three objects: Current Mode Setting Value, Maximal Speed In Current Mode and Modes Of Operation.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \param current Current Mode Setting Value (in mA unit).
 *  \param maxSpeedCurr Maximal Speed In Current Mode (in rpm unit).
 *  \param mode Modes Of Operation.
 *  \return void
 */
void EPOSController::setCurrentSpeedMode(int16_t current, uint32_t maxSpeedCurr, int8_t mode)
{
    //!> store the latest command.
    current_mode_setting_value_ = current; //TODO do a special case for torque mode
    maximal_speed_in_current_mode_ = maxSpeedCurr;
    modes_of_operation_ = mode;

    //!< change sign if need be.
    if(inverted_ == true) current = -1*current;

    //!> fill the data in each byte.
    data_[0] = (current & 0x000000FF); //LSB
    data_[1] = (current & 0x0000FF00) >> 8;
    data_[2] = (maxSpeedCurr & 0x000000FF);
    data_[3] = (maxSpeedCurr & 0x0000FF00) >> 8;
    data_[4] = (maxSpeedCurr & 0x00FF0000) >> 16;
    data_[5] = (maxSpeedCurr & 0xFF000000) >> 24;
    data_[6] = (mode & 0x000000FF);
    data_[7] = 0x00;

    //!> send the CAN frame.
    canToEposWrite(COB_ID_RECEIVE_PDO_4_ENABLE + node_id_, data_, 8); //write 8 bytes

    if(controller_type_ == EPOS2) ros::Duration(0.0001).sleep();
    else if(controller_type_ == EPOS4) ros::Duration(0.001).sleep();
}

/*! \fn void setCurrentModeSettingValue(int16_t current)
 *  \brief send a CAN frame using the Receive PDO 4, three objects: user defined Current Mode Setting Value, stored Maximal Speed In Current Mode and stored Modes Of Operation.
 *  \param node_id_ identifier of the node on the CAN bus
 *  \param current Current Mode Setting Value (in mA unit).
 *  \return void
 */
void EPOSController::setCurrentModeSettingValue(int16_t current)
{
    setCurrentSpeedMode(current, maximal_speed_in_current_mode_, modes_of_operation_);
}

/*! \fn void setMaximalSpeedInCurrentMode(uint32_t maxSpeedCurr)
 *  \brief send a CAN frame using the Receive PDO 4, three objects: stored Current Mode Setting Value, user defined Maximal Speed In Current Mode and stored Modes Of Operation.
 *  \param node_id_ identifier of the node on the CAN bus
 *  \param maxSpeedCurr Maximal Speed In Current Mode (in rpm unit).
 *  \return void
 */
void EPOSController::setMaximalSpeedInCurrentMode(uint32_t maxSpeedCurr)
{
    setCurrentSpeedMode(current_mode_setting_value_, maxSpeedCurr, modes_of_operation_);
}

/*! \fn void setModesOfOperation(int16_t current, uint32_t maxSpeedCurr, uint16_t controlword, int8_t mode)
 *  \brief send a CAN frame using the Receive PDO 4, three objects: stored Current Mode Setting Value, stored Maximal Speed In Current Mode and user defined Modes Of Operation.
 *  \param node_id_ identifier of the node on the CAN bus
 *  \param mode Modes Of Operation.
 *  \return void
 */
void EPOSController::setModesOfOperation(int8_t mode)
{
    //set activMode
    switch(mode)
    {
        case 7 : //VALUE_INTERPOLATED_POSITION_MODE :
	{
            activ_mode_ = INTERPOLATED_POSITION_MODE;
            break;
	}

        case 3 : //VALUE_PROFILE_VELOCITY_MODE :
	{
            activ_mode_ = PROFILE_VELOCITY_MODE;
            break;
	}

        case 1 : //VALUE_PROFILE_POSITION_MODE :
	{
            activ_mode_ = PROFILE_POSITION_MODE;
            break;
	}

        case -1 : //VALUE_POSITION_MODE :
	{
            activ_mode_ = POSITION_MODE;
            break;
	}

        case -2 : //VALUE_VELOCITY_MODE :
	{
            activ_mode_ = VELOCITY_MODE;
            break;
	}

        case -3 : //VALUE_CURRENT_MODE :
	{
            activ_mode_ = CURRENT_MODE;
            break;
	}

        case 10 : //VALUE_CYCLIC_SYNCHRONOUS_TORQUE_MODE :
	{
            activ_mode_ = CYCLIC_SYNCHRONOUS_TORQUE_MODE;
            break;
	}

        default :
	{
            ROS_INFO("Wrong mode value");
            return;
	}
    }

    setCurrentSpeedMode(current_mode_setting_value_, maximal_speed_in_current_mode_, mode);
}

/*! \fn void getPositionVelocity()
 *  \brief send a CAN frame using the Transmit PDO 1, two objects Position Actual Value and Velocity Actual Value Averaged, the answer frame is caught by interrupt.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \return void
 */
void EPOSController::getPositionVelocity()
{
	//ROS_INFO("node_id_=%d", node_id_);
	transmitPDOWrite(COB_ID_TRANSMIT_PDO_1_ENABLE);
}

/*! \fn void getCurrentFollErrStatusword()
 *  \brief send a CAN frame using the Transmit PDO 2, two objects Current Actual Value Averaged, Following Error Actual Value and Statusword, the answer frame is caught by interrupt.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \return void
 */
void EPOSController::getCurrentFollErrStatusword()
{
    transmitPDOWrite(COB_ID_TRANSMIT_PDO_2_ENABLE);
}

/*! \fn void getModesOfOperation()
 *  \brief send a CAN frame using the Transmit PDO 3, one object Modes Of Operation Display, the answer frame is caught by interrupt.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \return void
 */
void EPOSController::getModesOfOperation()
{
    transmitPDOWrite(COB_ID_TRANSMIT_PDO_3_ENABLE);
}

/*! \fn void getIncEnc1CntAtIdxPls()
 *  \brief send a CAN frame using the Transmit PDO 4, one object Incremental Encoder 1 counter at Index Pulse, the answer frame is caught by interrupt.
 *  \param node_id_ identifier of the node on the CAN bus.
 *  \return void
 */
void EPOSController::getIncEnc1CntAtIdxPls()
{
    transmitPDOWrite(COB_ID_TRANSMIT_PDO_4_ENABLE);
}

/*! \fn void calibrate()
 *  \brief calibration routine.
 *  \return int EPOS_OK or EPOS_ERROR
 */
int EPOSController::calibrate()
{
    uint16_t outCurLmt = 0;
    uint32_t profVel = 0;
    uint32_t profAcc = 0;
    uint32_t profDec = 0;
    uint32_t maxSpeed = 0;

    ROS_INFO("- Calibration for nodeID %d", node_id_);

	//set default parameters
	switch(motor_type_)
	{
		case NONE :
		{
			return EPOS_ERROR;
		}

		case DCX10 :
		{
			ROS_INFO("\tDCX10");

			if(controller_type_ == EPOS2)
			{
				ROS_INFO("\tEPOS2");

				outCurLmt = 2000;
				profVel = 2000;
				profAcc = 2000;
				profDec = 2000;
				maxSpeed = 10000;
			}
			else if(controller_type_ == EPOS4)
			{
				return EPOS_ERROR;
			}
			break;
		}

		case DCX14 :
		{
			ROS_INFO("\tDCX14");

			if(controller_type_ == EPOS2)
			{
				ROS_INFO("\tEPOS2");

				outCurLmt = 2000;
				profVel = 2000;
				profAcc = 2000;
				profDec = 2000;
				maxSpeed = 10000;
			}
			else if(controller_type_ == EPOS4)
			{
				return EPOS_ERROR;
			}
			break;
		}

		case DCX16 : //!< return error if it is not a DC type of motor
		{
			ROS_INFO("\tDCX16");

			if(controller_type_ == EPOS2)
			{
				ROS_INFO("\tEPOS2");

				outCurLmt = 1000;
				profVel = 800;
				profAcc = 1000;
				profDec = 1000;
				maxSpeed = 1000;
			}
			else if(controller_type_ == EPOS4)
			{
				return EPOS_ERROR;
			}
			break;
		}

		case DCX22 :
		{
			ROS_INFO("\tDCX22");

			if(controller_type_ == EPOS2)
			{
				ROS_INFO("\tEPOS2");

				outCurLmt = 2000;
				profVel = 2000;
				profAcc = 10000;
				profDec = 10000;
				maxSpeed = 10000;
			}
			else if(controller_type_ == EPOS4)
			{
				return EPOS_ERROR;
			}
			break;
		}

		case DCX32 :
		{
			ROS_INFO("\tDCX32");

			if(controller_type_ == EPOS2)
			{
				ROS_INFO("\tEPOS2");

				outCurLmt = 1000;
				profVel = 800;
				profAcc = 1000;
				profDec = 1000;
				maxSpeed = 1000;
			}
			else if(controller_type_ == EPOS4)
			{
				ROS_INFO("\tEPOS4");

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
			ROS_INFO("\tRE13");

			if(controller_type_ == EPOS2)
			{
				ROS_INFO("\tEPOS2");

				outCurLmt = 2000;
				profVel = 800;
				profAcc = 1000;
				profDec = 1000;
				maxSpeed = 10000;
			}
			else if(controller_type_ == EPOS4)
			{
				return EPOS_ERROR;
			}
			break;
		}

		case RE30 :
		{
			ROS_INFO("\tRE30");

			if(controller_type_ == EPOS2)
			{
				ROS_INFO("\tEPOS2");

				outCurLmt = 1000;
				profVel = 800;
				profAcc = 1000;
				profDec = 1000;
				maxSpeed = 1000;
			}
			else if(controller_type_ == EPOS4)
			{
				ROS_INFO("\tEPOS4");

				profVel = 800;
				profAcc = 1000;
				profDec = 1000;
				maxSpeed = 1000;
			}
			break;
		}

		case ECI40 :
		{
			ROS_INFO("\tECI40");

			if(controller_type_ == EPOS2)
			{
				ROS_INFO("\tEPOS2");

				outCurLmt = 1000;
				profVel = 800;
				profAcc = 1000;
				profDec = 1000;
				maxSpeed = 1000;
			}
			else if(controller_type_ == EPOS4)
			{
				ROS_INFO("\tEPOS4");

				profVel = 800;
				profAcc = 1000;
				profDec = 1000;
				maxSpeed = 1000;
			}
			break;
		}

		case ECI52 :
		{
			ROS_INFO("\tECI52");
			if(controller_type_ == EPOS2)
			{
				ROS_INFO("\tEPOS2");

				outCurLmt = 1000;
				profVel = 800;
				profAcc = 1000;
				profDec = 1000;
				maxSpeed = 1000;
			}
			else if(controller_type_ == EPOS4)
			{
				ROS_INFO("\tEPOS4");

				profVel = 800;
				profAcc = 1000;
				profDec = 1000;
				maxSpeed = 1000;
			}
			break;
		}

		case EC90 :
		{
			ROS_INFO("\tEC90");
			if(controller_type_ == EPOS2)
			{
				ROS_INFO("\tEPOS2");

				outCurLmt = 1000;
				profVel = 1500;
				profAcc = 10000;
				profDec = 10000;
				maxSpeed = 5000;
			}
			else if(controller_type_ == EPOS4)
			{
				ROS_INFO("\tEPOS4");

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
		}
	}

	//Apply values
	//RPDO3
	setControlword(0x0006); //Shutdown
	ros::Duration(0.01).sleep();
	setProfileVelocity(profVel); //NEW try this first
	ros::Duration(0.01).sleep();
	if(controller_type_ == EPOS2) setOutputCurrentLimit(outCurLmt); //10100 //10000 for EPOS2 70/10, see documentation
	ros::Duration(0.01).sleep();

	//RPDO2
	setProfileAcceleration(profAcc);
	ros::Duration(0.01).sleep();
	setProfileDeceleration(profDec);
	ros::Duration(0.01).sleep();

	//RPDO4
	setMaximalSpeedInCurrentMode(maxSpeed); //TODO change function name for fitting with EPOS4 too
	ros::Duration(0.01).sleep();

	//Common to all motor type
	setControlword(0x0006); //Shutdown
	ros::Duration(0.01).sleep();
	setControlword(0x000F); //SwitchOn
	ros::Duration(0.01).sleep();

	//Apply a default value based on the selected mode
	switch(mode_)
	{
		case INTERPOLATED_POSITION_MODE :
		{
			return EPOS_ERROR;
			//break;
		}

		case PROFILE_VELOCITY_MODE :
		{
			setTargetVelocity(value_);
			ros::Duration(0.1).sleep();
			setControlword(0x000F); //Start to move
			ROS_INFO("\tPROFILE_VELOCITY_MODE val[%d] setControlword[0x000F]", value_);
			break;
		}

		case PROFILE_POSITION_MODE :
		{
			setTargetPosition(value_);
			ros::Duration(0.1).sleep();
			setControlword(0x002F); //Start Positioning (absolute position and start immediately)
			ros::Duration(0.1).sleep();
			setControlword(0x003F);
			ROS_INFO("\tPROFILE_POSITION_MODE val[%d] setControlword[rising edge 0x002F to 0x003F]", value_);
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
			setCurrentModeSettingValue(value_);
			ros::Duration(0.1).sleep();
			ROS_INFO("\tCURRENT_MODE val[%d]", value_);
			break;
		}

		case CYCLIC_SYNCHRONOUS_TORQUE_MODE :
		{
			return EPOS_ERROR; //TODO
		}

		default :
		{
			return EPOS_ERROR;
		}
	}

   // ledchain[3] = 1;
	ros::Duration(0.01).sleep();

	//TPDO : get data once to fill in the arrays
	getPositionVelocity();
	ros::Duration(0.01).sleep();
	getCurrentFollErrStatusword();
	ros::Duration(0.01).sleep();

	//print results to check things are working properly
	ROS_INFO("\tpos=%d vel=%d cur=%d stwrd=%d", position_, velocity_, current_, statusword_);

    return EPOS_OK;
}

/*! \fn void getData()
 *  \brief
 *  \return void
 */
void EPOSController::getData()
{
	//ROS_INFO("getData");
	//for(int i=1; i<=numberEposBoards; i++)
	//for(int i=1; i<=2; i++)
	//{
		//uint8_t node_id = i;

		getPositionVelocity();
		//ros::Duration(0.0002).sleep();
		ros::Duration(0.002).sleep();
		//ros::Duration(0.002).sleep();
		getCurrentFollErrStatusword();
		//ros::Duration(0.0002).sleep();
		ros::Duration(0.002).sleep();
		//ros::Duration(0.002).sleep();
	//}
}
