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
 * @file master_board.cpp
 * @author Cyril Jourdan
 * @date Mar 03, 2017
 * @version 0.1.0
 * @brief Implementation file for the serial communication between ROS and the Master Board
 * Deprecated.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 28, 2013
 */

/*! Includes */
#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include "osa_msgs/MotorCmdMultiArray.h"
#include "osa_msgs/MotorDataMultiArray.h"
#include <sstream>
#include <string>

/*! Defines */
#define LOOP_RATE		15 //HEART_BEAT
#define TIMEOUT 		66//35//25 //20 // 1/(2*HEART_BEAT)

#define NUMBER_MAX_EPOS_PER_SLAVE 		16
#define NUMBER_SLAVE_BOARDS         	6 //maximum 128
#define NUMBER_OF_MOTORS				NUMBER_MAX_EPOS_PER_SLAVE*NUMBER_SLAVE_BOARDS //96

#define BYTES_PER_MSG			5 //6 //1 nodeID 1 mode 4 data
#define BYTES_PER_DATA			8
#define REPLY_SIZE 				NUMBER_OF_MOTORS*8+1 //+1 for the checksum
#define RS232_BAUDRATE			460800

/*! Variables */
char data[NUMBER_SLAVE_BOARDS][NUMBER_MAX_EPOS_PER_SLAVE][BYTES_PER_MSG];
bool msg_received = false;

/*! Functions */

/*! \fn char calculateChecksum(char* dataArray, int nbBytes)
 *  \brief
 *  \param dataArray
 *  \param nbBytes
 *  \return void
 */
char calculateChecksum(char* dataArray, int nbBytes)
{
	char checksum = 0x00;
	int sum = 0;

	for(int i=0; i<nbBytes; i++)
	{
		sum += dataArray[i];
		//ROS_INFO("0x%02X", dataArray[i]);
	}
	//ROS_INFO("sum 0x%02X", sum);

	checksum = (char)(~sum);
	//ROS_INFO("checksum 0x%02X", checksum);

	return checksum;
}

/*! \fn char verifyChecksum(char* data, int nbBytes)
 *  \brief
 *  \param data
 *  \param nbBytes
 *  \return void
 */
char verifyChecksum(char* data, int nbBytes)
{
	char checksum = 0x00;

	for(int i=0; i<nbBytes; i++)
	{
		checksum += data[i];
	}
	//ROS_INFO("sum 0x%02X", sum);

	checksum++;
	//checksum = (uint8_t)(checksum);
	//ROS_INFO("checksum 0x%02X", checksum);

	if(checksum == 0x00) return true;
	else
	{
		ROS_INFO("checksum 0x%02X", checksum);
		return false;
	}
}

/*! \fn void sendMotorCmdArrayCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& motorCmd_ma)
 *  \brief callback function that received EPOS commands and that fits it into the right format to be sent over RS232 bus.
 *  \param motorCmd_ma motor command multi-array.
 *  \return void
 */
void sendMotorCmdArrayCallback(const osa_msgs::MotorCmdMultiArrayConstPtr& motorCmd_ma)
{
	if((motorCmd_ma->layout.dim[0].size == NUMBER_SLAVE_BOARDS) && (motorCmd_ma->layout.dim[1].size == NUMBER_MAX_EPOS_PER_SLAVE))
	{
		//toggle flag, a message has been received
		msg_received = true;

		#ifdef TRS_DEBUG
		ROS_INFO("msg_received");
		#endif

		//ROS_INFO("cmd[%d] val[%d]", motorCmd_ma->motor_cmd[0].command, motorCmd_ma->motor_cmd[0].value);

		//update data with the new msg cmds
		for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
		{
			for(int j=0; j<NUMBER_MAX_EPOS_PER_SLAVE; j++)
			{
				//data[i][j][0] = (char)(motorCmd_ma->motor_cmd[i*NUMBER_MAX_EPOS_PER_SLAVE + j].nodeID);// + i*NUMBER_MAX_EPOS_PER_SLAVE); //nodeID
				data[i][j][0] = (char)(motorCmd_ma->motor_cmd[i*NUMBER_MAX_EPOS_PER_SLAVE + j].command); //command
				data[i][j][1] = (char)(motorCmd_ma->motor_cmd[i*NUMBER_MAX_EPOS_PER_SLAVE + j].value); //data 4 bytes
				data[i][j][2] = (char)(motorCmd_ma->motor_cmd[i*NUMBER_MAX_EPOS_PER_SLAVE + j].value >> 8);
				data[i][j][3] = (char)(motorCmd_ma->motor_cmd[i*NUMBER_MAX_EPOS_PER_SLAVE + j].value >> 16);
				data[i][j][4] = (char)(motorCmd_ma->motor_cmd[i*NUMBER_MAX_EPOS_PER_SLAVE + j].value >> 24);
			}
		}
	}
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
    ros::init(argc, argv, "osa_master_board_node");
    ros::NodeHandle nh("~");

    ros::Rate r(LOOP_RATE);

    // Parameters
    std::string usb_device_name;
    // Grab the parameters
    nh.param("usb_device", usb_device_name, std::string("/dev/ttyUSB0"));
    //TODO make LOOP_RATE and HEART_BEAT as a parameter

    cereal::CerealPort device;
    char reply[REPLY_SIZE] = {0x00};
    char checksum[1];

    bool data_valid = false;

	ros::Subscriber cmd_sub = nh.subscribe("/motor_cmd_array", 1, sendMotorCmdArrayCallback);
	ros::Publisher data_pub = nh.advertise<osa_msgs::MotorDataMultiArray>("/motor_data_array", 1);
	osa_msgs::MotorDataMultiArray motor_data_array;

	//create the data multi array
	motor_data_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_data_array.layout.dim[0].size = NUMBER_SLAVE_BOARDS;
	motor_data_array.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS_PER_SLAVE;
	motor_data_array.layout.dim[0].label = "slaves";
	motor_data_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_data_array.layout.dim[1].size = NUMBER_MAX_EPOS_PER_SLAVE;
	motor_data_array.layout.dim[1].stride = NUMBER_MAX_EPOS_PER_SLAVE;
	motor_data_array.layout.dim[1].label = "motors";
	motor_data_array.layout.data_offset = 0;
	motor_data_array.motor_data.clear();
	motor_data_array.motor_data.resize(NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS_PER_SLAVE);

	//init data array
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS_PER_SLAVE; j++)
		{
			motor_data_array.motor_data[i*NUMBER_MAX_EPOS_PER_SLAVE + j].position = 0;
			motor_data_array.motor_data[i*NUMBER_MAX_EPOS_PER_SLAVE + j].current = 0;
			motor_data_array.motor_data[i*NUMBER_MAX_EPOS_PER_SLAVE + j].status = 0;

			//init RS232 char data
			//data[i][j][0] = i*NUMBER_MAX_EPOS_PER_SLAVE + j + 1;
			data[i][j][0] = 0xFF; //0x02;
			data[i][j][1] = 0x00; //0x10 + i;
			data[i][j][2] = 0x00; //0x20 + i;
			data[i][j][3] = 0x00; //0x30 + i;
			data[i][j][4] = 0x00; //0x40 + i;
		}
	}

    // Change the next line according to your port name and baud rate
	try
	{
		device.open(usb_device_name.c_str(), RS232_BAUDRATE);
	}
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port %s at 460800 baud !!!", usb_device_name.c_str());
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened : %s at 460800 baud", usb_device_name.c_str());

    //Main loop
    while(ros::ok())
    {
    	msg_received = false;

    	//check for a new message incoming
    	ros::spinOnce(); //this will toggle the flag msg_received if a msg has been published by another node

    	//if no msg received
    	if(!msg_received)
    	{
    		//reset data mode with 0xFF, i.e. idle slave mode
    		for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
			{
				for(int j=0; j<NUMBER_MAX_EPOS_PER_SLAVE; j++)
				{
					//data[i][j][0] = 0x00;//optional

					//mode
					data[i][j][0] = 0xFF;

					//optional
					data[i][j][1] = 0x00;
					data[i][j][2] = 0x00;
					data[i][j][3] = 0x00;
					data[i][j][4] = 0x00;
				}
			}

    		//ROS_INFO("NO msg received!");
    	}
    /*	else
    	{
    		ROS_INFO("YES msg received!");
    	}*/


    	//write the new data on serial Tx (update in the subscriber or 0xFF data above)
    	checksum[0] = calculateChecksum(data[0][0], NUMBER_OF_MOTORS*BYTES_PER_MSG);

		device.write(data[0][0], NUMBER_OF_MOTORS*BYTES_PER_MSG); //send pointer to the first element, and size
		device.write(">>>>>", 5); //TODO send checksum //62 or 0x3E
		device.write(checksum, 1);

        //read data on serial Rx
        // Get the reply, the last value is the timeout in ms
        try
        {
        	device.readBytes(reply, REPLY_SIZE, TIMEOUT);
        }
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Timeout!");
        }

        //check if the data are valid
        data_valid = verifyChecksum(reply, REPLY_SIZE);

        if(data_valid)
        {
        	//ROS_INFO("Data valid");

        	//build the custom message
        	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
			{
				for(int j=0; j<NUMBER_MAX_EPOS_PER_SLAVE; j++)
				{
					motor_data_array.motor_data[i*NUMBER_MAX_EPOS_PER_SLAVE + j].position =
														  (uint8_t)reply[(i*NUMBER_MAX_EPOS_PER_SLAVE + j)*BYTES_PER_DATA + 0]
													  + (((uint8_t)reply[(i*NUMBER_MAX_EPOS_PER_SLAVE + j)*BYTES_PER_DATA + 1]) << 8)
													  + (((uint8_t)reply[(i*NUMBER_MAX_EPOS_PER_SLAVE + j)*BYTES_PER_DATA + 2]) << 16)
													  + (((uint8_t)reply[(i*NUMBER_MAX_EPOS_PER_SLAVE + j)*BYTES_PER_DATA + 3]) << 24);

					motor_data_array.motor_data[i*NUMBER_MAX_EPOS_PER_SLAVE + j].current =
														  (uint8_t)reply[(i*NUMBER_MAX_EPOS_PER_SLAVE + j)*BYTES_PER_DATA + 4]
													  + (((uint8_t)reply[(i*NUMBER_MAX_EPOS_PER_SLAVE + j)*BYTES_PER_DATA + 5]) << 8);

					motor_data_array.motor_data[i*NUMBER_MAX_EPOS_PER_SLAVE + j].status =
														  (uint8_t)reply[(i*NUMBER_MAX_EPOS_PER_SLAVE + j)*BYTES_PER_DATA + 6]
													  + (((uint8_t)reply[(i*NUMBER_MAX_EPOS_PER_SLAVE + j)*BYTES_PER_DATA + 7]) << 8);
				}
			}

        	//publish it
        	data_pub.publish(motor_data_array);
        	//ROS_INFO("Valid");
        	data_valid = false; //reset flag
        }
        else
        {
        	//ROS_INFO("Data NOT valid");
        }

        device.flush();

        r.sleep();
    }
}

